import torch
from pytorch_mppi import MPPI
import sys
import numpy as np
from shapely.geometry import Polygon, MultiPolygon
from shapely.vectorized import contains

from .config import *
from .utils import dynamics, normalize_angle, save_data

class SMMPPIController:
    def __init__(self, static_obs_passed, device):
        self.horizon = HORIZON_LENGTH
        self.dt = DT
        self.device = device
        self.angular_alignment_threshold = ANGULAR_THRESHOLD
        self.goals = torch.tensor(GOALS, dtype=torch.float32).to(self.device)
        self.max_cycles = NUM_CYCLES
        self.num_samples = NUM_SAMPLES

        current_static_obstacles = static_obs_passed
        if current_static_obstacles and isinstance(current_static_obstacles, list) and \
           all(isinstance(obs_coords, (list, tuple)) for obs_coords in current_static_obstacles):
            valid_obs_coords = [coords for coords in current_static_obstacles if coords]
            if valid_obs_coords:
                self.polygons = [Polygon(coords) for coords in valid_obs_coords]
                self.multi_polygon = MultiPolygon(self.polygons)
                self.bounds = self.multi_polygon.bounds
            else:
                self.polygons = []
                self.multi_polygon = MultiPolygon()
                self.bounds = (0.0, 0.0, 0.0, 0.0)
        else:
            self.polygons = []
            self.multi_polygon = MultiPolygon()
            self.bounds = (0.0, 0.0, 0.0, 0.0)

        self.s2_ego = torch.zeros((self.num_samples, 3)).to(self.device)

        self.current_goal_index = 0
        self.cycle_count = 0
        if self.goals.numel() > 0:
            self.goal = self.goals[self.current_goal_index]
        else:
            print("[SMMPPIController] WARNING: GOALS list in config.py is empty or invalid.", file=sys.stderr)
            self.goal = torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device)

        self.agent_weights = {i: torch.tensor([0.1, 0.1, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        self.interacting_agents = []

        cov = torch.eye(2, dtype=torch.float32).to(self.device)
        cov[0, 0] = 0.5
        cov[1, 1] = 0.5

        u_min_tensor = torch.tensor([0.0, -1.5], dtype=torch.float32).to(self.device)
        u_max_tensor = torch.tensor([VMAX, 1.5], dtype=torch.float32).to(self.device)

        self.mppi = MPPI(
            self.dynamics,
            self.cost,
            3,
            cov,
            num_samples=self.num_samples,
            horizon=self.horizon,
            device=self.device,
            terminal_state_cost=self.terminal_cost,
            lambda_=1.0,
            u_min=u_min_tensor,
            u_max=u_max_tensor,
        )

    def compute_control(self, current_state_robot, previous_robot_state_for_sm, robot_velocity_world,
                        current_agent_states_world, previous_agent_states_for_sm, agent_velocities_world):
        self.current_state_robot_for_cost = current_state_robot
        self.robot_velocity_world_for_cost = robot_velocity_world
        self.current_agent_states_world_for_cost = current_agent_states_world
        self.agent_velocities_world_for_cost = agent_velocities_world

        action = self.mppi.command(current_state_robot)
        distance_to_goal = torch.linalg.norm(current_state_robot[:2] - self.goal)
        termination = distance_to_goal < TERMINATION_TOLERANCE
        return action, None, None, termination

    def cost(self, states_at_t: torch.Tensor, actions_at_t: torch.Tensor) -> torch.Tensor:
        return torch.zeros(states_at_t.shape[0], device=self.device)

    def dynamics(self, s_batch: torch.Tensor, a_batch: torch.Tensor) -> torch.Tensor:
        dt = self.dt
        linear_vel = a_batch[:, 0]
        angular_vel = a_batch[:, 1]
        current_theta = s_batch[:, 2]

        is_turning = torch.abs(angular_vel) > 1e-5
        d_theta_turning = angular_vel * dt
        avg_theta_turning = normalize_angle(current_theta + d_theta_turning / 2.0)

        delta_x = torch.where(is_turning, linear_vel * torch.cos(avg_theta_turning) * dt, linear_vel * torch.cos(current_theta) * dt)
        delta_y = torch.where(is_turning, linear_vel * torch.sin(avg_theta_turning) * dt, linear_vel * torch.sin(current_theta) * dt)
        next_theta = torch.where(is_turning, normalize_angle(current_theta + d_theta_turning), current_theta)

        next_state = torch.zeros_like(s_batch)
        next_state[:, 0] = s_batch[:, 0] + delta_x
        next_state[:, 1] = s_batch[:, 1] + delta_y
        next_state[:, 2] = next_theta
        return next_state

    def terminal_cost(self, states_batch: torch.Tensor, actions_batch: torch.Tensor) -> torch.Tensor:
        state_squeezed = states_batch.squeeze()
        goal_expanded = self.goal[None, :]

        dist = torch.norm(state_squeezed[:, :, :2] - goal_expanded, dim=2)
        goal_cost = torch.sum(dist, dim=1)

        dynamic_obstacle_costs = torch.zeros(self.num_samples).to(self.device)
        sm_costs = torch.zeros(self.num_samples).to(self.device)

        for i in range(ACTIVE_AGENTS):
            if i not in self.current_agent_states_world_for_cost or i not in self.agent_velocities_world_for_cost:
                continue
            pos = self.current_agent_states_world_for_cost[i]
            vel = self.agent_velocities_world_for_cost[i]
            if pos is None or vel is None:
                continue
            time_steps = torch.linspace(self.dt, self.horizon * self.dt, self.horizon, device=self.device)
            pred_x = pos[0] + time_steps * vel[0]
            pred_y = pos[1] + time_steps * vel[1]
            human_states = torch.stack((pred_x, pred_y), dim=1)
            dist_to_agent = torch.norm(state_squeezed[:, :, :2] - human_states, dim=2)

            dynamic_cost = torch.where(dist_to_agent < 1.0, 1 / (1 + dist_to_agent**2), torch.tensor(0.0, device=self.device))
            dynamic_obstacle_costs += torch.sum(dynamic_cost, dim=1)

        static_costs = torch.zeros_like(goal_cost)
        if self.polygons:
            static_costs = self.collision_avoidance_cost(state_squeezed)

        return 2 * goal_cost + sm_costs + dynamic_obstacle_costs + 5 * static_costs

    def collision_avoidance_cost(self, states_batch: torch.Tensor) -> torch.Tensor:
        num_samples = states_batch.shape[0]
        horizon_len = states_batch.shape[1]
        xy_coords = states_batch[..., :2].reshape(-1, 2).cpu().numpy()
        collision_flags = contains(self.multi_polygon, xy_coords[:, 0], xy_coords[:, 1])
        flags = torch.tensor(collision_flags, dtype=torch.bool, device=self.device).view(num_samples, horizon_len)

        penalty = torch.where(flags, torch.tensor(100.0 * 5.0, device=self.device), torch.tensor(0.0, device=self.device))
        return torch.sum(penalty, dim=1)

    def move_to_next_goal(self):
        if self.current_goal_index == -1:
            return
        if self.goals.numel() == 0:
            return
        if self.current_goal_index < len(self.goals) - 1:
            self.current_goal_index += 1
        else:
            if REPEAT_GOALS:
                self.cycle_count += 1
                if self.max_cycles > 0 and self.cycle_count >= self.max_cycles:
                    return
                else:
                    self.current_goal_index = 0
            else:
                return
        self.goal = self.goals[self.current_goal_index]
