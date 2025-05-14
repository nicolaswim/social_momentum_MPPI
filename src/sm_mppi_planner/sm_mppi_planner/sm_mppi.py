import torch
from pytorch_mppi import MPPI # This is an external library, so direct import is fine
import sys # For sys.stderr
import numpy as np
from shapely.geometry import Polygon, MultiPolygon, Point
from shapely.vectorized import contains

# --- Relative imports for modules within the same package ---
from .config import *
from .utils import dynamics, normalize_angle, save_data 
# --- End Relative Imports ---


class SMMPPIController:
    def __init__(self,static_obs_passed, device):
        # Initialize parameters
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
        if self.goals.numel() > 0 : 
             self.goal = self.goals[self.current_goal_index]
        else:
            print("[SMMPPIController] WARNING: GOALS list in config.py is empty or invalid. Setting a placeholder goal.", file=sys.stderr)
            self.goal = torch.tensor([0.0,0.0], dtype=torch.float32).to(self.device)

        self.agent_weights = {i: torch.tensor([0.1, 0.1, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        self.interacting_agents = []

        default_cov_val_lin = 0.5 
        default_cov_val_ang = 0.5 
        cov = torch.eye(2, dtype=torch.float32).to(self.device)
        cov[0, 0] = default_cov_val_lin 
        cov[1, 1] = default_cov_val_ang 
        
        u_min_tensor = torch.tensor([0.0, -1.5], dtype=torch.float32).to(self.device)
        u_max_tensor = torch.tensor([VMAX, 1.5], dtype=torch.float32).to(self.device)

        dynamics_func = self.dynamics 

        self.mppi = MPPI(
            dynamics_func, 
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
        
        rollouts = None 
        costs = None

        distance_to_goal = torch.linalg.norm(current_state_robot[:2] - self.goal)
        termination = distance_to_goal < TERMINATION_TOLERANCE
        
        return action, costs, rollouts, termination

    def cost(self, states_at_t: torch.Tensor, actions_at_t: torch.Tensor) -> torch.Tensor:
        return torch.zeros(states_at_t.shape[0], device=self.device)

    def dynamics(self, s_batch: torch.Tensor, a_batch: torch.Tensor) -> torch.Tensor:
        assert s_batch.ndim == 2 and s_batch.shape[-1] == 3
        assert a_batch.ndim == 2 and a_batch.shape[-1] == 2
        dt = self.dt 
        linear_vel = a_batch[:, 0]
        angular_vel = a_batch[:, 1]
        current_theta = s_batch[:, 2]
        is_turning = torch.abs(angular_vel) > 1e-5 
        d_theta_turning = angular_vel * dt
        next_theta_turning = normalize_angle(current_theta + d_theta_turning) 
        avg_theta_turning = normalize_angle(current_theta + d_theta_turning / 2.0)
        delta_x_turning = linear_vel * torch.cos(avg_theta_turning) * dt
        delta_y_turning = linear_vel * torch.sin(avg_theta_turning) * dt
        delta_x_straight = linear_vel * torch.cos(current_theta) * dt
        delta_y_straight = linear_vel * torch.sin(current_theta) * dt
        next_theta_straight = current_theta 
        s2_global = torch.zeros_like(s_batch)
        s2_global[:, 0] = s_batch[:, 0] + torch.where(is_turning, delta_x_turning, delta_x_straight)
        s2_global[:, 1] = s_batch[:, 1] + torch.where(is_turning, delta_y_turning, delta_y_straight)
        s2_global[:, 2] = torch.where(is_turning, next_theta_turning, next_theta_straight)
        return s2_global

    def terminal_cost(self, states_batch: torch.Tensor, actions_batch: torch.Tensor) -> torch.Tensor:
        final_states = states_batch[:, -1, :] 
        
        # --- DEBUGGING PRINTS ---
        # These will print to the console where your ROS 2 launch output goes
        print(f"[SMMPPI_DEBUG] terminal_cost: final_states shape: {final_states.shape}")
        print(f"[SMMPPI_DEBUG] terminal_cost: final_states[:10, :2]: {final_states[:10, :2]}") # Print first 10 x,y of final_states
        print(f"[SMMPPI_DEBUG] terminal_cost: self.goal shape: {self.goal.shape}")
        print(f"[SMMPPI_DEBUG] terminal_cost: self.goal value: {self.goal}")
        # --- END DEBUGGING PRINTS ---
        
        dist_to_goal = torch.linalg.norm(final_states[:, :2] - self.goal, dim=1) # This is line 153 in built file
        
        goal_cost_weight = 2.0 
        if 'GOAL_COST_WEIGHT' in globals(): goal_cost_weight = GOAL_COST_WEIGHT
        goal_cost = dist_to_goal * goal_cost_weight

        dynamic_obstacle_costs_total = torch.zeros_like(goal_cost)
        sm_costs_total = torch.zeros_like(goal_cost)
        static_obstacle_costs_total = torch.zeros_like(goal_cost)

        if self.polygons: 
             static_obstacle_costs_total = self.collision_avoidance_cost(states_batch)

        if ACTIVE_AGENTS > 0 and hasattr(self, 'current_agent_states_world_for_cost') and hasattr(self, 'agent_velocities_world_for_cost'):
            num_samples_from_batch = states_batch.shape[0]
            for i in range(ACTIVE_AGENTS):
                if i not in self.current_agent_states_world_for_cost or \
                   i not in self.agent_velocities_world_for_cost or \
                   self.current_agent_states_world_for_cost[i] is None or \
                   self.agent_velocities_world_for_cost[i] is None:
                    continue
                current_agent_pos = self.current_agent_states_world_for_cost[i][:2]
                current_agent_vel = self.agent_velocities_world_for_cost[i][:2]
                time_steps_horizon = torch.arange(1, self.horizon + 1, device=self.device).float() * self.dt
                agent_path_x = current_agent_pos[0] + time_steps_horizon * current_agent_vel[0]
                agent_path_y = current_agent_pos[1] + time_steps_horizon * current_agent_vel[1]
                agent_path_horizon = torch.stack((agent_path_x, agent_path_y), dim=1) 
                agent_path_horizon_expanded = agent_path_horizon.unsqueeze(0).expand(num_samples_from_batch, -1, -1)
                dist_to_agent = torch.linalg.norm(states_batch[:, :, :2] - agent_path_horizon_expanded, dim=2)
                dynamic_obs_weight = 10.0 
                if 'DYNAMIC_OBS_COST_WEIGHT' in globals(): dynamic_obs_weight = DYNAMIC_OBS_COST_WEIGHT
                human_radius_for_cost = RADIUS 
                if 'HUMAN_RADIUS' in globals(): human_radius_for_cost = HUMAN_RADIUS
                collision_threshold = RADIUS + human_radius_for_cost
                dynamic_obs_penalty = torch.where(dist_to_agent < collision_threshold, 
                                                  (1.0 / (dist_to_agent + 1e-5)) * dynamic_obs_weight, 
                                                  torch.tensor(0.0, device=self.device))
                dynamic_obstacle_costs_total += torch.sum(dynamic_obs_penalty, dim=1)
                sm_penalty_for_agent = torch.zeros_like(goal_cost) 
                sm_costs_total += sm_penalty_for_agent
        total_cost = goal_cost + sm_costs_total + dynamic_obstacle_costs_total + static_obstacle_costs_total
        return total_cost
    
    def get_interacting_agents_for_cost(self, robot_state_for_cost):
        interacting_agents_for_cost = []
        if ACTIVE_AGENTS > 0 and hasattr(self, 'current_agent_states_world_for_cost'):
            fov_deg_val = 180.0 
            if 'FOV_DEG' in globals(): fov_deg_val = FOV_DEG
            fov_rad_half = torch.deg2rad(torch.tensor(fov_deg_val, device=self.device)) / 2.0
            interaction_dist_threshold = 5.0 
            if 'INTERACTION_DISTANCE' in globals(): interaction_dist_threshold = INTERACTION_DISTANCE
            for idx, agent_state_world in self.current_agent_states_world_for_cost.items():
                if agent_state_world is None or robot_state_for_cost is None: continue
                agent_state_world_dev = agent_state_world.to(self.device) 
                robot_state_for_cost_dev = robot_state_for_cost.to(self.device)
                direction_to_agent = torch.arctan2(agent_state_world_dev[1] - robot_state_for_cost_dev[1], 
                                                 agent_state_world_dev[0] - robot_state_for_cost_dev[0])
                distance_to_agent = torch.norm(agent_state_world_dev[:2] - robot_state_for_cost_dev[:2])
                angle_diff = normalize_angle(direction_to_agent - robot_state_for_cost_dev[2])
                if torch.abs(angle_diff) <= fov_rad_half and distance_to_agent < interaction_dist_threshold: 
                    interacting_agents_for_cost.append(idx)
        return interacting_agents_for_cost

    def collision_avoidance_cost(self, states_batch: torch.Tensor) -> torch.Tensor:
        if not self.polygons: 
            return torch.zeros(states_batch.shape[0], device=self.device)
        num_samples = states_batch.shape[0]
        horizon_len = states_batch.shape[1]
        xy_coords_batch = states_batch[..., :2]
        flattened_xy_coords = xy_coords_batch.reshape(-1, 2).cpu().numpy()
        collision_flags_flat = contains(self.multi_polygon, flattened_xy_coords[:, 0], flattened_xy_coords[:, 1])
        collision_flags_batch = torch.tensor(collision_flags_flat, dtype=torch.bool, device=self.device).view(num_samples, horizon_len)
        static_cost_weight = 5.0 
        if 'STATIC_COST_WEIGHT' in globals():
            static_cost_weight = STATIC_COST_WEIGHT
        collision_penalty = torch.where(collision_flags_batch, 
                                        torch.tensor(100.0 * static_cost_weight, device=self.device), 
                                        torch.tensor(0.0, device=self.device))
        total_collision_cost_per_sample = torch.sum(collision_penalty, dim=1)
        return total_collision_cost_per_sample

    def move_to_next_goal(self):
        if self.current_goal_index == -1: 
            print("[SMMPPIController] External goal reached/processed. Waiting for new external goal.", file=sys.stderr)
            return 
        if self.goals.numel() == 0:
            print("[SMMPPIController] No goals defined in config to cycle through.", file=sys.stderr)
            return
        if self.current_goal_index < len(self.goals) - 1:
            self.current_goal_index += 1
        else: 
            if REPEAT_GOALS: 
                self.cycle_count += 1
                if self.max_cycles > 0 and self.cycle_count >= self.max_cycles: 
                    print(f"[SMMPPIController] All {self.max_cycles} cycles completed! Will keep targeting last goal of cycle.", file=sys.stderr)
                    return 
                else: 
                    self.current_goal_index = 0 
            else: 
                print(f"[SMMPPIController] Finished all goals in the list (REPEAT_GOALS is False). Will keep targeting last goal.", file=sys.stderr)
                return 
        self.goal = self.goals[self.current_goal_index]
        max_c = self.max_cycles if self.max_cycles > 0 else "inf"
        print(f"[SMMPPIController] Moving to goal {self.current_goal_index + 1}/{len(self.goals)}: {self.goal.cpu().numpy()} (Cycle {self.cycle_count + 1}/{max_c if REPEAT_GOALS else 1})", file=sys.stderr)

    # The transform_base_action_to_world_batch and calculate_angular_momentum_z_batch 
    # are not directly used by the simplified terminal_cost at the moment.
    # They would be needed for a more complete SocialCost_for_MPPI implementation.
    def transform_base_action_to_world_batch(self, base_actions_batch, a_thetas_batch):
        vx_base = base_actions_batch[:, 0]
        vy_base = base_actions_batch[:, 1] 
        cos_theta = torch.cos(a_thetas_batch)
        sin_theta = torch.sin(a_thetas_batch)
        vx_world = vx_base * cos_theta - vy_base * sin_theta
        vy_world = vx_base * sin_theta + vy_base * cos_theta
        return torch.stack((vx_world, vy_world), dim=1)

    def calculate_angular_momentum_z_batch(self, q1_batch, v1_batch, q2_batch, v2_batch):
        com_q_batch = (q1_batch + q2_batch) / 2.0 
        r1_com_batch = q1_batch - com_q_batch 
        r2_com_batch = q2_batch - com_q_batch 
        L1_z_batch = r1_com_batch[:, 0] * v1_batch[:, 1] - r1_com_batch[:, 1] * v1_batch[:, 0]
        L2_z_batch = r2_com_batch[:, 0] * v2_batch[:, 1] - r2_com_batch[:, 1] * v2_batch[:, 0]
        return L1_z_batch + L2_z_batch
