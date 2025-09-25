#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <string>
#include <memory>

namespace gazebo
{
  class FollowerPlugin : public ModelPlugin
  {
  public:
    // --- CHANGE 1: ADD A CONSTRUCTOR TO INITIALIZE THE LOGGER ---
    FollowerPlugin() : logger_(rclcpp::get_logger("FollowerPlugin")) {}

    // Called when the plugin is loaded
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model_ = _model;
      this->world_ = _model->GetWorld();

      if (_sdf->HasElement("follow"))
      {
        this->target_name_ = _sdf->Get<std::string>("follow");
      }
      else
      {
        gzerr << "FollowerPlugin missing <follow> tag, cannot proceed." << std::endl;
        return;
      }

      this->ros_node_ = gazebo_ros::Node::Get(_sdf);
      
      // --- CHANGE 2: USE THE ALREADY-INITIALIZED LOGGER ---
      // (The old line "this->logger_ = this->ros_node_->get_logger();" is removed)
      RCLCPP_INFO(this->logger_, "FollowerPlugin loaded for model: %s, following: %s",
                  this->model_->GetName().c_str(), this->target_name_.c_str());

      this->pose_sub_ = this->ros_node_->create_subscription<gazebo_msgs::msg::LinkStates>(
          "/link_states", 
          10,
          std::bind(&FollowerPlugin::OnPoseMsg, this, std::placeholders::_1));
    }

  private:
    void OnPoseMsg(const gazebo_msgs::msg::LinkStates::SharedPtr _msg)
    {
      for (size_t i = 0; i < _msg->name.size(); ++i)
      {
        if (_msg->name[i].find(this->target_name_) != std::string::npos)
        {
          ignition::math::Pose3d target_pose(
              _msg->pose[i].position.x,
              _msg->pose[i].position.y,
              _msg->pose[i].position.z,
              _msg->pose[i].orientation.w,
              _msg->pose[i].orientation.x,
              _msg->pose[i].orientation.y,
              _msg->pose[i].orientation.z);
          this->model_->SetWorldPose(target_pose);
          return;
        }
      }
    }

    private:
      physics::ModelPtr model_;
      physics::WorldPtr world_;
      std::string target_name_;
      gazebo_ros::Node::SharedPtr ros_node_;
      rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr pose_sub_;
      rclcpp::Logger logger_;
  };

  GZ_REGISTER_MODEL_PLUGIN(FollowerPlugin)
}