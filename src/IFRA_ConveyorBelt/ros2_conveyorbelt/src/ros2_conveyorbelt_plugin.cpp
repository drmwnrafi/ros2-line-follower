#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_conveyorbelt/ros2_conveyorbelt_plugin.hpp"
#include <conveyorbelt_msgs/msg/conveyor_belt_state.hpp>
#include <conveyorbelt_msgs/msg/conveyor_power.hpp>  
#include <conveyorbelt_msgs/msg/conveyor_enable.hpp> 

#include <memory>
#include <vector>
#include <string>
#include <sstream>

namespace gazebo_ros
{

class ROS2ConveyorBeltPluginPrivate
{
public:
  gazebo_ros::Node::SharedPtr ros_node_;
  std::vector<gazebo::physics::JointPtr> belt_joints_;
  
  
  std::vector<double> belt_velocities_;
  std::vector<double> max_velocities_;
  std::vector<double> powers_;
  double belt_changer_velocity_;
  double limit_;

  rclcpp::Publisher<conveyorbelt_msgs::msg::ConveyorBeltState>::SharedPtr status_pub_;
  conveyorbelt_msgs::msg::ConveyorBeltState status_msg_;
  
  rclcpp::Subscription<conveyorbelt_msgs::msg::ConveyorPower>::SharedPtr power_sub_;
  rclcpp::Subscription<conveyorbelt_msgs::msg::ConveyorEnable>::SharedPtr enable_sub_;

  rclcpp::Time last_publish_time_;
  int update_ns_;
  gazebo::event::ConnectionPtr update_connection_;

  void PublishStatus();
  void OnUpdate();
  
  void OnPowerCommand(const conveyorbelt_msgs::msg::ConveyorPower::SharedPtr msg);
  void OnEnableCommand(const conveyorbelt_msgs::msg::ConveyorEnable::SharedPtr msg);
};

ROS2ConveyorBeltPlugin::ROS2ConveyorBeltPlugin()
  : impl_(std::make_unique<ROS2ConveyorBeltPluginPrivate>()) {}

ROS2ConveyorBeltPlugin::~ROS2ConveyorBeltPlugin() = default;

void ROS2ConveyorBeltPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  std::string joint_names_str = _sdf->Get<std::string>("joint_names", "").first;
  std::istringstream joint_names_iss(joint_names_str);
  std::vector<std::string> joint_names;
  std::string name;
  while (joint_names_iss >> name) {
    joint_names.push_back(name);
  }

  std::string max_velocities_str = _sdf->Get<std::string>("max_velocities", "").first;
  std::istringstream max_velocities_iss(max_velocities_str);
  std::vector<double> max_velocities;
  double velocity;
  while (max_velocities_iss >> velocity) {
    max_velocities.push_back(velocity);
  }

  if (joint_names.empty() || joint_names.size() != max_velocities.size()) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), 
                "Invalid joint_names or max_velocities configuration");
    return;
  }

  for (const auto& joint_name : joint_names) {
    auto joint = _model->GetJoint(joint_name);
    if (!joint) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint '%s' not found", joint_name.c_str());
      return;
    }
    impl_->belt_joints_.push_back(joint);
  }

  impl_->max_velocities_ = max_velocities;
  impl_->belt_velocities_.resize(joint_names.size(), 0.0);
  impl_->powers_.resize(joint_names.size(), 0.0);

  impl_->belt_changer_velocity_ = _sdf->Get<double>("belt_changer_velocity", 0.0).first;
  impl_->limit_ = impl_->belt_joints_[0]->UpperLimit();

  impl_->status_pub_ = impl_->ros_node_->create_publisher<conveyorbelt_msgs::msg::ConveyorBeltState>(
    "CONVEYORSTATE", 10);

  impl_->power_sub_ = impl_->ros_node_->create_subscription<conveyorbelt_msgs::msg::ConveyorPower>(
    "CONVEYORPOWER", 10,
    [this](const conveyorbelt_msgs::msg::ConveyorPower::SharedPtr msg) {
      impl_->OnPowerCommand(msg);
    });

  impl_->enable_sub_ = impl_->ros_node_->create_subscription<conveyorbelt_msgs::msg::ConveyorEnable>(
    "ENABLEBELT", 10,
    [this](const conveyorbelt_msgs::msg::ConveyorEnable::SharedPtr msg) {
      impl_->OnEnableCommand(msg);
    });

  double publish_rate = _sdf->Get<double>("publish_rate", 10).first;
  impl_->update_ns_ = static_cast<int>((1.0 / publish_rate) * 1e9);
  impl_->last_publish_time_ = impl_->ros_node_->get_clock()->now();

  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    [this](gazebo::common::UpdateInfo) { impl_->OnUpdate(); });

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Gazebo ConveyorBelt plugin loaded");
}

void ROS2ConveyorBeltPluginPrivate::OnUpdate()
{
  for (size_t i = 0; i < belt_joints_.size(); ++i) {
    belt_joints_[i]->SetVelocity(0, belt_velocities_[i]);
    if (belt_joints_[i]->Position(0) >= limit_) {
      belt_joints_[i]->SetPosition(0, -limit_);
    }
  }

  rclcpp::Time now = ros_node_->get_clock()->now();
  if (now - last_publish_time_ >= rclcpp::Duration(0, update_ns_)) {
    PublishStatus();
    last_publish_time_ = now;
  }
}

void ROS2ConveyorBeltPluginPrivate::OnPowerCommand(
  const conveyorbelt_msgs::msg::ConveyorPower::SharedPtr msg)
{
  if (msg->powers.size() != belt_joints_.size()) {
    RCLCPP_WARN(ros_node_->get_logger(), 
                "Power array size does not match number of joints");
    return;
  }

  for (size_t i = 0; i < belt_joints_.size(); ++i) {
    double power = msg->powers[i];
    if (std::abs(power) > 100) {
      RCLCPP_WARN(ros_node_->get_logger(), 
                  "Invalid power value for joint %zu. Must be between -100 and 100.", i);
      continue;
    }

    if (power < 0) {
      belt_joints_[i]->SetAxis(0, ignition::math::Vector3d(0, -1, 0));
      powers_[i] = -power;
    } else {
      belt_joints_[i]->SetAxis(0, ignition::math::Vector3d(0, 1, 0));
      powers_[i] = power;
    }

    belt_velocities_[i] = max_velocities_[i] * (powers_[i] / 100.0);
  }
}

void ROS2ConveyorBeltPluginPrivate::OnEnableCommand(
  const conveyorbelt_msgs::msg::ConveyorEnable::SharedPtr msg)
{
  if (msg->enable.size() != belt_joints_.size()) {
    RCLCPP_WARN(ros_node_->get_logger(), 
                "Enable array size does not match number of joints");
    return;
  }

  for (size_t i = 0; i < belt_joints_.size(); ++i) {
    if (msg->enable[i]) {
      belt_joints_[i]->SetVelocity(0, belt_changer_velocity_);
    }
  }
}

void ROS2ConveyorBeltPluginPrivate::PublishStatus()
{
  status_msg_.powers.clear(); 
  bool enabled = false;
  
  for (const auto& power : powers_) {
    status_msg_.powers.push_back(power);
    if (power > 0) enabled = true;
  }
  
  status_msg_.enabled = enabled;
  status_pub_->publish(status_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(gazebo_ros::ROS2ConveyorBeltPlugin)

} // namespace gazebo_ros