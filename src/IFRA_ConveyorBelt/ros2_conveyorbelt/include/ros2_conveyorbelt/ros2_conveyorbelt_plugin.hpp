#ifndef ROS2_CONVEYORBELT_PLUGIN_HPP
#define ROS2_CONVEYORBELT_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <memory>

namespace gazebo_ros
{

class ROS2ConveyorBeltPluginPrivate;

class ROS2ConveyorBeltPlugin : public gazebo::ModelPlugin  // Now fully defined
{
public:
  ROS2ConveyorBeltPlugin();
  ~ROS2ConveyorBeltPlugin() override;

  // Add proper SDF namespace for Gazebo version
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  std::unique_ptr<ROS2ConveyorBeltPluginPrivate> impl_;
};

} // namespace gazebo_ros

#endif