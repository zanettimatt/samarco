/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <gz/msgs/laserscan.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <mutex>
#include <string>
#include <queue>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/World.hh"
#include "gz/sim/Util.hh"

#include "magnetic_adhesion_plugin/MagneticAdhesionPlugin.hh"

using namespace gz;
using namespace gz::sim;
using namespace systems;

class ignition::gazebo::systems::MagneticAdhesionPluginPrivate
{
  /// \brief Callback for lidar subscription
  /// \param[in] _msg Lidar message
  public: void OnLidar(const msgs::LaserScan &_msg);

  /// \brief Calculate magnetic force based on distance
  /// \param[in] distance Distance from lidar
  /// \return Magnetic force
  public: double CalculateMagneticForce(double distance);

  /// \brief Communication node.
  public: transport::Node node;

  /// \brief ROS 2 node for publishing magnetic force
  public: std::shared_ptr<rclcpp::Node> rosNode;

  /// \brief ROS 2 publisher for magnetic force
  public: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr forcePublisher;

  /// \brief A mutex to protect lidar data
  public: std::mutex mutex;

  /// \brief Distance received from lidar
  public: double distance{0.0};

  /// \brief Entity to apply the force to
  public: Entity entity{kNullEntity};

  /// \brief Verbose mode
  public: bool verbose{true};
};

//////////////////////////////////////////////////
MagneticAdhesionPlugin::MagneticAdhesionPlugin()
  : dataPtr(std::make_unique<MagneticAdhesionPluginPrivate>())
{
}

//////////////////////////////////////////////////
void MagneticAdhesionPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Initialize ROS 2 node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->dataPtr->rosNode = std::make_shared<rclcpp::Node>("magnetic_adhesion_plugin");

  // Create a publisher for the magnetic force
  this->dataPtr->forcePublisher = this->dataPtr->rosNode->create_publisher<std_msgs::msg::Float64>(
      "/magnetic_force", 10);

  // Get the entity to apply the force to
  if (_sdf->HasElement("entity_name"))
  {
    std::string entityName = _sdf->Get<std::string>("entity_name");

    // Busca a entidade pelo nome
    this->dataPtr->entity = _ecm.EntityByComponents(
        components::Name(entityName),
        components::ParentEntity(_entity));

    if (this->dataPtr->entity == kNullEntity)
    {
      ignerr << "Entity with name [" << entityName << "] not found."
             << std::endl;
      return;
    }
  }
  else
  {
    ignerr << "Missing required parameter <entity_name>."
           << std::endl;
    return;
  }

  // Set verbose mode
  this->dataPtr->verbose = _sdf->Get<bool>("verbose", true).first;

  // Topic to subscribe to lidar data
  std::string topic{"/lidar"};
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  this->dataPtr->node.Subscribe(topic, &MagneticAdhesionPluginPrivate::OnLidar,
      this->dataPtr.get());

  ignmsg << "Listening to lidar data in [" << topic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
void MagneticAdhesionPlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("MagneticAdhesionPlugin::PreUpdate");

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Only apply force when not paused
  if (_info.paused)
    return;

  // Calculate magnetic force
  double forceMagnitude = this->dataPtr->CalculateMagneticForce(this->dataPtr->distance);

  // Publish the magnetic force to ROS 2 topic
  std_msgs::msg::Float64 forceMsg;
  forceMsg.data = forceMagnitude;
  this->dataPtr->forcePublisher->publish(forceMsg);

  // Apply force to the entity
  if (this->dataPtr->entity != kNullEntity)
  {
    Link link(this->dataPtr->entity);
    if (link.Valid(_ecm))
    {
      //math::Vector3d force(0, 0, -forceMagnitude); // Assuming force is applied along the Z-axis
      math::Vector3d force(0, 0, 0); // Assuming force is applied along the Z-axis
      link.AddWorldWrench(_ecm, force, math::Vector3d::Zero);

      /*if (this->dataPtr->verbose)
      {
        igndbg << "Applying magnetic force [" << force << "] to entity ["
               << link.Entity() << "]." << std::endl;
      }*/
    }
  }
}

void MagneticAdhesionPluginPrivate::OnLidar(const msgs::LaserScan &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Assuming the distance is the first range measurement
  if (_msg.ranges_size() > 0)
  {
    this->distance = _msg.ranges(0);
  }
}

double MagneticAdhesionPluginPrivate::CalculateMagneticForce(double distance)
{
  // Calculate magnetic force based on the provided equation
  return -0.0065 * std::pow(distance, 3) + 0.1872 * std::pow(distance, 2) - 2.1151 * distance + 10.9216;
}

IGNITION_ADD_PLUGIN(MagneticAdhesionPlugin,
                    System,
                    MagneticAdhesionPlugin::ISystemConfigure,
                    MagneticAdhesionPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MagneticAdhesionPlugin,
                          "gz::sim::systems::MagneticAdhesionPlugin")

IGNITION_ADD_PLUGIN_ALIAS(MagneticAdhesionPlugin,
                          "magnetic_adhesion_plugin")

// TODO(CH3): Deprecated, remove on version 8
IGNITION_ADD_PLUGIN_ALIAS(MagneticAdhesionPlugin,
                          "ignition::gazebo::systems::MagneticAdhesionPlugin")
