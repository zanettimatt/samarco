#include <gz/msgs/laserscan.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>

#include <mutex>
#include <string>
#include <queue>
#include <vector>
#include <thread>

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

  /// \brief Callback for inclination subscription
  /// \param[in] _msg Inclination message
  public: void OnInclination(const std_msgs::msg::Float32::SharedPtr _msg);

  /// \brief Communication node.
  public: transport::Node node;

  /// \brief ROS 2 node for publishing magnetic force
  public: std::shared_ptr<rclcpp::Node> rosNode;

  /// \brief ROS 2 publisher for magnetic force
  public: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr forcePublisher;

  /// \brief ROS 2 subscriber for inclination
  public: rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr inclinationSubscriber;

  /// \brief A mutex to protect lidar data
  public: std::mutex mutex;

  /// \brief Distance received from lidar
  public: double distance{0.0};

  /// \brief Inclination angle received from the inclination topic
  public: float inclination{0.0};

  /// \brief Entity to apply the force to
  public: Entity entity{kNullEntity};

  /// \brief Verbose mode
  public: bool verbose{true};

  /// \brief Topic name for the magnetic force
  public: std::string forceTopicName{"/magnetic_force"};
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

  // Read the topic name from the SDF
  if (_sdf->HasElement("topicName")) {
    this->dataPtr->forceTopicName = _sdf->Get<std::string>("topicName");
  }

  // Create a publisher for the magnetic force
  this->dataPtr->forcePublisher = this->dataPtr->rosNode->create_publisher<std_msgs::msg::Float64>(
      this->dataPtr->forceTopicName, 10);

  // Create a subscriber for the inclination topic
  this->dataPtr->inclinationSubscriber = this->dataPtr->rosNode->create_subscription<std_msgs::msg::Float32>(
      "/inclination_angle", 10,
      std::bind(&MagneticAdhesionPluginPrivate::OnInclination, this->dataPtr.get(), std::placeholders::_1));

  // Start ROS 2 spinning in a separate thread
  std::thread rosThread([this]() {
    rclcpp::spin(this->dataPtr->rosNode);
  });
  rosThread.detach();

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


  // Convert inclination angle from degrees to radians
  double inclinationRad = this->dataPtr->inclination * (M_PI / 180.0);

  // Decompose force into horizontal (X) and vertical (Z) components
  double forceZ = 1 * (432.621) * std::cos(inclinationRad); // Componente vertical
  double forceY = 1 * (432.621+410) * std::sin(inclinationRad); // Componente horizontal 420
  // double forceY = 432.621 - forceZ; // Componente horizontal

  // Create the force vector
  math::Vector3d force(0, forceY, -forceZ);

  // Apply force to the entity
  if (this->dataPtr->entity != kNullEntity)
  {
    Link link(this->dataPtr->entity);
    if (link.Valid(_ecm))
    {
      // Apply the force with adjusted components
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

void MagneticAdhesionPluginPrivate::OnInclination(const std_msgs::msg::Float32::SharedPtr _msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Store the inclination angle
  this->inclination = _msg->data;
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
