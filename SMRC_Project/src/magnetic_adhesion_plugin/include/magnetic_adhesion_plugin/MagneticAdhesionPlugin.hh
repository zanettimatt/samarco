#ifndef GZ_SIM_SYSTEMS_MAGNETICADHESIONPLUGIN_HH_
#define GZ_SIM_SYSTEMS_MAGNETICADHESIONPLUGIN_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class MagneticAdhesionPluginPrivate;

  /// \brief A plugin that applies a magnetic force to an entity based on lidar distance.
  ///
  /// This plugin subscribes to a lidar topic to get distance measurements and calculates
  /// a magnetic force using a predefined equation. The force is then applied to a specified
  /// entity (e.g., a link or model) in the simulation.
  ///
  /// ## Topics
  ///
  /// * /lidar
  ///     * Message type: msgs::LaserScan
  ///     * Effect: Provides distance measurements used to calculate the magnetic force.
  ///
  /// ## System Parameters
  ///
  /// The plugin requires the following SDF parameters:
  ///
  /// ```
  /// <entity_name>entity_name</entity_name>
  /// <topic>/lidar</topic>
  /// ```
  ///
  /// - `<entity_name>`: Name of the entity (link or model) to apply the force to.
  /// - `<topic>`: Topic to subscribe to for lidar data (default is `/lidar`).
  class MagneticAdhesionPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: MagneticAdhesionPlugin();

    /// \brief Destructor
    public: ~MagneticAdhesionPlugin() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<MagneticAdhesionPluginPrivate> dataPtr;
  };
  }
}
}
}

#endif
