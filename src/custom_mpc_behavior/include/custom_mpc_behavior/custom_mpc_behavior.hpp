#pragma once

#include "custom_mpc_behavior/site_tracking.hpp"
#include "moveit_pro_mpc/residual_functions/cartesian_acceleration.hpp"
#include "moveit_pro_mpc/residual_functions/cartesian_velocity.hpp"
#include "moveit_pro_mpc/residual_functions/force_torque.hpp"
#include "moveit_pro_mpc/residual_functions/joint_acceleration.hpp"
#include "moveit_pro_mpc/residual_functions/joint_velocity.hpp"
#include "moveit_studio_behavior_interface/mpc_behavior_base.hpp"

namespace custom_mpc_behavior
{
/**
 * @brief TODO(...)
 */
class CustomMpcBehavior : public moveit_studio::behaviors::MPCBehaviorBase<CustomMpcBehavior>
{
public:
  /**
   * @brief Constructor for the custom_mpc_behavior behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when
   * this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in
   * the behavior tree. This BehaviorContext is owned by the Studio Agent's ObjectiveServerNode.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the
   * Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this
   * Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after
   * the initialize() function is called, so these classes should not be used within the constructor.
   */
  CustomMpcBehavior(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the custom_mpc_behavior Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named
   * providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function
   * must return an empty BT::PortsList. This function returns a list of ports with their names and port info, which is
   * used internally by the behavior tree.
   * @return custom_mpc_behavior does not expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Define the residuals used by this MPC behavior.
   * @details This method allows the use of a MPCProblem instance to add residual-specific ports.
   * @return A BT::PortsList containing the required ports for the behavior.
   */
  static auto getResiduals()
  {
    return std::make_tuple(std::make_pair("site_tracking", custom_mpc_behavior::SiteTrackingResidualFunction()),
                           std::make_pair("velocity", moveit_pro_mpc::VelocityResidualFunction()),
                           std::make_pair("acceleration", moveit_pro_mpc::AccelerationResidualFunction()),
                           std::make_pair("force_torque", moveit_pro_mpc::ForceTorqueResidualFunction()),
                           std::make_pair("cartesian_velocity", moveit_pro_mpc::CartesianVelocityResidualFunction()),
                           std::make_pair("cartesian_acceleration",
                                          moveit_pro_mpc::CartesianAccelerationResidualFunction()));
  }

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();
};
}  // namespace custom_mpc_behavior
