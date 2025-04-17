#include "behaviortree_cpp/bt_factory.h"
#include "moveit_studio_behavior_interface/behavior_context.hpp"
#include "moveit_studio_behavior_interface/shared_resources_node_loader.hpp"

#include "calibration_behaviors/average_pose_stamped_vector.hpp"
#include "calibration_behaviors/calibrate_camera_pose.hpp"
#include "calibration_behaviors/save_calibration_pose_yaml.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace calibration_behaviors
{
class CalibrationBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<AveragePoseStampedVector>(factory, "AveragePoseStampedVector",
                                                                         shared_resources);
    moveit_studio::behaviors::registerBehavior<CalibrateCameraPose>(factory, "CalibrateCameraPose", shared_resources);
    moveit_studio::behaviors::registerBehavior<SaveCalibrationPoseYaml>(factory, "SaveCalibrationPoseYaml",
                                                                        shared_resources);
  }
};
}  // namespace calibration_behaviors

PLUGINLIB_EXPORT_CLASS(calibration_behaviors::CalibrationBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
