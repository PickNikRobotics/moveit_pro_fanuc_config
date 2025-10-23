#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <actuate_gripper/actuate_gripper.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace actuate_gripper
{
class ActuateGripperBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<ActuateGripper>(factory, "ActuateGripper", shared_resources);
  }
};
}  // namespace actuate_gripper

PLUGINLIB_EXPORT_CLASS(actuate_gripper::ActuateGripperBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
