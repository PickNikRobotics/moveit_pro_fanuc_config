#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace actuate_gripper
{
/**
 * @brief ServiceClient behavior that opens or closes the Fanuc gripper by calling the /actuate_fanuc_gripper service.
 */
class ActuateGripper : public moveit_studio::behaviors::ServiceClientBehaviorBase<std_srvs::srv::SetBool>
{
public:
  using SetBool = std_srvs::srv::SetBool;

  /**
   * @brief Constructor for the actuate_gripper behavior.
   * @param name The name of a particular instance of this Behavior.
   * @param config Runtime configuration info for this Behavior.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all Behaviors.
   */
  ActuateGripper(const std::string& name, const BT::NodeConfiguration& config,
              const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Constructor for testing with a custom client interface.
   */
  ActuateGripper(const std::string& name, const BT::NodeConfiguration& config,
              const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
              std::unique_ptr<moveit_studio::behaviors::ClientInterfaceBase<SetBool>> client_interface);

  /**
   * @brief Implementation of the required providedPorts() function.
   * @return List of input and output ports for this Behavior.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

protected:
  /**
   * @brief Get the service name to call.
   * @return The service name "/actuate_fanuc_gripper".
   */
  tl::expected<std::string, std::string> getServiceName() override;

  /**
   * @brief Create the service request based on the input port "command".
   * @return SetBool request with data=true for "open", data=false for "close".
   */
  tl::expected<SetBool::Request, std::string> createRequest() override;

  /**
   * @brief Process the service response.
   * @param response The service response.
   * @return true if the service call was successful, false otherwise.
   */
  tl::expected<bool, std::string> processResponse(const SetBool::Response& response) override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return response_future_;
  }

  /** @brief Holds the result of calling the service asynchronously. */
  std::shared_future<tl::expected<bool, std::string>> response_future_;
};
}  // namespace actuate_gripper
