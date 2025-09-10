#include <actuate_gripper/actuate_gripper.hpp>

#include "spdlog/spdlog.h"
#include <moveit_studio_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kPortIdCommand = "command";
constexpr auto kGripperServiceName = "/actuate_fanuc_gripper";

inline constexpr auto kDescriptionActuateGripper = R"(
                <p>Send a request to the <code>/actuate_fanuc_gripper</code> service to open or close the Fanuc gripper.</p>
                <p>This behavior calls the <code>/actuate_fanuc_gripper</code> service advertised by the GPIO gripper node.</p>
                <p>The service type is <code>std_srvs::srv::SetBool</code>.</p>
                <p>The <code>command</code> input port accepts "open" or "close" strings.</p>
                <p>If this Behavior receives a response from the service server indicating that the request succeeded, this Behavior exits with a SUCCESS status code.</p>
                <p>If any of the following failure states occur, this Behavior will exit with a FAILURE status code:</p>
                <ul>
                    <li>No service server is available with the specified name.</li>
                    <li>Invalid command string (not "open" or "close").</li>
                    <li>The service response indicates failure.</li>
                    <li>Gripper operation timed out.</li>
                </ul>
            )";
}  // namespace

namespace actuate_gripper
{
ActuateGripper::ActuateGripper(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<SetBool>(name, config, shared_resources)
{
}

ActuateGripper::ActuateGripper(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
    std::unique_ptr<moveit_studio::behaviors::ClientInterfaceBase<SetBool>> client_interface)
  : ServiceClientBehaviorBase<SetBool>(name, config, shared_resources, std::move(client_interface))
{
}

BT::PortsList ActuateGripper::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIdCommand, "Command string: \"open\" to open the gripper, \"close\" to close it") };
}

BT::KeyValueVector ActuateGripper::metadata()
{
  return { { moveit_studio::behaviors::kSubcategoryMetadataKey, "User Created Behaviors" },
           { moveit_studio::behaviors::kDescriptionMetadataKey, kDescriptionActuateGripper } };
}

tl::expected<std::string, std::string> ActuateGripper::getServiceName()
{
  return kGripperServiceName;
}

tl::expected<ActuateGripper::SetBool::Request, std::string> ActuateGripper::createRequest()
{
  // Get the command from the input port
  const auto command_result = getInput<std::string>(kPortIdCommand);
  if (!command_result)
  {
    return tl::make_unexpected("Failed to get command from input port: " + command_result.error());
  }

  const std::string command = command_result.value();
  SetBool::Request request;

  if (command == "open")
  {
    request.data = true;
    spdlog::info("ActuateGripper: Sending request to open gripper");
    shared_resources_->logger->publishInfoMessage(name(), "Requesting gripper to open");
  }
  else if (command == "close")
  {
    request.data = false;
    spdlog::info("ActuateGripper: Sending request to close gripper");
    shared_resources_->logger->publishInfoMessage(name(), "Requesting gripper to close");
  }
  else
  {
    const std::string error_msg = "Invalid command '" + command + "'. Expected 'open' or 'close'.";
    spdlog::error("ActuateGripper: {}", error_msg);
    shared_resources_->logger->publishFailureMessage(name(), error_msg);
    return tl::make_unexpected(error_msg);
  }

  return request;
}

tl::expected<bool, std::string> ActuateGripper::processResponse(const SetBool::Response& response)
{
  if (!response.success)
  {
    const std::string error_msg = "Gripper service call failed: " + response.message;
    spdlog::error("ActuateGripper: {}", error_msg);
    shared_resources_->logger->publishFailureMessage(name(), error_msg);
    return tl::make_unexpected(error_msg);
  }
  else
  {
    const std::string success_msg = "Gripper operation successful: " + response.message;
    spdlog::info("ActuateGripper: {}", success_msg);
    shared_resources_->logger->publishInfoMessage(name(), success_msg);
    return true;
  }
}

}  // namespace actuate_gripper

// Template instantiation for the ServiceClientBehaviorBase
template class moveit_studio::behaviors::ServiceClientBehaviorBase<std_srvs::srv::SetBool>;
template class moveit_studio::behaviors::ClientInterfaceBase<std_srvs::srv::SetBool>;
