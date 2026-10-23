#include <exception>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "first_motion",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true));

  const auto logger = node->get_logger();

  // Process incoming states and action responses while main() waits.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int result = 1;

  try {
    result = [&]() -> int {
      using moveit::planning_interface::MoveGroupInterface;
      MoveGroupInterface arm(node, "ur_manipulator");

      arm.setPlanningTime(5.0);
      arm.setMaxVelocityScalingFactor(0.1);
      arm.setMaxAccelerationScalingFactor(0.1);

      auto state = arm.getCurrentState(10.0);
      if (!state) {
        RCLCPP_ERROR(logger, "No current robot state received.");
        return 1;
      }

      const auto* group = state->getJointModelGroup("ur_manipulator");
      if (!group) {
        RCLCPP_ERROR(logger, "Planning group not found.");
        return 1;
      }

      std::vector<double> target;
      state->copyJointGroupPositions(group, target);
      const auto& names = group->getVariableNames();

      bool found = false;
      for (std::size_t i = 0; i < names.size(); ++i) {
        RCLCPP_INFO(
          logger, "%s: %.4f rad", names[i].c_str(), target[i]);

        if (names[i] == "shoulder_pan_joint") {
          target[i] += 0.2;
          found = true;
        }
      }

      if (!found || !arm.setJointValueTarget(target)) {
        RCLCPP_ERROR(logger, "Target joint missing or target out of bounds.");
        return 1;
      }

      // Plan from the same state used to calculate the target.
      arm.setStartState(*state);

      MoveGroupInterface::Plan plan;
      if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Planning failed; nothing will execute.");
        return 1;
      }

      RCLCPP_INFO(
        logger, "Plan found: %zu trajectory points. Executing...",
        plan.trajectory.joint_trajectory.points.size());

      if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Execution failed.");
        return 1;
      }

      RCLCPP_INFO(logger, "Motion completed successfully.");
      return 0;
    }();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(logger, "Exception: %s", error.what());
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result;
}