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

  // Обробляємо повідомлення та відповіді ROS в окремому потоці.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  int result = 1;

  try {
    result = [&]() -> int {
      using moveit::planning_interface::MoveGroupInterface;
      MoveGroupInterface arm(node, "ur_manipulator");

      arm.setPlanningTime(5.0);
      arm.setMaxVelocityScalingFactor(0.1);
      arm.setMaxAccelerationScalingFactor(0.1);

      // Чекаємо на поточний стан суглобів.
      auto state = arm.getCurrentState(10.0);

      if (!state) {
        RCLCPP_ERROR(logger, "No current robot state received.");
        return 1;
      }

      const auto* group =
        state->getJointModelGroup("ur_manipulator");

      if (!group) {
        RCLCPP_ERROR(logger, "Planning group not found.");
        return 1;
      }

      // Визначаємо систему координат і кінцеву ланку.
      const auto planning_frame = arm.getPlanningFrame();
      const auto tool_link = arm.getEndEffectorLink();

      RCLCPP_INFO(
        logger, "Planning frame: %s; end-effector link: %s",
        planning_frame.c_str(), tool_link.c_str());

      if (tool_link.empty()) {
        RCLCPP_ERROR(logger, "End-effector link is not configured.");
        return 1;
      }

      // Поточна поза кінця руки: позиція та орієнтація.
      const auto pose = arm.getCurrentPose(tool_link);

      RCLCPP_INFO(
        logger,
        "Current tool position in %s: x=%.3f y=%.3f z=%.3f m",
        pose.header.frame_id.c_str(),
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);

      RCLCPP_INFO(
        logger,
        "Current tool orientation (quaternion): "
        "x=%.4f y=%.4f z=%.4f w=%.4f",
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);

      // Копіюємо поточні положення суглобів як основу цілі.
      std::vector<double> target;
      state->copyJointGroupPositions(group, target);

      const auto& names = group->getVariableNames();
      bool found = false;

      for (std::size_t i = 0; i < names.size(); ++i) {
        RCLCPP_INFO(
          logger, "%s: %.4f rad",
          names[i].c_str(), target[i]);

        if (names[i] == "shoulder_pan_joint") {
          target[i] += 0.2;
          found = true;

          RCLCPP_INFO(
            logger, "Target shoulder_pan_joint: %.4f rad",
            target[i]);
        }
      }

      if (!found) {
        RCLCPP_ERROR(logger, "shoulder_pan_joint not found.");
        return 1;
      }

      if (!arm.setJointValueTarget(target)) {
        RCLCPP_ERROR(logger, "Joint target is out of bounds.");
        return 1;
      }

      // Використовуємо той самий стан для початку плану.
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