#include <exception>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// Position specifies the CENTER of the box, in meters.
moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id,
  double size_x, double size_y, double size_z,
  double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {size_x, size_y, size_z};

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  return object;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("setup_scene");

  int result = 1;

  try {
    // Connects to the running MoveIt planning scene services.
    moveit::planning_interface::PlanningSceneInterface scene;

    std::vector<moveit_msgs::msg::CollisionObject> objects;

    // Table: 1.2 x 1.2 m, thickness 8 cm.
    // Center z = -0.06 -> upper surface z = -0.02 m.
    objects.push_back(makeBox(
      "table",
      1.20, 1.20, 0.08,
      0.0, 0.0, -0.06));

    // Obstacle: bottom at z = -0.02, resting on the table.
    objects.push_back(makeBox(
      "obstacle",
      0.15, 0.20, 0.40,
      0.45, 0.0, 0.18));

    if (!scene.applyCollisionObjects(objects)) {
      RCLCPP_ERROR(logger, "MoveIt rejected the scene update.");
    } else {
      // Verify that both IDs can be read back from MoveIt.
      const auto stored = scene.getObjects({"table", "obstacle"});

      if (stored.count("table") && stored.count("obstacle")) {
        RCLCPP_INFO(logger, "Verified scene objects: table, obstacle.");
        result = 0;
      } else {
        RCLCPP_ERROR(logger, "Scene update could not be verified.");
      }
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(logger, "Exception: %s", error.what());
  }

  rclcpp::shutdown();
  return result;
}