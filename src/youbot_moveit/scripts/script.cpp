#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

// Функция для установки цвета объекта
void set_object_color(const std::string& object_id, const std_msgs::msg::ColorRGBA& color,
                      rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& planning_scene_publisher)
{
  moveit_msgs::msg::ObjectColor object_color;
  object_color.id = object_id;
  object_color.color = color;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.object_colors.push_back(object_color);

  planning_scene_publisher->publish(planning_scene);
}

void set_object_color_on_attach(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                const std::string& object_id,
                                rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& planning_scene_publisher)
{
  std_msgs::msg::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 1.0;
  set_object_color(object_id, color, planning_scene_publisher);
}

void set_object_color_on_detach(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                const std::string& object_id,
                                rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& planning_scene_publisher)
{
  std_msgs::msg::ColorRGBA color;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 1.0;
  set_object_color(object_id, color, planning_scene_publisher);
}

// Функция добавляет объект столкновения в планировщик сцены MoveIt
void add_collision_object(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "target_box";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 0.018;
  box.dimensions[1] = 0.018;
  box.dimensions[2] = 0.018;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = -0.000771;
  box_pose.orientation.x = 0.950070;
  box_pose.orientation.y = -0.312034;
  box_pose.orientation.z = -0.001299;
  box_pose.position.x = 0.371882;
  box_pose.position.y = -0.234594;
  box_pose.position.z = -0.03;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface.applyCollisionObject(collision_object);
}

void add_collision_object_2(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "box_2";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 0.2;
  box.dimensions[1] = 0.02;
  box.dimensions[2] = 0.2;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.orientation.x = 0.0;
  box_pose.orientation.y = 0.0;
  box_pose.orientation.z = 0.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.05;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface.applyCollisionObject(collision_object);
}

void add_collision_object_3(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "box_3";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 0.4;
  box.dimensions[1] = 0.5;
  box.dimensions[2] = 0.05;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.orientation.x = 0.0;
  box_pose.orientation.y = 0.0;
  box_pose.orientation.z = 0.0;
  box_pose.position.x = 0.53;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.08;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface.applyCollisionObject(collision_object);
}

void add_collision_object_4(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "box_4";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 0.4;
  box.dimensions[1] = 0.5;
  box.dimensions[2] = 0.05;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.orientation.x = 0.0;
  box_pose.orientation.y = 0.0;
  box_pose.orientation.z = 0.0;
  box_pose.position.x = 0.52;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.38;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface.applyCollisionObject(collision_object);
}

void attach_object(moveit::planning_interface::MoveGroupInterface& move_group, const std::string& object_id)
{
  const std::string gripper_link = "gripper";
  move_group.attachObject(object_id, gripper_link);
}

void detach_object(moveit::planning_interface::MoveGroupInterface& move_group, const std::string& object_id)
{
  move_group.detachObject(object_id);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_group_node");
  auto planning_scene_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

  // Настройка трансформера
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

  // Определение планировочных групп
  const std::string ARM_PLANNING_GROUP = "youbot_arm";
  const std::string GRIPPER_PLANNING_GROUP = "youbot_gripper";
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit::planning_interface::MoveGroupInterface move_group(node, ARM_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper_move_group(node, GRIPPER_PLANNING_GROUP);

  // Задаём и логируем используемые планировщики для руки и захвата.
  std::string arm_planner;
  if (!node->has_parameter("arm_planner"))
    node->declare_parameter("arm_planner", "RRTConnectkConfigDefault");
  node->get_parameter("arm_planner", arm_planner);
  move_group.setPlannerId(arm_planner);
  RCLCPP_INFO(node->get_logger(), "Initial arm planner: %s", arm_planner.c_str());

  std::string gripper_planner;
  if (!node->has_parameter("gripper_planner"))
    node->declare_parameter("gripper_planner", "RRTConnectkConfigDefault");
  node->get_parameter("gripper_planner", gripper_planner);
  gripper_move_group.setPlannerId(gripper_planner);
  RCLCPP_INFO(node->get_logger(), "Initial gripper planner: %s", gripper_planner.c_str());

  // Добавляем объекты столкновения в сцену
  add_collision_object(planning_scene_interface);
  add_collision_object_2(planning_scene_interface);
  add_collision_object_3(planning_scene_interface);
  add_collision_object_4(planning_scene_interface);

  // Основной цикл повторений
  for (int i = 1; i <= 10 && rclcpp::ok(); ++i)
  {
    // Возвращение объекта в первоначальное положение
    add_collision_object(planning_scene_interface);

    RCLCPP_INFO(node->get_logger(), "/===============================================================/");
    RCLCPP_INFO(node->get_logger(), "/======================== Attempt %d ========================/", i);
    RCLCPP_INFO(node->get_logger(), "/===============================================================/");
    RCLCPP_INFO(node->get_logger(), "/======================== Returning home =======================/");

    // Планирование для позиционирования "home"
    RCLCPP_INFO(node->get_logger(), "Attempt %d: Using arm planner: %s for returning home", i, arm_planner.c_str());
    double velocity_scaling_factor = 0.5;
    move_group.setMaxVelocityScalingFactor(velocity_scaling_factor);
    std::vector<double> target_joint_values_0 = {0.0, 0.0, 0.0, 0.0, 0.0};
    move_group.setJointValueTarget(target_joint_values_0);

    moveit::planning_interface::MoveGroupInterface::Plan joint_plan_0;
    bool joint_success_0 = (move_group.plan(joint_plan_0) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing joint plan №1 %s", joint_success_0 ? "" : "FAILED");

    if (joint_success_0 && rclcpp::ok())
    {
      RCLCPP_INFO(node->get_logger(), "Executing the joint trajectory.");
      move_group.execute(joint_plan_0);
    }

    if (!rclcpp::ok())
      break;
    RCLCPP_INFO(node->get_logger(), "Sleeping between moves.");
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Открытие захвата
    RCLCPP_INFO(node->get_logger(), "Attempt %d: Using gripper planner: %s for opening the gripper", i, gripper_planner.c_str());
    gripper_move_group.setNamedTarget("Open");
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool gripper_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing gripper plan (open) %s", gripper_success ? "" : "FAILED");

    if (gripper_success && rclcpp::ok())
    {
      RCLCPP_INFO(node->get_logger(), "Executing the gripper open trajectory.");
      gripper_move_group.execute(gripper_plan);
    }

    if (!rclcpp::ok())
      break;
    RCLCPP_INFO(node->get_logger(), "Sleeping between moves.");
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Движение (#1)
    RCLCPP_INFO(node->get_logger(), "Attempt %d: Using arm planner: %s for move (#1)", i, arm_planner.c_str());
    std::vector<double> target_joint_values = {3.822, 2.34, -2.26, 3.45, 0.0};
    move_group.setJointValueTarget(target_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    bool joint_success = (move_group.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing joint plan №1 %s", joint_success ? "" : "FAILED");

    if (joint_success && rclcpp::ok())
    {
      RCLCPP_INFO(node->get_logger(), "Executing the joint trajectory.");
      move_group.execute(joint_plan);
    }

    if (!rclcpp::ok())
      break;
    RCLCPP_INFO(node->get_logger(), "Sleeping between moves.");
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Движение (#2)
    RCLCPP_INFO(node->get_logger(), "Attempt %d: Using arm planner: %s for move (#2)", i, arm_planner.c_str());
    std::vector<double> target_joint_values_1 = {3.82, 2.46, -1.99, 3.05, 0.0};
    move_group.setJointValueTarget(target_joint_values_1);

    moveit::planning_interface::MoveGroupInterface::Plan joint_plan_1;
    bool joint_success_1 = (move_group.plan(joint_plan_1) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing joint plan №2 %s", joint_success_1 ? "" : "FAILED");

    if (joint_success_1 && rclcpp::ok())
    {
      RCLCPP_INFO(node->get_logger(), "Executing the joint trajectory.");
      move_group.execute(joint_plan_1);

      RCLCPP_INFO(node->get_logger(), "Attaching the object к схвату.");
      attach_object(move_group, "target_box");

      RCLCPP_INFO(node->get_logger(), "Changing the object color on attach.");
      set_object_color_on_attach(planning_scene_interface, "target_box", planning_scene_publisher);
    }

    if (!rclcpp::ok())
      break;
    RCLCPP_INFO(node->get_logger(), "Sleeping between moves.");
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Закрытие захвата
    RCLCPP_INFO(node->get_logger(), "Attempt %d: Using gripper planner: %s for closing the gripper", i, gripper_planner.c_str());
    gripper_move_group.setNamedTarget("Close");
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_2;
    bool gripper_success_2 = (gripper_move_group.plan(gripper_plan_2) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing gripper plan (close) %s", gripper_success_2 ? "" : "FAILED");

    if (gripper_success_2 && rclcpp::ok())
    {
      RCLCPP_INFO(node->get_logger(), "Executing the gripper close trajectory.");
      gripper_move_group.execute(gripper_plan_2);
    }

    if (!rclcpp::ok())
      break;
    RCLCPP_INFO(node->get_logger(), "Sleeping between moves.");
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Движение (#3) с позовой целью
    RCLCPP_INFO(node->get_logger(), "=======================================================================");
    RCLCPP_INFO(node->get_logger(), "Attempt %d: Using arm planner: %s for move (#3)", i, arm_planner.c_str());
    RCLCPP_INFO(node->get_logger(), "=======================================================================");
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    move_group.setNumPlanningAttempts(20);

    geometry_msgs::msg::PoseStamped target_pose_stamped;
    target_pose_stamped.header.frame_id = "base_link";
    target_pose_stamped.pose.orientation.w = 0.000243;
    target_pose_stamped.pose.orientation.x = 0.928814;
    target_pose_stamped.pose.orientation.y = 0.370542;
    target_pose_stamped.pose.orientation.z = -0.001494;
    target_pose_stamped.pose.position.x = 0.433988;
    target_pose_stamped.pose.position.y = 0.160393;
    target_pose_stamped.pose.position.z = 0.043095;
    move_group.setPoseTarget(target_pose_stamped);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success && rclcpp::ok())
    {
      RCLCPP_INFO(node->get_logger(), "Executing the planned trajectory.");
      move_group.execute(my_plan);
    }

    if (!rclcpp::ok())
      break;
    RCLCPP_INFO(node->get_logger(), "Sleeping between moves.");
    rclcpp::sleep_for(std::chrono::seconds(10));

    // Открытие захвата после движения (#3)
    RCLCPP_INFO(node->get_logger(), "Attempt %d: Using gripper planner: %s for opening the gripper after move (#3)", i, gripper_planner.c_str());
    gripper_move_group.setNamedTarget("Open");
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_3;
    bool gripper_success_3 = (gripper_move_group.plan(gripper_plan_3) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing gripper plan (open) %s", gripper_success_3 ? "" : "FAILED");

    if (gripper_success_3 && rclcpp::ok())
    {
      RCLCPP_INFO(node->get_logger(), "Executing the gripper open trajectory.");
      gripper_move_group.execute(gripper_plan_3);
    }
    
    RCLCPP_INFO(node->get_logger(), "Detaching the object from the gripper.");
    detach_object(move_group, "target_box");

    RCLCPP_INFO(node->get_logger(), "Changing the object color on detach.");
    set_object_color_on_detach(planning_scene_interface, "target_box", planning_scene_publisher);

    std::vector<std::string> object_ids = {"target_box"};
    planning_scene_interface.removeCollisionObjects(object_ids);

    if (!rclcpp::ok())
      break;
    RCLCPP_INFO(node->get_logger(), "Sleeping between moves.");
    rclcpp::sleep_for(std::chrono::seconds(3));
  }

  rclcpp::shutdown();
  return 0;
}
