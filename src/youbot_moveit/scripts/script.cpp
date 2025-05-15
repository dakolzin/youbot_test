#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

/* TCP ----------------------------------------------------------------*/
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>
#include <cstring>
/*---------------------------------------------------------------------*/

using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

/*---------------------------------------------------------------------------*/
static bool get_target_pose(const std::string& ip, int port,
                            geometry_msgs::msg::PoseStamped& pose,
                            const rclcpp::Logger& log)
{
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) { RCLCPP_ERROR(log, "socket() failed"); return false; }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(port);
  if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
    RCLCPP_ERROR(log, "inet_pton() failed"); close(sock); return false;
  }

  if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(log, "connect() failed to %s:%d", ip.c_str(), port);
    close(sock); return false;
  }

  char buf[256]{};
  ssize_t n = read(sock, buf, sizeof(buf) - 1);
  close(sock);
  if (n <= 0) { RCLCPP_ERROR(log, "read() failed"); return false; }

  double x, y, z, qx, qy, qz, qw;
  std::istringstream iss(buf);
  if (!(iss >> x >> y >> z >> qx >> qy >> qz >> qw)) {
    RCLCPP_ERROR(log, "parse error: \"%s\"", buf); return false;
  }

  pose.header.frame_id  = "base_link";
  pose.pose.position.x  = x;
  pose.pose.position.y  = y;
  pose.pose.position.z  = z;
  pose.pose.orientation.x = qx;
  pose.pose.orientation.y = qy;
  pose.pose.orientation.z = qz;
  pose.pose.orientation.w = qw;

  RCLCPP_INFO(log, "Pose: %.3f %.3f %.3f", x, y, z);
  return true;
}

/*---------------------------------------------------------------------------*/
static bool try_pose(moveit::planning_interface::MoveGroupInterface& arm,
                     const geometry_msgs::msg::PoseStamped& tgt,
                     const char* tag,
                     const rclcpp::Logger& log)
{
  arm.setPoseTarget(tgt);
  const auto& p = tgt.pose;
  RCLCPP_INFO(log,
              "%s target:  %.3f %.3f %.3f | ori: %.3f %.3f %.3f %.3f",
              tag,
              p.position.x, p.position.y, p.position.z,
              p.orientation.x, p.orientation.y,
              p.orientation.z, p.orientation.w);
  for (int i = 1; i <= 2; ++i) {
    RCLCPP_INFO(log, "%s attempt %d/2", tag, i);
    Plan p;
    if (arm.plan(p) == moveit::core::MoveItErrorCode::SUCCESS) {
      if (arm.execute(p) == moveit::core::MoveItErrorCode::SUCCESS) return true;
    }
    RCLCPP_WARN(log, "plan/execute failed");
  }
  RCLCPP_ERROR(log, "%s FAILED", tag);
  return false;
}

/*---------------------------------------------------------------------------*/
static bool pick_sequence(moveit::planning_interface::MoveGroupInterface& arm,
                          moveit::planning_interface::MoveGroupInterface& gripper,
                          const geometry_msgs::msg::PoseStamped& grasp,
                          const rclcpp::Logger& log)
{
  const double dz = 0.04;
  geometry_msgs::msg::PoseStamped pre = grasp;
  pre.pose.position.z += dz;

  arm.setPlanningTime(10);
  arm.setMaxVelocityScalingFactor(0.4);
  arm.setMaxAccelerationScalingFactor(0.4);
  arm.setNumPlanningAttempts(20);

  /*---------------- 1. Pre‑grasp ----------------*/
  if (!try_pose(arm, pre, "[Pre‑grasp]", log)) return false;
  rclcpp::sleep_for(std::chrono::seconds(1));

  /*---------------- 2. Descend ----------------*/
  if (!try_pose(arm, grasp, "[Descend]", log)) return false;
  rclcpp::sleep_for(std::chrono::seconds(1));

  /*---------------- 3. Close gripper ----------------*/
  RCLCPP_INFO(log, "[Close gripper]");
  gripper.setNamedTarget("Close");
  {
    Plan p;
    if (gripper.plan(p) != moveit::core::MoveItErrorCode::SUCCESS ||
        gripper.execute(p) != moveit::core::MoveItErrorCode::SUCCESS)
      return false;
  }
  rclcpp::sleep_for(std::chrono::seconds(1));

  /*---------------- 4. Lift ----------------*/
  if (!try_pose(arm, pre, "[Lift]", log)) return false;
  rclcpp::sleep_for(std::chrono::seconds(2));

  /*---------------- 5. Home with object ----------------*/
  RCLCPP_INFO(log, "[Home with object]");
  arm.setJointValueTarget({0,0,0,0,0});
  {
    Plan p;
    if (arm.plan(p) != moveit::core::MoveItErrorCode::SUCCESS ||
        arm.execute(p) != moveit::core::MoveItErrorCode::SUCCESS)
      return false;
  }
  return true;
}

/*---------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_group_node");
  auto log  = node->get_logger();

  auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

  constexpr char ARM_GROUP[]     = "youbot_arm";
  constexpr char GRIPPER_GROUP[] = "youbot_gripper";

  moveit::planning_interface::MoveGroupInterface arm(node, ARM_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper(node, GRIPPER_GROUP);

  arm.setPlannerId(node->declare_parameter("arm_planner", "RRTConnectkConfigDefault"));
  gripper.setPlannerId(node->declare_parameter("gripper_planner", "RRTConnectkConfigDefault"));

  /*---------------- Home ----------------*/
  RCLCPP_INFO(log, "[Home]");
  arm.setMaxVelocityScalingFactor(0.5);
  arm.setJointValueTarget({0,0,0,0,0});
  {
    Plan p;
    if (arm.plan(p) == moveit::core::MoveItErrorCode::SUCCESS) arm.execute(p);
  }
  rclcpp::sleep_for(std::chrono::seconds(2));

  /*---------------- Open gripper ----------------*/
  RCLCPP_INFO(log, "[Open gripper]");
  gripper.setNamedTarget("Open");
  {
    Plan p;
    if (gripper.plan(p) == moveit::core::MoveItErrorCode::SUCCESS) gripper.execute(p);
  }
  rclcpp::sleep_for(std::chrono::seconds(2));

  /*---------------- Main retry loop ----------------*/
  while (rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped pose_grasp;
    if (!get_target_pose("127.0.0.1", 5000, pose_grasp, log)) {
      RCLCPP_ERROR(log, "Failed to receive pose – aborting.");
      break;
    }

    if (pick_sequence(arm, gripper, pose_grasp, log)) {
      RCLCPP_INFO(log, "Pick succeeded.");
      break;
    }

    RCLCPP_WARN(log, "Pick failed – requesting new target pose.");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  rclcpp::shutdown();
  return 0;
}
