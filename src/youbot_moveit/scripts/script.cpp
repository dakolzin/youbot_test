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

  /*---------------- 1. Home ----------------*/
  RCLCPP_INFO(log, "[1] Home");
  arm.setMaxVelocityScalingFactor(0.5);
  arm.setJointValueTarget({0,0,0,0,0});
  {
    Plan p;
    if (arm.plan(p) == moveit::core::MoveItErrorCode::SUCCESS) arm.execute(p);
  }
  rclcpp::sleep_for(std::chrono::seconds(2));

  /*---------------- 2. Open gripper ----------------*/
  RCLCPP_INFO(log, "[2] Open gripper");
  gripper.setNamedTarget("Open");
  {
    Plan p;
    if (gripper.plan(p) == moveit::core::MoveItErrorCode::SUCCESS) gripper.execute(p);
  }
  rclcpp::sleep_for(std::chrono::seconds(2));

  /*---- read pose from server ----*/
  geometry_msgs::msg::PoseStamped pose_grasp;
  if (!get_target_pose("127.0.0.1", 5000, pose_grasp, log))
  { rclcpp::shutdown(); return 1; }

  const double dz = 0.05;
  geometry_msgs::msg::PoseStamped pose_pre = pose_grasp;
  pose_pre.pose.position.z += dz;

  arm.setPlanningTime(10);
  arm.setMaxVelocityScalingFactor(0.4);
  arm.setMaxAccelerationScalingFactor(0.4);
  arm.setNumPlanningAttempts(20);

  auto try_pose = [&](const geometry_msgs::msg::PoseStamped& tgt, const char* tag)->bool{
    for (int i=1;i<=5;++i){
      RCLCPP_INFO(log, "%s attempt %d/5", tag, i);
      arm.setPoseTarget(tgt);
      Plan p;
      if (arm.plan(p) == moveit::core::MoveItErrorCode::SUCCESS){
        arm.execute(p); return true;
      }
      RCLCPP_WARN(log, "plan failed");
    }
    RCLCPP_ERROR(log, "%s FAILED", tag);
    return false;
  };

  /*---------------- 3. Pre-grasp with retries ----------------*/
  if (!try_pose(pose_pre , "[3] Pre-grasp")) { rclcpp::shutdown(); return 1; }
  rclcpp::sleep_for(std::chrono::seconds(1));

  /*---------------- 4. Descend with retries ----------------*/
  if (!try_pose(pose_grasp, "[4] Descend" )) { rclcpp::shutdown(); return 1; }
  rclcpp::sleep_for(std::chrono::seconds(1));

  /*---------------- 5. Close gripper ----------------*/
  RCLCPP_INFO(log, "[5] Close gripper");
  gripper.setNamedTarget("Close");
  {
    Plan p;
    if (gripper.plan(p) == moveit::core::MoveItErrorCode::SUCCESS) gripper.execute(p);
  }
  rclcpp::sleep_for(std::chrono::seconds(1));

  /*---------------- 6. Lift with retries ----------------*/
  if (!try_pose(pose_pre , "[6] Lift"    )) { rclcpp::shutdown(); return 1; }
  rclcpp::sleep_for(std::chrono::seconds(2));

  /*---------------- 7. Home with object ----------------*/
  RCLCPP_INFO(log, "[7] Home with object");
  arm.setJointValueTarget({0,0,0,0,0});
  {
    Plan p;
    if (arm.plan(p) == moveit::core::MoveItErrorCode::SUCCESS) arm.execute(p);
  }

  rclcpp::shutdown();
  return 0;
}
