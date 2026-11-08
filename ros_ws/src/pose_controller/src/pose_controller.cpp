#include <Eigen/Eigen>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <string>
#include <vector>
ros::Publisher odom_pub;
double init_state_x, init_state_y, init_state_z, init_state_yaw;
std::shared_ptr<nav_msgs::Odometry> odom_ptr;

void positionCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd) {

  odom_ptr->header.stamp = cmd->header.stamp;

  odom_ptr->pose.pose.position.x = cmd->position.x;
  odom_ptr->pose.pose.position.y = cmd->position.y;
  odom_ptr->pose.pose.position.z = cmd->position.z;

  const double g = 9.81;

  Eigen::Vector3d a_thrust(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z + g);
  Eigen::Vector3d z_body = a_thrust.normalized();

  double yaw = cmd->yaw;
  Eigen::Vector3d x_c(cos(yaw), sin(yaw), 0.0);

  Eigen::Vector3d x_proj = x_c - x_c.dot(z_body) * z_body;

  Eigen::Vector3d x_body, y_body;
  if (x_proj.norm() < 1e-6) {

    Eigen::Vector3d y_c(-sin(yaw), cos(yaw), 0.0);
    x_proj = y_c - y_c.dot(z_body) * z_body;
    if (x_proj.norm() < 1e-6) {

      x_body = Eigen::Vector3d(1, 0, 0);
    } else {
      x_body = x_proj.normalized();
    }
  } else {
    x_body = x_proj.normalized();
  }

  y_body = z_body.cross(x_body).normalized();

  Eigen::Matrix3d R;
  R.col(0) = x_body;
  R.col(1) = y_body;
  R.col(2) = z_body;

  Eigen::Quaterniond q(R);
  q.normalize();

  odom_ptr->pose.pose.orientation.w = q.w();
  odom_ptr->pose.pose.orientation.x = q.x();
  odom_ptr->pose.pose.orientation.y = q.y();
  odom_ptr->pose.pose.orientation.z = q.z();

  odom_ptr->twist.twist.linear.x = cmd->velocity.x;
  odom_ptr->twist.twist.linear.y = cmd->velocity.y;
  odom_ptr->twist.twist.linear.z = cmd->velocity.z;

  odom_ptr->twist.twist.angular.x = 0.0;
  odom_ptr->twist.twist.angular.y = 0.0;
  odom_ptr->twist.twist.angular.z = cmd->yaw_dot;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "wonderful_controler");
  ros::NodeHandle nh("~");
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Subscriber cmd_sub = nh.subscribe("position_cmd", 1, positionCommandCallback);
  nh.param("init_state_x", init_state_x, 0.0);
  nh.param("init_state_y", init_state_y, 0.0);
  nh.param("init_state_z", init_state_z, 0.0);
  nh.param("init_state_yaw", init_state_yaw, 0.0);
  odom_ptr = std::make_shared<nav_msgs::Odometry>();
  odom_ptr->header.frame_id = "world";
  odom_ptr->pose.pose.position.x = init_state_x;
  odom_ptr->pose.pose.position.y = init_state_y;
  odom_ptr->pose.pose.position.z = init_state_z;

  // The launch argument is already expressed in the ROS world frame.
  std::cout << "Initial yaw in radians: " << init_state_yaw << std::endl;
  Eigen::Quaterniond init_q(Eigen::AngleAxisd(init_state_yaw, Eigen::Vector3d::UnitZ()));
  odom_ptr->pose.pose.orientation.w = init_q.w();
  odom_ptr->pose.pose.orientation.x = init_q.x();
  odom_ptr->pose.pose.orientation.y = init_q.y();
  odom_ptr->pose.pose.orientation.z = init_q.z();

  ros::Timer timer = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent &) { odom_pub.publish(*odom_ptr); });
  ros::spin();
  ros::shutdown();
  return 0;
}
