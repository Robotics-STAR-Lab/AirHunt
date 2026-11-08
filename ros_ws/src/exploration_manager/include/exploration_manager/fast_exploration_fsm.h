#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;

struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, SPIN, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };

class FastExplorationFSM {
private:

  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;
  bool is_spinning_;
  ros::Time spin_start_time_;
  int spin_count_;

  bool task2_mode_;
  bool task2_pose_received_;
  Eigen::Vector3d task2_target_pose_;
  Eigen::Vector3d task2_observe_point_;

  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, task2_pose_sub_, task1_pose_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, fsm_state_pub_;

  int callExplorationPlanner();
  int callSpinPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);

  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void visualizationCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void task2PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void task1PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void visualize();
  void clearVisMarker();
  void clearAllVisualizationExceptFrontierAndTask2();

public:
  FastExplorationFSM() {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif
