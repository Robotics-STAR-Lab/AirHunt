#ifndef _PERCEPTION_UTILS_H_
#define _PERCEPTION_UTILS_H_

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <iostream>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class PerceptionUtils {
public:
  PerceptionUtils(ros::NodeHandle& nh);
  ~PerceptionUtils() {
  }

  void setPose(const Vector3d& pos, const double& yaw);

  void getFOV(vector<Vector3d>& list1, vector<Vector3d>& list2);
  bool insideFOV(const Vector3d& point);
  void getFOVBoundingBox(Vector3d& bmin, Vector3d& bmax);

private:

  Vector3d pos_;
  double yaw_;

  vector<Vector3d> normals_;

  double left_angle_, right_angle_, top_angle_, max_dist_, vis_dist_;

  Vector3d n_top_, n_bottom_, n_left_, n_right_;

  Eigen::Matrix4d T_cb_, T_bc_;

  vector<Vector3d> cam_vertices1_, cam_vertices2_;
};

}
#endif
