#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <future>
#include <pcl_conversions/pcl_conversions.h>

namespace fast_planner {
void FastPlannerManager::planYawActMap(const Eigen::Vector3d& start_yaw) {
  ROS_INFO("Plan yaw active mapping-----------------");
  auto t1 = ros::Time::now();
  auto t2 = ros::Time::now();

  double t_traj = local_data_.duration_;

  const int seg_num = 12;
  double dt_yaw = local_data_.duration_ / seg_num;
  const int subsp = 2;
  double dt_path = dt_yaw * subsp;
  std::cout << "duration: " << local_data_.duration_ << ", seg_num: " << seg_num
            << ", dt_yaw: " << dt_yaw << ", dt_path: " << dt_path << std::endl;

  Eigen::Vector3d start_yaw3d = start_yaw;
  while (start_yaw3d[0] < -M_PI)
    start_yaw3d[0] += 2 * M_PI;
  while (start_yaw3d[0] > M_PI)
    start_yaw3d[0] -= 2 * M_PI;
  double last_yaw = start_yaw3d[0];

  const double forward_t = 4.0 / pp_.max_vel_;
  vector<Eigen::Vector3d> pts;
  vector<double> spyaw;
  bool unmapped = false;
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;

  for (int idx = 0; idx <= seg_num; ++idx) {
    if (idx % subsp != 0) continue;

    double tc = idx * dt_yaw;
    double tf = min(local_data_.duration_, tc + forward_t);
    Eigen::Vector3d pc = local_data_.position_traj_.evaluateDeBoorT(tc);
    Eigen::Vector3d pd = local_data_.position_traj_.evaluateDeBoorT(tf) - pc;
    double yc;
    if (pd.norm() > 1e-6) {

      yc = atan2(pd(1), pd(0));
      calcNextYaw(last_yaw, yc);
    } else {
      yc = spyaw.back();
    }
    last_yaw = yc;

    pts.push_back(pc);
    spyaw.push_back(yc);
    waypt_idx.push_back(idx);
    waypts.emplace_back(yc, 0, 0);
  }
  spyaw[0] = start_yaw3d[0];

  t2 = ros::Time::now();
  vector<double> path;
  heading_planner_->searchPathOfYaw(pts, spyaw, dt_path, local_data_.position_traj_.getControlPoint(),
                                    path);
  for (int i = 0; i < path.size(); ++i) {
    waypts[i][0] = path[i];
  }

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw3d;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(local_data_.duration_ - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw3d[0], 0, 0),
                                    Eigen::Vector3d(start_yaw3d[1], 0, 0),
                                    Eigen::Vector3d(start_yaw3d[2], 0, 0) };
  vector<Eigen::Vector3d> end = { Eigen::Vector3d(end_yaw[0], 0, 0), Eigen::Vector3d(0, 0, 0) };
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS | BsplineOptimizer::START |
      BsplineOptimizer::END;
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  plan_data_.path_yaw_ = path;
  plan_data_.dt_yaw_ = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw * subsp;

  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;

}

void FastPlannerManager::searchFrontier(const Eigen::Vector3d& p) {

  frontier_finder_->searchFrontiers();
}

void FastPlannerManager::test() {
  auto t1 = ros::Time::now();
  std::cout << "test-------------------" << std::endl;

  Graph graph_yaw;
  int vid = 0;
}

bool FastPlannerManager::localExplore(Eigen::Vector3d start, Eigen::Vector3d start_vel,
                                      Eigen::Vector3d start_acc, Eigen::Vector3d goal) {
  local_data_.start_time_ = ros::Time::now();

  Eigen::Vector3d gi;
  double dist_to_goal = (goal - start).norm();
  if (dist_to_goal < 5.0) {
    gi = goal;
    std::cout << "Select final gi" << std::endl;
  } else {

    random_device rd;
    default_random_engine eng = default_random_engine(rd());
    uniform_real_distribution<double> rand_u(-1.0, 1.0);

    vector<Eigen::Vector3d> points;
    const int sample_num = 16;
    const double radius1 = 3.5;
    const double radius2 = 5.0;
    while (points.size() < sample_num) {
      Eigen::Vector3d pt;
      pt[0] = radius2 * rand_u(eng);
      pt[1] = radius2 * rand_u(eng);

      if (pt.head(2).norm() > radius1 && pt.head(2).norm() < radius2 &&
          atan2(pt[1], pt[0]) < M_PI / 3.0 && atan2(pt[1], pt[0]) > -M_PI / 3.0) {
        pt += start;
        pt[2] = 0.5 * rand_u(eng) + 1;

        Eigen::Vector3i pt_idx;
        sdf_map_->posToIndex(pt, pt_idx);
        if (sdf_map_->getOccupancy(pt_idx) == SDFMap::FREE && sdf_map_->getDistance(pt_idx) > 0.2) {
          points.push_back(pt);
        }
      }
    }

    pcl::PointCloud<pcl::PointXYZ> points_cloud;
    for (int i = 0; i < points.size(); ++i) {
      pcl::PointXYZ pt;
      pt.x = points[i][0];
      pt.y = points[i][1];
      pt.z = points[i][2];
      points_cloud.points.push_back(pt);
    }
    points_cloud.width = points_cloud.points.size();
    points_cloud.height = 1;
    points_cloud.is_dense = true;
    points_cloud.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(points_cloud, cloud_msg);

    std::cout << "Generate sample" << std::endl;

    vector<double> gains;

    for (int i = 0; i < points.size(); ++i) {
      Eigen::Vector3d dir = points[i] - start;
      double yi = atan2(dir[1], dir[0]);
      double gain = heading_planner_->calcInfoGain(points[i], yi, 0);
      gains.push_back(gain);
    }

    const double dg = (goal - start).norm() + radius1;
    const double we = 1.0;
    const double wg = 1000.0;
    int idx = -1;
    double max_score = -1;
    for (int i = 0; i < points.size(); ++i) {
      double s = we * gains[i] + wg * (dg - (goal - points[i]).norm()) / dg;

      if (s > max_score) {
        idx = i;
        max_score = s;
      }
    }
    gi = points[idx];
    std::cout << "Select intermediate gi: " << gi.transpose() << std::endl;

    points_cloud.clear();
    points_cloud.points.push_back(pcl::PointXYZ(gi[0], gi[1], gi[2]));
    points_cloud.width = points_cloud.points.size();
    points_cloud.height = 1;
    points_cloud.is_dense = true;
    points_cloud.header.frame_id = "world";
    pcl::toROSMsg(points_cloud, cloud_msg);
  }

  path_finder_->reset();
  int status = path_finder_->search(start, gi);
  if (status == Astar::NO_PATH) {
    return false;
  }
  auto path = path_finder_->getPath();
  double len = topo_prm_->pathLength(path);
  int seg_num = len / pp_.ctrl_pt_dist * 1.2;
  int ctrl_pt_num = seg_num + 3;
  double dt = (len / pp_.max_vel_) / seg_num;
  vector<Eigen::Vector3d> pts;
  topo_prm_->pathToGuidePts(path, seg_num + 1, pts);
  std::cout << "Find path" << std::endl;
  plan_data_.kino_path_ = path;

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt, (1 / 3.0) * dt * dt, 1.0, 0.0, -(1 / 6.0) * dt * dt, 1.0, dt,
      (1 / 3.0) * dt * dt;

  Eigen::Matrix3d state_xyz;
  state_xyz.row(0) = start;
  state_xyz.row(1) = start_vel;
  state_xyz.row(2) = start_acc;
  Eigen::Matrix3d p_tmp = states2pts * state_xyz;
  Eigen::Vector3d p0 = p_tmp.row(0);
  Eigen::Vector3d p1 = p_tmp.row(1);
  Eigen::Vector3d p2 = p_tmp.row(2);
  pts.insert(pts.begin(), p0);
  pts[1] = p1;
  pts[2] = p2;

  std::cout << "Set boundary value" << std::endl;

  Eigen::MatrixXd ctrl_pts(pts.size(), 3);
  for (int i = 0; i < pts.size(); ++i) {
    ctrl_pts.row(i) = pts[i];
  }

  int cost_func = BsplineOptimizer::NORMAL_PHASE;
  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);

  std::cout << "Optimze" << std::endl;

  for (int i = 0; i < 3; ++i) {
    NonUniformBspline traj(ctrl_pts, 3, dt);
    traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
    double ratio = traj.checkRatio();
    std::cout << "ratio: " << ratio << std::endl;

    dt = ratio * dt;
    states2pts << 1.0, -dt, (1 / 3.0) * dt * dt, 1.0, 0.0, -(1 / 6.0) * dt * dt, 1.0, dt,
        (1 / 3.0) * dt * dt;
    p_tmp = states2pts * state_xyz;
    ctrl_pts.block<3, 3>(0, 0) = p_tmp;
    bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  }

  std::cout << "Local explore time: " << (ros::Time::now() - local_data_.start_time_).toSec()
            << std::endl;

  local_data_.position_traj_.setUniformBspline(ctrl_pts, 3, dt);
  updateTrajInfo();

  return true;
}

}
