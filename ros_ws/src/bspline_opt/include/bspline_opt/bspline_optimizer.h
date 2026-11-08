#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <active_perception/traj_visibility.h>
#include <ros/ros.h>

namespace fast_planner {
class EDTEnvironment;

class BsplineOptimizer {
public:
  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int START;
  static const int END;
  static const int GUIDE;
  static const int WAYPOINTS;
  static const int VIEWCONS;
  static const int MINTIME;

  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE;

  BsplineOptimizer() {
  }
  ~BsplineOptimizer() {
  }

  void setEnvironment(const shared_ptr<EDTEnvironment>& env);
  void setParam(ros::NodeHandle& nh);
  void optimize(Eigen::MatrixXd& points, double& dt, const int& cost_function, const int& max_num_id,
                const int& max_time_id);

  void setCostFunction(const int& cost_function);
  void setBoundaryStates(const vector<Eigen::Vector3d>& start, const vector<Eigen::Vector3d>& end);
  void setTimeLowerBound(const double& lb);

  void setGuidePath(const vector<Eigen::Vector3d>& guide_pt);
  void setWaypoints(const vector<Eigen::Vector3d>& waypts,
                    const vector<int>& waypt_idx);
  void setViewConstraint(const ViewConstraint& vc);
  void enableDynamic(double time_start);

  void optimize();

  Eigen::MatrixXd getControlPoints();
  vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);

private:

  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  void combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  void calcSmoothnessCost(const vector<Eigen::Vector3d>& q, const double& dt, double& cost,
                          vector<Eigen::Vector3d>& gradient_q, double& gt);
  void calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient_q);
  void calcFeasibilityCost(const vector<Eigen::Vector3d>& q, const double& dt, double& cost,
                           vector<Eigen::Vector3d>& gradient_q, double& gt);
  void calcStartCost(const vector<Eigen::Vector3d>& q, const double& dt, double& cost,
                     vector<Eigen::Vector3d>& gradient_q, double& gt);
  void calcEndCost(const vector<Eigen::Vector3d>& q, const double& dt, double& cost,
                   vector<Eigen::Vector3d>& gradient_q, double& gt);
  void calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                     vector<Eigen::Vector3d>& gradient_q);
  void calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                         vector<Eigen::Vector3d>& gradient_q);
  void calcViewCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient_q);
  void calcTimeCost(const double& dt, double& cost, double& gt);
  bool isQuadratic();

  shared_ptr<EDTEnvironment> edt_environment_;

  Eigen::MatrixXd control_points_;
  double knot_span_;

  int dim_;
  vector<Eigen::Vector3d> start_state_, end_state_;
  vector<Eigen::Vector3d> guide_pts_;
  vector<Eigen::Vector3d> waypoints_;
  vector<int> waypt_idx_;
  int max_num_id_, max_time_id_;
  int cost_function_;
  double time_lb_;
  bool dynamic_;
  double start_time_;

  int order_;
  int bspline_degree_;
  double ld_smooth_, ld_dist_, ld_feasi_, ld_start_, ld_end_, ld_guide_, ld_waypt_, ld_view_, ld_time_;
  double dist0_;
  double max_vel_, max_acc_;
  double wnl_, dlmin_;
  int algorithm1_;
  int algorithm2_;
  int max_iteration_num_[4];
  double max_iteration_time_[4];

  vector<Eigen::Vector3d> g_q_, g_smoothness_, g_distance_, g_feasibility_, g_start_, g_end_, g_guide_,
      g_waypoints_, g_view_, g_time_;

  int variable_num_;
  int point_num_;
  bool optimize_time_;
  int iter_num_;
  std::vector<double> best_variable_;
  double min_cost_;
  ViewConstraint view_cons_;
  double pt_dist_;

public:
  vector<double> vec_cost_;
  vector<double> vec_time_;
  ros::Time time_start_;

  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  double comb_time;

  typedef unique_ptr<BsplineOptimizer> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
#endif
