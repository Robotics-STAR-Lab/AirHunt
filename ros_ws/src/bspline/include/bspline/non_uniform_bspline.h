#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace fast_planner {

class NonUniformBspline {
public:
  NonUniformBspline();
  NonUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);
  ~NonUniformBspline();

  void setUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);

  void setKnot(const Eigen::VectorXd& knot);
  Eigen::VectorXd getKnot();
  Eigen::MatrixXd getControlPoint();
  double getKnotSpan();
  void getTimeSpan(double& um, double& um_p);

  Eigen::VectorXd evaluateDeBoor(const double& u);
  Eigen::VectorXd evaluateDeBoorT(const double& t);
  void computeDerivatives(const int& k, vector<NonUniformBspline>& ders);
  NonUniformBspline getDerivative();
  void getBoundaryStates(const int& ks, const int& ke, vector<Eigen::Vector3d>& start,
                         vector<Eigen::Vector3d>& end);

  static void parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                    const vector<Eigen::Vector3d>& start_end_derivative,
                                    const int& degree, Eigen::MatrixXd& ctrl_pts);

  void setPhysicalLimits(const double& vel, const double& acc);
  void lengthenTime(const double& ratio);
  double checkRatio();

  bool checkFeasibility(bool show = false);
  bool reallocateTime(bool show = false);

  double getTimeSum();
  double getLength(const double& res = 0.01);
  double getJerk();
  void getMeanAndMaxVel(double& mean_v, double& max_v);
  void getMeanAndMaxAcc(double& mean_a, double& max_a);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  Eigen::MatrixXd control_points_;

  int p_, n_, m_;
  Eigen::VectorXd u_;
  double knot_span_;

  Eigen::MatrixXd getDerivativeControlPoints();

  double limit_vel_, limit_acc_, limit_ratio_;
};
}
#endif
