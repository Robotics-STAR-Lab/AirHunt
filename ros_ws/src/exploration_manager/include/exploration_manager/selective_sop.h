#pragma once
#include <Eigen/Core>
#include <vector>
namespace fast_planner {
struct SOPResult {
  std::vector<int> order;
  double cost = 0;
  int constraints = 0;
  bool valid = false, fallback = false;
};
bool validSOPOrder(const std::vector<int>& order, const std::vector<double>& scores, double rho);
SOPResult solveSelectiveSOP(const Eigen::MatrixXd& geometry,
                            const std::vector<double>& scores, double rho);
}
