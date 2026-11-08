#include <exploration_manager/selective_sop.h>
#include <Eigen/Dense>
#include "sop_solver_interface.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
namespace fast_planner {
bool validSOPOrder(const std::vector<int>& order, const std::vector<double>& scores, double rho) {
  const int n = scores.size();
  if (static_cast<int>(order.size()) != n) return false;
  std::vector<int> rank(n, -1);
  for (int k = 0; k < n; ++k) {
    const int id = order[k];
    if (id < 0 || id >= n || rank[id] >= 0) return false;
    rank[id] = k;
  }
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      if (scores[i] - scores[j] > rho && rank[i] >= rank[j]) return false;
  return true;
}
SOPResult solveSelectiveSOP(const Eigen::MatrixXd& geo,
                          const std::vector<double>& scores, double rho) {
  const int n = scores.size();
  if (geo.rows() != n+1 || geo.cols() != n+1 || !geo.allFinite() ||
      (geo.array() < 0).any() || !std::isfinite(rho) || rho < 0)
    throw std::invalid_argument("Invalid SOP geometry/threshold");
  for (double s : scores)
    if (!std::isfinite(s) || s < 0 || s > 1) throw std::invalid_argument("Invalid semantic score");
  SOPResult r;
  if (n == 0) {r.valid = true; return r;}
  Eigen::MatrixXi matrix(n+1, n+1);
  for (int i = 0; i <= n; ++i)
    for (int j = 0; j <= n; ++j)
      matrix(i,j) = i == j ? 0 : static_cast<int>(std::llround(std::min(geo(i,j),1e6)*1000));
  for (int i = 1; i <= n; ++i) matrix(i,0) = -1;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      if (scores[i] - scores[j] > rho) {
        matrix(j+1,i+1) = -1;
        ++r.constraints;
      }
  if (n == 1) r.order = {0};
  else {

    static std::mutex solver_mutex;
    std::lock_guard<std::mutex> lock(solver_mutex);
    std::vector<int> path;
    solveSOP(matrix, path);
    for (int id : path) if (id != 0) r.order.push_back(id-1);
  }
  if (!validSOPOrder(r.order, scores, rho)) {

    r.fallback = true; r.order.clear();
    std::vector<bool> used(n, false);
    int current = 0;
    while (static_cast<int>(r.order.size()) < n) {
      int best = -1;
      for (int j = 0; j < n; ++j) {
        if (used[j]) continue;
        bool ready = true;
        for (int i = 0; i < n; ++i)
          if (!used[i] && scores[i]-scores[j] > rho) ready = false;
        if (ready && (best < 0 || geo(current,j+1) < geo(current,best+1))) best = j;
      }
      if (best < 0) throw std::runtime_error("Cyclic SOP constraints");
      r.order.push_back(best); used[best] = true; current = best+1;
    }
  }
  r.valid = validSOPOrder(r.order, scores, rho);
  int prev = 0;
  for (int id : r.order) { r.cost += geo(prev,id+1); prev = id+1; }
  return r;
}
}
