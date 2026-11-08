#ifndef _TRAJ_VISIBILITY_H_
#define _TRAJ_VISIBILITY_H_

#include <plan_env/edt_environment.h>
#include <unordered_map>
#include <utility>
#include <bspline/non_uniform_bspline.h>

class RayCaster;

namespace fast_planner {
struct VisiblePair {
  int from_;
  int to_;
  Eigen::Vector3d qb_;
};

struct ViewConstraint {
  Eigen::Vector3d pt_;
  Eigen::Vector3d pc_;
  Eigen::Vector3d dir_;
  Eigen::Vector3d pcons_;
  int idx_;
};

class VisibilityUtil {
private:

  int visible_num_;
  double min_visib_;
  unique_ptr<RayCaster> caster_;
  EDTEnvironment::Ptr edt_env_;
  double resolution_;
  Eigen::Vector3d offset_;
  double max_safe_dist_, safe_margin_;
  double max_acc_, r0_, wnl_, forward_;

public:
  VisibilityUtil();
  VisibilityUtil(const ros::NodeHandle& nh);
  ~VisibilityUtil();
  void setEDTEnvironment(const EDTEnvironment::Ptr& edt);

  vector<Eigen::Vector3d> precomputeForVisibility(const vector<Eigen::Vector3d>& ctrl_pts,
                                                  bool debug = false);
  void findVisibPairs(const vector<Eigen::Vector3d>& pts, vector<VisiblePair>& pairs);
  void calcViewConstraint(NonUniformBspline& traj, ViewConstraint& cons);

private:
  Eigen::Vector3d getMinDistVoxel(const Eigen::Vector3d& q1, const Eigen::Vector3d& q2,
                                  const Eigen::Vector3d& offset, const double& res);
  Eigen::Vector3d getMinDistVoxelOnLine(const Eigen::Vector3d& q1, const Eigen::Vector3d& q2,
                                        const Eigen::Vector3d& offset, const double& res, int& state,
                                        Eigen::Vector3d& block);
  Eigen::Vector3d getVirtualBlockPt(const vector<Eigen::Vector3d>& q, int i, int j,
                                    const Eigen::Vector3d& min_pt);
  bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, Eigen::Vector3d& pc);
  bool findUnknownPoint(NonUniformBspline& traj, Eigen::Vector3d& point, double& time);
  bool findCriticalPoint(NonUniformBspline& traj, const Eigen::Vector3d& unknown_pt,
                         const double& unknown_t, Eigen::Vector3d& pc, double& tc);
  bool findDirAndIdx(NonUniformBspline& traj, const double& unknown_t, const double& crit_t,
                     Eigen::Vector3d& dir, int& idx, Eigen::Vector3d& min_pt);
};

}
#endif
