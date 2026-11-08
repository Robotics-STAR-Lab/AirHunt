#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

namespace fast_planner {

double ViewNode::vm_;
double ViewNode::am_;
double ViewNode::yd_;
double ViewNode::ydd_;
double ViewNode::w_dir_;
shared_ptr<Astar> ViewNode::astar_;
shared_ptr<RayCaster> ViewNode::caster_;
shared_ptr<SDFMap> ViewNode::map_;

ViewNode::ViewNode(const Vector3d& p, const double& y) {
  pos_ = p;
  yaw_ = y;
  parent_ = nullptr;
  vel_.setZero();
}

double ViewNode::costTo(const ViewNode::Ptr& node) {
  vector<Vector3d> path;
  double c = ViewNode::computeCost(pos_, node->pos_, yaw_, node->yaw_, vel_, yaw_dot_, path);

  return c;
}

double ViewNode::searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path) {

  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_->getInflateOccupancy(idx) == 1 || map_->getOccupancy(idx) == SDFMap::UNKNOWN ||
        !map_->isInBox(idx)) {
      safe = false;
      break;
    }
  }
  if (safe) {
    path = { p1, p2 };
    return (p1 - p2).norm();
  }

  vector<double> res = { 0.4 };
  for (int k = 0; k < res.size(); ++k) {
    astar_->reset();
    astar_->setResolution(res[k]);
    if (astar_->search(p1, p2) == Astar::REACH_END) {
      path = astar_->getPath();
      return astar_->pathLength(path);
    }
  }

  path = { p1, p2 };
  return 1000;
}

double ViewNode::computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1, const double& y2,
                             const Vector3d& v1, const double& yd1, vector<Vector3d>& path) {

  double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;

  if (v1.norm() > 1e-3) {
    Vector3d dir = (p2 - p1).normalized();
    Vector3d vdir = v1.normalized();
    double diff = acos(vdir.dot(dir));
    pos_cost += w_dir_ * diff;

  }

  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = diff / yd_;
  return max(pos_cost, yaw_cost);

}
}
