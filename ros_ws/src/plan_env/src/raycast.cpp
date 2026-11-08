#include <Eigen/Eigen>
#include <cmath>
#include <limits>
#include <iostream>
#include <plan_env/raycast.h>

int signum(int x) {
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds) {

  if (ds < 0) {
    return intbound(-s, -ds);
  } else {
    s = mod(s, 1);

    return (1 - s) / ds;
  }
}

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output) {

  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  Eigen::Vector3d direction = (end - start);
  double maxDist = direction.squaredNorm();

  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      output[output_points_cnt](0) = x;
      output[output_points_cnt](1) = y;
      output[output_points_cnt](2) = z;

      output_points_cnt++;
      dist = sqrt((x - start(0)) * (x - start(0)) + (y - start(1)) * (y - start(1)) +
                  (z - start(2)) * (z - start(2)));

      if (dist > maxDist) return;

    }

    if (x == endX && y == endY && z == endZ) break;

    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {

        x += stepX;

        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output) {

  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  Eigen::Vector3d direction = (end - start);
  double maxDist = direction.squaredNorm();

  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  output->clear();

  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      output->push_back(Eigen::Vector3d(x, y, z));

      dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm();

      if (dist > maxDist) return;

      if (output->size() > 1500) {
        std::cerr << "Error, too many racyast voxels." << std::endl;
        throw std::out_of_range("Too many raycast voxels");
      }
    }

    if (x == endX && y == endY && z == endZ) break;

    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {

        x += stepX;

        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

bool RayCaster::setInput(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& end
) {
  start_ = start;
  end_ = end;

  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  endZ_ = (int)std::floor(end_.z());
  direction_ = (end_ - start_);
  maxDist_ = direction_.squaredNorm();

  dx_ = endX_ - x_;
  dy_ = endY_ - y_;
  dz_ = endZ_ - z_;

  stepX_ = (int)signum((int)dx_);
  stepY_ = (int)signum((int)dy_);
  stepZ_ = (int)signum((int)dz_);

  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);
  tMaxZ_ = intbound(start_.z(), dz_);

  tDeltaX_ = ((double)stepX_) / dx_;
  tDeltaY_ = ((double)stepY_) / dy_;
  tDeltaZ_ = ((double)stepZ_) / dz_;

  dist_ = 0;

  step_num_ = 0;

  if (stepX_ == 0 && stepY_ == 0 && stepZ_ == 0)
    return false;
  else
    return true;
}

bool RayCaster::step(Eigen::Vector3d& ray_pt) {

  ray_pt = Eigen::Vector3d(x_, y_, z_);

  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  if (tMaxX_ < tMaxY_) {
    if (tMaxX_ < tMaxZ_) {

      x_ += stepX_;

      tMaxX_ += tDeltaX_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  } else {
    if (tMaxY_ < tMaxZ_) {
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  }

  return true;
}

void RayCaster::setParams(const double& res, const Eigen::Vector3d& origin) {
  resolution_ = res;
  half_ = Eigen::Vector3d(0.5, 0.5, 0.5);
  offset_ = half_ - origin / resolution_;
}

bool RayCaster::input(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
  start_ = start / resolution_;
  end_ = end / resolution_;

  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  endZ_ = (int)std::floor(end_.z());
  direction_ = (end_ - start_);
  maxDist_ = direction_.squaredNorm();

  dx_ = endX_ - x_;
  dy_ = endY_ - y_;
  dz_ = endZ_ - z_;

  stepX_ = (int)signum((int)dx_);
  stepY_ = (int)signum((int)dy_);
  stepZ_ = (int)signum((int)dz_);

  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);
  tMaxZ_ = intbound(start_.z(), dz_);

  tDeltaX_ = ((double)stepX_) / dx_;
  tDeltaY_ = ((double)stepY_) / dy_;
  tDeltaZ_ = ((double)stepZ_) / dz_;

  dist_ = 0;

  step_num_ = 0;

  if (stepX_ == 0 && stepY_ == 0 && stepZ_ == 0)
    return false;
  else
    return true;
}

bool RayCaster::nextId(Eigen::Vector3i& idx) {
  auto tmp = Eigen::Vector3d(x_, y_, z_);
  idx = (tmp + offset_).cast<int>();

  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  const double inf = std::numeric_limits<double>::infinity();
  if (x_ == endX_) tMaxX_ = inf;
  if (y_ == endY_) tMaxY_ = inf;
  if (z_ == endZ_) tMaxZ_ = inf;

  if (tMaxX_ < tMaxY_) {
    if (tMaxX_ < tMaxZ_) {

      x_ += stepX_;

      tMaxX_ += tDeltaX_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  } else {
    if (tMaxY_ < tMaxZ_) {
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  }

  return true;
}

bool RayCaster::nextPos(Eigen::Vector3d& pos) {
  auto tmp = Eigen::Vector3d(x_, y_, z_);
  pos = (tmp + half_) * resolution_;

  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  const double inf = std::numeric_limits<double>::infinity();
  if (x_ == endX_) tMaxX_ = inf;
  if (y_ == endY_) tMaxY_ = inf;
  if (z_ == endZ_) tMaxZ_ = inf;

  if (tMaxX_ < tMaxY_) {
    if (tMaxX_ < tMaxZ_) {

      x_ += stepX_;

      tMaxX_ += tDeltaX_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  } else {
    if (tMaxY_ < tMaxZ_) {
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  }

  return true;
}
