#ifndef SOP_SOLVER_INTERFACE_H
#define SOP_SOLVER_INTERFACE_H
#include <Eigen/Core>
#include <vector>
int solveSOP(const Eigen::MatrixXi &cost_matrix, std::vector<int> &path);
#endif
