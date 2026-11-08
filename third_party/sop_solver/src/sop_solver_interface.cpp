#include <Eigen/Eigen>
#include <chrono>
#include <vector>
#include <unordered_set>
#include "digraph.h"
#include "edge.h"
#include "solver.h"
#include "sop_solver_interface.h"
using std::vector;
using std::unordered_set;

void creat_graphs_from_matrix(const Eigen::MatrixXi &cost_matrix, Digraph &g, Digraph &p) {
  g.set_size(cost_matrix.rows());
  p.set_size(cost_matrix.rows());
  for (int i = 0; i < cost_matrix.rows(); ++i) {
    for (int j = 0; j < cost_matrix.cols(); ++j) {
      if (cost_matrix(i, j) < 0) {
        p.add_edge(i, j, 0);
      } else if (i != j) {
        g.add_edge(i, j, cost_matrix(i, j));
      }
    }
  }
}

void remove_redundant_edges(Digraph &g, Digraph &p) {
  for (int i = 0; i < p.node_count(); ++i) {
    const vector<Edge> &preceding_nodes = p.adj_outgoing(i);
    unordered_set<int> expanded_nodes;
    for (int j = 0; j < preceding_nodes.size(); ++j) {
      vector<Edge> st;
      st.push_back(preceding_nodes[j]);
      while (!st.empty()) {
        Edge dependence_edge = st.back();
        st.pop_back();
        if (dependence_edge.source != i) {
          g.remove_edge(dependence_edge.dest, i);
          expanded_nodes.insert(dependence_edge.dest);
        }
        for (const Edge &e : p.adj_outgoing(dependence_edge.dest)) {
          if (expanded_nodes.find(e.dest) == expanded_nodes.end()) {
            st.push_back(e);
            expanded_nodes.insert(e.dest);
          }
        }
      }
    }
  }
}

void remove_redundant_edge_successors(Digraph &g, Digraph &p) {
  for (int i = 0; i < p.node_count(); ++i) {
    const vector<Edge> &preceding_nodes = p.adj_incoming(i);
    unordered_set<int> expanded_nodes;
    for (int j = 0; j < preceding_nodes.size(); ++j) {
      vector<Edge> st;
      st.push_back(preceding_nodes[j]);
      while (!st.empty()) {
        Edge dependence_edge = st.back();
        st.pop_back();
        if (dependence_edge.source != i) {
          g.remove_edge(i, dependence_edge.dest);
          expanded_nodes.insert(dependence_edge.dest);
        }
        for (const Edge &e : p.adj_outgoing(dependence_edge.dest)) {
          if (expanded_nodes.find(e.dest) == expanded_nodes.end()) {
            st.push_back(e);
            expanded_nodes.insert(e.dest);
          }
        }
      }
    }
  }
}

int solveSOP(const Eigen::MatrixXi &cost_matrix, vector<int> &path) {

  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::chrono::duration<double> elapsed_time;

  Digraph g;
  Digraph p;
  creat_graphs_from_matrix(cost_matrix, g, p);

  g.sort_edges();
  remove_redundant_edges(g, p);
  remove_redundant_edge_successors(g, p);

  Solver::set_cost_matrix(g.dense_hungarian());

  Solver s = Solver(&g, &p);
  s.set_time_limit(0.1, false);

  s.set_hash_size(10000);

  s.nearest_neighbor();

  start = std::chrono::system_clock::now();

  int static_lower_bound = s.get_static_lower_bound();
  int nearest_neighbor_cost = s.best_solution_cost();

  s.solve_sop_parallel(1);

  int best_solution_cost = s.best_solution_cost();

  path.clear();
  for (Edge e : s.best_solution_path()) {
    path.push_back(e.dest);
  }

  s.clear();

  return best_solution_cost;
}
