#pragma once
#include <Eigen/Core>
#include <osqp.h>
#include <vector>
#include <cmath>
#include <algorithm>
namespace sura_manipulation_actions::qp
{
struct SparseMatrix
{
  std::vector<c_float> values;
  std::vector<c_int> rows;
  std::vector<c_int> columns;
  csc matrix{};

  SparseMatrix(const Eigen::MatrixXd & dense, bool upper)
  {
    columns.push_back(0);
    for (Eigen::Index column = 0; column < dense.cols(); ++column) {
      for (Eigen::Index row = 0; row < dense.rows(); ++row) {
        if ((!upper || row <= column) && std::abs(dense(row, column)) > 1.0e-14) {
          values.push_back(static_cast<c_float>(dense(row, column)));
          rows.push_back(static_cast<c_int>(row));
        }
      }
      columns.push_back(static_cast<c_int>(values.size()));
    }
    matrix.nzmax = static_cast<c_int>(values.size());
    matrix.m = static_cast<c_int>(dense.rows());
    matrix.n = static_cast<c_int>(dense.cols());
    matrix.p = columns.data();
    matrix.i = rows.data();
    matrix.x = values.data();
    matrix.nz = -1;
  }
};

inline bool solveQp(
  const Eigen::MatrixXd & hessian, const Eigen::VectorXd & gradient,
  const Eigen::MatrixXd & constraints, const Eigen::VectorXd & lower,
  const Eigen::VectorXd & upper, double remaining, Eigen::VectorXd & solution)
{
  if (!hessian.allFinite() || !gradient.allFinite() || !constraints.allFinite() ||
    !lower.allFinite() || !upper.allFinite() || (lower.array() > upper.array()).any())
  {return false;}
  SparseMatrix p(0.5 * (hessian + hessian.transpose()), true), a(constraints, false);
  std::vector<c_float> q(gradient.data(), gradient.data() + gradient.size());
  std::vector<c_float> l(lower.data(), lower.data() + lower.size());
  std::vector<c_float> u(upper.data(), upper.data() + upper.size());
  OSQPData data{};
  data.n = static_cast<c_int>(gradient.size());
  data.m = static_cast<c_int>(lower.size());
  data.P = &p.matrix;
  data.A = &a.matrix;
  data.q = q.data();
  data.l = l.data();
  data.u = u.data();
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polish = 1;
  settings.max_iter = 1200;
  settings.eps_abs = 1.0e-8;
  settings.eps_rel = 1.0e-8;
#ifdef PROFILING
  settings.time_limit = std::max(1.0e-4, std::min(remaining, 0.05));
#else
  (void)remaining;
#endif
  OSQPWorkspace * workspace = nullptr;
  if (osqp_setup(&workspace, &data, &settings) != 0 || !workspace) {
    if (workspace) {osqp_cleanup(workspace);}
    return false;
  }
  const auto status = osqp_solve(workspace);
  const bool solved = status == 0 && workspace->info && workspace->solution &&
    workspace->solution->x && (workspace->info->status_val == OSQP_SOLVED ||
    workspace->info->status_val == OSQP_SOLVED_INACCURATE);
  if (solved) {
    solution.resize(gradient.size());
    for (Eigen::Index i = 0; i < solution.size(); ++i) {
      solution(i) = workspace->solution->x[i];
    }
  }
  osqp_cleanup(workspace);
  return solved && solution.allFinite();
}

}
