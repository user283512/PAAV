#include "particle/circle_fit.hpp"

Circle CircleFitByCeres(const std::vector<float> &aVecX,
                        const std::vector<float> &aVecY,
                        float aRadious)
{
  double x = aVecX.at(0);
  double y = aVecY.at(0);

  ceres::Problem problem;
  for (size_t i = 0; i < aVecX.size(); i++)
  {
    ceres::CostFunction *cost =
        new ceres::AutoDiffCostFunction<CircleFitting, 1, 1, 1>(new CircleFitting(aVecX.at(i), aVecY.at(i), aRadious));
    problem.AddResidualBlock(cost, nullptr, &x, &y);
  }

  // Build and solve the problem.
  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  return {x, y, aRadious};
}
