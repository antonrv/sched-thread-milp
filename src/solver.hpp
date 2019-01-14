#ifndef SOLVER_SCHED_THREAD_MILP_HPP
#define SOLVER_SCHED_THREAD_MILP_HPP

#include "problem.hpp"

#include <memory>
#include <vector>

class GRBEnv;

namespace stm {

class solver {
public:
  solver() = delete;
  solver(int argc, char **argv);
  ~solver();
  void run();

private:
  bool is_feasible(const std::vector<double> &task2startTime,
                   const std::vector<double> &task2endTime,
                   const matrix_t<bool> &matFullPrecedence) const;
  void rescale(matrix_t<double> &matDelay);

  std::unique_ptr<const problem> m_problemDefPtr;
  GRBEnv *m_grbEnv = nullptr;
  double m_intFeasTol = 0.0;
  double m_timeScale = 1.0;
};

} // namespace stm

#endif // SOLVER_THREAD_TASK_SCHEDULING_HPP
