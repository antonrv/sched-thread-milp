#ifndef STM_PROBLEM_HPP
#define STM_PROBLEM_HPP

#include "parser.hpp"

#include <string>
#include <vector>

namespace stm {

template <typename T> using matrix_t = std::vector<std::vector<T>>;

enum class optimize { time, energy, none };

class problem {
public:
  int get_task_count() const { return m_taskCount; }
  int get_max_thread_configurations() const { return m_threadConfigCount; }
  int get_processor_capacity() const { return m_capacity; }
  bool is_energy_modelled() const { return m_energyModelled; }
  const matrix_t<double> &get_delay_matrix() const { return m_matDelay; }
  const matrix_t<double> &
  get_power_matrix(const matrix_t<int> &matCapacity) const {
    return m_matPower;
  }
  const matrix_t<int> &get_capacity_matrix() const { return m_matCapacity; }
  const matrix_t<bool> &get_precedence_matrix() const {
    return m_matPrecedence;
  }
  const matrix_t<bool> &get_full_precedence_matrix() const {
    return m_matFullPrecedence;
  }
  double get_idle_power() const { return m_idlePower; }
  double get_max_power() const { return m_maxPower; }
  bool do_presolve() const { return m_presolve; }
  bool do_in_order() const { return m_inOrder; }
  const std::string &input_solution() const { return m_inputSolutionFilename; }
  optimize optimize_what() const { return m_optimize; }
  double get_bigm() const { return m_bigM; }

  problem(const parser::arg_dict_t &argDict);

private:
  void set_full_precedence_matrix();

  int m_taskCount = 0;
  int m_threadConfigCount = 0;
  int m_capacity = 0;

  matrix_t<double> m_matDelay;
  matrix_t<double> m_matPower;
  matrix_t<int> m_matCapacity;
  matrix_t<bool> m_matPrecedence;
  matrix_t<bool> m_matFullPrecedence;

  double m_idlePower = 0.0;
  double m_maxPower = 0.0;
  bool m_energyModelled = false;
  double m_lowerBound = 0.0;

  bool m_presolve = false;
  bool m_inOrder = false;

  std::string m_inputSolutionFilename;

  optimize m_optimize = optimize::time;

  double m_bigM = 0.0;
  // maube deprecated

  // Initial solution
  std::vector<int> m_threadAssignments;
  std::vector<double> m_startTimes;

  // Handler inner state
  bool m_initialized = false;
};

} // namespace stm

#endif // STM_PROBLEM_HPP
