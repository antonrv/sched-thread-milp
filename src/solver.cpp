#define _GLIBCXX_USE_CXX11_ABI 0

#include "solver.hpp"

#include "problem.hpp"

#include "gurobi_c++.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <thread>
#include <vector>
static double StrictEpsilon = 1e-6;

namespace stm {

using key_tu_t = std::array<size_t, 2>;

static void print_gurobi_status(const int grbStatus) {
  switch (grbStatus) {
  case GRB_LOADED:
    std::cout << "Model is loaded, but no solution information is available.\n";
    break;
  case GRB_OPTIMAL:
    std::cout << "Model was solved to optimality (subject to tolerances).\n";
    break;
  case GRB_INFEASIBLE:
    std::cout << "Model was proven to be infeasible.\n";
    break;
  case GRB_INF_OR_UNBD:
    std::cout << "Model was proven to be either infeasible or unbounded.\n";
    break;
  case GRB_UNBOUNDED:
    std::cout << "Model was proven to be unbounded.\n";
    break;
  case GRB_CUTOFF:
    std::cout << "Optimal objective for model was proven to be worse than "
                 "the value specified in the Cutoff parameter. No solution "
                 "information is available.\n";
    break;
  case GRB_ITERATION_LIMIT:
    std::cout << "Optimization terminated because the total number of "
                 "simplex iterations performed exceeded the value specified "
                 "in the IterationLimit parameter, or because the total "
                 "number of barrier iterations exceeded the value specified "
                 "in the BarIterLimit parameter.\n";
    break;
  case GRB_NODE_LIMIT:
    std::cout << "Optimization terminated because the total number of "
                 "branch-and-cut nodes explored exceeded the value specified "
                 "in the NodeLimit parameter.\n";
    break;
  case GRB_TIME_LIMIT:
    std::cout << "Optimization terminated because the time expended exceeded "
                 "the value specified in the TimeLimit parameter.\n";
    break;
  case GRB_SOLUTION_LIMIT:
    std::cout << "Optimization terminated because the number of solutions "
                 "found reached the value specified in the SolutionLimit "
                 "parameter.\n";
    break;
  case GRB_INTERRUPTED:
    std::cout << "Optimization was terminated by the user.\n";
    break;
  case GRB_NUMERIC:
    std::cout << "Optimization was terminated due to unrecoverable numerical "
                 "difficulties.\n";
    break;
  case GRB_SUBOPTIMAL:
    std::cout << "Unable to satisfy optimality tolerances; a sub-optimal "
                 "solution is available.\n";
    break;
  case GRB_INPROGRESS:
    std::cout << "An asynchronous optimization call was made, but the "
                 "associated optimization run is not yet complete.\n";
    break;
  case GRB_USER_OBJ_LIMIT:
    std::cout << "User specified an objective limit (a bound on either the "
                 "best objective or the best bound), and that limit has been "
                 "reached.\n";
    break;
  default:
    std::cout << "Unhandled GRB Status.\n";
    std::cout.flush();
    break;
  }
}

static void print_solution(const std::vector<int> &tcPos,
                           const std::vector<double> &taskStartTimes,
                           const std::vector<double> &taskEndTimes,
                           const std::vector<double> &taskDelays,
                           const std::vector<double> &taskPower,
                           const std::vector<double> &taskEnergy) {
  const auto n = tcPos.size();
  std::cout << "task,tconfig,start,delay,end,power,energy\n";
  for (auto t = 0u; t < n; ++t) {
    std::cout << t << "," << tcPos[t] << "," << taskStartTimes[t] << ","
              << taskDelays[t] << "," << taskEndTimes[t] << "," << taskPower[t]
              << "," << taskEnergy[t] << '\n';
  }
}

solver::solver(int argc, char **argv) {

  const auto argMap = parser::build_map(argc, argv);

  auto tol = 1e-9;
  auto nThreads = argMap.find("--threads") != argMap.end()
                      ? std::stoi(argMap.at("--threads"))
                      : std::thread::hardware_concurrency();
  std::cout << "solver::Assigned " << nThreads
            << " threads. [set : --threads=<INT>]\n";
  auto secTimeOut = argMap.find("--timeout") != argMap.end()
                        ? std::stoi(argMap.at("--timeout"))
                        : 10;
  std::cout << "solver::Timeout of " << secTimeOut
            << " seconds. [set : --timeout=<SEC>]\n";

  try {
    m_grbEnv = new GRBEnv();
    m_grbEnv->set(GRB_DoubleParam_FeasibilityTol, tol);
    m_grbEnv->set(GRB_DoubleParam_IntFeasTol, tol);
    m_intFeasTol = m_grbEnv->get(GRB_DoubleParam_IntFeasTol);
    m_grbEnv->set(GRB_DoubleParam_MIPGap, tol);
    m_grbEnv->set(GRB_DoubleParam_TimeLimit, secTimeOut);
    m_grbEnv->set(GRB_IntParam_Threads, nThreads);
    m_grbEnv->set(GRB_IntParam_OutputFlag, 1);

  } catch (GRBException &e) {
    auto grbmsg = e.getMessage();
    auto grberr = e.getErrorCode();
    std::cerr << "Gurobi exception in solver: " << grbmsg << '\n'
              << grberr << '\n';
  }
  m_problemDefPtr = std::make_unique<const problem>(argMap);
}

solver::~solver() {}

void solver::rescale(matrix_t<double> &matDelay) {
  const auto nTasks = matDelay.size();
  const auto nConfigs = matDelay.front().size();
  auto cum = 0.0;
  for (auto t = 0; t < nTasks; ++t) {
    cum = std::accumulate(matDelay[t].begin(), matDelay[t].end(), cum);
  }
  m_timeScale = 1; // cum / (nTasks * nConfigs); // cum / (nTasks * nConfigs);
  std::cout << "Rescaled times: / " << m_timeScale << "\n";
  for (auto t = 0; t < nTasks; ++t) {
    for (auto h = 0; h < nConfigs; ++h) {
      matDelay[t][h] /= m_timeScale;
    }
  }
}

static bool can_overlap(const size_t t, const size_t u,
                        const matrix_t<bool> &matFullPrecedence) {
  return t != u && matFullPrecedence[t][u] == false &&
         matFullPrecedence[u][t] == false;
}

static bool do_overlap(const size_t t, const size_t u,
                       const std::vector<double> &task2startTime,
                       const std::vector<double> &task2endTime) {
  if (task2endTime[t] > task2startTime[u] &&
      task2endTime[u] > task2startTime[t]) {
    return true;
  } else {
    return false;
  }
}

bool solver::is_feasible(const std::vector<double> &task2startTime,
                         const std::vector<double> &task2endTime,
                         const matrix_t<bool> &matFullPrecedence) const {
  bool feasible = true;
  const auto nTasks = task2startTime.size();
  for (int u = 0; u < nTasks; ++u) {
    for (int t = u - 1; t >= 0; --t) {
      if (matFullPrecedence[t][u] == 1 &&
          (task2endTime[t] - task2startTime[u]) > m_intFeasTol + 1e-12) {
        feasible = false;
        std::cout << "Error: Precedence constraints violated by "
                  << task2endTime[t] - task2startTime[u] << ". Task: " << t
                  << " preceeds " << u << " but end time [" << t
                  << "]: " << task2endTime[t] << " > start time [" << u
                  << "]: " << task2startTime[u] << std::endl;
      }
    }
  }
  return feasible;
}
template <typename T> static void print_matrix(const matrix_t<T> &mat) {
  for (auto &row : mat) {
    for (auto col : row) {
      std::cout << col << ",";
    }
    std::cout << '\n';
  }
}

static bool file_exists(const std::string &str) {
  std::ifstream ifs(str.c_str());
  return ifs.good();
}

static void free_grb(GRBEnv *grbEnv, std::vector<GRBVar *> &grbX, GRBVar *grbS,
                     std::map<key_tu_t, GRBVar *> &grbPhi, const size_t nTasks,
                     const matrix_t<bool> &matFullPrecedence) {
  for (auto t = 0u; t < nTasks; ++t) {
    delete[] grbX[t];
  }
  delete[] grbS;
  for (auto t = 0u; t < nTasks; ++t) {
    for (auto u = 0u; u < nTasks; ++u) {
      if (can_overlap(t, u, matFullPrecedence)) {
        delete[] grbPhi[key_tu_t({t, u})];
      }
    }
  }
  delete grbEnv;
}

void solver::run() {
  const auto nTasks = m_problemDefPtr->get_task_count();
  const auto nConfigs = m_problemDefPtr->get_max_thread_configurations();
  const auto maxCores = m_problemDefPtr->get_processor_capacity();

  const bool presolve = m_problemDefPtr->do_presolve();
  const double maxPower = m_problemDefPtr->get_max_power();
  const bool inOrderExecution = m_problemDefPtr->do_in_order();
  const std::string inputSolutionFilename = m_problemDefPtr->input_solution();

  const optimize opt = m_problemDefPtr->optimize_what();

  const double bigM = m_problemDefPtr->get_bigm();

  // Initialize matrices
  auto matDelay = m_problemDefPtr->get_delay_matrix();
  rescale(matDelay);

  auto matCapacity = m_problemDefPtr->get_capacity_matrix();
  matrix_t<double> matPower = m_problemDefPtr->get_power_matrix(matCapacity);
  double idlePower = 0;
  if ((maxPower > 0 || opt == optimize::energy)) {
    if (m_problemDefPtr->is_energy_modelled()) {
      idlePower = m_problemDefPtr->get_idle_power();
    } else {
      throw std::runtime_error(
          "No power/energy models available for power constraint or energy "
          "optimization");
    }
  }
  auto matPrecedence = m_problemDefPtr->get_precedence_matrix();
  auto matFullPrecedence = m_problemDefPtr->get_full_precedence_matrix();

  // Set upper bound
  double M = 0;
  if (bigM > 0) {
    M = bigM;
  } else {
    M = 0;
    for (auto t = 0u; t < nTasks; ++t) {
      auto maxRow = *std::max_element(matDelay[t].begin(), matDelay[t].end());
      if (M < maxRow) {
        M = maxRow;
      }
    }
    M *= nTasks;
  }

  std::vector<GRBVar *> grbX(nTasks);
  std::map<key_tu_t, GRBVar> grbDelta;
  std::map<key_tu_t, GRBVar> grbOmega;
  std::map<key_tu_t, GRBVar *> grbPhi;

  // Build LP model and run solver
  GRBVar *grbS;
  try {
    // Build model
    GRBModel grbModel = GRBModel(*m_grbEnv);
    grbModel.set(GRB_StringAttr_ModelName, "Thread scheduling problem");

    // allocate memory for decision variables
    for (auto t = 0u; t < nTasks; ++t) {
      grbX[t] = grbModel.addVars(nConfigs, GRB_BINARY);
    }
    grbS = grbModel.addVars(nTasks, GRB_SEMICONT);

    for (auto t = 0u; t < nTasks; ++t) {
      for (auto u = 0u; u < nTasks; ++u) {
        if (can_overlap(t, u, matFullPrecedence)) {
          std::string varName_delta_tu =
              "delta_" + std::to_string(t) + "_" + std::to_string(u);
          std::string varName_omega_tu =
              "omega_" + std::to_string(t) + "_" + std::to_string(u);
          if (grbDelta.find({t, u}) != grbDelta.end()) {
            throw std::logic_error("key repeated in delta");
          }
          if (grbOmega.find({t, u}) != grbOmega.end()) {
            throw std::logic_error("key repeated in omega");
          }
          if (grbPhi.find({t, u}) != grbPhi.end()) {
            throw std::logic_error("key repeated in phi");
          }
          grbDelta.insert(std::pair<key_tu_t, GRBVar>(
              {t, u}, grbModel.addVar(0.0, 1.0, 0.0, GRB_BINARY,
                                      varName_delta_tu.c_str())));
          grbOmega.insert(std::pair<key_tu_t, GRBVar>(
              {t, u}, grbModel.addVar(0.0, 1.0, 0.0, GRB_BINARY,
                                      varName_omega_tu.c_str())));
          grbPhi.insert(std::pair<key_tu_t, GRBVar *>(
              {t, u}, grbModel.addVars(nConfigs, GRB_BINARY)));
        }
      }
    }
    auto deltaSize = grbOmega.size();
    auto omegaSize = grbOmega.size();
    auto phiSize = grbPhi.size();

    // allocate makespan variable
    GRBVar grbMakespan = grbModel.addVar(0, M, 1, GRB_SEMICONT, "makespan");

    // set objective function
    GRBLinExpr obj = 0;
    if (opt == optimize::time) {
      obj += grbMakespan;
    } else if (opt == optimize::energy) {
      for (auto t = 0u; t < nTasks; ++t) {
        for (auto h = 0u; h < nConfigs; ++h) {
          obj += matPower[t][h] * matDelay[t][h] * grbX[t][h];
        }
      }
      obj += grbMakespan * idlePower;
    } else {
      throw std::runtime_error("Optimization target to implement");
    }
    grbModel.setObjective(obj, GRB_MINIMIZE);

    // ==================== set constraints
    // Constraints 0. Only one config for task
    for (auto t = 0u; t < nTasks; ++t) {
      GRBLinExpr expr = 0;
      for (auto h = 0u; h < nConfigs; ++h) {
        expr += grbX[t][h];
      }
      std::string s = "c0_t" + std::to_string(t);
      grbModel.addConstr(expr, GRB_EQUAL, 1, s);
    }

    // Constraints 1. Makespan is fixed from start and end times
    for (auto t = 0u; t < nTasks; ++t) {
      GRBLinExpr expr = grbS[t];
      for (auto h = 0u; h < nConfigs; ++h) {
        expr += grbX[t][h] * matDelay[t][h];
      }
      std::string s = "c1_t" + std::to_string(t);
      grbModel.addConstr(expr, GRB_LESS_EQUAL, grbMakespan, s);
    }

    // Constraints 2
    for (auto t = 0u; t < nTasks; ++t) {
      for (auto u = 0u; u < nTasks; ++u) {
        if (t != u && matPrecedence[t][u]) {
          // Only if t preceeds u, then u have to wait to start
          GRBLinExpr expr = grbS[t] - grbS[u];
          std::string s = "c2_t" + std::to_string(t) + "_t" + std::to_string(u);

          for (auto h = 0u; h < nConfigs; ++h) {
            expr += grbX[t][h] * matDelay[t][h];
          }
          grbModel.addConstr(expr, GRB_LESS_EQUAL, 0, s);
        }
      }
    }

    // Constraints 3. Delta are set from them.
    for (auto t = 0u; t < nTasks; ++t) {
      for (auto u = 0u; u < nTasks; ++u) {
        if (can_overlap(t, u, matFullPrecedence)) {
          // Only add constr if they are not dependable
          if (grbDelta.find({t, u}) == grbDelta.end()) {
            throw std::logic_error("key not found in delta");
          }
          GRBLinExpr expr = grbS[u] - grbS[t] + M * grbDelta[{t, u}];
          grbModel.addConstr(expr, GRB_GREATER_EQUAL, StrictEpsilon,
                             "c3a_t" + std::to_string(t) + "_u" +
                                 std::to_string(u));
          grbModel.addConstr(expr, GRB_LESS_EQUAL, M,
                             "c3b_t" + std::to_string(t) + "_u" +
                                 std::to_string(u));
        } else {
          // do not constraint anything
        }
      }
    }

    // Constraints 4. Omegas are set from them.
    for (auto t = 0u; t < nTasks; ++t) {
      for (auto u = 0u; u < nTasks; ++u) {
        if (can_overlap(t, u, matFullPrecedence)) {
          // Only add constr if they are not dependable
          if (grbOmega.find({t, u}) == grbOmega.end()) {
            throw std::logic_error("key not found in omega");
          }
          GRBLinExpr expr = grbS[t] - grbS[u] + M * grbOmega[{t, u}];
          GRBLinExpr exprSum = 0;
          for (auto h = 0u; h < nConfigs; ++h) {
            exprSum += grbX[u][h] * matDelay[u][h];
          }
          expr = expr - exprSum;
          grbModel.addConstr(expr, GRB_GREATER_EQUAL, 0,
                             "c4a_t" + std::to_string(t) + "_u" +
                                 std::to_string(u));
          grbModel.addConstr(expr - M, GRB_LESS_EQUAL, -StrictEpsilon,
                             "c4b_t" + std::to_string(t) + "_u" +
                                 std::to_string(u));
        } else {
          // do not constraint anything
        }
      }
    }

    // Constraints 5. Phis are constrained by deltas and omegas.
    for (auto t = 0u; t < nTasks; ++t) {
      for (auto u = 0u; u < nTasks; ++u) {
        if (can_overlap(t, u, matFullPrecedence)) {
          if (grbDelta.find({t, u}) == grbDelta.end()) {
            throw std::logic_error("key not found in delta");
          }
          if (grbOmega.find({u, t}) == grbOmega.end()) {
            throw std::logic_error("key not found in omega");
          }
          if (grbPhi.find({t, u}) == grbPhi.end()) {
            throw std::logic_error("key not found in phi");
          }
          for (auto h = 0u; h < nConfigs; ++h) {
            GRBLinExpr expr = grbPhi[{t, u}][h] - grbDelta[{t, u}] -
                              grbOmega[{t, u}] - grbX[u][h] + 2;
            grbModel.addConstr(expr, GRB_GREATER_EQUAL, 0,
                               "c5a_t" + std::to_string(t) + "_u" +
                                   std::to_string(u) + "_h" +
                                   std::to_string(h));
            grbModel.addConstr(
                grbPhi[{t, u}][h], GRB_LESS_EQUAL, grbDelta[{t, u}],
                "c5b_t" + std::to_string(t) + "_u" + std::to_string(u) + "_h" +
                    std::to_string(h));
            grbModel.addConstr(
                grbPhi[{t, u}][h], GRB_LESS_EQUAL, grbOmega[{t, u}],
                "c5c_t" + std::to_string(t) + "_u" + std::to_string(u) + "_h" +
                    std::to_string(h));
            grbModel.addConstr(grbPhi[{t, u}][h], GRB_LESS_EQUAL, grbX[u][h],
                               "c5d_t" + std::to_string(t) + "_u" +
                                   std::to_string(u) + "_h" +
                                   std::to_string(h));
          }
        } else {
          // do not constraint anything
        }
      }
    }

    // Constraints 6. Cores are finite
    for (auto t = 0u; t < nTasks; ++t) {
      GRBLinExpr exprThisCap = 0;
      for (auto h = 0u; h < nConfigs; ++h) {
        exprThisCap += grbX[t][h] * matCapacity[t][h];
      }
      auto nPotentialOverlaps = 0;
      GRBLinExpr exprNeighCap = 0;
      for (auto u = 0u; u < nTasks; ++u) {
        if (can_overlap(t, u, matFullPrecedence)) {
          if (grbPhi.find({t, u}) == grbPhi.end()) {
            throw std::logic_error("key not found in phi");
          }
          for (auto h = 0u; h < nConfigs; ++h) {
            exprNeighCap += grbPhi[{t, u}][h] * matCapacity[u][h];
          }
          nPotentialOverlaps++;
        }
      }
      if (nPotentialOverlaps) {
        GRBLinExpr finalExpr = exprThisCap + exprNeighCap;
        std::string s = "c6_t" + std::to_string(t);
        grbModel.addConstr(finalExpr, GRB_LESS_EQUAL, maxCores, s);
      }
    }

    if (maxPower > 0) {
      // Constraints 7. Power is limited
      for (auto t = 0u; t < nTasks; ++t) {
        GRBLinExpr exprThisPow = 0;
        for (auto h = 0u; h < nConfigs; ++h) {
          exprThisPow += grbX[t][h] * matPower[t][h];
        }
        auto nPotentialOverlaps = 0;
        GRBLinExpr exprNeighPow = 0;
        for (auto u = 0u; u < nTasks; ++u) {
          if (can_overlap(t, u, matFullPrecedence)) {
            if (grbPhi.find({t, u}) == grbPhi.end()) {
              throw std::logic_error("key not found in phi");
            }
            for (auto h = 0u; h < nConfigs; ++h) {
              exprNeighPow += grbPhi[{t, u}][h] * matPower[u][h];
            }
            nPotentialOverlaps++;
          }
        }
        if (nPotentialOverlaps) {
          GRBLinExpr finalExpr = exprThisPow + exprNeighPow + idlePower;
          std::string s = "c7a_t" + std::to_string(t);
          grbModel.addConstr(finalExpr, GRB_LESS_EQUAL, maxPower, s);
        } else {
          // no potential overlaps but task could exceed maxPower by itself
          std::string s = "c7b_t" + std::to_string(t);
          grbModel.addConstr(exprThisPow + idlePower, GRB_LESS_EQUAL, maxPower,
                             s);
        }
      }
    }
    if (inOrderExecution) {
      for (auto t = 1u; t < nTasks; ++t) {
        GRBLinExpr expr = grbS[t] - grbS[t - 1];
        std::string s = "c8_t" + std::to_string(t);
        grbModel.addConstr(expr, GRB_GREATER_EQUAL, 0, s);
      }
    }

    grbModel.update(); // not sure if needed
    if (grbOmega.size() != omegaSize || grbPhi.size() != phiSize) {
      throw std::logic_error("map sizes varied during constraints");
    }
    // ================ Run solver
    if (!inputSolutionFilename.empty()) {
      if (file_exists(inputSolutionFilename + ".hnt")) {
        grbModel.read(inputSolutionFilename + ".hnt");
      }
      if (file_exists(inputSolutionFilename + ".mst")) {
        grbModel.read(inputSolutionFilename + ".mst");
      }
      if (file_exists(inputSolutionFilename + ".sol")) {
        grbModel.read(inputSolutionFilename + ".sol");
      }
    } else {
      // start from scratch, do nothing
    }
    grbModel.optimize();

    // =============== Get solution
    auto grbStatus = grbModel.get(GRB_IntAttr_Status);
    print_gurobi_status(grbStatus);

    if (grbStatus != GRB_OPTIMAL && grbStatus != GRB_SUBOPTIMAL &&
        grbStatus != GRB_TIME_LIMIT) {
      std::cout << "M: " << M << ". pow constr: " << maxPower
                << ". idle pow: " << idlePower << '\n';
      std::cerr << "No solution to output. Returning M\n";
      throw M;
    }

    static constexpr size_t PosUndef = -1;
    std::vector<int> task2tcPosition(nTasks, (int)PosUndef);
    std::vector<double> task2startTime(nTasks);
    std::vector<double> task2endTime(nTasks);
    std::vector<double> task2delay(nTasks);
    std::vector<double> task2power(nTasks);
    std::vector<double> task2energy(nTasks);
    std::map<key_tu_t, int> finalPhi;
    auto makespan = grbMakespan.get(GRB_DoubleAttr_X);
    auto dynEnergy = 0.0;
    auto staticEnergy = m_timeScale * makespan * idlePower;
    for (auto t = 0u; t < nTasks; ++t) {
      for (auto h = 0u; h < nConfigs; ++h) {
        if (std::abs(grbX[t][h].get(GRB_DoubleAttr_X) - 1.0) <= m_intFeasTol) {
          task2tcPosition[t] = h;
          task2delay[t] = m_timeScale * matDelay[t][h];
          task2power[t] = matPower.size() ? matPower[t][h] : 0.0;
          task2energy[t] = (m_timeScale * matDelay[t][h]) * task2power[t];
          dynEnergy += (m_timeScale * matDelay[t][h]) * task2power[t];
        }
      }
      if (task2tcPosition[t] == PosUndef) {
        std::cerr << "Unable to retrieve thread parameters of task id " << t
                  << "\n";
      }
      task2startTime[t] = m_timeScale * grbS[t].get(GRB_DoubleAttr_X);
      task2endTime[t] =
          m_timeScale * grbS[t].get(GRB_DoubleAttr_X) + task2delay[t];
      for (auto u = 0u; u < nTasks; ++u) { // !!PROV
        if (can_overlap(t, u, matFullPrecedence)) {
          finalPhi[{t, u}] = grbPhi[{t, u}][0].get(
              GRB_DoubleAttr_X); // only one threading config
        }
      }
    }
    if (!is_feasible(task2startTime, task2endTime, matFullPrecedence)) {
      std::cout << "Infeasibility detected but printing anyway\n";
    }

    std::cout << "solver::detail\n";
    print_solution(task2tcPosition, task2startTime, task2endTime, task2delay,
                   task2power, task2energy);

    std::cout
        << "solver::summary\nmakespan,energy,static,dynamic,idle_power,lb\n";
    std::cout << m_timeScale * makespan << "," << dynEnergy + staticEnergy
              << "," << staticEnergy << "," << dynEnergy << "," << idlePower
              << "," << grbModel.get(GRB_DoubleAttr_ObjBound) << '\n';
    grbModel.write("gurobi_output.mst");
    grbModel.write("gurobi_output.hnt");
    grbModel.write("gurobi_output.sol");
  } catch (GRBException &e) {
    auto grbmsg = e.getMessage();
    auto grberr = e.getErrorCode();
    std::cerr << "Gurobi exception in solver: " << grbmsg << '\n'
              << grberr << '\n';
  } catch (double M) {
    free_grb(m_grbEnv, grbX, grbS, grbPhi, nTasks, matFullPrecedence);
    throw std::runtime_error(
        "Exception possibly due to floating point inacuraccies. Try bigger "
        "--bigm=<FLOAT>. Current: " +
        std::to_string(M));
  }
  // deallocation of gurobi memory
  free_grb(m_grbEnv, grbX, grbS, grbPhi, nTasks, matFullPrecedence);
}

} // namespace stm
