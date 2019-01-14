#include "problem.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>

namespace stm {

template <typename T> T cast(const std::string &strVec);
template <> bool cast<bool>(const std::string &str) {
  return (bool)std::stoi(str);
}
template <> int cast<int>(const std::string &str) { return std::stoi(str); }
template <> double cast<double>(const std::string &str) {
  return std::stod(str);
}

template <typename T>
static matrix_t<T> read_matrix(const int rows, const int cols,
                               const std::string &filename) {
  matrix_t<T> retMatrix;
  std::ifstream ifs(filename.c_str());

  if (ifs.is_open()) {
    std::string line;
    while (std::getline(ifs, line)) {
      auto strVec = stm::parser::split(line, ',');
      if (strVec.size() != cols) {
        throw std::runtime_error("Unexpected number of columns read in " +
                                 filename +
                                 ". Expected : " + std::to_string(cols) +
                                 ", Read : " + std::to_string(strVec.size()));
      }
      retMatrix.emplace_back();
      std::transform(std::begin(strVec), std::end(strVec),
                     std::back_inserter(retMatrix.back()),
                     [](const std::string &s) { return cast<T>(s); });
    }
    if (retMatrix.size() != rows) {
      throw std::runtime_error("Unexpected number of rows read in " + filename +
                               ". Expected : " + std::to_string(rows) +
                               ", Read : " + std::to_string(retMatrix.size()));
    }
  } else {
    throw std::runtime_error("Could not open file " + filename);
  }
  return retMatrix;
}

void problem::set_full_precedence_matrix() {
  m_matFullPrecedence =
      matrix_t<bool>(m_taskCount, std::vector<bool>(m_taskCount, false));
  for (auto u = 0u; u < m_taskCount; ++u) {
    for (auto t = 0u; t < u; ++t) {
      if (m_matPrecedence[t][u] == true) {
        m_matFullPrecedence[t][u] = true;
      } else {
        // t does not directly preceed u, but it is possible that t fully
        // preceeds a depU that directly preceeds u
        for (auto depU = 0u; depU < m_taskCount; ++depU) {
          if (m_matPrecedence[depU][u] == true &&
              m_matFullPrecedence[t][depU] == true) {
            m_matFullPrecedence[t][u] = true;
            break;
          }
        }
      }
    }
  }
}

problem::problem(const parser::arg_dict_t &argDict) {

  m_taskCount = std::stoi(argDict.at("--n"));
  if (m_taskCount < 1) {
    throw std::runtime_error("Expected # tasks > 0");
  }
  m_threadConfigCount = std::stoi(argDict.at("--h"));
  if (m_threadConfigCount < 1) {
    throw std::runtime_error("Expected # th_config > 0");
  }
  m_capacity = std::stoi(argDict.at("--c"));
  if (m_capacity < 1) {
    throw std::runtime_error("Expected capacity > 0");
  }

  m_matDelay = read_matrix<double>(m_taskCount, m_threadConfigCount,
                                   argDict.at("--delay"));
  if (argDict.find("--power") != argDict.end()) {
    m_matPower = read_matrix<double>(m_taskCount, m_threadConfigCount,
                                     argDict.at("--power"));
    if (argDict.find("--idlepower") != argDict.end()) {
      m_idlePower = std::stod(argDict.at("--idlepower"));
    } else {
      std::cout << "Defaulting idle power to " << m_idlePower << '\n';
    }
    if (argDict.find("--maxpower") != argDict.end()) {
      m_maxPower = std::stod(argDict.at("--maxpower"));
    } else {
      std::cout << "Defaulting max power to " << m_maxPower << '\n';
    }
    m_energyModelled = true;
  } // else power matrix not set and energy not modelled

  m_matCapacity = read_matrix<int>(m_taskCount, m_threadConfigCount,
                                   argDict.at("--capacity"));
  m_matPrecedence =
      read_matrix<bool>(m_taskCount, m_taskCount, argDict.at("--preced"));
  set_full_precedence_matrix();

  if (argDict.find("--presolve") != argDict.end()) {
    m_presolve = std::stoi(argDict.at("--presolve"));
  }
  if (argDict.find("--inorder") != argDict.end()) {
    m_inOrder = std::stoi(argDict.at("--inorder"));
  }

  if (argDict.find("--input") != argDict.end()) {
    m_inputSolutionFilename = argDict.at("--input");
  }

  if (argDict.find("--bigm") != argDict.end()) {
    m_bigM = std::stod(argDict.at("--bigm"));
  }

  std::string strOpt = "time";
  if (argDict.find("--optimize") != argDict.end()) {
    strOpt = argDict.at("--optimize");
    if (strOpt == "time") {
      m_optimize = optimize::time;
    } else if (strOpt == "energy") {
      m_optimize = optimize::energy;
    } else {
      throw std::runtime_error(
          "Unknown optimization target. Available : [time/energy]");
    }
  }
  m_initialized = true;
}

} // namespace stm
