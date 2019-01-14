#include "solver.hpp"

#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {

  try {

    stm::solver s(argc, argv);

    s.run();

  } catch (const std::runtime_error &rte) {
    std::cerr << "Runtime error : " << rte.what() << '\n';
    return 1;
  } catch (const std::logic_error &le) {
    std::cerr << "Logic error : " << le.what() << '\n';
    return 2;
  } catch (...) {
    std::cerr << "Unknown error\n";
    return 3;
  }
  return 0;
}
