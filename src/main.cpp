#include <iostream>
#include <algorithm>

#include "network.h"
#include "path_solver.h"

int main(int argc, char** argv) {
  if (argc != 3) {
      std::cout << "Error: requires initial and final supercharger names" << std::endl;
      return -1;
  }

  std::string initial_charger_name = argv[1];
  std::string goal_charger_name = argv[2];

  // double average_rate = 0.0;
  // for (auto charger : network) {
  //   average_rate += charger.rate;
  // }
  // std::cout << average_rate / network.size() << std::endl;

  PathSolver my_solver(initial_charger_name, goal_charger_name);
  auto solution = my_solver.solve();

  std::cout << solution << std::endl;
  return 0;
}
