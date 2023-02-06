/* generate_test.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */
 
#include <fstream>
#include <random>
#include <string>

#include "network.h"

int main(int argc, char** argv) {
  int number_of_test = 10;
  if (argc == 2) {
    number_of_test = std::stoi(argv[1]);
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> real_dist(0, static_cast<int>(network.size()));

  int count = 0;
  std::string test_string;
  while (count < number_of_test) {
    int start = real_dist(gen);
    int goal;
    do {
      goal = real_dist(gen);
    } while (goal == start);
    std::string line = network[start].name +
                       " " +
                       network[goal].name;
    count++;
    if (count != number_of_test) {
      line += "\n";
    }

    test_string += line;
  }

  std::ofstream test_file;
  test_file.open("test_data.txt");
  test_file << test_string;
  test_file.close();

  return 0;
}
