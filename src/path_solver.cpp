/* path_solver.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <algorithm>
#include <iostream>
#include "utility.h"
#include "path.h"
#include "path_solver.h"

PathSolver::PathSolver(
  const std::string& start_charger,
  const std::string& goal_charger):
  start_charger_{start_charger},
  goal_charger_{goal_charger},
  best_path_(start_charger, goal_charger) {
  this->reset_queue();
}

std::vector<std::string> PathSolver::find_neighbors(Path& parent) {
  std::vector<std::string> child_chargers;

  auto curr_charger = parent.current_charger();
  auto neighbors = database::get_neighbors(curr_charger);

  for (auto& charger : neighbors) {
    if (parent.charger_visited(charger)) {
      continue;
    }

    child_chargers.push_back(charger);
  }

  return child_chargers;
}

void PathSolver::reset_queue() {
  path_queue_ = std::priority_queue<PathAndCost>();
  std::shared_ptr<Path> init_path_ptr =
    std::make_shared<Path>(start_charger_, goal_charger_);
  double init_cost = init_path_ptr->heuristic_cost();
  path_queue_.emplace(std::make_pair(-init_cost, init_path_ptr));
}

std::string PathSolver::solve() {
  while (path_queue_.size() > 0) {
    // If the amount of path candidates grows too large
    // (Possilby hard to find path due to large distance)
    // Reset the path candidate queue, and restart with a
    // larger penalty on the goal distance
    if (path_queue_.size() > pathSolverParam::MAX_QUEUE_SIZE) {
      goal_weight_ += pathSolverParam::GOAL_WEIGHT_STEP;
      this->reset_queue();
      reset_count_++;
    }

    auto curr = path_queue_.top();
    path_queue_.pop();

    double curr_cost = curr.first;
    auto curr_path_ptr = curr.second;
    Path& curr_path = *curr_path_ptr;

    // If only start and goal in path (Shortest path),
    // then return the path
    if (curr_path.reached_goal) {
      if (curr_path.num_of_chargers() == 2) {
        return curr_path.to_string();
      }

      if (curr_path.heuristic_cost() < best_cost_) {
        best_cost_ = curr_path.heuristic_cost();
        best_path_ = curr_path;
      }

      candidate_count_++;
      // Compare multiple candidates for better result
      if (candidate_count_ == pathSolverParam::NUM_OF_CANDIDATE ||
          reset_count_ >= pathSolverParam::MAX_RESET) {
        return best_path_.to_string();
      }

      continue;
    }

    auto child_chargers = this->find_neighbors(curr_path);

    for (auto& charger : child_chargers) {
      std::shared_ptr<Path> child_path_ptr = std::make_shared<Path>(curr_path);
      child_path_ptr->add_charger(charger);
      double child_cost = child_path_ptr->heuristic_cost(goal_weight_);
      path_queue_.emplace(std::make_pair(-child_cost, child_path_ptr));
    }
  }

  return "";
}

