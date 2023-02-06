/* path.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "utility.h"
#include "path.h"

Path::Path(const std::string& start_charger,
           const std::string& goal_charger):
  start_charger_{start_charger},
  goal_charger_{goal_charger},
  chargers_{start_charger},
  chargers_set_{start_charger},
  charge_distances_{0},
  charge_rates_{},
  dists_{} {
  auto charge_rate =
      database::get_charger_record(start_charger_).rate;
  charge_rates_.push_back(charge_rate);
}

void Path::add_charger(std::string& next_charger) {
  auto curr_charger = this->current_charger();
  // Calculate distance between current charging station
  // and next charging station
  double dist = utility::calc_great_distance(
    curr_charger, next_charger);

  dists_.push_back(dist);

  if (dist > constant::FULL_CHARGE) {
    throw std::invalid_argument("Next charger is too far");
    return;
  }

  if (this->charger_visited(next_charger)) {
    throw std::invalid_argument("Charger already visited");
    return;
  }

  // Store the new charger
  chargers_.push_back(next_charger);
  chargers_set_.insert(next_charger);

  // Placeholder for charging amount at the new charger
  charge_distances_.push_back(0);

  // Store the charge rate of the new charger
  auto charge_rate =
    database::get_charger_record(next_charger).rate;
  charge_rates_.push_back(charge_rate);

  // If next charger is the goal,
  // then calculate the optimize charging amount
  if (next_charger == goal_charger_) {
    this->optimize_charge();
    this->reached_goal = true;
  }
}

double Path::time_cost() {
  // Calculate optmize charging amount
  // for time cost estimation
  this->optimize_charge();

  double total_time = 0.0;

  for (int i=0; i < chargers_.size() - 1; ++i) {
    std::string curr_charger = chargers_[i];

    // Charging time
    if (i < charge_distances_.size() &&
        i < charge_rates_.size()) {
      total_time += charge_distances_[i] / charge_rates_[i];
    }

    // Moving time
    if (i < dists_.size()) {
      total_time += dists_[i] / constant::SPEED;
    }
  }

  return total_time;
}

std::string Path::to_string() {
  std::string solution;
  std::stringstream solution_stream;

  for (int i=0; i < chargers_.size(); ++i) {
    auto curr_charger = chargers_[i];
    solution_stream << curr_charger;

    if (curr_charger != goal_charger_) {
      solution_stream << ", ";

      if (curr_charger != start_charger_ &&
          i < charge_distances_.size()) {
        auto charge_rate =
          database::get_charger_record(curr_charger).rate;
        double charge_time = charge_distances_[i] / charge_rate;
        charge_time = std::ceil(charge_time * 1e5) / 1e5;
        solution_stream << std::fixed << std::setprecision(5) <<
          charge_time;
        solution_stream << ", ";
      }
    }
  }

  return solution_stream.str();
}

bool Path::charger_visited(const std::string& next_charger) {
  return chargers_set_.find(next_charger) != chargers_set_.end();
}

std::string Path::current_charger() {
  return chargers_.back();
}

double Path::heuristic_cost(double goal_weight) {
  if (reached_goal) {
    return this->time_cost();
  }

  double goal_dist = utility::calc_great_distance(
    chargers_.back(), goal_charger_);


  double heuristic =
    this->time_cost() +  // Past travel time and charging time
    goal_weight * goal_dist / constant::SPEED +  // Estimated travel-to-goal time
    goal_dist / constant::AVERAGE_RATE;  // Estimated future charging time

  return heuristic;
}

int Path::num_of_chargers() {
  return chargers_.size();
}

void Path::optimize_charge() {
  // Only path longer than 2 needs charging optimization
  if (chargers_.size() < 3) {
    return;
  }

  double accumulate_dists = 0.0;
  double accumulate_charge = 0.0;

  // For every charger, there exists a maximum charging amount
  // and a minimum charging amount
  // Determine which one to use based on the relative charging speed of
  // current and next charger.
  for (int i=0; i < chargers_.size()-1; ++i) {
    // The charging amount after charging cannot exceed FULL_CHARGE value
    auto max_amount = constant::FULL_CHARGE -
      (constant::INIT_CHARGE + accumulate_charge - accumulate_dists);

    // The charging amount has to be enough to get to next charger
    accumulate_dists += dists_[i];
    auto min_amount =
      std::max(0.0, accumulate_dists - constant::INIT_CHARGE - accumulate_charge);

    // If next charger has faster charging rate
    // or next charger is the last charger,
    // charge minimal charge at current station
    bool minimal_charge_condition =
      charge_rates_[i] < charge_rates_[i+1] ||
      i + 1 == chargers_.size() - 1;

    charge_distances_[i] =
      (minimal_charge_condition) ? min_amount : max_amount;

    accumulate_charge += charge_distances_[i];
  }
}


