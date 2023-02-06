/* path.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "utility.h"

/**
 * @Brief  Tunable parameters for Path class
 */
namespace pathParam {
  /**
   * @Brief  Default weights to penalize
   *         large distance to goal
   */
  constexpr double DEFAULT_GOAL_WEIGHT = 1.0;
}  // namespace pathParam

/**
 * @Brief  Data structure to hold
 *         a path of charging station and charging history
 */
class Path {
 public:
   /**
    * @Brief  Constructor
    *
    * @Param start_charger The name of the initial charging station
    * @Param goal_charger The name of the goal charging station
    */
  Path(const std::string& start_charger,
       const std::string& goal_charger);

  /**
   * @Brief  Add a new charging station to the path
   *         and store information of the new charging station
   *
   * @Param next_charger
   */
  void add_charger(std::string& next_charger);

  /**
   * @Brief  Cost calculation based on the Path
   *
   *         Cost is the combination of time spending on distance
   *         and charging.
   *
   * @Returns  Time cost in hours
   */
  double time_cost();

  /**
   * @Brief  Convert the Path into the answer string format
   *
   * @Returns  The output string format for path checker
   */
  std::string to_string();

  /**
   * @Brief  Whether a charging station has been visited
   *
   * @Param next_charger The charging station to be checked
   *
   * @Returns  True if the charging station is in the Path
   */
  bool charger_visited(const std::string& next_charger);


  /**
   * @Brief  Get the latest charger in the Path
   *
   * @Returns  The latest charger in the Path
   */
  std::string current_charger();

  /**
   * @Brief Heuristic time cost based on already visited
   *        charging station and the estimated distance to goal
   *
   * @Param goal_weight A penalty value for estimated distance to goal
   *
   *                    Higher weights will result in higher cost with
   *                    large distance
   *
   * @Returns  The heuristic time cost in hours
   */
  double heuristic_cost(
      double goal_weight = pathParam::DEFAULT_GOAL_WEIGHT);

  /**
   * @Brief  Get number of visited charging station in the Path
   *
   * @Returns  Number of visited charging station in the Path
   */
  int num_of_chargers();

  /**
   * @Brief  True if the goal charging station is in the Path
   */
  bool reached_goal = false;

 private:
  /**
   * @Brief  Distribute charging amount to each charging station
   *         in Path that optimize the total charging time
   */
  void optimize_charge();

  /**
   * @Brief  The initial charging station
   */
  std::string start_charger_;

  /**
   * @Brief  The goal charging station
   */
  std::string goal_charger_;

  /**
   * @Brief  Visited charging stations
   */
  std::vector<std::string> chargers_;

  /**
   * @Brief  Set of visited charging stations for faster search
   *
   *         chargers_set shares the same index as chargers_
   */
  std::set<std::string> chargers_set_;

  /**
   * @Brief  The charging amount at each visited charging station
   *
   *         charge_distances_ shares the same index as chargers_
   *
   */
  std::vector<double> charge_distances_;  // km

  /**
   * @Brief  The charge rate of each charging station in Path
   *
   *         charge_rates_ shares the same index as chargers_
   */
  std::vector<double> charge_rates_;

  /**
   * @Brief  The distance between charging station in Path
   *
   *         dists_[i] will be the distance between
   *         chargers_[i] and chargers_[i+1]
   */
  std::vector<double> dists_;
};
