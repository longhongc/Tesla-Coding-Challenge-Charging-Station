/* path_solver.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <set>
#include <utility>
#include <vector>

#include "path.h"

using PathAndCost = std::pair<double, std::shared_ptr<Path>>;

/**
 * @Brief  Tunable parameter for Path Solver class
 */
namespace pathSolverParam {
  /**
   * @Brief  Number of generated valid candidate path
   *
   *         Increase this value also increase the 
   *         compuation time, and obtain more optimal
   *         solution
   */
  constexpr int NUM_OF_CANDIDATE = 20;

  /**
   * @Brief  Penalty to the goal weight when
   *         path search restart
   *
   *         Increase this value decrease the search 
   *         time for long distances, but decrease the 
   *         optimality of the solution
   */
  constexpr double GOAL_WEIGHT_STEP = 0.2;

  /**
   * @Brief  Maximum ongoing path search
   *
   *         Path search will restart
   *         if path queue size larger than maximum
   */
  constexpr int MAX_QUEUE_SIZE = 10000;

  /**
   * @Brief  Maximum time reset search and restart with
   *         a different weight
   */
  constexpr int MAX_RESET = 20;
}  // namespace pathSolverParam

/**
 * @Brief  A class for solving the path charging problem
 */
class PathSolver {
 public:
   /**
    * @Brief  Constructor
    *
    * @Param start_charger The name of the initial charging station
    * @Param goal_charger The name of the goal charging station
    */
  PathSolver(const std::string& start_charger,
             const std::string& goal_charger);

  /**
   * @Brief  Expanding parent Path by finding
   *         valid neighbor charging stations of
   *         current charging station of this Path
   *
   * @Param parent The parent path for expanding
   *
   * @Returns  A vector of neighbor chargin stations within distance
   */
  std::vector<std::string> find_neighbors(Path& parent);

  /**
   * @Brief  Search for valid paths and choose the best one to return
   *
   * @Returns  The best path searched so far
   */
  std::string solve();

 private:
  /**
   * @Brief  Reset the path candidate queue to only contain
   *         the start charging station
   */
  void reset_queue();

  /**
   * @Brief  A priorit queue that contains possible unfinished path candidate
   *
   *         The priority is based on the heuristic cost of the path
   */
  std::priority_queue<PathAndCost> path_queue_;

  /**
   * @Brief  The initial charging station
   */
  std::string start_charger_;

  /**
   * @Brief  The goal charging station
   */
  std::string goal_charger_;

  /**
   * @Brief The finished path that has the least cost so far
   */
  Path best_path_;

  /**
   * @Brief The cost of the best path so far
   */
  double best_cost_ = std::numeric_limits<double>::infinity();

  /**
   * @Brief The amount of candidate path that is finished
   */
  int candidate_count_ = 0;

  /**
   * @Brief The penalty weight to goal distance
   *
   *        This value will increase for every time
   *        search resets due to exceeding maximum path candidates in queue
   */
  double goal_weight_ = pathParam::DEFAULT_GOAL_WEIGHT;

  /**
   * @Brief  The number of reset
   */
  int reset_count_ = 0;
};
