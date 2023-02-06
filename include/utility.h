/* utility.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "network.h"

/**
 * @Brief  Constant setting for the challenge
 */
namespace constant {
  /**
   * @Brief  The radius of the Earth
   */
  constexpr double EARTH_RADIUS = 6356.752;  // km


  /**
   * @Brief  Max distance the tesla car can go
   *         when in full charge
   */
  constexpr double FULL_CHARGE = 320;  // km

  /**
   * @Brief  Initial charge at start charger
   */
  constexpr double INIT_CHARGE = FULL_CHARGE;  // km

  /**
   * @Brief  Average charging rate at charging station
   */
  constexpr double AVERAGE_RATE = 134;  // km/hr

  /**
   * @Brief  Constant velocity of tesla car
   */
  constexpr double SPEED = 105;  // km/hr

}  // namespace constant

namespace database {
  /**
   * @Brief  Get the charging station info by name
   *
   * @Param name The name of the charging station
   *
   * @Returns  The complete info of a charging station
   */
  row get_charger_record(std::string& name);

  /**
   * @Brief  Get neighbors within maximum distance 
   *         of a charging station
   *
   * @Param name The name of the charging station
   *
   * @Returns  All neighbors statino within maximum range
   */
  std::vector<std::string> get_neighbors(std::string& name);
}  // namespace database

namespace utility {
  /**
   * @Brief  Convert degree to radians
   *
   * @Param deg
   *
   * @Returns radian
   */
  double degree_to_radians(double deg);

  /**
   * @Brief  Calculate the great distance between two charging station
   *
   * @Param lat1 The latitude of the first charging staton
   * @Param lat2 The latitude of the first charging staton
   * @Param lon1 The longitude of the first charging station
   * @Param lon2 The longitude of the second charging station
   * @Param r The radius of the sphere (Default: the radius of Earth)
   *
   * @Returns
   */
  double calc_great_distance(
      double lat1, double lat2,
      double lon1, double lon2, double r = constant::EARTH_RADIUS);

  double calc_great_distance(row charger1, row charger2);
  double calc_great_distance(std::string& charger1, std::string& charger2);
  double calc_great_distance(std::string&& charger1, std::string&& charger2);
}  // namespace utility

// make_unique for C++11
template<class T, class... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
