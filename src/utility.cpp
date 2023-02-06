/* utility.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "network.h"
#include "utility.h"

row database::get_charger_record(std::string& name) {
  static std::unordered_map<std::string, row> chargers_record;
  if (chargers_record.empty()) {
    for (auto& charger : network) {
      chargers_record[charger.name] = charger;
    }
  }

  if (chargers_record.find(name) != chargers_record.end()) {
    return chargers_record[name];

  } else {
    throw std::invalid_argument("Charger not in database");
  }
}

std::vector<std::string> database::get_neighbors(std::string& name) {
  static std::unordered_map<std::string,
    std::vector<std::string>> neighbors_record;

  if (neighbors_record.find(name) != neighbors_record.end()) {
    return neighbors_record[name];
  }

  std::vector<std::string> neighbors;
  for (auto charger : network) {
    double dist = utility::calc_great_distance(
      name, charger.name);

    // Find neighbors within the maximum charging value
    if (dist <= constant::FULL_CHARGE) {
      neighbors.push_back(charger.name);
    }
  }
  neighbors_record[name] = neighbors;

  return neighbors;
}

double utility::degree_to_radians(double deg) {
  return deg * M_PI / 180;
}

double utility::calc_great_distance(
      double lat1, double lat2, double lon1, double lon2, double r) {
  auto lat1_r = degree_to_radians(lat1);
  auto lat2_r = degree_to_radians(lat2);
  auto lon1_r = degree_to_radians(lon1);
  auto lon2_r = degree_to_radians(lon2);

  return r * acos(cos(lat1_r) * cos(lat2_r) * cos(lon1_r - lon2_r)
      + sin(lat1_r) * sin(lat2_r));
}

double utility::calc_great_distance(row charger1, row charger2) {
  return calc_great_distance(
      charger1.lat, charger2.lat, charger1.lon, charger2.lon);
}

double utility::calc_great_distance(
    std::string& charger1, std::string& charger2) {
  return calc_great_distance(
      database::get_charger_record(charger1),
      database::get_charger_record(charger2));
}

double utility::calc_great_distance(
    std::string&& charger1, std::string&& charger2) {
  return calc_great_distance(charger1, charger2);
}
