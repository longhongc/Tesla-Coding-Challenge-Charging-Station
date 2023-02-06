/* unit_test.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <sstream>
#include <unordered_map>

#include "utility.h"
#include "path.h"
#include "path_solver.h"

#include <gtest/gtest.h>

double epsilon = 1e-3;

TEST(DummyTests, dummy1) {
  EXPECT_TRUE(true);
}

TEST(Utility, degree_to_radians) {
  double rad1 = utility::degree_to_radians(0);
  EXPECT_DOUBLE_EQ(rad1, 0);

  double rad2 = utility::degree_to_radians(90);
  EXPECT_DOUBLE_EQ(rad2, M_PI / 2);

  double rad3 = utility::degree_to_radians(180);
  EXPECT_DOUBLE_EQ(rad3, M_PI);
}

TEST(Database, get_charger_record) {
  std::string name1 = "Albany_NY";
  auto charger1 = database::get_charger_record(name1);
  EXPECT_EQ(name1, charger1.name);

  try {
    std::string name2 = "Wrong_name";
    auto charger2 = database::get_charger_record(name2);
  } catch(std::invalid_argument e) {
    return;
  }
  FAIL();
}

TEST(Utility, calc_great_distance) {
  double dist1 = utility::calc_great_distance(
      "Albany_NY", "Edison_NJ");
  EXPECT_NEAR(dist1, 244.047, epsilon);

  double dist2 = utility::calc_great_distance(
      "Mauston_WI", "Sheboygan_WI");
  EXPECT_NEAR(dist2, 185.316, epsilon);

  double dist3 = utility::calc_great_distance(
      std::string("Council_Bluffs_IA"),
      std::string("Worthington_MN"));
  EXPECT_NEAR(dist3, 268.425, epsilon);
}

class TestPath : public ::testing::Test {
 public:
  TestPath() {}

  void SetUp(std::string& start,
             std::string& goal) {
    path_ptr_ = make_unique<Path>(start, goal);
  }

 protected:
  std::unique_ptr<Path> path_ptr_;
};

TEST_F(TestPath, path_initialize) {
  std::string start = "Albany_NY";
  std::string goal = "Edison_NJ";
  SetUp(start, goal);

  auto cost = path_ptr_->time_cost();
  EXPECT_DOUBLE_EQ(cost, 0.0);

  auto path_str = path_ptr_->to_string();
  EXPECT_EQ("Albany_NY, ", path_str);
}

TEST_F(TestPath, direct_to_goal) {
  std::string start = "Albany_NY";
  std::string goal = "Edison_NJ";
  SetUp(start, goal);

  path_ptr_->add_charger(goal);
  EXPECT_TRUE(path_ptr_->reached_goal);

  auto cost = path_ptr_->time_cost();
  EXPECT_NEAR(cost, 2.324, epsilon);

  auto path_str = path_ptr_->to_string();
  EXPECT_EQ("Albany_NY, Edison_NJ", path_str);
}



/**
 * @Brief One charger between start and goal
 *
 */
TEST_F(TestPath, multiple_chargers_1) {
  std::string start = "Council_Bluffs_IA";
  std::string goal = "Albert_Lea_MN";
  SetUp(start, goal);

  std::string charger = "Worthington_MN";
  path_ptr_->add_charger(charger);
  EXPECT_FALSE(path_ptr_->reached_goal);

  EXPECT_TRUE(path_ptr_->charger_visited(charger));
  EXPECT_FALSE(path_ptr_->charger_visited(goal));

  EXPECT_EQ(charger, path_ptr_->current_charger());

  auto cost =
    268.425 / constant::SPEED +
    pathParam::DEFAULT_GOAL_WEIGHT * 179.713 / constant::SPEED +
    179.713 / constant::AVERAGE_RATE;

  EXPECT_NEAR(cost, path_ptr_->heuristic_cost(), epsilon);

  auto time_cost = 268.425 / constant::SPEED;
  EXPECT_NEAR(time_cost, path_ptr_->time_cost(), epsilon);

  auto path_str = path_ptr_->to_string();
  EXPECT_EQ("Council_Bluffs_IA, Worthington_MN, 0.00000, ", path_str);

  path_ptr_->add_charger(goal);
  EXPECT_TRUE(path_ptr_->reached_goal);

  time_cost =
    128.138 / 108 +
    268.425 / constant::SPEED +
    179.713 / constant::SPEED;

  EXPECT_NEAR(time_cost, path_ptr_->time_cost(), epsilon);

  path_str = path_ptr_->to_string();
  EXPECT_EQ(
      "Council_Bluffs_IA, Worthington_MN, 1.18647, Albert_Lea_MN", path_str);
}

/**
 * @Brief Two charger between start and goal
 *
 */
TEST_F(TestPath, multiple_chargers_2) {
  std::string start = "Council_Bluffs_IA";
  std::string goal = "Onalaska_WI";
  SetUp(start, goal);

  std::string charger1 = "Worthington_MN";
  path_ptr_->add_charger(charger1);

  std::string charger2 = "Albert_Lea_MN";
  path_ptr_->add_charger(charger2);

  EXPECT_FALSE(path_ptr_->reached_goal);

  auto cost =
    128.138 / 108 +
    (268.425 + 179.713) / constant::SPEED +
    pathParam::DEFAULT_GOAL_WEIGHT * 175.07 / constant::SPEED +
    175.07 / constant::AVERAGE_RATE;
  EXPECT_NEAR(cost, path_ptr_->heuristic_cost(), epsilon);

  auto time_cost =
    128.138 / 108 +
    (268.425 + 179.713) / constant::SPEED;
  EXPECT_NEAR(time_cost, path_ptr_->time_cost(), epsilon);

  auto path_str = path_ptr_->to_string();
  std::remove(path_str.begin(), path_str.end(), ' ');
  //
  std::stringstream ss(path_str);
  std::string element;
  std::getline(ss, element, ',');
  EXPECT_EQ(element, "Council_Bluffs_IA");

  std::getline(ss, element, ',');
  EXPECT_EQ(element, "Worthington_MN");

  std::getline(ss, element, ',');
  EXPECT_NEAR(std::stod(element), 128.138 / 108, epsilon);

  std::getline(ss, element, ',');
  EXPECT_EQ(element, "Albert_Lea_MN");

  path_ptr_->add_charger(goal);
  EXPECT_TRUE(path_ptr_->reached_goal);

  time_cost =
    268.425 / 108 +
    34.783 / 92 +
    (268.425 + 179.713 + 175.07) / constant::SPEED;
  EXPECT_NEAR(time_cost, path_ptr_->time_cost(), epsilon);
}

/**
 * @Brief Five chargers between start and goal
 *
 */
TEST_F(TestPath, multiple_chargers_5) {
  std::string start = "Council_Bluffs_IA";
  std::string goal = "Cadillac_MI";
  SetUp(start, goal);

  std::string charger1 = "Worthington_MN";
  path_ptr_->add_charger(charger1);

  std::string charger2 = "Albert_Lea_MN";
  path_ptr_->add_charger(charger2);

  std::string charger3 = "Onalaska_WI";
  path_ptr_->add_charger(charger3);

  std::string charger4 = "Mauston_WI";
  path_ptr_->add_charger(charger4);

  std::string charger5 = "Sheboygan_WI";
  path_ptr_->add_charger(charger5);

  EXPECT_FALSE(path_ptr_->reached_goal);

  double time_cost =
    268.425 / 108 + 34.783 / 92 + 90.828 / 130 + 185.317 / 138 +
    (268.425 + 179.713 + 175.07 + 90.828 + 185.317) / constant::SPEED;
  EXPECT_NEAR(time_cost, path_ptr_->time_cost(), epsilon);

  auto path_str = path_ptr_->to_string();
  std::remove(path_str.begin(), path_str.end(), ' ');

  std::stringstream ss(path_str);
  std::string element;
  std::getline(ss, element, ',');
  EXPECT_EQ(element, "Council_Bluffs_IA");

  std::getline(ss, element, ',');
  EXPECT_EQ(element, "Worthington_MN");

  std::getline(ss, element, ',');
  EXPECT_NEAR(std::stod(element), 268.425 / 108, epsilon);

  std::getline(ss, element, ',');
  EXPECT_EQ(element, "Albert_Lea_MN");

  std::getline(ss, element, ',');
  EXPECT_NEAR(std::stod(element), 34.783 / 92, epsilon);

  std::getline(ss, element, ',');
  EXPECT_EQ(element, "Onalaska_WI");

  std::getline(ss, element, ',');
  EXPECT_NEAR(std::stod(element), 90.828 / 130, epsilon);

  std::getline(ss, element, ',');
  EXPECT_EQ(element, "Mauston_WI");

  std::getline(ss, element, ',');
  EXPECT_NEAR(std::stod(element), 185.317 / 138, epsilon);

  std::getline(ss, element, ',');
  EXPECT_EQ(element, "Sheboygan_WI");

  path_ptr_->add_charger(goal);
  EXPECT_TRUE(path_ptr_->reached_goal);

  time_cost =
    268.425 / 108 + 34.783 / 92 + 90.828 / 130 +
    320.0 / 138 + 61.4396 / 116 +
    (268.425 + 179.713 + 175.07 + 90.828 +
     185.317 + 196.123) / constant::SPEED;

  EXPECT_NEAR(time_cost, path_ptr_->time_cost(), epsilon);
}


/**
 * @Brief heuristic calculation when start direct to goal
 *
 */
TEST_F(TestPath, heuristic1) {
  std::string start = "Council_Bluffs_IA";
  std::string goal = "Worthington_MN";
  SetUp(start, goal);

  auto cost =
      pathParam::DEFAULT_GOAL_WEIGHT * 268.425 / constant::SPEED +
      268.425 / constant::AVERAGE_RATE;

  EXPECT_NEAR(path_ptr_->heuristic_cost(), cost, epsilon);
}


/**
 * @Brief heuristic calculation for 1 charger
 *        between start and goal
 *
 */
TEST_F(TestPath, heuristic2) {
  std::string start = "Onalaska_WI";
  std::string goal = "Sheboygan_WI";
  SetUp(start, goal);

  auto cost =
      pathParam::DEFAULT_GOAL_WEIGHT * 275.8665 / constant::SPEED +
      275.8665 / constant::AVERAGE_RATE;

  EXPECT_NEAR(cost, path_ptr_->heuristic_cost(), epsilon);

  std::string charger = "Mauston_WI";
  path_ptr_->add_charger(charger);

  cost =
    90.828 / constant::SPEED +
    pathParam::DEFAULT_GOAL_WEIGHT * 185.3166 / constant::SPEED +
    185.3166 / constant::AVERAGE_RATE;

  EXPECT_NEAR(cost, path_ptr_->heuristic_cost(), epsilon);
}

