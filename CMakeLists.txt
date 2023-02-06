cmake_minimum_required(VERSION 3.10)
project(tesla_challenge)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-O1")

find_package(GTest REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include")

add_library(myLibs
	src/network.cpp
	src/utility.cpp
  src/path.cpp
  src/path_solver.cpp
)

add_executable(solution src/main.cpp)
target_link_libraries(solution 
  myLibs
)

add_executable(generate_test src/generate_test.cpp)
target_link_libraries(generate_test 
  myLibs
)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

enable_testing()

add_executable(unit_test test/unit_test.cpp)
target_link_libraries(unit_test 
  myLibs
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_test)

configure_file(challenge_files/checker_linux checker_linux COPYONLY)
configure_file(scripts/run_tests.sh run_tests.sh COPYONLY)
