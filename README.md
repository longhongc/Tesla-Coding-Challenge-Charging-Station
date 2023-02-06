# Tesla Challenge
The problem is try to find the optimal path for a Tesla car to move from  
a charging station to another with fuel constraint.

Challenger: Chang-Hong Chen

## My Solution
This challenge requires us to find out how to balance long distances and charging rates.    
And the generated path has to be valid between each charging station.  
I organize it into two separate problems.    

[**1. How to optimally distribute the charging time at each station?**]  
My initial thought was to charge to full at every station to guarantee valid path.    
And reduce the overcharge value after a path is found. After implemented the first version,  
I figured out that the charging problem is actually a linear programming problem.    

By using the maximum available charging amount and minimum available charging amount,  
we can construct an optimization problem with linear constraints. The maximum available charging amount  
is due to the fact that no station can charge over the full charging capacity.   
And the minimum available charging amount comes from the fact that the car has to have enough charge to   
get to the next station. By constructing the constraints, we can observed that the constraint of one station   
is only affected by the previous charging value. We can therefore find the solution of the linear programming problem     
by setting the charging value at the boundary of maximum charging or minimum charging depending on the charging rate.  

[**2. How to find the shortest path with reasonable charging time?**]  
For this problem, I choose to implement an A-Star like algorithm.  
The difference is that the cost function is an approximate one that combines information of distances and charging time.  
In order to balance the searching time for some large distance path, I design a search reset mechanism.   
This mechanism will be activated when search queue gets too large which implies that the search took some time.  
When this happens, the search queue will be reset, and the cost function will add penalty  
to the distance cost in the heuristic cost function. The added penalty will favor the search toward further charging station that is closer to the goal,  
and reduce the search time.  

There are some parameters that can be tuned to favor more optimal total time cost or faster compuation speed.

## File Structure
The problem statement and checker program are in the challenge_files.  
The results are in the results folder.  
```
├── CMakeLists.txt
├── README.md
├── challenge_files
│   ├── README
│   ├── checker_linux
│   └── checker_osx
├── include
│   ├── network.h
│   ├── path.h
│   ├── path_solver.h
│   └── utility.h
├── results
│   ├── results_100.txt
│   └── results_20.txt
├── scripts
│   └── run_tests.sh
├── src
│   ├── generate_test.cpp
│   ├── main.cpp
│   ├── network.cpp
│   ├── path.cpp
│   ├── path_solver.cpp
│   └── utility.cpp
└── test
    └── unit_test.cpp
```

## Environment
OS: Ubuntu 20.04 in WSL2

### Dependencies
CMake (Optional)
Gtest (Optional)

## Build
Build with CMake
```
mkdir build && cd build
cmake ..
make -j
```

Build with g++ (Only contains the solution executable)
```
g++ -std=c++11 -O1 -I include src/main.cpp src/network.cpp src/path.cpp src/path_solver.cpp src/utility.cpp -o solution
```

## Run
1. Run solution only  
Input any two charging station (Not the same)
```
./solution Council_Bluffs_IA Cadillac_MI
```

2. Run solution and check the answer   
```
./checker_linux "$(./solution Council_Bluffs_IA Cadillac_MI)"
```

3. Run multiple test
Generate pairs of charging station as test data   
Input the number of test data
```
./generate_test 20
```

Run solution and check the answer
```
./run_tests.sh
```

4. Run unit test
```
./unit_test
```

## Results
The results of 20 random test and 100 random test are copied into 
files results/results_20.txt and results/results_100.txt.

## Reference
The great distance formula    
https://en.wikipedia.org/wiki/Great-circle_distance  

Linear programming  
https://en.wikipedia.org/wiki/Linear_programming  

A-Star  
https://en.wikipedia.org/wiki/A*_search_algorithm  
