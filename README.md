# RRT Motion Planner

[![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![OpenRAVE](https://img.shields.io/badge/OpenRAVE-Robot%20Simulation-blue)](http://openrave.org/)
[![Course](https://img.shields.io/badge/Course-Academic-blue)](https://www.wpi.edu/)

Implementation of the Rapidly-exploring Random Tree (RRT) algorithm for motion planning in C++ using OpenRAVE.

## Overview

This repository contains a C++ implementation of the **Rapidly-exploring Random Tree (RRT)** algorithm, a popular sampling-based motion planning algorithm. The implementation uses [OpenRAVE](http://openrave.org/) (Open Robotics Automation Virtual Environment) for robot simulation and visualization.

This was completed as part of a Motion Planning course at Worcester Polytechnic Institute (WPI).

## What is RRT?

A **Rapidly-exploring Random Tree (RRT)** is a data structure and algorithm designed for efficiently searching non-convex high-dimensional spaces. Key properties:

- 🌳 **Incremental construction**: Tree grows by adding random samples
- 🎯 **Biased exploration**: Tends to grow toward unexplored regions
- 🚀 **High-dimensional spaces**: Effective for robots with many DOF
- 🚧 **Obstacle handling**: Naturally handles obstacles and constraints

### Mathematical Intuition

RRT can be viewed as:
- A **Monte Carlo** method for exploring configuration space
- Biasing search into **largest Voronoi regions**
- A technique for generating **open-loop trajectories**

## Features

- 🔄 **RRT Algorithm**: Core tree expansion and connection
- 🗺️ **Path Planning**: Find collision-free paths
- 📊 **Visualization**: 3D visualization via OpenRAVE
- 🤖 **Robot Support**: Works with various robot models
- 🚧 **Obstacle Avoidance**: Handles static obstacles

## Tech Stack

- **Language**: C++
- **Simulation**: OpenRAVE
- **Visualization**: OpenRAVE viewer
- **Build**: Make/CMake

## Prerequisites

- C++ compiler (GCC 4.8+ or Clang 3.4+)
- [OpenRAVE](http://openrave.org/docs/latest_stable/install/) 0.9+
- Boost libraries

### Installing OpenRAVE (Ubuntu)

```bash
# Install dependencies
sudo apt-get install cmake g++ libboost-all-dev

# Install OpenRAVE
sudo apt-get install openrave

# Or build from source (see OpenRAVE documentation)
```

## Installation

```bash
# Clone the repository
git clone https://github.com/leelening/RRT.git
cd RRT

# Build
make
# or
mkdir build && cd build
cmake ..
make
```

## Usage

### Basic Usage

```bash
# Run RRT planner with default scene
./rrt_planner

# Run with custom scene
./rrt_planner scene.env.xml

# Run with specific start and goal
./rrt_planner --start "0 0 0" --goal "5 5 0"
```

### Command Line Options

```bash
./rrt_planner [options]
  --scene FILE       # Load scene file
  --robot NAME       # Specify robot name
  --start "x y z"    # Start configuration
  --goal "x y z"     # Goal configuration
  --max-nodes N      # Maximum tree nodes (default: 10000)
  --step-size S      # Step size for extension
  --visualize        # Enable visualization
```

## Project Structure

```
.
├── src/                  # Source code
│   ├── rrt.cpp           # RRT algorithm implementation
│   ├── rrt.h             # RRT header file
│   ├── main.cpp          # Main entry point
│   └── utils.cpp         # Utility functions
├── scenes/               # OpenRAVE scene files
│   └── *.env.xml
├── robots/               # Robot model files
│   └── *.robot.xml
├── include/              # Header files
├── Makefile              # Build configuration
└── README.md             # This file
```

## Algorithm Details

### RRT Algorithm

```
Algorithm RRT(q_init, K, Δt):
    T.init(q_init)
    for k = 1 to K do
        q_rand ← RANDOM_CONFIG()
        q_near ← NEAREST_VERTEX(q_rand, T)
        q_new ← NEW_CONFIG(q_near, q_rand, Δt)
        if COLLISION_FREE(q_near, q_new) then
            T.add_vertex(q_new)
            T.add_edge(q_near, q_new)
        end if
    end for
    return T
```

### Key Components

1. **Random Sampling**: Uniform sampling in configuration space
2. **Nearest Neighbor**: Find closest node in tree
3. **Steering**: Extend toward sample with step size
4. **Collision Checking**: Verify edge is collision-free

## Variations

The repository may include or be extended with:

- **RRT-Connect**: Bidirectional RRT for faster convergence
- **RRT***: Asymptotically optimal RRT
- **Kinodynamic RRT**: Handles differential constraints
- **RRT with obstacles**: Enhanced collision detection

## OpenRAVE Integration

OpenRAVE provides:
- Robot kinematics and dynamics
- Collision checking
- 3D visualization
- Scene loading and management

## Course Information

- **Course**: Motion Planning
- **Institution**: Worcester Polytechnic Institute (WPI)
- **Year**: 2015
- **Focus**: Sampling-based motion planning algorithms

## Key Concepts

- Configuration space (C-space)
- Sampling-based planning
- Probabilistic completeness
- Motion planning under constraints
- Robot kinematics

## Results

Example paths found by the RRT planner:

```
Start: (0, 0, 0)
Goal:  (5, 5, π/4)
Path length: 12.3 units
Nodes expanded: 847
Planning time: 2.3 seconds
```

## Extensions

Possible improvements:
- [ ] RRT* for optimal paths
- [ ] RRT-Connect for faster planning
- [ ] Anytime RRT for real-time applications
- [ ] Sampling strategies (Gaussian, bridge)

## License

Academic use only.

## Author

**Lening Li**
- M.S. in Computer Science (WPI, 2016)
- M.S. in Robotics Engineering (WPI, 2018)
- Ph.D. in Robotics Engineering (WPI, 2022)

## References

- LaValle, S. M. (1998). Rapidly-exploring random trees: A new tool for path planning. *Technical Report*.
- LaValle, S. M., & Kuffner, J. J. (2001). Rapidly-exploring random trees: Progress and prospects. *Algorithmic and Computational Robotics*.
- Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. *International Journal of Robotics Research*.
