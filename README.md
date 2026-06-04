# Advanced Network Path & Graph Optimizer

## Overview

Advanced Network Path & Graph Optimizer is a C++ network routing simulator that models routers and weighted communication links using a custom adjacency-list graph representation.

The system computes optimized routing paths using graph search algorithms such as Dijkstra and A*. It also includes edge-case validation and stress tests for large network topologies.

## Features

- Object-oriented graph modeling
- Custom adjacency-list representation
- Dijkstra shortest path algorithm
- A* optimized path search
- Weighted bidirectional network links
- Route reconstruction
- Performance metrics:
  - Total path cost
  - Nodes visited
  - Execution time
- Edge-case tests:
  - Disconnected graph
  - Single-node graph
  - Invalid edge detection
  - Large stress topology

## Technologies Used

- C++
- Object-Oriented Programming
- Graph Algorithms
- Priority Queues
- STL Containers
- CMake

## Build Instructions

```bash
mkdir build
cd build
cmake ..
make
./NetworkOptimizer
