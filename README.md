# Graph Search Algorithms

This repository contains an implementation of various graph search algorithms in C++. The implemented algorithms include:

- **Breadth-First Search (BFS)**
- **Depth-First Search (DFS)**
- **Dijkstra's Algorithm**
- **A* (A-Star) Algorithm**

## Features
- Create a graph from a list of edges
- Find paths between two nodes using different search algorithms
- Compare execution times of different algorithms
- Uses priority queues for efficient shortest-path search in Dijkstra and A*

## Usage

### Compiling the Code
This project uses CMake for building. To compile, follow these steps:
```bash
mkdir build
cd build
cmake ..
make
```

### Running the Program
After compiling, execute the program:
```bash
 ./graph_search
```

## Code Structure
- `Graph_base` - Abstract class defining graph operations.
- `Graph` - Implementation of the graph and search algorithms.
- `Path` - Data structure for storing path results.
- `Node` - Represents a graph node with neighbors.
- `CMakeLists.txt` - CMake configuration file for building the project.

## Algorithms Overview

### BFS (Breadth-First Search)
BFS explores all neighbor nodes before moving to the next depth level. Used to find the shortest path in an unweighted graph.

### DFS (Depth-First Search)
DFS explores as deep as possible along each branch before backtracking. Useful for exploring connected components.

### Dijkstra's Algorithm
Finds the shortest path from a source node to a destination node using a priority queue. Works efficiently on graphs with weighted edges.

### A* (A-Star) Algorithm
An informed search algorithm that uses a heuristic to estimate the shortest path, combining aspects of Dijkstra’s and greedy search.

## Example
```cpp
Graph g;
g.create_graph({{1, 2}, {2, 3}, {3, 4}, {4, 5}, {2, 5}});
Path p = g.does_path_exist(1, 5, 3); // Dijkstra search
```

## License
This project is licensed under the MIT License.

## Contributions
Contributions are welcome! Feel free to submit pull requests or open issues.

## Author
Developed by Dhrumil Kotadia

