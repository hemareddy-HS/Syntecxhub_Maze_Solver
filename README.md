# Animated A* Maze Solver

This project implements the A* (A-Star) search algorithm to find the shortest path through a 2D grid maze. It features a real-time, animated visualization built with Matplotlib that shows the algorithm exploring nodes and tracing the final optimal path.

## How the Code Works

The script is broken down into three main operational phases:

### 1. Data Structures & Setup
* **The Maze:** Represented as a 10x10 NumPy array where `0` denotes traversable, open paths and `1` represents solid walls.
* **The Priority Queue (Open Set):** Uses Python's `heapq` library to always process the cell with the lowest overall estimated cost ($f\_score$).

### 2. The A* Search Algorithm
The search balances two distinct metrics to navigate efficiently:
* **G-Score ($g$):** The actual movement cost from the starting node to the current node.
* **Heuristic ($h$):** The estimated cost from the current node to the goal. This code utilizes the **Manhattan Distance** ($|x_1 - x_2| + |y_1 - y_2|$), which is perfect for grids restricted to 4-cardinal direction movements (Up, Down, Left, Right).
* **F-Score ($f = g + h$):** The sum of both scores. The node with the lowest $f$ value is prioritized next.

### 3. Matplotlib Animation Engine
Once the path is calculated, `matplotlib.animation.FuncAnimation` triggers a visual playback:
* **Exploration Phase (Light Blue):** Shows the historical order in which nodes were visited and evaluated by the open set.
* **Path Tracing Phase (Gold):** Backtracks from the goal to the start using a parent-tracking dictionary (`came_from`) to cleanly highlight the single shortest route.

## Requirements

Ensure you have the necessary libraries installed before running the script:

```bash
pip install numpy matplotlib
```
