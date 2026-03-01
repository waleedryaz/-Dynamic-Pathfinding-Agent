# Dynamic Pathfinding Agent (Informed Search Algorithms)

## Project Overview

This project implements a Dynamic Pathfinding Agent in a grid-based environment.

The agent navigates from a fixed Start Node to a fixed Goal Node using:

- Greedy Best-First Search (GBFS)
- A* Search

The environment supports dynamic obstacle generation, requiring real-time re-planning.

This project was developed as part of the Artificial Intelligence course at:
National University of Computer & Emerging Sciences
Chiniot-Faisalabad Campus

---

## 🎯 Features

###  Environment Features
- Dynamic Grid Size (User-defined Rows × Columns)
- Fixed Start & Goal Nodes
- Random Map Generation (Custom obstacle density)
- Interactive Map Editor (Click to add/remove walls)
- No static .txt maps used

###  Implemented Algorithms
- Greedy Best-First Search (f(n) = h(n))
- A* Search (f(n) = g(n) + h(n))

### Heuristics
- Manhattan Distance
- Euclidean Distance (if implemented)

###  Dynamic Mode
- Obstacles spawn randomly during execution
- Real-time collision detection
- Automatic path re-planning
- Optimized recalculation (only when needed)

###  Visualization (GUI)
Developed using: Pygame

Visual Indicators:
- 🟡 Frontier Nodes (Priority Queue)
- 🔴 Visited/Expanded Nodes
- 🟢 Final Path

###  Real-Time Metrics
- Nodes Visited
- Path Cost
- Execution Time (milliseconds)

---

##  Requirements

- Python 3.8+
- Pygame

Install dependency:

pip install pygame

---

##  How to Run

1. Install Python
2. Install required library:
   pip install pygame
3. Run the project:

   python project.py

---

## File Structure

project.py  → Main implementation file

---

##  Learning Outcomes

- Implementation of informed search algorithms
- Understanding heuristic functions
- Dynamic obstacle handling
- Real-time path re-planning
- GUI-based algorithm visualization
