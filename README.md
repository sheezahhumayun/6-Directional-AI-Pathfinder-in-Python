# 🧭 AI Pathfinder: Dynamic 6-Direction Search Visualizer

A Python-based simulation environment for exploring and visualizing classic AI search algorithms. Unlike standard grid-search apps, this pathfinder operates under a **restricted 6-directional movement model** and faces **dynamic environmental changes** in real-time.

---

## ✨ Key Features

* **Diverse Algorithm Suite:** Compare BFS, DFS, UCS, DLS, IDDFS, and Bidirectional Search in a single environment.
* **6-Directional Logic:** Movement is restricted to a specific clockwise priority to simulate unique traversal constraints:
    1.  **Up**
    2.  **Right**
    3.  **Bottom**
    4.  **Bottom-Right** (Main Diagonal)
    5.  **Left**
    6.  **Top-Left** (Main Diagonal)
    > **Note:** Top-Right and Bottom-Left movements are strictly excluded to test pathing efficiency under constraints.
* **Dynamic Obstacles:** A "Chaos Engine" spawns hurdles while the agent is in motion, triggering immediate re-planning if the current path is compromised.
* **Real-time Visualization:** Powered by `Matplotlib` to show the frontier expansion, explored nodes, and the final path.

---

## 🛠️ Algorithms Included

| Algorithm | Search Type | Optimality | Description |
| :--- | :--- | :--- | :--- |
| **BFS** | Uninformed | Optimal | Best for unweighted steps; finds the fewest steps. |
| **DFS** | Uninformed | Not Optimal | Memory efficient; explores deep branches first. |
| **UCS** | Cost-based | Optimal | Accounts for weighted steps ($\sqrt{2}$ for diagonals). |
| **IDDFS** | Uninformed | Optimal | Combines BFS optimality with DFS memory efficiency. |
| **Bidirectional** | Uninformed | Efficient | Simultaneous search from Start and Target. |

---

## 🚀 Getting Started

### Prerequisites
* Python 3.x
* `numpy`
* `matplotlib`

### Installation
1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/sheezahhumayun/6-Directional-AI-Pathfinder-in-Python.git](https://github.com/sheezahhumayun/6-Directional-AI-Pathfinder-in-Python.git)
    cd ai-pathfinder
    ```

2.  **Install dependencies:**
    ```bash
    pip install numpy matplotlib
    ```

3.  **Run the visualizer:**
    ```bash
    python main.py
    ```

---

## 🧠 The "Main Diagonal" Constraint
This project implements a specific movement cost model. While cardinal directions (North, East, South, West) have a cost of **1**, the diagonal movements (South-East and North-West) are calculated at $\sqrt{2} \approx 1.41$ within the **Uniform Cost Search** logic. 

This creates realistic pathfinding behavior where the AI avoids unnecessary zig-zags and prioritizes straight lines unless a diagonal shortcut is mathematically superior.

