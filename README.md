# BW-Dijkstra-BFS-for-a-Point-Cloud-Robot
This project implements the Backward Dijkstra and Backward BFS algorithms for a point robot navigating a custom obstacle space defined by the identifier 'PK0841'. Both algorithms search from the Goal node (Xg​) back to the Initial node (Xi​) as specified in the project requirements.

## 📂 Repository Structure
```text
├── BW-dijkstra_pranavjagdish_koli.py     # Implementation of the Backward Dijkstra algorithm
├── BW-BFS_pranavjagdish_koli.py          # Implementation of the Backward BFS algorithm
├── implementaion.mp4                     # Implementation video of both the algorithms
└── README.md                             # Project documentation
```

## ⚙️ Installation & Setup
### Prerequisites
* Ubuntu 22.04
* Python 3.10+
* NumPy, OpenCV, Heapq, Collections & Time libraries

```bash
# Install dependencies
pip install numpy opencv-python
```

### Build Instructions
```bash
# Setup the directory structure and clone the repo
mkdir -p ~/Project2
cd ~/Project2
git@github.com:pranavkolii/BW-Dijkstra-BFS-for-a-Point-Cloud-Robot.git
```

### Run Instructions
* Execute the Backward Dijkstra Algorithm
```bash
# Run the solver
python3 BW-dijkstra_pranavjagdish_koli.py 
```

* Execute the Backward BFS Algorithm
```bash
# Run the solver
python3 BW-dijkstra_pranavjagdish_koli.py 
```

* When prompted, enter the start and goal coordinates as integers separated by a space
```bash
# Example
Start x y: 5 5
Goal x y: 175 45
```

## Outcomes
* Pathfinding Efficiency:
  * Backward Dijkstra: Successfully finds the mathematically optimal path by prioritizing nodes with the lowest Cost-To-Come.
  * Backward BFS: Explores the space layer-by-layer to find a path using a uniform cost for all actions.
* Visualization: The project produces a complete animation showing the node exploration (how the algorithm "searches" the map) followed by the optimal path generated through backtracking.
* Action Logic: The robot correctly implements an 8-connected motion model, including straight and diagonal moves with appropriate costs (1.0 for BFS; 1.0 or 1.4 for Dijkstra).
