# Bi-directional-RRT

Dependencies

The code requires the following dependencies:

Python 3.x
NumPy
Matplotlib
Shapely

Installation

You can install the dependencies by running the following command:
  pip install numpy matplotlib shapely

To run the code, execute the following command in the terminal:
  python rrt.py

The code initializes the grid, start and goal positions, and obstacle polygons. 
The algorithm then samples random points in the search space and connects them to the nearest point in the tree. 
The algorithm terminates when a node in one tree is within a certain distance of a node in the other tree, and the algorithm constructs the path by connecting the two trees.
The code uses the Shapely library to create polygon objects and check if the nodes lie within the obstacle polygons. 
The code also uses the Matplotlib library to visualize the search space, the nodes in the tree, and the final path.
The code presents some modifications to the basic Bi-Directional RRT algorithm. 
It expands the tree in the direction of the sampled point by a fixed distance. 
The code also uses a path tracer function to trace the path from the start node to the goal node. 
Finally, the code alternates between the two trees to construct the final path.
