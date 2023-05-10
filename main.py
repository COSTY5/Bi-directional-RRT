import numpy as np
import math
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

""" 
A Node Class for localisation in the grid
"""
class Node(object):

    def __init__(self, x, y):
        self._x = x
        self._y = y

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    def __eq__(self, other):
        return self._x == other.x and self._y == other.y

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return 'Node({}, {})'.format(self._x, self._y)

    def __repr__(self):
        return str(self)

    def __hash__(self):
        return hash((self._x, self._y))


"""
A Fucntion that Calculates Heuristic using the distance formula
"""
def heuristic_function(point_a, point_b):
    return pow(pow(point_a.x - point_b.x, 2) + pow(point_a.y - point_b.y, 2), 0.5)

def path_tracer(dict_parent, curr_var, start):
    # path list storing the trace of path from
    # start to goal
    path = [curr_var]

    while curr_var != start:
        for var in dict_parent:
            if var == curr_var:
                curr_var = dict_parent[var]
                path.append(curr_var)

    return path[::-1]

"""
A Function for performing the RRT Algorithm
"""
def rrt(start, goal, n_r, n_c, vertices, polygon1, polygon2, polygon3):
    # Initializing point lists A and B to store the navigated points from Start Node and Goal Node respectively
    point_A = [start]
    point_B = [goal]

    # Initializing a dictionary to store the values of Parent Nodes
    dict_parent = {}
    delta_d = 0.02
    path1 = []

    # Initializing a Counter to check the current list
    count = 0

    for i in range(vertices):

        # Generating a random point
        rand_pt = Node(round(np.random.uniform(0,n_r),3), round(np.random.uniform(0,n_c),3))

        # Finding the node in the open_list with minimum heuristic_function to the
        # random point
        temp_var = point_A[0]
        temp_val = heuristic_function(temp_var, rand_pt)

        for var in point_A:
            if heuristic_function(var, rand_pt) < temp_val:
                temp_val = heuristic_function(var, rand_pt)
                temp_var = var

        # Calculating the angle between the selected node with minimum
        # distance and the randomly generated point and expanding the selected node
        theta= math.atan2(rand_pt.y - temp_var.y, rand_pt.x - temp_var.x)
        delta_dx = round(temp_var.x + delta_d*math.cos(theta),3)
        delta_dy = round(temp_var.y + delta_d*math.sin(theta),3)

        new_pt = Node(delta_dx, delta_dy)

        # To check if the expanded point lies in the obstacle or not
        # if not, then add the point to the point list
        if polygon1.contains(Point(new_pt.x, new_pt.y)) == False:
            if polygon2.contains(Point(new_pt.x,new_pt.y)) == False:
                if polygon3.contains(Point(new_pt.x,new_pt.y)) == False:
                    point_A.append(new_pt)
                    plt.plot([temp_var.x, new_pt.x], [temp_var.y, new_pt.y], color = 'orange')

                    # Storing the parent node of the new node
                    dict_parent[new_pt] = temp_var

        for var in point_B:
            if heuristic_function(var, new_pt) < 0.2:
                plt.plot([var.x, new_pt.x], [var.y, new_pt.y], color='orange')

                if count%2 == 0:
                    path1 = path_tracer(dict_parent, new_pt, start)
                    path2 = path_tracer(dict_parent, var, goal)

                if count%2 == 1:
                    path1 = path_tracer(dict_parent, var, start)
                    path2 = path_tracer(dict_parent, new_pt, goal)

                path1.extend(path2[::-1])
                point_A.extend(point_B)

                return point_A, path1

        point_A, point_B = point_B[:], point_A[:]
        count += 1

    # Return the path if goal is reached, else return None and
    # the navigated pt_list
    if not path1:
        print("Path not found: Increase the vertices")
        point_A.extend(point_B)
        return point_A, None


# Initialize the input grid, start and end goal
# n_r, n_c represents the no. of rows and cols in a grid
n_r, n_c = 6, 5
start = Node(0.5, 0.5)
goal = Node(5.5, 4.5)
vertices = 100000
clearance = 0.18

plt.figure()
plt.axis([0, n_r, 0, n_c])

# Defining the polygon obstacle
points1 = [[1,1], [2,1], [2,4], [1,4]]
points2 = [[3,0], [3.5,0], [3.5,3], [3,3]]
points3 = [[4.5,2.5], [5,2.5], [5,5], [4.5,5]]
polygon1 = Polygon(points1).buffer(clearance)
polygon2 = Polygon(points2).buffer(clearance)
polygon3 = Polygon(points3).buffer(clearance)
plt.gca().add_patch(plt.Polygon(points1, fill = True, color = 'grey'))
plt.gca().add_patch(plt.Polygon(points2, fill = True, color = 'grey'))
plt.gca().add_patch(plt.Polygon(points3, fill = True, color = 'grey'))

# Finding the path and the navigated list
final_list, path = rrt(start, goal, n_r, n_c, vertices, polygon1, polygon2, polygon3)

# Plotting the overall search nodes
for var in final_list:
    plt.plot(var.x, var.y, color='yellow', marker='o', markersize = 2)

plt.plot(start.x, start.y, color='red', marker='o')
plt.plot(goal.x, goal.y, color='green', marker='o')

# Plotting the final path and the nodes from start to goal
if path is not None:
    for i in range(1,len(path)):
        plt.plot(path[i].x, path[i].y, color='yellow', marker='o', markersize = 1)
        plt.plot([path[i].x, path[i-1].x],[path[i].y, path[i-1].y], color = 'brown')
        plt.pause(0.00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001)

plt.show()
plt.pause(30)
plt.close('all')