# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import cv2
import numpy as np
import queue
import heapq
from scipy.signal import convolve2d

my_map = []
pq = queue.PriorityQueue()
initial_input = []
goal_input = []

kernel = np.array([[0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                   [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                   [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                   [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                   [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                   [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                   [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                   [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                   [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                   [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                   [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]])

kernel2 = np.ones((9, 9))
have_ans = False

# my data structure to record node
class Node:
    def __init__(self, pose, parents, index, cost):
        self.pose = pose
        self.parents = parents
        self.child = []
        self.index = index
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

    # a function to dynamically add children
    def add_child(self, child):
        self.child.append(child)
        return

class CMap:
    def __init__(self, value, color):
        self.value = value
        self.color = color

    def change(self, x, y, value, color):
        self.value[x][y] = value
        self.color[x][y] = color

    def check_obstacle_color(self):
        for i in range(len(self.value)):
            for j in range(len(self.value[0])):
                if self.value[i][j] <= -1:
                    self.color[i][j] = np.array([0, 0, 0])
        # print(self.value)


# -1 obstacle, 0 unvisited, 1 visited, 2 initial pose, 3 goal pose, 4 in queue but have not visited yet

def create_map(cost_map):
    for i in range(len(cost_map)):
        for j in range(len(cost_map[0])):

            # my boundary
            if j == 0 or j == 399 or i == 0 or i == 249:
                cost_map[i][j] = -1
            # create three Convex obstacle
            # create circle and oval obstacle

            if c(40, 300, 185, i, j) == True:  # circle
                cost_map[i][j] = -1
            # create polygon obstacle
            elif f(1, -236, 0, j) == False and f(1, -165, 0, j) == True and f(np.tan(np.pi / 6),
                                                                              -200 * np.tan(np.pi / 6) + 65, i,
                                                                              j) == False and f(np.tan(np.pi / 6),
                                                                                                -200 * np.tan(
                                                                                                        np.pi / 6) + 135,
                                                                                                i, j) == True and f(
                    np.tan(np.pi * 5 / 6), -200 * np.tan(np.pi * 5 / 6) + 65, i, j) == False and f(
                    np.tan(np.pi * 5 / 6), -200 * np.tan(np.pi * 5 / 6) + 135, i, j) == True:
                cost_map[i][j] = -1
            # create two non-convex obstacle
            elif f(25 / 79, 13715 / 79, i, j) == True and f(-85 / 69, 5275 / 24, i, j) == False and f(11, 1055, i,
                                                                                                      j) == True:
                if f(-16 / 5, 436, i, j) == False and f(6 / 7, 780 / 7, i, j) == True:
                    continue
                else:
                    cost_map[i][j] = -1

    return cost_map


def f(a, b, y, x):
    if a * x + b - y >= 0:
        return True
    else:
        return False


def c(r, xc, yc, y, x):
    if (xc - x) * (xc - x) + (yc - y) * (yc - y) <= r * r:
        return True
    else:
        return False


def Dijkstra(First, times):  # Dijkstra algorithm
    # eight direction to explore
    direction = np.array([[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [-1, -1], [1, -1], [-1, 1]])
    pq = []
    # create a priority queue
    heapq.heappush(pq, First)
    #    pq.put((First.cost, First))

    while len(pq) >= 0:
        # get the minimum cost pose
        current = heapq.heappop(pq)
        my_map.change(current.pose[0], current.pose[1], 1, (255, 0, 0))
        # 8 direction
        for i in range(len(direction)):

            # if not visited yet, put it into the queue
            if clearance_map[current.pose[0] + direction[i][0]][current.pose[1] + direction[i][1]] == 0:
                clearance_map[current.pose[0] + direction[i][0]][current.pose[1] + direction[i][1]] = 4
                my_map.change(current.pose[0] + direction[i][0], current.pose[1] + direction[i][1], 4, (255, 0, 0))
                Explore_Node = Node(np.array([current.pose[0] + direction[i][0], current.pose[1] + direction[i][1]]),
                                    current, current.index + 1, current.cost + np.linalg.norm(direction[i]))
                current.add_child(Explore_Node)
                heapq.heappush(pq, Explore_Node)
                if times == 1:
                    Show_image()
            # if it is equal to goalm then return it
            elif my_map.value[current.pose[0] + direction[i][0]][current.pose[1] + direction[i][1]] == 3:
                global have_ans
                have_ans = True
                return Node(np.array([current.pose[0] + direction[i][0], current.pose[1] + direction[i][1]]), current,
                            current.index + 1, current.cost + np.linalg.norm(direction[i]))
                # pq.put((cost, Node(np.array([current[0] + direction[i][0], current[1] + direction[i][1]]), current,
                #                   current.index + 1, cost)))


    return


def Find_path(Final_node, times):
    # backtracking, from the goal to the initial via parents
    while Final_node.pose[0] != initial_input[0] or Final_node.pose[1] != initial_input[1]:
        if times == 0:
            print(Final_node.pose)
        my_map.change(Final_node.pose[0], Final_node.pose[1], 4, (0, 255, 255))
        Final_node = Final_node.parents
    if times == 0:
        print(Final_node.pose)
    return


def Show_image():
    # Original image
    cv2.imshow("map", my_map.color)
    # Rotate image
    #    cv2.imshow("map", cv2.flip(my_map.color, 0))
    cv2.waitKey(1)


if __name__ == '__main__':

    # define the initial and goal's (x, y)
    # initialize parameters
    OMap = create_map(np.zeros((250, 400), dtype=int))
    my_map = CMap(OMap, np.ones((250, 400, 3), np.uint8) * 255)  # value, color
    my_map.check_obstacle_color()
    # clearance 5mm
    clearance_map = convolve2d(OMap, kernel, mode='same')

    I_input = []
    G_input = []
    # input the initial and goal's (x, y), if the pose is occupied or too close to the obstacle then input again
    initial_input = input("Enter initial (x,y):\n").split()
    initial_input = [int(initial_input[0]), int(initial_input[1])]
    if clearance_map[initial_input[0]][initial_input[1]] > -1:
        I_input.append(int(initial_input[0]))
        I_input.append(int(initial_input[1]))
    else:
        while clearance_map[initial_input[0]][initial_input[1]] <= -1:
            initial_input = input("ERROR, initial is in obstacle, Please Enter initial (x, y) again\n").split()
            initial_input = [int(initial_input[0]), int(initial_input[1])]
            if clearance_map[initial_input[0]][initial_input[1]] > -1:
                I_input.append(int(initial_input[0]))
                I_input.append(int(initial_input[1]))
    goal_input = input("Enter goal (x,y):\n").split()
    goal_input = [int(goal_input[0]), int(goal_input[1])]
    if clearance_map[goal_input[0]][goal_input[1]] > -1:
        G_input.append(int(goal_input[0]))
        G_input.append(int(goal_input[1]))
    else:
        while clearance_map[goal_input[0]][goal_input[1]] <= -1:
            goal_input = input("ERROR, goal is in obstacle, Please Enter initial (x, y) again\n").split()
            goal_input = [int(goal_input[0]), int(goal_input[1])]
            if clearance_map[goal_input[0]][goal_input[1]] > -1:
                G_input.append(int(goal_input[0]))
                G_input.append(int(goal_input[1]))

    #initial_input.append(int(132))
    #initial_input.append(int(134))
    #goal_input.append(int(200))
    #goal_input.append(int(350))

    # Visualize initial pose and final pose
    my_map.change(I_input[0], I_input[1], 2, (255, 0, 0))
    my_map.change(G_input[0], G_input[1], 3, (0, 0, 255))
    clearance_map[G_input[0]][G_input[1]] = 3
    #    print(initial_state, goal_state)
    # Dijkstra and backtracking
    Find_path(Dijkstra(Node(np.array([I_input[0], I_input[1]]), None, 0, 0), 0), 0)  # pose, parents, index, cost
    # Show_image()

    # if have solution then visuliza
    if have_ans:
        OMap = create_map(np.zeros((250, 400), dtype=int))
        my_map = CMap(OMap, np.ones((250, 400, 3), np.uint8) * 255)  # value, color
        my_map.check_obstacle_color()
        clearance_map = convolve2d(OMap, kernel, mode='same')
        my_map.change(I_input[0], I_input[1], 2, (255, 0, 0))
        my_map.change(G_input[0], G_input[1], 3, (0, 0, 255))
        clearance_map[G_input[0]][G_input[1]] = 3
        Find_path(Dijkstra(Node(np.array([I_input[0], I_input[1]]), None, 0, 0), 1), 1)  # pose, parents, index, cost
        Show_image()
        cv2.waitKey(0)
