import cv2
import numpy as np
import math
from Queue import PriorityQueue
import time

class explored_nodes:
    def __init__(self, x, y, th, parent, cost):
        self.x = x
        self.y = y
        self.th = th
        self.parent = parent
        self.cost = cost

def getLineParam_rigid(x1, y1, x2, y2, d):
    if x2 - x1 == 0:
        if d == 0:
            return 1, 0, -x1 - thresh
        else:
            return 1, 0, -x1 + thresh
    else:
        a = float(y2 - y1) / float(x2 - x1)
    b = -1
    th = thresh * math.sqrt(1 + a**2)
    if d == 0:
        c = y1 - a * x1 + th
    else:
        c = y1 - a * x1 - th

    return -a, -b, -c

class obs_polygon():
    def __init__(self, points):
        lines = []
        for i in range(len(points)):
            l = []
            a, b, c = getLineParam_rigid(points[i][0], points[i][1], points[i][2], points[i][3], points[i][4])
            l.append(a)
            l.append(b)
            l.append(c)
            l.append(points[i][4])
            lines.append(l)

        self.lines = np.asarray(lines)

    def pretty_print(self):
        for l in self.lines:
            print(l)

    '''
    Input lines contains:

    lines[3] - defines which part of the half plane should be included in the obstacle
                0: Include negative half plane
                1: Include positive half plane
    '''
    def check_collision(self, x, y):
        res = True
        for l in self.lines:
            # print(l)
            val = l[0] * x + l[1] * y + l[2]
            # print(val)
            if l[3] == 1:
                if l[0] * x + l[1] * y + l[2] < 0:
                    res = False
                    break

            elif l[3] == 0:
                if l[0] * x + l[1] * y + l[2] > 0:
                    res = False
                    break

        return res

class obs_ellipse():
    def __init__(self, x, y, a, b, radius):
        self.x = x
        self.y = y
        self.a = a
        self.b = b
        self.radius = radius

    def pretty_print(self):
        print(self.x, self.y)
        print(self.a, self.b)
        print(self.radius)


    def check_collision(self, x, y):
        if float(x - self.x) ** 2 / self.a ** 2 + float(y - self.y) ** 2 / self.b ** 2 <= self.radius ** 2:
            return True
        else:
            return False

def world_map():
    w_map = []

    # obstacle1
    points1 = [[200, 25, 225, 40, 0],
              [250, 25, 225, 40, 0],
              [250, 25, 225, 10, 1],
              [200, 25, 225, 10, 1]]

    obs1 = obs_polygon(points1) 
    w_map.append(obs1)

    # obstacle2
    points21 = [[25, 185, 50, 185, 0],
              [50, 185, 50, 150, 0],
              [50, 150, 20, 120, 1],
              [20, 120, 25, 185, 0]]

    obs21 = obs_polygon(points21)
    w_map.append(obs21) 

    
    points22 = [[50, 185, 75, 185, 0],
              [75, 185, 100, 150, 0],
              [100, 150, 75, 120, 1],
              [75, 120, 50, 150, 1],
              [50, 150, 50, 185, 1]]

    obs22 = obs_polygon(points22)
    w_map.append(obs22)

    # obstacle3
    points3 = [[36, 77, 100, 39, 0],
              [100, 39, 95, 30, 1],
              [95, 30, 31, 68, 1],
              [31, 68, 36, 76, 0]]

    obs3 = obs_polygon(points3)
    w_map.append(obs3)

    # obstacle4
    obs4 = obs_ellipse(150, 100, 40+thresh, 20+thresh, 1)
    w_map.append(obs4)
    # obstacle5
    obs5 = obs_ellipse(225, 150, 1, 1, 25+thresh)
    w_map.append(obs5)

    return w_map

# function to get start points
def startPoint():
    # sx = int(input('Enter x coordinate for start point: '))
    # sy = int(input('Enter y coordinate for start point: '))
    sx = 5
    sy = 5
    return sx, sy


# function to get goal points
def goalPoint():
    # gx = int(input('Enter x coordinate for goal point: '))
    # gy = int(input('Enter y coordinate for goal point: '))
    gx = 295
    gy = 195
    return gx, gy

def get_estimated_cost(node, goal_point):
    h = math.sqrt((node.x - goal_point[0]) ** 2 + (node.y - goal_point[1]) ** 2)
    return node.cost + h

def goal_check(x, y):
    dist = math.sqrt((x - goal_point[0]) ** 2 + (y - goal_point[1]) ** 2)

    if dist < 0.5:
        return True
    else:
        return False

def round_off(a):
    return (round(a*2))

def get_index(x, y, th):
    i = round_off(x)
    j = round_off(y)
    k = th/30

    return int(i), int(j), int(k)

def check_collision(node):
    res = False

    for obs in world:
        # print(res)
        if(obs.check_collision(node.x, node.y)):
            res = True
            break;

    # print(res)

    return res

def get_children(node, visited):
    children = []

    for i in range(360/theta):
        th = i*(theta)
        x_ = node.x + d * math.cos(th)
        y_ = node.y + d * math.sin(th)

        # print(x_, y_, th)

        j, k, l = get_index(x_, y_, th)
        if (j >= 600 or j < 0 or k >= 400 or k < 0):
            continue
        if(visited[j][k][l] == float('inf')):
            # print("here")
            new_node = explored_nodes(x_, y_, th, node, d)
            if(not check_collision(new_node)):
                children.append(new_node)
        else:
            # print("inside else")
            children.append(explored_nodes(x_, y_, th, node, node.cost+d))

    return children

# main function
if __name__ == '__main__':

    # Get points
    path = []
    start_point = startPoint()
    goal_point = goalPoint()

    # get robot radius
    # r = int(input('Enter robot radius: '))
    r = 2

    # get clearance
    # c = int(input('Enter clearance: '))
    c = 2

    # get step size
    # d = int(input('Enter robot step size: '))
    d = 1

    # get theta
    # theta = int(input('Enter mininum angle of turn: '))
    theta = 30

    # threshold
    thresh = r+c

    # generate map
    world = world_map()
    # for obs in world:
    #     obs.pretty_print()

    # thresholds
    thresh_d = 0.5
    thresh_theta = 30

    visited = np.zeros((int(300/thresh_d), int(200/thresh_d), 360/thresh_theta))



