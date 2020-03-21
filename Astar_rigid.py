import cv2
import numpy as np
import math
import map2
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
              [225, 40, 250, 25, 0],
              [250, 25, 225, 10, 1],
              [225, 10, 200, 25, 1]]

    obs1 = obs_polygon(points1) 
    w_map.append(obs1)

    # obstacle2
    points21 = [[25, 185, 50, 185, 0],
              [50, 150, 50, 185, 1],
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
    sx = int(input('Enter x coordinate for start point: '))
    sy = int(input('Enter y coordinate for start point: '))
    s_th = int(input('Enter theta for start point: '))
    if (sx < 0 or sx >= 300 or sy < 0 or sy >= 200):
        print("Invalid input. Start point lies outside the map")
        return None

    if (check_collision(explored_nodes(sx, sy, s_th, -1, 0))):
        print("Invalid input. Start point lies inside the obstacle")
        return None
    return sx, sy, s_th


# function to get goal points
def goalPoint():
    gx = int(input('Enter x coordinate for goal point: '))
    gy = int(input('Enter y coordinate for goal point: '))
    if (gx < 0 or gx >= 300 or gy < 0 or gy >= 200):
        print("Invalid input. Goal point lies outside the map")
        return None
    if (check_collision(explored_nodes(gx, gy, 0, -1, 0))):
        print("Invalid input. Goal point lies inside the obstacle")
        return None
    return gx, gy

def get_estimated_cost(node, goal_point):
    h = math.sqrt((node.x - goal_point[0]) ** 2 + (node.y - goal_point[1]) ** 2)
    return node.cost + h

def goal_check(x, y):
    dist = math.sqrt((x - goal_point[0]) ** 2 + (y - goal_point[1]) ** 2)

    if dist < 1.5:
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
        if(obs.check_collision(node.x, node.y)):
            res = True
            break;

    return res

def get_children(node, visited):
    children = []


    for i in range(-2,3,1):
        th = abs((node.th + i*(theta))%360)
        x_ = node.x + d * math.cos(math.radians(th))
        y_ = node.y + d * math.sin(math.radians(th))

        j, k, l = get_index(x_, y_, th)
        if (j >= 600 or j < 0 or k >= 400 or k < 0):
            continue
        if(visited[j][k][l] == 0):
            new_node = explored_nodes(x_, y_, th, node, node.cost + d)
            if(not check_collision(new_node)):
                children.append(new_node)
    return children

# function to explore neighbors and lot more
def explorer(start_point, goal_point):
    size_x = int(300/thresh_d)
    size_y = int(200/thresh_d)
    size_z = 360/thresh_theta
    visited = np.asarray([[[0] * size_z] * size_y]* size_x)
    cost = np.asarray([[[float('inf')] * size_z] * size_y]* size_x)
    explored  = []

    start_node = explored_nodes(start_point[0], start_point[1], start_point[2], -1, 0)
    estimated_cost = get_estimated_cost(start_node, goal_point)

    pq = PriorityQueue()
    pq.put((estimated_cost, start_node))

    count = 1

    while not pq.empty():
        top = pq.get()
        cur_node = top[1]
        x = cur_node.x
        y = cur_node.y
        i, j, k = get_index(x, y, cur_node.th)
        visited[i][j][k] = 1
        if goal_check(x, y):
            print("goal reached")
            return cur_node, explored

        children = get_children(cur_node, visited)
        for child in children:
            p, q, r = get_index(child.x, child.y, child.th)
            cost_to_come = child.cost
            if (cost_to_come < cost[p][q][r]):
                explored.append(((2*i, 2*j), (2*p, 2*q)))
                child.parent = cur_node
                cost[p][q][r] = cost_to_come
                estimated_cost = get_estimated_cost(child, goal_point)
                pq.put((estimated_cost, child))

    return None, None

def is_start_node(node):
    if(node.x == start_point[0] and node.y == start_point[1]):
        return True

    return False


# function to backtrace the path
def backtrace(node):

    rev_path =[]
    count = 1
    while (not is_start_node(node)):
        rev_path.append((node.x, node.y))
        node = node.parent
        count = count + 1

    for i in range(len(rev_path)-1, -1, -1):
        path.append(rev_path[i])

    return path

# main function
if __name__ == '__main__':

    # get robot radius
    r = int(input('Enter robot radius: '))
    # r = 1

    # get clearance
    c = int(input('Enter clearance: '))
    # c = 1

    # get step size
    d = int(input('Enter robot step size: '))
    # d = 1

    # get theta
    # theta = int(input('Enter mininum angle of turn: '))
    theta = 30

    # threshold
    thresh = r+c

    # thresholds
    thresh_d = 0.5
    thresh_theta = 30

    # generate map
    world = world_map()

    # Get points
    path = []
    start_point = startPoint()
    if (start_point == None):
        exit() 

    goal_point = goalPoint()

    if (goal_point == None):
        exit()    

    goal_node, explored = explorer(start_point, goal_point)

    if (goal_node != None):
        backtrace(goal_node)
    else:
        print ("No path found")

    world_image = map2.get_world()

    world_image = cv2.flip(world_image, 0)
    m, n, _ = world_image.shape

    # Display goal 
    cv2.circle(world_image, (4*goal_point[0], m-4*goal_point[1]-1), r, (0, 0, 255), thickness=-1)

    # Display explored regions
    count = 0
    for points in explored:
        count = count + 1
        cv2.arrowedLine(world_image, (points[0][0], m-points[0][1]-1), (points[1][0], m-points[1][1]-1), (0, 255, 0))
        cv2.imshow("Final Path", world_image)
        if count == len(explored):
            cv2.waitKey(5)
            break
        else:
            cv2.waitKey(1)

    # Display path
    count = 1
    for loc in path:
        count = count + 1
        x, y, _ = get_index(loc[0], loc[1], 0)
        cv2.circle(world_image, (2*x, m-2*y-1), 2*r, (255, 0, 0), thickness=-1)
        cv2.imshow("Final Path", world_image)
        if count == len(path):
            cv2.waitKey(0)
        else:
            cv2.waitKey(1)