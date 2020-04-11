import cv2
import os
import time
import numpy as np
import math
import map2
from Queue import PriorityQueue
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

class explored_nodes:
    def __init__(self, x, y, th, parent, cost, loc, UL, UR):
        self.x = x
        self.y = y
        self.th = th
        self.parent = parent
        self.cost = cost
        self.loc = loc
        self.UR = UR
        self.UL = UL

    def pretty_print(self):
        print(" ")
        print("x, y, th: ", self.x, self.y, self.th)
        print("cost: ", self.cost)
        

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
            val = l[0] * x + l[1] * y + l[2]
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
    points1 = [[-4.75, 1.75, -3.25, 1.75, 0],
               [-3.25, 1.75, -3.25, 0.25, 0],
               [-3.25, 0.25, -4.75, 0.25, 1],
               [-4.75, 0.25, -4.75, 1.75, 1]]

    obs1 = obs_polygon(points1) 
    w_map.append(obs1)

    # obstacle2
    points2 = [[-2.75, 3.75, -1.25, 3.75, 0],
               [-1.25, 3.75, -1.25, 2.25, 0],
               [-1.25, 2.25, -2.75, 2.25, 1],
               [-2.75, 2.25, -2.75, 3.75, 1]]

    obs2 = obs_polygon(points2)
    w_map.append(obs2) 

    # obstacle3
    points3 = [[3.25, 0.75, 4.75, 0.75, 0],
               [4.75, 0.75, 4.75, -0.75, 0],
               [4.75, -0.75, 3.25, -0.75, 1],
               [3.25, -0.75, 3.25, 0.75, 1]]

    obs3 = obs_polygon(points3)
    w_map.append(obs3)

    # obstacle4
    obs4 = obs_ellipse(-2, -3, 1, 1, 1+thresh)
    w_map.append(obs4)
    # obstacle5
    obs5 = obs_ellipse(2, -3, 1, 1, 1+thresh)
    w_map.append(obs5)
    # obstacle6
    obs6 = obs_ellipse(0, 0, 1, 1, 1+thresh)
    w_map.append(obs6)
    # obstacle7
    obs7 = obs_ellipse(2, 3, 1, 1, 1+thresh)
    w_map.append(obs7)

    return w_map

# function to visualize map
def visualizeMap():
    fig=plt.figure()
    ax=fig.add_subplot(1,1,1)
    ax.axis('equal')
    ax.set_xlim([-5.5, 5.5])
    ax.set_ylim([-5.5, 5.5])

    # obstacle 1
    points1 = [[-4.75, 1.75, -3.25, 1.75, 0],
               [-3.25, 1.75, -3.25, 0.25, 0],
               [-3.25, 0.25, -4.75, 0.25, 1],
               [-4.75, 0.25, -4.75, 1.75, 1]]

    # obstacle 2
    points2 = [[-2.75, 3.75, -1.25, 3.75, 0],
               [-1.25, 3.75, -1.25, 2.25, 0],
               [-1.25, 2.25, -2.75, 2.25, 1],
               [-2.75, 2.25, -2.75, 3.75, 1]]

    # obstacle 3
    points3 = [[3.25, 0.75, 4.75, 0.75, 0],
               [4.75, 0.75, 4.75, -0.75, 0],
               [4.75, -0.75, 3.25, -0.75, 1],
               [3.25, -0.75, 3.25, 0.75, 1]]

    # world boundary 1
    points4 = [[-5, 5, 5, 5, 0],
               [5, 5, 5, -5, 0],
               [5, -5, -5, -5, 1],
               [-5, -5, -5, 5, 1]]

    # world boundary 2
    points5 = [[-5.2, 5.2, 5.2, 5.2, 0],
               [5.2, 5.2, 5.2, -5.2, 0],
               [5.2, -5.2, -5.2, -5.2, 1],
               [-5.2, -5.2, -5.2, 5.2, 1]]


    for i in range (4):
        plt.plot( [points1[i][0], points1[i][2]], [points1[i][1], points1[i][3]], color='k', linewidth=1)   
        plt.plot( [points2[i][0], points2[i][2]], [points2[i][1], points2[i][3]], color='k', linewidth=1)
        plt.plot( [points3[i][0], points3[i][2]], [points3[i][1], points3[i][3]], color='k', linewidth=1)
        plt.plot( [points4[i][0], points4[i][2]], [points4[i][1], points4[i][3]], color='k', linewidth=1)
        plt.plot( [points5[i][0], points5[i][2]], [points5[i][1], points5[i][3]], color='k', linewidth=1)

    # circular obstacles
    circ1 = plt.Circle((-2, -3), 1, fill = False)
    circ2 = plt.Circle((2, -3), 1, fill = False)
    circ3 = plt.Circle((0, 0), 1, fill = False)
    circ4 = plt.Circle((2, 3), 1, fill = False)

    # start and goal points
    circ5 = plt.Circle((goal_point[0], goal_point[1]), 0.2, fill = True, color = 'g')
    circ6 = plt.Circle((start_point[0], start_point[1]), 0.2, fill = True, color = 'r')

    ax.add_patch(circ1)
    ax.add_patch(circ2)
    ax.add_patch(circ3)
    ax.add_patch(circ4)
    ax.add_patch(circ5)
    ax.add_patch(circ6)

    return fig, ax

# function to get start points
def startPoint():
    # sx = int(input('Enter x coordinate for start point: '))
    # sy = int(input('Enter y coordinate for start point: '))
    # s_th = int(input('Enter theta for start point: '))
    sx = -4
    sy = -3
    s_th = 0
    if (sx <= -5 or sx >= 5 or sy <= -5 or sy >= 5):
        print("Invalid input. Start point either lies on boundry or lies outside the map")
        return None

    if (check_collision(explored_nodes(sx, sy, s_th, -1, 0, None, 0, 0))):
        print("Invalid input. Start point lies inside the obstacle or is too close to obstacle")
        return None
    return sx, sy, s_th


# function to get goal points
def goalPoint():
    # gx = int(input('Enter x coordinate for goal point: '))
    # gy = int(input('Enter y coordinate for goal point: '))
    # gx = 8
    # gy = 6.5
    gx = -2
    gy = -1
    if (gx <= -5 or gx >= 5 or gy <= -5 or gy >= 5):
        print("Invalid input. Goal point either lies on boundry or lies outside the map")
        return None
    if (check_collision(explored_nodes(gx, gy, 0, -1, 0, None, 0, 0))):
        print("Invalid input. Goal point lies inside the obstacle or is too close to obstacle")
        return None
    return gx, gy

def get_estimated_cost(node, goal_point):
    h = math.sqrt((node.x - goal_point[0]) ** 2 + (node.y - goal_point[1]) ** 2)
    return node.cost + h

def goal_check(x, y):
    dist = math.sqrt((x - goal_point[0]) ** 2 + (y - goal_point[1]) ** 2)

    if dist < 0.2:
        return True
    else:
        return False

def start_check(node):
    if(node.x == start_point[0] and node.y == start_point[1]):
        return True

    return False

def round_off(a):
    return (round(a))

def get_index(x, y, th):
    i = round_off(x/thresh_d)
    j = round_off(y/thresh_d)
    k = round_off(th/thresh_theta)

    return int(i), int(j), int(k)

def check_collision(node):
    res = False

    if node.x >= 5 or node.x <= -5 or node.y >= 5 or node.y <= -5:
        return True

    for obs in world:
        if(obs.check_collision(node.x, node.y)):
            res = True
            break;

    return res

def get_loc(Xi, Yi, Thetai, UL, UR):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180

    loc = []
    loc.append([Xn, Yn])
    dist = 0

    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += (2*math.pi/60)*(0.5*r * (UL + UR) * math.cos(Thetan) * dt)
        Yn += (2*math.pi/60)*(0.5*r * (UL + UR) * math.sin(Thetan) * dt)
        Thetan += (2*math.pi/60)*((r / L) * (UR - UL) * dt)
        
        if (check_collision(explored_nodes(Xn, Yn, Thetan, -1, 0, None, 0, 0))):
            return None, None, None, None, None, False
        
        temp = math.sqrt((Xn - Xs) ** 2 + (Yn - Ys) ** 2)
        dist = dist + temp
        loc.append([Xn, Yn])

    if Thetan > 6.28 or Thetan < -6.28:
        n = int(Thetan/6.28)
        Thetan = Thetan - n*6.28

    if Thetan > 3.14:
        Thetan = Thetan - 6.28
    elif Thetan < -3.14:
        Thetan = Thetan + 6.28

    Thetan = 180 * (Thetan) / 3.14

    return Xn, Yn, Thetan, loc, dist, True

def get_children(node, visited):
    children = []
    x_init = node.x
    y_init = node.y
    th_init = node.th

    vel = np.asarray([[0, rpm1],
                      [rpm1, 0],
                      [rpm1, rpm1],
                      [0, rpm2],
                      [rpm2, 0],
                      [rpm2, rpm2],
                      [rpm1, rpm2],
                      [rpm2, rpm1]])


    for i in range(8):
        x, y, th, loc, dist, check = get_loc(x_init, y_init, th_init, vel[i][0], vel[i][1])

        if (check):
            j, k, l = get_index(x, y, th)
            if (j >= int(5/thresh_d) or j < -int(5/thresh_d) or k >= int(5/thresh_d) or k < -int(5/thresh_d)):
                continue

            if(visited[j][k][l] == 0):
                new_node = explored_nodes(x, y, th, node, node.cost + dist, loc, (2*math.pi/60)*vel[i][0], (2*math.pi/60)*vel[i][1])
                if(not check_collision(new_node)):
                    children.append(new_node)

    return children

# function to explore neighbors and lot more
def explorer(start_point, goal_point):
    # text file to save visited nodes
    open('Nodes.txt', 'w').close() 
    f = open("Nodes.txt", "a+")
    
    # create visited and cost array
    size_x = int(10/thresh_d)
    size_y = int(10/thresh_d)
    size_z = 360/thresh_theta
    visited = np.asarray([[[0] * size_z] * size_y]* size_x)
    cost = np.asarray([[[float('inf')] * size_z] * size_y]* size_x)
    explored  = []

    start_node = explored_nodes(start_point[0], start_point[1], start_point[2], -1, 0, None, 0, 0)
    estimated_cost = get_estimated_cost(start_node, goal_point)

    # initialize priority queue
    pq = PriorityQueue()
    pq.put((estimated_cost, start_node))

    count = 1

    while not pq.empty():
        # get the node with lowest cost
        top = pq.get()
        cur_node = top[1]

        # write node in text file
        f.write(str(cur_node) + '\n')

        # mark the node as visited
        x = cur_node.x
        y = cur_node.y
        i, j, k = get_index(x, y, cur_node.th)
        visited[i][j][k] = 1

        # check if this is the goal
        if goal_check(x, y):
            print("goal reached")
            f.close()
            return cur_node, explored

        # get node children
        children = get_children(cur_node, visited)
        for child in children:
            p, q, r = get_index(child.x, child.y, child.th)
            cost_to_come = child.cost
            if (cost_to_come < cost[p][q][r]):
                explored.append(((cur_node.x, cur_node.y), (child.x, child.y)))
                child.parent = cur_node
                cost[p][q][r] = cost_to_come
                estimated_cost = get_estimated_cost(child, goal_point)
                pq.put((estimated_cost, child))
        count = count + 1

    f.close()
    return None, explored

# function to backtrace the path
def backtrace(node):
    # text file to save node in path
    open('nodePath.txt', 'w').close() 
    f = open("nodePath.txt", "a+")
    content = f.read()

    count = 1
    while (not start_check(node)):
        path.append(node.loc)
        vel.append((node.UL, node.UR))
        node = node.parent
        count = count + 1
        f.write(str(node) + '\n' + content)
    
    f.close()
    return path 

# main function
if __name__ == '__main__':
    # get two RPM from user
    # rpm1 = int(input('Enter RPM1: '))
    # rpm2 = int(input('Enter RPM2: '))
    rpm1 = 100
    rpm2 = 150


    # Robot Parameters
    R = 0.177 # robot radius (177 mm)
    r = 0.038 # wheel radius (38mm)
    C = 0.05 # robot clearance (50 mm)
    L = 0.354 # wheel distance (354 mm)

    # threshold
    thresh = R + C

    # generate map
    world = world_map()

    
    path = []
    vel = []

    # Get start and goal points
    start_point = startPoint()
    goal_point = goalPoint()

    if start_point == None or goal_point == None:
        exit(0)

    # precision values for checking duplicate nodes
    thresh_d = 0.1
    thresh_theta = 10

    t = time.time()

    goal_node, explored = explorer(start_point, goal_point)
    explored = np.asarray(explored)

    if (goal_node != None):
        backtrace(goal_node)
        path = np.asarray(path)
        m, n, _ = path.shape
        path = np.reshape(path, (n*m, 2))
        vel = np.asarray(vel)
    else:
        print ("No path found")

    # stop timer
    temp_t = t
    t = time.time()

    print('Time taken by algorithm: ', t - temp_t, 'sec')
    print("Total nodes explored" + str(explored.shape))

    # visualize map
    fig, ax = visualizeMap()

    plt.savefig("background.png")
    temp_img = cv2.imread("background.png")
    print(temp_img.shape)

    # video writer
    out = cv2.VideoWriter('Astar.avi', cv2.VideoWriter_fourcc(*'DIVX'), 15,
                              (temp_img.shape[1], temp_img.shape[0]))

    # plot explored nodes and save video
    count = 0
    for points in explored:
        count = count + 1
        plt.plot( [points[0][0], points[1][0]], [points[0][1], points[1][1]], color='g', linewidth=0.3)   
        if count%10 == 0:
            plt.savefig("Frames/frame" + str(count/10) + ".png")
            frame = cv2.imread("Frames/frame" + str(count/10) + ".png")
            out.write(frame)
        if count%1000 == 0:
            print(count)
    
    # plot shortest path
    plt.hold(True)
    loc_x=[start_point[0]]
    loc_y=[start_point[1]]
    for loc in path:
        loc_x.append(loc[0])
        loc_y.append(loc[1])
   
    ax.scatter(loc_x, loc_y, s=0.2, color='r')

    # Save figure with shortest path
    plt.savefig("Frames/frame" + str(count) + ".png")
    path_img = cv2.imread("Frames/frame" + str(count) + ".png")
    for i in range(75):
        out.write(path_img)

    out.release() 

    plt.show()
