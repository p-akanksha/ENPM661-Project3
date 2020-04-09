import cv2
import numpy as np
import math
import map2
from Queue import PriorityQueue
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
class explored_nodes:
    def __init__(self, x, y, th, parent, cost, loc):
        self.x = x
        self.y = y
        self.th = th
        self.parent = parent
        self.cost = cost
        self.loc = loc

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
    points1 = [[0.25, 6.75, 1.75, 6.75, 0],
               [1.75, 6.75, 1.75, 5.25, 0],
               [1.75, 5.25, 0.25, 5.25, 1],
               [0.25, 5.25, 0.25, 6.75, 1]]

    obs1 = obs_polygon(points1) 
    w_map.append(obs1)

    # obstacle2
    points2 = [[2.25, 8.75, 3.75, 8.75, 0],
               [3.75, 8.75, 3.75, 7.25, 0],
               [3.75, 7.25, 2.25, 7.25, 1],
               [2.25, 7.25, 2.25, 8.75, 1]]

    obs2 = obs_polygon(points2)
    w_map.append(obs2) 

    # obstacle3
    points3 = [[8.25, 5.75, 9.75, 5.75, 0],
               [9.75, 5.75, 9.75, 4.25, 0],
               [9.75, 4.25, 8.25, 4.25, 1],
               [8.25, 4.25, 8.25, 5.75, 1]]

    obs3 = obs_polygon(points3)
    w_map.append(obs3)

    # obstacle4
    obs4 = obs_ellipse(3, 2, 1, 1, 1+thresh)
    w_map.append(obs4)
    # obstacle5
    obs5 = obs_ellipse(7, 2, 1, 1, 1+thresh)
    w_map.append(obs5)
    # obstacle6
    obs6 = obs_ellipse(5, 5, 1, 1, 1+thresh)
    w_map.append(obs6)
    # obstacle7
    obs7 = obs_ellipse(7, 8, 1, 1, 1+thresh)
    w_map.append(obs7)

    return w_map

# function to get start points
def startPoint():
    # sx = int(input('Enter x coordinate for start point: '))
    # sy = int(input('Enter y coordinate for start point: '))
    # s_th = int(input('Enter theta for start point: '))
    sx = 1
    sy = 2
    s_th = 0
    return sx, sy, s_th


# function to get goal points
def goalPoint():
    # gx = int(input('Enter x coordinate for goal point: '))
    # gy = int(input('Enter y coordinate for goal point: '))
    gx = 5
    gy = 2
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
    i = round_off(x/thresh_d)
    j = round_off(y/thresh_d)
    k = th/30

    return int(i), int(j), int(k)

def check_collision(node):
    res = False

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

    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes

    loc = [Xn, Yn]
    dist = 0

    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        temp = math.sqrt((Xn - Xs) ** 2 + (Yn - Ys) ** 2)
        dist = dist + temp
        # plt.plot([Xs, Xn], [Ys, Yn], color="blue")
        loc.append([Xn, Yn])

    if Thetan > 3.14:
        n = int(Thetan/3.14)
        Thetan = Thetan - n*3.14

    Thetan = 180 * (Thetan) / 3.14
    if Thetan < 0:
        Thetan = -Thetan
    print(Thetan)

    return Xn, Yn, Thetan, loc, dist

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
        x, y, th, loc, dist = get_loc(x_init, y_init, th_init, vel[i][0], vel[i][1])
        # x_ = node.x + d * math.cos(math.radians(th))
        # y_ = node.y + d * math.sin(math.radians(th))
        # th = abs((node.th + i*(theta))%360)

        j, k, l = get_index(x, y, th)
        if (j >= int(10/thresh_d) or j < 0 or k >= int(10/thresh_d) or k < 0):
            continue
        if(visited[j][k][l] == 0):
            new_node = explored_nodes(x, y, th, node, dist, loc)
            if(not check_collision(new_node)):
                children.append(new_node)

    return children

# function to explore neighbors and lot more
def explorer(start_point, goal_point):
    # create visited and cost array
    size_x = int(10/thresh_d)
    size_y = int(10/thresh_d)
    size_z = 360/thresh_theta
    visited = np.asarray([[[0] * size_z] * size_y]* size_x)
    cost = np.asarray([[[float('inf')] * size_z] * size_y]* size_x)
    print("visited: ", visited.shape)
    explored  = []

    start_node = explored_nodes(start_point[0], start_point[1], start_point[2], -1, 0, None)
    estimated_cost = get_estimated_cost(start_node, goal_point)
    # i1, j1, _ = get_index(start_node.x, start_node.y, start_node.th)

    pq = PriorityQueue()
    pq.put((estimated_cost, start_node))

    count = 1

    while not pq.empty():
        top = pq.get()
        cur_node = top[1]
        # if (count == 2):
        #     print(count)
        #     return cur_node, explored
        # count = count + 1
        # explored.append(cur_node)
        x = cur_node.x
        y = cur_node.y
        i, j, k = get_index(x, y, cur_node.th)
        # print(i, j, k)
        # print(pq.qsize())
        visited[i][j][k] = 1
        if goal_check(x, y):
            print("goal reached")
            return cur_node, explored

        children = get_children(cur_node, visited)
        # print(len(children))
        for child in children:
            p, q, r = get_index(child.x, child.y, child.th)
            # print(cost[p][q][r])
            cost_to_come = child.cost
            if (cost_to_come < cost[p][q][r]):
                # print(i, j, p, q)
                explored.append(((cur_node.x, cur_node.y), (child.x, child.y)))
                child.parent = cur_node
                cost[p][q][r] = cost_to_come
                estimated_cost = get_estimated_cost(child, goal_point)
                pq.put((estimated_cost, child))
            # print(cost[p][q][r])

    return None, explored

def is_start_node(node):
    if(node.x == start_point[0] and node.y == start_point[1]):
        return True

    return False


# function to backtrace the path
def backtrace(node):

    count = 1
    while (not is_start_node(node)):
        path.append(node.loc)
        node = node.parent
        count = count + 1


    return path
    # if is_start_node(node):
    #     return path
    # else:
    #     backtrace(node.parent)
    #     path.append((node.x, node.y))
    #     return 

# main function
if __name__ == '__main__':

    # Get points
    path = []
    start_point = startPoint()
    goal_point = goalPoint()

    # get two RPM from user
    # rpm1 = int(input('Enter RPM1: '))
    # rpm2 = int(input('Enter RPM2: '))
    rpm1 = 20
    rpm2 = 40


    # get robot radius (177 mm)
    # r = int(input('Enter robot radius: '))
    r = 0.177

    # get clearance (100 mm)
    # c = int(input('Enter clearance: '))
    c = 0.100

    # get step size (100 mm)
    # d = int(input('Enter robot step size: '))
    d = 0.1

    # wheel distance (354 mm)
    l = 0.354

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
    thresh_d = 0.2
    thresh_theta = 30

    goal_node, explored = explorer(start_point, goal_point)

    if (goal_node != None):
        backtrace(goal_node)
    else:
        print ("No path found")

     # = []

    # print(len(path))
    print("All done")

    path = np.asarray(path)
    print(path.shape)
    
    # for loc in path:
    #     print(loc)

    # world_image = map2.get_world()

    # cv2.imshow("Explored region", world_image)
    # if cv2.waitKey(0) & 0xff == 27:
    # # if count == len(explored):
    #     cv2.destroyAllWindows()

    # # print(len(explored))

    # count = 0
    # for points in explored:
    #     # print(count)
    #     count = count + 1
    #     print(points[0][0], points[0][1])
    #     print(points[1][0], points[1][1])
    #     cv2.line(world_image, points[0], points[1], (0, 255, 0))
    #     # rgb_w[int(cord[0]), int(cord[1]), :] = [255, 0, 0]
    #     cv2.imshow("Final Path", world_image)
    #     if count == len(explored):
    #         cv2.waitKey(0)
    #         break
    #     else:
    #         cv2.waitKey(1)

    fig=plt.figure()
    ax=fig.add_subplot(1,1,1)
    ax.axis('equal')
    ax.set_xlim([-0.5,10.5])
    ax.set_ylim([-0.5,10.5])

    points1 = [[0.25, 6.75, 1.75, 6.75, 0],
               [1.75, 6.75, 1.75, 5.25, 0],
               [1.75, 5.25, 0.25, 5.25, 1],
               [0.25, 5.25, 0.25, 6.75, 1]]

    points2 = [[2.25, 8.75, 3.75, 8.75, 0],
               [3.75, 8.75, 3.75, 7.25, 0],
               [3.75, 7.25, 2.25, 7.25, 1],
               [2.25, 7.25, 2.25, 8.75, 1]]

    points3 = [[8.25, 5.75, 9.75, 5.75, 0],
               [9.75, 5.75, 9.75, 4.25, 0],
               [9.75, 4.25, 8.25, 4.25, 1],
               [8.25, 4.25, 8.25, 5.75, 1]]

    points4 = [[0, 10, 10, 10, 0],
               [10, 10, 10, 0, 0],
               [10, 0, 0, 0, 1],
               [0, 0, 0, 10, 1]]

    points5 = [[-0.2, 10.2, 10.2, 10.2, 0],
               [10.2, 10.2, 10.2, -0.2, 0],
               [10.2, -0.2, -0.2, -0.2, 1],
               [-0.2, -0.2, -0.2, 10.2, 1]]


    for i in range (4):
        # print(points1[i][0], points1[i][1], points1[i][2], points1[i][3])

        plt.plot( [points1[i][0], points1[i][2]], [points1[i][1], points1[i][3]], color='k', linewidth=1)   
        plt.plot( [points2[i][0], points2[i][2]], [points2[i][1], points2[i][3]], color='k', linewidth=1)
        plt.plot( [points3[i][0], points3[i][2]], [points3[i][1], points3[i][3]], color='k', linewidth=1)
        plt.plot( [points4[i][0], points4[i][2]], [points4[i][1], points4[i][3]], color='k', linewidth=1)
        plt.plot( [points5[i][0], points5[i][2]], [points5[i][1], points5[i][3]], color='k', linewidth=1)

    circ1 = plt.Circle((3, 2), 1, fill = False)
    circ2 = plt.Circle((7, 2), 1, fill = False)
    circ3 = plt.Circle((5, 5), 1, fill = False)
    circ4 = plt.Circle((7, 8), 1, fill = False)
    circ5 = plt.Circle((5, 2), 0.2, fill = True, color = 'g')
    # ellipse = Ellipse(xy=(150,100), width=80, height=40, 
    #                         edgecolor='k', fc='None', lw=1)

    ax.add_patch(circ1)
    ax.add_patch(circ2)
    ax.add_patch(circ3)
    ax.add_patch(circ4)
    ax.add_patch(circ5)
    # ax.add_patch(ellipse)

    plt.ion()
    plt.show()
    count = 0
    for points in explored:
        # print(count)
        count = count + 1
        plt.plot( [points[0][0], points[1][0]], [points[0][1], points[1][1]], color='g', linewidth=0.3)   
        plt.pause(0.01)
    
    plt.hold(True)
    loc_x=[start_point[0]]
    loc_y=[start_point[1]]
    for loc in path:
        loc_x.append(loc[0])
        loc_y.append(loc[1])
   
    ax.scatter(loc_x, loc_y, s=r, color='r')

    plt.pause(10)
    #     cv2.circle(world_image, (x, y), r, (255, 0, 0))
    #     # rgb_w[int(cord[0]), int(cord[1]), :] = [255, 0, 0]
    #     cv2.imshow("Final Path", world_image)
    #     if count == len(path):
    #         cv2.waitKey(0)
    #     else:
    #         cv2.waitKey(1)
