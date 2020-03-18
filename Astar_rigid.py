import cv2
import numpy as np
import math
from Queue import PriorityQueue
import time

class explored_nodes:
    def __init__(self, x, y, parent, cost):
        self.x = x
        self.y = y
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
            val = l[0] * x + l[1] * y + l[2]
            if l[3] == 1:
                if l[0] * x + l[1] * y + l[2] < 0:
                    res = False
                    break

            elif l[3] == 0:
                if l[0] * j + l[1] * i + l[2] > 0:
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
        if float(x - self.x) ** 2 / a ** 2 + float(y - self.y) ** 2 / b ** 2 <= radius ** 2:
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
    points2 = [[25, 185, 50, 185, 0],
              [50, 185, 50, 150, 0],
              [50, 150, 20, 120, 1],
              [20, 120, 25, 185, 0]]

    obs2 = obs_polygon(points2)
    w_map.append(obs2) 

    # obstacle3
    points31 = [[50, 185, 75, 185, 0],
              [75, 185, 100, 150, 0],
              [100, 150, 75, 120, 1],
              [75, 120, 50, 150, 1],
              [50, 150, 50, 185, 1]]

    obs31 = obs_polygon(points31)

    points32 = [[36, 77, 100, 39, 0],
              [100, 39, 95, 30, 1],
              [95, 30, 31, 68, 1],
              [31, 68, 36, 76, 0]]

    obs32 = obs_polygon(points32)
    w_map.append(obs31)
    w_map.append(obs32)

    # obstacle4
    obs4 = obs_ellipse(150, 100, 40+thresh, 20+thresh, 1)
    w_map.append(obs4)
    # obstacle5
    obs5 = obs_ellipse(225, 150, 1, 1, 25+thresh)
    w_map.append(obs5)

    return w_map

# def world(length, breadth):
#     w = np.ones((length, breadth))
#     return w

# def getLineParam(x1, y1, x2, y2):
#     if x2 - x1 == 0:
#         return 1, 0, -x1
#     else:
#         a = float(y2 - y1) / float(x2 - x1)
#     b = -1
#     c = y1 - a * x1

#     # print(a)

#     return -a, -b, -c


# def getMap(World):

#     points = [[200, 25, 225, 40, 0],
#               [250, 25, 225, 40, 0],
#               [250, 25, 225, 10, 1],
#               [200, 25, 225, 10, 1]]

#     # Get a, b, c values for every lines
#     lines = []
#     for i in range(4):
#         l = []
#         a, b, c = getLineParam(points[i][0], points[i][1], points[i][2], points[i][3])
#         l.append(a)
#         l.append(b)
#         l.append(c)
#         l.append(points[i][4])
#         lines.append(l)

#     lines = np.asarray(lines)

#     # Get obstacles for world
#     obstacle(lines, 200, 250, 10, 40, World)
#     circle_obstacle(150, 100, 40, 20, 1, World)
#     circle_obstacle(225, 150, 1, 1, 25, World)

#     points = [[25, 185, 50, 185, 0],
#               [50, 185, 50, 150, 0],
#               [50, 150, 20, 120, 1],
#               [20, 120, 25, 185, 0]]

#     # Get a, b, c values for every lines
#     lines1 = []
#     for i in range(4):
#         l = []
#         a, b, c = getLineParam(points[i][0], points[i][1], points[i][2], points[i][3])
#         l.append(a)
#         l.append(b)
#         l.append(c)
#         l.append(points[i][4])
#         lines1.append(l)

#     obstacle(lines1, 20, 50, 120, 185, World)

#     points = [[50, 185, 75, 185, 0],
#               [75, 185, 100, 150, 0],
#               [100, 150, 75, 120, 1],
#               [75, 120, 50, 150, 1],
#               [50, 150, 50, 185, 1]]

#     # Get a, b, c values for every lines
#     lines1 = []
#     for i in range(4):
#         l = []
#         a, b, c = getLineParam(points[i][0], points[i][1], points[i][2], points[i][3])
#         l.append(a)
#         l.append(b)
#         l.append(c)
#         l.append(points[i][4])
#         lines1.append(l)

#     obstacle(lines1, 50, 100, 120, 185, World)

#     points = [[36, 77, 100, 39, 0],
#               [100, 39, 95, 30, 1],
#               [95, 30, 31, 68, 1],
#               [31, 68, 36, 76, 0]]

#     # Get a, b, c values for every lines
#     lines1 = []
#     for i in range(4):
#         l = []
#         a, b, c = getLineParam(points[i][0], points[i][1], points[i][2], points[i][3])
#         l.append(a)
#         l.append(b)
#         l.append(c)
#         l.append(points[i][4])
#         lines1.append(l)

#     obstacle(lines1, 31, 100, 30, 77, World)

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
    gx = 195
    gy = 295
    return gx, gy


# function to get cost of every move
def getCostOfMove(cur, i, j):
    x, y = cur
    return math.sqrt((x - i) ** 2 + (y - j) ** 2)


# function to explore neighbors and lot more
def explorer(costs, c_pq):
    cost_pq = PriorityQueue()
    while not c_pq.empty():
        top = c_pq.get()
        explored.append(top[1])
        x, y = top[1]
        if (x, y) == goal_point:
            break
        for i in range(x-1, x+2):
            for j in range(y-1, y+2):
                if 0 <= i < 200 and 0 <= j < 300:
                    if w[i, j] == 1 and top[1] != (i, j):
                        temp_cost = costs[top[1]] + getCostOfMove(top[1], i, j)
                        if temp_cost < costs[i, j]:
                            c_pq.put((temp_cost, (i, j)))
                            x1 = int(top[1][0])
                            y1 = int(top[1][1])
                            parent[i, j, :] = [x1, y1]
                            costs[i][j] = temp_cost


# function to backtrace the path
def backtrace(x, y):
    if parent[int(x), int(y), 0] == -1:
        return path
    else:
        path.append(parent[x, y, :])
        return backtrace(int(parent[x, y, 0]), int(parent[x, y, 1]))


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

    # xx = explorer(visited, )

    # # generate rigid world with obstacles
    # w = world(200, 300)
    # getMap_rigid(w)

    # # original world
    # old_w = world(200, 300)
    # getMap(old_w)

    # # check goal_point lies on obstacle
    # if not w[goal_point]:
    #     print('Goal Point lies on obstacle !!')
    #     exit()

    # # check start point lies on obstacle
    # if not w[start_point]:
    #     print('Start Point lies on obstacle !!')
    #     exit()

    # # start timer
    # t = time.time()

    # # Arrays for cost, parent
    # cost_pq = PriorityQueue()
    # parent = np.zeros((200, 300, 2))
    # cost = np.empty_like(w)

    # # create empty parent and cost array
    # for i in range(w.shape[0]):
    #     for j in range(w.shape[1]):
    #         parent[i, j, :] = [-1, -1]
    #         cost[i, j] = float('inf')

    # cost[start_point] = 0
    # cost_pq.put((0, start_point))

    # # let's explore
    # explored = []
    # explorer(cost, cost_pq)

    # # get final path
    # path.append(goal_point)
    # temp_path = backtrace(goal_point[0], goal_point[1])
    # # print(temp_path)

    # # stop timer
    # temp_t = t
    # t = time.time()
    # print('Time taken by algorithm: ', t - temp_t, 'sec')

    # # entering in colorful world
    # old_w = 255 * old_w
    # old_w = old_w.astype(np.uint8)

    # rgb_w = cv2.cvtColor(old_w, cv2.COLOR_GRAY2RGB)

    # rgb_w = cv2.flip(rgb_w, 0)
    # m, n, _ = rgb_w.shape

    # # display animation
    # count = 1
    # for exp in explored:
    #     count = count + 1
    #     # cv2.circle(rgb_w, (int(exp[1]), int(exp[0])), 1, (0, 255, 0))
    #     rgb_w[m-int(exp[0])-1, int(exp[1]), :] = [0, 255, 0]
    #     cv2.imshow("Explored region", rgb_w)
    #     cv2.waitKey(1)
    #     if count == len(explored):
    #         cv2.destroyAllWindows()

    # count = 1
    # tem_path = temp_path[::-1]
    # for cord in tem_path:
    #     count = count + 1
    #     cv2.circle(rgb_w, (int(cord[1]), m-int(cord[0])-1), r, (255, 0, 0))
    #     # rgb_w[int(cord[0]), int(cord[1]), :] = [255, 0, 0]
    #     cv2.imshow("Final Path", rgb_w)
    #     if count == len(temp_path):

    #         # stop timer
    #         temp_t = t
    #         t = time.time()
    #         print('Time for visualisation: ', t - temp_t, 'sec')

    #         if cv2.waitKey(0) & 0xff == 27:
    #             cv2.destroyAllWindows()
    #     cv2.waitKey(10)


