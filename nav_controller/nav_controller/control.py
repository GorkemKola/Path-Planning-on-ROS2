import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import heapq
from nav_msgs.msg import OccupancyGrid , Odometry
from geometry_msgs.msg import PoseStamped , Twist
import math
import scipy.interpolate as si
from rclpy.qos import QoSProfile
import random

lookahead_distance = 0.15
speed = 0.1
expansion_size = 2 #for the wall

def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


'''
PATH FINDING ALGORTITHMS
'''
'''
ASTAR
'''
def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data, gscore.keys()
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data, gscore.keys()
    return False
'''
DIJKSTRA
'''
def dijkstra(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    oheap = []
    heapq.heappush(oheap, (0, start))  # Initial priority is 0
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data, gscore
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + 1  # Assuming each step costs 1
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                heapq.heappush(oheap, (tentative_g_score, neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = abs(node[0] - goal[0]) + abs(node[1] - goal[1])  # Manhattan distance
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data, gscore
    return False
'''
RRT
'''
class RRTNode:
    def __init__(self, position):
        self.position = position
        self.children = []

def rrt(array, start, goal, max_iter=1000, step_size=1):
    root = RRTNode(start)
    for _ in range(max_iter):
        random_point = np.random.randint(0, array.shape[0]), np.random.randint(0, array.shape[1])
        nearest_node = nearest(root, random_point)
        new_point = step_from_to(nearest_node.position, random_point, step_size)
        if is_valid_point(array, new_point):
            new_node = RRTNode(new_point)
            nearest_node.children.append(new_node)
            if np.linalg.norm(np.array(new_point) - np.array(goal)) < step_size:
                path = backtrack(new_node, start)
                return path
    return False

def nearest(node, point):
    min_dist = float('inf')
    nearest_node = None
    for child in get_descendants(node):
        dist = np.linalg.norm(np.array(child.position) - np.array(point))
        if dist < min_dist:
            min_dist = dist
            nearest_node = child
    return nearest_node

def step_from_to(p1, p2, step_size):
    direction = np.array(p2) - np.array(p1)
    distance = np.linalg.norm(direction)
    if distance <= step_size:
        return p2
    else:
        normalized_direction = direction / distance
        new_point = np.array(p1) + normalized_direction * step_size
        return tuple(new_point.astype(int))

def is_valid_point(array, point):
    x, y = point
    if 0 <= x < array.shape[0] and 0 <= y < array.shape[1] and array[x][y] == 0:
        return True
    return False

def backtrack(node, start):
    path = []
    current = node
    while current is not None:
        path.append(current.position)
        current = find_parent(current)
    path.append(start)
    return path[::-1]

def find_parent(node):
    if len(node.children) > 0:
        return node.children[0]
    return None

def get_descendants(node):
    queue = [node]
    descendants = []
    while queue:
        current = queue.pop(0)
        descendants.append(current)
        queue.extend(current.children)
    return descendants

'''
FINISHED
'''
def bspline_planning(x, y, sn):
    N = 2
    t = range(len(x))
    x_tup = si.splrep(t, x, k=N)
    y_tup = si.splrep(t, y, k=N)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, sn)
    rx = si.splev(ipl_t, x_list)
    ry = si.splev(ipl_t, y_list)

    return rx, ry

def pure_pursuit(current_x, current_y, current_heading, path,index):
    global lookahead_distance
    closest_point = None
    v = speed
    for i in range(index,len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path)-1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0
    return v,desired_steering_angle,index

def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data

def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i],ry[i]) for i in range(len(rx))]
    except:
        path = array
    return path

def visualize(data, width, height, path, discovered):
    data = np.array(data).reshape(height,width)
    for y, x in discovered:
        data[y, x] = 2
    for y, x  in path:
        data[y, x] = 3

    data[path[0][0], path[0][1]] = 4
    data[path[-1][0], path[-1][1]] = 4
    
    plt.figure(figsize=(8, 6))
    plt.imshow(data, cmap='viridis', interpolation='nearest')  # Change the colormap as needed
    plt.title('Path Visualization')
    plt.grid(True)
    plt.show()

class navigationControl(Node):
    def __init__(self):
        super().__init__('Navigation')
        self.subscription = self.create_subscription(OccupancyGrid,'map',self.listener_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.info_callback,10)
        self.subscription = self.create_subscription(PoseStamped,'goal_pose',self.goal_pose_callback,QoSProfile(depth=10))
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.path_finder = {'astar':astar,
                       'dijkstra':dijkstra,
                       'rrt':rrt}
        self.method = input('please type (astar / dijkstra / rrt):\n')

        print("Please set up a target")
        self.flag = 0

    def goal_pose_callback(self,msg):
        self.goal = (msg.pose.position.x,msg.pose.position.y)
        print("Target Location: ",self.goal[0],self.goal[1])
        self.flag = 1

    def listener_callback(self,msg):
        if self.flag == 1:
            resolution = msg.info.resolution
            originX = msg.info.origin.position.x
            originY = msg.info.origin.position.y
            column = int((self.x- originX)/resolution) #x,y koordinatlarından costmap indislerine çevirme
            row = int((self.y- originY)/resolution) #x,y koordinatlarından costmap indislerine çevirme
            columnH = int((self.goal[0]- originX)/resolution)#x,y koordinatlarından costmap indislerine çevirme
            rowH = int((self.goal[1]- originY)/resolution)#x,y koordinatlarından costmap indislerine çevirme
            data = costmap(msg.data,msg.info.width,msg.info.height,resolution) #costmap düzenleme
            self.data = data
            self.width = msg.info.width
            self.height = msg.info.height

            data[row][column] = 0 #robot konumu
            data[data < 0] = 1 
            data[data > 5] = 1 
            path, self.discovered = self.path_finder[self.method](data,(row,column),(rowH,columnH)) #astar algoritması ile yol bulma
            self.path1 = path
            path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path] #x,y koordinatlarına çevirme
            self.path = bspline_planning(path,len(path)*5) #bspline ile düzeltme
            print("Robot Location: ",self.x,self.y)
            print("Moving to Target...")
            self.i = 0
            self.flag = 2
    def timer_callback(self):
        if self.flag == 2:
            twist = Twist()
            twist.linear.x , twist.angular.z,self.i = pure_pursuit(self.x,self.y,self.yaw,self.path,self.i)
            if(abs(self.x - self.path[-1][0]) < 0.05 and abs(self.y - self.path[-1][1])< 0.05):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                self.flag = 0
                print("Target Achieved.\n")
                visualize(self.data, self.width, self.height, self.path1, self.discovered)

                self.method = input("astar or dijkstra:\n")
                print("Please set up a new target")


            self.publisher.publish(twist)
    def info_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)


def main(args=None):
    rclpy.init(args=args)
    navigation_control = navigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
