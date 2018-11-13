#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
#from math import radians, degrees, pi, sqrt, atan2
from math import *


from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

import random
import numpy as np


PI = pi


#####   RRT Algorithm   #####
delta_max = 1.0 # stepsize ???
delta_min = 0.2
delta = 0.05 # obstacles distance
NUMNODES = 5000

class Node(object):

    """

    RRT Node
    point    : tuple (x,y) 
    parent   : Node

    """
    def __init__(self, point, parent):
        super(Node, self).__init__() # parent is a Node too
        self.point = point # position
        self.parent = parent # parent Node


class GlobalPlanner():
    def __init__(self):
        rospy.init_node('my_global_planner', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)

        # SUBSCRIBERS

        # Get the cost map
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.global_costmap_cb)

        # 
        #rospy.Subscriber('/my_goal', PoseStamped, self.goal_pose_cb)

        # Start Pose provided by the AMCL
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.current_pose_cb)


        self.pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1000)

        # Publisher to manually control
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.rate = rospy.Rate(1)



        # member variables
        self.MAP = None
        self.ROWS = None
        self.COLUMNS = None
        self.XDIM = None
        self.YDIM = None


        self.goal = None
        self.start = None

        self.current_pose = None

        self.count_pose = 0

        self.grid = None

        #rospy.spin()


        # Test goals
        #goal_x = self.goal.pose.position.x
        #goal_y = self.goal.pose.position.y

        # goal 1
        #goal_x = -1.029
        #goal_y = 0.021
        #goal_theta = 0.5

        # goal 2
        goal_x = 5.277
        goal_y = 3.576
        goal_theta = 0.728

        # goal 3
        #goal_x = -5.748
        #goal_y = 0.508
        #goal_theta = -3.071

        # goal 4
        #goal_x = -6.894
        #goal_y = -6.460
        #goal_theta = -2.355

        self.goal_waypoint = (goal_x, goal_y, goal_theta)

        waypoints = list()
        self.poses = list()

        while (self.grid == None):
            print " CHECK GRID"
            self.rate.sleep()

        # TODO improve this part
        if self.current_pose != None:

            self.start = self.current_pose 
            start_x = self.start.pose.pose.position.x
            start_y = self.start.pose.pose.position.y

            print start_x, start_y

            #self.grid = self.make_grid(self.MAP.data, self.ROWS, self.COLUMNS)
    
                # Create a list to hold the waypoint poses


            # list of points (x,y)
            path = self.RRT((start_x, start_y), (goal_x, goal_y))

            # list of poses(x,y, yaw)
            if len(path) > 0:
                print " Path computed!"
                waypoints = self.get_waypoints(path)
            else:
                rospy.is_shutdown()

            # list of PoseStamped messages
            
            for waypoint in waypoints:
                self.poses.append(self.to_pose_stamped(waypoint))

            self.poses.append(self.to_pose_stamped(self.goal_waypoint))    
            

        # Markers
        # Initialize the visualization markers for RViz
        self.init_markers()

        # Cube List. This way is faster than publish individual markers on Rviz
        # Set a visualization marker at each waypoint
        for pose in self.poses:
            p = Point()
            p = pose.pose.position
            self.markers.points.append(p)


        self.loop()


    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.15
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}

        # Define a marker publisher
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=500)

        # Initialize the marker points list
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.marker_id = marker_id
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']

        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()



    # get the cell coord of the center point of the robot
    def world_to_map(self, x, y):
        cell_column = int((x - self.MAP.info.origin.position.x) / self.MAP.info.resolution)
        cell_row = int((y - self.MAP.info.origin.position.y) / self.MAP.info.resolution)

        return cell_row, cell_column
    # get the center point of the robot from the map cell
    def map_to_world(self):
        pass

    def loop(self):

        count_pose = 0
        distance_min = 0.4 # 20 cm

        poses_lenght = len(self.poses)           

        self.pose_pub.publish(self.poses[count_pose])
        while (not rospy.is_shutdown() and count_pose < poses_lenght):

            self.marker_pub.publish(self.markers)   

            distance_husky_waypoint = self.dist((self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y),
              (self.poses[count_pose].pose.position.x, self.poses[count_pose].pose.position.y))

            if distance_husky_waypoint < distance_min:
                count_pose = count_pose + 1
                self.pose_pub.publish(self.poses[count_pose])

            self.rate.sleep()
            #rospy.spin()


    def get_waypoints(self, path):
        """
        return a list of waypoints
        waypoint = (x, y, yaw)

        """
        # 
        waypoints = list()

        for i in range(1, len(path)-1):
            y2 = path[i+1][1]
            y1 = path[i][1]

            x2 = path[i+1][0]
            x1 = path[i][0]

            yaw = np.arctan(abs(y2 - y1)/abs(x2 - x1))

            yaw = (180/PI)*yaw

            if (y2 > y1) and (x2 < x1):
                yaw = 180 - yaw
            elif (y2 < y1) and (x2 < x1):
                yaw = 180 + yaw
            elif (y2 < y1) and (x2 > x1):
                yaw = 360 -  yaw

            waypoints.append((path[i][0], path[i][1], yaw))
    
        # goal waypoint
        #quaternion = self.goal.pose.orientation
        #euler = tf.transformations.euler_from_quaternion(quaternion)
        #yaw = degrees(euler[2])
        yaw = degrees(self.goal_waypoint[2])
        waypoints.append((path[len(path)-1][0], path[len(path)-1][1], yaw))

        return waypoints



    def to_pose_stamped(self, waypoint):

        x = waypoint[0]
        y = waypoint[1]
        yaw = waypoint[2]

        """

        """
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose 

        
    # global costmap callback
    def global_costmap_cb(self, msg):
        self.MAP = msg
        self.ROWS = msg.info.height
        self.COLUMNS = msg.info.width

        self.XDIM = msg.info.resolution*self.COLUMNS
        self.YDIM = msg.info.resolution*self.ROWS

        # Make the map grid only after receive the costmap
        self.grid = self.make_grid(self.MAP.data, self.ROWS, self.COLUMNS)


    def current_pose_cb(self, msg):
        self.current_pose = msg

    def goal_pose_cb(self, msg):
        self.goal = msg



    def make_grid(self, map_data, grid_height, grid_width):
        

        grid_ =  np.zeros([grid_height, grid_width])
        # Occupancy grid
        for index in range(len(map_data)):
            row = (int)(index/grid_width)
            column = index%grid_width
            grid_[row][column] = map_data[index]

        #print "grid shape: "
        #print grid.shape
        return grid_


    # Distance between two points
    def dist(self, p1,p2):    
        return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

    def point_circle_collision(self, p1, p2, radius):
        distance = self.dist(p1,p2)
        if (distance <= radius):
            return True
        return False

    def steer(self, p1,p2):
        distance = self.dist(p1,p2)
        if distance < delta_max and distance > delta_min:
            return p2
        else:
            theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
            return p1[0] + delta_max*cos(theta), p1[1] + delta_max*sin(theta)


    def collision(self, p):    #check if point collides with the obstacle

        # TODO: get a point and transform into cell grid

        # TODO: verify if thi

        cell_row, cell_column = self.world_to_map(p[0], p[1])

        if cell_column < 0:
            return True
        elif cell_row < 0:
            return True
        elif cell_column >= self.grid.shape[1]:
            return True
        elif cell_row >= self.grid.shape[0]:
            return True
        elif self.grid[cell_row][cell_column] > 0:
            return True
       
        return False

    def step_from_to2(self,p1,p2,n):

        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + n*delta*cos(theta), p1[1] + n*delta*sin(theta)

    # Get a random point located in a free area
    def sample_free(self):
        while True:
            #x_rand = random.random()*self.XDIM, random.random()*self.YDIM
            x_rand = (random.random() - 0.5)*20, (random.random() - 0.5)*20
            if self.collision(x_rand) == False:
                return x_rand

    # 
    def get_nearest(self, nodes, x_rand):
        # seach
        nn = nodes[0]
        for node in nodes:
            if self.dist(node.point, x_rand) < self.dist(nn.point, x_rand):
                nn = node

        return nn

    def obstacle_free(self, node_nearest, x_rand):
        distance = self.dist(node_nearest.point, x_rand)
        n = 1
        if distance < delta:
            if self.collision(x_rand):
                return False
            else:
                return True
        else:
            for i in range(int(floor(distance/delta))):
                x_n = self.step_from_to2(node_nearest.point, x_rand, n)
                if self.collision(x_n):
                    return False
                n = n + 1
        
            return True

    def RRT(self, start_point, goal_point):
        GOAL_RADIUS = 0.5


        # list all nodes
        nodes = list()

        # list all nodes final path
        path = list()

        initialNode = Node(start_point, None)
        nodes.append(initialNode)

        print initialNode.point

        goalFound = False

        bool_test = False


        while len(nodes) < NUMNODES:
            count =+ 1
            foundNext = False

            

            # search a node until get one in fre space
            while foundNext == False:
                x_rand = self.sample_free() # random point in the free space
                x_nearest = self.get_nearest(nodes, x_rand) # return the nearest node
                x_new = self.steer(x_nearest.point, x_rand)
                if self.obstacle_free(x_nearest, x_new):
                    parent_node = x_nearest
                    nodes.append(Node(x_new, parent_node))
                    foundNext = True

            # check if the distance between the goal node and the new node is less
            #################
            # TODO
            if self.point_circle_collision(x_new, goal_point, GOAL_RADIUS):

                goalNode = Node(goal_point, nodes[len(nodes)-1])
                nodes.append(goalNode)

                # Final path
                currentNode = goalNode
                while currentNode.parent != None:
                    path.insert(0, currentNode.point)
                    currentNode = currentNode.parent

                # return list of points (x,y)
                return path

        return []

    def RRT_star(self):
        pass

    def PRM(self):
        pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        GlobalPlanner()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")