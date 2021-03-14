#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion, Pose2D
from ngeeann_av_msgs.msg import Path2D, State2D
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from std_msgs.msg import Float32
from utils.heading2quaternion import heading_to_quaternion
from utils.cubic_spline_planner import *
from utils.spline_planner import *

class FrenetPath:

    def __init__(self):

        # Intialise the longitudinal frenet coordinates
        self.s = []
        self.s_s = []
        self.s_ss = []
        self.s_sss = []

        # Initialise the lateral frenet coordinates
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []

class LocalPathPlanner:

    def __init__(self):
        ''' 
        Class constructor to initialise the class 
        '''
        # Initialise publishers
        self.local_planner_pub = rospy.Publisher('/ngeeann_av/path', Path2D, queue_size=10)
        self.path_viz_pub = rospy.Publisher('/ngeeann_av/viz_path', Path, queue_size=10)
        #self.collisions_pub = rospy.Publisher('/ngeeann_av/viz_collisions', Path, queue_size=10)
        self.target_vel_pub = rospy.Publisher('/ngeeann_av/target_velocity', Float32, queue_size=10)

        # Initialise subscribers
        self.goals_sub = rospy.Subscriber('/ngeeann_av/goals', Path2D, self.goals_cb, queue_size=10)
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=10)
        self.gridmap_sub = rospy.Subscriber('/map', OccupancyGrid, self.gridmap_cb, queue_size=10)

        # Load parameters
        try:
            self.planner_params = rospy.get_param("/local_path_planner")
            self.frequency = self.planner_params["update_frequency"]
            self.frame_id = self.planner_params["frame_id"]
            self.car_width = self.planner_params["car_width"]
            self.cg2frontaxle = self.planner_params["centreofgravity_to_frontaxle"]

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.ds = 1 / self.frequency
        self.origin_x = 0
        self.origin_y = 0

        # Class variables to use whenever within the class when necessary
        self.target_vel = 3.0
        self.ax = []
        self.ay = []
        self.gmap = OccupancyGrid()

    def goals_cb(self, msg):
        '''
        Callback function to recieve immediate goals from global planner in global frame
        '''
        self.ax = []
        self.ay = []
        
        for i in range(0, len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            self.ax.append(px)
            self.ay.append(py)

        print("\nGoals received: {}".format(len(msg.poses)))

    def vehicle_state_cb(self, msg):
        ''' 
        Callback function to recieve vehicle state information from localization in global frame
        '''
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta

    def gridmap_cb(self, msg):
        ''' 
        Callback function to recieve map data
        '''
        self.gmap = msg

    def target_index_calculator(self, cx, cy):  
        ''' 
        Calculates closest point along the path to vehicle front axle
        '''
        # Calculate position of the front axle
        fx = self.x + self.cg2frontaxle * -np.sin(self.yaw)
        fy = self.y + self.cg2frontaxle * np.cos(self.yaw)

        dx = [fx - icx for icx in cx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in cy] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        reference_idx = np.argmin(d) # Find the shortest distance in the array
        return reference_idx

    def determine_path(self, cx, cy, cyaw):
        ''' 
        Map function to validate and determine a path by the following steps:

        1: Identify vehicle's progress along path, search all points ahead for potential collisions
        2: Draft a collision avoidance strategy
        '''
        # Initializing map information
        width = self.gmap.info.width
        height = self.gmap.info.height
        resolution = self.gmap.info.resolution
        origin_x = self.origin_x
        origin_y = self.origin_y
        collisions = []
        collide_id = None

        # Current vehicle progress along path
        lateral_ref_id = self.target_index_calculator(cx, cy)

        #  Validates path of collisions
        for n in range(self.react_dist, len(cyaw) - self.react_dist - 1):

            # Draws side profile of the vehicle along the path ahead
            for i in np.arange(-0.5 * self.car_width, 0.5 * self.car_width, resolution):
                ix = int((cx[n] + i*np.cos(cyaw[n] - 0.5 * np.pi) - origin_x) / resolution)
                iy = int((cy[n] + i*np.sin(cyaw[n] - 0.5 * np.pi) - origin_y) / resolution)
                p = iy * width + ix
                if (self.gmap.data[p] != 0):
                    collisions.append(n)
        
        if len(collisions) != 0:
            
            cx, cy, cyaw = self.collision_avoidance(collisions, cx, cy, cyaw)

        return cx, cy, cyaw, collisions

    def create_viz_path(self):
        ''' 
        Publish paths to RViz
        '''
        # Validated path returned
        cx, cy, cyaw, collisions = self.determine_path(ocx, ocy, ocyaw)

        cells = min(len(cx), len(cy), len(cyaw))
        target_path = Path2D()
        
        viz_path = Path()
        viz_path.header.frame_id = "map"
        viz_path.header.stamp = rospy.Time.now()

        for n in range(0, cells):
            # Appending to Target Path
            npose = Pose2D()
            npose.x = cx[n]
            npose.y = cy[n]
            npose.theta = cyaw[n]
            target_path.poses.append(npose)

            # Appending to Visualization Path
            vpose = PoseStamped()
            vpose.header.frame_id = self.frame_id
            vpose.header.seq = n
            vpose.header.stamp = rospy.Time.now()
            vpose.pose.position.x = cx[n]
            vpose.pose.position.y = cy[n]
            vpose.pose.position.z = 0.0
            vpose.pose.orientation = heading_to_quaternion(np.pi * 0.5 - cyaw[n])
            viz_path.poses.append(vpose)

        self.path_viz_pub.publish(viz_path)

    def create_target_path(self):
        '''
        Default path draw across waypoints
        '''
        ocx, ocy, ocyaw = calc_spline_course(self.ax, self.ay, self.ds)

def main():
    ''' 
    Main function to initialise the class and node. 
    '''
    # Initialise the class
    local_planner = LocalPathPlanner()

    # Initialise the node
    rospy.init_node('local_planner')

    # Set update rate
    r = rospy.Rate(local_planner.frequency) 

    # Wait for messages
    rospy.wait_for_message('/ngeeann_av/goals', Path2D)
    rospy.wait_for_message('/map', OccupancyGrid)

    while not rospy.is_shutdown():
        try:
            target_path = local_planner.create_target_path()
            local_planner.target_vel_pub.publish(local_planner.target_vel)

            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()