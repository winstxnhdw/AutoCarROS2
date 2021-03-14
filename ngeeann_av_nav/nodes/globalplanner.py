#!/usr/bin/env python

import rospy
import numpy as np
import pandas as pd

from geometry_msgs.msg import Pose2D, Pose, PoseArray
from ngeeann_av_msgs.msg import Path2D, State2D

class GlobalPathPlanner:

    def __init__(self):

        ''' Class constructor to initialise the class '''

        # Initialise publisher(s)
        self.goals_pub = rospy.Publisher('/ngeeann_av/goals', Path2D, queue_size=10)
        self.goals_viz_pub = rospy.Publisher('/ngeeann_av/viz_goals', PoseArray, queue_size=10)

        # Initialise suscriber(s)
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=10)

        # Load parameters
        try:
            self.global_planner_params = rospy.get_param("/global_path_planner")
            self.frequency = self.global_planner_params["update_frequency"]
            self.wp_ahead = self.global_planner_params["waypoints_ahead"]
            self.wp_behind = self.global_planner_params["waypoints_behind"]
            self.passed_threshold = self.global_planner_params["passed_threshold"]

            self.tracker_params = rospy.get_param("/path_tracker")
            self.cg2frontaxle = self.tracker_params["centreofgravity_to_frontaxle"]

            dir_path = rospy.get_param("/waypoints")

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Get path to waypoints.csv
        
        df = pd.read_csv(dir_path)

        print("Waypoint directory: {}".format(dir_path))

        # Import waypoints.csv into class variables ax and ay
        self.ax = df['X-axis'].values.tolist()
        self.ay = df['Y-axis'].values.tolist()
        
        # Class constants
        self.waypoints = min(len(self.ax), len(self.ay))
        self.wp_published = self.wp_ahead + self.wp_behind
        
        # Class variables to use whenever within the class when necessary
        self.x = None
        self.y = None
        self.theta = None

    def vehicle_state_cb(self, msg):
        ''' 
            Callback function to update vehicle state 

            Parameters:
                self.x          - Represents the current x-coordinate of the vehicle
                self.y          - Represents the current y-coordinate of the vehicle
                self.theta      - Represents the current yaw of the vehicle
        '''

        self.x = msg.pose.x
        self.y = msg.pose.y
        self.theta = msg.pose.theta

    def set_waypoints(self):
        ''' 
            Determines the appropriate set of waypoints to publish by the following steps

            1. Identify waypoint closest to front axle
            2. Determines if this point is ahead or behind, by transformation
            3. Preserves fixed number of points ahead or behind

            Parameters:
                self.wp_ahead           - Indicates number of waypoints to look ahead
                self.wp_behind          - Indicates number of waypoints to preserve behind the vehicle
                self.wp_published       - Indicates the total number of waypoints published
                self.passed_threshold   - Indicates the distance after which a waypoint is considered passed
                self.waypoints          - Total number of waypoints
        '''

        # Identify position of vehicle front axle
        fx = self.x + self.cg2frontaxle * -np.sin(self.theta)
        fy = self.y + self.cg2frontaxle * np.cos(self.theta)

        dx = [fx - icx for icx in self.ax] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.ay] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy)        # Find the distance from the front axle to the path
        closest_id = np.argmin(d)   # Returns the index with the shortest distance in the array

        transform = self.frame_transform(self.ax[closest_id], self.ay[closest_id], fx, fy, self.theta)

        if closest_id < 2:
            # If the vehicle is starting along the path
            print('Closest Waypoint #{} (Starting Path)'.format(closest_id))
            px = self.ax[0: self.wp_published]
            py = self.ay[0: self.wp_published]

        elif closest_id > (self.waypoints - self.wp_published):
            # If the vehicle is finishing the given set of waypoints
            print('Closest Waypoint #{} (Terminating Path)'.format(closest_id))
            px = self.ax[-self.wp_published:]
            py = self.ay[-self.wp_published:]    

        elif transform[1] < (0.0 - self.passed_threshold):
            # If the vehicle has passed, closest point is preserved as a point behind the car
            print('Closest Waypoint #{} (Passed)'.format(closest_id))
            px = self.ax[closest_id - (self.wp_behind - 1) : closest_id + (self.wp_ahead + 1)]
            py = self.ay[closest_id - (self.wp_behind - 1) : closest_id + (self.wp_ahead + 1)]

        else:
            # If the vehicle has yet to pass, a point behind the closest is preserved as a point behind the car
            print('Closest Waypoint #{} (Approaching)'.format(closest_id))
            px = self.ax[(closest_id - self.wp_behind) : (closest_id + self.wp_ahead)]
            py = self.ay[(closest_id - self.wp_behind) : (closest_id + self.wp_ahead)]
        
        self.publish_goals(px, py)

    def start_end_condition(self, closest_id):

        ''' [NOT IN USE] Dictates the goals published when vehicle is near the start / end of the waypoints list '''

        if (closest_id < self.wp_behind):
            px = self.ax[0 : self.wp_ahead]
            py = self.ay[0 : self.wp_ahead]
            return px, py

        elif  (closest_id > self.waypoints - 1):
            px = self.ax[(self.waypoints - self.wp_ahead - 1) : self.waypoints]
            py = self.ay[(self.waypoints - self.wp_ahead - 1) : self.waypoints]
            return px, py

    def frame_transform(self, point_x, point_y, axle_x, axle_y, theta):
        ''' 
            Recieves position of vehicle front axle, and id of closest waypoint. This waypoint is transformed from
            "map" frame to the vehicle frame

            Arguments:
                closest_id          - Index to closest waypoint to front axle in master waypoint list
                point_x, point_y    - Coordinates (x,y) of target point in world frame
                axle_x, axle_y      - Coordinates (x,y) of vehicle front axle position in world frame
        '''

        c = np.cos(-theta)  # Creates rotation matrix given theta
        s = np.sin(-theta)  # Creates rotation matrix given theta
        R = np.array(((c, -s), (s, c)))  

        p = np.array(((point_x), (point_y)))      # Position vector of closest waypoint (world frame)
        v = np.array(((axle_x), (axle_y)))        # Position vector of vehicle (world frame)
        vp = p - v                                # Linear translation between vehicle and point     
        transform = R.dot(vp)                     # Product of rotation matrix and translation vector

        return transform

    def publish_goals(self, px, py):

        ''' Publishes an array of waypoints for the Local Path Planner '''

        waypoints = min(len(px), len(py))
        goals = Path2D()

        viz_goals = PoseArray()
        viz_goals.header.frame_id = "map"
        viz_goals.header.stamp = rospy.Time.now()

        for i in range(0, waypoints):
            # Appending to Target Goals
            goal = Pose2D()
            goal.x = px[i]
            goal.y = py[i]
            goals.poses.append(goal)
            
            # Appending to Visualization Path
            vpose = Pose()
            vpose.position.x = px[i]
            vpose.position.y = py[i]
            vpose.position.z = 0.0
            vpose.orientation
            viz_goals.poses.append(vpose)
        
        self.goals_pub.publish(goals)
        self.goals_viz_pub.publish(viz_goals)

        print("Total goals published: {}\n".format(waypoints))
        
def main():
    
    # Initialise the class
    global_planner = GlobalPathPlanner()

    # Initialise the node
    rospy.init_node('global_planner')

    rospy.wait_for_message('/ngeeann_av/state2D', State2D)

    # Set update rate
    r = rospy.Rate(global_planner.frequency)

    while not rospy.is_shutdown():
        try:
            global_planner.set_waypoints()
            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()