#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float64

from autocar_msgs.msg import Path2D, State2D
from autocar_nav import generate_cubic_path, yaw_to_quaternion


class LocalPathPlanner(Node):

    def __init__(self):
        ''' 
        Class constructor to initialise the class 
        '''

        super().__init__('local_planner')

        # Initialise publishers
        self.local_planner_pub = self.create_publisher(Path2D, '/autocar/path', 10)
        self.path_viz_pub = self.create_publisher(Path, '/autocar/viz_path', 10)
        self.target_vel_pub = self.create_publisher(Float64, '/autocar/target_velocity', 10)

        # Initialise subscribers
        self.goals_sub = self.create_subscription(Path2D, '/autocar/goals', self.goals_cb, 10)
        self.localisation_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None),
                    ('frame_id', None),
                    ('car_width', None),
                    ('centreofgravity_to_frontaxle', None)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)
            self.frame_id = str(self.get_parameter("frame_id").value)
            self.car_width = float(self.get_parameter("car_width").value)
            self.cg2frontaxle = float(self.get_parameter("centreofgravity_to_frontaxle").value)

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.ds = 1 / self.frequency

        # Class variables to use whenever within the class when necessary
        self.target_vel = 3.0
        self.ax = []
        self.ay = []

        # For debug purposes, do not delete
        # self.ax = [103.67, 102.6610906864386, 99.65400001792553, 94.70725759380844, 87.91714612853669]
        # self.ay = [0, 14.428075376529984, 28.575324677548302, 42.16638778766821, 54.93673012305635]

        # Initialise timer
        self.timer = self.create_timer(self.ds, self.timer_cb)

    def timer_cb(self):

        msg = Float64()
        msg.data = self.target_vel
        self.target_vel_pub.publish(msg)

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

        self.publish_path()

    def vehicle_state_cb(self, msg):
        ''' 
        Callback function to recieve vehicle state information from localization in global frame
        '''
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta

    def publish_path(self):
        '''
        Default path draw across waypoints
        '''

        cx, cy, cyaw, _ = generate_cubic_path(self.ax, self.ay, self.ds)

        path_length = min(len(cx), len(cy), len(cyaw))

        target_path = Path2D()
        viz_path = Path()
        vpose = PoseStamped()

        viz_path.header.frame_id = "odom"
        viz_path.header.stamp = self.get_clock().now().to_msg()

        for n in range(0, path_length):
            # Appending to Target Path
            npose = Pose2D()
            npose.x = cx[n]
            npose.y = cy[n]
            npose.theta = cyaw[n]
            target_path.poses.append(npose)

            # Appending to Visualization Path
            vpose.header.frame_id = "odom"
            vpose.header.stamp = self.get_clock().now().to_msg()
            vpose.pose.position.x = cx[n]
            vpose.pose.position.y = cy[n]
            vpose.pose.position.z = 0.0
            vpose.pose.orientation = yaw_to_quaternion(np.pi * 0.5 - cyaw[n])
            viz_path.poses.append(vpose)

        self.local_planner_pub.publish(target_path)
        self.path_viz_pub.publish(viz_path)

def main(args=None):
    ''' 
    Main function to initialise the class and node. 
    '''

    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        local_planner = LocalPathPlanner()

        # Stop the node from exiting
        rclpy.spin(local_planner)

    finally:
        local_planner.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()