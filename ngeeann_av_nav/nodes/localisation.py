#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from gazebo_msgs.srv import GetModelState  
from nav_msgs.msg import Odometry
from ngeeann_av_msgs.msg import State2D

class Localisation(Node):

    def __init__(self):

        super().__init__('localisation')

        # Initialise publishers
        self.localisation_pub = self.create_publisher(State2D, '/ngeeann_av/state2D', 20)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.vehicle_state_cb, 20)

        # Class constants
        self.state = None

    def vehicle_state_cb(self, msg):
        self.state = msg
        self.update_state()

    # Gets vehicle position from Gazebo and publishes data
    def update_state(self):

        # Define vehicle pose x,y, theta
        state2d = State2D()
        state2d.pose.x = self.state.pose.pose.position.x
        state2d.pose.y = self.state.pose.pose.position.y
        state2d.pose.theta = 2.0 * np.arctan2(self.state.pose.pose.orientation.z, self.state.pose.pose.orientation.w)
        
        # Aligning heading to y-axis, accounts for double rotation error
        if state2d.pose.theta < 0.0:
            state2d.pose.theta += 2.0 * np.pi
        
        # Define linear velocity x,y and angular velocity w
        state2d.twist.x = self.state.twist.twist.linear.x
        state2d.twist.y = self.state.twist.twist.linear.y
        state2d.twist.w = -self.state.twist.twist.angular.z

        self.localisation_pub.publish(state2d)

        # Print state
        print("Position (x,y): ({},{})".format(round(state2d.pose.x, 5), round(state2d.pose.y, 5)))
        print("Heading: {}".format(round(state2d.pose.theta, 5)))
        print("Velocity (x,y): ({},{})".format(round(state2d.twist.x, 5), round(state2d.twist.y, 5)))

def main(args=None):

    # Initialise the node
    rclpy.init(args=args)
    
    # Initialise the class
    localisation = Localisation()

    while rclpy.ok():
        try:
            rclpy.spin(localisation)

        except KeyboardInterrupt:
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()