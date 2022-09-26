#!/usr/bin/env python3

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float64

from autocar_msgs.msg import Path2D, State2D
from autocar_nav import normalise_angle, yaw_to_quaternion


class PathTracker(Node):

    def __init__(self):

        super().__init__('path_tracker')

        # Initialise publishers
        self.tracker_pub = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
        self.lateral_ref_pub = self.create_publisher(PoseStamped, '/autocar/lateral_ref', 10)

        # Initialise subscribers
        self.localisation_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)
        self.path_sub = self.create_subscription(Path2D, '/autocar/path', self.path_cb, 10)
        self.target_vel_sub = self.create_subscription(Float64, '/autocar/target_velocity', self.target_vel_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None),
                    ('control_gain', None),
                    ('softening_gain', None),
                    ('yawrate_gain', None),
                    ('steering_limits', None),
                    ('centreofgravity_to_frontaxle', None)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)
            self.k = float(self.get_parameter("control_gain").value)
            self.ksoft = float(self.get_parameter("softening_gain").value)
            self.kyaw = float(self.get_parameter("yawrate_gain").value)
            self.max_steer = float(self.get_parameter("steering_limits").value)
            self.cg2frontaxle = float(self.get_parameter("centreofgravity_to_frontaxle").value)
        
        except ValueError:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class variables to use whenever within the class when necessary
        self.x = None
        self.y = None
        self.yaw = None
        self.target_vel = 0.0

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.target_idx = None
        self.heading_error = 0.0
        self.crosstrack_error = 0.0

        self.lock = threading.Lock()
        self.dt = 1 / self.frequency

        # Intialise timers
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def timer_cb(self):

        self.stanley_control()

    def vehicle_state_cb(self, msg):

        self.lock.acquire()
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta
        self.vel = np.sqrt((msg.twist.x**2.0) + (msg.twist.y**2.0))
        self.yawrate = msg.twist.w

        if self.cyaw:
            self.target_index_calculator()

        self.lock.release()

    def path_cb(self, msg):

        self.lock.acquire()
        self.cx = []
        self.cy = []
        self.cyaw = []

        for i in range(0, len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            ptheta = msg.poses[i].theta
            self.cx.append(px)
            self.cy.append(py)
            self.cyaw.append(ptheta) 
        self.lock.release()

    def target_vel_cb(self, msg):

        self.target_vel = msg.data

    def target_index_calculator(self):  

        ''' Calculates the target index and each corresponding error '''

        # Calculate position of the front axle
        fx = self.x + self.cg2frontaxle * -np.sin(self.yaw)
        fy = self.y + self.cg2frontaxle * np.cos(self.yaw)

        dx = [fx - icx for icx in self.cx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.cy] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        target_idx = np.argmin(d) # Find the shortest distance in the array

        # Cross track error, project RMS error onto the front axle vector
        front_axle_vec = [-np.cos(self.yaw + np.pi), -np.sin(self.yaw + np.pi)]
        self.crosstrack_error = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        # Heading error
        self.heading_error = normalise_angle(self.cyaw[target_idx] - self.yaw - np.pi * 0.5)
        self.target_idx = target_idx
    
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.cx[target_idx]
        pose.pose.position.y = self.cy[target_idx]
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(self.cyaw[target_idx])
        self.lateral_ref_pub.publish(pose)

    # Stanley controller determines the appropriate steering angle
    def stanley_control(self):

        self.lock.acquire()
        crosstrack_term = np.arctan2((self.k * self.crosstrack_error), (self.ksoft + self.target_vel))
        heading_term = normalise_angle(self.heading_error)
        
        sigma_t = crosstrack_term + heading_term

        # Constrains steering angle to the vehicle limits
        if sigma_t >= self.max_steer:
            sigma_t = self.max_steer

        elif sigma_t <= -self.max_steer:
            sigma_t = -self.max_steer

        self.set_vehicle_command(self.target_vel, sigma_t)
        self.lock.release()

    # Publishes to vehicle state
    def set_vehicle_command(self, velocity, steering_angle):

        ''' Publishes the calculated steering angle  '''
        
        drive = Twist()
        drive.linear.x = velocity
        drive.linear.y = 0.0
        drive.linear.z = 0.0

        drive.angular.x = 0.0
        drive.angular.y = 0.0
        drive.angular.z = steering_angle
        
        self.tracker_pub.publish(drive)

def main(args=None):
    
    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        path_tracker = PathTracker()

        # Stop the node from exiting
        rclpy.spin(path_tracker)

    finally:
        path_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()