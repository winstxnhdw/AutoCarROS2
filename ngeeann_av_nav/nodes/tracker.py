#!/usr/bin/env python3

import rclpy
import datetime
import threading
import numpy as np

from rclpy.node import Node
from ngeeann_av_msgs.msg import State2D, Path2D, AckermannDrive
from geometry_msgs.msg import Pose2D, PoseStamped, Quaternion
from std_msgs.msg import Float32
from normalise_angle import normalise_angle
from heading2quaternion import heading_to_quaternion

class PathTracker(Node):

    def __init__(self):

        super().__init__('path_tracker')

        # Initialise publishers
        self.tracker_pub = self.create_publisher(AckermannDrive, '/ngeeann_av/ackermann_cmd', 10)
        self.lateral_ref_pub = self.create_publisher(PoseStamped, '/ngeeann_av/lateral_ref', 10)

        # Initialise subscribers
        self.localisation_sub = self.create_subscription(State2D, '/ngeeann_av/state2D', self.vehicle_state_cb, 10)
        self.path_sub = self.create_subscription(Path2D, '/ngeeann_av/path', self.path_cb, 10)
        self.target_vel_sub = self.create_subscription(Float32, '/ngeeann_av/target_velocity', self.target_vel_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('control_gain', None),
                    ('softening_gain', None),
                    ('yawrate_gain', None),
                    ('steering_limits', None),
                    ('centreofgravity_to_frontaxle', None)
                ]
            )

            self.k = self.get_parameter("control_gain")
            self.ksoft = self.get_parameter("softening_gain")
            self.kyaw = self.get_parameter("yawrate_gain")
            self.max_steer = self.get_parameter("steering_limits")
            self.cg2frontaxle = self.get_parameter("centreofgravity_to_frontaxle")
        
        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class variables to use whenever within the class when necessary
        self.x = None
        self.y = None
        self.yaw = None
        self.target_vel = 0.0

        self.points = 1
        self.lock = threading.Lock()

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.target_idx = None
        self.heading_error = 0.0
        self.crosstrack_error = 0.0
        self.yawrate_error = 0.0

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

        # Yaw rate discrepancy
        try:
            self.yawrate_error = self.trajectory_yawrate_calc() - self.yawrate

        except:
            self.yawrate_error = 0.0
    
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.cx[target_idx]
        pose.pose.position.y = self.cy[target_idx]
        pose.pose.position.z = 0.0
        pose.pose.orientation = heading_to_quaternion(self.cyaw[target_idx])
        self.lateral_ref_pub.publish(pose)
    
    # Calculates the desired yawrate of the vehicle
    def trajectory_yawrate_calc(self):

        target_range = 2    #number of points to look ahead and behind
        delta_theta = 0.0
        delta_s = 0.0
        w = 0.0

        start = self.target_idx - target_range
        end = self.target_idx + target_range

        for n in range(start, end + 1):
            if (n >= 0) and ((n + 1) < len(self.cyaw)):

                x1 = self.cx[n]
                y1 = self.cy[n]
                x2 = self.cx[n+1]
                y2 = self.cy[n+1]

                delta_s += np.hypot(x2 - x1, y2 - y1)
                delta_theta += self.cyaw[n + 1] - self.cyaw[n]

            # Angular velocity calculation
            w = -(delta_theta / delta_s) * self.vel

        return w

    # Stanley controller determines the appropriate steering angle
    def stanley_control(self):

        self.lock.acquire()
        crosstrack_term = np.arctan2((self.k * self.crosstrack_error), (self.ksoft + self.target_vel))
        heading_term = normalise_angle(self.heading_error)
        yawrate_term = 0.0
        #yawrate_term = -self.kyaw * self.yawrate_error
        
        sigma_t = crosstrack_term + heading_term + yawrate_term

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
        
        drive = AckermannDrive()
        drive.speed = velocity
        drive.acceleration = 1.0
        drive.steering_angle = steering_angle
        drive.steering_angle_velocity = 0.0
        self.tracker_pub.publish(drive)

def main(args=None):
    
    # Time execution
    begin_time = datetime.datetime.now()
    n = 0
    track_error = []

    # Initialise the node
    rclpy.init(args=args)

    # Initialise the class
    path_tracker = PathTracker()

    while rclpy.ok():
        try:
            if path_tracker.cyaw:
                path_tracker.stanley_control()

            rclpy.spin(path_tracker)

            if n == 100:
                print("\nCurrent Tracking Error: {} m".format(path_tracker.crosstrack_error))
                print("Point {} of {} in current path".format(path_tracker.target_idx, len(path_tracker.cyaw)))
                track_error.append(path_tracker.crosstrack_error)
                n = 0
                
            else:
                n += 1

        except KeyboardInterrupt:
            print("\n\nExecution time     : {}".format(datetime.datetime.now() - begin_time))
            print("Average track error  : {}".format(sum(track_error) / len(track_error)))
            print("Shutting down ROS node...")

if __name__ == "__main__":
    main()