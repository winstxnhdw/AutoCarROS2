#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D, Point
from ngeeann_av_msgs.msg import Path2D
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from ngeeann_av_nav.cubic_spline_interpolator import generate_cubic_path

class ClickPlanner(Node):

    def __init__(self):
        ''' 
        Class constructor to initialise the class 
        '''

        super().__init__('click_planner')

        # Initialise publishers
        self.local_planner_pub = self.create_publisher(Path2D, '/ngeeann_av/path', 10)
        self.path_viz_pub = self.create_publisher(Path, '/ngeeann_av/viz_path', 10)
        self.goals_viz_pub = self.create_publisher(Marker, '/ngeeann_av/viz_goals', 10)
        self.target_vel_pub = self.create_publisher(Float64, '/ngeeann_av/target_velocity', 10)

        # Initialise subscribers
        self.goals_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goals_cb, 10)

        self.ax = []
        self.ay = []
        self.ds = 0.1
        self.goals = 0


    def goals_cb(self, msg):

        self.goals += 1
        self.ax.append(msg.pose.position.x)
        self.ay.append(msg.pose.position.y)
        self.display_waypoint(msg.pose.position.x, msg.pose.position.y)

        if (self.goals >= 2):
            self.create_display_path()

        vel = Float64()
        vel.data = 4.0
        self.target_vel_pub.publish(vel)


    def display_waypoint(self,x,y):

        points = Marker()		
        points.header.frame_id = "odom"	# Publish path in map frame		
        points.type = points.POINTS
        points.action = points.ADD

        life = Duration()
        life.sec = 1000
        life.nanosec = 0

        points.lifetime = life
        points.id = self.goals
        points.scale.x = 0.1
        points.scale.y = 0.1	
        points.color.a = 1.0
        points.color.r = 0.0
        points.color.g = 0.0
        points.color.b = 1.0
        points.pose.orientation.w = 1.0

        point = Point()
        point.x = x
        point.y = y
        
        points.points.append(point)

        # Publish the MarkerArray
        self.goals_viz_pub.publish(points)

    def create_display_path(self):

        cx, cy, cyaw, _ = generate_cubic_path(self.ax, self.ay, self.ds)

        target_path = Path2D()
        viz_path = Path()
        viz_path.header.frame_id = "odom"
        viz_path.header.stamp = self.get_clock().now().to_msg()

        for n in range(0, len(cyaw)):

            # Appending to Target Path
            npose = Pose2D()
            npose.x = cx[n]
            npose.y = cy[n]
            npose.theta = cyaw[n]
            target_path.poses.append(npose)

            # Appending to Visualization Path
            vpose = PoseStamped()
            vpose.header.frame_id = "odom"
            vpose.header.stamp = self.get_clock().now().to_msg()
            vpose.pose.position.x = cx[n]
            vpose.pose.position.y = cy[n]
            vpose.pose.position.z = 0.0
            vpose.pose.orientation.w = 1.0
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
        click_planner = ClickPlanner()

        # Stop the node from exiting
        rclpy.spin(click_planner)

    finally:
        click_planner.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()