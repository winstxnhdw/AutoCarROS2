#!/usr/bin/env python

import threading
import rospy
import numpy as np
import numpy.ma as ma
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from ngeeann_av_msgs.msg import Path2D, State2D
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan

class Map:

    def __init__(self, origin_x=0, origin_y=0, resolution=0.2, width=650, height=650):
        ''' 
        Constructs an empty occupancy grid upon initialization
        '''
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height 
        self.grid = np.zeros((height, width))

        # Creates occupied roadmap
        self.roadmap = np.zeros((height, width))

        # Lane Overrun Region
        for r in np.arange(97, 100, 0.05):
            for theta in np.arange(0, 0.5 * np.pi, 0.001):
                x = r * np.cos(theta)
                y = r * np.sin(theta)

                try:
                    ix = int((x - self.origin_x) / self.resolution)
                    iy = int((y - self.origin_y) / self.resolution)
                    self.roadmap[iy, ix] = 0.4

                except:
                    pass

        for r in np.arange(107.5, 110.5, 0.05):
            for theta in np.arange(0, 0.5*np.pi, 0.001):
                x = r * np.cos(theta)
                y = r * np.sin(theta)

                try:
                    ix = int((x - self.origin_x) / self.resolution)
                    iy = int((y - self.origin_y) / self.resolution)
                    self.roadmap[iy, ix] = 0.4

                except:
                    pass

        # Add barriers
        for r in np.arange(110.5, 110.75, 0.05):
            for theta in np.arange(0, 0.5*np.pi, 0.001):
                x = r * np.cos(theta)
                y = r * np.sin(theta)

                try:
                    ix = int((x - self.origin_x) / self.resolution)
                    iy = int((y - self.origin_y) / self.resolution)
                    self.roadmap[iy, ix] = 0.8

                except:
                    pass

        for r in np.arange(96.75, 97, 0.05):
            for theta in np.arange(0, 0.5*np.pi, 0.001):
                x = r * np.cos(theta)
                y = r * np.sin(theta)

                try:
                    ix = int((x - self.origin_x) / self.resolution)
                    iy = int((y - self.origin_y) / self.resolution)
                    self.roadmap[iy, ix] = 0.8

                except:
                    pass

        print('Road map initialised.')
        
        self.mask = self.roadmap
    
    def to_message(self):
        '''
        Returns nav_msgs/OccupancyGrid representation of the map
        '''
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        
        self.mask = np.clip((self.roadmap + self.grid), 0, 1)
        flat_grid = self.mask.reshape((self.grid.size,)) * 100
        grid_msg.data = list(np.round(flat_grid))
        return grid_msg

    def set_cell(self, x, y, val):
        '''
        Set the value of a cell in the grid. 

        Arguments: 
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).
        '''
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)

        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            pass    # indicates map too small

        else:
            self.grid[iy, ix] = self.grid[iy, ix] + val
            self.grid[iy, ix] = np.clip(self.grid[iy, ix], 0, 1)

class GridMapping(object):
    
    def __init__(self):

        self.lock = threading.Lock()
        self.scan = None
        self.cg2lidar = 2.34
        self.x = None
        self.y = None
        self.yaw = None

        self.gmap = Map()
        
        # Initialise publishers
        self.viz_map_pub = rospy.Publisher('/map', OccupancyGrid, latch=True, queue_size=30)

        # Initialise subscribers
        rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb)
        rospy.Subscriber('/laser/scan', LaserScan, self.scan_cb)

    def publish_map(self, gmap):
        '''
        Publishes map 
        '''
        msg = gmap.to_message()
        self.viz_map_pub.publish(msg)
        print('Sent Map')

    def scan_cb(self, data):

        self.scan = data

    def vehicle_state_cb(self, data):

        # Fill gridmap
        #self.lock.acquire()
        self.x = data.pose.x
        self.y = data.pose.y
        self.vel = np.sqrt(data.twist.x**2 + data.twist.y**2)
        self.yaw = data.pose.theta
        #self.lock.release()

    def raycasting(self):

        # Lidar Properties
        angle_min = self.scan.angle_min
        angle_max = self.scan.angle_max
        range_min = self.scan.range_min
        range_max = self.scan.range_max #self.scan.range_max
        angle_increment = self.scan.angle_increment

        print('Distance forwards = {}'.format(self.scan.ranges[360]))
        
        for i in range(0, len(self.scan.ranges)):
            theta = i * angle_increment

            # Draws an individual line representitive of a single line of measurement within the LIDAR
            # If the distance is greater than the range measured in this direction, the cell is assumed occupied
            # If the distance is lower than the range measured, the cell is considered empty

            if (theta < np.pi * 0.25) or (theta > np.pi * 0.75):
                range_max = 10
            else:
                range_max = self.scan.range_max

            look_range = min(range_max, self.scan.ranges[i])


            for d in np.arange(range_min, look_range + self.gmap.resolution, self.gmap.resolution):

                # Determines position of detected point in vehicle frame
                point_x = d*np.cos(theta)  
                point_y = d*np.sin(theta)
                transform = self.frame_transform(point_x, point_y)

                try: 
                    if (d < self.scan.ranges[i]):
                        self.gmap.set_cell(transform[0], transform[1], -0.5)
                    else:
                        self.gmap.set_cell(transform[0], transform[1], 0.5)
                        break
                except:
                    pass

        self.publish_map(self.gmap)

    def inverse_range_sensor_model(self):

        # Lidar Properties
        angle_min = self.scan.angle_min
        angle_max = self.scan.angle_max
        range_min = self.scan.range_min
        range_max = self.scan.range_max
        angle_increment = self.scan.angle_increment

        print('Distance forwards = {}'.format(self.scan.ranges[360]))
        
        for i in range(0, len(self.scan.ranges)):
            
            theta = i * angle_increment

            # Only accepts data within the valid range of the lidar
            if (self.scan.ranges[i] > range_min) and (self.scan.ranges[i] < range_max):
            
                # Determines position of detected point in vehicle frame
                point_x = self.scan.ranges[i]*np.cos(theta)  
                point_y = self.scan.ranges[i]*np.sin(theta)

                # Determines point to be updated in global frame
                transform = self.frame_transform(point_x, point_y)
                self.gmap.set_cell(transform[0], transform[1], 0.5)

        self.publish_map(self.gmap)
        
    def frame_transform(self, point_x, point_y):
        ''' 
            Recieves position of a point in the vehicle frame, and the position and orientation of the
            vehicle in the global frame. Returns position of the point in global frame

            Arguments:
                point_x, point_y   - Coordinates (x,y) of point in the vehicle frame
                self.x, self.y     - Coordinates (x,y) of vehicle centre of gravity in the world frame
                self.yaw           - Yaw angle of the vehicle respect to global frame
                self.cg2lidar      - Distance between the centre of gravity of the vehicle and lidar module
        '''
        # Lidar position in world frame
        lidar_x = self.x + self.cg2lidar * -np.sin(self.yaw)
        lidar_y = self.y + self.cg2lidar * np.cos(self.yaw)
        lidar_vect = np.array(((lidar_x), (lidar_y)))

        # Creates rotation matrix given theta
        c = np.cos(self.yaw)
        s = np.sin(self.yaw)
        R = np.array(((c, -s), (s, c)))  

        # Vector of point in vehicle frame
        vp = np.array(((point_x), (point_y)))

        rotate = R.dot(vp)                  # rotation to allign with global frame
        transform = rotate + lidar_vect     # translates point to global frame

        return transform 

def main():
    '''
        The main function.
    '''
    gridmapping = GridMapping()

    rospy.init_node("bof")

    r = rospy.Rate(10)

    rospy.wait_for_message('/laser/scan', LaserScan)

    while not rospy.is_shutdown():
        try:
            gridmapping.inverse_range_sensor_model()
            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__ == "__main__":
    main()
