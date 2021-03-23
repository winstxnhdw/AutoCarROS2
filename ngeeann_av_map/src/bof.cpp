#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/buffer.h"
#include <tf2/impl/utils.h>
#include <string.h>
#include <eigen3/Eigen/Core>
#include <stdio.h>
#include "gridmap.h"

using std::placeholders::_1;
using namespace std;
GridMap gmap(-100.0, -100.0, 0.2, 200, 200);

class OccupancyMapping : public rclcpp::Node
{
    public:
    OccupancyMapping()
    : Node("bof")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "ngeeann_av/odom", 20, std::bind(&OccupancyMapping::odom_callback, this, _1));
        subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&OccupancyMapping::scanCallback, this, _1));
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::SensorDataQoS());
        publisher2_ = this->create_publisher<sensor_msgs::msg::LaserScan>("ngeeann_av/scan", rclcpp::SensorDataQoS());
    }
   
    private:
    rclcpp::Time now;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double cg2lidar = 1.5;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher2_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;

        tf2::Quaternion orientation;
        orientation.setX( msg->pose.pose.orientation.x );
        orientation.setY( msg->pose.pose.orientation.y );
        orientation.setZ( msg->pose.pose.orientation.z );
        orientation.setW( msg->pose.pose.orientation.w );
        theta = tf2::impl::getYaw( orientation );
    }


    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        sensor_msgs::msg::LaserScan laser;
        laser.header.frame_id = "laser_frame";
        laser.header.stamp = now;
        laser.angle_min = msg->angle_min;
        laser.angle_max = msg->angle_max;
        laser.angle_increment = msg->angle_increment;
        laser.scan_time = msg->scan_time;
        laser.time_increment = msg->time_increment;
        laser.range_min = msg->range_min;
        laser.range_max = msg->range_max;
        laser.ranges = msg->ranges;
        laser.intensities = msg->intensities;
        printf("\nrecieved scan");
        publisher2_->publish(laser);

        updateMap(msg);
        nav_msgs::msg::OccupancyGrid occ_map;
        occ_map.header.frame_id = "odom";
        occ_map.info.width = 0.0;
    }

    void updateMap( sensor_msgs::msg::LaserScan::SharedPtr scan )
    {
        const double angle_inc = scan->angle_increment;
        const double range_max = scan->range_max;
        const double range_min = scan->range_min;

        for(size_t i = 0; i < scan->ranges.size(); i ++){
            
            double R = scan->ranges.at(i);
            if (R < range_min || R > range_max)
                continue;

            // Location of lidar in global frame
            double lidar_x = x + cg2lidar * -sin(theta);
            double lidar_y = y + cg2lidar * cos(theta);

            // Location of point in laser frame
            double angle = i*angle_inc;
            double pxl = R*cos(angle);
            double pyl = R*sin(angle);

            // Determines location of point in global frame
            double px = pxl*cos(theta) - pyl*sin(theta);
            double py = pxl*sin(theta) - pyl*cos(theta);
            px += lidar_x;
            py += lidar_y;

            gmap.setGridOcc(px, py);
        }
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyMapping>());
    rclcpp::shutdown();
    return 0;
}

