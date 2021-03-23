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

using std::placeholders::_1;
using namespace std;


class OccupancyMapping : public rclcpp::Node
{
    public:
    OccupancyMapping()
    : Node("bof")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "ngeeann_av/odom", 20, std::bind(&OccupancyMapping::topic_callback, this, _1));
        subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>("ngeeann_av/scan", rclcpp::SensorDataQoS(), std::bind(&OccupancyMapping::scanCallback, this, _1));
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::SensorDataQoS());
        publisher2_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    }
   
    private:
    rclcpp::Time now;
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        tf2::Quaternion orientation;
        orientation.setX( msg->pose.pose.orientation.x );
        orientation.setY( msg->pose.pose.orientation.y );
        orientation.setZ( msg->pose.pose.orientation.z );
        orientation.setW( msg->pose.pose.orientation.w );
        double theta = tf2::impl::getYaw( orientation );
        printf("Heading: %f\n", theta);
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
        publisher2_->publish(laser);
        laser.intensities = msg->intensities ;
    }
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher2_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyMapping>());
    rclcpp::shutdown();
    return 0;
}