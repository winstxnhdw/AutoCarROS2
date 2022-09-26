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

GridMap gmap(-200, -200, 0.2, 500, 500);

class OccupancyMapping : public rclcpp::Node
{
    public:
    OccupancyMapping()
    : Node("bof")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("autocar/odom", 20, std::bind(&OccupancyMapping::odom_callback, this, _1));
        subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&OccupancyMapping::scanCallback, this, _1));
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::SensorDataQoS());
    }
   
    private:
    rclcpp::Time now;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double cg2lidar = 2.4;
    float lat_update_range = 10.0, long_update_range = 30.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;


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
        updateMap(msg);
        nav_msgs::msg::OccupancyGrid occ_map;
        gmap.toRosOccMap(occ_map);
        publisher_->publish(occ_map);
    }


    void updateMap( sensor_msgs::msg::LaserScan::SharedPtr scan )
    {
        const double angle_inc = scan->angle_increment;
        const double range_max = scan->range_max;
        const double range_min = scan->range_min;
        double R, lidar_x, lidar_y, px, py, pxl, pyl, angle;

        for(size_t i = 0; i < scan->ranges.size(); i ++){
            
            R = scan->ranges.at(i);
            if (R < range_min)
                continue;

            for(double r = 0.0; r < range_max; r += 0.2)
            {

                // Location of point in laser frame
                angle = i*angle_inc;
                pxl = r*cos(angle);
                pyl = r*sin(angle);

                if ( (abs(pxl) > lat_update_range || abs(pyl) > long_update_range) && (R >range_max) )
                    continue;

                // Location of lidar in global frame
                lidar_x = x + cg2lidar * -sin(theta);
                lidar_y = y + cg2lidar * cos(theta);

                // Determines location of point in global frame
                px = pxl*cos(theta) - pyl*sin(theta) + lidar_x;
                py = pxl*sin(theta) + pyl*cos(theta) + lidar_y;

                if ( (R < range_max) && (r > R) )
                {
                    gmap.setGridOcc(px, py);
                    break;
                }
                else
                    gmap.setGridFree(px, py);
            }
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

