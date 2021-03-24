#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/buffer.h"
#include <tf2/impl/utils.h>
#include <string.h>
#include <eigen3/Eigen/Core>
#include <stdio.h>

using std::placeholders::_1;
using namespace std;

class LocalPlanner : public rclcpp::Node
{
    public:
    LocalPlanner()
    : Node("bof")
    {
    }
   
    private:
    rclcpp::Time now;

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlanner>());
    rclcpp::shutdown();
    return 0;
}

