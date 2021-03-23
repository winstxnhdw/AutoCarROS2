#include <memory>
#include <chrono>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <eigen3/Eigen/Core>
#include <math.h>
#include <stdio.h>

using std::placeholders::_1;
using namespace std;



class GridMap
{
private:
    double idx;
    double origin_x, origin_y, cell_size;
    int size_x, size_y;

public:
    Eigen::MatrixXd occ_grid;
    GridMap(double origin_x, double origin_y, double cell_size, int size_x, int size_y);
    void toRosOccMap(nav_msgs::msg::OccupancyGrid& ros_occ_grid);
    void setGridOcc(double x, double y);
    void setGridFree(double x, double y);
    float getCellSize(){
        return cell_size;
    }
};

GridMap::GridMap(double origin_x_, double origin_y_, double cell_size_, int size_x_, int size_y_)
{
    origin_x = origin_x_;
    origin_y = origin_y_;
    cell_size = cell_size_;
    size_x = size_x_;
    size_y = size_y_;

    occ_grid.resize(size_x_, size_y_);
    occ_grid.setOnes() *= 0.5;
}

void GridMap::toRosOccMap(nav_msgs::msg::OccupancyGrid& ros_occ_grid)
{
    ros_occ_grid.header.frame_id = "odom";
    ros_occ_grid.header.frame_id = 
    ros_occ_grid.info.width = size_x;
    ros_occ_grid.info.height = size_y;
    ros_occ_grid.info.resolution = cell_size;
    ros_occ_grid.info.origin.position.x = -origin_x*cell_size;
    ros_occ_grid.info.origin.position.y = -origin_y*cell_size;
    
    int N = size_x * size_y;
    for(int i = 0; i < N; i++)
    {

        ros_occ_grid.data.push_back(50.0);

        // double& value = occ_grid.data()[i];
        // if(value == 0.5)
        //     ros_occ_grid.data.push_back( -1);
        // else
        //     ros_occ_grid.data.push_back( value * 100);
    }
}

void GridMap::setGridOcc(double x, double y)
{
    int idx = floor( ( x / cell_size ) + origin_x );
    int idy = floor( ( y / cell_size ) + origin_y );

    if ( (idx < size_x) && (idy < size_y))
    {
        occ_grid(idx, idy) = occ_grid(idx, idy) + 0.1;
        if ( occ_grid(idx, idy) > 1.0 )
            occ_grid(idx, idy) = 1.0;
    }    

}

void GridMap::setGridFree(double x, double y)
{
    int idx = floor( ( x / cell_size ) + origin_x );
    int idy = floor( ( y / cell_size ) + origin_y );
    occ_grid(idx, idy) = occ_grid(idx, idy) - 0.1;

    if ( occ_grid(idx, idy) < 0.0 )
        occ_grid(idx, idy) = 0.0;
}