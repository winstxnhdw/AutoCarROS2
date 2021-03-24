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
    float cell_size;
    float origin_pose_x, origin_pose_y;
    int size_x, size_y, origin_x, origin_y;
    Eigen::MatrixXd occ_grid;

public:
    GridMap(double origin_x, double origin_y, float cell_size, int size_x, int size_y);
    void toRosOccMap(nav_msgs::msg::OccupancyGrid& ros_occ_grid);
    void setGridOcc(double x, double y);
    void setGridFree(double x, double y);
    float getCellSize(){ return cell_size; }
    void setRepositionMap(int idx, int idy);
};

GridMap::GridMap(double origin_x_, double origin_y_, float cell_size_, int size_x_, int size_y_)
{
    origin_x = origin_x_;
    origin_y = origin_y_;
    cell_size = cell_size_;
    origin_pose_x = origin_x*cell_size;
    origin_pose_y = origin_y*cell_size;
    size_x = size_x_;
    size_y = size_y_;
    occ_grid.resize(size_x_, size_y_);
    occ_grid.setOnes() *= 0.5;
}

void GridMap::toRosOccMap(nav_msgs::msg::OccupancyGrid& ros_occ_grid)
{
    ros_occ_grid.header.frame_id = "odom";
    ros_occ_grid.info.width = size_x;
    ros_occ_grid.info.height = size_y;
    ros_occ_grid.info.resolution = cell_size;
    ros_occ_grid.info.origin.position.x = origin_pose_x;
    ros_occ_grid.info.origin.position.y = origin_pose_y;
    
    int N = size_x * size_y;
    for(int i = 0; i < N; i++)
    {
        double& value = occ_grid.data()[i];
        ros_occ_grid.data.push_back( value * 100);
    }
}

// Increase probability that point is occupied
void GridMap::setGridOcc(double x, double y)
{
    int idx = floor( (x / cell_size ) - origin_x );
    int idy = floor( (y / cell_size ) - origin_y );

    if ( (idx < size_x) && (idy < size_y) && (0 <= idx) && (0 <= idy) )
    {
        occ_grid(idx, idy) = occ_grid(idx, idy) + 0.05;
        if ( occ_grid(idx, idy) > 1.0 )
            occ_grid(idx, idy) = 1.0;
    }
    else
        setRepositionMap(idx, idy);    
}


void GridMap::setGridFree(double x, double y)
{
    int idx = floor( (x / cell_size ) - origin_x );
    int idy = floor( (y / cell_size ) - origin_y );

    if ( (idx < size_x) && (idy < size_y) && (0 <= idx) && (0 <= idy) )
    {
        occ_grid(idx, idy) = occ_grid(idx, idy) - 0.05;
        if ( occ_grid(idx, idy) < 0.0 )
            occ_grid(idx, idy) = 0.0;
    }
    else
        setRepositionMap(idx, idy);    
}

void GridMap::setRepositionMap(int idx, int idy)
{
    Eigen::MatrixXd old_occ_grid = occ_grid.replicate(1,1);

    float boundary_thresh = 0.25;
    int shift_cells = round(size_y * boundary_thresh);

    if (idx > size_x)
    {
        origin_x = origin_x + shift_cells;
        origin_pose_x = origin_x*cell_size;
        occ_grid.topRows(size_x - shift_cells) = old_occ_grid.bottomRows(size_x - shift_cells);
        occ_grid.bottomRows(shift_cells).setOnes() *= 0.5;
    }
    if (idx < 0)
    {
        origin_x = origin_x - shift_cells;
        origin_pose_x = origin_x*cell_size;
        occ_grid.bottomRows(size_x - shift_cells) = old_occ_grid.topRows(size_x - shift_cells);
        occ_grid.topRows(shift_cells).setOnes() *= 0.5;
    }

    if (idy > size_y)
    {
        origin_y = origin_y + shift_cells;
        origin_pose_y = origin_y*cell_size;
        occ_grid.leftCols(size_y - shift_cells) = old_occ_grid.rightCols(size_y - shift_cells);
        occ_grid.rightCols(shift_cells).setOnes() *= 0.5;
    }
    
    if (idy < 0)
    {
        origin_y = origin_y - shift_cells;
        origin_pose_y = origin_y*cell_size;
        occ_grid.rightCols(size_y - shift_cells) = old_occ_grid.leftCols(size_y - shift_cells);
        occ_grid.leftCols(shift_cells).setOnes() *= 0.5;
    }


}