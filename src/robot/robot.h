#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <math.h>
#include "../navigation/navigation.h"
// #include "../miscellaneous/miscellaneous.h"

// Forward declaration
struct Point;

struct Pose
{
    double position_x; // Position wrt. global coordinate system in x direction, in meters
    double position_y;
    
    double orientation_z; // Orientation with respect to z axis, in radians, zero in +ve X

    Pose(){}

    Pose(double x, double y, double ang_z){
        position_x = x;
        position_y = y;
        orientation_z = ang_z;
    }
};

class Robot
{
public:

    Pose pose;

    double dimension_x = 0.5;
    double dimension_y = 0.5;

    // Helper for robot movement simulation
    // Should be in main loop, but here to avoid unnecessary function arguments
    double control_freq = 100; 
    double control_time_step = (1.0 / this->control_freq);

    // Helper, finding robot's four corners, see update_corner_points() for details
    std::vector<std::vector<int>> permutations = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
    std::vector<double> front_vector = {dimension_x/2.0, 0};
    std::vector<double> side_vector = {0, dimension_y/2.0};

    std::vector<Point> corner_points;

    Robot();
    void move(double cmd_lin_velo, double cmd_rot_velo);
    void simulate_pose(double cmd_lin_velo, double cmd_rot_velo);
    void update_corner_points();

};

#endif