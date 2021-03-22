#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <iostream>
#include <vector>
#include <math.h>
#include "../robot/robot.h"
#include "../miscellaneous/miscellaneous.h"

// Forward declarations
class Robot;
struct Pose;

struct Point
{
    double x, y;
    
    Point() {}

    Point(double x_, double y_){
        x = x_;
        y = y_;
    }
};

class Navigation
{
    std::vector<Point> path;
    int path_size = 0;
    int num_current_point = 0; // Current position on path vector
    
public:

    bool reached_goal=false;
    Point current_path_point;
    double DISTANCE_THRESHOLD = 0.001; // Threshold to decide whether the point is reached or not
    std::vector<double> velo_commands = {0.0, 0.0}; // {linear velocity, angular velocity}
    double k_p_linear = 0.2;
    double k_p_angular = 2.6; //2.2
    double k_static_linear = 0.0005;
    double k_static_angular = 0.0001;

    Navigation(std::vector<std::vector<double>>& input_path);
    void add_path(std::vector<std::vector<double>>& input_path);
    void print_path();
    bool forward_path_one_point();
    double distance_to_point(Pose &robot_pose, Point &path_point);
    double angle_to_point(Pose &robot_pose, Point &path_point);
    double calc_linear_velo(double distance_to_target);
    double calc_angular_velo(Pose& robot_pose, double target_orientation);
    bool calc_commands(Robot& robot, std::vector<double>& commands);
    
    int get_path_size();
    int get_num_current_point();

};

#endif