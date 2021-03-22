#include "navigation.h"

Navigation::Navigation(std::vector<std::vector<double>>& input_path)
    : current_path_point(Point(input_path[0][0], input_path[0][1]))
{
    this->add_path(input_path);
}

/// Append path
void Navigation::add_path(std::vector<std::vector<double>>& input_path)
{
    this->path_size += input_path.size();
    for(int i=0; i<input_path.size(); i++)
    {
        path.push_back(Point( {input_path[i][0], input_path[i][1]} ));
    }
}

void Navigation::print_path()
{
    for(int i=0; i<(this->path_size); i++)
    {
        std::cout << "x, y:  " << path[i].x << ", " << path[i].y << std::endl;
    }
}

/// Update the current_path_point to the next path point
bool Navigation::forward_path_one_point()
{
    if(this->num_current_point < (this->path_size-1)) // Check if it is the last point
    {
        this->num_current_point++;
        this->current_path_point = this->path[num_current_point];
        return true;
    }
    else
    {
        this->current_path_point = this->path[(this->path_size - 1)];
        this->reached_goal = true;
        return false;
    }
}

int Navigation::get_path_size()
{
    return this->path_size;
}

int Navigation::get_num_current_point()
{
    return this->num_current_point;
}

double Navigation::distance_to_point(Pose& robot_pose, Point& path_point)
{
    double squar_diff_x = pow((robot_pose.position_x - path_point.x), 2);
    double squar_diff_y = pow((robot_pose.position_y - path_point.y), 2);
    return pow((squar_diff_x + squar_diff_y), 0.5);
}

double Navigation::angle_to_point(Pose& robot_pose, Point& path_point)
{
    double x_diff = (path_point.x - robot_pose.position_x);
    double y_diff = (path_point.y - robot_pose.position_y);
    double angle_to_point = atan2(y_diff, x_diff);
    return angle_to_point;
}

double Navigation::calc_linear_velo(double distance_to_target)
{
    double cmd_linear_velo;
    if(distance_to_target>0.001) // Some threshold to remove constant movement
    {
        cmd_linear_velo = distance_to_target * this->k_p_linear + 0.01;
    }
    else { cmd_linear_velo = 0.0; }
    return cmd_linear_velo;
}

double Navigation::calc_angular_velo(Pose& robot_pose, double target_orientation)
{
    double angle_diff = target_orientation - robot_pose.orientation_z;
    
    // If angular difference is higher than 180 degrees then 
    //     find the difference other way around
    if(abs(angle_diff) > M_PI) 
    {
        angle_diff = p_remainder(target_orientation, (2.0*M_PI)) 
            - p_remainder(robot_pose.orientation_z, (2.0*M_PI));
    }

    // Controller
    double angular_velo = angle_diff * this->k_p_angular 
        + this->k_static_angular * p_sign(angle_diff) ; // ~Proportional type control
    
    return angular_velo;
}

bool Navigation::calc_commands(Robot& robot, std::vector<double>& commands)
{
    double distance_to_next_point = this->distance_to_point(robot.pose, this->current_path_point);
    if(distance_to_next_point > this->DISTANCE_THRESHOLD)
    {
        double angle_to_next_point = this->angle_to_point(robot.pose, this->current_path_point);
        commands[0] = this->calc_linear_velo(distance_to_next_point);
        commands[1] = this->calc_angular_velo(robot.pose, angle_to_next_point);
        return true;
    }
    else
    {
        this->forward_path_one_point();
        std::cout << "\n    Path point " << this->num_current_point
            << " reached"<< std::endl;
        std::cout << "    Robot Position: x, y, orientation z: " << robot.pose.position_x
            << ", " << robot.pose.position_y << ", " << robot.pose.orientation_z << "\n\n";
        return false;
    }
}
