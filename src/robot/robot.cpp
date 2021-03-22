#include "robot.h"

Robot::Robot() 
    : pose(Pose(0.0,0.0,0.0))
{}

/// Move the robot at specified speed and turning angle
void Robot::move(double cmd_lin_velo, double cmd_rot_velo)
{
    // Check commanded values for the hardware-limits (if command > max then command with max)
    // Pseudo - Set linear and rotational speed (in m/s and rad/s respectively)
    // std::cout << "cmd lin vel: " << cmd_lin_velo <<std::endl;
    
    // Robot pose is simulated for this case
    this->simulate_pose(cmd_lin_velo, cmd_rot_velo);
}

/// !!Assumption!!, robot is assumed to have differential drive
/// Approximate robot pose after one time step
void Robot::simulate_pose(double cmd_lin_velo, double cmd_rot_velo)
{
    // Assumption: Robot has limited linear and rotational speed, acceleration consideration is omitted altogether
    
    // Assuming robot start position to be in positive X direction
    double advancement_x = cos(this->pose.orientation_z) * cmd_lin_velo * this->control_time_step;
    double advancement_y = sin(this->pose.orientation_z) * cmd_lin_velo * this->control_time_step;
    double advancement_orientation_z = cmd_rot_velo * this->control_time_step;

    this->pose.position_x = this->pose.position_x + advancement_x;
    this->pose.position_y = this->pose.position_y + advancement_y;

    this->pose.orientation_z = this->pose.orientation_z + advancement_orientation_z;
    // Keeping pose in the range of (-PI, PI]
    if(this->pose.orientation_z <= -M_PI)
    {
        this->pose.orientation_z = pose.orientation_z + 2.0 * M_PI;
    }
    else if(this->pose.orientation_z > M_PI)
    {
        this->pose.orientation_z = pose.orientation_z - 2.0 * M_PI;
    }
}

void Robot::update_corner_points()
{
    // Four corner points can be found with summation of two perpendicular vectors to the center of the robot;
    //     two perpendicular vector's 4 permutations would provide 4 corners
    // These two vectors are, front and side direction of robot 
    //     i.e. at robot's zero position (robot front is in +X direction)
    //     (0, 0)->(0.25, 0) and (0, 0)->(0, 0.25)
    
    // Calculate those two perpendicular vector's current version (with changed orientation)
    // if x1, y1 is the original vector and β is the rotation, then formula is ..
    // x2 = cosβ x1 − sinβ y1 
    // y2 = sinβ x1 + cosβ y1
    double vector1_x = cos(this->pose.orientation_z) * this->front_vector[0]
        - sin(this->pose.orientation_z) * this->front_vector[1];

    double vector1_y = sin(this->pose.orientation_z) * this->front_vector[0]
        + cos(this->pose.orientation_z) * this->front_vector[1];

    double vector2_x = cos(this->pose.orientation_z) * this->side_vector[0]
        - sin(this->pose.orientation_z) * this->side_vector[1];

    double vector2_y = sin(this->pose.orientation_z) * this->side_vector[0]
        + cos(this->pose.orientation_z) * this->side_vector[1];

    this->corner_points.clear(); 
    Point point;
    for(int i=0; i<4; i++)
    {
        point.x = this->pose.position_x 
            + vector1_x * this->permutations[i][0] 
                + vector2_x * this->permutations[i][1];
        point.y = this->pose.position_y 
            + vector1_y * this->permutations[i][0] 
                + vector2_y * this->permutations[i][1];

        this->corner_points.push_back(point);

        // std::cout << "a corner point: " << point.x << ", " << point.y << std::endl;
    }
}
