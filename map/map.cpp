#include "map.h"

Map::Map()
{}

/** Create a pixelated square map,
 * with x pointing upwards, y pointing towards left and origin in the center
 */     
void Map::create_map(double size, double resolution)
{
    this->size = size;
    this->resolution = resolution;
    this->one_pixel_area = pow(resolution, 2);

    Pixel pixel;
    std::vector<Pixel> row; // to store a row of map-pixels
    this->map_width_pixels = int(size/resolution); // map size in pixels
    double x;
    double y;

    // Creating an empty map (in form of row-vector inside map-vector)
    for(int i = 0; i < this->map_width_pixels; i++)
    {
        row.clear();
        for(int j = 0; j < this->map_width_pixels; j++)
        {
            x = (this->map_width_pixels/2.0 - i) * resolution; // the x coordinate in meters
            y = (this->map_width_pixels/2.0 - j) * resolution;
            row.push_back(Pixel({ x, y, 0}));
        }
        this->data.push_back(row);
    }
    // print_map(map);
}

/// Print whole map for debugging
void Map::print_map()
{
    for (int i = 0; i < this->data.size(); i++) {
        for (int j = 0; j < this->data.size(); j++) 
        {
            // std::cout << "(" << this->data[i][j].x << ", " << this->data[i][j].y << ") " 
            //     << this->data[i][j].visited << " || ";
            
            if(data[i][j].visited)
            {
                std::cout << "# ";
            }
            else { std::cout << ". "; }
        }
        std::cout << "\n";
    }
}

/// Get id of pixel from points position in meters
std::vector<int> Map::get_pixel_id(double x, double y)
{
    std::vector<int> pixel_id;

    int a = round( (this->map_width_pixels / 2.0) - (x / this->resolution) );
    int b = round( (this->map_width_pixels / 2.0) - (y / this->resolution) );
    pixel_id.push_back(a);
    pixel_id.push_back(b);
    return pixel_id;
}

/// Returns true if point was not already marked
bool Map::mark_visited(double a, double b)
{
    if(this->data[a][b].visited == 1) return false;
    else { this->data[a][b].visited = 1; return true; }
}

std::vector<double> Map::calculate_line(Point& point_1, Point& point_2)
{
    // line equation is y = mx + c; m is slope and c is intercept with y-axis
    double m = (point_1.y - point_2.y) / (point_1.x - point_2.x);
    double c = point_1.y - (m * point_1.x);
    std::vector<double> line_eq = {m, c, point_1.x, point_1.y, point_2.x, point_2.y};
    return line_eq;
}

double Map::dire_wrt_line(Pixel& pixel, std::vector<double>& line, Robot& robot)
{
    if(line[0]==INFINITY || line[0]==-INFINITY) // In real system would occure only once (at the start)
    {
        int sign_pixel = p_sign( (pixel.x - line[2]) );
        int sign_robot = p_sign( (robot.pose.position_x - line[2]) );
        if(sign_pixel == sign_robot) return 1.0;
        else return 0;
    }
    else
    {
        // Check sign of ans, where ans = mx + c - y
        double ans = (line[0] * pixel.x) + line[1] - pixel.y;

        return p_sign(ans);
    }
}

bool Map::mark_area_visited(Robot& robot)
{
    // Calculate robot's 4 corner points at current position
    robot.update_corner_points();

    // Calculate lines and robot center's direction w.r.t. lines - (-1, 0, or 1)
    this->lines.clear();
    for(int i=0; i<4; i++)
    {
        this->lines.push_back(calculate_line( robot.corner_points[i], 
            robot.corner_points[ p_remainder((i+1),4) ] ));

        // Converting Pose to Pixels, to feed into dire_wrt_line() function
        Pixel pixel_robot_center;
        pixel_robot_center.x = robot.pose.position_x;
        pixel_robot_center.y = robot.pose.position_y;
        double robot_center_dire;
        robot_center_dire = this->dire_wrt_line(pixel_robot_center, this->lines[i], robot);

        this->lines[i].push_back(robot_center_dire);

        // std::cout << "line " << i << ": m, c, center dire. : " << lines[i][0] << ", "
        //     << lines[i][1] << ", " << lines[i][6] << ", " << std::endl;
    }

    // Loop over nearby points to find visited points
    int a, b;
    int approval_counter = 0;

    double start_pixel_x = robot.pose.position_x + (robot.dimension_x / 2.0 * 1.45);
    double start_pixel_y = robot.pose.position_y + (robot.dimension_x / 2.0 * 1.45);
    // 1.45 is taken from 2^0.5
    //     because that is maximum (square) robot's spread when oriented at 45 degrees
    
    std::vector<int> start_pixel = this->get_pixel_id(start_pixel_x, start_pixel_y); // Starting point
    int width = round( robot.dimension_x * 1.6 / this->resolution ); // Width in pixels for the area to be checked
    
    for(int a=start_pixel[0]; a < (start_pixel[0] + width); a++)
    {
        for(int b=start_pixel[1]; b < (start_pixel[1] + width); b++)
        {
            approval_counter = 0;
            for(int i=0; i<4; i++) // 4 iterations for 4 lines
            {
                // Checking pixels position wrt to line and then comparing with robot center's position
                if( dire_wrt_line(this->data[a][b], this->lines[i], robot) == this->lines[i][6] )
                {
                    approval_counter++;
                    // If reached the fourth iteration, that means pixel is below robot
                    //     hence mark the pixel as visited
                    if(approval_counter == 4) 
                    {
                        if( this->mark_visited(a, b) )
                        {
                            this->area_visited += this->one_pixel_area;
                        }
                    }
                }
                else { break; }
            }
        }
    }
    return true;
}

Map::~Map()
{}