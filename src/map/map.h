#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>
#include <math.h>
#include "../robot/robot.h"
#include "../miscellaneous/miscellaneous.h"
#include "../navigation/navigation.h"

struct Pixel
{
    double x, y;
    int visited = 0;
};

/// Map, to record visited areas and further
class Map
{
    std::vector<std::vector<Pixel>> data;
    double size = -1; // In meters
    double resolution = -1; // In meters
    int map_width_pixels = -1;
    double one_pixel_area;
    std::vector<std::vector<double>> lines; // Consists of line models, described by {slope, intercept}

public:
    
    double area_visited = 0;

    Map();
    ~Map();
    void create_map(double size=10.0, double resolution=1.0);
    void print_map();
    std::vector<int> get_pixel_id(double x, double y);
    bool mark_visited(double a, double b);
    std::vector<double> calculate_line(Point& point_1, Point& point_2);
    double dire_wrt_line(Pixel& pixel, std::vector<double>& line, Robot& robot);
    bool mark_area_visited(Robot& robot);

};

#endif