#ifndef __GRID_H__
#define __GRID_H__
#include <cstdio>
#include <GL/glut.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "math.h"
#include <string>
#include <vector>
#include <heat_map/Misc.h>
#include "heat_map/Objects.h"

struct Cell{
    int x, y, value; 
    bool robot_path; 
    //std::vector<float> heat_map_value;
    std::vector<Object> cell_objects;
    float heat_map_value;
    int last_time_used;
    std::string object_name; 
    int obj_x, obj_y;
};

struct MapSize{
    int min_x, min_y, max_x, max_y;
};

class Grid{
public:
    Grid(std::string obj_class); 
    Cell* getCell(int x, int y); 
    GoalCell goal_cell;
    int getMapScale();
    int getMapWidth();
    int getMapHeight();
    int getMapNumCellsInRow();
    void setMapWidth(int width);
    void setMapHeight(int height); 
    void setMapScale(float scale); 
/*     int matrixToVectorIndex(int i, int j);
    std::tuple<int, int> vectorToMatrixIndex(int index);
    std::tuple<int, int> transformCoordinateOdomToMap(float x, float y);
    std::tuple<float, float> transformCoordinateMapToOdom(int x, int y); */
    void setMapROSWidth(int new_width);
    void setMapROSOrigin(float ros_map_origin_x, float ros_map_origin_y);
    void setMapROSResolution(float ros_map_resolution);
    void cleanHeatMapVector();
    void draw(int xi, int yi, int xf, int yf);

    void updateBoundaries(int i, int j);
    bool plot_robot_path;
    int min_x, min_y, max_x, max_y;
    int num_view_modes, view_mode, iterations;
    int global_counter;
    int test_x, test_y;
    float map_ROS_origin_x_, map_ROS_origin_y_, map_ROS_resolution_;
    pthread_mutex_t* grid_mutex; 
    std::string goal_object_class;
    MapSize map_limits; 
    int map_width_, map_height_, num_cells_in_row_, half_num_cels_in_row_;
private:
    int map_ROS_width_;
    float map_scale_;

    Cell* my_map_;
    void drawCell(unsigned int i); 
    void drawText(unsigned int i);
    void drawCellWithColor(int x, int y, float r, float g, float b);
};
#endif // __GRID_H__