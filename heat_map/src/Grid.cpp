#include "heat_map/Grid.h"
#include <GL/gl.h>
#include <algorithm>

Grid::Grid(std::string obj_class){
    goal_object_class = obj_class;
    map_scale_ = 30; 
    map_width_ = map_height_ = 2000;
    num_cells_in_row_ = map_width_; 
    half_num_cels_in_row_ = map_width_ / 2; 
    my_map_ = new Cell[map_width_ * map_height_];

    for(unsigned int j = 0; j < num_cells_in_row_; ++j){
        for(unsigned int i = 0; i < num_cells_in_row_; ++i){
            unsigned int index = j * num_cells_in_row_ + i; 
            my_map_[index].x = -half_num_cels_in_row_ + 1 + i;
            my_map_[index].y = half_num_cels_in_row_ - j;
            my_map_[index].value = -1;
            my_map_[index].robot_path = false;
            //my_map_[index].heat_map_value.clear();
            my_map_[index].heat_map_value = 0;
            my_map_[index].object_name = "none";
            my_map_[index].last_time_used = 0; 
            my_map_[index].obj_x = 0; 
            my_map_[index].obj_y = 0; 
            my_map_[index].cell_objects.clear();
        }
    }

    num_view_modes = 5;
    view_mode = 0; 
    global_counter = 0;
    map_limits.min_x = map_limits.min_y = 1000000;
    map_limits.max_x = map_limits.max_y = -1000000;

    goal_cell.cell_x = -1;
    goal_cell.cell_y = -1;
    goal_cell.yaw = 0;

    test_x = test_y = -1;

    plot_robot_path = true;
}

Cell* Grid::getCell(int x, int y){
    int i = x + half_num_cels_in_row_ - 1; 
    int j = half_num_cels_in_row_ - y; 
    //std::cout<< "GET CELL: [" << x << ", " << y << "]" << std::endl;
    return &(my_map_[j * num_cells_in_row_ + i]);
}

int Grid::getMapScale(){
    return map_scale_;
}

int Grid::getMapWidth(){
    return map_width_;
}

int Grid::getMapHeight(){
    return map_height_;
}

int Grid::getMapNumCellsInRow(){
    return num_cells_in_row_;
}

/* void Grid::updateBoundaries(int i, int j){
    if(i < min_x) min_x = i;
    if(i > max_x) max_x = i;

    if(j < min_y) min_y = j;
    if(j > max_y) max_y = j;    
} */

void Grid::draw(int xi, int yi, int xf, int yf){
    glLoadIdentity();

    for(int i = xi; i <= xf; ++i){
        for(int j = yi; j <= yf; ++j){
            drawCell(i + j * num_cells_in_row_);
        }
    }

    if(goal_cell.cell_x != -1){
        drawCellWithColor(goal_cell.cell_x, goal_cell.cell_y, 0, 0, 1.0);
    }
}

void Grid::drawText(unsigned int i){

}

void Grid::drawCell(unsigned int n){
    switch (view_mode) {
    case 0: //simple map
        if(my_map_[n].value == 100)
            glColor3f(0, 0, 0);
        else if(my_map_[n].value == 0)
            glColor3f(.95, .95, .95);
        else if(my_map_[n].value == -1)
            glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
        break;
    case 1:
        if(my_map_[n].heat_map_value != 0){                
            glColor3f(1, 1 - my_map_[n].heat_map_value, 0); //R,G,B
        }else if(my_map_[n].value == 100)
            glColor3f(0, 0, 0);
        else if(my_map_[n].value == 0)
            glColor3f(0, .95, 0);
        else if(my_map_[n].value == -1)
            glColor4f(0.0f, 1.0f, 1.0f, 1.0f);      
        break;
    case 2:
        if(my_map_[n].heat_map_value != 0){                
            glColor3f(1, my_map_[n].heat_map_value, 0); //R,G,B
        }else if(my_map_[n].value == 100)
            glColor3f(0, 0, 0);
        else if(my_map_[n].value == 0)
            glColor3f(0, .95, 0);
        else if(my_map_[n].value == -1)
            glColor4f(0.0f, 1.0f, 1.0f, 1.0f);      
        break;        
    case 3:
/*        if(!my_map_[n].heat_map_value.empty()){
             float sum = 0;
            for(auto i : my_map_[n].heat_map_value)
                sum += i; */
        if(my_map_[n].heat_map_value != 0 && my_map_[n].object_name == goal_object_class){                
            glColor3f(1, 1 - my_map_[n].heat_map_value, 0); //R,G,B
        //if(my_map_[n].heat_map_value != 0){
            //glColor3f(1, my_map_[n].heat_map_value, 0);                
        }else if(my_map_[n].value == 100)
            glColor3f(0, 0, 0);
        else if(my_map_[n].value == 0)
            glColor3f(0, .95, 0);
        else if(my_map_[n].value == -1)
            glColor4f(0.0f, 1.0f, 1.0f, 1.0f);      
        break;
    case 4:
/*        if(!my_map_[n].heat_map_value.empty()){
             float sum = 0;
            for(auto i : my_map_[n].heat_map_value)
                sum += i; */
        if(my_map_[n].heat_map_value != 0 && my_map_[n].object_name == goal_object_class){                
            glColor3f(1, my_map_[n].heat_map_value, 0); //R,G,B
        //if(my_map_[n].heat_map_value != 0){
            //glColor3f(1, my_map_[n].heat_map_value, 0);                
        }else if(my_map_[n].value == 100)
            glColor3f(0, 0, 0);
        else if(my_map_[n].value == 0)
            glColor3f(0, .95, 0);
        else if(my_map_[n].value == -1)
            glColor4f(0.0f, 1.0f, 1.0f, 1.0f);      
        break;      
    }
    if(plot_robot_path)
        if(my_map_[n].robot_path)
            glColor3f(1.0, 0, 0);        
    
    glBegin( GL_QUADS );
    { 
        glVertex2f(my_map_[n].x+1, my_map_[n].y+1);
        glVertex2f(my_map_[n].x+1, my_map_[n].y  );
        glVertex2f(my_map_[n].x  , my_map_[n].y  );
        glVertex2f(my_map_[n].x  , my_map_[n].y+1);
    }
    glEnd();    
}

void Grid::drawCellWithColor(int x, int y, float r, float g, float b){
    if(x != -1 and y != -1){
        Cell* c = getCell(x,y);
        int size = 4;
        glColor3f(r,g,b);

/*         glBegin( GL_QUADS );
        {
            glVertex2f(c->x+size, c->y+size);
            glVertex2f(c->x+size, c->y-size);
            glVertex2f(c->x-size, c->y-size);
            glVertex2f(c->x-size, c->y+size);
        }
        glEnd(); */

        float r; 
        int num_segments;
        r = 6.0;
        num_segments = 100;
        glBegin(GL_LINE_LOOP);
        for (int ii = 0; ii < num_segments; ii++)   {
            float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle 
            float x = r * cosf(theta);//calculate the x component 
            float y = r * sinf(theta);//calculate the y component 
            glVertex2f(x + c->x, y + c->y);//output vertex 
        }
        glEnd();

        float i = 0.0f;
        r = 4.0;        
        glColor3f(1,0,1);        
        glBegin(GL_TRIANGLE_FAN);
        
        glVertex2f(c->x, c->y); // Center
        for(i = 0.0f; i <= 360; i++)
                glVertex2f(r*cos(M_PI * i / 180.0) + c->x, r*sin(M_PI * i / 180.0) + c->y);
        
        glEnd();        

    }
}


void Grid::setMapWidth(int width){
    map_width_ = width;
}
void Grid::setMapHeight(int height){
    map_height_ = height;
}
void Grid::setMapScale(float scale){
    map_scale_ = scale;
}

void Grid::setMapROSWidth(int new_width){
    map_ROS_width_ = new_width; 
}


void Grid::setMapROSOrigin(float ros_map_origin_x, float ros_map_origin_y){
    map_ROS_origin_x_ = ros_map_origin_x;
    map_ROS_origin_y_ = ros_map_origin_y;
}

void Grid::setMapROSResolution(float ros_map_resolution){
    map_ROS_resolution_ = ros_map_resolution;
}

/* int Grid::matrixToVectorIndex(int i, int j){
    return i + j * map_ROS_width_;
}

std::tuple<int, int> Grid::vectorToMatrixIndex(int index){
    int j = index / map_ROS_width_; 
    int i = index - j * map_ROS_width_; 
    return std::make_tuple(i, j);
}

std::tuple<int, int> Grid::transformCoordinateOdomToMap(float x, float y){
    int j = (y - map_ROS_origin_y_)/map_ROS_resolution_;
    int i = (x - map_ROS_origin_x_)/map_ROS_resolution_;
    return std::make_tuple(i, j);
}

std::tuple<float, float> Grid::transformCoordinateMapToOdom(int x, int y){
    float i = (x + map_ROS_origin_x_/map_ROS_resolution_)*map_ROS_resolution_;
    float j = (y + map_ROS_origin_y_/map_ROS_resolution_)*map_ROS_resolution_;
    return std::make_tuple(i, j);
} */


void Grid::cleanHeatMapVector(){
    for(unsigned int j = 0; j < num_cells_in_row_; ++j){
        for(unsigned int i = 0; i < num_cells_in_row_; ++i){
            unsigned int index = j * num_cells_in_row_ + i; 
            my_map_[index].heat_map_value = 0;
        }
    }
}