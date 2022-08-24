#ifndef OBJECTS_H
#define OBJECTS_H

#include <string> 
#include <vector>
#include <tuple>
#include <set>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <heat_map/Misc.h>

class Object{
public:
    Object();
    Object(std::string class_obj, float odom_obj_x, float odom_obj_y, float odom_robot_x, float odom_robot_y, float hours);
    float obj_odom_x, obj_odom_y, robot_odom_x, robot_odom_y, hours_detection; 
    std::string obj_class;

};

class Objects{
public:
    Objects(); 
    bool updateObjects(const std::vector<Object> current_list);
    bool insertIfNotExist(Object the_object);
    std::vector<Object> list_objects;
    bool writeObjectListOnFile(Object the_object);
    void readObjectListFromFile(std::string file_address, SearchingMode searchingMode);
private:
    std::vector<std::string> object_classes_;
    std::ofstream output_file;    
    std::ifstream list_objects_, objects_input_;
    bool correctObjectClass(Object the_object);
    void printObject(Object the_object);
    std::tuple<float, float> euclideanDistance(Object list_obj, Object new_obj);
};

#endif