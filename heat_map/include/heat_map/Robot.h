#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <sys/time.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <ros/time.h>

#include "heat_map/Misc.h"
#include "heat_map/Robot_ROS.h"
#include "heat_map/Grid.h"
#include "nav_msgs/OccupancyGrid.h"
#include "heat_map/Planning.h"

class Robot{

public:
    Robot();
    ~Robot();

    void initialize(LogMode logMode, SearchingMode searchingMode, std::string filename);
    void run();    

    bool isReady();
    bool isRunning();
    bool isObjectFound();
    RobotPose getRobotsPose();
    float computePathSize();
    float measureDistanceGoalAndRobotsPosition();
    bool first_goal_published;
    void drawRobot(const float robot_x, const float robot_y, const float robot_yaw);

    Grid* grid_map;
    Planning* plan;
    MotionMode motionMode;
    std::vector<Object> current_object_list;
    std::vector<Object*> all_objects_list;
    std::string input_objects_list;
    SearchingMode robot_searching_mode;

protected:
    bool ready_;
    bool running_;
    bool next_goal_;  
    bool object_found_; 
    LogMode logMode_;
    RobotPose robot_pose_;
    double next_goal_time_;

    int windowSize_;
    std::string object_goal_;
    std::vector<darknet_ros_msgs::BoundingBox> darknet_objects_;

    sensor_msgs::Image rgb_image_, rgbd_image_, rgb_darknet_image_;
    sensor_msgs::PointCloud2 point_cloud_;
    darknet_ros_msgs::ObjectCount n_boxes_;
    nav_msgs::OccupancyGrid grid_map_;

    Robot_ROS robotRos; 
};

#endif // ROBOT_H