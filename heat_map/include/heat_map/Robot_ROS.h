#ifndef ROBOT_ROS_H
#define ROBOT_ROS_H

#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <ctime>
#include <vector>
#include <cstdlib>
#include <tuple>
#include <utility>
#include <iostream>
#include <fstream>


#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/CheckForObjectsAction.h"
#include "darknet_ros_msgs/ObjectCount.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Transform.h>

#include "heat_map/Grid.h"
#include "heat_map/Objects.h"
#include "heat_map/Misc.h"

#define DEG2RAD(x) x*M_PI/180.0
#define RAD2DEG(x) x*180.0/M_PI

class Robot_ROS{

public:
    Robot_ROS();
    bool initialize();
    void justPrint();
    void resumeMovement();

    sensor_msgs::Image getRGBImage();
    cv::Mat getRGBImageOpencv();
    sensor_msgs::Image getRGBDImage();
    sensor_msgs::Image getRGBDarnetImage();
    sensor_msgs::PointCloud getPointCloud();
    darknet_ros_msgs::BoundingBoxes getDarknetObjects();
    std::vector<darknet_ros_msgs::BoundingBox> getVectorDarknetObjects();
    darknet_ros_msgs::ObjectCount getObjectCount();
    nav_msgs::OccupancyGrid getOccupancyGrid();
    bool getImageIsConverted();
    bool getRobotPoseReceived();
    std::vector<geometry_msgs::Pose> getRobotsPath();
    void objectsWithinMap();
    void combineAllInformation();
    void combineAllInformationQuery();
    void saveOccupancyGrid();
    void setGrid(Grid* g);
    void updateHeatValeuWithinMap();
    void plotRobotPathOnGrid();
    void publishGoalPosition(GoalCell goal_cell);
    void publishGoalPositionBruteForce(RobotPose new_goal);
    void publishGoalPositionSemantic(RobotPose new_goal);
    float distanceGoalAndRobotsPosition(RobotPose new_goal);
    float getTotalTravelledDistance();
    std::tuple<int, int> transformCoordinateOdomToMap(float x, float y);
    std::tuple<float, float> transformCoordinateMapToOdom(int x, int y);

    RobotPose getRobotsPose();
    std::vector<Object> getObjectList();
    
private:
    
    ros::NodeHandle* node_;
    ros::Rate* rate_;

    ros::Subscriber sub_tf_, sub_map_, sub_rgb_image_, sub_rgbd_image_, sub_point_cloud_, sub_rgb_darknet_image_, sub_bounding_boxes_, sub_objects_bounding_boxes_;
    ros::Publisher pub_map_output_, pub_obj_map_, pub_move_base_;

    tf2_ros::TransformListener* listener_tf2;
    tf2_ros::Buffer* tf_buffer_;     

    nav_msgs::OccupancyGrid mapROS_, map_output_, map_objects_;
    geometry_msgs::Pose husky_pose_;
    geometry_msgs::PoseStamped goal_pose_; 
    std::vector<geometry_msgs::Pose> all_robot_poses_;
    std::vector<float> past_robots_yaw_;
    int amount_yaw_saved_;

    darknet_ros_msgs::BoundingBoxes darknet_objects_;
    std::vector<darknet_ros_msgs::BoundingBox> dn_objects_;

    RobotMode current_robots_mode_;

    sensor_msgs::Image rgb_image_, rgbd_image_, rgb_darknet_image_;
    sensor_msgs::PointCloud point_cloud_;
    darknet_ros_msgs::ObjectCount n_boxes_;

    cv::Mat bridged_image_;

    std::vector<Object> object_list_; 
    bool image_is_converted_, point_cloud_read_, robot_pose_, grid_map_, darknet_bounding_box_, map_published_, published_goal_pose_;
    int pose_map_x_, pose_map_y_, publishing_count_;
    double roll_, pitch_, yaw_;

    std::time_t current_time_;
    std::tm calendar_time_;

    RobotPose current_pose_robot_;
    RobotPose previous_pose_;
    float total_travelled_distance_;

    Grid* grid_;

    void receiveMap(const nav_msgs::OccupancyGrid::ConstPtr &value);
    void receiveTf(const tf::tfMessage::ConstPtr &value);
    void receiveRGBImage(const sensor_msgs::ImageConstPtr &value);
    void receiveRGBDImage(const sensor_msgs::Image &value);
    void receivePointCloud(const sensor_msgs::PointCloud &value);
    void receiveRGBDarknetImage(const sensor_msgs::Image &value);
    void receiveBoundingBoxes(const darknet_ros_msgs::ObjectCount::ConstPtr &value);
    void receiveObjectsBoundingBoxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr &value);
    void plotSquareWithinMap(int x, int y, int which_map);
    void plotCircleWithinMap(int x, int y, int which_map);
    float computeDistanceFromRobot2Object(int xmin, int xmax, int ymin, int ymax);
    float computeStandardDeviation(std::vector<float> past_robots_yaw_);
    int matrixToVectorIndex(int i, int j);
    std::tuple<int, int> vectorToMatrixIndex(int index);
    std::tuple<int, int> bresenhamForObjects(int x0, int y0, int x1, int y1);
    std::tuple<int, int> findNearestFreeCell(int x, int y);
};

#endif // ROBOT_ROS_H