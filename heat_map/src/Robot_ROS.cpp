#include "../include/heat_map/Robot_ROS.h"
#include "ros/time.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <ctime>
#include <memory>
#include <tuple>
#include <utility>

Robot_ROS::Robot_ROS(){
    int argc = 0; 
    char** argv = NULL; 

    ros::init(argc, argv, "heat_map_info");

    node_ = new ros::NodeHandle("~");
    rate_ = new ros::Rate(30);

    tf_buffer_ = new tf2_ros::Buffer;
    listener_tf2 = new tf2_ros::TransformListener(*tf_buffer_);

    sub_map_ = node_->subscribe("/map", 10, &Robot_ROS::receiveMap, this);
    sub_tf_ = node_->subscribe("/tf", 10, &Robot_ROS::receiveTf, this);
    sub_rgb_image_ = node_->subscribe("/realsense/color/image_raw", 10,&Robot_ROS::receiveRGBImage, this);
    sub_rgbd_image_ = node_->subscribe("/realsense/depth/image_rect_raw", 10,&Robot_ROS::receiveRGBDImage, this);
    sub_point_cloud_ = node_->subscribe("/realsense/depth/color/points", 10,&Robot_ROS::receivePointCloud, this);
    sub_rgb_darknet_image_ = node_->subscribe("/darknet_ros/detection_image", 10,&Robot_ROS::receiveRGBDarknetImage, this);
    sub_objects_bounding_boxes_ = node_->subscribe("/darknet_ros/found_object", 10, &Robot_ROS::receiveBoundingBoxes, this);
    sub_bounding_boxes_ = node_->subscribe("/darknet_ros/bounding_boxes", 10, &Robot_ROS::receiveObjectsBoundingBoxes, this);
    pub_map_output_ = node_->advertise<nav_msgs::OccupancyGrid>("/heatmap/map_robo_path", 1);
    pub_obj_map_ = node_->advertise<nav_msgs::OccupancyGrid>("/heatmap/obj_map", 1);
    pub_move_base_ = node_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    
    pose_map_x_ = 0;
    pose_map_y_ = 0;

    roll_ = 0;
    pitch_ = 0;
    yaw_ = 0;

    image_is_converted_ = false;
    point_cloud_read_ = false;
    robot_pose_ = false;
    grid_map_ = false;
    darknet_bounding_box_ = false;
    map_published_ = false;
    published_goal_pose_ = false;
    publishing_count_ = 0;

    current_time_ = std::time(nullptr);
    calendar_time_ = *std::localtime(std::addressof(current_time_));

    amount_yaw_saved_ = 80;

    current_pose_robot_.robot_map_x = 0;
    current_pose_robot_.robot_map_y = 0;
    current_pose_robot_.robot_odom_x = 0;
    current_pose_robot_.robot_odom_y = 0;
    current_pose_robot_.robot_yaw = 0;

    previous_pose_.robot_odom_x = -1; 
    previous_pose_.robot_odom_y = -1;

    total_travelled_distance_ = 0;
}

bool Robot_ROS::initialize(){
    return true;
}

//#########################################
//          SUBSCRIBE FUNCTIONS
//#########################################
void Robot_ROS::receiveMap(const nav_msgs::OccupancyGrid::ConstPtr &value){
    mapROS_.header = value->header;
    mapROS_.info = value->info;
    mapROS_.data = value->data;
    map_output_ = mapROS_;
    map_objects_ = mapROS_;

    grid_->setMapROSOrigin(mapROS_.info.origin.position.x, mapROS_.info.origin.position.y);
    grid_->setMapROSWidth(mapROS_.info.width);

    grid_->setMapROSResolution(mapROS_.info.resolution);

    grid_->map_limits.min_x = grid_->map_limits.min_y = 1000000;
    grid_->map_limits.max_x = grid_->map_limits.max_y = -1000000;

    for(int j = 0; j < mapROS_.info.height; ++j){
        for(int i = 0; i < mapROS_.info.width; ++i){
            if(mapROS_.data[i + j * mapROS_.info.width] > -1){
                if(grid_->map_limits.min_x > i)
                    grid_->map_limits.min_x = i;
                if(grid_->map_limits.min_y > j)
                    grid_->map_limits.min_y = j;
                if(grid_->map_limits.max_x < i)
                    grid_->map_limits.max_x = i;
                if(grid_->map_limits.max_y < j)
                    grid_->map_limits.max_y = j;                    
            }       
        }
    }

    for(int j = grid_->map_limits.min_y; j <= grid_->map_limits.max_y; j++){
        for(int i = grid_->map_limits.min_x; i <= grid_->map_limits.max_x; i++){
            Cell *c = grid_->getCell(i, j);
            c->value = mapROS_.data[i + j * mapROS_.info.width];
            c->robot_path = false;
            //c->heat_map_value.clear();
            c->heat_map_value = 0;
        }
    }

    for(int i = 0; i < all_robot_poses_.size(); i++){
        int size = 4; 
        int radius = 3;
        int pose_x = (all_robot_poses_[i].position.x - mapROS_.info.origin.position.x) / mapROS_.info.resolution;
        int pose_y = (all_robot_poses_[i].position.y - mapROS_.info.origin.position.y) / mapROS_.info.resolution;         
        for(int l = pose_y - size; l <= pose_y + size; ++l){
            for(int k = pose_x - size; k <= pose_x + size; ++k){
                if(pow(l - pose_y, 2) + pow(k - pose_x, 2) <= pow(radius, 2)){         
                    Cell *c = grid_->getCell(k, l);
                    c->robot_path = true;
                }
            }
        }
    }
 
    saveOccupancyGrid();
    grid_map_ = true;
}

void Robot_ROS::plotRobotPathOnGrid(){
    for(int i = 0; i < all_robot_poses_.size(); i++){
        int size = 4; 
        int radius = 3;
        int pose_x = (all_robot_poses_[i].position.x - mapROS_.info.origin.position.x) / mapROS_.info.resolution;
        int pose_y = (all_robot_poses_[i].position.y - mapROS_.info.origin.position.y) / mapROS_.info.resolution;         
        for(int l = pose_y - size; l <= pose_y + size; ++l){
            for(int k = pose_x - size; k <= pose_x + size; ++k){
                if(pow(l - pose_y, 2) + pow(k - pose_x, 2) <= pow(radius, 2)){         
                    Cell *c = grid_->getCell(k, l);
                    c->robot_path = true;
                }
            }
        }
    }
}

void Robot_ROS::receiveTf(const tf::tfMessage::ConstPtr &value){

    try{
        auto transform = tf_buffer_->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0)); 

        husky_pose_.position.x = transform.transform.translation.x;
        husky_pose_.position.y = transform.transform.translation.y;
        husky_pose_.position.z = transform.transform.translation.z;

        husky_pose_.orientation.x = transform.transform.rotation.x;
        husky_pose_.orientation.y = transform.transform.rotation.y;
        husky_pose_.orientation.z = transform.transform.rotation.z;
        husky_pose_.orientation.w = transform.transform.rotation.w;      

        if(previous_pose_.robot_odom_x == 0 && previous_pose_.robot_odom_y == 0){
            previous_pose_.robot_odom_x = husky_pose_.position.x;
            previous_pose_.robot_odom_y = husky_pose_.position.y;
        }else{
            total_travelled_distance_ += sqrt(pow(previous_pose_.robot_odom_x - husky_pose_.position.x, 2) + pow(previous_pose_.robot_odom_y - husky_pose_.position.y, 2));
            previous_pose_.robot_odom_x = husky_pose_.position.x;
            previous_pose_.robot_odom_y = husky_pose_.position.y;            
        }

        all_robot_poses_.push_back(husky_pose_);

        tf::Quaternion my_quat(transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z,
                                transform.transform.rotation.w);

        tf::Matrix3x3 my_mat(my_quat);
        my_mat.getRPY(roll_, pitch_, yaw_);

        float angle = RAD2DEG(yaw_);
        //std::cout << "#\n" << husky_pose_.position.x << "\n" << husky_pose_.position.y << "\n" << yaw_ << "\n";
        if(yaw_ < 0)
            yaw_ += 2 * M_PI;
        //std::cout << yaw_  << "\n@\n"<< std::endl;

        yaw_ = abs(yaw_);

        pose_map_x_ = transform.transform.translation.x / mapROS_.info.resolution - mapROS_.info.origin.position.x / mapROS_.info.resolution;
        pose_map_y_ = transform.transform.translation.y / mapROS_.info.resolution - mapROS_.info.origin.position.y / mapROS_.info.resolution;

        robot_pose_ = true;

        if(past_robots_yaw_.size() == amount_yaw_saved_)
            past_robots_yaw_.pop_back();
        past_robots_yaw_.emplace(past_robots_yaw_.begin(), yaw_); 
        if(computeStandardDeviation(past_robots_yaw_) == 0)
            current_robots_mode_ = IDLE;
        else
            current_robots_mode_ = MOVING;

        current_pose_robot_.robot_map_x = pose_map_x_;
        current_pose_robot_.robot_map_y = pose_map_y_;
        current_pose_robot_.robot_odom_x = transform.transform.translation.x;
        current_pose_robot_.robot_odom_y = transform.transform.translation.y;
        current_pose_robot_.robot_yaw = angle;    

    }catch(tf2::TransformException &ex){
        ROS_WARN("THE TRANSFORMATION HAS FAILED");
        ros::Duration(0.5).sleep();
    }
}

float Robot_ROS::computeStandardDeviation(std::vector<float> past_robots_yaw_){
    float sum = 0, mean, std_dev = 0; 
    for(int i = 0; i < past_robots_yaw_.size(); i++)
        sum += past_robots_yaw_[i] * (180/M_PI);
        
    mean = sum / past_robots_yaw_.size(); 

    for(int i = 0; i < past_robots_yaw_.size(); i++){
        std_dev += pow((past_robots_yaw_[i] * (180/M_PI)) - mean, 2);
    }
    
    std_dev = sqrt(std_dev / past_robots_yaw_.size());
    
    if(std_dev <= 0.01)
        std_dev = 0;

    return std_dev;
}

void Robot_ROS::receiveRGBImage(const sensor_msgs::ImageConstPtr &value){
    rgb_image_.data = value->data;
    rgb_image_.header = value->header;
    rgb_image_.height = value->height;
    rgb_image_.width = value->width;
    rgb_image_.encoding = value->encoding;
    rgb_image_.step = value->step;
    rgb_image_.is_bigendian = value->is_bigendian;
}

void Robot_ROS::receiveRGBDImage(const sensor_msgs::Image &value){
    rgbd_image_ = value;
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(value, sensor_msgs::image_encodings::TYPE_32FC1);
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("CV BRIDGE IS NOT WORKING");
    }
    //std::cout << " --------------------- NEW IMAGE SIZE: " << cv_ptr->image.rows << ", " << cv_ptr->image.cols << std::endl;
    cv_ptr->image.copyTo(bridged_image_);
    image_is_converted_ = true; 
    if(darknet_objects_.bounding_boxes.empty())
        image_is_converted_ = false;
}

void Robot_ROS::receivePointCloud(const sensor_msgs::PointCloud &value){
    point_cloud_ = value;
    point_cloud_read_ = true;
}

void Robot_ROS::receiveRGBDarknetImage(const sensor_msgs::Image &value){
    rgb_darknet_image_ = value;
}

void Robot_ROS::receiveBoundingBoxes(const darknet_ros_msgs::ObjectCount::ConstPtr &value){
    n_boxes_.count = value->count;
    n_boxes_.header = value->header;
}

void Robot_ROS::receiveObjectsBoundingBoxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr &value){
    darknet_objects_.header = value->header;
    darknet_objects_.image_header = value->image_header;
    darknet_objects_.bounding_boxes = value->bounding_boxes;
    dn_objects_.clear(); 
    for(int i = 0 ; i < value->bounding_boxes.size(); i++){    
        dn_objects_.push_back(value->bounding_boxes[i]);
    }

    darknet_bounding_box_ = true;
} 

void Robot_ROS::publishGoalPosition(GoalCell goal_cell){
    float x,y;
    if(goal_cell.cell_x != -1 and goal_cell.cell_y != -1 and !published_goal_pose_){
        std::cout << "ROS - publishgoalposition | " << goal_cell.cell_x << "," << goal_cell.cell_y << std::endl;
        std::tie(x,y) = transformCoordinateMapToOdom(goal_cell.cell_x, goal_cell.cell_y);
        goal_pose_.header.frame_id = "odom";
        goal_pose_.header.stamp = ros::Time::now();
        goal_pose_.pose.position.x = x;
        goal_pose_.pose.position.y = y;
        goal_pose_.pose.position.z = 0;
        
        tf2::Quaternion myq;
        myq.setRPY(0, 0, goal_cell.yaw);
        myq = myq.normalize();
        goal_pose_.pose.orientation.x = myq.x();
        goal_pose_.pose.orientation.y = myq.y();
        goal_pose_.pose.orientation.z = myq.z();
        goal_pose_.pose.orientation.w = myq.w();
        pub_move_base_.publish(goal_pose_);
        publishing_count_++;
        if(publishing_count_ == 15){
            published_goal_pose_ = true; 
        }
    }
    std::cout << "ROS - publishgoalposition" << std::endl;

}

void Robot_ROS::publishGoalPositionBruteForce(RobotPose new_goal){
    std::cout << "ROBOT ROS - PUBLISHING BRUTE FORCE - ";
    if(new_goal.robot_odom_x != -1 and new_goal.robot_odom_y != -1){
        std::cout << "======= Robot ROS PUBLISHING: Goal:[" << new_goal.robot_odom_x << ", " << new_goal.robot_odom_y << ", " << new_goal.robot_yaw << "]";
        goal_pose_.header.frame_id = "odom";
        goal_pose_.header.stamp = ros::Time::now();
        goal_pose_.pose.position.x = new_goal.robot_odom_x;
        goal_pose_.pose.position.y = new_goal.robot_odom_y;
        goal_pose_.pose.position.z = 0;
        
        // tf2::Quaternion myq;
        // myq.setRPY(0, 0, new_goal.robot_yaw*180/M_PI);
        // myq = myq.normalize();
        // std::cout << "QUATERNION: {" << myq.x() << ", " << myq.y() << ", " << myq.z() << ", " << myq.w() << ", " << std::endl;
        goal_pose_.pose.orientation.x = 0;
        goal_pose_.pose.orientation.y = 0;
        goal_pose_.pose.orientation.z = new_goal.quat_z;
        goal_pose_.pose.orientation.w = new_goal.quat_w;
        pub_move_base_.publish(goal_pose_);
       // std::cout << " - just published - ";
    }
    std::cout << " FINISHING PUBLISH ROS BRUTE FORCE " << std::endl;
}

void Robot_ROS::publishGoalPositionSemantic(RobotPose new_goal){
    std::cout << "ROBOT ROS - PUBLISHING SEMANTIC - R:[" << new_goal.robot_map_x << "," << new_goal.robot_map_y << "]" << std::endl;   
    if(new_goal.robot_odom_x != -1 and new_goal.robot_odom_y != -1){
        float angle = atan2(new_goal.robot_odom_y - new_goal.robot_map_y, new_goal.robot_odom_x - new_goal.robot_map_x);
        float x,y; 
        std::tie(x,y) = transformCoordinateMapToOdom(new_goal.robot_map_x, new_goal.robot_map_y);
        goal_pose_.header.frame_id = "odom";
        goal_pose_.header.stamp = ros::Time::now();
        goal_pose_.pose.position.x = x;
        goal_pose_.pose.position.y = y;
        goal_pose_.pose.position.z = 0;    

        tf2::Quaternion myq;
        myq.setRPY(0, 0, angle);
        myq = myq.normalize();
        goal_pose_.pose.orientation.x = myq.x();
        goal_pose_.pose.orientation.y = myq.y();
        goal_pose_.pose.orientation.z = myq.z();
        goal_pose_.pose.orientation.w = myq.w();
        pub_move_base_.publish(goal_pose_);            
    }
}

//#########################################
//              GET FUNCTIONS
//#########################################

RobotPose Robot_ROS::getRobotsPose(){
    return current_pose_robot_;
}

sensor_msgs::Image Robot_ROS::getRGBImage(){
    return rgb_image_;
}

cv::Mat Robot_ROS::getRGBImageOpencv(){
    return bridged_image_;
}

sensor_msgs::Image Robot_ROS::getRGBDImage(){
    return rgbd_image_;
}

sensor_msgs::Image Robot_ROS::getRGBDarnetImage(){
    return rgb_darknet_image_;
}

sensor_msgs::PointCloud Robot_ROS::getPointCloud(){
    return point_cloud_;
}

darknet_ros_msgs::BoundingBoxes Robot_ROS::getDarknetObjects(){
    return darknet_objects_;
}

std::vector<darknet_ros_msgs::BoundingBox> Robot_ROS::getVectorDarknetObjects(){
    return dn_objects_;
}

darknet_ros_msgs::ObjectCount Robot_ROS::getObjectCount(){
    return n_boxes_;
}

bool Robot_ROS::getImageIsConverted(){
    return image_is_converted_;
}

bool Robot_ROS::getRobotPoseReceived(){
    return robot_pose_;
}

nav_msgs::OccupancyGrid Robot_ROS::getOccupancyGrid(){
    return mapROS_;
}

std::vector<geometry_msgs::Pose> Robot_ROS::getRobotsPath(){
    return all_robot_poses_;
}
 
//#########################################
//              OTHER FUNCTIONS
//#########################################
void Robot_ROS::combineAllInformation(){
    if(image_is_converted_ && robot_pose_ && grid_map_ && darknet_bounding_box_){
        object_list_.clear();
        for(int i = 0 ; i < darknet_objects_.bounding_boxes.size(); i++){    
            if(current_robots_mode_ == IDLE){             
                int xcenter, ycenter, obj_x_map, obj_y_map;                 
                float distance, obj_x, obj_y; 

                //calculate the center of the bounding box and get its distance from the robot in de depth image
                xcenter = ((darknet_objects_.bounding_boxes[i].xmax - darknet_objects_.bounding_boxes[i].xmin)/2 + darknet_objects_.bounding_boxes[i].xmin);
                ycenter = ((darknet_objects_.bounding_boxes[i].ymax - darknet_objects_.bounding_boxes[i].ymin)/2 + darknet_objects_.bounding_boxes[i].ymin);        
                distance = bridged_image_.at<float>(xcenter, ycenter); 
                
                //calculate the object's position within the map, in relation to the robot's position
                obj_x = husky_pose_.position.x + distance * cos(yaw_);
                obj_y = husky_pose_.position.y + distance * sin(yaw_);
                std::tie(obj_x_map, obj_y_map) = transformCoordinateOdomToMap(obj_x, obj_y);
                if(mapROS_.data[matrixToVectorIndex(obj_x_map, obj_y_map)] != 0){
                    std::tie(obj_x_map, obj_y_map) = findNearestFreeCell(obj_x_map, obj_y_map);
                    std::tie(obj_x, obj_y) = transformCoordinateMapToOdom(obj_x_map, obj_y_map);
                }

                //combine the information in a new object instance, and insert it into the vector
                Object current_object; 
                current_object.obj_class = darknet_objects_.bounding_boxes[i].Class;
                current_object.obj_odom_x = obj_x;
                current_object.obj_odom_y = obj_y;
                current_object.robot_odom_x = husky_pose_.position.x;
                current_object.robot_odom_y = husky_pose_.position.y;
                current_object.hours_detection = calendar_time_.tm_hour;
                object_list_.push_back(current_object);
            }
        }
        darknet_objects_.bounding_boxes.clear();
    }
    pub_map_output_.publish(map_output_);
    pub_obj_map_.publish(map_objects_);
    map_published_ = true;
}

void Robot_ROS::combineAllInformationQuery(){
    if(image_is_converted_ && robot_pose_ && grid_map_ && darknet_bounding_box_){
        //object_list_.clear();
        darknet_objects_.bounding_boxes.clear();
    }
    pub_map_output_.publish(map_output_);
    pub_obj_map_.publish(map_objects_);
    map_published_ = true;
}

std::vector<Object> Robot_ROS::getObjectList(){
    return object_list_;
}

std::tuple<int, int> Robot_ROS::findNearestFreeCell(int x, int y){

    grid_->global_counter++;
    std::vector<std::pair<int, int>> to_be_processed; 
    std::pair<int, int> free_cell;
    to_be_processed.clear(); 
    to_be_processed.push_back(std::make_pair(x, y)); 
    while(!to_be_processed.empty()){   
        int size = 1; 
        std::pair<int, int> index = to_be_processed.back(); 
        to_be_processed.pop_back(); 

        Cell *c = grid_->getCell(index.first, index.second);        
        c->last_time_used = grid_->global_counter;

        for(int l = index.second - size; l <= index.second + size; ++l){
            for(int k = index.first - size; k <= index.first + size; ++k){    
                c = grid_->getCell(k, l);        
                if(c->last_time_used != grid_->global_counter){
                    c->last_time_used = grid_->global_counter;
                    if(c->value != 0){
                        to_be_processed.push_back(std::make_pair(k, l)); 
                    }else{
                        return std::make_pair(k, l);
                    }
                }
            }
        }
    }   

}

std::tuple<int, int> Robot_ROS::bresenhamForObjects(int px1, int py1, int px2, int py2){
    std::cout << "BRESENHAM - BEGINNING" << std::endl;
    int dx = px2 - px1; 
    int dy = py2 - py1; 

    int dLong = abs(dx); 
    int dShort = abs(dy);

    int offsetLong = dx > 0 ? 1 : -1; 
    int offsetShort = dy > 0 ?  mapROS_.info.width: -1 *mapROS_.info.width;

    if (dLong < dShort){
        std::swap(dShort, dLong); 
        std::swap(offsetShort, offsetLong);
    }

    int error = dLong/2; 
    int index = py1 * mapROS_.info.width + px1; 

    const int offset[] = {offsetLong, offsetLong + offsetShort}; 
    const int abs_d[] = {dShort, dShort - dLong}; 
    int i = 0;
    
    while(i <= dLong || mapROS_.data[index] == 0){
        map_output_.data[index] = 100;
        const int errorIsTooBig = error >= dLong; 
        index += offset[errorIsTooBig]; 
        error += abs_d[errorIsTooBig];
        i++;
    }
    map_output_.data[index] = 100;
    return vectorToMatrixIndex(index);
}

void Robot_ROS::justPrint(){
    for(int i = 0; i < all_robot_poses_.size(); i++){
        int pose_x = all_robot_poses_[i].position.x / mapROS_.info.resolution - mapROS_.info.origin.position.x / mapROS_.info.resolution;
        int pose_y = all_robot_poses_[i].position.y / mapROS_.info.resolution - mapROS_.info.origin.position.y / mapROS_.info.resolution;  
        if(pose_x >= 0 && pose_x <= mapROS_.info.width && pose_y >= 0 && pose_y <= mapROS_.info.height)      
            plotCircleWithinMap(pose_x, pose_y, 1);
    }
}

void Robot_ROS::plotSquareWithinMap(int x, int y, int which_map){
    int size = 5; 
    for(int l = y - size; l <= y + size; ++l){
        for(int k = x - size; k <= x + size; ++k){
            if(which_map == 1)
                map_output_.data[k + l * map_output_.info.width] = 100;
            else
                map_objects_.data[k + l * map_output_.info.width] = 100;            
        }
    }
}

void Robot_ROS::plotCircleWithinMap(int x, int y, int which_map){
    int size = 6; 
    int radius = 5;
    for(int l = y - size; l <= y + size; ++l){
        for(int k = x - size; k <= x + size; ++k){
            if(pow(l - y, 2) + pow(k - x, 2) <= pow(radius, 2)){
                if(which_map == 1)
                    map_output_.data[k + l * map_output_.info.width] = 100;
                else
                    map_objects_.data[k + l * map_output_.info.width] = 100;
            }
        }
    }
}

float Robot_ROS::computeDistanceFromRobot2Object(int xmin, int xmax, int ymin, int ymax){
    float sum_distances = 0;
    int cont = 0; 
    for(int i = xmin; i <= xmax; i++){
        for(int j = ymin; j <= ymax; j++){
            sum_distances += bridged_image_.at<float>(i, j);
            cont++;
        }
    }
    return sum_distances/cont;
}

void Robot_ROS::resumeMovement(){
    ros::spinOnce();
    rate_->sleep();
}

void Robot_ROS::objectsWithinMap(){
    int x_min, y_min, x_max, y_max;
    for(int i = 0 ; i < darknet_objects_.bounding_boxes.size(); i++){
        x_min = (int)darknet_objects_.bounding_boxes[i].xmin;
        x_max = (int)darknet_objects_.bounding_boxes[i].xmax;
        y_min = (int)darknet_objects_.bounding_boxes[i].ymin;
        y_max = (int)darknet_objects_.bounding_boxes[i].ymax;
    }
}

void Robot_ROS::saveOccupancyGrid(){
    system("rosrun map_server map_saver -f six_rooms_oriented map:=/map");
}

void Robot_ROS::setGrid(Grid* g){
    grid_ = g;
}

int Robot_ROS::matrixToVectorIndex(int i, int j){
    return i + j * mapROS_.info.width;
}

std::tuple<int, int> Robot_ROS::vectorToMatrixIndex(int index){
    int j = index / mapROS_.info.width; 
    int i = index - j * mapROS_.info.width; 
    return std::make_tuple(i, j);
}

std::tuple<int, int> Robot_ROS::transformCoordinateOdomToMap(float x, float y){
    int j = (y - mapROS_.info.origin.position.y)/mapROS_.info.resolution;
    int i = (x - mapROS_.info.origin.position.x)/mapROS_.info.resolution;
    return std::make_tuple(i, j);
}

std::tuple<float, float> Robot_ROS::transformCoordinateMapToOdom(int x, int y){
    float i = (x + mapROS_.info.origin.position.x/mapROS_.info.resolution)*mapROS_.info.resolution;
    float j = (y + mapROS_.info.origin.position.y/mapROS_.info.resolution)*mapROS_.info.resolution;
    return std::make_tuple(i, j);
}

/* bool Robot_ROS::publishGoalPosition(geometry_msgs::Pose the_goal){
    //I have to construct the PoseStamp message here
    geometry_msgs::PoseStamped new_goal;
    new_goal.header.stamp = ros::Time::now();
    new_goal.header.frame_id = "";
    new_goal.pose.position.x = the_goal.position.x;
    new_goal.pose.position.y = the_goal.position.y;
    new_goal.pose.position.z = the_goal.position.z;
    new_goal.pose.orientation.x = the_goal.orientation.x;
    new_goal.pose.orientation.y = the_goal.orientation.y;
    new_goal.pose.orientation.z = the_goal.orientation.z;
    new_goal.pose.orientation.w = the_goal.orientation.w;
    
} */

float Robot_ROS::distanceGoalAndRobotsPosition(RobotPose new_goal){
    //std::cout << "................... ROBOT ROS - R: [ " << husky_pose_.position.x << ", " << husky_pose_.position.y << "]" << 
    //" G: [ " << new_goal.robot_odom_x << ", " << new_goal.robot_odom_y << ", " << new_goal.robot_yaw << "] Dist.: " << sqrt(pow(husky_pose_.position.x - new_goal.robot_odom_x, 2) + pow(husky_pose_.position.y - new_goal.robot_odom_y, 2)) << std::endl; 
    return sqrt(pow(husky_pose_.position.x - new_goal.robot_odom_x, 2) + pow(husky_pose_.position.y - new_goal.robot_odom_y, 2));   
}

float Robot_ROS::getTotalTravelledDistance(){
    return total_travelled_distance_;
}
