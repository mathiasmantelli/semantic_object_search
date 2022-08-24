#include "heat_map/Robot.h"
#include "heat_map/Misc.h"
#include <GL/gl.h>

Robot::Robot(){
    ready_ = false;
    running_ = true;
    next_goal_ = false;  
    object_found_ = false; 
    object_goal_ = "Mug";
    grid_map = new Grid(object_goal_); 
    
    plan = new Planning(object_goal_);
    plan->setGrid(grid_map);

    robotRos.setGrid(grid_map);
}
Robot::~Robot(){
    if(grid_map!=NULL)
        delete grid_map;    
}

void Robot::initialize(LogMode logMode, SearchingMode searchingMode, std::string filename){
    robot_searching_mode = searchingMode;
    logMode_ = logMode;
    plan->setLogMode(logMode_);
    plan->setSearchingMode(robot_searching_mode);
    input_objects_list = filename;
    first_goal_published = false;
    if(logMode == QUERYING)
        plan->objs.readObjectListFromFile(input_objects_list, robot_searching_mode);
    
    bool success = robotRos.initialize();
    if(!success){
        std::cout << "Error while initializing the Robot ROS class!" << std::endl;
        exit(0);
    }
    
}

void Robot::run(){
  
    if(logMode_ == RECORDING){
        std::cout << "Map saved" << std::endl;
        robotRos.saveOccupancyGrid();
        robotRos.combineAllInformation();
        current_object_list = robotRos.getObjectList();
        bool obj_update;
        if(!current_object_list.empty())
            obj_update = plan->objs.updateObjects(current_object_list);        
    }else{ //QUERYING or NONE
        robotRos.plotRobotPathOnGrid();   
        if(robot_searching_mode == BRUTE_FORCE){
            if(plan->current_goal.robot_odom_x != -1 && plan->current_goal.robot_odom_y != -1){
                //std::cout << "ROBOT RUN - DISTANCE: " << robotRos.distanceGoalAndRobotsPosition(plan->current_goal) << " - INCREMENTING THE COUNTER." << std::endl;
                if(robotRos.distanceGoalAndRobotsPosition(plan->current_goal) < 0.26 /* || !first_goal_published */){
                    if(!next_goal_){
                        next_goal_time_ = ros::Time::now().toSec();
                        next_goal_ = true;
                    }
                    double current_time = ros::Time::now().toSec();
                    float time_different = current_time - next_goal_time_;
                    std::cout << "************* TIME DIFFERENCE: " << time_different << std::endl;                    
                    if(time_different >= 3.5){
                        darknet_objects_ = robotRos.getVectorDarknetObjects();
                        for(int i = 0; i < (int)darknet_objects_.size(); i++){
                            std::cout << "OBJECT CLASS: " << darknet_objects_[i].Class << std::endl;
                            if(darknet_objects_[i].Class == object_goal_){
                                std::cout << "################################################## I FOUND IT ##################################################" << std::endl;
                                std::cout << darknet_objects_[i].Class << " == " << object_goal_ << std::endl;
                                object_found_ = true;
                            }
                        }                        
                        if(time_different >= 5 && !object_found_){
                            plan->increaseBruteForceGoalCounter();
                            robotRos.publishGoalPositionBruteForce(plan->current_goal);   
                            next_goal_ = false;
                        }
                    }
                    // std::cout << "ROBOT RUN - BRUTE FORCE - goal:[" << plan->current_goal.robot_odom_x << ", " << plan->current_goal.robot_odom_y << ", " << plan->current_goal.robot_yaw << "]" << std::endl;
                }else{
                    next_goal_ = false;
                }
                if(!first_goal_published){
                    robotRos.publishGoalPositionBruteForce(plan->current_goal);
                    first_goal_published = true;             
                }
            }
        }else if(robot_searching_mode == LAST_SEEN){
            std::cout << "ROBOT - QUERYING MODE - LAST_SEEN" << std::endl;
            robotRos.publishGoalPosition(grid_map->goal_cell);
            RobotPose new_goal;
            std::tie(new_goal.robot_odom_x, new_goal.robot_odom_y) = robotRos.transformCoordinateMapToOdom(grid_map->goal_cell.cell_x, grid_map->goal_cell.cell_y);
            if(robotRos.distanceGoalAndRobotsPosition(new_goal) < 0.26){
                darknet_objects_ = robotRos.getVectorDarknetObjects();
                for(int i = 0; i < (int)darknet_objects_.size(); i++){
                    std::cout << "OBJECT CLASS: " << darknet_objects_[i].Class << std::endl;
                    if(darknet_objects_[i].Class == object_goal_){
                        std::cout << "################################################## I FOUND IT ##################################################" << std::endl;
                        std::cout << darknet_objects_[i].Class << " == " << object_goal_ << std::endl;
                        object_found_ = true;
                    }
                }            
            }
        }else if(robot_searching_mode == SEMANTIC){
            if(plan->current_semantic_goal.robot_odom_x != -1 && plan->current_semantic_goal.robot_odom_y != -1 && !object_found_){
                // std::cout << "ROBOT - QUERYING MODE - SEMANTIC | " << plan->current_semantic_goal.robot_odom_x << "," << plan->current_semantic_goal.robot_odom_y << std::endl;
                RobotPose new_goal;
                std::tie(new_goal.robot_odom_x, new_goal.robot_odom_y) = robotRos.transformCoordinateMapToOdom(plan->current_semantic_goal.robot_map_x, plan->current_semantic_goal.robot_map_y);            
                //std::cout << "ROBOT RUN - DISTANCE: " << robotRos.distanceGoalAndRobotsPosition(plan->current_goal) << " - INCREMENTING THE COUNTER." << std::endl;
                if(robotRos.distanceGoalAndRobotsPosition(new_goal) < 0.3 || next_goal_){
                    if(!next_goal_){
                        next_goal_time_ = ros::Time::now().toSec();
                        next_goal_ = true;
                    }
                    double current_time = ros::Time::now().toSec();
                    float time_different = current_time - next_goal_time_;
                    std::cout << "************* SEMANTIC - TIME DIFFERENCE: " << time_different << std::endl;                    
                    std::vector<darknet_ros_msgs::BoundingBox> temp;
                    temp = robotRos.getVectorDarknetObjects();
                    darknet_objects_.insert(std::end(darknet_objects_), std::begin(temp), std::end(temp));                    
                    if(time_different >= 5){
                        for(int i = 0; i < (int)darknet_objects_.size(); i++){
                            std::cout << "OBJECT CLASS: " << darknet_objects_[i].Class << std::endl;
                            if(darknet_objects_[i].Class == object_goal_){
                                std::cout << "################################################## I FOUND IT ##################################################" << std::endl;
                                std::cout << darknet_objects_[i].Class << " == " << object_goal_ << std::endl;
                                object_found_ = true;
                            }
                        }                        
                        if(time_different >= 5.2 && !object_found_){
                            darknet_objects_.clear();
                            Cell new_cell = plan->increaseSemanticGoalCounter();
                            if(new_cell.x != -1 && new_cell.y != -1){
                                RobotPose temp_pose;
                                temp_pose.robot_map_x = new_cell.x;
                                temp_pose.robot_map_y = new_cell.y;
                                temp_pose.robot_odom_x = new_cell.obj_x;
                                temp_pose.robot_odom_y = new_cell.obj_y;
                                temp_pose.robot_yaw = -1;                            
                                robotRos.publishGoalPositionSemantic(temp_pose);
                                next_goal_ = false;
                            }
                        }                       
                    }                    
                    // std::cout << "ROBOT RUN - BRUTE FORCE - goal:[" << plan->current_goal.robot_odom_x << ", " << plan->current_goal.robot_odom_y << ", " << plan->current_goal.robot_yaw << "]" << std::endl;
                }else{
                    next_goal_ = false;
                }
                if(!first_goal_published){
                    robotRos.publishGoalPositionSemantic(plan->current_semantic_goal);
                    first_goal_published = true;             
                }                                 
            }
        }       
        //plan computes the position to go based on the query object 
        //robotRos receives the goal pose to navigate the robot towards it
        
        
    }
    
    robot_pose_ = robotRos.getRobotsPose();
    plan->setCurrentRobotsPose(robot_pose_);

    if(motionMode == ENDING)
        running_ = false;
    robotRos.resumeMovement();
    if(robotRos.getRobotPoseReceived())
        ready_ = true;

    usleep(50000);
}

bool Robot::isReady()
{
    return ready_;
}

bool Robot::isRunning()
{
    return running_;
}

RobotPose Robot::getRobotsPose(){
    return robot_pose_;
}

void Robot::drawRobot(const float robot_x, const float robot_y, const float robot_yaw){
    float scale = 3.8; //grid_map->getMapScale();
    glTranslatef(robot_x, robot_y, 0.0);
    glRotatef(robot_yaw, 0, 0, 1);

    glScalef(1.0/scale, 1.0/scale, 1.0/scale);

    glColor3f(1.0,1.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();
    glColor3f(0.0,0.0,0.0);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0); 
        glVertex2f(30, 0);
    }
    glEnd();

    glScalef(scale, scale, scale);
    glRotatef(-robot_yaw, 0, 0, 1.0);
    glTranslatef(-robot_x, -robot_y, 0);    

}

float Robot::computePathSize(){
    // float total_distance = 0; 
    // std::vector<geometry_msgs::Pose> all_poses = robotRos.getRobotsPath();
    // for(int i = 0; i < all_poses.size()-1; i++){
    //     std::cout << "ROBOT - COMPUTE PATH SIZE - Size: " << all_poses.size() << " i: " << i << " | [(" << all_poses[i].position.x << "," << all_poses[i].position.y << ") - (" << all_poses[i+1].position.x << "," << all_poses[i+1].position.y << ")]" << std::endl;
    //     float pow1 = pow(all_poses[i].position.x - all_poses[i+1].position.x, 2);
    //     std::cout << "powX = " << pow1 << std::endl;
    //     float pow2 = pow(all_poses[i].position.y - all_poses[i+1].position.y, 2);
    //     std::cout << "powY = " << pow2 << std::endl;
    //     total_distance += sqrt(pow1 + pow2);
    //     std::cout << "sqrt = " << sqrt(pow1 + pow2) << std::endl;
    // }
    // return total_distance; 
    return robotRos.getTotalTravelledDistance();
}

float Robot::measureDistanceGoalAndRobotsPosition(){
    return robotRos.distanceGoalAndRobotsPosition(plan->current_goal);
}

bool Robot::isObjectFound(){
    return object_found_;
}