#include "../include/heat_map/Misc.h"
#include <cmath>

Misc::Misc(){
    poses_brute_force_search.clear();

    robot_pose_file.open("../../../src/heat_map/config/robot_poses_new_odometry_six_oriented.txt", std::ios::in);
    
    std::string new_line;
    int cont = 0;
    if(robot_pose_file.is_open()){
        RobotPose new_pose;
        while(std::getline(robot_pose_file, new_line)){
            std::cout << new_line << std::endl;
            if(new_line == "#"){
                std::cout << "New pose to be included" << std::endl;
                cont = 1;
            }else if(new_line == "@"){
                poses_brute_force_search.push_back(new_pose);
            }else{
                switch(cont){
                    case 1:
                        new_pose.robot_odom_x = std::stof(new_line);
                        cont++;
                        break;
                    case 2:
                        new_pose.robot_odom_y = std::stof(new_line);
                        cont++;
                        break;
                    case 3:
                        new_pose.quat_z = std::stof(new_line);
                        cont++;
                        break;         
                    case 4:
                        new_pose.quat_w = std::stof(new_line);
                        cont++;
                        break;                                              
                }
            }
        }
        robot_pose_file.close();
        for(int i = 0; i < poses_brute_force_search.size(); i++)
            std::cout << "X: " << poses_brute_force_search[i].robot_odom_x << " Y: " << poses_brute_force_search[i].robot_odom_y << " YAW: " << poses_brute_force_search[i].robot_yaw << std::endl;
        std::cout << "ALL POSES WITHIN THE ROBOT'S POSE LIST" << std::endl;
    }else{
        std::cout << "The file 'robot_pose_file' is not open!" << std::endl;
    }    
}

RobotPose Misc::getNextGoal(int index){
    if(index >= 0 && index < poses_brute_force_search.size())
        return poses_brute_force_search[index];
}