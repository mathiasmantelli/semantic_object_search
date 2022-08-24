#ifndef MISC_H
#define MISC_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

enum LogMode {NONE, RECORDING, QUERYING};

enum RobotMode {IDLE, MOVING};    
enum MotionMode {RUNNING, ENDING, PREENDING};
enum SearchingMode {NONE_SEARCHING, SEMANTIC, BRUTE_FORCE, LAST_SEEN};

struct GoalCell{
        int cell_x, cell_y; 
        float yaw; 
};

struct RobotPose{
        int robot_map_x, robot_map_y;
        float robot_odom_x, robot_odom_y;
        float robot_yaw, quat_z, quat_w;
};

class Misc{
public:
        Misc();
        std::vector<RobotPose> poses_brute_force_search;
        std::ifstream robot_pose_file;
        RobotPose getNextGoal(int index);
};
#endif // MISC_H