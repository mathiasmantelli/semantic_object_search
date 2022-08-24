#ifndef __PLANNING_H__
#define __PLANNING_H__

class Planning;

#include <pthread.h>
#include <queue>
#include "heat_map/Grid.h"
#include "heat_map/Objects.h"
#include "heat_map/Robot.h"
#include <string>
#include "heat_map/SemanticHP.h"
#include "heat_map/Misc.h"
#include <cmath>
#include <ctime>

class Planning{
public:
    Planning(std::string goal_obj_class);
    ~Planning(); 

    bool run(); 
    
    void initialize(); 
    void setGrid(Grid* g);
    void updateHeatValeuWithinMapSemantic();
    void updateHeatValeuWithinMap();
    void setLogMode(LogMode log);
    void setSearchingMode(SearchingMode the_searching_mode);
    void increaseBruteForceGoalCounter();
    Cell increaseSemanticGoalCounter();
    bool updating_grid_now;
    void setCurrentRobotsPose(RobotPose);
    Grid* grid; 
    SearchingMode searchingMode;
    Objects objs; 
    RobotPose current_goal, current_semantic_goal, current_robot_pose;
    bool is_robot_near_goal;

private:
    std::time_t current_time_;
    std::tm calendar_time_;
    Misc* the_misc;
    LogMode logMode_;
    SemanticHP* semanticHP;
    std::string goal_object; 
    int goal_counter_;
    int brute_force_goals_counter_;
    bool object_found(std::string obj_class);
};

#endif /* __PLANNING_H__ */
