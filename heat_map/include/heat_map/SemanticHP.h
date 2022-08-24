#ifndef SEMANTICHP_H
#define SEMANTICHP_H

#include "heat_map/Grid.h"
#include "heat_map/Misc.h"
#include "heat_map/Objects.h"
#include <iostream>
#include <algorithm>
#include <deque>
#include <vector>
#include <iomanip>
#include <cmath>
#include <map>

class SemanticHP{
public:
    SemanticHP();
    ~SemanticHP();

    void findMostLikelyPosition(Grid *grid, const std::vector<Object> list_objects);
    void findMostLikelyPositionSemantic(Grid *grid, const std::vector<Object> list_objects);
    void setCurrentObjClassGoal(std::string new_goal);
    void initialize(std::string goal_obj_class, bool *upating_map);
    Cell incrementPossibleGoalsCounter();
    float analyseGridPatchSemantic(Grid* grid, Cell* c);
    float analyseGridPatch(Grid* grid, Cell* c);
    Cell getCurrentSemanticGoal();
    std::vector<std::vector<float>> hour_weight_table; 
    std::map<float, Cell, std::greater<float>> possible_goals;
    std::map<float, Cell>::iterator current_goal_pointer_;
    bool *updating_grid_now_, first_finding_;
private:
    void includeNewGoal(float current_sum, Cell * current_cell);
    std::string goal_object_class_;
    MapSize map_size;
    int patch_size_, global_counter_, offset_size_;

};
#endif /* __SEMANTICHP_H__ */