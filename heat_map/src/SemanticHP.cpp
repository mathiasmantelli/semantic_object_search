#include "../include/heat_map/SemanticHP.h"
#include "../include/heat_map/Grid.h"
#include <cmath>
#include <math.h>

//eight-neighbor offset
int offset[][8]={{-1,  1},
                 { 0,  1},
                 { 1,  1},
                 { 1,  0},
                 { 1, -1},
                 { 0, -1},
                 {-1, -1},
                 {-1,  0}
                };

SemanticHP::SemanticHP(){
    patch_size_ = 27;
    first_finding_ = false;
    global_counter_ = 0;
    offset_size_ = 8;
    possible_goals.clear();
}

void SemanticHP::initialize(std::string goal_obj_class, bool *updating_map){
    goal_object_class_ = goal_obj_class;    
    updating_grid_now_ = updating_map;
    
    int aux = 0; 
    for(int i = 0; i < 24; i++){
        std::vector<float> current_vector(24, 0);
        float value = i;
        bool increase;
        if(i == 0 || i > 12)
            increase = true;
        else
            increase = false;
        if(i > 12){
            aux++;
            value = 12.0 - aux; 
        }
        for(int j = 0; j < 24; j++){
            current_vector[j] = (1.0 - value/12.0);
            if(increase)
                value++; 
            else
                value--; 
            if(value >= 12)
                increase = false; 
            if(value == 0)
                increase = true; 
        }
        hour_weight_table.push_back(current_vector);
    }    
    // std::cout << "SEMANTICHP - INITIALIZED" << std::endl;
}

void SemanticHP::findMostLikelyPositionSemantic(Grid *grid, const std::vector<Object> list_objects){
    if(!*updating_grid_now_){
        int num_cells_in_row = grid->getMapNumCellsInRow();
        int goal_i, goal_j;
        goal_j = goal_i = 0;
        map_size = grid->map_limits;
        Cell *current_cell;
        float current_sum, biggest_sum; 
        biggest_sum = -1;
        for(int j = map_size.min_y; j <= map_size.max_y; j++){
            for(int i = map_size.min_x; i <= map_size.max_x; i++){ 
                current_cell = grid->getCell(i, j);
                if(current_cell->value == 0 and current_cell->object_name == goal_object_class_ and current_cell->heat_map_value != 0){
                    current_sum = analyseGridPatchSemantic(grid, current_cell);
                    // std::cout << "SHOULD INCLUDE? Sum:" << current_sum << " Cell:" << current_cell->obj_x << "," << current_cell->obj_y << std::endl;
                    includeNewGoal(current_sum, current_cell);
                }
            }
        }
/*         std::cout << "--------------------------" << std::endl;
        for(auto aux:possible_goals)
            std::cout << "Sum: " << aux.first << " | Point: " << aux.second.x << "," <<  aux.second.y << std::endl;
        std::cout << std::endl; */
        if(!first_finding_){
            first_finding_ = true;
            current_goal_pointer_ = possible_goals.begin();
        }        
        grid->goal_cell.cell_x = current_goal_pointer_->second.x;
        grid->goal_cell.cell_y = current_goal_pointer_->second.y;        
        if(grid->getCell(goal_i, goal_j)->obj_x != 0 && grid->getCell(goal_i, goal_j)->obj_y != 0){
            grid->goal_cell.yaw = atan2(grid->getCell(goal_i, goal_j)->obj_y - grid->goal_cell.cell_y, grid->getCell(goal_i, goal_j)->obj_x - grid->goal_cell.cell_x);
/*             if(grid->goal_cell.yaw > M_PI)
                grid->goal_cell.yaw -= 2 * M_PI;
            else if(grid->goal_cell.yaw <= -M_PI)
                grid->goal_cell.yaw += 2 * M_PI; */
            if(grid->goal_cell.yaw < 0)
                grid->goal_cell.yaw += 2 * M_PI;
            //std::cout << "------------------------------------- ROBOT'S YAW: " << grid->goal_cell.yaw << ", " << grid->goal_cell.yaw * 180/M_PI << std::endl;
        }
    }else{
        // std::cout << "IGNORED FINDING" << std::endl;
    }
}

Cell SemanticHP::incrementPossibleGoalsCounter(){
    current_goal_pointer_++;
    if(current_goal_pointer_ != possible_goals.end()){
        return current_goal_pointer_->second;
    }else{
        Cell new_cell;
        new_cell.x = -1; 
        new_cell.y = -1; 
        return new_cell;
    }
}

void SemanticHP::includeNewGoal(float current_sum, Cell *current_cell){
    bool exist = false, smaller = false; 
    Cell new_cell = *current_cell; 
    if(possible_goals.empty()) 
        possible_goals.emplace(current_sum, new_cell);
    else{
        std::map<float, Cell>::iterator it = possible_goals.begin();
        while(it != possible_goals.end()){
            // std::cout << "list: " << it->second.obj_x << "," << it->second.obj_y << " - " << it->first << " | NEW: " << current_cell->obj_x << "," << current_cell->obj_y << " - " << current_sum << std::endl;
            if(it->second.obj_x == current_cell->obj_x && it->second.obj_y == current_cell->obj_y){
                exist = true;
                if(it->first < current_sum){
                    smaller = true;
                    possible_goals.erase(it);
                    break;
                }
            }
            ++it;
        }    
        if(exist){
            if(smaller){
                possible_goals.emplace(current_sum, new_cell);    
                // std::cout << "INCLUDED: Sum:" << current_sum << " Cell:" << new_cell.obj_x << "," << new_cell.obj_y << std::endl;
            }else{
                // std::cout << "NOT INCLUDED: Sum:" << current_sum << " Cell:" << new_cell.obj_x << "," << new_cell.obj_y << std::endl;
            }
        }else{
            possible_goals.emplace(current_sum, new_cell);    
            // std::cout << "INCLUDED: Sum:" << current_sum << " Cell:" << new_cell.obj_x << "," << new_cell.obj_y << std::endl;
        }

    }
}

void SemanticHP::findMostLikelyPosition(Grid *grid, const std::vector<Object> list_objects){
    if(!*updating_grid_now_){
        int num_cells_in_row = grid->getMapNumCellsInRow();
        int goal_i, goal_j;
        goal_j = goal_i = 0;
        map_size = grid->map_limits;
        Cell *current_cell;
        float current_sum, biggest_sum; 
        biggest_sum = -1;
        for(int j = map_size.min_y; j <= map_size.max_y; j++){
            for(int i = map_size.min_x; i <= map_size.max_x; i++){ 
                current_cell = grid->getCell(i, j);
                if(current_cell->value == 0 and current_cell->object_name == goal_object_class_ and current_cell->heat_map_value != 0){
                    current_sum = analyseGridPatch(grid, current_cell);
                    //std::cout << "SUM : " << current_sum << std::endl;
                    if(current_sum > biggest_sum){
                        biggest_sum = current_sum;
                        //grid->goal_cell = current_cell;
                        goal_i = i; 
                        goal_j = j;
                    }
                }
            }
        }  
        //std::cout << "BIGGEST SUM : " << biggest_sum << std::endl;
        grid->goal_cell.cell_x = grid->getCell(goal_i, goal_j)->x;
        grid->goal_cell.cell_y = grid->getCell(goal_i, goal_j)->y;
        if(grid->getCell(goal_i, goal_j)->obj_x != 0 && grid->getCell(goal_i, goal_j)->obj_y != 0){
            grid->goal_cell.yaw = atan2(grid->getCell(goal_i, goal_j)->obj_y - grid->goal_cell.cell_y, grid->getCell(goal_i, goal_j)->obj_x - grid->goal_cell.cell_x);
/*             if(grid->goal_cell.yaw > M_PI)
                grid->goal_cell.yaw -= 2 * M_PI;
            else if(grid->goal_cell.yaw <= -M_PI)
                grid->goal_cell.yaw += 2 * M_PI; */
            if(grid->goal_cell.yaw < 0)
                grid->goal_cell.yaw += 2 * M_PI;
            //std::cout << "------------------------------------- ROBOT'S YAW: " << grid->goal_cell.yaw << ", " << grid->goal_cell.yaw * 180/M_PI << std::endl;
        }
    }else{
        std::cout << "IGNORED FINDING" << std::endl;
    }
}

float SemanticHP::analyseGridPatch(Grid* grid, Cell* c){
    std::deque<Cell*> to_be_analysed;
    Cell* current_cell, *neighboor_cell;
    global_counter_++;
    c->last_time_used = global_counter_;
    to_be_analysed.clear();
    to_be_analysed.emplace_back(c);
    int width = grid->getMapWidth();
    int height = grid->getMapHeight();
    float sum = 0;
    while(!to_be_analysed.empty()){
        current_cell = to_be_analysed.front();
        to_be_analysed.pop_front();
        for(int n = 0; n < offset_size_; n++){
            int new_x, new_y;
            new_x = current_cell->x+offset[n][0];
            new_y = current_cell->y+offset[n][1];
            
            if(new_x > 0 and new_x < width and new_y > 0 and new_y < height){
                neighboor_cell = grid->getCell(new_x, new_y);        
                float dist = pow(new_y - c->y, 2) + pow(new_x - c->x, 2);
                if(dist <= patch_size_ and neighboor_cell->value == 0 and 
                neighboor_cell->last_time_used != global_counter_ and 
                neighboor_cell->object_name == goal_object_class_){
                    neighboor_cell->last_time_used = global_counter_;
                    to_be_analysed.emplace_back(neighboor_cell);
                    //for(int i = 0; i < neighboor_cell->heat_map_value.size(); i++)
                        //sum += neighboor_cell->heat_map_value[i];
                        sum += 1 - neighboor_cell->heat_map_value;
                }
            }
        }
    }
    return sum;
}

float SemanticHP::analyseGridPatchSemantic(Grid* grid, Cell* c){
    std::deque<Cell*> to_be_analysed;
    Cell* current_cell, *neighboor_cell;
    global_counter_++;
    c->last_time_used = global_counter_;
    to_be_analysed.clear();
    to_be_analysed.emplace_back(c);
    int width = grid->getMapWidth();
    int height = grid->getMapHeight();
    float sum = 0;
    while(!to_be_analysed.empty()){
        current_cell = to_be_analysed.front();
        to_be_analysed.pop_front();
        for(int n = 0; n < offset_size_; n++){
            int new_x, new_y;
            new_x = current_cell->x+offset[n][0];
            new_y = current_cell->y+offset[n][1];
            
            if(new_x > 0 and new_x < width and new_y > 0 and new_y < height){
                neighboor_cell = grid->getCell(new_x, new_y);        
                float dist = pow(new_y - c->y, 2) + pow(new_x - c->x, 2);
                if(dist <= patch_size_ and neighboor_cell->value == 0 and 
                neighboor_cell->last_time_used != global_counter_ and 
                neighboor_cell->object_name == goal_object_class_){
                    neighboor_cell->last_time_used = global_counter_;
                    to_be_analysed.emplace_back(neighboor_cell);
                    //for(int i = 0; i < neighboor_cell->heat_map_value.size(); i++)
                        //sum += neighboor_cell->heat_map_value[i];
                        
                    sum += neighboor_cell->heat_map_value;
                }
            }
        }
    }
    // std::cout << "Sum: " << sum << std::endl;
    return sum;
}

void SemanticHP::setCurrentObjClassGoal(std::string new_goal){
    goal_object_class_ = new_goal;
}

Cell SemanticHP::getCurrentSemanticGoal(){
    if(first_finding_){
        return current_goal_pointer_->second;
    }else{
        Cell x; 
        x.obj_x = -1;
        x.obj_y = -1;
        x.heat_map_value = -1;
        x.last_time_used = -1;
        x.x = -1;
        x.y = -1;
        return x;
    }

}