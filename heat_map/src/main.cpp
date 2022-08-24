#include <pthread.h>
#include <iostream>
#include <string.h>

#include "heat_map/Robot.h"
#include "heat_map/GlutClass.h"
#include "heat_map/Misc.h"

LogMode logMode; 
SearchingMode searchingMode;
std::string filename;

pthread_mutex_t* mutex;

void* startRobotThread(void* ref){
    Robot* robot = (Robot*) ref; 
    robot->initialize(logMode, searchingMode, filename);
    
    while(robot->isRunning()){
        robot->run();
    }
    return NULL;
}

void* startGlutThread(void* ref){
    GlutClass* glut = GlutClass::getInstance();
    
    glut->setRobot((Robot*) ref);
    glut->initialize();
    glut->process();

    return NULL;
}

void* startPlanningThread(void* ref){
    Robot* robot=(Robot*) ref;
    
    while(!robot->isReady()){
        usleep(100000);
    }
    robot->motionMode = RUNNING;
    
    robot->plan->initialize();
    // std::cout << "PLANNING - was initialized" << std::endl;
    while(robot->isRunning()){
        // std::cout << "PLAN RUN: " << robot->plan->run() << " OBJ FOUND: " << robot->isObjectFound() << std::endl; 
        if(!robot->plan->run() || robot->isObjectFound()){
            robot->motionMode = PREENDING;
            std::cout << "DISTANCE TRAVELLED: " << robot->computePathSize() << std::endl;            
        }
        // std::cout << "PLANNING - running" << std::endl;
        usleep(100000);
    }
    return NULL;
}

int main(int argc, char** argv){

    logMode = NONE; 
    searchingMode = NONE_SEARCHING;
    filename = ""; // ../../../list_objects.txt
    
    if(argc > 1){
        if(!strncmp(argv[1], "-R", 2) || !strncmp(argv[1], "-r", 2)){
            logMode = RECORDING; 
            std::cout << argc << " - " << argv[1] << std::endl;
        }else if(!strncmp(argv[1], "-Q", 2) || !strncmp(argv[1], "-q", 2)){
            logMode = QUERYING;     
            filename = argv[2];
            std::cout << argc << " - " << argv[1] << " and " << argv[2];
            if(argc > 2){
                if(!strncmp(argv[3], "-S", 2) || !strncmp(argv[3], "-s", 2)){
                    searchingMode = SEMANTIC;
                }else if(!strncmp(argv[3], "-BF", 3) || !strncmp(argv[3], "-bf", 3)){
                    searchingMode = BRUTE_FORCE;
                }else if(!strncmp(argv[3], "-LS", 3) || !strncmp(argv[3], "-ls", 3)){
                    searchingMode = LAST_SEEN;
                }else{
                    searchingMode = NONE_SEARCHING;
                }
                std::cout << " and " << argv[3];
            }
            std::cout << std::endl;
        }else if(!strncmp(argv[1], "-n", 2))
            logMode = NONE;    
    }
    std::cout << "LOG MODE: " << logMode << " | SEARCHING MODE: " << searchingMode << std:: endl;
    Robot* r; 
    r = new Robot();

    r->grid_map->grid_mutex = new pthread_mutex_t;
    if(pthread_mutex_init(r->grid_map->grid_mutex, NULL) != 0){
        std::cout << "MUTEX INIT HAS FAILED" << std::endl;
        return 1;
    }

    pthread_t robotThread, glutThread, planningThread; 
    mutex = new pthread_mutex_t;
    pthread_mutex_unlock(mutex);

    pthread_create(&(robotThread),NULL,startRobotThread,(void*)r);
    pthread_create(&(glutThread),NULL,startGlutThread,(void*)r);
    pthread_create(&(planningThread),NULL,startPlanningThread,(void*)r);

    pthread_join(robotThread, 0);
    pthread_join(glutThread, 0);
    pthread_join(planningThread, 0);

    return 0;    
}
