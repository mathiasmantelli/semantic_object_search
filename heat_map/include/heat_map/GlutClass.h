#ifndef __GLUTCLASS_H__
#define __GLUTCLASS_H__

#include <GL/glut.h>
#include <FreeImage.h>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <sys/time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string.h>

#include "heat_map/Robot.h"
#include "heat_map/Grid.h"
#include "heat_map/Misc.h"

class GlutClass{

public:
    static GlutClass* getInstance();

    void initialize();    
    void process();
    void terminate();

    void setRobot(Robot* r);
    void screenshot(SearchingMode searchingMode, int testNumber, std::string reference);

    int glutWindowSize;
    int frame;

    int halfWindowSize;
    int x_aux, y_aux;    

private:
    GlutClass ();
    ~GlutClass ();
    static GlutClass* instance_;    
    Robot* robot_;
    Grid* grid_;
    RobotPose robot_pose_;

    int id_;
    bool lockCameraOnRobot;    

    void render();

    static void display();
    static void reshape(int w, int h);
    static void keyboard(unsigned char key, int x, int y);
    static void specialKeys(int key, int x, int y);    
};

#endif /* __GLUT_H__ */