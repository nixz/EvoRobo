/*
 * Main.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Joost Huizinga
 */

#include "TableSimulator.hpp"

GLDebugDrawer   gDebugDrawer;

/**
 * Main function, should create a window showing a table robot.
 */
int main(int argc, char* argv[]) {

    TableSimulator demoApp;

    demoApp.initPhysics();
    demoApp.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
    demoApp.addRobot();

    return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bulletphysics.com",&demoApp);

    return 0;
}



