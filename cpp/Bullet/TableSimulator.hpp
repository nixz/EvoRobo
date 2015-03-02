/*
 * TableSimulator.hpp
 *
 *  Created on: Jan 23, 2015
 *      Author: Joost Huizinga
 */

#ifndef TABLESIMULATOR_HPP_
#define TABLESIMULATOR_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletDynamicsCommon.h"
#include "GLDebugDrawer.h"

#include "TableSimulator.hpp"
#include "TableRobot.hpp"
#include "TouchSensor.hpp"

using namespace std;

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class TableSimulator;
static TableSimulator* tableSimulator;

/**
 * Callback function to process collisions with sensors.
 */
bool myContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1)
{
    TouchSensor* sensors[2];
    btCollisionObject* o1 = static_cast<btCollisionObject*>(body0);
    btCollisionObject* o2 = static_cast<btCollisionObject*>(body1);
    sensors[0] = static_cast<TouchSensor*>(o1->getUserPointer());
    sensors[1] = static_cast<TouchSensor*>(o2->getUserPointer());

    for(size_t i=0; i<2;++i){
        if(sensors[i]){
            sensors[i]->setValue(true);
        }
    }
    return false;
}

/**
 * Simulator for evolving walking tables.
 */
class TableSimulator : public GlutDemoApplication
{
    btAlignedObjectArray<class TableRobot*> m_tables;
    //keep the collision shapes, for deletion/cleanup
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
    btAlignedObjectArray<btHingeConstraint*> m_hinges;
    btAlignedObjectArray<btRigidBody*> m_body;
    btBroadphaseInterface*  m_broadphase;
    btCollisionDispatcher*  m_dispatcher;
    btConstraintSolver* m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    bool pause;
    bool oneStep;

public:
    int internalTimer;


public:
    /**
     * Initialized the world.
     *
     * Note that constructing the TableSimulator is not enough to create a world.
     */
    void initPhysics(){
        internalTimer = 0;
        pause = true;

        // Setup the basic world
        tableSimulator = this;
        gContactProcessedCallback = myContactProcessedCallback;
        setTexturing(true);
        setShadows(true);

        setCameraDistance(btScalar(5.));

        m_collisionConfiguration = new btDefaultCollisionConfiguration();

        m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

        btVector3 worldAabbMin(-10000,-10000,-10000);
        btVector3 worldAabbMax(10000,10000,10000);
        m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

        m_solver = new btSequentialImpulseConstraintSolver;

        m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
        //m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
        //m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;



        // Setup a big ground box
        {
            btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
            m_collisionShapes.push_back(groundShape);
            btTransform groundTransform;
            groundTransform.setIdentity();
            groundTransform.setOrigin(btVector3(0,-10,0));

    #define CREATE_GROUND_COLLISION_OBJECT 1
    #ifdef CREATE_GROUND_COLLISION_OBJECT
            btCollisionObject* fixedGround = new btCollisionObject();
            fixedGround->setCollisionShape(groundShape);
            fixedGround->setWorldTransform(groundTransform);
            fixedGround->setUserPointer(0);
            m_dynamicsWorld->addCollisionObject(fixedGround);

    #else
            localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
    #endif //CREATE_GROUND_COLLISION_OBJECT

        }

        gDisableDeactivation = true;

        clientResetScene();

    }

    /**
     * Destroy the world and perform cleanup.
     */
    void exitPhysics(){
        int i;

        for (i=0;i<m_tables.size();i++)
        {
            TableRobot* doll = m_tables[i];
            delete doll;
        }

        //cleanup in the reverse order of creation/initialization

        //remove the rigidbodies from the dynamics world and delete them

        for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
        {
            btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState())
            {
                delete body->getMotionState();
            }
            m_dynamicsWorld->removeCollisionObject( obj );
            delete obj;
        }

        //delete collision shapes
        for (int j=0;j<m_collisionShapes.size();j++)
        {
            btCollisionShape* shape = m_collisionShapes[j];
            delete shape;
        }

        //delete dynamics world
        delete m_dynamicsWorld;

        //delete solver
        delete m_solver;

        //delete broadphase
        delete m_broadphase;

        //delete dispatcher
        delete m_dispatcher;

        delete m_collisionConfiguration;
    }

    /**
     * Destructor, simply calls exit Physics.
     */
    virtual ~TableSimulator()
    {
        exitPhysics();
    }

    /**
     * Calls step for all tables in the simulation, then steps the simulation.
     */
    void step(){
        btScalar timeStep = 0.01;
        for (size_t i=0;i<m_tables.size();i++)
        {
            m_tables[i]->step(timeStep);
        }

        m_dynamicsWorld->stepSimulation(timeStep, 1, timeStep);
    }

    /**
     * Calls step and debugDraw if either pause is false or oneStep is true.
     */
    virtual void clientMoveAndDisplay(){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (!pause || (pause && oneStep))
        {
            step();
            m_dynamicsWorld->debugDrawWorld();
            oneStep = false;
        }

        renderme();

        glFlush();

        glutSwapBuffers();
    }

    /**
     * Performs all display operations, including clearing the depth buffers and color buffers.
     */
    virtual void displayCallback(){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        renderme();

        //optional but useful: debug drawing
        if (m_dynamicsWorld)
            m_dynamicsWorld->debugDrawWorld();

        glFlush();
        glutSwapBuffers();
    }

    /**
     * Keyboard callback function.
     *
     * You can press 'p' for pause and 'y' to take one step if the simulation is paused.
     */
    virtual void keyboardCallback(unsigned char key, int x, int y){
        switch (key)
        {
        case 'p':
            {
                    pause = !pause;
                    break;
            }
        case 'y':  //used y because o was also doing something else and was changing the camera angle
            {
                    oneStep = true;
                    break;
            }
        default:
            DemoApplication::keyboardCallback(key, x, y);
        }
    }

    /**
     * Create an instance of the table simulator.
     */
    static DemoApplication* Create()
    {
        TableSimulator* demo = new TableSimulator();
        demo->myinit();
        demo->initPhysics();
        return demo;
    }

    /**
     * Draws the scene and two spheres for orientation.
     */
    virtual void renderme()
    {
        extern GLDebugDrawer gDebugDrawer;
        GlutDemoApplication::renderme();
        gDebugDrawer.drawSphere(btVector3(0., 0., 0.), 0.1, btVector3(0., 0., 0.));
        gDebugDrawer.drawSphere(btVector3(0., 0., 1.), 0.1, btVector3(0., 0., 1.));
        //gDebugDrawer.drawSphere(btVector3(0, 0, 0), 1.9, btVector3(1, 0, 0)); //Creates bigger sphere under robot
    }


    /**
     * Adds a table robot to the world
     */
    TableRobot* addRobot(){
        btVector3 origin(0,0,0);
        TableRobot* table = new TableRobot (m_dynamicsWorld, origin);
        m_tables.push_back(table);
        return table;
    }

    /**
     * Returns the indicated table robot.
     */
    TableRobot* getRobot(size_t index = 0){
        return m_tables[index];
    }

    /**
     * Deletes all robots.
     */
    void deleteRobots(){
        for (size_t i=0;i<m_tables.size();i++)
        {
            TableRobot* table = m_tables[i];
            delete table;
        }
        m_tables.clear();
    }
};


#endif /* TABLESIMULATOR_HPP_ */
