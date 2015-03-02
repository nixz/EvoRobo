/*
 * TableRobot.hpp
 *
 *  Created on: Jan 23, 2015
 *      Author: Joost Huizinga
 */

#ifndef TABLEROBOT_HPP_
#define TABLEROBOT_HPP_

#include "TouchSensor.hpp"

/**
 * A simple, table like robot.
 */
class TableRobot
{
public:
    enum
    {
        BODYPART_BODY = 0,

        BODYPART_UPPER_LEG_1,
        BODYPART_UPPER_LEG_2,
        BODYPART_UPPER_LEG_3,
        BODYPART_UPPER_LEG_4,

        BODYPART_LOWER_LEG_1,
        BODYPART_LOWER_LEG_2,
        BODYPART_LOWER_LEG_3,
        BODYPART_LOWER_LEG_4,

        BODYPART_COUNT
    };

    enum
    {
        JOINT_BODY_UPPER_LEG_1 = 0,
        JOINT_BODY_UPPER_LEG_2,
        JOINT_BODY_UPPER_LEG_3,
        JOINT_BODY_UPPER_LEG_4,

        JOINT_KNEE_LEG_1,
        JOINT_KNEE_LEG_2,
        JOINT_KNEE_LEG_3,
        JOINT_KNEE_LEG_4,

        JOINT_COUNT
    };

    enum
    {
        SENSOR_LOWER_LEG_1 = 0,
        SENSOR_LOWER_LEG_2,
        SENSOR_LOWER_LEG_3,
        SENSOR_LOWER_LEG_4,

        SENSOR_COUNT
    };

private:
    btDynamicsWorld* m_ownerWorld;
    btCollisionShape* m_shapes[BODYPART_COUNT];
    btRigidBody* m_bodies[BODYPART_COUNT];
    btHingeConstraint* m_joints[JOINT_COUNT];
    TouchSensor* m_sensors[SENSOR_COUNT];
    double m_motorMaxImpulse;
    double m_motorSpeedModifier;

    /**
     * Creates a rigid body with the supplied mass, shape and transform.
     *
     * Helper for function for creating the body parts of the robot.
     */
    btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
    {
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0,0,0);
        if (isDynamic)
            shape->calculateLocalInertia(mass,localInertia);

        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        m_ownerWorld->addRigidBody(body);

        return body;
    }

public:
    /**
     * Constructor, creates a table robot and adds it to the supplied world at the supplied offset.
     */
    TableRobot (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
        : m_ownerWorld (ownerWorld)
    {
        //Set motor speed and impulse
        m_motorMaxImpulse = 4;
        m_motorSpeedModifier = 10;

        // Setup the geometry
        m_shapes[BODYPART_BODY] = new btBoxShape(btVector3(1, 0.2, 1));
        m_shapes[BODYPART_UPPER_LEG_1] = new btCylinderShapeX(btVector3(1, 0.2, 0.2));
        m_shapes[BODYPART_UPPER_LEG_2] = new btCylinderShapeX(btVector3(1, 0.2, 0.2));
        m_shapes[BODYPART_UPPER_LEG_3] = new btCylinderShapeZ(btVector3(0.2, 0.2, 1));
        m_shapes[BODYPART_UPPER_LEG_4] = new btCylinderShapeZ(btVector3(0.2, 0.2, 1));
        m_shapes[BODYPART_LOWER_LEG_1] = new btCylinderShape(btVector3(0.2, 1, 0.2));
        m_shapes[BODYPART_LOWER_LEG_2] = new btCylinderShape(btVector3(0.2, 1, 0.2));
        m_shapes[BODYPART_LOWER_LEG_3] = new btCylinderShape(btVector3(0.2, 1, 0.2));
        m_shapes[BODYPART_LOWER_LEG_4] = new btCylinderShape(btVector3(0.2, 1, 0.2));

        //Create sensors
        for(int i=0; i<SENSOR_COUNT; ++i){
            m_sensors[i] = new TouchSensor();
        }

        // Setup all the rigid bodies
        btTransform offset;
        offset.setIdentity();
        offset.setOrigin(positionOffset);

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(0, 2, 0));
        m_bodies[BODYPART_BODY] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_BODY]);
        m_bodies[BODYPART_BODY]->setUserPointer(0);

        transform.setIdentity();
        transform.setOrigin(btVector3(2.0, 2.0, 0.0));
        m_bodies[BODYPART_UPPER_LEG_1] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_UPPER_LEG_1]);
        m_bodies[BODYPART_UPPER_LEG_1]->setUserPointer(0);

        transform.setIdentity();
        transform.setOrigin(btVector3(-2, 2, 0));
        m_bodies[BODYPART_UPPER_LEG_2] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_UPPER_LEG_2]);
        m_bodies[BODYPART_UPPER_LEG_2]->setUserPointer(0);

        transform.setIdentity();
        transform.setOrigin(btVector3(0, 2, 2));
        m_bodies[BODYPART_UPPER_LEG_3] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_UPPER_LEG_3]);
        m_bodies[BODYPART_UPPER_LEG_3]->setUserPointer(0);

        transform.setIdentity();
        transform.setOrigin(btVector3(0, 2, -2));
        m_bodies[BODYPART_UPPER_LEG_4] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_UPPER_LEG_4]);
        m_bodies[BODYPART_UPPER_LEG_4]->setUserPointer(0);

        transform.setIdentity();
        transform.setOrigin(btVector3(3, 1, 0));
        m_bodies[BODYPART_LOWER_LEG_1] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LOWER_LEG_1]);
        m_bodies[BODYPART_LOWER_LEG_1]->setUserPointer(m_sensors[SENSOR_LOWER_LEG_1]);

        transform.setIdentity();
        transform.setOrigin(btVector3(-3, 1, 0));
        m_bodies[BODYPART_LOWER_LEG_2] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LOWER_LEG_2]);
        m_bodies[BODYPART_LOWER_LEG_2]->setUserPointer(m_sensors[SENSOR_LOWER_LEG_2]);

        transform.setIdentity();
        transform.setOrigin(btVector3(0, 1, 3));
        m_bodies[BODYPART_LOWER_LEG_3] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LOWER_LEG_3]);
        m_bodies[BODYPART_LOWER_LEG_3]->setUserPointer(m_sensors[SENSOR_LOWER_LEG_3]);

        transform.setIdentity();
        transform.setOrigin(btVector3(0, 1, -3));
        m_bodies[BODYPART_LOWER_LEG_4] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LOWER_LEG_4]);
        m_bodies[BODYPART_LOWER_LEG_4]->setUserPointer(m_sensors[SENSOR_LOWER_LEG_4]);

        // Setup some damping on the m_bodies
        for (int i = 0; i < BODYPART_COUNT; ++i)
        {
            m_bodies[i]->setDamping(0.05, 0.85);
            m_bodies[i]->setDeactivationTime(0.0f);
        }

        // Now setup the constraints
        btVector3 pivotInA;
        btVector3 pivotInB;
        btVector3 axis;

        pivotInA.setValue(1, 0, 0);
        pivotInB.setValue(-1, 0, 0);
        axis.setValue(0, 0, 1);
        m_joints[JOINT_BODY_UPPER_LEG_1] = new btHingeConstraint(*m_bodies[BODYPART_BODY], *m_bodies[BODYPART_UPPER_LEG_1], pivotInA, pivotInB, axis, axis);
        m_joints[JOINT_BODY_UPPER_LEG_1]->setLimit(-45.*3.14159 / 180., 45.*3.14159 / 180.);
        m_ownerWorld->addConstraint(m_joints[JOINT_BODY_UPPER_LEG_1], true);

        pivotInA.setValue(-1, 0, 0);
        pivotInB.setValue(1, 0, 0);
        axis.setValue(0, 0, -1);
        m_joints[JOINT_BODY_UPPER_LEG_2] = new btHingeConstraint(*m_bodies[BODYPART_BODY], *m_bodies[BODYPART_UPPER_LEG_2], pivotInA, pivotInB, axis, axis);
        m_joints[JOINT_BODY_UPPER_LEG_2]->setLimit(-45.*3.14159 / 180., 45.*3.14159 / 180.);
        m_ownerWorld->addConstraint(m_joints[JOINT_BODY_UPPER_LEG_2], true);

        pivotInA.setValue(0, 0, 1);
        pivotInB.setValue(0, 0, -1);
        axis.setValue(-1, 0, 0);
        m_joints[JOINT_BODY_UPPER_LEG_3] = new btHingeConstraint(*m_bodies[BODYPART_BODY], *m_bodies[BODYPART_UPPER_LEG_3], pivotInA, pivotInB, axis, axis);
        m_joints[JOINT_BODY_UPPER_LEG_3]->setLimit(-45.*3.14159 / 180., 45.*3.14159 / 180.);
        m_ownerWorld->addConstraint(m_joints[JOINT_BODY_UPPER_LEG_3], true);

        pivotInA.setValue(0, 0, -1);
        pivotInB.setValue(0, 0, 1);
        axis.setValue(1, 0, 0);
        m_joints[JOINT_BODY_UPPER_LEG_4] = new btHingeConstraint(*m_bodies[BODYPART_BODY], *m_bodies[BODYPART_UPPER_LEG_4], pivotInA, pivotInB, axis, axis);
        m_joints[JOINT_BODY_UPPER_LEG_4]->setLimit(-45.*3.14159 / 180., 45.*3.14159 / 180.);
        m_ownerWorld->addConstraint(m_joints[JOINT_BODY_UPPER_LEG_4], true);

        pivotInA.setValue(1, 0, 0);
        pivotInB.setValue(0, 1, 0);
        axis.setValue(0, 0, -1);
        m_joints[JOINT_KNEE_LEG_1] = new btHingeConstraint(*m_bodies[BODYPART_UPPER_LEG_1], *m_bodies[BODYPART_LOWER_LEG_1], pivotInA, pivotInB, axis, axis);
        m_joints[JOINT_KNEE_LEG_1]->setLimit(-45.*3.14159 / 180., 45.*3.14159 / 180.);
        m_ownerWorld->addConstraint(m_joints[JOINT_KNEE_LEG_1], true);

        pivotInA.setValue(-1, 0, 0);
        pivotInB.setValue(0, 1, 0);
        axis.setValue(0, 0, 1);
        m_joints[JOINT_KNEE_LEG_2] = new btHingeConstraint(*m_bodies[BODYPART_UPPER_LEG_2], *m_bodies[BODYPART_LOWER_LEG_2], pivotInA, pivotInB, axis, axis);
        m_joints[JOINT_KNEE_LEG_2]->setLimit(-45.*3.14159 / 180., 45.*3.14159 / 180.);
        m_ownerWorld->addConstraint(m_joints[JOINT_KNEE_LEG_2], true);

        pivotInA.setValue(0, 0, 1);
        pivotInB.setValue(0, 1, 0);
        axis.setValue(-1, 0, 0);
        m_joints[JOINT_KNEE_LEG_3] = new btHingeConstraint(*m_bodies[BODYPART_UPPER_LEG_3], *m_bodies[BODYPART_LOWER_LEG_3], pivotInA, pivotInB, axis, axis);
        m_joints[JOINT_KNEE_LEG_3]->setLimit(-45.*3.14159 / 180., 45.*3.14159 / 180.);
        m_ownerWorld->addConstraint(m_joints[JOINT_KNEE_LEG_3], true);

        pivotInA.setValue(0, 0, -1);
        pivotInB.setValue(0, 1, 0);
        axis.setValue(1, 0, 0);
        m_joints[JOINT_KNEE_LEG_4] = new btHingeConstraint(*m_bodies[BODYPART_UPPER_LEG_4], *m_bodies[BODYPART_LOWER_LEG_4], pivotInA, pivotInB, axis, axis);
        m_joints[JOINT_KNEE_LEG_4]->setLimit(-45.*3.14159 / 180., 45.*3.14159 / 180.);
        m_ownerWorld->addConstraint(m_joints[JOINT_KNEE_LEG_4], true);
    }

    /**
     * Destructor, removes all parts of the robot from the world and destroys them.
     */
    virtual ~TableRobot ()
    {
        int i;

        // Delete sensors
        for( i = 0; i < SENSOR_COUNT; ++i){
            delete m_sensors[i];
            m_sensors[i] = 0;
        }

        // Remove all constraints
        for ( i = 0; i < JOINT_COUNT; ++i)
        {
            m_ownerWorld->removeConstraint(m_joints[i]);
            delete m_joints[i];
            m_joints[i] = 0;
        }

        // Remove all bodies and shapes
        for ( i = 0; i < BODYPART_COUNT; ++i)
        {
            m_ownerWorld->removeRigidBody(m_bodies[i]);

            delete m_bodies[i]->getMotionState();
            delete m_bodies[i];
            m_bodies[i] = 0;
            delete m_shapes[i];
            m_shapes[i] = 0;
        }
    }


    /**
     * Returns the center of mass position for the main body of the robot.
     */
    btVector3 getPosition(){
        return m_bodies[BODYPART_BODY]->getCenterOfMassPosition();
    }

    /**
     * Resets all sensors of the robot.
     */
    void resetSensors(){
        for(int i = 0; i < SENSOR_COUNT; ++i){
            m_sensors[i]->reset();
        }
    }

    /**
     * Actuates the joint at index jointIndex towards the desired angle.
     *
     * Note that the joint will only move towards the desired angle,
     * there is no guarantee that the actuator will reach that angle in a single time-step.
     */
    void ActuateJoint(int jointIndex, double desiredAngle)
    {
        double diff = desiredAngle*3.14159 / 180 - m_joints[jointIndex]->getHingeAngle();
        m_joints[jointIndex]->enableAngularMotor(true, m_motorSpeedModifier*diff, m_motorMaxImpulse);
    }

    /**
     * Performs one step of the robot.
     *
     * This function will be used to actuate the robot in assignment 5.
     */
    void step(double timeStep){
        /* TODO YOUR CODE HERE IN ASSIGNMENT 5 */

        // Reset sensors should always be the last step
        resetSensors();
    }

};



#endif /* TABLEROBOT_HPP_ */
