#include "ofxBtJoint.h"

using namespace ofxBt;

Joint::Joint(btDynamicsWorld *m_dynamicsWorld, btRigidBody *obj0, btRigidBody *obj1)
{
    btVector3 pickPosA = obj0->getCenterOfMassTransform().getOrigin();
    btVector3 localPivotA = obj0->getCenterOfMassTransform().inverse() * pickPosA;
    
    btTransform frameInA;
    frameInA.setIdentity();
    frameInA.setOrigin(localPivotA);
    
    btVector3 pickPosB = obj1->getCenterOfMassTransform().getOrigin();
    btVector3 localPivotB = obj1->getCenterOfMassTransform().inverse() * pickPosB;
    
    btTransform frameInB;
    frameInB.setIdentity();
    frameInB.setOrigin(localPivotB);
	
	joint = new btGeneric6DofConstraint(*obj0, *obj1, frameInA, frameInB, true);
    m_dynamicsWorld->addConstraint(joint, true);
//
    setAngularMin(0, 0, 0);
    setAngularMax(100, 100, 100);
    setLinearMin(0.5, 0.5, 0.5);
    setLinearMax(10.0, 10.0, 10.0);
//
}

//--------------------------------------------------------------
void Joint::setAngularMin(float x, float y, float z){
    joint->setAngularLowerLimit(btVector3(x, y, y));
}

//--------------------------------------------------------------
void Joint::setAngularMax(float x, float y, float z){
    joint->setAngularUpperLimit(btVector3(x, y, z));
}

//--------------------------------------------------------------
void Joint::setLinearMin(float x, float y, float z){
    joint->setLinearLowerLimit(btVector3(x, y, z));
}

//--------------------------------------------------------------
void Joint::setLinearMax(float x, float y, float z){
    joint->setLinearUpperLimit(btVector3(x, y, z));
}

//--------------------------------------------------------------
void Joint::setAngularNLinear(btVector3 angularMin, btVector3 angularMax, btVector3 linearMin, btVector3 linearMax){
    joint->setAngularLowerLimit(angularMin);
    joint->setAngularUpperLimit(angularMax);
    joint->setLinearLowerLimit(linearMin);
    joint->setLinearUpperLimit(linearMax);
}
