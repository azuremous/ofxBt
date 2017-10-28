#pragma once

#pragma managed(push, off)
#include "btBulletDynamicsCommon.h"
#pragma managed(pop)

namespace ofxBt
{
	class Joint;
}

class ofxBt::Joint
{
public:
	
	Joint() : joint(NULL) {}
	Joint(btGeneric6DofConstraint *joint) : joint(joint) {}
	
	Joint(btDynamicsWorld *m_dynamicsWorld, btRigidBody *obj0, btRigidBody *obj1);

    void setAngularMin(float x, float y, float z);
    void setAngularMax(float x, float y, float z);
    void setLinearMin(float x, float y, float z);
    void setLinearMax(float x, float y, float z);
    
    void setAngularNLinear(btVector3 angularMin, btVector3 angularMax, btVector3 linearMin, btVector3 linearMax);

protected:
	
	btGeneric6DofConstraint *joint;
	
};
