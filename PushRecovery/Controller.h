// Collision Radius
#define COLLISION_RADIUS 0.0f

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include <stdio.h>  //printf debugging
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"



#include "../MultiThreadedDemo/CommonRigidBodyMTBase.h"

#include "../RenderingExamples/TimeSeriesCanvas.h"
#include "RagDoll.h"
#include "ConvexHull.h"
#include <iostream>
#include <fstream>


// Enrico: Shouldn't these three variables be real constants and not defines?
#ifndef M_PI
#define M_PI btScalar(3.14159265358979323846)
#endif

#ifndef M_PI_2
#define M_PI_2 btScalar(1.57079632679489661923)
#endif

#ifndef M_PI_4
#define M_PI_4 btScalar(0.785398163397448309616)
#endif

extern btScalar x;
extern btScalar y;
extern btScalar z;
extern btScalar omega;
extern bool imp;
extern btScalar piedx;
extern btScalar piedy;

extern TimeSeriesCanvas* m_timeSeriesCanvas;
extern btRigidBody* body;
extern btRigidBody* body2;

extern int index;

extern float gainComp;
extern float gainGoal;
extern float stick;
extern float spline;
extern float gainForce;
extern float filtre;

class Controller
{
public:
	
	enum
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_TORSO,
		JOINT_SPINE_HEAD,

		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,

		JOINT_LEFT_PELVIS,
		JOINT_RIGHT_PELVIS,

		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,

		JOINT_RIGHT_FOOT,
		JOINT_LEFT_FOOT,

		JOINT_LEFT_KNEE_SWING,
		JOINT_LEFT_FOOT_SWING,

		
		JOINT_RIGHT_KNEE_SWING,
		JOINT_RIGHT_FOOT_SWING,

		JOINT_COUNT
	};

	btVector3 wave[JOINT_COUNT];
	btVector3 stand[JOINT_COUNT];
	btVector3 base[JOINT_COUNT];
	btVector3 base2[JOINT_COUNT];
	btVector3 left[JOINT_COUNT];
	btVector3 stand2[JOINT_COUNT];
	btVector3 right[JOINT_COUNT];

	//Goal foot position
	float xd;
	float zd;

	//Feet convex hull
	ConvexHull convexHull =  ConvexHull();


	bool stepping = false;//Already in a waling state
	bool pasEffectue = false;//Logging purpose: enable if a step has been done since beginning of trial

	float p;//Phi parameter
	bool right_leg;//Current swinging leg
	float stepTime = 0.0f;//Duration fo the step
	bool unbalanced = false;//Step detection
	bool first = false;//First step of the movement or not
	bool lastStep = false;//Last step of the movement or not

	//Initialize the walk planner, with current swing leg and current progress
	Controller(float start, bool right)
	{
		p = start;
		right_leg = right;
	}
	float buttonTime = -0.05f;//Button timer start value

	void fit(RagDoll* r, bool rightleg, float progress, bool near, btRigidBody* cube, btTransform facingDir);

	btVector3 Controller::solve(const btVector3& p1, const btVector3& p2, const btVector3& n, double r1, double r2);

	bool Controller::isBalanced(RagDoll* r,float deltatime);

	void Controller::process(RagDoll* ragdoll, float deltatime, btRigidBody* cube, btTransform facingDir, bool pushed);
};

