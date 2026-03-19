//This code is based on the Ragdoll benchmark from the bullet3 demo
//Author: Jensen Alexis

/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

// Collision Radius
#define COLLISION_RADIUS 0.0f


///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include <stdio.h>  //printf debugging
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"

#include "mainCanid.h"

//HUD values
btScalar x = 0;
btScalar y = 0;
btScalar z = 0;
btScalar omega = 0;
bool imp = false;
btScalar piedx = -0.67;
btScalar piedy = -0.5;

//Trial parameters
float gainComp = 0;
float gainGoal = 0;
float stick = 0;
float spline = 0;
float gainForce =0;
float filtre = 0;

//Example ragdoll angles
float angles[19 * 3];
class btDynamicsWorld;
//Simulation index for logging
int index = 2108;

extern int iterations;

btRigidBody* body;
btRigidBody* body2;
btRigidBody* ground;

#define NUMRAYS 500
#define USE_PARALLEL_RAYCASTS 1

class btRigidBody;
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

//Trial 2
std::fstream myfile("../../canid/log/Heavy/Classeur4.txt", std::ios_base::in);
std::fstream myfile2("../../canid/log/Heavy/Pelvis.txt", std::ios_base::in);
//Trial 1
std::fstream myfile3("../../canid/log/Medium/Classeur4.txt", std::ios_base::in);
std::fstream myfile4("../../canid/log/Medium/Pelvis.txt", std::ios_base::in);
//Trial 0
std::fstream myfile5("../../canid/log/Classeur4.txt", std::ios_base::in);
std::fstream myfile6("../../canid/log/Pelvis.txt", std::ios_base::in);


#include "../MultiThreadedDemo/CommonRigidBodyMTBase.h"

#include "../RenderingExamples/TimeSeriesCanvas.h"

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

TimeSeriesCanvas* m_timeSeriesCanvas;

class mainCanid : public CommonRigidBodyMTBase
{
	//keep the collision shapes, for deletion/cleanup

	btAlignedObjectArray<RagDoll*> m_ragdolls;
	btAlignedObjectArray<Controller*> m_controller;


	int m_benchmark;

	void myinit()
	{
	}

	void setCameraDistance(btScalar dist)
	{
	}
	void createTest();


public:

	btRigidBody* cube;

	mainCanid(struct GUIHelperInterface* helper, int benchmark)
		: CommonRigidBodyMTBase(helper),
		  m_benchmark(benchmark)
	{

		//For each file ignore static beginning frames
		for (int i = 0; i < 100; i++)
		{
			myfile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile2.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile3.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile4.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile5.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile6.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}

		for (int i = 0; i < 150; i++)
		{
			myfile5.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile6.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}

		for (int i = 0; i < 70; i++)
		{
			myfile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile2.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}

		for (int i = 0; i < 50; i++)
		{
			myfile3.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile4.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}
			
	}
	virtual ~mainCanid()
	{

		exitPhysics();
	}
	void initPhysics();

	void exitPhysics();

	void stepSimulation(float deltaTime);

	void resetCamera()
	{
		float dist = 2;
		float pitch = -0;
		float yaw = 45;
		float targetPos[3] = {0, 1.5, -0} ;
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};


void mainCanid::stepSimulation(float deltaTime)
{
	for (int r = 0; r < m_controller.size(); r++)
	{
		//Compute subject's facing direction
		btTransform facingDir = m_ragdolls[r]->m_bodies[m_ragdolls[r]->BODYPART_PELVIS]->getWorldTransform();
		btVector3 dirX = -facingDir.getBasis().getColumn(0);
		dirX.setY(0);
		dirX.normalize();
		facingDir.setBasis(btMatrix3x3(dirX, btVector3(0, 1, 0), dirX.cross(btVector3(0, 1, 0))));

		//Compute goal angles
		m_controller[r]->process(m_ragdolls[r], deltaTime, cube, facingDir, r==0);

		//If high enough, follow goal angles + jacobian
		if (m_ragdolls[r]->getCOM().y() > 0.8*m_ragdolls[r]->scale/1.75)
			m_ragdolls[r]->update(m_controller[r]->right_leg, m_controller[r]->stepping);
		else
		{
			//Falling diabled state
			for (int i = 0; i < m_ragdolls[r]->BODYPART_COUNT; ++i)
			{
				m_ragdolls[r]->m_bodies[i]->setCollisionFlags(m_ragdolls[r]->m_bodies[i]->CF_HAS_CONTACT_STIFFNESS_DAMPING);
			}
			m_ragdolls[r]->m_bodies[m_ragdolls[r]->BODYPART_RIGHT_FOOT]->setLinearFactor(btVector3(1, 1, 1));
			m_ragdolls[r]->m_bodies[m_ragdolls[r]->BODYPART_LEFT_FOOT]->setLinearFactor(btVector3(1, 1, 1));
			
			for (int i = 0; i < m_ragdolls[r]->JOINT_COUNT; ++i)
			{
				m_ragdolls[r]->kps[i] = m_ragdolls[r]->kps[i]/100;
				m_ragdolls[r]->kds[i] = m_ragdolls[r]->kds[i] / 100;
			}

		}

		
	}


	if (m_ragdolls.size() > 1)
	{
		RagDoll* r = m_ragdolls[1];
		RagDoll* r0 = m_ragdolls[0];


		if (myfile.good() && myfile3.good() && myfile5.good() && myfile4.good())
		{
			//skip first row of headers
			for (int i = 0; i < 1; i++)
			{
				myfile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
				myfile2.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
				myfile3.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
				myfile4.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
				myfile5.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
				myfile6.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			}
				

			
			//Corresponding array column index of every joint
			float coor[r->JOINT_COUNT * 3] =
				{1, 2, 1,      //PELVIS
				 1, 5, 1,      //PELVIS2
				 1, 1, 1,      //HEAD
				 1, 1, 1,      //LHIP
				 1, 1, 1,      //LKNEE
				 25, 23, -24,  //24,26,26, //LPELVIS
				 19, 17, -18,  //18,19,20, //RPELVIS
				 1, 1, 1,      //RHIP
				 1, 1, 1,      //RKNEE
				 1, 1, 1,      //LSHOULDER
				 1, 39, 1,     //LELBOW
				 1, 1, 1,      //30,31,32, //RSHOULDER
				 1, 32, 1,     //RELBOW
				 1, 1, 1,      //RFOOT
				 1, 1, 1,      //LFOOT
				 1, 26, 1,     //LKNEES
				 1, 27, -28,   //LFOOTS (29?)
				 1, 20, 1,     //RKNEES
				 1, 21, -22};  //RFOOTS (23?)
			float a, b, c, d;
			btVector3 pelvis_rot = btVector3(0, 0, 0);

			//Depending on simulation trial index, get different value ofr different files
			if (iterations == 0)
			{
				for (int i = 0; i < 40; i++)
				{
					myfile5 >> a;
					if (i == 1)
						pelvis_rot[0] += a;
					if (i > 1 && i < 8)
						pelvis_rot[(i - 2) % 3] += a;
					for (int j = 0; j < r->JOINT_COUNT * 3; j++)
					{
						if (i == abs(coor[j]) - 1)
						{
							if (myfile5.good())
								angles[j] = a * coor[j] / abs(coor[j]);
						}
					}
				}

				if (myfile5.good())
				{
					angles[0] = 0 * pelvis_rot[2];
					angles[1] = pelvis_rot[0];
					angles[2] = pelvis_rot[1];
				}

				btTransform trans = r->m_bodies[r->BODYPART_PELVIS]->getCenterOfMassTransform();
				myfile6 >> a >> b >> c;
				trans.setOrigin(btVector3(c, b - 0.25, a - 3));

				//printf("%f %f \n", angles[3 * 4 + 1], b);

				myfile6 >> a >> b >> c;
				trans.setRotation(btQuaternion(b + M_PI, c, a - M_PI_2));

				if (myfile5.good())
					r->m_bodies[r->BODYPART_PELVIS]->setWorldTransform(trans);
			}

			if (iterations ==1 )
			{
				for (int i = 0; i < 40; i++)
				{
					myfile3 >> a;
					if (i == 1)
						pelvis_rot[0] += a;
					if (i > 1 && i < 8)
						pelvis_rot[(i - 2) % 3] += a;
					for (int j = 0; j < r->JOINT_COUNT * 3; j++)
					{
						if (i == abs(coor[j]) - 1)
						{
							//angles[j] = 0;
							if (myfile3.good())
							{
								angles[j] = a * coor[j] / abs(coor[j]);
								printf("Angle : %f\n", angles[j]);
							}
						}
					}
				}

				if (myfile3.good())
				{
					angles[0] = 0 * pelvis_rot[2];
					angles[1] = pelvis_rot[0];
					angles[2] = pelvis_rot[1];
				}

				btTransform trans = r->m_bodies[r->BODYPART_PELVIS]->getCenterOfMassTransform();
				myfile4 >> a >> b >> c;
				trans.setOrigin(btVector3(c, b - 0.3, a - 3));

				//printf("%f %f \n", angles[3 * 4 + 1], b);

				myfile4 >> a >> b >> c;
				trans.setRotation(btQuaternion(b + M_PI, c, a - M_PI_2));

				if (myfile3.good())
					r->m_bodies[r->BODYPART_PELVIS]->setWorldTransform(trans);
			
			}

			if (iterations == 2)
			{
				for (int i = 0; i < 40; i++)
				{
					myfile >> a;
					if (i == 1)
						pelvis_rot[0] += a;
					if (i > 1 && i < 8)
						pelvis_rot[(i - 2) % 3] += a;
					for (int j = 0; j < r->JOINT_COUNT * 3; j++)
					{
						if (i == abs(coor[j]) - 1)
						{
							if (myfile.good())
								angles[j] = a * coor[j] / abs(coor[j]);
						}
					}
				}

				if (myfile.good())
				{
					angles[0] = 0 * pelvis_rot[2];
					angles[1] = pelvis_rot[0];
					angles[2] = pelvis_rot[1];
				}

				btTransform trans = r->m_bodies[r->BODYPART_PELVIS]->getCenterOfMassTransform();
				myfile2 >> a >> b >> c;
				trans.setOrigin(btVector3(c, b - 0.3, a - 3));

				//printf("%f %f \n", angles[3 * 4 + 1], b);

				myfile2 >> a >> b >> c;
				trans.setRotation(btQuaternion(b + M_PI, c, a - M_PI_2));

				if (myfile.good())
					r->m_bodies[r->BODYPART_PELVIS]->setWorldTransform(trans);
			}

			
			//Apply angles for all relevant joints (avoiding duplicates)
			for (int i = 0; i < r->JOINT_COUNT; i++)
			{
				r->m_joints[i]->getRigidBodyA().setLinearFactor(btVector3(1, 1, 1));
				btTransform tempTrans = r->m_joints[i]->getRigidBodyA().getWorldTransform();
				if (i != 9 && i != 11)
					tempTrans = tempTrans * r->m_joints[i]->getFrameOffsetA() * btTransform(btQuaternion(angles[3 * i], angles[3 * i + 1], angles[3 * i + 2])) * r->m_joints[i]->getFrameOffsetB().inverse();
				else if (i == 9 || i == 11)
					tempTrans = tempTrans * r->m_joints[i]->getFrameOffsetA() * btTransform(btQuaternion(angles[3 * i], angles[3 * i + 1], angles[3 * i + 2])) * r->m_joints[i]->getFrameOffsetB().inverse();
				else
					tempTrans = tempTrans * r->m_joints[i]->getFrameOffsetA() * btTransform(btQuaternion(0, 0, 0)) * r->m_joints[i]->getFrameOffsetB().inverse();

				if (r->m_joints[i]->getRigidBodyB().getWorldArrayIndex() != r->m_bodies[r->BODYPART_PELVIS]->getWorldArrayIndex() && i != 7 && i != 8 && i != 4 && i != 3)
					r->m_joints[i]->getRigidBodyB().setWorldTransform(tempTrans);
			}
		}

		//Camera setting for two subjects
		float iter_yaw[] = {55, 55, 55, 55, 55, 55, 90, 90, 90, 45, 45, 45, 45,90,60,60,60};
		float iter_pitch[] = {0, 0, 0, 0, 0, 0, -50, -50, -50, 0, 0, 0, 0,0,0,0};

		float dist = 2.5;
		float targetPosi[3] = {(m_ragdolls[0]->getCOM().x() + r->getCOM().x())/2, 1, (m_ragdolls[0]->getCOM().z() + r->getCOM().z())/2};
		float targetPos[3] = {m_ragdolls[0]->getCOM().x(), 1, m_ragdolls[0]->getCOM().z()};
		m_guiHelper->resetCamera(dist, iter_yaw[iterations], iter_pitch[iterations], targetPosi[0], targetPosi[1], targetPosi[2]);

	}
	else
	{
		//Camera setting for one subject
		float iter_yaw[] = {45, -85, -85, 45, 45, 45, 0, 0, 0, 45, 45, 45, 45,90,55,55,55,55,55,55,55,55,55};
		float iter_pitch[] = {0, 0, -30, 0, 0, 0, -70, -70, -70, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0};

		float dist = 2;
		float targetPosi[3] = {m_ragdolls[0]->getCOM().x(), 1, m_ragdolls[0]->getCOM().z()};
		m_guiHelper->resetCamera(dist, iter_yaw[iterations], iter_pitch[iterations], targetPosi[0], targetPosi[1], targetPosi[2]);
	
	}

	
	//Manage simulation color palette
	double color3[4] = {0.9, 0.9, 0.9, 1};
	m_guiHelper->setBackgroundColor(color3);
	double color[4] = {0.5, 0.75, 1.0, 1.0};
	double color4[4] = {0.3, 0.8, 0.3, 1.0};
	double* colors[2] = {color, color4};
	for (int i = 0; i < m_ragdolls[0]->BODYPART_COUNT;i++)
		for(int j=0; j< m_ragdolls.size() ;j++)
			m_guiHelper->changeRGBAColor(m_ragdolls[j]->m_bodies[i]->getUserIndex(), colors[j]);
	double color2[4] = {0.8, 0.8, 0.8, 1};
	m_guiHelper->changeRGBAColor(ground->getUserIndex(), color2);

	
	//Call the simulation physics loop integration
	b3MouseButtonCallback(0);
	if (m_dynamicsWorld)
		{
			m_dynamicsWorld->stepSimulation(deltaTime);
			m_dynamicsWorld->getSolverInfo();
		}
};


static void boolPtrButtonCallback(int buttonId, bool buttonState, void* userPointer)
{
	if (bool* val = static_cast<bool*>(userPointer))
	{
		*val = buttonState;
	}
}


void mainCanid::initPhysics()
{
	//Dybamic world initialization
	progress++;
	m_guiHelper->setUpAxis(1);
	setCameraDistance(btScalar(100.));
	m_solverType = SOLVER_TYPE_MLCP_DANTZIG;
	createEmptyDynamicsWorld();
	m_dynamicsWorld->setApplySpeculativeContactRestitution(false);
	m_dynamicsWorld->getSolverInfo().m_friction = 1.0; 
	m_dynamicsWorld->getSolverInfo().m_frictionCFM = 0.0; 
	m_dynamicsWorld->getSolverInfo().m_frictionERP = 1.0; 
	m_dynamicsWorld->getSolverInfo().m_restitutionVelocityThreshold = 0.0;
	m_dynamicsWorld->getSolverInfo().m_numNonContactInnerIterations = 200;  //few solver iterations
	m_dynamicsWorld->getSolverInfo().m_numIterations = 200;  //few solver iterations
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	//Add all HUD buttons
	{
		SliderParams slider("x", &x);
		slider.m_minVal = -M_PI;
		slider.m_maxVal = M_PI;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		SliderParams slider("y", &y);
		slider.m_minVal = -M_PI;
		slider.m_maxVal = M_PI;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		SliderParams slider("z", &z);
		slider.m_minVal = -M_PI;
		slider.m_maxVal = M_PI;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	{
		ButtonParams button("Impulse", 0, true);
		bool* ptr = &imp;
		button.m_initialState = *ptr;
		button.m_userPointer = ptr;
		button.m_callback = boolPtrButtonCallback;
		m_guiHelper->getParameterInterface()->registerButtonParameter(button);

	}

	{
		SliderParams slider("Omega", &omega);
		slider.m_minVal = 0;
		slider.m_maxVal = 30;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	{
		SliderParams slider("Piedx", &piedx);
		slider.m_minVal = -M_PI;
		slider.m_maxVal = M_PI;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}
	{
		SliderParams slider("Piedy", &piedy);
		slider.m_minVal = -M_PI_2;
		slider.m_maxVal = M_PI_2;
		if (m_guiHelper->getParameterInterface())
			m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
	}

	m_dynamicsWorld->setGravity(btVector3(0, -9.8, 0));

		///create a few basic rigid bodies
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(250.), btScalar(10.), btScalar(250.)));
		//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),0);

		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -10, 0));

		//We can also use DemoApplication::createRigidBody, but for clarity it is provided here:
		{
			btScalar mass(100000.);
			mass = 0.f;
			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0, 0, 0);
			if (isDynamic)
				groundShape->calculateLocalInertia(mass, localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
			btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
			rbInfo.m_friction = 1;
			rbInfo.m_rollingFriction = 1.0;
			rbInfo.m_spinningFriction = 1.0;
			rbInfo.m_restitution = 0.9;
			ground = new btRigidBody(rbInfo);

			//add the body to the dynamics world
			ground->setFriction(1);
			ground->setLinearFactor(btVector3(0, 0, 0));
			ground->setAngularFactor(btVector3(0, 0, 0));

			m_dynamicsWorld->addRigidBody(ground);
		}

	createTest();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

}


void mainCanid::createTest()
{
	
	//Offset to skip empty motion start
	for (int i = 0; i < 150; i++)
	{
			myfile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile2.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile3.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile4.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile5.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			myfile6.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
			
	//Reset output files
	ofstream ofs;
	ofs.open("../../canid/log/output" + std::to_string(index) + ".txt", std::ofstream::out | std::ofstream::trunc);
	ofs.close();
	ofs.open("../../canid/log/output" + std::to_string(index) + ".txt");
	ofs << "q torque X Z Y vX vZ pasEffectue \n";
	ofs.close();

	ofstream ofs2;
	ofs2.open("../../canid/log/torques" + std::to_string(index) + ".txt", std::ofstream::out | std::ofstream::trunc);
	ofs2.close();
	ofs2.open("../../canid/log/torques" + std::to_string(index) + ".txt");
	for (int i =0;i<12;i++)
			ofs2 << "q" << i << " qdot" << i << " tau" << i << " ";
	ofs2 << "\n";
	ofs2.close();



	//Initialize base subject ragdoll
	btScalar scale(1.75);
	btVector3 pos(0,1,0);
	int agents = 1;
	for (int i = 0; i < agents; i++)
	{
		RagDoll* ragDoll = new RagDoll(m_dynamicsWorld, pos, scale);
		m_ragdolls.push_back(ragDoll);
		Controller* controller = new Controller(0, false);
		m_controller.push_back(controller);
		pos[2] -= 0.6;
	}


	//Add a reference ragdoll if necessary
	if (iterations < 3)
	{
		RagDoll* ragDoll3 = new RagDoll(m_dynamicsWorld, btVector3(1+1,-0.2,-3), scale);
		for (int i = 0; i < ragDoll3->BODYPART_COUNT; i++)
		{
			ragDoll3->m_bodies[i]->setActivationState(5);
			ragDoll3->m_bodies[i]->setGravity(btVector3(0, 0, 0));
			ragDoll3->m_bodies[i]->setCollisionFlags(ragDoll3->m_bodies[i]->CF_NO_CONTACT_RESPONSE);
		}
		for (int i = 0; i < ragDoll3->JOINT_COUNT; i++)
		{
			ragDoll3->m_joints[i]->setEnabled(false);
		}
		m_ragdolls.push_back(ragDoll3);
	}
	

	//Add a cube to the scene
	btBoxShape* blockShape = new btBoxShape(btVector3(0.03f - COLLISION_RADIUS, 0.03f - COLLISION_RADIUS, 0.5f - COLLISION_RADIUS));
	btTransform trans;
	trans.setIdentity();
	float mass = 0.f;
	btVector3 localInertia(0, 0, 0);
	blockShape->calculateLocalInertia(mass, localInertia);
	pos[1] = 4.f;
	pos[2] = 20.f;
	trans.setOrigin(pos);
	cube = createRigidBody(mass, trans, blockShape);
	cube->setActivationState(DISABLE_DEACTIVATION);
	cube->setCollisionFlags(cube->CF_NO_CONTACT_RESPONSE);


	//Initialize trial's push angle and force TODO leg detection
	float iter_y[] = {0, 0, 0, 0,0,0,M_PI_4, -M_PI_2, 0.8*M_PI,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	float iter_x[] = {66, 78, 148,300,150,150, 150, 150, 150,120,120,120,120,120,
		50,70,70,80,90,90,100,120,140};
	float iter_t[] = {0.5, 0.66, 0.74, 1.2, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7,
		0.8,0.6, 0.6, 0.75,0.8,0.8,1.2,0.8,0.6};
	y = iter_y[iterations];
	m_ragdolls[0]->force = iter_x[iterations];
	m_ragdolls[0]->time = iter_t[iterations];
	m_controller[0]->right_leg = iter_y[iterations]>=0;

}


void mainCanid::exitPhysics()
{

	myfile.clear();
	myfile.seekg(0);
	myfile2.clear();
	myfile2.seekg(0);

	myfile3.clear();
	myfile3.seekg(0);
	myfile4.clear();
	myfile4.seekg(0);

	myfile5.clear();
	myfile5.seekg(0);
	myfile6.clear();
	myfile6.seekg(0);

	int i;

	for (i = 0; i < m_ragdolls.size(); i++)
	{
		RagDoll* doll = m_ragdolls[i];
		delete doll;
	}
	m_ragdolls.clear();

	for (i = 0; i < m_controller.size(); i++)
	{
		Controller* doll = m_controller[i];
		delete doll;
	}
	m_controller.clear();

	CommonRigidBodyMTBase::exitPhysics();


	//m_timeSeriesCanvas->~TimeSeriesCanvas();
}

CommonExampleInterface* CanidCreateFunc(struct CommonExampleOptions& options)
{
	return new mainCanid(options.m_guiHelper, options.m_option);
}
