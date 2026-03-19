//TODO Change reference to character relative

// Collision Radius
#define COLLISION_RADIUS 0.0f


#ifndef M_PI
#define M_PI btScalar(3.14159265358979323846)
#endif

#ifndef M_PI_2
#define M_PI_2 btScalar(1.57079632679489661923)
#endif

#ifndef M_PI_4
#define M_PI_4 btScalar(0.785398163397448309616)
#endif

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"

#include "../CommonInterfaces/CommonParameterInterface.h"

#include <iostream>
#include <chrono>
#include <vector>


#include "BussIK/Node.h"
#include "BussIK/Tree.h"
#include "BussIK/Jacobian.h"
#include "BussIK/VectorRn.h"

#include <iostream>
#include <fstream>

#ifndef RAGDOLL_H
#define RAGDOLL_H

//HUD parameters of the same name, sliders
extern btScalar x;
extern btScalar y;
extern btScalar z;


//From mainCanid: ID of the simulation (for file management)
extern int index;

//From mainCanid: set of trial parameters
extern float gainComp;
extern float gainGoal;
extern float stick;
extern float spline;
extern float gainForce;
extern float filtre;

//Current trial iteration
extern int iterations;

static int progress = 0;

class RagDoll
{
public:

	btScalar total_weight = 70;
	btScalar scale = 1.75;
	bool isWalking = false;
	bool left = true;

	enum
	{
		BODYPART_PELVIS = 0,
		BODYPART_SPINE,
		BODYPART_TORSO,
		BODYPART_HEAD,

		BODYPART_LEFT_UPPER_LEG,
		BODYPART_LEFT_LOWER_LEG,

		BODYPART_RIGHT_UPPER_LEG,
		BODYPART_RIGHT_LOWER_LEG,

		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,

		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,

		BODYPART_RIGHT_FOOT,
		BODYPART_LEFT_FOOT,

		BODYPART_COUNT
	};

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
	int kds[JOINT_COUNT];
	int kps[JOINT_COUNT];
	float force;
	float time;

	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btGeneric6DofSpring2Constraint* m_joints[JOINT_COUNT];
	btScalar m_parent[JOINT_COUNT];
	btVector3 m_stance[JOINT_COUNT];
	btTransform facingDir;
	btScalar m_scales[BODYPART_COUNT][3];
	btScalar m_weight[BODYPART_COUNT][7];
	int temp = 0;

	btScalar logging[JOINT_COUNT][3][3] = {}; 

	btRigidBody* createRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{ 
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setActivationState(DISABLE_DEACTIVATION);
		body->setFlags(body->CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR);
		body->setCustomDebugColor(btVector3(1.0f, 0.0f, 0.0f));

		m_ownerWorld->addRigidBody(body);

		return body;
	}

	RagDoll(btDynamicsWorld* ownerWorld, btVector3& positionOffset, btScalar scale)
		: m_ownerWorld(ownerWorld)
	{

		std::fstream myfile("../../canid/log/inputOpti.txt", std::ios_base::in);//First set of body parameters

		//Trial parameters
		float iter_weight[] = {70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 90, 40, 110, 70, 70, 70, 70, 70, 70, 70,70,70,70,70,70,70};
		float iter_scale[] = {1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.2, 2.1, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75,1.75,1.75,1.75,1.75,1.75,1.75,1.75};


		total_weight = iter_weight[iterations];
		this->scale = iter_scale[iterations];
		scale = this->scale;
		scale = scale * 0.5; //For computations, scale is halved
		float a,b; //Temporary parameters for file input reading
		
		
		for (int i =0;i<JOINT_COUNT;i++)
		{
			myfile >> a >> b;
			kps[i] = a * (1+ 0.6*(this->total_weight-70.0)/70.0);//Scaling parameters for weight
			kds[i] = b * (1 + 0.8*(this->total_weight - 70.0)/70.0);

			if (iterations == 4) //Failure case of not enough gain
			{
				kps[i] *= 0.6;
				kds[i] *= 0.6;
			}
			if (iterations == 5) //Failure case of too much gain
			{
				kps[i] *= 2.2;
				kds[i] *= 2.2;
			}
		}

		myfile.close();

		std::fstream myfile2("../../canid/log/resultatTraj.txt", std::ios_base::in);//Second set of model parameters


	
		myfile2 >> a;
		gainComp = a;//Compensation jacobian
		myfile2 >> a;
		gainGoal = a;//CoM driving jacobian
		myfile2 >> a;
		stick = a;//Ground stickyness
		myfile2 >> a;
		spline = a;//Foot height
		myfile2 >> a;
		gainForce = a;//Simulation force gain
		myfile2 >> a;
		filtre = a;//StepTime filter
		myfile2 >> a >> b;
		force = a;//Maximum push force
		time = b;//Push duration

		myfile2.close();

		

		// Setup the geometry; data from Winter's table
		{

			m_scales[BODYPART_PELVIS][0] = btScalar(0.191/2);
			m_scales[BODYPART_PELVIS][1] = btScalar(0.045);
			m_scales[BODYPART_PELVIS][2] = btScalar(0.07);

			m_scales[BODYPART_SPINE][0] = btScalar(0.174/2);
			m_scales[BODYPART_SPINE][1] = btScalar(0.288)/2;
			m_scales[BODYPART_SPINE][2] = btScalar(0.08);

			m_scales[BODYPART_TORSO][0] = btScalar(0.28 / 2);
			m_scales[BODYPART_TORSO][1] = btScalar(0.288)/2;
			m_scales[BODYPART_TORSO][2] = btScalar(0.08);

			m_scales[BODYPART_HEAD][0] = btScalar(0.1);
			m_scales[BODYPART_HEAD][1] = btScalar(0.13);
			m_scales[BODYPART_HEAD][2] = btScalar(0.02);

			m_scales[BODYPART_LEFT_UPPER_LEG][0] = btScalar(0.05);
			m_scales[BODYPART_LEFT_UPPER_LEG][1] = btScalar(0.25);
			m_scales[BODYPART_LEFT_UPPER_LEG][2] = btScalar(0.06);

			m_scales[BODYPART_LEFT_LOWER_LEG][0] = btScalar(0.05);
			m_scales[BODYPART_LEFT_LOWER_LEG][1] = btScalar(0.28);
			m_scales[BODYPART_LEFT_LOWER_LEG][2] = btScalar(0.05);

			m_scales[BODYPART_RIGHT_UPPER_LEG][0] = btScalar(0.05);
			m_scales[BODYPART_RIGHT_UPPER_LEG][1] = btScalar(0.25);
			m_scales[BODYPART_RIGHT_UPPER_LEG][2] = btScalar(0.06);

			m_scales[BODYPART_RIGHT_LOWER_LEG][0] = btScalar(0.05);
			m_scales[BODYPART_RIGHT_LOWER_LEG][1] = btScalar(0.28);
			m_scales[BODYPART_RIGHT_LOWER_LEG][2] = btScalar(0.05);

			m_scales[BODYPART_LEFT_UPPER_ARM][0] = btScalar(0.05);
			m_scales[BODYPART_LEFT_UPPER_ARM][1] = btScalar(0.19);
			m_scales[BODYPART_LEFT_UPPER_ARM][2] = btScalar(0.05);

			m_scales[BODYPART_LEFT_LOWER_ARM][0] = btScalar(0.04);
			m_scales[BODYPART_LEFT_LOWER_ARM][1] = btScalar(0.25);
			m_scales[BODYPART_LEFT_LOWER_ARM][2] = btScalar(0.04);
			
			m_scales[BODYPART_RIGHT_UPPER_ARM][0] = btScalar(0.05);
			m_scales[BODYPART_RIGHT_UPPER_ARM][1] = btScalar(0.19);
			m_scales[BODYPART_RIGHT_UPPER_ARM][2] = btScalar(0.05);

			m_scales[BODYPART_RIGHT_LOWER_ARM][0] = btScalar(0.04);
			m_scales[BODYPART_RIGHT_LOWER_ARM][1] = btScalar(0.25);
			m_scales[BODYPART_RIGHT_LOWER_ARM][2] = btScalar(0.04);
		}

		
		//Creat a correpsonding shape, scale with height
			for (int i = 0; i < BODYPART_COUNT - 2 ; i++)
			{
			m_shapes[i] = new btBoxShape(btVector3(m_scales[i][0] * scale * (1 + 0.2 * (total_weight - 70) / 40), m_scales[i][1] * scale, m_scales[i][2] * scale * (1 + 0.2 * (total_weight - 70) / 40)));
			}

		m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(0.08) * scale, btScalar(0.13) * scale);
		m_shapes[BODYPART_LEFT_FOOT] = new btBoxShape(btVector3(0.055, 0.039, 0.11)*scale);
		m_shapes[BODYPART_RIGHT_FOOT] = new btBoxShape(btVector3(0.055, 0.039, 0.11)*scale);

		// Setup all the rigid bodies
		btTransform offset;//Position of the character
		offset.setIdentity();
		positionOffset[1] = scale * 0.55*2;//Change height depending on scale
		offset.setOrigin(positionOffset);

		btTransform transform;//Position of current limb
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(0), btScalar(0.)));

		btRigidBody* body2; //Temporary rigidbody for experimental treatment

		// Setup the geometry
		transform.getBasis().setEulerZYX(0, 0, 0);

		body2 = createRigidBody(btScalar(0.075) * total_weight, offset * transform, m_shapes[BODYPART_PELVIS]);
		//body2->setLinearFactor(btVector3(0, 0, 0));  //Force la position et rotation selon des axes
		//body2->setAngularFactor(btVector3(0, 0,0));
		m_bodies[BODYPART_PELVIS] = body2;

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.045 + 0.288/2), btScalar(0.)));
		body2 = createRigidBody(btScalar(0.425)/2 * total_weight, offset * transform, m_shapes[BODYPART_SPINE]);
		m_bodies[BODYPART_SPINE] = body2;

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.045 + 1.5*0.288), btScalar(0.)));
		body2 = createRigidBody(btScalar(0.425)/2 * total_weight, offset * transform, m_shapes[BODYPART_TORSO]);
		m_bodies[BODYPART_TORSO] = body2;

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(0.), btScalar(2*0.34+0.13), btScalar(0.)));
		m_bodies[BODYPART_HEAD] = createRigidBody(btScalar(0.081) * total_weight, offset * transform, m_shapes[BODYPART_HEAD]);

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(-0.191), btScalar(0.045-0.25), btScalar(0.)));
		m_bodies[BODYPART_LEFT_UPPER_LEG] = createRigidBody(btScalar(0.1) * total_weight, offset * transform, m_shapes[BODYPART_LEFT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(-0.191), btScalar(0.045 - 2*0.25-0.28), btScalar(0.)));
		m_bodies[BODYPART_LEFT_LOWER_LEG] = createRigidBody(btScalar(0.047) * total_weight, offset * transform, m_shapes[BODYPART_LEFT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(-0.191), btScalar(0.045 - 2*0.25 - 2*0.28-0.039), btScalar(-0.12)));
		//transform.getBasis().setEulerZYX(0, M_PI, 0);
		body2 = createRigidBody(btScalar(0.014) * total_weight *1, offset * transform, m_shapes[BODYPART_LEFT_FOOT]);
		//body2->setLinearFactor(btVector3(0.3,0.3,0.3));
		body2->setAngularFactor(btVector3(0,0,0));

			m_bodies[BODYPART_LEFT_FOOT] = body2;
		

		transform.setIdentity();
			transform.setOrigin(scale * btVector3(btScalar(0.191), btScalar(0.045 - 0.25), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_UPPER_LEG] = createRigidBody(btScalar(0.1) * total_weight, offset * transform, m_shapes[BODYPART_RIGHT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(0.191), btScalar(0.045 - 2*0.25 - 0.28), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_LOWER_LEG] = createRigidBody(btScalar(0.047) * total_weight, offset * transform, m_shapes[BODYPART_RIGHT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(0.191), btScalar(0.045 - 2*0.25 - 2*0.28 - 0.039), btScalar(-0.12)));
		body2 = createRigidBody(btScalar(0.014) * total_weight*1, offset * transform, m_shapes[BODYPART_RIGHT_FOOT]);
		//body2->setLinearFactor(btVector3(0.3,0.3,0.3));
		body2->setAngularFactor(btVector3(0,0,0));
		m_bodies[BODYPART_RIGHT_FOOT] = body2;

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(-0.268), btScalar(2*0.288+0.045-0.19), btScalar(0.)));
		body2 = createRigidBody(btScalar(0.028) * total_weight, offset * transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);
		m_bodies[BODYPART_LEFT_UPPER_ARM] = body2;

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(-0.268), btScalar(2*0.288 + 0.045-2*0.19-0.25), btScalar(0.)));
		m_bodies[BODYPART_LEFT_LOWER_ARM] = createRigidBody(0.022 * btScalar(1.) * total_weight, offset * transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);

		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(0.268), btScalar(2*0.288+0.045-0.19), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_UPPER_ARM] = createRigidBody(0.028 * btScalar(1.) * total_weight, offset * transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);


		transform.setIdentity();
		transform.setOrigin(scale * btVector3(btScalar(0.268), btScalar(2*0.288 + 0.045-2*0.19-0.25), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_LOWER_ARM] = createRigidBody(0.022 * btScalar(1.) * total_weight, offset * transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);

		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(btScalar(0.05), btScalar(0.85));
			m_bodies[i]->setDamping(btScalar(0.), btScalar(0.));
			m_bodies[i]->setDeactivationTime(btScalar(100.));
			m_bodies[i]->setSleepingThresholds(btScalar(1.), btScalar(1.));
			m_bodies[i]->setAnisotropicFriction(m_bodies[i]->getCollisionShape()->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
			m_bodies[i]->setAnisotropicFriction(m_bodies[i]->getCollisionShape()->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_FRICTION);
			m_bodies[i]->setContactStiffnessAndDamping(m_bodies[i]->getContactStiffness(), 0);
			m_bodies[i]->setRollingFriction(1.f);
			m_bodies[i]->setSpinningFriction(0.5f);
			m_bodies[i]->setFriction(1.f);
			m_bodies[i]->setRestitution(0);
		}

		//Certain limbs touch each other, lower friction to avoid noisy movements
		for (int i : {BODYPART_LEFT_LOWER_ARM, BODYPART_RIGHT_LOWER_ARM, BODYPART_LEFT_LOWER_LEG, BODYPART_RIGHT_LOWER_LEG, BODYPART_LEFT_UPPER_LEG, BODYPART_RIGHT_UPPER_LEG})
		{
			m_bodies[i]->setRollingFriction(0.f);
			m_bodies[i]->setSpinningFriction(0.f);
			m_bodies[i]->setFriction(0.f);
			//m_bodies[i]->setCollisionFlags(m_bodies[i]->CF_NO_CONTACT_RESPONSE); //To disable collisions

		}
		// Now setup the constraints
		btGeneric6DofSpring2Constraint* dofC;
		btTransform localA, localB;

		float targetVelocity = 0.f;
		float maxMotorImpulse = 5.f;

		btQuaternion quart = btQuaternion(0.f, 1.f, 0.f, 1.f);  // x fait varier l'angle du genou


		//Always the same structure :

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.05), btScalar(0.)));//Setup the local offsets
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.288)/2, btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);//Create the constraint between two rigibodies
		dofC->setLimit(3, -M_PI_4/3, M_PI_4/3);
		dofC->setLimit(4, -M_PI_4/4, M_PI_4/4);
		dofC->setLimit(5, -M_PI_4/4, M_PI_4/4);//Add  limits
		m_joints[JOINT_PELVIS_SPINE] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);
		m_parent[JOINT_PELVIS_SPINE] = -1;//Add parents (for jacobian later)

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.288)/2, btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.288)/2, btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_TORSO], localA, localB);
		dofC->setLimit(3, -M_PI_4 / 3, M_PI_4 / 3);
		dofC->setLimit(4, -M_PI_4 / 4, M_PI_4 / 4);
		dofC->setLimit(5, -M_PI_4 / 4, M_PI_4 / 4);
		m_joints[JOINT_SPINE_TORSO] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_TORSO], true);
		m_parent[JOINT_SPINE_TORSO] = JOINT_PELVIS_SPINE;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.34)-0.288/2, btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.13), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint(*m_bodies[BODYPART_TORSO], *m_bodies[BODYPART_HEAD], localA, localB);
		dofC->setLimit(3, 0, 0);
		dofC->setLimit(4, 0, 0);
		dofC->setLimit(5, 0, 0);
		m_joints[JOINT_SPINE_HEAD] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);
		m_parent[JOINT_SPINE_HEAD] = JOINT_SPINE_TORSO;

localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(-0.191/2), btScalar(0.045), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.05), btScalar(0.25), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint( *m_bodies[BODYPART_PELVIS],*m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
		dofC->setLimit(3, -M_PI * 5 / 6, M_PI * 5 / 6);
		dofC->setLimit(4, -M_PI * 5 / 6, M_PI * 5 / 6);
		dofC->setLimit(5, -M_PI * 2 / 6, M_PI * 2 / 6);
		m_joints[JOINT_LEFT_PELVIS] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_PELVIS], true);

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(-0.191/2), btScalar(0.045), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.05), btScalar(0.25), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint( *m_bodies[BODYPART_LEFT_UPPER_LEG],*m_bodies[BODYPART_PELVIS], localB, localA);
		dofC->setLimit(3, -M_PI * 5 / 6, M_PI * 5 / 6);
		dofC->setLimit(4, -M_PI * 5 / 6, M_PI * 5 / 6);
		dofC->setLimit(5, -M_PI * 2 / 6, M_PI * 2 / 6);
		m_joints[JOINT_LEFT_HIP] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);
		m_parent[JOINT_LEFT_HIP] = -1;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.25), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.28), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint( *m_bodies[BODYPART_LEFT_LOWER_LEG],*m_bodies[BODYPART_LEFT_UPPER_LEG], localB, localA);
		dofC->setLimit(5, 0,0);
		dofC->setLimit(4, 0,0);
		dofC->setLimit(3, -M_PI * 5 / 6, 0.05);
		m_joints[JOINT_LEFT_KNEE] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);
		m_parent[JOINT_LEFT_KNEE] = JOINT_LEFT_HIP;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.25), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.28), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint( *m_bodies[BODYPART_LEFT_UPPER_LEG],*m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
		dofC->setLimit(5, 0, 0);
		dofC->setLimit(4, 0, 0);
		dofC->setLimit(3, -0.05, M_PI * 5 / 6);
		m_joints[JOINT_LEFT_KNEE_SWING] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE_SWING], true);
		m_parent[JOINT_LEFT_KNEE_SWING] = JOINT_LEFT_PELVIS;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.191/2), btScalar(0.045), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(-0.05), btScalar(0.25), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint( *m_bodies[BODYPART_PELVIS],*m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
		dofC->setLimit(3, -M_PI * 5 / 6, M_PI * 5 / 6);
		dofC->setLimit(4, -M_PI * 5 / 6, M_PI * 5 / 6);
		dofC->setLimit(5, -M_PI * 2 / 6, M_PI * 2 / 6);
		m_joints[JOINT_RIGHT_PELVIS] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_PELVIS], true);

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.191/2), btScalar(0.045), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(-0.05), btScalar(0.25), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint(  *m_bodies[BODYPART_RIGHT_UPPER_LEG],*m_bodies[BODYPART_PELVIS], localB, localA);
		dofC->setLimit(3, -M_PI * 5 / 6, M_PI * 5 / 6);
		dofC->setLimit(4, -M_PI * 5 / 6, M_PI * 5 / 6);
		dofC->setLimit(5, -M_PI * 2 / 6, M_PI * 2 / 6);
		m_joints[JOINT_RIGHT_HIP] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);
		m_parent[JOINT_RIGHT_HIP] = -1;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.25), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.28), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint(*m_bodies[BODYPART_RIGHT_LOWER_LEG],*m_bodies[BODYPART_RIGHT_UPPER_LEG], localB,  localA);
		//dofC->setLimit(btScalar(0), btScalar(M_PI_2));
		dofC->setLimit(3, -M_PI * 5 / 6, 0.05);
		dofC->setLimit(4, 0, 0);
		dofC->setLimit(5, 0, 0);
		m_joints[JOINT_RIGHT_KNEE] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);
		m_parent[JOINT_RIGHT_KNEE] = JOINT_RIGHT_HIP;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.25), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.28), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint( *m_bodies[BODYPART_RIGHT_UPPER_LEG],*m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
		//dofC->setLimit(btScalar(0), btScalar(M_PI_2));
		dofC->setLimit(3, -0.05, M_PI * 5 / 6);
		dofC->setLimit(4, 0, 0);
		dofC->setLimit(5, 0, 0);
		m_joints[JOINT_RIGHT_KNEE_SWING] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE_SWING], true);
		m_parent[JOINT_RIGHT_KNEE_SWING] = JOINT_RIGHT_PELVIS;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(-0.259/2), btScalar(0.29)/2, btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.05), btScalar(0.19), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint(*m_bodies[BODYPART_TORSO], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
		dofC->setLimit(3, -M_PI * 5 / 6, M_PI * 5 / 6);
		dofC->setLimit(4, 0,0);
		dofC->setLimit(5, 0, M_PI * 5 / 6);
		m_joints[JOINT_LEFT_SHOULDER] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);
		m_parent[JOINT_LEFT_SHOULDER] = JOINT_SPINE_TORSO;
		//m_parent[JOINT_LEFT_SHOULDER] = -1;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.19), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.25), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
		dofC->setLimit(3, -M_PI * 5 / 6, 0);
		dofC->setLimit(4, -0, 0);
		dofC->setLimit(5, -0, 0);
		m_joints[JOINT_LEFT_ELBOW] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);
		m_parent[JOINT_LEFT_ELBOW] = JOINT_LEFT_SHOULDER;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.259/2), btScalar(0.29)/2, btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(-0.05), btScalar(0.19), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint(*m_bodies[BODYPART_TORSO], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
		dofC->setLimit(3, -M_PI * 5 / 6, M_PI * 5 / 6);
		dofC->setLimit(4, -0, 0);
		dofC->setLimit(5, -M_PI * 5 / 6, M_PI * 5 / 6);
		m_joints[JOINT_RIGHT_SHOULDER] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);
		m_parent[JOINT_RIGHT_SHOULDER] = JOINT_SPINE_TORSO;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.19), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.25), btScalar(0.)));
		dofC = new btGeneric6DofSpring2Constraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
		dofC->setLimit(3, -M_PI * 5 / 6, 0);
		dofC->setLimit(4, 0, 0);
		dofC->setLimit(5, -0, 0);
		m_joints[JOINT_RIGHT_ELBOW] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
		m_parent[JOINT_RIGHT_ELBOW] = JOINT_RIGHT_SHOULDER;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.28), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.039), btScalar(0.03)));
		dofC = new btGeneric6DofSpring2Constraint(*m_bodies[BODYPART_LEFT_FOOT],*m_bodies[BODYPART_LEFT_LOWER_LEG], localB,  localA);
		dofC->setLimit(3, -M_PI, M_PI);
		dofC->setLimit(4, -0, 0);
		dofC->setLimit(5, -M_PI, M_PI);

		m_joints[JOINT_LEFT_FOOT] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_FOOT], true);
		m_parent[JOINT_LEFT_FOOT] = JOINT_LEFT_KNEE;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0.), btScalar(-0.28), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.039), btScalar(0.03)));
		dofC = new btGeneric6DofSpring2Constraint( *m_bodies[BODYPART_LEFT_LOWER_LEG],*m_bodies[BODYPART_LEFT_FOOT], localA, localB);
		dofC->setLimit(3, -M_PI, M_PI);
		dofC->setLimit(4, -0, 0);
		dofC->setLimit(5, -M_PI, M_PI);
		m_joints[JOINT_LEFT_FOOT_SWING] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_FOOT_SWING], true);
		m_parent[JOINT_LEFT_FOOT_SWING] = JOINT_LEFT_KNEE_SWING;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0), btScalar(-0.28), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.039), btScalar(0.03)));
		dofC = new btGeneric6DofSpring2Constraint(*m_bodies[BODYPART_RIGHT_FOOT],  *m_bodies[BODYPART_RIGHT_LOWER_LEG], localB,localA);
		dofC->setLimit(3, -M_PI, M_PI);
		dofC->setLimit(4, -0, 0);
		dofC->setLimit(5, -M_PI, M_PI);
		m_joints[JOINT_RIGHT_FOOT] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_FOOT], true);
		m_parent[JOINT_RIGHT_FOOT] = JOINT_RIGHT_KNEE;

		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, 0, 0);
		localA.setOrigin(scale * btVector3(btScalar(0), btScalar(-0.28), btScalar(0.)));
		localB.getBasis().setEulerZYX(0, 0, 0);
		localB.setOrigin(scale * btVector3(btScalar(0.), btScalar(0.039), btScalar(0.03)));
		dofC = new btGeneric6DofSpring2Constraint( *m_bodies[BODYPART_RIGHT_LOWER_LEG],*m_bodies[BODYPART_RIGHT_FOOT], localA, localB);
		dofC->setLimit(3, -M_PI, M_PI);
		dofC->setLimit(4, -0, 0);
		dofC->setLimit(5, -M_PI, M_PI);
		m_joints[JOINT_RIGHT_FOOT_SWING] = dofC;
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_FOOT_SWING], true);
		m_parent[JOINT_RIGHT_FOOT_SWING] = JOINT_RIGHT_KNEE_SWING;


		m_parent[JOINT_RIGHT_HIP] = -1;
		m_parent[JOINT_LEFT_HIP] = -1;
		m_parent[JOINT_RIGHT_PELVIS] = -1;
		m_parent[JOINT_LEFT_PELVIS] = -1;

		//Default posiion
		for (int i = 0; i < JOINT_COUNT; i++)
			m_stance[i] = btVector3(0, 0, 0);

	}

	virtual ~RagDoll()
	{
		int i;

		// Remove all constraints
		for (i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i];
			m_joints[i] = 0;
		}

		// Remove all bodies and shapes
		for (i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);

			delete m_bodies[i]->getMotionState();

			delete m_bodies[i];
			m_bodies[i] = 0;
			delete m_shapes[i];
			m_shapes[i] = 0;
		}
	}

	btVector3 RagDoll::getCOM();
	btVector3 RagDoll::getCOMvel();
	btGeneric6DofSpring2Constraint* RagDoll::pd_controller_dof(btGeneric6DofSpring2Constraint* dof, btRigidBody* bodyA, btRigidBody* bodyB, btVector3 qd, btScalar kp, btScalar kd, btVector3 free);
	btGeneric6DofSpring2Constraint* RagDoll::pd_controller_dof_glob(btGeneric6DofSpring2Constraint* dof, btRigidBody* bodyA, btRigidBody* bodyB, btVector3 qd, btScalar kp, btScalar kd, btVector3 free);
	void RagDoll::update(bool right_leg,bool walking);
	void RagDoll::printParent(int joint,int index);
	int chain[5];
	void RagDoll::gravityComp(int bodypart, btVector3 endEffector, btVector3 force);
};

class CommonExampleInterface* CanidCreateFunc(struct CommonExampleOptions& options);

#endif  // !RAGDOLL_H

