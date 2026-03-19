
#include "RagDoll.h"

void RagDoll::update(bool right_leg, bool walking)
{
	//Reset the logging tab
	for (int k = 0; k < JOINT_COUNT; k++)
	{
		for (int l = 0; l < 3; l++)
		{
			for (int m = 0; m < 3; m++)
				logging[k][l][m] = 0;
		}
	}


	btGeneric6DofSpring2Constraint* dof;
	btRigidBody* bodyA;
	btRigidBody* bodyB;

	//Velocity compensation force
	btVector3 compensation = -gainComp*getCOMvel();

	//CoM driving force
	btVector3 center = m_bodies[BODYPART_RIGHT_FOOT]->getCenterOfMassTransform().getOrigin() + m_bodies[BODYPART_LEFT_FOOT]->getCenterOfMassTransform().getOrigin();
	center[1] = 2*m_bodies[BODYPART_PELVIS]->getCenterOfMassTransform().getOrigin().y();
	center = center / 2;
	btVector3 goalCOM = walking*gainGoal*(center - getCOM());

	//Depending on current state, activate the joints pd controllers
	dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_PELVIS_SPINE];
	bodyA = m_bodies[BODYPART_PELVIS];
	bodyB = m_bodies[BODYPART_SPINE];
	m_joints[JOINT_PELVIS_SPINE] = pd_controller_dof_glob(dof, bodyA, bodyB, m_stance[JOINT_PELVIS_SPINE], kps[JOINT_PELVIS_SPINE], kds[JOINT_PELVIS_SPINE], btVector3(1, 1, 1));
	gravityComp(JOINT_PELVIS_SPINE, bodyB->getCenterOfMassPosition(),btVector3(0,9.8,0));

	dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_SPINE_TORSO];
	bodyA = m_bodies[BODYPART_SPINE];
	bodyB = m_bodies[BODYPART_TORSO];
	m_joints[JOINT_SPINE_TORSO] = pd_controller_dof_glob(dof, bodyA, bodyB, m_stance[JOINT_SPINE_TORSO], kps[JOINT_SPINE_TORSO], kds[JOINT_SPINE_TORSO], btVector3(1, 1, 1));
	gravityComp(JOINT_SPINE_TORSO, bodyB->getCenterOfMassPosition(), btVector3(0, 9.8, 0));

	dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_SPINE_HEAD];
	bodyA = m_bodies[BODYPART_SPINE];
	bodyB = m_bodies[BODYPART_HEAD];
	bodyB->applyCentralForce(btVector3(0, 9.8, 0) * bodyB->getMass());
	
	dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_LEFT_SHOULDER];
	bodyA = m_bodies[BODYPART_SPINE];
	bodyB = m_bodies[BODYPART_LEFT_UPPER_ARM];
	m_joints[JOINT_LEFT_SHOULDER] = pd_controller_dof(dof, bodyA, bodyB, m_stance[JOINT_LEFT_SHOULDER], kps[JOINT_LEFT_SHOULDER], kds[JOINT_LEFT_SHOULDER], btVector3(1, 1, 1));
	gravityComp(JOINT_LEFT_SHOULDER, bodyB->getCenterOfMassPosition(), btVector3(0, 9.8, 0));

	dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_LEFT_ELBOW];
	bodyA = m_bodies[BODYPART_LEFT_UPPER_ARM];
	bodyB = m_bodies[BODYPART_LEFT_LOWER_ARM];
	m_joints[JOINT_LEFT_ELBOW] = pd_controller_dof(dof, bodyA, bodyB, m_stance[JOINT_LEFT_ELBOW], kps[JOINT_LEFT_ELBOW], kds[JOINT_LEFT_ELBOW], btVector3(1, 0, 0));
	gravityComp(JOINT_LEFT_ELBOW, bodyB->getCenterOfMassPosition(), btVector3(0, 9.8, 0));

	dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_RIGHT_SHOULDER];
	bodyA = m_bodies[BODYPART_SPINE];
	bodyB = m_bodies[BODYPART_RIGHT_UPPER_ARM];
	m_joints[JOINT_RIGHT_SHOULDER] = pd_controller_dof(dof, bodyA, bodyB, m_stance[JOINT_RIGHT_SHOULDER], kps[JOINT_RIGHT_SHOULDER], kds[JOINT_RIGHT_SHOULDER], btVector3(1, 1, 1));
	gravityComp(JOINT_RIGHT_SHOULDER, bodyB->getCenterOfMassPosition(), btVector3(0, 9.8, 0));
	
	dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_RIGHT_ELBOW];
	bodyA = m_bodies[BODYPART_RIGHT_UPPER_ARM];
	bodyB = m_bodies[BODYPART_RIGHT_LOWER_ARM];
	m_joints[JOINT_RIGHT_ELBOW] = pd_controller_dof(dof, bodyA, bodyB, m_stance[JOINT_RIGHT_ELBOW], kps[JOINT_RIGHT_ELBOW], kds[JOINT_RIGHT_ELBOW], btVector3(1, 0, 0));
	gravityComp(JOINT_RIGHT_ELBOW, bodyB->getCenterOfMassPosition(), btVector3(0, 9.8, 0));

	m_joints[JOINT_LEFT_HIP]->setEnabled(!walking || !right_leg);
	m_joints[JOINT_LEFT_PELVIS]->setEnabled(walking && right_leg);
	m_joints[JOINT_RIGHT_HIP]->setEnabled(!walking || right_leg);
	m_joints[JOINT_RIGHT_PELVIS]->setEnabled(walking && !right_leg);

	m_joints[JOINT_RIGHT_KNEE]->setEnabled(!walking || right_leg);
	m_joints[JOINT_RIGHT_KNEE_SWING]->setEnabled(walking && !right_leg);
	m_joints[JOINT_RIGHT_FOOT]->setEnabled(!walking || right_leg);
	//m_joints[JOINT_RIGHT_FOOT_SWING]->setEnabled(!right_leg);

	m_joints[JOINT_LEFT_KNEE]->setEnabled(!walking || !right_leg);
	m_joints[JOINT_LEFT_KNEE_SWING]->setEnabled(walking && right_leg);
	m_joints[JOINT_LEFT_FOOT]->setEnabled(!walking || !right_leg);
	//m_joints[JOINT_LEFT_FOOT_SWING]->setEnabled(right_leg);


	if (!walking || right_leg) //Stand right leg
	{


		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_RIGHT_FOOT];
		bodyA = m_bodies[BODYPART_RIGHT_LOWER_LEG];
		bodyB = m_bodies[BODYPART_RIGHT_FOOT];
		m_joints[JOINT_RIGHT_FOOT] = pd_controller_dof_glob(dof, bodyB, bodyA, m_stance[JOINT_RIGHT_FOOT], kps[JOINT_RIGHT_FOOT], kds[JOINT_RIGHT_FOOT], btVector3(1, 1, 1));
		gravityComp(JOINT_RIGHT_FOOT, getCOM(), compensation);
		gravityComp(JOINT_RIGHT_FOOT, getCOM(), goalCOM);
		bodyB->setLinearFactor(btVector3(1, 1, 1));
		

		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_RIGHT_KNEE];
		bodyA = m_bodies[BODYPART_RIGHT_UPPER_LEG];
		bodyB = m_bodies[BODYPART_RIGHT_LOWER_LEG];
		m_joints[JOINT_RIGHT_KNEE] = pd_controller_dof(dof, bodyB, bodyA, m_stance[JOINT_RIGHT_KNEE], kps[JOINT_RIGHT_KNEE], kds[JOINT_RIGHT_KNEE], btVector3(1, 0, 0));

		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_RIGHT_HIP];
		bodyA = m_bodies[BODYPART_PELVIS];
		bodyB = m_bodies[BODYPART_RIGHT_UPPER_LEG];
		if(walking || left)
			m_joints[JOINT_RIGHT_HIP] = pd_controller_dof_glob(dof, bodyB, bodyA, m_stance[JOINT_RIGHT_HIP], kps[JOINT_RIGHT_HIP], kds[JOINT_RIGHT_HIP], btVector3(1, 1, 1)); 

	}
	if (walking && right_leg) //Swing left leg
	{
		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_LEFT_PELVIS];
		bodyB = m_bodies[BODYPART_PELVIS];
		bodyA = m_bodies[BODYPART_LEFT_UPPER_LEG];
		m_joints[JOINT_LEFT_PELVIS] = pd_controller_dof_glob(dof, bodyB, bodyA, m_stance[JOINT_LEFT_PELVIS], kps[JOINT_LEFT_PELVIS], kds[JOINT_LEFT_PELVIS], btVector3(1, 1, 1));
		gravityComp(JOINT_LEFT_PELVIS, bodyA->getCenterOfMassPosition(), btVector3(0, 9.8, 0));

		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_LEFT_KNEE_SWING];
		bodyA = m_bodies[BODYPART_LEFT_UPPER_LEG];
		bodyB = m_bodies[BODYPART_LEFT_LOWER_LEG];
		m_joints[JOINT_LEFT_KNEE_SWING] = pd_controller_dof(dof, bodyA, bodyB, m_stance[JOINT_LEFT_KNEE_SWING], kps[JOINT_LEFT_KNEE_SWING], kds[JOINT_LEFT_KNEE_SWING], btVector3(1, 0, 0));
		gravityComp(JOINT_LEFT_KNEE_SWING, bodyB->getCenterOfMassPosition(), btVector3(0, 9.8, 0));

		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_LEFT_FOOT_SWING];
		bodyA = m_bodies[BODYPART_LEFT_LOWER_LEG];
		bodyB = m_bodies[BODYPART_LEFT_FOOT];
		gravityComp(JOINT_LEFT_FOOT_SWING, bodyB->getCenterOfMassPosition(), btVector3(0, 9.8, 0));
		bodyB->setLinearFactor(btVector3(1,1,1));
	}
	if (!walking || !right_leg)  //Stand left leg
	{
		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_LEFT_FOOT];
		bodyA = m_bodies[BODYPART_LEFT_LOWER_LEG];
		bodyB = m_bodies[BODYPART_LEFT_FOOT];
		m_joints[JOINT_LEFT_FOOT] = pd_controller_dof_glob(dof, bodyB, bodyA, m_stance[JOINT_LEFT_FOOT], kps[JOINT_LEFT_FOOT], kds[JOINT_LEFT_FOOT], btVector3(1, 1, 1));
		gravityComp(JOINT_LEFT_FOOT, getCOM(), compensation);
		gravityComp(JOINT_LEFT_FOOT, getCOM(), goalCOM);
		bodyB->setLinearFactor(btVector3(1, 1, 1));

		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_LEFT_KNEE];
		bodyA = m_bodies[BODYPART_LEFT_UPPER_LEG];
		bodyB = m_bodies[BODYPART_LEFT_LOWER_LEG];
		m_joints[JOINT_LEFT_KNEE] = pd_controller_dof(dof, bodyB, bodyA, m_stance[JOINT_LEFT_KNEE], kps[JOINT_LEFT_KNEE], kds[JOINT_LEFT_KNEE], btVector3(1, 0, 0));

		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_LEFT_HIP];
		bodyA = m_bodies[BODYPART_PELVIS];
		bodyB = m_bodies[BODYPART_LEFT_UPPER_LEG];

		if (walking || !left)
			m_joints[JOINT_LEFT_HIP] = pd_controller_dof_glob(dof, bodyB, bodyA, m_stance[JOINT_LEFT_HIP], kps[JOINT_LEFT_HIP], kds[JOINT_LEFT_HIP], btVector3(1, 1, 1));

	}
	if (walking && !right_leg)// swing right leg
	{
		

		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_RIGHT_PELVIS];
		bodyB = m_bodies[BODYPART_PELVIS];
		bodyA = m_bodies[BODYPART_RIGHT_UPPER_LEG];
		m_joints[JOINT_RIGHT_PELVIS] = pd_controller_dof_glob(dof, bodyB, bodyA, m_stance[JOINT_RIGHT_PELVIS], kps[JOINT_RIGHT_PELVIS], kds[JOINT_RIGHT_PELVIS], btVector3(1, 1, 1));
		gravityComp(JOINT_RIGHT_PELVIS, bodyA->getCenterOfMassPosition(), btVector3(0, 9.8, 0));

		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_RIGHT_KNEE_SWING];
		bodyA = m_bodies[BODYPART_RIGHT_UPPER_LEG];
		bodyB = m_bodies[BODYPART_RIGHT_LOWER_LEG];
		m_joints[JOINT_RIGHT_KNEE_SWING] = pd_controller_dof(dof, bodyA, bodyB, m_stance[JOINT_RIGHT_KNEE_SWING], kps[JOINT_RIGHT_KNEE_SWING], kds[JOINT_RIGHT_KNEE_SWING], btVector3(1, 0, 0));
		gravityComp(JOINT_RIGHT_KNEE_SWING, bodyA->getCenterOfMassPosition(), btVector3(0, 9.8, 0));
		
		dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_RIGHT_FOOT_SWING];
		bodyA = m_bodies[BODYPART_RIGHT_LOWER_LEG];
		bodyB = m_bodies[BODYPART_RIGHT_FOOT];
		gravityComp(JOINT_RIGHT_FOOT_SWING, bodyB->getCenterOfMassPosition(), btVector3(0, 9.8, 0));
		bodyB->setLinearFactor(btVector3(1,1,1));
	}


	dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_RIGHT_FOOT_SWING];
	bodyA = m_bodies[BODYPART_RIGHT_LOWER_LEG];
	bodyB = m_bodies[BODYPART_RIGHT_FOOT];
	m_joints[JOINT_RIGHT_FOOT_SWING] = pd_controller_dof(dof, bodyA, bodyB, m_stance[JOINT_RIGHT_FOOT_SWING], kps[JOINT_RIGHT_FOOT_SWING], kds[JOINT_RIGHT_FOOT_SWING], btVector3(1, 1, 1));

	dof = (btGeneric6DofSpring2Constraint*)m_joints[JOINT_LEFT_FOOT_SWING];
	bodyA = m_bodies[BODYPART_LEFT_LOWER_LEG];
	bodyB = m_bodies[BODYPART_LEFT_FOOT];
	m_joints[JOINT_LEFT_FOOT_SWING] = pd_controller_dof(dof, bodyA, bodyB, m_stance[JOINT_LEFT_FOOT_SWING], kps[JOINT_LEFT_FOOT_SWING], kds[JOINT_LEFT_FOOT_SWING], btVector3(1, 1, 1));



	//When touching the ground, restrict linear foot movement
	float stickyness = stick;
	if (m_bodies[BODYPART_RIGHT_FOOT]->getCenterOfMassPosition()[1] < 0.05 && !(walking && !right_leg))
	{
		m_bodies[BODYPART_RIGHT_FOOT]->setLinearFactor(btVector3(stickyness, stickyness, stickyness));
		m_bodies[BODYPART_RIGHT_FOOT]->setLinearVelocity(btVector3(0, 0, 0));

	}
	else
		m_bodies[BODYPART_RIGHT_FOOT]->setLinearFactor(btVector3(1, 1, 1));

	if (m_bodies[BODYPART_LEFT_FOOT]->getCenterOfMassPosition()[1] < 0.05 && !(walking && right_leg))
	{
		m_bodies[BODYPART_LEFT_FOOT]->setLinearVelocity(btVector3(0, 0, 0));
		m_bodies[BODYPART_LEFT_FOOT]->setLinearFactor(btVector3(stickyness, stickyness, stickyness));
	}
	else
		m_bodies[BODYPART_LEFT_FOOT]->setLinearFactor(btVector3(1, 1, 1));

}

btGeneric6DofSpring2Constraint* RagDoll::pd_controller_dof(btGeneric6DofSpring2Constraint* dof, btRigidBody* bodyA, btRigidBody* bodyB, btVector3 qd, btScalar kp, btScalar kd, btVector3 free)
{
	btScalar qdot, q;
	btScalar pd_control;

	//Get current joint state
	dof->calculateTransforms();
	btVector3 ori = btVector3(dof->getAngle(0), dof->getAngle(1), dof->getAngle(2));
	btVector3 Domega = bodyB->getAngularVelocity() - bodyA->getAngularVelocity();  // A remettre dans la bonne base // Limiter le controle aux axes concernés
	//Get joint base transition matrix
	btMatrix3x3 transiMat = btMatrix3x3(dof->getAxis(0).normalize(), dof->getAxis(1).normalize(), dof->getAxis(2).normalize());
	transiMat = transiMat.transpose();  

	const btScalar qd_dot = 0;
	btVector3 impulse; //Final applied torque
	//Compute torque for every joint
	for (int i = 0; i < 3; i++)
	{
		qd[i] = free[i] * qd[i];
		q = ori[i];
		qdot = Domega[i];


		pd_control = (-kd * (qd_dot - qdot)) + (kp * (qd[i] - q));
		
		impulse[i] = pd_control;
	}

	impulse = transiMat * impulse;
	bodyB->applyTorque(impulse);

	//Log values
	for (int j = 0; j < JOINT_COUNT; j++ )
	{
		if (dof == m_joints[j])
		{
			for (int i = 0; i < 3; i++)
			{
				logging[j][i][0] += ori[i];
				logging[j][i][1] += qd[i];
				logging[j][i][2] += impulse[i];
			}
		}	
	}

	return dof;
}

//Recursive chain function for parent limb
void  RagDoll::printParent(int joint, int index) {
	
	if (m_parent[joint] == -1)
	{
		chain[index] = joint;
		return;
	}
	else
	{
		chain[index] = joint;
		index++;
		printParent(m_parent[joint],index);
	}
}

//Function to apply a jacobian
void RagDoll::gravityComp(int lastjoint, btVector3 endEffector,btVector3 force)
{
	//Find all parent limbs
	std::fill_n(chain, 5, -1);
	printParent(lastjoint, 0);

	int length = 5;
	for (int bodyp : chain)
		if (bodyp == -1)
			length--;

	//3x3xlength Jacobian matrix
	std::vector<std::vector<float>> jacobian(3, std::vector<float>(3*length, 0.f));

	//For every joint compute the jacobian
	for (int i = 0; i < length; i++)
	{
		//Compute joint position
		int joint = chain[i];
		m_joints[joint]->calculateTransforms();
		btVector3 pos  = m_joints[joint]->getCalculatedTransformB().getOrigin();
		//Compute the vector between the joint and goal
		pos = endEffector - pos;
	

		for (int j = 0; j < 3; j++)  //Les axes du monde (ou sera appliqué le torsuer)
		{
			btVector3 axis = btVector3(0, 0, 0);
			axis[j] = 1;
			btVector3 temp = btCross(axis, pos);
			for (int k = 0; k < 3; k++)  //Leur influence sur les axes monde
				jacobian[k][3 * i + j] = temp[k];
		}
	}

	//Transpose the jacobian
	std::vector<std::vector<float>> transpose(3*length, std::vector<float>(3, 0.f));
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3 * length; j++)
			transpose[j][i] = jacobian[i][j];
	}

	//TODO Check this WEIGHT scaling
	btVector3 forceApplied = m_joints[lastjoint]->getRigidBodyB().getMass() * force;
	std::vector<float> result(3*length,0);
	for (int i = 0; i < 3 * length; i++)
	{
		result[i] = transpose[i][0] * forceApplied[0] + transpose[i][1] * forceApplied[1] + transpose[i][2] * forceApplied[2];
	}


	for (int i = 0; i < length; i++)
	{
		m_joints[chain[i]]->getRigidBodyB().applyTorque(btVector3(result[3 * i], result[3 * i + 1], result[3 * i + 2]));
		logging[chain[i]][0][2] += result[3 * i];
		logging[chain[i]][1][2] += result[3 * i+1];
		logging[chain[i]][2][2] += result[3 * i+2];
	}

}

btGeneric6DofSpring2Constraint* RagDoll::pd_controller_dof_glob(btGeneric6DofSpring2Constraint* dof, btRigidBody* bodyA, btRigidBody* bodyB, btVector3 qd, btScalar kp, btScalar kd, btVector3 free)
{

	btQuaternion rota = bodyA->getOrientation();
	btScalar X, Y, Z;
	rota.getEulerZYX(Z, Y, X);
	qd = qd + btVector3(X, Y, Z);
	return pd_controller_dof(dof, bodyA, bodyB, qd, kp,kd, free);
}

btVector3 RagDoll::getCOM()
{
	btVector3 result(0, 0, 0);
	float weight = 0;
	for (int i = 0; i < JOINT_COUNT; i++)
	{
		result += m_bodies[i]->getMass() * m_bodies[i]->getCenterOfMassPosition();
		weight += m_bodies[i]->getMass();
	}
	return result / weight;
}

btVector3 RagDoll::getCOMvel()
{
	btVector3 result(0, 0, 0);
	float weight = 0;
	for (int i = 0; i < JOINT_COUNT; i++)
	{
		result += m_bodies[i]->getMass() * m_bodies[i]->getLinearVelocity();
		weight += m_bodies[i]->getMass();
	}
	return result / weight;
}