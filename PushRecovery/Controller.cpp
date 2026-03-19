
//Adapted from https://salzis.wordpress.com/2014/05/01/convex-hull-how-to-tell-whether-a-point-is-inside-or-outside/


#include "Controller.h"

#include <../canid/spline.h>


void Controller::fit(RagDoll* r, bool rightleg, float phi, bool near, btRigidBody* cube,btTransform facingDir)
{
	//Define step height average trajectory
	Spline traj;
	{
		traj.setKnot(0, 0, 0.05);
		traj.setKnot(1, 0.15, 0.2);
		traj.setKnot(2, 0.7, 0.08);
		traj.setKnot(3, 1.0, -0.2);
	}
	
	btVector3 origin,normal,origin2,nParent;
	btScalar vParent, vChild;

	if (rightleg)
	{
		origin = r->m_joints[JOINT_LEFT_HIP]->getCalculatedTransformB().getOrigin();
		origin2 = r->m_joints[r->JOINT_LEFT_FOOT_SWING]->getCalculatedTransformB().getOrigin(); 
		normal = r->m_joints[r->JOINT_LEFT_KNEE_SWING]->getAxis(0);

	}
	else
	{
		origin = r->m_joints[JOINT_RIGHT_HIP]->getCalculatedTransformB().getOrigin();
		origin2 = r->m_joints[r->JOINT_RIGHT_FOOT_SWING]->getCalculatedTransformB().getOrigin();
		normal = r->m_joints[r->JOINT_RIGHT_KNEE_SWING]->getAxis(0);
	}

	//TODO Limb length 
	vParent = 0.25 * 1.75;
	nParent = btVector3(1, 0, 0);
	vChild = 0.28 *1.75;

	btTransform transform;
	transform.setIdentity();
	btVector3 goal = btVector3(btScalar((1 - phi) * origin2[0] + phi * xd), (1 + 2.5 * (r->scale - 1.75) / 1.75)* spline  *btScalar(traj.evaluate_catmull_rom(phi)), btScalar((1 - phi) * origin2[2] + phi * zd));
	
	origin = origin;
	origin2 = origin2;
	goal = goal;
	//Front angle
	btScalar angleZ = btAsin((goal.x() - origin2.x()) / (vParent+vChild));

	//TODO verifier defintion de n (normal)
	btVector3 knee = Controller::solve(origin, goal, btVector3(1,0,0), vParent, vChild);

	//transform.setOrigin(btVector3(-0,0.2,0)+goal);
	//cube->setWorldTransform(transform);
	
	//Angle of the swing hip
	btScalar angleX = -btAtan((((knee.z() - origin.z()) / (knee.y() - origin.y()))));
	//Angle of the swing knee
	btScalar angleK = -btAtan((((goal.z() - knee.z()) / (goal.y() - knee.y()))));


	if (rightleg)
	{
		r->m_stance[r->JOINT_LEFT_PELVIS][0] = angleX;
		r->m_stance[r->JOINT_LEFT_PELVIS][1] = 0;
		r->m_stance[r->JOINT_LEFT_PELVIS][2] = -angleZ;
		r->m_stance[r->JOINT_LEFT_KNEE_SWING][0] = -angleX + angleK;
		r->m_stance[r->JOINT_LEFT_FOOT_SWING][2] = angleZ;

		r->m_stance[r->JOINT_RIGHT_PELVIS] = btVector3(0, 0, 0);
		r->m_stance[r->JOINT_RIGHT_KNEE_SWING] = btVector3(0, 0, 0);
		r->m_stance[r->JOINT_RIGHT_FOOT_SWING] = btVector3(0, 0, 0);
		r->m_stance[r->JOINT_RIGHT_FOOT] = btVector3(0, 0, 0);
	}
	else
	{
		r->m_stance[r->JOINT_RIGHT_PELVIS][0] = angleX;
		r->m_stance[r->JOINT_RIGHT_PELVIS][1] = 0;
		r->m_stance[r->JOINT_RIGHT_PELVIS][2] = -angleZ;
		r->m_stance[r->JOINT_RIGHT_KNEE_SWING][0] = -angleX + angleK;
		r->m_stance[r->JOINT_RIGHT_FOOT_SWING][2] = angleZ;

		r->m_stance[r->JOINT_LEFT_PELVIS] = btVector3(0, 0, 0);
		r->m_stance[r->JOINT_LEFT_KNEE_SWING] = btVector3(0, 0, 0);
		r->m_stance[r->JOINT_LEFT_FOOT_SWING] = btVector3(0, 0, 0);
		r->m_stance[r->JOINT_LEFT_FOOT] = btVector3(0, 0, 0);
	}

	//Projected position of the goal CoM is between the feet
	btVector3 goalCOM = r->m_joints[JOINT_LEFT_FOOT]->getCalculatedTransformB().getOrigin() + r->m_joints[JOINT_RIGHT_FOOT]->getCalculatedTransformB().getOrigin();
	goalCOM[1] = 2 * r->m_bodies[r->BODYPART_PELVIS]->getCenterOfMassTransform().getOrigin().y();
	goalCOM = goalCOM/2;

	//Reverse process for stand leg
	if (!rightleg)
	{
		origin = r->m_joints[JOINT_LEFT_PELVIS]->getCalculatedTransformB().getOrigin();
		origin2 = r->m_joints[r->JOINT_LEFT_FOOT]->getCalculatedTransformB().getOrigin();
		normal = r->m_joints[r->JOINT_LEFT_KNEE]->getAxis(0);
	}
	else
	{
		origin = r->m_joints[JOINT_RIGHT_PELVIS]->getCalculatedTransformB().getOrigin();
		origin2 = r->m_joints[r->JOINT_RIGHT_FOOT]->getCalculatedTransformB().getOrigin();
		normal = r->m_joints[r->JOINT_RIGHT_KNEE]->getAxis(0);
	}

	vParent = 0.25 * 1.75;
	nParent = btVector3(1, 0, 0);
	vChild = 0.29 * 1.75;
	
	 angleZ = -btAsin((goalCOM.x() - origin2.x()) / (vParent + vChild));

	 //TODO same
	 knee = Controller::solve(goalCOM, origin2, btVector3(1, 0, 0), vParent, vChild);
	 angleX = -btAtan((goalCOM.z() - origin2.z()) / (vParent + vChild));


	if (rightleg)
	{
		r->m_stance[r->JOINT_LEFT_HIP][0] = angleX;
		r->m_stance[r->JOINT_LEFT_HIP][1] = 0;
		r->m_stance[r->JOINT_LEFT_HIP][2] = -angleZ;
		r->m_stance[r->JOINT_LEFT_KNEE][0] = 0;
		r->m_stance[r->JOINT_LEFT_FOOT][2] = angleZ;
		r->m_stance[r->JOINT_LEFT_FOOT][0] = -angleX;

		r->m_stance[r->JOINT_LEFT_HIP] = btVector3(0, 0, 0);
		r->m_stance[r->JOINT_RIGHT_KNEE] = btVector3(0, 0, 0);
		r->m_stance[r->JOINT_RIGHT_FOOT] = btVector3(0, 0, 0);
	}
	else 
	{
		r->m_stance[r->JOINT_RIGHT_HIP][0] = angleX;
		r->m_stance[r->JOINT_RIGHT_HIP][1] = 0;
		r->m_stance[r->JOINT_RIGHT_HIP][2] = -angleZ;
		r->m_stance[r->JOINT_RIGHT_KNEE][0] = 0;
		r->m_stance[r->JOINT_RIGHT_FOOT][2] = angleZ;
		r->m_stance[r->JOINT_RIGHT_FOOT][0] = -angleX;

		r->m_stance[r->JOINT_LEFT_HIP] = btVector3(0, 0, 0);
		r->m_stance[r->JOINT_LEFT_KNEE] = btVector3(0, 0, 0);
		r->m_stance[r->JOINT_LEFT_FOOT] = btVector3(0, 0, 0);
	}
};


btVector3 Controller::solve(const btVector3& p1, const btVector3& p2, const btVector3& n, double r1, double r2)
{
	//the solution for this comes from computation of the intersection of two circles of radii r1 and r2, located at
	//p1 and p2 respectively. There are, of course, two solutions to this problem. The calling application can differentiate between these
	//by passing in n or -n for the plane normal.

	//this is the distance between p1 and p2. If it is > r1+r2, then we have no solutions. To be nice about it,
	//we will set r to r1+r2 - the behaviour will be to reach as much as possible, even though you don't hit the target
	double r = (p2-p1).length();
	if (r > (r1 + r2))
		r = (r1 + r2);
	//this is the length of the vector starting at p1 and going to the midpoint between p1 and p2
	double a = (r1 * r1 - r2 * r2 + r * r) / (2 * r);
	double tmp = r1 * r1 - a * a;
	if (tmp < 0)
		tmp = 0;
	//and this is the distance from the midpoint of p1-p2 to the intersection point
	double h = sqrt(tmp);
	//now we need to get the two directions needed to reconstruct the intersection point
	btVector3 d1 = (p2-p1).normalize();
	btVector3 d2 = d1.cross(n).normalize();

	//and now get the intersection point
	btVector3 p = p1 + d1 * a + d2 * (-h);

	return p;
}


bool Controller::isBalanced(RagDoll* r, float deltatime)
{
	//Compute the hull TODO Adapat to size
	convexHull.corners.clear();
	convexHull.hull.clear();
	btVector3 corner1, corner2, corner3, corner4;
	btTransform transform = r->m_bodies[r->BODYPART_RIGHT_FOOT]->getCenterOfMassTransform();
	corner1 = transform.getOrigin() - (0.5 * 1.75 * 0.055) * transform.getBasis().getColumn(0) + 0.5 * 1.75 * (0.152) * transform.getBasis().getColumn(2);
	corner2 = transform.getOrigin() + (0.5 * 1.75 * 0.055) * transform.getBasis().getColumn(0) + 0.5 * 1.75 * (0.152) * transform.getBasis().getColumn(2);
	corner3 = transform.getOrigin() - (0.5 * 1.75 * 0.055) * transform.getBasis().getColumn(0) - 0.5 * 1.75 * (0.152) * transform.getBasis().getColumn(2);
	corner4 = transform.getOrigin() + (0.5 * 1.75 * 0.055) * transform.getBasis().getColumn(0) - 0.5 * 1.75 * (0.152) * transform.getBasis().getColumn(2);

	convexHull.convexAdd(corner1[0], corner1[2]);
	convexHull.convexAdd(corner2[0], corner2[2]);
	convexHull.convexAdd(corner3[0], corner3[2]);
	convexHull.convexAdd(corner4[0], corner4[2]);


	btVector3 corner5, corner6, corner7, corner8;
	transform = r->m_bodies[r->BODYPART_LEFT_FOOT]->getCenterOfMassTransform();
	corner5 = transform.getOrigin() - 1.75 * (0.055) * transform.getBasis().getColumn(0) + 0.5 * 1.75 * (0.152) * transform.getBasis().getColumn(2);
	corner6 = transform.getOrigin() + 1.75 * (0.055) * transform.getBasis().getColumn(0) + 0.5 * 1.75 * (0.152) * transform.getBasis().getColumn(2);
	corner7 = transform.getOrigin() - 1.75 * (0.055) * transform.getBasis().getColumn(0) - 0.5 * 1.75 * (0.152) * transform.getBasis().getColumn(2);
	corner8 = transform.getOrigin() + 1.75 * (0.055) * transform.getBasis().getColumn(0) - 0.5 * 1.75 * (0.152) * transform.getBasis().getColumn(2);

	convexHull.convexAdd(corner5[0], corner5[2]);
	convexHull.convexAdd(corner6[0], corner6[2]);
	convexHull.convexAdd(corner7[0], corner7[2]);
	convexHull.convexAdd(corner8[0], corner8[2]);
	convexHull.convexHull();

	int i = 0;

	for (ConvexHull::Point* p : convexHull.corners)
	{
		i++;
	}
	for (ConvexHull::Point* p : convexHull.hull)
	{
		i++;
	}

	
	//Compute the COM and direction
	btVector3 com = r->getCOM();
	btVector3 xcom = com + r->getCOMvel();

	//Compute ttBoS

	ConvexHull::Point* pcom = new ConvexHull::Point; //Predicted CoM position
	pcom->x = xcom[0];
	pcom->y = xcom[2];
	ConvexHull::Point* lcom = new ConvexHull::Point; //Current CoM position
	lcom->x = com[0];
	lcom->y = com[2];

	//Compute closest point index
	float distance=10;
	int indice = 0;
	for (int i = 0; i < convexHull.hull.size(); i++)
	{
		ConvexHull::Point* p = convexHull.hull[i];
		float delta = convexHull.distance_point(p, pcom);
		if (delta < distance)
		{
			distance = delta;
			indice = i;
		}	
	}

	//Closest bound has closest point in it
	ConvexHull::Point* A = convexHull.hull[indice];
	ConvexHull::Point* B1 = convexHull.hull[(indice - 1) % (convexHull.hull.size() - 1)];
	ConvexHull::Point* B2 = convexHull.hull[(indice + 1) % (convexHull.hull.size() - 1)];

	ConvexHull::Point* zero = new ConvexHull::Point();
	float DistanceSeg1 = convexHull.distance_point(convexHull.cross_line(A, B1, lcom, pcom), zero);
	float DistanceSeg2 = convexHull.distance_point(convexHull.cross_line(A, B2, lcom, pcom), zero);

	
	ConvexHull::Point* intersection;
	if (DistanceSeg1 < DistanceSeg2)
		intersection = convexHull.cross_line(B1, A, lcom, pcom);
	else
		intersection = convexHull.cross_line(A, B2, lcom, pcom);


	//Return if we are far enough
	float ttbos = convexHull.distance_point(intersection, lcom) / (btSqrt(r->getCOMvel()[0] * r->getCOMvel()[0] + r->getCOMvel()[2] * r->getCOMvel()[2]));
	
	//Logging TODO Move
	{

		//Log errors for optimisation along with trajectories
		ofstream myfile;
		myfile.open("../../canid/log/output" + std::to_string(index) + ".txt", ios_base::app);
		btScalar errorAngle = 0;
		btScalar errorEffort = 0;
		for (int i = 0; i < r->JOINT_COUNT; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				errorAngle += btPow(btFabs(r->logging[i][j][0] - r->logging[i][j][1]), 2);
				errorEffort += btPow(r->logging[i][j][2], 2);  
			}
		}
		myfile << fixed << errorAngle << " " << errorEffort << " ";
		myfile << r->getCOM()[0] << " " << -r->getCOM()[2] << " " << r->getCOM()[1] << " " << com[0] << " " << com[2] << " " << pasEffectue << "\n";
		myfile.close();

		//Log produced torques
		ofstream myfile2;
		myfile2.open("../../canid/log/torques" + std::to_string(index) + ".txt", ios_base::app);
		for (int i : {3, 7, 5, 6, 4, 8, 15, 16, 13, 14, 17, 18})
			myfile2 << (r->logging[i][0][0] + r->logging[i][1][0] + r->logging[i][2][0]) * 180 / M_PI << " " << (r->logging[i][0][1] + r->logging[i][1][1] + r->logging[i][2][1]) * 180 / M_PI << " " << (r->logging[i][0][2] + r->logging[i][1][2] + r->logging[i][2][2]) << " ";
		myfile2 << "\n";
		myfile2.close();

		//Log each joint angle
		ofstream myfile3;
		myfile3.open("../../canid/log/angule" + std::to_string(index) + ".txt", ios_base::app);
		for (int i : {0, 1, 3, 7, 5, 6, 4, 8, 15, 16, 13, 14, 17, 18})
			myfile3 << (r->logging[i][0][0]) << " ";
		myfile3 << "\n";
		myfile3.close();
	}


	return ttbos<0.5;
}


void Controller::process(RagDoll* ragdoll, float deltatime, btRigidBody* cube, btTransform facingDir, bool pushed)
{
	//Manage the push of the character
	b3MouseButtonCallback(0);
	btScalar forceMax,time;

	//Get the trial's parameters
	forceMax = ragdoll->force*gainForce;
	time = ragdoll->time;
	buttonTime += deltatime;  //TODO get system step
	//Manage the visualisation of the push
	btTransform transform;
	transform.setOrigin(btVector3(-0, 1.3, 0.55));
	transform.setRotation(btQuaternion(0, 0, 0));
	if ((buttonTime > 1) && (buttonTime < (1 + time)) && pushed)
	{
		//ragdoll->m_bodies[1]->clearForces();
		ragdoll->m_bodies[2]->applyForce((1 - (abs(buttonTime - (1 + time / 2)) * 2 / time)) * btVector3(sin(y/2) * forceMax * (1+x)*2, 0, -cos(y/2) * forceMax * (1+x) *2),btVector3(0,0.05*ragdoll->scale,0));
		transform.setOrigin(btVector3(-sin(y / 2)*0.55, 0.05 * ragdoll->scale, cos(y / 2) *0.55) + ragdoll->m_bodies[2]->getCenterOfMassPosition());
		transform.setRotation(btQuaternion(-y/2, 0, 0));
	}
	cube->setWorldTransform(transform);


	//Balance evaluation
	unbalanced = isBalanced(ragdoll,deltatime);

	//Pose Equilibre
	{
		ragdoll->m_stance[JOINT_PELVIS_SPINE] = btVector3(0, 0, 0);
		ragdoll->m_stance[JOINT_SPINE_TORSO] = btVector3(0, 0, 0);
		ragdoll->m_stance[JOINT_LEFT_HIP] = btVector3(0, 0, -0);
		ragdoll->m_stance[JOINT_RIGHT_HIP] = btVector3(0, 0, 0);
		ragdoll->m_stance[JOINT_LEFT_KNEE] = btVector3(0, 0, -0);
		ragdoll->m_stance[JOINT_RIGHT_KNEE] = btVector3(0, 0, 0);
		ragdoll->m_stance[JOINT_LEFT_FOOT] = btVector3(0, 0, 0);
		ragdoll->m_stance[JOINT_RIGHT_FOOT] = btVector3(0, 0, -0);
		ragdoll->m_stance[JOINT_LEFT_PELVIS] = btVector3(0, 0, -0);
		ragdoll->m_stance[JOINT_RIGHT_PELVIS] = btVector3(0, 0, 0);
		ragdoll->m_stance[JOINT_LEFT_KNEE_SWING] = btVector3(0, 0, -0);
		ragdoll->m_stance[JOINT_RIGHT_KNEE_SWING] = btVector3(0, 0, 0);
		ragdoll->m_stance[JOINT_LEFT_FOOT_SWING] = btVector3(0, 0, 0);
		ragdoll->m_stance[JOINT_RIGHT_FOOT_SWING] = btVector3(0, 0, -0);
		ragdoll->m_stance[ragdoll->JOINT_LEFT_ELBOW] = btVector3(0, 0, 0);
	}

	//Generate a new step Change foot
	if (unbalanced && !stepping && !lastStep)
	{
		first = true;
		stepping = true;
		right_leg = !right_leg;

		//Compute goal foot position
		xd = 0;
		zd = 0;
		xd += ragdoll->getCOMvel()[0];
		zd += ragdoll->getCOMvel()[2];
		

		if (abs(zd) > 0.6*ragdoll->scale)
			zd = ragdoll->scale * 0.6 * zd / abs(zd);
		if (abs(xd) > 0.4 * ragdoll->scale)
			xd = ragdoll->scale * 0.4 * xd / abs(zd);

		xd *= ragdoll->scale / 1.75;
		zd *= ragdoll->scale / 1.75;

		xd += ragdoll->scale * (0.05 + 0.191 / 2);
				if (right_leg)
			xd -= ragdoll->scale * (0.05 + 0.191);

		xd += ragdoll->m_bodies[ragdoll->BODYPART_PELVIS]->getCenterOfMassPosition()[0];
		zd += ragdoll->m_bodies[ragdoll->BODYPART_PELVIS]->getCenterOfMassPosition()[2];
	

		
		Controller::stepTime = 0.1845 + fabs(ragdoll->getCOMvel()[2]) * 0.2722;
	}
	else if (((stepping || lastStep) && p < stepTime) || (p>=stepTime && ragdoll->m_bodies[ragdoll->BODYPART_LEFT_FOOT]->getCenterOfMassPosition()[1]>0.05 && ragdoll->m_bodies[ragdoll->BODYPART_RIGHT_FOOT]->getCenterOfMassPosition()[1]>0.005))
	{//Step until the time is done and both feet touch the ground
		p += deltatime;
		if (p >= 1)
			p = 1;
	}
	else
	{
		if (!unbalanced && stepping == true && !lastStep && (ragdoll->m_bodies[ragdoll->BODYPART_LEFT_FOOT]->getCenterOfMassPosition()-ragdoll->m_bodies[ragdoll->BODYPART_RIGHT_FOOT]->getCenterOfMassPosition()).length() > 0.4)
		{
			//create a last step for both feet to end together for human like motion
			lastStep = true;
			right_leg = !right_leg;
			
			xd = ragdoll->m_bodies[ragdoll->BODYPART_LEFT_FOOT]->getCenterOfMassPosition()[0] + ragdoll->scale * (0.05 + 0.191);
			zd = ragdoll->m_bodies[ragdoll->BODYPART_LEFT_FOOT]->getCenterOfMassPosition()[2];
			if (right_leg)
			{
				xd = ragdoll->m_bodies[ragdoll->BODYPART_RIGHT_FOOT]->getCenterOfMassPosition()[0] - ragdoll->scale * (0.05 + 0.191);
				zd = ragdoll->m_bodies[ragdoll->BODYPART_RIGHT_FOOT]->getCenterOfMassPosition()[2];
			}
			Controller::stepTime *= 1.2;
			btTransform transform;
			transform.setOrigin(btVector3(xd, 0.5, zd));
			cube->setWorldTransform(transform);
		}
		else
		{
			lastStep = false;
			stepping = false;
		}
			
		//Reset
		p = 0;
		if (!unbalanced && !lastStep)
		{
			first = false;
		}
			
	}

	//Cancel noisy steps
	if (stepTime < filtre && !lastStep)
	{
		unbalanced = false;
		stepping = false;
		first = false;
		right_leg = !right_leg;
		p = 0;
	}

	//TODO verif si pas stepping plutot
	if (unbalanced || lastStep)
	{
		pasEffectue = true;
		fit(ragdoll, right_leg, p/stepTime, false,cube,facingDir);
	}

}
