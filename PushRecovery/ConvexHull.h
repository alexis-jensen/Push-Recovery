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
#include <../canid/spline.h>
#include <stack>
#include <vector>
#include <algorithm> 

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

extern btScalar z;

class ConvexHull
{
public:

	
	ConvexHull() {}

	struct Point
	{
	public:
		float x, y;
		using const_reference = const Point;
		using value_type = Point;
		using pointer = Point*;
		using reference = Point&;
		using size_type = btSizeType;
	};

	std::vector<Point*> corners;

	std::vector<Point*> hull;


	void ConvexHull::convexAdd(float x, float y);

	float ConvexHull::cross3(Point* o, Point* a, Point* b);

	bool ConvexHull::inside(Point* p);

	void ConvexHull::convexHull();

	float ConvexHull::distance_point(ConvexHull::Point* p1, ConvexHull::Point* p2);

	ConvexHull::Point* ConvexHull::cross_line(ConvexHull::Point* A1, ConvexHull::Point* B1, ConvexHull::Point* A2, ConvexHull::Point* B2);
};
