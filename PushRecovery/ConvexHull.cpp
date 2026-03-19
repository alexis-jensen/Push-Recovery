#include "ConvexHull.h"


void ConvexHull::convexAdd(float x, float y)
{
	Point* p = new Point;
	p->x = x;
	p->y = y;
	corners.push_back(p);
};

float ConvexHull::cross3(Point* o, Point* a, Point* b)
{
	return (a->x - o->x) * (b->y - o->y) - (a->y - o->y) * (b->x - o->x);
};

bool comparator(ConvexHull::Point* i, ConvexHull::Point* j)
{
	if (i->x < j->x)
	{
		return true;
	}
	if ((i->x == j->x) && (i->y < j->y))
	{
		return true;
	}
	return false;
}

void ConvexHull::convexHull()
{
	std::sort(corners.begin(), corners.end(), comparator);
	if (corners.size() <= 1)
	{
		return;
	}

	std::vector<Point*> lower;
	for (Point* p : corners)
	{
		while (lower.size() >= 2 && cross3(lower[lower.size() - 2], lower[lower.size() - 1], p) <= 0)
			lower.pop_back();
		lower.push_back(p);
	}

	std::vector<Point*> upper;
	std::vector<Point*> reverse = corners;
	std::reverse(reverse.begin(), reverse.end());
	for (Point* p : reverse)
	{
		while (upper.size() >= 2 && cross3(upper[upper.size() - 2], upper[upper.size() - 1], p) <= 0)
			upper.pop_back();
		upper.push_back(p);
	}

	lower.pop_back();
	lower.insert(lower.end(), upper.begin(), upper.end() - 1);

	for (Point* p : lower)
	{
		hull.push_back(p);
	}
};

bool ConvexHull::inside(Point* p)
{
	bool inside = true;
	for (int i = 1; i < hull.size(); i++)
	{
		float temp = cross3(hull[i - 1], hull[i], p);
		if (temp < 0)
			inside = false;
	}
	return inside;
}

float ConvexHull::distance_point(ConvexHull::Point* p1, ConvexHull::Point* p2)
{
	return btSqrt((p1->x - p2->x) * (p1->x - p2->x) + (p1->y - p2->y) * (p1->y - p2->y));
}

ConvexHull::Point* ConvexHull::cross_line(ConvexHull::Point* A1, ConvexHull::Point* B1, ConvexHull::Point* A2, ConvexHull::Point* B2)
{
	ConvexHull::Point* result = new ConvexHull::Point();

	//Find the coef of the lign equations
	//(every point on(A1, B1) solve a1X + b1Y + c1 = 0)
	if ((A1->x - B1->x) != 0 && (A2->x - B2->x) != 0)
	{
		float a1 = (A1->y - B1->y) / (A1->x - B1->x);
		float a2 = (A2->y - B2->y) / (A2->x - B2->x);
		float b1 = -1;
		float b2 = -1;
		float c1 = A1->y - a1 * A1->x;
		float c2 = A2->y - a2 * A2->x;
		try
		{
			result->x = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
			result->y = (c1 * a2 - a1 * c2) / (a1 * b2 - a2 * b1);
			return result;
		}
		catch (...)
		{
			if (a1 == a2 && b1 == b2 && c1 == c2)
			{
				//printf("Same line");
				return result;
			}
			else
			{
				//printf("Parallele lines");
				return result;  //Parallele lines
			}
		}
	}
	else if ((A1->x - B1->x) == 0 && (A2->x - B2->x) != 0)
	{
		//if (A1, B1) is vertical
		float a2 = (A2->y - B2->y) / (A2->x - B2->x);
		float c2 = A2->y - a2 * A2->x;
		result->x = A1->x;
		result->y = a2 * A1->x + c2;
		return result;
	}
	else if (((A2->x - B2->x) == 0 && (A1->x - B1->x) != 0))
	{
		// #if (A1, B1) is vertical
		float a1 = (A1->y - B1->y) / (A1->x - B1->x);
		float c1 = A1->y - a1 * A1->x;
		result->x = A2->x;
		result->y = a1 * A2->x + c1;
		return result;
	}
	else if (((A2->x - B2->x) == 0 && (A1->x - B1->x) == 0))
	{
		//printf("Parallele lines");
		return result;
	}
	else
	{
		//#Parallele lines else :
		return result;
	}
}