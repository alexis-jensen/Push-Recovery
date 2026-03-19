#pragma once
#define TINY 0.000000001

class Spline
{
	public: 

	double tValues[4];
	double Values[4];

	Spline() {
		this->setKnot(0, 0, 0);
		this->setKnot(1, 0.33, 0);
		this->setKnot(2, 0.66, 0);
		this->setKnot(3, 1, 0);
	}
	void setKnot(int i, double tval,double val) {
		this->tValues[i] = tval;
		this->Values[i] = val;
	}

	mutable int lastIndex = 0;

	int getFirstLargerIndex(double t) const
	{
		int size = 4;
		for (int i = 0; i < size; i++)
		{
			int index = (i + lastIndex) % size;
			if (t < tValues[index])
			{
				lastIndex = index;
				return index;
			}
		}
		return size;
	}

	double evaluate_catmull_rom(double t) const
	{
		int size = 4;
		if (t <= tValues[0]) return Values[0];
		if (t >= tValues[size - 1]) return Values[size - 1];
		int index = getFirstLargerIndex(t);

		//now that we found the interval, get a value that indicates how far we are along it
		t = (t - tValues[index - 1]) / (tValues[index] - tValues[index - 1]);
		//printf("Progress: %f\n",t);
		//approximate the derivatives at the two ends
		double t0, t1, t2, t3;
		double p0, p1, p2, p3;
		p0 = (index - 2 < 0) ? (Values[index - 1]) : (Values[index - 2]);
		p1 = Values[index - 1];
		p2 = Values[index];
		p3 = (index + 1 >= size) ? (Values[index]) : (Values[index + 1]);

		t0 = (index - 2 < 0) ? (tValues[index - 1]) : (tValues[index - 2]);
		t1 = tValues[index - 1];
		t2 = tValues[index];
		t3 = (index + 1 >= size) ? (tValues[index]) : (tValues[index + 1]);

		double d1 = (t2 - t0);
		double d2 = (t3 - t1);

		if (d1 > -TINY && d1 < 0) d1 = -TINY;
		if (d1 < TINY && d1 >= 0) d1 = TINY;
		if (d2 > -TINY && d2 < 0) d2 = -TINY;
		if (d2 < TINY && d2 >= 0) d2 = TINY;

#ifdef FANCY_SPLINES
		T m1 = (p2 - p0) * (1 - (t1 - t0) / d1);
		T m2 = (p3 - p1) * (1 - (t3 - t2) / d2);
#else
		double m1 = (p2 - p0) * 0.5;
		double m2 = (p3 - p1) * 0.5;
#endif

		t2 = t * t;
		t3 = t2 * t;

		//and now perform the interpolation using the four hermite basis functions from wikipedia
		return p1 * (2 * t3 - 3 * t2 + 1) + m1 * (t3 - 2 * t2 + t) + p2 * (-2 * t3 + 3 * t2) + m2 * (t3 - t2);
	}
};