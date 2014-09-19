#pragma once
#include "GeoPoint.h"
#include "PointGridIndex.h"
using namespace std;

extern MapDrawer md;

struct PtForce
{
	double forceX;
	double forceY;

	PtForce(double _forceX, double _forceY)
	{
		this->forceX = _forceX;
		this->forceY = _forceY;
	}

	PtForce operator+(PtForce& rhs)
	{
		return PtForce(this->forceX + rhs.forceX, this->forceY + rhs.forceY);
	}
};

class PointMover
{
public:
	void run(PointGridIndex* _ptIndex);
	void drawPoints();

private:
	double range = 35.0;
	PointGridIndex* ptIndex;
	vector<GeoPoint*> pts;
	void movePoint(GeoPoint* pt);
	PtForce resultantForce(GeoPoint* pt);
	PtForce calForce(GeoPoint* p0, GeoPoint* p);
	GeoPoint* selectPointToMove();
};
