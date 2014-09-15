#include "PointMover.h"


PtForce PointMover::calForce(GeoPoint* p0, GeoPoint* p)
{
	//////////////////////////////////////////////////////////////////////////
	///计算p对p0的作用力
	//////////////////////////////////////////////////////////////////////////
	double dist = GeoPoint::distM(p0, p);
	double force = dist * dist / 100;
}

PtForce PointMover::resultantForce(GeoPoint* pt)
{
	//////////////////////////////////////////////////////////////////////////
	///求作用于pt的合力
	//////////////////////////////////////////////////////////////////////////
	vector<GeoPoint*> nearPts;
	ptIndex->getNearPts(pt, range, nearPts);
	PtForce resultantF(0, 0);
	for (int i = 0; i < nearPts.size(); i++)
	{
		resultantF = resultantF + calForce(pt, nearPts[i]);
	}
	return resultantF;
}

void PointMover::movePoint(GeoPoint* pt)
{
	double alpha = 0.1;
	PtForce resultantF = resultantForce(pt);
	double moveLat = resultantF.forceY * alpha;
	double moveLon = resultantF.forceX * alpha;
	pt->lat += moveLat;
	pt->lon += moveLon;
	printf("moveLatM = %lf, moveLonM = %lf", moveLat * GeoPoint::geoScale, moveLon * GeoPoint::geoScale);
}

GeoPoint* PointMover::selectPointToMove()
{
	int victimId = int(((double)rand()) / RAND_MAX * (pts.size() - 1));
	return pts[victimId];
}

void PointMover::run(PointGridIndex* _ptIndex)
{
	this->ptIndex = _ptIndex;
	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for (list<GeoPoint*>::iterator iter = ptIndex->grid[row][col]->begin(); iter != ptIndex->grid[row][col]->end(); ++iter)
			{
				pts.push_back(*iter);
			}
		}
	}
	int iteration = 10000;
	for (int i = 0; i < iteration; i++)
	{
		GeoPoint* victimPt = selectPointToMove();
		movePoint(victimPt);
	}
}

void PointMover::drawPoints()
{
	for (int i = 0; i < pts.size(); i++)
	{
		md.drawPoint(Gdiplus::Color::Black, pts[i]->lat, pts[i]->lon);
	}
}