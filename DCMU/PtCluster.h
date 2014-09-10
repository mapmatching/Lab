#pragma once
#include <iostream>
#include "GeoPoint.h"
#include "PointGridIndex.h"
#include <list>
#include <vector>
#define PI 3.1415926535898

using namespace std;
extern Area area;
extern int gridWidth;
extern MapDrawer md;

struct Cluster
{
	list<GeoPoint*> pts;
	double direction;
	int clusterId;
	vector<GeoPoint*> polyline;
	double polylineLengthM;
	double avgDirection;

	Area area; //记录cluster的MBR	

	Cluster()
	{
		direction = 0;
		area.setArea(999, -999, 999, -999);
	}

	void add(GeoPoint* pt)
	{
		//////////////////////////////////////////////////////////////////////////
		///将pt加入cluster，并将pt的clusterId域更新为该cluster的Id同时重新计算平均direction
		///为了加快getPtsNearCluster操作，加入点时会同时更新当前cluster的mbr
		///[ATTENTION]:这儿不会判断pt的原来clusterId
		//////////////////////////////////////////////////////////////////////////
		pt->clusterId = clusterId;
		direction = (pts.size() * direction + pt->direction) / (pts.size() + 1);
		pts.push_back(pt);
		//update mbr
		if (pt->lat > area.maxLat) area.maxLat = pt->lat;
		if (pt->lat < area.minLat) area.minLat = pt->lat;
		if (pt->lon > area.maxLon) area.maxLon = pt->lon;
		if (pt->lon < area.minLon) area.minLon = pt->lon;
	}

};

class PtCluster
{
public:
	void run(PointGridIndex* _ptIndex);
	void drawClusters(MapDrawer& md);
	vector<Cluster*> clusters;

private:
	PointGridIndex* ptIndex;
	list<GeoPoint*> pts;
	//计算点方向
	void calPtsDirs();
	void count(GeoPoint* p0, GeoPoint* p, vector<int>& countVec, double d, double l, double angleStep);
	void calDir(GeoPoint* p0, double angleStep, double d, double l);

	//方向聚类
	void doDirCluster();
	void getPtsNearCluster(Cluster* cluster, double distThresM, double angleThres, vector<GeoPoint*>& nearPts);
	bool fetch(Cluster* cluster, double distThresM, double angleThres);
	Cluster* genOneCluster(int clusterId, GeoPoint* seedPt, double distThresM, double angleThres);

	Gdiplus::Color randomColor()
	{
		int r = int(((double)rand()) / RAND_MAX * 255);
		int g = int(((double)rand()) / RAND_MAX * 255);
		int b = int(((double)rand()) / RAND_MAX * 255);
		Gdiplus::Color color((byte)r, (byte)g, (byte)b);
		return color;
	}
};
