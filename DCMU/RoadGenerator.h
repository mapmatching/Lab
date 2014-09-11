#pragma once
#include "GeoPoint.h"
#include "PtCluster.h"
#include "PolylineGenerator.h"
#include "MapDrawer.h"
#include <iomanip>
#include <set>
#define eps 1e-8

using namespace std;
extern MapDrawer md;

class RoadGenerator
{
public:
	void run(PointGridIndex* _allPtIndex, vector<Cluster*>& _clusters);
	

private:
	PointGridIndex* allPtIndex;
	vector<Cluster*> clusters;

	bool isATinyCluster(Cluster* cluster);
	double cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3);
	double distM(GeoPoint* pt, vector<GeoPoint*>& polyline);
	bool isBadClusterEx(Cluster* cluster, vector<Cluster*>& clusters, MapDrawer& md);
	void genPolyLine(Cluster* cluster, MapDrawer& md);
	void genAllPolyLines();
	void drawAllPolylines(bool doOutput = true);
	void drawPolyline(Cluster* cluster, MapDrawer& md, Gdiplus::Color color);
	
	//refinement
	GeoPoint* intersectCheck(GeoPoint* p1, GeoPoint* p2, vector<GeoPoint*>& polyline);
	void refineOnePolyline(Cluster* objectCluster);
	void doRefinement();

};