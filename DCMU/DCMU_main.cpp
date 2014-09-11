#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <time.h>
#include "MapDrawer.h"
#include "Map.h"
#include <list>
#include <vector>
#include <set>
#include <math.h>
#include "GeoPoint.h"
#include "StringOperator.h"
#include <iomanip>
#include "PolylineGenerator.h"
#include "PointGridIndex.h"
#include "StringOperator.h"
#include "ExpGenerator.h"
#include "TrajDrawer.h"
#include "DCMU.h"

#define eps 1e-8
#define INFINITE 999999999
#define PI 3.1415926535898
using namespace std;
using namespace Gdiplus;

//Area area(1.294788, 1.327723, 103.784667, 103.825200); //small
Area area(1.294788, 1.393593, 103.784667, 103.906266); //big

Map roadNetwork;
Map originalRoadNetwork; //Œ¥”√
MapDrawer md;
PointGridIndex allPtIndex;
int gridWidth = 900;

void initialization()
{
	roadNetwork.setArea(&area);
	roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 150);
	roadNetwork.deleteEdges("D:\\trajectory\\singapore_data\\experiments\\big area\\deletedEdges.txt");
	
	TrajReader tr("D:\\trajectory\\singapore_data\\experiments\\big area\\newMMTrajs_unmatched.txt");
	list<Traj*> trajs;
	tr.readTrajs(trajs);// , 50000);
	list<GeoPoint*> allPts;
	for each(Traj* traj in trajs)
	{
		for each (GeoPoint* pt in *traj)
		{
			allPts.push_back(pt);
		}
	}
	allPtIndex.createIndex(allPts, &area, gridWidth);

	md.setArea(&area);
	md.setResolution(5000);
}

void main()
{
	initialization();
	DCMU dcmu;

	md.newBitmap();
	md.lockBits();
	dcmu.run();
	md.unlockBits();
	md.saveBitmap("cluster.png");
}