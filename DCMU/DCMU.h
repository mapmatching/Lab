#pragma once
#include <iostream>
#include "GeoPoint.h"
#include "PointGridIndex.h"
#include "Map.h"
#include "Denoiser.h"
#include "PtCluster.h"
#include "RoadGenerator.h"
#include "PointMover.h"
using namespace std;

extern Map roadNetwork;
extern PointGridIndex allPtIndex;
extern Area area;
extern MapDrawer md;
extern int gridWidth;

class DCMU
{
public:
	vector<PointGridIndex*> ptIndexes;
	void run();
	void run1();

private:
	//gird partition
	void gridClustering();
	void dfs(int row, int col, bool** dfsState, vector<pair<int, int>>& connectingCompnt);
	
	//denoise
	Denoiser denoiser;
	PointMover ptMover;

	//direction clustering
	PtCluster ptCluster;

	//road generation
};
