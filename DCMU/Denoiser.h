/* 
 * Last Updated at [2014/9/9 10:11] by wuhao
 */
#pragma once
#include <iostream>
#include "GeoPoint.h"
#include "PointGridIndex.h"
#include <list>
#include <vector>

using namespace std;
extern Area area;
extern int gridWidth;

class Denoiser
{
public:
	void run(PointGridIndex* _ptIndex);
	void drawPts(MapDrawer& md);
	PointGridIndex* ptIndex;
private:
	void calLMD(GeoPoint* pt, int k, double kNNThresholdM);
	void outlierValidation(GeoPoint* pt, int gridRange, double supportRatio);
	void outlierReValidation(GeoPoint* pt);
	void updatePtIndex(PointGridIndex* ptIndex);	
};