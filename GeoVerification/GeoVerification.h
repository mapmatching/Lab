/* 
 * Last Updated at [2014/9/3 21:22] by wuhao
 */
#pragma once
#include <iostream>
#include "map.h"
using namespace std;
typedef pair<GeoPoint, GeoPoint> Segment;

extern Map roadNetwork;
extern Map originalRoadNetwork;
extern MapDrawer md;

class GeoVerification
{
public:
	GeoVerification(Map& roadNetwork); //已废弃
	GeoVerification(){};
	void verificate(vector<Figure*>& genFigures, MapDrawer& md);

private:
	vector<Edge*> delEdges_oneway;
	bool verificateOneSegment(Segment segment, vector<Figure*>& genFigures);
	void clipEdges(vector<Edge*>& delEdges);
	//Map roadNetwork;
	double thresholdM = 20.0; //匹配阈值，找不到距离segment小于25m的路的认为匹配失败
	double clipThresM = 20.0; //每个segment的长度，不是准确长度，而是保证每个长度不长于这个值
	double correctLengthM = 0; //记录匹配正确的长度
	double totalDelLengthM = 0; //记录删除道路的总长度
	double totalGenLengthM = 0; //记录生成道路的总长度
	vector<Segment> segments;
};



