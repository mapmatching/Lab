#pragma once
#include <iostream>
#include "map.h"
using namespace std;
typedef pair<GeoPoint, GeoPoint> Segment;

extern Map roadNetwork;
extern Map originalRoadNetwork;
extern MapDrawer md;

class GeoValidation
{
public:
	GeoValidation(Map& roadNetwork);
	void validate(vector<Figure*>& figures, MapDrawer& md);

private:
	bool validateOneSegment(Segment segment);
	void clipEdges(vector<Figure*>& figures);
	//Map roadNetwork;
	double thresholdM = 20.0;
	double clipThresM = 20.0;
	double correctLengthM = 0;
	double totalLengthM = 0;
	vector<Segment> segments;
};



