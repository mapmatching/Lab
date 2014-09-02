/* 
 * Last Updated at [2014/4/22 10:44] by wuhao
 */
#pragma once
#include <list>
#include "GeoPoint.h"
using namespace std;
class IndexedTraj;

typedef list<IndexedTraj*> Cluster;
typedef list<GeoPoint*> Traj;

class IndexedTraj
{
public:
	int flag;
	Traj* traj;
	Cluster* cluster;
	IndexedTraj(Traj* traj);
	double lengthM(); //返回轨迹长度，单位为M
};

IndexedTraj::IndexedTraj(Traj* traj)
{
	flag = -1;
	this->traj = traj;
	cluster = NULL;
}

double IndexedTraj::lengthM()
{
	Traj::iterator ptIter = traj->begin();
	Traj::iterator nextIter = traj->begin(); nextIter++;
	double length = 0;
	while (nextIter != traj->end())
	{
		length += GeoPoint::distM(*ptIter, *nextIter);
		ptIter++;
		nextIter++;
	}
	return length;
}