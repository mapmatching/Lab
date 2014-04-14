/* 
 * Last Updated at [2014/1/24 8:41] by wuhao
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
};

IndexedTraj::IndexedTraj(Traj* traj)
{
	flag = -1;
	this->traj = traj;
	cluster = NULL;
}