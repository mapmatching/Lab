#pragma once

#include "GeoPoint.h"
#include <string>
#include <list>
#include <iostream>
#include <fstream>
using namespace std;

class Traj
{
public:
	static void read(string trajFilePath, list<Traj*>& dest);
	int size() const;
	void push_back(GeoPoint* pt);
	void pop_back();
	GeoPoint* getNext()

//private:
	list<GeoPoint*> traj;

};