/* 
 * Last Updated at [2014/4/21 10:01] by wuhao
 */
#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include "GeoPoint.h"
#include <iomanip>
using namespace std;
#define INF 999999999
#define INVALID_ROADID -1
typedef list<GeoPoint*> Traj;

class TrajReader
{
public:
	TrajReader();
	TrajReader(string filePath); //构造函数时就打开轨迹文件 
	void open(string filePath); //打开轨迹文件，轨迹文件路径为filePath
	void readTrajs(vector<Traj*>& dest, int count = INF); //读入count条轨迹放入dest(dest会先清空)，默认为全部读入，并关闭文件
	void readTrajs(list<Traj*>& dest, int count = INF); //读入count条轨迹放入dest(dest会先清空)，默认为全部读入。并关闭文件
	static void outputTrajs(list<Traj*>& trajs, string filePath, int count = INF); //将trajs内count条轨迹按照标准格式输出至filePath

private:
	//设定区域，在区域外的轨迹不读
	double minLat;
	double maxLat;
	double minLon;
	double maxLon;
	ifstream trajIfs;
};
