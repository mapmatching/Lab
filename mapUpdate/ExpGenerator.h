#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include "GeoPoint.h"
#include "TrajReader.h"
#include "MapMatching.h"

using namespace std;

extern Map roadNetwork;
extern Map originalRoadNetwork;
extern MapDrawer md;

class ExpGenerator
{
public:
	Area* area;
	string inputFolder = "D:\\trajectory\\singapore_data\\201202\\every day\\";
	vector<string> inputFileNames;
	string outputFolder = "D:\\trajectory\\singapore_data\\experiments\\big area\\";
	string newMMTrajsFileName = "newMMTrajs.txt";
	ofstream newMMTrajsFile;

	list<Traj*> rawTrajs;
	list<Traj*> trajsInArea;
	list<Traj*> doneTrajs;

	
	//driver func	
	void genExpData();
	void deleteForGeo();
	void deleteType1();
	void deleteType2();
	void deleteType3();


	void setArea(Area* area);
	//void dumpToFile(list<GeoPoint*>& source, string filename);

	

private:
	void genExpData(string rawTrajFilePath);
	void readRawTrajs(string rawTrajFilePath);
	void doSplit();
	void doMM(Map* roadNetwork, list<Traj*>& trajs, double thresM = 50.0);
	void outputNewTrajs(list<Traj*>& trajs);
	bool overDistLimit(GeoPoint* pt1, GeoPoint* pt2);
	void dumpTo(list<Traj*>& source, list<Traj*>& dest);
	void deleteList(list<Traj*>& victimList);
	//parameters in split func
	double limitSpeed = 50.0;
	double limitDist = 400;
	double limitTime = 100;
};

/*void genExperimentData()
{
	//////////////////////////////////////////////////////////////////////////
	///第一次匹配：原地图
	///第二次匹配：随机删除若干条路的地图
	///输出：除了第一次和第二次匹配都失败的轨迹外的所有轨迹
	//////////////////////////////////////////////////////////////////////////

	string mm1stPath = "D:\\trajectory\\singapore_data\\experiments\\wy_MMTrajs1.txt";
	string mm2ndPath = "D:\\trajectory\\singapore_data\\experiments\\wy_MMTrajs2.txt";
	ifstream mm1Ifs(mm1stPath);
	ifstream mm2Ifs(mm2ndPath);
	ofstream ofs("wy_MMTrajs_exp.txt");
	ofs << fixed << showpoint << setprecision(8);
	bool lastOutputIsNegative1 = false;
	while (mm1Ifs)
	{
		double lat, lon;
		int time, mmRoadId1, mmRoadId2;
		mm1Ifs >> time;
		mm2Ifs >> time;
		if (time != -1)
		{ 
			mm1Ifs >> lat >> lon >> mmRoadId1;
			mm2Ifs >> lat >> lon >> mmRoadId2;
		}
		if (mm1Ifs.fail() | mm2Ifs.fail())
			break;
		if (time == -1 && !lastOutputIsNegative1)
		{
			ofs << -1 << endl;
			lastOutputIsNegative1 = true;
		}
		if (time != -1)
		{
			if (mmRoadId1 != -1)
			{
				ofs << time << " " << lat << " " << lon << " " << mmRoadId2 << endl;
				lastOutputIsNegative1 = false;
			}
			else
			{
				if (!lastOutputIsNegative1)
				{
					ofs << -1 << endl;
					lastOutputIsNegative1 = true;
				}
			}
		}
	}
	mm2Ifs.close();
	mm1Ifs.close();
}*/


