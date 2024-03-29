/* 
 * Last Updated at [2014/1/24 22:23] by wuhao
 */
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <time.h>
#include "MapDrawer.h"
#include "Map.h"
#include <list>
#include <vector>
#include "GeoPoint.h"
#include "StringOperator.h"
#include "IndexedTraj.h"
#include "MapMatching.h"
#include <iomanip>
#include "PolylineGenerator.h"
#include "TrajReader.h"
#include "TrajDrawer.h"
#include "Matrix.h"

#define eps 1e-8
#define INFINITE 999999999
using namespace std;

//Area area(1.294788, 1.327723, 103.784667, 103.825200); //small
//Area area(1.294788, 1.393593, 103.784667, 103.906266); //big1
//Area area(1.343593, 1.442398, 103.784667, 103.906266); //big2
Area area(1.294788, 1.393593, 103.704667, 103.826266); //big3

int size = 15000;
//some switches
bool zoomed = true;
bool doExtendAndOutput = true;

typedef list<GeoPoint*> Traj;
extern Map roadNetwork;
MapDrawer md;

double limitSpeed = 50; //间隔大于33m/s的prune掉
double theta = 22.5;
double limitCosTheta = cos(theta/180*3.141592653); //转向角大于theta的prune掉
double limitDist = 400; //轨迹点之间超过300m的prune掉
double limitTime = 100; //采样间隔大于60秒的prune掉

//grid index
int gridWidth = 1200;
int gridHeight;
double gridSizeDeg;
list<IndexedTraj*> **grid = NULL;
int gridSearchRange = 1;

//cluster
double clusterDist = 50;
int supportThreshold = 5;
typedef list<IndexedTraj*> Cluster;

list<Traj*> rawTrajs;
list<Traj*> tempTrajs;
vector<IndexedTraj*> trajs;
vector<Cluster*> clusters;

//split
int outputFileIndex = 0;

/**********************************************************/
/*test code starts from here*/
vector<string> MMOutputFileName;
list<Figure*> gennedEgdes;
/*test code ends*/
/**********************************************************/
string trajDir;

//函数声明
bool overDistLimit(GeoPoint* pt1, GeoPoint* pt2);
void drawOneTraj(Gdiplus::Color color, Traj* traj);
bool smallerInX(simplePoint& pt1, simplePoint& pt2);

string getMMFileName(string fileName)
{
	//////////////////////////////////////////////////////////////////////////
	///根据轨迹文件名如"input_000011.txt"返回MM文件名"output_000011.txt"
	//////////////////////////////////////////////////////////////////////////
	string str = "000000";
	for (int i = 0; i < 6; i++)
		str[i] = fileName[i + 6];
	return "output_" + str + ".txt";
}

void readOneTrajectoryFail(std::string folderDir, std::string fileName)
{
	//////////////////////////////////////////////////////////////////////////
	///读一条"input_000011.txt"(fileName), 同时读一条"output_000011.txt"
	///只读取匹配失败的轨迹段
	//////////////////////////////////////////////////////////////////////////
	FILE *fpIn, *fpOut;
	double lat, lon;
	int time, roadId;
	double confidence;
	bool isStart = true;
	fpIn = fopen((folderDir + "//input//" + fileName).c_str(), "r");
	fpOut = fopen((folderDir + "//output//" + getMMFileName(fileName)).c_str(), "r");
	bool startFlag = true;
	Traj* traj = NULL;
	while (!feof(fpIn))
	{
		int flag = fscanf(fpIn, "%d,%lf,%lf", &time, &lat, &lon);
		fscanf(fpOut, "%d,%d,%lf", &time, &roadId, &confidence);
		//防止末行读入两遍
		if (flag == -1)
			break;
		if (roadId == -1) //map matching失败
		{
			if (!md.inArea(lat, lon)) //点在区域外
			{
				if (traj != NULL && traj->size() > 1)
				{
					startFlag = true;
					tempTrajs.push_back(traj);
				}
				continue;
			}
			if (startFlag)
			{
				traj = new Traj();
				GeoPoint* pt = new GeoPoint(lat, lon, time);
				traj->push_back(pt);
				startFlag = false;
			}
			else
			{
				//原地不动
				if (abs(lat - traj->back()->lat) < eps && abs(lon - traj->back()->lon) < eps)
					continue;
				GeoPoint* pt = new GeoPoint(lat, lon, time);
				traj->push_back(pt);
			}
		}
		else //map matching成功
		{
			if (!startFlag)
			{
				startFlag = true;
				if (traj->size() > 1)
				{
					tempTrajs.push_back(traj);
				}
				//else
					//delete(traj);
			}
		}
	}
	if (traj != NULL && traj->size() > 1)
	{
		tempTrajs.push_back(traj);
		//printf("traj made\n");
	}
	fclose(fpIn);
	fclose(fpOut);
}

Traj* readOneSRCTraj(std::string folderDir, std::string fileName, bool readMMFile)
{
	//////////////////////////////////////////////////////////////////////////
	///读一条轨迹文件,如"input_000011.txt",readMMFile为true时同时读入output_000011.txt
	//////////////////////////////////////////////////////////////////////////
	FILE *fpIn, *fpOut = NULL;
	double lat, lon;
	int time, roadId;
	double confidence;
	bool isStart = true;
	fpIn = fopen((folderDir + "input//" + fileName).c_str(), "r");
	if (readMMFile)
		fpOut = fopen((folderDir + "output//" + getMMFileName(fileName)).c_str(), "r");
	if (!fpIn)
	{
		cout << "open " + (folderDir + "input//" + fileName) + " error!\n";
		system("pause");
		exit(0);
	}
	Traj* traj = new Traj();
	while (!feof(fpIn))
	{
		int flag = fscanf(fpIn, "%d,%lf,%lf", &time, &lat, &lon);
		if (readMMFile)
			fscanf(fpOut, "%d,%d,%lf", &time, &roadId, &confidence);
		//防止末行读入两遍
		if (flag == -1)
			break;
		GeoPoint* pt;
		if (readMMFile)
			pt = new GeoPoint(lat, lon, time, roadId);
		else
			pt = new GeoPoint(lat, lon, time);
		traj->push_back(pt);
	}
	fclose(fpIn);
	if (readMMFile)
		fclose(fpOut);
	return traj;
}

void readSRCTrajs(string folderDir, string fileName)
{
	rawTrajs.push_back(readOneSRCTraj(folderDir, fileName, true));
	//MMOutputFileName.push_back(getMMFileName(fileName));
}

void genStdTrajFile(list<Traj*>& sourceTrajs, string fileName)
{
	//////////////////////////////////////////////////////////////////////////
	///将轨迹集合以标准格式输出到fileName文件中
	//////////////////////////////////////////////////////////////////////////
	ofstream ofs(fileName);
	ofs << fixed << showpoint << setprecision(8);
	for each (Traj* traj in sourceTrajs)
	{
		for (Traj::iterator ptIter = traj->begin(); ptIter != traj->end(); ptIter++)
		{
			ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*ptIter)->mmRoadId << endl;
		}
		ofs << "-1" << endl;
	}
	ofs.close();
}

double trajDistM(Traj* traj, double thresholdM = INFINITE)
{
	//////////////////////////////////////////////////////////////////////////
	///计算轨迹traj的长度并返回藏毒,单位为M
	///当计算长度时发现当前长度已经>thresholdM的话则不计算下去直接返回
	//////////////////////////////////////////////////////////////////////////
	double lengthM = 0;
	Traj::iterator ptIter = traj->begin(), nextPtIter = ptIter;
	nextPtIter++;
	while (1)
	{
		if (nextPtIter == traj->end())
			break;
		lengthM += GeoPoint::distM((*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
		if (lengthM > thresholdM)
			return lengthM;
		ptIter++;
		nextPtIter++;
	}
	return lengthM;
}

void doExtendForOneMMTraj(Traj* traj, list<Traj*>& dest, double extendDistM, double minTrajLength)
{
	//////////////////////////////////////////////////////////////////////////
	///4/21修改：轨迹两端只延伸一次，如果延伸后的点extendDistM内有路则保留，否则不延伸
	///如果子轨迹只有一个点则丢弃
	///[补充] 当轨迹长度小于minTrajLength的话就丢弃
	//////////////////////////////////////////////////////////////////////////
	dest.clear();
	Traj* rawTraj = traj;
	Traj* tmpTraj = NULL;
	//对每个点
	Traj::iterator currentPtIter = rawTraj->begin();
	GeoPoint* currentPt = (*currentPtIter), *prePt = currentPt;
	bool startFlag = true;
	while (currentPtIter != rawTraj->end())
	{
		currentPt = (*currentPtIter);
		if (overDistLimit(prePt, currentPt)) //与前一个点距离太远则切开
		{
			if (tmpTraj != NULL && tmpTraj->size() > 1 && trajDistM(tmpTraj) > minTrajLength)
			{
				dest.push_back(tmpTraj);
				tmpTraj = NULL;
			}
			startFlag = true;
		}
		if (currentPt->mmRoadId == -1) //map matching失败
		{
			if (!md.inArea(currentPt->lat, currentPt->lon)) //点在区域外则切开
			{
				if (tmpTraj != NULL && tmpTraj->size() > 1 && trajDistM(tmpTraj) > minTrajLength)
				{
					dest.push_back(tmpTraj);
					tmpTraj = NULL;
				}
				startFlag = true;
			}
			else //点在区域内
			{
				if (startFlag)
				{
					tmpTraj = new Traj();
					//向前扩展
					//4/21修改：新版本，最多只扩一次
					Traj::iterator frontExtendPtIter = currentPtIter;
					if (frontExtendPtIter != rawTraj->begin())
					{
						frontExtendPtIter--;
						GeoPoint* frontExtendPt = (*frontExtendPtIter);
						if (md.inArea(frontExtendPt->lat, frontExtendPt->lon)
						 && !overDistLimit(frontExtendPt, currentPt)
		                 &&	roadNetwork.getNearEdges(frontExtendPt->lat, frontExtendPt->lon, extendDistM).size() > 0)
							tmpTraj->push_front(frontExtendPt);
					}
					/*
					//下段代码是原版本，一直延伸到附近有路为止
					GeoPoint* extendPt = currentPt, *preExtendPt = currentPt;
					Traj::iterator extendPtIter = currentPtIter;
					while (md.inArea(extendPt->lat, extendPt->lon)//扩展点在区域内
						&& !overDistLimit(preExtendPt, extendPt)) //扩展点离前一个扩展点距离不长
						
					{
						
						if (roadNetwork.getNearEdges(extendPt->lat, extendPt->lon, extendDistM).size() > 0)
						//[注意]该语句可能会产生内存泄露			
							tmpTraj->push_front(extendPt);
						preExtendPt = extendPt;
						if (extendPtIter != rawTraj->begin())
						{
							extendPtIter--;
							extendPt = (*extendPtIter);
						}
						else
							break;
					} //while end*/
					startFlag = false;
				} //if (startFlag) end
				else
				{
					tmpTraj->push_back(currentPt);
				}
			}
		}
		else //map matching成功
		{
			if (!startFlag)
			{
				//向后扩展
				//4/21修改：新版本，最多只扩一次
				Traj::iterator backExtendIter = currentPtIter;
				backExtendIter++;
				if (backExtendIter != rawTraj->end())
				{
					GeoPoint* backExtendPt = (*backExtendIter);
					if (md.inArea(backExtendPt->lat, backExtendPt->lon)
						&& !overDistLimit(backExtendPt, currentPt)
						&& roadNetwork.getNearEdges(backExtendPt->lat, backExtendPt->lon, extendDistM).size() > 0)
						tmpTraj->push_back(backExtendPt);
				}
				/*
				//下段代码是原版本，一直延伸到附近有路为止
				GeoPoint* extendPt = currentPt, *preExtendPt = tmpTraj->back();
				Traj::iterator extendPtIter = currentPtIter;
				while (extendPtIter != rawTraj->end() //扩展点没到尾
					&& md.inArea(extendPt->lat, extendPt->lon) //扩展点在区域内
					&& !overDistLimit(preExtendPt, extendPt)) //扩展点离前一个扩展点距离不长
				{
					tmpTraj->push_back(extendPt);
					if (roadNetwork.getNearEdges(extendPt->lat, extendPt->lon, extendDistM).size() > 0)
						break;
					preExtendPt = extendPt;
					extendPtIter++;
					extendPt = (*extendPtIter);
				}*/

				if (tmpTraj->size() > 1)
				{
					dest.push_back(tmpTraj);
					tmpTraj = NULL;
				}
				else
					tmpTraj->clear();
				startFlag = true;
			}
		}
		prePt = currentPt;
		currentPtIter++;
	} //while end
	if (tmpTraj != NULL)
	{
		if (tmpTraj->size() > 1 && trajDistM(tmpTraj) > minTrajLength)
		{
			dest.push_back(tmpTraj);
			tmpTraj = NULL;
		}
		else
			tmpTraj->clear();
	}
}

void doExtend(list<Traj*>& src, list<Traj*>& dest, double extendDistM, double minTrajLength, bool doOutput = false)
{
	//////////////////////////////////////////////////////////////////////////
	///读入src, src必须为MM后的轨迹集合
	///对不匹配的子轨迹段两端延伸至extendDistM米内有路位置,并且轨迹点之间不能太远
	///输出至dest,doOutput为true时同时输出至文件
	///[附加]当轨迹距离长度低于minTrajLengthM的话则丢弃
	//////////////////////////////////////////////////////////////////////////
	ofstream ofs;
	if (doOutput)
	{
		ofs.open(trajDir + "wy_extended_unmatched_trajs.txt");
		ofs << fixed << showpoint << setprecision(8);
	}
	//对于每条轨迹
	for (list<Traj*>::iterator trajIter = src.begin(); trajIter != src.end(); trajIter++)
	{
		list<Traj*> extendSubTrajs;
		doExtendForOneMMTraj(*trajIter, extendSubTrajs, extendDistM, minTrajLength);
		dest.splice(dest.end(), extendSubTrajs);
	}

	cout << ">> extracting and extending unmatchied trajs finished" << endl;
	if (doOutput)
	{
		cout << ">> start output to file" << endl;
		for (list<Traj*>::iterator trajIter = dest.begin(); trajIter != dest.end(); trajIter++)
		{
			for (Traj::iterator ptIter = (*trajIter)->begin(); ptIter != (*trajIter)->end(); ptIter++)
			{
				ofs << (*ptIter)->time << " "
					<< (*ptIter)->lat << " "
					<< (*ptIter)->lon << " "
					<< (*ptIter)->mmRoadId << endl;
			}
			ofs << "-1" << endl;
		}
		cout << ">> output to file finished" << endl;
		ofs.close();
	}
}

void scanTrajFolder(string folderDir, void (*func)(string, string))
{
	//////////////////////////////////////////////////////////////////////////
	///遍历folderDir下的所有轨迹文件,轨迹文件目录有格式要求 
	//////////////////////////////////////////////////////////////////////////
	/*文件目录结构为
	* folderDir
	* |-input
	*   |-input_000011.txt ...
	* |-output
	*   |-output_000011.txt ...
	*/
	cout << ">> start scanning " << folderDir << endl;

	string tempS = folderDir + "\\input\\" + "*.txt";
	const char* dir = tempS.c_str();
	_finddata_t file;
	long lf;
	if ((lf = _findfirst(dir, &file)) == -1l)
		return;
	else
	{
		do
		{
			//readOneTrajectory(folderDir, file.name);
			//tempTrajs.push_back(readOneTrajectory(folderDir + "input\\" + file.name));
			func(folderDir, file.name);
		} while (_findnext(lf, &file) == 0);
		_findclose(lf);
		cout << ">> scanning " << folderDir << " finished" << endl;
		return;
	}
}

//划分轨迹
void readOneSRCTrajAndSplit(string folderDir, string fileName, ofstream& ofs)
{
	//////////////////////////////////////////////////////////////////////////
	///读入一条input_XXXXXX.txt轨迹文件,并将其拆分成若干条子轨迹(间隔距离太长时间太长的切断)
	//////////////////////////////////////////////////////////////////////////
	FILE *fpIn;
	double lat, lon;
	int time;
	bool isStart = true;
	fpIn = fopen((folderDir + "//input//" + fileName).c_str(), "r");
	GeoPoint prePt, currentPt;
	bool startFlag = true;

	while (!feof(fpIn))
	{
		int flag = fscanf(fpIn, "%d,%lf,%lf", &time, &lat, &lon);
		//防止末行读入两遍
		if (flag == -1)
			break;
		if (startFlag)
		{
			if (md.inArea(lat, lon))
			{
				ofs << time << " " << lat << " " << lon << " -1" << endl;
				prePt.lat = lat;
				prePt.lon = lon;
				prePt.time = time;
				startFlag = false;
			}
			else
				continue;
		}
		else
		{
			currentPt.lat = lat;
			currentPt.lon = lon;
			currentPt.time = time;
			if (md.inArea(lat, lon))
			{
				if (!overDistLimit(&prePt, &currentPt))
				{
					ofs << time << " " << lat << " " << lon << " -1" << endl;
					prePt.lat = lat;
					prePt.lon = lon;
					prePt.time = time;
					startFlag = false;
				}
				else
				{
					ofs << -1 << endl;
					ofs << time << " " << lat << " " << lon << " -1" << endl;
					prePt.lat = lat;
					prePt.lon = lon;
					prePt.time = time;
					startFlag = false;
				}
			}
			else
			{
				ofs << -1 << endl;
				startFlag = true;
			}
		}
	}
	if (startFlag == false)
		ofs << -1 << endl;
	fclose(fpIn);
}
void splitSRCTrajFiles(string folderDir)
{
	//////////////////////////////////////////////////////////////////////////
	///遍历folderDir\input下的所有轨迹文件,轨迹文件目录有格式要求 
	///将太远太长的轨迹切断,然后输出到folderDir下的splitedTrajs.txt
	//////////////////////////////////////////////////////////////////////////
	/*文件目录结构为
	* folderDir
	* |-input
	*   |-input_000011.txt ...
	*/
	cout << ">> start scanning " << folderDir << endl;
	ofstream ofs(folderDir + "splitedTrajs.txt");
	ofs << fixed << showpoint << setprecision(8);
	string tempS = folderDir + "input\\" + "*.txt";
	const char* dir = tempS.c_str();
	_finddata_t file;
	long lf;
	if ((lf = _findfirst(dir, &file)) == -1l)
		return;
	else
	{
		do
		{
			readOneSRCTrajAndSplit(folderDir, file.name, ofs);
		} while (_findnext(lf, &file) == 0);
		_findclose(lf);
		cout << ">> scanning " << folderDir << " finished" << endl;
		return;
	}
	ofs.close();
}

void splitTrajsAndOutput(list<Traj*>& rawTrajs)
{
	//////////////////////////////////////////////////////////////////////////
	///将
	//////////////////////////////////////////////////////////////////////////
	
	limitSpeed = 50.0;
	limitDist = 400;
	limitTime = 100;
	GeoPoint prePt, currentPt;
	ofstream ofs("splitedTrajs.txt");
	ofs << fixed << showpoint << setprecision(8);
	for (list<Traj*>::iterator trajIter = rawTrajs.begin(); trajIter != rawTrajs.end(); trajIter++)
	{
		bool startFlag = true;
		Traj* currentTraj = (*trajIter);
		for (Traj::iterator ptIter = currentTraj->begin(); ptIter != currentTraj->end(); ptIter++)
		{
			if (startFlag)
			{
				if (md.inArea((*ptIter)->lat, (*ptIter)->lon))
				{
					ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " -1" << endl;
					prePt.lat = (*ptIter)->lat;
					prePt.lon = (*ptIter)->lon;
					prePt.time = (*ptIter)->time;
					startFlag = false;
				}
				else
					continue;
			}
			else
			{
				currentPt.lat = (*ptIter)->lat;
				currentPt.lon = (*ptIter)->lon;
				currentPt.time = (*ptIter)->time;
				if (md.inArea((*ptIter)->lat, (*ptIter)->lon))
				{
					if (!overDistLimit(&prePt, &currentPt))
					{
						ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " -1" << endl;
						prePt.lat = (*ptIter)->lat;
						prePt.lon = (*ptIter)->lon;
						prePt.time = (*ptIter)->time;
						startFlag = false;
					}
					else
					{
						ofs << -1 << endl;
						ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " -1" << endl;
						prePt.lat = (*ptIter)->lat;
						prePt.lon = (*ptIter)->lon;
						prePt.time = (*ptIter)->time;
						startFlag = false;
					}
				}
				else
				{
					ofs << -1 << endl;
					startFlag = true;
				}
			}
		}
		if (startFlag == false)
			ofs << -1 << endl;
	}
}
//读入标准轨迹文件,格式:time lat lon mmRoadId
void readStdTrajs(string path, list<Traj*>& dest, int num = INFINITE)
{
	//////////////////////////////////////////////////////////////////////////
	///num代表读入轨迹数量，默认为INFINITE，代表全部读入
	//////////////////////////////////////////////////////////////////////////
	cout << ">> start reading std trajs" << endl;
	ifstream ifs(path);
	if (!ifs)
	{
		cout << "open traj file error" << endl;
		system("pause");
		exit(0);
	}
	bool isStart = true;
	int time, mmRoadId;
	double lat, lon;
	Traj* tmpTraj = NULL;
	int count = 0;
	while (ifs)
	{
		if (count == num)
		{
			break;
		}
		int time;
		ifs >> time;
		if (ifs.fail())
		{
			break;
		}
		if (time == -1)
		{
			isStart = true;
			if (tmpTraj != NULL && tmpTraj->size() > 1)
			{
				dest.push_back(tmpTraj);
				count++;
			}
			continue;
		}
		else
		{
			ifs >> lat >> lon >> mmRoadId;
			GeoPoint* pt = new GeoPoint(lat, lon, time, mmRoadId);
			if (isStart)
			{
				tmpTraj = new Traj();				
				tmpTraj->push_back(pt);
				isStart = false;
			}
			else
			{
				tmpTraj->push_back(pt);
			}
		}
	}
	cout << ">> reading std trajs finished" << endl;
}

void readStdTrajs(string path, vector<IndexedTraj*>& dest, int num = INFINITE)
{
	cout << ">> start reading std trajs" << endl;
	ifstream ifs(path);
	if (!ifs)
	{
		cout << "open traj file error" << endl;
		system("pause");
		exit(0);
	}
	bool isStart = true;
	int time, mmRoadId;
	double lat, lon;
	Traj* tmpTraj = NULL;
	IndexedTraj* tmpITraj = NULL;
	int count = 0;
	while (ifs)
	{
		if (count == num)
		{
			break;
		}
		int time;
		ifs >> time;
		if (ifs.fail())
		{
			break;
		}
		if (time == -1)
		{
			isStart = true;
			if (tmpTraj != NULL && tmpTraj->size() > 1)
			{
				tmpITraj = new IndexedTraj(tmpTraj);
				dest.push_back(tmpITraj);
				count++;
			}
			continue;
		}
		else
		{
			ifs >> lat >> lon >> mmRoadId;
			GeoPoint* pt = new GeoPoint(lat, lon, time, mmRoadId);
			if (isStart)
			{
				tmpTraj = new Traj();
				tmpTraj->push_back(pt);
				isStart = false;
			}
			else
			{
				tmpTraj->push_back(pt);
			}
		}
	}
	cout << ">> reading std trajs finished" << endl;
}

//raw data mapmatching
void rawMapMatching(list<Traj*>& trajs, string outputPath)
{
	cout << ">> start map matching" << endl;
	ofstream ofs(outputPath);
	ofs << fixed << showpoint << setprecision(8);
	//对每一条轨迹
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		list<Edge*> result = MapMatching(*(*trajIter), 50.0); //MapMatching
		Traj* traj = (*trajIter);
		if (traj == NULL)
			continue;
		//对每一个点
		Traj::iterator ptIter = traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //匹配成功
			{
				ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*edgeIter)->id << endl;
			}
			else //匹配失败
			{
				ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << "-1" << endl;
			}
			ptIter++;
			edgeIter++;
		}
		ofs << "-1" << endl;
		ofs.close();
		ofs << "output MM file to " << outputPath << endl;
	}
}

double cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3)
{
	//////////////////////////////////////////////////////////////////////////
	///求向量(pt1->pt2)与(pt2->pt3)夹角的余弦
	//////////////////////////////////////////////////////////////////////////
	double v1x = pt2->lon - pt1->lon;
	double v1y = pt2->lat - pt1->lat;
	double v2x = pt3->lon - pt2->lon;
	double v2y = pt3->lat - pt2->lat;
	return (v1x * v2x + v1y * v2y) / sqrt((v1x * v1x + v1y * v1y)*(v2x * v2x + v2y * v2y));
}

bool overDistLimit(GeoPoint* pt1, GeoPoint* pt2)
{
	//////////////////////////////////////////////////////////////////////////
	///时间间隔大于limitTime或者速度大于limitSpeed或者之间距离大于limitDist返回false
	//////////////////////////////////////////////////////////////////////////
	double  dist = GeoPoint::distM(*pt1, *pt2);
	double speed;
	if (pt2->time - pt1->time != 0)
	{
		speed = dist / abs(pt2->time - pt1->time);
	}
	else
		speed = 0;
	return dist > limitDist || abs(pt2->time - pt1->time) > limitTime || (speed > limitSpeed);
}

bool overAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3)
{
	return (cosAngle(pt1, pt2, pt3) < limitCosTheta);
}

void trajRefinement(Traj* traj)
{
	//////////////////////////////////////////////////////////////////////////
	///对一条匹配失败的轨迹进行筛选,留下"质量好"的轨迹段
	//////////////////////////////////////////////////////////////////////////
	if (traj->size() == 2)
	{
		if (!overDistLimit(traj->front(), traj->back()))
		{
			IndexedTraj* iTraj = new IndexedTraj(traj);
			trajs.push_back(iTraj);
		}
	}
	else if (traj->size() == 3)
	{
		Traj::iterator iter = traj->begin();
		GeoPoint* pt1 = (*iter);
		iter++;
		GeoPoint* pt2 = (*iter);
		iter++;
		GeoPoint* pt3 = (*iter);
		if (!overAngle(pt1, pt2, pt3))
		{
			if (!overDistLimit(pt1, pt2) && !overDistLimit(pt2, pt3))
			{
				Traj* tempTraj = new Traj();
				tempTraj->push_back(pt1);
				tempTraj->push_back(pt2);
				tempTraj->push_back(pt3);
				IndexedTraj* iTraj = new IndexedTraj(tempTraj);
				trajs.push_back(iTraj);
			}
		}
		else
		{
			if (!overDistLimit(pt1, pt2))
			{
				Traj* tempTraj = new Traj();
				tempTraj->push_back(pt1);
				tempTraj->push_back(pt2);
				IndexedTraj* iTraj = new IndexedTraj(tempTraj);
				trajs.push_back(iTraj);
			}
			if (overDistLimit(pt1, pt2) && !overDistLimit(pt2, pt3))
			{
				Traj* tempTraj = new Traj();
				tempTraj->push_back(pt2);
				tempTraj->push_back(pt3);
				IndexedTraj* iTraj = new IndexedTraj(tempTraj);
				trajs.push_back(iTraj);
			}
		}
	}
	else
	{
		Traj::iterator currentIter = traj->begin();
		Traj* tempTraj =  new Traj();
		while (currentIter != traj->end())
		{
			if (tempTraj->size() == 0)
			{
				Traj::iterator tempIter = currentIter;
				Traj::iterator nextIter = ++currentIter;
				if (nextIter == traj->end())
					break;
				if (!overDistLimit((*tempIter), (*nextIter)))
				{
					tempTraj->push_back((*tempIter));
					tempTraj->push_back((*nextIter));
				}
				else
					continue;
			}
			else
			{
				Traj::iterator iter = tempTraj->end();
				iter--;
				Traj::iterator preIter = iter;
				iter--;
				Traj::iterator prepreIter = iter;
				if (!overDistLimit((*preIter), (*currentIter)) && !overAngle((*prepreIter),(*preIter),(*currentIter)))
				{
					tempTraj->push_back((*currentIter));
				}
				else
				{
					IndexedTraj* iTraj = new IndexedTraj(tempTraj);
					trajs.push_back(iTraj);
					/*if (tempTraj->size() != 2 && tempTraj->size() != 3 && tempTraj->size() != 4)
					{
						cout << "size:" << tempTraj->size() << endl;
					}*/					
					tempTraj = new Traj();
					continue;
				}
			}									
			currentIter++;
		}
		//delete(traj);
	}
	
}

void refinement()
{
	//////////////////////////////////////////////////////////////////////////
	///对所有轨迹进行筛选,源轨迹集合为tempTrajs,目标轨迹集合为trajs
	//////////////////////////////////////////////////////////////////////////
	cout << ">> start refinement" << endl;
	for (list<Traj*>::iterator iter = tempTrajs.begin(); iter != tempTrajs.end(); iter++)
	{
		trajRefinement(*iter);
	}
	cout << ">> refinement finished" << endl;
}
//grid index
void createGridIndex(void(*pInsertFunc)(IndexedTraj*))
{
	//////////////////////////////////////////////////////////////////////////
	///将所有轨迹建立网格索引
	//////////////////////////////////////////////////////////////////////////
	cout << ">> start creating grid index" << endl;
	//initialization
	gridHeight = int((area.maxLat - area.minLat) / (area.maxLon - area.minLon) * double(gridWidth)) + 1;
	gridSizeDeg = (area.maxLon - area.minLon) / double(gridWidth);
	grid = new list<IndexedTraj*>*[gridHeight];
	
	for (int i = 0; i < gridHeight; i++)
		grid[i] = new list<IndexedTraj*>[gridWidth];
	
	printf("gridWidth = %d, gridHeight = %d\n", gridWidth, gridHeight);
	cout << "gridSize = " << gridSizeDeg * GeoPoint::geoScale << "m" << endl;
	for (vector<IndexedTraj*>::iterator iTrajIter = trajs.begin(); iTrajIter != trajs.end(); iTrajIter++)
	{
		pInsertFunc((*iTrajIter));
	}
	int count = 0;
	for (int row = 0; row < gridHeight; row++)
	{
		for (int col = 0; col < gridWidth; col++)
		{
			count += grid[row][col].size();
		}
	}
	cout << "grid traj count: " << count << endl;
	cout << ">> grid index created" << endl;
}
	//Lite edition
void createGridIndexForOneTraj_Lite(IndexedTraj* iTraj)
{
	//////////////////////////////////////////////////////////////////////////
	///将一条轨迹插入网格索引,将其加入轨迹上的点经过的所有网格
	///TODO:
	///		是否有必要实现经过所有的网格对其添加索引?
	//////////////////////////////////////////////////////////////////////////
	Traj* traj = iTraj->traj;
	for (Traj::iterator iter = traj->begin(); iter != traj->end(); iter++)
	{
		int row = ((*iter)->lat - area.minLat) / gridSizeDeg;
		int col = ((*iter)->lon - area.minLon) / gridSizeDeg;
		if (row >= gridHeight || row < 0 || col >= gridWidth || col < 0)
		{
			printf("pt(%lf, %lf), row = %d, col = %d\n", (*iter)->lat, (*iter)->lon, row, col);
			system("pause");
		}
		//该轨迹已经加入过某网格则不用再加
		if (grid[row][col].size() > 0 && grid[row][col].back() == iTraj)
			continue;
		else
			grid[row][col].push_back(iTraj);
	}
}

	//Full edition
void insertTrajIntoGrid(IndexedTraj* iTraj, int row, int col)
{
	//////////////////////////////////////////////////////////////////////////
	///将轨迹traj加入grid[row][col]中索引，如果已经加入过则不添加
	//////////////////////////////////////////////////////////////////////////
	
	if (grid[row][col].size() > 0 && grid[row][col].back() == iTraj)
		return;
	else
		grid[row][col].push_back(iTraj);
}

void createGridIndexForSegment(IndexedTraj *iTraj, GeoPoint* fromPT, GeoPoint* toPt)
{
	//////////////////////////////////////////////////////////////////////////
	///对traj中的fromPt->toPt段插入网格索引，经过的网格都加入其指针
	///如果与网格相交长度与网格边长之比小雨strictThreshold则不加入网格
	//////////////////////////////////////////////////////////////////////////
	if (iTraj == NULL)
		return;
	bool crossRow;
	bool strictThreshold = 0.1;
	GeoPoint* pt1 = fromPT;
	GeoPoint* pt2 = toPt;
	double x1 = pt1->lon - area.minLon;
	double y1 = pt1->lat - area.minLat;
	double x2 = pt2->lon - area.minLon;
	double y2 = pt2->lat - area.minLat;
	int row1 = y1 / gridSizeDeg;
	int row2 = y2 / gridSizeDeg;
	int col1 = x1 / gridSizeDeg;
	int col2 = x2 / gridSizeDeg;
	double A = y2 - y1;
	double B = -(x2 - x1);
	double C = -B * y1 - A * x1;
	int i, j;
	//pt1,pt2都在一个cell中
	if (row1 == row2 && col1 == col2)
	{
		insertTrajIntoGrid(iTraj, row1, col1);
		return;
	}
	//只穿越横向格子
	if (row1 == row2)
	{
		//头
		double headDist = ((min(col1, col2) + 1) * gridSizeDeg - min(x1, x2)) / gridSizeDeg;
		if (headDist / gridSizeDeg > strictThreshold)
			insertTrajIntoGrid(iTraj, row1, min(col1, col2));
		//中间
		for (i = min(col1, col2) + 1; i < max(col1, col2); i++)
		{
			insertTrajIntoGrid(iTraj, row1, i);
		}
		//尾
		double tailDist = (max(x1, x2) - max(col1, col2) * gridSizeDeg) / gridSizeDeg;
		if (tailDist / gridSizeDeg > strictThreshold)
			insertTrajIntoGrid(iTraj, row1, max(col1, col2));
		return;
	}
	//只穿越纵向格子
	if (col1 == col2)
	{
		//头
		double headDist = ((min(row1, row2) + 1) * gridSizeDeg - min(y1, y2)) / gridSizeDeg;
		if (headDist / gridSizeDeg > strictThreshold)
			insertTrajIntoGrid(iTraj, min(row1, row2), col1);
		//中间
		for (i = min(row1, row2) + 1; i < max(row1, row2); i++)
		{
			insertTrajIntoGrid(iTraj, i, col1);
		}
		//尾
		double tailDist = (max(y1, y2) - max(row1, row2) * gridSizeDeg) / gridSizeDeg;
		if (tailDist / gridSizeDeg > strictThreshold)
			insertTrajIntoGrid(iTraj, max(row1, row2), col1);
		return;
	}
	simplePoint pts[1000];
	int n_pts = 0;
	for (i = min(row1, row2) + 1; i <= max(row1, row2); i++)
	{
		pts[n_pts++] = std::make_pair((-C - B*i*gridSizeDeg) / A, i*gridSizeDeg);
	}
	for (i = min(col1, col2) + 1; i <= max(col1, col2); i++)
	{
		pts[n_pts++] = std::make_pair(i*gridSizeDeg, (-C - A*i*gridSizeDeg) / B);
	}
	std::sort(pts, pts + n_pts, smallerInX);

	GeoPoint* leftPt, *rightPt;
	if (x1 < x2)
	{
		leftPt = pt1;
		rightPt = pt2;
	}
	else
	{
		leftPt = pt2;
		rightPt = pt1;
	}
	double xL = leftPt->lon - area.minLon;
	double xR = rightPt->lon - area.minLon;
	double yL = leftPt->lat - area.minLat;
	double yR = rightPt->lat - area.minLat;

	//头
	double headDist = sqrt((xL - pts[0].first)*(xL - pts[0].first) + (yL - pts[0].second)*(yL - pts[0].second));
	if (headDist / gridSizeDeg > strictThreshold)
		insertTrajIntoGrid(iTraj, (int)(yL / gridSizeDeg), (int)(xL / gridSizeDeg));
	//中间
	for (i = 0; i < n_pts - 1; i++)
	{
		double dist = sqrt((pts[i].first - pts[i + 1].first)*(pts[i].first - pts[i + 1].first) + (pts[i].second - pts[i + 1].second)*(pts[i].second - pts[i + 1].second));
		if (dist / gridSizeDeg > strictThreshold)
			//insertEdgeIntoGrid(edge, getRowId(pts[i], pts[i + 1]), getColId(pts[i], pts[i + 1]));
		{
			//加1e-9是为了解决double的精度误差,比如原本row应该是13的,因为精度误差而算成12.99999999,取整后变成12
			int pts_i_row = (int)(pts[i].second / gridSizeDeg + 1e-9);
			int pts_i_col = (int)(pts[i].first / gridSizeDeg + 1e-9);
			int pts_i_plus_1_row = (int)(pts[i + 1].second / gridSizeDeg + 1e-9);
			int pts_i_plus_1_col = (int)(pts[i + 1].first / gridSizeDeg + 1e-9);
			int row = min(pts_i_row, pts_i_plus_1_row);
			int col = min(pts_i_col, pts_i_plus_1_col);
			insertTrajIntoGrid(iTraj, row, col);
		}
	}
	//尾
	double tailDist = sqrt((xR - pts[n_pts - 1].first)*(xR - pts[n_pts - 1].first) + (yR - pts[n_pts - 1].second)*(yR - pts[n_pts - 1].second));
	if (tailDist / gridSizeDeg > strictThreshold)
		insertTrajIntoGrid(iTraj, (int)(yR / gridSizeDeg), (int)(xR / gridSizeDeg));
	return;
}

void createGridIndexForOneTraj(IndexedTraj *iTraj)
{
	//////////////////////////////////////////////////////////////////////////
	///将轨迹iTraj加入网格索引
	///索引规则为: 对轨迹所经过的网格都添加轨迹指针,如果经过网格的长度过小则不添加
	//////////////////////////////////////////////////////////////////////////
	if (iTraj == NULL)
		return;
	Traj::iterator ptIter = iTraj->traj->begin();
	Traj::iterator nextPtIter = iTraj->traj->begin(); nextPtIter++;
	while (nextPtIter != iTraj->traj->end())
	{
		createGridIndexForSegment(iTraj, *ptIter, *nextPtIter);
		ptIter++;
		nextPtIter++;
	}
}

//drawer
Gdiplus::Color genColor(int i)
{
	switch (i % 10)
	{
	case 1: return Gdiplus::Color::Red;
	case 2: return Gdiplus::Color::Green;
	case 3: return Gdiplus::Color::Blue;
	case 4: return Gdiplus::Color::Yellow;
	case 5: return Gdiplus::Color::Aqua;
	case 6: return Gdiplus::Color::Pink;
	case 7: return Gdiplus::Color::Gray;
	case 8: return Gdiplus::Color::Brown;
	case 9: return Gdiplus::Color::Gold;
	case 10: return Gdiplus::Color::Purple;
	default:
		break;
	}
}

Gdiplus::Color randomColor()
{
	int r = int(((double)rand()) / RAND_MAX * 255);
	int g = int(((double)rand()) / RAND_MAX * 255);
	int b = int(((double)rand()) / RAND_MAX * 255);
	Gdiplus::Color color((byte)r, (byte)g, (byte)b);
	return color;
}

void drawGridLine(Gdiplus::Color color)
{
	//////////////////////////////////////////////////////////////////////////
	///在图片上画出网格线 
	//////////////////////////////////////////////////////////////////////////
	Gdiplus::ARGB argb = Gdiplus::Color::MakeARGB(90, color.GetR(), color.GetG(), color.GetB());
	color.SetValue(argb);
	double delta = 0.0000001;
	for (int i = 0; i < gridHeight; i++)
	{
		double lat = area.minLat + gridSizeDeg * i;
		md.drawLine(color, lat, area.minLon + delta, lat, area.maxLon - delta);
	}
	for (int i = 0; i < gridWidth; i++)
	{
		double lon = area.minLon + gridSizeDeg * i;
		md.drawLine(color, area.minLat + delta, lon, area.maxLat - delta, lon);
	}
}

void drawAllTrajs(std::string folderDir, std::string fileName)
{
	FILE *fpIn, *fpOut;
	double lat, lon;
	int time, roadId;
	double confidence;
	bool isStart = true;
	fpIn = fopen((folderDir + "//input//" + fileName).c_str(), "r");
	//fpOut = fopen((folderDir + "//output//" + getMMFileName(fileName)).c_str(), "r");
	bool startFlag = true;
	Traj* traj = NULL;
	double preLat, preLon;
	while (!feof(fpIn))
	{
		int flag = fscanf(fpIn, "%d,%lf,%lf", &time, &lat, &lon);
		//fscanf(fpOut, "%d,%d,%lf", &time, &roadId, &confidence);

		//防止末行读入两遍
		if (flag == -1)
			break;
		if (startFlag)
		{
			preLat = lat;
			preLon = lon;
			startFlag = false;
			continue;
		}
		else
		{
			md.drawLine(Gdiplus::Color::Green, lat, lon, preLat, preLon);
			md.drawPoint(Gdiplus::Color::Red, preLat, preLon);
			if (md.inArea(lat, lon) && md.inArea(preLat, preLon) && abs(lat - preLat) > 0.15)
			{
				cout << fileName << " time:" << time << endl;
				system("pause");
			}
			preLat = lat;
			preLon = lon;
		}

	}
	if (traj != NULL && traj->size() > 1)
		tempTrajs.push_back(traj);
	fclose(fpIn);
	//fclose(fpOut);
}

void drawTrajs(Gdiplus::Color color, list<Traj*> trajs, bool drawInterPt, bool boldLine)
{
	//////////////////////////////////////////////////////////////////////////
	///<重载> 画trajs中的所有轨迹,集合为list集合,元素为Traj*
	///drawInterPt为true则画上中间点,点为黑色十字点
	///boldLine为true则画粗线
	//////////////////////////////////////////////////////////////////////////
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		if (*trajIter == NULL)
			continue;
		Traj::iterator ptIter = (*trajIter)->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (1)
		{
			if (nextPtIter == (*trajIter)->end())
				break;
			if (boldLine)
				md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			else
				md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			if (drawInterPt)
			{
				md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
				md.drawBigPoint(Gdiplus::Color::Black, (*nextPtIter)->lat, (*nextPtIter)->lon);
			}
			ptIter++;
			nextPtIter++;
		}
	}
}

void drawTrajs(Gdiplus::Color color, vector<IndexedTraj*> trajs, bool drawInterPt, bool boldLine)
{
	//////////////////////////////////////////////////////////////////////////
	///<重载> 画trajs中的所有轨迹,集合为vector集合,元素为IndexedTraj*
	///drawInterPt为true则画上中间点,点为黑色十字点
	///boldLine为true则画粗线
	//////////////////////////////////////////////////////////////////////////
	for (vector<IndexedTraj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		Traj::iterator ptIter = (*trajIter)->traj->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (1)
		{
			if (nextPtIter == (*trajIter)->traj->end())
				break;
			if (boldLine)
				md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			else
				md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			if (drawInterPt)
			{
				md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
				md.drawBigPoint(Gdiplus::Color::Black, (*nextPtIter)->lat, (*nextPtIter)->lon);
			}
			ptIter++;
			nextPtIter++;
		}
	}
}

void drawTrajs(Gdiplus::Color color, list<IndexedTraj*> trajs, bool drawInterPt, bool boldLine)
{
	//////////////////////////////////////////////////////////////////////////
	///<重载> 画trajs中的所有轨迹,集合为list集合,元素为IndexedTraj*
	///drawInterPt为true则画上中间点,点为黑色十字点
	///boldLine为true则画粗线
	//////////////////////////////////////////////////////////////////////////	
	for (list<IndexedTraj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		Traj::iterator ptIter = (*trajIter)->traj->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (1)
		{
			if (nextPtIter == (*trajIter)->traj->end())
				break;
			if (boldLine)
				md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			else
				md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			if (drawInterPt)
			{
				md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
				md.drawBigPoint(Gdiplus::Color::Black, (*nextPtIter)->lat, (*nextPtIter)->lon);
			}
			ptIter++;
			nextPtIter++;
		}
	}
}

void drawOneTraj(Gdiplus::Color color, Traj* traj)
{
	/************************************************************************/
	/*以color色画一条轨迹,轨迹类型为list<GeoPoint*>*                           */
	/************************************************************************/
	if (traj == NULL)
		return;
	Traj::iterator ptIter = traj->begin(), nextPtIter = ptIter;
	md.drawPoint(color, (*ptIter)->lat, (*ptIter)->lon);
	nextPtIter++;
	while (1)
	{
		if (nextPtIter == traj->end())
			break;
		md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
		//cout << "draw: " << md.geoToScreen((*ptIter)->lat, (*ptIter)->lon).X << ", " <<
		//	md.geoToScreen((*ptIter)->lat, (*ptIter)->lon).Y << endl;
		if (!md.inArea((*ptIter)->lat, (*ptIter)->lon))
			system("pause");
		md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
		//md.drawBigPoint(Gdiplus::Color::Black, (*nextPtIter)->lat, (*nextPtIter)->lon);
		ptIter++;
		nextPtIter++;
	}
	//md.drawBigPoint(Gdiplus::Color::Black, traj->back()->lat, traj->back()->lon); //画终点
}

void drawClusteredTrajs()
{
	int count = 0;
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i] == NULL)
			continue;
		if (clusters[i]->size() > 0)
		{
			Gdiplus::Color color = randomColor();
			for (Cluster::iterator iTrajIter = clusters[i]->begin(); iTrajIter != clusters[i]->end(); iTrajIter++)
			{
				drawOneTraj(color, (*iTrajIter)->traj);
			}
			count += clusters[i]->size();
		}
	}
	cout << "cluster size: " << count << endl;
}

//core
	//new road generator
void drawPolyline(PolylineGenerator pg)
{
	cout << "polysize before drawing " << pg.polyline.size() << endl;
	for (int i = 0; i < pg.polyline.size() - 1; i++)
	{
		md.drawLine(Gdiplus::Color::Green, (int)pg.polyline[i].x, (int)pg.polyline[i].y, (int)pg.polyline[i + 1].x, (int)pg.polyline[i + 1].y);
	}
	for (int i = 0; i < pg.polyline.size(); i++)
	{
		md.drawBigPoint(Gdiplus::Color::Blue, (int)pg.polyline[i].x, (int)pg.polyline[i].y);
	}
}

void drawAllGennedEdges()
{
	drawTrajs(Gdiplus::Color::Black, gennedEgdes, true, true);
}

int extendAndSplitEdge(GeoPoint* prePt, GeoPoint* succPt, double threshold, bool& onExtend)
{
	//////////////////////////////////////////////////////////////////////////
	///沿prePt->succPt方向延伸,找到第一个相交的路段,返回交点生成的nodeId,onExtend为true
	///如果相交路段交在prePt<->succPt之间,则onExtend为false
	///如果threshold米内都没有满足要求的路段则返回-1
	//////////////////////////////////////////////////////////////////////////
	double A = succPt->lat - prePt->lat;
	double B = -(succPt->lon - prePt->lon);
	double C = prePt->lat * (succPt->lon - prePt->lon)
		- prePt->lon * (succPt->lat - prePt->lat);
	vector<Edge*> nearEdges;
	roadNetwork.getNearEdges(succPt->lat, succPt->lon, threshold, nearEdges);
	int candidateEdgeId = -1;
	double candidateIntersectX = -1.0;
	double candidateIntersectY = -1.0;
	double minDist = INFINITE;
	for each (Edge* edge in nearEdges)
	{
		Figure::iterator ptIter = edge->figure->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (nextPtIter != edge->figure->end())
		{
			GeoPoint* edgePt = *ptIter;
			GeoPoint* nextEdgePt = *nextPtIter;
			if ((A * edgePt->lon + B * edgePt->lat + C) * (A * nextEdgePt->lon + B * nextEdgePt->lat + C) < 0) //如果有交点
			{
				//求交点
				double _A = nextEdgePt->lat - edgePt->lat;
				double _B = -(nextEdgePt->lon - edgePt->lon);
				double _C = edgePt->lat * (nextEdgePt->lon - edgePt->lon)
					- edgePt->lon * (nextEdgePt->lat - edgePt->lat);
				double intersectY = (A * _C - _A * C) / (_A * B - A * _B);
				double intersectX = (-B * intersectY - C) / A;
				//判断是否在prePt->succPt延长线方向
				double preToSuccX = succPt->lon - prePt->lon;
				double preToSuccY = succPt->lat - prePt->lat;
				double succToIntersectX = intersectX - succPt->lon;
				double succToIntersectY = intersectY - succPt->lat;
				if (preToSuccX * succToIntersectX > 0 && preToSuccY * succToIntersectY > 0)//交点在延长线方向
				{
					double tmpDist = succPt->distM(intersectY, intersectX);
					if (tmpDist < threshold && tmpDist < minDist)
					{
						minDist = tmpDist;
						candidateEdgeId = edge->id;
						candidateIntersectX = intersectX;
						candidateIntersectY = intersectY;
						onExtend = true;
					}
				}
				else if (preToSuccX * succToIntersectX < 0 && preToSuccY * succToIntersectY < 0)//交点在pre和succ之间
				{
					double tmpDist = succPt->distM(intersectY, intersectX);
					if (tmpDist < threshold && tmpDist < minDist)
					{
						minDist = tmpDist;
						candidateEdgeId = edge->id;
						candidateIntersectX = intersectX;
						candidateIntersectY = intersectY;
						onExtend = false;
					}
				}
			}
			ptIter++;
			nextPtIter++;
		}
	}
	if (candidateEdgeId == -1) //范围内没有满足条件的路段
		return -1;
	else
	{
		//cout << "candidateEdgeId" << candidateEdgeId << endl;
		//printf("%.8lf,%.8lf", candidateIntersectY, candidateIntersectX);
		//system("pause");
		return roadNetwork.splitEdge(candidateEdgeId, candidateIntersectY, candidateIntersectX);
	}

}

Edge* addNewPolyLineIntoMap(PolylineGenerator& pg)
{
	//////////////////////////////////////////////////////////////////////////
	///将已经生成polyline的PolylineGenrator传入，将路加入路网，在genPolyline()被调用
	//////////////////////////////////////////////////////////////////////////
	
	//create figure
	//ofstream ofs("road.txt");
	//ofs << fixed << showpoint << setprecision(8);
	Figure* newFigure = new Figure();
	for (int i = 0; i < pg.polyline.size(); i++)
	{
		GeoPoint* pt = new GeoPoint(md.screenToGeo((int)pg.polyline[i].x, (int)pg.polyline[i].y));
		newFigure->push_back(pt);
	}
	//connection part
	GeoPoint* firstPt = newFigure->front();
	GeoPoint* lastPt = newFigure->back();
	double intersectionThres = 999;//30; //寻找在这个范围内有没有路口点,有就直接连上
	double splitRoadThres = 80; //延长线与路段的交点必须满足小于这个阈值,否则谁都不连
	//connectivity of the first
	vector<Edge*> nearEdges;
	roadNetwork.getNearEdges(firstPt->lat, firstPt->lon, intersectionThres, nearEdges);
	int minIntersectDist = INFINITE;
	int startNodeId = -1;
	for (int i = 0; i < nearEdges.size(); i++)
	{
		double distToStartNode = firstPt->distM(roadNetwork.nodes[nearEdges[i]->startNodeId]);
		if (distToStartNode < intersectionThres && distToStartNode < minIntersectDist)
		{
			minIntersectDist = distToStartNode;
			startNodeId = nearEdges[i]->startNodeId;
		}
		double distToEndNode = firstPt->distM(roadNetwork.nodes[nearEdges[i]->endNodeId]);
		if (distToEndNode < intersectionThres && distToEndNode < minIntersectDist)
		{
			minIntersectDist = distToEndNode;
			startNodeId = nearEdges[i]->endNodeId;
		}
	}
	if (startNodeId != -1) //在范围内找到了intersection则连上
	{
		newFigure->push_front(roadNetwork.nodes[startNodeId]);
	}
	else
	{
		Figure::iterator firstSuccPtIter = newFigure->begin();
		firstSuccPtIter++;
		GeoPoint* firstSuccPt = *firstSuccPtIter; //newFigure第一个点的后继点
		bool onExtend;
		int newNodeId = extendAndSplitEdge(firstSuccPt, firstPt, splitRoadThres, onExtend);
		if (newNodeId != -1) //找到了满足要求的路段
		{
			if (onExtend)
			{
				newFigure->push_front(roadNetwork.nodes[newNodeId]);
			}
			else
			{
				newFigure->front() = roadNetwork.nodes[newNodeId];
			}			
			startNodeId = newNodeId;
		}
		else //没有满足要求的路段,则不连
		{
			startNodeId = roadNetwork.insertNode(newFigure->front()->lat, newFigure->front()->lon);
		}
	}

	//connectivity of the last
	nearEdges.clear();
	roadNetwork.getNearEdges(lastPt->lat, lastPt->lon, intersectionThres, nearEdges);
	minIntersectDist = INFINITE;
	int endNodeId = -1;
	for (int i = 0; i < nearEdges.size(); i++)
	{
		double distToStartNode = lastPt->distM(roadNetwork.nodes[nearEdges[i]->startNodeId]);
		if (distToStartNode < intersectionThres && distToStartNode < minIntersectDist)
		{
			minIntersectDist = distToStartNode;
			endNodeId = nearEdges[i]->startNodeId;
		}
		double distToEndNode = lastPt->distM(roadNetwork.nodes[nearEdges[i]->endNodeId]);
		if (distToEndNode < intersectionThres && distToEndNode < minIntersectDist)
		{
			minIntersectDist = distToEndNode;
			endNodeId = nearEdges[i]->endNodeId;
		}
	}
	if (endNodeId != -1) //在范围内找到了intersection则连上
	{
		newFigure->push_back(roadNetwork.nodes[endNodeId]);
	}
	else
	{
		Figure::iterator lastPrePtIter = newFigure->end();
		lastPrePtIter--; lastPrePtIter--;
		GeoPoint* lastPrePt = *lastPrePtIter; //newFigure最后一个点的前继点
		bool onExtend;
		int newNodeId = extendAndSplitEdge(lastPrePt, lastPt, splitRoadThres, onExtend);
		if (newNodeId != -1) //找到了满足要求的路段
		{
			if (onExtend)
			{
				newFigure->push_back(roadNetwork.nodes[newNodeId]);
			}
			else
			{
				newFigure->back() = roadNetwork.nodes[newNodeId];
			}
			endNodeId = newNodeId;
		}
		else //没有满足要求的路段,则不连
		{
			endNodeId = roadNetwork.insertNode(newFigure->back()->lat, newFigure->back()->lon);
		}
	}
	//构造双向路
	Figure* newFigureReverse = new Figure();
	for (Figure::iterator iter = newFigure->begin(); iter != newFigure->end(); iter++)
	{
		newFigureReverse->push_front(*iter);
	}
	int newEdgeid = roadNetwork.insertEdge(newFigure, startNodeId, endNodeId);
	int newEdgeidR = roadNetwork.insertEdge(newFigureReverse, endNodeId, startNodeId);
	return roadNetwork.edges[newEdgeid];
	//drawOneTraj(Color::Black, map.edges[newEdgeid]->figure);
	/*for (int row = 0; row < map.gridHeight; row++)
	{
	for (int col = 0; col < map.gridWidth; col++)
	{
	for each (Edge* edge in *map.grid[row][col])
	{
	if (edge->id == newEdgeid)
	{
	printf("[%d, %d]\n", row, col);
	}
	}
	}
	}
	cout << gridWidth << endl;
	for (int row = 0; row < map.gridHeight; row++)
	{
	for (int col = 0; col < map.gridWidth; col++)
	{
	for each (Edge* edge in *map.grid[row][col])
	{
	if (edge->id == newEdgeidR)
	{
	printf("[%d, %d]\n", row, col);
	}
	}
	}
	}*/

	/*for (Figure::iterator ptIter = newFigure->begin(); ptIter != newFigure->end(); ptIter++)
	{
	ofs << (*ptIter)->lat << " " << (*ptIter)->lon << endl;
	}
	ofs.close();
	exit(0);*/
	//map.drawMap(Color::Blue, md);
}

Edge* addNewPolyLineIntoMapLite(PolylineGenerator& pg)
{
	//////////////////////////////////////////////////////////////////////////
	///将已经生成polyline的PolylineGenrator传入，将路加入路网，在genPolyline()被调用
	//////////////////////////////////////////////////////////////////////////

	Figure* newFigure = new Figure();
	for (int i = 0; i < pg.polyline.size(); i++)
	{
		GeoPoint* pt = new GeoPoint(md.screenToGeo((int)pg.polyline[i].x, (int)pg.polyline[i].y));
		newFigure->push_back(pt);
	}

	//构造双向路
	Figure* newFigureReverse = new Figure();
	for (Figure::iterator iter = newFigure->begin(); iter != newFigure->end(); iter++)
	{
		newFigureReverse->push_front(*iter);
	}
	int startNodeId = roadNetwork.insertNode(newFigure->front()->lat, newFigure->front()->lon);
	int endNodeId = roadNetwork.insertNode(newFigure->back()->lat, newFigure->back()->lon);
	int newEdgeid = roadNetwork.insertEdge(newFigure, startNodeId, endNodeId);
	int newEdgeidR = roadNetwork.insertEdge(newFigureReverse, endNodeId, startNodeId);
	return roadNetwork.edges[newEdgeid];
}

Edge* genPolyLine(Cluster& cluster)
{
	//////////////////////////////////////////////////////////////////////////
	///对cluster中的轨迹生成新路,返回新路指针(新路是双向的,只返回一个)
	///同时将新路加入地图
	///生成新路后,将cluster中轨迹删除 [TODO: 是否要放到reMM过程中再删除值得商榷]
	//////////////////////////////////////////////////////////////////////////
	PolylineGenerator pg;
	//将cluster里的轨迹点全部倒到pts里
	list<Pt> pts;
	
	for each (IndexedTraj* iTraj in cluster)
	{
		for each (GeoPoint* gPt in *(iTraj->traj))
		{
			Pt tmpPt;
			Gdiplus::Point gdiPt = md.geoToScreen(gPt->lat, gPt->lon);
			tmpPt.x = (double)gdiPt.X;
			tmpPt.y = (double)gdiPt.Y;
			pts.push_back(tmpPt);
		}
	}
	pg.genPolyline(pts);
	drawPolyline(pg);
	Edge* newEdge = addNewPolyLineIntoMapLite(pg);
	//删除cluster中的轨迹以及清空cluster
	//[TODO]不确定这么做好不好
	for each(IndexedTraj* iTraj in cluster)
	{
		delete iTraj->traj;
		iTraj->traj = NULL;
		iTraj->cluster = NULL;
	}
	cluster.clear();
	gennedEgdes.push_back(newEdge->figure);
	return newEdge;
	//TODO：下面一坨代码不知道干啥，先不理他
	//reMap matching
	list<Edge*> result;
	for (Cluster::iterator trajIter = cluster.begin(); trajIter != cluster.end(); trajIter++)
	{
		IndexedTraj* iTraj = *trajIter;
		Traj* traj = iTraj->traj;
		result = MapMatching(*(iTraj->traj), 50.0); //MapMatching
		//对每一个点
		Traj::iterator ptIter = traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //匹配成功
			{
				//tag:draw
				//md.drawBigPoint(Gdiplus::Color::Green, (*ptIter)->lat, (*ptIter)->lon);
			}
			else //匹配失败
			{
				//tag:draw
				//md.drawBigPoint(Gdiplus::Color::Yellow, (*ptIter)->lat, (*ptIter)->lon);
			}
			ptIter++;
			edgeIter++;
		}
	}	
}
	//cluster
double dist_old(GeoPoint* pt, Traj* traj)
{
	//////////////////////////////////////////////////////////////////////////
	///返回点pt到轨迹traj的投影距离，如果有多个投影返回最小的，如果没投影返回到端点最短距离
	///距离为经纬度距离，没有放大成米单位   
	///TODO:
	///		这个版本没有Map.cpp的版本好
	//////////////////////////////////////////////////////////////////////////
	Traj::iterator iter = traj->begin();
	Traj::iterator nextIter = traj->begin();
	nextIter++;
	double minDist = 9999;
	while (nextIter != traj->end())
	{
		//有投影
		if (cosAngle(pt, (*iter), (*nextIter)) <= 0 && cosAngle(pt, (*nextIter), (*iter)) <= 0)
		{
			double A = ((*nextIter)->lat - (*iter)->lat);
			double B = -((*nextIter)->lon - (*iter)->lon);
			double C = (*iter)->lat * ((*nextIter)->lon - (*iter)->lon)
				- (*iter)->lon * ((*nextIter)->lat - (*iter)->lat);
			double dist = abs(A * pt->lon + B * pt->lat + C) / sqrt(A * A + B * B);
			if (minDist > dist)
				minDist = dist;
		}
		iter++;
		nextIter++;
	}
	if (minDist > 9000) //没有投影则返回最短端点距离
	{
		double headDist = sqrt((pt->lat - traj->front()->lat) * (pt->lat - traj->front()->lat)
			+ (pt->lon - traj->front()->lon) * (pt->lon - traj->front()->lon));
		double tailDist = sqrt((pt->lat - traj->back()->lat) * (pt->lat - traj->back()->lat)
			+ (pt->lon - traj->back()->lon) * (pt->lon - traj->back()->lon));
		return headDist < tailDist ? headDist : tailDist;
	}
	else
		return minDist;
}

double distM(GeoPoint* pt, Traj* traj)
{
	//////////////////////////////////////////////////////////////////////////
	///返回点pt到traj的距离,单位为m
	///距离定义为：min(点到可投影边的投影距离，点到所有形状点的欧氏距离)
	//////////////////////////////////////////////////////////////////////////
	double minDist = 9999;
	//遍历端点距离
	for (Traj::iterator iter = traj->begin(); iter != traj->end(); iter++)
	{
		double tmpDist = GeoPoint::distM(pt->lat, pt->lon, (*iter)->lat, (*iter)->lon);
		if (tmpDist < minDist)
			minDist = tmpDist;
	}
	//遍历投影距离
	Traj::iterator iter = traj->begin();
	Traj::iterator nextIter = traj->begin();
	nextIter++;
	while (nextIter != traj->end())
	{
		//有投影
		if (cosAngle(pt, (*iter), (*nextIter)) <= 0 && cosAngle(pt, (*nextIter), (*iter)) <= 0)
		{
			double A = ((*nextIter)->lat - (*iter)->lat);
			double B = -((*nextIter)->lon - (*iter)->lon);
			double C = (*iter)->lat * ((*nextIter)->lon - (*iter)->lon)
				- (*iter)->lon * ((*nextIter)->lat - (*iter)->lat);
			double tmpDist = abs(A * pt->lon + B * pt->lat + C) / sqrt(A * A + B * B);
			tmpDist *= GeoPoint::geoScale;
			if (minDist > tmpDist)
				minDist = tmpDist;
		}
		iter++;
		nextIter++;
	}
	return minDist;
}

double hausdorffDist(Traj* traj1, Traj* traj2)
{
	//////////////////////////////////////////////////////////////////////////
	///返回两条轨迹的hausdoff距离(一条轨迹上的点到另一条轨迹的最远距离),返回单位为米
	//////////////////////////////////////////////////////////////////////////
	double maxDist = -1.0;
	//第一条到第二条
	for (Traj::iterator iter = traj1->begin(); iter != traj1->end(); iter++)
	{
		double tmpDist = distM((*iter), traj2);
		if (tmpDist > maxDist)
			maxDist = tmpDist;
	}
	//第二条到第一条
	for (Traj::iterator iter = traj2->begin(); iter != traj2->end(); iter++)
	{
		double tmpDist = distM((*iter), traj1);
		if (tmpDist > maxDist)
			maxDist = tmpDist;
	}
	return maxDist;
}

double hausdorffDist(Traj* traj1, Traj* traj2, double threshold)
{
	//////////////////////////////////////////////////////////////////////////
	///返回两条轨迹的hausdoff距离(一条轨迹上的点到另一条轨迹的最远距离),返回单位为米
	///如果计算过程中发现已经超过threshold米,则直接返回
	//////////////////////////////////////////////////////////////////////////
	double maxDist = -1.0;
	//第一条到第二条
	for (Traj::iterator iter = traj1->begin(); iter != traj1->end(); iter++)
	{
		double tmpDist = distM((*iter), traj2);
		if (tmpDist > maxDist)
		{
			maxDist = tmpDist;
			if (maxDist > threshold)
				return maxDist;
		}
	}
	//第二条到第一条
	for (Traj::iterator iter = traj2->begin(); iter != traj2->end(); iter++)
	{
		double tmpDist = distM((*iter), traj1);
		if (tmpDist > maxDist)
		{
			maxDist = tmpDist;
			if (maxDist > threshold)
				return maxDist;
		}
	}
	return maxDist;
}

double hausdorffDistLite(Traj* traj1, Traj* traj2)
{
	//////////////////////////////////////////////////////////////////////////
	///返回两条轨迹的hausdoff距离精简版本，只看两个端点     
	///	TODO:暂时有误,需修改
	//////////////////////////////////////////////////////////////////////////
	double maxDist = 0;
	Traj traj1Lite, traj2Lite;
	traj1Lite.push_back(traj1->front());
	traj1Lite.push_back(traj1->back());
	traj2Lite.push_back(traj2->front());
	traj2Lite.push_back(traj2->back());
	return hausdorffDist(&traj1Lite, &traj2Lite);
}

void getNearTrajs(Traj* traj, double thresholdM, list<IndexedTraj*>& dest)
{
	//////////////////////////////////////////////////////////////////////////
	///返回轨迹traj的mbr附近threshold米范围内索引里的所有轨迹(不精确,比精确值稍多),存入dest
	///可以保证的是不在dest内的轨迹,他们到iTraj的最短距离一定大于threshold
	//////////////////////////////////////////////////////////////////////////
	dest.clear();
	int gridSearchRange = int(thresholdM / (gridSizeDeg * GeoPoint::geoScale)) + 1;
	int minRow = INFINITE, maxRow = -1, minCol = INFINITE, maxCol = -1;
	for each (GeoPoint* pt in *traj)
	{
		int row = (pt->lat - area.minLat) / gridSizeDeg;
		int col = (pt->lon - area.minLon) / gridSizeDeg;
		if (row < minRow) minRow = row;
		if (row > maxRow) maxRow = row;
		if (col < minCol) minCol = col;
		if (col > maxCol) maxCol = col;
	}
	//求出搜索MBR
	minRow -= gridSearchRange;
	if (minRow < 0) minRow = 0;
	maxRow += gridSearchRange;
	if (maxRow >= gridHeight) maxRow = gridHeight - 1;
	minCol -= gridSearchRange;
	if (minCol < 0) minCol = 0;
	maxCol += gridSearchRange;
	if (maxCol >= gridWidth) maxCol = gridWidth - 1;
	int visitFlag = int(((double)rand()) / RAND_MAX * 99999999);
	double minDist = INFINITE;
	Cluster* destCluster = NULL;
	for (int i = minRow; i <= maxRow; i++)
	{
		for (int j = minCol; j <= maxCol; j++)
		{
			for (list<IndexedTraj*>::iterator iter = grid[i][j].begin(); iter != grid[i][j].end(); iter++)
			{
				if ((*iter)->traj == NULL)
					continue;
				if ((*iter)->traj == traj) //防止把自己加进去
					continue;
				//本次迭代还未访问过
				if ((*iter)->flag != visitFlag)
				{
					(*iter)->flag = visitFlag; //标记为已访问
					
					/**********************************************************/
					/*test code starts from here*/
					if ((*iter)->lengthM() < 200)
					{
						continue;
					}
					
					/*test code ends*/
					/**********************************************************/
					
					
					dest.push_back((*iter));
				}
			}
		}
	}
}

void doCluster(IndexedTraj* iTraj)
{
	//////////////////////////////////////////////////////////////////////////
	///对iTraj进行聚类
	//////////////////////////////////////////////////////////////////////////
	if (iTraj->traj == NULL)
		return;
	list<IndexedTraj*> nearTrajs;
	getNearTrajs(iTraj->traj, clusterDist, nearTrajs);
	double minDist = INFINITE;
	Cluster* destCluster = NULL;
	for each (IndexedTraj* candidateITraj in nearTrajs)
	{
		if (candidateITraj->cluster != iTraj->cluster)
		{
			double dist = hausdorffDist(iTraj->traj, candidateITraj->traj, clusterDist);
			if (dist < clusterDist && dist < minDist)
			{
				minDist = dist;
				destCluster = candidateITraj->cluster;
				//goto jmp;
			}
		}
	}
	//jmp:
	if (destCluster != NULL)
	{
		Cluster* srcCluster = iTraj->cluster;
		//将srcCluster里的所有轨迹所属的cluster全部变为destCluter
		for (Cluster::iterator iter = srcCluster->begin(); iter != srcCluster->end(); iter++)
		{
			(*iter)->cluster = destCluster;
		}
		destCluster->splice(destCluster->end(), *srcCluster);
	}
}

void doCluster_v2(IndexedTraj* iTraj)
{
	//////////////////////////////////////////////////////////////////////////
	///对iTraj进行聚类
	///将所有离iTraj距离小于clusterDistance的cluster全部merge起来
	//////////////////////////////////////////////////////////////////////////
	if (iTraj->traj == NULL)
		return;
	list<IndexedTraj*> nearTrajs;
	getNearTrajs(iTraj->traj, clusterDist, nearTrajs); //获得iTraj周围附近的轨迹
	double minDist = INFINITE;
	list<Cluster*> mergeClusters;
	for each (IndexedTraj* candidateITraj in nearTrajs)
	{
		if (candidateITraj->cluster != iTraj->cluster)
		{
			double dist = hausdorffDist(iTraj->traj, candidateITraj->traj, clusterDist);
			if (dist < clusterDist)
			{
				mergeClusters.push_back(candidateITraj->cluster);//同一cluster可重复放入，没关系的
			}
		}
	}
	Cluster* destCluster = iTraj->cluster;
	for each (Cluster* srcCluster in mergeClusters)
	{
		if (srcCluster->size() == 0)
			continue;
		//将srcCluster里的所有轨迹所属的cluster全部变为destCluter
		for (Cluster::iterator iter = srcCluster->begin(); iter != srcCluster->end(); iter++)
		{
			(*iter)->cluster = destCluster;
		}
		destCluster->splice(destCluster->end(), *srcCluster);
	}
}

void reMM_and_Cluster(Edge* newEdge)
{
	//////////////////////////////////////////////////////////////////////////
	///1.对新路newEdge周围的轨迹进行reMM
	///2.对reMM后与之前MM结果发生改变的那些轨迹进行split & extend
	///3.将原轨迹从trajs和对应的cluster中删除,然后将新轨迹加入
	///4.对处理后的新子轨迹逐一进行recluster
	//////////////////////////////////////////////////////////////////////////

	list<IndexedTraj*> nearTrajs;
	getNearTrajs(newEdge->figure, clusterDist, nearTrajs);
	//对每一个附近的候选轨迹
	for each(IndexedTraj* candidateITraj in nearTrajs)
	{
		//drawOneTraj(Color::Aqua, candidateITraj->traj);
		
		//********rematch********
		//TODO: MM候选区域范围设定值得商榷
		list<Edge*> result = MapMatching(*(candidateITraj->traj),30.0);
		bool mmRoadChanged = false; //标志reMM后有没有点本来没被匹配上的现在被匹配上了
		//对每一个点
		Traj::iterator ptIter = candidateITraj->traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != candidateITraj->traj->end())
		{
			if ((*edgeIter) != NULL) //匹配成功
			{
				md.drawBigPoint(Gdiplus::Color::Green, (*ptIter)->lat, (*ptIter)->lon);
				if ((*edgeIter)->id != (*ptIter)->mmRoadId)
				{
					(*ptIter)->mmRoadId = (*edgeIter)->id;
					mmRoadChanged = true;
				}
			}
			else //匹配失败
			{
				md.drawBigPoint(Gdiplus::Color::Red, (*ptIter)->lat, (*ptIter)->lon);
				if ((*ptIter)->mmRoadId != -1)
				{
					(*ptIter)->mmRoadId = -1;
					mmRoadChanged = true;
				}
			}
			ptIter++;
			edgeIter++;
		}
		
		//split & extend
		list<Traj*> newTrajs;
		if (mmRoadChanged)
		{
			doExtendForOneMMTraj(candidateITraj->traj, newTrajs, 20, 30);
			drawOneTraj(Gdiplus::Color::Aqua, candidateITraj->traj);
			drawTrajs(Gdiplus::Color::Red, newTrajs, false, false);
			//********recluster*******
				//先把原来老的轨迹删除
			candidateITraj->cluster->remove(candidateITraj);
			delete candidateITraj->traj;
			//delete candidateITraj;
			candidateITraj->traj = NULL;
			candidateITraj->cluster = NULL;
			//对每条生成的reMM子轨迹
			for each(Traj* traj in newTrajs)
			{
				//加入trajs和clusters
				IndexedTraj* tmpITraj = new IndexedTraj(traj);
				Cluster* tempCluster = new Cluster();
				tempCluster->push_back(tmpITraj);
				tmpITraj->cluster = tempCluster;
				trajs.push_back(tmpITraj);
				clusters.push_back(tempCluster);
				doCluster_v2(tmpITraj);
			}
		}
	} // end of [for each(IndexedTraj* candidateITraj in nearTrajs)]
}

void reMM_and_Cluster_v2(Edge* newEdge)
{
	//////////////////////////////////////////////////////////////////////////
	///1.对新路newEdge周围的轨迹进行reMM
	///2.对reMM后与之前MM结果发生改变的那些轨迹进行split & extend
	///3.将原轨迹从trajs和对应的cluster中删除,然后将新轨迹加入
	///4.对处理后的新子轨迹逐一进行recluster
	//////////////////////////////////////////////////////////////////////////

	list<IndexedTraj*> nearTrajs;
	getNearTrajs(newEdge->figure, clusterDist, nearTrajs);
	list<IndexedTraj*> _newTrajs;
	list<Cluster*> newClusters;
	int subSupportThreshold =4;
	//对每一个附近的候选轨迹
	//reMM和extend后加入newClusters
	for each(IndexedTraj* candidateITraj in nearTrajs)
	{
		//drawOneTraj(Color::Aqua, candidateITraj->traj);

		//********rematch********
		//TODO: MM候选区域范围设定值得商榷，姑且先设定30
		list<Edge*> result = MapMatching(*(candidateITraj->traj), 50);
		bool mmRoadChanged = false; //标志reMM后有没有点本来没被匹配上的现在被匹配上了
		//对每一个点
		Traj::iterator ptIter = candidateITraj->traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != candidateITraj->traj->end())
		{
			if ((*edgeIter) != NULL) //匹配成功
			{
				//tag:draw
				//md.drawBigPoint(Gdiplus::Color::Green, (*ptIter)->lat, (*ptIter)->lon);
				if ((*edgeIter)->id != (*ptIter)->mmRoadId)
				{
					(*ptIter)->mmRoadId = (*edgeIter)->id;
					mmRoadChanged = true;
				}
			}
			else //匹配失败
			{
				//tag:draw
				//md.drawBigPoint(Gdiplus::Color::Red, (*ptIter)->lat, (*ptIter)->lon);
				if ((*ptIter)->mmRoadId != -1)
				{
					(*ptIter)->mmRoadId = -1;
					mmRoadChanged = true;
				}
			}
			ptIter++;
			edgeIter++;
		}
		//split & extend
		list<Traj*> newTrajs;
		if (mmRoadChanged)
		{
			doExtendForOneMMTraj(candidateITraj->traj, newTrajs, 20, 50);
			//tag:draw
			//drawOneTraj(Gdiplus::Color::Aqua, candidateITraj->traj); //匹配成功的子轨迹段
			//drawTrajs(Gdiplus::Color::Red, newTrajs, false, false); //匹配失败的子轨迹段
			
			//********recluster*******
			//先把原来老的轨迹删除
			candidateITraj->cluster->remove(candidateITraj);
			delete candidateITraj->traj;
			candidateITraj->traj = NULL;
			candidateITraj->cluster = NULL;
			//对candidateITraj生成的每条reMM子轨迹
			for each(Traj* traj in newTrajs)
			{
				//加入newTrajs和newClusters
				IndexedTraj* tmpITraj = new IndexedTraj(traj);
				Cluster* tempCluster = new Cluster();
				tempCluster->push_back(tmpITraj);
				tmpITraj->cluster = tempCluster;
				_newTrajs.push_back(tmpITraj);
				newClusters.push_back(tempCluster);
			}
		}
	} // end of [for each(IndexedTraj* candidateITraj in nearTrajs)]


	//reCluster
	bool newRoadGenned = true;
	while (newRoadGenned) //当子区域没有新路生成则跳出
	{
		newRoadGenned = false;
		//遍历newTrajs中的所有traj进行聚类
		for (list<IndexedTraj*>::iterator newTrajIter = _newTrajs.begin(); newTrajIter != _newTrajs.end(); newTrajIter++)
		{
			IndexedTraj* currentITraj = *newTrajIter;
			if (currentITraj == NULL || currentITraj->traj == NULL)
			{
				continue;
			}
			doCluster_v2(currentITraj);
			//有新的子路生成
			if (currentITraj->cluster->size() > subSupportThreshold)
			{
				newRoadGenned = true;
				//tag:draw
				//drawTrajs(Gdiplus::Color::Green, *(currentITraj->cluster), true, false);
				cout << "有子路生成!" << endl;
				Edge* newSubEdge = genPolyLine(*(currentITraj->cluster));
				//ReMM
				list<IndexedTraj*> nearTrajs;
				getNearTrajs(newSubEdge->figure, clusterDist, nearTrajs);
				for each(IndexedTraj* candidateITraj in nearTrajs)
				{
					//drawOneTraj(Color::Aqua, candidateITraj->traj);

					//********rematch********
					//TODO: MM候选区域范围设定值得商榷
					list<Edge*> result = MapMatching(*(candidateITraj->traj), 50);
					bool mmRoadChanged = false; //标志reMM后有没有点本来没被匹配上的现在被匹配上了
					//对每一个点
					Traj::iterator ptIter = candidateITraj->traj->begin();
					list<Edge*>::iterator edgeIter = result.begin();
					while (ptIter != candidateITraj->traj->end())
					{
						if ((*edgeIter) != NULL) //匹配成功
						{
							//tag:draw
							//md.drawBigPoint(Gdiplus::Color::Green, (*ptIter)->lat, (*ptIter)->lon);
							if ((*edgeIter)->id != (*ptIter)->mmRoadId)
							{
								(*ptIter)->mmRoadId = (*edgeIter)->id;
								mmRoadChanged = true;
							}
						}
						else //匹配失败
						{
							//tag:draw
							//md.drawBigPoint(Gdiplus::Color::Red, (*ptIter)->lat, (*ptIter)->lon);
							if ((*ptIter)->mmRoadId != -1)
							{
								(*ptIter)->mmRoadId = -1;
								mmRoadChanged = true;
							}
						}
						ptIter++;
						edgeIter++;
					}

					//split & extend
					list<Traj*> newExtendTrajs;
					if (mmRoadChanged)
					{
						doExtendForOneMMTraj(candidateITraj->traj, newExtendTrajs, 20, 50);
						//drawOneTraj(Color::Aqua, candidateITraj->traj);
						//drawTrajs(Color::Red, newTrajs, false, false);
						//********recluster*******
						//先把原来老的轨迹删除
						candidateITraj->cluster->remove(candidateITraj);
						delete candidateITraj->traj;
						candidateITraj->traj = NULL;
						candidateITraj->cluster = NULL;
						//对每条生成的reMM子轨迹
						for each(Traj* traj in newExtendTrajs)
						{
							//加入trajs和clusters
							IndexedTraj* tmpITraj = new IndexedTraj(traj);
							Cluster* tempCluster = new Cluster();
							tempCluster->push_back(tmpITraj);
							tmpITraj->cluster = tempCluster;
							//trajs.push_back(tmpITraj);
							_newTrajs.push_back(tmpITraj);
							newClusters.push_back(tempCluster);
						}
					}
				}
			}
		}
	} // end while
	//将newTrajs和newClusters倒入trajs和clusters
	for each (IndexedTraj* iTraj in _newTrajs)
	{
		trajs.push_back(iTraj);
	}
	for each (Cluster* cluster in newClusters)
	{
		clusters.push_back(cluster);
	}
}

void doCluster(vector<IndexedTraj*>& trajs)
{
	//////////////////////////////////////////////////////////////////////////
	///对轨迹进行聚类,轨迹集合需为vector<IndexedTraj*>类型的trajs
	///[注意] 暂时先不用
	//////////////////////////////////////////////////////////////////////////
	cout << ">> start clustering" << endl;
	//initialization
	clusters.clear();
	//初始状态每条轨迹都是一个cluster
	for (int i = 0; i < trajs.size(); i++)
	{
		Cluster* tempCluster = new Cluster();
		tempCluster->push_back(trajs[i]);
		trajs[i]->cluster = tempCluster;
		clusters.push_back(tempCluster);
	}
	//对每条轨迹
	int tenPercent = trajs.size() / 10; //用于显示进度
	for (int iteration = 0; iteration < 3; iteration++)
	{
		for (int k = 0; k < trajs.size(); k++)
		{
			if (k % tenPercent == 0 && k != 0)
			{
				cout << k / tenPercent << "0% done" << endl;
			}
			//range iteration
			//轨迹的MBR
			//对于直的线段用这种处理
			/*int row1 = (trajs[k]->traj->front()->lat - minLat) / gridSizeDeg;
			int col1 = (trajs[k]->traj->front()->lon - minLon) / gridSizeDeg;
			int row2 = (trajs[k]->traj->back()->lat - minLat) / gridSizeDeg;
			int col2 = (trajs[k]->traj->back()->lon - minLon) / gridSizeDeg;
			int minRow = row1 < row2 ? row1 : row2;
			int maxRow = row1 < row2 ? row2 : row1;
			int minCol = col1 < col2 ? col1 : col2;
			int maxCol = col1 < col2 ? col2 : col1;*/
			//对于会拐弯的用这种处理
			int minRow = INFINITE, maxRow = -1, minCol = INFINITE, maxCol = -1;
			for each (GeoPoint* pt in (*trajs[k]->traj))
			{
				int row = (pt->lat - area.minLat) / gridSizeDeg;
				int col = (pt->lon - area.minLon) / gridSizeDeg;
				if (row < minRow) minRow = row;
				if (row > maxRow) maxRow = row;
				if (col < minCol) minCol = col;
				if (col > maxCol) maxCol = col;
			}			
			//求出搜索MBR
			minRow -= gridSearchRange;
			if (minRow < 0) minRow = 0;
			maxRow += gridSearchRange;
			if (maxRow >= gridHeight) maxRow = gridHeight - 1;
			minCol -= gridSearchRange;
			if (minCol < 0) minCol = 0;
			maxCol += gridSearchRange;
			if (maxCol >= gridWidth) maxCol = gridWidth - 1;
			trajs[k]->flag = k; //把自己标记为已访问
			double minDist = INFINITE;
			Cluster* destCluster = NULL;
			for (int i = minRow; i <= maxRow; i++)
			{
				for (int j = minCol; j <= maxCol; j++)
				{
					for (list<IndexedTraj*>::iterator iter = grid[i][j].begin(); iter != grid[i][j].end(); iter++)
					{
						//本次迭代还未访问过且两条轨迹在不同的cluster里
						if ((*iter)->flag != k && trajs[k]->cluster != (*iter)->cluster)
						{
							(*iter)->flag = k; //标记为已访问
							double dist = hausdorffDist(trajs[k]->traj, (*iter)->traj, clusterDist);
							if (dist < clusterDist && dist < minDist)
							{
								minDist = dist;
								destCluster = (*iter)->cluster;
								//goto jmp;
							}
						}
					}
				}
			}
		//jmp:
			if (destCluster != NULL)
			{
				Cluster* srcCluster = trajs[k]->cluster;
				//将srcCluster里的所有轨迹所属的cluster全部变为destCluter
				for (Cluster::iterator iter = srcCluster->begin(); iter != srcCluster->end(); iter++)
				{
					(*iter)->cluster = destCluster;
				}
				destCluster->splice(destCluster->end(), *srcCluster);
				if (destCluster->size() > supportThreshold)
				{
					drawTrajs(Gdiplus::Color::Red, *destCluster, true, false);
					genPolyLine(*destCluster);
					return;
				}				
				//cout << "src cluter size = " << srcCluster->size() << endl;
				//cout << "dest cluster size = " << destCluster->size() << endl;
				//system("pause");
			}
		}
	}
	cout << ">> clustering finished" << endl;
}

	//core
void core()
{
	//////////////////////////////////////////////////////////////////////////
	///旧版本，已停用
	//////////////////////////////////////////////////////////////////////////
	clusterDist = 20;
	cout << "doing core func..." << endl;
	//initialization
	//初始状态每条轨迹都是一个cluster
	for (int i = 0; i < trajs.size(); i++)
	{
		Cluster* cluster = new Cluster();
		cluster->push_back(trajs[i]);
		trajs[i]->cluster = cluster;
		clusters.push_back(cluster);
	}
	
	int newEdgeCount = 0;

	int tenPercent = trajs.size() / 10;
	for (int iteration = 0; iteration < 20; iteration++)
	{
		for (int k = 0; k < trajs.size(); k++)
		{
			if (k % tenPercent == 0 && k != 0)
			{
				cout << k / tenPercent << "0% done" << endl;
			}
			/*list<IndexedTraj*> nearTrajs;
			getNearTrajs(trajs[k]->traj, clusterDist, nearTrajs);
			double minDist = 9999;
			Cluster* destCluster = NULL;
			//对每一个附近的候选轨迹
			for each(IndexedTraj* candidateITraj in nearTrajs)
			{
			double dist = hausdorffDist(trajs[k]->traj, candidateITraj->traj, clusterDist);
			if (dist < clusterDist && dist < minDist)
			{
			minDist = dist;
			destCluster = candidateITraj->cluster;
			//goto jmp;
			}
			}
			//jmp:
			if (destCluster != NULL)
			{
			Cluster* srcCluster = trajs[k]->cluster;
			//将srcCluster里的所有轨迹所属的cluster全部变为destCluter
			for (Cluster::iterator iter = srcCluster->begin(); iter != srcCluster->end(); iter++)
			{
			(*iter)->cluster = destCluster;
			}
			destCluster->splice(destCluster->end(), *srcCluster);
			}*/
			//simple version
			if (trajs[k]->traj != NULL)
			{
				doCluster(trajs[k]);
				if (trajs[k]->cluster->size() > supportThreshold)
				{
					drawTrajs(Gdiplus::Color::Red, *(trajs[k]->cluster), true, false);
					reMM_and_Cluster(genPolyLine(*(trajs[k]->cluster)));
					newEdgeCount++;
					//TODO: need test!
					if (newEdgeCount == 6)
					{
						roadNetwork.drawMap(Gdiplus::Color::Blue, md);
						drawAllGennedEdges();
						drawClusteredTrajs();
						return;
					}
				}
			}
		} // end for (int k = 0; k < trajs.size(); k++)
	}
	roadNetwork.drawMap(Gdiplus::Color::Blue, md);
	drawClusteredTrajs();
}

void core_v2()
{
	//////////////////////////////////////////////////////////////////////////
	///改进版本
	//////////////////////////////////////////////////////////////////////////
	clusterDist = 20;
	supportThreshold = 4;
	cout << ">> doing core2 func..." << endl;
	//initialization
	//初始状态每条轨迹都是一个cluster
	for (int i = 0; i < trajs.size(); i++)
	{
		Cluster* cluster = new Cluster();
		cluster->push_back(trajs[i]);
		trajs[i]->cluster = cluster;
		clusters.push_back(cluster);
	}

	int newEdgeCount = 0;

	int tenPercent = trajs.size() / 10;
	for (int iteration = 0; iteration < 1; iteration++)
	{
		for (int k = 0; k < trajs.size(); k++)
		{
			if (k % tenPercent == 0 && k != 0)
			{
				cout << k / tenPercent << "0% done" << endl;
			}
			//simple version
			if (trajs[k]->traj != NULL)
			{
				doCluster_v2(trajs[k]);
				if (trajs[k]->cluster->size() > supportThreshold)
				{
					
					//return;
					Edge* gennedEdge = genPolyLine(*(trajs[k]->cluster));
					reMM_and_Cluster_v2(gennedEdge);
					//return;
					//system("pause");
					newEdgeCount++;
					//TODO: need test!
					/*if (newEdgeCount <= 5)
					{
						drawTrajs(Gdiplus::Color::Red, *(trajs[k]->cluster), true, false);
						genPolyLine(*(trajs[k]->cluster));
						roadNetwork.drawMap(Gdiplus::Color::Blue, md);
						drawAllGennedEdges();
					}*/
					/*if (newEdgeCount == 1)
					{
						drawAllGennedEdges();
						return;
					}*/
				}
			}
		} // end for (int k = 0; k < trajs.size(); k++)
	}
	roadNetwork.drawMap(Gdiplus::Color::Blue, md);
	drawAllGennedEdges();
	TrajReader::outputTrajs(gennedEgdes, "roads_wy.txt");
	roadNetwork.drawMap(Gdiplus::Color::Blue, md);
	//drawClusteredTrajs();
}

void drawTrajPts(vector<IndexedTraj*>& trajs)
{
	for each(IndexedTraj* iTraj in trajs)
	{
		Traj* traj = iTraj->traj;
		for (Traj::iterator ptIter = traj->begin(); ptIter != traj->end(); ptIter++)
		{
			if ((*ptIter)->mmRoadId != -1) //匹配成功
			{
				md.drawPoint(Gdiplus::Color::Green, (*ptIter)->lat, (*ptIter)->lon);
			}
			else //匹配失败
			{
				md.drawPoint(Gdiplus::Color::Red, (*ptIter)->lat, (*ptIter)->lon);
			}
		}
	}
}

void drawTrajPts(list<Traj*>& trajs)
{
	for each(Traj* traj in trajs)
	{
		for (Traj::iterator ptIter = traj->begin(); ptIter != traj->end(); ptIter++)
		{
			if ((*ptIter)->mmRoadId != -1) //匹配成功
			{
				//md.drawPoint(Gdiplus::Color::Green, (*ptIter)->lat, (*ptIter)->lon);
			}
			else //匹配失败
			{
				md.drawPoint(Gdiplus::Color::Red, (*ptIter)->lat, (*ptIter)->lon);
			}
		}
	}
}

void MM(list<Traj*>& unmatchedTrajs, bool doOutput = false, string outPutFilePath = "")
{
	//////////////////////////////////////////////////////////////////////////
	///<重载>对unmatchedTrajs中的每条轨迹进行MapMatching
	///doOutput:输出到文件开关
	///outPutFileName: 输出文件路径
	//////////////////////////////////////////////////////////////////////////
	int count = 0;
	ofstream fout;
	if (doOutput)
	{
		fout.open("wy_MMTrajs.txt");
		fout << fixed << showpoint << setprecision(8);
	}
	cout << ">> starting MapMatching" << endl
		<< unmatchedTrajs.size() << " trajs in total" << endl;
	//对每一条轨迹
	for (list<Traj*>::iterator trajIter = unmatchedTrajs.begin(); trajIter != unmatchedTrajs.end(); trajIter++, count++)
	{
		list<Edge*> result = MapMatching(*(*trajIter), 50); //MapMatching
		if (count % 500 == 0)
			cout << ">> MM" << count << " finished" << endl;
		Traj* traj = (*trajIter);
		if (traj == NULL)
			continue;
		//对每一个点
		Traj::iterator ptIter = traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //匹配成功
			{
				(*ptIter)->mmRoadId = (*edgeIter)->id;
				if (doOutput)
					fout << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*edgeIter)->id << endl;
			}
			else //匹配失败
			{
				(*ptIter)->mmRoadId = -1;
				if (doOutput)
					fout << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << -1 << endl;
			}
			ptIter++;
			edgeIter++;
		}
		if (doOutput)
			fout << -1 << endl;
	}
	if (doOutput)
		fout.close();
}

void MM(vector<IndexedTraj*>& unmatchedITrajs, bool doOutput = false)
{
	int count = 0;
	ofstream fout;
	if (doOutput)
	{
		fout.open("wy_MMTrajs.txt");
		fout << fixed << showpoint << setprecision(8);
	}
	cout << ">> starting MapMatching" << endl
		<< unmatchedITrajs.size() << " trajs in total" << endl;
	//对每一条轨迹
	for (vector<IndexedTraj*>::iterator trajIter = unmatchedITrajs.begin(); trajIter != unmatchedITrajs.end(); trajIter++, count++)
	{
		list<Edge*> result = MapMatching(*(*trajIter)->traj, 50); //MapMatching
		if (count % 500 == 0)
			cout << ">> MM" << count << " finished" << endl;
		Traj* traj = (*trajIter)->traj;
		if (traj == NULL)
			continue;
		//对每一个点
		Traj::iterator ptIter = traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //匹配成功
			{
				(*ptIter)->mmRoadId = (*edgeIter)->id;
				if (doOutput)
					fout << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*edgeIter)->id << endl;
			}
			else //匹配失败
			{
				(*ptIter)->mmRoadId = -1;
				if (doOutput)
					fout << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << -1 << endl;
			}
			ptIter++;
			edgeIter++;
		}
		if (doOutput)
			fout << -1 << endl;
	}
	if (doOutput)
		fout.close();
}

void testTCC()
{
	PolylineGenerator pg;
	list<Pt> pts;
	ifstream ifs("C:\\Users\\wuhao\\Desktop\\tcc.txt");
	double x, y;
	while (ifs)
	{
		ifs >> x >> y;
		Pt tmpPt(x, y);
		pts.push_back(tmpPt);
		if (ifs.fail())
		{
			break;
		}
	}
	pg.genPolyline(pts);

	//draw
	MapDrawer md;
	md.setResolution(10000, 10000);
	md.newBitmap();
	md.lockBits();
	//draw pts
	for each (Pt pt in pts)
	{
		md.drawBigPoint(Gdiplus::Color::Black, int(pt.x), int(pt.y));
		md.drawBigPoint(Gdiplus::Color::Black, int(pt.x) + 1, int(pt.y) + 1);
		md.drawBigPoint(Gdiplus::Color::Black, int(pt.x) - 1, int(pt.y) + 1);
		md.drawBigPoint(Gdiplus::Color::Black, int(pt.x) + 1, int(pt.y) - 1);
		md.drawBigPoint(Gdiplus::Color::Black, int(pt.x) - 1, int(pt.y) - 1);
	}
	//draw polyline
	for (int i = 0; i < pg.polyline.size() - 1; i++)
	{
		md.drawLine(Gdiplus::Color::Green, (int)pg.polyline[i].x, (int)pg.polyline[i].y, (int)pg.polyline[i + 1].x, (int)pg.polyline[i + 1].y);
	}
	for (int i = 0; i < pg.polyline.size(); i++)
	{
		md.drawBigPoint(Gdiplus::Color::Blue, (int)pg.polyline[i].x, (int)pg.polyline[i].y);
	}
	md.unlockBits();
	md.saveBitmap("testTCC.png");
	exit(0);
}

void testTCCAll()
{
	list<Pt> pts;
	ifstream ifs("C:\\Users\\wuhao\\Desktop\\pts.txt");
	double x, y;
	MapDrawer md;
	md.setResolution(1000, 1000);
	md.newBitmap();
	md.lockBits();
	while (ifs)
	{
		ifs >> x;
		if (x < 0)
		{
			if (pts.size()>0)
			{
				Gdiplus::Color color = randomColor();
				for each (Pt pt in pts)
				{
					md.drawBigPoint(color, int(pt.x), int(pt.y));
					/*md.drawBigPoint(color, int(pt.x) + 1, int(pt.y) + 1);
					md.drawBigPoint(color, int(pt.x) - 1, int(pt.y) + 1);
					md.drawBigPoint(color, int(pt.x) + 1, int(pt.y) - 1);
					md.drawBigPoint(color, int(pt.x) - 1, int(pt.y) - 1);*/
				}
				PolylineGenerator pg;
				pg.genPolyline(pts);
				for (int i = 0; i < pg.polyline.size() - 1; i++)
				{
					md.drawLine(color, (int)pg.polyline[i].x, (int)pg.polyline[i].y, (int)pg.polyline[i + 1].x, (int)pg.polyline[i + 1].y);
				}
				for (int i = 0; i < pg.polyline.size(); i++)
				{
					md.drawBigPoint(Gdiplus::Color::Blue, (int)pg.polyline[i].x, (int)pg.polyline[i].y);
				}
				pts.clear();
			}
		}
		else
		{
			ifs >> y;
			Pt tmpPt(x/10, y/10);
			pts.push_back(tmpPt);
		}
		/*if (ifs.fail())
		{
			break;
		}*/
	}
	

	//draw

	
	//draw polyline
	
	md.unlockBits();
	md.saveBitmap("testTCC_2.png");
	exit(0);
}

void drawIntersection(list<Traj*> trajs)
{
	double steadySpeed = 0.001;
	//先画MM匹配失败的点
	for each (Traj* traj in trajs)
	{
		Traj::iterator ptIter = traj->begin();
		for (; ptIter != traj->end(); ptIter++)
		{
			GeoPoint* pt = (*ptIter);
			if (md.inArea(pt->lat, pt->lon) && pt->mmRoadId == -1)
			{
				md.drawPoint(Gdiplus::Color::Red, pt->lat, pt->lon);
			}
		}
	}

	for each (Traj* traj in trajs)
	{
		Traj::iterator ptIter = traj->begin(); ptIter++;
		Traj::iterator preIter = traj->begin();

		double steadyTime = 0;
		double steadyTimeThreshold = 150;
		bool steadyStatus = false;
		for (; ptIter != traj->end(); ptIter++, preIter++)
		{
			GeoPoint* pt = (*ptIter), *prePt = (*preIter);
			if (md.inArea(pt->lat, pt->lon))
			{
				double dist = GeoPoint::distM(pt, prePt);
				if (dist < 5.0 && abs(pt->time - prePt->time) < steadyTimeThreshold && dist / abs(pt->time - prePt->time) < steadySpeed)
				{
					if (steadyStatus == false)
					{
						steadyStatus = true;
						steadyTime = abs(pt->time - prePt->time);
					}
					else
					{
						steadyTime += abs(pt->time - prePt->time);
					}
				}
				else
				{
					if (steadyStatus)
					{
						if (steadyTime < steadyTimeThreshold && steadyTime > 10)
						{
							md.drawBigPoint(Gdiplus::Color::Black, prePt->lat, prePt->lon);
						}
						steadyStatus = false;
					}
				}
			}
		}
	}
}

void drawIntersection_v2(list<Traj*> trajs)
{
	//////////////////////////////////////////////////////////////////////////
	///改进版本
	///十字路口特征：等待时间小于steadyTimeThreshold，前后车速不低于minSpeed
	//////////////////////////////////////////////////////////////////////////
	
	//先画MM匹配失败的点
	for each (Traj* traj in trajs)
	{
		Traj::iterator ptIter = traj->begin();
		for (; ptIter != traj->end(); ptIter++)
		{
			GeoPoint* pt = (*ptIter);
			if (md.inArea(pt->lat, pt->lon) && pt->mmRoadId == -1)
			{
				md.drawPoint(Gdiplus::Color::Green, pt->lat, pt->lon);
			}
		}
	}

	double steadySpeed = 0.01;
	double steadyTimeThreshold = 200;
	double minSpeed = 8;
	for each (Traj* traj in trajs)
	{
		Traj::iterator ptIter = traj->begin(); ptIter++;
		Traj::iterator preIter = traj->begin();

		double steadyTime = 0; //记录停留时间
		
		bool steadyStatus = false;
		double lastSpeed = 0; //记录最后的行车速度
		for (; ptIter != traj->end(); ptIter++, preIter++)
		{
			GeoPoint* pt = (*ptIter), *prePt = (*preIter);
			if (md.inArea(pt->lat, pt->lon))
			{
				double dist = GeoPoint::distM(pt, prePt);
				double speed = dist / (pt->time - prePt->time);
				if (speed > steadySpeed)
				{
					lastSpeed = speed;
					if (steadyStatus)
					{
						if (steadyTime < steadyTimeThreshold && steadyTime > 10 && lastSpeed >= minSpeed)
						{
							md.drawBigPoint(Gdiplus::Color::Red, prePt->lat, prePt->lon);
						}
						steadyStatus = false;
					}
				}
				else if (speed <= steadySpeed && abs(pt->time - prePt->time) < steadyTimeThreshold)
				{
					if (steadyStatus == false && lastSpeed >= minSpeed)
					{
						steadyStatus = true;
						steadyTime = abs(pt->time - prePt->time);
					}
					else
					{
						steadyTime += abs(pt->time - prePt->time);
					}
				}
				else //if (speed <= steadySpeed && abs(pt->time - prePt->time) >= steadyTimeThreshold)
				{
					lastSpeed = speed;
					steadyStatus = false;
				}
				
			}
		}
	}
}

double calTrajLengthM(Traj* traj)
{
	//////////////////////////////////////////////////////////////////////////
	///计算轨迹的长度，单位为M
	//////////////////////////////////////////////////////////////////////////
	
	double lengthM = 0;
	Figure::iterator ptIter = traj->begin(), nextPtIter = ptIter;
	nextPtIter++;
	while (1)
	{
		if (nextPtIter == traj->end())
			break;
		lengthM += GeoPoint::distM((*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
		ptIter++;
		nextPtIter++;
	}
	return lengthM;
}

void genExperimentData()
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
}

void gen60sData()
{
	string dataPath = "D:\\trajectory\\singapore_data\\experiments\\wy_MMTrajs.txt";
	string outPath = "D:\\trajectory\\singapore_data\\experiments\\wy_MMTrajs_60.txt";
	ifstream ifs(dataPath);
	ofstream ofs(outPath);
	if (!ifs)
	{
		cout << "open" << dataPath << "error" << endl;
		system("pause");
	}	
	if (!ofs)
	{
		cout << "open " << outPath << " error!" << endl;
		system("pause");
	}
	ofs << fixed << showpoint << setprecision(8);
	bool flag = true;

	while (ifs)
	{
		double lat, lon;
		int time, mmRoadId;
		ifs >> time;
		if (ifs.fail())
			break;
		if (time == -1)
		{
			ofs << -1 << endl;
			flag = true;
			continue;
		}
		else
		{
			ifs >> lat >> lon >> mmRoadId;
			if (flag)
				ofs << time << " " << lat << " " << lon << " " << mmRoadId << endl;
			flag = !flag;		
		}
	}
	ofs.close();
	ifs.close();	
}

void gen90sData()
{
	string dataPath = "D:\\trajectory\\singapore_data\\experiments\\wy_MMTrajs.txt";
	string outPath = "D:\\trajectory\\singapore_data\\experiments\\wy_MMTrajs_90.txt";
	ifstream ifs(dataPath);
	ofstream ofs(outPath);
	if (!ifs)
	{
		cout << "open" << dataPath << "error" << endl;
		system("pause");
	}
	if (!ofs)
	{
		cout << "open " << outPath << " error!" << endl;
		system("pause");
	}
	ofs << fixed << showpoint << setprecision(8);
	int flag = 0;

	while (ifs)
	{
		double lat, lon;
		int time, mmRoadId;
		ifs >> time;
		if (ifs.fail())
			break;
		if (time == -1)
		{
			ofs << -1 << endl;
			flag = 0;
			continue;
		}
		else
		{
			ifs >> lat >> lon >> mmRoadId;
			if (flag == 0)
				ofs << time << " " << lat << " " << lon << " " << mmRoadId << endl;
			flag++;
			flag %= 3;
		}
	}
	ofs.close();
	ifs.close();
}

void eyeballTest()
{
	int count = 0;
	int max = 10;
	for (list<Traj*>::iterator trajIter = tempTrajs.begin(); trajIter!= tempTrajs.end();trajIter++)
	{
		if (count == max)
		{
			break;
		}
		if (calTrajLengthM(*trajIter) < 1500)
		{
			continue;
		}
		
		md.newBitmap();
		md.lockBits();
		roadNetwork.drawMap(Gdiplus::Color::Blue, md);
		TrajDrawer::drawOneTraj(*trajIter, md, Gdiplus::Color::Green);
		bool unmatchedFlag = true;
		for each (GeoPoint* pt in *(*trajIter))
		{
			//cout << pt->mmRoadId << endl;
			if (pt->mmRoadId == -1)
				continue;
			//system("pause");
			unmatchedFlag = false;
			TrajDrawer::drawOneTraj(roadNetwork.edges[pt->mmRoadId]->figure, md, Gdiplus::Color::Red, true);
		}
		createGridIndex(createGridIndexForOneTraj);
		drawGridLine(Gdiplus::Color::Green);
		md.unlockBits();
		count++;
		if (unmatchedFlag == false)
			md.saveBitmap("D:\\project\\LAB\\trajectory\\TC1\\MMTest\\" + StringOperator::intToString(count)+".png");
	}
	system("pause");
	exit(0);
}

void drawTCCResult()
{
	//drawPt
	ifstream ptFile("D:\\trajectory\\singapore_data\\experiments\\ours\\missPoints.txt");
	vector<GeoPoint*> pts;
	while (ptFile)
	{
		double lat, lon;
		ptFile >> lat >> lon;
		if (ptFile.fail())
			break;
		GeoPoint* newPt = new GeoPoint(lat, lon);
		pts.push_back(newPt);
	}
	md.newBitmap();
	md.lockBits();
	for (int i = 0; i < pts.size(); i++)
	{
		md.drawBigPoint(Gdiplus::Color::Red, pts[i]->lat, pts[i]->lon);
	}
	md.unlockBits();
	md.saveBitmap("ourPts.png");

	//drawRd
	TrajReader tr("D:\\trajectory\\singapore_data\\experiments\\ours\\resultAll.txt");
	list<Traj*> roads;
	tr.readTrajs(roads);
	TrajDrawer td;
	md.newBitmap();
	md.lockBits();
	td.drawTrajs(roads, md, Gdiplus::Color::Black, true, true, false, false);
	md.unlockBits();
	md.saveBitmap("ourRoads.png");
	exit(0);
}
/////////////////////////////////////////我的算法///////////////////////////////////////////


void mFirstClust()
{
	//////////////////////////////////////////////////////////////////////////
	///将没有匹配的轨迹点聚成一个个的类
	//////////////////////////////////////////////////////////////////////////
	
}


void initialize()
{

}

void main()
{
	int startTime = clock();
	srand((unsigned)time(NULL));

	
//=======================================initialization start========================================//
	string trajPath = "D:\\trajectory\\singapore_data\\experiments\\big area\\geo\\area3\\newMMTrajs_unmatched.txt";
	
	md.setArea(&area);
	md.setResolution(size);
	
	/*zooming part start*/
	//以下代码已废弃，勿用
	zoomed = false;
	double zoomingRate;
	if (zoomed)
	{
		//int zoomWide = 800;
		//int zoomHeight = 600;
		int zoomWide = 1600;
		int zoomHeight = 1300;
		zoomingRate = (double)zoomWide / (double)size;
		gridWidth = int(zoomingRate * double(gridWidth));
		size = 5000;
		//md.zoomIn(8900, 7150, zoomWide, zoomHeight, size);
		md.zoomIn(6500, 6800, zoomWide, zoomHeight, size);
		printf("zoomed: minlat:%lf,maxlat:%lf,minlon:%lf,maxlon:%lf\nresolution:%d*%d\n", md.area->minLat, md.area->maxLat, md.area->minLon, md.area->maxLon, md.r_width, md.r_height);
	}
	/*zooming part end*/

	roadNetwork.setArea(&area);
	roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 50);
	roadNetwork.deleteEdges("D:\\trajectory\\singapore_data\\experiments\\big area\\geo\\area3\\deletedEdges.txt");
	printf("\n");
/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑initialization end↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
	
	
	/*一条龙*/
	//用来测试调整doExtend中使用的到的三个limit参数
	if (0)
	{
		//用这个！！
		//limitSpeed = 50; //间隔大于50m/s的prune掉
		limitDist = 300; //轨迹点之间超过300m的prune掉
		//limitTime = 100; //采样间隔大于60秒的prune掉
		double minTrajDist = 50;
		double extendDist = 25;
		TrajReader tReader(trajPath);
		tReader.readTrajs(tempTrajs);
		list<Traj*> extendTrajs;
		doExtend(tempTrajs, extendTrajs, extendDist, minTrajDist, true);
		//画
		md.newBitmap();
		md.lockBits();
		drawTrajs(Gdiplus::Color::Red, extendTrajs, true, false);
		createGridIndex(createGridIndexForOneTraj);
		drawGridLine(Gdiplus::Color::Green);
		//md.drawMap(Gdiplus::Color::Blue, mapFilePath);
		md.unlockBits();
		md.saveBitmap(trajDir + "testExtend.png");
		system("pause");
		exit(0);		
	}

	/*画出十字路口点*/
	if (0)
	{
		//trajFileName = "splitedTrajs.txt";
		readStdTrajs(trajPath, tempTrajs);
		cout << "tempTrajs size = " << tempTrajs.size() << endl;
		
		md.newBitmap();
		md.lockBits();
		//md.drawMap(Gdiplus::Color::Blue, mapFilePath);
		drawIntersection_v2(tempTrajs);
		md.unlockBits();
		md.saveBitmap("intersection_v2.png");
		system("pause");
		exit(0);
	}


	/*把SRC轨迹转成std轨迹格式*/
	if (0)
	{
		//for (int i = 0; i < trajFolders.size(); i++)
		//	scanTrajFolder(trajDir, readSRCTrajs);
		cout << "raw trajs's size = " << rawTrajs.size() << endl;
		//doExtend(rawTrajs, tempTrajs, true);
		genStdTrajFile(rawTrajs, trajDir + "20110102_03.txt");
		system("pause");
		exit(0);
	}
	
	/*预处理SRC轨迹,将其分割,在TrajFoloder生成splitedTrajs.txt*/
	if (0)
	{
		splitSRCTrajFiles(trajDir);
		system("pause");
		exit(0);
	}	

	/*用SRC的轨迹格式创建extendTraj*/	
	if (0)
	{
		//for (int i = 0; i < trajFolders.size(); i++)
		//	scanTrajFolder(trajFolders[i], readSRCTrajs);
		cout << "raw trajs's size = " << rawTrajs.size() << endl;
		double minTrajLength = 50;
		double extendDist = 15;
		doExtend(rawTrajs, tempTrajs, extendDist, minTrajLength, true);
		system("pause");
		exit(0);
	}

	/*用std轨迹格式(wy_MMTraj.txt)创建extendTraj*/
	if (0)
	{
		//不用这个
		double extendDist = 30;
		readStdTrajs(trajPath, rawTrajs);
		cout << "MMed trajs's size = " << rawTrajs.size() << endl;
		doExtend(rawTrajs, tempTrajs, extendDist, 50, true);
		system("pause");
		exit(0);
	}

/////////////////////////////////读入轨迹文件///////////////////////////////////////
	//TrajReader tReader(trajPath);
	//tReader.readTrajs(tempTrajs, 800000);
	readStdTrajs(trajPath, trajs);
	cout << "traj's size = " << trajs.size() << endl;
//////////////////////////////////////////////////////////////////////////////////

	//eyeballTest();

	/*读入原始轨迹，然后将过长的切断输出至"splitedTrajs.txt"*/
	if (0)
	{
		splitTrajsAndOutput(tempTrajs);
		system("pause");
		exit(0);
	}
	

	/*MM using viterbi and output in Project Folder*/	
	if (0)
	{
		bool doOutput = true;
		MM(tempTrajs, doOutput);
		system("pause");
		exit(0);
	}
	
	
	//画使用viterbi MM后的轨迹点
	if (0)
	{
		md.setResolution(3000);
		md.newBitmap();
		md.lockBits();
		//md.drawMap(Gdiplus::Color::Black, mapFilePath);
		TrajDrawer td;
		td.drawMMTrajs(tempTrajs, md, Gdiplus::Color::Green, false, false, false, true);
		//drawTrajPts(tempTrajs);
		createGridIndex(createGridIndexForOneTraj);
		drawGridLine(Gdiplus::Color::Green);
		md.unlockBits();
		md.saveBitmap(trajDir + "exp.png");
		system("pause");
		exit(0);
	}	


	/**********************************************************/
	/*test code starts from here*/
	//画MM失败的轨迹
	if (0)
	{
		md.newBitmap();
		md.lockBits();
		//md.drawMap(Gdiplus::Color::Blue, mapFilePath);
		drawTrajs(Gdiplus::Color::Red, trajs, true, false);
		createGridIndex(createGridIndexForOneTraj);
		drawGridLine(Gdiplus::Color::Green);
		md.unlockBits();
		md.saveBitmap("wymmtrajs_failedPts.png");
		system("pause");
		exit(0);
	}	
	/*test code ends*/
	/**********************************************************/

	createGridIndex(createGridIndexForOneTraj);
	cout << endl;

	//////////////////////////////////////////////////////////////////////////
	///Core Part
	//////////////////////////////////////////////////////////////////////////
	//drawing part
	cout << ">> start drawing..." << endl;
	md.newBitmap();
	md.lockBits();
	drawGridLine(Gdiplus::Color::Green);
	//md.drawMap(Gdiplus::Color::Blue, mapFilePath);
	core_v2();
	roadNetwork.drawMap(Gdiplus::Color::Blue, md);
	//drawClusteredTrajs();	
	md.unlockBits();
	
	
	
	//生成文件名
	/*string pngName =
		"trajSize=" + StringOperator::intToString(rawTrajs.size()) +
		"_speed=" + StringOperator::doubleToString(limitSpeed) +
		"_time=" + StringOperator::doubleToString(limitTime) +
		"_dist=" + StringOperator::doubleToString(limitDist);
	if (zoomed)
	{
		pngName += " [zoom in]";
	}
	pngName += ".png";*/
	string pngName = "wangyin_geo.png";
	md.saveBitmap(pngName);
	cout << ">> drawing finished, output to " + pngName << endl;
	int endTime = clock();
	cout << "running time: " << (endTime - startTime) / 1000.0 << "s" << endl;
	system("pause");
}
