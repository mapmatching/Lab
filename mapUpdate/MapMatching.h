/* 
 * Last Updated at [2014/9/1 12:34] by wuhao
 */
#pragma once
#include <iostream>
#include <fstream>
#include <direct.h>
#include <io.h>
#include <vector>
#include <map>
#include "GeoPoint.h"
#include "Map.h"
using namespace std;

//地图匹配所用参数
#define COEFFICIENT_FOR_EMISSIONPROB 140.2384599822997282786640971977//原始值为0.01402384599822997282786640971977，现扩大10000倍
#define COEFFICIENT_FOR_TRANSATIONPROB 0.31273997011//原始值为0.00931003342301998864175922391561，现扩大10000倍
//地图匹配通用参数
#define MINPROB 1e-150 //整体概率的下限

typedef list<GeoPoint*> Traj;


//地图匹配所用数据结构
struct Score//代表某个轨迹点对应的一个候选路段
{
	Edge* edge;//候选路段的指针
	long double score;//候选路段所具有的整体概率
	int preColumnIndex;//候选路段的前序路段的列索引
	double distLeft;//轨迹点的投影点到候选路段起点的距离
	Score(Edge* edge, long double score, int pre, double distLeft)
	{
		this->edge = edge;
		this->score = score;
		this->preColumnIndex = pre;
		this->distLeft = distLeft;
	}
};

class MapMatcher
{
public:	
	Map* roadNetwork;
	//保存计算过的两点间最短距离，键pair对表示起点和终点，值pair表示两点间最短距离和对应的deltaT
	//保存的deltaT的原因是：如果deltaT过小，则返回的最短距离可能为INF；而当再遇到相同起点和终点、而deltaT变大时，最短距离可能就不是INF了
	//类似的，当已保存的最短距离不是INF，而遇到某个更小的deltaT时，最短距离可能就是INF了
	std::map<pair<int, int>, pair<double, double>> shortestDistPair = std::map<pair<int, int>, pair<double, double>>();

	MapMatcher(Map* roadNetwork);
	void setMap(Map* roadNetwork);
	void MapMatching(list<GeoPoint*> &trajectory, list<Edge*>& resultEdges, double rangeOfCandidateEdges = 50.0);

private:
	double EmissionProb(double t, double dist);
	int GetStartColumnIndex(vector<Score> &row);
};