/* 
 * Last Updated at [2014/1/24 22:23] by wuhao
 */
#pragma once
#include "GeoPoint.h"
#include <list>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
using namespace std;
#define min(a,b)	(((a) < (b)) ? (a) : (b))
#define max(a,b)	(((a) > (b)) ? (a) : (b))
typedef list<GeoPoint*> Figure; //代表一条路形，每个GeoPoint*代表路形点，首尾节点即路的两个端点
typedef std::pair<double, double> simplePoint; //内部类型，勿改动

struct  Edge 
{
	Figure* figure;  //路形信息
	double lengthM;  //记录路段总长，单位为m
	int startNodeId; //路段起始点id
	int endNodeId;  //路段终止点id
	bool visited;  //辅助字段，外部调用勿改动
	int id;
};

struct AdjNode //邻接表结点
{
	int endPointId;
	int edgeId;
	AdjNode* next;
};

class Map
{
public:
	vector<Edge*> edges; //保存所有边的集合，如果边的两个端点至少一个不在范围内则为NULL，【逐个遍历需手动判断NULL】
	vector<GeoPoint*> nodes; //保存所有点的集合,如果点不在范围内则为NULL，【逐个遍历需手动判断NULL】
	vector<AdjNode*> adjList; //邻接表
	
	Map(string folderDir, int gridWidth);  //以gridWidth列的粒度来创建索引

	vector<Edge*> getNearEdges(double lat, double lon, double threshold) const; //返回距离(lat, lon)点严格小于threshold米的所有Edge*
	double distM(double lat, double lon, Edge* edge) const; //返回(lat,lon)点到edge的距离，单位为米
	double distM(double lat, double lon, Edge* edge, double& prjDist) const;//同上，同时记录投影点到edge起点的距离存入prjDist，无投影则记为0
	int hasEdge(int startNodeId, int endNodeId) const; //判断startNodeId与endNodeId之间有无边,没有边返回-1，有边返回edgeId
	void insertNode(double lat, double lon);
	void insertEdge(Edge* edge, int startNodeId, int endNodeId); //在当前图中插入边
	void delEdge(int edgeId);
	
	

private:
	int gridWidth, gridHeight;
	double gridSizeDeg;
	double strictThreshold = 0.1;
	list<Edge*>* **grid;
	/*double minLat = 1.22;
	double maxLat = 1.5;
	double minLon = 103.620;
	double maxLon = 104.0;*/
	double minLat = 0.99999;
	double maxLat = 1.6265;
	double minLon = 103.548;
	double maxLon = 104.1155;


	int getRowId(double lat) const;
	int getColId(double lon) const;
	double distM_withThres(double lat, double lon, Edge* edge, double threshold) const; //返回(lat,lon)点到edge的距离上界,提前预判优化版本	
	double calEdgeLength(Figure* figure) const;
	bool inArea(double lat, double lon) const;
	bool inArea(int nodeId) const;
	void createGridIndex();
	void createGridIndexForEdge(Edge *edge);
	void insertEdgeIntoGrid(Edge* edge,int row, int col);
	void insertEdge(int edgeId, int startNodeId, int endNodeId);
	
	void split(const string& src, const string& separator, vector<string>& dest);
	double cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3) const;
	void test();
};

