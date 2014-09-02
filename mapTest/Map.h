/* 
 * Last Updated at [2014/6/26 11:32] by wuhao
 */
#pragma once
#include "GeoPoint.h"
#include <list>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <queue>
#include "MapDrawer.h"
#define eps 1e-10
#define INF  1e7 //最短路径所用
#define MAXSPEED 50 //最大速度
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

//最短路径长度所用数据结构
struct NODE_DIJKSTRA 
{
	int t; double dist;
	NODE_DIJKSTRA(int i, double dist)
	{
		this->t = i;
		this->dist = dist;
	}
	bool operator < (const NODE_DIJKSTRA &rhs) const 
	{
		return dist > rhs.dist;
	}
};


class Map
{
public:
	vector<Edge*> edges; //保存所有边的集合，如果边的两个端点至少一个不在范围内则为NULL，【逐个遍历需手动判断NULL】
	vector<GeoPoint*> nodes; //保存所有点的集合,如果点不在范围内则为NULL，【逐个遍历需手动判断NULL】
	vector<AdjNode*> adjList; //邻接表
	
	void setArea(Area* area);
	Map(); //默认构造函数,需要手动调用open()函数来初始化
	Map(string folderDir, Area* area, int gridWidth = 0);  //在folderDir路径下载入地图文件,并以gridWidth列的粒度来创建索引,gridWidth<=0时为不建立索引
	void open(string folderDir, int gridWidth = 0);  //在folderDir路径下载入地图文件,并以gridWidth列的粒度来创建索引,适用于无参构造函数	


	/*基础功能函数*/
	int hasEdge(int startNodeId, int endNodeId) const; //判断startNodeId与endNodeId之间有无边,没有边返回-1，有边返回edgeId
	int insertNode(double lat, double lon); //插入一个新结点,返回新结点id
	int insertEdge(Figure* figure, int startNodeId, int endNodeId); //在当前图中插入边,返回新边id
	int splitEdge(int edgeId, double lat, double lon); //将edge在(lat,lon)点处分开成两段,(lat,lon)作为新结点加入,返回新结点的nodeId
	void getMinMaxLatLon(string nodeFilePath);
	void drawMap(Gdiplus::Color color, MapDrawer& md); //画出地图
	double distM(double lat, double lon, Edge* edge) const; //返回(lat,lon)点到edge的距离，单位为米

	/*Map Matching相关函数*/
	vector<Edge*> getNearEdges(double lat, double lon, double threshold) const; //返回距离(lat, lon)点严格小于threshold米的所有Edge*,会产生内存泄露
	void getNearEdges(double lat, double lon, double threshold, vector<Edge*>& dest); //推荐版本
	void getNearEdges(double lat, double lon, int k, vector<Edge*>& dest); //返回离(lat, lon)点距离最近的k条路段，存入dest
	double shortestPathLength(int ID1, int ID2, double dist1, double dist2, double deltaT);//求最短路，后两个参数啥意思我也不知道，不是我的代码
	double distM(double lat, double lon, Edge* edge, double& prjDist) const;//同上，同时记录投影点到edge起点的距离存入prjDist，无投影则记为0
	double distMFromTransplantFromSRC(double lat, double lon, Edge* edge, double& prjDist); //移植SRC版本：返回(lat,lon)点到edge的距离，单位为米；同时记录投影点到edge起点的距离存入prjDist
	
	
	/*删路相关*/
	void delEdge(int edgeId, bool delBirectionEdges = true); //从地图中删除edgeId，如果第二个参数为true则同时删除反向路
	void drawDeletedEdges(Gdiplus::Color color, MapDrawer& md); //画出删除的路
	void deleteEdgesRandomly(int delNum, double minEdgeLengthM); //在地图里随机删除长度不短于minEdgeLengthM的delNum条Edge(同时会删除反向路)，随机种子需自行在main中初始化
	//增强版本，随机删除长度不短于minEdgeLengthM的delNum条Edge，
	//同时删除的edge需满足到其距离小于aroundThresholdM的路段不超过aroundNumThreshold条（双向路以2计），
	//并将其周围的这些路全部删除，随机种子需自行在main中初始化
	//doOutput为true时将会把所有删除的路段号输出到文件（一行一个id）
	void deleteEdgesRandomlyEx(int delNum, double minEdgeLengthM, double aroundThresholdM, int aroundNumThreshold, bool doOutput = true);
	void deleteEdges(string path); //对于deleteEdgesRandomlyEx输出的所有删除的id号的文件读入后在路网中删除，保证调用此函数后的状态与调用deleteEdgesRandomlyEx()后的状态一样
	vector<Edge*> deletedEdges; //记录被删除掉的边

private:
	int gridWidth, gridHeight;
	double gridSizeDeg;
	double strictThreshold = 0;
	list<Edge*>* **grid;
	Area* area;
	//一些常用的area参考值
	//singapore half
	/*double minLat = 1.22;
	double maxLat = 1.5;
	double minLon = 103.620;
	double maxLon = 104.0;*/
	
	/*singapore full
	double minLat = 0.99999;
	double maxLat = 1.6265;
	double minLon = 103.548;
	double maxLon = 104.1155;*/

	//washington full
	//minLat:45.559102, maxLat : 49.108823, minLon : -124.722781, maxLon : -116.846465
	/*double minLat = 45.0;
	double maxLat = 49.5;
	double minLon = -125.0;
	double maxLon = -116.5;*/

	int getRowId(double lat) const;
	int getColId(double lon) const;
	double distM_withThres(double lat, double lon, Edge* edge, double threshold) const; //返回(lat,lon)点到edge的距离上界,提前预判优化版本	
	double calEdgeLength(Figure* figure) const;
	bool inArea(double lat, double lon) const;
	void createGridIndex();
	void createGridIndexForEdge(Edge *edge);
	void createGridIndexForSegment(Edge *edge, GeoPoint* fromPT, GeoPoint* toPt);
	void insertEdgeIntoGrid(Edge* edge,int row, int col);
	void insertEdge(int edgeId, int startNodeId, int endNodeId);
	
	void split(const string& src, const string& separator, vector<string>& dest);
	void split(const string& src, const char& separator, vector<string>& dest);
	double cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3) const;
	void test();
	void findNearEdges(Edge* edge, list<Edge*>& dest, double thresholdM); //返回距离edge长度不超过thresholdM的路段
	double distM(Edge* edge1, Edge* edge2, double threshold);
	void getNearEdges(Edge* edge, double thresholdM, vector<Edge*>& dest);
};

