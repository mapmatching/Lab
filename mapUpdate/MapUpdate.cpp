#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <time.h>
#include "MapDrawer.h"
#include "Map.h"
#include <list>
#include <vector>
#include <set>
#include <math.h>
#include "GeoPoint.h"
#include "StringOperator.h"
#include <iomanip>
#include "PolylineGenerator.h"
#include "PointGridIndex.h"
#include "StringOperator.h"
#include "ExpGenerator.h"
#include "TrajDrawer.h"
#include "GeoValidation.h"
//#include "MapMatching.h"

#define eps 1e-8
#define INFINITE 999999999
#define PI 3.1415926535898
using namespace std;
using namespace Gdiplus;

//Area area(1.294788, 1.327723, 103.784667, 103.825200); //small
//Area area(1.294788, 1.393593, 103.784667, 103.906266); //big
//Area area(1.343593, 1.442398, 103.784667, 103.906266); //big2
Area area(1.294788, 1.393593, 103.704667, 103.826266); //big3
int size = 5000;

bool zoomed = true;

Map roadNetwork;
Map originalRoadNetwork;
MapDrawer md;
PointGridIndex ptIndex;

int testSampleId;
string workspaceFolder;
string ptsFileName;
string pngFileName;

//***********************************************************************************************

//////////////////////////////////////////////////////////////////////////
///basic utiliy funcs
//////////////////////////////////////////////////////////////////////////
void extractPts(string trajFilePath)
{
	//////////////////////////////////////////////////////////////////////////
	///读入轨迹文件，将匹配失败的点输出到pts.txt,一次性
	//////////////////////////////////////////////////////////////////////////
	ifstream ifs(trajFilePath);
	ofstream ofs("out.txt");
	ofs << fixed << showpoint << setprecision(8);
	if (!ifs)
	{
		cout << "open file error" << endl;
		system("pause");
		exit(0);
	}
	while (ifs)
	{
		int time, mmRoadId;
		double lat, lon;
		ifs >> time;
		if (ifs.fail())
			break;
		if (time == -1)
			continue;
		ifs >> lat >> lon >> mmRoadId;
		if (mmRoadId == -1)
		{
			ofs << time  << " " << lat << " " << lon << endl;
		}
	}
	ofs.close();
	ifs.close();
}

void dumpInPts(string ptsFilePath, list<GeoPoint*>& pts)
{
	//////////////////////////////////////////////////////////////////////////
	///读入匹配失败的轨迹点文件，倒入pts容器中
	//////////////////////////////////////////////////////////////////////////
	ifstream ifs(ptsFilePath);
	if (!ifs)
	{
		cout << "打开文件 " << ptsFilePath << " 错误" << endl;
		system("pause");
	}
	while (ifs)
	{
		int time;
		double lat, lon;
		ifs >> time >> lat >> lon;
		if (ifs.fail())
			break;
		GeoPoint* pt = new GeoPoint(lat, lon, time);
		pts.push_back(pt);
	}
	ifs.close();
	cout << "点读入完成，共 " << pts.size() << " 个点" << endl;
}

void dumpInPtsEx(string ptsFilePath, list<GeoPoint*>& pts)
{
	//////////////////////////////////////////////////////////////////////////
	///读入匹配失败的轨迹点文件，倒入pts容器中
	//////////////////////////////////////////////////////////////////////////
	ifstream ifs(ptsFilePath);
	if (!ifs)
	{
		cout << "打开文件 " << ptsFilePath << " 错误" << endl;
		system("pause");
	}
	while (ifs)
	{
		int time, clusterId;
		double lat, lon, dir;
		ifs >> time >> lat >> lon >> dir >> clusterId;
		if (ifs.fail())
			break;
		GeoPoint* pt = new GeoPoint(lat, lon, time);
		pt->direction = dir;
		pt->clusterId = clusterId;
		pts.push_back(pt);
	}
	ifs.close();
	cout << "点读入完成，共 " << pts.size() << " 个点" << endl;
}

void dumpOutPts(string outFilePath, list<GeoPoint*>& pts)
{
	//////////////////////////////////////////////////////////////////////////
	///将pts中的轨迹点输出至outFilePath中
	//////////////////////////////////////////////////////////////////////////
	ofstream ofs(outFilePath);
	ofs << fixed << showpoint << setprecision(8);
	if (!ofs)
	{
		cout << "open file error" << endl;
		system("pause");
		exit(0);
	}
	for each (GeoPoint* pt in pts)
	{
		ofs << pt->time << " " << pt->lat << " " << pt->lon << " " << pt->direction  << " " << pt->clusterId << endl;
	}
	ofs.close();
}

void drawPts(MapDrawer& md, list<GeoPoint*>& pts, Gdiplus::Color color)
{
	for each (GeoPoint* pt in pts)
	{
		md.drawPoint(color, pt->lat, pt->lon);
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

//////////////////////////////////////////////////////////////////////////
///grid clustering
//////////////////////////////////////////////////////////////////////////
void dfs(int row, int col, bool** dfsState, vector<pair<int, int>>& connectingCompnt)
{
	//////////////////////////////////////////////////////////////////////////
	///对网格[row][col]进行dfs遍历，其中标记访问为此函数职责，dfsState为访问状态，true为已访问
	///将自己的索引编号存入connectingCompnt以记录连通分支信息
	//////////////////////////////////////////////////////////////////////////
	if (row >= ptIndex.gridHeight || row < 0 || col >= ptIndex.gridWidth || col < 0)
		return;
	if (dfsState[row][col])
		return;
	else
	{
		dfsState[row][col] = true;
		if (ptIndex.grid[row][col]->size() <= 3) //网格里点数量小于一定程度的忽略
			return;
		connectingCompnt.push_back(make_pair(row, col));
		dfs(row + 1, col, dfsState, connectingCompnt);
		dfs(row - 1, col, dfsState, connectingCompnt);
		dfs(row, col + 1, dfsState, connectingCompnt);
		dfs(row, col - 1, dfsState, connectingCompnt);
	}
}

void gridClustering()
{
	//////////////////////////////////////////////////////////////////////////
	///网格粗糙聚类，实际上就是个dfs
	///将每个聚类中的点输出到cluster i.txt的文件中
	///如果联通分支过小则忽略，不输出
	//////////////////////////////////////////////////////////////////////////
	bool** dfsState = new bool*[ptIndex.gridHeight];
	for (int i = 0; i < ptIndex.gridHeight; i++)
		dfsState[i] = new bool[ptIndex.gridWidth];
	for (int i = 0; i < ptIndex.gridHeight; i++)
	{
		for (int j = 0; j < ptIndex.gridWidth; j++)
		{
			dfsState[i][j] = false;
		}
	}
	int count = 0; //连通分支编号
	for (int row = 0; row < ptIndex.gridHeight; row++)
	{
		for (int col = 0; col < ptIndex.gridWidth; col++)
		{
			if (dfsState[row][col])
				continue;
			else
			{
				vector<pair<int, int>> connectingCompnt; //记录当前dfs的连通分支
				dfs(row, col, dfsState, connectingCompnt);
				if (connectingCompnt.size() < 10) //连通分支过小，忽略
					continue;
				else
				{
					//根据连通分支去索引里把点输出
					ofstream ofs("cluster " + StringOperator::intToString(count) + ".txt");
					ofs << fixed << showpoint << setprecision(8);
					
					for (int i = 0; i < connectingCompnt.size(); i++)
					{
						for each ( GeoPoint* pt in *(ptIndex.grid[connectingCompnt[i].first][connectingCompnt[i].second]) )
						{
							ofs << pt->time << " " << pt->lat << " " << pt->lon << endl;
						}
					}
					ofs.close();
					count++;
				}
			}
		}
	}
}


//////////////////////////////////////////////////////////////////////////
///outlier detecting
//////////////////////////////////////////////////////////////////////////
void calLMD(GeoPoint* pt, int k, double kNNThresholdM)
{
	//////////////////////////////////////////////////////////////////////////
	///计算pt到它的knn的平均距离
	//////////////////////////////////////////////////////////////////////////
	vector<GeoPoint*> kNNSet;
	ptIndex.kNN(pt, k, kNNThresholdM, kNNSet);
	double lmd = 0;
	for (int i = 0; i < kNNSet.size(); i++)
	{
		lmd += kNNSet[i]->dist;
	}
	lmd /= kNNSet.size();
	pt->lmd = lmd;
	/*for (int i = 0; i < kNNSet.size(); i++)
	{
		printf("%lf\n", kNNSet[i]->extendField0);
	}
	system("pause");*/
}

void outlierValidation(GeoPoint* pt, int gridRange, double supportRatio)
{
	
	/**********************************************************/
	/*test code starts from here*/
	/*if (pt->extendField1 > 20.0)
	{
		pt->extendField2 = 1;
		return;
	}*/
	if (pt->lmd < 3.0)
	{
		pt->isOutlier = 0;
		return;
	}
	//else
	//	pt->extendField2 = 1;
	//return;
	/*test code ends*/
	/**********************************************************/
	
	/*vector<GeoPoint*> nearPts;
	ptIndex.getNearPts(pt, gridRange, nearPts);
	for (int i = 0; i < nearPts.size(); i++)
	{
		if (nearPts[i]->extendField1 / pt->extendField1 > supportRatio)
		{
			pt->extendField2 = 1;
			return;
		}
	}*/
	pair<int, int> rolCol = ptIndex.getRowCol(pt);
	int rowPt = rolCol.first;
	int colPt = rolCol.second;
	int row1 = rowPt - gridRange;
	int col1 = colPt - gridRange;
	int row2 = rowPt + gridRange;
	int col2 = colPt + gridRange;
	if (row1 < 0) row1 = 0;
	if (row2 >= ptIndex.gridHeight) row2 = ptIndex.gridHeight - 1;
	if (col1 < 0) col1 = 0;
	if (col2 >= ptIndex.gridWidth) col2 = ptIndex.gridWidth - 1;
	int currentGridCount = ptIndex.grid[rowPt][colPt]->size();
	for (int row = row1; row <= row2; row++)
	{
		for (int col = col1; col <= col2; col++)
		{
			if (ptIndex.grid[row][col]->size() / currentGridCount > supportRatio)
			{
				pt->isOutlier = 1;
				return;
			}
		}
	}
}

void outlierReValidation(GeoPoint* pt)
{
	if (pt->isOutlier == 0)
		return;
	pair<int, int> rolCol = ptIndex.getRowCol(pt);
	int rowPt = rolCol.first;
	int colPt = rolCol.second;
	int row1 = rowPt - 1;
	int col1 = colPt - 1;
	int row2 = rowPt + 1;
	int col2 = colPt + 1;
	if (row1 < 0) row1 = 0;
	if (row2 >= ptIndex.gridHeight) row2 = ptIndex.gridHeight - 1;
	if (col1 < 0) col1 = 0;
	if (col2 >= ptIndex.gridWidth) col2 = ptIndex.gridWidth - 1;
	int innerPtCount = 0;
	int outlierCount = 0;
	for (int row = row1; row <= row2; row++)
	{
		for (int col = col1; col <= col2; col++)
		{
			if (row == rowPt && col == colPt)
				continue;
			for each (GeoPoint* pt in *(ptIndex.grid[row][col]))
			{
				if (pt->isOutlier == 1)
					outlierCount++;
				else
					innerPtCount++;
			}
		}
	}
	if (outlierCount == 0)
	{
		if (innerPtCount >= 30)
		{
			pt->isOutlier = 0;
			return;
		}
		else
			return;
	}
	if (innerPtCount / outlierCount > 10)
	{
		pt->isOutlier = 0;
	}
}

//driver func
void outlierDetecting()
{
	int k = 20;
	double kNNThresholdM = 3;
	int gridRange = 2;
	double supportRatio = 4.0;
	
	for (int row = 0; row < ptIndex.gridHeight; row++)
	{
		for (int col = 0; col < ptIndex.gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex.grid[row][col]))
			{
				calLMD(pt, k, kNNThresholdM);
			}
		}
		printf("row %d计算完成,共 %d 行\n", row, ptIndex.gridHeight);
	}
	printf("LMD计算完成\n");

	for (int row = 0; row < ptIndex.gridHeight; row++)
	{
		for (int col = 0; col < ptIndex.gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex.grid[row][col]))
			{
				outlierValidation(pt, gridRange, supportRatio);
			}
		}
		printf("row %d计算完成,共 %d 行\n", row, ptIndex.gridHeight);
	}
	printf("outlier点排除完成\n");
	return;

	for (int row = 0; row < ptIndex.gridHeight; row++)
	{
		for (int col = 0; col < ptIndex.gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex.grid[row][col]))
			{
				outlierReValidation(pt);
			}
		}
		printf("row %d计算完成,共 %d 行\n", row, ptIndex.gridHeight);
	}
}
//visualization func
void drawPtsWithoutOutliers(MapDrawer& md, Gdiplus::Color color)
{
	for (int row = 0; row < ptIndex.gridHeight; row++)
	{
		for (int col = 0; col < ptIndex.gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex.grid[row][col]))
			{
				if (pt->isOutlier == 1)
					//continue;
					md.drawPoint(Color::Red, pt->lat, pt->lon);
				else
					md.drawPoint(Color::Blue, pt->lat, pt->lon);
			}
		}
	}
	pngFileName = ptsFileName + "_without_noise";
}
//dump out func
void dumpPtsWithoutNoise(list<GeoPoint*>& pts)
{
	list<GeoPoint*> newPts;
	for each (GeoPoint* pt in pts)
	{
		if (pt->isOutlier != 1)
		{
			newPts.push_back(pt);
		}
	}
	dumpOutPts(workspaceFolder + ptsFileName + "_without_noise.txt", newPts);
}


//////////////////////////////////////////////////////////////////////////
///direction computing
//////////////////////////////////////////////////////////////////////////
//slow ver.
bool inRect(GeoPoint* p0, GeoPoint* p, double dirAngle, double d, double l)
{
	//////////////////////////////////////////////////////////////////////////
	///判断p是否在p0的dirAngle方向的长宽为l*d的矩形中
	//////////////////////////////////////////////////////////////////////////
	double dist = GeoPoint::distM(p0, p);
	pair<double, double> vec_dir, vec_p0p;
	if (abs(dirAngle - PI / 2) < eps)
		vec_dir = make_pair(0, 1);
	else
		vec_dir = make_pair(1 / tan(dirAngle), 1);
	vec_p0p = make_pair(p->lon - p0->lon, p->lat - p0->lat);
	double cos_theta = abs(vec_dir.first * vec_p0p.first + vec_dir.second * vec_p0p.second)
		/ sqrt((vec_dir.first * vec_dir.first + vec_dir.second * vec_dir.second) * (vec_p0p.first * vec_p0p.first + vec_p0p.second * vec_p0p.second));
	double sin_theta = sqrt(1 - cos_theta * cos_theta);
	if (dist * sin_theta <= d / 2)
		return true;
	else
		return false;
}
double calCount(GeoPoint* p0, double dirAngle, double d, double l, vector<GeoPoint*>& nearPts)
{
	//////////////////////////////////////////////////////////////////////////
	///统计p0的dirAngle方向的长宽为l*d的矩形中点的数量
	///为了不重复获取附近的点，nearPts记录距离p0不长于sqrt((d/2)^2+l^2)的所有点
	//////////////////////////////////////////////////////////////////////////
	int count = 0;
	for each (GeoPoint* pt in nearPts)
	{
		if (inRect(p0, pt, dirAngle, d, l))
			count++;
	}
	return count;
}
void calDir(GeoPoint* p0, double angleStep, double d, double l)
{
	//////////////////////////////////////////////////////////////////////////
	///计算p0的点方向
	///stepAngle为最小角度粒度，方向范围为[0,π)
	///方向存入p0的directin域中
	//////////////////////////////////////////////////////////////////////////

	vector<GeoPoint*> nearPts;
	ptIndex.getNearPts(p0, sqrt(d / 2 * d / 2 + l*l), nearPts);
	vector<int> countVec;
	for (double dir = 0; dir < PI; dir += angleStep)
	{
		countVec.push_back(calCount(p0, dir, d, l, nearPts));
	}
	double bestDir = countVec[0];
	int maxCount = -1;
	for (int i = 0; i < countVec.size(); i++)
	{
		if (countVec[i] > maxCount)
		{
			maxCount = countVec[i];
			bestDir = i * angleStep;
		}
	}
	p0->direction = bestDir;
}

//fast ver.
void countEx(GeoPoint* p0, GeoPoint* p, vector<int>& countVec, double d, double l, double angleStep)
{
	if (p == p0)
		return;
	double theta_1, theta_2;
	pair<double, double> vec_p0p;
	GeoPoint tempMirrorPt;
	if (p->lat <= p0->lat)
	{
		tempMirrorPt.lon = 2 * p0->lon - p->lon;
		tempMirrorPt.lat = 2 * p0->lat - p->lat;
		p = &tempMirrorPt;
	}
	vec_p0p = make_pair(p->lon - p0->lon, p->lat - p0->lat);
	theta_1 = acos((1 * vec_p0p.first + 0 * vec_p0p.second)
		/ sqrt((1 * 1 + 0 * 0) * (vec_p0p.first * vec_p0p.first + vec_p0p.second * vec_p0p.second)));
	theta_2 = acos((d / 2) / GeoPoint::distM(p0, p));

	double theta_s, theta_e;
	theta_s = theta_1 + theta_2 - PI / 2;
	theta_e = theta_1 - theta_2 + PI / 2;
	/*printf("p0 = (%.8lf, %.8lf), p = (%.8lf, %.8lf)\n", p0->lon, p0->lat, p->lon, p->lat);
	printf("p0p(%.8lf, %.8lf)\n", vec_p0p.first, vec_p0p.second);
	printf("t1 = %lf, t2 = %lf, ts = %lf, te = %lf\n", theta_1, theta_2, theta_s, theta_e);
	system("pause");*/
	for (double dir = 0, i=0; dir < PI; dir+=angleStep, i++)
	{
		if (dir >= theta_s && dir <= theta_e)
			countVec[i]++;
	}
}
void calDirEx(GeoPoint* p0, double angleStep, double d, double l)
{
	//////////////////////////////////////////////////////////////////////////
	///计算p0的点方向
	///angleStep为最小角度粒度，方向范围为[0,π)
	///方向存入p0的direction域中
	//////////////////////////////////////////////////////////////////////////

	vector<GeoPoint*> nearPts;
	ptIndex.getNearPts(p0, sqrt(d / 2 * d / 2 + l*l), nearPts);
	vector<int> countVec;
	for (double dir = 0; dir < PI; dir += angleStep)
	{
		countVec.push_back(0);
	}
	for each (GeoPoint* p in nearPts)
	{
		countEx(p0, p, countVec, d, l, angleStep);
	}
	double bestDir = countVec[0];
	int maxCount = -1;
	for (int i = 0; i < countVec.size(); i++)
	{
		if (countVec[i] > maxCount)
		{
			maxCount = countVec[i];
			bestDir = i * angleStep;
		}
	}
	p0->direction = bestDir;
}

//driver func
void calPtsDirs(list<GeoPoint*>& pts)
{
	//////////////////////////////////////////////////////////////////////////
	///计算pts中所有点的点方向
	//////////////////////////////////////////////////////////////////////////
	double d = 4.0; //矩形宽度
	double l = 36.0; //矩形长度
	double angleStep = PI / 24; //角度枚举粒度
	int tenPercent = pts.size() / 10;
	int count = 1;
	for each (GeoPoint* pt in pts)
	{
		if (count % tenPercent == 0)
			printf("计算点方向 %d0%% 已完成\n", count / tenPercent);
		//calDir(pt, angleStep, d, l);
		calDirEx(pt, angleStep, d, l);
		count++;
	}
	printf("计算点方向已完成\n");
}
//visualization func
void drawDir(GeoPoint* pt, MapDrawer& md)
{
	//////////////////////////////////////////////////////////////////////////
	///画出pt的点方向
	//////////////////////////////////////////////////////////////////////////
	GeoPoint* ptEnd = new GeoPoint;
	double length = 3 / GeoPoint::geoScale;
	ptEnd->lat = pt->lat + length * sin(pt->direction);
	ptEnd->lon = pt->lon + length * cos(pt->direction);
	md.drawLine(Color::Blue, pt->lat, pt->lon, ptEnd->lat, ptEnd->lon);
}
void drawPtsDir(list<GeoPoint*>& pts, MapDrawer& md)
{
	for each (GeoPoint* pt in pts)
	{
		drawDir(pt, md);
	}
	pngFileName = ptsFileName + "_dir";
}
//dump out func
void dumpPtsWithDir(list<GeoPoint*>& pts)
{
	dumpOutPts(ptsFileName+ "_dir.txt", pts);
}


//////////////////////////////////////////////////////////////////////////
///direction clustering
//////////////////////////////////////////////////////////////////////////
struct Cluster
{
	list<GeoPoint*> pts;
	double direction;
	int clusterId;
	vector<GeoPoint*> polyline;
	double polylineLengthM;
	double avgDirection;

	Area area; //记录cluster的MBR	

	Cluster()
	{
		direction = 0;
		area.setArea(999, -999, 999, -999);
	}

	void add(GeoPoint* pt)
	{
		//////////////////////////////////////////////////////////////////////////
		///将pt加入cluster，并将pt的clusterId域更新为该cluster的Id同时重新计算平均direction
		///为了加快getPtsNearCluster操作，加入点时会同时更新当前cluster的mbr
		///[ATTENTION]:这儿不会判断pt的原来clusterId
		//////////////////////////////////////////////////////////////////////////
		pt->clusterId = clusterId;
		direction = (pts.size() * direction + pt->direction) / (pts.size() + 1);
		pts.push_back(pt);
		//update mbr
		if (pt->lat > area.maxLat) area.maxLat = pt->lat;
		if (pt->lat < area.minLat) area.minLat = pt->lat;
		if (pt->lon > area.maxLon) area.maxLon = pt->lon;
		if (pt->lon < area.minLon) area.minLon = pt->lon;
	}

};

void getPtsNearCluster(Cluster* cluster, double distThresM, double angleThres, vector<GeoPoint*>& nearPts)
{
	//////////////////////////////////////////////////////////////////////////
	///对cluster的mbr向外扩展一圈的网格中满足角度与cluster角度只差小于angleThres的点加入nearPts中
	//////////////////////////////////////////////////////////////////////////
	nearPts.clear();
	pair<int, int> rowCol_s = ptIndex.getRowCol(new GeoPoint(cluster->area.minLat, cluster->area.minLon));
	pair<int, int> rowCol_e = ptIndex.getRowCol(new GeoPoint(cluster->area.maxLat, cluster->area.maxLon));
	rowCol_s.first--;
	rowCol_s.second--;
	rowCol_e.first++;
	rowCol_e.second++;
	if (rowCol_s.first < 0) rowCol_s.first = 0;
	if (rowCol_e.first >= ptIndex.gridHeight) rowCol_e.first = ptIndex.gridHeight - 1;
	if (rowCol_s.second < 0) rowCol_s.second = 0;
	if (rowCol_e.second >= ptIndex.gridWidth) rowCol_e.second = ptIndex.gridWidth - 1;

	for (int row = rowCol_s.first; row <= rowCol_e.first; row++)
	{
		for (int col = rowCol_s.second; col <= rowCol_e.second; col++)
		{
			for each (GeoPoint* pt in *(ptIndex.grid[row][col]))
			{
				if (pt->clusterId == -1 && 
					(abs(pt->direction - cluster->direction) < angleThres || 
					PI - abs(pt->direction - cluster->direction) < angleThres))
				{
					nearPts.push_back(pt);
				}
			}
		}
	}
}

bool fetch(Cluster* cluster, double distThresM, double angleThres)
{
	//////////////////////////////////////////////////////////////////////////
	///将cluster附近满足聚类条件的点吸进来，如果没点可吸返回false
	//////////////////////////////////////////////////////////////////////////
	bool canFetch = false;
	vector<GeoPoint*> nearPts;
	getPtsNearCluster(cluster, distThresM, angleThres, nearPts);
	for (int i = 0; i < nearPts.size(); i++)
	{
		bool validation = false;
		GeoPoint* currentPt = nearPts[i];
		vector<GeoPoint*> ptsNearCurrentPt;
		ptIndex.getNearPts(currentPt, 1, ptsNearCurrentPt);
		//遍历currentPt附近格子里属于cluster中的点
		for (int j = 0; j < ptsNearCurrentPt.size(); j++)
		{
			if (ptsNearCurrentPt[j]->clusterId != cluster->clusterId)
				continue;
			if (GeoPoint::distM(ptsNearCurrentPt[j], currentPt) < distThresM)
			{
				validation = true;
				break;
			}
		}
		if (validation)
		{
			cluster->add(currentPt);
			canFetch = true;
		}
	}
	return canFetch;
}

Cluster* genOneCluster(int clusterId, GeoPoint* seedPt, double distThresM, double angleThres)
{
	//////////////////////////////////////////////////////////////////////////
	///根据种子点seedPt创建一个cluster
	///[TODO]:种子点一个还是多个比较好？种子点选取是否有讲究？
	//////////////////////////////////////////////////////////////////////////
	Cluster* cluster = new Cluster;
	cluster->clusterId = clusterId;
	cluster->add(seedPt);
	while (fetch(cluster, distThresM, angleThres)){}
	return cluster;
}

//driver func
void doDirCluster(list<GeoPoint*>& pts, vector<Cluster*>& clusters)
{
	//////////////////////////////////////////////////////////////////////////
	///对pts中所有点做cluster,所有cluster存入clusters容器中
	///每个点的clusterId域中记录所在cluster的Id，该id也是clusters数组中的索引
	///每个点初始clusterId为-1
	//////////////////////////////////////////////////////////////////////////
	clusters.clear();
	double distThresM = 15;
	double angleThres = 30.0 / 180.0 * PI;
	int currentClusterId = 0;
	int count = 1;
	int tenPercent = pts.size() / 10;
	for each (GeoPoint* pt in pts)
	{
		if (count % tenPercent == 0)
			printf("方向聚类 %d0%% 已完成\n", count / tenPercent);
		count++;
		if (pt->clusterId != -1)
		{
			continue;
		}
		Cluster* cluster = genOneCluster(currentClusterId, pt, distThresM, angleThres);
		clusters.push_back(cluster);
		currentClusterId++;
		
	}
}
//visualization func
void drawPtsWithCluster(vector<Cluster*>& clusters, MapDrawer& md)
{
	for (int i = 0; i < clusters.size(); i++)
	{
		Color color = randomColor();
		for each (GeoPoint* pt in clusters[i]->pts)
		{
			md.drawPoint(color, pt->lat, pt->lon);
		}
	}
	pngFileName = ptsFileName + "_cluster";
}
//dump out func
void dumpPtsWithCluster(vector<Cluster*>& clusters)
{
	ofstream ofs(ptsFileName + "_cluster.txt");
	ofs << fixed << showpoint << setprecision(8);
	for (int i = 0; i < clusters.size(); i++)
	{
		for each (GeoPoint* pt in clusters[i]->pts)
		{
			ofs << pt->time << " " << pt->lat << " " << pt->lon << " " << pt->direction << " " << pt->clusterId << endl;
		}
	}
}
//dump in func
void dumpInPtsWithCluster(string ptsFilePath, list<GeoPoint*>& pts, vector<Cluster*>& clusters)
{
	//////////////////////////////////////////////////////////////////////////
	///读入匹配失败的轨迹点文件，倒入pts容器中,同时将聚类存入clusters
	//////////////////////////////////////////////////////////////////////////
	ifstream ifs(ptsFilePath);
	if (!ifs)
	{
		cout << "打开文件 " << ptsFilePath << " 错误" << endl;
		system("pause");
	}
	int currentClusterId = -1;
	Cluster* cluster = NULL;
	while (ifs)
	{
		int time, clusterId;
		double lat, lon, dir;
		ifs >> time >> lat >> lon >> dir >> clusterId;
		if (ifs.fail())
			break;
		GeoPoint* pt = new GeoPoint(lat, lon, time);
		pt->direction = dir;
		pt->clusterId = clusterId;
		pts.push_back(pt);
		if (currentClusterId != clusterId)
		{
			if (cluster != NULL)
			{
				clusters.push_back(cluster);
			}
			cluster = new Cluster;
			currentClusterId++;
			cluster->clusterId = currentClusterId;			
		}
		cluster->add(pt);
	}
	ifs.close();
	cout << "点读入完成，共 " << pts.size() << " 个点" << endl;
}


//////////////////////////////////////////////////////////////////////////
///generate polyline
//////////////////////////////////////////////////////////////////////////
void drawPolyline(PolylineGenerator pg, MapDrawer& md)
{
	cout << "polysize before drawing " << pg.polyline.size() << endl;
	for (int i = 0; i < pg.polyline.size() - 1; i++)
	{
		md.drawBoldLine(Gdiplus::Color::Black, (int)pg.polyline[i].x, (int)pg.polyline[i].y, (int)pg.polyline[i + 1].x, (int)pg.polyline[i + 1].y);
	}
	for (int i = 0; i < pg.polyline.size(); i++)
	{
		md.drawBigPoint(Gdiplus::Color::Red, (int)pg.polyline[i].x, (int)pg.polyline[i].y);
	}
}

bool isAGoodCluster(Cluster* cluster)
{
	//////////////////////////////////////////////////////////////////////////
	///判断cluster是否值得生成polyline
	//////////////////////////////////////////////////////////////////////////
	set<pair<int, int>> gridSet;
	for each(GeoPoint* pt in cluster->pts)
	{
		pair<int, int> rowCol = ptIndex.getRowCol(pt);
		if (gridSet.count(rowCol) == 0)
			gridSet.insert(rowCol);
	}
	if (gridSet.size() <= 2)
		return false;
	else
		return true;
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

double distM(GeoPoint* pt, vector<GeoPoint*>& polyline)
{
	//////////////////////////////////////////////////////////////////////////
	///返回点pt到poly的距离,单位为m
	///距离定义为：min(点到可投影边的投影距离，点到所有形状点的欧氏距离)
	//////////////////////////////////////////////////////////////////////////
	double minDist = 9999;
	//遍历端点距离
	for (vector<GeoPoint*>::iterator iter = polyline.begin(); iter != polyline.end(); iter++)
	{
		double tmpDist = GeoPoint::distM(pt, (*iter));
		if (tmpDist < minDist)
			minDist = tmpDist;
	}
	//遍历投影距离
	vector<GeoPoint*>::iterator iter = polyline.begin();
	vector<GeoPoint*>::iterator nextIter = polyline.begin();
	nextIter++;
	while (nextIter != polyline.end())
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

bool isBadClusterEx(Cluster* cluster, vector<Cluster*>& clusters, MapDrawer& md)
{
	//////////////////////////////////////////////////////////////////////////
	///判断cluster是否值得生成polyline
	//////////////////////////////////////////////////////////////////////////
	double lengthThresM = 50;
	double distThresM = 40;
	double angleThresM_1 = 50 * PI / 180; //两条polyline的方向差
	double angleThresM_2 = 30 * PI / 180; //polyline方向与点平均方向差

	if (cluster->polylineLengthM <= lengthThresM)
		return true;
	//cal pts ave dir in cluster
	double avgPtDirection = 0;
	for each(GeoPoint* pt in cluster->pts)
	{
		avgPtDirection += pt->direction;
	}
	avgPtDirection /= cluster->pts.size();

	
	/**********************************************************/
	/*test code starts from here*/
	//draw polyline dir
	/*GeoPoint* ptEnd = new GeoPoint;
	double length = 100 / GeoPoint::geoScale;
	ptEnd->lat = cluster->polyline[0]->lat + length * sin(cluster->avgDirection);
	ptEnd->lon = cluster->polyline[0]->lon + length * cos(cluster->avgDirection);
	md.drawLine(Color::Blue, cluster->polyline[0]->lat, cluster->polyline[0]->lon, ptEnd->lat, ptEnd->lon);
	/*test code ends*/
	/**********************************************************/
		if (abs(cluster->avgDirection - avgPtDirection) > angleThresM_2 &&
		PI - abs(cluster->avgDirection - avgPtDirection) > angleThresM_2)
		return true;

	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i]->polyline.size() == 0)
			continue;
		if (clusters[i] == cluster)
			continue;
		if (distM(cluster->polyline[0], clusters[i]->polyline) > distThresM ||
			distM(cluster->polyline[cluster->polyline.size() - 1], clusters[i]->polyline) > distThresM)
			continue;
		if ((abs(cluster->avgDirection - clusters[i]->avgDirection) < angleThresM_1 ||
			PI - abs(cluster->avgDirection - clusters[i]->avgDirection) < angleThresM_1) &&
			cluster->polylineLengthM < clusters[i]->polylineLengthM)
			return true;
	}
	return false;
}

void genPolyLine(Cluster* cluster, MapDrawer& md)
{	
	if (cluster->pts.size() < 20)
	{
		return;
	}
	if (!isAGoodCluster(cluster))
	{
		return;
	}
	printf("clusterId = %d, #pts = %d\n", cluster->clusterId, cluster->pts.size());
	
	PolylineGenerator pg;
	//将cluster里的轨迹点全部倒到pts里
	list<Pt> pts;

	//设置坐标转换器，将空间坐标映射到5000像素范围上比较合适
	MapDrawer mdTempForCoordThrans;
	mdTempForCoordThrans.setArea(&area);
	mdTempForCoordThrans.setResolution(5000);

	for each (GeoPoint* gPt in cluster->pts)
	{
		Pt tmpPt;
		Gdiplus::Point gdiPt = mdTempForCoordThrans.geoToScreen(gPt->lat, gPt->lon);
		tmpPt.x = (double)gdiPt.X;
		tmpPt.y = (double)gdiPt.Y;
		pts.push_back(tmpPt);
	}
	pg.genPolyline(pts);
	//update polyline info in cluster
	GeoPoint* prePt = NULL;
	double preAngle = 9999;
	for (int i = 0; i < pg.polyline.size(); i++)
	{
		GeoPoint* pt = new GeoPoint(mdTempForCoordThrans.screenToGeo((int)pg.polyline[i].x, (int)pg.polyline[i].y));
		cluster->polyline.push_back(pt);
		if (prePt == NULL)
		{
			cluster->polylineLengthM = 0;
			cluster->avgDirection = 0;
		}
		else
		{
			//计算polyline的平均方向（每段方向*每段长度权）
			cluster->polylineLengthM += GeoPoint::distM(prePt, pt);
			double vec_x = pt->lon - prePt->lon;
			double vec_y = pt->lat - prePt->lat;
			double angle = acos(vec_x / sqrt(vec_x * vec_x + vec_y * vec_y));
			//printf("old_angle = %.8lf, pre_angle = %.8lf, vec_y = %.8lf\n", angle / PI * 180.0, preAngle / PI * 180.0, vec_y);
			if (vec_y < 0)
			{
				if (preAngle > 2 * PI)
				{
					angle = PI - angle;
				}
				else if (angle < PI / 2)
				{
					if (abs(-angle - preAngle) < abs(PI - angle - preAngle))
						angle = -angle;
					else
						angle = PI - angle;
				}
				else
				{
					if (abs(PI-angle - preAngle) < abs(2 * PI - angle - preAngle))
						angle = PI - angle;
					else
						angle = 2 * PI - angle;
				}
			}
			else
			{
				if (preAngle > 2 * PI)
				{
					angle = angle;
				}
				else if (angle < PI / 2)
				{
					if (abs(PI + angle - preAngle) < abs(angle - preAngle))
						angle = PI + angle;
				}
				else
				{
					if (abs(angle - PI - preAngle) < abs(angle - preAngle))
						angle = angle - PI;
				}
			}
			preAngle = angle;
			//printf("angle = %.8lf\n", angle / PI * 180.0);
			cluster->avgDirection += angle * GeoPoint::distM(pt, prePt);
		}
		prePt = pt;
	}
	cluster->avgDirection /= cluster->polylineLengthM;
	if (cluster->avgDirection < 0)
		cluster->avgDirection += PI;
	if (cluster->avgDirection >= PI)
		cluster->avgDirection -= PI;
	/**********************************************************/
	/*test code starts from here*/
	double avgPtDirection = 0;
	for each(GeoPoint* pt in cluster->pts)
	{
		avgPtDirection += pt->direction;
	}
	avgPtDirection /= cluster->pts.size();
	printf("length = %lf, avgDir = %lf, avgPtDir = %lf\n",
		cluster->polylineLengthM, cluster->avgDirection, avgPtDirection);
	/*test code ends*/
	/**********************************************************/

	cout << "polyline 生成成功！" << endl;
}

void genAllPolyLinesEx()
{
	//////////////////////////////////////////////////////////////////////////
	///
	//////////////////////////////////////////////////////////////////////////
	
}

void drawPolyline(Cluster* cluster, MapDrawer& md, Color color)
{
	for (int i = 0; i < cluster->polyline.size() - 1; i++)
	{
		md.drawBoldLine(color, cluster->polyline[i]->lat, cluster->polyline[i]->lon, 
			cluster->polyline[i + 1]->lat, cluster->polyline[i + 1]->lon);
	}
	for (int i = 0; i < cluster->polyline.size(); i++)
	{
		md.drawBigPoint(Gdiplus::Color::Red, cluster->polyline[i]->lat, cluster->polyline[i]->lon);
	}
	md.drawBigPoint(Gdiplus::Color::Green, cluster->polyline[0]->lat, cluster->polyline[0]->lon);
}

//driver func
void genAllPolyLines(vector<Cluster*>& clusters, MapDrawer& md)
{
	for (int i = 0; i < clusters.size(); i++)
	{
		genPolyLine(clusters[i], md);
	}

	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i]->polyline.size() == 0)
			continue;
		if (isBadClusterEx(clusters[i], clusters, md))
		{
			//drawPolyline(clusters[i], md, Color::Aqua);
			clusters[i]->polyline.clear();
		}
		else
		{
			//drawPolyline(clusters[i], md, Color::Gray);
		}
	}

	pngFileName = ptsFileName + "_polyline";
}


//////////////////////////////////////////////////////////////////////////
///polyline refinement
//////////////////////////////////////////////////////////////////////////
GeoPoint* intersectCheck(GeoPoint* p1, GeoPoint* p2, vector<GeoPoint*>& polyline)
{
	//////////////////////////////////////////////////////////////////////////
	///判断对于路段polyline,是否会与线段p1p2相交
	///相交则返回交点，不相交则返回NULL
	//////////////////////////////////////////////////////////////////////////
	double rate = 1000;
	double x1 = p1->lon;
	double y1 = p1->lat;
	double x2 = p2->lon;
	double y2 = p2->lat;

	x1 *= rate;
	y1 *= rate;
	x2 *= rate;
	y2 *= rate;


	double A = y2 - y1;
	double B = x1 - x2;
	double C = -A * x1 - B * y1;
	for (int i = 0; i < polyline.size() - 1; i++)
	{
		double _x1 = polyline[i]->lon;
		double _y1 = polyline[i]->lat;
		double _x2 = polyline[i + 1]->lon;
		double _y2 = polyline[i + 1]->lat;

		_x1 *= rate;
		_y1 *= rate;
		_x2 *= rate;
		_y2 *= rate;

		double _A = _y2 - _y1;
		double _B = _x1 - _x2;
		double _C = -_A * _x1 - _B * _y1;
		double val1 = A * _x1 + B * _y1 + C;
		double val2 = A * _x2 + B * _y2 + C;
		double _val1 = _A * x1 + _B * y1 + _C;
		double _val2 = _A * x2 + _B * y2 + _C;
		
		/**********************************************************/
		/*test code starts from here*/
		//printf("val1 = %.8lf, val2 = %.8lf, _val1 = %.8lf, _val2 = %.8lf\n", val1, val2, _val1, _val2);
		/*test code ends*/
		/**********************************************************/
		
		if (val1 * val2 < 1e-4 && _val1 * _val2 <= 1e-4)
		{
			//cal intersection			
			double intersectY = (A * _C - _A * C) / (_A * B - A * _B);
			double intersectX = (-B * intersectY - C) / A;

			intersectX /= rate;
			intersectY /= rate;

			GeoPoint* intersection = new GeoPoint(intersectY, intersectX);
			return intersection;
		}
	}
	return NULL;

}

void refineOnePolyline(Cluster* objectCluster, vector<Cluster*>& allClusters)
{
	vector<GeoPoint*> newPolyline;
	double thresholdM = 30;  //如果当前扫描距离超过thresholdM则不切断

	//find front cut point
	double currentLength = 0; //记录当前扫描距离
	int intersectAfter_s = -1; //记录intersection前面离其最近的顶点索引号
	GeoPoint* intersection_s = NULL, *intersection_e = NULL;
	bool cutFinishFlag = false;
	//对于objectCluster的每一段
	for (int i = 0; i < objectCluster->polyline.size() - 1; i++)
	{
		if (currentLength > thresholdM)
			break;
		//对于每一条其他的路段
		for each (Cluster* cluster in allClusters)
		{
			if (cluster->polyline.size() == 0)
				continue;
			if (cluster == objectCluster)
				continue;
			intersection_s = intersectCheck(objectCluster->polyline[i], objectCluster->polyline[i + 1], cluster->polyline);
			
			if (intersection_s == NULL) //无交点
				continue;
			else //有交点
			{
				if (currentLength + GeoPoint::distM(objectCluster->polyline[i], intersection_s) > thresholdM)
				{
					intersection_s = NULL;
					cutFinishFlag = true;
					break;
				}
				intersectAfter_s = i;
				cutFinishFlag = true;
				break;
			}				
		}
		if (cutFinishFlag)
			break;
		currentLength += GeoPoint::distM(objectCluster->polyline[i], objectCluster->polyline[i+1]);
	}

	//find back cut point
	cutFinishFlag = false;
	currentLength = 0;
	int intersectAfter_e = -1; //记录intersection前面离其最近的顶点索引号
	//对于objectCluster的每一段
	for (int i = objectCluster->polyline.size()-2; i >= 0; i--)
	{
		if (currentLength > thresholdM)
			break;
		//对于每一条其他的路段
		for each (Cluster* cluster in allClusters)
		{
			if (cluster->polyline.size() == 0)
				continue;
			if (cluster == objectCluster)
				continue;
			intersection_e = intersectCheck(objectCluster->polyline[i], objectCluster->polyline[i + 1], cluster->polyline);
			if (intersection_e == NULL) //无交点
				continue;
			else //有交点
			{
				if (currentLength + GeoPoint::distM(objectCluster->polyline[i+1], intersection_e) > thresholdM)
				{
					intersection_e = NULL;
					cutFinishFlag = true;
					break;
				}
				intersectAfter_e = i;
				cutFinishFlag = true;
				break;
			}
		}
		if (cutFinishFlag)
			break;
		currentLength += GeoPoint::distM(objectCluster->polyline[i], objectCluster->polyline[i + 1]);
	}
	//cut
	if (intersection_s == NULL && intersection_e == NULL) //两头都不用切
		return;
	else if (intersection_s != NULL && intersection_e == NULL) //只切头不切尾
	{
		for (int i = 0; i < objectCluster->polyline.size(); i++)
		{
			if (i < intersectAfter_s)
				continue;
			else if (i == intersectAfter_s)
				newPolyline.push_back(intersection_s);
			else
				newPolyline.push_back(objectCluster->polyline[i]);

		}
	}
	else if (intersection_s == NULL && intersection_e != NULL) //只切尾不切头
	{
		for (int i = 0; i < objectCluster->polyline.size(); i++)
		{
			if (i > intersectAfter_e)
				continue;
			else if (i == intersectAfter_e)
				newPolyline.push_back(intersection_e);
			else
				newPolyline.push_back(objectCluster->polyline[i]);
		}
	}
	else //两头都切
	{
		//////////////////////////////////////////////////////////////////////////
		///异常情况
		//////////////////////////////////////////////////////////////////////////
		if (intersectAfter_s >= intersectAfter_e)
		{
			printf("异常：intersectAfter_s = %d, intersectAfter_e = %d\n", intersectAfter_s, intersectAfter_e);
		}
		for (int i = 0; i < objectCluster->polyline.size(); i++)
		{
			if (i < intersectAfter_s || i > intersectAfter_e)
				continue;
			else if (i == intersectAfter_s)
				newPolyline.push_back(intersection_s);
			else if (i == intersectAfter_e)
				newPolyline.push_back(intersection_e);
			else
				newPolyline.push_back(objectCluster->polyline[i]);
		}
	}
	objectCluster->polyline = newPolyline;
	//[TODO]:新的polyline信息未更新	
}

//dirver func
void doRefinement(vector<Cluster*>& allClusters, MapDrawer& md)
{
	for each (Cluster* cluster in allClusters)
	{
		if (cluster->polyline.size() == 0)
			continue;
		cout << "start refine " << cluster->clusterId << endl;
		refineOnePolyline(cluster, allClusters);
		drawPolyline(cluster, md, Color::Black);
	}
	pngFileName = ptsFileName + "_polyline_refine";
}


//////////////////////////////////////////////////////////////////////////
///connect
//////////////////////////////////////////////////////////////////////////
//old ver
pair<Edge*, double> getNearestEdge(GeoPoint* pt)//, GeoPoint* prePt)
{
	//////////////////////////////////////////////////////////////////////////
	///返回路网中距离pt最近的一条路段
	///或者与线段pt,prePt相交的路段
	///返回的第一个参数记录最近路段的指针，第二个参数double记录最短距离,单位为米
	///如果有路段相交则优先返回相交路段，第二个参数返回一个负值（-1）
	//////////////////////////////////////////////////////////////////////////
	
	double minDistM = 9999.0;
	Edge* nearestEdge = NULL;
	vector<Edge*> nearEdges;
	roadNetwork.getNearEdges(pt->lat, pt->lon, 1000.0, nearEdges);
	
		//////////////////////////////////////////////////////////////////////////
		///异常
		//////////////////////////////////////////////////////////////////////////
		/**********************************************************/
		/*test code starts from here*/
		if (nearEdges.size() == 0)
		{
			cout << "[异常] nearEdges's size = 0 in getCandidateCluster step!" << endl;
			system("pause");
		}
		/*test code ends*/
		/**********************************************************/
	
	//看不相交的情况
	for (int i = 0; i < nearEdges.size(); i++)
	{
		double distM = roadNetwork.distM(pt->lat, pt->lon, nearEdges[i]);
		if (distM < minDistM)
		{
			minDistM = distM;
			nearestEdge = nearEdges[i];
		}
	}

	//看相交的情况
	/*for (int i = 0; i < nearEdges.size(); i++)
	{
		vector<GeoPoint*> tempPolyline;
		for (list<GeoPoint*>::iterator ptIter = nearEdges[i]->figure->begin(); ptIter != nearEdges[i]->figure->end(); ptIter++)
		{
			tempPolyline.push_back(*ptIter);
		}
		if (intersectCheck(prePt, pt, tempPolyline))
		{
			return make_pair(nearEdges[i], -1);
		}
	}*/
	return make_pair(nearestEdge, minDistM);
}

Cluster* getCandidateCluster(vector<Cluster*>& clusters)
{
	//////////////////////////////////////////////////////////////////////////
	///返回离当前地图最接近的一个cluster
	///如果没有可以返回的cluster则返回NULL
	//////////////////////////////////////////////////////////////////////////
	cout << "开始找侯选边" << endl;

	Cluster* candidateCluster = NULL;
	double currentMinDist = 999999;
	//cal each cluster's minDist to Map
	for each (Cluster* cluster in clusters)
	{
		if (cluster->polyline.size() == 0)
			continue;
		cout << "scan cluster #" << cluster->clusterId << endl;
		GeoPoint* firstPt = cluster->polyline[0];
		GeoPoint* lastPt = cluster->polyline[cluster->polyline.size() - 1];
		//cal head dist
		double dist = getNearestEdge(firstPt).second;
		if (dist < currentMinDist)
		{
			currentMinDist = dist;
			candidateCluster = cluster;
		}
		printf("head dist = %lf\n", dist);
		//cal tail dist
		dist = getNearestEdge(lastPt).second;
		if (dist < currentMinDist)
		{
			currentMinDist = dist;
			candidateCluster = cluster;
		}
		printf("tail dist = %lf\n", dist);
	}
	return candidateCluster;
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

int extendAndSplitEdge(GeoPoint* prePt, GeoPoint* succPt, Edge* renketsusaki)
{
	//////////////////////////////////////////////////////////////////////////
	///沿prePt->succPt方向延伸,找到第一个相交的路段,返回交点生成的nodeId,onExtend为true
	///如果相交路段交在prePt<->succPt之间,则onExtend为false
	///如果threshold米内都没有满足要求的路段则返回-1
	//////////////////////////////////////////////////////////////////////////
	double rate = 1000.0;
	double x1 = prePt->lon;
	double y1 = prePt->lat;
	double x2 = succPt->lon;
	double y2 = succPt->lat;

	x1 *= rate;
	y1 *= rate;
	x2 *= rate;
	y2 *= rate;

	double A = y2 - y1;
	double B = x1 - x2;
	double C = -A * x1 - B * y1;
	
	double intersectX = -1.0;
	double intersectY = -1.0;
	Figure::iterator ptIter = renketsusaki->figure->begin(), nextPtIter = ptIter;
	nextPtIter++;
	while (nextPtIter != renketsusaki->figure->end())
	{
		GeoPoint* edgePt = *ptIter;
		GeoPoint* nextEdgePt = *nextPtIter;
		edgePt->print();
		nextEdgePt->print();
		double _x1 = edgePt->lon;
		double _y1 = edgePt->lat;
		double _x2 = nextEdgePt->lon;
		double _y2 = nextEdgePt->lat;

		_x1 *= rate;
		_x2 *= rate;
		_y1 *= rate;
		_y2 *= rate;
		
		if ((A * _x1 + B * _y1 + C) * (A * _x2 + B * _y2 + C) < 1e-4) //如果有交点
		{
			/**********************************************************/
			/*test code starts from here*/
			/*printf("A = %.8lf, B = %.8lf, C = %.8lf\n", A, B, C);
			printf("1:(%.8lf, %.8lf), 2:(%.8lf, %.8lf)\n", _x1, _y1, _x2, _y2);
			printf("%lf,%lf\n",(A * _x1 + B * _y1 + C), (A * _x2 + B * _y2 + C));*/
			/*test code ends*/
			/**********************************************************/
					
			//求交点
			double _A = _y2 - _y1;
			double _B = _x1 - _x2;
			double _C = -_A * _x1 - _B * _y1;

			intersectY = (A * _C - _A * C) / (_A * B - A * _B);
			intersectX = (-B * intersectY - C) / A;

			intersectX /= rate;
			intersectY /= rate;
			/**********************************************************/
			/*test code starts from here*/
			//printf("intersect(%.8lf, %.8lf)\n", intersectX, intersectY);
			/*test code ends*/
			/**********************************************************/
			break;
		}
		ptIter++;
		nextPtIter++;
	}
	//////////////////////////////////////////////////////////////////////////
	///异常
	//////////////////////////////////////////////////////////////////////////
	if (intersectY < 0 || intersectX < 0)
	{
		cout << "[异常]: intersectX = " << intersectX << " intersectY = " << intersectY << " in func int extendAndSplitEdge(GeoPoint* prePt, GeoPoint* succPt, Edge* renketsusaki)" << endl;
		system("pause");
	}
	int newNodeId = roadNetwork.splitEdge(renketsusaki->id, intersectY, intersectX);
	return newNodeId;
}

void addOneRoad(Cluster* cluster)
{
	vector<GeoPoint*>& polyline = cluster->polyline;
	double intersectionThresM = 70;
	double splitRoadThresM = 70;
	GeoPoint* firstPt = polyline[0];
	GeoPoint* lastPt = polyline[polyline.size() - 1];
	
	//create figure
	Figure* newFigure = new Figure();
	for (int i = 0; i < polyline.size(); i++)
	{
		GeoPoint* pt = polyline[i];
		newFigure->push_back(pt);
	}
	
	//connectivity of the head
	pair<Edge*, double> tempPair = getNearestEdge(firstPt);
	Edge* renketsusaki = tempPair.first;
	double distToRenketsusaki = tempPair.second; //记录head到连结先的最短距离
	double distToRenketsusaki_s = GeoPoint::distM(firstPt, renketsusaki->figure->front()); //记录head到连结先的头部距离
	double distToRenketsusaki_e = GeoPoint::distM(firstPt, renketsusaki->figure->back()); //记录head到连结先的尾部距离
	double distToRenketsusaki_Endpoint = distToRenketsusaki_s < distToRenketsusaki_e ? distToRenketsusaki_s : distToRenketsusaki_e; //取两个距离里面小的那个
	
	/**********************************************************/
	/*test code starts from here*/
	cout << "开始加入head" << endl;
	/*test code ends*/
	/**********************************************************/
	
	int newStartNodeId = -1;
	if (distToRenketsusaki < splitRoadThresM)
	{
		if ( abs(distToRenketsusaki - distToRenketsusaki_Endpoint) < 10 ) //最短距离与到端点距离相差不大就直接连端点
		{
			if (distToRenketsusaki_s < distToRenketsusaki_e)
				newStartNodeId = renketsusaki->startNodeId;
			else
				newStartNodeId = renketsusaki->endNodeId;
			newFigure->push_front(roadNetwork.nodes[newStartNodeId]);
		}
		else //extend
		{
			/**********************************************************/
			/*test code starts from here*/
			cout << "do head Entend" << endl;
			/*test code ends*/
			/**********************************************************/
			Figure::iterator firstSuccPtIter = newFigure->begin();
			firstSuccPtIter++;
			GeoPoint* firstSuccPt = *firstSuccPtIter; //newFigure第一个点的后继点
			bool onExtend;
			firstSuccPt->print();
			firstPt->print();
			int newNodeId = extendAndSplitEdge(firstSuccPt, firstPt, renketsusaki);

			/**********************************************************/
			/*test code starts from here*/
			cout << "entend ok. new startNodeId = " << newStartNodeId << endl;
			/*test code ends*/
			/**********************************************************/
			newFigure->push_front(roadNetwork.nodes[newNodeId]);
			newStartNodeId = newNodeId;
			//newStartNodeId = roadNetwork.insertNode(newFigure->front()->lat, newFigure->front()->lon);
		}
	}
	else
	{
		newStartNodeId = roadNetwork.insertNode(newFigure->front()->lat, newFigure->front()->lon);
	}
	/**********************************************************/
	/*test code starts from here*/
	cout << "newStartNodeId = " << newStartNodeId << endl;
	/*test code ends*/
	/**********************************************************/
	
	/**********************************************************/
	/*test code starts from here*/
	cout << "开始加入tail" << endl;
	/*test code ends*/
	/**********************************************************/

	//connectivity of the tail
	tempPair = getNearestEdge(lastPt);
	renketsusaki = tempPair.first;
	distToRenketsusaki = tempPair.second;
	distToRenketsusaki_s = GeoPoint::distM(lastPt, renketsusaki->figure->front());
	distToRenketsusaki_e = GeoPoint::distM(lastPt, renketsusaki->figure->back());
	distToRenketsusaki_Endpoint = distToRenketsusaki_s < distToRenketsusaki_e ? distToRenketsusaki_s : distToRenketsusaki_e;
	int newEndNodeId = -1;
	if (distToRenketsusaki < splitRoadThresM)
	{
		if (abs(distToRenketsusaki - distToRenketsusaki_Endpoint) < 10) //连端点
		{
			if (distToRenketsusaki_s < distToRenketsusaki_e)
				newEndNodeId = renketsusaki->startNodeId;
			else
				newEndNodeId = renketsusaki->endNodeId;
			newFigure->push_back(roadNetwork.nodes[newEndNodeId]);
			md.drawLine(Color::Red, lastPt->lat, lastPt->lon, roadNetwork.nodes[newEndNodeId]->lat, roadNetwork.nodes[newEndNodeId]->lon);
		}
		else //extend
		{
			/**********************************************************/
			/*test code starts from here*/
			cout << "do head Entend" << endl;
			/*test code ends*/
			/**********************************************************/
			Figure::iterator lastPrePtIter = newFigure->end();
			lastPrePtIter--; lastPrePtIter--;
			GeoPoint* lastPrePt = *lastPrePtIter; //newFigure最后一个点的前继点
			bool onExtend;
			int newNodeId = extendAndSplitEdge(lastPrePt, lastPt, renketsusaki);
			/**********************************************************/
			/*test code starts from here*/
			cout << "entend ok. new startNodeId = " << newStartNodeId << endl;
			/*test code ends*/
			/**********************************************************/
			newFigure->push_back(roadNetwork.nodes[newNodeId]);
			newEndNodeId = newNodeId;
			//newEndNodeId = roadNetwork.insertNode(newFigure->back()->lat, newFigure->back()->lon);
		}
	}
	else
	{
		newEndNodeId = roadNetwork.insertNode(newFigure->back()->lat, newFigure->back()->lon);
	}


	/**********************************************************/
	/*test code starts from here*/
	cout << "newEndNodeId = " << newEndNodeId << endl;
	//md.drawBoldLine(Color::Red, lastPt->lat, lastPt->lon, roadNetwork.nodes[newEndNodeId]->lat, roadNetwork.nodes[newEndNodeId]->lon);
	/*test code ends*/
	/**********************************************************/

	//构造双向路
	Figure* newFigureReverse = new Figure();
	for (Figure::iterator iter = newFigure->begin(); iter != newFigure->end(); iter++)
	{
		newFigureReverse->push_front(*iter);
	}
	int newEdgeid = roadNetwork.insertEdge(newFigure, newStartNodeId, newEndNodeId);
	int newEdgeidR = roadNetwork.insertEdge(newFigureReverse, newEndNodeId, newStartNodeId);

	
}

//new ver
void extend(Cluster* cluster)
{
	vector<GeoPoint*>& polyline = cluster->polyline;
	double intersectionThresM = 70;
	double splitRoadThresM = 70;
	GeoPoint* firstPt = polyline[0];
	GeoPoint* lastPt = polyline[polyline.size() - 1];

	//create figure
	Figure* newFigure = new Figure();
	for (int i = 0; i < polyline.size(); i++)
	{
		GeoPoint* pt = polyline[i];
		newFigure->push_back(pt);
	}
	//extend head
		//直接连intersection

		//延长相交

	//extend tail
		//直接连intersection
		//延长相交
}

void addOneRoadEx(Cluster* cluster)
{

}

//driver func
void addAllPolylines(vector<Cluster*>& allClusters)
{
	while (1)
	{
		Cluster* candidateCluster = getCandidateCluster(allClusters);

		if (candidateCluster == NULL)
			break;
		
		/**********************************************************/
		/*test code starts from here*/
		if (candidateCluster->clusterId == 44)
		{
			candidateCluster->polyline.clear();
			continue;
		}
		printf("开始加入cluster %d\n", candidateCluster->clusterId);
		/*test code ends*/
		/**********************************************************/
		addOneRoad(candidateCluster);
		candidateCluster->polyline.clear();
		system("pause");
	}
	pngFileName = ptsFileName + "_polyline_refine_added";
}
//***********************************************************************************************************

//////////////////////////////////////////////////////////////////////////
///expTest
//////////////////////////////////////////////////////////////////////////

void deleteEdge()
{
	roadNetwork.setArea(&area);
	roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 50);
	ExpGenerator eg;
	eg.deleteForGeo();

	md.setArea(&area);
	md.setResolution(5000);
	md.newBitmap();
	md.lockBits();

	roadNetwork.drawMap(Color::Black, md);
	roadNetwork.drawDeletedEdges(Gdiplus::Color::Red, md);
	md.unlockBits();
	md.saveBitmap("deleteEdge.png");
	system("pause");
	exit(0);
}

void expTest()
{
	//////////////////////////////////////////////////////////////////////////
	///生成两次匹配后的轨迹数据
	//////////////////////////////////////////////////////////////////////////
	roadNetwork.setArea(&area);
	roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 50);
	originalRoadNetwork.setArea(&area);
	originalRoadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 50);
	roadNetwork.deleteEdges("D:\\trajectory\\singapore_data\\experiments\\big area\\geo\\area3\\deletedEdges.txt");

	ExpGenerator eg;
	
	eg.inputFileNames.push_back("logs_20120207_20120208.txt");
	eg.inputFileNames.push_back("logs_20120208_20120209.txt");
	eg.inputFileNames.push_back("logs_20120209_20120210.txt");
	eg.inputFileNames.push_back("logs_20120210_20120211.txt");
	eg.inputFileNames.push_back("logs_20120211_20120212.txt");
	eg.inputFileNames.push_back("logs_20120212_20120213.txt");
	eg.inputFileNames.push_back("logs_20120215_20120216.txt");
	eg.inputFileNames.push_back("logs_20120216_20120217.txt");
	eg.inputFileNames.push_back("logs_20120217_20120218.txt");
	eg.inputFileNames.push_back("logs_20120218_20120219.txt");
	eg.setArea(&area);

	//eg.genExpData();
	//system("pause");
	//exit(0);
	/**********************************************************/
	/*test code starts from here*/
	TrajReader tr("D:\\trajectory\\singapore_data\\experiments\\big area\\geo\\area3\\newMMTrajs.txt");
	list<Traj*> trajs;
	tr.readTrajs(trajs);// , 50000);
	/*test code ends*/
	/**********************************************************/

	
	//draw
	md.setArea(&area);
	md.setResolution(5000);
	md.newBitmap();
	md.lockBits();

	roadNetwork.drawMap(Color::Black, md);
	roadNetwork.drawDeletedEdges(Gdiplus::Color::Red, md);
	TrajDrawer::drawMMTrajs(trajs, md, Color::Red, false, false , false, true);
	md.unlockBits();
	md.saveBitmap("expTest.png");
	system("pause");
	exit(0);
}

void geoValidation(vector<Cluster*>& allClusters, MapDrawer& md)
{
	cout << "start geo-validation " << endl;
	GeoValidation gv(roadNetwork);
	vector<Figure*> figures;
	for each (Cluster* cluster in allClusters)
	{
		if (cluster->polyline.size() == 0)
			continue;
		Figure* newFigure = new Figure();
		for (int i = 0; i < cluster->polyline.size(); i++)
		{
			GeoPoint* pt = cluster->polyline[i];
			newFigure->push_back(pt);
		}
		figures.push_back(newFigure);
	}
	gv.validate(figures, md);
}

void testTCC()
{
	TrajReader tr("D:\\trajectory\\singapore_data\\experiments\\3\\wy_MMTrajs.txt");
	md.setArea(&area);
	md.setResolution(15000);
	vector<Traj*> trajs;
	tr.readTrajs(trajs,10000);
	md.newBitmap();
	md.lockBits();
	TrajDrawer::drawTrajs(trajs, md, Color::Red, false, true);
	md.unlockBits();
	md.saveBitmap("tcc.png");
}

void main()
{
	int startTime = clock();
	srand((unsigned)time(NULL));
	/**********************************************************/
	/*test code starts from here*/
	//deleteEdge();
	expTest();
	/*test code ends*/
	/**********************************************************/
	

	testSampleId = 11;
	workspaceFolder = "D:\\trajectory\\singapore_data\\experiments\\unmatched pts\\";
	
	//ptsFileName = "cluster " + StringOperator::intToString(testSampleId) + "";
	//ptsFileName = "cluster " + StringOperator::intToString(testSampleId) + "_without_noise";
	//ptsFileName = "cluster " + StringOperator::intToString(testSampleId) + "_without_noise_dir";
	ptsFileName = "cluster " + StringOperator::intToString(testSampleId) + "_without_noise_dir_cluster";
	pngFileName = "out"; 

	roadNetwork.setArea(&area);
	roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 50);
	originalRoadNetwork.setArea(&area);
	originalRoadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 50);
	//roadNetwork.deleteEdges("D:\\trajectory\\singapore_data\\experiments\\3\\deletedEdges.txt");
	ExpGenerator eg;
	eg.deleteForGeo();


	list<GeoPoint*> pts;
	vector<Cluster*> clusters;
	md.setArea(&area);
	md.setResolution(5000);
	
	//dump in
	//dumpInPts(workspaceFolder + ptsFileName + ".txt", pts); //废弃，不用
	//dumpInPtsEx(workspaceFolder + ptsFileName + ".txt", pts);
	//dumpInPtsWithCluster(workspaceFolder + ptsFileName + ".txt", pts, clusters);
	//ptIndex.createIndex(pts, &area, 300);
	
	//detect outlier
	//outlierDetecting();
	//dumpPtsWithoutNoise(pts);
	
	//cal dir
	//md.setResolution(15000);
	//calPtsDirs(pts);
	//dumpPtsWithDir(pts);


	//do dir cluster
	//doDirCluster(pts, clusters);
	//dumpPtsWithCluster(clusters);


	//drawing part
	md.newBitmap();
	md.lockBits();
	ptIndex.drawGridLine(Gdiplus::Color::Green, md);
	
	//drawPts(md, pts, Color::Blue);
	//drawPtsWithoutOutliers(md, Color::Red);
	//drawPtsDir(pts, md);
	//drawPtsWithCluster(clusters, md);
	//genAllPolyLines(clusters, md);
	//doRefinement(clusters, md);
	//addAllPolylines(clusters);
	//geoValidation(clusters, md);
	roadNetwork.drawMap(Color::Blue, md);
	roadNetwork.drawDeletedEdges(Gdiplus::Color::Red, md);
	md.unlockBits();
	//md.saveBitmap(pngFileName + ".png");
	md.saveBitmap("deleteGeo.png");
	cout << "output to " << pngFileName << ".png" << endl;
	system("pause");
	exit(0);
}