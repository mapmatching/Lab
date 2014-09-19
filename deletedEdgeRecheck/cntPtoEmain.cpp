#include <iostream>
#include <fstream>
#include <iomanip>
#include "GeoPoint.h"
#include "MapDrawer.h"
#include "Map.h"
#include "PointGridIndex.h"
#include "getDelEdgeFile.h"
#include "TrajReader.h"
using namespace std;
using namespace Gdiplus;

Map roadNetwork;
//Area area(1.294788, 1.393593, 103.784667, 103.906266); //big
Area area(1.343593, 1.442398, 103.784667, 103.906266); //big2
//Area area(1.294788, 1.393593, 103.704667, 103.826266); //big3

MapDrawer md;
vector<Figure*> figures;

string workspaceFolder = "D:\\trajectory\\singapore_data\\experiments\\big area\\geo\\area2\\";
string inputDeletedEdgesPath = workspaceFolder + "deletedEdges.txt";

void initialization()
{
	//////////////////////////////////////////////////////////////////////////
	///初始化地图，删路
	///读入生成的道路
	//////////////////////////////////////////////////////////////////////////
	//初始化地图
	roadNetwork.setArea(&area);
	roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 150);
	//roadNetwork.deleteEdges(filePathIn + "deletedEdges.txt");
}


//返回值：返回满足要求的边
list<GeoPoint*> redPoint; //存放需要匹配的点的链表
vector<int> LargeDelEdge;
vector<int> otherDelEdge;
void dealwithDeleteEdge(int gridwith, double thresholdM, int thresholdI)
{
	ifstream EdgeIfs, PointIfs;
	double lat, lon;
	int edgeId;

	PointGridIndex pGridIndex;
	
	//打开需要判断的点文件
	TrajReader tr(workspaceFolder + "newMMTrajs_unmatched.txt");
	list<Traj*> trajs;
	tr.readTrajs(trajs);
	list<GeoPoint*> allPts;
	for each(Traj* traj in trajs)
	{
		for each (GeoPoint* pt in *traj)
		{
			redPoint.push_back(pt);
		}
	}
	pGridIndex.createIndex(redPoint, &area, gridwith);

	//打开记录红色路段的文件
	EdgeIfs.open(inputDeletedEdgesPath);
	if (!EdgeIfs){
		cout << "can not open deleteEdges.txt." << endl;
		system("pause");
		exit(0);
	}
	list<GeoPoint*> dest; //存放需要索引的网格内的点
	
	LargeDelEdge.clear();
	otherDelEdge.clear();

	//对边进行遍历
	int icount = 0;
	while (EdgeIfs){
		//cout << "haaaaaaaaaaaaaaaaaaaaaaaaa " << icount++ << " hhahhahahhhhhhhhhhhhhhhhhhhhhhhhhhhhh" << endl;
		EdgeIfs >> edgeId;

		Edge *edge = roadNetwork.edges[edgeId];
		list<GeoPoint*> *figure = edge->figure;

		//以路段为中心得到一个矩形框，接着对矩形框内的所有点进行遍历
		pair<int, int> minIndex;
		pair<int, int> maxIndex;
		double maxlat=0, maxlon=0, minlat=1000, minlon=1000;

		GeoPoint *pmax = new GeoPoint(maxlat, maxlon), *pmin = new GeoPoint(minlat, minlon);
		for (list<GeoPoint*>::iterator iter = figure->begin(); iter != figure->end(); iter++){
			if (pmax->lat < (*iter)->lat) pmax->lat = (*iter)->lat;
			if (pmax->lon < (*iter)->lon) pmax->lon = (*iter)->lon;
			if (pmin->lat > (*iter)->lat) pmin->lat = (*iter)->lat;
			if (pmin->lon > (*iter)->lon) pmin->lon = (*iter)->lon;
		}
		maxIndex = pGridIndex.getRowCol(pmax);
		minIndex = pGridIndex.getRowCol(pmin);

		maxIndex.first = (maxIndex.first + 1 < pGridIndex.gridHeight) ? maxIndex.first + 1 : pGridIndex.gridHeight - 1;
		maxIndex.second = (maxIndex.second + 1 < pGridIndex.gridWidth) ? maxIndex.second + 1 : pGridIndex.gridWidth - 1;
		minIndex.first = (minIndex.first - 1 >= 0) ? minIndex.first - 1 : 0;
		minIndex.second = (minIndex.second - 1 >= 0) ? minIndex.second - 1 : 0;

		//处理矩形框内的点
		dest.clear();
		int maxSize = 50000;

		for (int row = minIndex.first; row < maxIndex.first; ++row){
			for (int col = minIndex.second; col < maxIndex.second; ++col){
				for (list<GeoPoint*>::iterator iter = pGridIndex.grid[row][col]->begin(); iter != pGridIndex.grid[row][col]->end(); iter++){
					dest.push_back((*iter));
					if (dest.size()>maxSize) break;
				}
				if (dest.size()>maxSize) break;
			}
			if (dest.size()>maxSize) break;
		}

		if (dest.size() > maxSize) {
			LargeDelEdge.push_back(edgeId);
		}
		else{
			int RNcount = 0;
			for (list<GeoPoint*>::iterator iter = dest.begin(); iter != dest.end(); iter++){
				if (roadNetwork.distM((*iter)->lat, (*iter)->lon, edge) <= thresholdM) RNcount++;
				if (RNcount > thresholdI) break;
			}
			if (RNcount <= thresholdI || roadNetwork.edges[edgeId]->lengthM > 4000){// * roadNetwork.edges[edgeId]->lengthM) {
				otherDelEdge.push_back(edgeId);
			}
			else {
				LargeDelEdge.push_back(edgeId);
			}
		}
	}

	//输出
	ofstream ofs("newDeletedEdges.txt");
	for (vector<int>::iterator iter = LargeDelEdge.begin(); iter != LargeDelEdge.end(); iter++)
	{
		ofs << *iter << endl;
	}
}

void draw()
{
	md.setArea(&area);
	md.setResolution(5000);
	md.newBitmap();
	md.lockBits();

	for (list<GeoPoint*>::iterator iter = redPoint.begin(); iter != redPoint.end(); iter++){
		md.drawPoint(Color::Blue, (*iter)->lat, (*iter)->lon);
	}

	cout <<"large:"<< LargeDelEdge.size() <<" less:"<<otherDelEdge.size()<< endl;
	for (vector<int>::iterator iter = LargeDelEdge.begin(); iter != LargeDelEdge.end(); iter++){
		Edge *delEdge = roadNetwork.edges[(*iter)];
		Figure *figure = delEdge->figure;
		list<GeoPoint*>::iterator iter2 = figure->begin(), iter1 = figure->begin();
		iter2++;
		for (; iter2 != figure->end(); iter1++, iter2++){
			md.drawBoldLine(Color::Red, (*iter1)->lat, (*iter1)->lon, (*iter2)->lat, (*iter2)->lon);
		}
	}
	for (vector<int>::iterator iter = otherDelEdge.begin(); iter != otherDelEdge.end(); iter++){
		Edge *delEdge = roadNetwork.edges[(*iter)];
		Figure *figure = delEdge->figure;
		list<GeoPoint*>::iterator iter2 = figure->begin(), iter1 = figure->begin();
		iter2++;
		for (; iter2 != figure->end(); iter1++, iter2++){
			md.drawBoldLine(Color::Yellow, (*iter1)->lat, (*iter1)->lon, (*iter2)->lat, (*iter2)->lon);
		}
	}


	md.unlockBits();
	md.saveBitmap("c.png");
}

void test()
{
	initialization();
	//参数
	int thresholdI = 200; //密度阈值为1点/m
	double thresholdM = 30;
	int gridwith = 500; 

	dealwithDeleteEdge(gridwith,thresholdM, thresholdI);
	draw();
	system("pause");
}

void main()
{
	test();
	//DelEdgeFile deleteFile(dataPath,"in\\newMMTrajs.txt");

}