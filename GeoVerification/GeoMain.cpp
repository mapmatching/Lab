#include <iostream>
#include "GeoPoint.h"
#include "GeoVerification.h"
#include "MapDrawer.h"
#include "Map.h"
#include "TrajReader.h"
#include "TrajDrawer.h"
using namespace std;

Map roadNetwork;
Area area(1.294788, 1.393593, 103.784667, 103.906266); //big
MapDrawer md;
vector<Figure*> figures;

void initialization()
{
	//////////////////////////////////////////////////////////////////////////
	///初始化地图，删路
	///读入生成的道路
	//////////////////////////////////////////////////////////////////////////
	
	//初始化地图
	roadNetwork.setArea(&area);
	roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 50);
	roadNetwork.deleteEdges("D:\\trajectory\\singapore_data\\experiments\\big area\\newDeletedEdges.txt");

	//读路
	//TrajReader tReader("roads_MI.txt");
	//TrajReader tReader("roads_bones.txt");
	//TrajReader tReader("D:\\trajectory\\singapore_data\\experiments\\big area\\roads_wy2.txt");
	TrajReader tReader("D:\\trajectory\\singapore_data\\experiments\\big area\\roads_DCMU_new.txt");
	tReader.readTrajs(figures);

	//初始化画板
	md.setArea(&area);
	md.setResolution(5000);
	md.newBitmap();
}

void test()
{
	initialization();
	GeoVerification gv;
	md.lockBits();
	TrajDrawer::drawTrajs(figures, md, Gdiplus::Color::Blue);
	gv.verificate(figures, md);	
	md.unlockBits();
	md.saveBitmap("wy.png");
}


void main()
{
	test();
	system("pause");
}