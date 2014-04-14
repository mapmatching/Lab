/* 
 * Last Updated at [2014/4/3 11:33] by wuhao
 */
#include <iostream>
#include <time.h>
#include "BaiduMap.h"
#include "MapDrawer.h"
#include "BaiduTrajReader.h"
#include "TrajDrawer.h"
using namespace std;

double minLat = 31.853;
double maxLat = 31.893;
double minLon = 117.250;
double maxLon = 117.336;

MapDrawer md;
BaiduMap bMap;
BaiduTrajReader bTrajReader("D:\\trajectory\\baiduMap\\LBS_DATASET\\traj_hefei.txt");

vector<Traj*> trajs;

void main()
{
	srand((unsigned)time(NULL));
	bTrajReader.readTrajs(trajs, 5000);
	
	md.setArea(minLat, maxLat, minLon, maxLon);
	md.setResolution(10000);
	printf("resolution: %d * %d\n", md.r_width, md.r_height);
	bMap.setArea(md);
	bMap.open("D:\\trajectory\\baiduMap\\LBS_DATASET\\", 150);
	md.newBitmap();
	md.lockBits();
	bMap.drawMap(Color::Blue, md);
	bMap.drawGridLine(Color::Green, md);
	TrajDrawer::drawTrajs(trajs, md, Color::Red);
	md.unlockBits();
	md.saveBitmap("test.png");
	system("pause");
}

