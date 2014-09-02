#include <iostream>
#include "GeoPoint.h"
#include "Map.h"
#include "MapDrawer.h"
using namespace std;
using namespace Gdiplus;

void printFigure(Figure* figure)
{
	for each (GeoPoint* pt in *figure)
	{
		printf("(%lf, %lf),", pt->lat, pt->lon);
	}
	cout << endl;
}

void main()
{	
	//Area* area = new Area(1.22, 1.5, 103.620, 104.0); //singapore half
	Area* area = new Area(39.309978, 40.380323, 115.743800, 117.076883); //beijin half
	//Map map("D:\\trajectory\\singapore_data\\singapore_map\\border\\", area);
	Map map("D:\\trajectory\\beijing_data\\beijing_map\\border\\", area);
	//map.getMinMaxLatLon("D:\\trajectory\\beijing_data\\beijing_map\\nodeOSM.txt");
	/*for (int i = 0; i < map.edges.size(); i++)
	{
		int reverseId = map.hasEdge(map.edges[i]->endNodeId, map.edges[i]->startNodeId);
		if ( reverseId != -1)
		{
			printf("edgeid = %d, reverseId = %d, startNode = %d, endNode = %d\n", map.edges[i]->id, reverseId, map.edges[i]->startNodeId, map.edges[i]->endNodeId);
			printFigure(map.edges[i]->figure);
			system("pause");
		}

	}*/
	MapDrawer md;
	md.setArea(area);
	md.setResolution(1000);
	md.newBitmap();
	md.lockBits();
	//md.drawMap(Color::Black, "D:\\trajectory\\singapore_data\\singapore_map\\new\\edgeOSM.txt");
	map.drawMap(Color::Black, md);
	md.unlockBits();
	md.saveBitmap("border_beijing.png");
	system("pause");
}

