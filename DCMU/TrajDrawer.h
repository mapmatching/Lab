/*
* Last Updated at [2014/4/3 11:34] by wuhao
*/
#pragma once
#include "MapDrawer.h"
#include <iostream>
#include "GeoPoint.h"
#include <list>
#include <vector>
using namespace std;

typedef list<GeoPoint*> Traj;
class TrajDrawer
{
public:
	//////////////////////////////////////////////////////////////////////////
	///画一般的轨迹的工具方法
	///boolLine: 轨迹是否用粗线画，默认为false
	///drawLines: 是否画出轨迹线条，默认为true
	///bigPoint: 轨迹点是否是十字点，默认为true
	///drawTrajPt: 轨迹是否画出轨迹点，默认为true,轨迹点颜色为黑色
	//////////////////////////////////////////////////////////////////////////
	static void drawOneTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);

	//////////////////////////////////////////////////////////////////////////
	///专用画匹配后的轨迹
	///默认匹配成功的点为绿点，失败的点为红点
	//////////////////////////////////////////////////////////////////////////
	static void drawOneMMTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawMMTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawMMTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);

};

void TrajDrawer::drawOneTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	if (traj == NULL)
		return;
	Traj::iterator ptIter = traj->begin(), nextPtIter = ptIter;
	if (drawTrajPts) //画起点
	{
		if (bigPoint)
			md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
		else
			md.drawPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
	}
	nextPtIter++;
	while (1)
	{
		if (nextPtIter == traj->end())
			break;
		if (drawLines)
		{
			if (boldLine)
				md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			else
				md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
		}
		if (drawTrajPts)
		{
			if (bigPoint)
				md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
			else
				md.drawPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
		}

		ptIter++;
		nextPtIter++;
	}
	if (drawTrajPts)//画终点
	{
		if (bigPoint)
			md.drawBigPoint(Gdiplus::Color::Black, traj->back()->lat, traj->back()->lon);
		else
			md.drawPoint(Gdiplus::Color::Black, traj->back()->lat, traj->back()->lon);
	}
}

void TrajDrawer::drawTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	for (vector<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		drawOneTraj((*trajIter), md, color, boldLine, drawLines, drawTrajPts);
	}
}

void TrajDrawer::drawTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		drawOneTraj((*trajIter), md, color, boldLine, drawLines, drawTrajPts);
	}
}

void TrajDrawer::drawOneMMTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	if (traj == NULL)
		return;
	Traj::iterator ptIter = traj->begin(), nextPtIter = ptIter;
	if (drawTrajPts) //画起点
	{
		Gdiplus::Color ptColor = (*ptIter)->mmRoadId == -1 ? Gdiplus::Color::Red : Gdiplus::Color::Green;
		if (bigPoint)
			md.drawBigPoint(ptColor, (*ptIter)->lat, (*ptIter)->lon);
		else
			md.drawPoint(ptColor, (*ptIter)->lat, (*ptIter)->lon);
	}
	nextPtIter++;
	while (1)
	{
		if (nextPtIter == traj->end())
			break;
		if (drawLines)
		{
			if (boldLine)
				md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			else
				md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
		}
		if (drawTrajPts)
		{
			Gdiplus::Color ptColor = (*ptIter)->mmRoadId == -1 ? Gdiplus::Color::Red : Gdiplus::Color::Green;
			if (bigPoint)
				md.drawBigPoint(ptColor, (*ptIter)->lat, (*ptIter)->lon);
			else
				md.drawPoint(ptColor, (*ptIter)->lat, (*ptIter)->lon);
		}

		ptIter++;
		nextPtIter++;
	}
	if (drawTrajPts)//画终点
	{
		Gdiplus::Color ptColor = (*ptIter)->mmRoadId == -1 ? Gdiplus::Color::Red : Gdiplus::Color::Green;
		if (bigPoint)
			md.drawBigPoint(ptColor, traj->back()->lat, traj->back()->lon);
		else
			md.drawPoint(ptColor, traj->back()->lat, traj->back()->lon);
	}
}

void TrajDrawer::drawMMTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	for (vector<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		drawOneMMTraj((*trajIter), md, color, boldLine, drawLines, drawTrajPts);
	}
}

void TrajDrawer::drawMMTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		drawOneMMTraj((*trajIter), md, color, boldLine, drawLines, drawTrajPts);
	}
}
