/* 
 * Last Updated at [2014/9/3 21:22] by wuhao
 */
#include "GeoVerification.h"
extern Map roadNetwork;
extern Map originalRoadNetwork;

GeoVerification::GeoVerification(Map& roadNetwork)
{
	//this->roadNetwork = roadNetwork;
}

void GeoVerification::verificate(vector<Figure*>& genFigures, MapDrawer& md)
{
	//计算totalDelLength
	for (int i = 0; i < roadNetwork.deletedEdges.size(); i++)
	{
		totalDelLengthM += roadNetwork.deletedEdges[i]->lengthM;
	}

	//计算totalGenLength
	for (int i = 0; i < genFigures.size(); i++)
	{
		Figure::iterator ptIter = genFigures[i]->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (1)
		{
			if (nextPtIter == genFigures[i]->end())
				break;
			totalGenLengthM += GeoPoint::distM((*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			ptIter++;
			nextPtIter++;
		}
	}

	//删除掉双向路的一边
	//[注意]deletedEdges必须保证双向路在数组中的存放顺序一定是连续的
	int index = 0;
	while (1)
	{
		if (index < roadNetwork.deletedEdges.size() - 1)
		{
			if ((roadNetwork.deletedEdges[index]->startNodeId == roadNetwork.deletedEdges[index + 1]->endNodeId)
				&& (roadNetwork.deletedEdges[index]->endNodeId == roadNetwork.deletedEdges[index + 1]->startNodeId)) //当前路是双向的
			{
				delEdges_oneway.push_back(roadNetwork.deletedEdges[index]);
				index += 2;
			}
			else //当前路是单向的
			{
				delEdges_oneway.push_back(roadNetwork.deletedEdges[index]);
				index += 1;
			}
		}
		else if (index > roadNetwork.deletedEdges.size() - 1)
		{
			break;
		}
		else
		{
			delEdges_oneway.push_back(roadNetwork.deletedEdges[index]);
			break;
		}		
	}
	
	/**********************************************************/
	/*test code starts from here*/
	printf("delEdge size = %d, delEdge_oneway.size = %d\n", roadNetwork.deletedEdges.size(), delEdges_oneway.size());
	/*test code ends*/
	/**********************************************************/
	
	clipEdges(delEdges_oneway); //将删除的路(留下单向的部分)切割开来存入segments向量中

	for (int i = 0; i < segments.size(); i++)
	{
		if (verificateOneSegment(segments[i], genFigures))
		{
			/**********************************************************/
			/*test code starts from here*/
			md.drawLine(Gdiplus::Color::Green, segments[i].first.lat, segments[i].first.lon, segments[i].second.lat, segments[i].second.lon);
			//md.drawBigPoint(Gdiplus::Color::Black, segments[i].second.lat, segments[i].second.lon);
			//md.drawBigPoint(Gdiplus::Color::Aqua, segments[i].second.lat, segments[i].second.lon);
			/*test code ends*/
			/**********************************************************/
		}
		else
		{
			md.drawLine(Gdiplus::Color::Red, segments[i].first.lat, segments[i].first.lon, segments[i].second.lat, segments[i].second.lon);
		}
	}
	//map inference 25m补正
	//correctLengthM += 25 * 20 * 2;
	//totalGenLengthM += 25 * 20 * 2;
	//for wang yin
	totalDelLengthM /= 2;
	cout << "totalDelLength = " << totalDelLengthM << endl;
	cout << "totalGenLength = " << totalGenLengthM << endl;
	cout << "correctLength = " << correctLengthM << endl;
	cout << "recall = " << correctLengthM / totalDelLengthM << endl;
	cout << "precision = " << correctLengthM / totalGenLengthM << endl;
}


void GeoVerification::clipEdges(vector<Edge*>& delEdges)
{
	//////////////////////////////////////////////////////////////////////////
	///将被删除的道路切成一段段线段存入segments，每段长度不会超过clipThresM
	//////////////////////////////////////////////////////////////////////////
	//对每一条路
	for (int i = 0; i < delEdges.size(); i++)
	{
		Figure* fig = delEdges[i]->figure;
		Figure::iterator currentIter = fig->begin();
		Figure::iterator nextIter = fig->begin(); nextIter++;
		GeoPoint fromPt = *(*currentIter);
		GeoPoint toPt;
		//对每一段
		while (nextIter != fig->end())
		{
			if (GeoPoint::distM(&fromPt, *nextIter) < clipThresM + 1e-2)
			{
				toPt = *(*nextIter);
				currentIter++;
				nextIter++;
			}
			else
			{
				toPt.lon = fromPt.lon + ((*nextIter)->lon - fromPt.lon) * clipThresM / GeoPoint::distM(&fromPt, *nextIter);
				toPt.lat = fromPt.lat + ((*nextIter)->lat - fromPt.lat) * clipThresM / GeoPoint::distM(&fromPt, *nextIter);
			}
			segments.push_back(make_pair(fromPt, toPt));
			fromPt = toPt;
		}
	}
}

bool GeoVerification::verificateOneSegment(Segment segment, vector<Figure*>& genFigures)
{
	//////////////////////////////////////////////////////////////////////////
	///检测一段segment是否能够匹配到某条路上
	///存在两条edge[注意]是两条，因为我现在把每个segment代表的是双向路上的，使得segment的两个端点到edge的距离都小于thresholdM，则认为segment是匹配成功的
	///同时更新correctLengthM
	//////////////////////////////////////////////////////////////////////////
	
	GeoPoint fromPt = segment.first;
	GeoPoint toPt = segment.second;
	int correctEdgeCount = 0;
	for (int i = 0; i < genFigures.size(); i++)
	{
		Edge tempEdge;
		tempEdge.figure = genFigures[i];
		if (roadNetwork.distM(fromPt.lat, fromPt.lon, &tempEdge) < thresholdM &&
			roadNetwork.distM(toPt.lat, toPt.lon, &tempEdge) < thresholdM)
		{
			correctEdgeCount++;
			correctLengthM += GeoPoint::distM(fromPt, toPt);
		}
		if (correctEdgeCount == 1)
		{
			return true;
		}		
	}
	if (correctEdgeCount == 1)
	{
		return true;
	}	
	return false;
}
