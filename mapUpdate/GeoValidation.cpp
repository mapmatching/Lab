#include "GeoValidation.h"
extern Map roadNetwork;
extern Map originalRoadNetwork;

GeoValidation::GeoValidation(Map& roadNetwork)
{
	//this->roadNetwork = roadNetwork;
}

void GeoValidation::validate(vector<Figure*>& figures, MapDrawer& md)
{
	clipEdges(figures);
	for (int i = 0; i < segments.size(); i++)
	{
		if (validateOneSegment(segments[i]))
		{

			/**********************************************************/
			/*test code starts from here*/
			md.drawLine(Gdiplus::Color::Red, segments[i].first.lat, segments[i].first.lon, segments[i].second.lat, segments[i].second.lon);
			md.drawBigPoint(Gdiplus::Color::Aqua, segments[i].second.lat, segments[i].second.lon);
			md.drawBigPoint(Gdiplus::Color::Aqua, segments[i].second.lat, segments[i].second.lon);
			/*test code ends*/
			/**********************************************************/
		}
	}
	cout << "totalLength = " << totalLengthM << endl;
	cout << "correctLength = " << correctLengthM << endl;
}


void GeoValidation::clipEdges(vector<Figure*>& figures)
{
	//////////////////////////////////////////////////////////////////////////
	///将生成的道路切成一段段线段存入segments，每段长度不会超过clipThresM
	//////////////////////////////////////////////////////////////////////////
	//对每一条路
	for (int i = 0; i < figures.size(); i++)
	{
		Figure* fig = figures[i];
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

bool GeoValidation::validateOneSegment(Segment segment)
{
	GeoPoint fromPt = segment.first;
	GeoPoint toPt = segment.second;
	vector<Edge*> candidateEdges;
	candidateEdges = roadNetwork.deletedEdges;
	//roadNetwork.getNearEdges(fromPt.lat, fromPt.lon, 50.0, nearEdges);
	//cout << candidateEdges.size() << endl;
	//system("pause");
	totalLengthM += GeoPoint::distM(fromPt, toPt);
	for (int i = 0; i < candidateEdges.size(); i++)
	{
		if (roadNetwork.distM(fromPt.lat, fromPt.lon, candidateEdges[i]) < thresholdM &&
			roadNetwork.distM(toPt.lat, toPt.lon, candidateEdges[i]) < thresholdM)
		{
			correctLengthM += GeoPoint::distM(fromPt, toPt);
			return true;
		}
	}
	return false;
}
