#include "RoadGenerator.h"

//////////////////////////////////////////////////////////////////////////
///public part
//////////////////////////////////////////////////////////////////////////
void RoadGenerator::run(PointGridIndex* _allPtIndex, vector<Cluster*>& _clusters)
{
	this->allPtIndex = _allPtIndex;
	this->clusters = _clusters;
	genAllPolyLines();
	//doRefinement();
	drawAllPolylines();
}

//////////////////////////////////////////////////////////////////////////
///private part
//////////////////////////////////////////////////////////////////////////
bool RoadGenerator::isAGoodCluster(Cluster* cluster)
{
	//////////////////////////////////////////////////////////////////////////
	///判断cluster是否值得生成polyline
	//////////////////////////////////////////////////////////////////////////
	set<pair<int, int>> gridSet;
	for each(GeoPoint* pt in cluster->pts)
	{
		pair<int, int> rowCol = allPtIndex->getRowCol(pt);
		if (gridSet.count(rowCol) == 0)
			gridSet.insert(rowCol);
	}
	if (gridSet.size() <= 2)
		return false;
	else
		return true;
}

double RoadGenerator::cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3)
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

double RoadGenerator::distM(GeoPoint* pt, vector<GeoPoint*>& polyline)
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

bool RoadGenerator::isBadClusterEx(Cluster* cluster, vector<Cluster*>& clusters, MapDrawer& md)
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

void RoadGenerator::genPolyLine(Cluster* cluster, MapDrawer& md)
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
					if (abs(PI - angle - preAngle) < abs(2 * PI - angle - preAngle))
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

void RoadGenerator::drawPolyline(Cluster* cluster, MapDrawer& md, Gdiplus::Color color)
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

void RoadGenerator::genAllPolyLines()
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

}

void RoadGenerator::drawAllPolylines(bool doOutput /* = true */)
{
	ofstream ofs;
	if (doOutput)
	{
		ofs.open("roads_DCMU.txt");
		ofs << fixed << showpoint << setprecision(8);
	}
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i]->polyline.size() == 0)
			continue;
		if (isBadClusterEx(clusters[i], clusters, md))
		{
			drawPolyline(clusters[i], md, Gdiplus::Color::Aqua);
			clusters[i]->polyline.clear();
		}
		else
		{
			drawPolyline(clusters[i], md, Gdiplus::Color::Black);
			if (doOutput)
			{
				for (int j = 0; j < clusters[i]->polyline.size(); j++)
				{
					ofs << 0 << " " << clusters[i]->polyline[j]->lat << " " << clusters[i]->polyline[j]->lon << " " << -1 << endl;
				}
				ofs << -1 << endl;
			}
		}
	}
	if (doOutput)
	{
		ofs.close();
	}
}

GeoPoint* RoadGenerator::intersectCheck(GeoPoint* p1, GeoPoint* p2, vector<GeoPoint*>& polyline)
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

void RoadGenerator::refineOnePolyline(Cluster* objectCluster)
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
		for each (Cluster* cluster in clusters)
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
		currentLength += GeoPoint::distM(objectCluster->polyline[i], objectCluster->polyline[i + 1]);
	}

	//find back cut point
	cutFinishFlag = false;
	currentLength = 0;
	int intersectAfter_e = -1; //记录intersection前面离其最近的顶点索引号
	//对于objectCluster的每一段
	for (int i = objectCluster->polyline.size() - 2; i >= 0; i--)
	{
		if (currentLength > thresholdM)
			break;
		//对于每一条其他的路段
		for each (Cluster* cluster in clusters)
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
				if (currentLength + GeoPoint::distM(objectCluster->polyline[i + 1], intersection_e) > thresholdM)
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

void RoadGenerator::doRefinement()
{
	for each (Cluster* cluster in clusters)
	{
		if (cluster->polyline.size() == 0)
			continue;
		cout << "start refine " << cluster->clusterId << endl;
		refineOnePolyline(cluster);
	}
}
