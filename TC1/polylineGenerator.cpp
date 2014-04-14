/* 
 * Last Updated at [2014/3/4 15:47] by wuhao
 */
#include "PolylineGenerator.h"


double dist(Pt& pt1, Pt& pt2)
{
	return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

double dist(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double cosAngle(Pt& pt1, Pt& pt2, Pt& pt3)
{
	//////////////////////////////////////////////////////////////////////////
	///返回∠pt1_pt2_pt3的cos值
	//////////////////////////////////////////////////////////////////////////
	double v1x = pt1.x - pt2.x;
	double v1y = pt1.y - pt2.y;
	double v2x = pt3.x - pt2.x;
	double v2y = pt3.y - pt2.y;
	return (v1x * v2x + v1y * v2y) / sqrt((v1x * v1x + v1y * v1y)*(v2x * v2x + v2y * v2y));
}

pair<double, double> PolylineGenerator::central_difference(double (PolylineGenerator::*f)(double, double, int), int vi)
{
	//////////////////////////////////////////////////////////////////////////
	///求df/dx@x, df/dy@y
	//////////////////////////////////////////////////////////////////////////
	double ff, gg;
	int k;
	int max_it = 50;
	double eps = 1e-5;
	double h = 1;
	ff = 0.0;
	pair<double, double> ans;
	double x = polyline[vi].x;
	double y = polyline[vi].y;
	
	//求df/dx
	for (k = 0; k < max_it; k++)
	{
		gg = ((this->*f)(x + h, y, vi) - (this->*f)(x - h, y, vi)) / (h + h);
		if (fabs(gg - ff) < eps)
		{
			ans.first = gg;
			break;
		}
		h *= 0.5;
		ff = gg;
	}
	if (k == max_it)
	{
		printf("未能达到精度要求,需增大迭代次数!");
		system("pause");
	}
	ans.first = gg;

	//求df/dy
	ff = 0.0;
	h = 1;
	for (k = 0; k < max_it; k++)
	{
		gg = ((this->*f)(x, y + h, vi) - (this->*f)(x, y - h, vi)) / (h + h);
		if (fabs(gg - ff) < eps)
		{
			ans.second = gg;
			break;
		}
		h *= 0.5;
		ff = gg;
	}
	if (k == max_it)
	{
		printf("未能达到精度要求,需增大迭代次数!");
		system("pause");
	}
	ans.second = gg;
	return ans;
}

double PolylineGenerator::calculate(double x, double y, int i)
{
	double Gvi = 0;
	//计算前半部分
	double delta_nvi = 0;
	double vvi = 0, sigma_posi_vi = 0, sigma_nega_vi = 0;
	int k = polyline.size() - 1;
	//求vvi, sigma_posi_vi, sigma_nega_vi
	for each (Pt pt in (*vSet[i]))
	{
		vvi += dist(x, y, pt.x, pt.y) * dist(x, y, pt.x, pt.y);
	}
	//printf("vvi = %lf\n", vvi);
	if (i < k)
	{
		for each (Pt pt in (*sSet[i]))
		{
			double x0 = pt.x, y0 = pt.y;
			double _x = polyline[i + 1].x;
			double _y = polyline[i + 1].y;
			double cos = ((x - x0) * (x - _x) + (y - y0) * (y - _y)) /
				sqrt(((x - x0) * (x - x0) + (y - y0) * (y - y0)) * ((x - _x) * (x - _x) + (y - _y) * (y - _y)));
			double sin = sqrt(1 - cos * cos);
			double d = dist(x, y, x0, y0) * sin;
			sigma_posi_vi += d * d;
		}
		//printf("sigma_posi_vi = %lf\n", sigma_posi_vi);
	}
	if (i > 0)
	{
		for each (Pt pt in (*sSet[i - 1]))
		{
			double x0 = pt.x, y0 = pt.y;
			double _x = polyline[i - 1].x;
			double _y = polyline[i - 1].y;
			double cos = ((x - x0) * (x - _x) + (y - y0) * (y - _y)) / 
				sqrt(((x - x0) * (x - x0) + (y -y0) * (y - y0)) * ((x - _x) * (x - _x) + (y - _y) * (y - _y)));
			double sin = sqrt(1 - cos * cos);
			double d = dist(x, y, x0, y0) * sin;
			sigma_nega_vi += d * d;
		}
		//printf("sigma_nega_vi = %lf\n", sigma_nega_vi);
	}
	//求delta_nvi
	if (i == 0)
	{
		delta_nvi = vvi + sigma_posi_vi;
	}
	else if (i == k)
	{
		delta_nvi = vvi + sigma_nega_vi;
	}
	else
	{
		delta_nvi = vvi + sigma_nega_vi + sigma_posi_vi;
	}
	Gvi += delta_nvi / n;

	//计算后半部分,锐角惩罚
	double pvi = 0;
	//特殊情况特判
	if (k == 1)
		return Gvi;
	else if (k == 2 && i == 1)
	{
		pvi += dist(x, y, polyline[i - 1].x, polyline[i - 1].y);
		pvi += r * r * (1 + cosAngle(polyline[i - 1], Pt(x, y), polyline[i + 1]));
		pvi += dist(x, y, polyline[i + 1].x, polyline[i + 1].y);
	}
	else
	{
		if (i == 0)
		{
			pvi += dist(x, y, polyline[i + 1].x, polyline[i + 1].y); //μ+(vi)
			pvi += r * r * (1 + cosAngle(Pt(x, y), polyline[i + 1], polyline[i + 2])); //π(vi+1)
		}
		else if (i == 1)
		{
			pvi += dist(x, y, polyline[i - 1].x, polyline[i - 1].y); //μ-(vi)
			pvi += r * r * (1 + cosAngle(polyline[i - 1], Pt(x, y), polyline[i + 1])); //π(vi)
			pvi += r * r * (1 + cosAngle(Pt(x, y), polyline[i + 1], polyline[i + 2])); //π(vi+1)
		}
		else if (i == k - 1)
		{
			pvi += r * r * (1 + cosAngle(polyline[i - 2], polyline[i - 1], Pt(x, y))); //π(vi-1)
			pvi += r * r * (1 + cosAngle(polyline[i - 1], Pt(x, y), polyline[i + 1])); //π(vi)
			pvi += dist(x, y, polyline[i + 1].x, polyline[i + 1].y); //μ+(vi)
		}
		else if (i == k)
		{
			pvi += r * r * (1 + cosAngle(polyline[i - 2], polyline[i - 1], Pt(x, y))); //π(vi-1)
			pvi += dist(x, y, polyline[i - 1].x, polyline[i - 1].y); //μ-(vi)
		}
		else
		{
			pvi += r * r * (1 + cosAngle(polyline[i - 2], polyline[i - 1], Pt(x, y))); //π(vi-1)
			pvi += r * r * (1 + cosAngle(polyline[i - 1], Pt(x, y), polyline[i + 1])); //π(vi)
			pvi += r * r * (1 + cosAngle(Pt(x, y), polyline[i + 1], polyline[i + 2])); //π(vi+1)
		}
	}
	//cout << "angle penalty = " << lambdaP * 1 / (k + 1) * pvi << endl;
	Gvi += lambdaP * 1 / (k + 1) * pvi;
	return Gvi;
}

PolylineGenerator::PolylineGenerator()
{

}

void PolylineGenerator::genPolyline(list<Pt>& pts)
{
	for each(Pt pt in pts)
	{
		this->pts.push_back(pt);
	}
	initialization();
}

double PolylineGenerator::getRadius()
{
	//////////////////////////////////////////////////////////////////////////
	///返回数据点的半径
	//////////////////////////////////////////////////////////////////////////                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
	double medianY = 0, medianX = 0;
	for (list<Pt>::iterator ptIter = pts.begin(); ptIter != pts.end(); ptIter++)
	{
		medianX += ptIter->x;
		medianY += ptIter->y;
	}
	medianY /= pts.size();
	medianX /= pts.size();
	double radius = 0;
	for (list<Pt>::iterator ptIter = pts.begin(); ptIter != pts.end(); ptIter++)
	{
		double d = dist(ptIter->x, ptIter->y, medianX, medianY);
		if (d > radius)
			radius = d;
	}
	return radius;
}

void PolylineGenerator::initialization()
{
	polyline.clear();
	n = pts.size();
	r = getRadius();
	//初始化,先找两个端点,取lat坐标距离最大的两个或lon坐标距离最大的两个
	double minX = 99999999, maxX = 0;
	double minY = 99999999, maxY = 0;
	Pt maxXpt, minXpt, maxYpt, minYpt;
	for each (Pt pt in pts)
	{
		if (pt.x < minX)
		{
			minX = pt.x;
			minXpt.x = pt.x;
			minXpt.y = pt.y;
		}
		if (pt.x > maxX)
		{
			maxX = pt.x;
			maxXpt.x = pt.x;
			maxXpt.y = pt.y;
		}
		if (pt.y < minY)
		{
			minY = pt.y;
			minYpt.x = pt.x;
			minYpt.y = pt.y;
		}
		if (pt.y > maxY)
		{
			maxY = pt.y;
			maxYpt.x = pt.x;
			maxYpt.y = pt.y;
		}
	}
	//printf("(%lf, %lf) , (%lf, %lf)\n", minX, minY, maxX, maxY);
	Pt startPt, endPt;
	if (dist(minXpt, maxXpt) > dist(minYpt, maxYpt))
	{
		startPt = minXpt;
		endPt = maxXpt;
	}
	else
	{
		startPt = minYpt;
		endPt = maxYpt;
	}
	polyline.push_back(startPt);
	polyline.push_back(endPt);
	vSet.push_back(new list<Pt>);
	vSet.push_back(new list<Pt>);
	sSet.push_back(new list<Pt>);
	vSet[0]->splice(vSet[0]->end(), pts);
	doProject();
	//printf("(%lf, %lf) , (%lf, %lf)\n", polyline[0].x, polyline[0].y, polyline[1].x, polyline[1].y);
}

void PolylineGenerator::doProject(int index)
{
	//////////////////////////////////////////////////////////////////////////
	///TODO:
	///对点i更新投影集,需要更新的为Si-2,Si-1,Si,Si+1,Vi-1,Vi,Vi+1这些集合
	//////////////////////////////////////////////////////////////////////////
	//update S
/*	list<PrjPoint> tempPtds;

	for (int i = -2; i <= 1; i++)
	{
		if (index + i < 0 || index + 1 >= polyline.size() - 1)
			continue;
		tempPtds.splice(tempPtds.end(), polyline[index]->vSet);
		tempPtds.splice(tempPtds.end(), polyline[index]->sSet);

	}
	//update V
	for (int i = -1; i <= 1; i++)
	{
		if (index + i < 0 || index + 1 >= polyline.size());
	}
	*/
}

void PolylineGenerator::doProject()
{
	//////////////////////////////////////////////////////////////////////////
	///1.清空vSet和sSet
	///2.将点重新投影后到各自vSet与sSet中
	///3.更新lambdaP
	//////////////////////////////////////////////////////////////////////////
	list<Pt> tempPts;
	//回收所有点
	for (int i = 0; i < polyline.size() - 1; i++)
	{
		tempPts.splice(tempPts.end(), (*vSet[i]));
		tempPts.splice(tempPts.end(), (*sSet[i]));
	}
	tempPts.splice(tempPts.end(), (*vSet[polyline.size() - 1]));
	//重新分配
	while (tempPts.size() != 0)
	{
		Pt top = tempPts.back();
		reProject(top);
		tempPts.pop_back();
	}
	//更新lambdaP
	double delta_n_f = 0;
	for (int i = 0; i < vSet.size(); i++)
	{
		for each (Pt pt in (*vSet[i]))
		{
			delta_n_f += pt.dist * pt.dist;
		}
	}
	for (int i = 0; i < sSet.size(); i++)
	{
		for each (Pt pt in (*sSet[i]))
		{
			delta_n_f += pt.dist * pt.dist;
		}
	}
	lambdaP = anglePenalty * (polyline.size()-1) * pow(n, -1 / 3) * pow(delta_n_f, 0.5) / r;
}

void PolylineGenerator::optimization()
{
	//////////////////////////////////////////////////////////////////////////
	///不用这个版本
	//////////////////////////////////////////////////////////////////////////
	int k = polyline.size() - 1;
	double convergeThreshold = 1;
	//对每个点迭代
	cout << "poly size " << polyline.size() << endl;
	for (int i = 0; i < polyline.size(); i++)
	{
		//直到这个点的调整幅度达到收敛阈值
		//TEST: 先对每个点迭代3次
		for (int iter = 0; iter < 3; iter++)
		{
			double delta_nvi = 0;
			double vvi = 0, sigma_posi_vi = 0, sigma_nega_vi = 0;
			//求vvi, sigma_posi_vi, sigma_nega_vi
			for each (Pt pt in (*vSet[i]))
			{
				vvi += (pt.dist * pt.dist);
			}
			if (i < k)
			{
				for each (Pt pt in (*sSet[i]))
				{
					sigma_posi_vi += (pt.dist * pt.dist);
				}
			}
			if (i > 1)
			{
				for each (Pt pt in (*sSet[i - 1]))
				{
					sigma_nega_vi += (pt.dist * pt.dist);
				}
			}
			//求delta_nvi
			if (i == 0)
			{
				delta_nvi = vvi + sigma_posi_vi;
			}
			else if (i == k)
			{
				delta_nvi = vvi + sigma_nega_vi;
			}
			else
			{
				delta_nvi = vvi + sigma_nega_vi + sigma_posi_vi;
			}
			double Gvi = 1 / n * delta_nvi;
			//求梯度
			double sigma_x_in_seti = 0, sigma_y_in_seti = 0;
			int setPtCount = 0;
			//test code starts
			cout << "vSet" << i << " size: " << vSet[1]->size() << endl;
			//test code ends
			for each(Pt pt in (*vSet[i]))
			{
				sigma_x_in_seti += pt.x;
				sigma_y_in_seti += pt.y;
			}
			setPtCount += vSet[i]->size();
			if (i == 0)
			{
				for each(Pt pt in (*sSet[i]))
				{
					sigma_x_in_seti += pt.x;
					sigma_y_in_seti += pt.y;
				}
				setPtCount += sSet[i]->size();
			}
			else if (i == k)
			{
				for each(Pt pt in (*sSet[i - 1]))
				{
					sigma_x_in_seti += pt.x;
					sigma_y_in_seti += pt.y;
				}
				setPtCount += sSet[i - 1]->size();
			}
			else
			{
				for each(Pt pt in (*sSet[i - 1]))
				{
					sigma_x_in_seti += pt.x;
					sigma_y_in_seti += pt.y;
				}
				for each(Pt pt in (*sSet[i]))
				{
					sigma_x_in_seti += pt.x;
					sigma_y_in_seti += pt.y;
				}
				setPtCount += sSet[i]->size() + sSet[i - 1]->size();
			}
			cout << "setPtCount = " << setPtCount << endl;
			double gradient_x = 2 / n * setPtCount * polyline[i].x - 2 / n * sigma_x_in_seti;
			double gradient_y = 2 / n * setPtCount * polyline[i].y - 2 / n * sigma_y_in_seti;
			double direction_x = -gradient_x / sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
			double direction_y = -gradient_y / sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
			//test code start
			printf("v%d gradient(%lf, %lf)\n", i, gradient_x, gradient_y);
			printf("v%d direction(%lf, %lf)\n", i, direction_x,direction_y);
			//double step = gradient_x * gradient_x + gradient_y * gradient_y;
			double step = 10;
			double newX = polyline[i].x + step * direction_x;
			double newY = polyline[i].y + step * direction_y;
			double delta = (newX - polyline[i].x) * (newX - polyline[i].x) + (newY - polyline[i].y) * (newY - polyline[i].y);
			//if (delta < convergeThreshold)
		//		break;
			//else
			{
				polyline[i].x = newX;
				polyline[i].y = newY;
				doProject();
			}			
		}
	}
	//add a new point
	double maxLength = 0;
	int candidateV;
	for (int i = 0; i < polyline.size() - 1; i++)
	{
		double tempDist = dist(polyline[i], polyline[i + 1]);
		if (tempDist > maxLength)
		{
			maxLength = tempDist;
			candidateV = i;
		}
	}
	Pt newPt;
	newPt.x = (polyline[candidateV].x + polyline[candidateV + 1].x) / 2;
	newPt.y = (polyline[candidateV].y + polyline[candidateV + 1].y) / 2;
	polyline.insert(polyline.begin() + candidateV + 1, newPt);
	vSet.push_back(new list<Pt>);
	sSet.push_back(new list<Pt>);
	doProject();
}

void PolylineGenerator::optimization_()
{
	int k = polyline.size() - 1;
	double convergeThreshold = 1;
	//对每个点迭代
	cout << "poly size " << polyline.size() << endl;
	for (int _i = 0; _i < 10; _i++)
	{
		for (int i = 0; i < polyline.size(); i++)
		{
			//直到这个点的调整幅度达到收敛阈值
			//TEST: 先对每个点迭代3次
			double step = 0.2;
			for (int iter = 0; iter < 100; iter++)
			{
				double Gvi = calculate(polyline[i].x, polyline[i].y, i);
				//printf("Gvi = %lf\n", Gvi);
				//求梯度
				double (PolylineGenerator::* pCalcFunc)(double, double, int); //一个类成员函数指针变量pmf的定义
				pCalcFunc = &PolylineGenerator::calculate;
				pair<double, double> gradient = central_difference(pCalcFunc, i);
				double gradient_x = gradient.first;
				double gradient_y = gradient.second;
				//归一化
				double direction_x = -gradient_x / sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
				double direction_y = -gradient_y / sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
				//test code start
				//printf("v%d gradient(%lf, %lf)\n", i, gradient_x, gradient_y);
				//printf("v%d direction(%lf, %lf)\n", i, direction_x, direction_y);
				//double step = gradient_x * gradient_x + gradient_y * gradient_y;
				//double step = sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
				//step = step * 14 / 15;
				double newX = polyline[i].x + step * direction_x;
				double newY = polyline[i].y + step * direction_y;
				double delta = (newX - polyline[i].x) * (newX - polyline[i].x) + (newY - polyline[i].y) * (newY - polyline[i].y);
				//if (delta < convergeThreshold)
				//		break;
				//else
				{
					polyline[i].x = newX;
					polyline[i].y = newY;
					doProject();
				}
			}
		}
	}
	//add a new point
	double maxLength = 0;
	int candidateV;
	/*for (int i = 0; i < polyline.size() - 1; i++)
	{
		double tempDist = dist(polyline[i], polyline[i + 1]);
		if (tempDist > maxLength)
		{
			maxLength = tempDist;
			candidateV = i;
		}
	}*/
	double maxNum = -1;
	for (int i = 0; i < polyline.size() - 1; i++)
	{
		if (sSet[i]->size() > maxNum)
		{
			maxNum = sSet[i]->size();
			candidateV = i;
		}
	}
	Pt newPt;
	newPt.x = (polyline[candidateV].x + polyline[candidateV + 1].x) / 2;
	newPt.y = (polyline[candidateV].y + polyline[candidateV + 1].y) / 2;
	polyline.insert(polyline.begin() + candidateV + 1, newPt);
	vSet.push_back(new list<Pt>);
	sSet.push_back(new list<Pt>);
	doProject();
}

void PolylineGenerator::reProject(Pt& pt)
{
	//////////////////////////////////////////////////////////////////////////
	///将pt投影到对应的vSet或sSet中
	//////////////////////////////////////////////////////////////////////////
	double minDist = 9999999.0;
	list<Pt>* dest = NULL;
	//遍历端点距离
	for (int i = 0; i < polyline.size(); i++)
	{
		double tmpDist = dist(polyline[i], pt);
		if (tmpDist < minDist)
		{
			minDist = tmpDist;
			dest = vSet[i];
		}
	}
	//test code starts
	/*if (dest == NULL)
	{
		printf("v0 (%lf, %lf)\n", polyline[0].x, polyline[0].y);
		cout << "dist to v0" << dist(polyline[0], pt) << endl;
		cout << "dist to v1" << dist(polyline[1], pt) << endl;
		cout << "polyline size: " << polyline.size() << endl;
		printf("pt (%lf, %lf)\n", pt.x, pt.y);
		cout << "dest should not bu null! " << endl;
		system("pause");
		exit(0);
	}*/
	//test code ends

	//遍历投影距离
	for (int i = 0; i < polyline.size() - 1; i++)
	{
		//有投影
		if (cosAngle(pt, polyline[i], polyline[i + 1]) >= 0 && cosAngle(pt, polyline[i + 1], polyline[i]) >= 0)
		{
			double A = polyline[i + 1].y - polyline[i].y;
			double B = -(polyline[i + 1].x - polyline[i].x);
			double C = polyline[i].y * (polyline[i + 1].x - polyline[i].x)
				- polyline[i].x * (polyline[i + 1].y - polyline[i].y);
			double tmpDist1 = abs(A * pt.x + B * pt.y + C) / sqrt(A * A + B * B);
			double cos = cosAngle(pt, polyline[i], polyline[i + 1]);
			double sin = sqrt(1 - cos*cos);
			double tmpDist2 = dist(pt, polyline[i]) * sin;
			if (abs(tmpDist1 - tmpDist2)> 10e-7)
			{
				cout << "not equal!" << endl;
				printf("using line func = %.9lf, using cos = %.9lf\n", tmpDist1, tmpDist2);
			}
			if (minDist > tmpDist2)
			{
				minDist = tmpDist2;
				dest = sSet[i];
			}
		}
	}
	pt.dist = minDist;
	if (dest == NULL)
	{
		cout << "NULL! " << endl;
		system("pause");
		exit(0);
	}
	dest->push_back(pt);
}