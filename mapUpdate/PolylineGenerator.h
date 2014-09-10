/*
* Last Updated at [2014/9/10 22:04] by wuhao
*/
#pragma once
#include <iostream>
#include <vector>
#include <list>
#include "Matrix.h"
using namespace std;

struct Pt
{
	double x;
	double y;
	double dist;
	Pt(double x, double y)
	{
		this->x = x;
		this->y = y;
	}
	Pt(){}
};


class PolylineGenerator
{
public:
	vector<Pt> polyline;
	PolylineGenerator();
	void genPolyline(list<Pt>& pts);

	//private:
	vector<list<Pt>*> vSet;
	vector<list<Pt>*> sSet;
	list<Pt> pts;
	double r;
	int n;
	double lambdaP;
	double anglePenalty = 0.01; //跑wangyin实验用0.02比较好

	//中心差分求导用
	int max_it = 100000;
	double stopEps = 1e-5;

	void sampling(int sampleNum);
	void doProject(int index);
	void doProject();
	double getRadius();
	void initialization();
	void reProject(Pt& pt);
	void optimization();
	void optimizationEx();

	double calcStep(int x, int y, int vi); //计算optimization中沿负梯度方向移动点的步长
	double d2f_dx2(int x, int y, int i);
	double d2f_dy2(int x, int y, int i);
	double d2f_dxdy(int x, int y, int i);
	pair<double, double> central_difference(double(PolylineGenerator::*f)(double, double, int), int vi);
	double calculate(double x, double y, int i);
};
