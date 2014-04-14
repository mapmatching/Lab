/* 
 * Last Updated at [2014/3/4 15:47] by wuhao
 */
#include <iostream>
#include <vector>
#include <list>
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
	vector<list<Pt>*> vSet;
	vector<list<Pt>*> sSet;
	list<Pt> pts;
	double r;
	int n;
	double lambdaP;
	double anglePenalty = 0.01; //跑wangyin实验用0.035比较好

	PolylineGenerator();
	void genPolyline(list<Pt>& pts);

	void doProject(int index);
	void doProject();
	double getRadius();
	void initialization();

//should be private
	void reProject(Pt& pt);
	void optimization();
	void optimization_();
	pair<double, double> central_difference(double(PolylineGenerator::*f)(double, double, int), int vi);
	double calculate(double x, double y, int vi);

private:

};
