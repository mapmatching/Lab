#include <iostream>
#include <list>
using namespace std;


class myStruct
{
	int a;
	double b;
	double c;
	int d;
};

typedef list<myStruct*> listStruct;
list<listStruct*> data;

void genData()
{
	for (int i = 0; i < 16000; i++)
	{
		listStruct* ls = new listStruct;
		for (int j = 0; j < 1000; j++)
		{
			myStruct* ms = new myStruct;
			ls->push_back(ms);
		}
		data.push_back(ls);
	}
	cout << "数据生成完成" << endl;
}

void genData2()
{
	listStruct ls;
	for (int i = 0; i < 16000000; i++)
	{
		myStruct* ms = new myStruct;
		ls.push_back(ms);
	}
	cout << "数据生成完成" << endl;
	system("pause");
}

void destruction()
{
	cout << "开始析构" << endl;
	for each (listStruct* ls in data)
	{
		for each (myStruct* ms in *ls)
		{
			delete ms;
		}
		delete ls;
	}
	cout << "析构完成" << endl;
	data.clear();
}


void main()
{
	for (int i = 0; i < 10; i++)
	{
		genData();
		destruction();
	}

	system("pause");
}