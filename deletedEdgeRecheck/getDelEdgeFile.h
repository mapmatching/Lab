#pragma once
#include <fstream>
#include <string>
#include <iostream>
#include <iomanip>
using namespace std;

class DelEdgeFile
{
public:
	DelEdgeFile();
	DelEdgeFile(string filePath, string fileName);
	void openFile(string filePath, string fileName);

private:
	ifstream infileIfs;
	ofstream outfileOfs;
};