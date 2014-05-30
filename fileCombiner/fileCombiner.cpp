#include <iostream>
#include <fstream>
#include <string>
using namespace std;

void main()
{
	string appendToFilePath = "D:\\trajectory\\singapore_data\\experiments\\wy_MMTrajs.txt";
	string appendFromFilePath = "D:\\trajectory\\singapore_data\\experiments\\wy_MMTrajs2.txt";
	ofstream ofs(appendToFilePath, ios::app);
	ifstream ifs(appendFromFilePath);
	if (!ofs)
	{
		cout << "open file " << appendToFilePath << " error!" << endl;
		system("pause");
		exit(0);
	}
	if (!ifs)
	{
		cout << "open file " << appendFromFilePath << " error!" << endl;
		system("pause");
		exit(0);
	}
	string strLine;
	while (getline(ifs, strLine))
	{
		if (ifs.fail())
			break;
		ofs << strLine << endl;
	}
	ifs.close();
	ofs.close();
}