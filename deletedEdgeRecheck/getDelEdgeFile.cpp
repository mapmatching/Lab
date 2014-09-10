#include "getDelEdgeFile.h"

DelEdgeFile::DelEdgeFile(string filePath, string fileName)
{
	DelEdgeFile::openFile(filePath, fileName);
}

void DelEdgeFile::openFile(string filePath, string fileName)
{
	int first, last;
	double lon, lat;

	infileIfs.open(filePath + fileName);
	if (!infileIfs)
	{
		cout << "can not open " + filePath + fileName + ".\n";
		system("pause");
		exit(0);
	}

	outfileOfs.open(filePath+"out\\deletePoint.txt");
	outfileOfs << fixed << showpoint << setprecision(8);
	if (!outfileOfs){
		cout << "can not create deletePoint.txt.\n";
		system("pause");
		exit(0);
	}

	while (infileIfs){
		infileIfs >> first;
		if (infileIfs.fail()){
			break;
		}
		if (first < 0){
			continue;
		}
		infileIfs >> lat >> lon >> last;
		if (first >= 0 && -1 == last){
			outfileOfs << lat << " " << lon << "\n";
			cout << first << " " << lat << " " << lon << " " << last << endl;
		}
	}

	cout << "file-reading finished!" << endl;
	system("pause");
	infileIfs.close();
	outfileOfs.close();
}