/* 
 * Last Updated at [2014/9/9 10:12] by wuhao
 */
#include "Denoiser.h"

//////////////////////////////////////////////////////////////////////////
///public part
//////////////////////////////////////////////////////////////////////////
void Denoiser::run(PointGridIndex* _ptIndex)
{
	this->ptIndex = _ptIndex;
	int k = 20;
	double kNNThresholdM = 3;
	int gridRange = 2;
	double supportRatio = 4.0;

	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				calLMD(pt, k, kNNThresholdM);
			}
		}
		printf("row %d计算完成,共 %d 行\n", row, ptIndex->gridHeight);
	}
	printf("LMD计算完成\n");

	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				outlierValidation(pt, gridRange, supportRatio);
			}
		}
		printf("row %d计算完成,共 %d 行\n", row, ptIndex->gridHeight);
	}
	printf("outlier点排除完成\n");
	return;

	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				outlierReValidation(pt);
			}
		}
		printf("row %d计算完成,共 %d 行\n", row, ptIndex->gridHeight);
	}

	updatePtIndex(ptIndex);
}

void Denoiser::drawPts(MapDrawer& md)
{
	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				if (pt->isOutlier == 1)
					//continue;
					md.drawPoint(Gdiplus::Color::Red, pt->lat, pt->lon);
				else
					md.drawPoint(Gdiplus::Color::Blue, pt->lat, pt->lon);
			}
		}
	}
}
//////////////////////////////////////////////////////////////////////////
///private part
//////////////////////////////////////////////////////////////////////////
void Denoiser::calLMD(GeoPoint* pt, int k, double kNNThresholdM)
{
	//////////////////////////////////////////////////////////////////////////
	///计算pt到它的knn的平均距离
	//////////////////////////////////////////////////////////////////////////
	vector<GeoPoint*> kNNSet;
	ptIndex->kNN(pt, k, kNNThresholdM, kNNSet);
	double lmd = 0;
	for (int i = 0; i < kNNSet.size(); i++)
	{
		lmd += kNNSet[i]->dist;
	}
	lmd /= kNNSet.size();
	pt->lmd = lmd;
}

void Denoiser::outlierValidation(GeoPoint* pt, int gridRange, double supportRatio)
{

	/**********************************************************/
	/*test code starts from here*/
	/*if (pt->extendField1 > 20.0)
	{
	pt->extendField2 = 1;
	return;
	}*/
	if (pt->lmd < 3.0)
	{
		pt->isOutlier = 0;
		return;
	}
	//else
	//	pt->extendField2 = 1;
	//return;
	/*test code ends*/
	/**********************************************************/

	/*vector<GeoPoint*> nearPts;
	ptIndex.getNearPts(pt, gridRange, nearPts);
	for (int i = 0; i < nearPts.size(); i++)
	{
	if (nearPts[i]->extendField1 / pt->extendField1 > supportRatio)
	{
	pt->extendField2 = 1;
	return;
	}
	}*/
	pair<int, int> rolCol = ptIndex->getRowCol(pt);
	int rowPt = rolCol.first;
	int colPt = rolCol.second;
	int row1 = rowPt - gridRange;
	int col1 = colPt - gridRange;
	int row2 = rowPt + gridRange;
	int col2 = colPt + gridRange;
	if (row1 < 0) row1 = 0;
	if (row2 >= ptIndex->gridHeight) row2 = ptIndex->gridHeight - 1;
	if (col1 < 0) col1 = 0;
	if (col2 >= ptIndex->gridWidth) col2 = ptIndex->gridWidth - 1;
	int currentGridCount = ptIndex->grid[rowPt][colPt]->size();
	for (int row = row1; row <= row2; row++)
	{
		for (int col = col1; col <= col2; col++)
		{
			if (ptIndex->grid[row][col]->size() / currentGridCount > supportRatio)
			{
				pt->isOutlier = 1;
				return;
			}
		}
	}
}

void Denoiser::outlierReValidation(GeoPoint* pt)
{
	if (pt->isOutlier == 0)
		return;
	pair<int, int> rolCol = ptIndex->getRowCol(pt);
	int rowPt = rolCol.first;
	int colPt = rolCol.second;
	int row1 = rowPt - 1;
	int col1 = colPt - 1;
	int row2 = rowPt + 1;
	int col2 = colPt + 1;
	if (row1 < 0) row1 = 0;
	if (row2 >= ptIndex->gridHeight) row2 = ptIndex->gridHeight - 1;
	if (col1 < 0) col1 = 0;
	if (col2 >= ptIndex->gridWidth) col2 = ptIndex->gridWidth - 1;
	int innerPtCount = 0;
	int outlierCount = 0;
	for (int row = row1; row <= row2; row++)
	{
		for (int col = col1; col <= col2; col++)
		{
			if (row == rowPt && col == colPt)
				continue;
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				if (pt->isOutlier == 1)
					outlierCount++;
				else
					innerPtCount++;
			}
		}
	}
	if (outlierCount == 0)
	{
		if (innerPtCount >= 30)
		{
			pt->isOutlier = 0;
			return;
		}
		else
			return;
	}
	if (innerPtCount / outlierCount > 10)
	{
		pt->isOutlier = 0;
	}
}

void Denoiser::updatePtIndex(PointGridIndex* ptIndex)
{
	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for (list<GeoPoint*>::iterator iter = ptIndex->grid[row][col]->begin(); iter != ptIndex->grid[row][col]->end();)
			{
				if ((*iter)->isOutlier == 0)
				{
					iter = ptIndex->grid[row][col]->erase(iter);
				}
				else
				iter++;
			}
		}
	}
}
