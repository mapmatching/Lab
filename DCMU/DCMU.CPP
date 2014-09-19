#include "DCMU.h"

//////////////////////////////////////////////////////////////////////////
///public part
//////////////////////////////////////////////////////////////////////////

void DCMU::run()
{
	//网格聚类
	gridClustering();
	//降噪
 	for (int i = 0; i < ptIndexes.size(); i++)
 	{
 		//denoiser.run(ptIndexes[i]);
 
// 		/**********************************************************/
// 		/*test code starts from here*/
// 		denoiser.drawPts(md);
// 		/*test code ends*/
// 		/**********************************************************/
// 
 	}
	
	//方向聚类
	for (int i = 0; i < ptIndexes.size(); i++)
	{
		ptCluster.run(ptIndexes[i]);
		ptCluster.drawClusters(md);
	}

	//生成路段
	RoadGenerator rg;
	rg.run(&allPtIndex, ptCluster.clusters);
}


void DCMU::run1()
{
	allPtIndex.drawGridLine(Gdiplus::Color::Green, md);
	//网格聚类
	int index = 54;
	gridClustering();
	//denoiser.run(ptIndexes[index]);
	for (int i = 0; i < ptIndexes.size(); i++)
	{
		int count = 0;
		for (int row = 0; row < ptIndexes[i]->gridHeight; row++)
		{
			for (int col = 0; col < ptIndexes[i]->gridWidth; col++)
			{
				count += ptIndexes[i]->grid[row][col]->size();
			}
		}
		if (count > 50000)
		{
			cout << i << ": " << count <<endl;
			//system("pause");
		}
	}

	ptMover.run(ptIndexes[index]);
	ptMover.drawPoints();
	return;
	//降噪
	// 	for (int i = 0; i < ptIndexes.size(); i++)
	// 	{
	// 
	// 		/**********************************************************/
	// 		/*test code starts from here*/
	// 		denoiser.drawPts(md);
	// 		/*test code ends*/
	// 		/**********************************************************/
	// 
	// 	}

	//方向聚类
	ptCluster.run(ptIndexes[index]);
	ptCluster.drawClusters(md);

	//生成路段
	RoadGenerator rg;
	rg.run(ptIndexes[index], ptCluster.clusters);
}


//////////////////////////////////////////////////////////////////////////
///private part
//////////////////////////////////////////////////////////////////////////
void DCMU::gridClustering()
{
	//////////////////////////////////////////////////////////////////////////
	///网格粗糙聚类，实际上就是个dfs
	///将每个聚类中的点输出到ptIndexes[i]
	///如果联通分支过小则忽略，不输出
	//////////////////////////////////////////////////////////////////////////
	bool** dfsState = new bool*[allPtIndex.gridHeight];
	for (int i = 0; i < allPtIndex.gridHeight; i++)
		dfsState[i] = new bool[allPtIndex.gridWidth];
	for (int i = 0; i < allPtIndex.gridHeight; i++)
	{
		for (int j = 0; j < allPtIndex.gridWidth; j++)
		{
			dfsState[i][j] = false;
		}
	}
	int count = 0; //连通分支编号

	for (int row = 0; row < allPtIndex.gridHeight; row++)
	{
		for (int col = 0; col < allPtIndex.gridWidth; col++)
		{
			if (dfsState[row][col])
				continue;
			else
			{
				vector<pair<int, int>> connectingCompnt; //记录当前dfs的连通分支
				dfs(row, col, dfsState, connectingCompnt);
				if (connectingCompnt.size() < 10) //连通分支过小，忽略
					continue;
				else
				{
					//根据连通分支去索引里把点输出
					PointGridIndex* tempPtIndex = new PointGridIndex;
					list<GeoPoint*> tempPts;
					for (int i = 0; i < connectingCompnt.size(); i++)
					{						
						for each (GeoPoint* pt in *(allPtIndex.grid[connectingCompnt[i].first][connectingCompnt[i].second]))
						{
							tempPts.push_back(pt);
						}
					}
					tempPtIndex->createIndex(tempPts, &area, gridWidth);
					ptIndexes.push_back(tempPtIndex);
					count++;
				}
			}
		}
	}	
}

void DCMU::dfs(int row, int col, bool** dfsState, vector<pair<int, int>>& connectingCompnt)
{
	//////////////////////////////////////////////////////////////////////////
	///对网格[row][col]进行dfs遍历，其中标记访问为此函数职责，dfsState为访问状态，true为已访问
	///将自己的索引编号存入connectingCompnt以记录连通分支信息
	//////////////////////////////////////////////////////////////////////////
	if (row >= allPtIndex.gridHeight || row < 0 || col >= allPtIndex.gridWidth || col < 0)
		return;
	if (dfsState[row][col])
		return;
	else
	{
		dfsState[row][col] = true;
		if (allPtIndex.grid[row][col]->size() <= 3) //网格里点数量小于一定程度的忽略
			return;
		connectingCompnt.push_back(make_pair(row, col));
		dfs(row + 1, col, dfsState, connectingCompnt);
		dfs(row - 1, col, dfsState, connectingCompnt);
		dfs(row, col + 1, dfsState, connectingCompnt);
		dfs(row, col - 1, dfsState, connectingCompnt);
	}
}