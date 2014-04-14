#include "MapMatching.h"


//地图匹配所用参数
#define SIGMAZ 4.591689
#define N2_SIGMAZ2 -0.0237151062104234
#define SQR_2PI_SIGMAZ 0.0868835586212075
#define RANGEOFCANADIDATEEDGES 50.0 //候选路段选取范围
#define TRAJSAMPLERATE 30 //轨迹采样率
#define MINPROB 1e-150 //整体概率的下限
#define INF  1000000000 //最短路径所用
#define MAXSPEED 50 //最大速度
typedef list<GeoPoint*> Traj;

//在不同采样率（1~30s/点）下各种beta数值，打表得出= =
const double BETA_ARR[31] = {
	0,
	0.490376731, 0.82918373, 1.24364564, 1.67079581, 2.00719298,
	2.42513007, 2.81248831, 3.15745473, 3.52645392, 4.09511775,
	4.67319795, 5.41088180, 6.47666590, 6.29010734, 7.80752112,
	8.09074504, 8.08550528, 9.09405065, 11.09090603, 11.87752824,
	12.55107715, 15.82820829, 17.69496773, 18.07655652, 19.63438911,
	25.40832185, 23.76001877, 28.43289797, 32.21683062, 34.56991141
};

//地图匹配所用数据结构
struct Score//代表某个轨迹点对应的一个候选路段
{
	Edge* edge;//候选路段的指针
	long double score;//候选路段所具有的整体概率
	int preColumnIndex;//候选路段的前序路段的列索引
	double distLeft;//轨迹点的投影点到候选路段起点的距离
	Score(Edge* edge, long double score, int pre, double distLeft){
		this->edge = edge;
		this->score = score;
		this->preColumnIndex = pre;
		this->distLeft = distLeft;
	}
};

//最短路径长度所用数据结构
struct NODE_DIJKSTRA {
	int t; double dist;
	NODE_DIJKSTRA(int i, double dist){
		this->t = i;
		this->dist = dist;
	}
	bool operator < (const NODE_DIJKSTRA &rhs) const {
		return dist > rhs.dist;
	}
};

//A路段起点到B路段起点的最小路网距离
double ShortestPathLength(int ID1, int ID2, double dist1, double dist2, double deltaT){
	int size = map.nodes.size();
	//double *dist = new double[size];
	double dist[100000];
	//bool *flag = new bool[size];
	bool flag[100000];
	memset(flag, 0, sizeof(flag));
	for (int i = 0; i < size; i++) {
		dist[i] = INF;
	}
	dist[ID1] = 0;
	priority_queue<NODE_DIJKSTRA> Q;
	NODE_DIJKSTRA tmp(ID1, 0);
	Q.push(tmp);
	while (!Q.empty()) {
		NODE_DIJKSTRA x = Q.top();
		Q.pop();
		int u = x.t;
		if (x.dist + dist1 - dist2 > deltaT*MAXSPEED){
			return INF;
		}
		if (flag[u]) {
			continue;
		}
		flag[u] = true;
		if (u == ID2) {
			break;
		}
		for (AdjNode* i = map.adjList[u]->next; i != NULL; i = i->next) {
			if (dist[i->endPointId] > dist[u] + map.edges[i->edgeId]->lengthM) {
				dist[i->endPointId] = dist[u] + map.edges[i->edgeId]->lengthM;
				NODE_DIJKSTRA tmp(i->endPointId, dist[i->endPointId]);
				Q.push(tmp);
			}
		}
	}
	double resultLength = dist[ID2];
	//delete []dist;
	//delete []flag;
	return resultLength;
}

//放射概率：使用轨迹点到候选路段的距离在高斯分布上的概率
//参数t：为Wang Yin算法而设，表示前后轨迹点间的时间差
//参数dist：轨迹点到候选路段的距离
double logEmissionProb(double t, double dist){
	return ((t + dist * dist + N2_SIGMAZ2))*log(exp(1)) + log(SQR_2PI_SIGMAZ);
}

//辅助函数：给定行索引，计算scoreMatrix中该行中整体概率最大的候选路段的索引
int GetStartColumnIndex(vector<Score> &row){
	int resultIndex = -1;
	long double currentMaxProb = -1;
	for (int i = 0; i<row.size(); i++){
		if (row.at(i).score>currentMaxProb){
			currentMaxProb = row.at(i).score;
			resultIndex = i;
		}
	}
	return resultIndex;
}

list<Edge*> MapMatching(Traj &trajectory){
	list<Edge*> mapMatchingResult;
	long double BT = (long double)BETA_ARR[TRAJSAMPLERATE];//当前轨迹所采用的beta值，计算转移概率时使用
	vector<vector<Score>> scoreMatrix = vector<vector<Score>>();//所有轨迹点的概率矩阵
	GeoPoint* formerTrajPoint = NULL;//上一个轨迹点，计算路网距离时需要
	bool cutFlag = true;//没有前一轨迹点或前一轨迹点没有达标的候选路段
	int currentTrajPointIndex = 0;//当前轨迹点的索引
	list<GeoPoint*>::iterator trajectoryIterator = trajectory.begin();
	double deltaT;
	for (; trajectoryIterator != trajectory.end(); trajectoryIterator++)
	{
		long double currentMaxProb = -1e10;//当前最大概率
		vector<Score> scores = vector<Score>();//当前轨迹点的Score集合
		//vector<Edge*> canadidateEdges = map.getNearEdges((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, RANGEOFCANADIDATEEDGES);//获得所有在指定范围内的候选路段集合

		long double *emissionProbs = new long double[canadidateEdges.size()];//保存这些候选路段的放射概率
		int currentCanadidateEdgeIndex = 0;//当前候选路段的索引
		
		for each (Edge* canadidateEdge in canadidateEdges)
		{
			int preColumnIndex = -1;//保存当前候选路段的前序路段的列索引
			double currentDistLeft = 0;//当前轨迹点在候选路段上的投影点距路段起点的距离
			double DistBetweenTrajPointAndEdge = map.distM((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, canadidateEdge, currentDistLeft);
			//计算这些候选路段的放射概率（Wang Yin写法）
			if (cutFlag == false){
				deltaT = (*trajectoryIterator)->time - formerTrajPoint->time;
				emissionProbs[currentCanadidateEdgeIndex] = logEmissionProb(deltaT, DistBetweenTrajPointAndEdge);
			}
			else{
				//宋神写法
				emissionProbs[currentCanadidateEdgeIndex] = logEmissionProb(1, DistBetweenTrajPointAndEdge);
			}
			//若当前不是第一个轨迹点，则计算这些候选路段的转移概率，结果也存在emissionProbs里面
			if (cutFlag == false){
				int currentMaxProbTmp = -1e10;
				for (int i = 0; i < scoreMatrix.back().size(); i++)
				{
					double formerDistLeft = scoreMatrix[currentTrajPointIndex - 1][i].distLeft;//前一个轨迹点在候选路段上的投影点距路段起点的距离
					double routeNetworkDistBetweenTwoEdges;//两路段起点间的距离
					double routeNetworkDistBetweenTwoTrajPoints;//两轨迹点对应的投影点间的路网距离
					if (canadidateEdge == scoreMatrix[currentTrajPointIndex - 1][i].edge){
						routeNetworkDistBetweenTwoTrajPoints = currentDistLeft - scoreMatrix[currentTrajPointIndex - 1][i].distLeft;
					}
					else
					{
						routeNetworkDistBetweenTwoEdges = ShortestPathLength(scoreMatrix[currentTrajPointIndex - 1][i].edge->startNodeId, canadidateEdge->startNodeId, currentDistLeft, formerDistLeft, deltaT);
						routeNetworkDistBetweenTwoTrajPoints = routeNetworkDistBetweenTwoEdges + currentDistLeft - formerDistLeft;
					}
					//两轨迹点间的直接距离
					double distBetweenTwoTrajPoints = GeoPoint::distM((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, formerTrajPoint->lat, (*trajectoryIterator)->lon);
					//double transactionProb = exp(-fabs((long double)routeNetworkDistBetweenTwoTrajPoints - (long double)distBetweenTwoTrajPoints) / BT) / BT;
					//Wang Yin算法
					long double transactionProb = log(exp(-fabs((long double)routeNetworkDistBetweenTwoTrajPoints) / BT) / BT);
					/*GIS2012CUP的优化加在此处，对transactionProb进行修改*/
					long double tmpTotalProbForTransaction = scoreMatrix[currentTrajPointIndex - 1][i].score + transactionProb;
					if (tmpTotalProbForTransaction > currentMaxProbTmp){
						currentMaxProbTmp = tmpTotalProbForTransaction;
						preColumnIndex = i;
					}
				}
				emissionProbs[currentCanadidateEdgeIndex] += currentMaxProbTmp;
			}
			//if (emissionProbs[currentCanadidateEdgeIndex] > MINPROB){
			//	scores.push_back(Score(canadidateEdge, emissionProbs[currentCanadidateEdgeIndex], preColumnIndex, distLeft));
			//}
			scores.push_back(Score(canadidateEdge, emissionProbs[currentCanadidateEdgeIndex], preColumnIndex, currentDistLeft));
			if (currentMaxProb < emissionProbs[currentCanadidateEdgeIndex]){
				currentMaxProb = emissionProbs[currentCanadidateEdgeIndex];
			}
			currentCanadidateEdgeIndex++;
		}
		delete[]emissionProbs;
		formerTrajPoint = (*trajectoryIterator);
		currentTrajPointIndex++;
		for (int i = 0; i < scores.size(); i++)
		{
			scores[i].score /= currentMaxProb;
		}
		//把该轨迹点的Scores数组放入scoreMatrix中
		scoreMatrix.push_back(scores);
		//若scores数组为空，则说明没有一个达标的候选路段，cutFlag设为true，后续轨迹作为新轨迹进行匹配
		if (scores.size() == 0){
			cutFlag = true;
			formerTrajPoint = NULL;
		}
		else
		{
			cutFlag = false;
		}
	}
	//得到全局匹配路径
	int startColumnIndex = GetStartColumnIndex(scoreMatrix.back());//得到最后一个轨迹点的在scoreMatrix对应行中得分最高的列索引，即全局匹配路径的终点
	for (int i = scoreMatrix.size() - 1; i >= 0; i--){
		if (startColumnIndex != -1){
			mapMatchingResult.push_front(scoreMatrix[i][startColumnIndex].edge);
			startColumnIndex = scoreMatrix[i][startColumnIndex].preColumnIndex;
		}
		else
		{
			mapMatchingResult.push_front(NULL);
			if (i > 0){
				startColumnIndex = GetStartColumnIndex(scoreMatrix[i - 1]);
			}
		}
	}
	return mapMatchingResult;

	//得到神经病路径段
	//list<list<GeoPoint*>> damnTrajPointsList = list<list<GeoPoint*>>();
	//bool startFlag = true;
	//list<GeoPoint*>::iterator trajIter = Traj.begin();
	//for each (Edge* matchedEdge in mapMatchingResult)
	//{
	//	if (matchedEdge == NULL){
	//		if (startFlag){
	//			startFlag = false;
	//			damnTrajPointsList.push_back(list<GeoPoint*>());
	//			if (trajIter != Traj.begin()){
	//				trajIter--;
	//				damnTrajPointsList.back().push_back(*trajIter);
	//				trajIter++;
	//			}	
	//		}
	//		damnTrajPointsList.back().push_back(*trajIter);
	//	}
	//	else{
	//		if (startFlag == false){
	//			damnTrajPointsList.back().push_back(*trajIter);
	//		}
	//		startFlag = true;
	//	}
	//	trajIter++;
	//}
	//return damnTrajPointsList;
}