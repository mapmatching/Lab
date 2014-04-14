#include "MapMatching.h"

struct Score
{
	Edge* edge;
	long double score;
	int preColumnIndex;
	double distLeft;
	Score(Edge* edge, long double score, int pre, double distLeft){
		this->edge = edge;
		this->score = score;
		this->preColumnIndex = pre;
		this->distLeft = distLeft;
	}
};

//修改过；按新加坡轨迹数据统计得到的系数；并改为取log后的计算公式
double EmissionProb(double t, double dist){
	return t*sqrt(dist)*COEFFICIENT_FOR_EMISSIONPROB;
}

//修改过：取“概率”最小的为匹配路径起点
int GetStartColumnIndex(vector<Score> &row){
	int resultIndex = -1;
	long double currentMaxProb = 1e10;
	for (int i = 0; i < row.size(); i++){
		if (currentMaxProb > row.at(i).score){
			currentMaxProb = row.at(i).score;
			resultIndex = i;
		}
	}
	return resultIndex;
}

list<Edge*> MapMatching(list<GeoPoint*> &trajectory, double candidatesPickRange)
{
	list<Edge*> mapMatchingResult;
	vector<vector<Score>> scoreMatrix = vector<vector<Score>>();
	GeoPoint* formerTrajPoint = NULL;
	bool cutFlag = true;
	int currentTrajPointIndex = 0;
	list<GeoPoint*>::iterator trajectoryIterator = trajectory.begin();
	for (; trajectoryIterator != trajectory.end(); trajectoryIterator++)
	{
		double deltaT = -1;
		if (formerTrajPoint != NULL){
			deltaT = (*trajectoryIterator)->time - formerTrajPoint->time;
		}
		//修改过；取log后，初始值从最小值变为最大值
		long double currentMaxProb = 1e10;
		vector<Score> scores = vector<Score>();
		vector<Edge*> canadidateEdges;
		map.getNearEdges((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, candidatesPickRange, canadidateEdges);
		long double *emissionProbs = new long double[canadidateEdges.size()];
		int currentCanadidateEdgeIndex = 0;
		for each (Edge* canadidateEdge in canadidateEdges)
		{
			int preColumnIndex = -1;
			double currentDistLeft = 0;
			double DistBetweenTrajPointAndEdge = map.distM((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, canadidateEdge, currentDistLeft);
			if (cutFlag){
				emissionProbs[currentCanadidateEdgeIndex] = EmissionProb(1, DistBetweenTrajPointAndEdge);
			}
			else{
				emissionProbs[currentCanadidateEdgeIndex] = EmissionProb(deltaT, DistBetweenTrajPointAndEdge);
				//修改过；取log后，初始值从最小值变为最大值
				long double currentMaxProbTmp = 1e10;
				for (int i = 0; i < scoreMatrix.back().size(); i++)
				{
					double formerDistLeft = scoreMatrix[currentTrajPointIndex - 1][i].distLeft;
					double routeNetworkDistBetweenTwoEdges;
					double routeNetworkDistBetweenTwoTrajPoints;
					if (canadidateEdge == scoreMatrix[currentTrajPointIndex - 1][i].edge){
						routeNetworkDistBetweenTwoTrajPoints = currentDistLeft - scoreMatrix[currentTrajPointIndex - 1][i].distLeft;
					}
					else
					{
						routeNetworkDistBetweenTwoEdges = map.shortestPathLength(scoreMatrix[currentTrajPointIndex - 1][i].edge->startNodeId, canadidateEdge->startNodeId, currentDistLeft, formerDistLeft, deltaT);
						routeNetworkDistBetweenTwoTrajPoints = routeNetworkDistBetweenTwoEdges + currentDistLeft - formerDistLeft;
					}
					double distBetweenTwoTrajPoints = GeoPoint::distM((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, formerTrajPoint->lat, (*trajectoryIterator)->lon);
					//修改过；按新加坡轨迹数据统计得到的系数
					long double transactionProb = (long double)routeNetworkDistBetweenTwoTrajPoints*COEFFICIENT_FOR_TRANSATIONPROB;
					long double tmpTotalProbForTransaction = scoreMatrix[currentTrajPointIndex - 1][i].score * transactionProb;
					//修改过；取局部最小值
					if (currentMaxProbTmp > tmpTotalProbForTransaction){
						currentMaxProbTmp = tmpTotalProbForTransaction;
						preColumnIndex = i;
					}
				}
				//修改过；取log后乘法变加法
				emissionProbs[currentCanadidateEdgeIndex] += currentMaxProbTmp;
			}
			scores.push_back(Score(canadidateEdge, emissionProbs[currentCanadidateEdgeIndex], preColumnIndex, currentDistLeft));
			//修改过；取局部最小值
			if (currentMaxProb > emissionProbs[currentCanadidateEdgeIndex]){
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
		scoreMatrix.push_back(scores);
		if (scores.size() == 0){
			cutFlag = true;
			formerTrajPoint = NULL;
		}
		else
		{
			cutFlag = false;
		}
	}
	int startColumnIndex = GetStartColumnIndex(scoreMatrix.back());
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
}