#include "Traj.h"

void Traj::read(string trajFilePath, list<Traj*>& dest)
{
	cout << ">> start reading " << trajFilePath << endl;
	ifstream ifs(trajFilePath);
	if (!ifs)
	{
		cout << "open " << trajFilePath << " error" << endl;
		system("pause");
		exit(0);
	}
	bool isStart = true;
	int time, mmRoadId;
	double lat, lon;
	Traj* tmpTraj = NULL;
	while (ifs)
	{
		int time;
		ifs >> time;
		if (ifs.fail())
		{
			break;
		}
		if (time == -1)
		{
			isStart = true;
			if (tmpTraj != NULL && tmpTraj->size() > 1)
			{
				dest.push_back(tmpTraj);
			}
			continue;
		}
		else
		{
			ifs >> lat >> lon >> mmRoadId;
			GeoPoint* pt = new GeoPoint(lat, lon, time, mmRoadId);
			if (isStart)
			{
				tmpTraj = new Traj();
				tmpTraj->push_back(pt);
				isStart = false;
			}
			else
			{
				tmpTraj->push_back(pt);
			}
		}
	}
	cout << ">> reading std trajs finished" << endl;
}

int Traj::size() const
{
	return traj.size();
}

void Traj::push_back(GeoPoint* pt)
{
	traj.push_back(pt);
}

void Traj::pop_back()
{
	traj.pop_back();
}