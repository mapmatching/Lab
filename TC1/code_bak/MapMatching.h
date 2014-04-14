#pragma once
#include <iostream>
#include <fstream>
#include <direct.h>
#include <io.h>
#include <vector>
#include <algorithm>
#include <queue>
#include "GeoPoint.h"
#include "Map.h"
using namespace std;

extern Map map;
extern list<Edge*> MapMatching(list<GeoPoint*> &trajectory);