#pragma once

#include <vector>
#include <map>

#include "Network.h"

void findSpanTree(Network& network, int indV, std::vector<int>& VertArr, std::vector<bool>& visited, std::vector<int>& spanVertArr, std::map<std::tuple<int, int>, bool>& spanArcMap);

bool findCyclicUtil(int indV, int indPar, std::vector<int>& VertArr, std::vector<bool>& visited, std::vector<int>& cycleArr, bool& isDone, std::map<std::tuple<int, int>, bool>& spanArcMap);

bool findCycles(Network& network, std::vector<int>& VertArr, std::vector<std::vector<int>>& cycleMx);