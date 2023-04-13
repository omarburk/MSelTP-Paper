#include "Algos.h"
//#include <iostream>
//DFS

void findSpanTree(Network& network, int indV, std::vector<int>& VertArr, std::vector<bool>& visited, std::vector<int>& spanVertArr, std::map<std::tuple<int, int>, bool>& spanArcMap)
{
	int v1 = VertArr[indV];
	int v2;
	visited[indV] = true;
	spanVertArr.push_back(v1);

	for (int i = 0; i < VertArr.size(); i++)
	{
		if (i != indV && !visited[i])
		{
			v2 = VertArr[i];
			
			if (network.ArcMap[std::make_tuple(v1, v2)])
			{
				spanArcMap[std::make_tuple(v1, v2)] = true;
				spanArcMap[std::make_tuple(v2, v1)] = true;

				findSpanTree(network, i, VertArr, visited, spanVertArr, spanArcMap);
			}
		}
	}
}

bool findCyclicUtil(int indV, int indPar, std::vector<int>& VertArr, std::vector<bool>& visited, std::vector<int>& cycleArr, bool& isDone, std::map<std::tuple<int, int>, bool>& spanArcMap)
{
	int v1 = VertArr[indV];
	int v2;
	visited[indV] = true;

	for (int i = 0; i < VertArr.size(); i++)
	{
		if (i != indV && i != indPar)
		{
			v2 = VertArr[i];

			if (spanArcMap[std::make_tuple(v1, v2)])
			{
				if (!visited[i])
				{
					if (findCyclicUtil(i, indV, VertArr, visited, cycleArr, isDone, spanArcMap))
					{
						if (!isDone)
						{
							if (v2 != cycleArr[0])
								cycleArr.push_back(v2);
							else
								isDone = true;
						}

						return true;
					}
				}
				else
				{
					cycleArr.push_back(v2);
					return true;
				}
			}
		}
	}

	return false;
}

bool findCycles(Network& network, std::vector<int>& VertArr, std::vector<std::vector<int>>& cycleMx)
{
	std::vector<bool> visited(VertArr.size(), false); //thats for spanning tree search
	int numCycle = cycleMx.size();
	int cnt = 0;

	for (int i = 0; i < VertArr.size(); i++)
		if (!visited[i])
		{
			std::vector<int> spanVertArr;
			spanVertArr.reserve(VertArr.size());

			std::map<std::tuple<int, int>, bool> spanArcMap;
			findSpanTree(network, i, VertArr, visited, spanVertArr, spanArcMap);

			if (spanArcMap.size() > 0)
				for (auto v1 : spanVertArr)
					for (auto v2 : spanVertArr)
						if (v1 < v2)
							if (network.ArcMap[std::make_tuple(v1, v2)] && !spanArcMap[std::make_tuple(v1, v2)])
							{
								//std::cout << "edges" << std::endl;
								//std::cout << v1 << " " << v2 << std::endl;
								spanArcMap[std::make_tuple(v1, v2)] = true;
								spanArcMap[std::make_tuple(v2, v1)] = true;

								bool isDone = false;
								std::vector<bool> visited2(spanVertArr.size(), false); //thats for cycle search
								std::vector<int> cycleArr;
								cycleArr.reserve(spanVertArr.size());

								if (findCyclicUtil(0, -1, spanVertArr, visited2, cycleArr, isDone, spanArcMap))
								{
									cycleMx[cnt] = cycleArr;
									cnt++;

									if (cnt >= numCycle)
										return true;
								}

								spanArcMap.erase(std::make_tuple(v1, v2));
								spanArcMap.erase(std::make_tuple(v2, v1));
							}
		}


	if (cnt > 0)
		return true;
	else
		return false;
}