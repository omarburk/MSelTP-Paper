#pragma once

#include <vector>
#include <map>

struct Vertex
{
	bool Exist;
	int c; //cluster id
	int v; //vertex id

	Vertex();
};

struct Arc
{
	Vertex i;
	Vertex j;

	void AddVertexToArc(Vertex& v_i, Vertex& v_j);
};

struct Cluster
{
	std::vector<Vertex> VertexSet;
	int c;
	int size;

	Cluster();
	void AddVertexToCluster(int NumVertex, int startID);
};

struct Network
{
	std::vector<Cluster> ClustSet;
	std::vector<Vertex> AllVertexSet;
	std::vector<Arc> ArcSet;
	std::map<std::tuple<int, int>, bool> ArcMap;
	int NumClust;
	int NumVert;
	int NumArc;

	Network();
	void CreateClusterInNetwork(std::vector<int> clustSizes);
	void AddArcOnNetwork(int v1, int v2);
};

void swapTwoInt(int& x, int& y);