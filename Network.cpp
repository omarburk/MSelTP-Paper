#include "Network.h"

Vertex::Vertex()
{
	Exist = false;
	c = -1;
	v = -1;
}

void Arc::AddVertexToArc(Vertex& v_i, Vertex& v_j)
{
	i = v_i;
	j = v_j;
}

Cluster::Cluster()
{
	c = -1;
	size = 0;
}

Network::Network()
{
	NumClust = 0;
	NumVert = 0;
	NumArc = 0;
}

void Network::CreateClusterInNetwork(std::vector<int> clustSizes)
{
	int size = clustSizes.size();
	NumClust = size;
	ClustSet.resize(size);

	for (auto n : clustSizes)
		NumVert += n;

	AllVertexSet.reserve(NumVert);

	int inStart = 0;
	for (int i = 0; i < size; i++)
	{
		Cluster& inClust = ClustSet[i];
		inClust.c = i;
		inClust.AddVertexToCluster(clustSizes[i], inStart);
		AllVertexSet.insert(AllVertexSet.end(), std::begin(inClust.VertexSet), std::end(inClust.VertexSet));
		inStart += clustSizes[i];
	}	
}

void Cluster::AddVertexToCluster(int NumVertex, int startID)
{
	size = NumVertex;
	VertexSet.resize(NumVertex);
	for (int i = 0; i < NumVertex; i++)
	{
		Vertex& inVertex = VertexSet[i];
		inVertex.Exist = true;
		inVertex.c = c;
		inVertex.v = i + startID;
	}
}

void Network::AddArcOnNetwork(int v1, int v2)
{
	Arc tempArc;
	if (v1 < AllVertexSet.size() && v2 < AllVertexSet.size())
	{
		if (v1 > v2)
			swapTwoInt(v1, v2);

		Vertex& vert1 = AllVertexSet[v1];
		Vertex& vert2 = AllVertexSet[v2];
		tempArc.AddVertexToArc(vert1, vert2);
		ArcSet.push_back(tempArc);

		ArcMap[std::make_tuple(v1, v2)] = true;
		ArcMap[std::make_tuple(v2, v1)] = true;
	}
}

void swapTwoInt(int& x, int& y)
{
	int temp = x;
	x = y;
	y = temp;
}