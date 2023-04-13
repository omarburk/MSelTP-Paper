#include <iomanip>
#include <numeric>
#include <iostream>
#include <fstream>
#include <map>
#include <chrono>
#include <random>

#include "Network.h"
#include "FlowModelOpt.h"
#include "DecomModelOpt_Callback_MultiCycles.h"
#include "DecomModelOptWoY_Callback_MultiCycles.h"

void setClusterData(std::string filename, Network& netw)
{
	std::fstream my_file;
	my_file.open(filename, std::ios::in);
	if (!my_file)
	{
		std::cout << "No such file";
	}
	else if (netw.NumClust > 0)
	{
		std::cout << "Not empty cluster in the network";
	}
	else
	{
		std::vector<int> clustSizes;
		int data;
		while (my_file >> data)
		{
			clustSizes.push_back(data);
		}
		netw.CreateClusterInNetwork(clustSizes);
	}
}

void setArcData(std::string filename, Network& netw)
{
	std::fstream my_file;
	my_file.open(filename, std::ios::in);
	if (!my_file)
	{
		std::cout << "No such file";
	}
	else if (netw.NumClust == 0)
	{
		std::cout << "No cluster in the network";
	}
	else
	{
		int count = 0;
		int data[2];
		while (my_file >> data[0] >> data[1])
		{
			netw.AddArcOnNetwork(data[0], data[1]);
			count += 1;
		}
		netw.NumArc = count;
	}
}

void printDetail(Network& netw)
{
	std::cout << " num arc = " << netw.NumArc << " num vert = " << netw.NumVert << " num clust " << netw.NumClust << std::endl;

	for (int i = 0; i < netw.NumClust; i++)
	{
		Cluster& clust = netw.ClustSet[i];
		for (int j = 0; j < clust.size; j++)
		{
			Vertex& vert = clust.VertexSet[j];
			std::cout << " cluster id = " << vert.c << " vertex id = " << vert.v << std::endl;
		}
	}

	for (int i = 0; i < netw.NumVert; i++)
	{
		Vertex& vert = netw.AllVertexSet[i];
		std::cout << " cluster id = " << vert.c << " vertex id = " << vert.v << std::endl;
	}

	for (int i = 0; i < netw.NumArc; i++)
	{
		Arc& arc = netw.ArcSet[i];
		std::cout << " vertex1 cluster, id = " << arc.i.c << ", " << arc.i.v << " vertex2 cluster, id = " << arc.j.c << ", " << arc.j.v << std::endl;

		std::cout << netw.ArcMap[std::make_tuple(arc.i.v, arc.j.v)] << std::endl;
	}
}

void isConn(Network& network, int indV, std::vector<int>& VertArr, std::vector<bool>& visited, int& cnt)
{
	int v1 = VertArr[indV];
	int v2;
	visited[indV] = true;
	cnt++;

	if (cnt == VertArr.size())
		return;

	for (int i = 0; i < VertArr.size(); i++)
	{
		if (i != indV && !visited[i])
		{
			v2 = VertArr[i];

			if (network.ArcMap[std::make_tuple(v1, v2)])
			{
				isConn(network, i, VertArr, visited, cnt);
				if (cnt == VertArr.size())
					return;
			}
		}
	}
}

bool isConnRec(Network& network, std::vector<int>& VertArr, size_t index)
{
  int cnt = 0;

  if (index == VertArr.size()) {
    std::vector<bool> visited(VertArr.size(), false);
    cnt = 0;

    isConn(network, 0, VertArr, visited, cnt);

    if (cnt == network.NumClust)
      return true;
    else
      return false; 
  }
  else
  {
    for (auto vert : network.ClustSet[index].VertexSet) {
      VertArr[index] = vert.v;
      if (isConnRec(network, VertArr, index + 1))
        return true;
    }
  }

  return false;
}

void createRandData(int clustSize, int totalVertSize, double density, std::string fileCl, std::string fileArc)
{
	clustSize--; //minus 1 is needed
	Network network;

	while (true)
	{
		long vertSize = 0;
		int edgeCnt = 0;

		Network netw;
		std::vector<int> clustSizesSet;

		std::vector<int> vertArr;
		std::vector<int> sepArr(clustSize);
		std::vector<int> seq((size_t)(totalVertSize)-1);
		std::iota(seq.begin(), seq.end(), 1);

		auto gen = std::mt19937{ std::random_device{}() };
		std::sample(seq.begin(), seq.end(),
			sepArr.begin(),
			clustSize,
			gen);

		for (int i = 0, prev = 0; i <= clustSize; prev = i, i++)
		{
			if (i == 0)
				vertSize = sepArr[i];
			else if (i == clustSize)
				vertSize = totalVertSize - sepArr[prev];
			else
				vertSize = sepArr[i] - sepArr[prev];

			for (int j = 0; j < vertSize; j++)
				vertArr.push_back(i);

			clustSizesSet.push_back(vertSize);
		}
		netw.CreateClusterInNetwork(clustSizesSet);

		std::uniform_real_distribution<double> dist(0.0, 1.0);

		for (int i = 0; i < totalVertSize - 1; i++)
			for (int j = i + 1; j < totalVertSize; j++)
				if (vertArr[i] != vertArr[j])
					if (dist(gen) < density)
					{
						netw.AddArcOnNetwork(i, j);
						edgeCnt += 1;
					}
		netw.NumArc = edgeCnt;

		//check if the clusters are connected (there is a path having a vertex on each cluster) 
		std::vector<int> VertArr(netw.NumClust);
		if (isConnRec(netw, VertArr, 0))
		{
			network = netw;
			break;
		}
	}

	std::fstream cl_file;
	std::fstream arc_file;
	cl_file.open(fileCl, std::ios::out);
	arc_file.open(fileArc, std::ios::out);

	if (cl_file.is_open() && arc_file.is_open())
	{

		for (int i = 0; i < network.NumClust; i++)
			cl_file << network.ClustSet[i].size << std::endl;

		for (int i = 0; i < network.NumArc; i++)
		{
			Arc& arc = network.ArcSet[i];
			arc_file << arc.i.v << '\t' << arc.j.v << std::endl;
		}

		cl_file.close();
		arc_file.close();
	}
}

void RandSet()
{
	std::vector<int> clIns{ 5,10,15,20,25 };
	std::vector<int> vrIns{ 3,6,10 };
	std::vector<double> dnIns{ 0.1,0.3,0.5,0.7 };

	std::string flName, flCl, flArc;
	for (auto i : clIns)
		for (auto j : vrIns)
			for (auto k : dnIns)
			{
				std::ostringstream Convert;
				Convert << std::fixed << std::setprecision(1) << k;
				std::string dec = Convert.str();

				flName = "RandomData/Cluster " + std::to_string(i) + "/avg vert " + std::to_string(j) + "/avg density " + dec;
				for (int p = 1; p <= 10; p++)
				{
					flCl = flName + "/clusters_" + std::to_string(p);
					flArc = flName + "/arcs_" + std::to_string(p);
					createRandData(i, i * j, k, flCl, flArc);
				}
			}
}

void Run(std::string algo, int i, int j, int k, int p)
{
	Network network;
	std::string flCl = "RandomData/Cluster " + std::to_string(i) + "/avg vert " + std::to_string(j) + "/avg density 0." + std::to_string(k) + "/clusters_" + std::to_string(p);
	std::string flArc = "RandomData/Cluster " + std::to_string(i) + "/avg vert " + std::to_string(j) + "/avg density 0." + std::to_string(k) + "/arcs_" + std::to_string(p);
	setClusterData(flCl, network);
	setArcData(flArc, network);

	std::vector<double> info(12, 0);
	std::vector<int> VertArr(network.NumVert, 0);
	std::vector<int> ArcArr(network.NumArc, 0);
	std::vector<double> ArcArr2(network.NumArc, 0);


	if (algo == "Flow_opt")
		SolveIPFlowOpt(network, VertArr, ArcArr, info);
	else if (algo == "Decom_opt_cb_mult")
		SolveIPDecomOpt_Callback_Mult(network, VertArr, ArcArr, info);
	else if (algo == "Decom_opt_wo_y_cb_mult")
		SolveIPDecomOptWoY_Callback_Mult(network, VertArr, ArcArr, info);
	else
		return;

	int ExpRes = 0;
	
	std::fstream my_file;
	std::string res = "Results/" + algo + "/Cluster_" + std::to_string(i) + "_avgVert_" + std::to_string(j) + "_avgDensity_0." + std::to_string(k) + "_instance_" + std::to_string(p);

	my_file.open(res, std::ios::out);
	if (my_file.is_open())
	{
		if (info[0])
			my_file << "optimum " << std::endl;
		else if (info[2] == -1)
			my_file << "infeasible " << std::endl;
		else
			my_file << "feasible " << std::endl;

		my_file << "Exp = " << ExpRes;
		my_file << " LB, UB = " << info[1] << ", " << info[2] << " Gap %: " << info[3] << " No. of iterations: " << info[4] << " No. of LazyCuts" << info[5] << " Algo Dur." << info[6] << std::endl;

		if (info[2] != -1)
		{
			my_file << "Vertex array:" << std::endl;
			for (auto i : VertArr)
				my_file << i << ' ';
			my_file << std::endl;

			my_file << "Edge array:" << std::endl;
			for (auto i : ArcArr)
				my_file << i << ' ';
			my_file << std::endl;
		}

		my_file << "Duration: " << info[11] << " seconds" << std::endl;

		my_file.close();
	}

	std::fstream my_file2;
	std::string resAll = "Results/" + algo + "/all";;

	my_file2.open(resAll, std::ios::out | std::ios::app);
	if (my_file2.is_open())
	{
		my_file2 << i << "\t" << i * j << "\t" << k << "\t" << p << "\t" << ExpRes << "\t";
		if (info[0])
			my_file2 << "opt" << "\t";
		else if (info[2] == -1)
			my_file2 << "infeas" << "\t";
		else
			my_file2 << "feas" << "\t";

		for (auto i : info)
			my_file2 << i << "\t";

		my_file2 << std::endl;;

		my_file2.close();
	}
}

int main()
{
	//RandSet();

	std::vector<int> clIns{ 5,10,15,20,25 };
	std::vector<int> vrIns{ 3,6,10 };
	std::vector<int> dnIns{ 1,3,5,7 };
	std::string algo = "";
	
	algo = "Flow_opt";
	for (auto i : clIns)
		for (auto j : vrIns)
			for (auto k : dnIns)
				for (int p = 1; p <= 10; p++)
					Run(algo, i, j, k, p);

	algo = "Decom_opt_wo_y_cb_mult";
	for (auto i : clIns)
		for (auto j : vrIns)
			for (auto k : dnIns)
				for (int p = 1; p <= 10; p++)
					Run(algo, i, j, k, p);

	algo = "Decom_opt_cb_mult";
	for (auto i : clIns)
		for (auto j : vrIns)
			for (auto k : dnIns)
				for (int p = 1; p <= 10; p++)
					Run(algo, i, j, k, p);

	return 0;
}
