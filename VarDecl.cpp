#include "VarDecl.h"

double Epsilon = 0.001;

void SetName(IloExtractable obj, const char* prefix, int i)
{
	std::string name = prefix;
	name += '_' + std::to_string(i);
	obj.setName(name.c_str());
}

void SetName2(IloExtractable obj, const char* prefix, int i, int j)
{
	std::string name = prefix;
	name += '_' + std::to_string(i);
	name += '_' + std::to_string(j);
	obj.setName(name.c_str());
}

void SetName3(IloExtractable obj, const char* prefix, int i, int j, int k)
{
	std::string name = prefix;
	name += '_' + std::to_string(i);
	name += '_' + std::to_string(j);
	name += '_' + std::to_string(k);
	obj.setName(name.c_str());

}

IloBoolVarArray CreateBoolVarArray(IloEnv env, IloInt m, const char* prefix)
{
	IloBoolVarArray array = IloBoolVarArray(env, m);
	for (int i = 0; i < m; i++)
		SetName(array[i], prefix, i);
	return array;
}

IloArcMap CreateArcMap(IloEnv env, Network& netw, const char* prefix, bool directed)
{
	IloArcMap arcMap;

	for (int i = 0; i < netw.NumArc; i++)
	{
		Arc& arc = netw.ArcSet[i];

		arcMap[std::make_tuple(arc.i.v, arc.j.v)] = IloBoolVar(env);
		SetName2(arcMap[std::make_tuple(arc.i.v, arc.j.v)], prefix, arc.i.v, arc.j.v);

		if (directed)
		{
			arcMap[std::make_tuple(arc.j.v, arc.i.v)] = IloBoolVar(env);
			SetName2(arcMap[std::make_tuple(arc.j.v, arc.i.v)], prefix, arc.j.v, arc.i.v);
		}
	}

	return arcMap;
}

IloArcFlowMap CreateArcFlowMap(IloEnv env, Network& netw, const char* prefix, bool root)
{
	IloArcFlowMap arcFlowMap;
	int start = root; // in opt formulation, no need to discard the root cluster, thus 0.

	for (int i = 0; i < netw.NumArc; i++)
	{
		Arc& arc = netw.ArcSet[i];

		for (int k = start; k < netw.NumClust; k++)
		{
			arcFlowMap[std::make_tuple(arc.i.v, arc.j.v, k)] = IloBoolVar(env);
			arcFlowMap[std::make_tuple(arc.j.v, arc.i.v, k)] = IloBoolVar(env);
			SetName3(arcFlowMap[std::make_tuple(arc.i.v, arc.j.v, k)], prefix, arc.i.v, arc.j.v, k);
			SetName3(arcFlowMap[std::make_tuple(arc.j.v, arc.i.v, k)], prefix, arc.j.v, arc.i.v, k);
		}
	}

	return arcFlowMap;
}

IloVertFlowMap CreateVertFlowMapOpt(IloEnv env, Network& netw, const char* prefix)
{
	IloVertFlowMap vertFlowMap;

	for (int i = 0; i < netw.NumVert; i++)
	{
		for (int k = 0; k < netw.NumClust; k++)
		{
			vertFlowMap[std::make_tuple(i, k)] = IloIntVar(env, -1, 1);
			SetName2(vertFlowMap[std::make_tuple(i, k)], prefix, i, k);
		}
	}

	return vertFlowMap;
}