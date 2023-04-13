#pragma once

#include <vector>
#include <map>

#include <ilcplex/ilocplex.h>
#include "Network.h"

extern double Epsilon;

void SetName(IloExtractable obj, const char* prefix, int i);
void SetName2(IloExtractable obj, const char* prefix, int i, int j);
void SetName3(IloExtractable obj, const char* prefix, int i, int j, int k);

IloBoolVarArray CreateBoolVarArray(IloEnv env, IloInt m, const char* prefix);

typedef std::map<std::tuple<int, int>, IloBoolVar> IloArcMap;
typedef std::map<std::tuple<int, int, int>, IloBoolVar> IloArcFlowMap;
typedef std::map<std::tuple<int, int>, IloIntVar> IloVertFlowMap;

IloArcMap CreateArcMap(IloEnv env, Network& netw, const char* prefix, bool directed);

IloArcFlowMap CreateArcFlowMap(IloEnv env, Network& netw, const char* prefix, bool root);

IloVertFlowMap CreateVertFlowMapOpt(IloEnv env, Network& netw, const char* prefix);