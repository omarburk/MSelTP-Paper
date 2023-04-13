#pragma once

#include <vector>
#include <map>

#include <ilcplex/ilocplex.h>
#include "VarDecl.h"
#include "ConstFunctions.h"

void SolveIPFlowOpt(Network& network, std::vector<int>& VertArr, std::vector<int>& ArcArr, std::vector<double>& info);
