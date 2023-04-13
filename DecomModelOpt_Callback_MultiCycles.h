#pragma once

#include <vector>
#include <map>
#include <chrono>

#include <ilcplex/ilocplex.h>
#include "Network.h"
#include "VarDecl.h"
#include "Algos.h"
#include "ConstFunctions.h"

void SolveIPDecomOpt_Callback_Mult(Network& network, std::vector<int>& FinalVertArr, std::vector<int>& ArcArr, std::vector<double>& info);
