#pragma once

#include <vector>
#include <map>

#include <ilcplex/ilocplex.h>
#include "Network.h"
#include "VarDecl.h"

void EdgeVertexDependancy(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& x, IloArcMap& y);

void EdgeTreeCons(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& t, IloArcMap& y);

void OneVertexPerClusterOpt(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& x, IloBoolVarArray& t);

void SetObjectiveOpt(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& t, IloObjective& objective);

//Flow Constraints

void FlowModelOpt(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& x, IloArcFlowMap& f, IloVertFlowMap& flow, IloBoolVarArray& s, IloBoolVarArray& t);

void EdgeFlowDependancyOpt(Network& network, IloEnv& env, IloModel& model, IloArcMap& y, IloArcFlowMap& f, IloArcMap& w);

void SourceDetermination(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& s, IloBoolVarArray& t);