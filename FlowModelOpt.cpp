#include "FlowModelOpt.h"

//double Epsilon = 0.001;

void SolveIPFlowOpt(Network& network, std::vector<int>& VertArr, std::vector<int>& ArcArr, std::vector<double>& info)
{
	IloEnv env;
	IloModel model(env);

	IloBoolVarArray x = CreateBoolVarArray(env, network.NumVert, "x");
	IloBoolVarArray t = CreateBoolVarArray(env, network.NumClust, "t");
	IloBoolVarArray s = CreateBoolVarArray(env, network.NumClust, "s");
	IloArcMap y = CreateArcMap(env, network, "y", false);
	IloArcFlowMap f = CreateArcFlowMap(env, network, "f", 0);
	IloVertFlowMap flow = CreateVertFlowMapOpt(env, network, "flow");
	IloArcMap w = CreateArcMap(env, network, "w", true);

	IloObjective objective = IloMinimize(env, IloSum(t));
	model.add(objective);

	OneVertexPerClusterOpt(network, env, model, x, t);
	EdgeTreeCons(network, env, model, t, y);
	EdgeVertexDependancy(network, env, model, x, y);
	SourceDetermination(network, env, model, s, t);
	FlowModelOpt(network, env, model, x, f, flow, s, t);
	EdgeFlowDependancyOpt(network, env, model, y, f, w);

	IloCplex cplex(model);
	cplex.exportModel("GenTreeFlowOpt.lp");

	cplex.setParam(IloCplex::Param::ClockType, 1);
	cplex.setParam(IloCplex::Param::TimeLimit, 1500);
	cplex.setParam(IloCplex::Param::MIP::Display, 0);
	cplex.setParam(IloCplex::Param::Threads, 1);
	
	auto tbegin = cplex.getCplexTime();

	auto Success = cplex.solve();

	auto tend = cplex.getCplexTime();

	if (Success)
	{
		double UB = cplex.getObjValue();
		double LB = cplex.getBestObjValue();

		//cout << "UB = " << UB << " LB = " << LB << endl;

		if (abs(LB - UB) < 0.001)
			info[0] = 1;
		else
			info[0] = 0;
		info[1] = LB;
		info[2] = UB;
		info[3] = cplex.getMIPRelativeGap() * 100.0;

		IloNumArray vertSol(env, network.NumVert);
		cplex.getValues(vertSol, x);

		for (int i = 0; i < network.NumVert; i++)
			if (vertSol[i] > Epsilon)
				VertArr[i] = 1;
		vertSol.end();

		IloNum arcSol = 0;

		for (int i = 0; i < network.NumArc; i++)
		{
			Arc& arc = network.ArcSet[i];
			arcSol = cplex.getValue(y[std::make_tuple(arc.i.v, arc.j.v)]);
			if (arcSol > Epsilon)
				ArcArr[i] = arcSol;
		}
	}
	else
	{
		info[0] = 0;
		info[1] = cplex.getBestObjValue();
		info[2] = -1;
		info[3] = 100.0;
	}

	info[4] = cplex.getNiterations();
	info[5] = 0;
	info[6] = 0;
	info[7] = 0;
	info[8] = 0;
	info[9] = 0;
	info[10] = 0;
	info[11] = tend - tbegin;

	env.end();
}
