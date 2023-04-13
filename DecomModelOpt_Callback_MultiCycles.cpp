#include "DecomModelOpt_Callback_MultiCycles.h"

ILOLAZYCONSTRAINTCALLBACK6(LazyCallbackMult, Network&, network, IloBoolVarArray&, x, IloBoolVarArray&, t, IloArcMap&, y, std::vector<int>&, counter, std::vector<double>&, algoDura)
{
	counter[0]++;
	std::chrono::duration<double> time_span;
	auto stTime = std::chrono::steady_clock::now();

	double obj = 0;
	bool cyclicInfo;
	int numArc = 0;
	IloEnv Menv = getEnv();

	std::vector<int> VertArr; // store indices of the selected vertices in order
	VertArr.reserve(network.NumClust);

	IloNumArray vertSol(Menv, network.NumVert);
	getValues(vertSol, x);

	for (int i = 0; i < network.NumVert; i++)
		if (vertSol[i] > Epsilon)
			VertArr.push_back(i);

	for (int i = 0; i < network.NumArc; i++)
	{
		Arc& arc = network.ArcSet[i];
		if (vertSol[arc.i.v] > Epsilon && vertSol[arc.j.v] > Epsilon)
			numArc++;
	}

	int numCycle = 75;
	std::vector<std::vector<int>> cycleMx(numCycle); // store vertices form a cycle in its each vector
	for (auto c : cycleMx)
		c.reserve(VertArr.size());

	cyclicInfo = findCycles(network, VertArr, cycleMx); //it also updates cycleMx
	if (cyclicInfo)
	{
		auto stTimeCut = std::chrono::steady_clock::now();

		for (auto cyc : cycleMx)
		{
			if (cyc.size() > 0)
			{
				counter[1]++;
				IloExpr ExpCut(Menv);
				IloNum RHS = cyc.size() - 1;

				int v1, v2;
				for (auto it = cyc.begin(); it != cyc.end(); it++) //cycleArr stores vertices of cycle
				{
					v1 = *it;
					if (it + 1 != cyc.end())
						v2 = *(it + 1);
					else
						v2 = cyc[0];

					if (v1 > v2)
						swapTwoInt(v1, v2);

					ExpCut += y[std::make_tuple(v1, v2)];
				}
				
				add(ExpCut <= RHS).end();
				ExpCut.end();
			}
		}
		time_span = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - stTimeCut);
		algoDura[1] += time_span.count();
	}

	vertSol.end();

	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - stTime);
	algoDura[0] += time_span.count();

	return;
}

void SolveIPDecomOpt_Callback_Mult(Network& network, std::vector<int>& FinalVertArr, std::vector<int>& ArcArr, std::vector<double>& info)
{
	// Main feasibility problem
	IloEnv Menv;
	IloModel Mmodel(Menv);

	IloBoolVarArray x = CreateBoolVarArray(Menv, network.NumVert, "x");
	IloBoolVarArray t = CreateBoolVarArray(Menv, network.NumClust, "t");
	IloArcMap y = CreateArcMap(Menv, network, "y", false);

	IloObjective Mobjective = IloMinimize(Menv, IloSum(t));
	Mmodel.add(Mobjective);

	OneVertexPerClusterOpt(network, Menv, Mmodel, x, t);
	EdgeVertexDependancy(network, Menv, Mmodel, x, y);
	EdgeTreeCons(network, Menv, Mmodel, t, y);

	IloCplex Mcplex(Mmodel);

	Mcplex.setParam(IloCplex::Param::ClockType, 1);
	Mcplex.setParam(IloCplex::Param::TimeLimit, 1500);
	Mcplex.setParam(IloCplex::Param::MIP::Tolerances::AbsMIPGap, 0.9999);
	Mcplex.setParam(IloCplex::Param::MIP::Display, 0);

	std::vector<int> counter(2, 0);
	std::vector<double> algoDura(2, 0.0);

	Mcplex.use(LazyCallbackMult(Menv, network, x, t, y, counter, algoDura));

	auto tbegin = Mcplex.getCplexTime();

	auto res = Mcplex.solve();

	auto tend = Mcplex.getCplexTime();

	if (res)
	{
		double UB, LB;
		//cout << "Optimal solution is found" << endl;
		LB = Mcplex.getBestObjValue();
		UB = Mcplex.getObjValue();
		//std::cout << "fin " << Mcplex.getBestObjValue() << " " << Mcplex.getObjValue() << std::endl;

		if (abs(LB - UB) < Epsilon)
			info[0] = 1;
		else
			info[0] = 0;
		info[1] = LB;
		info[2] = UB;
		info[3] = Mcplex.getMIPRelativeGap() * 100.0;
		
		IloNumArray vertSol(Menv, network.NumVert);
		Mcplex.getValues(vertSol, x);

		for (int i = 0; i < network.NumVert; i++)
			if (vertSol[i] > Epsilon)
				FinalVertArr[i] = 1;
		vertSol.end();

		IloNum arcSol = 0;

		for (int i = 0; i < network.NumArc; i++)
		{
			Arc& arc = network.ArcSet[i];
			arcSol = Mcplex.getValue(y[std::make_tuple(arc.i.v, arc.j.v)]);
			if (arcSol > Epsilon)
				ArcArr[i] = arcSol;
		}
	}
	else
	{
		info[0] = 0;
		info[1] = Mcplex.getBestObjValue();
		info[2] = -1;
		info[3] = 100.0;
	}

	info[4] = Mcplex.getNiterations();
	info[5] = counter[0];
	info[6] = algoDura[0];
	info[7] = counter[1];
	info[8] = algoDura[1];
	info[9] = 0;
	info[10] = 0;
	info[11] = tend - tbegin;

	//Mcplex.exportModel("GenTreeDecomOptCbM.lp");

	Menv.end();
}