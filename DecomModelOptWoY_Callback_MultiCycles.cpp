#include "DecomModelOptWoY_Callback_MultiCycles.h"

ILOLAZYCONSTRAINTCALLBACK5(LazyCallbackWoYMult, Network&, network, IloBoolVarArray&, x, IloBoolVarArray&, t, std::vector<int>&, counter, std::vector<double>&, algoDura)
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

	std::vector<int> ClustArr; // store indices of the nonselected clusters in order
	ClustArr.reserve(network.NumClust);

	IloNumArray vertSol(Menv, network.NumVert);
	getValues(vertSol, x);

	IloNumArray clSol(Menv, network.NumClust);
	getValues(clSol, t);


	for (int i = 0; i < network.NumVert; i++)
		if (vertSol[i] > Epsilon)
			VertArr.push_back(i);

	for (int i = 0; i < network.NumClust; i++)
		if (clSol[i] > Epsilon)
			ClustArr.push_back(i);

	for (int i = 0; i < network.NumArc; i++)
	{
		Arc& arc = network.ArcSet[i];
		if (vertSol[arc.i.v] > Epsilon && vertSol[arc.j.v] > Epsilon)
			numArc++;
	}
	/*
	for (auto i : VertArr)
		Menv.out() << i << " ";
	Menv.out() << "\n";
	*/

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
			//Menv.out() << cyc.size() << " ";
			if (cyc.size() > 0)
			{
				counter[1]++;
				IloExpr ExpCut(Menv);
				IloNum RHS = cyc.size() - 1;

				for (auto i : cyc) // cycleArr stores vertices of cycle
					ExpCut += x[i];
				/*
				for (auto i : cyc)
					Menv.out() << i << " ";
				Menv.out() << "\n";
				*/
				add(ExpCut <= RHS).end();
				ExpCut.end();
			}
		}
		//Menv.out() << "\n";
		time_span = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - stTimeCut);
		algoDura[1] += time_span.count();
	}
	else
	{
		obj = getObjValue();
		double diff = network.NumClust - 1 - obj - numArc;
		if (abs(diff) > Epsilon)
		{
			counter[2]++;
			auto stTimeCut2 = std::chrono::steady_clock::now();

			IloExpr ExpCut(Menv);
			IloNum VertSelBnd = network.NumClust - 1;

			for (auto& i : VertArr) // store indices of the selected vertices in order
				ExpCut += x[i];

			for (auto& i : ClustArr) // store indices of the non selected clusters in order
				ExpCut += t[i];

			add(ExpCut <= VertSelBnd).end();
			ExpCut.end();

			time_span = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - stTimeCut2);
			algoDura[2] += time_span.count();
		}
	}

	vertSol.end();
	clSol.end();

	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - stTime);
	algoDura[0] += time_span.count();

	return;
}

void SolveIPDecomOptWoY_Callback_Mult(Network& network, std::vector<int>& FinalVertArr, std::vector<int>& ArcArr, std::vector<double>& info)
{
	// Main feasibility problem
	IloEnv Menv;
	IloModel Mmodel(Menv);

	IloBoolVarArray x = CreateBoolVarArray(Menv, network.NumVert, "x");
	IloBoolVarArray t = CreateBoolVarArray(Menv, network.NumClust, "t");

	IloObjective Mobjective = IloMinimize(Menv, IloSum(t));
	Mmodel.add(Mobjective);

	OneVertexPerClusterOpt(network, Menv, Mmodel, x, t);

	IloCplex Mcplex(Mmodel);

	//Mcplex.setParam(IloCplex::Param::TimeLimit, 1500);
	Mcplex.setParam(IloCplex::Param::ClockType, 1);
	Mcplex.setParam(IloCplex::Param::TimeLimit, 1500);
	Mcplex.setParam(IloCplex::Param::MIP::Tolerances::AbsMIPGap, 0.9999);
	Mcplex.setParam(IloCplex::Param::MIP::Display, 0);

	std::vector<int> counter(3, 0);
	std::vector<double> algoDura(3, 0.0);

	Mcplex.use(LazyCallbackWoYMult(Menv, network, x, t, counter, algoDura));

	auto tbegin = Mcplex.getCplexTime();

	auto res = Mcplex.solve();

	auto tend = Mcplex.getCplexTime();
		
	if (res)
	{
		double UB, LB;

		LB = Mcplex.getBestObjValue();
		UB = Mcplex.getObjValue();

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

		for (int i = 0; i < network.NumArc; i++)
		{
			Arc& arc = network.ArcSet[i];
			if (FinalVertArr[arc.i.v] == 1 && FinalVertArr[arc.j.v] == 1)
				ArcArr[i] = 1;
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
	info[9] = counter[2];
	info[10] = algoDura[2];
	info[11] = tend - tbegin;

	Menv.end();
}