#include "ConstFunctions.h"

void EdgeVertexDependancy(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& x, IloArcMap& y)
{
	for (int i = 0; i < network.NumArc; i++)
	{
		Arc& arc = network.ArcSet[i];
		IloExtractable constr1 = y[std::make_tuple(arc.i.v, arc.j.v)] <= x[arc.i.v];
		IloExtractable constr2 = y[std::make_tuple(arc.i.v, arc.j.v)] <= x[arc.j.v];
		IloExtractable constr3 = x[arc.i.v] + x[arc.j.v] - 1 <= y[std::make_tuple(arc.i.v, arc.j.v)];
		model.add(constr1);
		model.add(constr2);
		model.add(constr3);
	}
}

void EdgeTreeCons(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& t, IloArcMap& y)
{
	IloExpr Exp(env);

	for (int i = 0; i < network.NumClust; i++)
		Exp += t[i];

	for (int i = 0; i < network.NumArc; i++)
	{
		Arc& arc = network.ArcSet[i];
		Exp += y[std::make_tuple(arc.i.v, arc.j.v)];
	}

	double numTreeEdge = network.NumClust - 1;
	model.add(Exp == numTreeEdge);
	Exp.end();
}

void OneVertexPerClusterOpt(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& x, IloBoolVarArray& t)
{
	for (int i = 0; i < network.NumClust; i++)
	{
		IloExpr Exp(env);
		Cluster& clust = network.ClustSet[i];

		for (int j = 0; j < clust.size; j++)
			Exp += x[clust.VertexSet[j].v];

		Exp += t[i];

		model.add(Exp == 1);
		Exp.end();
	}
}

void SetObjectiveOpt(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& t, IloObjective& objective)
{
	for (int i = 0; i < network.NumClust; i++)
	{
		objective.setLinearCoef(t[i], 1);
	}
}

//Flow Constraints

void FlowModelOpt(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& x, IloArcFlowMap& f, IloVertFlowMap& flow, IloBoolVarArray& s, IloBoolVarArray& t)
{
	int v_id, isDefined;

	for (int k = 0; k < network.NumClust; k++)
	{
		for (int c = 0; c < network.NumClust; c++)
		{
			Cluster& clust = network.ClustSet[c];

			for (int i = 0; i < clust.size; i++)
			{
				v_id = clust.VertexSet[i].v;
				IloExpr flowBalanceExp(env);

				isDefined = 0;
				// Outbound flow from i
				for (int j = 0; j < network.NumVert; ++j)
				{
					if (f.count(std::make_tuple(v_id, j, k)) > 0)
					{
						flowBalanceExp += f.at(std::make_tuple(v_id, j, k));
						isDefined = 1;
					}
				}

				// Inbound flow from i
				for (int j = 0; j < network.NumVert; ++j)
				{
					if (f.count(std::make_tuple(j, v_id, k)) > 0)
					{
						flowBalanceExp -= f.at(std::make_tuple(j, v_id, k));
						isDefined = 1;
					}
				}

				if (isDefined)
					model.add(flowBalanceExp == flow[std::make_tuple(v_id, k)]);

				if (c == k)
				{
					IloExtractable constr1 = flow[std::make_tuple(v_id, k)] <= s[k] - x[v_id];
					IloExtractable constr2 = flow[std::make_tuple(v_id, k)] >= s[k] - 1;
					model.add(constr1);
					model.add(constr2);
				}
				else
				{
					IloExtractable constr1 = flow[std::make_tuple(v_id, k)] <= s[c];
					IloExtractable constr2 = flow[std::make_tuple(v_id, k)] <= 1 - t[k];
					IloExtractable constr3 = flow[std::make_tuple(v_id, k)] >= x[v_id] + s[c] - t[k] - 1;
					IloExtractable constr4 = flow[std::make_tuple(v_id, k)] >= 0;
					model.add(constr1);
					model.add(constr2);
					model.add(constr3);
					model.add(constr4);
				}

				flowBalanceExp.end();
			}
		}
	}
}

void EdgeFlowDependancyOpt(Network& network, IloEnv& env, IloModel& model, IloArcMap& y, IloArcFlowMap& f, IloArcMap& w)
{
	for (int i = 0; i < network.NumArc; i++)
	{
		IloExpr constr3(env);

		Arc& arc = network.ArcSet[i];

		for (int k = 0; k < network.NumClust; k++)
		{
			IloExtractable constr1 = f[std::make_tuple(arc.i.v, arc.j.v, k)] <= w[std::make_tuple(arc.i.v, arc.j.v)];
			IloExtractable constr2 = f[std::make_tuple(arc.j.v, arc.i.v, k)] <= w[std::make_tuple(arc.j.v, arc.i.v)];
			model.add(constr1);
			model.add(constr2);
		}
		constr3 = w[std::make_tuple(arc.i.v, arc.j.v)] + w[std::make_tuple(arc.j.v, arc.i.v)];

		model.add(constr3 == y[std::make_tuple(arc.i.v, arc.j.v)]);
		constr3.end();
	}

}

void SourceDetermination(Network& network, IloEnv& env, IloModel& model, IloBoolVarArray& s, IloBoolVarArray& t)
{
	IloExpr Exp(env);

	for (int k = 0; k < network.NumClust; k++)
	{
		IloExtractable constr = s[k] <= 1 - t[k];
		model.add(constr);

		Exp += s[k];
	}

	model.add(Exp == 1);
	Exp.end();
}