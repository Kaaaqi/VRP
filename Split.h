#pragma once
#include"Global_Parameter.h"
#include"Individual.h"
#include<vector>
struct ClientSplit
{
	double demandD;
	double demandP;
	double d0_x;
	double dnext;
	bool PbiggerD;
	ClientSplit() : demandD(0.), demandP(0.), d0_x(0.), dnext(0.), PbiggerD(false) {};
};
class Deque
{
public:
	std::vector<int > dq;	//i denotes index of the position in the chrom
	int head;
	int tail;
	void pop_front() { head++; }
	void pop_back() { tail--; }
	void push_back(int i) { dq[++tail]=i; }
	int get_front() { return dq[head]; }
	int get_front(int i) { return dq[head + i]; }
	int get_back() { return dq[tail]; }
	void reset(int k) { dq[0] = k, head = 0, tail = 0; }
	Deque(const GlobalParameter& para) {
		dq = std::vector<int>(para.Num, 0);
		head = 0;
		tail = 0;
	}
	Deque()
	{//used in local search initialization
		head = 0;
		tail = 0;
	};
	
};
class Split
{
public:
	Deque myDeque;
	GlobalParameter* the_para;
	std::vector < ClientSplit > cliSplit;
	std::vector <std::vector<double> > potential;  // Potential vector
	std::vector < std::vector<int> > pred;  // Indice of the predecessor in an optimal path
	std::vector < double > sumDistance; // sumDistance[i] for i > 1 contains the sum of distances : sum_{k=1}^{i-1} d_{k,k+1}
	std::vector < double > sumDemandD; 
	std::vector < double > sumDemandP;
	
	int max_vehicle;

	//only j>i
	inline double propagate(int i, int j,int k,Deque& mark_dq)
	{	//from (i+1) to (j)
		double test = sumDemandD[j] - sumDemandD[i];
		if (test <= the_para->Capacity + the_para->Precision) 
		{
			if (mark_dq.tail - mark_dq.head >= 0)
			{
				if (test - sumDemandD[mark_dq.get_front()] + sumDemandD[i] 
					+ sumDemandP[mark_dq.get_front()] - sumDemandP[i] <= the_para->Capacity + the_para->Precision)
				{
					return potential[k][i] + sumDistance[j] - sumDistance[i + 1] + cliSplit[i + 1].d0_x + cliSplit[j].d0_x;
				}
				else return 1.e30;
			}
			else return potential[k][i] + sumDistance[j] - sumDistance[i + 1] + cliSplit[i + 1].d0_x + cliSplit[j].d0_x;
		}
		else return 1.e30;
	}

	//check whether j dominate i for all nodes x >=j+1, only j >i
	inline bool dominate(int i, int j,int k)
	{
		return potential[k][j] + cliSplit[j + 1].d0_x - sumDistance[j + 1] 
			<= potential[k][i] + cliSplit[i + 1].d0_x - sumDistance[i + 1] + the_para->Precision;
	}

	inline bool mark_dominate(int i, int j)
	{	//check whether find dominate relationship in the mark deque		
		//calculate load change between i to j     check load[j]>load[i]?   if true, then j dominates i
		return (sumDemandD[j] - sumDemandD[i])
			<= (sumDemandP[j] - sumDemandP[i] + the_para->Precision);
	}
	void run(Individual& indiv, const GlobalParameter& para);	
	int doSplit(Individual& indiv,const GlobalParameter& para);	////if K of this solution is bigger than para.Vehicle, do Limited Fleet Split
	int doLFSplit(Individual& indiv,const GlobalParameter& para);
	Split(GlobalParameter& para);
	Split() = default;
};
