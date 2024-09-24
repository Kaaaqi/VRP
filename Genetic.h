#pragma once
#include"Global_Parameter.h"
#include"Individual.h"
#include"Split.h"
#include"Population.h"
#include"LocalSearch.h"
class Genetic
{
public:
	Population Pop;
	Split Sp;
	LocalSearch Ls;
	Individual* p1;
	Individual* p2;
	Individual* Son;
	GlobalParameter* the_para;
	std::vector<Node>Node_Vec;			// vector to store pairs

	int best_index = -1;

	int NoChangeBest = 1;				// the number of times in which the best solution has not been changed

	void ToNode(GlobalParameter& para);
	void GetNodeInformFromRoute(const Individual& indiv);
	void RenewChrom(Individual& Indiv);
	void Output(GlobalParameter& para);


	void Initialization(Population& pop, GlobalParameter& para);
	void SelectParents(GlobalParameter& para);
	void CrossOver(Individual& son, Individual* parent1, Individual* parent2);
	void PopManagement();

	Genetic(GlobalParameter&para);		// main running function
};