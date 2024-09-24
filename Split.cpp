#include"Split.h"
int Split::doSplit(Individual& indiv,const GlobalParameter& para)
{	// normal VRP's Split
	//constructor of split
	for (int i = 1; i < para.Num; i++)
	{
		cliSplit[i].demandD = para.d[indiv.Chrom[i - 1]];
		cliSplit[i].demandP = para.p[indiv.Chrom[i - 1]];
		cliSplit[i].PbiggerD = cliSplit[i].demandP > cliSplit[i].demandD ? true : false;

		cliSplit[i].d0_x = para.Dist[0][para.Which[indiv.Chrom[i - 1]]];
		if (i < para.Num - 1) cliSplit[i].dnext = para.Dist[para.Which[indiv.Chrom[i - 1]]][para.Which[indiv.Chrom[i]]];
		else cliSplit[i].dnext = -1.e30;
		sumDemandD[i] = sumDemandD[i - 1] + cliSplit[i].demandD;
		sumDemandP[i] = sumDemandP[i - 1] + cliSplit[i].demandP;
		sumDistance[i] = sumDistance[i - 1] + cliSplit[i - 1].dnext;
	}

	//main algorithm of split the whole chrom   to get route of Individual
	//1.reinitialize the structure 
	std::vector<std::vector<double>>(max_vehicle + 1, std::vector< double >(para.Num, 1.e30)).swap(potential);
	for (int i = 0; i < para.Num; i++)
	{
		potential[0][i] = 1.e30;
	}
	potential[0][0] = 0;
	//2.start the main algorithm
	Deque dq(para);
	Deque mark_dq(para);	//point need to check     (p>d)
	if (cliSplit[1].PbiggerD)mark_dq.reset(1);
	else mark_dq.tail = -1;
	for (int i = 1; i < para.Num && dq.tail - dq.head >= 0; i++)
	{
		potential[0][i] = propagate(dq.get_front(), i, 0,mark_dq);
		pred[0][i] = dq.get_front();
		if (i < para.Num-1)
		{
			if (potential[0][i] < 1.e29)
			{
				while ((dq.tail - dq.head >= 0) && (dominate(dq.get_back(), i, 0)))dq.pop_back();
				dq.push_back(i);
				if (dq.tail - dq.head == 0) while ((mark_dq.tail - mark_dq.head >= 0) && (mark_dq.get_front() <= i))mark_dq.pop_front();
			}
			if (cliSplit[i+1].PbiggerD)
			{
				while ((mark_dq.tail - mark_dq.head >= 0) && (mark_dominate(mark_dq.get_back(), i + 1)))mark_dq.pop_back();
				mark_dq.push_back(i + 1);
			}
			while ((dq.tail - dq.head >= 0) && (propagate(dq.get_front(), i + 1, 0,mark_dq) > 1.e29))
			{	
				dq.pop_front();
				//if (dq.tail - dq.head < 0)while ((mark_dq.tail - mark_dq.head >= 0) && (mark_dq.get_front() <= i))mark_dq.pop_front();
				if (dq.tail - dq.head < 0)break;
				else while ((mark_dq.tail - mark_dq.head >= 0) && (mark_dq.get_front() <= dq.get_front()))mark_dq.pop_front();
			}
		}
	}
	if (potential[0][para.Num-1] > 1.e29)
		throw std::string("ERROR : no Split solution has been propagated until the last node");
	//Fill the Chrom
	int count = 0;
	for (int i = para.Num-1; i != 0; i = pred[0][i], count++);
	if (count > para.Vehicle)return doLFSplit(indiv, para);//if K is bigger than para.Vehicle, do limited fleet Split
	else
	{
		indiv.cost = potential[0][para.Num - 1];

		std::vector<std::vector<int > >(count, std::vector<int >()).swap(indiv.Route);
		int end = para.Num - 1;
		for (int i = count - 1; i >= 0; i--)
		{
			int begin = pred[0][end];
		
			for (int j = begin; j < end; j++)
			{
				indiv.Route[i].push_back(indiv.Chrom[j]);
			}
			end = begin;		
		}

		return count<=para.Vehicle;		
	}
}

int Split::doLFSplit(Individual& indiv,const GlobalParameter& para)
{	// limited fleet VRPs' linear split
	for (int k = 0; k <= max_vehicle; k++)
	{
		for (int i = 0; i < para.Num; i++)
		{
			potential[k][i] = 1.e30;
		}
	}
	potential[0][0] = 0;
	Deque dq(para);
	Deque mark_dq(para);
	for (int k = 0; k < max_vehicle; k++)
	{	// k represent total (k+1£©vehiles 
		dq.reset(k);
		if (cliSplit[k + 1].PbiggerD)mark_dq.reset(k + 1);
		else
		{
			mark_dq.head = 0;
			mark_dq.tail = -1;
		}
		for (int i = k + 1; i<para.Num&&dq.tail-dq.head>=0;i++)
		{
			potential[k+1][i] = propagate(dq.get_front(), i, k, mark_dq);
			pred[k+1][i] = dq.get_front();
			if (i < para.Num - 1)
			{
				if (potential[k+1][i] < 1.e29)
				{
					while ((dq.tail - dq.head >= 0) && (dominate(dq.get_back(), i, k)))dq.pop_back();
					dq.push_back(i);
					if (dq.tail - dq.head == 0)while ((mark_dq.tail - mark_dq.head >= 0) && (mark_dq.get_front() <= i))mark_dq.pop_front();
				}
				if (cliSplit[i+1].PbiggerD)
				{
					while ((mark_dq.tail - mark_dq.head >= 0) && (mark_dominate(mark_dq.get_back(), i + 1)))mark_dq.pop_back();
					mark_dq.push_back(i+1);
				}
				while ((dq.tail - dq.head >= 0) && (propagate(dq.get_front(), i + 1, k, mark_dq) > 1.e29))
				{	
					dq.pop_front();
					if (dq.tail - dq.head < 0)break;
					else while((mark_dq.tail - mark_dq.head >= 0) && (mark_dq.get_front() <= dq.get_front()))mark_dq.pop_front();
				}
			}
		}
	}
	if (potential[max_vehicle][para.Num-1] > 1.e29)
		throw std::string("ERROR : no Split solution has been propagated until the last node");//Even max_vehicle vechiles cannot handle all the demands ,according to this solution
	
	double minCost = 1.e30;
	int nbRoutes = max_vehicle;
	for (int k = 1; k <= max_vehicle; k++)
		if (potential[k][para.Num - 1] < 1.e29)
		{
			minCost = potential[k][para.Num - 1];
			nbRoutes = k;
			break;
		}
	indiv.cost = minCost;

	std::vector<std::vector<int > >(nbRoutes, std::vector<int>()).swap(indiv.Route);
	int end = para.Num - 1;
	for (int k = nbRoutes - 1; k >= 0; k--)
	{
		int begin = pred[k + 1][end];
		for (int j = begin; j < end; j++)
		{
			indiv.Route[k].push_back(indiv.Chrom[j]);
		}
		end = begin;
	}
	return nbRoutes <= para.Vehicle;
}

void Split::run(Individual& indiv, const GlobalParameter& para)
{
	if (!doSplit(indiv, para))indiv.need_elimination = true;
}

Split::Split(GlobalParameter& para):the_para(&para)
{

	//set a bigger vehicle to confirm split but not throw error
	max_vehicle = 3 + std::max<int>((int)std::ceil(1.3 * para.total_d / para.Capacity), (int)std::ceil(1.3 * para.total_p / para.Capacity));
	
	max_vehicle = std::max<int>(max_vehicle, para.Vehicle);
	//constructor: initialize the structure of split algorithm
	cliSplit = std::vector <ClientSplit>(para.Num);
	sumDistance = std::vector<double>(para.Num, 0.);
	sumDemandD = std::vector<double>(para.Num, 0.);
	sumDemandP = std::vector<double>(para.Num, 0.);
	potential = std::vector < std::vector <double >> (max_vehicle+1, std::vector<double>(para.Num, 1.e30));
	pred = std::vector <std::vector<int>>(max_vehicle + 1, std::vector<int>(para.Num, 0));
};