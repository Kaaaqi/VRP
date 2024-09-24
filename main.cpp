#include"Global_Parameter.h"
#include"Individual.h"
#include"Split.h"
#include"Population.h"
#include"LocalSearch.h"
#include"Genetic.h"
#include<iostream>
#include<ctime>
#include <direct.h>
#include<string>
#include<sstream>
#include<iomanip>
using namespace std;
 
GlobalParameter para;


bool Read_File( int& this_run, const int& run_first, const int& run_end, const string& run_mode)
{	// read the instance information
	string File;
	para.Input >> File;
	
	if (this_run < run_first || this_run > run_end)
	{
		this_run++;
		return false;
	}

	//set all the matrix to Zero
	memset(para.d_orig, 0, sizeof(para.d_orig));
	memset(para.p_orig, 0, sizeof(para.p_orig));
	memset(para.d, 0, sizeof(para.d));
	memset(para.p, 0, sizeof(para.p));
	memset(para.Which, 0, sizeof(para.Which));
	memset(para.Dist, 0, sizeof(para.Dist));
	memset(para.Position, 0, sizeof(para.Position));
	para.total_d = 0, para.total_p = 0;
	para.ifBetter = 0;

	
	ifstream Data(File, ios::out);
	string tmp;
	while (tmp != "NAME:")Data >> tmp;
	Data >> para.File_Name;
	//std::cout << para.File_Name << "	";
	while (tmp != "COMMENT:")Data >> tmp; 
	Data >> para.Best;
	while (tmp != "VEHICLE:")Data >> tmp;
	Data >> para.Vehicle;
	while (tmp != "TIME:")Data >> tmp;
	Data >> para.Time;
	//para.Time /= 10;
	while (tmp != "DIMENSION:")Data >> tmp;
	Data >> para.Dimension;
	while (tmp != "CAPACITY:")Data >> tmp;
	Data >> para.Capacity;
	while (tmp != "DELIVERY_SECTION")Data >> tmp;
	Data >> tmp;
	for (int i=0;tmp != "PICKUP_SECTION"; Data >> tmp,i++)
	{// get delivery demands for each d order
		Data >> para.d_orig[i][0] >> para.d_orig[i][1];
		para.total_d += para.d_orig[i][1];
	}
	Data >> tmp;
	for (int i = 0; tmp != "NODE_SECTION"; Data >> tmp, i++)
	{// get pickup demands for each p order
		Data >> para.p_orig[i][0] >> para.p_orig[i][1];
		para.total_p += para.p_orig[i][1];
	}

	para.near_numb =	para.Dimension;
	Data >> tmp;
	if (tmp != "DISTANCE_SECTION")
	{// get position information for each node 
		para.Position[0][0] = atoi(tmp.c_str());
		Data >> para.Position[0][1];
		for (int i = 1; i < para.Dimension; i++)
		{
			Data >> para.Position[i][0] >> para.Position[i][1];
		}

		for (int i = 0; i < para.Dimension; i++)
		{
			for (int j = 0; j < para.Dimension; j++)
			{
				para.Dist[i][j] = std::sqrt(
					(para.Position[i][0] - para.Position[j][0]) * (para.Position[i][0] - para.Position[j][0]) +
					(para.Position[i][1] - para.Position[j][1]) * (para.Position[i][1] - para.Position[j][1])
				);
			}
		}
	}
	else 
	{	
		for (int i = 0; i < para.Dimension - 1; i++)
		{
			for (int j = i + 1; j < para.Dimension; j++)
			{
				Data >> para.Dist[i][j];
				para.Dist[j][i] = para.Dist[i][j];
			}
		}
	}
	while (tmp != "DEPOT_SECTION")Data >> tmp;
	Data >> tmp;
	if (tmp != "0")cout << "Depot is not Zero!" << endl;
	Data >> tmp;
	if (tmp != "EOF")cout << "More data after depot section" << endl;


	if (this_run >= 0 && this_run <= 171 )
	{	
		// Set the running time of the first example set to 60 seconds
		para.Time = para.Time > 60 ? 60 : para.Time;
	}
	return true;
}

void Choose_Run_Mode(int& run_times, int& run_first, int& run_end, string& run_mode)
{	
	// run mode :	1. all:run overall 221 instances		2.choose: set the start and end idx of instance 
	// node mode :	1.combine: combine to pairs			
	// run times :	running times of each instances
	// start idx:	when run mode is "choose",  "start idx" is the index of the first instance of this running
	// end idx:		when run mode is "choose",  "end idx" is the index of the last instance of this running
	cout << "run mode:" << '\t'<< "node mode:" << '\t' << "run times:" << '\t' << "start idx:" << '\t' << "end idx:" << endl;
	cin >> run_mode >> para.Node_mode >> run_times >> run_first >> run_end;
	para.Each_Run_Time = run_times;
	if (run_mode == "all")
	{	
		run_first = 0, run_end = 221;
		return;
	}
	if (run_mode == "choose") {}
	else
	{
		throw std::string("ERROR : Not a right mode");
	}
}

void CreatePath() {
	// Establish folder storage results based on running time
	std::string dir_path = "./Result/";
	std::time_t current_time = std::time(nullptr);
	struct tm tm;
	localtime_s(&tm, &current_time);
	std::stringstream ss;
	ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
	para.instance_path = ss.str();
	dir_path += para.instance_path;
	para.instance_path = dir_path;
	int status = _mkdir(dir_path.c_str());
	if (status == 0)
	{
		std::cout << "Created folder with name: " << dir_path << std::endl;
	}
	else
	{
		std::cerr << "Error creating folder with name: " << dir_path << std::endl;
	}
	para.allout = std::ofstream(para.instance_path + "/Result.txt", std::ios::out);
}

void release(Genetic& run)
{	
	// release the memory
	memset(para.p, 0, sizeof(para.p));
	memset(para.d, 0, sizeof(para.d));
	for (int i = 0; i < para.Initial_Pop_Size; i++)
	{
		delete run.Pop.the_indiv[i];
	}
}


int main()
{

	int run_first = -1, run_end = -1, run_times = 0, this_run = 0;
	std::string run_mode;


	Choose_Run_Mode(run_times, run_first, run_end, run_mode);
	// store results in ./Result/
	CreatePath();


	// set random run seed
	srand(unsigned(time));
	while (!para.Input.eof())
	{
		if (!Read_File(this_run, run_first, run_end, run_mode))continue;
		for (para._the_run_time = 0; para._the_run_time < run_times; para._the_run_time++)
		{
			para.allout << this_run << '\t';
			para.better_done = false;
			para.time_begin = clock();	//begin
			Genetic run(para);

			release(run);
		}
		// record win/loss/draw times 
		if (para.ifBetter == 0)para.count_lose++;
		else if (para.ifBetter == 1)para.count_draw++;
		else if (para.ifBetter == 2)para.count_win++;
		this_run++;
	}
	para.allout << std::endl << "Win: " << para.count_win << '\t' << "Draw: " << para.count_draw << '\t' << "Lose: " << para.count_lose;
	std::system("pause");
	return 0;
}