#pragma once
#include<algorithm>
#include<iostream>
#include<fstream>
#include<vector>
#include<string>
  
class GlobalParameter
{
public:
	//include all the global parameter

	clock_t time_begin;
	clock_t time_new_best;
	clock_t time_first_better;
	std::string instance_path;
	bool better_done = false;

	int _the_run_time = 0;

	const double Precision = 1e-5;

	std::ifstream Input;// ("dofile.txt", std::ios::in);
	std::ofstream allout;// (para.instance_path + "/Result.txt", std::ios::out);
	std::string File_Name;
	double d_orig[1000][2] = { 0 };		//whichpoint+delivery
	double p_orig[1000][2] = { 0 };

	double total_d;
	double total_p;

	double d[1000] = { 0 };			//pair's delivery demand
	double p[1000] = { 0 };			//pair's pickup demand
	int Which[1000] = { 0 };		//record which point

	double Dist[110][110];
	double Position[110][2];	//x y location information 
	int Vehicle;
	double Best;
	double Time;
	int Dimension;				//number of points
	int Num;					//number of pairs,including depot pair
	double Capacity;

	int near_numb;		// number of the neareast point. never use this

	int Each_Run_Time;

	std::vector<int>Mark_Current;
	std::vector<int>Mark_Best;

	int ifBetter = 0;		//0-> lose		 1-> draw		2-> win
	int count_win = 0;
	int count_draw = 0;
	int count_lose = 0;


	std::string Node_mode;

	int Initial_Pop_Size = 10;
	const int MaxNoChangeBest = 3e4;//default
	const int DisturbTimes = 5; 

	GlobalParameter()
	{
		Input = std::ifstream("dofile.txt", std::ios::in);
	}
  };