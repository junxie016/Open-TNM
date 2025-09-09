#include "..\TNA\header\stdafx.h"
#include <algorithm>
#include <math.h>
#include <iostream>
#include <tchar.h>

using namespace std;



int TestGreedy( const string &path, const string &name)
{
	TNM_FloatFormat::SetFormat(18,6);
	TAP_Greedy *ofw = new TAP_Greedy;
	
	/*Set up parameters*/
	ofw->SetConv(1e-10);//Convergence criterion
	ofw->SetMaxIter(500);//Maximum allowed iteration number
	ofw->SetLPF(BPRLK);
	//ofw->SetCostScalar(60);
	ofw->SetCostCoef(1.0,0.00); //link generalized cost = time_factor * Link travel time  + distance factor * distance; 1.0 is time factor and 0.0 is distance_factor

	string input, output;
	input = path +   name;
	output = path  + name + "_Greedy";

	/*Build the network*/
	if(ofw->Build(input, output,NETTAPAS)!=0)
	{
		cout<<"\tFail to build network object"<<endl;
		return 0;
	}

	ofw->SetTollType(TT_NOTOLL); 
	ofw->SetCentroidsBlocked(false);  //to determine whether the shortest path is permitted to traverse through the centroids.

	/*Solve the problem*/
	if(ofw->Solve()== ErrorTerm) 
	{
		cout<<"something wrong happend in solving the problem"<<endl;
		return	0;
	}

	cout<<"OFV="<<ofw->OFV<<endl;

	/*Write result files*/
	ofw->reportIterHistory=true;
	ofw->reportLinkDetail=true;
	ofw->reportPathDetail=true;
	ofw->Report();

	system("PAUSE");

	delete ofw;
	return 0;

	
}

int main()
{
	string cs="ChicagoSketch";
	string sf = "SiouxFalls";

	
	TestGreedy("..\\..\\..\\Network\\sf\\",sf); //"..\\..\\..\\Network\\sf\\",sf
};