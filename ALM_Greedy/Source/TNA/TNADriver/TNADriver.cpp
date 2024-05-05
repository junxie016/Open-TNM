#include "..\TNA\header\stdafx.h"
#include <algorithm>
#include <math.h>
#include <iostream>
#include <tchar.h>

using namespace std;

int TestCUP(const string& InPath, const string& OutPath, const string& name)
{
	TNM_FloatFormat::SetFormat(18, 6);
	CUP_LY* cup = new CUP_LY;
	string input, output;
	input = InPath + name;
	output = OutPath + name;
	cup->SetLPF(BPRLK);
	if (cup->Build(input, output, NETTAPAS) != 0)
	{
		cout << "\tFail to build network object" << endl;
		return 0;
	}
	
	if (!cup->CyclicUnblockPath())
	{
		cout << "\t" << endl;
		return 0;
	}
	cup->RewriteNetFile(input, output);
	cout << "\n\tFinish the algorithm!" << endl;
	system("PAUSE");
	delete cup;
	return 0;
}

int TestALM_Greedy( const string &path, const string &name)
{
	TNM_FloatFormat::SetFormat(18,6);
	CTAP_ALM_Greedy *ofw = new CTAP_ALM_Greedy;
	/*Set up parameters*/
	ofw->SetLPF(BPRLK);
	ofw->SetKapa(2.0);//Parameter for updating penalty coefficient
	ofw->SetBeta(0.7);//Parameter for updating penalty coefficient
	ofw->SetRGCriterion(1e-6);//Convergence criterion
	ofw->SetFICriterion(1e-6);//Convergence criterion
	ofw->SetGAPCriterion(1e-8);//Convergence criterion
	ofw->SetMaxIter(100);//Maximum allowed iteration number of Main-Loops
	ofw->SetSubConvCriterion(1e-6);//Termination criterion for Sub-Loops
	ofw->SetMaxSubIter(5);//Maximum allowed iteration number of Sub-Loops
	ofw->SetCostCoef(1.0,0.00); //link generalized cost = time_factor * Link travel time  + distance factor * distance; 1.0 is time factor and 0.0 is distance_factor

	string input, output;
	input = path +   name;
	output = path  + name + "_ALM_GREEDY";

	/*Build the network*/
	if(ofw->Build(input, output,NETTAPAS)!=0)
	{
		cout<<"\tFail to build network object"<<endl;
		return 0;
	}
	
	ofw->SetTollType(TT_NOTOLL); 
	ofw->SetCentroidsBlocked(false);  //to determine whether the shortest path is permitted to traverse through the centroids.

	/*Solve the problem*/
	if(ofw->Solve()== ErrorTerm)  //solve traffic assignment problem
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
	string cr_truck="chi";
	string cs="ChicagoSketch";
	string sf = "SiouxFalls";
	string hearn = "hearn";

	
	TestALM_Greedy("..\\..\\..\\Network\\cs\\", cs); //"..\\..\\..\\Network\\sf\\",sf

	//TestCUP("..\\..\\..\\Network\\testCUP\\cs\\", "..\\..\\..\\Network\\testCUP\\temp\\", cs); // use the CUP algorithm to revise the link capacity
																								// Parameters of this function: (a) address of the original network data
																								//								(b) address of the revised network date
																								//								(c) network name
};