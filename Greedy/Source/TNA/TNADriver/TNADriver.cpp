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
	ofw->SetCostScalar(60);
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

int main(int argc, char* argv[])
{
    std::string networkPath, networkName;

    if (argc > 2) {
        networkPath = argv[1];   // Path containing the TNTP network files e.g., "..\\..\\Network\\sf\\"
        networkName = argv[2];   // Name of the TNTP network files e.g., "SiouxFalls" it will automatically search for _node, _net and _trips
    } else {
        // fallback default
        networkPath = "..\\..\\Network\\cs\\"; // in current GIT structure, the network folder is in greedy\network, that is 2 levels (bin\x64) away not 3 levels away
        networkName = "ChicagoSketch";
    }

    TestGreedy(networkPath, networkName);
    return 0;
};
