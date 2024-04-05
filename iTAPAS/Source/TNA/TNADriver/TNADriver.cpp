#include "..\TNA\header\stdafx.h"
#include <algorithm>
#include <math.h>
#include <iostream>
#include <tchar.h>

using namespace std;




int TestiTAPAS(const string& path, const string& name)
{
	TNM_FloatFormat::SetFormat(18, 4);
	TAP_iTAPAS* itapas = new TAP_iTAPAS;
	//itapas->SetCostScalar(60);
	itapas->SetConv(1e-12);
	itapas->SetInnerConv(1e-12);
	itapas->totalExcessPrecision = 1e-12;
	itapas->SetCostCoef(1.0, 0.00); //link generalized cost = time_factor * Link travel time  + distance factor * distance; 1.0 is time factor and 0.0 is distance_factor
	
		
	itapas->DoProportionality = false;
	itapas->SetCostScalar(60);
	itapas->SetMaxInnerIter(20);
	itapas->SetMaxIter(100);
	itapas->SetLPF(BPRLK);
	itapas->EnablePrintVersion();
	//
	string input, output;


	input = path + name;
	output = path + name + "_iTAPAS";
	if (itapas->Build(input, output, NETTAPAS) != 0)
	{
		//cout<<"\tFail to build network object"<<endl;
		return 0;
	}



	itapas->network->ScaleDemand(1.0);
	itapas->reportIterHistory = true;
	itapas->reportLinkDetail = true;
	itapas->SetCentroidsBlocked(false);  //to determine whether the shortest path is permitted to traverse through the centroids.

	itapas->EnableReportOrgFlows(true);
	//	bob->EnableReportInnerIters();
	itapas->EnableEvRGap();
	itapas->SetTollType(TT_NOTOLL);

	if (itapas->Solve() == ErrorTerm)  //solve traffic assignment problem
	{
		//cout<<"something wrong happend in solving the problem"<<endl;
		return	0;
	}
	//TNM_FloatFormat::SetFormat(16, 8);
	itapas->PrepareReport();
	itapas->Report();
	itapas->GeneratePathbyDFS();
	itapas->SavePathtoFile();
	system("PAUSE");
	delete itapas;

	return 0;
}

int main()
{
	string cs="ChicagoSketch";
	string sf = "SiouxFalls";
	string cr = "ChicagoRegional";


	TestiTAPAS("..\\..\\..\\Network\\sf\\", sf);
};