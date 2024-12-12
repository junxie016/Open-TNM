#include "..\TNA\header\stdafx.h"
#include <algorithm>
#include <math.h>
#include <iostream>
#include <tchar.h>

using namespace std;

int TestiGP( const string &path, const string &name)
{
	TNM_FloatFormat::SetFormat(18,6);
	TAP_iGP *ofw = new TAP_iGP;
	ofw->SetCostScalar(60);
	ofw->SetConv(1e-10);
	ofw->SetInnerConv(1e-10);
	ofw->SetMaxLsIter(15);
	ofw->SetMaxInnerIter(20);
	ofw->SetMaxIter(500);
	ofw->SetLPF(BPRLK);
	//ofw->network->ScaleDemand(0.01);

	string input, output;
	input = path +   name;
	output = path  + name + "_iGP";

	if(ofw->Build(input, output,NETTAPAS)!=0)
	{
		cout<<"\tFail to build network object"<<endl;
		return 0;
	}

	ofw->SetTollType(TT_NOTOLL); 

	if(ofw->Solve()== ErrorTerm)  //solve traffic assignment problem
	{
		cout<<"something wrong happend in solving the problem"<<endl;
		return	0;
	}

	//vector<TNM_SLINK*> linkv = ofw->network->linkVector;
	//for(int llcount=0;llcount<=20;llcount++)//linkv.size()
	//{
	//	cout<<"ID"<<linkv[llcount]->id<<"head"<<linkv[llcount]->head->id<<"tail"<<linkv[llcount]->tail->id<<"flow"<<linkv[llcount]->volume<<endl;
	//}


	/*cout<<"OFV="<<ofw->OFV<<endl;
	for(int a=0;a<ofw->network->numOfLink;a++)
	{
		TNM_SLINK* link=ofw->network->linkVector[a];
		cout<<link->id<<" "<<link->volume<<endl;
	}*/

	ofw->reportIterHistory=true;
	ofw->reportLinkDetail=true;
	ofw->reportPathDetail=true;
	ofw->Report();
	system("PAUSE");
	delete ofw;
	return 0;
	
}

int TestBCPCC( const string &path, const string &name)
{
	TNM_FloatFormat::SetFormat(18,6);
	TAP_BCP *bcp = new TAP_BCP;
	bcp->SetConv(1e-8); //the relative gap used for stop the iGP algorithm for solving the TAP (see Problem (4) in the paper) or TAP-like problems (e.g., Problem (17) in the paper)
	bcp->SetInnerConv(1e-10); //the gap used for stop the innerloop of iGP
	bcp->SetMaxLsIter(15); //maximum line search iterations for iGP
	bcp->SetMaxInnerIter(15);//maximum inner loop iteration for iGP
	bcp->SetMaxIter(20); //maximum mainloop iteration for iGP
	bcp->SetLPF(BPRLK);  // take the BPR link form
	//bcp->SetCostScalar(60);

	string input, output;
	input = path +   name;
	output = path  + name + "_bcpcc";

	//build the network
	if(bcp->Build(input, output,NETTAPAS)!=0)
	{
		cout<<"\tFail to build network object"<<endl;
		return 0;
	}
	//not allow the external input of toll 
	bcp->SetTollType(TT_NOTOLL); 


	//call the CBCP algorithm
	bcp->AlgorithmBCPCC();


	bcp->reportIterHistory=true;
	bcp->reportLinkDetail=true;
	bcp->Report();


	cout<<"The algorithm is finished!"<<endl;
	system("PAUSE");

	return 0;

}

int main()
{
	string cs="ChicagoSketch"; 
	string sf = "SiouxFalls";
	//string hearn = "hearn";
	string nn = "nn"; // Hearn's nine-node network

	 TestBCPCC("..\\..\\..\\Network\\sf\\",sf); 
};