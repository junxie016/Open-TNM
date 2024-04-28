#include "..\TNA\header\stdafx.h"
#include <algorithm>
#include <math.h>
#include <iostream>
#include <tchar.h>

using namespace std;


//BSP algorithm 1 :Bi-criteria shortest path algorithm based on OD pairs
void Testdial1979(const string &path, const string &name)
{
	cout<<"start test bi-criteria spp time used Dial 1979 method"<<endl;
	TNM_FloatFormat::SetFormat(12,4);
	floatType votmin = 6.0;
	floatType votmax = 30.0;
	string distname = "uni";
	int treenumber = 1;
	TAP_MMBA1_o *baa = new TAP_MMBA1_o(votmin, votmax, distname, treenumber);
	string input, output;
	input = path +   name;
	output = path  + name + "_dial1979";
	baa->SetLPF(BPRLK);

	if(baa->Build(input, output,NETTAPAS)!=0)
	{
		cout<<"\tFail to build network object"<<endl;		
	}
	
	baa->SetTollType(TT_NOTOLL);
	baa->reportLinkDetail  = true;
	baa->init_pathElemVector(baa->votdist->treenumber);
	baa->usegap_columngeneration = 0;
	baa->votdist->calc_cdf_gra(baa->network);// initialize gradients of all of pairs
	baa->network->AllocatePathBuffer(4);
	baa->network->AllocateDestBuffer(1);
	baa->network->UpdateLinkCost();
	TNM_SORIGIN* pOrg;
	clock_t start, end;
	start = clock();
	for (int i=0; i<baa->network->numOfOrigin; i++)
	{
		pOrg = baa->network->originVector[i];
		for (int j=0; j<pOrg->numOfDest; j++)
		{
			TNM_SDEST* dest=baa->network->originVector[i]->destVector[j];
			dest->buffer[0] = 0;
			baa->Likelypath(baa->network->originVector[i], dest, 0);
			baa->MMall_or_nothing(dest);
		}
	}
	end = clock();
	double usetime = end - start;
	cout<<"Dial 1979 all OD time = "<<usetime/CLOCKS_PER_SEC<<endl;
	baa->cout_Q_node_count(path, name, "dial1979");
	baa->bi_spp_path_output(path, name, "dial1979", usetime);
	baa->Report();
	delete baa;
}

//BSP algorithm 2 :Bi-criteria shortest path basic algorithm based on origin
void Testbasic_ori(const string &path, const string &name)
{
	cout<<"start test bi-criteria spp time using basic method for each origin"<<endl;
	TNM_FloatFormat::SetFormat(12,4);
	floatType votmin = 6.0;
	floatType votmax = 30.0;
	string distname = "uni";
	int treenumber = 1;
	TAP_MMBA1_o *baa = new TAP_MMBA1_o(votmin, votmax, distname, treenumber);
	string input, output;
	input = path +   name;
	output = path  + name + "_basic_ori";
	baa->SetLPF(BPRLK);
	if(baa->Build(input, output,NETTAPAS)!=0)
	{
		cout<<"\tFail to build network object"<<endl;
		
	}
	baa->reportLinkDetail  = true;
	baa->SetTollType(TT_NOTOLL);
	baa->init_pathElemVector(baa->votdist->treenumber);
	//initialize
	baa->sumo_path = 0.0;
	baa->spptreenum = 0;
	baa->shortestpathtime = 0.0;
	baa->truespptime = 0.0;
	baa->ltatime = 0.0;
	baa->ltachecktime = 0.0;
	baa->theta_min = 1/baa->votdist->vot_max;
	baa->theta_max = 1/baa->votdist->vot_min;
	baa->network->AllocatePathBuffer(4); //buffer[0] path toll; buffer[1] add path status(is new add or not); buffer[2] path lower bound; buffer[3] path upper bound
	baa->build_node_dest();//find dest of a node
	baa->network->AllocateLinkBuffer(9); // allocate float memory for link variables, 0 is a, 1 is u, 2 is new x, 3 is new u ,4 is origin-based new flow, 5 is origin-based new u, 6 is link label: 0 is not in the tree, 1 is in the tree; 7 is x_link, 8 is u_link(LTA);
	baa->network->AllocateNodeBuffer(9);
	//initialize
	baa->network->UpdateLinkCost();
	TNM_SORIGIN* pOrg;
	clock_t start, end;
	start = clock();
	for (int i=0; i<baa->network->numOfOrigin; i++)
	{
		pOrg = baa->network->originVector[i];
		for(int j =0 ;j<baa->network->numOfLink;j++)
		{
			baa->network->linkVector[j]->buffer[4] = 0;
			baa->network->linkVector[j]->buffer[5] = 0;
		}
		int oldtreenum = baa->spptreenum;
		baa->MPA_path(pOrg, 1, 1);
		baa->tree_num.push_back(baa->spptreenum - oldtreenum);//save tree num
	}
	end = clock();
	double usetime = end - start;
	cout<<"basic method for all origin running time = "<<usetime/CLOCKS_PER_SEC<<endl;
	string outtreenum;
	outtreenum = path + name + "_basic_ori_treenum.tnum";
	baa->cout_tree_num(outtreenum);
	baa->cout_Q_node_count_origin(path, name, "basic_ori");
	baa->bi_spp_path_output(path, name, "basic_ori", usetime);

	baa->Report();
	delete baa;
}

//BSP algorithm 3 :Bi-criteria shortest path pivot spanning tree algorithm based on origin
void TestPTU_ori(const string &path, const string &name)
{
	cout<<"start test bi-criteria spp time using PTU method for each origin"<<endl;
	TNM_FloatFormat::SetFormat(12,4);
	floatType votmin = 6.0;
	floatType votmax = 30.0;
	string distname = "uni";
	int treenumber = 1;
	TAP_MMBA1_o *baa = new TAP_MMBA1_o(votmin, votmax, distname, treenumber);
	string input, output;
	input = path +   name;
	baa->SetLPF(BPRLK);
	output = path  + name + "_PTU_ori";
	if(baa->Build(input, output,NETTAPAS)!=0)
	{
		cout<<"\tFail to build network object"<<endl;
		
	}
	baa->reportLinkDetail  = true;
	baa->SetTollType(TT_NOTOLL);
	baa->init_pathElemVector(baa->votdist->treenumber);
	//initialize
	baa->sumo_path = 0.0;
	baa->spptreenum = 0;
	baa->shortestpathtime = 0.0;
	baa->truespptime = 0.0;
	baa->ltatime = 0.0;
	baa->ltachecktime = 0.0;
	baa->theta_min = 1/baa->votdist->vot_max;
	baa->theta_max = 1/baa->votdist->vot_min;
	baa->network->AllocatePathBuffer(4); //buffer[0] path toll; buffer[1] add path status(is new add or not); buffer[2] path lower bound; buffer[3] path upper bound
	baa->build_node_dest();//find dest of a node
	baa->network->AllocateLinkBuffer(9); // allocate float memory for link variables, 0 is a, 1 is u, 2 is new x, 3 is new u ,4 is origin-based new flow, 5 is origin-based new u, 6 is link label: 0 is not in the tree, 1 is in the tree; 7 is x_link, 8 is u_link(LTA);
	baa->network->AllocateNodeBuffer(9);
	//initialize
	baa->network->UpdateLinkCost();
	TNM_SORIGIN* pOrg;
	clock_t start, end;
	start = clock();
	for (int i=0; i<baa->network->numOfOrigin; i++)
	{
		pOrg = baa->network->originVector[i];
		for(int j =0 ;j<baa->network->numOfLink;j++)
		{
			baa->network->linkVector[j]->buffer[4] = 0;
			baa->network->linkVector[j]->buffer[5] = 0;
		}
		int oldtreenum = baa->spptreenum;
		baa->modified_T2_MPA_path(pOrg, 1, 1);
		baa->tree_num.push_back(baa->spptreenum - oldtreenum);//save tree num
	}
	end = clock();
	double usetime = end - start;
	cout<<"PTU method for all origin running time = "<<usetime/CLOCKS_PER_SEC<<endl;
	string outtreenum;
	outtreenum = path + name + "_PTU_ori_treenum.tnum";
	baa->cout_tree_num(outtreenum);
	baa->cout_Q_node_count_origin(path, name, "PTU_ori");
	baa->bi_spp_path_output(path, name, "PTU_ori", usetime);
	baa->Report();
	delete baa;
}

int main()
{
	string sf = "SiouxFalls";
	string an="Anaheim";
	string bar="Barcelona";
	string win="Winnipeg";
	string cs="ChicagoSketch";
	string cr="ChicagoRegional";
	//BSP algorithms
	//Testdial1979("..\\..\\..\\Network\\sf\\",sf); //"..\\..\\..\\Network\\cs\\",cs
	//Testbasic_ori("..\\..\\..\\Network\\sf\\",sf); //"..\\..\\..\\Network\\cs\\",cs
	//TestPTU_ori("..\\..\\..\\Network\\sf\\",sf); //"..\\..\\..\\Network\\cs\\",cs
	system("pause");
};