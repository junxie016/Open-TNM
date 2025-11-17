#include "..\..\DllProj\trafficNet\stdafx.h"
#include <algorithm>
#include <math.h>
#include <iostream>
#include <tchar.h>

using namespace std;

void bi_spp_path_output(TAP_SBA* sba, const string &path, const string &name, const string &algname)
{
	TNM_SORIGIN* pOrg;
	ofstream out;
	ofstream out2;
	TNM_OpenOutFile(out,path  + name + "_" + algname + ".info");
	TNM_OpenOutFile(out2,path  + name +"_" + algname +  ".pfp");
	out<<"O\tD\tEffPathNum\tBoundaries\n";
	out2<<"PathID        Origin      Dest      Demand      Flow      Time     Toll     num of link     links   \n";
	int pthid = 0;
	for (int i=0;i<sba->network->numOfOrigin;i++)
	{
		pOrg = sba->network->originVector[i];
		for (int j=0; j<pOrg->numOfDest; j++)
		{
			TNM_SDEST* dest = sba->network->originVector[i]->destVector[j];
			out<<TNM_IntFormat(dest->origin->id, 4)<<TNM_IntFormat(dest->dest->id, 4)<<TNM_IntFormat(dest->pathSetbound.size()-1, 4);
			for (int k=0; k<dest->pathSetbound.size(); k++)
			{
				out<<TNM_FloatFormat(dest->pathSetbound[k], 12, 6);
			}
			out<<"\n";
			for (int u=0; u<dest->pathSet.size(); u++)
			{
				pthid ++;
				TNM_SPATH* path = dest->pathSet[u];
				out2<<TNM_IntFormat(pthid,4)<<TNM_IntFormat(pOrg->origin->id,4)<<TNM_IntFormat(dest->dest->id,4)<<TNM_FloatFormat(dest->assDemand,12,6)<<TNM_FloatFormat(path->flow,12,6)
					<<TNM_FloatFormat(path->PathCostS(),12,6)<<TNM_FloatFormat(path->buffer[0],14,6)<<TNM_IntFormat(path->path.size(),4);
				for (int li=0; li<path->path.size();li++)
				{
					TNM_SLINK* link = path->path[li];
					out2<<TNM_IntFormat(link->id,4);
				}
				out2<<endl;
			}
			out2<<endl;
		}
	}
	out.close();
	out2.close();
}


void TestSBA(const string &path, const string &name, floatType demandlevel=1.0, string dist = "uni", floatType rhovalue=0.0)
{
	cout<<"Enter Test SBA"<<endl;
	TNM_FloatFormat::SetFormat(12,4);
	//The minimum value of VOT
	floatType votmin = 6.0;
	//The maximum value of VOT
	floatType votmax = 30.0;
	//"uni"-vot uniform distribution "rational"-theta rational function used in sensitive analysis
	string distname = dist;
	int treenumber = 1;
	TAP_SBA* sba = new TAP_SBA(votmin, votmax, distname, treenumber);
	if (dist == "rational")
	{
		sba->votdist->set_rho(rhovalue);
	}
	sba->SetConv(1e-12);
	sba->SetLPF(BPRLK);
	string input, output;
	input = path +   name;
	output = path  + name + "_SBA";
	if(sba->Build(input, output,NETTAPAS)!=0)
	{
		cout<<"\tFail to build network object"<<endl;

	}
	sba->network->ScaleDemand(demandlevel);
	sba->SetMaxIter(30);
	sba->reportIterHistory = true;
	sba->reportLinkDetail  = true;
	sba->SetTollType(TT_NOTOLL);
	sba->init_pathElemVector(sba->votdist->treenumber);
	sba->network->UpdateLinkCost();
	if(sba->Solve()== ErrorTerm) 
	{
		cout<<"something wrong happened in solving the problem"<<endl;

	}
	//change the time to time without gap
	sba->iterRecord[sba->curIter]->time = 1.0*(sba->withoutgaptime)/CLOCKS_PER_SEC;
	sba->Report();
	cout<<"Exit reason:"<<sba->termFlag<<endl;
	string outtreenum;
	outtreenum = path + name + "_SBA_treenum.tnum";
	sba->tree_num_output(outtreenum);
	bi_spp_path_output(sba, path, name, "SBA");
	delete sba;
}


void TestT2theta(const string& path, const string& name)
{
	TNM_FloatFormat::SetFormat(16, 12);
	floatType votmin = 6.0;
	floatType votmax = 30.0;
	string distname = "uni";
	int treenumber = 1;
	TAP_T2theta* t2 = new TAP_T2theta(votmin, votmax, distname, treenumber);
	t2->lpf = BPRLK;
	t2->SetConv(1e-7);
	t2->SetMaxLsIter(50);
	t2->maxbadIter = 100;
	if (name == "chi")
	{
		cout << "-- Test chicago region network: max 100 steps --" << endl;
		t2->SetMaxIter(100);
	}
	else
		t2->SetMaxIter(1000);
	string input, output;

	input = path + name;
	output = path  + name + "_T2theta";
	if(t2->Build(input,output,NETTAPAS) !=0)
	{
		cout<<"\tFail to build network object"<<endl;
		return;
	}

	t2->T2theta_ReadToll(input);
	cout<<"have read the toll"<<endl;

	t2->network->ScaleDemand(1);
	t2->reportIterHistory = true;
	t2->reportLinkDetail  = true;
	t2->SetTollType(TT_NOTOLL);
	t2->network->UpdateLinkCost();
	if(t2->Solve()== ErrorTerm) 
	{
		cout<<"something wrong happened in solving the problem"<<endl;

	}
	t2->Report();
	cout<<"Exit reason:"<<t2->termFlag<<endl;
	delete t2;
	//system("PAUSE");
}



int main()
{
	string cs="chicagosketch";
	string an="Anaheim";
	string cr="chi";
	string sf = "sf";
	string win="Winnipeg";
	string ba="Barcelona";
	string br="br";

	////Interactive operation
	//cout<<"Convergence test"<<endl;
	//cout<<"Please input the path of your network files: (For example D:\\Data\\network\\sf\\)"<<endl;
	//string ads;
	//getline(cin, ads);

	//cout<<"Please input the network name: (For example sf, Anaheim, Barcelona, Winnipeg, chicagosketch, chi)"<<endl;
	//string nam;
	//cin>>nam;

	//cout<<"Please input the algorithm name you want to test: (SBA or FW)"<<endl;
	//string algname;
	//cin>>algname;


	//if(algname == "SBA")
	//{
	//	TestSBA(ads,nam);
	//}
	//else if (algname == "FW")
	//{
	//	TestT2theta(ads,nam);
	//}
	//else
	//{
	//	cout<<"This algorithm name does not exist! Please input the correct name!"<<endl;
	//}
	 
	 



	////////////////////////////// Employ the algorithms ///////////////////////////////////////////////////////////
	// // Manual modification test
	//TestSBA("..\\..\\..\\data\\TestNetwork\\sf\\", sf);
	//TestT2theta("..\\..\\..\\data\\TestNetwork\\sf\\", sf);


	////////////////////////////// Numerical experiments in the paper //////////////////////////////////////////////
	// Manual modification test
	string base_file_path = "..\\..\\..\\data\\TestNetwork\\";

	////////////////////////////// Section 6.1 Convergence speed test////////////////////////////////////////////////
	//TestSBA(base_file_path + "br\\", br);
	TestSBA(base_file_path + "sf\\", sf);
	//TestSBA(base_file_path + "Anaheim\\", an);
	//TestSBA(base_file_path + "bar\\", ba);
	//TestSBA(base_file_path + "win\\", win);
	//TestSBA(base_file_path + "cs\\", cs);
	//TestSBA(base_file_path + "cr\\", cr);

	//TestT2theta(base_file_path + "br\\", br);
	//TestT2theta(base_file_path + "sf\\", sf);
	//TestT2theta(base_file_path + "Anaheim\\", an);
	//TestT2theta(base_file_path + "bar\\", ba);
	//TestT2theta(base_file_path + "win\\", win);
	//TestT2theta(base_file_path + "cs\\", cs);
	//TestT2theta(base_file_path + "cr\\", cr);

	///////////////////////////// Section 6.2 Different toll schemes////////////////////////////////////////////////
	//TestSBA(base_file_path + "cs\\section6.2\\notoll\\", cs);
	//TestT2theta(base_file_path + "cs\\section6.2\\notoll\\", cs);
	//TestSBA(base_file_path + "cs\\section6.2\\tollscheme1\\", cs);
	//TestT2theta(base_file_path + "cs\\section6.2\\tollscheme1\\", cs);
	//TestSBA(base_file_path + "cs\\section6.2\\tollscheme2\\", cs);
	//TestT2theta(base_file_path + "cs\\section6.2\\tollscheme2\\", cs);
	//TestSBA(base_file_path + "cs\\section6.2\\tollscheme3\\", cs);
	//TestT2theta(base_file_path + "cs\\section6.2\\tollscheme3\\", cs);
	// 
	/////////////////////////// Section 6.3.1 Magnitude of tolls////////////////////////////////////////////////
	//TestSBA(base_file_path + "cs\\section6.3.1\\2\\", cs);
	//TestSBA(base_file_path + "cs\\section6.3.1\\6\\", cs);
	//TestSBA(base_file_path + "cs\\section6.3.1\\12\\", cs);
	//TestSBA(base_file_path + "cs\\section6.3.1\\20\\", cs);
	//TestSBA(base_file_path + "cs\\section6.3.1\\30\\", cs);
	//TestSBA(base_file_path + "cs\\section6.3.1\\60\\", cs);
	//TestSBA(base_file_path + "cs\\section6.3.1\\100\\", cs);
	//TestSBA(base_file_path + "cs\\section6.3.1\\160\\", cs);
	/////////////////////////// Section 6.3.2 Demand level - constant toll
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemandconstoll\\0.2\\", cs, 0.2);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemandconstoll\\0.4\\", cs, 0.4);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemandconstoll\\0.6\\", cs, 0.6);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemandconstoll\\0.8\\", cs, 0.8);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemandconstoll\\1\\", cs, 1.0);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemandconstoll\\1.2\\", cs, 1.2);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemandconstoll\\1.4\\", cs, 1.4);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemandconstoll\\1.6\\", cs, 1.6);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemandconstoll\\1.8\\", cs, 1.8);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemandconstoll\\2\\", cs, 2.0);
	/////////////////////////// Section 6.3.2 Demand level - different toll
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemanddifftoll\\0.2\\", cs, 0.2);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemanddifftoll\\0.4\\", cs, 0.4);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemanddifftoll\\0.6\\", cs, 0.6);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemanddifftoll\\0.8\\", cs, 0.8);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemanddifftoll\\1\\", cs, 1.0);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemanddifftoll\\1.2\\", cs, 1.2);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemanddifftoll\\1.4\\", cs, 1.4);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemanddifftoll\\1.6\\", cs, 1.6);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemanddifftoll\\1.8\\", cs, 1.8);
	//TestSBA(base_file_path + "cs\\section6.3.2\\diffdemanddifftoll\\2\\", cs, 2.0);
	/////////////////////////// Section 6.3.3 Different distribution
	//TestSBA(base_file_path + "cs\\section6.3.3\\0\\", cs, 1.0, "rational", 0.0);
	//TestSBA(base_file_path + "cs\\section6.3.3\\-0.1\\", cs, 1.0, "rational", -0.1);
	//TestSBA(base_file_path + "cs\\section6.3.3\\-0.5\\", cs, 1.0,  "rational", -0.5);
	//TestSBA(base_file_path + "cs\\section6.3.3\\-0.9\\", cs, 1.0,  "rational", -0.9);
	//TestSBA(base_file_path + "cs\\section6.3.3\\1\\", cs, 1.0, "rational", 1.0);
	//TestSBA(base_file_path + "cs\\section6.3.3\\5\\", cs, 1.0, "rational", 5.0);
	//TestSBA(base_file_path + "cs\\section6.3.3\\9\\", cs, 1.0, "rational", 9.0);

	system("PAUSE");
	return 0;
};	