#include "..\TNA\header\stdafx.h"
#include <algorithm>
#include <cstdio>
#include <math.h>
#include <iostream>

using namespace std;

floatType ScaleDemandCoe = 1.0;
struct MEUE_result
{
    floatType ScaleDemandCoe;
    double congestion;
    int numOfUEpath;
    floatType UEentropy;
    floatType TwoOrderEntropy;
    floatType MaxEntropy;
};
MEUE_result meue_rlt;

int TestMEUE_TAPAS( const string &path, const string &name)
{
	TNM_FloatFormat::SetFormat(18,6);
	string input, output;
	input = path +   name;
	output = path  + name + "_MEUE_tapas";

	MEUE_TAPAS *ofw=new MEUE_TAPAS;
	ofw->ReadFromOUE = true;

	if(ofw->Build(input, output,NETTAPAS)!=0)
	{
		cout<<"\tFail to build network object"<<endl;
		return 0;
	}

	ofw->SetMaxIter(400);
	ofw->SetMaxMainGapConv(1e-7);
	ofw->SetMaxMainFIConv(1e-7);
	ofw->SetMaxSubConv(1e-15);
	ofw->SetMaxSubIter(5);
	ofw->OUEfileType=1;
	ofw->network->ScaleDemand(ScaleDemandCoe);
	ofw->reportIterHistory = false;
	ofw->reportLinkDetail  = false;
	ofw->reportbushflow = false;
	ofw->reportpathflow=false;
	ofw->IsReportPAS=false;
	ofw->EnableEvRGap();
	ofw->SetTollType(TT_NOTOLL); 

	if(ofw->Solve()== ErrorTerm)  
	{
		cout<<"something wrong happend in solving the problem"<<endl;
		return	0;
	}

	floatType accGap=0.0;
	int numofuepath=0;
	for(int i=0;i<ofw->network->numOfOrigin;i++)
	{
		TNM_SORIGIN* orgn=ofw->network->originVector[i];
		for(int j=0;j<orgn->numOfDest;j++)
		{
			TNM_SDEST* dest=orgn->destVector[j];
			floatType mincost=1e15,maxcost=0.0;
			for(int k=0;k<dest->pathSet.size();k++)
			{
				TNM_SPATH* path=dest->pathSet[k];
				numofuepath++;
				path->cost=0.0;
				for(int a=0;a<path->path.size();a++)
				{
					path->cost += path->path[a]->cost;
				}
				if(path->cost>maxcost) maxcost=path->cost;
				if(path->cost<mincost) mincost=path->cost;
			}
			if((maxcost-mincost) > accGap) accGap=(maxcost-mincost);
		}
	}
	meue_rlt.numOfUEpath=numofuepath;
	meue_rlt.MaxEntropy=ofw->OFV;
	ofw->SaveMEUESolution();
	ofw->SaveOriginLinkFlow(path + "MEUE_olinkflow.txt");
	ofw->network->ReleaseLinkBuffer();
	ofw->network->ReleasePathBuffer();
	
	delete ofw;
	return 0;
}

int TestiGP( const string &path, const string &name)
{
	TNM_FloatFormat::SetFormat(18,4);
	TAP_iGP *ofw = new TAP_iGP;
	ofw->SetCostScalar(60);
	ofw->SetConv(1e-12);
	ofw->SetInnerConv(1e-10);
	ofw->SetMaxLsIter(15);
	ofw->SetMaxInnerIter(20);
	ofw->SetMaxIter(500);
	ofw->SetLPF(BPRLK);
	ofw->recordODPath = false;

	string input, output;

	input = path +   name;
	output = path  + name + "_iGP";

	if(ofw->Build(input, output,NETTAPAS)!=0)
	{
		//cout<<"\tFail to build network object"<<endl;
		return 0;
	}



	ofw->network->ScaleDemand(ScaleDemandCoe);
	ofw->reportIterHistory = false;
	ofw->reportLinkDetail  = true;
	ofw->EnableEvRGap();
	ofw->SetTollType(TT_NOTOLL); 

	if(ofw->Solve()== ErrorTerm)  //solve traffic assignment problem
	{
		//cout<<"something wrong happend in solving the problem"<<endl;
		return	0;
	}

	
	cout<<"OFV="<<ofw->OFV<<endl;
	double entr = ofw->ComputeEntropyfromPath();
	char chr[20];
	sprintf(chr,"%.4f",entr);
	cout<<"Entropy="<<chr<<endl;
	cout<<"terminateType="<<ofw->termFlag<<endl;
	floatType aveVC=0.0;
	for(int a=0;a<ofw->network->numOfLink;a++)
	{
		TNM_SLINK* link=ofw->network->linkVector[a];
		aveVC += link->volume / link->capacity;
	}
	aveVC /= ofw->network->numOfLink;
	sprintf(chr,"%.4f",aveVC);
	cout<<"\tAverage V/C="<<chr<<endl;
	meue_rlt.congestion=aveVC;
	meue_rlt.UEentropy=entr;
	
	ofw->PrepareReport();
	ofw->Report();

	delete ofw;
	return 0;
}

int TestiOEMA( const string &path, const string &name)
{
	TNM_FloatFormat::SetFormat(12,4);
	TAP_iOEMA *oema = new TAP_iOEMA;

	oema->SetConv(1e-14);
	oema->SetInnerConv(1e-14);
	oema->SetMaxInnerIter(20);
	oema->SetCostScalar(60);
	oema->ReadFromOUE = true;
	oema->OUEfileType=1;

	oema->SetMaxIter(150);
	oema->SetLPF(BPRLK);

	//
	string input, output;

	input = path +   name;
	output = path  + name + "_iOEMA";
	if(oema->Build(input, output,NETTAPAS)!=0)
	{
		//cout<<"\tFail to build network object"<<endl;
		return 0;
	}
	


	oema->network->ScaleDemand(ScaleDemandCoe);
	oema->reportIterHistory = false;
	oema->reportLinkDetail  = false;

	oema->EnableReportOrgFlows(false);
	oema->SetTollType(TT_NOTOLL); 

	if(oema->Solve()== ErrorTerm)  //solve traffic assignment problem
	{
		//cout<<"something wrong happend in solving the problem"<<endl;
		return	0;
	}

	//cout<<"terminateType="<<oema->termFlag<<endl;
	meue_rlt.TwoOrderEntropy=oema->entr;
	oema->SaveProSolution();
	oema->SaveOriginLinkFlow(path + "O2P_olinkflow.txt");
	delete oema;
	return 0;
}

int CompareOlinkFlowOfMEUEAndO2P(const string &path, string netname)
{
	TNM_FloatFormat::SetFormat(12,4);
	TAP_iOEMA *oema = new TAP_iOEMA;
	string input, output;
	input = path +   netname;
	output = path  + netname + "_cp";
	if(oema->Build(input, output,NETTAPAS)!=0)
	{
		//cout<<"\tFail to build network object"<<endl;
		return 0;
	}
	vector<floatType> oLinkFlowVector;
	for(int a=0;a<oema->network->numOfLink;a++)
	{
		oLinkFlowVector.push_back(0.0);
	}


	floatType ave_bushdiff=0.0, max_bushdiff=0.0;
	int numofOlink=0;
	int olinkdiff_g100=0, olinkdiff_g10=0, olinkdiff_g1=0, olinkdiff_ot=0;
	int olinkdiff_re_g100=0, olinkdiff_re_g10=0, olinkdiff_re_g1=0, olinkdiff_re_ot=0;

	string InOUEName =  path + netname + "_MEUE_tapas.omeue";
	ifstream oueFile;
	if (!TNM_OpenInFile(oueFile, InOUEName))
	{
		cout<<"\t Cannot Open the omeueFile "<<endl;
	}
	InOUEName =  path + netname + "_iOEMA.opro";
	ifstream oprFile;
	if (!TNM_OpenInFile(oprFile, InOUEName))
	{
		cout<<"\t Cannot Open the oproFile "<<endl;
	}

	string line;
	getline(oueFile,line);
	getline(oueFile,line);
	cout<<"\t Reading the origin-based link flow.."<<endl;
	int numOrigin;
	oueFile>>numOrigin;
	if (numOrigin != oema->network->numOfOrigin)
	{
		cout<<"The origin number is not equal to network origin."<<endl;
		system("PAUSE");
	}
	getline(oprFile,line);
	getline(oprFile,line);
	getline(oprFile,line);
	for (int oi=0; oi<numOrigin; oi++)
	{
		if(oi%100==1)
			cout<<"\t\tReading orign "<<oi<<"/"<<numOrigin<<endl;
		int oid;
		int numLink;
		oueFile>>oid;
		
		oprFile>>oid;
		oprFile>>numLink;
		for (int li=0; li<numLink; li++)
		{
			int linkid;
			floatType flow;
			oprFile>>linkid;
			oprFile>>flow;
			oLinkFlowVector[linkid-1]=flow;
		}
		
		floatType bushdiff=0.0;
		oueFile>>numLink;
		for (int li=0; li<numLink; li++)
		{
			int linkid;
			floatType flow1,flow2;
			oueFile>>linkid;
			oueFile>>flow1;
			flow2=oLinkFlowVector[linkid-1];

			if(flow1>10.0)
			{
			numofOlink++;
			floatType diff=abs(flow1-flow2);
			floatType diff_re=0.0;
			if(flow1>0.0) diff_re=diff/flow1;

			if(diff>=100) olinkdiff_g100++;
			else if(diff>=10) olinkdiff_g10++;
			else if(diff>=1) olinkdiff_g1++;
			else olinkdiff_ot++;

			if(diff_re>=1.0) olinkdiff_re_g100++;
			else if(diff_re>=0.1) olinkdiff_re_g10++;
			else if(diff_re>=0.01) olinkdiff_re_g1++;
			else olinkdiff_re_ot++;

			bushdiff+=diff;
			}
		}

		ave_bushdiff+=bushdiff;
		if(bushdiff>max_bushdiff) max_bushdiff=bushdiff;
	}
	oueFile.close();
	oprFile.close();
	ave_bushdiff/=oema->network->numOfOrigin;

	cout<<"Result of the olink flow difference:"<<endl;
	cout<<"ave_bushdiff="<<ave_bushdiff<<",max_bushdiff="<<max_bushdiff<<endl;
	cout<<"numofOlink="<<numofOlink<<endl;
	cout<<"Distrobution of the olink flow diff: >=100\t>=10\t>=1\tothers"<<endl;
	cout<<olinkdiff_g100<<"\t"<<olinkdiff_g10<<"("<<olinkdiff_g10/numofOlink*1e4<<")"<<"\t"<<olinkdiff_g1<<"\t"<<olinkdiff_ot<<endl;
	cout<<"Distrobution of the relative olink flow diff: >=100%\t>=10%\t>=1%\tothers"<<endl;
	cout<<olinkdiff_re_g100<<"\t"<<olinkdiff_re_g10<<"("<<olinkdiff_re_g10/numofOlink*1e4<<")"<<"\t"<<olinkdiff_re_g1<<"\t"<<olinkdiff_re_ot<<endl;
	return 0;
}

int MEUE_Experiment(const string &path, string netname)
{
	
	ScaleDemandCoe=1.0;
	meue_rlt.ScaleDemandCoe=ScaleDemandCoe;

	TestiGP(path,netname);
	cout<<"Finish iGP!"<<endl;

	TestMEUE_TAPAS(path,netname);
	cout<<"Finish MEUE_TAPAS!"<<endl;

	TestiOEMA(path,netname);
	cout<<"Finish iOEMA!"<<endl;

	// These files are required to initialize MEUE and O2P, but are not final outputs.
	std::remove((path + netname + "_iGP_tapas.oue").c_str());
	std::remove((path + netname + "_iGP.lfp").c_str());

	return 0;
}

int main(int argc, char* argv[])
{
    string path = "..\\..\\..\\Network\\sf\\";
    string netname = "SiouxFalls";

    MEUE_Experiment(path, netname);

    return 0;
}
