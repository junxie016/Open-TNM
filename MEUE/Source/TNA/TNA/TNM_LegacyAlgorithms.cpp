#include "header\stdafx.h"
#include <math.h>
#include <stack>

TAP_iGP::TAP_iGP()
{
    recordODPath = false;
    columnG = false;
    nPath = 0;
    TotalFlowChange = aveFlowChange = totalShiftFlow = innerShiftFlow = maxPathGap = 0.0;
    numOfPathChange = numOfD = 0;
    ziggIter = badIter = 0;
    algorNameCard = "iGP";
    modelNameCard = "Static Deterministic Traffic Assignment Problem";
}

TAP_iGP::~TAP_iGP()
{
    for (size_t i = 0; i < dePathSet.size(); ++i) delete dePathSet[i];
}

void TAP_iGP::Initialize()
{
    network->UpdateLinkCost();
    network->InitialSubNet3();
    network->UpdateLinkCost();
    network->UpdateLinkCostDer();
    ComputeOFV();
    for (int oi = 0; oi < network->numOfOrigin; ++oi)
    {
        TNM_SORIGIN* org = network->originVector[oi];
        for (int di = 0; di < org->numOfDest; ++di)
        {
            TNM_SDEST* dest = org->destVector[di];
            for (size_t pi = 0; pi < dest->pathSet.size(); ++pi)
            {
                dest->pathSet[pi]->id = ++nPath;
            }
        }
    }
}

double TAP_iGP::ComputeEntropyfromPath()
{
    double entropy = 0.0;
    for (int oi = 0; oi < network->numOfOrigin; ++oi)
    {
        TNM_SORIGIN* org = network->originVector[oi];
        for (int di = 0; di < org->numOfDest; ++di)
        {
            TNM_SDEST* dest = org->destVector[di];
            if (dest->assDemand <= 1e-8) continue;
            for (size_t pi = 0; pi < dest->pathSet.size(); ++pi)
            {
                TNM_SPATH* path = dest->pathSet[pi];
                if (path->flow > 1e-8) entropy -= path->flow * log(path->flow / dest->assDemand);
            }
        }
    }
    return entropy;
}

void TAP_iGP::PrintPath(TNM_SPATH* path)
{
    for (size_t i = 0; i < path->path.size(); ++i) cout << path->path[i]->id << " ";
    cout << endl;
}

void TAP_iGP::MainLoop()
{
	//
	TotalFlowChange = 0.0;
	numOfPathChange =0;
	floatType oldOFV = OFV;
	totalShiftFlow = 0.0;
	maxPathGap = 0;
	TNM_SORIGIN *pOrg;
	for (int i = network->numOfOrigin-1;i>=0;i--)
	{
		pOrg = network->originVector[i];

		network->UpdateSP(pOrg->origin);
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			dest->shiftFlow = 1.0;
			ColumnGeneration(pOrg,dest);
		    
			columnG = true;
			UpdatePathFlowLazy(pOrg,dest);
		}

	}
	
	//inner loop
	innerShiftFlow =1.0;
	int il;
	int numofPath;
	double preFlowPre =1e-10;
	maxPathGap = 0.0;
	numOfD =0;
	int numofD2 =0;
	int maxInIter = 300;
	for (il =0; il< maxInIter; il++)
	{
		innerShiftFlow = 0.0;
		numofPath = 0;
		numofD2 = 0;
		for (int i = network->numOfOrigin-1;i>=0;i--)
		{
			pOrg = network->originVector[i];
		
			for (int j=0;j<pOrg->numOfDest;j++)
			{
				TNM_SDEST* dest = pOrg->destVector[j];
				if (il%(maxInIter/10) == 0)
				{
					dest->shiftFlow =1.0;
				}
				if (dest->shiftFlow> convIndicator/2.0)
				{
					columnG = false;
					UpdatePathFlowLazy(pOrg,dest);
					numofD2++;
					
				}
				numofPath+=dest->pathSet.size();
				

			}

		}
		for(int i=0;i<network->numOfOrigin;i+=2)
		{
			pOrg = network->originVector[i];
			for (int j=0;j<pOrg->numOfDest;j++)
			{
				TNM_SDEST* dest = pOrg->destVector[j];
				if (il%(maxInIter/10) == 0)
				{
					dest->shiftFlow =1.0;
				}
				if (dest->shiftFlow> convIndicator/2.0)
				{
					columnG = false;
					UpdatePathFlowLazy(pOrg,dest);
					numofD2++;
				}
				numofPath+=dest->pathSet.size();
			}
		}
		
		if (innerShiftFlow < 1e-10)
		{
			break;
		}
	}
	//delete the dePathset
	int depathsetsize = dePathSet.size();
	TNM_SPATH* path;
	for (int pi=0; pi<dePathSet.size(); pi++)
	{
		path = dePathSet[pi];
		delete path;
	}
	dePathSet.clear();


	convIndicator = RelativeGap();
	aveFlowChange = TotalFlowChange/numOfPathChange;

	ComputeOFV(); 
	cout<<"Iter "<<curIter<<" OFV="<<OFV<<" RG="<<convIndicator<<" time="<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
	ziggIter=0;
	badIter=0;
}

void TAP_iGP::UpdatePathFlowLazy(TNM_SORIGIN* pOrg,TNM_SDEST* dest)
{
	//check if the yPath is existed in the path set
	TNM_SPATH* ePath = NULL;
	bool find = true;
	TNM_SPATH* path;
	dest->shiftFlow =0.0;
	if (columnG)
	{
		for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin(); pv != dest->pathSet.end();pv++)
		{
			find = true;
			path = *pv;
			if (path->path.size() == yPath->path.size())
			{

				for (int li=0; li< yPath->path.size();li++)
				{
					if (path->path[li] != yPath->path[li])
					{
						find = false;
						break;
					}
				}
			}
			else
			{
				find = false;
			}

			if (find)
			{
				ePath = path;
				break;
			}

		}
		//
		if (!find)
		{
			nPath++;
			TNM_SPATH* addPath = new TNM_SPATH;
			addPath->path = yPath->path;
			addPath->id = nPath;
			dest->pathSet.push_back(addPath);
			ePath = addPath;
		}
	
	}
	//added to celerate
	double maxCost =0.0;
	double minCost = 100000000.0;
	for (int pi=0;pi<dest->pathSet.size();pi++)
	{
		path = dest->pathSet[pi];
		path->cost = 0.0;
		path->fdCost = 0.0;
		for (int li=0;li<path->path.size();li++)
		{
			TNM_SLINK* slink = path->path[li];
			path->cost+=slink->cost;
			path->fdCost+= slink->fdCost;

		}
		if (path->cost>maxCost)
		{
			maxCost = path->cost;
		}
		if (path->cost < minCost)
		{
			minCost = path->cost;
			ePath = path;
		}
	}
	//cout<<"3"<<endl;
	double ss;
	if (curIter == 1)
	{
		ss = 1e-2;
	}
	else
	{
		ss = convIndicator/100.0;
	}

	if (maxCost - minCost > maxPathGap)
	{
		maxPathGap = maxCost - minCost;
	}
	dest->shiftFlow = maxCost - minCost;
	///end of adding

	if (dest->shiftFlow > ss)
	{
		//make the label of the ePath;
		ePath->preFlow = ePath->flow;
		for (int li=0;li<ePath->path.size();li++)
		{
			ePath->path[li]->markStatus = ePath->id;

		}

		//update the path flows
		if (dest->pathSet.size()>1)
		{
			//cout<<"The size of path set is "<<dest->pathSet.size()<<endl;
			TNM_SLINK* slink;
			double tFlow =0.0;
			for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin(); pv != dest->pathSet.end();pv++)
			{
				path = *pv;
				if (path !=ePath)
				{
					//if (abs(path->cost-ePath->cost) > convIndicator/10.0)
					{
						path->preFlow = path->flow;
						path->cost = 0.0;
						vector<TNM_SLINK*> dLinkSet;
						vector<TNM_SLINK*> iLinkSet;
						iLinkSet.clear();
						dLinkSet.clear();
						//compute the second order derivative
						double scost = 0.0;

						for (int li=0;li<path->path.size();li++)
						{
							//
							slink = path->path[li];
							path->cost += slink->cost;
							//
							if (slink->markStatus != ePath->id)
							{
								dLinkSet.push_back(slink);
								//sdLinkSet.push_back(slink);
								scost+=slink->fdCost;
							}
							else
							{
								slink->markStatus = path->id;
							}
						}
						//update those paths with significant difference

						for (int li=0;li<ePath->path.size();li++)
						{
							slink = ePath->path[li];
							//
							if (slink->markStatus == ePath->id)
							{
								//sdLinkSet.push_back(slink);
								iLinkSet.push_back(slink);
								scost+=slink->fdCost;
							}
							else
							{
								slink->markStatus = ePath->id;
							}
						}
						//now compute the flow shift 
						double dflow = (path->cost - ePath->cost)/scost;

						if (dflow > 1e-11)
						{
							totalShiftFlow+=abs(dflow);

							innerShiftFlow+=abs(dflow);

							//path's flow become 0
							if (dflow >= path->flow)
							{
								dflow = path->flow;
								path->flow =0.0;
								tFlow+=path->flow;
							}
							else
							{
								path->flow = path->flow - dflow;
								tFlow+=path->flow;

							}
							//update the link flow of the path
							for (int lj =0; lj<path->path.size();lj++)
							{
								slink = path->path[lj];
								slink->volume = slink->volume - dflow;
								if (abs(slink->volume) < 1e-8)
								{
									slink->volume = 0.0;
								}
								if (slink->volume < 0)
								{
									cout<<"Wrong in updating the path link flow"<<endl;
									cout<<"The volume is "<<slink->volume<<endl;
									cout<<"dflow is "<<dflow<<endl;
									cout<<"preFlow is "<<path->preFlow<<endl;
									cout<<"path flow is "<<path->flow<<endl;
									PrintPath(path);
									system("PAUSE");

								}
								slink->cost = slink->GetCost();
								slink->fdCost = slink->GetDerCost();

							}

						}
						else
						{
							tFlow+=path->flow;
						}

					}

				}

			}
			//end of the  path search
			if (tFlow-dest->assDemand>1e-5)
			{
				cout<<"The total OD flow of paths is larger than assDemand"<<endl;
				system("PAUSE");
			}
			else
			{
				ePath->flow = dest->assDemand - tFlow;
				//updating the path flows
				double aFlow = ePath->flow - ePath->preFlow;
				for (int lj=0;lj<ePath->path.size();lj++)
				{
					slink = ePath->path[lj];
					slink->volume = slink->volume + aFlow;
					slink->cost = slink->GetCost();
					slink->fdCost = slink->GetDerCost();
				}
			}
		}

		//
		for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin(); pv != dest->pathSet.end();)
		{
			path = *pv;
			if (path->flow == 0.0)
			{
				pv = dest->pathSet.erase(pv);
				dePathSet.push_back(path);
			}
			else
			{
				pv++;
			}
		}
	}
	
	
}

void TAP_iGP::PostProcess()
{
	intWidth = TNM_IntFormat::GetWidth();
	floatWidth = TNM_FloatFormat::GetWidth();
	for(int a=0;a<network->numOfLink;a++) network->linkVector[a]->oLinkPtr=NULL;

	TNM_SORIGIN *pOrg;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		pOrg->obLinkVector.clear();

		for (int j=0;j<pOrg->numOfDest;j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin(); pv != dest->pathSet.end();pv++)
			{
				TNM_SPATH* path = *pv;
				for(int a=0;a<path->path.size();a++)
				{
					TNM_SLINK* link=path->path[a];
					if(link->oLinkPtr == NULL)
					{
						ORGLINK * olink=new ORGLINK;
						link->oLinkPtr = olink;
						olink->oFlow=0.0;
						olink->linkPtr=link;
						pOrg->obLinkVector.push_back(olink);
					}

					link->oLinkPtr->oFlow += path->flow;
				}
			}
		}

		for(int a=0;a<network->numOfLink;a++) network->linkVector[a]->oLinkPtr=NULL;
	}

	string oueFileName = outFileName+"_tapas.oue";
	ofstream out;
	cout<<"\tWriting UE solution information into "<<outFileName<<" following the TAPAS output format"<<endl;
	if (!TNM_OpenOutFile(out,oueFileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .oue file to write UE solution information!"<<endl;
		return;
	}
	//
	out<<setw(5)<<"Origin"<<setw(5)<<"Tail"<<setw(5)<<"Head"<<setw(10)<<"Flow"<<endl;
	for (int oi=0;oi<network->numOfOrigin;oi++)
	{
		//
		TNM_SORIGIN* pOrg = network->originVector[oi];
		for (int li=0;li<pOrg->obLinkVector.size();li++)
		{
			ORGLINK* olink = pOrg->obLinkVector[li];
			if (olink->oFlow > 1e-10)
			{
				out<<left<<setw(intWidth)<<TNM_IntFormat(pOrg->id())<<setw(intWidth)<<TNM_IntFormat(olink->linkPtr->tail->id)<<setw(floatWidth)<<TNM_IntFormat(olink->linkPtr->head->id)<<setw(floatWidth)<<setw(floatWidth)<<TNM_FloatFormat(olink->oFlow,20,11)<<setw(floatWidth)<<setw(floatWidth)<<endl;
			}
			
		}
	}
	out.close();


	TNM_TAP::PostProcess();
}

// ---------------- iOEMA (flattened onto TAP_iTAPAS) ----------------

void TAP_iOEMA::ReadTAPASOUE()
{
	//
	cout<<"Reading the OUE file from TAPAS output"<<endl;
	string InOUEName;
	if(OUEfileType == 1)
		InOUEName =  inFileName + "_iGP_tapas.oue";
	else
		InOUEName =  inFileName + "_iTAPAS_tapas.oue";//_iTAPAS_tapas.oue
	cout<<"Reading "<<InOUEName<<endl;

	ifstream oueFile;
	if (!TNM_OpenInFile(oueFile, InOUEName))
	{
		cout<<"\t Cannot Open the oueFile "<<endl;

	}
	//
	string line;
	vector<string> words;
	getline(oueFile,line);

	while (!oueFile.eof())
	{
		//
		getline(oueFile,line);
		TNM_GetWordsFromLine(line,words);
		//
		//cout<<line<<endl;
		//cout<<"words' size is "<<words.size()<<endl;
		//system("PAUSE");
		//
		if (words.size() == 4)
		{
			int originID, tailID, headID;
			floatType flow;
			TNM_FromString(originID,words[0],std::dec);
			TNM_FromString(tailID,words[1],std::dec);
			TNM_FromString(headID,words[2],std::dec);
			TNM_FromString(flow,words[3],std::dec);
			//
			TNM_SORIGIN* pOrg = originMap[originID];
			PasOrigin2* por = POList[pOrg->m_class];
			//
			TNM_SNODE* tailPtr = network->nodeVector[tailID-1];
			//TNM_SNODE* headPtr = network->nodeVector[headID-1];
			TNM_SLINK* crLink = NULL;
			for (int ni=0; ni<tailPtr->forwStar.size();ni++)
			{
				TNM_SLINK* sl = tailPtr->forwStar[ni];
				if(sl->head->id == headID)
				{
					crLink = sl;
					break;
				}
			}
			//
			if (crLink == NULL)
			{
				cout<<"wrong in reading OUE from TAPAS"<<endl;
				system("PAUSE");
			}
			por->oLinkFlowVector[crLink->id-1] = flow;
		}


	}
	//computing the onode flow
	for (int oi=0;oi<network->numOfOrigin;oi++)
	{
		PasOrigin2* por = POList[oi];
		for (int ni=0; ni< network->numOfNode; ni++)
		{
			TNM_SNODE* snode = network->nodeVector[ni];
			floatType nodeFlow = 0;
			for (int li=0; li<snode->backStar.size();li++)
			{
				TNM_SLINK* link = snode->backStar[li];
				nodeFlow+=por->oLinkFlowVector[link->id-1];
			}
			por->onodeflow[snode->id-1] = nodeFlow;
		}
		//
		for (int li=0; li<network->numOfLink;li++)
		{
			TNM_SLINK* link = network->linkVector[li];
			por->oLinkApproachVector[link->id-1] = por->oLinkFlowVector[link->id-1]/por->onodeflow[link->head->id-1];
		}
	}
	//
	cout<<"Reading Total Link flow from TAPAS output"<<endl;

	string OUEName;
	if(OUEfileType == 1)
		OUEName =  inFileName + "_iGP.lfp";
	else
		OUEName =  inFileName + "_iTAPAS.lfp";//_iTAPAS.lfp

	ifstream TotalLinkFile;
	if (!TNM_OpenInFile(TotalLinkFile, OUEName))
	{
		cout<<"\t Cannot Open the oueFile "<<endl;

	}
	//
	while (!TotalLinkFile.eof())
	{
		//
		getline(TotalLinkFile,line);
		//cout<<line<<endl;

		TNM_GetWordsFromLine(line,words);
		//cout<<"words' size "<<words.size()<<endl;
		//system("PAUSE");
		if (words.size() == 7)
		{
			// Current iTAPAS link reports may omit the legacy 17-line banner.
			// Only consume rows whose first field is a numeric link id.
			int linkID;
			istringstream idReader(words[0]);
			if (!(idReader >> linkID)) continue;
			//
			int tailID,headID;
			floatType volume,cost;
			TNM_FromString(tailID,words[1],std::dec);
			TNM_FromString(headID,words[2],std::dec);
			TNM_FromString(volume,words[4],std::dec);
			TNM_FromString(cost,words[5],std::dec);

			TNM_SNODE* tailPtr = network->nodeVector[tailID-1];
			TNM_SLINK* crLink = NULL;
			for (int ni=0; ni< tailPtr->forwStar.size();ni++)
			{
				TNM_SLINK* sl = tailPtr->forwStar[ni];
				if (sl->head->id == headID)
				{
					//
					crLink = sl;
					break;
				}
			}
			if (crLink == NULL)
			{
				cout<<"wrong in reading total flow"<<endl;
				system("PAUSE");
			}
			//
			crLink->volume = volume;
			crLink->cost = cost;

		}

	}
}

floatType TAP_iOEMA::ComputeConsistency()
{
	//
	int TotalSecond =0;
	int NodeSecond  = 0;
	int LinkSecond = 0;
	int unclear =0;
	int connects =0;
	int ZeroVolume = 0;

	floatType minRca = 1;
	floatType maxRca = 0;
	//
	//cout<<"1"<<endl;
	for (int oi=0; oi<network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		PasOrigin2* por = POList[pOrg->m_class];
		//cout<<"2"<<endl;
		//pOrg->MarkOBLinksOnNet();
		network->UpdateSP(pOrg->origin);
		//cout<<"3"<<endl;
		for (int li=0; li<network->numOfLink; li++)
		{
			TNM_SLINK* slink = network->linkVector[li];
			floatType rca = slink->tail->pathElem->cost+slink->cost-slink->head->pathElem->cost;
			
			if ( (por->oLinkFlowVector[slink->id-1]  <entropyFlowPrecision ) && rca >=0 && por->onodeflow[slink->head->id-1] > entropyFlowPrecision && slink->length > 0 && slink->volume > entropyFlowPrecision )
			{
				//if ( por->onodeflow[slink->head->id-1] > entropyFlowPrecision && por->oLinkLable[slink->id-1]==false)
				{
					if (minRca > rca)
					{
						minRca = rca;
					}
				}
			}
			else if (por->oLinkFlowVector[slink->id-1]> entropyFlowPrecision)
			{
				if (maxRca < rca)
				{
					maxRca = rca;
				}

			}
			entropyCostPrecision = maxRca;
			//
			if ( por->oLinkFlowVector[slink->id-1] < entropyFlowPrecision  && rca < entropyCostPrecision && rca >=0) //&& POList[pOrg->id()-1]->onodeflow[slink->head->id-1] > 1e-12)  //
			{
				TotalSecond++;
				if (por->onodeflow[slink->head->id-1] < entropyFlowPrecision )
				{
					//cout<<"5.1";
					NodeSecond++;
				}
				else if (slink->volume < entropyFlowPrecision)
				{
					//cout<<"5.2";
					ZeroVolume++;
				}
				else if (slink->length ==0)
				{
					connects++;
					
				}
				else
				{
					//cout<<"5.5";
					unclear++;
				}

				

			}
			
			//cout<<"7"<<endl;

		}
		
	}
	

	logFile<<"============================================"<<endl;
	logFile<<"Total second class is "<<TotalSecond<<endl;
	logFile<<"The zero node  is : "<<NodeSecond<<endl;
	logFile<<"The single origin link is : "<<LinkSecond<<endl;
	logFile<<"The connects link is "<<connects<<endl;
	logFile<<"The unclear num is : "<<unclear<<endl;
	logFile<<"The zero volume is : "<<ZeroVolume++<<endl;
	logFile<<"minRca is : "<<minRca<<endl;
	logFile<<"maxRca is : "<<maxRca<<endl;

	return 0;
}

void TAP_iOEMA::SaveOUESolution()
{
	string oueFileName = outFileName+"_tapas.oue";
	ofstream out;
	cout<<"\tWriting UE solution information into "<<outFileName<<" after the OEMA algorithm"<<endl;
	if (!TNM_OpenOutFile(out,oueFileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .oue file to write UE solution information!"<<endl;
		return;
	}
	//
	out<<setw(5)<<"Origin"<<setw(5)<<"Tail"<<setw(5)<<"Head"<<setw(10)<<"Flow"<<endl;
	for (int oi=0;oi<POList.size();oi++)
	{
		PasOrigin2* por = POList[oi];
		
		for (int li=0;li<network->numOfLink;li++)
		{
			floatType flow = por->oLinkFlowVector[li];
			if (flow>0)
			{
				TNM_SLINK* link = network->linkVector[li];
				out<<TNM_IntFormat(por->OriginPtr->id())<<setw(5)<<TNM_IntFormat(link->tail->id)<<setw(5)<<TNM_IntFormat(link->head->id)<<setw(10)<<TNM_FloatFormat(flow,16,12)<<endl;
			}
		}
	}
	out.close();
}
//

int TAP_iOEMA::PrepareReport()
{
	//
	
	
	string PasFileName;
	
	cout<<"writing ITE "<<endl;
	//system("PAUSE");

	PasFileName  = outFileName + "_OEMA.ent";
	

	if (!TNM_OpenOutFile(EntropyIterFile, PasFileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .pas file to write origin flows!"<<endl;
		readyToReport = false;
		return 2;
	}
	PrintVersion(EntropyIterFile);
	
	
	return 0;
	
}

void TAP_iOEMA::ReportEntropyIter(ofstream &out)
{
	EntropyIterFile<<"~Iteration"<<setw(15)<<"Entropy"<<setw(15)<<"Relative Gap"<<setw(15)<<"Time"<<endl;
	for (int i=0; i<EntropyInfoList.size();i++)
	{
		ProInfo* pif = EntropyInfoList[i];
		EntropyIterFile<<setw(5)<<TNM_IntFormat(pif->iter)<<setw(20)<<TNM_FloatFormat(pif->Entro,15,6)<<setw(20)<<TNM_FloatFormat(pif->consis)<<setw(20)<<TNM_FloatFormat(pif->time,6,3)<<endl;
	}
	EntropyIterFile.close();
}

int TAP_iOEMA::Report()
{
	ReportEntropyIter(EntropyIterFile);
	SaveOUESolution();
	SaveUESolution();
	return 0;
}
void TAP_iOEMA::SaveUESolution()
{
	string oueFileName = outFileName+".oue";
	ofstream out;
	cout<<"\tWriting UE solution information into "<<outFileName<<" for post process"<<endl;
	if (!TNM_OpenOutFile(out,oueFileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .oue file to write UE solution information!"<<endl;
		return;
	}
	//
	cout<<"\tWriting the network link flow info ..."<<endl;
	out<<TNM_IntFormat(network->numOfLink,6)<<endl;
	for (int li=0; li<network->numOfLink; li++)
	{
		TNM_SLINK* sl = network->linkVector[li];
		out<<TNM_IntFormat(sl->tail->id,6)<<TNM_IntFormat(sl->head->id,6)<<TNM_FloatFormat(sl->volume,20,12);
	}
	out<<endl;
	//
	cout<<"\t Writing the origin-based link flow info..."<<endl;
	out<<TNM_IntFormat(network->numOfOrigin,6)<<endl;
	for (int oi=0; oi< network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		PasOrigin2* por = POList[pOrg->m_class];
		int num =0;
		for (int li=0; li<por->oLinkFlowVector.size();li++)
		{
			if (por->oLinkFlowVector[li] > flowPrecision)
			{
				num++;
			}
		}
		out<<TNM_IntFormat(pOrg->id(),6)<<TNM_IntFormat(num,6)<<endl;
		for (int oj=0; oj<por->oLinkFlowVector.size(); oj++)
		{
			
			if (por->oLinkFlowVector[oj]> flowPrecision)
			{
				out<<TNM_IntFormat(network->linkVector[oj]->tail->id,6)<<TNM_IntFormat(network->linkVector[oj]->head->id,6)<<TNM_FloatFormat(por->oLinkFlowVector[oj],20,12);
			}

		}
		out<<endl; 
	}
	out<<TNM_IntFormat(0,6)<<endl;

	out.close();
}
//

void TAP_iOEMA::CheckLinkFlow()
{
	
	for (int li=0; li<network->numOfLink; li++)
	{
		floatType flow =0;
		TNM_SLINK* slink = network->linkVector[li];
		for (int oi=0; oi<network->numOfOrigin; oi++)
		{
			TNM_SORIGIN* pOrg = network->originVector[oi];
			
			flow+=POList[pOrg->m_class]->oLinkFlowVector[slink->id-1];
			
			
		}
		if (abs(flow - slink->volume) >1e-3)
		{
			cout<<"This link is unconsistent with total flow"<<endl;
			cout<<"The flow is : "<<flow<<endl;
			cout<<"The Volume is : "<<slink->volume<<endl;
			cout<<"The difference is : "<<abs(flow-slink->volume)<<endl;
			//system("PAUSE");
		}
	}
}

floatType TAP_iOEMA::CalEntropy()
{
	floatType entr = 0;
	//
	for (int oi=0; oi<network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		PasOrigin2* por = POList[pOrg->m_class];
		for (int ni=0; ni<network->numOfNode; ni++)
		{
			TNM_SNODE* snode = network->nodeVector[ni];
			if (por->onodeflow[snode->id-1] > entropyFlowPrecision)
			{
				for (int li=0; li< snode->backStar.size(); li++)
				{
					TNM_SLINK* slink = snode->backStar[li];
					if (por->oLinkFlowVector[slink->id-1] > entropyFlowPrecision)
					{
						entr+=-por->oLinkFlowVector[slink->id-1]*log(por->oLinkFlowVector[slink->id-1]/por->onodeflow[snode->id-1]);
					}
				}
			}
		}

		
	}
	return entr;
}

void TAP_iOEMA::Initialize()
{
	network->AllocateLinkBuffer(1);
	network->AllocateNodeBuffer(1);
	logFile<<"The num of link is : "<<network->numOfLink<<endl;
	logFile<<"The num of origin is : "<<network->numOfOrigin<<endl;
    logFile<<"The num of node is : "<<network->numOfNode<<endl;

	cout<<"The num of link is : "<<network->numOfLink<<endl;
	cout<<"The num of origin is : "<<network->numOfOrigin<<endl;
	cout<<"The num of node is : "<<network->numOfNode<<endl;
	//ini the POList
	POList.reserve(network->numOfOrigin);
	originMap.clear();
	for( int i=0;i<network->numOfOrigin;i++)
	{
		TNM_SORIGIN* or = network->originVector[i];
		or->m_class = i;
		originMap.insert(map<int,TNM_SORIGIN*>::value_type(or->id(),or));
		//cout<<"ID of origin "<<i<<" = "<<or->id()<<endl;
		PasOrigin2* por = new PasOrigin2;
		//ini the olinkList in por
		por->oLinkFlowVector.reserve(network->numOfLink);
		por->oLinkApproachVector.reserve(network->numOfLink);
		por->oNewLinkFlowVector.reserve(network->numOfLink);
		for(int li=0;li<network->numOfLink;li++)
		{
			por->oLinkFlowVector.push_back(0.0);
			por->oNewLinkFlowVector.push_back(0.0);
			por->oLinkApproachVector.push_back(0.0);
		}

		//ini the onodeflow in por, the total flow of backstar links of this node
		por->onodeflow.reserve(network->numOfNode);
		for(int li=0;li<network->numOfNode;li++)
		{
			por->onodeflow.push_back(0);
		}
		//give the origin ptr
		por->OriginPtr = or;
		POList.push_back(por);//same order with "network->originVector"
	}
	///////////////////read the OUE file


	//ReadUESolution();
	ReadTAPASOUE();
	//ReadOUEfromTAPAS();
	//check 
	CheckLinkFlow();

	network->UpdateLinkCost();
	network->UpdateLinkCostDer();

	ComputeOFV(); //compute objective function value
	//--test
	cout<<"The OFV after ReadUESolution is : "<<OFV<<endl;
	logFile<<"The OFV after ReadUESolution is : "<<OFV<<endl;
	//system("PAUSE");

	//ini the link and node vector
	linkFlowVector.clear();
	linkApproachVector.clear();
	nodeFlowVector.clear();
	linkFlowVector.reserve(network->numOfLink);
	linkApproachVector.reserve(network->numOfLink);
	nodeFlowVector.reserve(network->numOfNode);
	for (int li=0;li<network->numOfLink;li++)
	{
		linkFlowVector.push_back(0.0);
		linkApproachVector.push_back(0.0);

	}
	for (int ni=0;ni<network->numOfNode;ni++)
	{
		nodeFlowVector.push_back(0.0);
		//
		TNM_SNODE* node = network->nodeVector[ni];
		node->buffer[0] = 1000000;
	}
	

	CalLinkApproach();

	cout<<"The initial Entropy Value is : "<<CalEntropy()<<endl;
	logFile<<"The initial Entropy Value is : "<<CalEntropy()<<endl;

	ComputeConsistency();

	//cout<<"0"<<endl;
}

floatType TAP_iOEMA::DeleteCycleFlow(TNM_SNODE* sNode,PasOrigin2* por)
{
	//
	floatType df =0;
	TNM_SLINK* link,*link2;
	TNM_SNODE* node;
	for (int li=0;li<sNode->backStar.size();li++)
	{
		link = sNode->backStar[li];
		if (por->oLinkFlowVector[link->id-1] > entropyFlowPrecision)
		{
			floatType mflow = por->oLinkFlowVector[link->id-1];
			node = link->tail;
			for (int lj=0;lj<node->backStar.size();lj++)
			{
				link2 = node->backStar[lj];
				if (por->oLinkFlowVector[link2->id-1] > entropyFlowPrecision && link2->tail == sNode)
				{
					if (mflow > por->oLinkFlowVector[link2->id-1])
					{
						mflow = por->oLinkFlowVector[link2->id-1];
					}
					df+=mflow;
				}
			}
		}
	}
	return df;
}

void TAP_iOEMA::FromOrigintoNode(TNM_SNODE* sNode)
{
	originVector.clear();
	for (int li=0;li<network->numOfLink;li++)
	{
		linkFlowVector[li] = 0.0;
		linkApproachVector[li] = 0.0;
	}
	//cout<<"1.2"<<endl;
	for (int oi=0;oi<network->numOfOrigin;oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		PasOrigin2* por = POList[pOrg->m_class];

		if (pOrg->origin != sNode)
		{
			
			//floatType df = DeleteCycleFlow(sNode,por);
			floatType dflow = 0.0;
			for (int li=0;li<sNode->backStar.size();li++)
			{
				if (por->oLinkFlowVector[sNode->backStar[li]->id-1] > entropyFlowPrecision)
				{
					dflow+=por->oLinkFlowVector[sNode->backStar[li]->id-1];
				}
				
			}
			//dflow-=df;
			//deFlow+=df;
			if (dflow > entropyFlowPrecision )
			{
				//
				por->oFlow = dflow;
				por->changedLinkVector.clear();
				//por->changedLinkVector.reserve();
				originVector.push_back(por);
				BFSOriginToNode(por,sNode,dflow);
				//cout<<"1.4"<<endl;

			}

		}
	}

}

//
//
double TAP_iOEMA::RemoveNodeBasedCycle(TNM_SNODE* sNode)
{
	//
	TNM_SLINK* link,*link2;
	TNM_SNODE* head;
	TNM_SNODE* tail;
	floatType dd =0;
	for (int li=0;li<network->numOfLink;li++)
	{
		link = network->linkVector[li];
		if (linkFlowVector[link->id-1] > 0)
		{
			double ff = linkFlowVector[link->id-1];
			head = link->head;
			for (int lj=0;lj<head->forwStar.size();lj++)
			{
				link2 = head->forwStar[lj];
				if (link2->head == link->tail)
				{
					if (linkFlowVector[link2->id-1] > 0 )
					{
						if (ff > linkFlowVector[link2->id-1])
						{
							ff = linkFlowVector[link2->id-1];
						}
						
						//
						linkFlowVector[link->id-1]-=ff;
						linkFlowVector[link2->id-1]-=ff;
						dd+=ff;
					}
				}
			}
			//

		}
	}

	return dd;
}
//
void TAP_iOEMA::FromNodeToOrigin(TNM_SNODE* sNode)
{
	TNM_SLINK* link;
	TNM_SNODE* node;

	PreCycleFlow =0.0;
	NumOfCycle =0;
	//update the link approach for this node
	for (int ni=0;ni<network->numOfNode;ni++)
	{
		node = network->nodeVector[ni];
		floatType af = 0.0;
		for (int li=0;li<node->forwStar.size();li++)
		{
			//
			link = node->forwStar[li];
			if (linkFlowVector[link->id-1] > entropyFlowPrecision)
			{
				af+=linkFlowVector[link->id-1];
			}
			

		}
		nodeFlowVector[node->id-1] = af;
		//
		if (af> entropyFlowPrecision)
		{
			for (int li=0;li<node->forwStar.size();li++)
			{
				link = node->forwStar[li];
				if (linkFlowVector[link->id-1] > entropyFlowPrecision)
				{
					linkApproachVector[link->id-1] = linkFlowVector[link->id-1]/af;
				}
				else
				{
					linkApproachVector[link->id-1] = 0.0;
				}
				
			}
		}
		else
		{
			for (int li=0;li<node->forwStar.size();li++)
			{
				link = node->forwStar[li];
				linkApproachVector[link->id-1] = 0.0;
			}
		}
	}
	//
	for (int oi=0;oi<originVector.size();oi++)
	{
		PasOrigin2* por = originVector[oi];
		BFSNodeToOrigin(por,sNode,por->oFlow);
		
	}
	
}
//

void TAP_iOEMA::BFSNodeToOrigin(PasOrigin2* por,TNM_SNODE* sNode,floatType oFlow)
{
	TNM_SORIGIN* pOrg = por->OriginPtr;
	floatType dFlow = 0.0;

	deque<TNM_SNODE*> breadthList;//keep the node list in the breadth first search
	vector<floatType> nodeFlow(network->numOfNode,0.0);
	//to check if the destination demand is consistent 

	breadthList.push_back(pOrg->origin);
	nodeFlow[pOrg->origin->id-1]=oFlow;
	bool find = false;
	int fnum =0;
	while(breadthList.size()>0)
	{
		fnum ++;
		TNM_SNODE* Tnode =NULL;

		TNM_SNODE* Hnode= breadthList.front();
		breadthList.pop_front();

		floatType HnodeFlow = nodeFlow[Hnode->id-1];

		if (HnodeFlow > entropyFlowPrecision) //to exclude some repeated inserted node
		{
			nodeFlow[Hnode->id-1]=0.0;//to exclude some repreatd inserted node
			floatType smallFlow = 0.0;
			TNM_SLINK* maxLink = NULL;
			floatType ff=0.0;
			vector<TNM_SLINK*>::iterator pv;
			//breadthList.push_back(node);
			for(pv = Hnode->forwStar.begin(); pv!=Hnode->forwStar.end(); pv++)
			{

				TNM_SLINK* flink = *pv; // search backward link for Hnode
				if(linkFlowVector[flink->id-1] > entropyFlowPrecision ) //  && !( blink->head ==endNode )
				{

					if (ff < linkApproachVector[flink->id-1])
					{
						ff = linkApproachVector[flink->id-1];
						maxLink = flink;
					}
					floatType dLinkFlow = HnodeFlow*linkApproachVector[flink->id-1];

					if (dLinkFlow < entropyFlowPrecision )
					{
						smallFlow += dLinkFlow;
						continue;
					}

					map<int,floatType>::iterator pv = por->changedLinkVector.find(flink->id);
					if (pv == por->changedLinkVector.end())
					{
						// not find the link
						por->changedLinkVector.insert(pair<int, floatType>(flink->id,por->oLinkFlowVector[flink->id-1]));
					}
					
					//
					
					por->oLinkFlowVector[flink->id-1]+=dLinkFlow;
					//
					

					//por->changedLinkVector.push_back(flink->id);

					//
					Tnode = flink->head;
					if (Tnode != sNode)
					{
						breadthList.push_back(Tnode);
						nodeFlow[Tnode->id-1]+=dLinkFlow;
					}
					else
					{
						dFlow+=dLinkFlow;
					}



				}
			}

			if (smallFlow > 0 && maxLink != NULL)
			{
				

				map<int,floatType>::iterator pv = por->changedLinkVector.find(maxLink->id);
				if (pv == por->changedLinkVector.end())
				{
					// not find the link
					por->changedLinkVector.insert(pair<int, floatType>(maxLink->id,por->oLinkFlowVector[maxLink->id-1]));
				}

				por->oLinkFlowVector[maxLink->id-1]+=smallFlow;

				Tnode = maxLink->head;
				if (Tnode != sNode)
				{
					breadthList.push_back(Tnode);
					nodeFlow[Tnode->id-1]+=smallFlow;
				}
				else
				{
					dFlow+=smallFlow;
				}
			}
			else
			{
				lostFlow+=smallFlow;
			}
		}
		else if(HnodeFlow < entropyFlowPrecision && HnodeFlow > 0)
		{
			nodeFlow[Hnode->id-1]=0.0;//to exclude some repreatd inserted node
			vector<TNM_SLINK*>::iterator pv;
			//breadthList.push_back(node);
			TNM_SLINK* maxLink = NULL;
			floatType ff = 0;
			for(pv = Hnode->forwStar.begin(); pv!=Hnode->forwStar.end(); pv++)
			{
				TNM_SLINK* flink = *pv;
				if (linkFlowVector[flink->id-1] > ff)
				{
					ff = linkFlowVector[flink->id-1];
					maxLink = flink;
				}
			}
			if (maxLink != NULL && linkFlowVector[maxLink->id-1] > entropyFlowPrecision)
			{
				map<int,floatType>::iterator pv = por->changedLinkVector.find(maxLink->id);
				if (pv == por->changedLinkVector.end())
				{
					// not find the link
					por->changedLinkVector.insert(pair<int, floatType>(maxLink->id,por->oLinkFlowVector[maxLink->id-1]));
				}
				por->oLinkFlowVector[maxLink->id-1]+=HnodeFlow;
				
				Tnode = maxLink->head;
				if (Tnode != sNode)
				{
					breadthList.push_back(Tnode);
					nodeFlow[Tnode->id-1]+=HnodeFlow;
				}
				else
				{
					dFlow+=HnodeFlow;
				}
			}
			else
			{
				lostFlow+=HnodeFlow;
			}
		}
		
	}

	if (abs(oFlow - dFlow)> 1e-3)
	{
		cout<<"Wrong in BFSNodeToOrigin"<<endl;
		cout<<"The difference is "<<oFlow-dFlow<<endl;
		cout<<"oFlow is "<<oFlow<<endl;
		cout<<"dFlow is "<<dFlow<<endl;
		//system("PAUSE");
	}
}
//

void TAP_iOEMA::BFSOriginToNode(PasOrigin2* por,TNM_SNODE* sNode,floatType dflow)
{
	//
	TNM_SORIGIN* pOrg = por->OriginPtr;

	floatType oFlow =0.0;
	lostFlow =0.0;
	deque<TNM_SNODE*> breadthList;//keep the node list in the breadth first search
	vector<floatType> nodeFlow(network->numOfNode,0.0);
	
	breadthList.push_back(sNode);
	nodeFlow[sNode->id-1] = dflow;
	bool find = false;
	int fnum =0;
	int hnum =0;
	while(breadthList.size()>0)
	{
		fnum ++;
		TNM_SNODE* Tnode =NULL;
		
		TNM_SNODE* Hnode= breadthList.front();
		breadthList.pop_front();
		
		floatType HnodeFlow = nodeFlow[Hnode->id-1];

		if (HnodeFlow > entropyFlowPrecision) //to exclude some repeated inserted node
		{
			//
			nodeFlow[Hnode->id-1]=0.0;//to exclude some repreatd inserted node
			floatType smallFlow = 0.0;
			vector<TNM_SLINK*>::iterator pv;
			TNM_SLINK* maxLink = NULL;
			floatType ff =0.0;
			//breadthList.push_back(node);
			for(pv = Hnode->backStar.begin(); pv!=Hnode->backStar.end(); pv++)
			{
				//floatType flowp=flowPrecision;

				TNM_SLINK* blink = *pv; // search backward link for Hnode
				if(por->oLinkFlowVector[blink->id-1] > entropyFlowPrecision ) //  && !( blink->head ==endNode )
				{
					floatType dLinkFlow = HnodeFlow*por->oLinkApproachVector[blink->id-1];
					if (por->oLinkApproachVector[blink->id-1] > ff)
					{
						ff = por->oLinkApproachVector[blink->id-1];
						maxLink = blink;
					}
					if (dLinkFlow < entropyFlowPrecision)
					{
						smallFlow +=dLinkFlow;
						//lostFlow +=dLinkFlow;
						continue;
					}

					
					linkFlowVector[blink->id-1]+=dLinkFlow; // add dLinkFlow to the vector ending sNode

					map<int,floatType>::iterator pv = por->changedLinkVector.find(blink->id);
					if (pv == por->changedLinkVector.end())
					{
						// not find the link
						por->changedLinkVector.insert(pair<int, floatType>(blink->id,por->oLinkFlowVector[blink->id-1]));
					}
					

					//por->changedLinkVector.insert(pair<int,floatType>(blink->id,por->oLinkFlowVector[blink->id-1]));
					por->oLinkFlowVector[blink->id-1] = por->oLinkFlowVector[blink->id-1] - dLinkFlow; //delete the dLinkFlow from por network
					
					Tnode = blink->tail;
					if (Tnode != pOrg->origin)
					{
						breadthList.push_back(Tnode);
						nodeFlow[Tnode->id-1]+=dLinkFlow;
					}
					else
					{
						oFlow = oFlow+ dLinkFlow;
						
						
					}
					


				}
			}
			//
			if (smallFlow > 0 && maxLink != NULL)
			{
				map<int,floatType>::iterator pv = por->changedLinkVector.find(maxLink->id);
				if (pv == por->changedLinkVector.end())
				{
					// not find the link
					por->changedLinkVector.insert(pair<int, floatType>(maxLink->id,por->oLinkFlowVector[maxLink->id-1]));
				}

				linkFlowVector[maxLink->id-1]+=smallFlow;
				por->oLinkFlowVector[maxLink->id-1]-=smallFlow;

				

				Tnode = maxLink->tail;
				if (Tnode != pOrg->origin)
				{
					breadthList.push_back(Tnode);
					nodeFlow[Tnode->id-1]+=smallFlow;
				}
				else
				{
					oFlow = oFlow+ smallFlow;

				}
			}
			else
			{
				lostFlow+=smallFlow;
			}

		}
		else if(HnodeFlow < entropyFlowPrecision && HnodeFlow > 0)
		{
			nodeFlow[Hnode->id-1]=0.0;//to exclude some repreatd inserted node
			vector<TNM_SLINK*>::iterator pv;
			//breadthList.push_back(node);
			TNM_SLINK* maxLink = NULL;
			floatType ff = 0;
			for(pv = Hnode->backStar.begin(); pv!=Hnode->backStar.end(); pv++)
			{
				TNM_SLINK* blink = *pv;
				if (por->oLinkFlowVector[blink->id-1] > ff)
				{
					ff = por->oLinkFlowVector[blink->id-1];
					maxLink = blink;
				}
			}
			if (maxLink != NULL && por->oLinkFlowVector[maxLink->id-1] > entropyFlowPrecision)
			{
				linkFlowVector[maxLink->id-1]+=HnodeFlow;
				map<int,floatType>::iterator pv = por->changedLinkVector.find(maxLink->id);
				if (pv == por->changedLinkVector.end())
				{
					// not find the link
					por->changedLinkVector.insert(pair<int, floatType>(maxLink->id,por->oLinkFlowVector[maxLink->id-1]));
				}
				else
				{
					hnum++;
				}
				por->oLinkFlowVector[maxLink->id-1]-=HnodeFlow;
				Tnode = maxLink->tail;
				if (Tnode != pOrg->origin)
				{
					breadthList.push_back(Tnode);
					nodeFlow[Tnode->id-1]+=HnodeFlow;
				}
				else
				{
					oFlow = oFlow+ HnodeFlow;
					

				}
			}
			else
			{
				lostFlow+=HnodeFlow;
			}
		}
	//	
    }

	if (abs(oFlow - dflow)> 1e-3)
	{

		cout<<"Wrong in BFSOriginToNode"<<endl;
		cout<<"sNode is "<<sNode->id<<endl;
		cout<<"Origin is "<<por->OriginPtr->origin->id<<endl;
		cout<<"The difference is "<<TNM_FloatFormat(oFlow-dflow,16,12)<<endl;
		cout<<"lostflow is "<<TNM_FloatFormat(lostFlow,16,12)<<endl;
		cout<<"oFlow is "<<TNM_FloatFormat(oFlow,16,12)<<endl;
		cout<<"dFlow is "<<TNM_FloatFormat(dflow,16,12)<<endl;
		//system("PAUSE");
	}
}
void TAP_iOEMA::MainLoop()
{
	cout<<"In main loop"<<endl;
	//
	TNM_SLINK* link;
	TNM_SNODE* node;
	lostFlow = 0;
	numOfChangedLink =0;
	numOfChangedNode = 0;
	changedFlow =0.0;
	deFlow =0.0;
	{
		for (int ni=0;ni<network->numOfNode;ni++)
		{
			
			TNM_SNODE* sNode = network->nodeVector[ni];
			//cout<<sNode->id<<" ";
			floatType inFlow = 0;
			int num = 0;
			for (int li=0;li<sNode->backStar.size();li++)
			{
				link = sNode->backStar[li];
				if (link->volume> entropyFlowPrecision)
				{
					inFlow+=link->volume;
					num++;
				}

			}
			if (curIter % 5 ==0)
			{
				sNode->buffer[0] = 100;
			}

			if ( inFlow > entropyFlowPrecision && sNode->buffer[0] > aveNodeChange/2.0  )
			{
				numOfChangedNode++;
				//cout<<"For Snode "<<sNode->id<<endl;
				if (sNode->id % 1000 == 0)
				{
					//cout<<"For Node : "<<sNode->id<<endl;

					//compute the entropy
					floatType interEntropy = CalEntropy();
					ProInfo * pif = new ProInfo;
					pif->iter = curIter;
					pif->Entro = interEntropy;
					pif->consis =0;
					pif->time = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
					EntropyInfoList.push_back(pif);
				}
				//lostFlow = 0;
				FromOrigintoNode(sNode);

				FromNodeToOrigin(sNode);

				UpdateOrigins(sNode);


			}
			

		}
	}



	flowChangePrecision = changedFlow/floatType(numOfChangedLink);
	
	preEntr = entr;
	entr = CalEntropy();
	aveNodeChange = changedFlow/floatType(numOfChangedNode);
	//cout<<"7"<<endl;

	ComputeConsistency();
	//
	ProInfo * pif = new ProInfo;
	pif->iter = curIter;
	pif->Entro = entr;
	pif->consis =0;
	pif->time = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	EntropyInfoList.push_back(pif);
	
	logFile<<"========================================================================"<<endl;
	logFile<<"The main iteration is : "<<curIter<<endl;
	logFile<<"The time now is : "<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
	logFile<<"The Entropy Value is : "<<entr<<endl;
	logFile<<"The OFV of initialization is : "<<OFV<<endl;
	logFile<<"The average changed flow for each link is "<<changedFlow/floatType(numOfChangedLink)<<endl;
	logFile<<"The changed flow is "<<changedFlow<<endl;
	logFile<<"The number of changed link is "<<numOfChangedLink<<endl;
	logFile<<"The average changed flow for each node is "<<aveNodeChange<<endl;
	logFile<<"The changed node is "<<numOfChangedNode<<endl;
	logFile<<"The lost flow is "<<lostFlow<<endl;

	//cout the result 
	cout<<"The main iteration is : "<<curIter<<endl;
	cout<<"The time now is : "<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
	cout<<"The Entropy Value is : "<<TNM_FloatFormat(entr,12,4)<<endl;
	cout<<"The average changed flow for each link is "<<changedFlow/floatType(numOfChangedLink)<<endl;
	cout<<"The average changed flow for each node is "<<aveNodeChange<<endl;
	if(curIter == 15)
	{
		cout<<"The algorithm is stopping because the total changed flow is less than "<<3<<endl;
		logFile<<"The algorithm is stopping because the total changed flow is less than "<<3<<endl;
		maxMainIter = 2;
	}
	//system("PAUSE");
	ziggIter=0;


}


void TAP_iOEMA::UpdateOrigins2()
{
	for (int oi=0;oi<originVector.size();oi++)
	{

	}
}
void TAP_iOEMA::UpdateOrigins(TNM_SNODE* sNode)
{
	//cout<<"The size of origins are : "<<originVector.size()<<endl;
	TNM_SLINK* link;
	//int lid;
	TNM_SNODE* head;
	floatType tflow = 0.0;
	int tlink =0;
	int num =0;
	floatType sumc = 0.0;
	floatType avec = 0.0;
	int numc =0;
	for (int oi=0;oi<originVector.size();oi++)
	{
		PasOrigin2* por = originVector[oi];
		
		num+=por->changedLinkVector.size();
		
		//
		vector<int> nodes;
		nodes.clear();
		//////////////////////////////////////////////////////////////////////////
		for (map<int,floatType>::iterator pv = por->changedLinkVector.begin(); pv!=por->changedLinkVector.end();pv++)
		{
			int lid = pv->first;
			floatType ch = abs(pv->second-por->oLinkFlowVector[lid-1]);
			//cout<<"ch is "<<ch<<endl;
			
			if (ch > entropyFlowPrecision)
			{
				//cout<<"LInk "<<lid<<" 'ch is : "<<ch<<endl;
				numOfChangedLink++;
				changedFlow+=ch;
				//
				numc++;
				sumc+=ch;
				//por->oLinkFlowVector[lid-1] = por->oNewLinkFlowVector[lid-1];
				nodes.push_back(network->linkVector[lid-1]->head->id);
			}
		}

	    
		//update the approcach
		int nid;
		for (int ni=0;ni<nodes.size();ni++)
		{
			nid = nodes[ni];
			head = network->nodeVector[nid-1];


			floatType af = 0.0;
			for (int lj=0;lj<head->backStar.size();lj++)
			{
				if (por->oLinkFlowVector[head->backStar[lj]->id-1] > entropyFlowPrecision)
				{
					af+=por->oLinkFlowVector[head->backStar[lj]->id-1];
				}
				
			}
			por->onodeflow[head->id-1] = af;
			
			if (af > entropyFlowPrecision)
			{
				for (int lj=0;lj<head->backStar.size();lj++)
				{
					if (por->oLinkFlowVector[head->backStar[lj]->id-1] > entropyFlowPrecision)
					{
						por->oLinkApproachVector[head->backStar[lj]->id-1] = por->oLinkFlowVector[head->backStar[lj]->id-1]/af;
					}
					else
					{
						por->oLinkApproachVector[head->backStar[lj]->id-1] = 0.0;
					}
					
				}
			}
			else
			{
				for (int lj=0;lj<head->backStar.size();lj++)
				{
					por->oLinkApproachVector[head->backStar[lj]->id-1] = 0.0;
				}
			}
		}
		
		por->changedLinkVector.clear();
	}
	sNode->buffer[0] = sumc;


}

TAP_iOEMA::TAP_iOEMA()
{
	//pid = 0;
	flowChangePrecision =1.0;
	aveNodeChange = 0.0;
}

TAP_iOEMA::~TAP_iOEMA()
{
	
}

void TAP_iOEMA::CalLinkApproach()
{

	for (int oi=0; oi<network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		
		int oid = pOrg->m_class;
		PasOrigin2* por = POList[oid];

		
		TNM_SNODE *node ;
		TNM_SLINK* link;
		for (int ni=0; ni<network->numOfNode; ni++)
		{
			node = network->nodeVector[ni];
			floatType af = 0.0;
			por->onodeflow[pOrg->origin->id-1] = 0.0;
			if (node != pOrg->origin)
			{
				//cal the onode flow
				for (int li=0; li<node->backStar.size(); li++)
				{
					link = node->backStar[li];
					//if (por->oLinkFlowVector[link->id-1] > entropyFlowPrecision)
					{
						af += por->oLinkFlowVector[link->id-1];
					}

				}
				//
				por->onodeflow[node->id-1] = af;
				//cal the link approach
				for (int li=0; li<node->backStar.size(); li++)
				{
					link = node->backStar[li];
					//por->oNewLinkFlowVector[li] = por->oLinkFlowVector[li];
					if (af > entropyFlowPrecision  ) //&& por->oLinkFlowVector[link->id-1] > entropyFlowPrecision
					{
						por->oLinkApproachVector[link->id-1] = por->oLinkFlowVector[link->id-1]/af;
					}
					else
					{
						por->oLinkApproachVector[link->id-1] = 0.0;
					}

				}
			}
		}

	}
}

double TAP_iOEMA::RemoveNodeBasedCycle2(TNM_SNODE* dest)
{
	TNM_SNODE* sNode;
	TNM_SLINK* sLink;

	CNList.clear();//clear the CNList

	for(int ni=0; ni<network->numOfNode; ni++)
	{
		sNode  = network->nodeVector[ni];
		sNode->scanStatus =0;
		sNode->tmpNumOfIn = sNode->NumOfOrgInLink();
	}

	NodeDFS(dest);



	return 0;
}

void TAP_iOEMA::NodeDFS(TNM_SNODE* serNode)
{
	if( serNode->scanStatus ==1)
	{
		//check if head is in CNList
		TNM_SNODE* cn;
		vector<TNM_SNODE*> cycleNode;
		bool exist =false;

		for(deque<TNM_SNODE*>::iterator pw = CNList.begin(); pw!=CNList.end(); pw++)
		{
			cn = *pw;
			if(cn == serNode)
			{
				exist = true;
			}
			if(exist)
			{
				cycleNode.push_back(cn);
			}
		}
		cycleNode.push_back(serNode);

		if(exist)
		{
			deque<TNM_SLINK*> cycleLink;
			TNM_SNODE* ltail;
			TNM_SNODE* lhead;
			for(int i=0; i<cycleNode.size()-1; i++)
			{
				lhead = cycleNode[i];
				ltail = cycleNode[i+1];
				//search the link
				for(int j=0; j<ltail->forwStar.size(); j++)
				{
					TNM_SLINK* sl = ltail->forwStar[j];
					if( sl->head == lhead) //linkFlowVector[sl->id-1]>0 &&
					{
						cycleLink.push_back(sl);
					}
				}
			}

			if(cycleLink.size() == cycleNode.size()-1)
			{

				//delete the cycle
				//delete cycle flow
				TNM_SLINK* dlink;
				floatType dflow=100000000000;
				for(int j =0 ; j<cycleLink.size(); j++)
				{
					TNM_SLINK* link = cycleLink[j];
					if(dflow > linkFlowVector[link->id-1])
					{
						dflow = linkFlowVector[link->id-1];
						dlink = link;
					}
				}
				if (dflow > 0)
				{
					//
					PreCycleFlow += dflow;
					NumOfCycle++;
					//deleting cycle flows
					for(int j=0; j< cycleLink.size(); j++)
					{
						TNM_SLINK* link = cycleLink[j];
						linkFlowVector[link->id-1] -=dflow;
						
					}

				}


			}
			return;
		}

		return;
	}
	//when no cycle exist
	serNode->scanStatus =1;
	CNList.push_back(serNode);

	TNM_SLINK* serLink;
	//search the subnetwork
	for(vector<TNM_SLINK*>::iterator pv = serNode->backStar.begin(); pv!=serNode->backStar.end(); pv++)
	{
		serLink = *pv;
		//
		if(linkFlowVector[serLink->id-1] > 0 ) //&& serLink->markStatus ==0
		{
			TNM_SNODE* tail = serLink->tail;
			NodeDFS(tail);
		}
	}

	CNList.pop_back();

}

void TAP_iOEMA::BuildAllPath(TNM_SORIGIN* orgn, PasOrigin2* porgn, floatType entropyFlowPrecision)
{
	//orgn->MarkOBLinksOnNet();
	TNM_SNODE* node=orgn->origin;
	//cout<<"N-"<<node->id<<" ";
	GotoNextNode(node,orgn,porgn,entropyFlowPrecision);

}

void TAP_iOEMA::GotoNextNode(TNM_SNODE* node, TNM_SORIGIN* orgn, PasOrigin2* porgn, floatType entropyFlowPrecision)
{
	for(int i=0;i<node->forwStar.size();i++)
	{
		TNM_SLINK* link=node->forwStar[i];

		if(porgn->oLinkFlowVector[link->id-1]>entropyFlowPrecision)
		{
			TNM_SNODE* hnode=link->head;
			//cout<<"N-"<<hnode->id<<" ";
			hnode->pathElem->via=link;
			TNM_SDEST* dest;
			bool isdest=false;
			for(int j=0;j<orgn->numOfDest;j++)
			{
				if(orgn->destVector[j]->dest->id==hnode->id && orgn->destVector[j]->dest->id!=orgn->origin->id)
				{
					dest=orgn->destVector[j];
					isdest=true;
					break;
				}
			}
			if(isdest)
			{
				//new path;
				TNM_SPATH* path=new TNM_SPATH;
				path->path.clear();
				TNM_SNODE* snode=hnode;
				path->flow=dest->assDemand;
				while(snode->id!=orgn->origin->id)
				{
					TNM_SLINK* slink=snode->pathElem->via;
					path->path.push_back(slink);
					//cout<<"L-"<<link->id<<" ";
					if(porgn->oLinkFlowVector[slink->id-1]>entropyFlowPrecision)
						path->flow*=porgn->oLinkApproachVector[slink->id-1];
					else
						path->flow=0;
					snode=slink->tail;
				}
				dest->pathSet.push_back(path);
			}
			//else
			{
				GotoNextNode(hnode,orgn,porgn,entropyFlowPrecision);
			}

		}
	}
	// goto previous aspect;
}

void TAP_iOEMA::BuildSpecialPath(TNM_SORIGIN* orgn, PasOrigin2* porgn, floatType entropyFlowPrecision)
{
	for(int i=0;i<network->numOfNode;i++)
		node2dest[network->nodeVector[i]]=NULL;
	for(int j=0;j<orgn->numOfDest;j++)
	{
		TNM_SDEST* dest=orgn->destVector[j];
		node2dest[dest->dest]=dest;
	}
	count_ofbuildpath=0;
	TNM_SNODE* node=orgn->origin;
	GotoNextNode_buildSpecialPath(node,orgn,porgn,entropyFlowPrecision);
}

void TAP_iOEMA::GotoNextNode_buildSpecialPath(TNM_SNODE* node, TNM_SORIGIN* orgn, PasOrigin2* porgn, floatType entropyFlowPrecision)
{
	for(int i=0;i<node->forwStar.size();i++)
	{
		TNM_SLINK* link=node->forwStar[i];

		if(porgn->oLinkFlowVector[link->id-1]>entropyFlowPrecision)
		{
			TNM_SNODE* hnode=link->head;
			//cout<<"N-"<<hnode->id<<" ";
			hnode->pathElem->via=link;
			//if(porgn->oLinkBufferVector[link->id-1]>1.0) count_ofbuildpath++;
			if(link->markStatus==1) count_ofbuildpath++;
			if(count_ofbuildpath>0)
			{
				TNM_SDEST* dest;
				bool isdest=false;

				if(node2dest[hnode]!=NULL)
				{
					dest=node2dest[hnode];
					if(dest->dest->id!=orgn->origin->id)
						isdest=true;
				}
				if(isdest)
				{
					//new path;
					TNM_SPATH* path=new TNM_SPATH;
					path->path.clear();
					TNM_SNODE* snode=hnode;
					path->flow=dest->assDemand;
					while(snode->id!=orgn->origin->id)
					{
						TNM_SLINK* slink=snode->pathElem->via;
						path->path.push_back(slink);
						//cout<<"L-"<<link->id<<" ";
						if(porgn->oLinkFlowVector[slink->id-1]>entropyFlowPrecision)
							path->flow*=porgn->oLinkApproachVector[slink->id-1];
						else
							path->flow=0;
						snode=slink->tail;
					}
					dest->pathSet.push_back(path);
				}
			}
			GotoNextNode_buildSpecialPath(hnode,orgn,porgn,entropyFlowPrecision);
			//if(porgn->oLinkBufferVector[link->id-1]>1.0)  count_ofbuildpath--;
			if(link->markStatus==1)  count_ofbuildpath--;
		}
	}
	// goto previous aspect;
}

void TAP_iOEMA::SaveProSolution()
{
	string oueFileName = outFileName+".opro";
	ofstream out;
	cout<<"\tWriting O2P solution information into "<<outFileName<<" for post process"<<endl;
	if (!TNM_OpenOutFile(out,oueFileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .opro file to write O2P solution information!"<<endl;
		return;
	}
	//
	cout<<"\tWriting the network link flow info ..."<<endl;
	out<<TNM_IntFormat(network->numOfLink,6)<<endl;
	for (int li=0; li<network->numOfLink; li++)
	{
		TNM_SLINK* sl = network->linkVector[li];
		out<<TNM_IntFormat(sl->tail->id,6)<<TNM_IntFormat(sl->head->id,6)<<setw(floatWidth)<<TNM_FloatFormat(sl->volume,20,11)<<setw(floatWidth);
	}
	out<<endl;
	//
	cout<<"\t Writing the origin-based link flow info..."<<endl;
	out<<TNM_IntFormat(network->numOfOrigin,6)<<endl;
	for (int oi=0; oi< network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		out<<TNM_IntFormat(pOrg->id(),6)<<TNM_IntFormat(network->numOfLink,6)<<endl;
		PasOrigin2* porgn=POList[pOrg->m_class];

		for (int oj=0; oj<network->numOfLink; oj++)
		{
			TNM_SLINK* link = network->linkVector[oj];
			//if (olink->oFlow > 0)
			{
				out<<TNM_IntFormat(link->id,6)<<setw(floatWidth)<<setw(floatWidth)<<TNM_FloatFormat(porgn->oLinkFlowVector[link->id-1],20,11)<<setw(floatWidth)<<setw(floatWidth);
			}
			
		}
		out<<endl; 
	}

	out.close();
}

void TAP_iOEMA::SaveOriginLinkFlow(const string& fileName)
{
	ofstream out;
	cout<<"\tWriting O2P origin-based link flows into "<<fileName<<endl;
	if (!TNM_OpenOutFile(out,fileName))
	{
		cout<<"\n\tCannot open O2P_olinkflow.txt to write origin-based link flows!"<<endl;
		return;
	}

	out<<"origin\ttail_node\thead_node\tflow"<<endl;
	out<<scientific<<setprecision(12);
	for (int oi=0; oi<network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		PasOrigin2* porgn = POList[pOrg->m_class];
		for (int li=0; li<network->numOfLink; li++)
		{
			floatType flow = porgn->oLinkFlowVector[li];
			if (flow > 1e-10)
			{
				TNM_SLINK* link = network->linkVector[li];
				out<<pOrg->id()<<'\t'<<link->tail->id<<'\t'<<link->head->id<<'\t'<<flow<<endl;
			}
		}
	}
	out.close();
}

int TAP_iOEMA::Reportpath()
{

	ofstream f;
	string name=outFileName+".pth";
	TNM_OpenOutFile(f,name);
	int pthid =0;
	f<<"path id      "<<"origin       "<<" dest      "<<"     path flow    "<<"   num of link    "<<"   links  "<<endl;
	for(int i=0;i<network->numOfOrigin;i++)
	{
		TNM_SORIGIN* orgn=network->originVector[i];
		PasOrigin2* porgn=POList[orgn->m_class];
		BuildAllPath(orgn,porgn,1e-10);
		for(int j=0;j<orgn->numOfDest;j++)
		{
			TNM_SDEST* dest=orgn->destVector[j];
			string rel;
			for(int k=0;k<dest->pathSet.size();k++)
			{
				TNM_SPATH* path=dest->pathSet[k];

				pthid ++;
				path->id=pthid;

				char pp[50];
				sprintf(pp,"%d\t%d\t%d\t%.10f\t%zu\t", path->id, orgn->origin->id, dest->dest->id, path->flow, path->path.size());
				rel += pp;
				for (int li=0; li<path->path.size();li++)
				{
					TNM_SLINK* link = path->path[li];
					rel += to_string(link->id) + " ";

				}
				rel += "\n";

			}
			f<<rel;
			dest->EmptyPathSet();
		}
	}
	f.close();
	return 0;
}

// ---------------- MEUE (MEUE_DOB implementation base + TAPAS variant) ----------------

MEUE_DOB::MEUE_DOB()
{
    Gap = RG = FI = MaxFI = 0.0;
    SubIter = 0;
    reportbushflow = false;
    reportpathflow = false;
    addbushlink = iniiter = totalpas = numpas = validpas = 0;
    MaxInnerConv = 0.0;
    FIsatisfied = false;
    OUEfileType = 1;
    pntCoe = 0.03;
    MaxMainGapConv = MaxMainFIConv = 1e-7;
    MaxSubConv = 1e-15;
    MaxSubIter = 5;
    alpha = 0.25;
    begintime = currenttime = 0;
    ziggIter = badIter = 0;
    ziggIndicator = 1.0;
    m_OOFV = m_BOFV = m_OConv = m_BConv = 0.0;
}

MEUE_DOB::~MEUE_DOB()
{
}

MEUE_TAPAS::MEUE_TAPAS()
{
    PasStructID = oldcurIter = 0;
    rt = 1;
    pre0 = pre2 = pre4 = pre6 = pre7 = false;
}

MEUE_TAPAS::~MEUE_TAPAS()
{
    for (size_t i = 0; i < PNList_ly.size(); ++i) delete PNList_ly[i];
    for (size_t i = 0; i < PasList_ly.size(); ++i) delete PasList_ly[i];
}

void MEUE_DOB::ClearSolution()
{
	//clear the total link flow
	for (int li=0; li<network->numOfLink; li++)
	{
		TNM_SLINK* slink = network->linkVector[li];
		slink->volume = 0;
	}
	//clear the origin based link flow
	for (int oi=0; oi<network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		for (int oj =0; oj<pOrg->obLinkVector.size(); oj++)
		{
			ORGLINK* olink = pOrg->obLinkVector[oj];
			olink->oFlow = 0;
		}
	}
}

//Create pas from bushs solution;

void MEUE_DOB::SaveUESolution()
{
	string oueFileName = outFileName+".oue";
	ofstream out;
	cout<<"\tWriting UE solution information into "<<outFileName<<" for post process"<<endl;
	if (!TNM_OpenOutFile(out,oueFileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .oue file to write UE solution information!"<<endl;
		return;
	}
	//
	cout<<"\tWriting the network link flow info ..."<<endl;
	out<<TNM_IntFormat(network->numOfLink,6)<<endl;
	for (int li=0; li<network->numOfLink; li++)
	{
		TNM_SLINK* sl = network->linkVector[li];
		out<<TNM_IntFormat(sl->tail->id,6)<<TNM_IntFormat(sl->head->id,6)<<setw(floatWidth)<<TNM_FloatFormat(sl->volume,20,11)<<setw(floatWidth);
	}
	out<<endl;
	//
	cout<<"\t Writing the origin-based link flow info..."<<endl;
	out<<TNM_IntFormat(network->numOfOrigin,6)<<endl;
	for (int oi=0; oi< network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		out<<TNM_IntFormat(pOrg->id(),6)<<TNM_IntFormat(pOrg->obLinkVector.size(),6)<<endl;
		for (int oj=0; oj<pOrg->obLinkVector.size(); oj++)
		{
			ORGLINK* olink = pOrg->obLinkVector[oj];
			//if (olink->oFlow > 0)
			{
				out<<TNM_IntFormat(olink->linkPtr->tail->id,6)<<TNM_IntFormat(olink->linkPtr->head->id,6)<<setw(floatWidth)<<setw(floatWidth)<<TNM_FloatFormat(olink->oFlow,20,11)<<setw(floatWidth)<<setw(floatWidth);
			}
			
		}
		out<<endl; 
	}
	cout<<"\tWriting the PAS info..."<<endl;
	out<<TNM_IntFormat(PasList.size(),6)<<endl;
	for (int pi = 0; pi < PasList.size(); pi++)
	{
		PairSegment2* pas = PasList[pi];
		out<<TNM_IntFormat(pas->seg1.size(),6);
		for (int si=pas->seg1.size()-1; si>=0; si--)
		{
			TNM_SLINK* sl = pas->seg1[si];
			out<<TNM_IntFormat(sl->tail->id,6)<<TNM_IntFormat(sl->head->id,6);
		}
		out<<endl;
		out<<TNM_IntFormat(pas->seg2.size(),6);
		for (int si= pas->seg2.size()-1; si>=0; si--)
		{
			TNM_SLINK* sl = pas->seg2[si];
			out<<TNM_IntFormat(sl->tail->id,6)<<TNM_IntFormat(sl->head->id,6);
		}
		out<<endl;
		out<<TNM_IntFormat(pas->OriginList.size(),6);
		for (int si=0; si<pas->OriginList.size();si++)
		{
			TNM_SORIGIN* pOrg = pas->OriginList[si]->org;
			out<<TNM_IntFormat(pOrg->id(),6);

		}
		out<<endl;
	}

	out.close();
}


void MEUE_DOB::PreProcess()
{
	stepSize                  = 1.0;
	ziggIter                  = 0;
	badIter                   = 0;
	convIndicator             = 1e3;           
	ziggIndicator             = 1.0;
	//SetInitTerm();
	OFV                       = pow(2.0, 52.0); //set the intial value to a fairly big number.
	numLineSearch             = 0;
	m_OOFV = m_BOFV = OFV;
	m_OConv = m_BConv = convIndicator;
	alpha=0.25;
	ly_IterResult.clear();
	ly_BushFlowResult.clear();
	RG=0;
	FIsatisfied=false;
	//ClearIterRecord();
}

void MEUE_DOB::SetInipntCoe()
{
	pntCoe=0.03;
}

void MEUE_DOB::SetiniMultipler()
{
	

	for(int a=0;a<network->numOfLink;a++)
	{
		network->linkVector[a]->buffer[0]=0;
	}
}

void MEUE_DOB::UpdateMultipler()
{
	for(int a=0;a<network->numOfLink;a++)
	{
		network->linkVector[a]->buffer[0] -= pntCoe*(network->linkVector[a]->volume-network->linkVector[a]->buffer[1]);
	}
}

void MEUE_DOB::UpdateFlowInfeasibility()
{
	FI=0;
	MaxFI=0;
	for(int a=0;a<network->numOfLink;a++)
	{
		if(abs(network->linkVector[a]->buffer[1]-network->linkVector[a]->volume)>MaxFI)
			MaxFI=abs(network->linkVector[a]->buffer[1]-network->linkVector[a]->volume);
		FI += pow(network->linkVector[a]->buffer[1]-network->linkVector[a]->volume,2);
		//cout<<"LID="<<network->linkVector[a]->id<<",flow different="<<network->linkVector[a]->buffer[1]-network->linkVector[a]->volume<<",Multipler="<<network->linkVector[a]->buffer[0]<<endl;
	}
	FI=pow(FI/network->numOfLink,0.5);
}

floatType MEUE_DOB::GetRelativeGAP()
{
	floatType rg=0;
	floatType tt=0;
	for(int i=0;i<network->numOfOrigin;i++)
	{
		TNM_SORIGIN* orgn=network->originVector[i];
		orgn->MarkOBLinksOnNet();
		//update link and node entropy
		vector<ORGLINK*>::iterator pl; 
		//int nunu=0;
		floatType l1=0;
		floatType p1=0;
		floatType d1=0;
		floatType x1=0;
		for(pl=orgn->obLinkVector.begin();pl<orgn->obLinkVector.end();pl++)
		{//nunu++;
			TNM_SNODE* node=(*pl)->linkPtr->head;
			node->m_orgNode->Inflow=0.0;
			vector<TNM_SLINK*>::iterator bkl;
			for(bkl=node->backStar.begin();bkl<node->backStar.end();bkl++)
			{
				if((*bkl)->oLinkPtr!=NULL)
					node->m_orgNode->Inflow += (*bkl)->oLinkPtr->oFlow;
			}

			if((*pl)->oFlow>0)
			{
				(*pl)->entro=GetOlinkEntro((*pl)->linkPtr);
				(*pl)->fdentro=GetOlinkfdEntro((*pl)->linkPtr);
				tt += (*pl)->entro*(*pl)->oFlow;
				l1+=(*pl)->entro*(*pl)->oFlow;
				x1+=(*pl)->oFlow;
			}
			else
			{
				(*pl)->entro=GetOlinkEntro((*pl)->linkPtr);
				(*pl)->fdentro=GetOlinkfdEntro((*pl)->linkPtr);
			}
			if((*pl)->oFlow<0 || _isnan((*pl)->oFlow))
				cout<<"Error!! The flow of Link "<<(*pl)->linkPtr->id<<" of bush "<<orgn->origin->id<<" is "<<(*pl)->oFlow<<endl;
			if((*pl)->linkPtr->volume<0 || _isnan((*pl)->linkPtr->volume))
				cout<<"Error!! The flow of Link "<<(*pl)->linkPtr->id<<" is "<<(*pl)->linkPtr->volume<<endl;
			if(node->m_orgNode->Inflow<0 || _isnan(node->m_orgNode->Inflow))
				cout<<"Error!! The flow into node "<<node->id<<" is "<<node->m_orgNode->Inflow<<endl;
		}

		BuildPathTreeII(orgn);
		for(int j=0;j<orgn->numOfDest;j++)
		{
			TNM_SDEST* dest=orgn->destVector[j];
			rg += dest->dest->pathElem->cost*dest->assDemand;
			p1+=dest->dest->pathElem->cost*dest->assDemand;
			d1+=dest->assDemand;
		}
		orgn->RemarkOBLinksOnNet();
	}
	
	rg=(tt-rg)/abs(tt);
	return rg;
}

void MEUE_DOB::ReadTAPASOUE()
{
	//
	cout<<"Reading the OUE file from TAPAS output"<<endl;
	string InOUEName;
	if(OUEfileType == 1)
		InOUEName =  inFileName + "_iGP_tapas.oue";
	else
		InOUEName =  inFileName + "_iTAPAS_tapas.oue";
	cout<<"Reading "<<InOUEName<<endl;
	ifstream oueFile;
	if (!TNM_OpenInFile(oueFile, InOUEName))
	{
		cout<<"\t Cannot Open the oueFile "<<endl;

	}

	map<int,TNM_SORIGIN*> originMap;
	originMap.clear();
	for(int i=0;i<network->numOfOrigin;i++)
	{
		TNM_SORIGIN* orgn=network->originVector[i];
		originMap.insert(map<int,TNM_SORIGIN*>::value_type(orgn->id(),orgn));
		//orgn->obLinkVector.clear();
		for(vector<ORGLINK*>::iterator pol=orgn->obLinkVector.begin();pol<orgn->obLinkVector.end();)
		{
			ORGLINK* olink=(*pol);
			pol=orgn->obLinkVector.erase(pol);
			//link->oLinkPtr = NULL;
			delete olink;
		}
		orgn->obLinkVector.clear();
	}
	//
	string line;
	vector<string> words;
	getline(oueFile,line);

	//cout<<"1"<<endl;
	clock_t t1=clock();
	while (!oueFile.eof())
	{
		//
		getline(oueFile,line);
		TNM_GetWordsFromLine(line,words);
		
		if (words.size() == 4)
		{
			int originID, tailID, headID;
			floatType flow;
			TNM_FromString(originID,words[0],std::dec);
			TNM_FromString(tailID,words[1],std::dec);
			TNM_FromString(headID,words[2],std::dec);
			TNM_FromString(flow,words[3],std::dec);
			//
			TNM_SORIGIN* pOrg = originMap[originID];
			pOrg->MarkOBLinksOnNet();
			//PasOrigin2* por = POList[pOrg->m_class];
			//
			TNM_SNODE* tailPtr = network->nodeVector[tailID-1];
			//TNM_SNODE* headPtr = network->nodeVector[headID-1];
			TNM_SLINK* crLink = NULL;
			for (int ni=0; ni<tailPtr->forwStar.size();ni++)
			{
				TNM_SLINK* sl = tailPtr->forwStar[ni];
				if(sl->head->id == headID)
				{
					crLink = sl;
					break;
				}
			}
			//
			if (crLink == NULL)
			{
				cout<<"wrong in reading OUE from TAPAS"<<endl;
				system("PAUSE");
			}
			//por->oLinkFlowVector[crLink->id-1] = flow;
			if (crLink->oLinkPtr == NULL)
			{
				ORGLINK* olink = new ORGLINK;
				olink->linkPtr = crLink;
				olink->oFlow = flow;
				crLink->oLinkPtr = olink;
				pOrg->obLinkVector.push_back(olink);
				//por->olinkList[sl->id-1] = olink;
			}
			else
			{
				ORGLINK* olink = crLink->oLinkPtr;
				olink->oFlow = flow;
			}
			pOrg->RemarkOBLinksOnNet();
		}


	}
	oueFile.close();
	
	cout<<"Reading Total Link flow from TAPAS output"<<endl;

	string OUEName;
	if(OUEfileType == 1)
		OUEName =  inFileName + "_iGP.lfp";
	else
		OUEName =  inFileName + "_iTAPAS.lfp";
	ifstream TotalLinkFile;
	if (!TNM_OpenInFile(TotalLinkFile, OUEName))
	{
		cout<<"\t Cannot Open the oueFile "<<endl;

	}
	//
	while (!TotalLinkFile.eof())
	{
		//
		getline(TotalLinkFile,line);
		//cout<<line<<endl;

		TNM_GetWordsFromLine(line,words);
		//cout<<"words' size "<<words.size()<<endl;
		//system("PAUSE");
		if (words.size() == 7)
		{
			int linkID;
			istringstream idReader(words[0]);
			if (!(idReader >> linkID)) continue;
			//
			int tailID,headID;
			floatType volume,cost;
			TNM_FromString(tailID,words[1],std::dec);
			TNM_FromString(headID,words[2],std::dec);
			TNM_FromString(volume,words[4],std::dec);
			TNM_FromString(cost,words[5],std::dec);

			TNM_SNODE* tailPtr = network->nodeVector[tailID-1];
			TNM_SLINK* crLink = NULL;
			for (int ni=0; ni< tailPtr->forwStar.size();ni++)
			{
				TNM_SLINK* sl = tailPtr->forwStar[ni];
				if (sl->head->id == headID)
				{
					//
					crLink = sl;
					break;
				}
			}
			if (crLink == NULL)
			{
				cout<<"wrong in reading total flow"<<endl;
				system("PAUSE");
			}
			//
			crLink->volume = volume;
			crLink->cost = cost;

		}

	}
	TotalLinkFile.close();
	//getchar();
}

void MEUE_DOB::CalculateOFV()
{
	OFV=0;
	for(int i=0;i<network->numOfOrigin;i++)
	{
		TNM_SORIGIN* orgn=network->originVector[i];
		orgn->MarkOBLinksOnNet();
		

		for(int ni=0;ni<orgn->tplNodeVector.size();ni++)
		{
			ORGNODE* onode=orgn->tplNodeVector[ni];
			TNM_SNODE* snode=onode->nodePtr;
			if (snode->m_orgNode->Inflow > 1e-6)
			{
				for (int li=0; li< snode->backStar.size(); li++)
				{
					TNM_SLINK* slink = snode->backStar[li];
					if (slink->oLinkPtr != NULL)
					{
						if(slink->oLinkPtr->oFlow>1e-6)
							OFV+=slink->oLinkPtr->oFlow*log(slink->oLinkPtr->oFlow/snode->m_orgNode->Inflow);
					}
				}
			}
		}

		

		orgn->RemarkOBLinksOnNet();
	}
	OFV=-OFV;
}

void MEUE_DOB::CheckLinkFlow()
{
	map<int,floatType> linkflowmap;
	linkflowmap.clear();
	for (int li=0; li<network->numOfLink; li++)
	{
		TNM_SLINK* slink = network->linkVector[li];
		linkflowmap.insert(map<int,floatType>::value_type(slink->id,0));
	}
	for (int oi=0; oi<network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		pOrg->MarkOBLinksOnNet();
		for (int li=0; li<network->numOfLink; li++)
		{
			TNM_SLINK* slink = network->linkVector[li];
			if(slink->oLinkPtr!=NULL)
				linkflowmap[slink->id]+=slink->oLinkPtr->oFlow;
		}
		pOrg->RemarkOBLinksOnNet();
	}
	for (int li=0; li<network->numOfLink; li++)
	{
		TNM_SLINK* slink = network->linkVector[li];
		if (abs(linkflowmap[slink->id] - slink->volume) >1e-4)
		{
			cout<<"This link is unconsistent with total flow"<<endl;
			cout<<"The flow is : "<<linkflowmap[slink->id]<<endl;
			cout<<"The Volume is : "<<slink->volume<<endl;
			cout<<"The difference is : "<<abs(linkflowmap[slink->id]-slink->volume)<<endl;
			//system("PAUSE");
		}
	}

	
}

void MEUE_DOB::ComplementUEBushII()
{
	cout<<"Complement UE Bush";
	addbushlink=0;
	for(int i=0;i<network->numOfOrigin;i++)
	{
		TNM_SORIGIN* orgn=network->originVector[i];
		cout<<orgn->origin->id<<" ";
		orgn->MarkOBLinksOnNet();
		//if(orgn->origin->id%300==1)
		//	cout<<"read origin "<<orgn->origin->id;
		network->UpdateSP(orgn->origin);
		
		CNList.clear();
		for (std::vector<ORGNODE*>::iterator pl = orgn->tplNodeVector.begin();pl!=orgn->tplNodeVector.end(); pl++) CNList.push_back((*pl)->nodePtr);
		while(!CNList.empty())
		{
			TNM_SNODE* node=CNList.front();
			CNList.pop_front();
			bool skipthis=true;
			for(PTRTRACE pv = node->backStar.begin(); pv!= node->backStar.end(); pv++)
			{
				if((*pv) && (*pv)->oLinkPtr) {skipthis=false;break;}
			}
			if(skipthis) continue;
			for(int i=0;i<node->forwStar.size();i++)
			{
				TNM_SLINK* link=node->forwStar[i];
				TNM_SNODE* ttail=link->tail;
				TNM_SNODE* hhead=link->head;
				//bool add=false;
				
				if(link->oLinkPtr==NULL)
				{
					floatType res=link->tail->pathElem->cost+link->cost-link->head->pathElem->cost;
					if(res>=0 && res<1e-8)
					{
						bool havecycle=false;
						//some link's cost=0. adding it may appear cycle
						if(link->cost==0)
						{
							for(int fi=0;fi<ttail->backStar.size();fi++)
							{
								TNM_SLINK* llink=ttail->backStar[fi];
								if(llink->tail->id==hhead->id && llink->oLinkPtr!=NULL)
								{
									havecycle=true;
									//cout<<"tt";
								}
							}
						}
						//cout<<havecycle;
						if(!havecycle)
						{
							bool nno=true;
							for(PTRTRACE pt = hhead->backStar.begin(); pt!= hhead->backStar.end(); pt++)
							{
								if((*pt) && (*pt)->oLinkPtr) {nno=false;break;}
							}
							if(nno) CNList.push_back(hhead);

							ORGLINK* olink=new ORGLINK;
							olink->linkPtr=link;
							link->oLinkPtr=olink;
							olink->oFlow=0;
							orgn->obLinkVector.push_back(olink);
							addbushlink++;
						}
					}
				}

			}
		}

		for(int i = 0;i<network->numOfNode;i++)
		{
			TNM_SNODE* node = network->nodeVector[i];
			node->tmpNumOfIn = node->NumOfOrgInLink();
			if(node->tmpNumOfIn == 0 && node!= orgn->origin)  //for such node, no orgnode associated except origin.
			{
				bool err=false;
				for(PTRTRACE pv = node->forwStar.begin(); pv!= node->forwStar.end(); pv++)
				{
					if((*pv) && (*pv)->oLinkPtr) 
					{
						err=true;
						TNM_SLINK* link=(*pv);
						floatType res=link->tail->pathElem->cost+link->cost-link->head->pathElem->cost;
						cout<<"L"<<link->id<<"="<<link->tail->id<<"->"<<link->head->id<<"="<<res<<endl;
					}
				}
				if(err)
				{
					for(PTRTRACE pv = node->backStar.begin(); pv!= node->backStar.end(); pv++)
					{
						TNM_SLINK* link=(*pv);
						floatType res=link->tail->pathElem->cost+link->cost-link->head->pathElem->cost;
						cout<<"E"<<link->id<<"="<<link->tail->id<<"->"<<link->head->id<<"="<<res<<endl;
					}
				}
				
			}
		}

		orgn->TopologOrder(network);
		orgn->RemarkOBLinksOnNet();
		//cout<<endl;
		
	}
	cout<<"The num of adding bush link is "<<addbushlink<<endl;
	//getchar();
}


void MEUE_DOB::InitialSolution_tapasoue()
{
	network->UpdateLinkCost();
	network->InitialSubNet();
	network->UpdateLinkCost();
	ClearSolution();
	ReadTAPASOUE();
	network->UpdateLinkCost();
	network->UpdateLinkCostDer();
	CheckLinkFlow();
	cout<<"\tFinish read oue file!!!"<<endl;
	//getchar();
	ComplementUEBushII();

	cout<<"\tGot initial solution!!!"<<endl;

}

floatType MEUE_DOB::GetOlinkEntro(TNM_SLINK* link)
{
	if(link->oLinkPtr->oFlow>0)
		return log(link->oLinkPtr->oFlow/link->head->m_orgNode->Inflow)-link->buffer[0]+pntCoe*(link->volume-link->buffer[1]);
	else
		return -POS_LARGE_FLOAT;
}

floatType MEUE_DOB::GetOlinkfdEntro(TNM_SLINK* link)
{
	if(link->oLinkPtr->oFlow>0)
		return 1/link->oLinkPtr->oFlow+pntCoe-1/link->head->m_orgNode->Inflow;
	else
		return -POS_LARGE_FLOAT;
}

floatType MEUE_DOB::GetOlinkEntro(ORGLINK* olink,ORGNODE* onode)
{
	if(olink->oFlow>0)
		return log(olink->oFlow/onode->Inflow)-olink->linkPtr->buffer[0]+pntCoe*(olink->linkPtr->volume-olink->linkPtr->buffer[1]);
	else
		return -POS_LARGE_FLOAT;
}

floatType MEUE_DOB::GetOlinkfdEntro(ORGLINK* olink,ORGNODE* onode)
{
	if(olink->oFlow>0)
		return 1/olink->oFlow+pntCoe-1/onode->Inflow;
	else
		return -POS_LARGE_FLOAT;
}

void MEUE_DOB::UpdateEntropyII(TNM_SORIGIN* orgn)
{
	//update link and node entropy
	vector<ORGLINK*>::iterator pl; 
	//int nunu=0;
	for(pl=orgn->obLinkVector.begin();pl<orgn->obLinkVector.end();pl++)
	{//nunu++;
		TNM_SNODE* node=(*pl)->linkPtr->head;
		node->m_orgNode->Inflow=0.0;
		//if(orgn->origin->id==24 && (*pl)->linkPtr->id==71 && curIter==2)
		//	cout<<(*pl)->oFlow<<" "<<nunu<<endl;
		vector<TNM_SLINK*>::iterator bkl;
		for(bkl=node->backStar.begin();bkl<node->backStar.end();bkl++)
		{
			if((*bkl)->oLinkPtr!=NULL)
				node->m_orgNode->Inflow += (*bkl)->oLinkPtr->oFlow;
		}


		(*pl)->entro=GetOlinkEntro((*pl)->linkPtr);
		(*pl)->fdentro=GetOlinkfdEntro((*pl)->linkPtr);

		if((*pl)->oFlow<0 || _isnan((*pl)->oFlow))
			cout<<"Error!! The flow of Link "<<(*pl)->linkPtr->id<<" of bush "<<orgn->origin->id<<" is "<<(*pl)->oFlow<<endl;
		if((*pl)->linkPtr->volume<0 || _isnan((*pl)->linkPtr->volume))
			cout<<"Error!! The flow of Link "<<(*pl)->linkPtr->id<<" is "<<(*pl)->linkPtr->volume<<endl;
		if(node->m_orgNode->Inflow<0 || _isnan(node->m_orgNode->Inflow))
			cout<<"Error!! The flow into node "<<node->id<<" is "<<node->m_orgNode->Inflow<<endl;

		
	}
}

floatType MEUE_DOB::BuildPathTreeII(TNM_SORIGIN* orgn)
{
	//search sp and lp
	floatType tgap=0;
	vector<ORGNODE*>::iterator rp;
	for(rp=orgn->tplNodeVector.begin();rp<orgn->tplNodeVector.end();rp++)
	{
		(*rp)->nodePtr->pathElem->cost=POS_INF_FLOAT;
		(*rp)->nodePtr->pathElem->via=NULL;
		(*rp)->nodePtr->rPathElem->cost=-POS_INF_FLOAT;
		(*rp)->nodePtr->rPathElem->via=NULL;
	}
	orgn->origin->pathElem->cost=0.0;
	orgn->origin->rPathElem->cost=0.0;
	//bool in=false;
	for(rp=orgn->tplNodeVector.begin();rp<orgn->tplNodeVector.end();rp++)
	{
		TNM_SNODE* node=(*rp)->nodePtr;
		//if(orgn->origin->id==9 && curIter==2)
		//	cout<<"tplnodeOrder "<<node->id<<endl;
		node->buffer[0]=0;
		vector<TNM_SLINK*>::iterator sl;
		for(sl=node->forwStar.begin();sl<node->forwStar.end();sl++)
		{
			if((*sl)->oLinkPtr!=NULL)
			{
				(*sl)->markStatus=0;
				if((node->pathElem->cost+(*sl)->oLinkPtr->entro)<=(*sl)->head->pathElem->cost)
				{
					(*sl)->head->pathElem->cost=(node->pathElem->cost+(*sl)->oLinkPtr->entro);
					(*sl)->head->pathElem->via=*sl;
				}
				if((node->rPathElem->cost+(*sl)->oLinkPtr->entro)>=(*sl)->head->rPathElem->cost)
				{
					
					(*sl)->head->rPathElem->cost=(node->rPathElem->cost+(*sl)->oLinkPtr->entro);
					(*sl)->head->rPathElem->via=*sl;
				}
			}
		}
		if(node->rPathElem->cost>-1e8 || node->pathElem->cost>-1e8)
		{
			if((node->rPathElem->cost-node->pathElem->cost)!=0)
			{
				if(node->rPathElem->cost!=0)
				{
					if(abs((node->rPathElem->cost-node->pathElem->cost)/node->rPathElem->cost)>tgap)
						tgap=abs((node->rPathElem->cost-node->pathElem->cost)/node->rPathElem->cost);
					
				}
				else
				{
					if(abs((node->rPathElem->cost-node->pathElem->cost)/node->pathElem->cost)>tgap)
						tgap=abs((node->rPathElem->cost-node->pathElem->cost)/node->pathElem->cost);
					
				}
			}
		}
		
	}

	return tgap;
}

floatType MEUE_DOB::BuildPathTree_PAS(TNM_SORIGIN* orgn)
{
	NSPLinkVector.clear();
	//search sp and lp
	floatType tgap=0;
	vector<ORGNODE*>::iterator rp;
	for(rp=orgn->tplNodeVector.begin();rp<orgn->tplNodeVector.end();rp++)
	{
		(*rp)->nodePtr->pathElem->cost=POS_INF_FLOAT;
		(*rp)->nodePtr->pathElem->via=NULL;
		(*rp)->nodePtr->rPathElem->cost=-POS_INF_FLOAT;
		(*rp)->nodePtr->rPathElem->via=NULL;
	}
	orgn->origin->pathElem->cost=0.0;
	orgn->origin->rPathElem->cost=0.0;
	//bool in=false;
	for(rp=orgn->tplNodeVector.begin();rp<orgn->tplNodeVector.end();rp++)
	{
		TNM_SNODE* node=(*rp)->nodePtr;
		node->buffer[0]=0;
		vector<TNM_SLINK*>::iterator sl;
		for(sl=node->forwStar.begin();sl<node->forwStar.end();sl++)
		{
			if((*sl)->oLinkPtr!=NULL)
			{
				(*sl)->markStatus=0;
				if((node->pathElem->cost+(*sl)->oLinkPtr->entro)<=(*sl)->head->pathElem->cost)
				//if((node->pathElem->cost+(*sl)->oLinkPtr->entro)<(*sl)->head->pathElem->cost)
				{
					if((*sl)->head->pathElem->via!=NULL)
						NSPLinkVector.push_back((*sl)->head->pathElem->via->oLinkPtr);
					(*sl)->head->pathElem->cost=(node->pathElem->cost+(*sl)->oLinkPtr->entro);
					(*sl)->head->pathElem->via=*sl;
				}
				else
				{
					NSPLinkVector.push_back((*sl)->oLinkPtr);
				}
				if(node == orgn->origin || node->rPathElem->via!=NULL)
				{
				if((node->rPathElem->cost+(*sl)->oLinkPtr->entro)>=(*sl)->head->rPathElem->cost && (*sl)->oLinkPtr->oFlow>0.0)
				{
					
					(*sl)->head->rPathElem->cost=(node->rPathElem->cost+(*sl)->oLinkPtr->entro);
					(*sl)->head->rPathElem->via=*sl;
					
				}
				}
			}
		}
		
		if(node->rPathElem->cost>-1e8 || node->pathElem->cost>-1e8)
		{
			if((node->rPathElem->cost-node->pathElem->cost)>tgap)
				tgap=(node->rPathElem->cost-node->pathElem->cost);
			
		}
		
	}

	return tgap;
}

floatType MEUE_DOB::PasFlowShift(PairSegment_ly* pas)
{
	////update link cost
	floatType seg1cost=0.0,seg1fdcost=0.0;
	floatType seg1minflow=POS_INF_FLOAT,seg2minflow=POS_INF_FLOAT;
	bool seg1Noflow=false;
	for(int a=0;a<pas->seg1.size();a++)
	{
		ORGLINK* olink=pas->seg1[a];
		ORGNODE* onode=pas->oNodeList1[a];
		//if((olink->oFlow-onode->Inflow)>0 && (olink->oFlow-onode->Inflow)<1e-15)
		//		onode->Inflow=olink->oFlow;
		if(olink->oFlow<1e-100)
		{
			olink->oFlow=0;
			onode->Inflow -= olink->oFlow;
		}
		if(onode->Inflow<1e-100)
		{
			onode->Inflow=0;
			for(int h=0;h<onode->nodePtr->backStar.size();h++)
			{
				if(onode->nodePtr->backStar[h]->oLinkPtr!=NULL)
					onode->nodePtr->backStar[h]->oLinkPtr->oFlow=0;
			}
		}

		if(olink->oFlow==0 || onode->Inflow==0)
			seg1Noflow=true;
		olink->entro=GetOlinkEntro(olink,onode);
		olink->fdentro=GetOlinkfdEntro(olink,onode);
		seg1cost+=olink->entro;
		seg1fdcost+=olink->fdentro;
		if(seg1minflow>olink->oFlow)
			seg1minflow=olink->oFlow;

	}
	
	floatType seg2cost=0.0,seg2fdcost=0.0;
	bool seg2Noflow=false;
	for(int a=0;a<pas->seg2.size();a++)
	{
		ORGLINK* olink=pas->seg2[a];
		ORGNODE* onode=pas->oNodeList2[a];

		if(olink->oFlow<1e-100)
		{
			olink->oFlow=0;
			onode->Inflow -= olink->oFlow;
		}
		if(onode->Inflow<1e-100)
		{
			onode->Inflow=0;
			for(int h=0;h<onode->nodePtr->backStar.size();h++)
			{
				if(onode->nodePtr->backStar[h]->oLinkPtr!=NULL)
					onode->nodePtr->backStar[h]->oLinkPtr->oFlow=0;
			}
		}

		if(olink->oFlow==0 || onode->Inflow==0)
			seg2Noflow=true;
		olink->entro=GetOlinkEntro(olink,onode);
		olink->fdentro=GetOlinkfdEntro(olink,onode);
		seg2cost+=olink->entro;
		seg2fdcost+=olink->fdentro;
		if(seg2minflow>olink->oFlow)
			seg2minflow=olink->oFlow;

	}
	
	floatType gg=0.0;
	double pasgap=1e-3;
	if(RG/10000.0<1e-3 && curIter > 1)
	{pasgap = RG/10000.0;}

	

	gg=abs(seg2cost-seg1cost);
	pas->costDif=abs(seg2cost-seg1cost);
	
	floatType shiftflow=0.0;
	if(pas->costDif>pasgap)
	{
		numpas++;
		totalpas++;
		if(seg2Noflow && !seg1Noflow)
			shiftflow=-min(seg1minflow*1e-2,long double(1e-6));
		else if(seg1Noflow && !seg2Noflow)
			shiftflow=min(seg2minflow*1e-2,long double(1e-6));
		else if(!seg1Noflow && !seg2Noflow)
			shiftflow=(seg2cost-seg1cost)/(seg2fdcost+seg1fdcost+2/pas->oNodeList1[0]->Inflow);
		else
		{
			cout<<"Both segment of PAS"<<pas->Pid<<" are empty"<<endl;
			//return 0.0;
			getchar();
		}
		
		if(shiftflow>0)
		{
			if(shiftflow>=seg2minflow)
				shiftflow=seg2minflow*(1-1e-3);
		}
		else
		{
			if(-shiftflow>=seg1minflow)
				shiftflow=-seg1minflow*(1-1e-3);
		}

		if(shiftflow>1e-6)
			validpas++;
		
		pas->shiftFlow=shiftflow;
		pas->time1=seg1cost;
		pas->time2=seg2cost;
		pas->der1=seg1fdcost;
		pas->der2=seg2fdcost;

		//shift flow
		seg1cost=0.0;
		seg2cost=0.0;
		for(int a=0;a<pas->seg1.size();a++)
		{
			ORGLINK* olink=pas->seg1[a];
			ORGNODE* onode=pas->oNodeList1[a];
			olink->linkPtr->volume += shiftflow;
			olink->oFlow += shiftflow;
			
			if(olink->linkPtr->volume<0 && olink->linkPtr->volume>-1e-12)
			{olink->linkPtr->volume=0;olink->oFlow=0;}
			if(olink->oFlow<0 && olink->oFlow>-1e-12)
				olink->oFlow=0;

			if(a>0)
			{
				onode->Inflow += shiftflow;
				if(onode->Inflow<0 && onode->Inflow>-1e-12)
				{onode->Inflow=0;olink->oFlow=0;}
				
			}
			if((olink->oFlow-onode->Inflow)>0 && (olink->oFlow-onode->Inflow)<1e-15)
				onode->Inflow=olink->oFlow;

			olink->entro=GetOlinkEntro(olink,onode);
			olink->fdentro=GetOlinkfdEntro(olink,onode);

			bool er1=false;
			bool er2=false;
			bool er3=false;
			if(olink->oFlow<0 || _isnan(olink->oFlow))
			{cout<<"Error in shiftflow!! The flow of Link "<<olink->linkPtr->id<<" of bush "<<pas->OriginLabel<<" is "<<olink->oFlow<<" "<<curIter<<" "<<iniiter<<" "<<pas->Pid<<endl;er1=true;}
			if(olink->linkPtr->volume<0 || _isnan(olink->linkPtr->volume))
			{cout<<"Error in shiftflow!! The flow of Link "<<olink->linkPtr->id<<" is "<<olink->linkPtr->volume<<" shift flow is "<<shiftflow<<" seg1minflow="<<seg1minflow<<" seg2minflow="<<seg2minflow
			<<" "<<(seg2cost-seg1cost)<<" "<<seg2fdcost<<" "<<seg1fdcost<<" "<<2/pas->oNodeList1[0]->Inflow<<endl;er2=true;}
			if(onode->Inflow<0 || _isnan(onode->Inflow))
			{cout<<"Error in shiftflow!! The flow into node "<<onode->nodePtr->id<<" is "<<onode->Inflow<<endl;er3=true;}
			if(er1 || er2 || er3)
				getchar();
			

			seg1cost+=olink->entro;
		}
		for(int a=0;a<pas->seg2.size();a++)
		{
			ORGLINK* olink=pas->seg2[a];
			ORGNODE* onode=pas->oNodeList2[a];
			//if(curIter==1 && SubIter==1 && iniiter==1 && pas->Pid==2)
			//	cout<<"s2 "<<link->oLinkPtr->oFlow<<" "<<link->oLinkPtr->entro<<" "<<-link->buffer[0]+pntCoe*(link->volume-link->buffer[1])<<" "<<log(link->oLinkPtr->oFlow/node->m_orgNode->Inflow)<<" "<<log(link->oLinkPtr->oFlow/node->m_orgNode->Inflow)-link->buffer[0]+pntCoe*(link->volume-link->buffer[1]);
			olink->linkPtr->volume -= shiftflow;
			olink->oFlow -= shiftflow;
			
			if(olink->linkPtr->volume<0 && olink->linkPtr->volume>-1e-12)
			{olink->linkPtr->volume=0;olink->oFlow=0;}
			if(olink->oFlow<0 && olink->oFlow>-1e-12)
				olink->oFlow=0;

			
			if(a>0)
			{
				onode->Inflow -= shiftflow;
				if(onode->Inflow<0 && onode->Inflow>-1e-12)
				{onode->Inflow=0;olink->oFlow=0;}
				
			}
			
			olink->entro=GetOlinkEntro(olink,onode);
			olink->fdentro=GetOlinkfdEntro(olink,onode);

			bool er1=false;
			bool er2=false;
			bool er3=false;
			if(olink->oFlow<0 || _isnan(olink->oFlow))
			{cout<<"Error in shiftflow!! The flow of Link "<<olink->linkPtr->id<<" of bush "<<pas->OriginLabel<<" is "<<olink->oFlow<<endl;er1=true;}
			if(olink->linkPtr->volume<0 || _isnan(olink->linkPtr->volume))
			{cout<<"Error in shiftflow!! The flow of Link "<<olink->linkPtr->id<<" is "<<olink->linkPtr->volume<<endl;er2=true;}
			if(onode->Inflow<0 || _isnan(onode->Inflow))
			{cout<<"Error in shiftflow!! The flow into node "<<onode->nodePtr->id<<" is "<<onode->Inflow<<endl;er3=true;}
			if(er1 || er2 || er3)
				getchar();
			
			seg2cost+=olink->entro;
		}
		

		pas->time1=seg1cost;
		pas->time2=seg2cost;
		pas->minTotalFlow1=seg1minflow;
		pas->minTotalFlow2=seg2minflow;
		//pas->costDif=gg;
		pas->costDif=abs(seg2cost-seg1cost);
		
		
	}
	
	return shiftflow;
}


void MEUE_DOB::PostProcess()
{
	
	if(reportpathflow)
	{
	string ofname=outFileName+".pth";
	ofstream of;
	TNM_OpenOutFile(of,ofname);
	int pathnum=0;
	
	int pthid =0;
	of<<"path id      "<<"origin       "<<" dest      "<<"     path flow    "<<"   num of link    "<<"   links  "<<endl;
	for(int i=0;i<network->numOfOrigin;i++)
	{
		TNM_SORIGIN* orgn=network->originVector[i];
		orgn->MarkOBLinksOnNet();
		BuildAllPath(orgn);
		for(int j=0;j<orgn->numOfDest;j++)
		{
			TNM_SDEST* dest=orgn->destVector[j];
			string rel;
			for(int k=0;k<dest->pathSet.size();k++)
			{
				TNM_SPATH* path=dest->pathSet[k];

				floatType flow=dest->assDemand;
				for(int a=0;a<path->path.size();a++)
				{
					TNM_SLINK* link=path->path[a];
					if(link->oLinkPtr->oFlow>0)
					{
						flow *= link->oLinkPtr->oFlow/link->head->m_orgNode->Inflow;
					}
					else
					{
						flow=0.0;
					}
				}
				path->flow=flow;

				pthid ++;
				path->id=pthid;


				char pp[50];
				sprintf(pp,"%d\t%d\t%d\t%.10f\t%zu\t", path->id, orgn->origin->id, dest->dest->id, path->flow, path->path.size());
				rel += pp;
				for (int li=0; li<path->path.size();li++)
				{
					TNM_SLINK* link = path->path[li];
					rel += to_string(link->id) + " ";
					//f<<TNM_IntFormat(link->id,4);
				}
				rel += "\n";

			}
			of<<rel;
			//dest->EmptyPathSet();
		}
		orgn->RemarkOBLinksOnNet();
	}
	/////////////

	}
	if(network->numOfOrigin<101)
    // Relation-matrix diagnostics intentionally omitted from the focused port.
	AnalysisLinkFlowError();
	//ComparePathFlow("D:\\Data\\network\\cs\\chicagosketch_MEUE_PBD.pth");
}

void MEUE_DOB::SaveMEUESolution()
{
	string oueFileName = outFileName+".omeue";
	ofstream out;
	cout<<"\tWriting MEUE solution information into "<<outFileName<<" for post process"<<endl;
	if (!TNM_OpenOutFile(out,oueFileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .omeue file to write MEUE solution information!"<<endl;
		return;
	}
	//
	cout<<"\tWriting the network link flow info ..."<<endl;
	out<<TNM_IntFormat(network->numOfLink,6)<<endl;
	for (int li=0; li<network->numOfLink; li++)
	{
		TNM_SLINK* sl = network->linkVector[li];
		out<<TNM_IntFormat(sl->tail->id,6)<<TNM_IntFormat(sl->head->id,6)<<setw(floatWidth)<<TNM_FloatFormat(sl->volume,20,11)<<setw(floatWidth);
	}
	out<<endl;
	//
	cout<<"\t Writing the origin-based link flow info..."<<endl;
	out<<TNM_IntFormat(network->numOfOrigin,6)<<endl;
	for (int oi=0; oi< network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		out<<TNM_IntFormat(pOrg->id(),6)<<TNM_IntFormat(pOrg->obLinkVector.size(),6)<<endl;
		for (int oj=0; oj<pOrg->obLinkVector.size(); oj++)
		{
			ORGLINK* olink = pOrg->obLinkVector[oj];
			//if (olink->oFlow > 0)
			{
				out<<TNM_IntFormat(olink->linkPtr->id,6)<<setw(floatWidth)<<setw(floatWidth)<<TNM_FloatFormat(olink->oFlow,20,11)<<setw(floatWidth)<<setw(floatWidth);
			}
			
		}
		out<<endl; 
	}

	out.close();
}

void MEUE_DOB::SaveOriginLinkFlow(const string& fileName)
{
	ofstream out;
	cout<<"\tWriting MEUE origin-based link flows into "<<fileName<<endl;
	if (!TNM_OpenOutFile(out,fileName))
	{
		cout<<"\n\tCannot open MEUE_olinkflow.txt to write origin-based link flows!"<<endl;
		return;
	}

	out<<"origin\ttail_node\thead_node\tflow"<<endl;
	out<<scientific<<setprecision(12);
	for (int oi=0; oi<network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		for (int li=0; li<pOrg->obLinkVector.size(); li++)
		{
			ORGLINK* olink = pOrg->obLinkVector[li];
			if (olink->oFlow > 1e-10)
			{
				TNM_SLINK* link = olink->linkPtr;
				out<<pOrg->id()<<'\t'<<link->tail->id<<'\t'<<link->head->id<<'\t'<<olink->oFlow<<endl;
			}
		}
	}
	out.close();
}

void MEUE_DOB::AnalysisLinkFlowError()
{
	cout<<"Analysis the error of link flow...."<<endl;
	//vector<floatType> linkerror;
	//linkerror.reserve(network->numOfLink);
	int diffnum=0,numleq9=0,num89=0,num78=0,num67=0,num56=0,num45=0,num34=0,num23=0,num12=0,num01=0,numgeq0=0;
	floatType maxerr=0.0;
	for(int a=0;a<network->numOfLink;++a)
	{
		TNM_SLINK* link=network->linkVector[a];
		floatType err=abs(link->volume-link->buffer[1]);
		if(err > 0.0)
		{
			if(link->buffer[1]==0.0)
			{
				if(err>maxerr)
					maxerr=err;
				err=INF_LINK_COST;
			}
			else
				err=err/link->buffer[1];
			//linkerror.push_back(err);
			if(err>0.0)
			{
				diffnum++;
				if(err<=1e-9) numleq9++;
				else if(err<=1e-8) num89++;
				else if(err<=1e-7) num78++;
				else if(err<=1e-6) num67++;
				else if(err<=1e-5) num56++;
				else if(err<=1e-4) num45++;
				else if(err<=1e-3) num34++;
				else if(err<=1e-2) num23++;
				else if(err<=1e-1) num12++;
				else if(err<=1) num01++;
				else numgeq0++;
			}
		}
		
	}
	cout<<"diffnum,numleq9,num89,num78,num67,num56,num45,num34,num23,num12,num01,numgeq0\n"
		<<diffnum<<","<<numleq9<<","<<num89<<","<<num78<<","<<num67<<","<<num56<<","<<num45<<","<<num34<<","<<num23<<","<<num12<<","<<num01<<","<<numgeq0<<endl;
	cout<<"The max error of the zero UE flow link is "<<maxerr<<endl;
}

void MEUE_DOB::OpenRecordField()
{
	string ofname=outFileName+".rr";
	TNM_OpenOutFile(outf,ofname);
	ly_IterResult.clear();
	if(reportbushflow)
	{
		ofname=outFileName+".bush";
		TNM_OpenOutFile(outBushFlowf,ofname);
		ly_BushFlowResult.clear();
	}
}

void MEUE_DOB::RecordIterResult()
{
	currenttime=clock();
	floatType eltime = (floatType)(currenttime - begintime)/CLOCKS_PER_SEC;
	char res[200];

	if(curIter==0)
		sprintf(res,"%d\t%f\t%.14f\t%.10f\t%.5f\t%d\t%.3f\n",curIter,OFV,0.0,0.0,0.0,0,0.0);
	else
		sprintf(res,"%d\t%f\t%.14f\t%.10f\t%.5f\t%d\t%.3f\n",curIter,OFV,RG,FI,pntCoe,SubIter,eltime);
	string strr=res;
	ly_IterResult.push_back(strr);

	if(reportbushflow)
	{
		strr="";
		for(int i=0;i<network->numOfOrigin;i++)
		{
			TNM_SORIGIN* orgn=network->originVector[i];
			strr += "O="+to_string(orgn->origin->id)+"\n";
			orgn->MarkOBLinksOnNet();

			for(int a=0;a<network->numOfLink;a++)
			{
				TNM_SLINK* link=network->linkVector[a];
				if(link->oLinkPtr!=NULL)
				{
					strr += to_string(link->id)+":"+to_string(link->oLinkPtr->oFlow)+"\t";
				}
			}
			strr += "\n";
			orgn->RemarkOBLinksOnNet();
		}
	}
	ly_BushFlowResult.push_back(strr);
}

void MEUE_DOB::CloseRecordFile()
{
	outf<<"Iter\tOFV\tRG\tFlow_Infeasibility\tPenalty Coefficient\tSubIter\tTime\n";
	for(int i=0;i<ly_IterResult.size();i++)
		outf<<ly_IterResult[i];
	outf.close();

	if(reportbushflow)
	{
		outBushFlowf<<"Bush flow of every iteration\n";
		for(int i=0;i<ly_BushFlowResult.size();i++)
			outBushFlowf<<"-----------------Iter="<<to_string(i+1)<<"-----------------\n"<<ly_BushFlowResult[i];
		outBushFlowf.close();
	}

}

void MEUE_DOB::BuildAllPath(TNM_SORIGIN* orgn)
{
	//orgn->MarkOBLinksOnNet();
	TNM_SNODE* node=orgn->origin;
	//cout<<"N-"<<node->id<<" ";
	GotoNextNode(node,orgn);

	//orgn->RemarkOBLinksOnNet();
}

void MEUE_DOB::GotoNextNode(TNM_SNODE* node, TNM_SORIGIN* orgn)
{
	for(int i=0;i<node->forwStar.size();i++)
	{
		TNM_SLINK* link=node->forwStar[i];
		//cout<<"L-"<<link->id<<" ";
		if(link->oLinkPtr!=NULL)
		{
			TNM_SNODE* hnode=link->head;
			//cout<<"N-"<<hnode->id<<" ";
			hnode->pathElem->via=link;
			TNM_SDEST* dest;
			bool isdest=false;
			for(int j=0;j<orgn->numOfDest;j++)
			{
				if(orgn->destVector[j]->dest->id==hnode->id && orgn->destVector[j]->dest->id!=orgn->origin->id)
				{
					dest=orgn->destVector[j];
					isdest=true;
					break;
				}
			}
			if(isdest)
			{
				//new path;
				TNM_SPATH* path=new TNM_SPATH;
				path->path.clear();
				TNM_SNODE* snode=hnode;
				while(snode->id!=orgn->origin->id)
				{
					TNM_SLINK* slink=snode->pathElem->via;
					path->path.push_back(slink);
					//cout<<"L-"<<link->id<<" ";
					snode=slink->tail;
				}
				dest->pathSet.push_back(path);

			}
			//else
			{
				GotoNextNode(hnode,orgn);
			}

		}
	}
	// goto previous aspect;
}


void MEUE_TAPAS::Initialize()
{
	//InitialSolution_oue();
	InitialSolution_tapasoue();

	int numofolink=0;
	for(int i=0;i<network->numOfOrigin;i++)
	{
		TNM_SORIGIN* orgn=network->originVector[i];
		numofolink += orgn->obLinkVector.size();
	}
	cout<<"the number of olink is "<<numofolink<<endl;
	
	

	begintime=clock();
	network->AllocateLinkBuffer(2);
	network->AllocateNodeBuffer(2);
	//initialize link buffer
	for(int a=0;a<network->numOfLink;a++)
	{
		network->linkVector[a]->buffer[0]=0;//multipler
		network->linkVector[a]->buffer[1]=network->linkVector[a]->volume;//optimal flow
		if(network->linkVector[a]->volume<0 && network->linkVector[a]->volume>-1e-12)
			network->linkVector[a]->volume=0;
	}
	//ini onode inflow
	for(int i=0;i<network->numOfOrigin;i++)
	{
		TNM_SORIGIN* orgn=network->originVector[i];
		orgn->MarkOBLinksOnNet();
		vector<ORGNODE*>::iterator rp;
		for(rp=orgn->tplNodeVector.begin();rp<orgn->tplNodeVector.end();rp++)
		{
			TNM_SNODE* node=(*rp)->nodePtr;
			(*rp)->Inflow=0.0;
			vector<TNM_SLINK*>::iterator bkl;
			for(bkl=node->backStar.begin();bkl<node->backStar.end();bkl++)
			{
				if((*bkl)->oLinkPtr!=NULL)
					(*rp)->Inflow += (*bkl)->oLinkPtr->oFlow;
			}
			//cout<<"Inflow="<<(*rp)->Inflow<<endl;
		}
		orgn->RemarkOBLinksOnNet();
	}
	//RecordUEpath();
	//ini PNList
	PNList_ly.reserve(network->numOfNode);
	for(int i=0;i<network->numOfNode;i++)
	{
		TNM_SNODE* node=network->nodeVector[i];
		PasNode_ly *pn=new PasNode_ly;
		node->buffer[1]=i;
		pn->NodePtr=node;
		pn->NPasList.clear();
		PNList_ly.push_back(pn);
	}
	PasList_ly.clear();

	CalculateOFV();
	char res[200];
	sprintf(res,"%.4f",OFV);
	cout<<"OFV="<<res<<endl;
	//initialize coefficients
	SetInipntCoe();
	SetiniMultipler();
	RecordIterResult();
	rt =1;

	char chr[20];
	floatType aveVC=0.0;
	for(int a=0;a<network->numOfLink;a++)
	{
		TNM_SLINK* link=network->linkVector[a];
		aveVC += link->volume / link->capacity;
	}
	aveVC /= network->numOfLink;
	sprintf(chr,"%.4f",aveVC);
	cout<<"\tAverage V/C="<<chr<<endl;

}

void MEUE_TAPAS::MainLoop()
{

	floatType temp_maxsubiter=MaxSubIter;
	if(curIter==1)
		MaxSubIter=3;

	SubIter=0;
	do
	{
		SubLoop_ALM();
	}while(Gap>MaxSubConv && SubIter<MaxSubIter);

	MaxSubIter=temp_maxsubiter;
	//compute entropy
	CalculateOFV();
	floatType oldfi=FI;
	UpdateFlowInfeasibility();

	floatType oldrg=RG;
	RG=GetRelativeGAP();
	currenttime=clock();
	floatType eltime = (floatType)(currenttime - begintime)/CLOCKS_PER_SEC;

	floatType egap=0.0;
	for(int a=0;a<network->numOfLink;a++)
	{
		floatType diff=network->linkVector[a]->volume-network->linkVector[a]->buffer[1];
		egap += network->linkVector[a]->buffer[0]*diff - 0.5*pntCoe*diff*diff;
	}
	
	char chr[20];
	sprintf(chr,"%.4f",OFV);
	cout<<curIter<<". OFV="<<chr<<",FI="<<FI<<",MaxFI="<<MaxFI<<",ent gap="<<egap<<",RG="<<RG<<",pntCoe="<<pntCoe<<",time="<<eltime<<endl;

	if(!pre0)
	{
		if(RG<1.0 && FI<1.0)
		{
			int nuelink=0;
			for(int a=0;a<network->numOfLink;a++)
			{
				if(abs(network->linkVector[a]->buffer[1]-network->linkVector[a]->volume)>1e-5)
					nuelink++;
			}
			cout<<"  non_UE link "<<nuelink<<endl;
			cout<<"precision 0 time: "<<eltime<<endl; pre0=true;
		}
	}
	if(!pre2)
	{
		if(RG<1e-2 && FI<1e-2)
		{
			int nuelink=0;
			for(int a=0;a<network->numOfLink;a++)
			{
				if(abs(network->linkVector[a]->buffer[1]-network->linkVector[a]->volume)>1e-5)
					nuelink++;
			}
			cout<<"  non_UE link "<<nuelink<<endl;
			cout<<"precision 2 time: "<<eltime<<endl; pre2=true;
		}
	}
	if(!pre4)
	{
		if(RG<1e-4 && FI<1e-4)
		{
			int nuelink=0;
			for(int a=0;a<network->numOfLink;a++)
			{
				if(abs(network->linkVector[a]->buffer[1]-network->linkVector[a]->volume)>1e-5)
					nuelink++;
			}
			cout<<"  non_UE link "<<nuelink<<endl;
			cout<<"precision 4 time: "<<eltime<<endl; pre4=true;
		}
	}
	if(!pre6)
	{
		if(RG<1e-6 && FI<1e-6)
		{
			int nuelink=0;
			for(int a=0;a<network->numOfLink;a++)
			{
				if(abs(network->linkVector[a]->buffer[1]-network->linkVector[a]->volume)>1e-5)
					nuelink++;
			}
			cout<<"  non_UE link "<<nuelink<<endl;
			cout<<"precision 6 time: "<<eltime<<endl; pre6=true;
		}
	}
	if(!pre7)
	{
		if(RG<1e-7 && FI<1e-7)
		{
			int nuelink=0;
			for(int a=0;a<network->numOfLink;a++)
			{
				if(abs(network->linkVector[a]->buffer[1]-network->linkVector[a]->volume)>1e-5)
					nuelink++;
			}
			cout<<"  non_UE link "<<nuelink<<endl;
			cout<<"precision 7 time: "<<eltime<<endl; pre7=true;
		}
	}
	///
	
	

	//system("PAUSE");
	//update multipler and coefficience
	UpdateMultipler();
	if(RG<MaxMainGapConv && FI<MaxMainFIConv)
		maxMainIter=1;

	if(FI/RG<0.01 || FI<MaxMainFIConv)
	{
		rt=2;
	}
	if(RG/FI<0.01 || RG<MaxMainGapConv)
	{
		rt=1;
	}

	if(curIter-oldcurIter>0)
	{
		if(  rt==1)
	    {
		    if(FI>0.7*oldfi)
		   {  
			  pntCoe = pntCoe*4.0;
			  oldcurIter=curIter;
		   }
	    }
	    else if(rt==2) 
	    {
		   if(RG>0.7*oldrg)
		   {
			  pntCoe = pntCoe/4.0;
			  oldcurIter=curIter;
		   }
	    }
	
	}

	

	if(curIter==1)
	{
		floatType pn=0.0;
		for(int a=0;a<network->numOfLink;a++)
		{
			TNM_SLINK* link=network->linkVector[a];
			pn+=pow(link->volume-link->buffer[1],2);
		}
		pntCoe=OFV/(1*pn);//sigma
	}

	
	//
	floatType pena=0.0;
	for(int a=0;a<network->numOfLink;a++)
	{
		TNM_SLINK* link=network->linkVector[a];
		floatType diff=link->volume - link->buffer[1];
		pena += - link->buffer[0]*(diff) + pntCoe*pow(diff,2)/2.0;
	}
	cout<<"penalty obj="<<pena<<endl;
	//

	badIter=0;
	RecordIterResult();
}

void MEUE_TAPAS::SubLoop_ALM()
{

	SubIter++;
	iniiter=0;
	Gap=0.0;
	floatType tgap=0.0,ggap=0.0;
	clock_t t1=0,t2=0,t3=0,t0;
	int n2, n6, n10, n14, n16;
	n2=0;
	n6=0;
	n10=0;
	n14=0;
	n16=0;
	//vector<floatType> orgngap;
	//orgngap.clear();
	t0=clock();
	for(int i=0;i<network->numOfOrigin;i++)
	{
		TNM_SORIGIN* orgn=network->originVector[i];
		//orgngap.push_back(1e10);
		//cout<<" node of origin "<<orgn->origin->id<<endl;
		orgn->MarkOBLinksOnNet();

		//Update link "cost"
		UpdateEntropyII(orgn);

		//Build shorest path tree
		BuildPathTree_PAS(orgn);

		//generate PAS
		floatType gg=GeneratePASII(orgn);
		if(gg>ggap)
			ggap=gg;
		
		

		orgn->RemarkOBLinksOnNet();
	}
	t1=clock()-t0;

	//inner loop
	floatType MaxIniConv=1e-6;
	if(MaxIniConv>RG/10000.0)
		MaxIniConv=RG/10000.0;
	t0=clock();
	tgap=1e10;
	int totalvp=0;
	int numPAS=0,numOPAS=0;
	//int n2, n6, n10, n14, n16;
	totalpas =0;
	//int hardid=0;
	while(iniiter<50 && tgap>MaxIniConv)
	{
		iniiter++;
		tgap=0;
		validpas=0;
		n2=0;
		n6=0;
		n10=0;
		n14=0;
		n16=0;
		numpas =0;
		numPAS=0;
		numOPAS=0;
		//int hardid2=0;
		//hardid=0;
		for(int i=0;i<network->numOfNode;i++)
		{
			TNM_SNODE* node=network->nodeVector[i];
			PasNode_ly* pn=PNList_ly[node->buffer[1]];
			floatType pgap=0.0;
			numPAS += pn->NPasList.size();
			for(int k=0;k<pn->NPasList.size();k++)
			{
				PairSegment_struct* pass=pn->NPasList[k];
				numOPAS += pass->PASoriginList.size();
				for(int p=0;p<pass->PASoriginList.size();p++)
				{
					PairSegment_ly* pas=pass->PASoriginList[p];
					if(pas!=NULL)
					{
						if(iniiter%5==1)
							pas->costDif=1e10;
						if(pas->costDif<MaxIniConv/100.0)
							continue;

						floatType oldgap=pas->costDif;
					
						PasFlowShift(pas);

						if(pas->costDif>1e-6 && pas->costDif>oldgap*0.5)
						{
							for(int j=0;j<5;j++)
								PasFlowShift(pas);
							
						}
						

						if(pgap<pas->costDif)
						{
							pgap=pas->costDif;
						}

						//

						if (pas->costDif>=1e-2)
						{n2++;}
						if(pas->costDif<1e-2 && pas->costDif>=1e-6)
						{n6++;}
						if(pas->costDif<1e-6 && pas->costDif>=1e-12)
						{n10++;}
						if(pas->costDif<1e-12 && pas->costDif>=1e-14)
						{n14++;}
						if(pas->costDif<1e-14)
						{n16++;}
					
					}
				}
			}
			if(tgap<pgap)
			{
				tgap=pgap;
				//hardid2=hardid;
			}

		}
		totalvp+=validpas;
		Gap=tgap;

		
		
	}

	t2=clock()-t0;
	//deletePAS
	t0=clock();
	numPAS=0;
	numOPAS=0;
	for(int i=0;i<network->numOfNode;i++)
	{
		TNM_SNODE* node=network->nodeVector[i];
		PasNode_ly* pn=PNList_ly[node->buffer[1]];
		for(int k=0;k<pn->NPasList.size();k++)
		{
			PairSegment_struct* pass=pn->NPasList[k];
			numPAS++;
			for(int p=0;p<pass->PASoriginList.size();p++)
			{
				PairSegment_ly* pas=pass->PASoriginList[p];
				numOPAS++;
				if(pas!=NULL)
				{
					delete pas;
				}
			}
			delete pass;
		}
		pn->NPasList.clear();
	}
	PasID=0;
	PasStructID=0;
	t3=clock()-t0;
	
	cout<<"\t"<<SubIter<<". GAP="<<ggap<<"->"<<Gap<<",shiftflowT="<<(floatType)t1/CLOCKS_PER_SEC<<",innerloopT="<<(floatType)t2/CLOCKS_PER_SEC<<",deleteT="<<(floatType)t3/CLOCKS_PER_SEC<<", PAS num="<<numPAS<<", OPAS num="<<numOPAS<<" shifted pas is "<<totalpas<<", valid pas is "<<totalvp<<endl;
	//system("PAUSE");
}

floatType MEUE_TAPAS::GeneratePASII(TNM_SORIGIN* orgn)
{
	floatType tgap=0;
	vector<ORGNODE*>::reverse_iterator rp;
	TNM_SNODE* nodeshort,* nodelarge;
	for(rp=orgn->tplNodeVector.rbegin();rp<orgn->tplNodeVector.rend();rp++)
	{
		if((*rp)->nodePtr->rPathElem->via==NULL) continue;
		if((*rp)->mutliEnd && (*rp)->nodePtr->pathElem->via!=(*rp)->nodePtr->rPathElem->via)
		{
			nodeshort=(*rp)->nodePtr;
			nodelarge=(*rp)->nodePtr;
			TNM_SLINK* linkshort;
			TNM_SLINK* linklarge;
			floatType fdshort=0.0,fdlarge=0.0;
			floatType sdshort=0.0,sdlarge=0.0;
			floatType minflow1=POS_INF_FLOAT,minflow2=POS_INF_FLOAT;
			//bool samepath=true,nonlargepath=false;
			bool unusedlargepath=false,unusedshortpath=false;
			int nnn=0;
			//segment 1
			while(nodeshort!=orgn->origin && nodeshort!=(*rp)->lcnPtr)
			{
				linkshort=nodeshort->pathElem->via;
				if(linkshort->oLinkPtr->oFlow==0)
					unusedshortpath=true;
				nnn++;
				if(nnn>network->numOfLink)
					cout<<"Infinite loop "<<linkshort->id<<" *"<<linkshort->tail->id<<" *"<<linkshort->head->id<<" ";
				
				nodeshort->buffer[0]=1;
				linkshort->oLinkPtr->entro=GetOlinkEntro(linkshort);
				linkshort->oLinkPtr->fdentro=GetOlinkfdEntro(linkshort);
				fdshort += linkshort->oLinkPtr->entro;
				sdshort += linkshort->oLinkPtr->fdentro;
				
				if(minflow1>linkshort->oLinkPtr->oFlow)
					minflow1=linkshort->oLinkPtr->oFlow;
				nodeshort=linkshort->tail;
			}
			//segment 2
			nnn=0;
			while(nodelarge!=orgn->origin && nodelarge!=(*rp)->lcnPtr)
			{
				linklarge=nodelarge->rPathElem->via;
				if(linklarge->oLinkPtr->oFlow==0)
					unusedlargepath=true;
				nnn++;
				if(nnn>network->numOfLink)
					cout<<"Infinite loop "<<linklarge->id<<" ";
				
				if(nodelarge->id!=(*rp)->nodePtr->id && nodelarge->buffer[0]==1)
					break;
				linklarge->oLinkPtr->entro=GetOlinkEntro(linklarge);
				linklarge->oLinkPtr->fdentro=GetOlinkfdEntro(linklarge);
				fdlarge += linklarge->oLinkPtr->entro;
				sdlarge += linklarge->oLinkPtr->fdentro;
				
				if(minflow2>linklarge->oLinkPtr->oFlow)
					minflow2=linklarge->oLinkPtr->oFlow;
				nodelarge=linklarge->tail;
			}


			if(unusedlargepath && unusedshortpath)
			{
				nodeshort=(*rp)->nodePtr;
				while(nodeshort!=orgn->origin && nodeshort!=(*rp)->lcnPtr)
				{
					linkshort=nodeshort->pathElem->via;
					//linkshort->markStatus=0;
					nodeshort->buffer[0]=0;
					nodeshort=linkshort->tail;
				}
			}
			else
			{
				TNM_SNODE* endnode=nodelarge;
				if(endnode!=(*rp)->lcnPtr)
				{
					TNM_SNODE* node1=nodelarge;
					TNM_SLINK* link1;
					floatType minflow3=POS_INF_FLOAT;
					while(nodeshort!=node1)
					{
						link1=node1->pathElem->via;
						nnn++;
						if(nnn>500)
							cout<<"Infinite loop "<<link1->id<<" *"<<link1->tail->id<<" *"<<link1->head->id<<" ";
						//link1->markStatus=0;
						node1->buffer[0]=0;
						fdshort -= link1->oLinkPtr->entro;
						sdshort -= link1->oLinkPtr->fdentro;
						if(minflow3>link1->oLinkPtr->oFlow)
							minflow3=link1->oLinkPtr->oFlow;
						node1=link1->tail;
					}
					if(minflow1==minflow3)
					{
						minflow1=POS_INF_FLOAT;
						node1=(*rp)->nodePtr;
						while(nodelarge!=node1)
						{
							link1=node1->pathElem->via;
							if(minflow1>link1->oLinkPtr->oFlow)
								minflow1=link1->oLinkPtr->oFlow;
							node1=link1->tail;
						}
					}
					else if(minflow1>minflow3)
						cout<<"ShiftBushFlow  ERROR3!!!\n";
				}

				if(tgap<abs(fdlarge-fdshort))
					tgap=abs(fdlarge-fdshort);

				PairSegment_ly* pas=new PairSegment_ly;
				pas->end=(*rp)->nodePtr;
				pas->begin=endnode;

				//shift flow
				floatType shiftflow;
				if(unusedshortpath && !unusedlargepath)
					shiftflow=min(minflow2*1e-2,long double(1e-6));
				else if(unusedlargepath && !unusedshortpath)
					shiftflow=-min(minflow1*1e-2,long double(1e-6));
				else
					shiftflow=(fdlarge-fdshort)/(sdlarge+sdshort+2/(*rp)->Inflow);
				if(shiftflow>0)
				{
					if(shiftflow>minflow2)
						shiftflow=minflow2*(1-1e-3);
				}
				else
				{
					if(-shiftflow>minflow1)
						shiftflow=-minflow1*(1-1e-3);
				}
				
				//cout<<"shiftflow:"<<shiftflow<<" "<<(fdlarge-fdshort)/(sdlarge+sdshort+2/(*rp)->Inflow)<<endl;
				nodeshort=(*rp)->nodePtr;
				nodelarge=(*rp)->nodePtr;
				fdshort=0.0;
				fdlarge=0.0;
				while(nodeshort!=endnode)
				{
					linkshort=nodeshort->pathElem->via;
					nodeshort->buffer[0]=0;
					linkshort->volume += shiftflow;
					//cout<<"L:"<<linkshort->id<<" flow="<<linkshort->oLinkPtr->oFlow<<endl;
					linkshort->oLinkPtr->oFlow += shiftflow;

					if(linkshort->volume<0 && linkshort->volume>-1e-12)
					{linkshort->volume=0;linkshort->oLinkPtr->oFlow=0;}
					if(linkshort->oLinkPtr->oFlow<0 && linkshort->oLinkPtr->oFlow>-1e-12)
						linkshort->oLinkPtr->oFlow=0;

					if(nodeshort!=(*rp)->nodePtr)
					{
						nodeshort->m_orgNode->Inflow +=shiftflow;
						if(nodeshort->m_orgNode->Inflow<0 && nodeshort->m_orgNode->Inflow>-1e-12)
						{nodeshort->m_orgNode->Inflow=0;linkshort->oLinkPtr->oFlow=0;}
					}
					linkshort->oLinkPtr->entro=GetOlinkEntro(linkshort);
					linkshort->oLinkPtr->fdentro=GetOlinkfdEntro(linkshort);
					
					bool er1=false;
					bool er2=false;
					bool er3=false;
					if(linkshort->oLinkPtr->oFlow<0 || _isnan(linkshort->oLinkPtr->oFlow))
					{cout<<"Error in shiftflow!! The flow of Link "<<linkshort->id<<" of bush "<<orgn->origin->id<<" is "<<linkshort->oLinkPtr->oFlow<<endl;er1=true;}
					if(linkshort->volume<0 || _isnan(linkshort->volume))
					{cout<<"Error in shiftflow!! The flow of Link "<<linkshort->id<<" is "<<linkshort->volume<<endl;er2=true;}
					if(nodeshort->m_orgNode->Inflow<0 || _isnan(nodeshort->m_orgNode->Inflow))
					{cout<<"Error in shiftflow!! The flow into node "<<nodeshort->id<<" is "<<nodeshort->m_orgNode->Inflow<<endl;er3=true;}
					if(er1 || er2 || er3)
						getchar();
					
					fdshort += linkshort->oLinkPtr->entro;
					pas->seg1.push_back(linkshort->oLinkPtr);
					pas->oNodeList1.push_back(nodeshort->m_orgNode);
					nodeshort=linkshort->tail;
				}
				
				while(nodelarge!=endnode)
				{
					linklarge=nodelarge->rPathElem->via;
					linklarge->volume -= shiftflow;
					//cout<<"L:"<<linklarge->id<<" flow="<<linklarge->oLinkPtr->oFlow<<endl;
					linklarge->oLinkPtr->oFlow -= shiftflow;

					if(linklarge->volume<0 && linklarge->volume>-1e-12)
					{linklarge->volume=0;linklarge->oLinkPtr->oFlow=0;}
					if(linklarge->oLinkPtr->oFlow<0 && linklarge->oLinkPtr->oFlow>-1e-12)
						linklarge->oLinkPtr->oFlow=0;

					if(nodelarge!=(*rp)->nodePtr)
					{
						nodelarge->m_orgNode->Inflow -= shiftflow;
						if(nodelarge->m_orgNode->Inflow<0 && nodelarge->m_orgNode->Inflow>-1e-12)
						{nodelarge->m_orgNode->Inflow=0;linklarge->oLinkPtr->oFlow=0;}
					}
					linklarge->oLinkPtr->entro=GetOlinkEntro(linklarge);
					linklarge->oLinkPtr->fdentro=GetOlinkfdEntro(linklarge);

					
					bool er1=false;
					bool er2=false;
					bool er3=false;
					if(linklarge->oLinkPtr->oFlow<0 || _isnan(linklarge->oLinkPtr->oFlow))
					{cout<<"Error in shiftflow!! The flow of Link "<<linklarge->id<<" of bush "<<orgn->origin->id<<" is "<<linklarge->oLinkPtr->oFlow<<endl;er1=true;}
					if(linklarge->volume<0 || _isnan(linklarge->volume))
					{cout<<"Error in shiftflow!! The flow of Link "<<linklarge->id<<" is "<<linklarge->volume<<endl;er2=true;}
					if(nodelarge->m_orgNode->Inflow<0 || _isnan(nodelarge->m_orgNode->Inflow))
					{cout<<"Error in shiftflow!! The flow into node "<<nodelarge->id<<" is "<<nodelarge->m_orgNode->Inflow<<endl;er3=true;}
					if(er1 || er2 || er3)
						getchar();
					
					fdlarge += linklarge->oLinkPtr->entro;
					pas->seg2.push_back(linklarge->oLinkPtr);
					pas->oNodeList2.push_back(nodelarge->m_orgNode);
					nodelarge=linklarge->tail;
				}
				
				pas->shiftFlow=shiftflow;
				pas->time1=fdshort;
				pas->time2=fdlarge;
				//cout<<"cost1:"<<fdshort<<"\tcost2:"<<fdlarge<<endl;
				pas->costDif=abs(fdlarge-fdshort);

				pas->OriginLabel=orgn->origin->id;
				//PasID++;
				//pas->Pid=PasID;

				if(ExistPas((*rp)->nodePtr,pas))
					delete pas;


			}
		}

	}

	return tgap;
}

bool MEUE_TAPAS::ExistPas(TNM_SNODE* node,PairSegment_ly* pas)
{
	bool exist=false;
	PasNode_ly* pn=PNList_ly[node->buffer[1]];
	bool findsamestruct=false;
	for(int k=0;k<pn->NPasList.size();k++)
	{
		PairSegment_struct* pass=pn->NPasList[k];
		if(pass->begin==pas->begin)
		{
			//check the pas struct
			bool same1=false,same2=false;
			if(pass->PASoriginList[0]->seg1.size()==pas->seg1.size() && pass->PASoriginList[0]->seg2.size()==pas->seg2.size())
			{
				same1=true;
				for(int a=0;a<pas->seg1.size();a++)
				{
					if(pas->seg1[a]->linkPtr->id!=pass->PASoriginList[0]->seg1[a]->linkPtr->id)
					{
						same1=false;
						break;
					}
				}
				if(same1)
				{
					for(int a=0;a<pas->seg2.size();a++)
					{
						if(pas->seg2[a]->linkPtr->id!=pass->PASoriginList[0]->seg2[a]->linkPtr->id)
						{
							same1=false;
							break;
						}
					}
				}
			}
			if(!same1)
			{
				if(pass->PASoriginList[0]->seg1.size()==pas->seg2.size() && pass->PASoriginList[0]->seg2.size()==pas->seg1.size())
				{
					same2=true;
					for(int a=0;a<pas->seg1.size();a++)
					{
						if(pas->seg1[a]->linkPtr->id!=pass->PASoriginList[0]->seg2[a]->linkPtr->id)
						{
							same2=false;
							break;
						}
					}
					if(same2)
					{
						for(int a=0;a<pas->seg2.size();a++)
						{
							if(pas->seg2[a]->linkPtr->id!=pass->PASoriginList[0]->seg1[a]->linkPtr->id)
							{
								same2=false;
								break;
							}
						}
					}
				}
			}

			if(same1 || same2)
			{
				findsamestruct=true;
				//check origin-based pas
				for(int i=0;i<pass->PASoriginList.size();i++)
				{
					if(pas->OriginLabel==pass->PASoriginList[i]->OriginLabel)
					{
						exist=true;
						break;
					}
				}
				if(!exist)
				{
					PasID++;
					pas->Pid=PasID;
					pass->PASoriginList.push_back(pas);
				}

				break;
			}

		}
	}

	if(!findsamestruct)
	{
		//creat new pas
		PairSegment_struct *pass=new PairSegment_struct;
		pass->begin=pas->begin;
		pass->end=pas->end;
		pass->PASoriginList.push_back(pas);
		pn->NPasList.push_back(pass);
		PasID++;
		pas->Pid=PasID;
		PasStructID++;
		pass->Pid=PasStructID;
		//PasList_ly.push_back(pass);
	}

	return exist;
}

void MEUE_TAPAS::PreProcess()
{
	MEUE_DOB::PreProcess();
	PasStructID=0;
	oldcurIter=0;
}
