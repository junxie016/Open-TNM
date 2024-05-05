#include "header\stdafx.h"
#include <math.h>
#include <stack>

int  TNM_TAP::intWidth       = 0;
int  TNM_TAP::floatWidth     = 0;

TNM_TAP::TNM_TAP()
{
	network            = NULL; 
	costScalar         = 1.0;
	OFV                = pow(2.0, 52.0); //set the intial value to a fairly big number.
	cpuTime            = 0.0;
	termFlag           = InitTerm;    //this is an intial value, which menas Solve has not been executed yet.
	numLineSearch      = 0;
	maxMainIter        = 100;   // maximum allowed iteration number
	convCriterion      = 0.001; // convergence criterion
	stepSize           = 1.0;     // current step size
	convIndicator      = 1.0;     // convergence indicator
	objectID           = GEN_IA_ID;
	//m_storeResult	   = false;
	m_resetNetworkOnSolve = true;
	yPath			   = new TNM_SPATH;
	reportIterHistory  =false;
	reportLinkDetail   =false;
	reportPathDetail   =false;
	timeCostCoefficient = 1.0;
	distCostCoefficient = 0.0;
}

TNM_TAP::~TNM_TAP()
{
	ClearIterRecord();
}

void TNM_TAP::SetConv(floatType tf)
{
	if(tf>MAXCONV || tf <MINCONV)
	{
		cout<<"\tAccuracy should range between "<<MINCONV<<" and "<<MAXCONV
			<<"\n\tThe default value "<<convCriterion<<" is retained."<<endl;
	}
	else
	{
		convCriterion = tf;
	}
}

void TNM_TAP::SetMaxLsIter(int ti)
{
	if(ti>MAXLSITER || ti <MINLSITER)
	{
		cout<<"\tMaximum allowed line search iterations should range between "<<MINLSITER<<" and "<<MAXLSITER
			<<"\n\tThe default value "<<maxLineSearchIter<<" is retained."<<endl;
	}
	else
	{
		maxLineSearchIter = ti;
	}
}

void TNM_TAP::SetMaxIter(int ti)
{
	if(ti>MAXMAXITER || ti <MINMAXITER)
	{
		cout<<"\tMaximum allowed iterations should range between "<<MINMAXITER<<" and "<<MAXMAXITER
			<<"\n\tThe default value "<<maxMainIter<<" is retained."<<endl;
	}
	else
	{
		maxMainIter = ti;
	}

}

int TNM_TAP::SetTollType(TNM_TOLLTYPE tl)
{
	m_tlType = tl;
	if(!network->CheckBuildStatus(false))
	{
		cout<<"\t cannot set toll type, please build network first!"<<endl;
		return 1;
	}
	else
	{
		if(tl != TT_MXTOLL) //all dummy links will not be set here.
		{
			network->SetLinkTollType(tl);
		}
	}
		return 0;
	
}

int TNM_TAP::Build(const string& inFile, const string& outFile, MATINFORMAT in)
{
	inFileName = inFile;
	outFileName = outFile;
	if(network!=NULL)
	{
		delete network;
		network = NULL;
	}
	else        
		network = new TNM_SNET(inFile);
	network->SetLinkCostScalar(costScalar);
	network->InitializeCostCoef(timeCostCoefficient,distCostCoefficient);
	switch (in)
	{
	case NETTAPAS:
		/*Build the network using Hillel Bar-Gera's file format*/
		if(network->BuildTAPAS(true, lpf)!=0) 
		{
			cout<<"\tEncounter problems when building a network object!"<<endl;
			return 4;
		}
		break;
	default:
		cout<<"\tUnrecognized network format. "<<endl;
		return 5;
	}
	network->ClearZeroDemandOD();
	return 0;
};

TERMFLAGS TNM_TAP::Solve()
{
	m_startRunTime = clock();//CPU time
	PreProcess();
	numLineSearch = 0;
	if(termFlag!=ErrorTerm)
	{
		curIter = 0;//Current iterations
		/*Get an initial solution*/
		Initialize();
		/*Solve the problem iteratively*/
		if(!Terminate())
		{
			RecordCurrentIter();
			do 
			{
				curIter ++;
				MainLoop();
				RecordCurrentIter();
			}while (!Terminate());
		}
		PostProcess();
		termFlag = TerminationType();
	}
	cout<<"\n\n\tSolution process terminated"<<endl;
	cpuTime = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	return termFlag;
}

void TNM_TAP::PreProcess()
{
	stepSize                  = 1.0;
	convIndicator             = 1e3;    
	OFV                       = pow(2.0, 52.0); //set the intial value to a fairly big number.
	numLineSearch             = 0;
	ClearIterRecord();
}

void TNM_TAP::ClearIterRecord()
{
	for (vector<ITERELEM*>::iterator pv = iterRecord.begin(); pv != iterRecord.end(); pv++)
	{
		delete *pv;
	}
	if(!iterRecord.empty()) iterRecord.clear();
}

ITERELEM * TNM_TAP::RecordCurrentIter()
{
	ITERELEM *iterElem;
	iterElem = new ITERELEM;
	iterElem->objID = objectID;
	iterElem->iter  = curIter;
	iterElem->ofv   = OFV;
	iterElem->step  = stepSize;
	iterElem->conv  = convIndicator;
	iterElem->time  = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
#ifdef IA_DEBUG
	cout<<*iterElem;
#endif
	iterRecord.push_back(iterElem); 
	return iterElem;
}

bool TNM_TAP::Terminate()
{
	return ReachAccuracy() || ReachMaxIter() || ReachError() || ReachUser();
}

TERMFLAGS TNM_TAP::TerminationType()
{
	if(ReachAccuracy()) return ConvergeTerm;
	if(ReachMaxIter())  return MaxIterTerm;
	if(ReachUser())     return UserTerm;
	return ErrorTerm;

}

void TNM_TAP::ColumnGeneration(TNM_SORIGIN* pOrg,TNM_SDEST* dest)
{

	yPath->path.clear();
	yPath->flow = 0;
	yPath->cost = 0;
	//
	TNM_SNODE* snode;
	TNM_SLINK* slink;
	snode = dest->dest;

	while(snode != pOrg->origin)
	{
		slink = snode->pathElem->via;
		if(slink == NULL) 
		{
			cout<<"The link is null in calculating the direction for OFW algorithm"<<endl;
			break;
		}
		
		snode = slink->tail;
		//save the new generated path in yPath for each dest
		yPath->path.push_back(slink);
		
	}

}

floatType TNM_TAP::RelativeGap(bool scale)
{
	TNM_SLINK *link;
	TNM_SORIGIN *org;
	TNM_SDEST *dest;
	floatType gap, tt =0.0, tmd = 0.0;

	for (int i = 0;i<network->numOfLink;i++)
	{
		link = network->linkVector[i];
		tt += link->volume * link->cost;
		//gap += link->volume * link->cost;
	}
	gap = tt;
	for(int i = 0;i<network->numOfOrigin;i++)
	{
		org = network->originVector[i];
		network->UpdateSP(org->origin);
		tmd += org->m_tdmd;
		for(int j = 0;j<org->numOfDest;j++)
		{
			dest = org->destVector[j];
			gap -= (dest->dest->pathElem->cost * dest->assDemand);
		}
	}
	
	if(scale) gap /= tt;
	
	return fabs(gap); //enforce postive
}

double TNM_TAP::ComputeBeckmannObj(bool toll)
{
    double ofv = 0.0;
	for (int i = 0;i<network->numOfLink;i++) ofv += network->linkVector[i]->GetIntCost(toll);
	return ofv;

}

void TNM_TAP::ComputeOFV()
{
	OFV  = ComputeBeckmannObj();
}

int TNM_TAP::Report()
{
	intWidth           = TNM_IntFormat::GetWidth();
	floatWidth         = TNM_FloatFormat::GetWidth();
	if(reportIterHistory)
	{
		string iteFileName = outFileName + ".ite"; //out file
		if (!TNM_OpenOutFile(iteFile, iteFileName))
		{
			cout<<"\n\tFail to build an algorithm object: cannot open .ite file to write!"<<endl;
		}
		else
		{
			cout<<"\tWriting the iteration history into file "<<(outFileName + ".ite")<<"..."<<endl;
			ReportIter(iteFile);
		}
	}
	if(reportLinkDetail)
	{
		string lfpFileName  = outFileName + ".lfp";
		if (!TNM_OpenOutFile(lfpFile, lfpFileName))
		{
			cout<<"\n\tFail to Initialize an algorithm object: Cannot open .lfp file to write!"<<endl;
		}
		else
		{
			cout<<"\tWriting link details into file "<<(outFileName + ".lfp")<<"..."<<endl;
			ReportLink(lfpFile); 
			
		}

	}
	if(reportPathDetail)
	{
		string pthFileName  = outFileName + ".pth";
		if (!TNM_OpenOutFile(pthFile, pthFileName))
		{
			cout<<"\n\tFail to Initialize an algorithm object: Cannot open .pth file to write!"<<endl;
		}
		else
		{
			cout<<"\tWriting path details into file "<<(outFileName + ".pth")<<"..."<<endl;
			ReportPath(pthFile); 
			
		}

	}
	if(iteFile.is_open()) iteFile.close();
	if(lfpFile.is_open()) lfpFile.close();
	if(pthFile.is_open()) pthFile.close();
	return 0;
}

void TNM_TAP::ReportPath(ofstream &out)
{
	int pthid =0;
	out<<"path id      "<<"origin       "<<" dest      "<<"     path flow    "<<"   num of link    "<<"   links  "<<endl;
	for (int oi=0; oi<network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		for (int di=0; di< pOrg->numOfDest; di++)
		{
			TNM_SDEST* sdest = pOrg->destVector[di];

			for (int pi=0; pi<sdest->pathSet.size(); pi++)
			{
				TNM_SPATH* path = sdest->pathSet[pi];
				if (path->flow > 1e-4)
				{
					//output the path 
					pthid ++;
					//
					out<<TNM_IntFormat(pthid,4)<<TNM_IntFormat(pOrg->origin->id,4)<<TNM_IntFormat(sdest->dest->id,4)<<" "<<TNM_FloatFormat(path->flow,12,6)<<" "
						<<TNM_IntFormat(path->path.size(),4);
					for (int li=0; li<path->path.size();li++)
					{
						TNM_SLINK* link = path->path[li];
						out<<TNM_IntFormat(link->id,4);
					}
					out<<endl;
				}
			}
		}
	}
}

void TNM_TAP::ReportLink(ofstream &linkFile)
{
	linkFile<<setw(intWidth)<<"ID"
			<<setw(intWidth)<<"From"
			<<setw(intWidth)<<"To"
			<<setw(floatWidth)<<"Cap"
			<<setw(floatWidth)<<"Flow"
			<<setw(floatWidth)<<"Cost"
			<<setw(floatWidth)<<"Toll"<<endl;
	for (int i = 0;i<network->numOfLink;i++)
	{
		TNM_SLINK *link = network->linkVector[i];
		//link->fdCost = link->GetToll();
		link->fdCost = 0.;
		linkFile<<TNM_IntFormat(link->id)
				<<TNM_IntFormat(link->tail->id)
				<<TNM_IntFormat(link->head->id)
				<<TNM_FloatFormat(link->capacity)<<" "
				<<TNM_FloatFormat(link->volume,18,11)<<" "
				<<TNM_FloatFormat(link->cost)<<" "
				<<TNM_FloatFormat(link->fdCost)<<endl;
	}
}

void TNM_TAP::ReportIter(ofstream &out)
{
	if(!iterRecord.empty())
	{
		out<<setw(intWidth)<<"Iter"
			<<setw(floatWidth)<<"OFV"
			<<setw(floatWidth)<<"ConvIndc"
			<<setw(floatWidth)<<"Time"
			<<endl;
	 	for (vector<ITERELEM*>::iterator pv = iterRecord.begin();pv != iterRecord.end(); pv++)
		{
			out<<TNM_IntFormat((*pv)->iter)<<" "
				<<TNM_FloatFormat((*pv)->ofv)<<" "
				<<TNM_FloatFormat((*pv)->conv)<<" "
				<<TNM_FloatFormat((*pv)->time)<<endl;
		}
	}

}

void TAP_Greedy::PreProcess()
{
	TNM_TAP::PreProcess();
	if(m_resetNetworkOnSolve) 
	{
		network->Reset();
	}
}

void TAP_Greedy::Initialize()
{

	network->AllocateNodeBuffer(2);
	network->AllocateLinkBuffer(3);

	network->UpdateLinkCost();
	network->InitialSubNet3();//Implementing the all-or-nothing algorithm
	network->UpdateLinkCost();


	network->UpdateLinkCostDer();
	ComputeOFV(); //compute objective function value

	cout<<"The initial objective is : "<<OFV<<endl;
	
	for (int oi=0;oi<network->numOfOrigin;oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		for (int di=0;di<pOrg->numOfDest;di++)
		{
			TNM_SDEST* sdest = pOrg->destVector[di];
			if(pOrg->origin == sdest->dest) continue;
			TNM_SPATH* spath = sdest->pathSet.front();
			nPath++;
			spath->id = nPath;
				
		}
	}

	cout<<"The num of links are"<<network->numOfLink<<endl;

	cout<<"end of the ini"<<endl;

}

void TAP_Greedy::PostProcess()
{
	network->UpdateLinkCost();
}

TAP_Greedy::TAP_Greedy()
{
	aveFlowChange = 1.0;
	convIndicator = 1.0;
	nPath = 0;
}

TAP_Greedy::~TAP_Greedy()
{
	//delete yPath;
}

void TAP_Greedy::MainLoop()
{
	TotalFlowChange = 0.0;
	numOfPathChange =0;
	floatType oldOFV = OFV;
	totalShiftFlow = 0.0;
	maxPathGap = 0;

	/*Loop over all OD pairs, update the path set and perform a flow shift*/
	TNM_SORIGIN *pOrg;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		/*Updating the shortest path tree*/
		network->UpdateSP(pOrg->origin);
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			if(pOrg->origin == dest->dest) continue;
			dest->shiftFlow = 1.0;
			/*Updating the path set*/
			ColumnGeneration(pOrg,dest);
		    
			columnG = true;
			/*Perform a flow shift*/
			UpdatePathFlowGreedy(pOrg,dest);
		}

	}
	
	/*Inner loop*/
	innerShiftFlow =1.0;
	int il;
	int numofPath;
	double preFlowPre =1e-10;
	maxPathGap = 0.0;
	numOfD =0;
	int numofD2 =0;
	int maxInIter = 500;
	for (il =0; il< maxInIter; il++)
	{
		innerShiftFlow = 0.0;
		
		numofPath = 0;
		numofD2 = 0;
		for (int i = 0;i<network->numOfOrigin;i++)
		{
			pOrg = network->originVector[i];
		
			for (int j=0;j<pOrg->numOfDest;j++)
			{
				TNM_SDEST* dest = pOrg->destVector[j];
				if(pOrg->origin == dest->dest) continue;
				if (il%(maxInIter/100) == 0)
				{
					dest->shiftFlow =1.0;
				}
				if (dest->shiftFlow> convIndicator/2.0)
				{
					columnG = false;
					/*Perform a flow shift*/
					UpdatePathFlowGreedy(pOrg,dest);
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
	
	/*Remove paths with zero flow*/
	int depathsetsize = dePathSet.size();
	TNM_SPATH* path;
	for (int pi=0; pi<dePathSet.size(); pi++)
	{
		path = dePathSet[pi];
		delete path;
	}
	dePathSet.clear();
	//
	convIndicator = RelativeGap();//compute Relative Gap
	aveFlowChange = TotalFlowChange/numOfPathChange;

	ComputeOFV(); //compute objective function value;


	cout<<"The curIter is "<<curIter<<endl;
	cout<<"The OFV is "<<OFV<<endl;
	cout<<"The gap is "<<convIndicator<<endl;
	cout<<"The total shifted Flow in this iteration is "<<totalShiftFlow<<endl;
	cout<<"il is "<<il<<endl;
	cout<<"innerFlowShift is "<<innerShiftFlow<<endl;
	cout<<"The number of dest shifted is "<<numOfD<<endl;
	cout<<"The number of searched OD pair is "<<numofD2<<endl;
	cout<<"The max path cost gap is "<<maxPathGap<<endl;
	cout<<"The number of Path is "<<numofPath<<endl;
	cout<<"The size of dePathSet is "<<depathsetsize<<endl;
	cout<<"The current time is "<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
	
}

void TAP_Greedy::UpdatePathFlowGreedy(TNM_SORIGIN* pOrg,TNM_SDEST* dest)
{
	//check if the yPath is existed in the path set
	bool find = true;
	TNM_SLINK* slink;
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
			//
		}
		

	}
	
	
	if (dest->pathSet.size()>1)
	{
		int doIter = 0;
		bool repeat = false;
		do 
		{
			doIter++;
			//update the path cost
			double maxCost =0.0;
			double minCost = 100000000.0;
			for (int pi=0;pi<dest->pathSet.size();pi++)
			{
				path = dest->pathSet[pi];
				path->preFlow = path->flow;
				path->preRatio = path->preFlow/dest->assDemand;
				path->cost = 0.0;
				path->fdCost = 0.0;
				path->curRatio =0.0;
				path->markStatus = 0;
				for (int li=0;li<path->path.size();li++)
				{
					slink = path->path[li];
					path->cost+=slink->cost;
					path->fdCost+= slink->fdCost;

				}
				path->estCost=path->cost - path->fdCost*dest->assDemand*path->preRatio;
				if (path->cost>maxCost)
				{
					maxCost = path->cost;
				}
				if (path->cost < minCost)
				{
					minCost = path->cost;
				}
			}
			double ss;
			if (curIter == 1)
			{
				ss = 1e-3;
			}
			else
			{
				ss = convIndicator/2.0;
			}

			if (maxCost - minCost > maxPathGap)
			{
				maxPathGap = maxCost - minCost;
			}
			dest->shiftFlow = maxCost - minCost;
			if (maxCost - minCost > ss)
			{
				repeat = true;
				numOfD++;
				//sort the pathset according to the path's cost 
				QuickSortPath(dest->pathSet,0,dest->pathSet.size()-1);
				dest->costDif = abs(dest->pathSet[dest->pathSet.size()-1]->estCost - dest->pathSet[0]->estCost);
				//re-assign the path flows by greedy algorithm
				double w = dest->pathSet[0]->estCost + dest->pathSet[0]->fdCost*dest->assDemand;
				double B = 1.0/(dest->pathSet[0]->fdCost*dest->assDemand);
				double C = dest->pathSet[0]->estCost/(dest->pathSet[0]->fdCost*dest->assDemand);
				vector<TNM_SPATH*> tempPathSet;
				tempPathSet.clear();
				
				dest->pathSet[0]->markStatus = 1;
				tempPathSet.push_back(dest->pathSet[0]);
				dest->pathSet[0]->curRatio = 1.0;
				int wi = 1;
				
				while (  wi < dest->pathSet.size() && dest->pathSet[wi]->estCost < w)
				{
					path = dest->pathSet[wi];
					path->markStatus = 1;
					C = C+ path->estCost/(path->fdCost*dest->assDemand);
					B = B + 1.0/(path->fdCost*dest->assDemand);
					tempPathSet.push_back(path);
					w = (1.0+C)/B;
					double tr =0.0;
					for (int pj=0;pj<tempPathSet.size();pj++)
					{
						TNM_SPATH* kpath = tempPathSet[pj];
						kpath->curRatio = (w-kpath->estCost)/(kpath->fdCost*dest->assDemand);

					}

					wi++;
				}
				while(wi < dest->pathSet.size())
				{
					dePathSet.push_back(dest->pathSet[wi]);
					wi++;
				}
				
				//update the flow and link cost
				for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin();pv != dest->pathSet.end();pv++)
				{
					path = *pv;
					{
						//this path has flow 
						path->flow = path->curRatio*dest->assDemand;
						double dflow = path->flow - path->preFlow;

						if (abs(dflow)> 1e-10)
						{
							dest->shiftFlow = abs(dflow);
							totalShiftFlow+=abs(dflow);
							innerShiftFlow+=abs(dflow);
							for (int li =0; li<path->path.size();li++)
							{
								slink = path->path[li];
								slink->volume = slink->volume + dflow;
								if ( abs(slink->volume) < 1e-8)
								{
									slink->volume =0.0;
								}
								if (slink->volume<0)
								{
									cout<<"status "<<1<<endl;
									cout<<"slink's volume is "<<slink->volume<<endl;
									cout<<"dflow is "<<dflow<<endl;
									cout<<"curRatio is "<<path->curRatio<<endl;
									cout<<"path flow "<<path->flow<<endl;
									cout<<"path preflow is "<<path->preFlow<<endl;
									cout<<"demand is "<<dest->assDemand<<endl;
									system("PAUSE");
								}
								slink->cost = slink->GetCost();
								slink->fdCost = slink->GetDerCost();
							}
						}

					}

				}
				//update the pathset
				if (tempPathSet.size()<dest->pathSet.size())
				{
					dest->pathSet = tempPathSet;

				}

			}
			else
			{
				repeat = false;
			}
		} while (repeat && doIter < 1);
	   
		
	}
}

//sort the path set according to the path's cost fro min to max
void TAP_Greedy::QuickSortPath(vector<TNM_SPATH*> & order,int low, int high)
{
	if( low < high)
	{
		TNM_SPATH * spath = order[low];
		int i = low;
		int j = high;
		while(i<j)
		{
			while ( (i < j) && (order[j]->estCost >= spath->estCost))
			{
				j=j-1;
			}
			order[i] = order[j];
			while( ( i<j ) && (order[i]->estCost <= spath->estCost))
			{
				i=i+1;
			}
			order[j] = order[i];
		}
		order[i] = spath;

		QuickSortPath(order,low,i-1);
		QuickSortPath(order,i+1,high);
	}
}

void CTAP_ALM_Greedy::InitialSolution()
{
	//*Get an initial solution using the All-nothing assignment*//
	TNM_SLINK* link;
	for(int a=0;a<network->numOfLink;a++)
	{
		link=network->linkVector[a];
		link->cost=link->fft;
		link->volume=0.0;
	}
	TNM_SORIGIN* pOrg;
	for(int mlai=0;mlai<network->numOfOrigin;mlai++)
	{
		pOrg=network->originVector[mlai];
		network->UpdateSP(pOrg->origin);
		for (int mlaj=0;mlaj<pOrg->numOfDest;mlaj++)
		{
			TNM_SDEST* dest = pOrg->destVector[mlaj];
			if(pOrg->origin->id!=dest->dest->id)
			{
				ly_ColumnGeneration(pOrg,dest);
				TNM_SPATH* Spath=dest->yPath;
				Spath->flow=dest->assDemand;
				for(int a=0;a<Spath->path.size();a++)
				{
					Spath->path[a]->volume += Spath->flow;
				}
			}
		}
	}


	//*Get a UE solution using the Greedy assignment*//
	int ccc=0;
	network->UpdateLinkCost();
	convIndicator=RelativeGap();
	do
	{
		TAP_Greedy::MainLoop();
		ccc++;
	}while(ccc<100 && convIndicator>1e-8);
	
	for(int a=0;a<network->numOfLink;a++)
	{
		link=network->linkVector[a];
	}
	cout<<"Obtained a UE initial solition with RG="<<convIndicator<<endl;
}

void CTAP_ALM_Greedy::Initialize()
{
	InitialSolution();

	FI=CalculateFlowInfeasible();
	ComputeOFV();
	SetiniPntCoe();
	SetiniMultiplier();
}

void CTAP_ALM_Greedy::SetiniPntCoe()
{
	floatType mm=0;
	TNM_SLINK* link;
	for(int a=0;a<network->numOfLink;a++)
	{
		link=network->linkVector[a];
		if(link->volume>link->capacity)
			mm += pow((link->volume-link->capacity),2);
	}
	if(mm>0)
		PntCoe=OFV/(mm*10.0);
	else
		PntCoe=1;
}

void CTAP_ALM_Greedy::SetiniMultiplier()
{
	for(int smi=0;smi<network->numOfLink;smi++)
	{
		TNM_SLINK* link=network->linkVector[smi];
		link->multiplier=link->GetCost()+PntCoe*max(long double(0),(link->volume-link->capacity))-link->fft*1.15;
		link->multiplier=max(long double(0),link->multiplier);
	}
}

void CTAP_ALM_Greedy::MainLoop()
{
	UpdateLinkgCost();
	
	//Sub-Loop for solving the Subproblem
	subIter=0;
	do
	{
		SubLoop_ALM();
	}while((RG>=SubConvCriterion && subIter<=MaxSubIter));

	GAP=CalculateGAP();
	ComputeOFV();//compute objective function value
	preFI=FI;
	FI=CalculateFlowInfeasible();
	cout<<"CurIter="<<curIter<<",OFV="<<OFV<<",RG="<<RG<<",FI="<<FI<<",GAP="<<GAP<<",PntCoe="<<PntCoe
		<<", current time="<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
		
	UpdateMultiplier();
	UpdatePntCoe();

}

void CTAP_ALM_Greedy::SubLoop_ALM()
{
	subIter++;
	InnerIter=0;
	TNM_SORIGIN* orgn;
	for(int i=0;i<network->numOfOrigin;i++)
	{
		orgn=network->originVector[i];
		UpdateSPandPathSet(orgn);
		for(int j=0;j<orgn->numOfDest;j++)
		{
			TNM_SDEST* dest=orgn->destVector[j];
			if(orgn->origin->id!=dest->dest->id)
			{
				CalculateFlow_GREEDY_ALM(orgn,dest);
			}
		}
	}

	//Inner-Loop
	int odN=0;
	int numofPath=0;
	while(InnerIter<100)
	{
		InnerIter++;
		numofPath=0;
		innerShiftFlow=0.0;
		TNM_SORIGIN* orgn;
		for(int i=0;i<network->numOfOrigin;i++)
		{
			orgn=network->originVector[i];
			TNM_SDEST* dest;
			for(int j=0;j<orgn->numOfDest;j++)
			{
				dest=orgn->destVector[j];
				if(orgn->origin->id!=dest->dest->id)
				{
					if (InnerIter%20 == 0)
				    {
					    dest->shiftFlow =1.0;
				    }
					if (dest->shiftFlow> RG/10.0)
				    {
					    if(CalculateFlow_GREEDY_ALM(orgn,dest))
						    odN++;
				    }

					numofPath+=dest->pathSet.size();
				}
			}
		}
		
		if(innerShiftFlow<1e-5)
		{
			//cout<<"total shift flow is less than 1e-5"<<endl;
			break;
		}
			
	}
	
	RG=UpdateRelativeGap();
	DeleteZeroPath();
	
	cout<<"\tSubRG="<<RG<<", numof search od="<<odN<<",current time="<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
}

void CTAP_ALM_Greedy::DeleteZeroPath()
{
	TNM_SORIGIN* pOrg;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			TNM_SPATH* path;
			for(PTRPATH pv=dest->pathSet.begin();pv!=dest->pathSet.end();)
			{
				if((*pv)->flow==0.0)
				{
					if((*pv)->buffer != NULL)
					{
						delete[] (*pv)->buffer;
						(*pv)->buffer = NULL;
					}
					delete (*pv);
					pv=dest->pathSet.erase(pv);
				}
				else
					pv++;
			}
		}
	}
}

floatType CTAP_ALM_Greedy::UpdateRelativeGap()
{
	TNM_SLINK* link,*temp;
	floatType hh=0,gap=0,min=1000000;
	for(int a=0;a<network->numOfLink;a++)
	{
		link=network->linkVector[a];
		hh += link->volume*link->cost;
	}
	gap=hh;
	//
	TNM_SORIGIN* pOrg;
	for(int i=0;i<network->numOfOrigin;i++)
	{
		pOrg=network->originVector[i];
		network->UpdateSP(pOrg->origin);
		for (int mlaj=0;mlaj<pOrg->numOfDest;mlaj++)
		{
			TNM_SDEST* dest = pOrg->destVector[mlaj];
			if(pOrg->origin->id==dest->dest->id)
				continue;
			gap -= (dest->dest->pathElem->cost * dest->assDemand);
		}
	}
	
	gap /= hh;
	return gap;
}

bool CTAP_ALM_Greedy::CalculateFlow_GREEDY_ALM(TNM_SORIGIN* orgn,TNM_SDEST* dest)
{
	TNM_SLINK* link;
	TNM_SPATH* path;
	floatType cmax=0,cmin=1000000;
	for(int k=0;k<dest->pathSet.size();k++)
	{
		path=dest->pathSet[k];
		path->cost=0;
		path->fdCost=0;
		path->markStatus=0;
		for(int a=0;a<path->path.size();a++)
		{
			link=path->path[a];
			path->cost += link->cost;
			path->fdCost += link->fdCost;
		}
		path->estCost=path->cost-path->fdCost*path->flow;
		if(k==0)
		{
			cmax=path->cost;
			cmin=path->cost;
		}
		else
		{
			if(path->cost>cmax)
				cmax=path->cost;
			if(path->cost<cmin)
				cmin=path->cost;
		}
	}
	dest->shiftFlow = cmax-cmin;
	//calculate path flow
	floatType ss;
	if(curIter==1)
		ss=1e-2;
	else
		ss=RG/100.0;
	if(abs(cmax-cmin)>ss )
	{
		QuickSortPath(dest->pathSet,0,dest->pathSet.size()-1);
		if(dest->pathSet[0]->fdCost!=0)
		{
			vector<TNM_SPATH*> tempPathset;
			tempPathset.clear();
			floatType B=1/(dest->pathSet[0]->fdCost*dest->assDemand);
			floatType C=dest->pathSet[0]->estCost/(dest->pathSet[0]->fdCost*dest->assDemand);
			floatType W=dest->pathSet[0]->estCost+dest->pathSet[0]->fdCost*dest->assDemand;
			tempPathset.push_back(dest->pathSet[0]);
			dest->pathSet[0]->markStatus=1;
			int count=2;
			while(count<=dest->pathSet.size() && dest->pathSet[count-1]->estCost<W)
			{
				path=dest->pathSet[count-1];
				path->markStatus=1;
				C += path->estCost/(path->fdCost*dest->assDemand);
				B += 1/(path->fdCost*dest->assDemand);
				W=(1+C)/B;
				tempPathset.push_back(path);
				count++;
			}
			//update path flow and link flow
			floatType demad=0;
			int num=0;
			for(vector<TNM_SPATH*>::iterator pv=dest->pathSet.begin();pv!=dest->pathSet.end();pv++)
			{
				floatType diff;
				if((*pv)->markStatus==0)
				{
					(*pv)->preFlow=(*pv)->flow;
					(*pv)->flow = 0;
					diff=(*pv)->flow-(*pv)->preFlow;
				}
				else
				{
					(*pv)->preFlow=(*pv)->flow;
					if(tempPathset.size()==1)
						(*pv)->flow=dest->assDemand;
					else
						(*pv)->flow = (W-(*pv)->estCost)/(*pv)->fdCost;

					if((*pv)->flow<0 && (*pv)->flow>-1e-10)
						(*pv)->flow=0;

					num++;
					if(num==tempPathset.size())
						(*pv)->flow=dest->assDemand-demad;

					if((demad+(*pv)->flow)>dest->assDemand)
						(*pv)->flow=dest->assDemand-demad;
					
					demad += (*pv)->flow;
					diff=(*pv)->flow-(*pv)->preFlow;
				}
				if((*pv)->flow<0)
				{
					cout<<"Error address:"<<curIter<<","<<subIter<<","<<InnerIter<<","<<orgn->origin->id<<","<<dest->dest->id<<",Pid="<<(*pv)->id<<", flow="<<(*pv)->flow<<endl;
				}
				
				for(int a=0;a<(*pv)->path.size();a++)
				{
					link=(*pv)->path[a];
					//link->buffer[1]=link->volume;//it's not the true preflow of link, one link belong to many path.
					link->volume += diff;
					if(link->volume<0 && link->volume>-1e-7)
						link->volume=0;
				}
				
				innerShiftFlow+=abs(diff);
			}
			if(abs(dest->assDemand-demad)>1e-10)
			{
				cout<<"less demand "<<dest->assDemand-demad<<endl;
				cout<<"Error address:"<<curIter<<","<<subIter<<","<<InnerIter<<","<<orgn->origin->id<<","<<dest->dest->id<<endl;
			}
		}
		else
		{
			//update path flow and link flow
			for(int k=0;k<dest->pathSet.size();k++)
			{
				floatType diff;
				path=dest->pathSet[k];
				if(k==0)
				{
					path->preFlow=path->flow;
					path->flow=dest->assDemand;
					diff=path->flow-path->preFlow;
				}
				else
				{
					path->preFlow=path->flow;
					path->flow=0;
					diff=path->flow-path->preFlow;
				}
				
				for(int a=0;a<path->path.size();a++)
				{
					link=path->path[a];
					link->volume += diff;
				}
				innerShiftFlow+=abs(diff);
			}
		}
		UpdateLinkgCost(dest);
		return true;
	}
	return false;
}

void CTAP_ALM_Greedy::UpdateSPandPathSet(TNM_SORIGIN* pOrg)
{
	network->UpdateSP(pOrg->origin);
	for (int mlaj=0;mlaj<pOrg->numOfDest;mlaj++)
	{
		TNM_SDEST* dest = pOrg->destVector[mlaj];
		if(pOrg->origin->id!=dest->dest->id)
		{
			ly_ColumnGeneration(pOrg,dest);
		}
	}
}

void CTAP_ALM_Greedy::ly_ColumnGeneration(TNM_SORIGIN* pOrg,TNM_SDEST* dest)
{
	yPath->path.clear();
	yPath->flow = 0;
	yPath->cost = 0;
	//
	TNM_SNODE* snode;
	TNM_SLINK* slink;
	snode = dest->dest;
	if(pOrg->origin->id!=dest->dest->id)
	{
		while(snode != pOrg->origin)
		{
			slink = snode->pathElem->via;
			if(slink == NULL) 
			{
				cout<<"OID="<<pOrg->origin->id<<",DID="<<dest->dest->id<<endl;
				TNM_SNODE* enode;
				TNM_SLINK* elink;
				for(int aa=0;aa<network->numOfLink;aa++)
				{
					if(_isnan(network->linkVector[aa]->cost))
						cout<<" "<<network->linkVector[aa]->id<<" "<<network->linkVector[aa]->cost<<" "<<network->linkVector[aa]->volume<<endl;
				}
				enode=dest->dest;
				while(enode != pOrg->origin)
				{
					elink = snode->pathElem->via;
					cout<<elink->orderID<<"->";
					enode =elink->tail;
				}
				cout<<"Fail to find shortest path."<<endl;
				break;
			}
		
			snode = slink->tail;
			//save the new generated link in yPath
			yPath->path.push_back(slink);
			
		}
		
		//check if yPath is in the path set
		bool find=false;
		TNM_SPATH* path;
		if(dest->pathSet.size()>0)
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
					dest->yPath = path;
					break;
				}

			}
		}
		//save the new generated path in yPath
		if (!find)
		{
			nPath++;
			TNM_SPATH* addPath = new TNM_SPATH;
			addPath->path = yPath->path;
			addPath->id = nPath;
			dest->pathSet.push_back(addPath);
			
			dest->yPath=addPath;
			if(!dest->yPath)
				cout<<"Fail to create new path."<<endl;
		}

	}
}

void CTAP_ALM_Greedy::UpdateLinkgCost()
{
	for(int ulci=0;ulci<network->numOfLink;ulci++)
	{
		TNM_SLINK* link=network->linkVector[ulci];
		//link->buffer[0]=link->fft*(1+0.15*pow((link->volume/link->capacity),4));
		link->cost=link->GetCost()+max(long double(0),link->multiplier+PntCoe*(link->volume-link->capacity));
		if(link->volume<(link->capacity-link->multiplier/PntCoe))
			//link->fdCost=0.6*link->fft*pow(link->volume,3)/pow(link->capacity,4);
			link->fdCost=link->GetDerCost();
		else
			link->fdCost=link->GetDerCost()+PntCoe;
	}
}

void CTAP_ALM_Greedy::UpdateLinkgCost(TNM_SDEST* dest)
{
	for(int k=0;k<dest->pathSet.size();k++)
	{
		TNM_SPATH* path=dest->pathSet[k];
		for(int a=0;a<path->path.size();a++)
		{
			TNM_SLINK* link=path->path[a];
			//link->buffer[0]=link->fft*(1+0.15*pow((link->volume/link->capacity),4));
			link->cost=link->GetCost()+max(long double(0),link->multiplier+PntCoe*(link->volume-link->capacity));
			if(link->volume<(link->capacity-link->multiplier/PntCoe))
				link->fdCost=link->GetDerCost();
			else
				link->fdCost=link->GetDerCost()+PntCoe;
		}
	}
}

floatType CTAP_ALM_Greedy::CalculateGAP()
{
	floatType gap=0;
	floatType MultiplierTerm=0;
	for(int ctaa=0;ctaa<network->numOfLink;ctaa++)
	{
		TNM_SLINK* link=network->linkVector[ctaa];
		gap+=pow(max(long double(0),link->multiplier+PntCoe*(link->volume-link->capacity)),2)-pow(link->multiplier,2);
		MultiplierTerm+=link->multiplier*(link->volume-link->capacity);
	}
	gap=gap/(2*PntCoe);
	gap=gap/OFV;

	return gap;
}

floatType CTAP_ALM_Greedy::CalculateFlowInfeasible()
{
	floatType norm=0;
	for(int clfni=0;clfni<network->numOfLink;clfni++)
	{
		TNM_SLINK* link=network->linkVector[clfni];
		if(link->volume>link->capacity)
		{
			norm+=pow((link->volume-link->capacity),2);
		}
	}
	norm=pow(norm,0.5);
	return norm;
}

void CTAP_ALM_Greedy::UpdateMultiplier()
{
	for(int umi=0;umi<network->numOfLink;umi++)
	{
		TNM_SLINK* link=network->linkVector[umi];
		link->multiplier=max(long double(0),link->multiplier+PntCoe*(link->volume-link->capacity));
	}
}

void CTAP_ALM_Greedy::UpdatePntCoe()
{
	if(FI >= ly_beta*preFI)
		PntCoe=PntCoe*ly_kapa;
}

ITERELEM * CTAP_ALM_Greedy::RecordCurrentIter()
{
	ITERELEM *iterElem;
	iterElem = new ITERELEM;
	iterElem->objID = objectID;
	iterElem->iter  = curIter;
	iterElem->ofv   = OFV;
	iterElem->step  = stepSize;
	iterElem->conv  = convIndicator;
	iterElem->FI	= FI;
	iterElem->GAP	= GAP;
	iterElem->RG	= RG;
	iterElem->time  = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
#ifdef IA_DEBUG
	cout<<*iterElem;
#endif
	iterRecord.push_back(iterElem); 
	return iterElem;
}

void CTAP_ALM_Greedy::ReportIter(ofstream &out)
{
	if(!iterRecord.empty())
	{
		out<<setw(intWidth)<<"Iter"
			<<setw(floatWidth)<<"OFV"
			<<setw(floatWidth)<<"FI"
			<<setw(floatWidth)<<"GAP"
			<<setw(floatWidth)<<"RG"
			<<setw(floatWidth)<<"Time"
			<<endl;
	 	for (vector<ITERELEM*>::iterator pv = iterRecord.begin();pv != iterRecord.end(); pv++)
		{
			out<<TNM_IntFormat((*pv)->iter)<<" "
				<<TNM_FloatFormat((*pv)->ofv)<<" "
				<<TNM_FloatFormat((*pv)->FI)<<" "
				<<TNM_FloatFormat((*pv)->GAP)<<" "
				<<TNM_FloatFormat((*pv)->RG)<<" "
				<<TNM_FloatFormat((*pv)->time)<<endl;
		}
	}

}

void CUP_LY::Recover(TNM_SORIGIN* orgn, TNM_SDEST* dest)
{
	for (int i = (UnblockNode.size() - 1); i > 0; i--)
	{
		UnblockNode[i]->NodeSet.clear();
		delete UnblockNode[i];
		UnblockNode.pop_back();
	}
	UnblockNode[0]->NodeSet[0] = orgn->origin;
	orgn->origin->scanStatus = 1;
	//cout<<ResidualNode.size()<<endl;
	//ResidualNode.clear();
	//cout<<ResidualNode.size()<<endl;
	TNM_SNODE* node;
	for (int i = 0; i < network->numOfNode; i++)
	{
		node = network->nodeVector[i];
		network->nodeVector[i]->buffer[0] = -1;//the id of unblock link
		network->nodeVector[i]->buffer[1] = 0;//direction of the link
		network->nodeVector[i]->buffer[2] = 0;//residual capacity of this path
		if (node->id != orgn->origin->id)
		{
			//ResidualNode.push_back(node);
			node->scanStatus = 0;
		}
		else
			orgn->origin->buffer[2] = dest->buffer[0];
	}
	//for(int i=0;i<network->numOfNode;i++)
	//{
	//	network->nodeVector[i]->buffer[0]=-1;//the id of unblock link
	//	network->nodeVector[i]->buffer[1]=0;//direction of the link
	//	network->nodeVector[i]->buffer[2]=0;//residual capacity of this path
	//}
	//orgn->origin->buffer[2]=dest->buffer[0];
}

bool CUP_LY::HavebeSearched(TNM_SNODE* node)
{
	//cout<<123<<endl;
	bool have = false;
	for (int i = 0; i < UnblockNode.size(); i++)
	{
		subNodeSet* sns = UnblockNode[i];
		for (int j = 0; j < sns->NodeSet.size(); j++)
		{
			TNM_SNODE* hnode = sns->NodeSet[j];
			//cout<<hnode->id<<","<<node->id<<endl;
			if (hnode->id == node->id)
				have = true;
		}
	}
	//if(have)
	//	cout<<"have"<<endl;
	return have;
}

void CUP_LY::deletefromResidual(TNM_SNODE* node)
{
	bool done = false;
	for (vector<TNM_SNODE*>::iterator pv = ResidualNode.begin(); pv != ResidualNode.end();)
	{
		if ((*pv)->id == node->id)
		{
			ResidualNode.erase(pv); done = true; break;
		}
		//cout<<"finish delete node"<<(*pv)->id<<endl;
		pv++;
	}
	if (!done)
		cout << "can't find node" << node->id << " from ResidualNode set." << endl;
}

void CUP_LY::SearchUnblockPath(TNM_SORIGIN* orgn, TNM_SDEST* dest)
{
	//int i=0;
	TNM_SLINK* neckLink;
	TNM_SNODE* necknode;
	bool bottleneck, finddest;
	do
	{

		do
		{
			bottleneck = false;
			finddest = false;
			subNodeSet* sns;//=UnblockNode[i];
			subNodeSet* new_sns = new subNodeSet;
			TNM_SLINK* flink;
			TNM_SNODE* node, * fnode;
			UnblockNode.push_back(new_sns);
			floatType minreducap = 1e15;
			/*if(orgn->origin->id>=11 && dest->dest->id>=22)
			{cout<<"\thave "<<sns->NodeSet.size()<<" node should be search"<<endl;}*/
			for (int h = 0; h < UnblockNode.size(); h++)
			{
				sns = UnblockNode[h];

				for (int j = 0; j < sns->NodeSet.size(); j++)
				{
					node = sns->NodeSet[j];
					for (int k = 0; k < node->forwStar.size(); k++)
					{
						flink = node->forwStar[k];
						fnode = flink->head;
						//cout<<12<<endl;
						//if(!HavebeSearched(fnode))
						if (fnode->scanStatus == 0)
						{
							//cout<<1234<<endl;
							if (flink->volume < flink->capacity)
							{
								new_sns->NodeSet.push_back(fnode);
								fnode->scanStatus = 1;
								//deletefromResidual(fnode);
								fnode->buffer[0] = flink->id;
								fnode->buffer[1] = 1;
								if (node->buffer[2] > (flink->capacity - flink->volume))
									fnode->buffer[2] = flink->capacity - flink->volume;
								else
									fnode->buffer[2] = node->buffer[2];
								if (fnode->id == dest->dest->id)
									finddest = true;
								/*if(orgn->origin->id>=11 && dest->dest->id>=22)
								{
									cout<<"find node"<<fnode->id<<endl;}*/
							}
							else
							{
								if (minreducap > node->buffer[2])
								{
									neckLink = flink;
									necknode = node;
									minreducap = node->buffer[2];
								}
							}

						}
					}
				}

			}

			if (new_sns->NodeSet.size() == 0)
			{
				bottleneck = true;
				UnblockNode.pop_back();
				delete new_sns;
			}

		} while (!bottleneck && !finddest);
		if (finddest)
		{
			TNM_SNODE* node = dest->dest;
			TNM_SLINK* link;
			do
			{
				if (node->buffer[1] == 1)
				{
					for (vector<TNM_SLINK*>::iterator pv = node->backStar.begin(); pv != node->backStar.end();)
					{
						if ((*pv)->id == node->buffer[0])
						{
							link = *pv; break;
						}
						pv++;
					}
					link->volume += dest->dest->buffer[2];
					node = link->tail;
				}
				else
					cout << "Path direction of node" << node->id << " is error!" << endl;
			} while (node->id != orgn->origin->id);
			dest->buffer[0] -= dest->dest->buffer[2];
			//orgn->origin->buffer[2]=dest->buffer[0];
		}

		if (bottleneck)
		{
			if (necknode->buffer[2] > 0.3 * dest->buffer[0])
				neckLink->capacity += 0.3 * dest->buffer[0] + 0.1;
			else
				neckLink->capacity += necknode->buffer[2] + 0.1;
			if (dest->buffer[0] < 0)
				cout << "negative reduced demand O=" << orgn->origin->id << " D=" << dest->dest->id << " reduced demand=" << dest->buffer[0] << endl;
			if (necknode->buffer[2] < 0)
				cout << "negatie reduced capacity O=" << orgn->origin->id << " D=" << dest->dest->id << " Nid=" << necknode->id << " reduced cap=" << necknode->buffer[2] << endl;
			//cout<<"Enlarge link"<<neckLink->id<<"'s capacity to "<<neckLink->capacity<<endl;
			//i--;
		}

	} while (bottleneck);

}

void CUP_LY::MainLoop()
{
	TNM_SORIGIN* orgn;
	for (int i = 0; i < network->numOfOrigin; i++)
	{
		orgn = network->originVector[i];
		TNM_SDEST* dest;
		for (int j = 0; j < orgn->numOfDest; j++)
		{
			dest = orgn->destVector[j];
			if (dest->buffer[0] != 0)
			{
				if (orgn->origin->id != dest->dest->id)
				{
					//cout<<"o="<<orgn->origin->id<<",d="<<dest->dest->id<<endl;
					odbeginT = clock();
					do
					{
						Recover(orgn, dest);
						SearchUnblockPath(orgn, dest);
						nowT = clock();
						floatType etime = (floatType)(nowT - odbeginT) / CLOCKS_PER_SEC;
						if (etime > 180)
						{
							cout << "This od pair takes too much time!! O=" << orgn->origin->id << " D=" << dest->dest->id << " reduced demand=" << dest->buffer[0] << endl;
							odbeginT = nowT;
						}
						//cout<<"link39's flow="<<network->linkVector[38]->volume<<endl;
					} while (dest->buffer[0] > 0);
				}
			}
		}
	}
}

void CUP_LY::Initialize()
{
	network->AllocateNodeBuffer(3);
	network->AllocateDestBuffer(1);
	TNM_SORIGIN* orgn;
	for (int i = 0; i < network->numOfOrigin; i++)
	{
		orgn = network->originVector[i];
		TNM_SDEST* dest;
		for (int j = 0; j < orgn->numOfDest; j++)
		{
			dest = orgn->destVector[j];
			dest->buffer[0] = dest->assDemand;//residual demand
		}
	}
	subNodeSet* sns = new subNodeSet;
	sns->NodeSet.push_back(network->nodeVector[0]);
	UnblockNode.push_back(sns);
}

bool CUP_LY::GameOver()
{
	bool canOver = true;
	floatType totreddemand = 0.0;
	TNM_SORIGIN* orgn;
	for (int i = 0; i < network->numOfOrigin; i++)
	{
		orgn = network->originVector[i];
		TNM_SDEST* dest;
		for (int j = 0; j < orgn->numOfDest; j++)
		{
			dest = orgn->destVector[j];
			if (orgn->origin->id != dest->dest->id)
			{
				totreddemand += dest->buffer[0];
				if (dest->buffer[0] != 0)
				{
					canOver = false;
					//break;
				}
			}
		}
	}
	cout << "Total reduced demand is " << totreddemand << endl;
	return canOver;
}

int CUP_LY::CyclicUnblockPath()
{

	Initialize();
	while (!GameOver())
	{
		MainLoop();
	}
	//for (int a = 0; a < network->numOfLink; a++)
	//{
	//	cout << "link" << network->linkVector[a]->id << "'s capacity=" << network->linkVector[a]->capacity << " flow=" << network->linkVector[a]->volume << endl;
	//	if (network->linkVector[a]->capacity < 0)
	//		cout << 1;
	//	if (network->linkVector[a]->capacity < network->linkVector[a]->volume)
	//		cout << 2;
	//}
	return 1;

}

void CUP_LY::RewriteNetFile(const string& inFile, const string& outFile)
{
	ifstream inf;
	//cout<<"orogonal file "<<inFileName<<endl;
	TNM_OpenInFile(inf, inFile + "_net.tntp");
	//string outfile = inFileName.replace(3, 7, "Enlarge network");//G:\\test network\\hearn\\hearn
	//outfile += "_net.dat";
	cout << "\n\tRewrite the net flie, adress is " << outFile << endl;
	ofstream outf;
	TNM_OpenOutFile(outf, outFile + "_net.tntp");
	vector<string> words;
	do
	{
		string line;
		getline(inf, line);
		outf << line << "\n";
		TNM_GetWordsFromLine(line, words);
	} while (words[0] != "~");
	int rownum = 0;
	while (!inf.eof())
	{
		string line;
		getline(inf, line);
		vector<string> words;
		TNM_GetWordsFromLine(line, words);
		for (int i = 0; i < words.size(); i++)
		{
			if (i == 2)
			{
				//outf<<"\t"<<TNM_FloatFormat(network->linkVector[rownum]->capacity,18,6);
				//cout<<"\t"<<network->linkVector[rownum]->id;
				outf << "\t" << network->linkVector[rownum]->capacity;
			}
			else
			{
				//cout<<"\t"<<words[i];
				outf << "\t" << words[i];
			}
		}
		//cout<<"\n";
		outf << "\n";
		rownum++;
	}

	inf.close();
	outf.close();
}