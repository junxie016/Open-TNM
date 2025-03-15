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


floatType TNM_TAP::RelativeGap2(bool scale)
{
	TNM_SLINK* link;
	TNM_SORIGIN* org;
	TNM_SDEST* dest;
	floatType gap, tt = 0.0, tmd = 0.0;

	for (int i = 0; i < network->numOfLink; i++)
	{
		link = network->linkVector[i];
		tt += link->volume * link->cost;
		//gap += link->volume * link->cost;
	}
	gap = tt;
	for (int i = 0; i < network->numOfOrigin; i++)
	{
		org = network->originVector[i];
		//network->UpdateSP(org->origin);
		tmd += org->m_tdmd;
		for (int j = 0; j < org->numOfDest; j++)
		{
			dest = org->destVector[j];
			network->SPath(org->origin, dest->dest);
			gap -= (org->origin->pathElem->cost * dest->assDemand);
		}
	}

	if (scale) gap /= tt;

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
					out<<TNM_IntFormat(pthid,4)<<TNM_IntFormat(pOrg->origin->id,4)<<TNM_IntFormat(sdest->dest->id,4)<<TNM_FloatFormat(path->flow,12,6)<< " "
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
				<<TNM_FloatFormat(link->volume,18,11)<< " "
				<<TNM_FloatFormat(link->cost)<< " "
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
			out<<TNM_IntFormat((*pv)->iter)
				<<TNM_FloatFormat((*pv)->ofv)<< " "
				<<TNM_FloatFormat((*pv)->conv)<< " "
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

	cout << "num of origin is " << network->numOfOrigin << endl;
	cout << " num of nodes is " << network->numOfNode << endl;

	network->UpdateLinkCost();
	network->InitialSubNet3();//Implementing the all-or-nothing algorithm
	network->UpdateLinkCost();


	//network->UpdateLinkCostDer();
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
	//yPath = new TNM_SPATH;
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



//////////////////////////////methods for Greedy_dijk
TAP_Greedy_dijk::TAP_Greedy_dijk( )
{

}

TAP_Greedy_dijk::~TAP_Greedy_dijk()
{

}

void TAP_Greedy_dijk::Initialize()
{

	network->AllocateNodeBuffer(2);
	network->AllocateLinkBuffer(3);

	cout << "num of origin is " << network->numOfOrigin << endl;
	cout << " num of nodes is " << network->numOfNode << endl;
	cout << " num of OD is " << network->numOfOD << endl;

	network->UpdateLinkCost();
	network->InitialSubNet4();//Implementing the all-or-nothing algorithm
	network->UpdateLinkCost();

	//network->UpdateLinkCostDer();
	ComputeOFV(); //compute objective function value

	cout << "The initial objective is : " << OFV << endl;

	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		for (int di = 0; di < pOrg->numOfDest; di++)
		{
			TNM_SDEST* sdest = pOrg->destVector[di];
			if (pOrg->origin == sdest->dest) continue;
			TNM_SPATH* spath = sdest->pathSet.front();
			nPath++;
			spath->id = nPath;

		}
	}

	cout << "The num of links are" << network->numOfLink << endl;

	cout << "end of the ini" << endl;

}


void TAP_Greedy_dijk::MainLoop()
{
	TotalFlowChange = 0.0;
	numOfPathChange = 0;
	floatType oldOFV = OFV;
	totalShiftFlow = 0.0;
	maxPathGap = 0;

	/*Loop over all OD pairs, update the path set and perform a flow shift*/
	TNM_SORIGIN* pOrg;
	for (int i = 0; i < network->numOfOrigin; i++)
	{
		pOrg = network->originVector[i];
		/*Updating the shortest path tree*/
		//network->UpdateSP(pOrg->origin);
		for (int j = 0; j < pOrg->numOfDest; j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			if (pOrg->origin == dest->dest) continue;
			dest->shiftFlow = 1.0;
			/*Updating the path set*/
			ColumnGeneration(pOrg, dest);

			columnG = true;
			/*Perform a flow shift*/
			UpdatePathFlowGreedy(pOrg, dest);
		}

	}

	/*Inner loop*/
	innerShiftFlow = 1.0;
	int il;
	int numofPath;
	double preFlowPre = 1e-10;
	maxPathGap = 0.0;
	numOfD = 0;
	int numofD2 = 0;
	int maxInIter = 500;
	for (il = 0; il < maxInIter; il++)
	{
		innerShiftFlow = 0.0;

		numofPath = 0;
		numofD2 = 0;
		for (int i = 0; i < network->numOfOrigin; i++)
		{
			pOrg = network->originVector[i];

			for (int j = 0; j < pOrg->numOfDest; j++)
			{
				TNM_SDEST* dest = pOrg->destVector[j];
				if (pOrg->origin == dest->dest) continue;
				if (il % (maxInIter /100) == 0)
				{
					dest->shiftFlow = 1.0;
				}
				if (dest->shiftFlow > convIndicator / 2.0)
				{
					columnG = false;
					/*Perform a flow shift*/
					UpdatePathFlowGreedy(pOrg, dest);
					numofD2++;

				}
				numofPath += dest->pathSet.size();
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
	for (int pi = 0; pi < dePathSet.size(); pi++)
	{
		path = dePathSet[pi];
		delete path;
	}
	dePathSet.clear();
	//
	convIndicator = RelativeGap2();//compute Relative Gap
	aveFlowChange = TotalFlowChange/numOfPathChange;

	ComputeOFV(); //compute objective function value;


	cout << "The curIter is " << curIter << endl;
	cout << "The OFV is " << OFV << endl;
	cout << "The gap is " << convIndicator << endl;
	cout << "The total shifted Flow in this iteration is " << totalShiftFlow << endl;
	cout << "il is " << il << endl;
	cout << "innerFlowShift is " << innerShiftFlow << endl;
	cout << "The number of dest shifted is " << numOfD << endl;
	cout << "The number of searched OD pair is " << numofD2 << endl;
	cout << "The max path cost gap is " << maxPathGap << endl;
	cout << "The number of Path is " << numofPath << endl;
	cout << "The size of dePathSet is " << depathsetsize << endl;
	cout << "The current time is " << 1.0 * (clock() - m_startRunTime) / CLOCKS_PER_SEC << endl;

}

void TAP_Greedy_dijk::ColumnGeneration(TNM_SORIGIN* pOrg, TNM_SDEST* dest)
{

	yPath->path.clear();
	yPath->flow = 0;
	yPath->cost = 0;

	yPath = network->SPath(pOrg->origin, dest->dest);


}