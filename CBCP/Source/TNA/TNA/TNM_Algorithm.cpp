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
	/*if(IsLoadTransit() && (tl== TT_MCTOLL || tl == TT_MTTOLL))
	{
		cout<<"\tCaution: when transit is included, performing system optimal assignment should be avoid...."<<endl;
		//tl = TT_NOTOLL;
	}*/
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
	switch (in)
	{
	//case NETFORT:
	//	if(network->BuildFort(lpf)!=0) 
	//	{
	//		cout<<"\tEncounter problems when building a network object!"<<endl;
	//		return 4;
	//	}
	//	break;
	//case NETDANET2:
	//	if(network->BuildDanet2(lpf)!=0) 
	//	{
	//		cout<<"\tEncounter problems when building a network object!"<<endl;
	//		return 4;
	//	}
	//	break;
	//case NETVISUM:
	//	if(network->BuildVISUM()!=0) 
	//	{
	//		cout<<"\tEncounter problems when building a network object!"<<endl;
	//		return 4;
	//	}
	//	break;
	//case NETVISUMS:
	//	if(network->BuildVISUMS()!=0) 
	//	{
	//		cout<<"\tEncounter problems when building a network object!"<<endl;
	//		return 4;
	//	}
	//	break;
	case NETTAPAS:
		if(network->BuildTAPAS(true, lpf)!=0) 
		{
			cout<<"\tEncounter problems when building a network object!"<<endl;
			return 4;
		}
		break;
	//case NETTRANSCAD:
	//	//cout<<"TRANSCAD file format is not ready yet"<<endl;
	//	if(network->BuildTRANSCAD()!=0) 
	//	{
	//		cout<<"\tEncounter problems when building a network object!"<<endl;
	//		return 4;
	//	}
	//	break;
	default:
		cout<<"\tUnrecognized network format. "<<endl;
		return 5;
	}
	network->ClearZeroDemandOD();
	return 0;
};

TERMFLAGS TNM_TAP::Solve()
{
	m_startRunTime = clock();
	PreProcess();
	numLineSearch = 0;
	if(termFlag!=ErrorTerm)
	{
		curIter = 0;
		Initialize();
		if(!Terminate())
		{
			RecordCurrentIter();
			do 
			{
				//m_OOFV = OFV;
				//m_OConv = convIndicator;
				//if(OFV < m_BOFV) m_BOFV = OFV;
				//if(convIndicator < m_BConv) m_BConv = convIndicator;
				curIter ++;
				//if(watchTime)
				//{
				//	UpdateReport();
				//}
				MainLoop();
				RecordCurrentIter();
			}while (!Terminate());
		}
		//if(termFlag!=ErrorTerm)
		//{
			PostProcess();
			//cout<<"I am strying to test if I need to store result.."<<endl;
			//if(IsStoreResult()) 
			//{
			//	//cout<<"storing result..."<<endl;
			//	StoreResult();
			//}
			termFlag = TerminationType();
		//}
	}
	cout<<"\n\n\tSolution process terminated"<<endl;
	cpuTime = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	return termFlag;
}

void TNM_TAP::PreProcess()
{
	stepSize                  = 1.0;
	//ziggIter                  = 0;
	//badIter                   = 0;
	convIndicator             = 1e3;           
	//ziggIndicator             = 1.0;
	//SetInitTerm();
	OFV                       = pow(2.0, 52.0); //set the intial value to a fairly big number.
	numLineSearch             = 0;
	//m_OOFV = m_BOFV = OFV;
	//m_OConv = m_BConv = convIndicator;
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
	//cout<<"termination type"<<endl;
	//cout<<"reach zigg or not?"<<(ReachZigg()?"YES":"NO")<<endl;
	if(ReachAccuracy()) return ConvergeTerm;
	if(ReachMaxIter())  return MaxIterTerm;
	//if(ReachMaxBad())   return MaxBadTerm;
	//if(ReachMinStep())  return MinStepTerm;
	//if(ReachZigg())     return ZiggTerm;
	if(ReachUser())     return UserTerm;
	//if(ReachObject())   return ObjectTerm;
	//if(ReachMaxCPU())   return MaxCPUTerm;
	return ErrorTerm;

}

void TNM_TAP::ColumnGeneration(TNM_SORIGIN* pOrg,TNM_SDEST* dest)
{

	//new a path
	//dest->yPath = new TNM_SPATH;
	yPath->path.clear();
	yPath->flow = 0;
	yPath->cost = 0;
	//
	TNM_SNODE* snode;
	TNM_SLINK* slink;
	snode = dest->dest;
	//test
	//cout<<"1.11"<<endl;

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
	//if(!IsMultiClass())
	{

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
	}
	//else
	//{
	//	TNM_MCSNET *net = (TNM_MCSNET*) network;
	//	int nclass = net->GetNumClass();
	//	gap = 0.0;		
	//	for(int i = 0;i<nclass;i++)
	//	{
	//		net->PrepareMultiClassAssignment(i + 1, true);
	//		//net->UpdateLinkCost();
	//		for (int j = 0;j<network->numOfLink;j++)
	//		{
	//			link = network->linkVector[j];
	//			tt += link->volume * link->cost;
	//			gap += link->volume * link->cost;
	//		}
	//		//gap+= tt;
	//		int startod, endod;
	//		if(i==0) startod = 0;
	//		else     startod = net->GetClassInfo(i)->m_orgIx;
	//		endod = net->GetClassInfo(i+1)->m_orgIx;
	//		for(int l = startod;l<endod;l++)
	//		{
	//			org = net->originVector[l];
	//			net->UpdateSP(org->origin);
	//			tmd += org->m_tdmd;
	//			for(int j = 0;j<org->numOfDest;j++)
	//			{
	//				dest = org->destVector[j];
	//				gap -= (dest->dest->pathElem->cost * dest->assDemand);
	//			}
	//		}
	//		net->FinishMultiClassAssignment();
	//	}
	//}
	if(scale) gap /= tt;
	
	return fabs(gap); //enforce postive
}

double TNM_TAP::ComputeBeckmannObj(bool toll)
{
    double ofv = 0.0;
/*	if(!network->firstBestToll)
	{
		for (int i = 0;i<network->numOfLink;i++) ofv += network->linkVector[i]->GetIntCost();
	}
	else
	{
		for (int i = 0;i<network->numOfLink;i++) ofv += network->linkVector[i]->GetIntCostWT();
	}*/
	for (int i = 0;i<network->numOfLink;i++) ofv += network->linkVector[i]->GetIntCost(toll);
	return ofv;

}

void TNM_TAP::ComputeOFV()
{
  //if(!IsMultiClass())
  {
	  OFV  = ComputeBeckmannObj();
  }
//  else
//  {
//
//	  TNM_MCSNET *net = (TNM_MCSNET*)network;
//	  net->UpdateTotalLinkFlow();
//	  OFV = ComputeBeckmannObj();
///*	  OFV = 0.0;
//	  for(int i = 1;i<=net->GetNumClass();i++)
//	  {
//		  net->PrepareMultiClassAssignment(i);
//		  net->UpdateLinkCost();
//		  OFV  += ComputeBeckmannObj();
//		  net->FinishMultiClassAssignment(true); //we update class specific cost here. 
//	  }*/
//  }
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
					out<<TNM_IntFormat(pthid,4)<<TNM_IntFormat(pOrg->origin->id,4)<<TNM_IntFormat(sdest->dest->id,4)<<TNM_FloatFormat(path->flow,12,6)
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
				<<TNM_FloatFormat(link->capacity)
				<<TNM_FloatFormat(link->buffer[0])
				<<TNM_FloatFormat(link->cost)
				<<TNM_FloatFormat(link->buffer[1])<<endl;
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
				<<TNM_FloatFormat((*pv)->ofv)
				<<TNM_FloatFormat((*pv)->conv)
				<<TNM_FloatFormat((*pv)->time)<<endl;
		}
	}

}

TAP_iGP::TAP_iGP()
{

}
TAP_iGP::~TAP_iGP()
{

}

void TAP_iGP::MainLoop()
{
	//
	TotalFlowChange = 0.0;
	numOfPathChange =0;
	floatType oldOFV = OFV;
	totalShiftFlow = 0.0;
	maxPathGap = 0;
	//cout<<"1"<<endl;
	TNM_SORIGIN *pOrg;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];

		network->UpdateSP(pOrg->origin);
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			dest->shiftFlow = 1.0;
			//cout<<"1"<<endl;
			ColumnGeneration(pOrg,dest);
		    
			columnG = true;
			//cout<<"2"<<endl;
			UpdatePathFlowLazy(pOrg,dest);
			//cout<<"3"<<endl;
		}

	}
	//cout<<"2"<<endl;
	//inner loop
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
		//
		//cout<<"il is "<<il<<endl;
		//
		innerShiftFlow = 0.0;
		numofPath = 0;
		numofD2 = 0;
		for (int i = 0;i<network->numOfOrigin;i++)
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
	//cout<<"3"<<endl;
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

	ComputeOFV(); //compute objective function value;
	//cout<<"Main iter = "<<curIter<<" OFV = "<<OFV<<" conv = "<<convIndicator<<endl;

	/*double entr = ComputeEntropyfromPath();
	ProInfo * pif = new ProInfo;
	pif->iter = curIter;
	pif->Entro = entr;
	pif->consis =0;
	pif->time = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	EntropyinfoVector.push_back(pif);*/

	//cout<<"The curIter is "<<curIter<<endl;
	////cout<<"The current Entropy is "<<entr<<endl;
	//cout<<"The OFV is "<<OFV<<endl;
	//cout<<"The gap is "<<convIndicator<<endl;
	//cout<<"The total shifted Flow in this iteration is "<<totalShiftFlow<<endl;
	//cout<<"il is "<<il<<endl;
	//cout<<"innerFlowShift is "<<innerShiftFlow<<endl;
	//cout<<"The number of dest shifted is "<<numOfD<<endl;
	//cout<<"The number of searched OD pair is "<<numofD2<<endl;
	//cout<<"The max path cost gap is "<<maxPathGap<<endl;
	//cout<<"The number of Path is "<<numofPath<<endl;
	//cout<<"The size of dePathSet is "<<depathsetsize<<endl;
	//cout<<"The current time is "<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
	//cout<<"aveFlowchange is "<<aveFlowChange<<endl;
	//system("PAUSE");
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
									//PrintPath(path);
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

void TAP_iGP::PreProcess()
{
	TNM_TAP::PreProcess();
	if(m_resetNetworkOnSolve) 
	{
		network->Reset();
	}
}

void TAP_iGP::Initialize()
{

	//cout<<"This is the OFW algorithm!"<<endl;
	//system("PAUSE");
	//PrepareLogFile();
	//TAP_BBA::Initialize();
	network->AllocateNodeBuffer(2);
	network->AllocateLinkBuffer(3);

	
	//cout<<"0"<<endl;

	//if(!IsInitFromFile())
	{

		network->UpdateLinkCost();

		network->InitialSubNet3();

		network->UpdateLinkCost();


	}

	network->UpdateLinkCostDer();
	ComputeOFV(); //compute objective function value

	cout<<"The initial objective is : "<<OFV<<endl;
	//system("PAUSE");

	//initialize the OD path set
	//if (recordODPath)
	//{
	//	//
	//	for (int oi=0;oi<network->numOfOrigin;oi++)
	//	{
	//		TNM_SORIGIN* pOrg = network->originVector[oi];
	//		for (int di=0;di<pOrg->numOfDest;di++)
	//		{
	//			TNM_SDEST* sdest = pOrg->destVector[di];
	//			TNM_SPATH* spath = sdest->pathSet.front();
	//			nPath++;
	//			sdest->pathSet.front()->id = nPath;
	//			spath->PathCost();
	//			//
	//			FWRoute* fw = new FWRoute;
	//			fw->iter =0;
	//			fw->curPathID = nPath;
	//			fw->demand = sdest->assDemand;
	//			fw->entro = 0;
	//			fw->flowSet.push_back(sdest->assDemand);
	//			fw->CostSet.push_back(spath->cost);
	//			fw->preEnt =0;
	//			fw->curPathFlow = sdest->assDemand;
	//			fw->stepSize = 0;
	//			sdest->iteInfoVector.push_back(fw);

	//		}
	//	}
	//}
	//else
	{
		for (int oi=0;oi<network->numOfOrigin;oi++)
		{
			TNM_SORIGIN* pOrg = network->originVector[oi];
			for (int di=0;di<pOrg->numOfDest;di++)
			{
				TNM_SDEST* sdest = pOrg->destVector[di];
				TNM_SPATH* spath = sdest->pathSet.front();
				nPath++;
				spath->id = nPath;

				//cout<<pOrg->origin->id<<"-->"<<sdest->dest->id<<"  's demand is "<<sdest->assDemand<<endl;
				
			}
		}
	}

	cout<<"The num of links are"<<network->numOfLink<<endl;

	//for (int i = 0; i < network->numOfLink; i++)
	//{
	//	TNM_SLINK* slink = network->linkVector[i];
	//	//cout<<slink->tail->id<<"-->"<<slink->head->id<<" : "<<slink->volume<< "   "<<slink->cost<<endl;
	//	cout<<slink->fft<<endl;
	//	cout<<slink->type<<endl;
	//	slink->GetCost();


	//}

	//PNList.reserve(network->numOfNode);
	//for(int i=0;i<network->numOfNode;i++)
	//{
	//	TNM_SNODE* onode = network->nodeVector[i];
	//	PasNode2* pon = new PasNode2;

	//	pon->NodePtr = onode;
	//	PNList.push_back(pon);
	//}
	////
	////ini the POList
	//POList.reserve(network->numOfOrigin);
	//for( int i=0;i<network->numOfOrigin;i++)
	//{
	//	TNM_SORIGIN* or = network->originVector[i];
	//	or->m_class = i;
	//	//cout<<"ID of origin "<<i<<" = "<<or->id()<<endl;
	//	PasOrigin2* por = new PasOrigin2;
	//	//ini the olinkList in por
	//	por->olinkList.reserve(network->numOfLink);
	//	for(int li=0;li<network->numOfLink;li++)
	//	{
	//		por->olinkList.push_back(NULL);
	//	}
	//	for(std::vector<ORGLINK*>::iterator pv =or->obLinkVector.begin(); pv != or->obLinkVector.end();pv++)
	//	{
	//		ORGLINK* olink =*pv;
	//		int id=olink->linkPtr->id-1;//all the id of links are plus 1from i
	//		por->olinkList[id] = olink;
	//	}
	//	//ini the onodeflow in por, the total flow of backstar links of this node
	//	por->onodeflow.reserve(network->numOfNode);
	//	for(int li=0;li<network->numOfNode;li++)
	//	{
	//		por->onodeflow.push_back(0);
	//	}
	//	//
	//	por->oLinkLable.reserve(network->numOfLink);
	//	for (int li=0; li<network->numOfLink;li++)
	//	{
	//		por->oLinkLable.push_back(false);
	//	}
	//	//give the origin ptr
	//	por->OriginPtr = or;
	//	POList.push_back(por);//same order with "network->originVector"
	//}
	////clear the PASList
	//for (int pi=0; pi<PasList.size();pi++)
	//{
	//	PairSegment2* pas = PasList[pi];
	//	delete pas;
	//}
	//PasList.clear();
	//for (int ni=0; ni<PNList.size();ni++)
	//{
	//	PasNode2* pn = PNList[ni];
	//	pn->NPasList.clear();
	//}

	//double entr = ComputeEntropyfromPath();
	//ProInfo * pif = new ProInfo;
	//pif->iter = curIter;
	//pif->Entro = entr;
	//pif->consis =0;
	//pif->time = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	//EntropyinfoVector.push_back(pif);
	cout<<"end of the ini"<<endl;

	//system("PAUSE");
}

void TAP_iGP::PostProcess()
{
	network->UpdateLinkCost();
}


////////////////////methods for the bcp algoirthm

TAP_BCP::TAP_BCP()
{
	myRow1 =0.1;
}
TAP_BCP::~TAP_BCP()
{

}


void TAP_BCP::UpdateLinkCost2()
{

	TNM_SLINK *link;
	for(int i = 0;i<network->numOfLink;i++)
	{
		link = network->linkVector[i];
		link->cost = (1.0 + myRow1)*link->GetCost()+link->volume*link->GetDerCost()+myRow1*link->buffer[2];
	}

}

void TAP_BCP::UpdateLinkCostDer2()
{

	TNM_SLINK *link;
	for(int i = 0;i<network->numOfLink;i++)
	{
		link = network->linkVector[i];
		link->fdCost = (2.0 + myRow1)*link->GetDerCost()+link->volume*link->GetDer2Cost();
	}

}

void TAP_BCP::Initialize2()
{

	//cout<<"This is the OFW algorithm!"<<endl;
	//system("PAUSE");
	//PrepareLogFile();
	//TAP_BBA::Initialize();

	//cout<<" initialize of assignment 2 is beginning"<<endl;
	
	//cout<<"0"<<endl;

	//if(!IsInitFromFile())
	{

		//network->UpdateLinkCost();
		UpdateLinkCost2();

		network->InitialSubNet3();

		//network->UpdateLinkCost();
		UpdateLinkCost2();


	}

	//network->UpdateLinkCostDer();
	UpdateLinkCostDer2();


	OFV = ComputeOFV2(); //compute objective function value

	//cout<<"The initial objective of assignment2 is : "<<OFV<<endl;
	//system("PAUSE");
	nPath =0;
	
	{
		for (int oi=0;oi<network->numOfOrigin;oi++)
		{
			TNM_SORIGIN* pOrg = network->originVector[oi];
			for (int di=0;di<pOrg->numOfDest;di++)
			{
				TNM_SDEST* sdest = pOrg->destVector[di];
				TNM_SPATH* spath = sdest->pathSet.front();
				nPath++;
				spath->id = nPath;
				
			}
		}
	}

	//cout<<"The num of links are"<<network->numOfLink<<endl;

	//cout<<"end of the ini"<<endl;

}

void TAP_BCP::Initialize22()
{

	TNM_SORIGIN* pOrg;
	TNM_SDEST* dest;
	TNM_SLINK* slink;

	for (int li = 0; li < network->numOfLink; li++)
	{
		slink = network->linkVector[li];
		slink->volume = slink->aux_volume2;
	}

	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		pOrg = network->originVector[oi];

		for (int di = 0; di < pOrg->numOfDest; di++)
		{
			dest = pOrg->destVector[di];

			//dest->pathSet.clear();
			dest->pathSet = dest->aux_pathSet2;
		}
	}


	UpdateLinkCost2();



	//network->UpdateLinkCostDer();
	UpdateLinkCostDer2();


	OFV = ComputeOFV2(); //compute objective function value


	//cout<<"This is the initialition"<<endl;
	//system("PAUSE");
}

void TAP_BCP::MainLoop2()
{
	//
	TotalFlowChange = 0.0;
	numOfPathChange =0;
	floatType oldOFV = OFV;
	totalShiftFlow = 0.0;
	maxPathGap = 0;
	//cout<<"1"<<endl;
	TNM_SORIGIN *pOrg;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];

		network->UpdateSP(pOrg->origin);
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			dest->shiftFlow = 1.0;
			//cout<<"1"<<endl;
			ColumnGeneration(pOrg,dest);
		    
			columnG = true;
			//cout<<"2"<<endl;
			UpdatePathFlowLazy2(pOrg,dest);
			//cout<<"3"<<endl;
		}

	}
	//cout<<"2"<<endl;
	//inner loop
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
		//
		//cout<<"il is "<<il<<endl;
		//
		innerShiftFlow = 0.0;
		numofPath = 0;
		numofD2 = 0;
		for (int i = 0;i<network->numOfOrigin;i++)
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
					UpdatePathFlowLazy2(pOrg,dest);
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
	//cout<<"3"<<endl;
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

	OFV=ComputeOFV2(); //compute objective function value;
	//cout<<"Main iter = "<<curIter<<" OFV = "<<OFV<<" conv = "<<convIndicator<<endl;

	/*double entr = ComputeEntropyfromPath();
	ProInfo * pif = new ProInfo;
	pif->iter = curIter;
	pif->Entro = entr;
	pif->consis =0;
	pif->time = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	EntropyinfoVector.push_back(pif);*/
	//cout<<"///////////////////////////////////"<<endl;
	//cout<<"The curIter of Assignment 2 is "<<curIter<<endl;
	////cout<<"The current Entropy is "<<entr<<endl;
	//cout<<"The OFV is "<<OFV<<endl;
	//cout<<"The gap is "<<convIndicator<<endl;
	//cout<<"The total shifted Flow in this iteration is "<<totalShiftFlow<<endl;
	//cout<<"il is "<<il<<endl;
	//cout<<"innerFlowShift is "<<innerShiftFlow<<endl;
	//cout<<"The number of dest shifted is "<<numOfD<<endl;
	//cout<<"The number of searched OD pair is "<<numofD2<<endl;
	//cout<<"The max path cost gap is "<<maxPathGap<<endl;
	//cout<<"The number of Path is "<<numofPath<<endl;
	//cout<<"The size of dePathSet is "<<depathsetsize<<endl;
	//cout<<"The current time is "<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
	//cout<<"aveFlowchange is "<<aveFlowChange<<endl;
	//system("PAUSE");
}


TERMFLAGS TAP_BCP::SolveAssingment2()
{
	//
	//cout<<"the assignment 2 is beginning"<<endl;

	//m_startRunTime = clock();
	PreProcess2();
	numLineSearch = 0;
	if(termFlag!=ErrorTerm)
	{
		curIter = 0;
		Initialize2();
		if(!Terminate())
		{
			RecordCurrentIter();
			do 
			{
				//m_OOFV = OFV;
				//m_OConv = convIndicator;
				//if(OFV < m_BOFV) m_BOFV = OFV;
				//if(convIndicator < m_BConv) m_BConv = convIndicator;
				curIter ++;
				//if(watchTime)
				//{
				//	UpdateReport();
				//}
				MainLoop2();
				RecordCurrentIter();
			}while (!Terminate());
		}
		//if(termFlag!=ErrorTerm)
		//{
			PostProcess2();
			//cout<<"I am strying to test if I need to store result.."<<endl;
			//if(IsStoreResult()) 
			//{
			//	//cout<<"storing result..."<<endl;
			//	StoreResult();
			//}
			termFlag = TerminationType();
		//}
	}

	cpuTime = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	//cout<<"\n\n\tSolution process terminated: "<<cpuTime<<endl;
	
	return termFlag;
}


TERMFLAGS TAP_BCP::SolveAssingment22()
{
	//
	//cout<<"the assignment 2 is beginning"<<endl;

	//m_startRunTime = clock();
	//PreProcess2();
	numLineSearch = 0;
	if(termFlag!=ErrorTerm)
	{
		curIter = 0;
		Initialize22();
		//if(!Terminate())
		{
			RecordCurrentIter();
			do 
			{
				//m_OOFV = OFV;
				//m_OConv = convIndicator;
				//if(OFV < m_BOFV) m_BOFV = OFV;
				//if(convIndicator < m_BConv) m_BConv = convIndicator;
				curIter ++;
				//if(watchTime)
				//{
				//	UpdateReport();
				//}
				//cout<<"/////////main loop22"<<endl;
				MainLoop2();
				RecordCurrentIter();
			}while (!Terminate());
		}
		//if(termFlag!=ErrorTerm)
		//{
			PostProcess2();
			//cout<<"I am strying to test if I need to store result.."<<endl;
			//if(IsStoreResult()) 
			//{
			//	//cout<<"storing result..."<<endl;
			//	StoreResult();
			//}
			termFlag = TerminationType();
		//}
	}

	cpuTime = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	//cout<<"\n\n\tSolution process terminated: "<<cpuTime<<endl;
	
	return termFlag;
}

void TAP_BCP::UpdatePathFlowLazy2(TNM_SORIGIN* pOrg,TNM_SDEST* dest)
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
									//PrintPath(path);
									system("PAUSE");

								}
								slink->cost = (1.0 + myRow1)*slink->GetCost()+slink->volume*slink->GetDerCost()+myRow1*slink->buffer[2];  //link cost for the bcp problem
								slink->fdCost = (2.0 + myRow1)*slink->GetDerCost()+slink->volume*slink->GetDer2Cost();

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
					slink->cost = (1.0 + myRow1)*slink->GetCost()+slink->volume*slink->GetDerCost()+myRow1*slink->buffer[2]; 
					slink->fdCost = (2.0 + myRow1)*slink->GetDerCost()+slink->volume*slink->GetDer2Cost();
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


void TAP_BCP::PreProcess2()
{
	TNM_TAP::PreProcess();
	if(m_resetNetworkOnSolve) 
	{
		network->Reset();
	}
}

void TAP_BCP::PostProcess2()
{
	//UpdateLinkCost2();
	//update the buffer varibales
	//cout<<"The link's volume and cost is "<<endl;
	TNM_SLINK* slink;
	for (int i = 0; i < network->numOfLink; i++)
	{
		slink =network->linkVector[i];
		slink->buffer[0] = slink->volume; //update the link volume
		slink->buffer[4] = slink->GetCost(); //update the corresponding link time cost

		//
		/*cout<<"volume is "<<slink->volume<<endl;
		cout<<"buffer[0] is "<<slink->buffer[0]<<endl;
		cout<<"the link time is "<<slink->buffer[4]<<endl;
		cout<<"The link cost is "<<slink->cost<<endl;*/
	}
	//store the pathset and volume
	for(int i=0;i<network->numOfLink;i++)
	{
		network->linkVector[i]->aux_volume2 = network->linkVector[i]->volume;
	}
	for(int oi=0; oi<network->numOfOrigin; oi++)
	{
		for(int di=0; di<network->originVector[oi]->numOfDest; di++)
		{
			//network->originVector[oi]->destVector[di]->aux_pathSet2.clear();
			network->originVector[oi]->destVector[di]->aux_pathSet2 = network->originVector[oi]->destVector[di]->pathSet;
			network->originVector[oi]->destVector[di]->pathSet.clear();
		}
	}
	//system("PAUSE");
}

//methods for the assignment problem 1


void TAP_BCP::Initialize1()
{

	//cout<<"0"<<endl;

	//if(!IsInitFromFile())
	{

		//network->UpdateLinkCost();
		UpdateLinkCost1();

		network->InitialSubNet3();

		//network->UpdateLinkCost();
		UpdateLinkCost1();


	}

	//network->UpdateLinkCostDer();
	UpdateLinkCostDer1();


	OFV=ComputeOFV1(); //compute objective function value

	//cout<<"The initial objective of assignment 1 is : "<<OFV<<endl;
	//system("PAUSE");

	{
		for (int oi=0;oi<network->numOfOrigin;oi++)
		{
			TNM_SORIGIN* pOrg = network->originVector[oi];
			for (int di=0;di<pOrg->numOfDest;di++)
			{
				TNM_SDEST* sdest = pOrg->destVector[di];
				TNM_SPATH* spath = sdest->pathSet.front();
				nPath++;
				spath->id = nPath;
				
			}
		}
	}

	//cout<<"The num of links are"<<network->numOfLink<<endl;

	
	//cout<<"end of the ini of assignment 1"<<endl;

}

void TAP_BCP::Initialize12()
{

	TNM_SORIGIN* pOrg;
	TNM_SDEST* dest;
	TNM_SLINK* slink;

	for (int li = 0; li < network->numOfLink; li++)
	{
		slink = network->linkVector[li];
		slink->volume = slink->aux_volume1;
	}

	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		pOrg = network->originVector[oi];

		for (int di = 0; di < pOrg->numOfDest; di++)
		{
			dest = pOrg->destVector[di];

			//dest->pathSet.clear();
			dest->pathSet = dest->aux_pathSet1;
		}
	}

		//network->UpdateLinkCost();
	UpdateLinkCost1();



	//network->UpdateLinkCostDer();
	UpdateLinkCostDer1();


	OFV=ComputeOFV1(); //compute objective function value

	

}


void TAP_BCP::MainLoop1()
{
	//
	TotalFlowChange = 0.0;
	numOfPathChange =0;
	floatType oldOFV = OFV;
	totalShiftFlow = 0.0;
	maxPathGap = 0;
	//cout<<"1"<<endl;
	TNM_SORIGIN *pOrg;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];

		network->UpdateSP(pOrg->origin);
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			dest->shiftFlow = 1.0;
			//cout<<"1"<<endl;
			ColumnGeneration(pOrg,dest);
		    
			columnG = true;
			//cout<<"2"<<endl;
			UpdatePathFlowLazy1(pOrg,dest);
			//cout<<"3"<<endl;
		}

	}
	//cout<<"2"<<endl;
	//inner loop
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
		//
		//cout<<"il is "<<il<<endl;
		//
		innerShiftFlow = 0.0;
		numofPath = 0;
		numofD2 = 0;
		for (int i = 0;i<network->numOfOrigin;i++)
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
					UpdatePathFlowLazy1(pOrg,dest);
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
	//cout<<"3"<<endl;
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

	OFV  = ComputeOFV1(); //compute objective function value;
	//cout<<"Main iter = "<<curIter<<" OFV = "<<OFV<<" conv = "<<convIndicator<<endl;
	
	/*double entr = ComputeEntropyfromPath();
	ProInfo * pif = new ProInfo;
	pif->iter = curIter;
	pif->Entro = entr;
	pif->consis =0;
	pif->time = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	EntropyinfoVector.push_back(pif);*/

	/*cout<<"The curIter is of Assignment 1 "<<curIter<<endl;
	cout<<"The current Entropy is "<<entr<<endl;
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
	cout<<"aveFlowchange is "<<aveFlowChange<<endl;*/
	//system("PAUSE");
}

TERMFLAGS TAP_BCP::SolveAssignment1()
{
	//cout<<"beginning of the Assignment 1"<<endl;

	//m_startRunTime = clock();
	PreProcess1();
	numLineSearch = 0;
	if(termFlag!=ErrorTerm)
	{
		curIter = 0;
		Initialize1();
		if(!Terminate())
		{
			RecordCurrentIter();
			do 
			{
				//m_OOFV = OFV;
				//m_OConv = convIndicator;
				//if(OFV < m_BOFV) m_BOFV = OFV;
				//if(convIndicator < m_BConv) m_BConv = convIndicator;
				curIter ++;
				//if(watchTime)
				//{
				//	UpdateReport();
				//}
				MainLoop1();
				RecordCurrentIter();
			}while (!Terminate());
		}
		//if(termFlag!=ErrorTerm)
		//{
			PostProcess1();
			//cout<<"I am strying to test if I need to store result.."<<endl;
			//if(IsStoreResult()) 
			//{
			//	//cout<<"storing result..."<<endl;
			//	StoreResult();
			//}
			termFlag = TerminationType();
		//}
	}

	cpuTime = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	//cout<<"\n\n\tSolution process terminated: "<<cpuTime<<endl;
	
	return termFlag;
}


TERMFLAGS TAP_BCP::SolveAssignment12()
{
	//cout<<"beginning of the Assignment 1"<<endl;

	//m_startRunTime = clock();
	//PreProcess1();
	numLineSearch = 0;
	if(termFlag!=ErrorTerm)
	{
		curIter = 0;
		Initialize12();
		//if(!Terminate())
		{
			RecordCurrentIter();
			do 
			{
				//m_OOFV = OFV;
				//m_OConv = convIndicator;
				//if(OFV < m_BOFV) m_BOFV = OFV;
				//if(convIndicator < m_BConv) m_BConv = convIndicator;
				curIter ++;
				//if(watchTime)
				//{
				//	UpdateReport();
				//}\

				MainLoop1();
				//cout<<"main loop "<<curIter<<endl;
				//system("PAUSE");
				RecordCurrentIter();
			}while (!Terminate());
		}
		//if(termFlag!=ErrorTerm)
		//{
			PostProcess1();
			//cout<<"I am strying to test if I need to store result.."<<endl;
			//if(IsStoreResult()) 
			//{
			//	//cout<<"storing result..."<<endl;
			//	StoreResult();
			//}
			termFlag = TerminationType();
		//}
	}

	cpuTime = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	//cout<<"\n\n\tSolution process terminated: "<<cpuTime<<endl;
	
	return termFlag;
}

void TAP_BCP::UpdatePathFlowLazy1(TNM_SORIGIN* pOrg,TNM_SDEST* dest)
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
									//PrintPath(path);
									system("PAUSE");

								}
								slink->cost = slink->GetCost()+slink->buffer[2];
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
					slink->cost = slink->GetCost()+slink->buffer[2];
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


void TAP_BCP::UpdateLinkCost1()
{

	TNM_SLINK *link;
	for(int i = 0;i<network->numOfLink;i++)
	{
		link = network->linkVector[i];
		link->cost = link->GetCost()+link->buffer[2];
	}

}

void TAP_BCP::UpdateLinkCostDer1()
{

	TNM_SLINK *link;
	for(int i = 0;i<network->numOfLink;i++)
	{
		link = network->linkVector[i];
		link->fdCost = link->GetDerCost();
	}

}



void TAP_BCP::PreProcess1()
{
	TNM_TAP::PreProcess();
	if(m_resetNetworkOnSolve) 
	{
		network->Reset();
	}
	
	

}

void TAP_BCP::PostProcess1()
{
	UpdateLinkCost1();
	//save the link volumn and paths
	TNM_SLINK* slink;
	for(int i=0;i<network->numOfLink;i++)
	{
		network->linkVector[i]->aux_volume1 = network->linkVector[i]->volume;
	}
	for(int oi=0; oi<network->numOfOrigin; oi++)
	{
		for(int di=0; di<network->originVector[oi]->numOfDest; di++)
		{
			//network->originVector[oi]->destVector[di]->aux_pathSet1.clear();
			network->originVector[oi]->destVector[di]->aux_pathSet1 = network->originVector[oi]->destVector[di]->pathSet;
			network->originVector[oi]->destVector[di]->pathSet.clear();
		}
	}
}

//The main procedures for the proposed CBCP algorithm, including Algorithm 2 and Algorithm 1

void TAP_BCP::AlgorithmBCPCC()
{

	m_startRunTime = clock();
	//initialize 
	network->AllocateNodeBuffer(2);
	//allocate new memory to save link related variables 
	network->AllocateLinkBuffer(6); //buffer[0] is the v_a variable; buffer[1] is the u_a variable; buffer[2] is the z_a variable; buffer[3] is the z variable in the PGBB algorithm;buffer[4] is the link travel time of the mainloop
	                                   ////buffer[5] is mark the linkset
	//Temporary variables for cal the convergence indicator
	vector<double> preVolume(network->numOfLink);
	vector<double> preZvariable(network->numOfLink);
	vector<double> preUvariable(network->numOfLink);
	double vConvergence = 0.0;
	double zConvergence = 0.0;
	double uConvergence = 0.0;
	double wholegap1 = 0.11;
	double wholegap2 = 0.11;
	numofAssignment1 =0;
	numofAssignment2 = 0;
	U = 10000;

	//IMPORTANT: set the cardinality constraints here
	kappa = 10;


	//seting initial z values
	for (int i = 0; i < network->numOfLink; i++)
	{
		network->linkVector[i]->buffer[2] = 1.200; // for small kappa value
		//network->linkVector[i]->buffer[2] = 0.00; //for large kappa value
	}
	
	//Begining of Algorithm 2(PBCD algorithm) 
	TNM_SLINK* slink;
	vector<TNM_SLINK*> order = network->linkVector;
	pd_gamma = 1.8; //Scale factor for myRow1, see gamma_1 in Algorithm 2 
	pd_gamma2 =5; // Scale factor for myRow2, see gamma_2 in Algorithm 2
	myRow1 = 1.0; //penalty for (f-V) gap
	myRow2 = 1.0;  //penality for ||z-u|| gap
	Upsilon = 1e10;
	bool outerloop = true;
	bool innerloop = true;
	int  outerIter =0;
	int  innerIter =0;
	currentups =0;
	

	//step 0 of Algorithm 2: to determine the Upsilon value
	 TERMFLAGS termFlag= SolveAssignment1(); //the path-based algorithm for solving the TAP Problem (4)
	 double iniOFV = OFV;
	 double Fv = 0;
	 //compute the total cost of the UE flow pattern
	 for (int i = 0; i < network->numOfLink; i++)
	 {
		 Fv+=network->linkVector[i]->cost*network->linkVector[i]->volume;
	 }

	 Upsilon = Fv;
	 //test
	 cout<<" ini OFV is "<<OFV<<endl;
	 cout<<"Fv is "<<Fv<<endl;
	 cout<<"kappa is "<<kappa<<endl;
	 //
	 termFlag=   SolveAssingment2(); // the path-based algorithm for solving Problem (16), see Appendix B
	 
	 numofAssignment2++;
	 double iniOFV2 = OFV;
	 if (Upsilon<iniOFV2)
	 {
		 Upsilon = iniOFV2;
	 }
	 //
	 cout<<"OFV IS "<<OFV<<endl;
	 cout<<"The penalty objective is "<<iniOFV2<<endl;
	 cout<<"Upsilon is "<<Upsilon<<endl;
	// system("PAUSE");
	//the main loop
	do
	{
		outerIter++;
		//=========step 1 of Algorithm2 : call Algorithm 1 to solve the PA problem (10)
		innerIter=0;
		do
		{
			innerIter++;
			
			//step 1 of Algoirthm 1: determine the u variables according to z

		    QuickSort(order,0,network->numOfLink-1); //sort the order according to the z value from smallest to the largest
	
			if (kappa<=network->numOfLink)
			{
				for (int i = network->numOfLink-1; i >= 0; i--)
				{
						 //
						 //cout<<"i is "<<i<<endl;
				    if (i>=network->numOfLink-kappa)
					{
						order[i]->buffer[1] = order[i]->buffer[2];
					}
					else
					{
						order[i]->buffer[1] = 0.0;
					}
				 }
			}

		
			// solve Problem (16)
		    SolveAssingment22();

			double ofv2 = OFV;
			numofAssignment2++;
			//
		    /// step 2 of Algorithm 1: check the convergence

			double obj12 =OFV;
			TERMFLAGS termFlag= SolveAssignment12();
			
			numofAssignment1++;
			obj12 = obj12 - myRow1* OFV;
			double uzgap12 =0;
			//
			vConvergence =0.0;
			zConvergence =0.0;
			uConvergence =0.0;
			double totalv=0.0;
			double totalz =0.0;
			double totalu =0.0;
			for (int i = 0; i < network->numOfLink; i++)
			{
				slink = network->linkVector[i];
				//
				//cout<<"prevolume "<<i<<" is "<< preVolume[i]<<"  ;  buffer[0] is  "<<slink->buffer[0]<<endl;
				//
				vConvergence+=abs(preVolume[i] - slink->buffer[0]);
				zConvergence+=abs(preZvariable[i] - slink->buffer[2]);
				uConvergence +=abs(preUvariable[i] - slink->buffer[1]);
				//
				totalv+=slink->buffer[0];
				totalz+=slink->buffer[2];
				totalu+=slink->buffer[1];
				//
				preVolume[i] = slink->buffer[0];
				preZvariable[i] = slink->buffer[2];
				preUvariable[i] = slink->buffer[1];

				//
				uzgap12+=(slink->buffer[2]-slink->buffer[1])*(slink->buffer[2]-slink->buffer[1]);
			}
			//
			if(vConvergence<1e-6 && zConvergence<1e-6 && uConvergence<1e-6)       //((vConvergence<1 && zConvergence<0.01 && uConvergence<0.01) || (innerIter>5 && outerIter<3)|| (innerIter>20 && outerIter>=3)  ) // || (innerIter>5 && outerIter<5)|| (innerIter>20 && outerIter>=5)    ||  (innerIter>20 && outerIter>=0)
			{
				innerloop =false;
				
				
			}
			else
			{
				innerloop = true;

				//Step 3 of Algorithm 1: cal the z value
		        AlgorithmPGBB(); //Algorithm 3 for solving Problem (17)
			}
			
		} while (innerloop && innerIter<25);
	   //==============step 2 of Algorithm 2:  check convergence
		double uzgap =0.0;
		double olduzgap =0.0;
		double totalu =0.0;
		for (int i = 0; i < network->numOfLink; i++)
		{
			network->linkVector[i]->volume = network->linkVector[i]->buffer[0];
			uzgap+= abs(network->linkVector[i]->buffer[1]-network->linkVector[i]->buffer[2]); //*(network->linkVector[i]->buffer[1]-network->linkVector[i]->buffer[2]
			totalu+=network->linkVector[i]->buffer[1];
		}
		double currentOFV = ComputeOFV1();
		TERMFLAGS termFlag= SolveAssignment12();
		double newOFV = OFV;
		if (totalu>1.0)
		{
			uzgap = uzgap/totalu;
		}
		///
		cout<<"current OFV is "<<currentOFV<<"  new OFV is "<<newOFV<<" wholegap1 is "<<wholegap1<<"  proporton is "<<abs(currentOFV-newOFV)/wholegap1<<endl;
		cout<<"uzgap is "<<uzgap<<"  olduzgap is "<<wholegap2<<"  proportion is "<<abs(uzgap)/wholegap2<<endl;
		//
		if(abs(currentOFV-newOFV)/currentOFV<1e-4 && uzgap<0.001)
		{
			outerloop = false;
		}
		else 
		{
			if( abs(currentOFV-newOFV)/wholegap1>0.80 && abs(currentOFV-newOFV)/currentOFV>1e-4 && myRow1<80)  //abs(currentOFV-newOFV)>0.8*wholegap1 &&  && myRow1<8
			{
				myRow1 = myRow1*pd_gamma;
			}

			if ( abs(uzgap)/wholegap2> 0.80 && uzgap> 0.001 && myRow2<1e8 ) //uzgap>0.5*wholegap2 &&  && myRow2<1000
			{
				myRow2 = myRow2*pd_gamma2;
			}
			
			outerloop = true;
		}


		double totalCost = ComputeTotalCost();
		wholegap1 = abs(currentOFV-newOFV);
		wholegap2 = uzgap;
		olduzgap = uzgap;


		int numnz =0;
		cout<<"us is "<<endl;
		for (int i = 0; i < network->numOfLink; i++)
		{
			
			//cout<<i<<"  z is "<<network->linkVector[i]->buffer[2]<<endl;
			if (network->linkVector[i]->buffer[1]>0)
			{
				cout<<i<<"  u is "<<network->linkVector[i]->buffer[1]<<endl;
			}
			
		}
		cout<<"the num of nonzero z is "<<numnz<<endl;


		//cal the current objective value
		        SolveAssingment22();
				currentups = OFV;
				cout<<"OFV for assignment 2 is "<<OFV<<endl;


		////=================setp 3 of Algorithm 2: update z value
		//currentups=currentups;
		if (currentups>Upsilon)
		{
			for (int i = 0; i < network->numOfLink; i++)
			{
				network->linkVector[i]->buffer[2] = network->linkVector[i]->buffer[1]; //seting the u_fea
			}
		}
		
		//output some useful information to observe the convergence of the Algorithm
		cout<<"///////////////////////The current outter iteration  of PD is "<<outerIter<<endl;
		cout<<"The current OFV is "<<currentOFV<<endl;
		cout<<"The new OFV is "<<newOFV<<endl;
		cout<<"The current Upsilon is "<<currentups<<endl;
		cout<<"Upsilon is "<<Upsilon<<endl;
		cout<<"The total cost of the system is "<<TNM_FloatFormat(totalCost) <<endl;
		cout<<"OFV GAP IS "<<abs(currentOFV-newOFV)/currentOFV<<endl;
		cout<<"Myrow1 is "<<myRow1<<endl;
		cout<<"Myrow2 is "<<myRow2<<endl;
		cout<<"uzgap is "<<uzgap<<endl;
		cout<<"total u is "<<totalu<<endl;
		cout<<"The current time is "<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
		cout<<"num of assignment 1 is "<<numofAssignment1<<endl;
		cout<<"nu of assignment 2 is "<<numofAssignment2<<endl;
		
		//
		
		//system("PAUSE");

	} while (outerloop);

	//Post Process

	//recalculate the total cost using the obtained solution
	for (int i = 0; i < network->numOfLink; i++)
	{
		network->linkVector[i]->buffer[2] = network->linkVector[i]->buffer[1];
	}
	SolveAssignment1();

	for (int i = 0; i < network->numOfLink; i++)
	{
		network->linkVector[i]->buffer[0] = network->linkVector[i]->volume;
	}

	double totalCost = ComputeTotalCost();

	cout<<"Total Cost is : "<<totalCost<<endl;

	// further improve the toll level precisions for the fixed tolled link set. 
	//This function can be commented out.
	PostTollLevelOptimizaiton();
}

//

void TAP_BCP::PostTollLevelOptimizaiton()
{
	///////
	cout<<"/////////////begin the post toll level optimization//////////////////////"<<endl;
	//variables for cal the convergence indicator
	vector<double> preVolume(network->numOfLink);
	vector<double> preZvariable(network->numOfLink);
	vector<double> preUvariable(network->numOfLink);
	double vConvergence = 0.0;
	double zConvergence = 0.0;
	double uConvergence = 0.0;
	double wholegap1 = 0.11;
	double wholegap2 = 0.11;
	numofAssignment1 =0;
	numofAssignment2 = 0;
	U = 1000;
	//kappa = 30;

	//int llset[10] = {11, 14, 24, 32, 35, 51, 52, 66, 70, 73};
	//seting initial z values
	int linkcount =0;
	for (int i = 0; i < network->numOfLink; i++)
	{
		network->linkVector[i]->buffer[2] = 0.0;
		network->linkVector[i]->buffer[5] = 0.0;
		
		if (network->linkVector[i]->buffer[1]>0)
		{
			network->linkVector[i]->buffer[5] = 1.0;
			network->linkVector[i]->buffer[2] = network->linkVector[i]->buffer[1];
			linkcount++;
		}
		
		//cout<<" i= "<<i<<" "<<network->linkVector[i]->tail->id<<"-->"<<network->linkVector[i]->head->id<<endl;
		//system("PAUSE");
	}
	//
	cout<<"link count is : "<<linkcount<<endl;
	//Penalty Decomposion problem 
	TNM_SLINK* slink;
	vector<TNM_SLINK*> order = network->linkVector;
	pd_gamma = 4.0;
	pd_gamma2 = 4.0;
	myRow1 = 1.0; //
	myRow2 = 1.0;  //
	Upsilon = 1e10;
	bool outerloop = true;
	bool innerloop = true;
	int  outerIter =0;
	int  innerIter =0;
	currentups =0;
	
	//PD algorithm
	//step 0 : to determine the Upsilon value
	 TERMFLAGS termFlag= SolveAssignment1();
	 double iniOFV = OFV;
	 double Fv = 0;
	 for (int i = 0; i < network->numOfLink; i++)
	 {
		 Fv+=network->linkVector[i]->cost*network->linkVector[i]->volume;
	 }

	 Upsilon = Fv;
	 //test
	 cout<<" ini OFV is "<<OFV<<endl;
	 cout<<"Fv is "<<Fv<<endl;
	 cout<<"kappa is "<<kappa<<endl;
	 //
	 termFlag=   SolveAssingment2();
	 
	 numofAssignment2++;
	 double iniOFV2 = OFV;
	 if (Upsilon<iniOFV2)
	 {
		 Upsilon = iniOFV2;
	 }
	 //
	 cout<<"OFV IS "<<OFV<<endl;
	 cout<<"The penalty objective is "<<iniOFV2<<endl;
	 cout<<"Upsilon is "<<Upsilon<<endl;
	// system("PAUSE");
	//the main loop
	do
	{
		outerIter++;
		//=========step 1 of PD algorithm
		////BCP algorithm for the PA problem 
		innerIter=0;
		do
		{
			innerIter++;
			//cout<<"======The current inner iteration is "<<innerIter<<endl;
			for (int i = 0; i < network->numOfLink; i++)
			{
				if (network->linkVector[i]->buffer[5]==1.0)
				{
					network->linkVector[i]->buffer[1] = network->linkVector[i]->buffer[2];
				}
				else
				{
					network->linkVector[i]->buffer[1] = 0.0;
					network->linkVector[i]->buffer[2] = 0.0;
				}
			}
		    //solve the assignment problem 2 to determine the link volume

		    SolveAssingment22();

			//cout<<"end of assingment 22"<<endl;
			//system("PAUSE");

			double ofv2 = OFV;
			numofAssignment2++;
			//
			//cout<<"assignment 2 is finished"<<endl;
		
		    ///check the convergence step 2
			//check the objective of formulation (12)
			double obj12 =OFV;
			TERMFLAGS termFlag= SolveAssignment12();
			numofAssignment1++;
			obj12 = obj12 - myRow1* OFV;
			double uzgap12 =0;
			//
			vConvergence =0.0;
			zConvergence =0.0;
			uConvergence =0.0;
			double totalv=0.0;
			double totalz =0.0;
			double totalu =0.0;
			for (int i = 0; i < network->numOfLink; i++)
			{
				slink = network->linkVector[i];
				//
				//cout<<"prevolume "<<i<<" is "<< preVolume[i]<<"  ;  buffer[0] is  "<<slink->buffer[0]<<endl;
				//
				vConvergence+=abs(preVolume[i] - slink->buffer[0]);
				zConvergence+=abs(preZvariable[i] - slink->buffer[2]);
				uConvergence +=abs(preUvariable[i] - slink->buffer[1]);
				//
				totalv+=slink->buffer[0];
				totalz+=slink->buffer[2];
				totalu+=slink->buffer[1];
				//
				preVolume[i] = slink->buffer[0];
				preZvariable[i] = slink->buffer[2];
				preUvariable[i] = slink->buffer[1];

				//
				uzgap12+=(slink->buffer[2]-slink->buffer[1])*(slink->buffer[2]-slink->buffer[1]);
			}
	

			//
			if(vConvergence<1e-6 && zConvergence<1e-6 && uConvergence<1e-6)       //((vConvergence<1 && zConvergence<0.01 && uConvergence<0.01) || (innerIter>5 && outerIter<3)|| (innerIter>20 && outerIter>=3)  ) // || (innerIter>5 && outerIter<5)|| (innerIter>20 && outerIter>=5)    ||  (innerIter>20 && outerIter>=0)
			{
				innerloop =false;
				//cal the current objective value
				currentups = OFV;
				cout<<"OFV for assingment 1 is "<<OFV<<endl;
				//TERMFLAGS termFlag= SolveAssingment2();
				cout<<"OFV for assignment 2 is "<<ofv2<<endl;
				currentups = currentups - myRow1*OFV;
				cout<<"current ups without uzgap is "<<currentups<<endl;
				
			}
			else
			{
				innerloop = true;

				//Step 3: cal the z value
		        FixedAlgorithmPGBB();

			    //cout<<"End of Algorithm PGBB "<<endl;
			    //system("PAUSE");
			}
		    //system("PAUSE");

		} while (innerloop && innerIter<50);
	   //==============step 2 of PD algorithm  check convergence
		double uzgap =0.0;
		double olduzgap =0.0;
		double totalu =0.0;
		for (int i = 0; i < network->numOfLink; i++)
		{
			network->linkVector[i]->volume = network->linkVector[i]->buffer[0];
			uzgap+= abs(network->linkVector[i]->buffer[1]-network->linkVector[i]->buffer[2]); //*(network->linkVector[i]->buffer[1]-network->linkVector[i]->buffer[2]
			totalu+=network->linkVector[i]->buffer[1];
		}
		double currentOFV = ComputeOFV1();
		TERMFLAGS termFlag= SolveAssignment12();
		double newOFV = OFV;
		//uzgap = uzgap/totalu;
		uzgap =0.0;
		///
		cout<<"current OFV is "<<currentOFV<<"  new OFV is "<<newOFV<<" wholegap1 is "<<wholegap1<<"  proporton is "<<abs(currentOFV-newOFV)/wholegap1<<endl;
		cout<<"uzgap is "<<uzgap<<"  olduzgap is "<<wholegap2<<"  proportion is "<<abs(uzgap)/wholegap2<<endl;
		//
		if(abs(currentOFV-newOFV)/currentOFV<1e-5 && uzgap<0.001)
		{
			outerloop = false;
		}
		else 
		{
			if( abs(currentOFV-newOFV)/wholegap1>0.9 && abs(currentOFV-newOFV)/currentOFV>1e-5 )  //abs(currentOFV-newOFV)>0.8*wholegap1 &&  && myRow1<8
			{
				myRow1 = myRow1*pd_gamma;
			}

			if ( abs(uzgap)/wholegap2> 0.8 && uzgap> 0.001 ) //uzgap>0.5*wholegap2 &&  && myRow2<1000
			{
				myRow2 = myRow2*pd_gamma2;
			}
			
			outerloop = true;
		}
		double totalCost = ComputeTotalCost();
		wholegap1 = abs(currentOFV-newOFV);
		wholegap2 = uzgap;
		olduzgap = uzgap;


		int numnz =0;
		cout<<"us is "<<endl;
		for (int i = 0; i < network->numOfLink; i++)
		{
			//cout<<i<<"  z is "<<network->linkVector[i]->buffer[2]<<endl;
			if (network->linkVector[i]->buffer[1]>0)
			{
				cout<<i<<"  u is "<<network->linkVector[i]->buffer[1]<<endl;
			}
			
		}
		cout<<"the num of nonzero z is "<<numnz<<endl;

		/*for (int i = 0; i < network->numOfLink; i++)
		{
			cout<<"link id " <<network->linkVector[i]->tail->id<<"-->"<<network->linkVector[i]->head->id<<"  : u is "<<network->linkVector[i]->buffer[1]<< " ; z is   "<< network->linkVector[i]->buffer[2] <<endl;
			
		}*/

		////=================setp 3 of PD algorithm update z value
			   SolveAssingment22();
				currentups = OFV;
				cout<<"OFV for assignment 2 is "<<OFV<<endl;
		if (currentups>Upsilon)
		{
			for (int i = 0; i < network->numOfLink; i++)
			{
				network->linkVector[i]->buffer[2] = network->linkVector[i]->buffer[1]; //seting the u_fea
			}
		}
		
		
		cout<<"///////////////////////The current outter iteration  of PD is "<<outerIter<<endl;
		cout<<"The current OFV is "<<currentOFV<<endl;
		cout<<"The new OFV is "<<newOFV<<endl;
		cout<<"The current Upsilon is "<<currentups<<endl;
		cout<<"Upsilon is "<<Upsilon<<endl;
		cout<<"The total cost of the system is "<<TNM_FloatFormat(totalCost) <<endl;
		cout<<"OFV GAP IS "<<abs(currentOFV-newOFV)/currentOFV<<endl;
		cout<<"Myrow1 is "<<myRow1<<endl;
		cout<<"Myrow2 is "<<myRow2<<endl;
		cout<<"uzgap is "<<uzgap<<endl;
		cout<<"total u is "<<totalu<<endl;
		cout<<"The current time is "<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
		cout<<"num of assignment 1 is "<<numofAssignment1<<endl;
		cout<<"nu of assignment 2 is "<<numofAssignment2<<endl;
		


	} while (outerloop);  //outerloop

	//
	for (int i = 0; i < network->numOfLink; i++)
	{
		network->linkVector[i]->buffer[2] = network->linkVector[i]->buffer[1];
	}
	SolveAssignment1();

	for (int i = 0; i < network->numOfLink; i++)
	{
		network->linkVector[i]->buffer[0] = network->linkVector[i]->volume;
	}

	double totalCost = ComputeTotalCost();

	cout<<"========================final solution========================="<<endl;
	cout<<"Total Cost is : "<<totalCost<<endl;
}


// the Algorithm for the fixed tolled link set

void TAP_BCP::FixedAlgorithmBCPCC()
{

	m_startRunTime = clock();
	//initialize 
	network->AllocateNodeBuffer(2);
	network->AllocateLinkBuffer(6); //buffer[0] is the v_a variable; buffer[1] is the u_a variable; buffer[2] is the z_a variable; buffer[3] is the z variable in the PGBB algorithm;buffer[4] is the link travel time of the mainloop
	                                 //buffer[5] is mark the linkset
	//variables for cal the convergence indicator
	vector<double> preVolume(network->numOfLink);
	vector<double> preZvariable(network->numOfLink);
	vector<double> preUvariable(network->numOfLink);
	double vConvergence = 0.0;
	double zConvergence = 0.0;
	double uConvergence = 0.0;
	double wholegap1 = 0.11;
	double wholegap2 = 0.11;
	numofAssignment1 =0;
	numofAssignment2 = 0;
	U = 1000;
	kappa = 30;

	int llset[10] = {11, 14, 24, 32, 35, 51, 52, 66, 70, 73};
	//seting initial z values
	for (int i = 0; i < network->numOfLink; i++)
	{
		network->linkVector[i]->buffer[2] = 0.0;
		network->linkVector[i]->buffer[5] = 0.0;
		for (int j = 0; j < 10; j++)
		{
			if (llset[j] == i)
			{
				network->linkVector[i]->buffer[5] = 1.0;
			}
		}
		//cout<<" i= "<<i<<" "<<network->linkVector[i]->tail->id<<"-->"<<network->linkVector[i]->head->id<<endl;
		//system("PAUSE");
	}
	//Penalty Decomposion problem 
	TNM_SLINK* slink;
	vector<TNM_SLINK*> order = network->linkVector;
	pd_gamma = 4.0;
	pd_gamma2 = 4.0;
	myRow1 = 1.0; //
	myRow2 = 1.0;  //
	Upsilon = 1e10;
	bool outerloop = true;
	bool innerloop = true;
	int  outerIter =0;
	int  innerIter =0;
	currentups =0;
	
	//PD algorithm
	//step 0 : to determine the Upsilon value
	 TERMFLAGS termFlag= SolveAssignment1();
	 double iniOFV = OFV;
	 double Fv = 0;
	 for (int i = 0; i < network->numOfLink; i++)
	 {
		 Fv+=network->linkVector[i]->cost*network->linkVector[i]->volume;
	 }

	 Upsilon = Fv;
	 //test
	 cout<<" ini OFV is "<<OFV<<endl;
	 cout<<"Fv is "<<Fv<<endl;
	 cout<<"kappa is "<<kappa<<endl;
	 //
	 termFlag=   SolveAssingment2();
	 
	 numofAssignment2++;
	 double iniOFV2 = OFV;
	 if (Upsilon<iniOFV2)
	 {
		 Upsilon = iniOFV2;
	 }
	 //
	 cout<<"OFV IS "<<OFV<<endl;
	 cout<<"The penalty objective is "<<iniOFV2<<endl;
	 cout<<"Upsilon is "<<Upsilon<<endl;
	// system("PAUSE");
	//the main loop
	do
	{
		outerIter++;
		//=========step 1 of PD algorithm
		////BCP algorithm for the PA problem 
		innerIter=0;
		do
		{
			innerIter++;
			//cout<<"======The current inner iteration is "<<innerIter<<endl;
			for (int i = 0; i < network->numOfLink; i++)
			{
				if (network->linkVector[i]->buffer[5]==1.0)
				{
					network->linkVector[i]->buffer[1] = network->linkVector[i]->buffer[2];
				}
				else
				{
					network->linkVector[i]->buffer[1] = 0.0;
					network->linkVector[i]->buffer[2] = 0.0;
				}
			}
		    //solve the assignment problem 2 to determine the link volume

		    SolveAssingment22();

			//cout<<"end of assingment 22"<<endl;
			//system("PAUSE");

			double ofv2 = OFV;
			numofAssignment2++;
			//
			//cout<<"assignment 2 is finished"<<endl;
			//cout<<"the v values are "<<endl;
			//for (int i = 0; i < network->numOfLink; i++)
			//{
			//	cout<<"link "<<i+1<<" 's volume is "<<network->linkVector[i]->buffer[0]<<endl;

			//}
			////system("PAUSE");
			//cout<<"the u values are "<<endl;
			//for (int i = 0; i < network->numOfLink; i++)
			//{
			//	cout<<"link "<<i+1<<" 's u value is "<<network->linkVector[i]->buffer[1]<<endl;

			//}
			//system("PAUSE");
		    ///check the convergence step 2
			//check the objective of formulation (12)
			double obj12 =OFV;
			TERMFLAGS termFlag= SolveAssignment12();
			numofAssignment1++;
			obj12 = obj12 - myRow1* OFV;
			double uzgap12 =0;
			//
			vConvergence =0.0;
			zConvergence =0.0;
			uConvergence =0.0;
			double totalv=0.0;
			double totalz =0.0;
			double totalu =0.0;
			for (int i = 0; i < network->numOfLink; i++)
			{
				slink = network->linkVector[i];
				//
				//cout<<"prevolume "<<i<<" is "<< preVolume[i]<<"  ;  buffer[0] is  "<<slink->buffer[0]<<endl;
				//
				vConvergence+=abs(preVolume[i] - slink->buffer[0]);
				zConvergence+=abs(preZvariable[i] - slink->buffer[2]);
				uConvergence +=abs(preUvariable[i] - slink->buffer[1]);
				//
				totalv+=slink->buffer[0];
				totalz+=slink->buffer[2];
				totalu+=slink->buffer[1];
				//
				preVolume[i] = slink->buffer[0];
				preZvariable[i] = slink->buffer[2];
				preUvariable[i] = slink->buffer[1];

				//
				uzgap12+=(slink->buffer[2]-slink->buffer[1])*(slink->buffer[2]-slink->buffer[1]);
			}
			//vConvergence = vConvergence/totalv;
			//uConvergence = uConvergence/totalu;
			//zConvergence = zConvergence/totalz;
			//test
			//cout<<"the inneriter is "<<innerIter<<endl;
			//cout<<"the v convergence is "<<vConvergence<<endl;
			//cout<<"The u convergence is "<<uConvergence<<endl;
			//cout<<"The z convergence is "<<zConvergence<<endl;
			//cout<<"The objective function of 12 is "<<obj12+myRow2*uzgap12<<endl;

			//
			if(vConvergence<1e-6 && zConvergence<1e-6 && uConvergence<1e-6)       //((vConvergence<1 && zConvergence<0.01 && uConvergence<0.01) || (innerIter>5 && outerIter<3)|| (innerIter>20 && outerIter>=3)  ) // || (innerIter>5 && outerIter<5)|| (innerIter>20 && outerIter>=5)    ||  (innerIter>20 && outerIter>=0)
			{
				innerloop =false;
				//cal the current objective value
				currentups = OFV;
				cout<<"OFV for assingment 1 is "<<OFV<<endl;
				//TERMFLAGS termFlag= SolveAssingment2();
				cout<<"OFV for assignment 2 is "<<ofv2<<endl;
				currentups = currentups - myRow1*OFV;
				cout<<"current ups without uzgap is "<<currentups<<endl;
				
			}
			else
			{
				innerloop = true;

				//Step 3: cal the z value
		        FixedAlgorithmPGBB();

			    //cout<<"End of Algorithm PGBB "<<endl;
			    //system("PAUSE");
			}
		    //system("PAUSE");

		} while (innerloop && innerIter<50);
	   //==============step 2 of PD algorithm  check convergence
		double uzgap =0.0;
		double olduzgap =0.0;
		double totalu =0.0;
		for (int i = 0; i < network->numOfLink; i++)
		{
			network->linkVector[i]->volume = network->linkVector[i]->buffer[0];
			uzgap+= abs(network->linkVector[i]->buffer[1]-network->linkVector[i]->buffer[2]); //*(network->linkVector[i]->buffer[1]-network->linkVector[i]->buffer[2]
			totalu+=network->linkVector[i]->buffer[1];
		}
		double currentOFV = ComputeOFV1();
		TERMFLAGS termFlag= SolveAssignment12();
		double newOFV = OFV;
		//uzgap = uzgap/totalu;
		uzgap =0.0;
		///
		cout<<"current OFV is "<<currentOFV<<"  new OFV is "<<newOFV<<" wholegap1 is "<<wholegap1<<"  proporton is "<<abs(currentOFV-newOFV)/wholegap1<<endl;
		cout<<"uzgap is "<<uzgap<<"  olduzgap is "<<wholegap2<<"  proportion is "<<abs(uzgap)/wholegap2<<endl;
		//
		if(abs(currentOFV-newOFV)/currentOFV<1e-5 && uzgap<0.001)
		{
			outerloop = false;
		}
		else 
		{
			if( abs(currentOFV-newOFV)/wholegap1>0.9 && abs(currentOFV-newOFV)/currentOFV>1e-5 )  //abs(currentOFV-newOFV)>0.8*wholegap1 &&  && myRow1<8
			{
				myRow1 = myRow1*pd_gamma;
			}

			if ( abs(uzgap)/wholegap2> 0.8 && uzgap> 0.001 ) //uzgap>0.5*wholegap2 &&  && myRow2<1000
			{
				myRow2 = myRow2*pd_gamma2;
			}
			
			outerloop = true;
		}
		double totalCost = ComputeTotalCost();
		wholegap1 = abs(currentOFV-newOFV);
		wholegap2 = uzgap;
		olduzgap = uzgap;


		int numnz =0;
		cout<<"us is "<<endl;
		for (int i = 0; i < network->numOfLink; i++)
		{
			//cout<<i<<"  z is "<<network->linkVector[i]->buffer[2]<<endl;
			if (network->linkVector[i]->buffer[1]>0)
			{
				cout<<i<<"  u is "<<network->linkVector[i]->buffer[1]<<endl;
			}
			
		}
		cout<<"the num of nonzero z is "<<numnz<<endl;

		/*for (int i = 0; i < network->numOfLink; i++)
		{
			cout<<"link id " <<network->linkVector[i]->tail->id<<"-->"<<network->linkVector[i]->head->id<<"  : u is "<<network->linkVector[i]->buffer[1]<< " ; z is   "<< network->linkVector[i]->buffer[2] <<endl;
			
		}*/

		////=================setp 3 of PD algorithm update z value
			   SolveAssingment22();
				currentups = OFV;
				cout<<"OFV for assignment 2 is "<<OFV<<endl;
		if (currentups>Upsilon)
		{
			for (int i = 0; i < network->numOfLink; i++)
			{
				network->linkVector[i]->buffer[2] = network->linkVector[i]->buffer[1]; //seting the u_fea
			}
		}
		
		
		cout<<"///////////////////////The current outter iteration  of PD is "<<outerIter<<endl;
		cout<<"The current OFV is "<<currentOFV<<endl;
		cout<<"The new OFV is "<<newOFV<<endl;
		cout<<"The current Upsilon is "<<currentups<<endl;
		cout<<"Upsilon is "<<Upsilon<<endl;
		cout<<"The total cost of the system is "<<TNM_FloatFormat(totalCost) <<endl;
		cout<<"OFV GAP IS "<<abs(currentOFV-newOFV)/currentOFV<<endl;
		cout<<"Myrow1 is "<<myRow1<<endl;
		cout<<"Myrow2 is "<<myRow2<<endl;
		cout<<"uzgap is "<<uzgap<<endl;
		cout<<"total u is "<<totalu<<endl;
		cout<<"The current time is "<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
		cout<<"num of assignment 1 is "<<numofAssignment1<<endl;
		cout<<"nu of assignment 2 is "<<numofAssignment2<<endl;
		
		//record the outIterinformation
		//ITERELEM *iterElem;
	    //iterElem = new ITERELEM;
		//iterElem->iter  = outerIter;
		//iterElem->ofv   = totalCost;
	    //iterElem->time  = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	    //outIterRecord.push_back(iterElem); 
	    
		//system("PAUSE");

	} while (outerloop);  //outerloop

	//
	for (int i = 0; i < network->numOfLink; i++)
	{
		network->linkVector[i]->buffer[2] = network->linkVector[i]->buffer[1];
	}
	SolveAssignment1();

	for (int i = 0; i < network->numOfLink; i++)
	{
		network->linkVector[i]->buffer[0] = network->linkVector[i]->volume;
	}

	double totalCost = ComputeTotalCost();

	cout<<"========================final solution========================="<<endl;
	cout<<"Total Cost is : "<<totalCost<<endl;

}
//

void TAP_BCP::FixedAlgorithmPGBB()
{

	//
	//cout<<"===============The beginning of the PGBB algorithm "<<endl;
	//set parameters
	a_min = 1e-12;
	a_max = 1;
	a_stepsize = 0.0001;
	eta =0.1;
	double row = 0.01;
	double zgap = 0.0;

	double tau = a_stepsize;
	double S_k = 0.0;
	double S_k1 = 0.0;
	double g_k =0.0;
	double g_k1 = 0.0;
	double g2_k = 0.0;
	double g2_k1 = 0.0;
	double zu_k =0.0;
	double zu_k1 =0.0;
	double dgzz =0.0;
	int Oiter=0;
	int iiter=0;
	//ini the variables
	vector<double> v_k(network->numOfLink);
	vector<double> v_k1(network->numOfLink);
	vector<double> z_k(network->numOfLink); // z value at the k iteration
	vector<double> z_k1(network->numOfLink); // z value at the k+1 iteration
	vector<double> s_k(network->numOfLink); 
	vector<double> y_k(network->numOfLink);
	vector<double> dg_k(network->numOfLink);
	vector<double> dg_k1(network->numOfLink);
	TNM_SLINK* slink;

	//for (int i = 0; i < network->numOfLink; i++)
	//{
	//	network->linkVector[i]->buffer[2] = 0.0; // z variables
	//	
	//}

	//compute the S(z) 
	//
	//cout<<"Solving the z values using projected gradient algorithm with BB stepsize"<<endl;
	//  cout<<"the first iteration of assignment 1"<<endl;
	    TERMFLAGS termFlag= SolveAssignment12();
		numofAssignment1++;


		//system("PAUSE");
		S_k = OFV;
		for (int jj = 0; jj < network->numOfLink; jj++)
		{
			slink = network->linkVector[jj];
			z_k[jj] = slink->buffer[2];
			v_k[jj] = slink->volume;
		}
	//
	

	do
	{
		//step 2 of Algoeirthm 3
		Oiter++;
		

		tau = a_stepsize;
		bool bbcon = false;
		
		////compute the S(z) 
	 //   TERMFLAGS termFlag= SolveAssignment1();
		//S_k = OFV;
		//for (int jj = 0; jj < network->numOfLink; jj++)
		//{
		//	slink = network->linkVector[jj];
		//	z_k[jj] = slink->buffer[2];
		//	v_k[jj] = slink->volume;
		//}
		int innniter=0;
		do
		{
			innniter++;
			//cout<<"The inner loop iteration is "<<iiter<<endl;
			//cout<<"Tau is "<<tau<<endl;

			//update variables
			for (int j = 0; j < network->numOfLink; j++)
			{
				slink = network->linkVector[j];
				
				dg_k[j]= myRow1*(slink->buffer[0] - v_k[j]) + myRow2*(2*z_k[j] - 2*slink->buffer[1]); //compute the gradient of g function at the current z

				//dg_k[j]=  slink->fft*(1.0+ 0.15 * pow(slink->buffer[0]/slink->capacity,4.0)) -  slink->fft*(1.0+ 0.15 * pow(v_k[j]/slink->capacity,4.0)) + (2*z_k[j] - 2*slink->buffer[1]);

				z_k1[j] = z_k[j]-tau* dg_k[j];
				//test

				//cout<<"not projected z_k1 is "<<z_k1[j]<<endl;
				//i
				if (network->linkVector[j]->buffer[5] == 1.0)
				{
					if (z_k1[j]<0.0)
				   {
					    z_k1[j] = 0.0;

				    }
				    else if (z_k1[j] > U)
				   {
					   z_k1[j] = U;
				   }

				}
				else
				{
					z_k1[j] =0.0;
				}
			
				

				//set buffer[2] to z_k1
				slink->buffer[2] = z_k1[j];
				//test
				/*cout<<"dg_k is "<<dg_k[j]<<endl;
				cout<<"v-v* is "<<slink->buffer[0] - v_k[j]<<endl;
				cout<<"2z-zu is "<<(2*z_k[j] - 2*slink->buffer[1])<<endl;
				cout<<"tau is "<<tau<<endl;
				cout<<"projected z_k1 is "<<z_k1[j]<<endl;*/
				//system("PAUSE");
			}
			////check bb conditions
			//compute the S(z+1)
			//if (innniter ==1)
			{
				//cout<<"////////////////////////////////begining of new assingment 12 ///////////////////////////"<<endl;
				TERMFLAGS termFlag= SolveAssignment12();
				numofAssignment1++;
				//cout<<"This is the end of assignment 1 without reset the network"<<endl;
				//system("PAUSE");
			}
			
			S_k1 = OFV;
			g_k = 0.0; g_k1 =0.0;
			g2_k =0.0; g2_k1=0.0;
			zu_k =0.0;
			zu_k1 =0.0;
			dgzz =0.0;
			for (int j = 0; j < network->numOfLink; j++)
			{
				slink = network->linkVector[j];
				
				v_k1[j] = slink->volume;

				//fft*(1.0 + alpha * pow(volume/capacity,beta));

				dg_k1[j]= myRow1*(slink->buffer[0] - v_k1[j])+ myRow2*(2*z_k1[j] - 2*slink->buffer[1]);

				//dg_k1[j]= slink->fft*(1.0+ 0.15 * pow(slink->buffer[0]/slink->capacity,4.0)) -  slink->fft*(1.0+ 0.15 * pow(v_k1[j]/slink->capacity,4.0)) + (2*z_k1[j] - 2*slink->buffer[1]);

				//intcost = slink->fft*slink->volume *(1.0 + 0.15 * pow(slink->volume/slink->capacity, 4.0)/(4.0+1.0)) + slink->volume*slink->buffer[2];
			    //buffer[4] is the link cost of the current mainloop, buffer[0] is the link volume of the current mainloop; buffer[2] is the z_k; buffer[1] is u_a
				g_k += myRow1* slink->fft*slink->buffer[0] *(1.0 + slink->bpr_alpha * pow(slink->buffer[0]/slink->capacity, slink->bpr_beta)/(slink->bpr_beta+1)) + myRow1* slink->buffer[0]*z_k[j] + myRow2*(z_k[j]-slink->buffer[1])*(z_k[j]-slink->buffer[1]);
				g_k1 += myRow1*slink->fft*slink->buffer[0] *(1.0 + slink->bpr_alpha * pow(slink->buffer[0]/slink->capacity, slink->bpr_beta)/(slink->bpr_beta+1)) + myRow1*slink->buffer[0]*z_k1[j]  + myRow2*(z_k1[j]-slink->buffer[1])*(z_k1[j]-slink->buffer[1]);
				g2_k += myRow1*slink->fft*slink->buffer[0] *(1.0 + slink->bpr_alpha * pow(slink->buffer[0]/slink->capacity, slink->bpr_beta)/(slink->bpr_beta+1)) + myRow1*slink->buffer[0]*z_k[j]; // for test output only
				g2_k1 += myRow1*slink->fft*slink->buffer[0] *(1.0 + slink->bpr_alpha * pow(slink->buffer[0]/slink->capacity, slink->bpr_beta)/(slink->bpr_beta+1)) + myRow1*slink->buffer[0]*z_k1[j]; // for test output only
				zu_k += myRow2*(z_k[j]-slink->buffer[1])*(z_k[j]-slink->buffer[1]); // for test output only
				zu_k1 +=  myRow2*(z_k1[j]-slink->buffer[1])*(z_k1[j]-slink->buffer[1]); // for test output only

				//
				s_k[j] = z_k1[j]-z_k[j];
				y_k[j] = dg_k1[j] - dg_k[j];
				
				//
				dgzz+=row*dg_k[j]*(z_k1[j]-z_k[j]);
				//
			}
			//cal the objective function of two z values
			g_k = g_k - myRow1* S_k;
			g_k1 =g_k1-myRow1* S_k1;
			
			//
			
			/*cout<<"g_k is "<<g_k<<endl;
			cout<<"g_k1 is "<<g_k1<<endl;
			cout<<"fzv at k iteration is "<<g2_k<<endl;
			cout<<"Vz at k iteration is "<<S_k<<endl;
			cout<<"fzv at k+1 iteration is "<<g2_k1<<endl;
			cout<<"Vz at k+1 iteration is "<<S_k1<<endl;
			cout<<"zu gap at k is "<<zu_k<<endl;
			cout<<"zu gap at k+1 is "<<zu_k1<<endl;
			cout<<"dgzz is "<<dgzz<<endl;
			cout<<"the current iteration is "<<innniter<<endl;
			cout<<"gk1 and g_k+dgzz is "<<g_k1<<"   vs  "<<g_k+dgzz<<endl;
			cout<<"tau is "<<tau<<endl;*/
			//system("PAUSE");
			//check the conditions
			if (g_k1<=g_k+dgzz)
			{
				//
				bbcon = false;
			}
			else
			{
				bbcon = true;
				tau = eta* tau;
				for (int i = 0; i < network->numOfLink; i++)
				{
					network->linkVector[i]->buffer[2] = z_k[i];
				}
			}

		} while (bbcon && innniter<7);
		
		//step 3 of the algorithm
		double ua =0.0;
		double la =0.0;
		zgap =0.0;
		for (int i = 0; i < network->numOfLink; i++)
		{
			slink = network->linkVector[i];
			la+=s_k[i]*y_k[i];
			ua+=s_k[i]*s_k[i];

			
			//
			zgap+=abs(z_k1[i]-z_k[i]);

			//update the z_k and v_k
			z_k[i] = z_k1[i];
			v_k[i] = v_k1[i];

		}
		//
		S_k = S_k1;

		//upate the stepsize
		if (la<=0)
		{
			a_stepsize = 1.0;
		}
		else 
		{
			a_stepsize = ua/la;
		}
		//
		/*for (int i = 0; i < network->numOfLink; i++)
		{
			cout<<" u " <<i<< " is "<<network->linkVector[i]->buffer[1]<<endl;
			cout<<" z " <<i<< " is "<<network->linkVector[i]->buffer[2]<<endl;
		}*/
		//cout<<"The outer iteration is PGBB is "<<Oiter<<endl;
		//cout<<"The inneriter is "<<innniter<<endl;
		//cout<<"zgap is "<<zgap<<endl;
		//cout<<"astepsize is "<<a_stepsize<<endl;
		//cout<<"The u value is "<<endl;
		
		//system("PAUSE");
		////compute the gap at the current iteration
		//for (int i = 0; i < network->numOfLink; i++)
		//{
		//	//
		//	double zz = z_k1[i]-dg_k1[i];

		//	if (zz<0.0)
		//	{
		//		zz = 0.0;

		//	}
		//	else if (zz > U)
		//	{
		//		zz = U;
		//	}
		//	//
		//	zgap+=abs(zz-z_k1[i]);
		//	
		//}

	}while(zgap>0.00001 && Oiter<10);

	//
	//cout<<"This is the end of the PGBB algorithm "<<endl;
	//cout<<"The z value is "<<endl;
	///*for (int i = 0; i < network->numOfLink; i++)
	//{
	//	cout<<"link "<<i+1<<" 's toll is "<<network->linkVector[i]->buffer[2]<<endl;
	//}*/
	//system("PAUSE");

}
//

double TAP_BCP::ComputeOFV1()
{

    double ofv = 0.0;
	TNM_SLINK* slink;
	for (int i = 0;i<network->numOfLink;i++)
	{
		slink = network->linkVector[i];

		double intcost =0.0;
		if (slink->volume>0)
		{
			//intcost = slink->fft*slink->volume *(1.0 + 0.15 * pow(slink->volume/slink->capacity, 4.0)/(4.0+1.0)) + slink->volume*slink->buffer[2];
			intcost = slink->fft*slink->volume*(1.0 + slink->bpr_alpha*pow(slink->volume/slink->capacity,slink->bpr_beta)/(slink->bpr_beta+1) ) + slink->volume*slink->buffer[2];
		}

		ofv += intcost;
		
	}
		
	return ofv;

}

double TAP_BCP::ComputeTotalCost()
{

    double ofv = 0.0;
	TNM_SLINK* slink;
	for (int i = 0;i<network->numOfLink;i++)
	{
		slink = network->linkVector[i];

		double intcost =0.0;
		if (slink->buffer[0]>0)
		{
			//intcost =   slink->fft*(1.0 + 0.15 * pow(slink->buffer[0]/slink->capacity,4.0))*slink->buffer[0];   //slink->fft*slink->volume *(1.0 + 0.15 * pow(slink->volume/slink->capacity, 4.0)/(4.0+1.0)) + slink->volume*slink->buffer[2];
			intcost =   slink->fft*(1.0 + slink->bpr_alpha * pow(slink->buffer[0]/slink->capacity,slink->bpr_beta))*slink->buffer[0];
		}

		ofv += intcost;
	}
		
	return ofv;

}

double TAP_BCP::ComputeOFV2()
{

    double ofv = 0.0;
	TNM_SLINK* slink;
	for (int i = 0;i<network->numOfLink;i++)
	{
		slink = network->linkVector[i];

		double intcost =0.0;
		double totalcost = 0.0;
		if (slink->volume>0)
		{
			//intcost = (slink->fft*slink->volume *(1.0 + 0.15 * pow(slink->volume/slink->capacity, 4.0)/(4.0+1.0)) + slink->volume*slink->buffer[2]);
			//totalcost = slink->fft*slink->volume *(1.0 + 0.15 * pow(slink->volume/slink->capacity, 4.0));

			intcost = (slink->fft*slink->volume *(1.0 + slink->bpr_alpha * pow(slink->volume/slink->capacity, slink->bpr_beta)/(slink->bpr_beta+1)) + slink->volume*slink->buffer[2]);
			totalcost = slink->fft*slink->volume *(1.0 + slink->bpr_alpha * pow(slink->volume/slink->capacity, slink->bpr_beta));
		}

		ofv +=  myRow1* intcost;
		ofv += totalcost;
	}
		
	return ofv;

}


//double TNM_TAP::ComputeBeckmannObj(bool toll)
//{
//    double ofv = 0.0;
///*	if(!network->firstBestToll)
//	{
//		for (int i = 0;i<network->numOfLink;i++) ofv += network->linkVector[i]->GetIntCost();
//	}
//	else
//	{
//		for (int i = 0;i<network->numOfLink;i++) ofv += network->linkVector[i]->GetIntCostWT();
//	}*/
//	for (int i = 0;i<network->numOfLink;i++) ofv += network->linkVector[i]->GetIntCost(toll);
//	return ofv;
//
//}


//methods for the Algorithm 3

//Implementation of Algorithm 3
void TAP_BCP::AlgorithmPGBB()
{

	//
	//cout<<"===============The beginning of Algorithm 3 "<<endl;
	//set parameters
	a_min = 1e-12;
	a_max = 1;
	a_stepsize = 0.0001;
	eta =0.1;
	double row = 0.01;
	double zgap = 0.0;

	double tau = a_stepsize;
	double S_k = 0.0;
	double S_k1 = 0.0;
	double g_k =0.0;
	double g_k1 = 0.0;
	double g2_k = 0.0;
	double g2_k1 = 0.0;
	double zu_k =0.0;
	double zu_k1 =0.0;
	double dgzz =0.0;
	int Oiter=0;
	int iiter=0;
	//ini the temporary variables
	vector<double> v_k(network->numOfLink);
	vector<double> v_k1(network->numOfLink);
	vector<double> z_k(network->numOfLink); // z value at the k iteration
	vector<double> z_k1(network->numOfLink); // z value at the k+1 iteration
	vector<double> s_k(network->numOfLink); 
	vector<double> y_k(network->numOfLink);
	vector<double> dg_k(network->numOfLink);
	vector<double> dg_k1(network->numOfLink);
	TNM_SLINK* slink;


	//
	//cout<<"Solving the z values using projected gradient algorithm with BB stepsize"<<endl;
	
	    TERMFLAGS termFlag= SolveAssignment12(); //the path-based algorithm for solving the TAP Problem (4) with warm start
		numofAssignment1++;


		//system("PAUSE");
		S_k = OFV;
		for (int jj = 0; jj < network->numOfLink; jj++)
		{
			slink = network->linkVector[jj];
			z_k[jj] = slink->buffer[2];
			v_k[jj] = slink->volume;
		}
	//
	

	do
	{
		//step 2 of Algoeirthm 3
		Oiter++;
		

		tau = a_stepsize;
		bool bbcon = false;
		
		////compute the S(z) 
		int innniter=0;
		do
		{
			innniter++;

			//update variables
			for (int j = 0; j < network->numOfLink; j++)
			{
				slink = network->linkVector[j];
				
				dg_k[j]= myRow1*(slink->buffer[0] - v_k[j]) + myRow2*(2*z_k[j] - 2*slink->buffer[1]); //compute the gradient of g function at the current z

				//dg_k[j]=  slink->fft*(1.0+ 0.15 * pow(slink->buffer[0]/slink->capacity,4.0)) -  slink->fft*(1.0+ 0.15 * pow(v_k[j]/slink->capacity,4.0)) + (2*z_k[j] - 2*slink->buffer[1]);

				z_k1[j] = z_k[j]-tau* dg_k[j];
				//test

				//cout<<"not projected z_k1 is "<<z_k1[j]<<endl;
				//
				if (z_k1[j]<0.0)
				{
					z_k1[j] = 0.0;

				}
				else if (z_k1[j] > U)
				{
					z_k1[j] = U;
				}

				//set buffer[2] to z_k1
				slink->buffer[2] = z_k1[j];
	
			}

				//cout<<"////////////////////////////////begining of new assingment 12 ///////////////////////////"<<endl;
				TERMFLAGS termFlag= SolveAssignment12();
				numofAssignment1++;
		

			
			S_k1 = OFV;
			g_k = 0.0; g_k1 =0.0;
			g2_k =0.0; g2_k1=0.0;
			zu_k =0.0;
			zu_k1 =0.0;
			dgzz =0.0;
			for (int j = 0; j < network->numOfLink; j++)
			{
				slink = network->linkVector[j];
				
				v_k1[j] = slink->volume;

				//fft*(1.0 + alpha * pow(volume/capacity,beta));

				dg_k1[j]= myRow1*(slink->buffer[0] - v_k1[j])+ myRow2*(2*z_k1[j] - 2*slink->buffer[1]);

				//dg_k1[j]= slink->fft*(1.0+ 0.15 * pow(slink->buffer[0]/slink->capacity,4.0)) -  slink->fft*(1.0+ 0.15 * pow(v_k1[j]/slink->capacity,4.0)) + (2*z_k1[j] - 2*slink->buffer[1]);

				//intcost = slink->fft*slink->volume *(1.0 + 0.15 * pow(slink->volume/slink->capacity, 4.0)/(4.0+1.0)) + slink->volume*slink->buffer[2];
			    //buffer[4] is the link cost of the current mainloop, buffer[0] is the link volume of the current mainloop; buffer[2] is the z_k; buffer[1] is u_a
				g_k += myRow1* slink->fft*slink->buffer[0] *(1.0 + slink->bpr_alpha * pow(slink->buffer[0]/slink->capacity, slink->bpr_beta)/(slink->bpr_beta+1)) + myRow1* slink->buffer[0]*z_k[j] + myRow2*(z_k[j]-slink->buffer[1])*(z_k[j]-slink->buffer[1]);
				g_k1 += myRow1*slink->fft*slink->buffer[0] *(1.0 + slink->bpr_alpha * pow(slink->buffer[0]/slink->capacity, slink->bpr_beta)/(slink->bpr_beta+1)) + myRow1*slink->buffer[0]*z_k1[j]  + myRow2*(z_k1[j]-slink->buffer[1])*(z_k1[j]-slink->buffer[1]);
				g2_k += myRow1*slink->fft*slink->buffer[0] *(1.0 + slink->bpr_alpha * pow(slink->buffer[0]/slink->capacity, slink->bpr_beta)/(slink->bpr_beta+1)) + myRow1*slink->buffer[0]*z_k[j]; // for test output only
				g2_k1 += myRow1*slink->fft*slink->buffer[0] *(1.0 + slink->bpr_alpha * pow(slink->buffer[0]/slink->capacity, slink->bpr_beta)/(slink->bpr_beta+1)) + myRow1*slink->buffer[0]*z_k1[j]; // for test output only
				zu_k += myRow2*(z_k[j]-slink->buffer[1])*(z_k[j]-slink->buffer[1]); // for test output only
				zu_k1 +=  myRow2*(z_k1[j]-slink->buffer[1])*(z_k1[j]-slink->buffer[1]); // for test output only

				//
				s_k[j] = z_k1[j]-z_k[j];
				y_k[j] = dg_k1[j] - dg_k[j];
				
				//
				dgzz+=row*dg_k[j]*(z_k1[j]-z_k[j]);
				//
			}
			//cal the objective function of two z values
			g_k = g_k - myRow1* S_k;
			g_k1 =g_k1-myRow1* S_k1;
			

			//check the conditions
			if (g_k1<=g_k+dgzz)
			{
				//
				bbcon = false;
			}
			else
			{
				bbcon = true;
				tau = eta* tau;
				for (int i = 0; i < network->numOfLink; i++)
				{
					network->linkVector[i]->buffer[2] = z_k[i];
				}
			}

		} while (bbcon && innniter<7);
		
		//step 3 of Algorithm 3
		double ua =0.0;
		double la =0.0;
		zgap =0.0;
		for (int i = 0; i < network->numOfLink; i++)
		{
			slink = network->linkVector[i];
			la+=s_k[i]*y_k[i];
			ua+=s_k[i]*s_k[i];

			
			//
			zgap+=abs(z_k1[i]-z_k[i]);

			//update the z_k and v_k
			z_k[i] = z_k1[i];
			v_k[i] = v_k1[i];

		}
		//
		S_k = S_k1;

		//upate the stepsize
		if (la<=0)
		{
			a_stepsize = 1.0;
		}
		else 
		{
			a_stepsize = ua/la;
		}
		

	}while(zgap>0.00001 && Oiter<10);



}


////////////quict sort for the z variables
void TAP_BCP::QuickSort(vector<TNM_SLINK*> &order, int low , int high)
{
	//cout<<"begin of sort"<<endl;
	//system("PAUSE");
	if( low < high)
	{
		TNM_SLINK * slink = order[low];
		int i = low;
		int j = high;
		while(i<j)
		{
			while ( (i < j) && (order[j]->buffer[2] >= slink->buffer[2]))
			{
				j=j-1;
			}
			order[i] = order[j];
			while( ( i<j ) && (order[i]->buffer[2] <= slink->buffer[2]))
			{
				i=i+1;
			}
			order[j] = order[i];
		}
		order[i] = slink;

		QuickSort(order,low,i-1);
		QuickSort(order,i+1,high);
	}
}