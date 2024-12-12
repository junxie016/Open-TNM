#include "stdafx.h"
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
	m_maxCPUTime	   = 3600;
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
	//cout<<"Begin to build net!"<<endl;
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
	case NETTAPAS:
		if(network->BuildTAPAS(true, lpf)!=0) 
		{
			cout<<"\tEncounter problems when building a network object!"<<endl;
			return 4;
		}
		break;
	case NETSZFAS:
		//cout<<"build NETSZFAS"<<endl;
		if(network->BuildSZFormatASII()!=0) 
		{
			cout<<"\tEncounter problems when building a network object!"<<endl;
			return 4;
		}
		break;
	case NETSZFMC:
		network->IsMultiClass=true;
		if(network->BuildSZFormatMCAS(lpf)!=0) 
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
	return ReachAccuracy() || ReachMaxIter() || ReachError() || ReachUser() || ReachMaxCPU();
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
	if(ReachMaxCPU())   return MaxCPUTerm;
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
			tt += (link->volume - link->beckFlow) * link->cost;
			//gap += link->volume * link->cost;
		}
		gap = tt;
		for(int i = 0;i<network->numOfOrigin;i++)
		{
			org = network->originVector[i];
			network->UpdateSP(org->origin);
			//tmd += org->m_tdmd;
			for(int j = 0;j<org->numOfDest;j++)
			{
				dest = org->destVector[j];
				if(dest->skip) continue;
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

int TNM_TAP::ReportSZFormat()
{
	intWidth           = TNM_IntFormat::GetWidth();
	floatWidth         = TNM_FloatFormat::GetWidth();
	if(reportIterHistory)
	{
		string iteFileName = inFileName + "log.csv"; //out file
		if (!TNM_OpenOutFile(iteFile, iteFileName))
		{
			cout<<"\n\tFail to Initialize an algorithm object: cannot open log.csv to write!"<<endl;
		}
		else
		{
			cout<<"\tWriting the iteration history into file log.csv..."<<endl;
			ReportIterSZ(iteFile);
		}
	}
	if(reportLinkDetail)
	{
		string lfpFileName  = inFileName + "link_result.csv";
		if (!TNM_OpenOutFile(lfpFile, lfpFileName))
		{
			cout<<"\n\tFail to Initialize an algorithm object: Cannot open link_result.csv to write!"<<endl;
		}
		else
		{
			cout<<"\tWriting link details into file link_result.csv..."<<endl;
			ReportLinkSZ(lfpFile); 
			
		}

	}
	if(reportPathDetail)
	{
		string pthFileName  = inFileName + "path_result.csv";
		if (!TNM_OpenOutFile(pthFile, pthFileName))
		{
			cout<<"\n\tFail to Initialize an algorithm object: Cannot open path_result.csv to write!"<<endl;
		}
		else
		{
			cout<<"\tWriting path details into file path_result.csv..."<<endl;
			ReportPathSZ(pthFile); 
			
		}

	}
	if(lfpFile.is_open()) lfpFile.close();
	if(pthFile.is_open()) pthFile.close();
	return 0;
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
	out<<"path id      "<<"origin       "<<" dest      "<<"     path cost    "<<"     path flow    "<<"   num of link    "<<"   links  "<<endl;
	for (int oi=0; oi<network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		for (int di=0; di< pOrg->numOfDest; di++)
		{
			TNM_SDEST* sdest = pOrg->destVector[di];

			if(!network->IsMultiClass)
			{
				for (int pi=0; pi<sdest->pathSet.size(); pi++)
				{
					TNM_SPATH* path = sdest->pathSet[pi];
					if (path->flow > 1e-4)
					{
						//output the path 
						pthid ++;
						//
						out<<TNM_IntFormat(pthid,4)<<TNM_IntFormat(pOrg->origin->id,4)<<TNM_IntFormat(sdest->dest->id,4)<<TNM_FloatFormat(path->cost,12,6)
							<<TNM_FloatFormat(path->flow,12,6)<<TNM_IntFormat(path->path.size(),4);
						for (int li=0; li<path->path.size();li++)
						{
							TNM_SLINK* link = path->path[li];
							out<<TNM_IntFormat(link->id,4);
						}
						out<<endl;
					}
				}
			}
			else
			{
				for(int ki=0;ki<network->numOfClass;ki++)
				{
					for (int pi=0; pi<sdest->mc_pathset[ki]->pathSet.size(); pi++)
					{
						TNM_SPATH* path = sdest->mc_pathset[ki]->pathSet[pi];
						if (path->flow > 1e-4)
						{
							//output the path 
							pthid ++;
							//
							if(ki==0)
								out<<"#";
							out<<TNM_IntFormat(pthid,4)<<TNM_IntFormat(pOrg->origin->id,4)<<TNM_IntFormat(sdest->dest->id,4)<<TNM_FloatFormat(path->cost,12,6)
								<<TNM_FloatFormat(path->flow,12,6)<<TNM_IntFormat(path->path.size(),4);
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
	}
}

void TNM_TAP::ReportPathSZ(ofstream &out)
{
	if(network->IsMultiClass)
	{
		out<<"from_node,to_node,path,mode,flow,num\n";
		for(int i=0;i<network->numOfOrigin;i++)
		{
			TNM_SORIGIN* orgn=network->originVector[i];
			for(int j=0;j<orgn->numOfDest;j++)
			{
				TNM_SDEST* dest=orgn->destVector[j];
				for(int k=0;k<dest->mc_pathset.size();k++)
				{
					for(int h=0;h<dest->mc_pathset[k]->pathSet.size();h++)
					{
						TNM_SPATH* path=dest->mc_pathset[k]->pathSet[h];
						char res[200];
						sprintf(res,"%d,%d,",orgn->origin->id,dest->dest->id);
						string strr=res;
						cout<<strr;
						floatType co=0.0;
						for(int a=path->path.size()-1;a>=0;a--)
						{
							TNM_SLINK* link=path->path[a];
							sprintf(res,"%d",link->id);
							strr+=res;
							if(a>0)
								strr+="_";
							co+=link->GetCost(network->multiclassInfo[k]->vot);
						}
						cout<<network->multiclassInfo[k]->type<<","<<co<<endl;
						sprintf(res,",%s,%.4f,%d\n",network->multiclassInfo[k]->type,path->flow,path->path.size());
						strr+=res;
						out<<strr;
					}
				}
			}
		}
	}
	else
	{
		out<<"from_node,to_node,path,flow,num\n";
		for(int i=0;i<network->numOfOrigin;i++)
		{
			TNM_SORIGIN* orgn=network->originVector[i];
			for(int j=0;j<orgn->numOfDest;j++)
			{
				TNM_SDEST* dest=orgn->destVector[j];
				if(dest->skip) continue;
				for(int h=0;h<dest->pathSet.size();h++)
				{
					TNM_SPATH* path=dest->pathSet[h];
					char res[200];
					sprintf(res,"%d,%d,",orgn->origin->id,dest->dest->id);
					string strr=res;
					//cout<<strr<<path->cost<<endl;
					for(int a=path->path.size()-1;a>=0;a--)
					{
						TNM_SLINK* link=path->path[a];
						sprintf(res,"%d",link->id);
						strr+=res;
						if(a>0)
							strr+="_";
					}
					sprintf(res,",%.4f,%d\n",path->flow,path->path.size());
					strr+=res;
					out<<strr;
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
		//link->fdCost = 0.;
		linkFile<<TNM_IntFormat(link->id)
				<<TNM_IntFormat(link->tail->id)
				<<TNM_IntFormat(link->head->id)
				<<TNM_FloatFormat(link->capacity)
				<<TNM_FloatFormat(link->volume,18,11)
				<<TNM_FloatFormat(link->cost)
				<<TNM_FloatFormat(link->toll)<<endl;
	}
}

void TNM_TAP::ReportLinkSZ(ofstream &linkFile)
{
	linkFile<<"from_node,to_node,";
	if(network->IsMultiClass)
	{
		for(int i=0;i<network->numOfClass;i++)
		{
			linkFile<<network->multiclassInfo[i]->type+"_flow,";
		}
	}
	else
	{
		linkFile<<"flow,";
	}
	linkFile<<"v/c"<<endl;
	for (int i = 0;i<network->numOfLink;i++)
	{
		TNM_SLINK *link = network->linkVector[i];
		//link->fdCost = link->GetToll();
		//link->fdCost = 0.;
		char res[200];
		sprintf(res,"%d,%d",link->tail->id,link->head->id);
		string strr=res;
		if(network->IsMultiClass)
		{
			for(int k=0;k<network->numOfClass;k++)
			{
				sprintf(res,",%.4f",link->mc_volume[k]);
				strr+=res;
			}
		}
		else
		{
			sprintf(res,",%.4f",link->volume);
			strr+=res;
		}
		sprintf(res,",%.4f\n",link->volume/link->capacity);
		strr+=res;
		linkFile<<strr;
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

void TNM_TAP::ReportIterSZ(ofstream &iteFile)
{
	if(!iterRecord.empty())
	{
		iteFile<<"Iter,ConvIndc,Time"<<endl;
		for (vector<ITERELEM*>::iterator pv = iterRecord.begin();pv != iterRecord.end(); pv++)
		{
			iteFile<<(*pv)->iter<<","<<(*pv)->conv<<","<<(*pv)->time<<endl;
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
			if(dest->skip) continue;
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
				if(dest->skip) continue;
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

	cout<<"The curIter is "<<curIter<<endl;
	//cout<<"The current Entropy is "<<entr<<endl;
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
	//cout<<"aveFlowchange is "<<aveFlowChange<<endl;
	//system("PAUSE");
	//getchar();
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
			//if((maxCost - minCost)>500)
			//	cout<<"The size of path set is "<<dest->pathSet.size()<<endl;
			TNM_SLINK* slink;
			double tFlow =0.0;
			for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin(); pv != dest->pathSet.end();pv++)
			{
				path = *pv;
				if (path !=ePath)
				{
					//if (abs(path->cost-ePath->cost) > convIndicator/10.0)
					{
						//if((maxCost - minCost)>500)
						//	cout<<"path "<<path->id;
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
							//if((maxCost - minCost)>500)
							//	cout<<" "<<slink->id<<"("<<slink->cost<<","<<slink->volume<<")";
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
						//if((maxCost - minCost)>500)
						//	cout<<endl;
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
						//if((maxCost - minCost)>500)
						//{
						//	cout<<"Path "<<path->id<<" flow "<<path->flow<<" cost "<<path->cost<<endl;
						//	cout<<"Path "<<ePath->id<<" flow "<<ePath->flow<<" cost "<<ePath->cost<<endl;
						//	cout<<"scost="<<scost<<" dflow="<<dflow<<endl;
						//	getchar();
						//}
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
	cout<<"Initializing..."<<endl;
	network->AllocateNodeBuffer(2);
	network->AllocateLinkBuffer(3);

	for(int a=0;a<network->numOfLink;a++)
	{
		TNM_SLINK* link=network->linkVector[a];
		link->volume=link->beckFlow;
	}
	//cout<<"0"<<endl;

	//if(!IsInitFromFile())
	{

		network->UpdateLinkCost();
		
		network->InitialSubNet4();
		
		network->UpdateLinkCost();
		

	}
	//getchar();

	network->UpdateLinkCostDer();
	ComputeOFV(); //compute objective function value

	//cout<<"The initial objective is : "<<OFV<<endl;
	//system("PAUSE");

	{
		for (int oi=0;oi<network->numOfOrigin;oi++)
		{
			TNM_SORIGIN* pOrg = network->originVector[oi];
			for (int di=0;di<pOrg->numOfDest;di++)
			{
				TNM_SDEST* sdest = pOrg->destVector[di];
				if(sdest->skip)
					continue;
				TNM_SPATH* spath = sdest->pathSet.front();
				nPath++;
				spath->id = nPath;
				
			}
		}
	}

	cout<<"The num of links are "<<network->numOfLink<<endl;
	cout<<"end of the ini"<<endl;

}

void TAP_iGP::PostProcess()
{
	network->UpdateLinkCost();
}

MC_iGP::MC_iGP()
{
	
}
MC_iGP::~MC_iGP()
{

}

void MC_iGP::Initialize()
{
	cout<<"begin initialize!"<<endl;
	network->AllocateNodeBuffer(2);
	network->AllocateLinkBuffer(3);

	for(int k=0;k<network->multiclassInfo.size();k++)
	{
		CLASSINFO* cinfo=network->multiclassInfo[k];
		cinfo->MarkBanlink();
		for(int a=0;a<network->numOfLink;a++)
		{
			TNM_SLINK* link=network->linkVector[a];
			if(!link->NoEntry)
			{
				link->volume+=link->mc_beckFlow[k];
				link->mc_volume[k]=link->mc_beckFlow[k];
			}
		}
		cinfo->ReMarkBanlink();
	}
	//cout<<"1"<<endl;
	for(int k=0;k<network->multiclassInfo.size();k++)
	{
		CLASSINFO* cinfo=network->multiclassInfo[k];
		cinfo->MarkBanlink();
		//cout<<"class "<<k<<" banlink "<<cinfo->BanLinkSet.size()<<endl;
		network->UpdateLinkCost(cinfo->vot);
		//cout<<"initial sub net"<<endl;
		network->InitialMCSubNet(k);
		network->UpdateLinkCost(cinfo->vot);
		cinfo->ReMarkBanlink();
	}
	//cout<<"2"<<endl;
	network->UpdateLinkCostDer();

	///////≤‚ ‘
	//for(int a=0;a<network->numOfLink;a++)
	//{

	//	cout<<"link "<<network->linkVector[a]->id<<" flow "<<network->linkVector[a]->volume<<endl;
	//}
	//for(int i=0;i<network->numOfOrigin;i++)
	//{
	//	TNM_SORIGIN* orgn=network->originVector[i];
	//	for(int j=0;j<orgn->numOfDest;j++)
	//	{
	//		TNM_SDEST* dest=orgn->destVector[j];
	//		cout<<"Origin "<<orgn->origin->id<<" dest "<<dest->dest->id<<" path "<<endl;
	//		for(int k=0;k<network->numOfClass;k++)
	//		{
	//			for(int h=0;h<dest->mc_pathset[k]->pathSet.size();h++)
	//			{
	//				TNM_SPATH* path=dest->mc_pathset[k]->pathSet[h];
	//				for(int a=0;a<path->path.size();a++)
	//					cout<<path->path[a]->id<<" ";
	//			}
	//			cout<<"\n";
	//		}
	//		
	//	}
	//}
	///////////
	//getchar();

	for(int k=0;k<network->multiclassInfo.size();k++)
	{
		for (int oi=0;oi<network->numOfOrigin;oi++)
		{
			TNM_SORIGIN* pOrg = network->originVector[oi];
			for (int di=0;di<pOrg->numOfDest;di++)
			{
				TNM_SDEST* sdest = pOrg->destVector[di];
				TNM_SPATH* spath = sdest->mc_pathset[k]->pathSet.front();
				nPath++;
				spath->id = nPath;
				
			}
		}
	}
	//cout<<"3"<<endl;
}

void MC_iGP::MainLoop()
{
	//cout<<"conduct mainloop"<<endl;
	//
	TotalFlowChange = 0.0;
	numOfPathChange =0;
	//floatType oldOFV = OFV;
	totalShiftFlow = 0.0;
	maxPathGap = 0;
	//cout<<"1"<<endl;
	TNM_SORIGIN *pOrg;
	//for(int k=network->multiclassInfo.size()-1;k>=0;k--)
	for(int k=0;k<network->multiclassInfo.size();k++)
	{
		CLASSINFO* cinfo=network->multiclassInfo[k];
		cinfo->MarkBanlink();
		
		for (int i = 0;i<network->numOfLink;i++)
		{
			TNM_SLINK* link = network->linkVector[i];
			link->cost=link->GetCost(network->multiclassInfo[k]->vot);
		}
		for (int i = 0;i<network->numOfOrigin;i++)
		{
			pOrg = network->originVector[i];

			network->UpdateSP(pOrg->origin);
			for (int j=0;j<pOrg->numOfDest;j++)
			{
				TNM_SDEST* dest = pOrg->destVector[j];
				//if(dest->mc_pathset[k]->demand==0.0)
				//	break;
				//dest->shiftFlow = 1.0;
				dest->mc_pathset[k]->shiftFlow = 1.0;
				//cout<<"1"<<endl;
				ColumnGeneration(pOrg,dest);
		    
				columnG = true;
				//cout<<"2"<<endl;
				UpdatePathFlowLazy(pOrg,dest,k);
				//cout<<"3"<<endl;
			}

		}
		cinfo->ReMarkBanlink();
	}
	cout<<"The total shifted Flow in this iteration is "<<totalShiftFlow<<endl;
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
		//numofD2 = 0;
		//for(int k=network->multiclassInfo.size()-1;k>=0;k--)
		for(int k=0;k<network->multiclassInfo.size();k++)
		{
			CLASSINFO* cinfo=network->multiclassInfo[k];
			cinfo->MarkBanlink();

			for (int i = 0;i<network->numOfLink;i++)
			{
				TNM_SLINK* link = network->linkVector[i];
				link->cost=link->GetCost(network->multiclassInfo[k]->vot);
			}
			for (int i = 0;i<network->numOfOrigin;i++)
			{
				pOrg = network->originVector[i];
		
				for (int j=0;j<pOrg->numOfDest;j++)
				{
					TNM_SDEST* dest = pOrg->destVector[j];
					//if(dest->mc_pathset[k]->demand==0.0)
					//	break;
					if (il%(maxInIter/10) == 0)
					{
						//dest->shiftFlow =1.0;
						dest->mc_pathset[k]->shiftFlow = 1.0;
					}
					if (dest->mc_pathset[k]->shiftFlow> convIndicator/2.0)
					{
						columnG = false;
						UpdatePathFlowLazy(pOrg,dest,k);
						numofD2++;
					
					}
					numofPath+=dest->mc_pathset[k]->pathSet.size();
				

				}

			}
			cinfo->ReMarkBanlink();
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


	convIndicator = MCRelativeGap();
	aveFlowChange = TotalFlowChange/numOfPathChange;

	//ComputeOFV(); //compute objective function value;
	//cout<<"Main iter = "<<curIter<<" OFV = "<<OFV<<" conv = "<<convIndicator<<endl;

	/*double entr = ComputeEntropyfromPath();
	ProInfo * pif = new ProInfo;
	pif->iter = curIter;
	pif->Entro = entr;
	pif->consis =0;
	pif->time = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	EntropyinfoVector.push_back(pif);*/

	cout<<"/////////////////////\nThe curIter is "<<curIter<<endl;
	//cout<<"The current Entropy is "<<entr<<endl;
	//cout<<"The OFV is "<<OFV<<endl;
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
	//cout<<"aveFlowchange is "<<aveFlowChange<<endl;
	//system("PAUSE");
	
	//for(int k=0;k<network->multiclassInfo.size();k++)
	//{
	//	CLASSINFO* cinfo=network->multiclassInfo[k];
	//	cinfo->MarkBanlink();
	//	
	//	for(int a=0;a<network->numOfLink;a++)
	//	{
	//		network->linkVector[a]->length=0.0;
	//	}
	//	for (int i = 0;i<network->numOfOrigin;i++)
	//	{
	//		pOrg = network->originVector[i];

	//		for (int j=0;j<pOrg->numOfDest;j++)
	//		{
	//			TNM_SDEST* dest = pOrg->destVector[j];
	//			for(int h=0;h<dest->mc_pathset[k]->pathSet.size();h++)
	//			{
	//				TNM_SPATH* path=dest->mc_pathset[k]->pathSet[h];
	//				for(int a=0;a<path->path.size();a++)
	//				{
	//					TNM_SLINK* link=path->path[a];
	//					link->length+=path->flow;
	//				}
	//			}
	//		}

	//	}
	//	for(int a=0;a<network->numOfLink;a++)
	//	{
	//		TNM_SLINK* link=network->linkVector[a];
	//		//if(link->mc_volume[k]!=link->length)
	//		if((link->mc_volume[k]-link->length)>1e-10)
	//			cout<<"link"<<link->id<<" flow1 "<<link->mc_volume[k]<<" flow2 "<<link->length<<" diff "<<(link->mc_volume[k]-link->length)<<endl;
	//	}
	//	cinfo->ReMarkBanlink();
	//}
	//getchar();
}

void MC_iGP::UpdatePathFlowLazy(TNM_SORIGIN* pOrg,TNM_SDEST* dest,int classorder)
{
	//check if the yPath is existed in the path set
	TNM_SPATH* ePath = NULL;
	bool find = true;
	TNM_SPATH* path;
	dest->mc_pathset[classorder]->shiftFlow =0.0;
	if (columnG)
	{
		for (vector<TNM_SPATH*>::iterator pv = dest->mc_pathset[classorder]->pathSet.begin(); pv != dest->mc_pathset[classorder]->pathSet.end();pv++)
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
			dest->mc_pathset[classorder]->pathSet.push_back(addPath);
			ePath = addPath;
		}
	
	}
	//added to celerate
	double maxCost =0.0;
	double minCost = 100000000.0;
	for (int pi=0;pi<dest->mc_pathset[classorder]->pathSet.size();pi++)
	{
		path = dest->mc_pathset[classorder]->pathSet[pi];
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
	dest->mc_pathset[classorder]->shiftFlow = maxCost - minCost;
	///end of adding

	if (dest->mc_pathset[classorder]->shiftFlow > ss)
	{
		//make the label of the ePath;
		ePath->preFlow = ePath->flow;
		for (int li=0;li<ePath->path.size();li++)
		{
			ePath->path[li]->markStatus = ePath->id;

		}

		//update the path flows
		numOfD++;
		if (dest->mc_pathset[classorder]->pathSet.size()>1)
		{
			//cout<<"The size of path set is "<<dest->pathSet.size()<<endl;
			TNM_SLINK* slink;
			double tFlow =0.0;
			for (vector<TNM_SPATH*>::iterator pv = dest->mc_pathset[classorder]->pathSet.begin(); pv != dest->mc_pathset[classorder]->pathSet.end();pv++)
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
								slink->mc_volume[classorder] = slink->mc_volume[classorder] - dflow;
								if (abs(slink->mc_volume[classorder]) < 1e-8)
								{
									slink->volume = slink->volume - slink->mc_volume[classorder];
									slink->mc_volume[classorder] = 0.0;
								}
								if (slink->mc_volume[classorder] < 0)
								{
									cout<<"Wrong in updating the path link flow"<<endl;
									cout<<"The volume is "<<slink->volume<<endl;
									cout<<"dflow is "<<dflow<<endl;
									cout<<"preFlow is "<<path->preFlow<<endl;
									cout<<"path flow is "<<path->flow<<endl;
									//PrintPath(path);
									system("PAUSE");

								}
								slink->cost = slink->GetCost(network->multiclassInfo[classorder]->vot);
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
			if (tFlow-dest->mc_pathset[classorder]->demand>1e-5)
			{
				cout<<"The total OD flow of paths is larger than assDemand"<<endl;
				system("PAUSE");
			}
			else
			{
				ePath->flow = dest->mc_pathset[classorder]->demand - tFlow;
				//updating the path flows
				double aFlow = ePath->flow - ePath->preFlow;
				for (int lj=0;lj<ePath->path.size();lj++)
				{
					slink = ePath->path[lj];
					slink->volume = slink->volume + aFlow;
					slink->mc_volume[classorder] = slink->mc_volume[classorder] + aFlow;
					slink->cost = slink->GetCost(network->multiclassInfo[classorder]->vot);
					slink->fdCost = slink->GetDerCost();
				}
			}
		}

		//
		for (vector<TNM_SPATH*>::iterator pv = dest->mc_pathset[classorder]->pathSet.begin(); pv != dest->mc_pathset[classorder]->pathSet.end();)
		{
			path = *pv;
			if (path->flow == 0.0)
			{
				pv = dest->mc_pathset[classorder]->pathSet.erase(pv);
				dePathSet.push_back(path);
			}
			else
			{
				pv++;
			}
		}
	}
	
	
}

floatType MC_iGP::MCRelativeGap()
{
	TNM_SLINK *link;
	TNM_SORIGIN *org;
	TNM_SDEST *dest;
	floatType gap=0.0, tt =0.0;
	
	for(int k=0;k<network->multiclassInfo.size();k++)
	{
		CLASSINFO* cinfo=network->multiclassInfo[k];
		cinfo->MarkBanlink();

		for (int i = 0;i<network->numOfLink;i++)
		{
			link = network->linkVector[i];
			link->cost=link->GetCost(network->multiclassInfo[k]->vot);
			//cout<<"link "<<link->id<<" toll "<<link->toll<<endl;
			if(!link->NoEntry)
				tt += (link->mc_volume[k] - link->mc_beckFlow[k]) * link->cost;
			//gap += link->volume * link->cost;
		}
		//gap = tt;
		for(int i = 0;i<network->numOfOrigin;i++)
		{
			org = network->originVector[i];
			network->UpdateSP(org->origin);
			//tmd += org->m_tdmd;
			for(int j = 0;j<org->numOfDest;j++)
			{
				dest = org->destVector[j];
				gap += (dest->dest->pathElem->cost * dest->mc_pathset[k]->demand);
			}
		}

		cinfo->ReMarkBanlink();
	}

	//gap /= tt;
	gap = (tt-gap)/tt;
	
	return fabs(gap); //enforce postive
}