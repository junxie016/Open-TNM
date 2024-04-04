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
	m_reportOrgFlows = false;
	m_orgFlowReportID = -1;
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


void TNM_TAP::PrintVersion(ofstream& out)
{
	if (IsPrintVersion())
	{
		time_t t;
		time(&t);
		out << "===============================================================================\n"
			<< "               open-source Toolkit of Network Modeling (open-TNM) package\n"
			<< "                         Version 1.0\n\n"
			<< "                                                          Marco Nie; Jun Xie\n"
			<< "                      Northwestern University;  Southwest Jiaotong University\n"
			<< "===============================================================================\n"
			<< "       Model:     " << setw(40) << modelNameCard << "\n"
			<< "       Algorithm: " << setw(40) << algorNameCard << "\n"
			<< "===============================================================================\n"
			<< "       This file was created at: " << ctime(&t)
			<< "===============================================================================\n"
			<< endl;
	}

}

void TNM_TAP::ReportOrgFlows(ofstream& orgFile)
{
	if (m_orgFlowReportID == -1)
	{
		for (int i = 0; i < network->numOfOrigin; i++)
		{
			TNM_SORIGIN* org = network->originVector[i];
			ORGLINK* oLink;
			if (org->obLinkVector.empty())
			{
				orgFile << "Orign link flows not available for Origin " << org->id_() << endl;
			}
			else
			{
				orgFile << "Link used in Origin " << org->id_() << endl;
				orgFile << setw(intWidth) << "ID"
					<< setw(intWidth) << "From"
					<< setw(intWidth) << "To"
					<< setw(floatWidth) << "OFlow"
					<< setw(floatWidth) << "Cost"
					<< setw(floatWidth) << "R-Cost" << endl;

				org->MarkOBLinksOnNet();
				org->UpdateCFSPTreeOnly();
				for (int j = 0; j < network->numOfNode; j++)
				{
					TNM_SNODE* node = network->nodeVector[j];
					for (PTRTRACE pt = node->forwStar.begin(); pt != node->forwStar.end(); pt++)
					{
						TNM_SLINK* link = *pt;
						if (link && link->oLinkPtr)
						{
							floatType t = link->tail->pathElem->cost + link->cost;
							floatType rc = t - link->head->pathElem->cost;
							oLink = link->oLinkPtr;
							orgFile << TNM_IntFormat(oLink->linkPtr->id)
								<< TNM_IntFormat(oLink->linkPtr->tail->id)
								<< TNM_IntFormat(oLink->linkPtr->head->id)
								<< TNM_FloatFormat(oLink->oFlow)
								<< TNM_FloatFormat(oLink->linkPtr->cost)
								<< TNM_FloatFormat(rc)
								<< endl;
						}
					}
				}
				org->RemarkOBLinksOnNet();
			}
		}
	}

	else
	{
		TNM_SORIGIN* org = network->CatchOrgPtr(m_orgFlowReportID);
		ORGLINK* oLink;
		if (org->obLinkVector.empty())
		{
			orgFile << "Orign link flows not available for Origin " << org->id_() << endl;
		}
		else
		{
			orgFile << "Link used in Origin " << org->id_() << endl;
			orgFile << setw(intWidth) << "ID"
				<< setw(intWidth) << "From"
				<< setw(intWidth) << "To"
				<< setw(floatWidth) << "OFlow"
				<< setw(floatWidth) << "Cost"
				<< setw(floatWidth) << "R-Cost" << endl;

			org->MarkOBLinksOnNet();
			org->UpdateCFSPTreeOnly();
			for (int j = 0; j < network->numOfNode; j++)
			{
				TNM_SNODE* node = network->nodeVector[j];
				for (PTRTRACE pt = node->forwStar.begin(); pt != node->forwStar.end(); pt++)
				{
					TNM_SLINK* link = *pt;
					if (link && link->oLinkPtr)
					{
						floatType t = link->tail->pathElem->cost + link->cost;
						floatType rc = t - link->head->pathElem->cost;
						oLink = link->oLinkPtr;
						orgFile << TNM_IntFormat(oLink->linkPtr->id)
							<< TNM_IntFormat(oLink->linkPtr->tail->id)
							<< TNM_IntFormat(oLink->linkPtr->head->id)
							<< TNM_FloatFormat(oLink->oFlow)
							<< TNM_FloatFormat(oLink->linkPtr->cost)
							<< TNM_FloatFormat(rc)
							<< endl;
					}
				}
			}
			org->RemarkOBLinksOnNet();
		}
	}
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

int TNM_TAP::CloseReport()
{
	if (sumFile.is_open()) sumFile.close();
	if (iteFile.is_open()) iteFile.close();
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
				<<TNM_FloatFormat(link->volume,18,11)
				<<TNM_FloatFormat(link->cost)
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
				<<TNM_FloatFormat((*pv)->ofv)
				<<TNM_FloatFormat((*pv)->conv)
				<<TNM_FloatFormat((*pv)->time)<<endl;
		}
	}

}



//////////////////////////methods for the class Pairsegment2//////////////////////////

PairSegment2::PairSegment2()
{
	Pid = 0;
	begin = NULL;
	end = NULL;
	time1 = 0;
	time2 = 0;
	der1 = 0;
	ProStatus = true;
	der2 = 0;
	flow1 = 0;
	flow2 = 0;
	shiftFlow = 0;
	minTotalFlow1 = 0;
	minTotalFlow2 = 0;
	higher = 0;
	active = true;
	proportion = 0;
	equilibrated = false;
	costDif = 0;
	OriginLabel = 0;
	Pro2 = 0;
	ta = 0;
	tb = 0;
	tc = 0;
	linear = false;
	boundPro = 0;
	prodif = 0;
	bProDif = 0;

}

//get functions
TNM_SLINK* PairSegment2::getmlSeg1()
{
	if (!seg1.empty())
	{
		return seg1.front();
	}
	else
		return NULL;
}
TNM_SLINK* PairSegment2::getmlSeg2()
{
	if (!seg2.empty())
	{
		return seg2.front();
	}
	else
		return NULL;
}
//erase origin from the pas
void PairSegment2::eraseOrigin(TNM_SORIGIN* pOrg)
{
	for (std::vector<PasOrigin1*>::iterator pv = OriginList.begin(); pv != OriginList.end();)
	{
		PasOrigin1* po1 = *pv;
		if (po1->org == pOrg)
		{
			pv = OriginList.erase(pv);
			delete po1;
			break;
		}
		else
		{
			pv++;
		}
	}
}
//
int PairSegment2::LinkSegment(TNM_SLINK* slink)
{
	//cout<<"004"<<endl;
	int n = 0;
	for (int i = 0; i < seg1.size(); i++)
	{
		TNM_SLINK* link = seg1[i];
		if (slink == link)
		{
			n = 1;
			break;
		}
	}
	for (int i = 0; i < seg2.size(); i++)
	{
		TNM_SLINK* link = seg2[i];
		if (slink == link)
		{
			n = 2;
			break;
		}
	}
	//cout<<"005"<<endl;
	return n;
}
//add origin into the pas
bool PairSegment2::addOrigin(TNM_SORIGIN* pOrg)
{
	bool re = false;
	bool lag = false;
	for (int i = 0; i < OriginList.size(); i++)
	{
		PasOrigin1* po = OriginList[i];
		if (po->org == pOrg)
		{
			lag = true;
		}
	}
	if (!lag)
	{
		PasOrigin1* po = new PasOrigin1;
		po->org = pOrg;
		OriginList.push_back(po);
		re = true;
	}
	return lag;
}
//add origin into the pas
PasOrigin1* PairSegment2::addOriginNew(TNM_SORIGIN* pOrg)
{
	bool re = false;
	bool lag = false;
	PasOrigin1* ret = NULL;
	for (int i = 0; i < OriginList.size(); i++)
	{
		PasOrigin1* po = OriginList[i];
		if (po->org == pOrg)
		{
			lag = true;
			ret = po;
			break;
		}
	}
	if (!lag)
	{
		PasOrigin1* po = new PasOrigin1;
		po->org = pOrg;
		OriginList.push_back(po);
		re = true;
		ret = po;
	}
	return ret;
}
//clear the content of pas
void PairSegment2::ClearPAS()
{
	//
	seg1.clear();
	seg2.clear();
	begin = NULL;
	end = NULL;
	flow1 = 0;
	flow2 = 0;
	time1 = 0;
	time2 = 0;
	der2 = 0;
	der1 = 0;
	for (int ni = 0; ni < OriginList.size(); ni++)
	{
		PasOrigin1* po = OriginList[ni];
		delete po;
	}
	OriginList.clear();
	shiftFlow = 0;
	minTotalFlow2 = 0;
	minTotalFlow1 = 0;

}
//show the content of the pas
void PairSegment2::showPas()
{
	cout << "-----------Now We Will Show The Content of Pas-----------" << endl;
	cout << "PAS ID : " << Pid << endl;
	cout << "Status : " << active << endl;
	cout << "The head is " << end->id << endl;
	cout << "The higher seg is : " << higher << endl;
	cout << "The Cost Difference is : " << costDif << endl;
	cout << "The cal shift flow is : " << shiftFlow << endl;
	cout << "The mintotalflow1 is : " << minTotalFlow1 << endl;
	cout << "The mintotalflow2 is : " << minTotalFlow2 << endl;
	cout << "/////////////////////////" << endl;
	cout << "Num of Origin is : " << OriginList.size() << endl;
	for (int i = 0; i < OriginList.size(); i++)
	{
		PasOrigin1* po1 = OriginList[i];
		cout << "------Origin: " << po1->org->id() << "  ; The number of dest is : " << po1->DestVector.size() << endl;
		cout << " They are: ";
		for (int j = 0; j < po1->DestVector.size(); j++)
		{
			PasDest* dest = po1->DestVector[j];
			cout << dest->dest->dest->id << ";  ";
		}
		cout << endl;
	}
	cout << "/////////////////////////" << endl;
	cout << "Segment 1 : " << seg1.back()->tail->id << setw(10);
	for (std::deque<TNM_SLINK*>::reverse_iterator pv = seg1.rbegin(); pv != seg1.rend(); pv++)
	{
		TNM_SLINK* slink = *pv;
		TNM_SNODE* snode = slink->head;
		cout << snode->id << setw(10);
	}
	cout << endl;
	cout << "Segment 2 : " << seg2.back()->tail->id << setw(10);
	for (std::deque<TNM_SLINK*>::reverse_iterator pv = seg2.rbegin(); pv != seg2.rend(); pv++)
	{
		TNM_SLINK* slink = *pv;
		TNM_SNODE* snode = slink->head;
		cout << snode->id << setw(10);
	}
	cout << endl;
	cout << "Time of Seg1 is : " << time1 << endl;
	cout << "Time of Seg2 is : " << time2 << endl;
	cout << "Flow of Seg1 is : " << flow1 << endl;
	cout << "Flow of Seg2 is : " << flow2 << endl;
	//show the origin-based segment flow
	cout << "The proportion is : " << proportion << endl;
	cout << "The origin-based proportion difference is : " << prodif << endl;
	//

	//
	cout << "----------------------------------------------------------------------------------" << endl;
	cout << endl;
	system("PAUSE");
}
//
void PairSegment2::updatePasFlow()
{
	TNM_SLINK* sl;
	minTotalFlow1 = 1e10;
	for (int i = 0; i < seg1.size(); i++)
	{
		sl = seg1[i];
		if (sl->oLinkPtr->oFlow < minTotalFlow1)
		{
			minTotalFlow1 = sl->oLinkPtr->oFlow;
		}

	}
	minTotalFlow2 = 1e10;
	for (int i = 0; i < seg2.size(); i++)
	{
		sl = seg2[i];
		if (sl->oLinkPtr->oFlow < minTotalFlow2)
		{
			minTotalFlow2 = sl->oLinkPtr->oFlow;
		}

	}
}
//cal the flow need to be shifted between the segments
floatType PairSegment2::calShiftFlow()
{
	shiftFlow = 0;
	//
	if (higher == 1)
	{
		shiftFlow = (time1 - time2) / (der1 + der2);
		if (shiftFlow < 0)
		{
			cout << "the higher is wrong, please check findhigher function" << endl;;
			//system("PAUSE");
		}
	}
	else if (higher == 2)
	{
		shiftFlow = (time2 - time1) / (der1 + der2);
		if (shiftFlow < 0)
		{
			cout << "the higher is wrong, please check findhigher function" << endl;;
			//system("PAUSE");
		}
	}
	else
	{
		cout << "The higher is wrong" << endl;
		//system("PAUSE");
	}
	return shiftFlow;
}
//update the segment time of pas
floatType PairSegment2::findHigher()
{
	time1 = 0;
	der1 = 0;
	for (int i = 0; i < seg1.size(); i++)
	{
		TNM_SLINK* link = seg1[i];
		time1 += link->cost;
		der1 += link->fdCost;
	}
	time2 = 0;
	der2 = 0;
	for (int i = 0; i < seg2.size(); i++)
	{
		TNM_SLINK* link = seg2[i];
		time2 += link->cost;
		der2 += link->fdCost;
	}
	if (time1 >= time2)
	{
		higher = 1;
	}
	else
	{
		higher = 2;
	}
	costDif = abs(time1 - time2);
	return abs(time1 - time2);
}
PairSegment2::~PairSegment2()
{
	for (std::vector<PasOrigin1*>::iterator pv = OriginList.begin(); pv != OriginList.end(); pv++)
	{
		PasOrigin1* po = *pv;
		delete po;
	}

}

//////////////======================METHODS FOR ITAPAS=============================/////////////////////////
//construction function
TAP_iTAPAS::TAP_iTAPAS()
{
	PasID = 0;
	objectID = TAP_TAPAS_ID; // an TAP_TAPAS_ID is added in the "mat_header.h"
	algorNameCard = "iTAPAS";
	modelNameCard = "Static Determinisitic Traffic Assignment Problem";
	numOfShift = 20;
	flowPrecision = 1e-10;
	costPrecision = 1e-14;
	flowPrecision2 = 1e-12;
	costfactor = 0.5;
	flowfactor = 0.25;
	IsReportPAS = true;
	acyclic = false;
	TotalExcessCost = 0;
	AveExcessCost = 0;
	NumOfNoRegPas = 0;
	AvePasOrg = 0;
	RandomNum = 400;
	NumOfCycle = 0;
	findNewPas = false;
	ReadFromOUE = false;
	DoProportionality = false;
	isReadProSolution = false;
	entropyCostPrecision = 1e-10;
	entropyFlowPrecision = 1e-6;
	zeroDeleteFlow = 0;
}


TAP_iTAPAS::~TAP_iTAPAS()
{
	//
	for (int i = 0; i < PasInfoList.size(); i++)
	{
		PasInfo* pi = PasInfoList[i];
		delete pi;
	}
	//delete the POList
	for (int i = 0; i < POList.size(); i++)
	{
		PasOrigin2* por = POList[i];
		delete por;
	}
	//delete the PNList
	for (int i = 0; i < PNList.size(); i++)
	{
		PasNode2* pon = PNList[i];
		delete pon;
	}
	//delete the PasList
	for (int i = 0; i < PasList.size(); i++)
	{
		PairSegment2* pas = PasList[i];
		delete pas;
	}
	//delete the ProinfoList
	for (int i = 0; i < ProInfoList.size(); i++)
	{
		ProInfo* pin = ProInfoList[i];
		delete pin;
	}

}

void TAP_iTAPAS::PrepareLogFile()
{
	string logFileName;
	if (ReadFromOUE == true)
	{
		logFileName = outFileName + "_Pro.log";
	}
	else
	{
		logFileName = outFileName + ".log";
	}

	if (!TNM_OpenOutFile(logFile, logFileName))
	{
		cout << "\n\tFail to Initialize an algorithm object: Cannot open .log file to write!" << endl;
		//readyToReport = false;
		//return 2;
	}
}

void TAP_iTAPAS::Initialize()
{
	PrepareLogFile();
	//network->AllocateNodeBuffer(2);
	network->AllocateLinkBuffer(3);
	//flowPrecision = __min(1e-10, convCriterion*1E3);
	
		network->UpdateLinkCost();
		//network->PrintLinks();
		network->InitialSubNet();
		network->UpdateLinkCost();
	
	ComputeOFV(); //compute objective function value
	//--test
	cout << "The OFV of initialization is : " << OFV << endl;
	//system("PAUSE");
	/////////////ini  TAPAS Variables
	//cout<<"the size of origin is "<<network->numOfOrigin<<endl;
	//cout<<"the size of origin vector is "<<network->originVector.size()<<endl;
	//system("PAUSE");
	//ini the POList
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		POList.push_back(NULL);
	}
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* or = network->originVector[oi];
		or ->m_class = oi;
		//cout<<"ID of origin "<<i<<" = "<<or->id()<<endl;
		PasOrigin2* por = new PasOrigin2;
		por->id = oi + 1;
		//ini the olinkList in por
		por->olinkList.reserve(network->numOfLink);
		for (int li = 0; li < network->numOfLink; li++)
		{
			por->olinkList.push_back(NULL);
		}
		for (std::vector<ORGLINK*>::iterator pv = or ->obLinkVector.begin(); pv != or->obLinkVector.end(); pv++)
		{
			ORGLINK* olink = *pv;
			int id = olink->linkPtr->id - 1;//all the id of links are plus 1from i
			por->olinkList[id] = olink;
		}
		//ini the onodeflow in por, the total flow of backstar links of this node
		por->onodeflow.reserve(network->numOfNode);
		for (int ni = 0; ni < network->numOfNode; ni++)
		{
			por->onodeflow.push_back(0);
		}
		//
		por->oLinkLable.reserve(network->numOfLink);
		for (int li = 0; li < network->numOfLink; li++)
		{
			por->oLinkLable.push_back(false);
		}
		//give the origin ptr
		por->OriginPtr = or ;
		POList[oi] = por;
	}

	//ini the PNList
	PNList.reserve(network->numOfNode);
	for (int i = 0; i < network->numOfNode; i++)
	{
		TNM_SNODE* onode = network->nodeVector[i];
		PasNode2* pon = new PasNode2;

		pon->NodePtr = onode;
		PNList.push_back(pon);
	}
	//ini the PasList
	int no = 250000;
	PasList.reserve(no);
	

	ProInfo* pin = new ProInfo;
	pin->iter = curIter;
	pin->time = 1.0 * (clock() - m_startRunTime) / CLOCKS_PER_SEC;
	pin->proprotion = convPro;
	pin->Entro = CalEntropy();
	//pin->consis = CheckConsistency2();
	ProInfoList.push_back(pin);
	////test 
	//cout<<"The size of POList is : "<<POList.size()<<endl;
	//cout<<"The size of PNList is : "<<PNList.size()<<endl;
	//system("PAUSE");
	//ini the LinkPasVector;
	for (int li = 0; li < network->numOfLink; li++)
	{
		//
		LinkPasVector.push_back(NULL);
	}
}

void TAP_iTAPAS::MainLoop()
{
	TNM_SORIGIN* pOrg;
	floatType gap, tgap; //set initial value of the maximum flow 
	TotalExcessCost = 0.0;
	NumOfMatch = 0;
	NumOfNewPas = 0;
	NumOfNoRegPas = 0;
	NumOfCycle = 0;
	numofCycleCreate = 0;
	cycleFlowCreate = 0;
	NumOfOverShift = 0;
	NumOfCycleBranch = 0;
	int NumOfDlink = 0;
	NumofNoBranch = 0;
	Numofgood = 0;
	deFlow = 0;
	int BadPas = 0;
	branchFlow = 0;
	lineSearch = false;
	BranchCycleFlow = 0;
	PreCycleFlow = 0;
	NumOfTotalShift = 0;
	TotalFlowShift = 0;
	NumOfCreatePas = 0;
	NumOfSegLink = 0;
	SegLinkFlowRatio = 0;
	deCycle = 0;
	//floatType BranchFlow =0;
	//int NumBranch =0;
	int NumAcy = 0;
	vector<TNM_SNODE*>::iterator pv;
	TNM_SNODE* node;
	gap = 0.0;
	RatioOfFlowLink = 0;
	NumofRepeat = 0;


	//
	cout << "==========PAS construction begins" << endl;

	for (int i = 0; i < network->numOfOrigin; i++)
	{
		pOrg = network->originVector[i];

		
		MarkobLink(pOrg, i);
		//mark the used link  and shortest path link in the subnet
		
		//remove cycle

		RemoveCycle(pOrg);

		//
		MarkUsedLink(pOrg);

	
		//generate pas for this  all used links which are not in shortest path tree
		BadPas += GeneratePAS(pOrg);
	

		LocalPasShift(pOrg);


		pOrg->RemarkOBLinksOnNet();

	}

	cout << endl << "==========PAS shift begins " << endl;

	//eliminate pas 
	//shift flow on all pases

	for (int i = 0; i < 20; i++)
	{

		for (std::vector<PairSegment2*>::iterator pv = PasList.begin(); pv != PasList.end(); pv++)
		{
			PairSegment2* pas = *pv;
			if (pas->active)
			{
				if (!eliminatePas(pas))
				{
					PasFlowShift(pas);
				}
			}



		}
	}


	int dp = deletePas();

	//-----3. redistribute flow between origins
	//if (curIter > 3)
	//{
	//	DistributeOrigin(); // there is one DistributeOrigin in postprocess, here we comment it to reduce compuation time
	//}




	//cal some statistics
	TNM_SLINK* cl;
	for (int i = 0; i < network->numOfLink; i++)
	{
		cl = network->linkVector[i];
		if (abs(cl->volume) < flowPrecision)
		{
			cl->volume = 0;
		}
	}

	floatType pasorigin = 0;
	int maxpas = 0;
	int midpas = 0;
	//
	int linkNum = 0;
	int MaxLinkNum = 0;
	for (std::vector<PairSegment2*>::iterator pv = PasList.begin(); pv != PasList.end(); pv++)
	{
		PairSegment2* pas = *pv;
		if (pas->active)
		{
			pasorigin += pas->OriginList.size();
			if (maxpas < pas->OriginList.size())
			{
				maxpas = pas->OriginList.size();
			}
		}
		//
		linkNum += pas->seg1.size();
		linkNum += pas->seg2.size();
		//
		if (MaxLinkNum < pas->seg1.size())
		{
			MaxLinkNum = pas->seg1.size();
		}
		if (MaxLinkNum < pas->seg2.size())
		{
			MaxLinkNum = pas->seg2.size();
		}
	}

	AvePasOrg = pasorigin / PasList.size();
	//
	int nPas = 0;
	for (int ni = 0; ni < PNList.size(); ni++)
	{
		PasNode2* pn = PNList[ni];
		nPas += pn->NPasList.size();
	}

	//updateLinkFlow();
	ComputeOFV();
	convIndicator = RelativeGap();

	//to collect the PAS information
	PasInfo* pi = new PasInfo;
	pi->Iter = curIter;
	pi->NumOfPas = PasList.size();
	pi->NumOfAveOrigins = AvePasOrg;
	pi->NumOfNewPas = NumOfNewPas;
	pi->NumOfMatch = NumOfMatch;
	pi->NumOfFirstDelete = NumOfNoRegPas;
	pi->NumOfAvePasNode = PasList.size() / floatType(network->numOfNode);
	pi->NumOfLinksSeg = floatType(NumOfSegLink) / floatType(NumOfCreatePas * 2);
	pi->averageRatio = SegLinkFlowRatio / floatType(NumOfSegLink);
	pi->TotalPasCreate = NumOfCreatePas;
	pi->MaxLinkSeg = MaxLinkNum;
	pi->MaxOriginPas = maxpas;
	pi->NumOfdelete = dp;
	pi->numOfShift = NumOfTotalShift;
	pi->TotalShiftFlow = TotalFlowShift;
	PasInfoList.push_back(pi);
	//collect the Cycle Info
	CycleInfo* ci = new CycleInfo;
	ci->Iter = curIter;
	ci->PreCycles = NumOfCycle;
	ci->PreFlow = PreCycleFlow;
	ci->InCycles = numofCycleCreate + NumOfCycleBranch;
	ci->InFlow = cycleFlowCreate + BranchCycleFlow;
	CycleInList.push_back(ci);

	//convIndicator = TotalExcessCost;
	logFile << "==============================================================================" << endl;
	logFile << "The main iteration is : " << curIter << endl;
	logFile << "The time now is : " << 1.0 * (clock() - m_startRunTime) / CLOCKS_PER_SEC << endl;
	logFile << "The deleted pas num is : " << dp << endl;
	logFile << "The active pas num is : " << PasList.size() << endl;
	//cout<<"The num of pas in PNList is : "<<nPas<<endl;
	logFile << "The matched pas is : " << NumOfMatch << endl;
	logFile << "The number of new pas is " << NumOfNewPas << endl;
	logFile << "The average pas num of each node is : " << PasList.size() / floatType(network->numOfNode) << endl;
	logFile << "The average pas origin size is : " << AvePasOrg << endl;
	logFile << "The largest pas origin size is : " << maxpas << endl;
	logFile << "The total Num of Cycles is : " << NumOfCycle << endl;
	logFile << "The total deleted Cycle Flow is " << PreCycleFlow << endl;
	//logFile<<"The number of cycles identified in NewBranchShift : "<<NumOfCycleBranch<<endl;
	logFile << "The deCycle is : " << deCycle << endl;
	logFile << "The total excess cost is : " << TotalExcessCost << endl;
	logFile << "The no registered pas is : " << NumOfNoRegPas << endl;
	logFile << "The number of cycles identified in create pas is : " << numofCycleCreate << endl;
	logFile << "The cycle flows removed in create pas is: " << cycleFlowCreate << endl;
	logFile << "The total deFlow in creatPAS is : " << deFlow << endl;
	logFile << "the bad pas generated is : " << BadPas << endl;
	logFile << "The Repeated PAS is " << NumofRepeat << endl;
	logFile << "=============The OFV is : " << OFV << endl;
	logFile << "===============The new GAP is ==================: " << convIndicator << endl;
	//test
	cout << "The main iteration is : " << curIter << endl;
	cout << "The time now is : " << 1.0 * (clock() - m_startRunTime) / CLOCKS_PER_SEC << endl;
	cout << "The deleted pas num is : " << dp << endl;
	cout << "The active pas num is : " << PasList.size() << endl;
	//cout<<"The num of pas in PNList is : "<<nPas<<endl;
	cout << "The matched pas is : " << NumOfMatch << endl;
	cout << "The number of new pas is " << NumOfNewPas << endl;
	cout << "The average pas num of each node is : " << PasList.size() / floatType(network->numOfNode) << endl;
	cout << "The average pas origin size is : " << AvePasOrg << endl;
	cout << "The largest pas origin size is : " << maxpas << endl;
	cout << "The total Num of Cycles is : " << NumOfCycle << endl;
	cout << "The total deleted Cycle Flow is " << PreCycleFlow << endl;
	//cout<<"The number of cycles identified in NewBranchShift : "<<NumOfCycleBranch<<endl;
	cout << "The deCycle is : " << deCycle << endl;
	cout << "The total excess cost is : " << TotalExcessCost << endl;
	cout << "The no registered pas is : " << NumOfNoRegPas << endl;
	cout << "The number of cycles identified in create pas is : " << numofCycleCreate << endl;
	cout << "The cycle flows removed in create pas is: " << cycleFlowCreate << endl;
	cout << "The total deFlow in creatPAS is : " << deFlow << endl;
	cout << "the bad pas generated is : " << BadPas << endl;
	cout << "The Repeated PAS is " << NumofRepeat << endl;
	cout << cout.precision(12);
	cout << "=============The OFV is : " << OFV << endl;
	cout << "===============The new GAP is ==================: " << convIndicator << endl;

	if (curIter > 2 && TotalExcessCost < totalExcessPrecision)
	{
		cout << "The algorithm is stopping because the total excess is less than " << totalExcessPrecision << endl;
		logFile << "The algorithm is stopping because the total excess is less than " << totalExcessPrecision << endl;
		maxMainIter = 2;
	}


}


void TAP_iTAPAS::PostProcess()
{

	CalLinkApproach();
	//PrePareProportion();
	ComputeConsistency();
	//logFile.close();
	


	if (DoProportionality)
	{
		
		//add and delete olinks according to reduced cost
		PrePareProportion();
		//collect origins for PASs
		
		CalLinkApproach();

		
		PostProportionality();

	
		cout << "The number of PAS IS " << PasList.size() << endl;
		//OriginbasedPASAnalysis();

		ComputeConsistency();

		ReducePAS();

		SaveUESolution2();
		SaveTAPASOUE2();
		ReportProportion();
	}
	logFile.close();
	
}


//
void TAP_iTAPAS::ReportProportion()
{
	string oueFileName = outFileName + "_Pro.ent";
	ofstream out;
	cout << "\tWriting Entropy and Proportion into " << outFileName << " for post process" << endl;
	if (!TNM_OpenOutFile(out, oueFileName))
	{
		cout << "\n\tFail to prepare report: Cannot open .oue file to write UE solution information!" << endl;
		return;
	}
	PrintVersion(out);
	out << setw(12) << setw(intWidth) << "   Iter   "
		<< setw(floatWidth) << "   Entropy    "
		<< setw(intWidth) << "    Consistency  "
		<< setw(floatWidth) << "  Ave Proportion   "
		<< setw(floatWidth) << "   Time  "
		<< endl;
	for (int i = 0; i < ProInfoList.size(); i++)
	{
		ProInfo* pin = ProInfoList[i];
		out << setw(12)
			<< TNM_IntFormat(pin->iter) << setw(12) << TNM_FloatFormat(pin->Entro, 14, 4) << setw(12) << TNM_IntFormat(pin->consis, 8) << setw(12) << TNM_FloatFormat(pin->proprotion, 14, 4) << setw(12) << TNM_FloatFormat(pin->time, 8, 4) << endl;
	}
}

void TAP_iTAPAS::SaveTAPASOUE2()
{
	string oueFileName = outFileName + "_Pro_tapas.oue";
	ofstream out;
	cout << "\tWriting UE solution information into " << outFileName << " following the TAPAS output format" << endl;
	if (!TNM_OpenOutFile(out, oueFileName))
	{
		cout << "\n\tFail to prepare report: Cannot open .oue file to write UE solution information!" << endl;
		return;
	}
	//
	out << setw(5) << "Origin" << setw(5) << "Tail" << setw(5) << "Head" << setw(10) << "Flow" << endl;
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		//
		TNM_SORIGIN* pOrg = network->originVector[oi];
		for (int li = 0; li < pOrg->obLinkVector.size(); li++)
		{
			ORGLINK* olink = pOrg->obLinkVector[li];
			if (olink->oFlow > flowPrecision)
			{
				out << left << setw(intWidth) << TNM_IntFormat(pOrg->id()) << setw(intWidth) << TNM_IntFormat(olink->linkPtr->tail->id) << setw(intWidth) << TNM_IntFormat(olink->linkPtr->head->id) << setw(floatWidth) << setw(floatWidth) << TNM_FloatFormat(olink->oFlow, 20, 11) << setw(floatWidth) << setw(floatWidth) << endl;
			}

		}
	}
	out.close();
}

void TAP_iTAPAS::SaveUESolution2()
{
	string oueFileName = outFileName + "_Pro.oue";
	ofstream out;
	cout << "\tWriting UE solution information into " << outFileName << " for post process" << endl;
	if (!TNM_OpenOutFile(out, oueFileName))
	{
		cout << "\n\tFail to prepare report: Cannot open .oue file to write UE solution information!" << endl;
		return;
	}
	//
	cout << "\tWriting the network link flow info ..." << endl;
	out << TNM_IntFormat(network->numOfLink, 6) << endl;
	for (int li = 0; li < network->numOfLink; li++)
	{
		TNM_SLINK* sl = network->linkVector[li];
		out << TNM_IntFormat(sl->tail->id, 6) << TNM_IntFormat(sl->head->id, 6) << setw(floatWidth) << TNM_FloatFormat(sl->volume, 20, 11) << setw(floatWidth);
	}
	out << endl;
	//
	cout << "\t Writing the origin-based link flow info..." << endl;
	out << TNM_IntFormat(network->numOfOrigin, 6) << endl;
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		out << TNM_IntFormat(pOrg->id(), 6) << TNM_IntFormat(pOrg->obLinkVector.size(), 6) << endl;
		for (int oj = 0; oj < pOrg->obLinkVector.size(); oj++)
		{
			ORGLINK* olink = pOrg->obLinkVector[oj];
			//if (olink->oFlow > 0)
			{
				out << TNM_IntFormat(olink->linkPtr->tail->id, 6) << TNM_IntFormat(olink->linkPtr->head->id, 6) << setw(floatWidth) << setw(floatWidth) << TNM_FloatFormat(olink->oFlow, 20, 11) << setw(floatWidth) << setw(floatWidth);
			}

		}
		out << endl;
	}
	cout << "\tWriting the PAS info..." << endl;
	out << TNM_IntFormat(PasList.size(), 6) << endl;
	for (int pi = 0; pi < PasList.size(); pi++)
	{
		PairSegment2* pas = PasList[pi];
		out << TNM_IntFormat(pas->seg1.size(), 6);
		for (int si = pas->seg1.size() - 1; si >= 0; si--)
		{
			TNM_SLINK* sl = pas->seg1[si];
			out << TNM_IntFormat(sl->tail->id, 6) << TNM_IntFormat(sl->head->id, 6);
		}
		out << endl;
		out << TNM_IntFormat(pas->seg2.size(), 6);
		for (int si = pas->seg2.size() - 1; si >= 0; si--)
		{
			TNM_SLINK* sl = pas->seg2[si];
			out << TNM_IntFormat(sl->tail->id, 6) << TNM_IntFormat(sl->head->id, 6);
		}
		out << endl;
		out << TNM_IntFormat(pas->OriginList.size(), 6);
		for (int si = 0; si < pas->OriginList.size(); si++)
		{
			TNM_SORIGIN* pOrg = pas->OriginList[si]->org;
			out << TNM_IntFormat(pOrg->id(), 6);

		}
		out << endl;
	}

	out.close();
}

int TAP_iTAPAS::ReducePAS()
{
	//
	int nPas = 0;
	PairSegment2* pas;
	PasNode2* pn;
	for (int i = 0; i < PNList.size(); i++)
	{
		pn = PNList[i];
		pn->NPasList.clear();
	}
	for (vector<PairSegment2*>::iterator pv = PasList.begin(); pv != PasList.end(); )
	{
		pas = *pv;
		updatePasFlow(pas);
		if (pas->minTotalFlow1 == 0 || pas->minTotalFlow2 == 0)
		{
			pv = PasList.erase(pv);
			delete pas;
		}
		else
		{
			TNM_SNODE* end = pas->end;
			PNList[end->id - 1]->NPasList.push_back(pas);
			pv++;
		}
	}
	////
	//for (int pi=0; pi<PasList.size(); pi++)
	//{
	//	PairSegment2* pas = PasList[pi];
	//	

	//}

	return 0;
}


//a post process for the proportionaltiy 
void TAP_iTAPAS::PostProportionality()
{
	//
	convPro = 1;
	int pIter = 0;
	int failFind = 0;
	int addO = 0;
	floatType pdd = 0;
	int cre = 0;
	int mat = 0;
	//m_startRunTime = clock();
	while (convPro > 1e-14 && pIter < 50 && 1.0 * (clock() - m_startRunTime) / CLOCKS_PER_SEC < 10000) //convPro > 1e-10 && 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC <450
	{
		failFind = 0;
		addO = 0;
		cre = 0;
		mat = 0;
		int matched = 0;
		int newadd = 0;
		int zeronode = 0;
		pIter++;
		// PAS construction
		for (int oi = 0; oi < network->numOfOrigin; oi++)
		{
			TNM_SORIGIN* pOrg = network->originVector[oi];
			if (pOrg->id() % 100 == 0)
			{
				cout << pOrg->id() << endl;
			}
			pOrg->MarkOBLinksOnNet();
			MarkobLink(pOrg, oi);
			network->UpdateSP(pOrg->origin);
			//test

			/*cout<<"The origin is : "<<pOrg->id()<<endl;
			system("PAUSE");*/
			//generate pas
			for (int li = 0; li < network->numOfLink; li++)
			{
				TNM_SLINK* slink = network->linkVector[li];
				floatType rca = slink->tail->pathElem->cost + slink->cost - slink->head->pathElem->cost;
				if (rca < entropyCostPrecision && slink->oLinkPtr == NULL)
				{
					ORGLINK* olink = new ORGLINK;
					olink->linkPtr = slink;
					slink->oLinkPtr = olink;
					olink->oFlow = 0.0;
					olink->approach = 0.0;
					pOrg->obLinkVector.push_back(olink);
					POList[pOrg->m_class]->olinkList[slink->id - 1] = olink;
				}
				if (rca < entropyCostPrecision && slink->cost != 0)
				{
					TNM_SNODE* pend = slink->head;
					bool exi = false;

					//if ( slink->oLinkPtr->oFlow < entropyFlowPrecision   ) // || slink->head->pathElem->via != slink
					{
						//
						TNM_SNODE* head = slink->head;
						floatType af = 0;
						int numl = 0;
						TNM_SLINK* mlink = slink;
						for (int hi = 0; hi < head->backStar.size(); hi++)
						{
							TNM_SLINK* sl = head->backStar[hi];
							if (sl && sl->oLinkPtr)
							{
								//if (sl->oLinkPtr->oFlow > flowPrecision)
								{
									if (sl->oLinkPtr->oFlow > mlink->oLinkPtr->oFlow)
									{
										mlink = sl;
									}
									af += sl->oLinkPtr->oFlow;
									numl++;
								}


							}

						}
						if (af < flowPrecision)
						{
							zeronode++;
							continue;
						}
						if (mlink == slink)
						{
							continue;
						}
						//
						newadd++;
						bool ToNew = false;
						slink->oLinkPtr->rc = rca;

						//int num = CreateMultiProPAS(slink,pOrg);
						PairSegment2* npas = CreateProPAS(slink, pOrg);
						int num = 0;
						if (npas)
						{
							num = 1;
						}


						if (num == 0)
						{
							failFind++;
						}
						else
						{
							cre++;
							//test
							/*updatePasFlow(npas);
							npas->showPas();
							cout<<"A PAS is created!"<<endl;
							system("PAUSE");*/
						}
						//}

					}

				}


			}
			//add origins for this PAS
			for (int pi = 0; pi < PasList.size(); pi++)
			{
				PairSegment2* pas = PasList[pi];
				bool ex = false;
				for (int pj = 0; pj < pas->OriginList.size(); pj++)
				{
					if (pOrg == pas->OriginList[pj]->org)
					{
						ex = true;
					}
				}
				//
				if (ex == false)
				{
					floatType flow1 = 1000000000000;
					floatType flow2 = 1000000000000;
					bool n1 = false;
					bool n2 = false;
					for (int si = 0; si < pas->seg1.size(); si++)
					{
						TNM_SLINK* sl = pas->seg1[si];
						if (sl != NULL && sl->oLinkPtr != NULL)
						{
							if (flow1 > sl->oLinkPtr->oFlow)
							{
								flow1 = sl->oLinkPtr->oFlow;
							}
						}
						else
						{
							flow1 = 0;
							n1 = true;
							break;
						}
					}
					for (int si = 0; si < pas->seg2.size(); si++)
					{
						TNM_SLINK* sl = pas->seg2[si];
						if (sl != NULL && sl->oLinkPtr != NULL)
						{
							if (flow2 > sl->oLinkPtr->oFlow)
							{
								flow2 = sl->oLinkPtr->oFlow;
							}
						}
						else
						{
							flow2 = 0;
							n2 = true;
							break;
						}
					}

					//
					if (!n1 && !n2 && (flow1 + flow2 > 1e-8))
					{
						pas->addOrigin(pOrg);
						addO++;
					}
				}
			}



			pOrg->RemarkOBLinksOnNet();
		}

		//adjust Proportion
		for (std::vector<PairSegment2*>::iterator pv = PasList.begin(); pv != PasList.end(); pv++)
		{
			PairSegment2* pas = *pv;
			pas->linear = true;

			if (pas->active == true)
			{
				int singNum = 0;
				floatType pasdif = UpdateSegFlow(pas);
				//pas->linear = false;

				while (pasdif > convPro / 1.0 && singNum < 20 && (pas->flow1 > 1e-8 && pas->flow2 > 1e-8)) //&& singNum < 20
				{
					//
					singNum++;
					//
					//UpdateSegFlow(pas);
					UnityProportion6(pas, pIter);
					pasdif = UpdateSegFlow(pas);

				}


			}
		}
		//
		pdd = 0;
		floatType totalPdd = 0;
		//CalLinkApproach();
		for (int pi = 0; pi < PasList.size(); pi++)
		{
			PairSegment2* pas = PasList[pi];
			if (pas->active)
			{
				UpdateSegFlow(pas);
				totalPdd += pas->prodif;

				if (pdd < pas->prodif)
				{
					pdd = pas->prodif;
				}


			}


		}
		convPro = totalPdd / floatType(PasList.size());
		//
		ProInfo* pin = new ProInfo;
		pin->iter = pIter;
		pin->time = 1.0 * (clock() - m_startRunTime) / CLOCKS_PER_SEC;
		pin->proprotion = convPro;
		pin->Entro = CalEntropy();
		//pin->consis = CheckConsistency2();
		ProInfoList.push_back(pin);

		//
		//cout the origin based link and flow
		/*TNM_SORIGIN* pOrg = network->originVector[3];
		for (int li=0; li<pOrg->obLinkVector.size(); li++)
		{
		ORGLINK* olink = pOrg->obLinkVector[li];
		cout<<olink->linkPtr->tail->id<<"-->"<<olink->linkPtr->head->id<<" 's oflow is : "<<olink->oFlow<<endl;
		}
		system("PAUSE");*/
		ComputeConsistency();
		if (pIter % 10 == 0)
		{
			//
			cout << "The pIter is " << pIter << endl;
			cout << "The time now is: " << pin->time << endl;
			cout << "The average proportion is " << pin->proprotion << endl;
			cout << "The Entropy is : " << pin->Entro << endl;
			cout << "The consistency is : " << pin->consis << endl;
			cout << "The failed link is : " << failFind << endl;
			cout << "The new add PAS is : " << cre << endl;
			cout << "The matched PAS is :" << mat << endl;
			cout << "The added origins are ; " << addO << endl;
			cout << "The matched PAS is " << matched << endl;
			cout << "The new added PAS is " << newadd << endl;
			cout << "The Size of PAS IS " << PasList.size() << endl;
			cout << "the number of zeronode is " << zeronode << endl;
		}
		logFile << "The pIter is " << pIter << endl;
		logFile << "The time now is: " << pin->time << endl;
		logFile << "The average proportion is " << pin->proprotion << endl;
		logFile << "The Entropy is : " << pin->Entro << endl;
		logFile << "The consistency is : " << pin->consis << endl;
		logFile << "The failed link is : " << failFind << endl;
		logFile << "The new add PAS is : " << cre << endl;
		logFile << "The matched PAS is :" << mat << endl;
		logFile << "The added origins are ; " << addO << endl;
		logFile << "The matched PAS is " << matched << endl;
		logFile << "The new added PAS is " << newadd << endl;
		logFile << "The Size of PAS IS " << PasList.size() << endl;
		logFile << "the number of zeronode is " << zeronode << endl;
		logFile << "/////////////////////////////////////////////////////////////////" << endl;

		//system("PAUSE");
	}
}


floatType TAP_iTAPAS::UnityProportion6(PairSegment2* pas, floatType pIter)
{
	floatType maxDiffPro = 0;
	TNM_SNODE* endNode = pas->end;
	int enID = endNode->id - 1;



	//do the adjustment flow for each origin segment
	if (pas->OriginList.size() > 1)
	{
		floatType cdp = 0;
		floatType totalsum = 0;


		if (pas->active)
		{
			badQua++;


			for (std::vector<PasOrigin1*>::iterator pw = pas->OriginList.begin(); pw != pas->OriginList.end(); pw++)
			{
				//cal the dp for this origin
				PasOrigin1* po1 = *pw;
				if (po1->gs1 + po1->gs2 > entropyFlowPrecision)
				{
					po1->dp = pas->proportion * (po1->gs1 + po1->gs2) - po1->gs1;

					cdp += po1->dp;

					/*if (abs(po1->dp)<flowPrecision)
					{
					po1->dp = 0;
					}*/
					//
					if (po1->dp + po1->gs1 < 0 || po1->gs2 - po1->dp < 0)
					{
						cout << "The dp is wrong" << endl;
						//system("PAUSE");
					}
				}
				else
				{
					po1->dp = 0;
				}

				//test


			}
		}




		if (abs(cdp) < 1e-5)
		{
			UpdateApproach(pas);
		}
		else
		{
			//UpdateApproach(pas);
			cout << "The cdp is : " << cdp << endl;
			//system("PAUSE");
		}

	}


	return maxDiffPro;

}



floatType TAP_iTAPAS::UpdateApproach(PairSegment2* pas)
{
	for (std::vector<PasOrigin1*>::iterator pw = pas->OriginList.begin(); pw != pas->OriginList.end(); pw++)
	{
		PasOrigin1* po1 = *pw;
		int oid = po1->org->m_class;
		TNM_SLINK* segLink = NULL;
		for (int sli = 0; sli < pas->seg1.size(); sli++)
		{
			segLink = pas->seg1[sli];
			ORGLINK* olink = POList[oid]->olinkList[segLink->id - 1];
			if (olink->linkPtr != segLink)
			{
				cout << "The olink is not the link" << endl;
				cout << "slink is " << segLink->tail->id << "-->" << segLink->head->id << endl;
				cout << "olink is " << olink->linkPtr->tail->id << "-->" << olink->linkPtr->head->id << endl;
				system("PAUSE");
			}
			if (olink == NULL)
			{
				cout << "The olink in updateapproach is NULL" << endl;
				//system("PAUSE");
			}
			floatType of = olink->oFlow;
			olink->oFlow = olink->oFlow + po1->dp;
			/*if (abs(olink->oFlow)<flowPrecision)
			{
			olink->oFlow = 0;
			}*/
			if (olink->oFlow < 0)
			{
				cout << "Oflow is smaller than zero" << endl;
				cout << "oflow is " << olink->oFlow << endl;
				cout << "The link is " << olink->linkPtr->tail->id << "-->" << olink->linkPtr->head->id << endl;
				logFile << "The link is " << olink->linkPtr->tail->id << "-->" << olink->linkPtr->head->id << endl;
				logFile << "oflow is " << olink->oFlow << endl;
				zeroDeleteFlow += olink->oFlow;
				olink->oFlow = 0;
			}
		}
		for (int sli = 0; sli < pas->seg2.size(); sli++)
		{
			segLink = pas->seg2[sli];
			ORGLINK* olink = POList[oid]->olinkList[segLink->id - 1];
			if (olink->linkPtr != segLink)
			{
				cout << "The olink is not the link" << endl;
				cout << "slink is " << segLink->tail->id << "-->" << segLink->head->id << endl;
				cout << "olink is " << olink->linkPtr->tail->id << "-->" << olink->linkPtr->head->id << endl;
				system("PAUSE");
			}
			if (olink == NULL)
			{
				cout << "The olink in updateapproach is NULL" << endl;
				//system("PAUSE");
			}
			olink->oFlow = olink->oFlow - po1->dp;
			/*if (abs(olink->oFlow)<flowPrecision)
			{
			olink->oFlow = 0;
			}*/
			if (olink->oFlow < 0)
			{
				cout << "oflow is smaller than zero" << endl;
				cout << "oflow is " << olink->oFlow << endl;
				cout << "The link is " << olink->linkPtr->tail->id << "-->" << olink->linkPtr->head->id << endl;
				logFile << "The link is " << olink->linkPtr->tail->id << "-->" << olink->linkPtr->head->id << endl;
				logFile << "oflow is " << olink->oFlow << endl;
				//system("PAUSE");
				zeroDeleteFlow += olink->oFlow;
				olink->oFlow = 0;
			}
		}

		//update the link approach for seg1
		for (int sl = 0; sl < pas->seg1.size(); sl++)
		{
			TNM_SNODE* head = pas->seg1[sl]->head;
			floatType sumf = 0;
			if (head != po1->org->origin)
			{
				for (int hi = 0; hi < head->backStar.size(); hi++)
				{
					TNM_SLINK* blink = head->backStar[hi];
					ORGLINK* olink = POList[oid]->olinkList[blink->id - 1];
					if (olink != NULL)
					{
						sumf += olink->oFlow;

					}
				}

				POList[oid]->onodeflow[head->id - 1] = sumf;

				for (int hi = 0; hi < head->backStar.size(); hi++)
				{
					TNM_SLINK* blink = head->backStar[hi];
					ORGLINK* olink = POList[oid]->olinkList[blink->id - 1];
					if (olink != NULL)
					{
						if (sumf > entropyFlowPrecision)
						{
							olink->approach = olink->oFlow / sumf;
						}
						else
						{
							olink->approach = 0;
						}

						if (olink->approach > entropyFlowPrecision)
						{
							olink->entro = -log(olink->approach) - 1;
						}
						else
						{
							olink->entro = 0;
						}

					}

				}
			}


		}

		//update the link approach for seg2
		for (int sl = 0; sl < pas->seg2.size(); sl++)
		{
			TNM_SNODE* head = pas->seg2[sl]->head;
			floatType sumf = 0;
			if (head != po1->org->origin)
			{
				for (int hi = 0; hi < head->backStar.size(); hi++)
				{
					TNM_SLINK* blink = head->backStar[hi];
					ORGLINK* olink = POList[oid]->olinkList[blink->id - 1];
					if (olink != NULL)
					{
						sumf += olink->oFlow;
					}
				}

				POList[oid]->onodeflow[head->id - 1] = sumf;

				for (int hi = 0; hi < head->backStar.size(); hi++)
				{
					TNM_SLINK* blink = head->backStar[hi];
					ORGLINK* olink = POList[oid]->olinkList[blink->id - 1];
					if (olink != NULL)
					{
						if (sumf > entropyFlowPrecision)
						{
							olink->approach = olink->oFlow / sumf;
						}
						else
						{
							olink->approach = 0;
						}
						if (olink->approach > entropyFlowPrecision)
						{
							olink->entro = -log(olink->approach) - 1;
						}
						else
						{
							olink->entro = 0;
						}



					}
				}
			}


		}

	}

	return 0;
}


//
floatType TAP_iTAPAS::UpdateSegFlow(PairSegment2* pas)
{
	floatType gs1 = 0;
	floatType gs2 = 0;
	floatType psum = 0;

	for (std::vector<PasOrigin1*>::iterator pw = pas->OriginList.begin(); pw != pas->OriginList.end(); pw++)
	{
		PasOrigin1* po1 = *pw;
		int oid = po1->org->m_class;
		floatType endFlow = POList[oid]->onodeflow[pas->end->id - 1];//find the endNode flow for this origin
		TNM_SLINK* segLink = NULL;
		floatType segFlow = endFlow;
		//
		//cout<<"Segment 1 is : "<<endl;
		floatType minLinkFlow = 100000000000;
		for (int si = 0; si < pas->seg1.size(); si++)
		{
			segLink = pas->seg1[si];

			if (POList[oid]->olinkList[segLink->id - 1] == NULL)
			{
				cout << "There is NULL link in adjust the proportionality!" << endl;
				system("PAUSE");
			}
			else
			{
				segFlow = segFlow * POList[oid]->olinkList[segLink->id - 1]->approach;
				if (minLinkFlow > POList[oid]->olinkList[segLink->id - 1]->oFlow)
				{
					minLinkFlow = POList[oid]->olinkList[segLink->id - 1]->oFlow;
				}
			}
			//TEST
			//cout<<segLink->tail->id<<"-->"<<segLink->head->id<<" ; ";

		}
		po1->gs1 = segFlow;
		/*if (po1->gs1 > minLinkFlow)
		{
		po1->gs1 = minLinkFlow;
		}*/
		gs1 += po1->gs1;

		minLinkFlow = 10000000000;
		segFlow = endFlow;
		for (int si = 0; si < pas->seg2.size(); si++)
		{

			segLink = pas->seg2[si];
			if (POList[oid]->olinkList[segLink->id - 1] == NULL)
			{
				cout << "There is NULL link in adjusting the proportionality!" << endl;
				system("PAUSE");
			}
			else
			{
				segFlow = segFlow * POList[oid]->olinkList[segLink->id - 1]->approach;
				if (minLinkFlow > POList[oid]->olinkList[segLink->id - 1]->oFlow)
				{
					minLinkFlow = POList[oid]->olinkList[segLink->id - 1]->oFlow;
				}
			}

			//
			//test
			//cout<<segLink->tail->id<<"-->"<<segLink->head->id<<" ; ";
				//
		}

		//system("PAUSE");
		po1->gs2 = segFlow;
		/*if (po1->gs2 > minLinkFlow)
		{
		po1->gs2 = minLinkFlow;
		}*/
		gs2 += po1->gs2;

		/*if (minLinkFlow + 1e-12- po1->gs2  < 0)
		{
		cout<<"There is something wrong in cal the segment 2 flow"<<endl;
		cout<<"min flow is "<<minLinkFlow<<endl;
		cout<<"segment flow is "<<po1->gs2<<endl;
		system("PAUSE");
		}*/




		po1->proportion = po1->gs1 / (po1->gs1 + po1->gs2);
		psum += po1->proportion;


		//cout<<"The proportion is : "<<po1->proportion<<endl;
		//system("PAUSE");

	}
	floatType Pro = gs1 / (gs1 + gs2);
	floatType pdif = 0;

	for (std::vector<PasOrigin1*>::iterator pw = pas->OriginList.begin(); pw != pas->OriginList.end(); pw++)
	{
		PasOrigin1* po1 = *pw;
		po1->proDif = abs(Pro - po1->proportion);
		if (pdif < po1->proDif)
		{
			pdif = po1->proDif;
		}
	}
	pas->prodif = pdif;
	pas->flow1 = gs1;
	pas->flow2 = gs2;
	pas->proportion = Pro;

	return pdif;

}


//
PairSegment2* TAP_iTAPAS::CreateProPAS(TNM_SLINK* slink, TNM_SORIGIN* pOrg)
{
	findNewPas = false;

	PairSegment2* goodpas = NULL;
	TNM_SNODE* origin = pOrg->origin;
	TNM_SNODE* endNode = slink->head;//the merge node of new pas
	TNM_SNODE* beginNode = NULL;

	//check if the end node have a positive flow
	floatType endFlow = 0;

	for (int ni = 0; ni < endNode->backStar.size(); ni++)
	{
		TNM_SLINK* link = endNode->backStar[ni];
		if (link != NULL && link->oLinkPtr != NULL && link->oLinkPtr->oFlow > entropyFlowPrecision)
		{
			endFlow += link->oLinkPtr->oFlow;
		}
	}

	if (endFlow > entropyFlowPrecision)
	{
		//search a new PAS for it
		PairSegment2* pas = new PairSegment2();
		pas->addOrigin(pOrg);
		pas->end = endNode;

		TNM_SNODE* rn;
		for (int i = 0; i < network->numOfNode; i++)
		{
			rn = network->nodeVector[i];
			//rn->rPathElem->via = NULL;
			rn->scanStatus = 0;
		}

		//find out the shortNodeList
		TNM_SNODE* node = slink->tail;
		node->scanStatus = 1;
		while (node != origin)
		{
			TNM_SLINK* link = node->pathElem->via;

			if (link == NULL)
			{
				cout << "there is something wrong in finding the shortNodeList" << endl;
			}
			node = link->tail;
			node->scanStatus = 1;
		}
		//
		vector<int>nodeLabel(network->numOfNode, 0);
		bool find = false;
		//pas->seg2.push_back(slink);
		TNM_SNODE* Hnode = slink->head;
		nodeLabel[Hnode->id - 1] = 1;
		//nodeLabel[endNode->id-1] =1;
		int fnum = 0;
		while (!find)
		{

			if (Hnode->scanStatus == 1)
			{
				find = true;
				beginNode = Hnode;
				break;
			}
			else
			{
				TNM_SLINK* nextLink = NULL;
				floatType of = 0;
				for (int li = 0; li < Hnode->backStar.size(); li++)
				{
					TNM_SLINK* blink = Hnode->backStar[li];
					if (blink->oLinkPtr != NULL && of < blink->oLinkPtr->oFlow && blink->oLinkPtr->oFlow > 0 && blink != slink) //&& nodeLabel[blink->tail->id-1] != 1
					{
						nextLink = blink;
						of = blink->oLinkPtr->oFlow;
					}
				}
				//put the next link into seg2
				if (nextLink != NULL)
				{
					//test 
					//cout<<"next link is : "<<nextLink->tail->id<<"-->"<<nextLink->head->id<<endl;
					//
					pas->seg2.push_back(nextLink);
					Hnode = nextLink->tail;
					//check if a cycle exist
					if (nodeLabel[Hnode->id - 1] == 1)
					{
						// a cycle exist
						deque<TNM_SLINK*> cycleVector;
						cycleVector = pas->seg2;
						while (cycleVector.front()->head != Hnode && cycleVector.size() > 0)
						{
							cycleVector.pop_front();
						}

						floatType minflow = 1e10;
						for (int si = 0; si < cycleVector.size(); si++)
						{
							TNM_SLINK* segl = cycleVector[si];

							if (segl->oLinkPtr->oFlow < minflow)
							{
								minflow = segl->oLinkPtr->oFlow;
							}

						}
						//
						/*cout<<"a cycle exist when create new PAS for proportionality and its min flow is "<<minflow<<endl;
						system("PAUSE");
						*/
						//remove the cycle flows
						if (minflow > 0)
						{
							for (int ci = 0; ci < cycleVector.size(); ci++)
							{
								TNM_SLINK* clink = cycleVector[ci];
								clink->oLinkPtr->oFlow = clink->oLinkPtr->oFlow - minflow;
								/*if (clink->oLinkPtr->oFlow < flowPrecision)
								{
								clink->oLinkPtr->oFlow = 0;
								}*/
								clink->volume = clink->volume - minflow;
								clink->cost = clink->GetCost();
								clink->fdCost = clink->GetDerCost();

							}

							numofCycleCreate++;
							cycleFlowCreate += minflow;
						}


						PotentialLinks.push_back(slink);


						//
						find = false;
						//iscycle = true;
						break;
					}
					else
					{
						//
						nodeLabel[Hnode->id - 1] = 1;
					}

				}
				else
				{

					find = false;
					break;
				}
			}

			if (fnum > network->numOfNode)
			{
				cout << "There is something wrong in searching new pas " << endl;
				cout << "The seg2 's size is : " << pas->seg2.size() << endl;
				system("PAUSE");
			}

		}

		if (find)
		{
			findNewPas = true;
			pas->begin = beginNode;

			//define seg1 following shortest path 
			pas->seg1.push_back(slink);
			TNM_SNODE* shNode = slink->tail;
			int endnum = 0;
			while (shNode != beginNode)
			{
				endnum++;
				TNM_SLINK* shLink = shNode->pathElem->via;
				pas->seg1.push_back(shLink);
				shNode = shLink->tail;
				if (endnum > network->numOfNode)
				{
					cout << "there is something wrong in defien seg1 " << endl;
					//system("PAUSE");
				}
			}
			//
			PairSegment2* epas = CheckExist(pas);
			if (epas == NULL)
			{
				PasList.push_back(pas);
				pas->Pid = PasList.size();
				int nid = endNode->id - 1;
				pas->active = true;
				PNList[nid]->AddPas(pas);// need to check the repeat here for pas
				goodpas = pas;
				NumOfNewPas++;
			}
			else
			{

				//addSubnet(pOrg,epas,1);
				//addSubnet(pOrg,epas,2);
				epas->addOrigin(pOrg);
				goodpas = epas;
				NumOfNoRegPas++;
				delete pas;
			}
		}
		else
		{
			delete pas;
		}

	}


	return goodpas;



}

//add and delete links according to the reduced cost
int TAP_iTAPAS::PrePareProportion()
{
	//
	int addnum = 0;
	int denum = 0;
	int wrnum = 0;
	int mnum = 0;
	int potential = 0;
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		pOrg->MarkOBLinksOnNet();

		network->UpdateSP(pOrg->origin);
		//
		for (int li = 0; li < network->numOfLink; li++)
		{
			TNM_SLINK* slink = network->linkVector[li];
			floatType rca = slink->tail->pathElem->cost + slink->cost - slink->head->pathElem->cost;
			if (rca <= 1e-8 && rca >= 0)
			{
				//
				if (slink->oLinkPtr == NULL)
				{
					ORGLINK* olink = new ORGLINK;
					olink->linkPtr = slink;
					slink->oLinkPtr = olink;
					olink->oFlow = 0;
					olink->approach = 0;
					pOrg->obLinkVector.push_back(olink);
					POList[pOrg->m_class]->olinkList[slink->id - 1] = olink;
					addnum++;
				}
			}

		}

		//check
		for (int li = 0; li < pOrg->obLinkVector.size(); li++)
		{
			ORGLINK* olink = pOrg->obLinkVector[li];
			TNM_SLINK* slink = olink->linkPtr;
			floatType rca = slink->tail->pathElem->cost + slink->cost - slink->head->pathElem->cost;
			if (rca > entropyCostPrecision && olink->oFlow > entropyFlowPrecision)
			{
				mnum++;
			}
			if (olink->oFlow < entropyFlowPrecision && rca < entropyCostPrecision && rca >= 0)
			{
				potential++;
			}
		}
		//

		pOrg->RemarkOBLinksOnNet();
	}

	cout << "The add num is : " << addnum << endl;
	cout << "The delete num is : " << denum << endl;
	cout << "The wrong num is:" << wrnum << endl;
	cout << "The first class link num is: " << mnum << endl;
	cout << "The second class num is " << potential << endl;
	//system("PAUSE");
	return 0;
}

//
floatType TAP_iTAPAS::ComputeConsistency()
{
	//
	int TotalSecond = 0;
	int NodeSecond = 0;
	int LinkSecond = 0;
	int unclear = 0;
	int connects = 0;
	int ZeroVolume = 0;

	floatType minRca = 1;
	floatType maxRca = 0;
	//
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		pOrg->MarkOBLinksOnNet();
		network->UpdateSP(pOrg->origin);

		for (int li = 0; li < network->numOfLink; li++)
		{
			TNM_SLINK* slink = network->linkVector[li];
			floatType rca = slink->tail->pathElem->cost + slink->cost - slink->head->pathElem->cost;
			ORGLINK* olink = slink->oLinkPtr;
			if ((olink == NULL || (olink != NULL && olink->oFlow < entropyFlowPrecision)) && rca > 0)
			{
				if (POList[pOrg->m_class]->onodeflow[slink->head->id - 1] > entropyFlowPrecision && slink->volume > entropyFlowPrecision && POList[pOrg->m_class]->oLinkLable[slink->id - 1] == false) //&& POList[pOrg->id()-1]->oLinkLable[slink->id-1]==false
				{
					if (minRca > rca)
					{
						minRca = rca;
					}
				}
			}
			else if (olink != NULL && olink->oFlow > entropyFlowPrecision)
			{
				if (maxRca < rca)
				{
					maxRca = rca;
				}


			}

			entropyCostPrecision = maxRca;

			if ((olink == NULL || (olink != NULL && olink->oFlow < entropyFlowPrecision)) && rca < entropyCostPrecision && rca >= 0) //&& POList[pOrg->id()-1]->onodeflow[slink->head->id-1] > 1e-12)  //
			{
				TotalSecond++;

				if (POList[pOrg->m_class]->onodeflow[slink->head->id - 1] < entropyFlowPrecision)
				{
					NodeSecond++;
				}
				else if (slink->volume < entropyFlowPrecision)
				{
					ZeroVolume++;
				}
				else if (slink->length == 0)
				{
					connects++;
					//
					/*cout<<"The Origin is : "<<pOrg->id()<<endl;
					cout<<"Link is : "<<slink->tail->id<<"-->"<<slink->head->id<<endl;
					cout<<"Oflow is : "<<slink->oLinkPtr->oFlow<<endl;
					cout<<"Total flow is : "<<slink->volume<<endl;
					cout<<"End flow is : "<<POList[pOrg->id()-1]->onodeflow[slink->head->id-1]<<endl;
					system("PAUSE");*/

				}
				else if (POList[pOrg->m_class]->oLinkLable[slink->id - 1])
				{


					LinkSecond++;

				}
				else
				{
					unclear++;

				}



			}


		}
		pOrg->RemarkOBLinksOnNet();
	}

	entropyCostPrecision = maxRca;
	//logFile<<"============================================"<<endl;
	logFile << "Total second class is " << TotalSecond << endl;
	logFile << "The zero node  is : " << NodeSecond << endl;
	logFile << "The single origin link is : " << LinkSecond << endl;
	logFile << "The connects link is " << connects << endl;
	logFile << "The unclear num is : " << unclear << endl;
	logFile << "The zero volume is : " << ZeroVolume++ << endl;
	logFile << "minRca is : " << minRca << endl;
	logFile << "maxRca is : " << maxRca << endl;



	cout << "============================================" << endl;
	cout << "Total second class is " << TotalSecond << endl;
	cout << "The zero node  is : " << NodeSecond << endl;
	cout << "The single origin link is : " << LinkSecond << endl;
	cout << "The connects link is " << connects << endl;
	cout << "The unclear num is : " << unclear << endl;
	cout << "The zero volume is : " << ZeroVolume++ << endl;
	cout << "minRca is : " << minRca << endl;
	cout << "maxRca is : " << maxRca << endl;
	//system("PAUSE");

	return 0;
}

//report
int TAP_iTAPAS::Report()
{
	intWidth = TNM_IntFormat::GetWidth();
	floatWidth = TNM_FloatFormat::GetWidth();
	if (reportIterHistory)
	{
		cout << "\tWriting the iteration history into file " << (outFileName + ".ite") << "..." << endl;
		ReportIter(iteFile);	
	}
	if (reportLinkDetail)
	{

		cout << "\tWriting link details into file " << (outFileName + ".lfp") << "..." << endl;
		ReportLink(lfpFile);

	}
	if (IsReportOrgFlows())
	{
		cout << "\tWriting origin-based flows into file " << (outFileName + ".ofw") << "..." << endl;
		ReportOrgFlows(m_orgFile);
	}
	
	if (IsReportPAS)
	{

		cout << "\tWriting PAS files into file " << (outFileName + ".pas") << "..." << endl;
		//ReportOrgFlows(m_orgFile);
		ReportPas(PasFile);


	}
	if (IsReportPAS)
	{
		cout << "\tWriting PAS information files into file " << (outFileName + ".pin") << "..." << endl;
		ReportPasInfo(PasInfoFile);
	}

	if (IsReportPAS)
	{
		cout << "\tWriting cycle information files into file " << (outFileName + ".cin") << "..." << endl;
		ReportCycleInfo(CycleInfoFile);
	}
	return 0;

}

void TAP_iTAPAS::ReportCycleInfo(ofstream& out)
{
	CycleInfoFile << "~ <Iteration> " << setw(15) << "<PreCycleNum>" << setw(15) << "<PreCycleFlow>" << setw(15) << "<InCycleNum>" << setw(15) << "InCycleFlow" << setw(15) << endl;
	for (int i = 0; i < CycleInList.size(); i++)
	{
		CycleInfo* ci = CycleInList[i];
		CycleInfoFile << setw(5) << ci->Iter << setw(15) << ci->PreCycles << setw(15) << ci->PreFlow << setw(15) << ci->InCycles << setw(15) << ci->InFlow << endl;
	}
}

void TAP_iTAPAS::ReportPasInfo(ofstream& out)
{
	PasInfoFile << "~ <Iteration>" << setw(23) << "<PasNum>" << setw(23) << "<MatchPas>" << setw(23) << "<NoRegistedPas>" << setw(23) << "NewPas " << setw(23) << "<DeletePas>" << setw(23) << "<TotalCreate>" << setw(23) << "AveRatio" << setw(23) << setw(23) << "<AveOrigin PerPas>" << setw(23) << "<Maximum Origin>" << setw(23) << " <AveLink>" << setw(23) << "<Maximum Link>" << setw(24) << "<NumOfPasShift>" << setw(25) << "<TotalFlowShift>" << endl;
	for (int i = 0; i < PasInfoList.size(); i++)
	{
		PasInfo* pi = PasInfoList[i];
		PasInfoFile << setw(5) << pi->Iter << setw(23) << pi->NumOfPas << setw(23) << pi->NumOfMatch << setw(23) << pi->NumOfFirstDelete << setw(23) << pi->NumOfNewPas << setw(23) << pi->NumOfdelete << setw(23) << pi->TotalPasCreate << setw(23) << pi->averageRatio << setw(23) << pi->NumOfAveOrigins << setw(23) << pi->MaxOriginPas << setw(23) << pi->NumOfLinksSeg << setw(23) << pi->MaxLinkSeg << setw(24) << pi->numOfShift << setw(25) << pi->TotalShiftFlow << setw(25) << pi->NumOfMatchOrigin << endl;
	}

}

void TAP_iTAPAS::ReportPas(ofstream& PasFile)
{
	//report the Pas
	PasFile << "Active PAS Num :  " << PasList.size() << endl;
	PasFile << "Average Origin Num for each PAS : " << AvePasOrg << endl;
	int nPas = 0;
	PairSegment2* pas;
	PasNode2* pn;
	for (int i = 0; i < PNList.size(); i++)
	{
		pn = PNList[i];
		for (int ij = 0; ij < pn->NPasList.size(); ij++)
		{
			nPas++;
			pas = pn->NPasList[ij];
			//pas = PasList[i];
			if (pas->active)
			{
				updatePasOrigin(pas);
				updatePasFlow(pas);
				pas->findHigher();
				PasFile << endl;
				PasFile << "PAS ID : " << pas->Pid << "  ," << setw(1) << "Proportion : " << TNM_FloatFormat(pas->proportion, 8, 6) << " , " << setw(1) << "MinTotalFlow 1 : " << TNM_FloatFormat(pas->minTotalFlow1, 8, 6) << " , " << setw(1) << "MinTotalFlow 2 : " << TNM_FloatFormat(pas->minTotalFlow2, 8, 6) << " ; " << endl;
				PasFile << "Segment Flow 1: " << TNM_FloatFormat(pas->flow1, 8, 6) << " , " << "Segment Flow 2: " << TNM_FloatFormat(pas->flow2, 8, 6) << " ; " << endl;
				PasFile << "Cost 1 : " << TNM_FloatFormat(pas->time1, 8, 6) << " , " << setw(1) << "Cost 2 : " << TNM_FloatFormat(pas->time2, 8, 6) << " , " << setw(1) << "Cost Diff(1-2) : " << TNM_FloatFormat(pas->time1 - pas->time2, 8, 6) << " ; " << " Pro Dif : " << TNM_FloatFormat(pas->prodif, 8, 6) << endl;
				PasFile << "Origins : ";
				for (int j = 0; j < pas->OriginList.size(); j++)
				{
					int oid = pas->OriginList[j]->org->id();
					PasFile << TNM_IntFormat(oid, 6) << " : " << TNM_FloatFormat(pas->OriginList[j]->proportion, 8, 6) << " ; " << setw(1);
				}
				PasFile << endl;
				PasFile << "Segment 1 : ";
				TNM_SLINK* sl = NULL;
				for (std::deque <TNM_SLINK*>::reverse_iterator pv = pas->seg1.rbegin(); pv != pas->seg1.rend(); pv++)
				{
					sl = *pv;
					if (sl == NULL)
					{
						cout << "There is error in Reporting PAS" << endl;
						system("PAUSE");
						break;
					}
					int tid = sl->tail->id;
					PasFile << TNM_IntFormat(tid, 4) << ",    ";
				}
				PasFile << TNM_IntFormat(pas->seg1.front()->head->id, 4) << endl;
				PasFile << "Segment 2 : ";
				for (std::deque <TNM_SLINK*>::reverse_iterator pv = pas->seg2.rbegin(); pv != pas->seg2.rend(); pv++)
				{
					sl = *pv;
					if (sl == NULL)
					{
						cout << "There is error in Reporting PAS" << endl;
						system("PAUSE");
						break;
					}
					int tid = sl->tail->id;
					PasFile << TNM_IntFormat(tid, 4) << ",    ";
				}
				PasFile << TNM_IntFormat(pas->seg2.front()->head->id, 4) << endl;
			}

		}
	}
	//PasFile<<"The total num of PNList pas is : "<<nPas<<endl;

}

//delete pas from pasList
int TAP_iTAPAS::deletePas()
{
	int num = 0;
	for (std::vector<PairSegment2*>::iterator pv = PasList.begin(); pv != PasList.end(); )
	{
		PairSegment2* pas = *pv;
		if (pas->active == false)
		{
			//delete pas 
			num++;
			pv = PasList.erase(pv);
			delete pas;
		}
		else
		{
			pv++;
		}
	}
	return num;
}

//eliminate pas
bool TAP_iTAPAS::eliminatePas(PairSegment2* pas)
{
	//cout<<"1"<<endl;
	//check the zShift
	if ((pas->OriginList.size() == 0) || ((pas->minTotalFlow1 < flowPrecision || pas->minTotalFlow2 < flowPrecision))) // && abs(pas->time2 - pas->time1) > costPrecision  )  // pas->zShift>200 ||
	{
		//cout<<"2"<<endl;
		//------------should be eliminated
		TNM_SNODE* endNode = pas->end;
		pas->active = false;
		//delete this pas from PNList
		PNList[endNode->id - 1]->ErasePas(pas);

		return true;
	}
	else
	{
		//cout<<"3"<<endl;
		//updatePasFlow(pas);
		//-----------check the origin should be eliminate from the originlist of pas
		for (std::vector<PasOrigin1*>::iterator pv = pas->OriginList.begin(); pv != pas->OriginList.end(); )
		{
			PasOrigin1* po1 = *pv;
			if ((po1->minFlow1 < flowPrecision || po1->minFlow2 < flowPrecision)) //&& abs(pas->time2 - pas->time1) > convIndicator/1000.0
			{
				pv = pas->OriginList.erase(pv);
				//delete the pas from the originlist
				int oid = po1->org->id() - 1;

				delete po1;
			}
			else
			{
				pv++;
			}
		}
		return false;
	}

}


//do flow shift for all the pas related with the origin
floatType TAP_iTAPAS::LocalPasShift(TNM_SORIGIN* pOrg)
{
	if (POList[pOrg->m_class]->OriginPtr != pOrg)
	{
		cout << "wrong in LocalPasShift of TAPAS" << endl;
		system("PAUSE");
	}
	PairSegment2* pas;
	if (curIter <= 2)
	{

		for (int j = 0; j < 1; j++)
		{
			for (int i = 0; i < POList[pOrg->m_class]->OPasList.size(); i++)
			{
				pas = POList[pOrg->m_class]->OPasList[i];
				if (pas != NULL && pas->active)
				{
					PasFlowShift(pas);
				}
			}
		}
	}

	if (curIter >= 2)
	{
		//cout<<"In Iteration "<<curIter<<endl;
		for (int i = 0; i < 400; i++)
		{
			int rnum = Random(0, PasList.size());
			//test
			//cout<<rnum<<endl;

			//
			pas = PasList[rnum];
			if (pas != NULL && pas->active)
			{
				PasFlowShift(pas);
			}
		}
	}
	//system("PAUSE");
	return 0;
}

int TAP_iTAPAS::Random(int start, int end)
{
	return start + 1.0 * (end - start) * rand() / (RAND_MAX + 1.0);
}

//generate the PASs for each origin
int TAP_iTAPAS::GeneratePAS(TNM_SORIGIN* pOrg)
{
	int bad = 0;
	BadLinks.clear();
	POList[pOrg->m_class]->OPasList.clear();
	if (POList[pOrg->m_class]->OriginPtr != pOrg)
	{
		cout << "wrong in generatePAS of iTAPAS" << endl;
		system("PAUSE");
	}
	//BranchList.clear();
	floatType rcs;
	//create a PAS for every link with positive reduced cost
	//for(std::vector<ORGLINK*>::iterator pv = pOrg->obLinkVector.begin();pv != pOrg->obLinkVector.end();pv++)

	while (PotentialLinks.size() > 0)
	{
		TNM_SLINK* slink = PotentialLinks.front();
		ORGLINK* olink = slink->oLinkPtr;
		PotentialLinks.pop_front();

		rcs = slink->tail->pathElem->cost + slink->cost - slink->head->pathElem->cost;
		/*if (rcs < 0)
		{
		cout<<slink->tail->id<<"-->"<<slink->head->id<<" : "<<rcs<<endl;
		cout<<"flow is "<<slink->oLinkPtr->oFlow<<endl;

		system("PAUSE");
		}*/
		//cout<<"slink is : "<<slink->tail ->id <<"-->"<<slink->head ->id <<endl;
		floatType pre;
		if (curIter == 1)
		{
			pre = 1e-1;
		}
		else if (curIter == 2)
		{
			pre = 1e-3;
		}
		else
		{
			pre = convIndicator / 100.0;
		}



		if (olink->oFlow > flowPrecision && rcs > pre)//should be in a pas  (olink->markStatus == -1 ) &&
		{
			//
			/*if (convIndicator < 1e-8)
			{
			cout<<"8";
			}*/

			//TotalExcessCost+=rcs*olink->oFlow;
			PairSegment2* mpas = matchPas(pOrg, olink);

			if (mpas != NULL)
			{
				//
				/*if (convIndicator < 1e-8)
				{
				cout<<"6";
				}*/
				NumOfMatch++;
				PasFlowShift(mpas);
				updatePasFlow(mpas);
				floatType rdc = slink->tail->pathElem->cost + slink->cost - slink->head->pathElem->cost;
				if (mpas->minTotalFlow1 <= flowPrecision || mpas->minTotalFlow2 <= flowPrecision)
				{
					PairSegment2* npas = createPas(pOrg, slink, 0.25);
					NumofRepeat++;
				}
			}
			else
			{
				//generate the pas
				/*if (convIndicator < 1e-8)
				{
				cout<<"7";
				}*/
				PairSegment2* npas = createPas(pOrg, slink, 0.25);


			}
		}

	}
	return bad;
}



PairSegment2* TAP_iTAPAS::createPas(TNM_SORIGIN* pOrg, TNM_SLINK* slink, floatType fe)
{

	//cout<<"create a new pas"<<endl;
	findNewPas = false;
	headend = false;
	repeated = false;
	iscycle = false;
	PairSegment2* goodpas = NULL;
	TNM_SNODE* origin = pOrg->origin;
	TNM_SNODE* endNode = slink->head;//the merge node of new pas
	TNM_SNODE* beginNode = NULL;
	deque<TNM_SNODE*> breadthList;//keep the node list in the breadth first search
	vector<int>nodeLabel(network->numOfNode, 0);
	//create a new pas

	PairSegment2* pas = new PairSegment2();
	pas->addOrigin(pOrg);
	pas->end = endNode;
	TNM_SNODE* rn;
	for (int i = 0; i < network->numOfNode; i++)
	{
		rn = network->nodeVector[i];
		//rn->rPathElem->via = NULL;
		rn->scanStatus = 0;
	}

	//find out the shortNodeList
	TNM_SNODE* node = endNode;
	while (node != origin)
	{
		TNM_SLINK* link = node->pathElem->via;

		//test
		/*cout<<"The shortest path is : "<<endl;
		cout<<"  "<<link->tail->id<<"-->"<<link->head->id;*/
		//LeastSegment->cycle.push_back(link);//
		if (link == NULL)
		{
			cout << "there is something wrong in finding the shortNodeList" << endl;
		}
		node = link->tail;
		node->scanStatus = 1;
	}
	//test
	/*cout<<"The slink is : "<<slink->tail->id<<"-->"<<slink->head->id<<endl;
	cout<<"The oflow of slink is : "<<slink->oLinkPtr->oFlow<<endl;
	system("PAUSE");*/
	//create the larger segment
	bool find = false;
	pas->seg2.push_back(slink);
	TNM_SNODE* Hnode = slink->tail;
	nodeLabel[Hnode->id - 1] = 1;
	nodeLabel[endNode->id - 1] = 1;
	int fnum = 0;

	while (!find)
	{
		//test
		//cout<<"new Hnode is "<<Hnode->id<<endl;
		//check if the pas is found
		if (Hnode->scanStatus == 1)
		{
			find = true;
			beginNode = Hnode;
			break;
		}
		else
		{
			TNM_SLINK* nextLink = NULL;
			floatType of = 0;
			for (int li = 0; li < Hnode->backStar.size(); li++)
			{
				TNM_SLINK* blink = Hnode->backStar[li];
				if (blink->oLinkPtr != NULL && of < blink->oLinkPtr->oFlow && blink->oLinkPtr->oFlow> flowPrecision) //&& nodeLabel[blink->tail->id-1] != 1
				{
					nextLink = blink;
					of = blink->oLinkPtr->oFlow;
				}
			}
			//put the next link into seg2
			if (nextLink != NULL)
			{
				//test 
				//cout<<"next link is : "<<nextLink->tail->id<<"-->"<<nextLink->head->id<<endl;
				//
				pas->seg2.push_back(nextLink);
				Hnode = nextLink->tail;
				//check if a cycle exist
				if (nodeLabel[Hnode->id - 1] == 1)
				{
					// a cycle exist
					deque<TNM_SLINK*> cycleVector;
					cycleVector = pas->seg2;
					while (cycleVector.front()->head != Hnode && cycleVector.size() > 0)
					{
						cycleVector.pop_front();
					}

					floatType minflow = 1e10;
					for (int si = 0; si < cycleVector.size(); si++)
					{
						TNM_SLINK* segl = cycleVector[si];

						if (segl->oLinkPtr->oFlow < minflow)
						{
							minflow = segl->oLinkPtr->oFlow;
						}

					}


					//remove the cycle flows
					if (minflow > 0)
					{
						for (int ci = 0; ci < cycleVector.size(); ci++)
						{
							TNM_SLINK* clink = cycleVector[ci];
							clink->oLinkPtr->oFlow = clink->oLinkPtr->oFlow - minflow;
							/*if (clink->oLinkPtr->oFlow < flowPrecision)
							{
							clink->oLinkPtr->oFlow = 0;
							}*/
							clink->volume = clink->volume - minflow;
							clink->cost = clink->GetCost();
							clink->fdCost = clink->GetDerCost();

						}

						numofCycleCreate++;
						cycleFlowCreate += minflow;
					}


					PotentialLinks.push_back(slink);


					//
					find = false;
					iscycle = true;
					break;
				}
				else
				{
					//
					nodeLabel[Hnode->id - 1] = 1;
				}

			}
			else
			{
				//cout<<"no link exist in the backstar of node "<<Hnode->id<<endl;
				//delete the flow
				floatType mf = 10000000;
				for (int li = 0; li < pas->seg2.size(); li++)
				{
					TNM_SLINK* ll = pas->seg2[li];
					if (ll->oLinkPtr->oFlow < mf)
					{
						mf = ll->oLinkPtr->oFlow;
					}
				}

				for (int li = 0; li < pas->seg2.size(); li++)
				{
					TNM_SLINK* ll = pas->seg2[li];
					ll->oLinkPtr->oFlow = ll->oLinkPtr->oFlow - mf;
					ll->volume = ll->volume - mf;
					ll->cost = ll->GetCost();
					ll->fdCost = ll->GetDerCost();
				}
				deFlow += mf;
				headend = true;
				find = false;
				break;
			}
		}

		if (fnum > network->numOfNode)
		{
			cout << "There is something wrong in searching new pas " << endl;
			cout << "The seg2 's size is : " << pas->seg2.size() << endl;
			system("PAUSE");
		}

	}

	//creat a new pas
	if (find)
	{
		findNewPas = true;
		pas->begin = beginNode;

		//define seg1 following shortest path 
		TNM_SNODE* shNode = endNode;
		int endnum = 0;
		while (shNode != beginNode)
		{
			endnum++;
			TNM_SLINK* shLink = shNode->pathElem->via;
			pas->seg1.push_back(shLink);
			shNode = shLink->tail;
			if (endnum > network->numOfNode)
			{
				cout << "there is something wrong in defien seg1 " << endl;
				//system("PAUSE");
			}
		}


		//add the subnet
		addSubnet(pOrg, pas, 1);
		pas->updatePasFlow();

		//a new PAS is created 
		NumOfCreatePas++;
		NumOfSegLink += pas->seg1.size();
		NumOfSegLink += pas->seg2.size();
		SegLinkFlowRatio += pas->minTotalFlow2 / slink->oLinkPtr->oFlow;
		//

		if (ImmediatePasShift(pas)) //&& (CheckExist(pas) == NULL))
		{

			PairSegment2* epas = CheckExist(pas);

			if (epas == NULL)
			{
				PasList.push_back(pas);
				PasID++;
				pas->Pid = PasID;
				int nid = endNode->id - 1;
				pas->active = true;
				PNList[nid]->AddPas(pas);// need to check the repeat here for pas
				goodpas = pas;
				NumOfNewPas++;
			}
			else
			{
				//epas->addOrigin(pOrg);
				//addSubnet(pOrg,epas,1);
				//addSubnet(pOrg,epas,2);
				goodpas = epas;
				NumOfNoRegPas++;
				delete pas;
			}

		}
		else
		{

			NumOfNoRegPas++;
			floatType rdc = slink->tail->pathElem->cost + slink->cost - slink->head->pathElem->cost;

			if (slink->buffer[1] < 100 && rdc > costPrecision && slink->oLinkPtr->oFlow > flowPrecision && pas->time2 > pas->time1)
			{
				PotentialLinks.push_back(slink);
				slink->buffer[1]++;
				NumofRepeat++;
				repeated = true;
			}



			delete pas;


		}
	}





	return goodpas;
}



//check if the new pas is exist
PairSegment2* TAP_iTAPAS::CheckExist(PairSegment2* npas)
{
	//
	bool exist = false;
	PairSegment2* exPas = NULL;
	npas->findHigher();
	TNM_SNODE* end = npas->end;
	PairSegment2* epas;
	PasNode2* pn = PNList[end->id - 1];
	for (int i = 0; i < pn->NPasList.size(); i++)
	{
		epas = pn->NPasList[i];
		epas->findHigher();
		if (epas->begin == npas->begin)
		{
			if ((epas->time1 == npas->time1 && epas->time2 == npas->time2) || (epas->time1 == npas->time2 && epas->time2 == npas->time1) || abs(epas->costDif) == abs(npas->costDif))
			{
				//
				exist = true;
				exPas = epas;
				break;
				//
				/*if (epas->active)
				{
				exist = true;
				exPas = epas;
				break;
				}
				else
				{
				PNList[epas->end->id-1]->ErasePas(epas);
				}*/
			}
		}
	}

	return exPas;

}



// shift flow immediately after the PAS is created
bool TAP_iTAPAS::ImmediatePasShift(PairSegment2* pas)
{
	floatType sf = 0;
	floatType pr = costPrecision;

	updatePasOrigin(pas);

	if (pas->OriginList.size() > 0)
	{
		//update pas flow
		updatePasFlow(pas);
		//update segment cost
		pas->findHigher();

		floatType sflow = pas->calShiftFlow();
		sf = doShiftFlow(pas);
		//update pas
		updatePasFlow(pas);
		pas->findHigher();

		if (abs(pas->minTotalFlow1) < flowPrecision || abs(pas->minTotalFlow2) < flowPrecision)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		return false;
	}
}


bool TAP_iTAPAS::addSubnet(TNM_SORIGIN* pOrg, PairSegment2* pas, int seg)//seg refer to the seg should be added
{
	TNM_SLINK* slink;
	//bool llag=false;
	if (seg == 1)
	{
		for (int i = 0; i < pas->seg1.size(); i++)
		{
			slink = pas->seg1[i];
			//add the link into the subnet
			if (slink->oLinkPtr == NULL)
			{
				ORGLINK* olink = new ORGLINK;
				olink->linkPtr = slink;
				olink->oFlow = 0.0;
				slink->oLinkPtr = olink;
				olink->markStatus = 0;//the added link is on the shortest path
				pOrg->obLinkVector.push_back(olink);
				POList[pOrg->m_class]->olinkList[slink->id - 1] = olink;
			}
		}
	}
	else if (seg == 2)
	{
		for (int i = 0; i < pas->seg2.size(); i++)
		{
			slink = pas->seg2[i];
			//add the link into the subnet
			if (slink->oLinkPtr == NULL)
			{
				ORGLINK* olink = new ORGLINK;
				olink->linkPtr = slink;
				olink->oFlow = 0.0;
				slink->oLinkPtr = olink;
				olink->markStatus = 1;//the added link is on the shortest path
				pOrg->obLinkVector.push_back(olink);
				POList[pOrg->m_class]->olinkList[slink->id - 1] = olink;
			}
		}
	}
	return true;
}



//flow shift in a pas
floatType TAP_iTAPAS::PasFlowShift(PairSegment2* pas)
{
	//cout<<"4"<<endl;
	floatType sf = 0;
	updatePasOrigin(pas);
	floatType ss = convIndicator / 1000.0;
	if (ss > 1e-3)
	{
		ss = 1e-3;
	}
	else if (convIndicator < 1e-9)
	{
		ss = convIndicator / 1000.0;
	}

	if (pas->OriginList.size() > 0 && pas->findHigher() > ss)//&& pas->findHigher() >ss
	{
		updatePasFlow(pas);
		floatType sflow = pas->calShiftFlow();
		sf = doShiftFlow(pas);

	}

	return sf;
}

//shift flow according to the shiftFlow
floatType TAP_iTAPAS::doShiftFlow(PairSegment2* pas)
{
	floatType totalflow = 0;
	if (pas->higher == 1)
	{
		//check if the shiftFlow is larger than the mintotalflow of segment 1
		if (pas->shiftFlow > pas->minTotalFlow1)
		{
			totalflow = pas->minTotalFlow1;
			if (totalflow <= flowPrecision)
			{
				pas->zShift++;
			}
			else
			{
				pas->zShift = 0;
				flowShift(pas, totalflow);
			}
		}
		else
		{
			totalflow = pas->shiftFlow;
			if (totalflow > flowPrecision)
			{
				pas->zShift = 0;
				flowShift(pas, totalflow);
			}
			else
			{
				pas->zShift++;
			}

		}
	}
	else if (pas->higher == 2)
	{
		if (pas->shiftFlow > pas->minTotalFlow2)
		{
			totalflow = pas->minTotalFlow2;
			if (totalflow <= flowPrecision)
			{
				pas->zShift++;
			}
			else
			{
				pas->zShift = 0;
				flowShift(pas, totalflow);
			}
		}
		else
		{
			totalflow = pas->shiftFlow;
			if (totalflow > flowPrecision)
			{
				pas->zShift = 0;
				flowShift(pas, totalflow);
			}
			else
			{
				pas->zShift++;
			}

		}
	}
	else
	{
		cout << "the higher is wrong in doshiftflow" << endl;
		//system("PAUSE");
	}
	return totalflow;
}


//shift flow for each origin in the list of PAS
floatType TAP_iTAPAS::flowShift(PairSegment2* pas, floatType tflow)
{
	if (pas->higher == 1)
	{
		for (std::vector<PasOrigin1*>::iterator pv = pas->OriginList.begin(); pv != pas->OriginList.end(); pv++)
		{
			bool del = false;
			PasOrigin1* po1 = *pv;
			int oid = po1->org->m_class;
			if (POList[oid]->OriginPtr != po1->org)
			{
				cout << "wrong in flowshift of TAPAS" << endl;
				system("PAUSE");
			}

			if (tflow == pas->minTotalFlow1)
			{
				po1->sflow = po1->minFlow1;
			}
			else
			{
				po1->sflow = (po1->minFlow1 / pas->minTotalFlow1) * tflow;

			}
			//update the zeroShift in pas origin
			if (po1->sflow <= flowPrecision)
			{
				po1->zeroShift++;
			}
			else
			{
				po1->zeroShift = 0;

				//reduce flow on seg1
				TNM_SLINK* link = NULL;
				ORGLINK* olink = NULL;
				int lid = 0;
				for (std::deque<TNM_SLINK*>::iterator pw = pas->seg1.begin(); pw != pas->seg1.end(); pw++)
				{
					link = *pw;
					//update the link total flow
					link->volume = link->volume - po1->sflow;

					lid = link->id - 1;
					olink = POList[oid]->olinkList[lid];
					if (olink == NULL)
					{
						cout << "link " << link->id << " 's olink is null for origin " << oid + 1 << endl;
						//system("PAUSE");
					}
					olink->oFlow = olink->oFlow - po1->sflow;
					//if(abs(olink->oFlow) < flowPrecision ) //&& abs(olink->oFlow) > 0
					//{

					//	olink->oFlow =0;

					//}

				}
				//add flow on seg2
				for (std::deque<TNM_SLINK*>::iterator pw = pas->seg2.begin(); pw != pas->seg2.end(); pw++)
				{
					link = *pw;
					//update the linktotoal flow
					link->volume = link->volume + po1->sflow;

					lid = link->id - 1;
					olink = POList[oid]->olinkList[lid];
					if (olink == NULL)
					{
						cout << "link " << link->id << " 's olink is null for origin " << oid + 1 << endl;
						system("PAUSE");
					}
					olink->oFlow = olink->oFlow + po1->sflow;

					/*if(abs(olink->oFlow) < flowPrecision && abs(olink->oFlow) > 0)
					{
					olink->oFlow =0;

					}*/
				}
				//
			}
		}
	}
	else if (pas->higher == 2)
	{
		for (std::vector<PasOrigin1*>::iterator pv = pas->OriginList.begin(); pv != pas->OriginList.end(); pv++)
		{
			bool del = false;
			PasOrigin1* po1 = *pv;
			int oid = po1->org->m_class;
			if (POList[oid]->OriginPtr != po1->org)
			{
				cout << "wrong in flowShift of TAPAS" << endl;
				system("PAUSE");
			}
			if (tflow == pas->minTotalFlow2)
			{
				po1->sflow = po1->minFlow2;
			}
			else
			{
				po1->sflow = (po1->minFlow2 / pas->minTotalFlow2) * tflow;

			}
			//update the zeroShift in pas origin
			if (po1->sflow <= flowPrecision)
			{
				po1->zeroShift++;
			}
			else
			{
				po1->zeroShift = 0;

				//reduce flow on seg1
				TNM_SLINK* link = NULL;
				ORGLINK* olink = NULL;
				int lid = 0;
				//delete flow on segment2
				for (std::deque<TNM_SLINK*>::iterator pw = pas->seg2.begin(); pw != pas->seg2.end(); pw++)
				{
					link = *pw;
					//update the total link flow
					link->volume = link->volume - po1->sflow;
					//link->cost = link->GetCost();
					//link->fdCost = link->GetDerCost();

					lid = link->id - 1;
					olink = POList[oid]->olinkList[lid];
					if (olink == NULL)
					{
						cout << "link " << link->id << " 's olink is null for origin " << oid + 1 << endl;
						//system("PAUSE");
					}
					olink->oFlow = olink->oFlow - po1->sflow;

					/*if(abs(olink->oFlow )< flowPrecision && abs(olink->oFlow) > 0 )
					{
					olink->oFlow =0;
					}*/
				}
				//add flow on seg1
				for (std::deque<TNM_SLINK*>::iterator pw = pas->seg1.begin(); pw != pas->seg1.end(); pw++)
				{
					link = *pw;
					//update the total flow
					link->volume = link->volume + po1->sflow;
					lid = link->id - 1;
					olink = POList[oid]->olinkList[lid];
					if (olink == NULL)
					{
						cout << "link " << link->id << " 's olink is null for origin " << oid + 1 << endl;
						system("PAUSE");
					}
					olink->oFlow = olink->oFlow + po1->sflow;
					/*if(abs(olink->oFlow) < flowPrecision && abs( olink->oFlow )> 0)
					{
					olink->oFlow =0;
					}*/
				}
			}
		}
	}
	else
	{
		cout << "shift flow between segments are wrong, please check the higher" << endl;
		system("PAUSE");
	}

	//update the link cost and fdcost for all the links in pas
	for (std::deque<TNM_SLINK*>::iterator pv = pas->seg1.begin(); pv != pas->seg1.end(); pv++)
	{
		TNM_SLINK* link = *pv;
		link->cost = link->GetCost();
		if (lineSearch == false)
		{
			link->fdCost = link->GetDerCost();
		}

	}
	for (std::deque<TNM_SLINK*>::iterator pv = pas->seg2.begin(); pv != pas->seg2.end(); pv++)
	{
		TNM_SLINK* link = *pv;
		link->cost = link->GetCost();
		if (lineSearch == false)
		{
			link->fdCost = link->GetDerCost();
		}

	}

	//
	NumOfTotalShift += 1;
	TotalFlowShift += tflow;

	return 0;
}

//update pas origin list
int TAP_iTAPAS::updatePasOrigin(PairSegment2* pas)
{
	int num = 0;
	//
	for (std::vector<PasOrigin1*>::iterator pv = pas->OriginList.begin(); pv != pas->OriginList.end(); )
	{
		PasOrigin1* po1 = *pv;
		TNM_SORIGIN* po = po1->org;
		int oid = po1->org->m_class;
		if (POList[oid]->OriginPtr != po)
		{
			cout << "wrong in updatePasOrigin of TAPAS" << endl;
			system("PAUSE");
		}
		//check if this pas is effective for this origin
		bool ef = true;
		for (int i = 0; i < pas->seg1.size(); i++)
		{
			TNM_SLINK* segLink = pas->seg1[i];
			if (POList[oid]->olinkList[segLink->id - 1] == NULL) //make sure that POList's olinkList is updated
			{
				//
				ef = false;
			}
		}
		for (int i = 0; i < pas->seg2.size(); i++)
		{
			//
			TNM_SLINK* segLink = pas->seg2[i];
			if (POList[oid]->olinkList[segLink->id - 1] == NULL)
			{
				ef = false;
			}
		}

		if (ef == false)
		{
			num++;
			pv = pas->OriginList.erase(pv);
			delete po1;
			//delete the pas from POList
			//POList[oid]->ErasePas(pas);
		}
		else
		{
			pv++;
		}
	}
	return num;
}
//find a exist pas for the olink
PairSegment2* TAP_iTAPAS::matchPas(TNM_SORIGIN* pOrg, ORGLINK* olink)
{
	PairSegment2* goodPas = NULL;
	TNM_SNODE* end = olink->linkPtr->head;
	TNM_SLINK* slink = olink->linkPtr;
	int id = end->id - 1;
	PasNode2* pn2 = PNList[id];
	floatType rca = slink->tail->pathElem->cost + slink->cost - slink->head->pathElem->cost;

	//scan every pas related to end node
	for (int i = 0; i < pn2->NPasList.size(); i++)
	{
		PairSegment2* pas = pn2->NPasList[i];
		if (pas->active)
		{
			//update the segment cost
			pas->findHigher();//very important

			if ((pas->getmlSeg1() == slink) && (pas->time1 - pas->time2 > costfactor * rca))
			{
				//for seg1 as used and seg2 as shortest
				updatePasFlow(pas);
				if (pas->minTotalFlow1 > slink->oLinkPtr->oFlow * flowfactor)//check if two segments are satisfied 
				{
					//check if all links are good to add to the subnet
					//addSubnet(pOrg,pas,2);//add the link of segment 2 into the subnetwork
					//pas->addOrigin(pOrg);//add the origin into the pas list
					goodPas = pas;//the pas is matched
					break;//find the pas
				}
				else
				{
					continue;//continue to find the pas
				}
			}
			else if ((pas->getmlSeg2() == slink) && (pas->time2 - pas->time1 > costfactor * rca))
			{
				//for seg2 as used and seg1 as shortest
				updatePasFlow(pas);
				if (pas->minTotalFlow2 > slink->oLinkPtr->oFlow * flowfactor)//seg2 is the used segment and it's good
				{
					//check if all links are good to add to the subnet
					//addSubnet(pOrg,pas,1);//add the links of seg1 into the subnet
					//pas->addOrigin(pOrg);
					goodPas = pas;//the pas is matched
					break;//find the pas
				}
				else
				{
					continue;//continue to find the pas
				}
			}
		}
		else
		{
			cout << "The pas is not in PNList when match a pas" << endl;
		}
		//end for PNList
	}

	return goodPas;
}


//update the flow in the pas
int TAP_iTAPAS::updatePasFlow(PairSegment2* pas)
{
	//update the origin based min flow for two segments

	int st = 1;
	for (std::vector<PasOrigin1*>::iterator pv = pas->OriginList.begin(); pv != pas->OriginList.end(); pv++)
	{
		PasOrigin1* po1 = *pv;
		TNM_SORIGIN* pOrg = po1->org;
		int oid = pOrg->m_class;
		if (POList[oid]->OriginPtr != pOrg)
		{
			cout << "wrong in updatePasFlow of TAPAS" << endl;
			system("PAUSE");
		}
		int lid = 0;
		ORGLINK* olink = NULL;
		//find the min flow for seg1 in subnet of pOrg
		po1->minFlow1 = 100000000000;

		for (std::deque<TNM_SLINK*>::iterator pw = pas->seg1.begin(); pw != pas->seg1.end(); pw++)
		{
			TNM_SLINK* link = *pw;
			lid = link->id - 1;
			olink = POList[oid]->olinkList[lid];

			//check if the olink is NULL
			if (olink == NULL)
			{
				cout << "\n\tlink " << lid << "of pas " << pas->Pid << " do not exist in the subnet of origin " << oid << endl;
				st = 0;
				system("PAUSE");
				break;
			}
			//
			if (po1->minFlow1 > olink->oFlow)
			{
				po1->minFlow1 = olink->oFlow;
			}

		}

		//find the min flow for seg2 in subnet of pOrg
		po1->minFlow2 = 100000000000;
		for (std::deque<TNM_SLINK*>::iterator pw = pas->seg2.begin(); pw != pas->seg2.end(); pw++)
		{
			TNM_SLINK* link = *pw;
			lid = link->id - 1;
			olink = POList[oid]->olinkList[lid];

			if (olink == NULL)
			{
				cout << "\n\tlink " << lid << "of pas " << pas->Pid << " do not exist in the subnet of origin " << oid << endl;
				st = 0;
				system("PAUSE");
				break;
			}
			//
			if (po1->minFlow2 > olink->oFlow)
			{
				po1->minFlow2 = olink->oFlow;
			}
		}
	}
	//test
	/*cout<<"minflow find ends"<<endl;
	system("PAUSE");*/
	//end the origin search
	//cal the sum of min flow
	pas->minTotalFlow1 = 0;
	pas->minTotalFlow2 = 0;
	for (std::vector<PasOrigin1*>::iterator pv = pas->OriginList.begin(); pv != pas->OriginList.end(); pv++)
	{
		PasOrigin1* po1 = *pv;
		pas->minTotalFlow1 += po1->minFlow1;
		pas->minTotalFlow2 += po1->minFlow2;
	}


	return st;
}


int TAP_iTAPAS::PrepareReport()
{
	

	if (IsReportOrgFlows())
	{
		string orgFileName = outFileName + ".ofw";
		if (!TNM_OpenOutFile(m_orgFile, orgFileName))
		{
			cout << "\n\tFail to prepare report: Cannot open .ofw file to write origin flows!" << endl;
			readyToReport = false;
			return 2;
		}
		PrintVersion(m_orgFile);
		//return 0;
	}

	if (reportLinkDetail)
	{
		string lfpFileName;
		
		lfpFileName = outFileName + ".lfp";
		
		if (!TNM_OpenOutFile(lfpFile, lfpFileName))
		{
			cout << "\n\tFail to Initialize an algorithm object: Cannot open .lfp file to write!" << endl;
			readyToReport = false;
			return 2;
		}
		PrintVersion(lfpFile);
		
	}
	

	if (reportIterHistory)
	{
		string iteFileName;
		
		iteFileName = outFileName + ".ite"; //out file
		

		if (!TNM_OpenOutFile(iteFile, iteFileName))
		{
			cout << "\n\tFail to build an algorithm object: cannot open .ite file to write!" << endl;
			return 2;
		}
		PrintVersion(iteFile);
	}
	intWidth = TNM_IntFormat::GetWidth();
	floatWidth = TNM_FloatFormat::GetWidth();
	readyToReport = true;
	//
	if (IsReportPAS)
	{
		string PasFileName;
		
		PasFileName = outFileName + ".pas";
		
		if (!TNM_OpenOutFile(PasFile, PasFileName))
		{
			cout << "\n\tFail to prepare report: Cannot open .pas file to write origin flows!" << endl;
			readyToReport = false;
			return 2;
		}
		PrintVersion(PasFile);
		//return 0;
	}

	if (IsReportPAS)
	{
		string PasInfoFileName;

		PasInfoFileName = outFileName + ".pin";
		
		if (!TNM_OpenOutFile(PasInfoFile, PasInfoFileName))
		{
			cout << "\n\tFail to prepare report: Cannot open .pin file to write origin flows!" << endl;
			readyToReport = false;
			return 2;
		}
		PrintVersion(PasInfoFile);
		//return 0;
	}

	if (IsReportPAS)
	{
		string CycleInfoFileName;

		CycleInfoFileName = outFileName + ".cin";
		
		if (!TNM_OpenOutFile(CycleInfoFile, CycleInfoFileName))
		{
			cout << "\n\tFail to prepare report: Cannot open .cin file to write origin flows!" << endl;
			readyToReport = false;
			return 2;
		}
		PrintVersion(CycleInfoFile);
		//return 0;
	}

	return 0;

}


//generate the path for the whole network
void TAP_iTAPAS::GeneratePathbyDFS()
{
	//clear the path set
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		for (int di = 0; di < pOrg->numOfDest; di++)
		{
			TNM_SDEST* sdest = pOrg->destVector[di];
			for (int pi = 0; pi < sdest->pathSet.size(); pi++)
			{
				TNM_SPATH* path = sdest->pathSet[pi];
				delete path;
			}
			sdest->pathSet.clear();
		}
	}
	//

	//delete the olinks
	NumOfCycle = 0;
	int numdelink = 0;
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		int oid = pOrg->m_class;

		//pOrg->MarkOBLinksOnNet();



		for (vector<ORGLINK*>::iterator pv = pOrg->obLinkVector.begin(); pv != pOrg->obLinkVector.end();)
		{
			ORGLINK* olink = *pv;
			if (olink->oFlow < 1e-6)
			{
				pv = pOrg->obLinkVector.erase(pv);
				delete olink;
				numdelink++;
			}
			else
			{
				pv++;
			}
		}
		//RemoveCycle(pOrg);
		//pOrg->RemarkOBLinksOnNet();

	}
	cout << "The total num of deleted o link is " << numdelink << endl;
	//cout<<"The number of cycle is "<<NumOfCycle<<endl;
	//update the approach
	CalLinkApproach();
	//
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		pOrg->MarkOBLinksOnNet();

		for (int di = 0; di < pOrg->numOfDest; di++)
		{
			TNM_SDEST* sdest = pOrg->destVector[di];

			if (sdest->assDemand > 1e-6 && sdest->dest != pOrg->origin)
			{
				//reset the node scanstatus
				for (int ni = 0; ni < network->numOfNode; ni++)
				{
					TNM_SNODE* node = network->nodeVector[ni];
					node->scanStatus = 0;
				}

				CNList.clear();
				CNList.push_back(sdest->dest);
				DFSPath(sdest->dest, pOrg, sdest);
				CNList.pop_back();
				//CNList.pop_back();
			}

			////test
			//double dedemand =0.0;
			//for (int pi=0;pi<sdest->pathSet.size();pi++)
			//{
			//	TNM_SPATH* path = sdest->pathSet[pi];
			//	dedemand+=path->flow;
			//}
			//if (sdest->assDemand -dedemand > 1e-5)
			//{
			//	cout<<"The origin is "<<pOrg->origin->id<<endl;
			//	cout<<"The destination is "<<sdest->dest->id<<endl;
			//	cout<<"assdemand is "<<sdest->assDemand<<endl;
			//	cout<<"dedemand is "<<dedemand<<endl;
			//	system("PAUSE");
			//}


		}



		pOrg->RemarkOBLinksOnNet();
	}
}

void TAP_iTAPAS::SavePathtoFile()
{
	//
	double totalPathFlow = 0.0;
	double totaldemand = 0.0;
	int pthid = 0;
	string pathFileName = outFileName + ".pth";
	ofstream out;
	cout << "\tWriting path information into " << outFileName << " for post process" << endl;
	if (!TNM_OpenOutFile(out, pathFileName))
	{
		cout << "\n\tFail to prepare report: Cannot open .oue file to write UE solution information!" << endl;
		return;
	}
	//
	cout << "\tWriting the paths info ..." << endl;
	out << "~path id      " << "origin       " << " dest      " << "     path flow    " << "   num of link    " << "   links  " << endl;
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		for (int di = 0; di < pOrg->numOfDest; di++)
		{
			TNM_SDEST* sdest = pOrg->destVector[di];
			totaldemand += sdest->assDemand;
			for (int pi = 0; pi < sdest->pathSet.size(); pi++)
			{
				TNM_SPATH* path = sdest->pathSet[pi];
				if (path->flow > 1e-6)
				{
					//output the path 
					pthid++;
					totalPathFlow += path->flow;
					//
					out << TNM_IntFormat(pthid, 4) << TNM_IntFormat(pOrg->id(), 4) << TNM_IntFormat(sdest->dest->id, 4) << TNM_FloatFormat(path->flow, 12, 6)
						<< TNM_IntFormat(path->path.size(), 4);
					for (int li = 0; li < path->path.size(); li++)
					{
						TNM_SLINK* link = path->path[li];
						out << TNM_IntFormat(link->id, 4);
					}
					out << endl;
				}
			}
		}
	}
	out << "~/////////////////////////////////////////////////////////////////////" << endl;
	out << "~The total path flow is: " << totalPathFlow << endl;
	out << "~The total demand is : " << totaldemand << endl;

	out.close();

}

//DFS method to find all the paths for the same destination
void TAP_iTAPAS::DFSPath(TNM_SNODE* serNode, TNM_SORIGIN* pOrg, TNM_SDEST* dest)
{
	//

	serNode->scanStatus = 1;
	TNM_SLINK* serLink;
	//serch backward of of serNode
	for (vector<TNM_SLINK*>::iterator pv = serNode->backStar.begin(); pv != serNode->backStar.end(); pv++)
	{
		serLink = *pv;
		//
		if (serLink != NULL && serLink->oLinkPtr != NULL && serLink->oLinkPtr->oFlow > 1e-6)
		{
			TNM_SNODE* tail = serLink->tail;

			if (tail == pOrg->origin)
			{
				//create a path
				CNList.push_back(tail);
				TNM_SPATH* path = new TNM_SPATH;
				TNM_SNODE* lhead;
				TNM_SNODE* ltail;
				for (int ci = 0; ci < CNList.size() - 1; ci++)
				{
					lhead = CNList[ci];
					ltail = CNList[ci + 1];
					//serch the link
					TNM_SLINK* sl;
					for (int j = 0; j < ltail->forwStar.size(); j++)
					{
						sl = ltail->forwStar[j];
						if (sl->oLinkPtr != NULL && sl->head == lhead)
						{
							path->path.push_back(sl);
						}
					}
				}
				dest->pathSet.push_back(path);
				////////////////test
				/*if (path->path.front()->head != dest->dest || path->path.back()->tail != pOrg->origin)
				{
				cout<<"This path is not complete"<<endl;
				system("PAUSE");
				}*/
				//
				floatType demand = dest->assDemand;
				for (int li = 0; li < path->path.size(); li++)
				{
					TNM_SLINK* ll = path->path[li];
					if (ll->oLinkPtr == NULL)
					{
						cout << "The olink is NULL in DFSPATH" << endl;
						system("PAUSE");
					}
					//
					demand = demand * ll->oLinkPtr->approach;
				}
				path->flow = demand;
				CNList.pop_back();
				//
				continue;//serch next node
			}
			else if (tail->scanStatus == 0)
			{
				CNList.push_back(tail);
				DFSPath(tail, pOrg, dest);
				CNList.pop_back();
				tail->scanStatus = 0;
			}
		}
	}

}

//cal the link approcah for the whole network
void TAP_iTAPAS::CalLinkApproach()
{
	//cout<<"2.1"<<endl;
	//calculate the approach 
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		int oid = pOrg->m_class;

		pOrg->MarkOBLinksOnNet();

		//RemoveCycle(pOrg);

		MarkobLink(pOrg, oi);//redefine the poList->olinklist according to the new oblinkvector
		//cout<<"2.2"<<endl;
		//floatType pdd = RemoveCycle(pOrg);
		//cout<<"pdd is "<<pdd<<endl;

		POList[oid]->onodeflow[pOrg->origin->id - 1] = 0;
		//update the node flow for this origin and approach proportion
		//vector<ORGNODE*>::iterator pw;
		TNM_SNODE* node;
		//cout<<"2.3"<<endl;
		//for (pw = pOrg->tplNodeVector.begin(); pw!=pOrg->tplNodeVector.end(); pw++)
		for (std::vector<TNM_SNODE*>::iterator pw = network->nodeVector.begin(); pw != network->nodeVector.end(); pw++)
		{
			node = (*pw);
			floatType app = 0.0, af = 0.0;
			if (node->m_orgNode != NULL && node != pOrg->origin)
			{
				for (PTRTRACE pl = node->backStar.begin(); pl != node->backStar.end(); pl++)
				{
					if ((*pl) && (*pl)->oLinkPtr && (*pl)->oLinkPtr->oFlow > entropyFlowPrecision)
					{
						af += (*pl)->oLinkPtr->oFlow;
					}
				}
				//update the onodeflow for each origin
				int nid = node->id - 1;
				POList[oid]->onodeflow[nid] = af;
				node->m_orgNode->potential = af;
				//ORGLINK
				//
				for (PTRTRACE pl = node->backStar.begin(); pl != node->backStar.end(); pl++)
				{
					if ((*pl) && (*pl)->oLinkPtr)
					{
						if (af > entropyFlowPrecision && (*pl)->oLinkPtr->oFlow > entropyFlowPrecision) (*pl)->oLinkPtr->approach = (*pl)->oLinkPtr->oFlow / af;
						else      (*pl)->oLinkPtr->approach = 0.0;

						if ((*pl)->oLinkPtr->approach < 0)
						{
							cout << "The approcach is negative" << endl;
							cout << "it is " << (*pl)->oLinkPtr->approach << endl;
							system("PAUSE");
						}
					}

				}
			}
		}

		pOrg->RemarkOBLinksOnNet();
	}
}
//
//mark used links which are also in the shortest path tree
void TAP_iTAPAS::MarkUsedLink(TNM_SORIGIN* pOrg)
{
	//clear the markstatus
	ORGLINK* oLink;
	TNM_SNODE* snode;
	TNM_SLINK* slink;
	int isoNode = 0;
	//update the shortest path tree
	network->UpdateSP(pOrg->origin);

	for (std::vector<ORGLINK*>::iterator pv = pOrg->obLinkVector.begin();
		pv != pOrg->obLinkVector.end(); pv++)
	{
		oLink = *pv;
		oLink->markStatus = -1;
	}

	//mark the used link
	for (int i = 0; i < network->numOfNode; i++)
	{
		snode = network->nodeVector[i];
		if (snode != pOrg->origin)
		{
			if (snode->pathElem->via != NULL)
			{
				slink = snode->pathElem->via;
				if (slink->oLinkPtr != NULL) //this link is used by the subnet
				{
					slink->oLinkPtr->markStatus = 1;//this used link is on the shortest path tree
				}
			}
			else
			{

				isoNode++;
				for (int lj = 0; lj < snode->forwStar.size(); lj++)
				{
					TNM_SLINK* islink = snode->forwStar[lj];
					if (islink->oLinkPtr != NULL)
					{
						islink->oLinkPtr->isolated = true;
					}
				}
			}
		}
	}
	//cout<<"isonodes is "<<isoNode<<endl;
	//system("PAUSE");
	//
	PotentialLinks.clear();
	int num = 0;
	//put the links with positive and without shortest path tree into 
	for (std::vector<ORGLINK*>::iterator pv = pOrg->obLinkVector.begin();
		pv != pOrg->obLinkVector.end(); pv++)
	{
		oLink = *pv;
		slink = oLink->linkPtr;
		floatType ec = slink->tail->pathElem->cost + slink->cost - slink->head->pathElem->cost;
		TotalExcessCost += ec * oLink->oFlow;
		if (oLink->markStatus == -1) //&& oLink->isolated == false && oLink->oFlow > flowPrecision
		{
			num++;
			PotentialLinks.push_back(oLink->linkPtr);
			oLink->linkPtr->buffer[1] = 0;
		}
	}
	//cout<<"num is "<<num<<endl;
	//system("PAUSE");
}

//remove cycles by DFS Algorithm
int TAP_iTAPAS::RemoveCycle(TNM_SORIGIN* pOrg)
{
	//
	TNM_SNODE* origin = pOrg->origin;
	TNM_SNODE* sNode;
	TNM_SLINK* sLink;

	CNList.clear();//clear the CNList

	for (int ni = 0; ni < network->numOfNode; ni++)
	{
		sNode = network->nodeVector[ni];
		sNode->scanStatus = 0;
		sNode->tmpNumOfIn = sNode->NumOfOrgInLink();
	}

	DFSCycle(origin, pOrg);



	return 0;
}


void TAP_iTAPAS::DFSCycle(TNM_SNODE* serNode, TNM_SORIGIN* pOrg)
{
	if (serNode->scanStatus == 1)
	{
		//check if head is in CNList
		TNM_SNODE* cn;
		vector<TNM_SNODE*> cycleNode;
		bool exist = false;

		for (deque<TNM_SNODE*>::iterator pw = CNList.begin(); pw != CNList.end(); pw++)
		{
			cn = *pw;
			if (cn == serNode)
			{
				exist = true;
			}
			if (exist)
			{
				cycleNode.push_back(cn);
			}
		}
		cycleNode.push_back(serNode);

		if (exist)
		{
			deque<TNM_SLINK*> cycleLink;
			TNM_SNODE* ltail;
			TNM_SNODE* lhead;
			for (int i = 0; i < cycleNode.size() - 1; i++)
			{
				ltail = cycleNode[i];
				lhead = cycleNode[i + 1];
				//search the link
				for (int j = 0; j < ltail->forwStar.size(); j++)
				{
					TNM_SLINK* sl = ltail->forwStar[j];
					if (sl->oLinkPtr != NULL && sl->head == lhead)
					{
						cycleLink.push_back(sl);
					}
				}
			}

			if (cycleLink.size() == cycleNode.size() - 1)
			{

				//delete the cycle
				//delete cycle flow
				TNM_SLINK* dlink;
				floatType dflow = 100000000000;
				for (int j = 0; j < cycleLink.size(); j++)
				{
					if (dflow > cycleLink[j]->oLinkPtr->oFlow)
					{
						dflow = cycleLink[j]->oLinkPtr->oFlow;
						dlink = cycleLink[j];
					}
				}
				if (dflow > flowPrecision)
				{
					//
					PreCycleFlow += dflow;
					NumOfCycle++;
					//deleting cycle flows
					for (int j = 0; j < cycleLink.size(); j++)
					{
						cycleLink[j]->oLinkPtr->oFlow = cycleLink[j]->oLinkPtr->oFlow - dflow;
						////
						//if(abs(cycleLink[j]->oLinkPtr->oFlow) < flowPrecision && abs(cycleLink[j]->oLinkPtr->oFlow)>0  )
						//{
						//	cycleLink[j]->oLinkPtr->oFlow =0;
						//}
						//link flow
						cycleLink[j]->volume = cycleLink[j]->volume - dflow;


					}

					//update the cost of cycle link
					for (int j = 0; j < cycleLink.size(); j++)
					{
						TNM_SLINK* cl = cycleLink[j];
						cl->cost = cl->GetCost();

						cl->fdCost = cl->GetDerCost();


					}

				}


			}
			return;
		}

		return;
	}
	//when no cycle exist
	serNode->scanStatus = 1;
	CNList.push_back(serNode);

	TNM_SLINK* serLink;
	//search the subnetwork
	for (vector<TNM_SLINK*>::iterator pv = serNode->forwStar.begin(); pv != serNode->forwStar.end(); pv++)
	{
		serLink = *pv;
		//
		if (serLink != NULL && serLink->oLinkPtr != NULL) //&& serLink->markStatus ==0
		{
			TNM_SNODE* head = serLink->head;
			DFSCycle(head, pOrg);
		}
	}

	CNList.pop_back();

}

int TAP_iTAPAS::MarkobLink(TNM_SORIGIN* pOrg, int idi)
{
	//int Oid = pOrg->id()-1;
	PasOrigin2* po = POList[pOrg->m_class];

	if (po != POList[pOrg->m_class])
	{

		cout << "po id is " << po->id << endl;
		cout << "pOrg id is " << POList[pOrg->m_class]->id << endl;
		system("PAUSE");
	}
	//make all elements as NULL
	for (int i = 0; i < network->numOfLink; i++)
	{
		po->olinkList[i] = NULL;
		network->linkVector[i]->oLinkPtr = NULL;
	}
	//redefine the olinkList according to the updated obLinkVector
	for (std::vector<ORGLINK*>::iterator pv = pOrg->obLinkVector.begin(); pv != pOrg->obLinkVector.end(); pv++)
	{
		ORGLINK* olink = *pv;
		if (olink == NULL)
		{
			cout << "olink is null" << endl;
			system("PAUSE");
		}
		olink->linkPtr->oLinkPtr = olink;
		int lid = olink->linkPtr->id - 1;
		po->olinkList[lid] = olink;
		if (olink->oFlow < flowPrecision)
		{
			olink->oFlow = 0;
		}

	}

	return 0;

}


floatType TAP_iTAPAS::CalEntropy()
{
	floatType entr = 0;
	//
	for (int oi = 0; oi < network->numOfOrigin; oi++)
	{
		TNM_SORIGIN* pOrg = network->originVector[oi];
		//pOrg->MarkOBLinksOnNet();
		//
		for (int ni = 0; ni < network->numOfNode; ni++)
		{
			TNM_SNODE* snode = network->nodeVector[ni];
			if (snode != pOrg->origin)
			{
				//
				floatType af = 0.0;
				for (int li = 0; li < snode->backStar.size(); li++)
				{
					if (POList[pOrg->m_class]->olinkList[snode->backStar[li]->id - 1] != NULL)
					{
						af += POList[pOrg->m_class]->olinkList[snode->backStar[li]->id - 1]->oFlow;
					}


				}
				POList[pOrg->m_class]->onodeflow[snode->id - 1] = af;
				//
				if (af > entropyFlowPrecision)
				{
					for (int li = 0; li < snode->backStar.size(); li++)
					{
						TNM_SLINK* slink = snode->backStar[li];
						if (POList[pOrg->m_class]->olinkList[slink->id - 1] != NULL && POList[pOrg->m_class]->olinkList[slink->id - 1]->oFlow > entropyFlowPrecision)//
						{
							entr += -POList[pOrg->m_class]->olinkList[slink->id - 1]->oFlow * log(POList[pOrg->m_class]->olinkList[slink->id - 1]->oFlow / af);
						}
					}
				}
			}

		}

		//pOrg->RemarkOBLinksOnNet();
	}
	return entr;
}