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
	case NETTAPAS:
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
		yPath->path.push_back(slink);
		
	}

}

floatType TNM_TAP::RelativeGap(bool scale)
{
	TNM_SLINK *link;
	TNM_SORIGIN *org;
	TNM_SDEST *dest;
	floatType gap, tt =0.0, tmd = 0.0;
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
			tmd += org->m_tdmd;
			for(int j = 0;j<org->numOfDest;j++)
			{
				dest = org->destVector[j];
				gap -= (dest->dest->pathElem->cost * dest->assDemand);
			}
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
		linkFile<<TNM_IntFormat(link->id)
				<<TNM_IntFormat(link->tail->id)
				<<TNM_IntFormat(link->head->id)
				<<TNM_FloatFormat(link->capacity)
				<<TNM_FloatFormat(link->volume,18,11)
				<<TNM_FloatFormat(link->cost)
				<<TNM_FloatFormat(link->toll)<<endl;
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
			//out<<TNM_IntFormat((*pv)->iter)
			//	<<TNM_FloatFormat((*pv)->ofv)
			//	<<TNM_FloatFormat((*pv)->conv)
			//	<<TNM_FloatFormat((*pv)->time)<<endl;
			out << TNM_IntFormat((*pv)->iter)
				<< TNM_FloatFormat((*pv)->ofv);

			out << setw(floatWidth)
				<< scientific << setprecision(4)
				<< (*pv)->conv
				<< fixed;

			out << TNM_FloatFormat((*pv)->time)<<endl;
		}
	}

}

////////////////////////////////////////////////////////////////////////////////
//Continuous VOT and theta(1/VOT) Distribution
////////////////////////////////////////////////////////////////////////////////

VOT_DIST::VOT_DIST()
{
	vot_min = 0;
	vot_max = 0;
	votdist_name = "uni";
}

VOT_DIST::VOT_DIST(floatType votmin, floatType votmax, string distname, int tree_number)
{
	cout<<"Enter build VOT_DIST"<<endl;
	vot_min = votmin;
	vot_max = votmax;
	votdist_name = distname;
	vot_intervallength = (vot_max - vot_min)/(tree_number - 1);
	treenumber = tree_number;
	if (distname == "first")
	{
		first_k = 5.0;
		first_c0 = 7.0; //((the_max + the_min)*k + 2c0)*(the_max - the_min)=2
		first_c1 = -1.0/2.0*first_k*pow(1.0/votmax, 2)- first_c0*1.0/votmax;
		//first_c1 = 1 - first_k/(2.0*pow(votmin, 2))- first_c0/votmin;
		if (((1.0/vot_max)*first_k + first_c0  < 0) || ((1.0/vot_min)*first_k + first_c0  < 0))
		{
			cout<<"It's not a correct distribution"<<endl;
			system("PAUSE");
		}
	}
	if (distname == "exp")
	{
		exp_lambda = 30.0;
	}
	if (distname == "rational")
	{
		floatType rho = 0; // the parameter that control the shape of rational distribution
		floatType theta_min = 1.0/votmax;
		floatType theta_max = 1.0/votmin;
		rational_a = (1 + rho)*(theta_max - theta_min);
		rational_b = -1*rho*theta_min + theta_max - theta_min;
		rational_rho = rho;
	}
}

VOT_DIST::~VOT_DIST()
{
}

void VOT_DIST::set_rho(floatType rhovalue)
{
	floatType theta_min = 1.0/vot_max;
	floatType theta_max = 1.0/vot_min;
	rational_a = (1 + rhovalue)*(theta_max - theta_min);
	rational_b = -1*rhovalue*theta_min + theta_max - theta_min;
	rational_rho = rhovalue;
}


floatType VOT_DIST::MMBA1_gtheta(floatType theta)
{
	floatType result;
	if (votdist_name == "uni")
	{
		result = 1.0/((vot_max - vot_min)*pow(theta, 2));
	}
	if (votdist_name == "first")
	{
		result = first_k*theta + first_c0;
	}
	if (votdist_name == "exp")
	{
		result = exp_lambda* exp(-1.0*exp_lambda*(theta - 1.0/vot_max));
	}
	if (votdist_name == "rational")
	{
		result = rational_a/(pow(rational_rho*theta + rational_b, 2));
	}
	return result;

}

floatType VOT_DIST::MMBA1_Gtheta(floatType theta)
{
	floatType result;
	if (votdist_name == "uni")
	{
		result = 1.0 - 1.0/(vot_max - vot_min)*(1.0/theta - vot_min);
	}
	if (votdist_name == "first")
	{
		result = first_k*pow(theta, 2)/2.0 + first_c0*theta + first_c1;
	}
	if (votdist_name == "exp")
	{
		result = (-1.0)*exp(-1.0*exp_lambda*(theta - 1.0/vot_max)) + 1.0;
	}
	if (votdist_name == "rational")
	{
		result = 1.0 + (theta-1.0/vot_min)/(rational_rho*theta + rational_b);
	}
	return result;
}

floatType VOT_DIST::MMBA1_gprimetheta(floatType theta)
{
	floatType result;
	if (votdist_name == "uni")
	{
		result = (-2.0)/(vot_max - vot_min)*pow(theta, -3);
	}
	if (votdist_name == "first")
	{
		result = first_k;
	}
	if (votdist_name == "exp")
	{
		result = (-1.0)*pow(exp_lambda, 2)*exp(-1.0*exp_lambda*(theta - 1.0/vot_max));
	}
	if (votdist_name == "rational")
	{
		result = -1 * (rational_a* rational_rho)/(pow(rational_rho*theta + rational_b, 3));
	}
	return result;
}

floatType VOT_DIST::MMBA1_Ptheta(floatType theta)//int theta\cdot g(\theta)
{
	floatType result;
	if (votdist_name == "uni")
	{
		result = 1.0/(vot_max - vot_min)*log(theta);
	}
	if (votdist_name == "first")
	{
		result = 1.0/3.0*first_k*pow(theta, 3)+1.0/2.0*first_c0*pow(theta, 2); // constant is ignore here.
	}
	if (votdist_name == "exp")
	{
		result =  ((-1.0)*theta - 1.0/exp_lambda)* exp(-1.0*exp_lambda*(theta - 1.0/vot_max));// constant is ignore here.
	}
	if (votdist_name == "rational")
	{
		if (fabs(rational_rho)<1e-10)
		{
			result = 1.0/(1.0/vot_min - 1.0/vot_max)*1.0/2.0*pow(theta, 2);
		}
		else
		{
			result = rational_a/pow(rational_rho, 2)*log(rational_rho*theta + rational_b) + rational_a * rational_b/pow(rational_rho, 2)*1.0/(rational_rho*theta + rational_b);
			// constant is ignore here.
		}
	}
	return result;
}



/////////////////////////////////////////////////////////////////////////////////////////////////
//Single-boundary adjustment algorithm(SBA) for continuous bi-criteria traffic assignment(C-BiTA)
/////////////////////////////////////////////////////////////////////////////////////////////////
// Constructors
TAP_SBA::TAP_SBA()
{
	useMPAstrategy = true;  // An indicator to control whether use the shortest path tree link-opt-in-opt-out strategy, refer Dial (1997).
	aveInnerGap = 50;  // A metric used in the main loop to control the adaptive inner loop intensity
	deletestrategy = 1; // Decide when to delete path, 1. delete after all boundaries are adjusted 
	                                                // 2. delete k/k+1 if zero flow after adjust theta_k 
	                                                // 3. k/k+1 if zero flow after adjust theta_{k+1}
	                                                // 4. a **preliminary** descending pass and ascending pass version
	armijo_rule = false;
}

// Constructors with inputs
TAP_SBA::TAP_SBA(floatType votmin1, floatType votmax1, string votname1, int treenumber1)
{
	cout<<"Enter TAP_SBA(): Single-boundary adjustment algorithm(SBA) for continuous bi-criteria traffic assignment(C-BiTA)"<<endl;
	votdist = new VOT_DIST(votmin1, votmax1, votname1, treenumber1);
	aveInnerGap = 1;
	useMPAstrategy = true;
	deletestrategy = 1; // Decide when to delete path, 1. delete after all boundaries are adjusted (it is what we used in the paper numerical test)
	// 2. delete k/k+1 if zero flow after adjust theta_k (delete potential efficient path)
	// 3. k/k+1 if zero flow after adjust theta_{k+1}
	// 4. a **preliminary** descending pass and ascending pass version
	armijo_rule = false; // if True, use armijo rule line search, if False, use constant step size (what we used in the paper numerical test)
}

// Destructor
TAP_SBA::~TAP_SBA()
{
	cout<<"delete TAP_SBA()"<<endl;
}


void TAP_SBA::init_pathElemVector(int column_generation_num)
{
	TNM_SNODE *node;
	for(int i=0; i<network->numOfNode; i++)
	{
		node = network->nodeVector[i];
		for(int j=0; j<column_generation_num; j++)
		{
			PATHELEM* pathelem = new PATHELEM;
			node->pathElemVector.push_back(pathelem);
		}
	}
}


floatType TAP_SBA::setdecimal_lower(floatType fl,int num)
{
	floatType b;
	switch(num)
	{
	case 0:
		cout<<"Please select a positive integer"<<endl;
		return FALSE;
	case 1:
		b=(floor)((fl*10))/10.0;
		break;
	case 2:
		b=(floor)((fl*100))/100.0;
		break;
	case 3:
		b=(floor)((fl*1000))/1000.0;
		break;
	case 4:
		b=(floor)((fl*10000))/10000.0;
		break;
	case 5:
		b=(floor)((fl*100000))/100000.0;
		break;
	case 6:
		b=(floor)((fl*1000000))/1000000.0;
		break;
	case 7:
		b=(floor)((fl*10000000))/10000000.0;
		break;
	case 8:
		b=(floor)((fl*100000000))/100000000.0;
		break;
	case 9:
		b=(floor)((fl*1000000000))/1000000000.0;
		break;
	case 10:
		b=(floor)((fl*10000000000))/10000000000.0;
		break;
	default:
		b=(floor)((fl*10000000000))/10000000000.0;
		cout<<"The large number is 10"<<endl;
	}
	return b;
}


floatType TAP_SBA::setdecimal(floatType fl,int num)
{
	floatType b;
	switch(num)
	{
	case 0:
		cout<<"Please select a positive integer"<<endl;
		return FALSE;
	case 1:
		b=(ceil)((fl*10))/10.0;
		break;
	case 2:
		b=(ceil)((fl*100))/100.0;
		break;
	case 3:
		b=(ceil)((fl*1000))/1000.0;
		break;
	case 4:
		b=(ceil)((fl*10000))/10000.0;
		break;
	case 5:
		b=(ceil)((fl*100000))/100000.0;
		break;
	case 6:
		b=(ceil)((fl*1000000))/1000000.0;
		break;
	case 7:
		b=(ceil)((fl*10000000))/10000000.0;
		break;
	case 8:
		b=(ceil)((fl*100000000))/100000000.0;
		break;
	case 9:
		b=(ceil)((fl*1000000000))/1000000000.0;
		break;
	case 10:
		b=(ceil)((fl*10000000000))/10000000000.0;
		break;
	case 11:
		b=(ceil)((fl*100000000000))/100000000000.0;
		break;
	case 12:
		b=(ceil)((fl*1000000000000))/1000000000000.0;
		break;
	case 13:
		b=(ceil)((fl*10000000000000))/10000000000000.0;
		break;
	default:
		b=(ceil)((fl*10000000000000))/10000000000000.0;
		cout<<"The large number is 13"<<endl;
	}
	return b;
}


floatType TAP_SBA::setdecimal_round(floatType fl,int num)
{
	floatType b;
	switch(num)
	{
		case 7:
			b=(floor)((fl*10000000+0.5))/10000000.0;
			break;
		case 8:
			b=(floor)((fl*100000000+0.5))/100000000.0;
			break;
		case 9:
			b=(floor)((fl*1000000000+0.5))/1000000000.0;
			break;
		case 10:
			b=(floor)((fl*10000000000+0.5))/10000000000.0;
			break;
		case 11:
			b=(floor)((fl*100000000000+0.5))/100000000000.0;
			break;
		case 12:
			b=(floor)((fl*1000000000000+0.5))/1000000000000.0;
			break;
		case 13:
			b=(floor)((fl*10000000000000+0.5))/10000000000000.0;
			break;
		case 14:
			b=(floor)((fl*100000000000000+0.5))/100000000000000.0;
			break;
		case 15:
			b=(floor)((fl*1000000000000000+0.5))/1000000000000000.0;
			break;
		case 16:
			b=(floor)((fl*10000000000000000+0.5))/10000000000000000.0;
			break;
	default:
		b=(floor)((fl*10000000+0.5))/10000000.0;
		cout<<"The lowest number is 7 and the largest number is 16. Use the default number equals 7."<<endl;
	}
	return b;
}


int TAP_SBA::Generalized_cost_spp(TNM_SNODE * rootNode,vector<floatType> &gv)
{
	deque<TNM_SNODE*> nl;  // call it node queue, used for shortest path
	TNM_SNODE *node;
	//initialize node for shortest path calculation
	for (int j = 0;j<network->numOfNode;j++)
	{
		node = network->nodeVector[j];
		node->InitPathElem();  // initialize for the shortest path problem
		node->scanStatus = 0;  // node scan status
		node->buffer[0] = 0; // reset time  from the origin to the current node on the shortest path
		node->buffer[1] = 0; // reset toll from the origin to the current node on the shortest path
	}
	TNM_SNODE *curNode;
	rootNode->pathElem->cost = 0;  // set the cost of the origin node to 0
	nl.push_back(rootNode);  // add origin to the node queue
	rootNode->buffer[8] += 1;
	if (!nl.empty()) //if the list is not empty
	{
		curNode = nl.front(); // get the current node from the beginning of the node queue
		nl.pop_front(); //delete it from the deque;
		curNode->scanStatus = -1; //mark it has been searched but not in queue now.
	}
	else
		curNode = NULL;
	while (curNode != NULL)
	{
		if(curNode->m_isThrough || curNode == rootNode)  // can connect to curNode or it is the origin
		{
		   	LINK_COST_TYPE dp, curTT;
			vector<TNM_SLINK *>::iterator pv;
			TNM_SNODE *scanNode;
			for (pv = curNode->forwStar.begin();pv!=curNode->forwStar.end();pv++)
			{
				if(*pv!=NULL)
				{
					scanNode = (*pv)->head; // get upstream node
					int linkID = (*pv)->id;
					curTT    = gv[linkID-1]; //get link cost
					dp       = curNode->pathElem->cost; //get the label of current node
					if(scanNode->pathElem->cost > curTT + dp)
					{ // if so, update the routing table
						scanNode->pathElem->cost = curTT + dp;
						if (scanNode->pathElem->via != NULL)
						{
							scanNode->pathElem->via->buffer[6] = 0; // mark the current via link out of tree
						}
						scanNode->pathElem->via  = *pv;  // update via link
						scanNode->pathElem->via->buffer[6] = 1; // mark the new via link in tree
						scanNode->buffer[0] = curNode->buffer[0] + (*pv)->cost; // add the node time
						scanNode->buffer[1] = curNode->buffer[1] + (*pv)->toll; // add the node toll
						//InsertANode()
    					if(scanNode->scanStatus == 0) // if never been searched, insert it to the back of node queue
						{
							nl.push_back(scanNode); 
							rootNode->buffer[8] += 1;
							scanNode->scanStatus = 1; // now being used
						}
						else if (scanNode->scanStatus == -1) // if it has ever been searched, insert it to the front of node queue
						{
							nl.push_front(scanNode); 
							rootNode->buffer[8] += 1;
							scanNode->scanStatus = 1; // now being used
						}
					}
				}
			}
	   }
	   if (!nl.empty()) //if the node queue is not empty
	   {
		   curNode = nl.front();  // get the current node from the beginning of the node queue
		   nl.pop_front();  //delete it from the deque
		   curNode->scanStatus = -1;  //mark it has been used but not in queue now
	   }
	   else
		   curNode = NULL;
   }
	return 0;
}


TNM_SPATH* TAP_SBA::revert_SPath(TNM_SNODE *origin, TNM_SNODE *dest)
{
	TNM_SNODE *node;
	TNM_SLINK *link;
	int count = 0;
	TNM_SPATH *path = new TNM_SPATH; 
	assert(path!=0);
	node = dest;
	while (node != origin)
	{
		for (int li=0; li<node->backStar.size(); li++)
		{
			if (node->backStar[li]->buffer[6] == 1)  //This backStar link is in the tree
			{
				link = node->backStar[li];
				break;
			}
		}
		if (link == NULL)
		{
			cout<<"\tno shortest path found at node "<<node->id<<endl;
			delete path;
			return NULL;
		}
    	node = link->tail;  // trace one more node
		path->path.push_back(link);  //collect link
		count++;  // update link elements count
		// cycle check
		if(count>network->numOfNode)
		{
			cout<<"Error: shortest path tree contains a cycle!"<<endl;
			cout<<"Report: origin = "<<origin->id<<" dest = "<<dest->id<<" path be generated so far "<<endl;
			delete path;
			return NULL;
		}
	}
	// initialize the created path
	vector<TNM_SLINK*>(path->path).swap(path->path);
	reverse(path->path.begin(), path->path.end());
	path->cost = 0.0;
	path->buffer[0] = 0.0;
	for(int i=0; i<path->path.size();i++)
	{
		path->cost += path->path[i]->cost;
		path->buffer[0] += path->path[i]->toll;
	}
	return path;
}



void TAP_SBA::update_pathlinkflow(TNM_SPATH* pa, floatType addvalue)
{
	for (int j=0; j<pa->path.size(); j++)
	{
		pa->path[j]->volume += addvalue;
	}
}


void TAP_SBA::printpath(TNM_SPATH* pa)
{
	cout<<pa->path[0]->tail->id<<"->";
	for (int pi=0;pi<pa->path.size(); pi++)
	{
		cout<<pa->path[pi]->head->id<<"->";
	}
	cout<<endl;
}


// Initialization for the SBA algorithm
void TAP_SBA::Initialize()
{
	cout<<"Enter TAP_SBA Initialize()"<<endl;
	// initialization for shortest path
	sumo_path = 0.0;
	// Count initialize
	periter_addpathcount = 0;
	periter_deletepathcount = 0;
	total_pathset_size = 0;
	max_pathset_size = 0;
	total_shift_flow = 0.0;
	spptreenum = 0;
	// Count time initialize
	withoutgaptime = 0.0;
	shortestpathtime = 0.0;
	truespptime = 0.0;
	ltatime = 0.0;
	ltachecktime = 0.0;
	// Initialize theta range
	theta_min = 1.0/votdist->vot_max;
	theta_max = 1.0/votdist->vot_min;
	network->UpdateLinkCost();//update cost
	// Allocate buffer
	network->AllocatePathBuffer(4); //buffer[0] path toll; buffer[1] add path status(is new add or not); buffer[2] path lower bound; buffer[3] path upper bound
	network->AllocateLinkBuffer(9); // allocate float memory for link variables, 0 is divide theta, 1 is u assign to the link, 2 is new x, 3 is new u
	//4 is origin-based new flow, 5 is origin-based new u, 6 is link label: 0 is not in the tree, 1 is in the tree; 7 is x_link, 8 is u_link(LTA);
	network->AllocateNodeBuffer(9); //allocate float memory for node variables, 0 is node time(s-node), 1 is node toll(d-node); 2 (MPA from scrach) node is or is not already added in to Dset/(PTU case)node in H intialset or not; nodeHsetinit,3 is u_node(LTA);
	//4 is a_node(LTA): boundary that already assign; 5 is d_x(LTA), 6 is d_u(LTA);7 is topological order 8 is how many nodes total in the H set
	//network->AllocateDestBuffer(2);
	// Pre-process
	map_node_dest();  // map the node and dest obj in the network and get node_dest
	TNM_SORIGIN* pOrg;
	TNM_SDEST* dest;
	floatType sumlink = 0.0;  // use to get the network total generalized cost (for relative gap calculation) of the current pattern
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		for(int li =0 ;li<network->numOfLink;li++)
		{
			network->linkVector[li]->buffer[4] = 0;  // initialize origin-based new flow x
			network->linkVector[li]->buffer[5] = 0;  // initialize origin-based one-moment u
		}
		floatType oldtreenum = spptreenum; // record the cumulative shortest path tree num at last origin
		// Here provide two strategies: 
		// when useMPAstrategy is true - use Dial(1997) tree link opt-in opt-out PTU tree replacement strategies to change from one tree and get the next
		//                     is false - call shortest path algorithm to get the next shortest path tree from scratch
		if (useMPAstrategy == true)
		{
			Dial_MPA_path(pOrg, 1, 1);
		}
		else
		{
			MPA_path_scrach(pOrg, 1, 1);
		}
		tree_num.push_back(spptreenum - oldtreenum);  //save tree num for this origin
		// The following part is for checking.
		for (int j=0; j<pOrg->numOfDest; j++)
		{
			dest = pOrg->destVector[j];
			floatType sumdemand = 0;
			total_pathset_size += dest->pathSet.size();
			if (dest->pathSet.size()>max_pathset_size)
			{
				max_pathset_size = dest->pathSet.size();
			}
			for(int k=0; k<dest->pathSet.size(); k++)
			{
				sumlink += dest->assDemand * dest->pathSet[k]->buffer[0] * (
					votdist->MMBA1_Ptheta(dest->pathSetbound[k+1]) - 
					votdist->MMBA1_Ptheta(dest->pathSetbound[k]));
				sumlink += dest->pathSet[k]->PathCostS() * dest->pathSet[k]->flow;
				sumdemand += network->originVector[i]->destVector[j]->pathSet[k]->flow;
				// same toll path checking
				if (k >= 1 && network->originVector[i]->destVector[j]->pathSet[k]->buffer[0]==network->originVector[i]->destVector[j]->pathSet[k-1]->buffer[0])
				{
					cout<<"Attention! There is path have same money cost!"<<
						network->originVector[i]->origin->id<<"->"<<
						network->originVector[i]->destVector[j]->dest->id<<endl;
					TNM_SPATH* pathk = network->originVector[i]->destVector[j]->pathSet[k];
					TNM_SPATH* pathk_1 = network->originVector[i]->destVector[j]->pathSet[k-1];
					printpath(pathk);
					cout<<k<<" toll="<<pathk->buffer[0]<<
						"time = "<<pathk->PathCostS()<<
						"lower bound="<<pathk->buffer[2]<<
						"upper bound="<<pathk->buffer[3]<<
						"boundary value difference="<<pathk->buffer[3] - pathk->buffer[2]<<endl;
					printpath(pathk_1);
					cout<<k-1<<" toll="<<pathk_1->buffer[0]<<
						"time = "<<pathk_1->PathCostS()<<
						"lower bound="<<pathk_1->buffer[2]<<
						"upper bound="<<pathk_1->buffer[3]<<
						"boundary value difference="<<pathk_1->buffer[3] - pathk_1->buffer[2]<<endl;
					cout<<"path time difference = "<<pathk_1->buffer[0] - pathk->buffer[0]<<endl;
					system("PAUSE");
				}
				// Monetary cost order and boundary order checking
				if (k>=1)
				{
					if (network->originVector[i]->destVector[j]->pathSetbound[k] - network->originVector[i]->destVector[j]->pathSetbound[k-1] < 0)
					{
						cout<<"ERROR: path set bound order is wrong!"<<
							network->originVector[i]->origin->id<<"->"<<
							network->originVector[i]->destVector[j]->dest->id<<endl;
					}
					if (network->originVector[i]->destVector[j]->pathSet[k]->buffer[0] - network->originVector[i]->destVector[j]->pathSet[k-1]->buffer[0] > 0)
					{
						cout<<"ERROR: toll order is wrong!"<<
							network->originVector[i]->origin->id<<"->"<<
							network->originVector[i]->destVector[j]->dest->id<<endl;
						TNM_SPATH* pathk = network->originVector[i]->destVector[j]->pathSet[k];
						TNM_SPATH* pathk_1 = network->originVector[i]->destVector[j]->pathSet[k-1];
						TNM_SPATH* path0 = network->originVector[i]->destVector[j]->pathSet[0];
						printpath(pathk);
						cout<<k<<" toll="<<pathk->buffer[0]<<
							"time = "<<pathk->PathCostS()<<
							"lower bound="<<pathk->buffer[2]<<
							"upper bound="<<pathk->buffer[3]<<
							"boundary value difference="<<
							pathk->buffer[3] - pathk->buffer[2]<<endl;
						printpath(pathk_1);
						cout<<k-1<<" toll="<<pathk_1->buffer[0]<<
							"time = "<<pathk_1->PathCostS()<<
							"lower bound="<<pathk_1->buffer[2]<<
							"upper bound="<<pathk_1->buffer[3]<<
							"boundary value difference="<<
							pathk_1->buffer[3]-pathk_1->buffer[2]<<endl;
						printpath(path0);
						cout<<0<<" toll="<<path0->buffer[0]<<
							"time = "<<path0->PathCostS()<<
							"lower bound="<<path0->buffer[2]<<
							"upper bound="<<path0->buffer[3]<<
							"boundary value difference="<<
							path0->buffer[3]-path0->buffer[2]<<endl;
						cout<<"path time difference = "<<pathk_1->buffer[0] - pathk->buffer[0]<<endl;
						cout<<"toll difference ="<<pathk->buffer[0]-pathk_1->buffer[0]<<endl;
						system("PAUSE");

					}
				}
			}
			// Flow conservation check
			if (fabs(sumdemand - network->originVector[i]->destVector[j]->assDemand) > 1e-6)
			{
				cout<<"Error: violate demand happens at"<<"o:"<<network->originVector[i]->origin->id<<"->"<<
					"d:"<<network->originVector[i]->destVector[j]->dest->id<<
					"sum path flow - demand = "<<
					sumdemand - network->originVector[i]->destVector[j]->assDemand<<endl;
			}
		}
	}
	// Update cost
	network->UpdateLinkCost();
	withoutgaptime += (clock() - m_startRunTime);  //Cumulate time that is not belong to the Gap calculation
	//******************************************************
	//cout<<"Without Gap Time"<<1.0*(withoutgaptime)/CLOCKS_PER_SEC<<endl;
	//cout<<"Total add path is "<<periter_addpathcount<<" delete path is "<<periter_deletepathcount<<endl;
	//cout<<"Avg OD pathset size is "<<double(total_pathset_size)/double(network->numOfOD)<<" max path set size is "<<max_pathset_size<<endl;
	//******************************************************
	if (network->networkName.find("win")!=string::npos)
	{
		useMPAstrategy = false;
	}
	sumlink = 0.0;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		for (int j=0; j<pOrg->numOfDest; j++)
		{
			dest = pOrg->destVector[j];
			for(int k=0; k<dest->pathSet.size(); k++)
			{
				sumlink += dest->assDemand * dest->pathSet[k]->buffer[0] * (
					votdist->MMBA1_Ptheta(dest->pathSetbound[k+1]) - 
					votdist->MMBA1_Ptheta(dest->pathSetbound[k]));
				sumlink += dest->pathSet[k]->PathCostS() * dest->pathSet[k]->flow;
			}
		}
	}
	sumo_path = 0.0; //make sumo_path back to 0 and it will updated in SBA_GAP
	SBA_GAP();
	// print for numerical example
	//TNM_SPATH* path_p;
	//TNM_SORIGIN* origin_p;
	//TNM_SDEST* dest_p;
	//for (int ori = 0;ori<network->numOfOrigin;ori++)
	//{
	//	origin_p = network->originVector[ori];
	//	for (int dj=0; dj<origin_p->numOfDest; dj++)
	//	{
	//		dest_p = origin_p->destVector[dj];
	//		cout<<"origin:"<<dest_p->origin->id<<"->"<<"dest:"<<dest_p->dest->id<<endl;
	//		for(int pk=0; pk<dest_p->pathSet.size(); pk++)
	//		{
	//			path_p = dest_p->pathSet[pk];
	//			printpath(path_p);
	//			cout<<"path flow"<<path_p->flow<<" path time = "<<path_p->PathCostS()<<endl;
	//			cout<<"lower bound = "<<path_p->buffer[2]<<" upper bound="<<path_p->buffer[3]<<endl;
	//		}
	//	}
	//}
}


void TAP_SBA::map_node_dest()
{
	for (int i=0; i<network->numOfOrigin; i++)
	{
		TNM_SORIGIN* ori = network->originVector[i];
		map<int, TNM_SDEST*> origin_node_dest;
		for (int n=0; n<network->numOfNode; n++)
		{
			origin_node_dest.insert(pair<int, TNM_SDEST*>(network->nodeVector[n]->id, NULL));
		}

		for (int j=0; j<ori->numOfDest; j++)
		{
			TNM_SDEST* dest = ori->destVector[j];
			origin_node_dest[dest->dest->id] = dest;
			//origin and dest are same
			if (dest->dest ==  dest->origin && dest->assDemand != 0)
			{
				dest->pathSet.push_back(revert_SPath(dest->origin, dest->dest));
				dest->pathSet[0]->flow = dest->assDemand;
				dest->pathSet[0]->buffer[2] = theta_min;
				dest->pathSet[0]->buffer[3] = theta_max;
				dest->pathSetbound.push_back(theta_min);
				dest->pathSetbound.push_back(theta_max);
			}
		}
		node_dest.insert(make_pair(ori->origin->id, origin_node_dest));
	}
}


void TAP_SBA::SBA_obj()
{
	// just calculate the objective function value in SBA
	floatType	sumobjf= 0.0;
	floatType	sumobjs= 0.0;
	floatType	odobjs= 0.0;
	TNM_SORIGIN *pOrg;
	TNM_SDEST* dest;
	// calculate the first term
	for (int li=0; li<network->numOfLink; li++)
	{
		sumobjf += network->linkVector[li]->GetIntCost();
	}
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			dest = pOrg->destVector[j];
			odobjs = 0.0;
			for (int k=0; k < dest->pathSet.size(); k++)
			{
				// aggregate the second term
				odobjs += (dest->assDemand * dest->pathSet[k]->buffer[0] * (
					votdist->MMBA1_Ptheta(dest->pathSetbound[k+1]) - 
					votdist->MMBA1_Ptheta(dest->pathSetbound[k])));
			}
			sumobjs += odobjs;
		}
	}
	OFV =  sumobjf + sumobjs;
}


void TAP_SBA::SBA_GAP()
{
	floatType   sumobjf= 0.0;
	floatType	sumobjs= 0.0;
	floatType	sumpath= 0.0;
	floatType	sumlink= 0.0;
	floatType	odobjs= 0.0;
	TNM_SORIGIN *pOrg;
	TNM_SPATH* pa;
	likelysppcount = 0;
	clock_t start, end;
	double gapshortestpathtime=0.0;
	shortestpathtime = 0.0;
	truespptime = 0.0;
	ltatime = 0.0;
	ltachecktime = 0.0;
	lta_specific_check_time = 0.0;
	totalDset = 0;
	spptreenum = 0;
	floatType tempresult = 0.0;
	// calculate the first term of the objective function 
	for (int i=0; i<network->numOfLink; i++)
	{
		sumobjf += network->linkVector[i]->GetIntCost();
	}
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		//cout << "in gap current i = " << i << endl;
		pOrg = network->originVector[i];
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			odobjs = 0.0;
			for (int k=0; k < dest->pathSet.size(); k++)
			{
				tempresult = dest->assDemand * dest->pathSet[k]->buffer[0] * (
					votdist->MMBA1_Ptheta(dest->pathSetbound[k+1])- votdist->MMBA1_Ptheta(dest->pathSetbound[k]));
				odobjs += tempresult;
				sumlink += tempresult;
				// check negative flow and wrong boundary order
				if (tempresult<0)
				{
					cout<<"sumlink reduce"<<tempresult<<endl;
					system("PAUSE");
				}
				if (dest->pathSet[k]->flow >= 0)
				{
					floatType tempflowvalue;
					tempflowvalue = dest->assDemand *(
						votdist->MMBA1_Gtheta(dest->pathSetbound[k+1]) - votdist->MMBA1_Gtheta(dest->pathSetbound[k]));
					if (tempflowvalue>=0)
					{
						sumlink += dest->pathSet[k]->PathCostS() * tempflowvalue;
					}
					else
					{
						cout<<"in gap calculation: boundary has something wrong!"<<
							dest->origin->id<<"->"<<dest->dest->id<<
							"boundary difference="<<(dest->pathSetbound[k+1]-dest->pathSetbound[k])<<endl;
					}
				}
				else
				{
					cout<<"in gap calculation: negative flow"<<
						dest->origin->id<<"->"<<dest->dest->id<<"flow="<<dest->pathSet[k]->flow<<endl;
					sumlink += 0;
				}
				// check whether boundary result and flow result matches
				if (fabs((dest->pathSet[k]->flow) - (dest->assDemand *(
					votdist->MMBA1_Gtheta(dest->pathSetbound[k+1]) - votdist->MMBA1_Gtheta(dest->pathSetbound[k]))) )>1e-5)
				{
					cout<<"path set size="<<dest->pathSet.size()<<endl;
					cout<<"path set bound=";
					for(int u=0; u<dest->pathSetbound.size(); u++)
					{
						cout<<dest->pathSetbound[u];

					}
					cout<<endl;
					cout<<pOrg->origin->id<<"->"<<dest->dest->id<<endl;
					cout<<"k="<<k<<"method1="<<dest->pathSet[k]->flow<<
						"method2="<<dest->assDemand *(
						votdist->MMBA1_Gtheta(dest->pathSetbound[k+1]) - votdist->MMBA1_Gtheta(dest->pathSetbound[k]))<<endl;
					cout<<"dest->pathSetbound[k]="<<dest->pathSetbound[k]<<
						"dest->pathSetbound[k+1]="<<dest->pathSetbound[k+1]<<
						"buffer[2]="<<dest->pathSet[k]->buffer[2]<<
						"buffer[3]="<<dest->pathSet[k]->buffer[3]<<endl;
					cout<<"lower bound difference="<<(dest->pathSetbound[k] - dest->pathSet[k]->buffer[2])<<
						"upper bound difference"<<(dest->pathSetbound[k+1] - dest->pathSet[k]->buffer[3])<<endl;
					system("pause");
				}
			}
			// aggregate the second term of the objective function
			sumobjs += odobjs;
		}
		start = clock();
		// call parametric shortest path to find their difference with current pattern
		floatType oldtreenum = spptreenum;
		if (useMPAstrategy == true)
		{
			Dial_MPA_path(pOrg, 0, 0);
		}
		else
		{
			MPA_path_scrach(pOrg, 0, 0);
		}
		tree_num.push_back(spptreenum - oldtreenum);
		end = clock();
		gapshortestpathtime += (end-start);
	}
	// sumo_path is updated by MPA_path
	//cout << "in gap lta time = " << ltatime / CLOCKS_PER_SEC << " specific reverse time = "<<lta_specific_check_time / CLOCKS_PER_SEC << "ltanewtime" << ltachecktime / CLOCKS_PER_SEC << endl;
	//cout<<"in gap Dsettotal="<<totalDset<<endl;
	sumpath = sumo_path;
	OFV =  sumobjf + sumobjs;
	convIndicator = fabs(1 - sumpath/sumlink);
	cout<<"Step"<<curIter;
	cout<<" Relative gap="<<convIndicator;
	cout<<" OFV="<<OFV<<endl;
}



void TAP_SBA::Set_addpathbound(TNM_SDEST* dest)
{
	TNM_SPATH* temppath2;
	// 
	if (dest->pathSet.size() == 1)
	{
		dest->pathSet[0]->buffer[2] = 1.0/votdist->vot_max;
		dest->pathSet[0]->buffer[3] = 1.0/votdist->vot_min;
		return;
	}
	dest->pathSetbound.clear();
	dest->pathSetbound.push_back(1.0/votdist->vot_max);
	for(int i=0; i<dest->pathSet.size(); i++)
	{
		if(fabs(dest->pathSet[i]->buffer[1] - 1.0) < 1e-10)
		{
			temppath2 = dest->pathSet[i];
			if(i==0)
			{
				temppath2->buffer[2] = 1.0/votdist->vot_max;
				temppath2->buffer[3] = 1.0/votdist->vot_max;
			}
			else if(i==dest->pathSet.size()-1)
			{
				temppath2->buffer[2] = 1.0/votdist->vot_min;
				temppath2->buffer[3] = 1.0/votdist->vot_min;
			}
			else
			{
				temppath2->buffer[2] = dest->pathSet[i-1]->buffer[3];
				temppath2->buffer[3] = temppath2->buffer[2];
			}
			temppath2->flow = 0;
			temppath2->buffer[1] = 0.0;
		}
		dest->pathSetbound.push_back(dest->pathSet[i]->buffer[3]);
	}
}


bool TAP_SBA::check_money(TNM_SDEST* dest)
{
	for (int i=0;i<dest->pathSet.size()-1;i++)
	{
		if (dest->pathSet[i]->buffer[0] - dest->pathSet[i+1]->buffer[0]<0)
		{
			cout<<"----------------Money order is wrong!-------------"<<dest->origin->id<<"->"<<dest->dest->id<<
				"previous"<<"i="<<i<<" toll:"<<dest->pathSet[i]->buffer[0]<<
				"after"<<"i="<<(i+1)<<" toll:"<<dest->pathSet[i+1]->buffer[0]<<endl;
			return false;
		}
	}
	return true;
}


void TAP_SBA::adjust_one_bound(TNM_SDEST* dest, int k, TNM_SPATH* pathk, TNM_SPATH* pathkplus1)//k: index of pathk in path set
{
	floatType gthetak = votdist->MMBA1_gtheta(dest->pathSetbound[k+1]);//path k upper bound->pathset bound k+1
	//floatType gthetakprime = votdist->MMBA1_gprimetheta(dest->pathSetbound[k+1]);
	floatType pathtimediff = pathk->PathCostS() - pathkplus1->PathCostS();
	//calculate numerical result
	floatType gprimethetak = votdist->MMBA1_gprimetheta(dest->pathSetbound[k+1]);
	floatType wbd1_fd = dest->assDemand * gthetak * pathtimediff;
	floatType sd2path = 0.0;
	for (int i=0; i<pathk->path.size(); i++)
	{
		sd2path += pathk->path[i]->GetDerCost();
		pathk->path[i]->markStatus = 1;
	}
	for (int i=0; i<pathkplus1->path.size(); i++)
	{
		if (pathkplus1->path[i]->markStatus != 1)
		{sd2path += pathkplus1->path[i]->GetDerCost();}
		else
		{sd2path -= pathkplus1->path[i]->GetDerCost();}
	}
	floatType wbd2_fd = dest->assDemand * dest->pathSetbound[k+1] * gthetak * (pathk->buffer[0] - pathkplus1->buffer[0]);
	floatType wbd1_sd = pow(dest->assDemand, 2)*pow(gthetak, 2)*(sd2path);
	//floatType wbd1_sd = pow(dest->assDemand, 2)*pow(gthetak, 2)*(sd2path) + dest->assDemand*gthetakprime*pathtimediff;
	floatType wbd2_sd = dest->assDemand * (pathk->buffer[0] - pathkplus1->buffer[0]) * (gthetak);
	//floatType wbd2_sd = dest->assDemand * (pathk->buffer[0] - pathkplus1->buffer[0]) * (gthetak + dest->pathSetbound[k+1]*gprimethetak);

	floatType armijo_beta_0 = 1.0;
	floatType armijo_alpha = 0.9;
	floatType armijo_sigma = 1e-8;
	int max_substep = 1;
	floatType	singleboundary;
	floatType origin_obj_value;
	if (armijo_rule == true)
	{
		max_substep = 10;
		singleboundary= 0.0;
		for (int li=0; li< dest->pathSet[k]->path.size(); li++)
		{
			singleboundary +=  dest->pathSet[k]->path[li]->GetIntCost();
		}
		for (int li=0; li< dest->pathSet[k+1]->path.size(); li++)
		{
			singleboundary +=  dest->pathSet[k+1]->path[li]->GetIntCost();
		}
		singleboundary += (dest->assDemand * dest->pathSet[k]->buffer[0] * (
			votdist->MMBA1_Ptheta(dest->pathSetbound[k+1]) - 
			votdist->MMBA1_Ptheta(dest->pathSetbound[k])));
		singleboundary += (dest->assDemand * dest->pathSet[k+1]->buffer[0] * (
			votdist->MMBA1_Ptheta(dest->pathSetbound[k+2]) - 
			votdist->MMBA1_Ptheta(dest->pathSetbound[k+1])));
		origin_obj_value = singleboundary;
	}
	//floatType step;
	for (int stepsearch = 0; stepsearch < max_substep; stepsearch++)
	{
		floatType theta_adjust;
		if (armijo_rule == true)
		{
			theta_adjust = (wbd1_fd+wbd2_fd)/(wbd1_sd+wbd2_sd)* armijo_beta_0 * pow(armijo_alpha, stepsearch);
		}
		else if (armijo_rule == false)
		{
			if (convIndicator<1e-8)
			{
				theta_adjust = (wbd1_fd+wbd2_fd)/(wbd1_sd+wbd2_sd) * 0.7;
			}
			else
			{
				theta_adjust = (wbd1_fd+wbd2_fd)/(wbd1_sd+wbd2_sd);
			}
		}
		if (wbd1_sd+wbd2_sd<0)
		{
			cout<<"The second derivative is less than 0!"<<wbd1_sd+wbd2_sd<<
				"first term"<<wbd1_sd<<"second term"<<wbd2_sd<<
				"o="<<dest->origin->id<<"d="<<dest->dest->id<<"sd2path="<<sd2path<<endl;
			system("pause");
		}
		// adjust theta
		floatType achieve_result = dest->pathSetbound[k+1] - theta_adjust;
		floatType oldtheta = dest->pathSetbound[k+1];
		floatType oldbuffer3 = pathk->buffer[3];
		floatType oldbuffer2 = pathkplus1->buffer[2];
		floatType shiftflow = 0.0;
		// Direction within boundary
		if ((achieve_result >= dest->pathSetbound[k]) && (achieve_result <= dest->pathSetbound[k+2]))
		{
			dest->pathSetbound[k+1] = achieve_result;
			pathk->buffer[3] = achieve_result;
			pathkplus1->buffer[2] = achieve_result;
			shiftflow = dest->assDemand*(votdist->MMBA1_Gtheta(dest->pathSetbound[k+1]) - votdist->MMBA1_Gtheta(oldtheta));
			pathk->flow += shiftflow;
			pathkplus1->flow -= shiftflow;
		}
		else if (achieve_result < dest->pathSetbound[k])
		{
			dest->pathSetbound[k+1] = dest->pathSetbound[k];
			pathk->buffer[3] = dest->pathSetbound[k+1];
			pathkplus1->buffer[2] = dest->pathSetbound[k+1];
			shiftflow = (-1)*pathk->flow;
			pathk->flow = 0.0;
			pathkplus1->flow -= shiftflow;

		}
		else if (achieve_result > dest->pathSetbound[k+2])
		{
			dest->pathSetbound[k+1] = dest->pathSetbound[k+2];
			pathk->buffer[3] = dest->pathSetbound[k+1];
			pathkplus1->buffer[2] = dest->pathSetbound[k+1];
			shiftflow = pathkplus1->flow;
			pathk->flow += shiftflow;
			pathkplus1->flow = 0.0;
		}
		else
		{
			cout<<"There is no such result in theory!"<<endl;
		}
		//update flow 
		total_shift_flow += (fabs(shiftflow));
		for(int i=0; i< pathk->path.size(); i++)
		{
			pathk->path[i]->volume += shiftflow;
			pathk->path[i]->cost = pathk->path[i]->GetCost();
			pathk->path[i]->fdCost = pathk->path[i]->GetDerCost();
			pathk->path[i]->markStatus = 0;
		}
		for(int i=0; i< pathkplus1->path.size(); i++)
		{
			pathkplus1->path[i]->volume -= shiftflow;
			pathkplus1->path[i]->cost = pathkplus1->path[i]->GetCost();
			pathkplus1->path[i]->fdCost = pathkplus1->path[i]->GetDerCost();
			pathkplus1->path[i]->markStatus = 0;
		}
		if (armijo_rule == true)
		{
			singleboundary= 0.0;
			for (int li=0; li< dest->pathSet[k]->path.size(); li++)
			{
				singleboundary +=  dest->pathSet[k]->path[li]->GetIntCost();
			}
			for (int li=0; li< dest->pathSet[k+1]->path.size(); li++)
			{
				singleboundary +=  dest->pathSet[k+1]->path[li]->GetIntCost();
			}
			singleboundary += (dest->assDemand * dest->pathSet[k]->buffer[0] * (
				votdist->MMBA1_Ptheta(dest->pathSetbound[k+1]) - 
				votdist->MMBA1_Ptheta(dest->pathSetbound[k])));
			singleboundary += (dest->assDemand * dest->pathSet[k+1]->buffer[0] * (
				votdist->MMBA1_Ptheta(dest->pathSetbound[k+2]) - 
				votdist->MMBA1_Ptheta(dest->pathSetbound[k+1])));
			if (origin_obj_value - singleboundary >= armijo_sigma * armijo_beta_0 * pow(armijo_alpha, stepsearch) * pow(wbd1_fd+wbd2_fd, 2)/(wbd1_sd+wbd2_sd)) // satisfies armijo rule
			{
				break;
			}
			else //return to the origin status
			{
				dest->pathSetbound[k+1] = oldtheta;
				pathk->buffer[3] = oldbuffer3;
				pathkplus1->buffer[2] = oldbuffer2;
				pathk->flow -= shiftflow;
				pathkplus1->flow += shiftflow;
				for(int i=0; i< pathk->path.size(); i++)
				{
					pathk->path[i]->volume -= shiftflow;
					pathk->path[i]->cost = pathk->path[i]->GetCost();
					pathk->path[i]->fdCost = pathk->path[i]->GetDerCost();
					pathk->path[i]->markStatus = 0;
				}
				for(int i=0; i< pathkplus1->path.size(); i++)
				{
					pathkplus1->path[i]->volume += shiftflow;
					pathkplus1->path[i]->cost = pathkplus1->path[i]->GetCost();
					pathkplus1->path[i]->fdCost = pathkplus1->path[i]->GetDerCost();
					pathkplus1->path[i]->markStatus = 0;
				}
			}
		}
	}
	//tiny flow handling
	if ((pathk->flow < 0) || (pathkplus1->flow < 0))
	{
		if (((pathk->flow < 0)  && (fabs(pathk->flow) >=1e-9) ) || ((pathkplus1->flow < 0) && (fabs(pathkplus1->flow) >= 1e-9)))
		{
			cout<<"pathk flow"<<pathk->flow<<"pathkplus1->flow"<<pathkplus1->flow<<endl;
			system("PAUSE");
		}
		if (((fabs(pathk->flow) < 1e-9) && (pathk->flow < 0)) || (fabs(pathkplus1->flow) < 1e-9) && (pathkplus1->flow < 0))
		{
			if ((fabs(pathk->flow) < 1e-9) && (pathk->flow < 0))
			{
				floatType oldtinyflow = pathk->flow;
				pathk->flow = 0.0;
				for(int i=0; i< pathk->path.size(); i++)
				{
					pathk->path[i]->volume += (-1)* oldtinyflow;
					pathk->path[i]->cost = pathk->path[i]->GetCost();
					pathk->path[i]->fdCost = pathk->path[i]->GetDerCost();
				}
				pathkplus1->flow += oldtinyflow;
				for(int i=0; i< pathkplus1->path.size(); i++)
				{
					pathkplus1->path[i]->volume += oldtinyflow;
					pathkplus1->path[i]->cost = pathkplus1->path[i]->GetCost();
					pathkplus1->path[i]->fdCost = pathkplus1->path[i]->GetDerCost();
				}
			}
			if ((fabs(pathkplus1->flow) < 1e-9) && (pathkplus1->flow < 0))
			{
				floatType oldtinyflow = pathkplus1->flow;
				pathkplus1->flow = 0.0;
				for(int i=0; i< pathkplus1->path.size(); i++)
				{
					pathkplus1->path[i]->volume += (-1)* oldtinyflow;
					pathkplus1->path[i]->cost = pathkplus1->path[i]->GetCost();
					pathkplus1->path[i]->fdCost = pathkplus1->path[i]->GetDerCost();
				}
				pathk->flow += oldtinyflow;
				for(int i=0; i< pathk->path.size(); i++)
				{
					pathk->path[i]->volume += oldtinyflow;
					pathk->path[i]->cost = pathk->path[i]->GetCost();
					pathk->path[i]->fdCost = pathk->path[i]->GetDerCost();
				}

			}

		}
		else
		{
			cout<<"pathk->flow:"<<pathk->flow<<"     pathkplus1->flow:"<<pathkplus1->flow<<endl;
			cout<<"the value of path flow belows 0, wrong!"<<endl;
		}
	}
}


void  TAP_SBA::boundadjusting_OD(TNM_SDEST* dest)
{
	if (dest->pathSet.size() == 1)
	{
		return;
	}
	if(deletestrategy != 4)
	{
		int pi = 0;
		for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin(); pi < dest->pathSet.size() - 1;)//pv != dest->pathSet.end()-1
		{
			delete_k_status = false;
			delete_kplus1_status = false;
			TNM_SPATH* pa = *pv;
			adjust_one_bound(dest, pi, dest->pathSet[pi], dest->pathSet[pi+1]);
			if(deletestrategy == 1)
			{
				pv++;
				pi += 1;
			}
			else
			{
				if (deletestrategy == 3)
				{
					// adjust theta_{k+1}
					if (pi < dest->pathSet.size() - 2)// pathSet[pi+2] exists//pv != dest->pathSet.end()-2
					{
						adjust_one_bound(dest, pi+1, dest->pathSet[pi+1], dest->pathSet[pi+2]);
					}
				}
				// Delete zero flow k or k+1
				if (dest->pathSet[pi]->flow == 0.0)
				{
					delete_k_status = true;
					dest->pathSetbound.erase(dest->pathSetbound.begin() + (pi + 1));
					dest->pathSet.erase(dest->pathSet.begin() + pi);
					depathset.push_back(pa);
					// delete pa;
					periter_deletepathcount ++;

				}
				int add_index;
				if (delete_k_status)  add_index = 0;
				else add_index = 1;
				if (dest->pathSet[pi+add_index]->flow == 0.0)
				{
					delete_kplus1_status = true;
					dest->pathSetbound.erase(dest->pathSetbound.begin() + (pi + 1 + add_index));
					TNM_SPATH* kplus1 = dest->pathSet[pi + add_index];
					dest->pathSet.erase(dest->pathSet.begin() + pi + add_index);
					depathset.push_back(kplus1);
					// delete kplus1;
					periter_deletepathcount ++;
				}
				// fetch next path boundary
				if ((delete_k_status == false))
				{
					pi += 1;
					pv++;
				}

			}
		}
	}
	else // delete strategy == 4
	{
		TNM_SPATH* pathk;
		TNM_SPATH* pathkplus1;
		TNM_SPATH* pathkminus1;
		// The descending pass: 
		int totaldecentnum = 1;
		int pi;
		while (totaldecentnum > 0) // This while loop is designed for restart
		{
			totaldecentnum -= 1;
			pi = dest->pathSet.size() - 2; 
			// |K| - 1 to 1 -> index: |K|-2 to 0
			for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.end() - 2; pi >= 0;)
			{
				pathk = dest->pathSet[pi];
				pathkplus1 = dest->pathSet[pi + 1];
				if ((pathkplus1->PathCostS() + dest->pathSetbound[pi + 1] * pathkplus1->buffer[0]) < (pathk->PathCostS() + dest->pathSetbound[pi + 1] * pathk->buffer[0]))
				{
					adjust_one_bound(dest, pi, dest->pathSet[pi], dest->pathSet[pi+1]);
					if ((dest->pathSet[pi]->flow == 0.0) && pi > 0)
					{
						pathkminus1 = dest->pathSet[pi - 1];
						if ((pathkminus1->PathCostS() + dest->pathSetbound[pi] * pathkminus1->buffer[0]) <= (pathk->PathCostS() + dest->pathSetbound[pi] * pathk->buffer[0]))
						{
							dest->pathSetbound.erase(dest->pathSetbound.begin() + (pi + 1));
							dest->pathSet.erase(dest->pathSet.begin() + pi);
							depathset.push_back(pathk);
							periter_deletepathcount ++;
							totaldecentnum = 1;
							break;
						}
						else
						{
							pi-= 1;
							pv--;
						}
					}
					else
					{
						pi-= 1;
						pv--;
					}
				}
				else if ((pathkplus1->PathCostS() + dest->pathSetbound[pi + 1] * pathkplus1->buffer[0]) > (pathk->PathCostS() + dest->pathSetbound[pi + 1] * pathk->buffer[0]))
				{
					adjust_one_bound(dest, pi, dest->pathSet[pi], dest->pathSet[pi+1]);
					pi-= 1;
					pv--;
				}
				else
				{
					pi-= 1;
					pv--;
				}
			}
		}
		// ascending path
		TNM_SPATH* pathkplus2;
		totaldecentnum = 1;
		while (totaldecentnum > 0) // This while loop is designed for restart
		{
			totaldecentnum -= 1;
			pi = 0; 
			// 1 to |K| - 1 -> index: 0 to |K|-2
			for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin(); pi < dest->pathSet.size() - 1;)
			{
				pathk = dest->pathSet[pi];
				pathkplus1 = dest->pathSet[pi + 1];
				if ((pathkplus1->PathCostS() + dest->pathSetbound[pi + 1] * pathkplus1->buffer[0]) > (pathk->PathCostS() + dest->pathSetbound[pi + 1] * pathk->buffer[0]))
				{
					adjust_one_bound(dest, pi, dest->pathSet[pi], dest->pathSet[pi+1]);
					if ((dest->pathSet[pi + 1]->flow == 0.0) && pi + 2 < dest->pathSet.size())
					{
						pathkplus2 = dest->pathSet[pi + 2];
						if ((pathkplus1->PathCostS() + dest->pathSetbound[pi + 2] * pathkplus1->buffer[0]) >= (pathkplus2->PathCostS() + dest->pathSetbound[pi + 2] * pathkplus2->buffer[0]))
						{
							dest->pathSetbound.erase(dest->pathSetbound.begin() + (pi + 2));
							dest->pathSet.erase(dest->pathSet.begin() + pi + 1);
							depathset.push_back(pathkplus1);
							periter_deletepathcount ++;
							totaldecentnum = 1;
							break;
						}
						else
						{
							pi+= 1;
							pv++;
						}
					}
					else
					{
						pi+= 1;
						pv++;
					}
				}
				else if ((pathkplus1->PathCostS() + dest->pathSetbound[pi + 1] * pathkplus1->buffer[0]) < (pathk->PathCostS() + dest->pathSetbound[pi + 1] * pathk->buffer[0]))
				{
					adjust_one_bound(dest, pi, dest->pathSet[pi], dest->pathSet[pi+1]);
					pi+= 1;
					pv++;
				}
				else
				{
					pi+= 1;
					pv++;
				}
			}
		}
	}
}


floatType TAP_SBA::SBA_innerGap_OD(TNM_SDEST* dest)
{
	if (dest->pathSet.size() == 1)
	{
		return 0.0;
	}
	TNM_SPATH* pathk;
	TNM_SPATH* pathkplus1;
	floatType gthetak, pathtimediff, wbd_fd, thetak;
	floatType wbd_fd_max = 0.0;
	for (int k=0; k<dest->pathSet.size()-1; k++)
	{
		pathk = dest->pathSet[k];
		pathkplus1 = dest->pathSet[k+1];
		thetak = dest->pathSetbound[k+1];
		gthetak = votdist->MMBA1_gtheta(thetak);
		pathtimediff = pathk->PathCostS() - pathkplus1->PathCostS();
		wbd_fd = dest->assDemand * gthetak * pathtimediff + dest->assDemand * thetak * gthetak * (pathk->buffer[0] - pathkplus1->buffer[0]);
		if (fabs(wbd_fd)>wbd_fd_max)
		{wbd_fd_max = fabs(wbd_fd);}
	}
	return wbd_fd_max;
}


void TAP_SBA::deletenousepath_OD(TNM_SDEST* dest)
{
	if(dest->pathSet.size()==1)
	{
		return;
	}
	else
	{
		int pi = 0;
		for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin(); pv != dest->pathSet.end();)
		{
			TNM_SPATH* pa = *pv;
			if (fabs(pa->flow)< 1e-10)
			{
				if (pi!= 0)
				{
					dest->pathSet[pi-1]->flow += pa->flow;
					for(int j=0; j<dest->pathSet[pi-1]->path.size(); j++)
					{
						dest->pathSet[pi-1]->path[j]->volume += pa->flow;
						dest->pathSet[pi-1]->path[j]->cost = dest->pathSet[pi-1]->path[j]->GetCost();
						dest->pathSet[pi-1]->path[j]->fdCost = dest->pathSet[pi-1]->path[j]->GetDerCost();
					}
					dest->pathSet[pi-1]->buffer[3] = dest->pathSet[pi]->buffer[3];

					dest->pathSetbound.erase(dest->pathSetbound.begin() + (pi));
				}
				else
				{
					dest->pathSet[pi+1]->flow += pa->flow;
					for(int j=0; j<dest->pathSet[pi+1]->path.size(); j++)
					{
						dest->pathSet[pi+1]->path[j]->volume += pa->flow;
						dest->pathSet[pi+1]->path[j]->cost = dest->pathSet[pi+1]->path[j]->GetCost();
						dest->pathSet[pi+1]->path[j]->fdCost = dest->pathSet[pi+1]->path[j]->GetDerCost();
					}
					dest->pathSet[pi+1]->buffer[2] = dest->pathSet[pi]->buffer[2];
					dest->pathSetbound.erase(dest->pathSetbound.begin() + (pi+1));
				}
				dest->pathSet.erase(dest->pathSet.begin()+pi);
				depathset.push_back(pa);
				periter_deletepathcount ++;
			}
			else
			{
				pv++;
				pi += 1;
			}
		}
		for (int i=0; i<dest->pathSet.size(); i++)
		{
			TNM_SPATH* pa = dest->pathSet[i];
			if (fabs(pa->buffer[3] - dest->pathSetbound[i+1])>1e-6)
			{
				cout<<dest->origin->id<<"->"<<dest->dest->id<<
					"k="<<i<<" pa->buffer[2]="<<pa->buffer[2]<<
					" pa->buffer[3]="<<pa->buffer[3]<<"dest->pathSetbound[i+1]="<<dest->pathSetbound[i+1]<<endl;
				cout<<dest->origin->id<<"->"<<dest->dest->id<<
					"k+1="<<i+1<<
					" dest->pathSet[i+1]->buffer[2]="<<dest->pathSet[i+1]->buffer[2]<<
					" dest->pathSet[i+1]->buffer[3]="<<dest->pathSet[i+1]->buffer[3]<<
					"dest->pathSetbound[i+2]="<<dest->pathSetbound[i+2]<<endl;
				cout<<"Path size="<<dest->pathSet.size()<<" Path boundary size="<<dest->pathSetbound.size()<<endl;
				for (int bi=0; bi<dest->pathSetbound.size(); bi++)
				{
					cout<<dest->pathSetbound[bi]<<"  ";
				}
				cout<<endl;
				system("PAUSE");
			}
		}
	}
}

int TAP_SBA::adaptive_inner_2(floatType deltaod, int innerFC, TNM_SDEST* dest)
{
	if (convIndicator>1e-4)
	{
		 if (deltaod > aveInnerGap/50.0)
		 {
			  innerFC += 1;
			  boundadjusting_OD(dest);
		 }
	}
	else if (convIndicator<=1e-4 &&convIndicator>1e-6)
	{
		 if (deltaod > aveInnerGap/20.0)
		 {
		  innerFC += 1;
		  boundadjusting_OD(dest);
		 }
	}
	else if (convIndicator<=1e-6 &&convIndicator>1e-8)
	{
		if (deltaod > aveInnerGap/10.0)
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	else
	{
		 if (deltaod > aveInnerGap*10.0)
		 {
		  innerFC += 1;
		  boundadjusting_OD(dest);
		 }
	}
	return innerFC;
}

int TAP_SBA::adaptive_inner(floatType deltaod, int innerFC, TNM_SDEST* dest)
{
	if (convIndicator>1e-1)
	{
		if (deltaod > convIndicator/100.0) // /100
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	else if (convIndicator>1e-2)
	{
		if (deltaod > convIndicator/10.0) // /10
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	else if (convIndicator>1e-4)
	{
		if (deltaod > convIndicator*100.0) // *100
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	else if (convIndicator>1e-6)
	{
		if (deltaod > convIndicator*1000.0)  // *1000
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	else if (convIndicator>1e-8)
	{
		if (deltaod > convIndicator*10000.0) // *10000
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	else
	{
		if (deltaod > convIndicator*10000.0)   // *10000
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	return innerFC;
}

int TAP_SBA::adaptive_inner_3(floatType deltaod, int innerFC, TNM_SDEST* dest)
{
	if (convIndicator > 1e-2)
	{
		if (deltaod > convIndicator / 100.0)
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	else if (convIndicator > 1e-4)
	{
		if (deltaod > convIndicator / 10.0)
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	else if (convIndicator > 1e-6) 
	{
		if (deltaod > convIndicator)
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	else if (convIndicator > 1e-8)
	{
		if (deltaod > convIndicator * 10.0)
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	else
	{
		if (deltaod > convIndicator * 10.0)
		{
			innerFC += 1;
			boundadjusting_OD(dest);
		}
	}
	return innerFC;
}

// Main loop for the SBA algorithm
void TAP_SBA::MainLoop()
{
	//change to the time without gap
	iterRecord[curIter-1]->time = 1.0*(withoutgaptime)/CLOCKS_PER_SEC;
	cout<<"current time="<<iterRecord[curIter-1]->time<<endl;
	clock_t withoutgaptime_s, withoutgaptime_e;
	withoutgaptime_s = clock();
	periter_addpathcount = 0;
	periter_deletepathcount = 0;
	total_pathset_size = 0;
	max_pathset_size = 0;
	total_shift_flow = 0.0;
	sumo_path=0.0;
	spptreenum = 0;
	totalDset = 0;
	if(curIter == 1)
	{
		convIndicator = 1;
	}
	clock_t start_innerloop, end_innerloop, start_adjust, end_adjust, start_innerod, end_innerod;
	double innerlooptime = 0.0;
	double adjusttime = 0.0;
	double innerodtime = 0.0;
	TNM_SORIGIN *pOrg;
	TNM_SDEST* dest;
	bool wrong;
	// column generation
	clock_t startmain1 = clock();
	double MPAtime = 0.0;
	ltatime = 0.0;
	ltachecktime = 0.0;
	lta_specific_check_time = 0.0;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		floatType oldtreenum = spptreenum;
		clock_t startmain2 = clock();
		if (useMPAstrategy == true)
		{
			Dial_MPA_path(pOrg, 0, 1);
		}
		else
		{
			MPA_path_scrach(pOrg, 0, 1);
		}
		clock_t endmain2 = clock();
		MPAtime += (endmain2 - startmain2);
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			dest = pOrg->destVector[j];
			Set_addpathbound(dest);//remake the path bound in the path set
			wrong = check_money(dest);//check money order
			//adjust
			start_adjust = clock();
			boundadjusting_OD(dest);
			end_adjust = clock();
			adjusttime += (end_adjust - start_adjust);
			dest->rdmSet.clear();
		}
	}
	clock_t endmain1 = clock();
	//cout << "mainloop lta time = " << ltatime / CLOCKS_PER_SEC << "specific add time"<< lta_specific_check_time/CLOCKS_PER_SEC<< "  ltanewtime = " << ltachecktime / CLOCKS_PER_SEC << endl;
	//cout<<"Dsettotal="<<totalDset<<endl;
	//cout<<"adjust time"<<adjusttime/CLOCKS_PER_SEC<<"total time"<<(endmain1 - startmain1)/CLOCKS_PER_SEC<<"MPA time"<<MPAtime/CLOCKS_PER_SEC<<endl;
	//cout<<"ltatime"<<ltatime/CLOCKS_PER_SEC<<"true spp time"<<truespptime/CLOCKS_PER_SEC<<endl;
	//cout<<"ltachecktime"<<ltachecktime/CLOCKS_PER_SEC<<endl;
	//cout<<"spptree"<<spptreenum<<endl;
	//cout<<"Dsettotal="<<totalDset<<endl;
	deletestrategy = 1;

	floatType deltaod;
	start_innerloop = clock();
	int innerI=0;
	int innerMaxI = 300;
	int innerFC=0;
	double migap=50;
	floatType totalinnergap=0;
	int       totalgapcount=0;
	while(innerI<innerMaxI)
	{
		//cout<<"InnerI = "<<innerI<<endl;
		migap=0;
		if (convIndicator < 1e-8)//1e-8
		{
			innerMaxI = 600;
		}
		innerI += 1;
		innerFC = 0;
		for (int i=0;i<network->numOfOrigin;i++)
		{
			pOrg = network->originVector[i];
			for (int j=0;j<pOrg->numOfDest;j++)
			{
				dest = pOrg->destVector[j];
				deltaod = SBA_innerGap_OD(dest);
				/////////////////////////inner loop Scheme 1////////////////////////////
				if (network->numOfNode > 10000)
				{
					// For large-scale networks such as Chicago Regional, smaller values of $\gamma$ are adopted. 
					// Empirical tests suggest that this adjustment enhances convergence efficiency for the Chicago Regional case.
					innerFC = adaptive_inner_3(deltaod, innerFC, dest);
				}
				else
				{
					// For networks of normal size, we adopt the same $\gamma$ values as those reported in the paper.
					innerFC = adaptive_inner(deltaod, innerFC, dest);
				}
				if (deletestrategy == 1)
				{
					if (innerI%5==0)
					{
						// Delete no use path for an O-D pair
						deletenousepath_OD(dest);
					}
				}
			}
		}
		if (innerFC <= 3)
		{
			break;
		}
	}
	//delete path
	if (deletestrategy == 1)
	{
		for (int i = 0;i<network->numOfOrigin;i++)
		{
			pOrg = network->originVector[i];
			for (int j=0;j<pOrg->numOfDest;j++)
			{
				dest = pOrg->destVector[j];
				// Delete no use path for an O-D pair
				deletenousepath_OD(dest);
				total_pathset_size += dest->pathSet.size();
				if (dest->pathSet.size() > max_pathset_size)
				{
					max_pathset_size = dest->pathSet.size();
				}
			}
		}
	}
	end_innerloop = clock();
	innerlooptime = end_innerloop - start_innerloop;
	withoutgaptime_e = clock();
	withoutgaptime += (withoutgaptime_e - withoutgaptime_s);

	deletestrategy = 1;

	// print for numerical example
	//for (int li=0; li<network->numOfLink; li++)
	//{
	//	cout<<network->linkVector[li]->tail->id<<" to "<<network->linkVector[li]->head->id
	//		<<"link time="<<network->linkVector[li]->GetCost()<<endl;
	//}

	///////////////////cs/////////////////////////
	sumo_path=0.0;
	SBA_GAP();
	///////////////////cr/////////////////////////////
	//SBA_obj();


	// Delete object in depathset
	TNM_SPATH* path;
	for (int pi=0; pi<depathset.size(); pi++)
	{
		path = depathset[pi];
		delete path;
	}
	depathset.clear();
	//////////////////////////////////////////////////
	// aveInnerGap=totalinnergap/totalgapcount;
	//******************************************************
	//cout<<"The curIter is "<<curIter<<endl;
	//cout<<"The OFV is "<<OFV<<endl;
	//cout<<"The gap is "<<convIndicator<<endl;
	//cout<<"The inner loop iteration is "<<innerI<<endl;
	//cout<<"The minimum inner gap is "<<migap<<endl;
	//cout<<"The average inner gap is "<<aveInnerGap<<endl;
	//cout<<"Total shift flow is "<<total_shift_flow<<endl;
	//cout<<"Total add path is "<<periter_addpathcount<<" delete path is "<<periter_deletepathcount<<endl;
	//cout<<"Avg OD pathset size is "<<double(total_pathset_size)/double(network->numOfOD)<<" max path set size is "<<max_pathset_size<<endl;
	//******************************************************
	//cout<<"Cur Time"<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
	//cout<<"Without Gap Time"<<1.0*(withoutgaptime)/CLOCKS_PER_SEC<<endl;
	//******************************************************
	//cout<<"Inner Loop Time"<<innerlooptime/CLOCKS_PER_SEC<<endl;
	//cout<<"Inner OD Time"<<innerodtime/CLOCKS_PER_SEC<<endl;
	//cout<<"Adjust Time include out and inner"<<adjusttime/CLOCKS_PER_SEC<<endl;
}



int TAP_SBA::MPA_path_scrach(TNM_SORIGIN* origin, int allornothing, int addset)
{
	clock_t start1, end1, start2, end2, start3, end3;
	for(int ni=0;ni<network->numOfNode;ni++)
	{
		network->nodeVector[ni]->buffer[0] = 0;// node time
		network->nodeVector[ni]->buffer[1] = 0;// node toll
		network->nodeVector[ni]->buffer[2] = 0;// node already in D set or not
		network->nodeVector[ni]->buffer[4] = theta_min; //a_node: boundary that already assign
	}
	for(int li=0;li<network->numOfLink;li++)
	{
		network->linkVector[li]->buffer[0] = -1;//set the link divide theta as -1
		network->linkVector[li]->buffer[6] = 0; // link in tree or not of the shortest path, reset to 0
	}
	// Initialize theta_low and theta_up
	theta_low = theta_min;
	theta_up = theta_max;
	// calculate the shortest path tree for theta_min
	Dial_MIN_path(origin);
	spptreenum += 1;  // update tree number
	// initialize vectors and objects
	vector<TNM_SLINK*>   pllist(network->numOfNode, NULL);
	vector<floatType>   plmoney(network->numOfNode, NULL);
	vector<floatType>   pltime(network->numOfNode, NULL);
	floatType ptheta_up = theta_min; // initialize previous theta_up as theta_min
	vector<floatType> gCostlow(network->numOfLink, 0);
	TNM_SLINK* clink;
	TNM_SNODE* inode;
	TNM_SNODE* jnode;
	TNM_SNODE* previousnode;
	TNM_SNODE* nownode;
	TNM_SNODE* thisnode;
	TNM_SLINK* previouslink;
	do
	{
		spptreenum += 1;
		//record the last tree
		for(int i=0; i<network->numOfNode; i++)
		{
			network->nodeVector[i]->buffer[2] = 0; // node is or is not add in to Dset;
			pllist[i] = network->nodeVector[i]->pathElem->via;
			plmoney[i] = network->nodeVector[i]->buffer[1];
			pltime[i] = network->nodeVector[i]->buffer[0];
		}
		ptheta_up = theta_up;  // shift tree record theta_up
		theta_low = theta_up;  // shift tree
		theta_low += 1e-12;    // make sure tree shift is successful, add a small number
		theta_up = theta_max; // initialize theta_up
		start3 = clock();
		for(int li =0 ; li< network->numOfLink;li++)
		{
			gCostlow[li] = network->linkVector[li]->cost + theta_low * network->linkVector[li]->toll;
			network->linkVector[li]->buffer[0] = -1; //set the link divide theta as -1
			network->linkVector[li]->buffer[6] = 0;  // link in tree or not of the shortest path, reset to 0
		}
		Generalized_cost_spp(origin->origin, gCostlow);
		end1 = clock();
		truespptime += (end1 - start3);
		start3 = clock();
		clink = NULL;
		inode = NULL;
		jnode = NULL;
		for(int li =0; li < network->numOfLink; li++)
		{
			clink  = network->linkVector[li];
			clink->buffer[0] = theta_max; // reset buffer[0]
			if(clink->buffer[6] == 0) // this link is not in the current tree
			{
				inode = clink->tail;
				jnode = clink->head;
				if(fabs(inode->buffer[1] + clink->toll - jnode->buffer[1]) < 1e-9)
				{
					clink->buffer[0] = theta_max;
				}
				else
				{
					floatType theta_a = (jnode->buffer[0] - clink->cost - inode->buffer[0])/( inode->buffer[1]+ clink->toll - jnode->buffer[1]);
					clink->buffer[0] = setdecimal(theta_a, 12);//set the decimal point
					if (clink->buffer[0]< theta_up && clink->buffer[0]>theta_low)
					{
						theta_up = clink->buffer[0];
					}
				}
			}

		}
		//find changed node
		Dset.clear();
		for(int di=0; di<origin->numOfDest; di++)
		{
			thisnode = origin->destVector[di]->dest;
			if (thisnode == origin->origin)
			{
				continue;
			}
			nownode = origin->destVector[di]->dest;
			do
			{
				previousnode = nownode->pathElem->via->tail;
				previouslink = nownode->pathElem->via;
				if (previouslink != pllist[nownode->id - 1]) 
				{//node->buffer[2] is in D set or not, avoid repeated additions
					if ((thisnode->buffer[2]==0)&&((fabs(thisnode->buffer[0] - pltime[thisnode->id - 1])>1e-14)||(fabs(thisnode->buffer[1]- plmoney[thisnode->id-1])>1e-14)))
					{
						// add node to the D set
						thisnode->buffer[2] = 1;
						Dset.push_back(thisnode);
					}					
					break;
				}
				else
				{
					nownode = previousnode;
				}
			}while(previousnode != origin->origin);
		}
		end3 = clock();
		shortestpathtime += (end3-start3);
		start2 = clock();
		// Lazy assignment for the previous tree
		Dial_LTA_path(origin, allornothing, addset, pllist, ptheta_up);
		end2 = clock();
		ltatime += (end2 - start2);
	}while(theta_up != theta_max);
	// At last round, put all node except origin to the Dset
	for(int ni=0; ni<network->numOfNode; ni++)
	{
		pllist[ni] = network->nodeVector[ni]->pathElem->via;
		if (network->nodeVector[ni] != origin->origin)
		{
			Dset.push_back(network->nodeVector[ni]);
		}
	}
	ptheta_up = theta_up;
	start2 = clock();
	// Lazy assignment for the last tree
	Dial_LTA_path(origin, allornothing, addset, pllist, ptheta_up);
	end2 = clock();
	ltatime += (end2 - start2);
	return 0;
}


int TAP_SBA::Dial_MPA_path(TNM_SORIGIN* origin, int allornothing, int addset)
{
	clock_t start1, end1;
	for(int ni=0;ni<network->numOfNode;ni++)
	{
		network->nodeVector[ni]->buffer[0] = 0;
		network->nodeVector[ni]->buffer[1] = 0;
		network->nodeVector[ni]->buffer[2] = 0;
		network->nodeVector[ni]->buffer[4] = theta_min;
	}
	for(int li=0;li<network->numOfLink;li++)
	{
		network->linkVector[li]->buffer[0] = -1;
		network->linkVector[li]->buffer[6] = 0;
	}
	theta_low = theta_min;
	theta_up = theta_max;
	Dial_MIN_path(origin);
	spptreenum += 1;
	vector<TNM_SLINK*>   pllist(network->numOfNode,NULL);
	floatType ptheta_up = theta_min;
	do
	{
		spptreenum += 1;
		//record the last tree
		for(int i=0; i<network->numOfNode; i++)
		{
			pllist[i] = network->nodeVector[i]->pathElem->via;
		}
		ptheta_up = theta_up;
		Dial_PTU_path(origin->origin);
		start1 = clock();
		Dial_LTA_path(origin, allornothing, addset, pllist, ptheta_up);
		end1 = clock();
		ltatime += (end1 - start1);
	}while(theta_up != theta_max);
	for(int i=0; i<network->numOfNode; i++)
	{
		pllist[i] = network->nodeVector[i]->pathElem->via;
		if (network->nodeVector[i] != origin->origin)
		{
			Dset.push_back(network->nodeVector[i]);
		}
	}
	ptheta_up = theta_up;
	start1 = clock();
	Dial_LTA_path(origin, allornothing, addset, pllist, ptheta_up);
	end1 = clock();
	ltatime += (end1 - start1);
	return 0;
}


int TAP_SBA::Dial_MIN_path(TNM_SORIGIN* origin)
{
	TNM_SLINK* clink;
	TNM_SNODE* inode;
	TNM_SNODE* jnode;
	vector<floatType> gCostlow(network->numOfLink,0);
	for(int li =0 ; li< network->numOfLink;li++)
	{
		gCostlow[li] = network->linkVector[li]->cost + theta_low * network->linkVector[li]->toll;
		network->linkVector[li]->buffer[0] = -1;
		network->linkVector[li]->buffer[6] = 0;
	}
	clock_t start2, end2;
	start2 = clock();
	// use gCostlow to get the shortest path tree
	Generalized_cost_spp(origin->origin, gCostlow);
	end2 = clock();
	truespptime += (end2-start2);
	theta_up = theta_max;
	Hset.clear();
	start2 = clock();
	for(int li =0; li< network->numOfLink; li++)
	{
		clink  = network->linkVector[li];
		clink->buffer[0] = theta_max;  // initialized as divide unavailable
		inode = network->linkVector[li]->tail;
		jnode = network->linkVector[li]->head;
		if(clink->buffer[6] == 0) //clink is not in the theta_min shortest path tree
		{
			if(setdecimal_round(inode->buffer[1] + clink->toll, 9) == setdecimal_round(jnode->buffer[1], 9))
			{
				clink->buffer[0] = theta_max;  // true divide unavailable
			}
			else
			{
				floatType theta_a = (jnode->buffer[0] - clink->cost - inode->buffer[0])/( inode->buffer[1] + clink->toll - jnode->buffer[1]);
				clink->buffer[0] = setdecimal(theta_a, 12);
				if (clink->buffer[0] < theta_up && clink->buffer[0] > theta_low)
				{
					theta_up = clink->buffer[0]; // divide available: make theta_up the min{buffer [0]} but larger than theta_low -> similar to increasing theta_up to the shortest board
					Hset.clear();  // Clear the node already in the Hset because they are not correspond to theta_up anymore
					Hset.push_back(inode);  //add inode
					origin->origin->buffer[8] += 1;
				}
				else if (clink->buffer[0] == theta_up && clink->buffer[0]>theta_low)
				{
					int Hexist = 0;
					// find inode already exist or not
					if(!Hset.empty())
					{
						for(unsigned hi=0;hi<Hset.size();hi++)
						{
							if(Hset[hi] == inode) Hexist = 1;
						}
					}
					// add node to h set
					if(!Hexist && inode != origin->origin) 
					{
						Hset.push_back(inode);
						origin->origin->buffer[8] += 1;
					}
				}
			}
		}

	}
	// after this process, theta_up is the first place to divide theta.
	end2 = clock();
	shortestpathtime += (end2 - start2);
	return 0;
}


int TAP_SBA::Dial_PTU_path(TNM_SNODE * origin)
{
	clock_t start1, end1;
	start1 = clock();
	Dset.clear();
	for (deque<TNM_SNODE*>::iterator no = Hset.begin(); no != Hset.end(); no++)
	{
		(*no)->buffer[2] = 1;
	}
	while(!Hset.empty())
	{
		TNM_SNODE * scanNode = Hset.front();
		Hset.pop_front();
		// add node in H set except the initial H set into Dset
		if (scanNode->buffer[2] == 1)
		{
			scanNode->buffer[2] = 0;
		}
		else{Dset.push_back(scanNode);}
		for(vector<TNM_SLINK*>::iterator pv = scanNode->forwStar.begin(); pv !=scanNode->forwStar.end(); pv++)
		{
			if( *pv != NULL)
			{
				TNM_SLINK * scanLink = *pv;
				TNM_SNODE * jnode = scanLink->head;
				// find whether to change label or not
				floatType d_star = scanNode->buffer[1] + scanLink->toll;
				floatType s_star = scanNode->buffer[0] + scanLink->cost;

				floatType g_star = (theta_up+(1e-12))*d_star + s_star;
				floatType g_j = (theta_up+(1e-12))*jnode->buffer[1] + jnode->buffer[0];
				floatType g_star_1 = setdecimal_round(g_star, 14);
				floatType g_j_1 = setdecimal_round(g_j, 14);
				bool condition1_1 = (g_j_1>g_star_1);
				bool condition1_2 = ((g_star_1==g_j_1)&&(setdecimal_round(d_star, 12)<setdecimal_round(jnode->buffer[1],12)));
				bool condition1 = (condition1_1 || condition1_2);
				bool condition2 = 0;
				//Re-label
				if (condition1||condition2)
				{
					//update the current tree
					jnode->buffer[0] = s_star;
					jnode->buffer[1] = d_star;
					//update link label
					TNM_SLINK* oldlink = jnode->pathElem->via;
					//scanLink in tree
					jnode->pathElem->via = scanLink;
					oldlink->buffer[6] = 0;
					scanLink->buffer[6] = 1;

					//put jnode into the H set if it is not exist
					int Hexist = 0;
					if(!Hset.empty())
					{
						for(int i=0;i<Hset.size();i++)
						{
							if(Hset[i] == jnode) Hexist = 1;
						}
					}
					if(!Hexist && jnode != origin) 
					{
						// update H set and update the number of nodes checked in the H set
						Hset.push_back(jnode);
						origin->buffer[8] += 1;
					}
				}
			}
		}
	}
	end1 =clock();
	truespptime += (end1 - start1);
	theta_low = theta_up;
	theta_up = theta_max;
	Hset.clear();
	TNM_SLINK* clink;
	TNM_SNODE* inode;
	TNM_SNODE* jnode;
	for(int li =0; li< network->numOfLink; li++)
	{
		clink  = network->linkVector[li];
		clink->buffer[0] = theta_max;
		inode = network->linkVector[li]->tail;
		jnode = network->linkVector[li]->head;
		// this link is not in the current tree
		if(clink->buffer[6] == 0)
		{
			if(setdecimal_round(inode->buffer[1] + clink->toll, 9) == setdecimal_round(jnode->buffer[1],9))
			{
				clink->buffer[0] = theta_max;
			}
			else
			{
				// find those nodes that need to put into H set
				floatType theta_a = (jnode->buffer[0] - clink->cost - inode->buffer[0])/( inode->buffer[1]+ clink->toll - jnode->buffer[1]);
				clink->buffer[0] = setdecimal(theta_a, 12);
				if (clink->buffer[0]< theta_up && clink->buffer[0]>theta_low)
				{
					theta_up = clink->buffer[0];
					Hset.clear();
					Hset.push_back(inode);
					origin->buffer[8] += 1;
				}
				else if (clink->buffer[0]== theta_up && clink->buffer[0]>theta_low)
				{
					int Hexist = 0;
					if(!Hset.empty())
					{
						for(unsigned hi=0;hi<Hset.size();hi++)
						{
							if(Hset[hi] == inode) Hexist = 1;
						}
					}
					if(!Hexist && inode != origin) 
					{
						Hset.push_back(inode);
						origin->buffer[8] += 1;
					}
				}
			}
		}

	}
	return 0;
}


int TAP_SBA::Dial_LTA_path(TNM_SORIGIN* ori, int allornothing, int addset,vector<TNM_SLINK*> & plist, floatType ptheta_up)
{
	floatType f1 = 0.0;
	floatType g1 = 0.0;
	
	// Here we don't need to sort Dset according to Topology order because it has no accumulation effects of  link volumes.
	if (allornothing == 1)
	{
		clock_t start_assign, end_assign;
		TNM_SPATH* path1;
		TNM_SNODE* jnode;
		TNM_SDEST* dest;
		while(!Dset.empty())
		{
			// get node from D set
			jnode = Dset.back();
			Dset.pop_back();
			// decide to assign on the tree
			if (fabs(jnode->buffer[4] - ptheta_up) > 1e-10)
			{
				//get dest object from node
				//start_assign = clock();
				dest = node_dest[ori->origin->id][jnode->id];
				//end_assign = clock();
				//ltachecktime += (end_assign - start_assign);
				
				if (dest == NULL)
				{
					jnode->buffer[4] = ptheta_up;
					continue;
				}
				// revert shortest path
				path1 = revert_SPath_lasttree(ori->origin, jnode, plist);
				// find a place to add theta_min in to the path set boundary list
				if (dest->pathSet.size()==0)
				{
					dest->pathSetbound.push_back(theta_min);
				}
				dest->pathSet.push_back(path1);
				// assign flow
				f1 = dest->assDemand*(votdist->MMBA1_Gtheta(ptheta_up)-votdist->MMBA1_Gtheta(jnode->buffer[4]));
				// assign one moment
				g1 = dest->assDemand*(votdist->MMBA1_Ptheta(ptheta_up)-votdist->MMBA1_Ptheta(jnode->buffer[4]));
				path1->buffer[2]=jnode->buffer[4];  // assign lower bound
				path1->buffer[3]=ptheta_up;  // assign upper bound
				jnode->buffer[4] = ptheta_up;  // change already assign boundary
				path1->flow = f1;
				update_pathlinkflow(path1, f1);  // update link flow that along the path
				dest->pathSetbound.push_back(ptheta_up);  // add boundary into boundary list
				sumo_path += (f1 * path1->PathCostS() + g1 * path1->buffer[0]);
				periter_addpathcount ++;
			}
		}
	}
	else if (allornothing==2)// T2 link-based result assignment
	{
		TNM_SNODE* jnode;
		TNM_SDEST* dest;
		TNM_SNODE *node;
		TNM_SLINK *link;
		while(!Dset.empty())
		{
			jnode = Dset.back();
			Dset.pop_back();
			if (fabs(jnode->buffer[4] - ptheta_up)>1e-10)
			{
				dest = node_dest[ori->origin->id][jnode->id];
				if (dest == NULL)
				{
					jnode->buffer[4] = ptheta_up;
					continue;
				}	
				f1 = dest->assDemand*(votdist->MMBA1_Gtheta(ptheta_up)-votdist->MMBA1_Gtheta(jnode->buffer[4]));
				g1 = dest->assDemand*(votdist->MMBA1_Ptheta(ptheta_up)-votdist->MMBA1_Ptheta(jnode->buffer[4]));

				node = jnode;
				while (node != ori->origin)
				{
					link = plist[node->id - 1];
					if (link == NULL)
					{
						cout<<"\tno shortest path found at node "<<node->id<<endl;
					}
					if (addset == 1) //direction finding in FW
					{
						link->buffer[7] += f1;// assign flow to link
						link->buffer[8] += g1;// assign flow moment to link
					}
					else if (addset == 0)//Adding flow to the network directly
					{
						link->volume += f1;
						link->buffer[1] += g1;
					}
    				node = link->tail;
				}
				jnode->buffer[4] = ptheta_up;				
			}
		}
	}
	else //not all or nothing
	{
		clock_t start_check, end_check;
		bool find;
		TNM_SPATH* path1;
		TNM_SNODE* jnode;
		TNM_SDEST* dest;
		//totalDset += Dset.size();
		while(!Dset.empty())
		{
			
			jnode = Dset.back();
			Dset.pop_back();
			if (fabs(jnode->buffer[4] - ptheta_up) > 1e-10)
			{
				
				dest = node_dest[ori->origin->id][jnode->id];
				if (dest == NULL)
				{
					jnode->buffer[4] = ptheta_up;
					continue;
				}
				start_check = clock();
				path1 = revert_SPath_lasttree(ori->origin, jnode, plist);
				end_check = clock();
				lta_specific_check_time += (end_check - start_check);
				

				f1 = dest->assDemand*(votdist->MMBA1_Gtheta(ptheta_up)-votdist->MMBA1_Gtheta(jnode->buffer[4]));
				g1 = dest->assDemand*(votdist->MMBA1_Ptheta(ptheta_up)-votdist->MMBA1_Ptheta(jnode->buffer[4]));

				jnode->buffer[4] = ptheta_up;
				sumo_path += (f1*path1->PathCostS() + g1*path1->buffer[0]);
				
				if (!addset)
				{
					depathset.push_back(path1);
				}
				

				if (addset)
				{
					
					find = add_to_pathset_path(dest, path1);
					if (find)
					{
						depathset.push_back(path1);
						//delete path1; // Delete will waste too much time!
					}
					else
					{
						periter_addpathcount++;
					}
					
				}
				
				
			}
			
		}
	}
	return 0;
}


void TAP_SBA::quickSort1(deque<TNM_SNODE*> &order, int low , int high)
{
	if( low < high)
	{
		TNM_SNODE * snode = order[low];
		int i = low;
		int j = high;
		while(i<j)
		{
			while ( (i < j) && (order[j]->buffer[7] >= snode->buffer[7]))
			{
				j=j-1;
			}
			order[i] = order[j];
			while( ( i<j ) && (order[i]->buffer[7] <= snode->buffer[7]))
			{
				i=i+1;
			}
			order[j] = order[i];
		}
		order[i] = snode;

		quickSort1(order,low,i-1);
		quickSort1(order,i+1,high);
	}
}



void TAP_SBA::T2_sortTopo(deque<TNM_SNODE*> & D,vector<TNM_SLINK*> const & pred)
{
	for(int i=0;i<D.size();i++)
	{
		floatType mark = 0;
		TNM_SNODE * snode;
		TNM_SLINK * slink = pred[D[i]->id - 1];
		while(slink != NULL)
		{
			mark=mark+1;
			snode = slink->tail;
			slink = pred[snode->id-1];
		}
		D[i]->buffer[7] = mark;
	}

	quickSort1(D,0,D.size()-1);
	for(int i=0;i<D.size();i++)
	{
		D[i]->buffer[7] = 0;
	}
}


int TAP_SBA::Dial_LTA_link(TNM_SORIGIN* ori, int allornothing, int laststep,vector<TNM_SLINK*> plist, floatType ptheta_up)
{
	floatType f1=0.0;
	floatType g1=0.0;
	TNM_SDEST* dest;
	clock_t start_assign, end_assign;
	//start_assign = clock();
	T2_sortTopo(Dset,plist);
	//end_assign =clock();
	//ltachecktime += (end_assign - start_assign);
	while(!Dset.empty())
	{
		TNM_SNODE* jnode = Dset.back();
		Dset.pop_back();
		if (fabs(jnode->buffer[4] - ptheta_up)>1e-9)
		{
			if(jnode !=ori->origin)
			{
				floatType dx = -1;
				floatType du = -1;
				dest = node_dest[ori->origin->id][jnode->id];
				if (dest==NULL)
				{
					dx = 0;
					du = 0;
				}
				else
				{
					dx = dest->assDemand*(votdist->MMBA1_Gtheta(ptheta_up)-votdist->MMBA1_Gtheta(jnode->buffer[4]));
					du = dest->assDemand*(votdist->MMBA1_Ptheta(ptheta_up)-votdist->MMBA1_Ptheta(jnode->buffer[4]));
				}				
				if (plist[jnode->id - 1] != NULL)
				{
					TNM_SLINK* elink = plist[jnode->id - 1];
					TNM_SNODE* inode = elink->tail;
					elink->buffer[4] = elink->buffer[4] + jnode->buffer[5] + dx; //update elink->x
					elink->buffer[5] = elink->buffer[5] + jnode->buffer[6] + du; // update elink->u
					inode->buffer[5] = inode->buffer[5] + jnode->buffer[5] + dx; //update inode-> x
					inode->buffer[6] = inode->buffer[6] + jnode->buffer[6] + du;// update inode-> u
				}
				jnode->buffer[5] = 0.0; //reset the jnode->x
				jnode->buffer[6] = 0.0;//reset the jnode->u
				jnode->buffer[4] = ptheta_up;

			}
		}
	}
	
	
	TNM_SLINK* li;
	if (allornothing==1)
	{
		for(int i =0; i<network->numOfLink;i++)
		{
			li = network->linkVector[i]; 
			li->volume += li->buffer[4];
			li->buffer[1] += li->buffer[5];
			li->buffer[7] += li->buffer[4];
			li->buffer[8] += li->buffer[5];
			li->buffer[4] = 0.0;
			li->buffer[5] = 0.0;
		}
	}
	else
	{
		for(int i =0; i<network->numOfLink;i++)
		{
			li = network->linkVector[i]; 
			li->buffer[7] += li->buffer[4];
			li->buffer[8] += li->buffer[5];
			li->buffer[4] = 0.0;
			li->buffer[5] = 0.0;
		}
	}
	
	return 0;
}


int TAP_SBA::Dial_MPA_link(TNM_SORIGIN* origin, int allornothing, int addset)
{
	clock_t start1, end1, start2, end2;
	for(int ni=0;ni<network->numOfNode;ni++)
	{
		//allocate float memory for node variables, 0 is node time(s-node), 1 is node toll(d-node); 2 is x_node(LTA),3 is u_node(LTA);
		//4 is a_node(LTA); 5 is d_x(LTA), 6 is d_u(LTA);
		network->nodeVector[ni]->buffer[0] = 0;// node time
		network->nodeVector[ni]->buffer[1] = 0;// node toll
		network->nodeVector[ni]->buffer[2] = 0;// nodeHsetinit 
		network->nodeVector[ni]->buffer[4] = theta_min; //a_node
		network->nodeVector[ni]->buffer[5] = 0;//d_x
		network->nodeVector[ni]->buffer[6] = 0;//d_u
	}
	for(int li=0;li<network->numOfLink;li++)
	{
		// allocate float memory for link variables, 0 is a, 1 is u, 2 is new x, 3 is new u
		//4 is origin-based new flow, 5 is origin-based new u, 6 is link label: 0 is not in the tree, 1 is in the tree; 7 is x_link, 8 is u_link(LTA);
		network->linkVector[li]->buffer[0] = -1;//set the link a as -1//what -1 use?
		network->linkVector[li]->buffer[6] = 0;// link label
		network->linkVector[li]->buffer[4] = 0;//x_link
		network->linkVector[li]->buffer[5] = 0;//u_link
	}
	//reset the lower bounds and upper bounds of theta
	theta_low = theta_min;
	theta_up = theta_max;
	//initialize the shortest path tree
	Dial_MIN_path(origin);
	spptreenum += 1;
	vector<TNM_SLINK*>   pllist(network->numOfNode,NULL);
	floatType ptheta_up = theta_min;
	do
	{
		spptreenum += 1;
		//record the last tree
		for(unsigned i=0; i<network->numOfNode; i++)
		{
			pllist[i] = network->nodeVector[i]->pathElem->via;
		}
		ptheta_up = theta_up;
		start1 = clock();
		Dial_PTU_path(origin->origin);
		end1 = clock();
		shortestpathtime += (end1-start1);
		start2 = clock();
		Dial_LTA_link(origin, allornothing, 0, pllist, ptheta_up);
		end2 = clock();
		ltatime += (end2 - start2);
	}while(theta_up != theta_max);//theta_up != theta_max
	for(unsigned i=0; i<network->numOfNode; i++)
	{
		pllist[i] = network->nodeVector[i]->pathElem->via;
		if (network->nodeVector[i] != origin->origin)
		{
			Dset.push_back(network->nodeVector[i]);
		}
	}
	ptheta_up = theta_up;
	start2 = clock();
	Dial_LTA_link(origin, allornothing, 1, pllist, ptheta_up);
	end2 = clock();
	ltatime += (end2 - start2);
	return 0;
}


TNM_SPATH* TAP_SBA::revert_SPath_lasttree(TNM_SNODE *origin, TNM_SNODE *dest, vector<TNM_SLINK*> & plist)
{
	clock_t start_check, end_check;
	TNM_SNODE *node;
	TNM_SLINK *link;
	int count = 0;
	start_check = clock();
	totalDset += 1;
	TNM_SPATH *path = new TNM_SPATH; //allocate memory for new path
	end_check = clock();
	ltachecktime += (end_check - start_check);
	assert(path!=0);
	node = dest;
	while (node != origin)
	{
		link = plist[node->id - 1];
		if (link == NULL)
		{
			cout<<"\tno shortest path found at node "<<node->id<<endl;
			delete path;
			return NULL;
		}
    	node = link->tail;
		path->path.push_back(link);
		count++;
		if(count>network->numOfNode)
		{
			cout<<"Error: shortest path tree contains a cycle!"<<endl;
			cout<<"Report: origin = "<<origin->id<<" dest = "<<dest->id<<" path be generated so far "<<endl;
			delete path;
			return NULL;
		}
	}
	//this is to recycling unused memory allocated for path.
	vector<TNM_SLINK*>(path->path).swap(path->path); 
	//make the path is from origin to dest.
	reverse(path->path.begin(), path->path.end()); 
	path->cost = 0.0;
	path->buffer[0] = 0.0;
	for(int i=0; i<path->path.size();i++)
	{
		path->cost += path->path[i]->cost;
		path->buffer[0] += path->path[i]->toll;
	}
	return path;
}


bool TAP_SBA::add_to_pathset_path(TNM_SDEST* dest, TNM_SPATH* pa)
{
	bool find;
	TNM_SPATH* temppath;
	for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin(); pv != dest->pathSet.end();pv++)
	{
		find = true;
		temppath = *pv;
		if (temppath->path.size() == pa->path.size()) // contains same number of links or not
		{
			for (int li=0; li< pa->path.size();li++)
			{
				if (temppath->path[li] != pa->path[li])
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
		if(find)
		{
			break;
		}
	}
	if (!find)
	{
		bool addin = false;
		pa->buffer[1] = 1.0;  // new found path, used for adding boundary to the path set bound list
		//find a place in the path set to add
		for(int k=0; k<dest->pathSet.size(); k++)
		{
			if (pa->buffer[0] >= dest->pathSet[k]->buffer[0]) // add into path set according to the reverse toll order
			{
				addin = true;
				pa->flow = 0.0;
				dest->pathSet.insert(dest->pathSet.begin() + k, pa);
				break;
			}
		}
		if (addin == false)
		{
			pa->flow=0.0;
			dest->pathSet.push_back(pa);
		}
	}
	return find;
}




void TAP_SBA::tree_num_output(string outname)
{
	ofstream out;
	TNM_OpenOutFile(out,outname);
	for (int i=0;i<tree_num.size();i++)
	{
		if (i%network->numOfOrigin==network->numOfOrigin-1)
		{
			out<<to_string(tree_num[i])<<"\n";
		}
		else
		{
			out<<to_string(tree_num[i])<<" ";
		}
	}
}






/////////////////////////////////////////////////////////////////////////////////
//Frank-Wolfe Algorithm(FW) for continuous bi-criteria traffic assignment(C-BiTA)
/////////////////////////////////////////////////////////////////////////////////
//Constructors
TAP_T2theta::TAP_T2theta()
{
	lineSearchAccuracy = 1e-4;
}

// Constructors with input
TAP_T2theta::TAP_T2theta(floatType votmin1, floatType votmax1, string votname1, int treenumber1)
{
	cout<<"Enter TAP_T2theta(): Frank-Wolfe Algorithm(FW) for continuous bi-criteria traffic assignment(C-BiTA)"<<endl;
	votdist = new VOT_DIST(votmin1, votmax1, votname1, treenumber1);//Right? I don't know.
	theta_max = 1.0/votmin1;
	theta_min = 1.0/votmax1;
	lineSearchAccuracy = 1e-4;
}

// Destructors
TAP_T2theta::~TAP_T2theta()
{
}

// Initialization for the T2 (theta) FW algorithm
void TAP_T2theta::Initialize()
{
	convIndicator = 1.0;
	network->AllocatePathBuffer(4);
	network->AllocateLinkBuffer(9); 
	network->AllocateNodeBuffer(9); 
	//find dest of a node
	map_node_dest();
	for(int oi=0;oi<network->numOfOrigin;oi++)
	{
		Dial_MPA_link(network->originVector[oi], 1, 0);
	}
	for(int i=0;i<network->numOfLink;i++)
	{
		TNM_SLINK* link = network->linkVector[i];
		sumo_path += (network->linkVector[i]->volume*network->linkVector[i]->cost + network->linkVector[i]->buffer[1]*network->linkVector[i]->toll);
		link->cost = link->GetCost();
	}
	T2theta_Objective();
	cout<<"Object = "<<OFV<<endl;
}

// Main loop for the T2 (theta) FW algorithm
void TAP_T2theta::MainLoop()
{
	badIter = 1;
	ziggIter = 1;
	if(curIter == 1)
	{
		convIndicator = 1;
	}
	for(int i=0;i<network->numOfLink;i++)
	{
		network->linkVector[i]->buffer[2] = 0;
		network->linkVector[i]->buffer[3] = 0;
		network->linkVector[i]->buffer[7] = 0;
		network->linkVector[i]->buffer[8] = 0;
	}
	for(int oi=0;oi<network->numOfOrigin;oi++)
	{
		Dial_MPA_link(network->originVector[oi], 0, 0);
	}

	for(int i=0;i<network->numOfLink;i++)
	{
		network->linkVector[i]->buffer[2] += (network->linkVector[i]->buffer[7] - network->linkVector[i]->volume);
		network->linkVector[i]->buffer[3] += (network->linkVector[i]->buffer[8] - network->linkVector[i]->buffer[1]);
	}
	T2theta_GAP();
	badIter += T2theta_lineSearch(1.0);
	T2theta_updateSolution();
	cout<<" Relative gap="<<convIndicator<<" OFV="<<OFV<<endl;
	T2theta_Objective();
	if (badIter>maxbadIter)
	{
		maxMainIter = 1;
	}
}

// Calculate the relative gap for T2 (theta) FW algorihtm
void TAP_T2theta::T2theta_GAP()
{
	floatType numerator = 0.0;
	floatType denominator = 0.0;
	TNM_SLINK* link;
	for(int i=0; i<network->numOfLink; i++)
	{
		link = network->linkVector[i];
		floatType xlambdatime = GetBPRLinkTime(link, link->volume);
		numerator += (xlambdatime * (link->volume - link->buffer[7]) + link->toll * (link->buffer[1] - link->buffer[8]));
		denominator += (xlambdatime * link->volume + link->toll*link->buffer[1]);
	}
	convIndicator = numerator/denominator;
}

// Line search for T2 (theta) FW algorithm
int TAP_T2theta::T2theta_lineSearch(floatType maxSize)
{
	floatType dz;
	if(L_function(1, 1)<=0)
	{
		stepSize = 1;
	}
	else
	{
		floatType a =0;
		floatType b=maxSize;
		dz = 1.0;
		int iter=0;
		while( dz > 0 || (iter < maxLineSearchIter && (b-a)>=lineSearchAccuracy * maxSize) )
		{
			stepSize = (a+b)/2.0;
			iter++;
			dz = L_function(stepSize, 1);
			if (dz<0.0) 
				a = stepSize;
			else
				b=stepSize;
			numLineSearch++;
		}
	}
	if (dz<0) return 0;
	else return 1;
}

// L function which as different meaning with different label
floatType TAP_T2theta::L_function(floatType la, int label)
{
	floatType re = 0;
	//total cost according to the new x and new u
	if(label == 0)
	{
		for(int i=0;i<network->numOfLink;i++)
		{
			TNM_SLINK* slink = network->linkVector[i];
			re += slink->toll*slink->buffer[8] + GetBPRLinkTime(slink, slink->volume + la*slink->buffer[2])*slink->buffer[7];
		}
	}
	//cal the derivative of objective with respect to step size
	else if ( label == 1)
	{
		for(int i=0;i<network->numOfLink;i++)
		{
			TNM_SLINK* slink = network->linkVector[i];
			re += slink->toll*slink->buffer[3]+GetBPRLinkTime(slink,slink->volume+la*slink->buffer[2])*slink->buffer[2];
		}
	}
	//cal total cost according to the current x and u
	else if( label == 2) 
	{
		for(int i=0;i<network->numOfLink;i++)
		{
			TNM_SLINK* slink = network->linkVector[i];
			re += slink->toll*slink->buffer[1] + slink->cost*slink->volume;
		}
	}
	return re;
}

// Calculate link BPR travel time given a link volume
floatType TAP_T2theta::GetBPRLinkTime(TNM_SLINK * slink, floatType volume)
{
	floatType time =0;
	floatType oldv = slink->volume;
	slink->volume=volume;
	time = slink->GetCost();
	slink->volume=oldv;
	return time;
}

// Update the solution based on the line search result.
void TAP_T2theta::T2theta_updateSolution()
{
	TNM_SLINK* link;
	for(int i=0;i<network->numOfLink;i++)
	{
		link = network->linkVector[i];
		link->volume=link->volume+ stepSize*link->buffer[2];
		link->buffer[1] = link->buffer[1] + stepSize*link->buffer[3];
		link->cost = link->GetCost();
	}
}

// Calculate the objective function value
void TAP_T2theta::T2theta_Objective()
{
	floatType obj=0;
	for(int i=0;i<network->numOfLink;i++)
	{
		TNM_SLINK* link=network->linkVector[i];
		obj+= (link->fft*link->volume+ ((0.03*link->fft)/(pow(link->capacity,4)))*pow(link->volume,5) + link->toll * link->buffer[1]);
	}
	OFV = obj;
}

// Read toll file  for T2 (theta) FW algorithm
int TAP_T2theta::T2theta_ReadToll(const string & path)
{
	string filename = path+"_tol.tntp";
	ifstream inFile(filename,ios::in);
	if(!inFile)
	{
		cerr<<"Could not open" + path + "_tol.tntp file"<<endl;
		exit(1);
	}
	int num, low, up;
	inFile>>num>>low>>up;
	for(int i=0;i<num;i++)
	{
		int inode;
		int jnode;
		floatType toll;
		inFile>>inode>>jnode>>toll;
		if(toll<low)
		{
			toll =low;
		}
		if(toll>up)
		{
			toll =up;
		}
		TNM_SNODE* snode;
		snode = network->nodeVector[inode-1];
		for(vector<TNM_SLINK*>::iterator pv=snode->forwStar.begin();pv!=snode->forwStar.end();pv++)
		{
			TNM_SLINK * slink = *pv;
			if(slink->head->id == jnode)
			{
				slink->toll = toll;
			}
		}

	}

	inFile.close();
	return 0;
}


