#include "header\stdafx.h"
#include <math.h>
#include <stack>

const int MaxNumOfThreads = 4 ;
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


/*   continuous VOT   */
VOT_DIST::VOT_DIST()
{
	vot_min = 0;
	vot_max = 0;
	votdist_name = "uni";
}

VOT_DIST::VOT_DIST(floatType votmin, floatType votmax, string distname, int tree_number)
{
	//cout<<"Enter build VOT_DIST"<<endl;
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
		if (((1/vot_max)*first_k + first_c0  < 0) || ((1/vot_min)*first_k + first_c0  < 0))
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
		rational_a = (1.0 + rho)*(theta_max - theta_min);
		rational_b = -1.0 * rho * theta_min + theta_max - theta_min;
		rational_rho = rho;
	}
}

VOT_DIST::~VOT_DIST()
{
}

floatType VOT_DIST::vot_pdf(TNM_SDEST* dest)//calc gradient
{
	floatType dF;
	if (votdist_name == "uni")
	{dF = dest->assDemand / (vot_max - vot_min);}
	if (votdist_name == "first")
	{
		dF = 1;
	}
	if (votdist_name == "exp")
	{
		dF = 1;
	}
	if (votdist_name == "rational")
	{
		dF = 1;
	}
	return dF;
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
			result = 1.0/(1.0/vot_min - 1.0/vot_max)*0.5*pow(theta, 2);
		}
		else
		{
			result = rational_a/pow(rational_rho, 2)*log(rational_rho*theta + rational_b) + rational_a * rational_b/pow(rational_rho, 2)*1.0/(rational_rho*theta + rational_b);
			// constant is ignore here.
		}
	}
	if (votdist_name == "theta_uni")
	{
		result = 0.5 / (1.0/vot_min - 1.0/vot_max) * (pow(theta, 2) - pow(1.0/vot_max, 2));
	}
	return result;
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
	if (votdist_name == "theta_uni")
	{
		result = 1.0 / (1.0/vot_min - 1.0/vot_max);
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
		result = -1.0 * (rational_a* rational_rho)/(pow(rational_rho*theta + rational_b, 3));
	}
	if (votdist_name == "theta_uni")
	{
		result = 0.0;
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
		result = 1 + (theta-1/vot_min)/(rational_rho*theta + rational_b);
	}
	if (votdist_name == "theta_uni")
	{
		result = (theta - 1.0/vot_max)/(1.0/vot_min - 1.0/vot_max);
	}
	return result;
}

void VOT_DIST::calc_cdf_gra(TNM_SNET* net)
{
	net->AllocateDestBuffer(1);
	TNM_SORIGIN *org;
	TNM_SDEST *dest;
	for(int i = 0;i<net->numOfOrigin;i++)
	{
		org = net->originVector[i];
		for(int j = 0;j<org->numOfDest;j++)
		{
			dest = org->destVector[j];
			dest->buffer[0]= vot_pdf(dest);
		}
	}
}



/* Bicriteria Shortest Path Problem with continuous VOT.*/
TAP_MMBA1_o::TAP_MMBA1_o()
{
	aveInnerGap=50;
}

TAP_MMBA1_o::TAP_MMBA1_o(floatType votmin1, floatType votmax1, string votname1, int treenumber1)
{
	//cout<<"Enter TAP_MMBA1_o()"<<endl;
	algorNameCard = "Multiclass Multicriter Bound Adjust algorithm based theory";
	votdist = new VOT_DIST(votmin1, votmax1, votname1, treenumber1);
	aveInnerGap =1;
}

TAP_MMBA1_o::~TAP_MMBA1_o()
{
	//cout<<"delete TAP_MMBA1_o()"<<endl;
}

floatType TAP_MMBA1_o::setdecimal_lower(floatType fl,int num)
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
		b=(floor)((fl*1000000000))/1000000000.0;
		cout<<"The large number is 9"<<endl;
	}
	return b;
}

floatType TAP_MMBA1_o::setdecimal(floatType fl,int num)
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
	case 12:
		b=(ceil)((fl*1000000000000))/1000000000000.0;
		break;
	case 13:
		b=(ceil)((fl*10000000000000))/10000000000000.0;
		break;
	default:
		b=(ceil)((fl*1000000000))/1000000000.0;
		cout<<"The large number is 9"<<endl;
	}
	return b;
}

floatType TAP_MMBA1_o::setdecimal_round(floatType fl,int num)
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
		case 12:
			b=(floor)((fl*1000000000000+0.5))/1000000000000.0;
			break;
		case 13:
			b=(floor)((fl*10000000000000+0.5))/10000000000000.0;
			break;
		case 14:
			b=(floor)((fl*100000000000000+0.5))/100000000000000.0;
			break;
		case 16:
			b=(floor)((fl*10000000000000000+0.5))/10000000000000000.0;
			break;
	default:
		b=(floor)((fl*1000000000+0.5))/1000000000.0;
		cout<<"The large number is 9"<<endl;
	}
	return b;
}

TNM_SPATH* TAP_MMBA1_o::GetSPath(TNM_SNODE *origin, TNM_SNODE *dest, int treenum)
{
	TNM_SNODE *node;
	TNM_SLINK *link;
	int count = 0;
	TNM_SPATH *path = new TNM_SPATH; //allocate memory for new path
	assert(path!=0);
	node = dest;
	while (node != origin)
	{
		link = node->pathElemVector[treenum]->via;
		if (link == NULL) //there may exist no shortest path at all, or there must be sth wrong.
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
			path->Print(true);
			delete path;
			return NULL;
		}
	}
	vector<TNM_SLINK*>(path->path).swap(path->path); //this is to recycling unused memory allocated for path.
	reverse(path->path.begin(), path->path.end()); //make the path is from origin to dest.
	path->cost = 0.0;
	path->buffer[0] = 0.0;
	for(int i=0; i<path->path.size();i++)
	{
		path->cost += path->path[i]->cost;
		path->buffer[0] += path->path[i]->toll;
	}
	return path;
}


TNM_SPATH* TAP_MMBA1_o::updateMMSPOD(TNM_SORIGIN* ori, TNM_SDEST* dest, floatType tempvot, int treenum)
{
	TNM_SNODE* node;
	TNM_SLINK* link;
	TNM_SPATH* path;
	multimap<floatType, TNM_SNODE*, less<floatType>> Q;
	for (int i=0; i<network->numOfNode; i++)
	{
		node = network->nodeVector[i];
		node->pathElemVector[treenum]->cost = POS_INF_FLOAT;
		node->pathElemVector[treenum]->via = NULL;
		node->scanStatus = 0;
	}
	ori->origin->pathElemVector[treenum]->cost = 0;
	ori->origin->scanStatus = 1;
	Q.insert(std::pair<floatType, TNM_SNODE*>(0.0, ori->origin));
	///////////////////////////////////////////////////////////////////////////////
	dest->buffer[0] += 1; 
	dest->buffer[1] += 1;
	///////////////////////////////////////////////////////////////////////////////
	int niter = 0;
	multimap<floatType, TNM_SNODE*, less<floatType>>::iterator firstElem;
	while(!Q.empty())
	{
		niter++;
		firstElem = Q.begin();
		node = firstElem->second;
		node->scanStatus = 0;
		Q.erase(firstElem);
		if (node->id == dest->dest->id)
		{
			
			break;
		}//has reach the destination!
		for(vector<TNM_SLINK*>::iterator pv = node->forwStar.begin(); pv!=node->forwStar.end(); pv++)
		{
			link = *pv;
			TNM_SNODE* headnode = link->head;
			floatType t = node->pathElemVector[treenum]->cost + tempvot*link->cost + link->toll;
			if(headnode->pathElemVector[treenum]->cost > t)
			{
				if (headnode->scanStatus == 0)//never scan
				{
					Q.insert(std::pair<floatType, TNM_SNODE*>(t, headnode));
					///////////////////////////////////////////////////////////////////////////////
					dest->buffer[0] += 1;
					///////////////////////////////////////////////////////////////////////////////
					headnode->scanStatus = 1;
				}
				else//has scaned
				{
					multimap<floatType, TNM_SNODE*, less<floatType> >::iterator lower =Q.lower_bound(headnode->pathElemVector[treenum]->cost - 1e-10),
					upper = Q.upper_bound(headnode->pathElemVector[treenum]->cost + 1e-10), piter;
					piter = lower;
					while(piter!=upper && piter->second->id!=headnode->id)
					{
						piter++;
					}
					if(piter == Q.end()) 
					{
						cout<<"Warning: cannot find headnode, it's cost is "<<headnode->pathElemVector[treenum]->cost<<endl;
						cout<<"\tCurrently, Q has the following nodes"<<endl;
                        for(lower = Q.begin(); lower!=Q.end();lower++)
								cout<<lower->second->id<<", "<<lower->first<<","<<lower->second->pathElemVector[treenum]->cost<<endl;
					}
					if(piter->second!=headnode)
					{
						cout<<"\tError: failed to find headnode "<<headnode->id<<" while it is in the queue."<<endl;
					}
					else
					{
						Q.erase(piter);
						Q.insert(std::pair<floatType, TNM_SNODE*>(t, headnode));
						///////////////////////////////////////////////////////////////////////////////
						dest->buffer[0] += 1;
						///////////////////////////////////////////////////////////////////////////////
					}
				}
				headnode->pathElemVector[treenum]->cost = t;
				headnode->pathElemVector[treenum]->via  = *pv;
			}
		}
	}
	path = GetSPath(ori->origin, dest->dest, treenum);
	if(path == NULL) 
	{
		cout<<"Cannot find shortest path between origin "<<ori->origin->id
			<<" and dest "<<dest->dest->id<<endl;
        cout<<"\tPossible reasons:\n1. Network is not connected.\n2.Link cost has extremely large value.\n3.For TAPAS format, the first through node setting in _net file may be inappropriate."
            <<endl;
		return NULL;
	}
	else return path;
}

void TAP_MMBA1_o::MMall_or_nothing(TNM_SDEST* dest)
{
	floatType tempratio, ratio = 0.0;
	for (int i=1; i<dest->pathSetbound.size();i++)//start from 1
	{
		dest->pathSet[i-1]->buffer[2] = dest->pathSetbound[i-1];//add_bound_set
		dest->pathSet[i-1]->buffer[3] = dest->pathSetbound[i];//add_bound_set

		ratio = votdist->MMBA1_Gtheta(dest->pathSetbound[i]);
		//parameter can change
		if (dest->assDemand*fabs(ratio - tempratio) > 1e-12)
		{
			dest->pathSet[i - 1]->flow = dest->assDemand*(ratio - tempratio);
			for (int j=0; j<dest->pathSet[i - 1]->path.size(); j++)
			{
				dest->pathSet[i-1]->path[j]->volume += dest->assDemand*(ratio - tempratio);//update link volume
			}
		}
		tempratio = ratio;
		//parameter can change
		if ((i==dest->pathSetbound.size()-1)&&(fabs(tempratio-1)> 1e-9))
		{
			cout<<"o:"<<dest->origin->id<<"d:"<<dest->dest->id<<"violate demand conservation"<<endl;
		}
	}
}
void TAP_MMBA1_o::Likelypath(TNM_SORIGIN* ori, TNM_SDEST* dest, int tree_num)
{
	floatType alpha1 = 0.0;
	int i = 0;
	int position = 0;
	floatType vot_min = votdist->vot_min;
	floatType vot_max = votdist->vot_max;
	dest->pathSetbound.push_back(1.0/vot_max);//Add min first
	TNM_SPATH* path1 = updateMMSPOD(ori, dest, vot_max, tree_num);//max toll path
	TNM_SPATH* path2 = updateMMSPOD(ori, dest, vot_min, tree_num);//min toll path
	dest->pathSet.push_back(path1);
	//parameter can change
	if ((fabs(path2->PathCostS() - path1->PathCostS()) < 1e-6)&&(fabs(path2->buffer[0] - path1->buffer[0])< 1e-6))
	{
		dest->pathSetbound.push_back(1.0/vot_min);
		return;}
	else {dest->pathSet.push_back(path2);}
	while (1)
	{
		alpha1 = (dest->pathSet[position]->buffer[0] - dest->pathSet[position+1]->buffer[0])/(dest->pathSet[position+1]->PathCostS() - dest->pathSet[position]->PathCostS());
		TNM_SPATH* pathn = updateMMSPOD(ori, dest, alpha1, tree_num);//µ±Ê±ÓÃÁËtry except
		//parameter can change
		if (((fabs(pathn->PathCostS() - dest->pathSet[position+1]->PathCostS()) > 1e-6)&&(fabs(pathn->buffer[0] - dest->pathSet[position+1]->buffer[0])> 1e-6))
			&&((fabs(pathn->PathCostS() - dest->pathSet[position]->PathCostS()) > 1e-6)&&(fabs(pathn->buffer[0] - dest->pathSet[position]->buffer[0])> 1e-6))&&
			(fabs((dest->pathSet[position]->buffer[0] - pathn->buffer[0])/(pathn->PathCostS()-dest->pathSet[position]->PathCostS()) - alpha1) >= 1e-6))
		{
			dest->pathSet.insert(dest->pathSet.begin() + position + 1, pathn);		
		}
		else
		{
			position += 1;
			delete pathn;
		}
		if (position >= dest->pathSet.size()-1)
		{
			break;
		}
	}
	//Calculate the boundary of paths in pathset
	TNM_SPATH *path;
	PTRPATH pv = dest->pathSet.begin();
	bool iserase;
	int pi = 0;
	while(pv!=dest->pathSet.end()-1)
	{
		path = *pv;
		iserase = false;
		if (dest->pathSet[pi]->buffer[0] <= dest->pathSet[pi+1]->buffer[0])
		{cout<<"The order of pathset is wrong!"<<"o="<<ori->origin->id<<"d="<<dest->dest->id<<endl;
		cout<<"toll1:"<<dest->pathSet[pi]->buffer[0]<<"toll2:"<<dest->pathSet[pi+1]->buffer[0]<<endl;}
		floatType tempdist = (dest->pathSet[pi]->PathCostS() - dest->pathSet[pi+1]->PathCostS()) / (dest->pathSet[pi+1]->buffer[0] - dest->pathSet[pi]->buffer[0]);
		if (fabs(tempdist-dest->pathSetbound[pi])<=1e-9)
		{
			iserase = true;
		}
		if (iserase)
		{
			pv = dest->pathSet.erase(pv);
			delete path;
		}
		else
		{
			dest->pathSetbound.push_back(tempdist);
			pv++;
			pi += 1;
		}
		
	}
	if (fabs(dest->pathSetbound[dest->pathSet.size()-1]-1.0/vot_min)>1e-9)
	{
		dest->pathSetbound.push_back(1.0/vot_min);
	}//Add max last
	else
	{
		dest->pathSet.erase(dest->pathSet.begin()+dest->pathSet.size()-1);
	}
	return;
}


int TAP_MMBA1_o::modified_T2_shortestpath(TNM_SNODE * rootNode,vector<floatType> &gv)
{
	deque<TNM_SNODE*> nl;
	TNM_SNODE *node;
	//initialize node for shortest path calculation
	for (int j = 0;j<network->numOfNode;j++)
	{
		node = network->nodeVector[j];
		node->InitPathElem();            
		node->scanStatus = 0;
		node->buffer[0] = 0; //reset node time 
		node->buffer[1] = 0; //reset node toll
	}
	//call scanList 's major method to compute shortest path
	//insert the root into scanList, whatever data strucrue it uses.
	TNM_SNODE *curNode;
	//InitRoot(rootNode);
	rootNode->pathElem->cost = 0;
	//InsertANode(rootNode);
	nl.push_back(rootNode);
	//////////////////////////////////////////////////////////
	rootNode->buffer[8] += 1;
	/////////////////////////////////////////////////////////
	//curNode = GetNextNode();
	if (!nl.empty()) //if the list is not empty
	{
		curNode = nl.front(); // get the current node
		nl.pop_front(); //delete it from the deque;
		curNode->scanStatus = -1; //mark it as been used but not in queue right now.
	}
	else
		curNode = NULL;
	while (curNode != NULL)
	{
		if(curNode->m_isThrough || curNode == rootNode)// meigai
		{
		   //curNode->SearchMinOutLink(this);
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
						///////////////////////////////////////////////////////////////////////////
						//scanNode->pathElem->via  = *pv;
						if (scanNode->pathElem->via != NULL)
						{
							scanNode->pathElem->via->buffer[6]=0;
						}
						scanNode->pathElem->via  = *pv;
						scanNode->pathElem->via->buffer[6] = 1;
						scanNode->buffer[0] = curNode->buffer[0] + (*pv)->cost; // ini the node time
						scanNode->buffer[1] = curNode->buffer[1] + (*pv)->toll; // ini the node toll
						//InsertANode()
    					if(scanNode->scanStatus == 0) // if never been used, insert it to the back of dq
						{
							nl.push_back(scanNode); 
							//////////////////////////////////////////////////////////
							rootNode->buffer[8] += 1;
							/////////////////////////////////////////////////////////
							scanNode->scanStatus = 1; // now being used
						}
						else if (scanNode->scanStatus == -1) // if it has ever been used, insert it to the front of dq
						{
							nl.push_front(scanNode); 
							//////////////////////////////////////////////////////////
							rootNode->buffer[8] += 1;
							/////////////////////////////////////////////////////////
							scanNode->scanStatus = 1; // now being used
						}
					}// end if
				}
			}
	   }
		//curNode = GetNextNode();
	   if (!nl.empty()) //if the list is not empty
	   {
		   curNode = nl.front(); // get the current node
		   nl.pop_front(); //delete it from the deque;
		   curNode->scanStatus = -1; //mark it as been used but not in queue right now.
	   }
	   else
		   curNode = NULL;
   }
	return 0;
}

TNM_SPATH* TAP_MMBA1_o::revert_SPath(TNM_SNODE *origin, TNM_SNODE *dest)
{
	TNM_SNODE *node;
	TNM_SLINK *link;
	int count = 0;
	TNM_SPATH *path = new TNM_SPATH; //allocate memory for new path
	assert(path!=0);
	node = dest;
	while (node != origin)
	{
		for (int li=0; li<node->backStar.size(); li++)
		{
			if (node->backStar[li]->buffer[6]==1)
			{
				link = node->backStar[li];
				break;
			}
		}
		if (link == NULL) //there may exist no shortest path at all, or there must be sth wrong.
		{
			cout<<"\t revert_SPath no shortest path found at node "<<node->id<<endl;
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
			path->Print(true);
			if (origin->id==41 && dest->id == 8)
			{
				for(int i=0; i<network->numOfLink; i++)
				{
					cout<<network->linkVector[i]->tail->id<<"->"<<network->linkVector[i]->head->id<<"buffer[6]="<<network->linkVector[i]->buffer[6]<<endl;
				}
				system("PAUSE");
			}
			delete path;
			return NULL;
		}
	}
	vector<TNM_SLINK*>(path->path).swap(path->path); //this is to recycling unused memory allocated for path.
	reverse(path->path.begin(), path->path.end()); //make the path is from origin to dest.
	//cout<<"Its here"<<endl;
	path->cost = 0.0;
	path->buffer[0] = 0.0;
	for(int i=0; i<path->path.size();i++)
	{
		path->cost += path->path[i]->cost;
		path->buffer[0] += path->path[i]->toll;
	}
	return path;
}

void TAP_MMBA1_o::update_pathlinkflow(TNM_SPATH* pa, floatType addvalue)
{
	for (int j=0; j<pa->path.size(); j++)
	{
		pa->path[j]->volume += addvalue;
	}
}

void TAP_MMBA1_o::printpath(TNM_SPATH* pa)
{
	cout<<pa->path[0]->tail->id<<"->";
	for (int pi=0;pi<pa->path.size(); pi++)
	{
		cout<<pa->path[pi]->head->id<<"->";
	}
	cout<<endl;
}

void TAP_MMBA1_o::Set_addpathbound(TNM_SDEST* dest)
{
	TNM_SPATH* temppath2;
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
			// initial new add buffer
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

bool TAP_MMBA1_o::check_money(TNM_SDEST* dest)
{
	for (int i=0;i<dest->pathSet.size()-1;i++)
	{
		if (dest->pathSet[i]->buffer[0] - dest->pathSet[i+1]->buffer[0]<0)
		{
			cout<<"----------------Money order is wrong!-------------"<<dest->origin->id<<"->"<<dest->dest->id<<"previous"<<"i="<<i<<" moeny:"<<dest->pathSet[i]->buffer[0]<<"after"<<"i="<<(i+1)<<" moeny:"<<dest->pathSet[i+1]->buffer[0]<<endl;
			return false;
		}
	}
	return true;
}

void TAP_MMBA1_o::init_pathElemVector(int columngenerationnum)
{
	TNM_SNODE *node;
	for(int i=0; i<network->numOfNode;i++)
	{
		node = network->nodeVector[i];
		for(int j=0; j<columngenerationnum; j++)
		{
			PATHELEM* pathelem = new PATHELEM;
			node->pathElemVector.push_back(pathelem);
		}
	}
}

void TAP_MMBA1_o::bi_spp_path_output(const string &path, const string &name, const string &algname, double usetime)
{
	TNM_SORIGIN* pOrg;
	ofstream out;
	ofstream out2;
	TNM_OpenOutFile(out,path  + name + "_" + algname + ".info");
	TNM_OpenOutFile(out2,path  + name +"_" + algname +  ".pfp");
	out<<"O\tD\tEffPathNum\tBoundaries\n";
	out2<<"PathID        Origin      Dest      Demand      Flow      Time     Toll     num of link     links   \n";
	int pthid = 0;
	for (int i=0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		for (int j=0; j<pOrg->numOfDest; j++)
		{
			TNM_SDEST* dest=network->originVector[i]->destVector[j];
			out<<TNM_IntFormat(dest->origin->id, 4)<<TNM_IntFormat(dest->dest->id, 4)<<TNM_IntFormat(dest->pathSetbound.size()-1, 4);
			for (int k=0; k<dest->pathSetbound.size(); k++)
			{
				out<<TNM_FloatFormat(dest->pathSetbound[k], 12, 6);
			}
			out<<"\n";
			for (int u=0; u<dest->pathSet.size(); u++)
			{
				pthid ++;
				////////////////////////////////////
				//dest->buffer[2] += 1;
				////////////////////////////////////
				TNM_SPATH* path = dest->pathSet[u];
				out2<<TNM_IntFormat(pthid,4)<<TNM_IntFormat(pOrg->id(),4)<<TNM_IntFormat(dest->dest->id,4)<<TNM_FloatFormat(dest->assDemand,12,6)<<TNM_FloatFormat(path->flow,12,6)
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
	cout<<"\tWriting path details into file "<<(path  + name +"_" + algname +  ".pfp")<<"..."<<endl;
}


int TAP_MMBA1_o::cout_Q_node_count( const string &path, const string &name, const string &algname)
{
	int totalnodenum = 0;
	int totalpathnum = 0;
	int totalsppnum = 0;
	TNM_SORIGIN* pOrg;
	ofstream out;
	TNM_OpenOutFile(out,path  + name + "_" + algname + ".nodecount");
	out<<"O\tD\tcheck node count\n";
	for (int i=0;i<network->numOfOrigin;i++)
	{
		int origin_total = 0;
		pOrg = network->originVector[i];
		for (int j=0; j<pOrg->numOfDest; j++)
		{
			TNM_SDEST* dest=network->originVector[i]->destVector[j];
			out<<TNM_IntFormat(dest->origin->id, 4)<<TNM_IntFormat(dest->dest->id, 4)<<TNM_IntFormat(dest->buffer[0], 4);
			totalnodenum += dest->buffer[0];
			totalpathnum += dest->buffer[1];
			totalsppnum += dest->buffer[2];
			origin_total += dest->buffer[0];
			out<<"\n";
		}
		out<<"Origin"<<TNM_IntFormat(pOrg->origin->id, 4)<<" Total node count = "<<TNM_IntFormat(origin_total, 8)<<endl;
	}
	out<<"The Network Total check node count = "<<TNM_IntFormat(totalnodenum, 8);
	out.close();
	cout<<"The Network Total check node count ="<<totalnodenum<<endl;
	return totalnodenum,totalpathnum,totalsppnum;
}

void TAP_MMBA1_o:: cout_Q_node_count_origin( const string &path, const string &name, const string &algname)
{
	int totalnum = 0;
	TNM_SORIGIN* pOrg;
	ofstream out;
	TNM_OpenOutFile(out,path  + name + "_" + algname + ".nodecount");
	for (int i=0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		out<<"Origin"<<TNM_IntFormat(pOrg->origin->id, 4)<<" Total node count = "<<TNM_IntFormat(pOrg->origin->buffer[8], 8)<<endl;
		totalnum += pOrg->origin->buffer[8];
	}
	out<<"The Network Total node count = "<<TNM_IntFormat(totalnum, 8);
	out.close();
	cout<<"The Network Total check node count = "<<totalnum<<endl;
}



void TAP_MMBA1_o::adjust_one_bound(TNM_SDEST* dest, int k, TNM_SPATH* pathk, TNM_SPATH* pathkplus1)//k: index of pathk in pathset
{
	floatType gthetak = votdist->MMBA1_gtheta(dest->pathSetbound[k+1]);//path k upper bound->pathset bound k+1
	floatType gthetakprime = votdist->MMBA1_gprimetheta(dest->pathSetbound[k+1]);
	///////////////////////revise path cost//////////////////////
	floatType pathtimediff = pathk->PathCostS() - pathkplus1->PathCostS();
	//calculate numerical result
	floatType gprimethetak = votdist->MMBA1_gprimetheta(dest->pathSetbound[k+1]);
	floatType wbd1_fd = dest->assDemand * gthetak * pathtimediff;
	floatType sd2path = 0.0;
	for (int i=0; i<pathk->GetLinkNum(); i++)
	{
		sd2path += pathk->path[i]->GetDerCost();
		pathk->path[i]->markStatus = 1;
	}
	for (int i=0; i<pathkplus1->GetLinkNum(); i++)
	{
		if (pathkplus1->path[i]->markStatus != 1)
		{sd2path += pathkplus1->path[i]->GetDerCost();}
		else
		{sd2path -= pathkplus1->path[i]->GetDerCost();}
	}
	floatType wbd2_fd = dest->assDemand * dest->pathSetbound[k+1] * gthetak * (pathk->buffer[0] - pathkplus1->buffer[0]);
	floatType wbd1_sd = pow(dest->assDemand, 2)*pow(gthetak, 2)*(sd2path);
	floatType wbd2_sd = dest->assDemand * (pathk->buffer[0] - pathkplus1->buffer[0]) * (gthetak);
	
	floatType wbd_sd = wbd1_sd+wbd2_sd;
	if (wbd_sd < 0)
	{
		wbd_sd = (pow(dest->assDemand, 2)*pow(gthetak, 2)*(sd2path) + dest->assDemand * (pathk->buffer[0] - pathkplus1->buffer[0]) * (gthetak))*0.00001;
	}
	floatType theta_adjust;
	if (convIndicator<1e-8)
	{
		theta_adjust = (wbd1_fd+wbd2_fd)/(wbd_sd)*0.7;
	}
	else
	{
		theta_adjust = (wbd1_fd+wbd2_fd)/(wbd_sd);
	}
	
	floatType achieve_result = dest->pathSetbound[k+1] - theta_adjust;
	floatType oldtheta = dest->pathSetbound[k+1];
	floatType shiftflow = 0.0;
	if ((achieve_result >= dest->pathSetbound[k]) && (achieve_result <= dest->pathSetbound[k+2]))
	{
		dest->pathSetbound[k+1] = achieve_result;
		pathk->buffer[3] = achieve_result;
		pathkplus1->buffer[2] = achieve_result;
		shiftflow = dest->assDemand*(votdist->MMBA1_Gtheta(dest->pathSetbound[k+1]) - votdist->MMBA1_Gtheta(oldtheta));
	}
	else if (achieve_result < dest->pathSetbound[k])
	{
		dest->pathSetbound[k+1] = dest->pathSetbound[k];
		pathk->buffer[3] = dest->pathSetbound[k+1];
		pathkplus1->buffer[2] = dest->pathSetbound[k+1];
		shiftflow = (-1)*pathk->flow;
	}
	else if (achieve_result > dest->pathSetbound[k+2])
	{
		dest->pathSetbound[k+1] = dest->pathSetbound[k+2];
		pathk->buffer[3] = dest->pathSetbound[k+1];
		pathkplus1->buffer[2] = dest->pathSetbound[k+1];
		shiftflow = pathkplus1->flow;
	}
	else
	{
		cout<<"There is no such result in theory!"<<endl;
	}

	total_shift_flow += (fabs(shiftflow));
	pathk->flow += shiftflow;
	for(int i=0; i< pathk->path.size(); i++)
	{
		pathk->path[i]->volume += shiftflow;
		pathk->path[i]->cost = pathk->path[i]->GetCost();
		pathk->path[i]->fdCost = pathk->path[i]->GetDerCost();
		pathk->path[i]->markStatus = 0;
	}
	pathkplus1->flow -= shiftflow;
	for(int i=0; i< pathkplus1->path.size(); i++)
	{
		pathkplus1->path[i]->volume -= shiftflow;
		pathkplus1->path[i]->cost = pathkplus1->path[i]->GetCost();
		pathkplus1->path[i]->fdCost = pathkplus1->path[i]->GetDerCost();
		pathkplus1->path[i]->markStatus = 0;
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


void  TAP_MMBA1_o:: boundadjusting_OD(TNM_SDEST* dest)
{
	if (dest->pathSet.size() == 1)
	{
		return;
	}
	for (int i=0; i<dest->pathSet.size()-1; i++)//µ÷ÕûÒ»ÂÖ
	{
		adjust_one_bound(dest, i, dest->pathSet[i], dest->pathSet[i+1]);
	}
}

void  TAP_MMBA1_o::deletenousepath_OD(TNM_SDEST* dest)
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
				delete pa;
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
				cout<<dest->origin->id<<"->"<<dest->dest->id<<"k="<<i<<" pa->buffer[2]="<<pa->buffer[2]<<" pa->buffer[3]="<<pa->buffer[3]<<"dest->pathSetbound[i+1]="<<dest->pathSetbound[i+1]<<endl;
				cout<<dest->origin->id<<"->"<<dest->dest->id<<"k+1="<<i+1<<" dest->pathSet[i+1]->buffer[2]="<<dest->pathSet[i+1]->buffer[2]<<" dest->pathSet[i+1]->buffer[3]="<<dest->pathSet[i+1]->buffer[3]<<"dest->pathSetbound[i+2]="<<dest->pathSetbound[i+2]<<endl;
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


floatType TAP_MMBA1_o::MMBA1_innerGap_OD(TNM_SDEST* dest)
{
	if (dest->pathSet.size() == 1)
	{
		return 0.0;
	}
	TNM_SPATH* pathk;
	TNM_SPATH* pathkplus1;
	floatType gthetak, pathtimediff, gprimethetak, wbd_fd, thetak;
	floatType wbd_fd_max = 0.0;
	for (int k=0; k<dest->pathSet.size()-1; k++)
	{
		pathk = dest->pathSet[k];
		pathkplus1 = dest->pathSet[k+1];
		thetak = dest->pathSetbound[k+1];
		gthetak = votdist->MMBA1_gtheta(thetak);
		///////////////////////revise path cost//////////////////////
		pathtimediff = pathk->PathCostS() - pathkplus1->PathCostS();
		gprimethetak = votdist->MMBA1_gprimetheta(thetak);
		wbd_fd = dest->assDemand * gthetak * pathtimediff + dest->assDemand * thetak * gthetak * (pathk->buffer[0] - pathkplus1->buffer[0]);
		if (fabs(wbd_fd)>wbd_fd_max)
		{wbd_fd_max = fabs(wbd_fd);}
	}
	return wbd_fd_max;
}

void TAP_MMBA1_o::build_node_dest()
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
			///////////////origin and dest are same////////////////////////
			if (dest->dest ==  dest->origin && dest->assDemand != 0)
			{
				dest->pathSet.push_back(revert_SPath(dest->origin, dest->dest));
				dest->pathSet[0]->flow = dest->assDemand;
				dest->pathSet[0]->buffer[2] = theta_min;
				dest->pathSet[0]->buffer[3] = theta_max;
				dest->pathSetbound.push_back(theta_min);
				dest->pathSetbound.push_back(theta_max);
			}
			////////////////////////////////////////////////////////////
		}
		node_dest.insert(make_pair(ori->origin->id, origin_node_dest));
	}
}

void TAP_MMBA1_o::MMBA1_o_obj()
{
	floatType	sumobjf= 0.0;
	floatType	sumobjs= 0.0;
	floatType	odobjs= 0.0;
	TNM_SORIGIN *pOrg;
	floatType tempresult = 0.0;
	//////////
	for (int i=0; i<network->numOfLink; i++)
	{
		sumobjf += network->linkVector[i]->GetIntCost();
	}
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			odobjs = 0.0;
			for (int k=0; k < dest->pathSet.size(); k++)
			{
				tempresult = dest->assDemand * dest->pathSet[k]->buffer[0] * (votdist->MMBA1_Ptheta(dest->pathSetbound[k+1])- votdist->MMBA1_Ptheta(dest->pathSetbound[k]));
				odobjs += tempresult;
			}
			sumobjs += odobjs;
		}
	}
	OFV =  sumobjf + sumobjs;
}


void TAP_MMBA1_o::MMBA1_o_GAP()
{
	floatType   sumobjf= 0.0;
	floatType	sumobjs= 0.0;
	floatType	sumpath= 0.0;
	floatType	sumlink= 0.0;
	floatType	odobjs= 0.0;
	TNM_SORIGIN *pOrg;
	TNM_SPATH* pa;
	TNM_SPATH* temppath;
	bool find = true;
	likelysppcount = 0;
	clock_t start, end;
	double gapshortestpathtime=0.0;
	shortestpathtime = 0.0;
	truespptime = 0.0;
	ltatime = 0.0;
	ltachecktime = 0.0;
	spptreenum = 0;
	floatType tempresult = 0.0;

	for (int i=0; i<network->numOfLink; i++)
	{
		sumobjf += network->linkVector[i]->GetIntCost();
	}
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			TNM_SDEST* dest = pOrg->destVector[j];
			odobjs = 0.0;
			for (int k=0; k < dest->pathSet.size(); k++)
			{
				tempresult = dest->assDemand * dest->pathSet[k]->buffer[0] * (votdist->MMBA1_Ptheta(dest->pathSetbound[k+1])- votdist->MMBA1_Ptheta(dest->pathSetbound[k]));
				odobjs += tempresult;
				sumlink += tempresult;
				if (tempresult<0)
				{
					cout<<"sumlink reduce"<<tempresult<<endl;
					system("PAUSE");
				}
				///////////////////////revise path cost//////////////////////
				if (dest->pathSet[k]->flow >= 0)
				{
					floatType tempflowvalue;
					tempflowvalue = dest->assDemand *(votdist->MMBA1_Gtheta(dest->pathSetbound[k+1]) - votdist->MMBA1_Gtheta(dest->pathSetbound[k]));
					if (tempflowvalue>=0)
					{
						sumlink += dest->pathSet[k]->PathCostS() * tempflowvalue;
					}
					else
					{
						cout<<"in gap calc: boundary has something wrong!"<<dest->origin->id<<"->"<<dest->dest->id<<"boundary diff="<<(dest->pathSetbound[k+1]-dest->pathSetbound[k])<<endl;
					}
				}
				else
				{cout<<"in gap calc: negative flow"<<dest->origin->id<<"->"<<dest->dest->id<<"flow="<<dest->pathSet[k]->flow<<endl;
				sumlink += 0;}
				///////////////////////revise path cost//////////////////////
				if (fabs((dest->pathSet[k]->flow) - (dest->assDemand *(votdist->MMBA1_Gtheta(dest->pathSetbound[k+1]) - votdist->MMBA1_Gtheta(dest->pathSetbound[k]))) )>1e-5)
				{
					cout<<"pathsetsize="<<dest->pathSet.size()<<endl;
					cout<<"pathsetbound=";
					for(int u=0; u<dest->pathSetbound.size(); u++)
					{
						cout<<dest->pathSetbound[u];

					}
					cout<<endl;
					cout<<pOrg->origin->id<<"->"<<dest->dest->id<<endl;
					cout<<"k="<<k<<"method1="<<dest->pathSet[k]->flow<<"method2="<<dest->assDemand *(votdist->MMBA1_Gtheta(dest->pathSetbound[k+1]) - votdist->MMBA1_Gtheta(dest->pathSetbound[k]))<<endl;
					cout<<"dest->pathSetbound[k]="<<dest->pathSetbound[k]<<"dest->pathSetbound[k+1]="<<dest->pathSetbound[k+1]<<"buffer[2]="<<dest->pathSet[k]->buffer[2]<<"buffer[3]="<<dest->pathSet[k]->buffer[3]<<endl;
					cout<<"lower bound diff="<<(dest->pathSetbound[k] - dest->pathSet[k]->buffer[2])<<"upper bound diff"<<(dest->pathSetbound[k+1] - dest->pathSet[k]->buffer[3])<<endl;
					system("pause");
				}
			}
			sumobjs += odobjs;
		}
		start = clock();
		
		floatType oldtreenum = spptreenum;
		modified_T2_MPA_path(pOrg, 0, 0);
		tree_num.push_back(spptreenum - oldtreenum);
		end = clock();
		gapshortestpathtime += (end-start);
	}
	sumpath = sumo_path;
	OFV =  sumobjf + sumobjs;
	convIndicator = 1-sumpath/sumlink;

	cout<<" Conv="<<convIndicator<<" OFV="<<OFV<<endl;
}




void TAP_MMBA1_o::Initialize()
{
	cout<<"Enter Initialize()"<<endl;
	periter_addpathcount = 0;
	periter_deletepathcount = 0;
	total_pathset_size = 0;
	max_pathset_size = 0;
	total_shift_flow = 0.0;
	withoutgaptime = 0.0;
	sumo_path = 0.0;
	spptreenum = 0;
	shortestpathtime = 0.0;
	truespptime = 0.0;
	ltatime = 0.0;
	ltachecktime = 0.0;
	theta_min = 1.0/votdist->vot_max;
	theta_max = 1.0/votdist->vot_min;
	network->AllocatePathBuffer(4); //buffer[0] path toll; buffer[1] add path status(is new add or not); buffer[2] path lower bound; buffer[3] path upper bound
	build_node_dest();//find dest of a node
	network->UpdateLinkCost();//update cost
	network->AllocateLinkBuffer(9); // allocate float memory for link variables, 0 is a, 1 is u, 2 is new x, 3 is new u
	//4 is origin-based new flow, 5 is origin-based new u, 6 is link label: 0 is not in the tree, 1 is in the tree; 7 is x_link, 8 is u_link(LTA);
	network->AllocateNodeBuffer(9); //allocate float memory for node variables, 0 is node time(s-node), 1 is node toll(d-node); 2 is x_node(LTA),3 is u_node(LTA);
	//4 is a_node(LTA); 5 is d_x(LTA), 6 is d_u(LTA);7 is topological order
	//network->AllocateDestBuffer(2); 
	TNM_SORIGIN* pOrg;
	floatType sumlink = 0.0;
	TNM_SDEST* dest;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		for(int j =0 ;j<network->numOfLink;j++)
		{
			network->linkVector[j]->buffer[4] = 0;
			network->linkVector[j]->buffer[5] = 0;
		}
		floatType oldtreenum = spptreenum;
		modified_T2_MPA_path(pOrg, 1, 1);
		tree_num.push_back(spptreenum - oldtreenum);//save tree num
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
				sumlink += dest->assDemand * dest->pathSet[k]->buffer[0] * (votdist->MMBA1_Ptheta(dest->pathSetbound[k+1]) - votdist->MMBA1_Ptheta(dest->pathSetbound[k]));
				///////////////////////revise path cost//////////////////////
				sumlink += dest->pathSet[k]->PathCostS() * dest->pathSet[k]->flow;
				///////////////////////revise path cost//////////////////////
				sumdemand += network->originVector[i]->destVector[j]->pathSet[k]->flow;
				if (k>=1 && network->originVector[i]->destVector[j]->pathSet[k]->buffer[0]==network->originVector[i]->destVector[j]->pathSet[k-1]->buffer[0])
				{
					cout<<"Attention! There is path have same money cost!"<<network->originVector[i]->origin->id<<"->"<<network->originVector[i]->destVector[j]->dest->id<<endl;
					TNM_SPATH* pathk = network->originVector[i]->destVector[j]->pathSet[k];
					TNM_SPATH* pathk_1 = network->originVector[i]->destVector[j]->pathSet[k-1];
					printpath(pathk);
					cout<<k<<" money="<<pathk->buffer[0]<<"time = "<<pathk->PathCostS()<<"lower bound="<<pathk->buffer[2]<<"upper bound="<<pathk->buffer[3]<<"diff="<<pathk->buffer[3]-pathk->buffer[2]<<endl;
					printpath(pathk_1);
					cout<<k-1<<" money="<<pathk_1->buffer[0]<<"time = "<<pathk_1->PathCostS()<<"lower bound="<<pathk_1->buffer[2]<<"upper bound="<<pathk_1->buffer[3]<<"diff="<<pathk_1->buffer[3]-pathk_1->buffer[2]<<endl;
					cout<<"pathtime diff = "<<pathk_1->buffer[0] - pathk->buffer[0]<<endl;
					system("PAUSE");
				}
				if (k>=1)
				{
					if (network->originVector[i]->destVector[j]->pathSetbound[k] - network->originVector[i]->destVector[j]->pathSetbound[k-1] < 0)
					{
						cout<<"ERROR: pathset bound order is wrong!"<<network->originVector[i]->origin->id<<"->"<<network->originVector[i]->destVector[j]->dest->id<<endl;
					}
					if (network->originVector[i]->destVector[j]->pathSet[k]->buffer[0] - network->originVector[i]->destVector[j]->pathSet[k-1]->buffer[0] > 0)
					{
						cout<<"ERROR: money order is wrong!"<<network->originVector[i]->origin->id<<"->"<<network->originVector[i]->destVector[j]->dest->id<<endl;
						TNM_SPATH* pathk = network->originVector[i]->destVector[j]->pathSet[k];
						TNM_SPATH* pathk_1 = network->originVector[i]->destVector[j]->pathSet[k-1];
						TNM_SPATH* path0 = network->originVector[i]->destVector[j]->pathSet[0];
						printpath(pathk);
						cout<<k<<" money="<<pathk->buffer[0]<<"time = "<<pathk->PathCostS()<<"lower bound="<<pathk->buffer[2]<<"upper bound="<<pathk->buffer[3]<<"diff="<<pathk->buffer[3]-pathk->buffer[2]<<endl;
						printpath(pathk_1);
						cout<<k-1<<" money="<<pathk_1->buffer[0]<<"time = "<<pathk_1->PathCostS()<<"lower bound="<<pathk_1->buffer[2]<<"upper bound="<<pathk_1->buffer[3]<<"diff="<<pathk_1->buffer[3]-pathk_1->buffer[2]<<endl;
						printpath(path0);
						cout<<0<<" money="<<path0->buffer[0]<<"time = "<<path0->PathCostS()<<"lower bound="<<path0->buffer[2]<<"upper bound="<<path0->buffer[3]<<"diff="<<path0->buffer[3]-path0->buffer[2]<<endl;
						cout<<"pathtime diff = "<<pathk_1->buffer[0] - pathk->buffer[0]<<endl;
						cout<<"money diff ="<<network->originVector[i]->destVector[j]->pathSet[k]->buffer[0]-network->originVector[i]->destVector[j]->pathSet[k-1]->buffer[0]<<endl;
						system("PAUSE");
					
					}
				}
			}
			if (fabs(sumdemand - network->originVector[i]->destVector[j]->assDemand)>1e-6)
			{
				cout<<"violate demand"<<"o"<<network->originVector[i]->origin->id<<"d"<<network->originVector[i]->destVector[j]->dest->id<<sumdemand - network->originVector[i]->destVector[j]->assDemand<<endl;
			}
		}
	}
	network->UpdateLinkCost();
	//******************************************************
	//cout<<"Initialize Time"<<1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC<<endl;
	//******************************************************
	withoutgaptime += (clock() - m_startRunTime);
	//******************************************************

	sumlink = 0.0;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		for (int j=0; j<pOrg->numOfDest; j++)
		{
			dest = pOrg->destVector[j];
			for(int k=0; k<dest->pathSet.size(); k++)
			{
				sumlink += dest->assDemand * dest->pathSet[k]->buffer[0] * (votdist->MMBA1_Ptheta(dest->pathSetbound[k+1]) - votdist->MMBA1_Ptheta(dest->pathSetbound[k]));
				sumlink += dest->pathSet[k]->PathCostS() * dest->pathSet[k]->flow;
			}
		}
	}

	sumo_path = 0.0;
	MMBA1_o_GAP();	
}

void TAP_MMBA1_o::MainLoop()
{
	//////////////////////change to the time without gap/////////////////////////////////
	iterRecord[curIter-1]->time = 1.0*(withoutgaptime)/CLOCKS_PER_SEC;
	//////////////////////////////////////////////////////////////
	clock_t withoutgaptime_s, withoutgaptime_e;
	withoutgaptime_s = clock();
	periter_addpathcount = 0;
	periter_deletepathcount = 0;
	total_pathset_size = 0;
	max_pathset_size = 0;
	total_shift_flow = 0.0;
	badIter = 1;
	ziggIter = 1;
	sumo_path=0.0;
	///////////////////////////////////////////////////////
	//parameter can change
	if(curIter == 1)
	{
		convIndicator = 1;
	}
	//////////////////////////////////////////////////////
	clock_t start_innerloop, end_innerloop, start_adjust, end_adjust, start_innerod, end_innerod;
	double innerlooptime = 0.0;
	double adjusttime = 0.0;
	double innerodtime = 0.0;
	TNM_SORIGIN *pOrg;
	TNM_SDEST* dest;
	bool wrong;
	//cout<<"Main loop: "<<endl;
	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		//
		floatType oldtreenum = spptreenum;
		modified_T2_MPA_path(pOrg, 0, 1);
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
			dest->rdmSet.clear();//columngeneration o set
		}
	}
	//cout<<"***********************start inner loop*******************"<<endl;
	floatType deltaod;
	start_innerloop = clock();
	int innerI=0;
	int innerMaxI=300;// change from 30 to 300
	int innerFC=0;
	double migap=50;
	floatType totalinnergap=0;
	int       totalgapcount=0;
	while(innerI<innerMaxI)
	{
		migap=0;
		if (convIndicator<1e-8)
		{
			innerMaxI=600;
		}
		innerI += 1;
		innerFC = 0;
		for (int i=0;i<network->numOfOrigin;i++)
		{
			pOrg = network->originVector[i];
			for (int j=0;j<pOrg->numOfDest;j++)
			{
				dest = pOrg->destVector[j];
				//start_innerod = clock();
				deltaod = MMBA1_innerGap_OD(dest);
				if (convIndicator>1e-1)
				{
					if (deltaod > convIndicator/100.0)//change from /10 to /100
					{
						innerFC += 1;
						boundadjusting_OD(dest);
					}
				}
				else if (convIndicator>1e-2)
				{
					if (deltaod > convIndicator/10.0)//change from /10 to /100
					{
						innerFC += 1;
						boundadjusting_OD(dest);
					}
				}
				else if (convIndicator>1e-4)
				{
					if (deltaod > convIndicator*100.0)
					{
						innerFC += 1;
						boundadjusting_OD(dest);
					}
				}
				else if (convIndicator>1e-6)
				{
					if (deltaod > convIndicator*1000.0)
					{
						innerFC += 1;
						boundadjusting_OD(dest);
					}
				}
				else if (convIndicator>1e-8)
				{
					if (deltaod > convIndicator*10000.0)
					{
						innerFC += 1;
						boundadjusting_OD(dest);
					}
				}
				else
				{
					if (deltaod > convIndicator*10000.0)
					{
						innerFC += 1;
						boundadjusting_OD(dest);
					}
				}
				////////////////////////////////////////////////////
				if (innerI%5==0)
				{
					deletenousepath_OD(dest);//simplified
				}
			}
		}

		if (innerFC <= 3)
		{
			break;
		}
	}

	for (int i = 0;i<network->numOfOrigin;i++)
	{
		pOrg = network->originVector[i];
		for (int j=0;j<pOrg->numOfDest;j++)
		{
			dest = pOrg->destVector[j];
			deletenousepath_OD(dest);//simplified
			total_pathset_size += dest->pathSet.size();
			if (dest->pathSet.size() > max_pathset_size)
			{
				max_pathset_size = dest->pathSet.size();
			}
		}
	}
	end_innerloop = clock();
	innerlooptime = end_innerloop - start_innerloop;
	withoutgaptime_e = clock();
	withoutgaptime += (withoutgaptime_e - withoutgaptime_s);
	sumo_path=0.0;
	MMBA1_o_GAP();
	aveInnerGap=totalinnergap/totalgapcount;
}


int TAP_MMBA1_o::MPA_path(TNM_SORIGIN* origin, int allornothing, int addset)
{
	clock_t start1, end1, start2, end2, start3, end3;
	for(int ni=0;ni<network->numOfNode;ni++)
	{
		network->nodeVector[ni]->buffer[0] = 0;// node time
		network->nodeVector[ni]->buffer[1] = 0;// node toll
		network->nodeVector[ni]->buffer[2] = 0;// nodeHsetinit 
		network->nodeVector[ni]->buffer[4] = theta_min; //a_node
	}
	for(int li=0;li<network->numOfLink;li++)
	{
		network->linkVector[li]->buffer[0] = -1;//set the link a as -1//what -1 use?
		network->linkVector[li]->buffer[6] = 0;// link label shortest path
	}
	//reset the lowbounds and upbounds of theta
	theta_low = theta_min;
	theta_up = theta_max;
	//initialize the shortest path tree
	modified_T2_MIN_path(origin);
	spptreenum += 1;
	///////////////////////////////////////////
	vector<TNM_SLINK*>   pllist(network->numOfNode,NULL);
	vector<floatType>   plmoney(network->numOfNode,NULL);
	vector<floatType>   pltime(network->numOfNode,NULL);
	floatType ptheta_up = theta_min;
	vector<floatType> gCostUp(network->numOfLink,0);
	TNM_SLINK* clink;
	TNM_SNODE* inode;
	TNM_SNODE* jnode;
	TNM_SNODE* beforenode;
	TNM_SNODE* nownode;
	TNM_SNODE* thisnode;
	TNM_SLINK* beforelink;
	do
	{
		spptreenum += 1;
		//record the last tree
		for(unsigned i=0; i<network->numOfNode; i++)
		{
			network->nodeVector[i]->buffer[2] = 0;// node is or isnot add in to Dset by PTU;
			pllist[i] = network->nodeVector[i]->pathElem->via;
			plmoney[i] = network->nodeVector[i]->buffer[1];
			pltime[i] = network->nodeVector[i]->buffer[0];
		}
		ptheta_up = theta_up;
		theta_low = theta_up;
		theta_low += 1e-12;
		/////////////////////////////////////////////////////////////////
		theta_up = theta_max;
		start3 = clock();
		for(int li =0 ; li< network->numOfLink;li++)
		{
			gCostUp[li] = network->linkVector[li]->cost + theta_low * network->linkVector[li]->toll;
			network->linkVector[li]->buffer[0] = -1; // ini the a value for each link
			network->linkVector[li]->buffer[6] = 0;  //reset the link label
		}
		modified_T2_shortestpath(origin->origin, gCostUp);
		end1 = clock();
		truespptime += (end1 - start3);
		start3 = clock();
		clink = NULL;
		inode = NULL;
		jnode = NULL;
		for(int li =0; li< network->numOfLink; li++)
		{
			clink  = network->linkVector[li];
			clink->buffer[0] = theta_max;
			if(clink->buffer[6] == 0) // this link is not in the current tree
			{
				inode = clink->tail;
				jnode = clink->head;
				if(fabs(inode->buffer[1] + clink->toll - jnode->buffer[1])<1e-10)//can value compare like this?
				{
					clink->buffer[0] = theta_max;
					//cout<<"has situation that mi+ma=mj"<<endl;
				}
				else
				{
					floatType theta_a = (jnode->buffer[0] - clink->cost - inode->buffer[0])/( inode->buffer[1]+ clink->toll - jnode->buffer[1]);
					clink->buffer[0] = setdecimal(theta_a, 12);//set the decimal point
					if (clink->buffer[0]< theta_up && clink->buffer[0]>theta_low)
					{
						theta_up = clink->buffer[0];
						///////////////////////////////////////////////
						origin->origin->buffer[8] += 1;
						//////////////////////////////////////////////
					}
				}
			}

		}
		/////////////////find changed node//////////////////////////////////////////////////
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
				beforenode = nownode->pathElem->via->tail;
				beforelink = nownode->pathElem->via;
				if (beforelink != pllist[nownode->id-1 ]) 
				{//node->buffer[2] is in D set or not, avoid repeated additions
					if ((thisnode->buffer[2]==0)&&((fabs(thisnode->buffer[0] - pltime[thisnode->id - 1])>1e-14)||(fabs(thisnode->buffer[1]- plmoney[thisnode->id-1])>1e-14)))
					{
						thisnode->buffer[2] = 1;
						Dset.push_back(thisnode);
					}					
					break;
				}
				else
				{
					nownode = beforenode;
				}
			}while(beforenode != origin->origin);
		}
		end3 = clock();
		shortestpathtime += (end3-start3);
		start2 = clock();
		modified_T2_LTA_path(origin, allornothing, addset, pllist, ptheta_up);
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
	modified_T2_LTA_path(origin, allornothing, addset, pllist, ptheta_up);
	end2 = clock();
	ltatime += (end2 - start2);
	return 0;
}

int TAP_MMBA1_o::modified_T2_MPA_path(TNM_SORIGIN* origin, int allornothing, int addset)
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
	}
	for(int li=0;li<network->numOfLink;li++)
	{
		// allocate float memory for link variables, 0 is a, 1 is u, 2 is new x, 3 is new u
	//4 is origin-based new flow, 5 is origin-based new u, 6 is link label: 0 is not in the tree, 1 is in the tree; 7 is x_link, 8 is u_link(LTA);
		network->linkVector[li]->buffer[0] = -1;//set the link a as -1//what -1 use?
		network->linkVector[li]->buffer[6] = 0;// link label
	}
	//reset the lowbounds and upbounds of theta
	theta_low = theta_min;
	theta_up = theta_max;
	//initialize the shortest path tree
	///////////////////////////////////////////////////////////////////////////
	modified_T2_MIN_path(origin);
	///////////////////////////////////////////////////////////////////////////
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
		modified_T2_PTU_path(origin->origin);

		modified_T2_LTA_path(origin, allornothing, addset, pllist, ptheta_up);
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
	modified_T2_LTA_path(origin, allornothing, addset, pllist, ptheta_up);
	return 0;
}

int TAP_MMBA1_o::modified_T2_MIN_path(TNM_SORIGIN* origin)
{
	vector<floatType> gCostUp(network->numOfLink,0);
	for(int li =0 ; li< network->numOfLink;li++)
	{
		gCostUp[li] = network->linkVector[li]->cost + theta_low * network->linkVector[li]->toll;
		network->linkVector[li]->buffer[0] = -1; // ini the a value for each link
		network->linkVector[li]->buffer[6] = 0;  //reset the link label
	}
	//shortest path algorithm
	clock_t start2, end2;
	start2 = clock();
	modified_T2_shortestpath(origin->origin, gCostUp);
	end2 = clock();
	truespptime += (end2-start2);
	theta_up = theta_max;
	Hset.clear();
	start2 = clock();
	for(int li =0; li< network->numOfLink; li++)
	{
		TNM_SLINK* clink  = network->linkVector[li];
		clink->buffer[0] = theta_max;//why?
		TNM_SNODE* inode = network->linkVector[li]->tail;
		TNM_SNODE* jnode = network->linkVector[li]->head;
		if(clink->buffer[6] == 0) // this link is not in the current tree
		{
			if(setdecimal_round(inode->buffer[1] + clink->toll, 9) == setdecimal_round(jnode->buffer[1], 9))//can value compare like this?
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
					Hset.clear();
					Hset.push_back(inode);
					///////////////////////////////////////////////
					origin->origin->buffer[8] += 1;
					//////////////////////////////////////////////
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
					if(!Hexist && inode != origin->origin) 
					{
						Hset.push_back(inode);
						///////////////////////////////////////////////
						origin->origin->buffer[8] += 1;
						//////////////////////////////////////////////
					}
				}
			}
		}

	}
	end2 = clock();
	shortestpathtime += (end2 - start2);
	return 0;
}

int TAP_MMBA1_o::modified_T2_PTU_path(TNM_SNODE * origin)
{
	clock_t start1, end1;
	start1 = clock();
	Dset.clear();
	for (deque<TNM_SNODE*>::iterator no = Hset.begin(); no != Hset.end(); no++)
	{
		(*no)->buffer[2] = 1;
	}
	//begin the loop
	while(!Hset.empty())
	{		
		//step 2: next node to scan
		TNM_SNODE * scanNode = Hset.front();
		Hset.pop_front();
		if (scanNode->buffer[2] == 1)
		{
			scanNode->buffer[2] = 0;
		}
		else{Dset.push_back(scanNode);}
		//step 3: scan links stem from scanNode
		for(vector<TNM_SLINK*>::iterator pv = scanNode->forwStar.begin(); pv !=scanNode->forwStar.end(); pv++)
		{
			if( *pv != NULL)
			{
				TNM_SLINK * scanLink = *pv;
				TNM_SNODE * jnode = scanLink->head;
				//define the following variables
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
				///////////////////////////////////////////
				if(condition2==1 && condition1==0)
				{
					cout<<"exist condition2==1 && condition1==0"<<endl;
				}
				////////////////////////////////////////
				if (condition1||condition2)
				{
					jnode->buffer[0] = s_star;
					jnode->buffer[1] = d_star;
					//update link label
					TNM_SLINK* oldlink = jnode->pathElem->via;
					jnode->pathElem->via = scanLink;
					oldlink->buffer[6] = 0;
					scanLink->buffer[6] = 1;

					//put jnode into the H set
					int Hexist = 0;
					if(!Hset.empty())
					{
						for(unsigned i=0;i<Hset.size();i++)
						{
							if(Hset[i] == jnode) Hexist = 1;
						}
					}
					if(!Hexist && jnode != origin) 
					{
						Hset.push_back(jnode);
						//////////////////////////////////////////////
						origin->buffer[8] += 1;
						///////////////////////////////////////////////
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
		if(clink->buffer[6] == 0) // this link is not in the current tree
		{
			if(setdecimal_round(inode->buffer[1] + clink->toll, 9) == setdecimal_round(jnode->buffer[1],9))//can value compare like this?
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
					Hset.clear();
					Hset.push_back(inode);
					//////////////////////////////////////////////
					origin->buffer[8] += 1;
					///////////////////////////////////////////////
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
						//////////////////////////////////////////////
						origin->buffer[8] += 1;
						///////////////////////////////////////////////
					}
				}
			}
		}

	}
	return 0;
}

int TAP_MMBA1_o::modified_T2_LTA_path(TNM_SORIGIN* ori, int allornothing, int addset,vector<TNM_SLINK*> & plist, floatType ptheta_up)
{
	floatType f1=0.0;
	floatType g1=0.0;
	if (allornothing==1)
	{
		clock_t start_assign, end_assign;
		TNM_SPATH* path1;
		TNM_SNODE* jnode;
		TNM_SDEST* dest;
		//Dset process hai mei xie
		
		while(!Dset.empty())
		{
			jnode = Dset.back();
			Dset.pop_back();
			if (fabs(jnode->buffer[4] - ptheta_up)>1e-9)
			{
				//init dest
				start_assign = clock();
				dest = node_dest[ori->origin->id][jnode->id];
				end_assign = clock();
				ltachecktime += (end_assign - start_assign);
				
				if (dest == NULL)
				{
					
					jnode->buffer[4] = ptheta_up;
					continue;
				}
				path1 = revert_SPath_lasttree(ori->origin, jnode, plist);
				if (dest->pathSet.size()==0)
				{
					dest->pathSetbound.push_back(theta_min);
				}
				dest->pathSet.push_back(path1);
				f1 = dest->assDemand*(votdist->MMBA1_Gtheta(ptheta_up)-votdist->MMBA1_Gtheta(jnode->buffer[4]));
				g1 = dest->assDemand*(votdist->MMBA1_Ptheta(ptheta_up)-votdist->MMBA1_Ptheta(jnode->buffer[4]));
				path1->buffer[2]=jnode->buffer[4];
				path1->buffer[3]=ptheta_up;
				jnode->buffer[4] = ptheta_up;
				path1->flow = f1;
				update_pathlinkflow(path1, f1);
				dest->pathSetbound.push_back(ptheta_up);
				///////////////////////revise path cost//////////////////////
				//sumo_path += (f1*path1->PathCost() + g1*path1->buffer[0]);
				sumo_path += (f1*path1->PathCostS() + g1*path1->buffer[0]);
				///////////////////////revise path cost//////////////////////
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
		//Dset process hai mei xie
		while(!Dset.empty())
		{
			jnode = Dset.back();
			Dset.pop_back();
			if (fabs(jnode->buffer[4] - ptheta_up)>1e-9)
			{
				//init dest
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
					///////////////////////////////////////////////////////////////////////////
					link = plist[node->id - 1];
					///////////////////////////////////////////////////////////////////////////
					if (link == NULL) //there may exist no shortest path at all, or there must be sth wrong.
					{
						cout<<"\tno shortest path found at node "<<node->id<<endl;
					}
					if (addset == 1)//direction finding
					{
						link->buffer[7] += f1;// assign flow to link
						link->buffer[8] += g1;// assign flow moment to link
					}
					else if (addset == 0)//initilize adding
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
	else//not all or nothing
	{
		bool find;
		TNM_SPATH* path1;
		TNM_SNODE* jnode;
		TNM_SDEST* dest;
		//Dset process hai mei xie
		while(!Dset.empty())
		{
			
			jnode = Dset.back();
			Dset.pop_back();
			if (fabs(jnode->buffer[4] - ptheta_up)>1e-9)
			{
				//init dest
				dest = node_dest[ori->origin->id][jnode->id];
				if (dest == NULL)
				{
					jnode->buffer[4] = ptheta_up;
					continue;
				}
				path1 = revert_SPath_lasttree(ori->origin, jnode, plist);
				
				f1 = dest->assDemand*(votdist->MMBA1_Gtheta(ptheta_up)-votdist->MMBA1_Gtheta(jnode->buffer[4]));
				g1 = dest->assDemand*(votdist->MMBA1_Ptheta(ptheta_up)-votdist->MMBA1_Ptheta(jnode->buffer[4]));

				jnode->buffer[4] = ptheta_up;

				///////////////////////revise path cost//////////////////////
				//sumo_path += (f1*path1->PathCost() + g1*path1->buffer[0]);
				sumo_path += (f1*path1->PathCostS() + g1*path1->buffer[0]);
				///////////////////////revise path cost//////////////////////
				
				if (addset)
				{
					find = add_to_pathset_path(dest, path1);
					if (find)
					{
						delete path1;
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


TNM_SPATH* TAP_MMBA1_o::revert_SPath_lasttree(TNM_SNODE *origin, TNM_SNODE *dest, vector<TNM_SLINK*> & plist)
{
	TNM_SNODE *node;
	TNM_SLINK *link;
	int count = 0;
	TNM_SPATH *path = new TNM_SPATH; //allocate memory for new path
	assert(path!=0);
	node = dest;
	while (node != origin)
	{
		///////////////////////////////////////////////////////////////////////////
		link = plist[node->id - 1];
		///////////////////////////////////////////////////////////////////////////
		if (link == NULL) 
		{
			cout<<"\t revert_SPath_lasttree no shortest path found at node "<<node->id<<endl;
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
			path->Print(true);
			delete path;
			return NULL;
		}
	}
	vector<TNM_SLINK*>(path->path).swap(path->path); //this is to recycling unused memory allocated for path.
	//memory allocation strategy is one-by-one, so it might be slower.
	reverse(path->path.begin(), path->path.end()); //make the path is from origin to dest.
	path->cost = 0.0;
	path->buffer[0] = 0.0;
	for(int i=0; i<path->path.size();i++)
	{
		path->cost += path->path[i]->cost;
		path->buffer[0] += path->path[i]->toll;
	}
	return path;
}

bool TAP_MMBA1_o::add_to_pathset_path(TNM_SDEST* dest, TNM_SPATH* pa)
{
	bool find;
	TNM_SPATH* temppath;
	for (vector<TNM_SPATH*>::iterator pv = dest->pathSet.begin(); pv != dest->pathSet.end();pv++)
	{
		find = true;
		temppath = *pv;
		if (temppath->path.size() == pa->path.size())
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
		pa->buffer[1] = 1.0;
		for(int k=0; k<dest->pathSet.size(); k++)
		{
			if (pa->buffer[0] >= dest->pathSet[k]->buffer[0])
			{
				addin = true;
				pa->flow=0.0;
				dest->pathSet.insert(dest->pathSet.begin()+k, pa);
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

void TAP_MMBA1_o::cout_tree_num(string outname)
{
	ofstream out;
	int all_tree = 0;
	TNM_OpenOutFile(out,outname);
	for (int i=0;i<tree_num.size();i++)
	{
		if (i%network->numOfOrigin==network->numOfOrigin-1)
		{
			out<<to_string(tree_num[i])<<"\n";
			all_tree += tree_num[i];
		}
		else
		{
			out<<to_string(tree_num[i])<<" ";
			all_tree += tree_num[i];
		}

	}
	out<<" all tree num = "<<to_string(all_tree)<<"\n";
	cout<<"All tree num = "<<to_string(all_tree)<<endl;
}






