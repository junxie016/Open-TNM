#include "PTNet.h"
//#include "TNM_utility.h"
//#include "..\..\..\include\tnm\TNM_utility.h"

bool comp_link_cost(const PTLink *a,const PTLink *b)
{
	return a->tmpuse < b->tmpuse;
};

bool comp_link_id(const PTLink *a,const PTLink *b)
{
	return  a->id <b->id;
};

bool comp_gnode_level(const GNODE *a,const GNODE *b)
{
	return  a->m_tpLevel > b->m_tpLevel;
};

int	PTNET::PCTAE_Solver(PCTAE_algorithm alg)
{
	PCTAE_SetAlgorithm(alg);

	if (PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_GP_TL || 
		PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL ||
		PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_GFW_TL ||
		PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL
		)
	{
		CreateWalkLink(); //在站点之间构建步行弧 （Gentile网络）
		//CreateODLink(); //在起终点之间构建步行弧（sf/winnibeg/shenzhen网络）
	}



	if (m_symLinks)
	{	
		SetLinksAttribute(true);//symmtric functional form
	}
	else 
	{	
		SetLinksAttribute(false);//asymmtric functional form
	}




	switch(alg)
	{
		case PCTAE_algorithm::PCTAE_A_GFW:
			SolveFWTEAP();
			break;
		case PCTAE_algorithm::PCTAE_A_SD:
			SolveSDTEAP();
			break;
		case PCTAE_algorithm::PCTAE_P_Greedy:
			SolvePathTEAP();
			break;
		case PCTAE_algorithm::PCTAE_P_NGP:
			SolvePathTEAP();
			break;
		case PCTAE_algorithm::PCTAE_P_iGreedy:
			SolvePathTEAP();
			break;
		case PCTAE_algorithm::PCTAE_P_iNGP:
			SolvePathTEAP();
			break;
		case PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL:  //Greedy algorithm for suproblem in MOM framework
			SolveCapacityHyperpath();
			break;
		case PCTAE_algorithm::CAP_PCTAE_P_GP_TL:	 //GP algorithm for suproblem in MOM framework
			SolveCapacityHyperpath();
			break;
		case PCTAE_algorithm::CAP_PCTAE_P_GFW_TL:	 //link-based solver for suproblem in MOM framework
			SolveCapacityHyperpath();
			break;
		case PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL:	 //GP algorithm for suproblem in IPF framework
			SolveCapacityHyperpathIPF();
			break;
		default:
			cout << "No alg defined in the solver, please check solve!" << endl;
			return 1;
			break;

	}

	return 0;
}

int	PTNET::SolveFWTEAP()
{
	AllocateLinkBuffer(1);
	AllocateNetBuffer(1);
	UpatePTNetworkLinkCost();
	PTAllOrNothing();
	UpatePTNetworkLinkCost();
	m_startRunTime = clock();
	PrintNetLinks();
	//ComputeConvGap();
	//cout<<"iter:"<<curIter<<",\tcurrent gap:"<<TNM_FloatFormat(GapIndicator,5,15)<<", relative gap:"<<TNM_FloatFormat(RGapIndicator,5,15)<<endl;

	if(!(ReachAccuracy(RGapIndicator) || ReachMaxIter() || ReachMaxTime((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0))))
	{
		do
		{
			clock_t sclock = clock();
			SaveLinkWaitVariables();
			PTAllOrNothing();    //寻找下降方向：全由全无分配
			if (PCTAE_ALG == PCTAE_algorithm::PCTAE_A_GFW)	BisecSearch();  //线搜索 最优步长
			//cout<<stepSize<<","<<buffer[0]<<","<<netTTwaitcost<<endl;
			NewSolution(1);
		
			UpatePTNetworkLinkCost();
			IterMainlooptime = 1.0 * (clock() - sclock)/CLOCKS_PER_SEC;
			ComputeConvGap();
			curIter ++;		
			RecordTEAPCurrentIter();
			cout<<"iter:"<<curIter<<",\tcurrent gap:"<<TNM_FloatFormat(GapIndicator,5,15)<<", relative gap:"<<TNM_FloatFormat(RGapIndicator,5,15)<<endl;
		}while (!(ReachAccuracy(RGapIndicator) || ReachMaxIter() || ReachMaxTime((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0))));
	}
	cout<<"CPU TIME: "<<1.0*(clock() - m_startRunTime)<<endl;
	//PrintNetLinks();
	return 0;
}


StgLinks*	PTNET::GenerateNonBoardingStg(PTLink* link)
{
	StgLinks* newstg = new StgLinks;		
	newstg->waitT = 0;
	vector<floatType> probs;
	probs.push_back(1.0);
	newstg->SetLinkProbVec(probs);		
	newstg->sname += std::to_string(link->fID) + "-";
	newstg->StgLinkPosVec.push_back(link->fID);	
	return newstg;

}


void PTNET::GenerateBoardingStg(vector<PTLink*> links, vector<PTLink*> &AttractiveSet,floatType &ec/*expected cost*/,floatType &ew/*expected waiting delays*/)
{
	if (links.size()>1)
		sort(links.begin(),links.end(),comp_link_cost);
	floatType ttfreq;
	//cout<<endl<<"tailnode:"<<links[0]->tail->id<<endl;
	PTRTRACE pv = links.begin();
	do 
    {
		AttractiveSet.push_back(*pv);
		ttfreq = 0.0;
		for (PTRTRACE ai = AttractiveSet.begin();ai!= AttractiveSet.end();ai++)
		{
			ttfreq += 1.0/ (*ai)->m_hwmean;
		}
		ew = 1/ttfreq;
		ec = ew;
		double p =0;
		for (PTRTRACE ai = AttractiveSet.begin();ai!= AttractiveSet.end();ai++)
		{
			ec += ((1.0/ (*ai)->m_hwmean)/ttfreq) * ((*ai)->tmpuse);
			(*ai)->m_probdata = (1.0/(*ai)->m_hwmean)/ttfreq;
		}
		pv++;
	}while(pv!=links.end() && (*pv)->tmpuse < ec );

	if (AttractiveSet.size()>1) 
		sort(AttractiveSet.begin(),AttractiveSet.end(),comp_link_id);


}


void PTNET::SaveLinkWaitVariables(int col)
{
	for (int i = 0;i<numOfLink;i++)
		linkVector[i]->buffer[col - 1 ] = linkVector[i]->volume;
	buffer[0] = netTTwaitcost;
	//cout<<"current solution: netTTwaitcost="<<buffer[0]<<endl;
}

void	PTNET::SaveLinkSolution(int col)
{
	PTLink* plink;
	double tmp;
	for (int i = 0;i<numOfLink;i++)
	{
		plink = linkVector[i];
		plink->buffer[col - 1]=  plink->volume;
	}
	buffer[0]=netTTwaitcost;
	cout<<"netwait =  "<<netTTwaitcost<<endl;
}



void	PTNET::NewSolution(int col)
{
	PTLink* plink;
	double tmp;
	//convIndicator = 0.0;
	if (linkBufferSize<col)
	{
		cout<<"\tTNM_A-NewSolution: you need a buffer array of size "<<col<<" on links. "
			<<"\tAbnormal termination!"<<endl;
		exit(0);
	}
	for (int i = 0;i<numOfLink;i++)
	{
		plink = linkVector[i];
		
		//cout<<"link:"<<plink->id<<",old volume:"<<plink->buffer[col - 1]<<",new volume:"<<plink->volume<<",stepSize:"<<stepSize<<endl;

		plink->volume  = plink->buffer[col - 1] + (plink->volume - plink->buffer[col - 1]) * stepSize;

		
		//if (link->buffer[col-1] != link->volume)
		//{
		//	cout<<"oldvol = "<<link->buffer[col-1]<<" link volume is updated to "<<link->volume<<endl;
		//}
	}
	//cout<<"oldnetwait = "<<buffer[0]<<" , auxi netwait =  "<<netTTwaitcost;
	netTTwaitcost = buffer[0] + (netTTwaitcost - buffer[0])* stepSize; //update network wait cost 
	//cout<<" , new netwait =  "<<netTTwaitcost<<endl;
}

floatType PTNET::ComputePTDz()
{
	floatType dz = 0, 
		diff, tmpVol,rtemVol,rdiff;
	PTLink* plink;
	for (int i = 0;i<numOfLink;i++) 
	{
		plink   = linkVector[i];
		if (plink->rLink)
		{
			diff    = plink->volume - plink->buffer[0];
			rdiff	= plink->rLink->volume - plink->rLink->buffer[0];
			tmpVol  = plink->volume;
			rtemVol = plink->rLink->volume;
			plink->volume = plink->buffer[0] + diff*stepSize;
			plink->rLink->volume = plink->rLink->buffer[0] + rdiff*stepSize;
			dz     += diff* plink->GetPTLinkCost();
			plink->volume = tmpVol;
			plink->rLink->volume = rtemVol;

		}
		else
		{
			diff    = plink->volume - plink->buffer[0];
			tmpVol  = plink->volume;
			plink->volume = plink->buffer[0] + diff*stepSize;
			dz  += diff * plink->GetPTLinkCost();
			plink->volume = tmpVol;	
		}
	}
	dz += netTTwaitcost - buffer[0];
	

	return dz;

}




int PTNET::BisecSearch(floatType maxStep)
{
	floatType a = 0, dz = 1.0, b;
	int iter = 0; 
	b = maxStep;
	stepSize = b/2.0;
	int maxLineSearchIter = 100;
	floatType lineSearchAccuracy = 1e-8;
	//cout<<"netTTwaitcost:"<<netTTwaitcost<<", buffer[0]:"<<buffer[0]<<endl;
 //cout<<"Start bisec search, maximum step size = "<<b<<endl;
	//while (((b-a)>=lineSearchAccuracy * maxStep ||dz > 0)&&iter<maxLineSearchIter) {
	while(dz > 0 || (iter < maxLineSearchIter && (b-a)>=lineSearchAccuracy * maxStep)) 
	//while( (iter < maxLineSearchIter && (b-a)>=lineSearchAccuracy * maxStep)) 
	{
		stepSize = (a + b)/2.0;
		iter = iter + 1;
		//stepSize = 0.0;
		dz = ComputePTDz();
		//cout<<"lineiter:"<<iter<<",dz:"<<dz<<endl;
		if (dz<0.0)  a = stepSize;
		else b=stepSize;

		if (iter >= maxLineSearchIter ) 
		{
			stepSize = 1.0;
			break;
		}
		//cout<<"dz = "<<dz<<" iter = "<<iter<<" a="<<a<<" b="<<b<<" b-a = "<<b-a<<" maxstep = "
		//<<maxStep<<" step size = "<<stepSize<<endl;
		//cout<<endl;
		//if(iter > 20) getchar();
	}  
	//getchar();
	if(fabs(stepSize - maxStep) <= b-a) 
	{
		stepSize = maxStep;
		//cout<<"step size is taken as the maximum"<<endl;
	}
	//cout<<"after bisec, dz = "<<dz<<" iter = "<<iter<<" b-a="<<b-a<<" step size = "<<stepSize<<endl;
	////getchar();
	//stepSize =1;
	
	if (dz<0) return 0;
	else return 1;
}

int	PTNET::InitializeHyperpathLS(PTNode* dest)
{
	PTNode *node;
    PTLink *link;
    multimap<double, PTNode*, less<double> > Q;
    for(int i = 0;i<numOfNode;i++)
    {
        node = nodeVector[i];
		node->StgElem->cost = POS_INF_FLOAT;
		node->StgElem->vialink = NULL;
        node->scanStatus = 0;
		node->m_wait = 0.0;
		node->m_attProb.clear();
    }
	for(int i = 0;i<numOfLink;i++)	linkVector[i]->stglinkptr = NULL; 


	dest->StgElem->cost = 0;
    dest->scanStatus = 1;
    Q.insert(std::pair<double, PTNode*>(0.0, dest)); //initialize Q

	int niter = 0;
    multimap<double, PTNode*, less<double> >::iterator firstElem;
	while(!Q.empty())
    {
        niter++;
        firstElem = Q.begin();
        node = firstElem->second;
        node->scanStatus = 0;
		Q.erase(firstElem);

		for(PTRTRACE pv = node->backStar.begin(); pv!= node->backStar.end(); pv++)
		{
			link = *pv;
            PTNode* tailNode = link->tail;
			
			double t = node->StgElem->cost + link->cost;
			
			if(tailNode->StgElem->cost > t  )
			{
				if(link->GetTransitLinkType() == PTLink::ABOARD)
				{
					//construct common-line problem
					int nn = tailNode->forwStar.size();
					vector<PTLink*> candidatelinks;
					for(int i = 0;i<nn;i++)
                    {
						PTLink* vlink = tailNode->forwStar[i];
						if(vlink->GetTransitLinkType() == PTLink::ABOARD && vlink->head->StgElem->cost < POS_INF_FLOAT)	
						{
							vlink->tmpuse = vlink->head->StgElem->cost + vlink->cost;
							candidatelinks.push_back(vlink);				
						}
					}
					if (candidatelinks.size()>0)
					{
						vector<PTLink*> attlinks;
						floatType ct,wt;

						GenerateBoardingStg(candidatelinks,attlinks,ct,wt);
						if(ct < tailNode->StgElem->cost)
						{
							if(tailNode->scanStatus == 0)  //tailnode is not in Q
							{
								Q.insert(std::pair<double, PTNode*>(ct, tailNode));
								tailNode->scanStatus = 1;
							}
							else
							{
								multimap<double, PTNode*, less<double> >::iterator lower =Q.lower_bound(tailNode->StgElem->cost - 1e-10),
									upper = Q.upper_bound(tailNode->StgElem->cost + 1e-10), piter;
								piter = lower;
								while(piter!=upper && piter->second!=tailNode)
								{
									piter++;
								}
								if(piter == Q.end()) 
								{
									cout<<"Warning: cannot find tailNode, it's cost is "<<tailNode->StgElem->cost<<endl;
									cout<<"\tCurrently, Q has the following nodes"<<endl;
                                   
										for(lower = Q.begin(); lower!=Q.end();lower++)
											cout<<lower->second->id<<", "<<lower->first<<","<<lower->second->StgElem->cost<<endl;

									return 1;
								}
								if(piter->second!=tailNode)
								{
									cout<<"\tError: failed to find tailNode "<<tailNode->id<<" while it is in the queue."<<endl;
									return 1;
								}
								else
								{
									Q.erase(piter);
									Q.insert(std::pair<double, PTNode*>(ct, tailNode));
								}
                                
							}
							tailNode->m_wait = wt;
							tailNode->StgElem->cost = ct;
							tailNode->CleanStgLinksOnHyperPath();

							// set stg link pointer
							PTRTRACE pa = attlinks.begin();
							PTLink* curlink = *pa;
							tailNode->m_attProb.push_back(curlink->m_probdata);
					
							tailNode->StgElem->vialink = curlink;
							pa++;
							while (pa!=attlinks.end())
							{
								curlink->stglinkptr = (*pa);													
								curlink = (*pa);
								tailNode->m_attProb.push_back(curlink->m_probdata);
								pa++;
							}

							//cout<<endl;
						}
					}
				}
				else
				{
					
					if(tailNode->scanStatus == 0)  //tailnode is not in Q
					{                              
						Q.insert(std::pair<double, PTNode*>(t, tailNode));
						tailNode->scanStatus = 1;
					}
					else
					{
						multimap<double, PTNode*, less<double> >::iterator lower =Q.lower_bound(tailNode->StgElem->cost - 1e-10),
							upper = Q.upper_bound(tailNode->StgElem->cost + 1e-10), piter;

						piter = lower;
						while(piter!=upper && piter->second!=tailNode)
						{
							piter++;
						}
						if(piter == Q.end()) 
						{
							cout<<"Warning: cannot find tailNode, it's cost is "<<tailNode->StgElem->cost<<endl;
							cout<<"\tCurrently, Q has the following nodes"<<endl;
                                   
								for(lower = Q.begin(); lower!=Q.end();lower++)
									cout<<lower->second->id<<", "<<lower->first<<","<<lower->second->StgElem->cost<<endl;

							return 1;
						}
						if(piter->second!=tailNode)
						{
							cout<<"\tError: failed to find tailNode "<<tailNode->id<<" while it is in the queue."<<endl;
							return 1;
						}
						else
						{
							Q.erase(piter);
							Q.insert(std::pair<double, PTNode*>(t, tailNode));                                    
						}
					}

					if (tailNode->GetTransitNodeType()==PTNode::TRANSFER) 
						tailNode->CleanStgLinksOnHyperPath();
					else
						tailNode->m_attProb.clear();
					tailNode->StgElem->cost = t;
					tailNode->StgElem->vialink = link;		
					tailNode->m_attProb.push_back(1.0);
					//if(tailNode->m_attProb.size()>1) 
					//{
					//	cout<<tailNode->GetTransitNodeType()<<endl;
					//	system("PAUSE");
					//}
					tailNode->m_wait = 0.0;
				}				
			}
		}
	}
	return 0;
}

bool TNM_HyperPath::InitializeHP(PTNode *org, PTNode *dest)
{
	//cout<<"org:"<<org->id<<",cost:"<<org->StgElem->cost<<endl;
	if(dest->StgElem->vialink != NULL || org->StgElem->vialink == NULL)//destination node must carry no outgoing hyperpath
    {
		cout<<"\tHyper path tree was not properly set for od:"<<org->id<<","<<dest->id<<endl;
        return false;
    }
	else
	{
		
		std::queue<PTNode*> Q;  
		Q.push(org);
		org->scanStatus = 1;
		while(!Q.empty())
        {
			PTNode* node = Q.front();
			PTLink* link = node->StgElem->vialink;
			while (link)
			{
				link->head->tmpNumOfIn++;
				if (link->head->scanStatus == 0)
				{
					link->head->scanStatus = 1;
					Q.push(link->head); 
				}
				link = link->stglinkptr;
			}
			Q.pop();
		}
		//rescanning the hyperpath again to build the topological node order. 
		Q.push(org);
		org->m_tmpdata = 1.0;
		AddGlinks(org);

		int count = 0;
		while(!Q.empty())
        {
			PTNode *node = Q.front();            
            count++;
			
			node->scanStatus = 0; //reset scan status;

			PTLink* link = node->StgElem->vialink;
			int ix = 0;
			while (link)
			{
				link->head->tmpNumOfIn--;
				link->head->m_tmpdata += node->m_tmpdata * node->m_attProb[ix];
				//cout<<"link id:"<<link->id<<",tail node prob:"<<node->m_tmpdata<<", link app:"<<node->m_attProb[ix]<<endl;
				if (link->head->tmpNumOfIn == 0)
				{
					AddGlinks(link->head);
					Q.push(link->head);		
				}
				link = link->stglinkptr;
				ix ++;
			}
			Q.pop();
			//if (node==dest) cout<<node->m_tmpdata<<endl;
			node->m_tmpdata = 0.0;
		}
		//cout<<endl;
	 }

	if(m_links[m_links.size()-1]->m_linkPtr->head!=dest)
	//if(m_nodes[m_nodes.size()-1]->m_ptnodePtr != dest)
    {
		cout<<"\tNo valid path between origin and destinaion. "<<endl;
		return false;
	}
	
	
	return true;
	
}

void PTNode::CleanStgLinksOnHyperPath()
{
	//cout<<"links on the node:"<<m_attProb.size()<<endl;
	//cout<<"clear links for node:"<<id<<endl;
	m_attProb.clear();
	PTLink* link = StgElem->vialink;
	StgElem->vialink = NULL;
	while(link)
	{
		PTLink* plink = link->stglinkptr;
		
		link->stglinkptr = NULL;
		link = plink;
	}
}

int PTNET::PTAllOrNothing(PTDestination* dest)
{
	PTOrg* org;
	InitializeHyperpathLS(dest->destination); //InitializeGeneralHyperpathLS(dest->destination);
	for (int j = 0; j<dest->numOfOrg; j++)
	{
		TNM_HyperPath* path = new TNM_HyperPath();
		PTOrg* org= dest->orgVector[j];
		if (path->InitializeHP(org->org,dest->destination))
		{
			double dmd = org->assDemand;//total demand.
			netTTwaitcost += path->WaitCost * dmd;
			//cout<<"od:"<<org->org->id<<"->"<<dest->destination->id<<",waitcost:"<<path->WaitCost<<",dmd:"<<dmd<<endl;
			vector<GLINK*> glinks=path->GetGlinks();
			
			for(int i=0;i<glinks.size();++i)
			{
				glinks[i]->m_linkPtr->volume += glinks[i]->m_data * dmd;
			}
		}
		else
		{
			cout<<"OD-pair <"<<org->org->GetStopPtr()->m_id<<","<<dest->destination->GetStopPtr()->m_id<<"> can not initialize hyperpath tree"<<endl;
			return 1;
		}
	}
	return 0;
}

int PTNET::PTAllOrNothing()
{
	netTTwaitcost = 0.0;
	PTLink *link;
	for(int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		link->volume = 0.0;
	}

	for (int i = 0 ;i<numOfPTDest;i++) //for each destination;
	{
		//cout<<"dest:"<<PTDestVector[i]->destination->id<<endl;
		if (PTAllOrNothing(PTDestVector[i])!=0) return 1; //if not correctly, return 1;
	
	}
	return 0;
}

void	PTNET::FlowReAssignment()
{
	PTLink *link;
	for(int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		link->volume = 0.0;
	}
	for (int i = 0 ;i<numOfPTDest;i++)
	{
		PTDestination* dest= PTDestVector[i];
		for (int j=0; j<dest->numOfOrg; j++)
		{
			PTOrg* org= dest->orgVector[j];
			for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();it++) 
			{
				TNM_HyperPath* path = *it;
				vector<GLINK*> glinks=path->GetGlinks();		
				for(int i=0;i<glinks.size();++i)
				{
					glinks[i]->m_linkPtr->volume += glinks[i]->m_data * path->flow;
				}
			}
		
		}
	
	}
	UpatePTNetworkLinkCost();
}


void	PTNET::CoutLinkInoCAP()
{
	//输出所有弧流量、容量、乘子、费用
	cout<<"============link info=============="<<endl;
	for(int i = 0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		cout<<"id:"<<link->id<<"("<<link->GetTransitLinkTypeName()<<"), cap:"<<TNM_FloatFormat(link->cap,7,2)<<", volume:"<<TNM_FloatFormat(link->volume,6,2)<<", mu:"<<TNM_FloatFormat(link->mu,6,2)<<", cost:"<<TNM_FloatFormat(link->cost,6,2)<<",  gcost:"<<TNM_FloatFormat(link->g_cost,7,2)<<endl;
	}
}


bool	PTNET::InitialHyperpathNetFlow()
{
	PTDestination* dest;

	for (int i = 0;i<numOfPTDest; i++)
	{
		dest = PTDestVector[i];
		InitializeHyperpathLS(dest->destination);
		for (int j=0; j<dest->numOfOrg; j++)
		{		
			TNM_HyperPath* path = new TNM_HyperPath();
			PTOrg* org= dest->orgVector[j];

			if (path->InitializeHP(org->org,dest->destination))
			{
				double dmd = org->assDemand;//total demand.
				//cout<<"od:"<<org->org->id<<"->"<<dest->destination->id<<",waitcost:"<<path->WaitCost<<",dmd:"<<dmd<<endl; //看看是否都分到步行弧上去了
				vector<GLINK*> glinks=path->GetGlinks();		
				for(int i=0;i<glinks.size();i++)
				{
					glinks[i]->m_linkPtr->volume += glinks[i]->m_data * dmd;
					glinks[i]->m_linkPtr->UpdatePTLinkCost();
					glinks[i]->m_linkPtr->UpdatePTDerLinkCost();
					if (glinks[i]->m_linkPtr->rLink)
					{
						glinks[i]->m_linkPtr->rLink->UpdatePTLinkCost();
						glinks[i]->m_linkPtr->rLink->UpdatePTDerLinkCost();			
					}
				}
				path->flow = dmd;
				//cout<<path->name<<endl;
				org->pathSet.push_back(path);
			}
			else
			{
				cout<<"OD-pair <"<<org->org->GetStopPtr()->m_id<<","<<dest->destination->GetStopPtr()->m_id<<"> can not initialize hyperpath tree"<<endl;
				return false;
			}
		}

	}
	return true;
}

void	PTNET::ColumnGeneration(PTDestination* dest,PTOrg* org)
{
	TNM_HyperPath* cpath = new TNM_HyperPath();		
	if (cpath->InitializeHP(org->org,org->dest))
	{
		bool pin=false;
		

		for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin();it!=org->pathSet.end();it++)
		{
			if ((*it)->name == cpath->name)
			{
				pin=true;
				break;
				//cout<<"OD:"<<org->org->id<<"-"<<org->dest->id<<" same path!"<<endl<<endl;
			}
		}
		
		

		if (!pin) org->pathSet.push_back(cpath);
	}

}

int	PTNET::SolvePathTEAP()
{
	m_startRunTime = clock();
	AllocateLinkBuffer(2);// store prob for longest and shortest hyperpath pass the link
	UpatePTNetworkLinkCost();
	InitialHyperpathNetFlow();
	UpatePTNetworkLinkCost();
	
	//PrintNetLinks();
	//ComputeConvGap();
	//cout<<"iter:"<<curIter<<",\tcurrent gap:"<<TNM_FloatFormat(GapIndicator,5,15)<<", relative gap:"<<TNM_FloatFormat(RGapIndicator,5,15)<<", hyperpath num:"<<numaOfHyperpath<<",inneriters:"<<InnerIters<<endl<<endl;
 	if(!(ReachAccuracy(RGapIndicator) || ReachMaxIter() || ReachMaxTime((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0))))
	{
		do
		{
			clock_t sclock = clock();
			PTDestination* dest;
			for (int i = 0;i<numOfPTDest; i++)
			{
				dest = PTDestVector[i];
				if (!InitializeHyperpathLS(dest->destination))
				{
					for (int j=0; j<dest->numOfOrg; j++)
					{													
						PTOrg* org= dest->orgVector[j];
						ColumnGeneration(dest,org) ;
						org->UpdatePathSetCost();
						if (org->pathSet.size()>1)
						{
							if (PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iGreedy || PCTAE_ALG == PCTAE_algorithm::PCTAE_P_Greedy)	UpdateHyperPathGreedyFlow(dest,org);	
							else if (PCTAE_ALG == PCTAE_algorithm::PCTAE_P_NGP || PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iNGP)	UpdateHyperpathGPFlow(dest,org);
						}
					}

				}
				else return 1;
			}

			IterMainlooptime = 1.0 * (clock() - sclock)/CLOCKS_PER_SEC;	
			
			if (PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iGreedy|| PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iNGP) 
				HyperpathInnerLoop(200);

			//FlowReAssignment();

			ComputeConvGap();
			curIter ++;	
			RecordTEAPCurrentIter();
			cout<<"iter:"<<curIter<<",\tcurrent gap:"<<TNM_FloatFormat(GapIndicator,5,15)<<", relative gap:"<<TNM_FloatFormat(RGapIndicator,5,15)<<", hyperpath num:"<<numaOfHyperpath<<",inneriters:"<<InnerIters<<endl<<endl;
			float norm = 0.0;
			for(int i=0;i<numOfLink;i++)
			{   
   				PTLink* link = linkVector[i];
	
				if (link->GetTransitLinkType() == PTLink::ENROUTE)
				{
					if (link->volume>link->cap)
					{
						norm+=pow((link->volume-link->cap),2);
						//norm+=fabs((link->volume-link->cap));
						//cout<<"id:"<<link->id<<",flow:"<<link->volume<<",cap:"<<link->cap<<endl;
					}
				}
			}
			norm = sqrt(norm);
			cout<<"infeasible flow = "<<norm<<endl;
			ReportAvgpaxinfo();
            }while (!(ReachAccuracy(RGapIndicator) || ReachMaxIter() || ReachMaxTime((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0))));
	}
	cout<<"CPU TIME: "<<1.0*(clock() - m_startRunTime)<<endl;



	cout<<"total netcost:"<<endl;

	ReportPTHyperpaths();
	ReportPTlinkflow();
	return 0;
}

void PTNET::UpdateHyperPathGreedyFlow(PTDestination* dest,PTOrg* org)
{

	double beta = 1 ;
	multimap<double, TNM_HyperPath*,less<double>> Orderpaths;
	for(int i = 0;i<org->pathSet.size();i++)
	{				
		TNM_HyperPath* path = org->pathSet[i];     
		path->Preflow = path->flow;
		//path->s1 = 1/ beta; //g_k choose from identity matrix
		path->s1 = path->fdcost == 0 ? 1e-10:path->fdcost/ beta; //g_k choose from identity matrix
		path->s2 = path->cost - path->flow * path->s1;
		Orderpaths.insert(pair<double, TNM_HyperPath*>(path->s2, path));//key is the current label value of  hyperpath  'avg cost'
	}
	multimap<double, TNM_HyperPath*,less<double>>::iterator pv= Orderpaths.begin();     
	vector<TNM_HyperPath*>  AttractivePathSet;
	double B = pv->second->s2 / pv->second->s1;
	double C = 1.0 / pv->second->s1; 
	double w = (org->assDemand + B)/C;
	AttractivePathSet.push_back(pv->second);
	pv++;
	while(pv!=Orderpaths.end() && pv->second->s2 < w)
	{
		B += pv->second->s2 / pv->second->s1;
		C += 1.0 / pv->second->s1; 
		w = (org->assDemand + B)/C;
		AttractivePathSet.push_back(pv->second);
		pv++;
	}
	
	for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();it++) 
	{
		TNM_HyperPath* path = *it;
				
		vector<TNM_HyperPath*>::iterator fit = find(AttractivePathSet.begin(),AttractivePathSet.end(),path);

		if (fit!=AttractivePathSet.end()) 
		{
			path->flow = (w - path->s2 ) / path->s1;						
		}
		else
		{
			path->flow = 0.0;
		}
		double dflow = path->flow - path->Preflow;
		vector<GLINK*> glinks=path->GetGlinks();
		for(int i=0;i<glinks.size();++i)
		{
			PTLink* plink= glinks[i]->m_linkPtr;
			double lprob= glinks[i]->m_data;
			plink ->volume += lprob*dflow;
			if (abs(plink->volume) < 1e-8)
			{
				plink->volume = 0.0;
			}
			plink ->UpdatePTLinkCost();
			plink ->UpdatePTDerLinkCost();
			if (plink ->rLink)
			{
				plink ->rLink ->UpdatePTLinkCost();
				plink ->rLink ->UpdatePTDerLinkCost();//current link flow variation influnce the related link cost and derivative cost
			}
		}			
	}


	org->PathFlowConservation();


}



void PTNET::UpdateHyperpathGPFlow(PTDestination* dest,PTOrg* org)
{
	TNM_HyperPath* MinCostpath=org->pathSet[org->minIx];
	vector<GLINK*> mglinks=MinCostpath->GetGlinks();
	PTLink *link,*rlink;

	for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();it++) 
	{
		TNM_HyperPath* path = *it;
		path->Preflow = path->flow;
		MinCostpath->Preflow = MinCostpath->flow;
		if (path != MinCostpath)
		{
			path->UpdateHyperpathCost();
			MinCostpath->UpdateHyperpathCost();
			vector<GLINK*> glinks= path->GetGlinks();
			floatType DerSum = 0.0;	
			//shift flow from path -> minpath
			for(int i=0;i<mglinks.size();i++)
			{
				link = mglinks[i]->m_linkPtr;
				link->buffer[0] = mglinks[i]->m_data;
				link->markStatus = 1;// tag 1 for arcs on the shortest path
			}
			for(int i=0;i<glinks.size();i++)
			{
				glinks[i]->m_linkPtr->buffer[1] = glinks[i]->m_data;
			}
				
			for(int i=0;i<mglinks.size();i++)
			{
				link = mglinks[i]->m_linkPtr;
				double tmp = link->pfdcost * (link->buffer[1] - link->buffer[0]);
				if (link->rLink)
				{
					tmp += link->rpfdcost * (link->rLink->buffer[1] - link->rLink->buffer[0]);
					//cout<<"rpfdcost="<<link->rpfdcost<<","<<link->rLink->buffer[0]<<","<<link->rLink->buffer[1]<<endl;
				}
				DerSum += tmp *(link->buffer[1] - link->buffer[0]);				
			}
			// calculate derivative for non-shortest links
			for(int i=0;i<glinks.size();i++)
			{
				link = glinks[i]->m_linkPtr;
				if (link->markStatus != 1)
				{
					double tmp = link->pfdcost * (link->buffer[1] - link->buffer[0]);
					if (link->rLink)
					{
						tmp += link->rpfdcost * (link->rLink->buffer[1] - link->rLink->buffer[0]);
						//cout<<"rpfdcost="<<link->rpfdcost<<","<<link->rLink->buffer[0]<<","<<link->rLink->buffer[1]<<endl;
					}
					DerSum += tmp *(link->buffer[1] - link->buffer[0]);	
				
				}
			}


			if (DerSum ==0)	DerSum = 1e-8;//no need to shit flow from current path to the shortest path 
			double dev = path->cost - MinCostpath->cost;
			double dflow;
			if (dev>=0) 
			{
				dflow =  __min(1.0 * dev /DerSum, path->flow);
				//cout<<1.0 * dev /DerSum<<","<< path->flow<<",dflow"<<dflow<<endl;
			}
			else
			{
				dflow =  __max(1.0 * dev /DerSum, -MinCostpath->flow);	
				//cout<<1.0 * dev /DerSum<<","<< MinCostpath->flow<<",dflow"<<dflow<<endl;
			}
			//cout<<"org:"<<org->org->id<<",dest:"<<dest->destination->id<<",dflow:"<<dflow<<",DerSum:"<<DerSum<<",pathcost:"<<path->cost<<",minpathcost:"<<MinCostpath->cost<<endl;
				
			MinCostpath->flow = MinCostpath->flow + dflow; 	
			path->flow = path->flow - dflow; 			
			if (path->flow<0||MinCostpath->flow<0)
			{
				cout<<path->flow<<","<<MinCostpath->flow<<endl;
				system("PAUSE");
			}
			for(int lj=0;lj<glinks.size();lj++)
			{
				link = glinks[lj]->m_linkPtr;
				link ->volume += glinks[lj]->m_data* (path->flow - path->Preflow);
				if (link->volume<0)
				{
					//cout<<link->volume<<",preflow:"<<path->Preflow<<",flow:"<<path->flow<<",dflow:"<<dflow<<endl;
				}

				if (abs(link->volume) < 1e-8) link ->volume = 0.0;
				link ->UpdatePTLinkCost();
				link ->UpdatePTDerLinkCost();
				if (link ->rLink)
				{
					link ->rLink ->UpdatePTLinkCost();
					link ->rLink ->UpdatePTDerLinkCost();//current link flow variation influnce the related link cost and derivative cost
				}
						
				link->buffer[1]=0.0;// reset the link prob
			}

			for(int ii=0;ii<mglinks.size();++ii)
			{
				link = mglinks[ii]->m_linkPtr;
				link ->volume += mglinks[ii]->m_data * (MinCostpath->flow - MinCostpath->Preflow);
				if (link->volume<0)
				{
					//cout<<link->volume<<",preflow:"<<MinCostpath->Preflow<<",flow:"<<MinCostpath->flow<<",dflow:"<<dflow<<endl;
				}
				if (abs(link->volume) < 1e-8) link ->volume = 0.0;
				link->buffer[0]=0.0;// reset the link prob
				link->markStatus =0;
				link ->UpdatePTLinkCost();
				link ->UpdatePTDerLinkCost();
				if (link ->rLink)
				{
					link ->rLink ->UpdatePTLinkCost();
					link ->rLink ->UpdatePTDerLinkCost();//current link flow variation influnce the related link cost and derivative cost
				}
			}
		}
	}
	org->PathFlowConservation();
}

void PTNET::HyperpathInnerLoop(int maxiters)
{
	int n = 0;
	int numofbadODpairs;
	PTDestination* dest;
	clock_t sclock = clock();
	double inner_indicator = RGapIndicator;
	while (n < maxiters)
	{
		numofbadODpairs = 0;
		double tmpcost = 0.0;
		double tmpmincost = 0.0;
		for (int i = 0;i<numOfPTDest; i++)
		{
			dest = PTDestVector[i];
			for (int j=0; j<dest->numOfOrg; j++)
			{
				PTOrg* org= dest->orgVector[j];
				org->UpdatePathSetCost();
			
				double mincost = org->pathSet[org->minIx]->cost;
				double rg = fabs( 1 - mincost * org->assDemand / org->currentTotalCost);		
				if(rg > inner_indicator && org->pathSet.size() > 1)
				{
					if (PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iGreedy)		UpdateHyperPathGreedyFlow(dest,org);	
					else if (PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iNGP)	UpdateHyperpathGPFlow(dest,org);
					numofbadODpairs++;
				}
				tmpcost += org->currentTotalCost;
				tmpmincost += mincost * org->assDemand;
			}
		}
		double crg =  fabs((tmpcost -tmpmincost)/tmpcost);
		inner_indicator = crg;
		n++;
		if(numofbadODpairs == 0 ) break;
	}
	IterInnerlooptime = 1.0 * (clock() - sclock)/CLOCKS_PER_SEC;
	InnerIters = n;
}





int	PTNET::SolveSDTEAP()
{
	SDInitializedCH();
	SDProjGap = 100; 
	ComputeConvGap();
	cout<<"iter:"<<curIter<<",\tcurrent gap:"<<TNM_FloatFormat(GapIndicator,5,15)<<", relative gap:"<<TNM_FloatFormat(RGapIndicator,5,15)<<endl;
	m_startRunTime = clock();
	if(!(ReachAccuracy(RGapIndicator) || ReachMaxIter() || ReachMaxTime((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0))))
	{
		do
		{
			clock_t sclock = clock();
			//SDMasterProblem();
			IterMainlooptime = 1.0 * (clock() - sclock)/CLOCKS_PER_SEC;		
			// solve sub-problem: SPP
			UpatePTNetworkLinkCost(); 
							
			ComputeConvGap();

			curIter ++;			
			numaOfHyperpath =c_h.size();// add the number of extreme point into the 'column of hyperpath' 
		
			RecordTEAPCurrentIter();
			
			if (!SDAddCurrentSolution2CH())
				break;
			cout<<"iter:"<<curIter<<",\tcurrent gap:"<<TNM_FloatFormat(GapIndicator,5,15)<<", relative gap:"<<TNM_FloatFormat(RGapIndicator,5,15)<<", extreme points:"<<c_h.size()<<endl;

			
		}while (!(ReachAccuracy(RGapIndicator) || ReachMaxIter() || ReachMaxTime((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0))));
	}
	cout<<"CPU TIME: "<<1.0*(clock() - m_startRunTime)<<endl;
	return 0;
}

void PTNET::SDInitializedCH()
{
	c_h.clear();
	UpatePTNetworkLinkCost();
	//initialize two extreme points
	PTAllOrNothing();
	
	SDAddCurrentSolution2CH();
	
	PTAllOrNothing();

	SDAddCurrentSolution2CH();
}

bool PTNET::SDAddCurrentSolution2CH()
{
	floatType* solution;
	bool flag; // the flag indicate there exist the same solution in the convex hull
	int ix=1;
	UpatePTNetworkLinkCost();
	
	flag = true;			
	//store a new extreme point
	solution = new floatType[numOfLink+1];// the last variable is the total waiting time 
	for(int i = 0;i<numOfLink;i++)
	{
		PTLink* link =  linkVector[i];
		solution[i] = link->volume;
	}
	solution[numOfLink] = netTTwaitcost;
	// check whether current solution is in the convex hull set 
	for (int i=0; i<c_h.size();i++)
	{
		floatType* h = c_h[i];
		bool isSame=true;
		for (int j =0;j<=numOfLink;j++)
		{
			if (solution[j] != h[j]) isSame=false;
		}

		if (isSame)
		{
			flag =false;
			cout<<"Find the same solution with "<<i<<" extreme point!"<<endl;
			break;
			//system("PAUSE");
		}
	}
	if (flag)  c_h.push_back(solution);

	return flag;
}

double ObjFunc2(unsigned n, const double *x, double *grad, void *data) 
{
	PTNET* net = (PTNET*) data;
	PTLink* link;
	double obj = 0.0;
	for(int i = 0;i<net->numOfLink;i++)
	{
		link =  net->linkVector[i];
		double tmpFlow = 0.0;
		
		for(int j=0;j<n;j++)
		{
			floatType* es = net->c_h[j];
			tmpFlow += x[j] * es[i];	
			//cout<<es[i]<<"\t";
		}
		//cout<<link->volume<<"\t"<<link->cost<<"\t"<<link->pfdcost<<endl;
		//double flow = link ->SCvolume;

		link->volume = link ->SCvolume;
		if (link->rLink) link->rLink->volume = link->rLink ->SCvolume;

		obj += tmpFlow * link->GetPTLinkCost() + (0.5 * link->GetPTLinkDerCost()* tmpFlow * tmpFlow - link->GetPTLinkDerCost() * link->volume * tmpFlow) * net->stepSize;

	}
	for(int j=0;j<n;j++)
	{
		floatType* es = net->c_h[j];
		obj += x[j] *  es[net->numOfLink];
	}
	//cout<<obj<<","<<net->stepSize<<endl;

	return obj;
}

double SimplexEqualityConstraint(unsigned int n, const double *x, double *grad, void *data) 
{
	double s = 0.0;

	for (int i=0;i<n;i++) s += x[i];

	return s - 1.0;
}







void	PTNET::ExportHyperpathUEsolution()
{
	if (PCTAE_ALG ==PCTAE_algorithm::PCTAE_P_Greedy || PCTAE_ALG ==PCTAE_algorithm::PCTAE_P_iGreedy 
		||PCTAE_ALG ==PCTAE_algorithm::PCTAE_P_NGP || PCTAE_ALG ==PCTAE_algorithm::PCTAE_P_iNGP 
		)
	{
		string fileName  = networkName + ".hyperpathue";
		ofstream outfile;
		if (!TNM_OpenOutFile(outfile, fileName))
		{
			cout<<"\n\t Fail to prepare report: Cannot open .hyperpathue file to write information!"<<endl;
			return;
		}
		cout<<"\tWriting hyperpath solution into "<<fileName<<" hyperpath solution!"<<endl;

		PTDestination* dest;
		PTOrg* org;

		for (int i = 0;i<numOfPTDest;i++)
		{
			dest = PTDestVector[i];
			for (int j=0; j<dest->numOfOrg; j++)
			{
				
				org= dest->orgVector[j];
				outfile<<"OD-"<<TNM_IntFormat(j)<<"-"<<TNM_IntFormat(i)<<endl;
				for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();it++) 
				{
					TNM_HyperPath* path = *it;
					ostringstream line("");
					line<<TNM_FloatFormat(path->flow,18,12)<<","<<TNM_FloatFormat(path->WaitCost,18,12)<<","<<TNM_FloatFormat(path->cost,18,12)<<",";
					for(int k=0;k<path->GetGlinks().size();k++)
					{
						GLINK* glink = path->GetGlinks()[k];
						line<<TNM_IntFormat(glink->m_linkPtr->id)<<";"<< TNM_FloatFormat(glink->m_data,18,12) <<",";
					}
					string li = line.str().substr(0,line.str().size()-1);
					outfile<<li<<endl;
				}
			}
		}
		cout<<"finished writing hyperpath solution"<<endl;
		outfile.close();
	}
	else
	{
		cout<<"Please call hyperpath-based algs to write hyperpath flow.";
		system("PAUSE");
	}
}


bool	PTNET::ImportHyperpathUESolution()
{
	string fileName  = networkName + ".hyperpathue";
	ifstream infile;
	PTDestination* dest;
	int oix,dix,lid;
	PTLink* link;
	PTOrg* org;
	floatType flow,wc,p;
    if(!TNM_OpenInFile(infile, fileName))
    {
		cout<<"\t cannot read file"<<fileName<<" to read"<<endl;
        return false;
    }
	vector<string> words,iwords;
	string pline;
	while(getline(infile, pline))
    {
		//cout<<pline<<endl;
        if(!pline.empty())//skip an empty line
        {
			if (pline.substr(0,2)=="OD")
			{
				TNM_GetWordsFromLine(pline, words, '-', '"');
				TNM_FromString(oix, words[1], std::dec);
				TNM_FromString(dix, words[2], std::dec);
				dest = PTDestVector[dix];
				org = dest->orgVector[oix];
				if (!dest || !org)
				{
					cout<<"org or dest does not exist."<<endl;
					return false;
				}
			}
			else
			{
				TNM_HyperPath* path = new TNM_HyperPath();		
				TNM_GetWordsFromLine(pline, words, ',', '"');
				TNM_FromString(flow, words[0], std::dec);
				TNM_FromString(wc, words[1], std::dec);
				path->flow = flow;
				path->WaitCost = wc;
				for(int i=2;i<words.size();i++)
				{
					TNM_GetWordsFromLine(words[i], iwords, ';', '"');
					TNM_FromString(lid, iwords[0], std::dec);
					TNM_FromString(p, iwords[1], std::dec);
					link = CatchLinkPtr(lid);
					path->name += std::to_string(link->id) + "-";
					GLINK* newglink = new GLINK(link);
					newglink->m_data = p;
					link->volume += flow * p;
					path->m_links.push_back(newglink);
				}
				org->pathSet.push_back(path);
				
			}
		}
	}
	return true;
}



void	PTNET::GenerateSPP()
{
	string fileName  = networkName + "-route.txt";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\t Fail to prepare report: Cannot open .hyperpathue file to write information!"<<endl;
		return;
	}
	outfile<<"ORGSTOP,DESTSTOP,LINENAME,DIRECTION,TAILSTOP,HEADSTOP,TRANSFERTYPE,SEQUENCE"<<endl;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		if (link->GetTransitLinkType() ==  PTLink::ENROUTE) 
		{
			link->cost = link->length/35.0*60 ;
		}
		else if (link->GetTransitLinkType() ==  PTLink::ABOARD) link->cost = 0;
		else link->cost = 0.01;
	
	}
	vector<string> words;
	int c=0;
	for(PTStopMapIter pv = m_stops.begin(); pv!= m_stops.end(); pv++) 
	{
		c++;
		if (c%10==0) cout<<c<<endl;
		PTStop* deststop=pv->second;
		PTNode* dest = deststop->GetTransferNode();
		InitializeFeasiblePathLS(dest);

		for(PTStopMapIter pt = m_stops.begin(); pt!= m_stops.end(); pt++) 
		{
			PTStop* orgstop=pt->second;
			PTNode* org = orgstop->GetTransferNode();
			PTNode* node = org;
			int n=0;
			if (orgstop!=deststop)
			{
				//cout<<"org:"<<orgstop->m_id<<",dest:"<<deststop->m_id<<endl;
				string lend = "-1";
				while(node!=dest)
				{
					PTLink* link  = node->StgElem->vialink;
					if (link->GetTransitLinkType() ==  PTLink::ENROUTE)
					{
						n++;
						//cout<<link->m_shape->m_id<<","<<link->tail->GetStopPtr()->m_id<<","<<link->head->GetStopPtr()->m_id<<endl;

						TNM_GetWordsFromLine(link->m_shape->m_id, words, '-');
						if (lend =="-1") outfile<<orgstop->m_id<<","<<deststop->m_id<<","<<words[0]<<","<<words[1]<<","<<link->tail->GetStopPtr()->m_id<<","<<link->head->GetStopPtr()->m_id<<","<<1<<","<<n<<endl;
						else
						{
							if (words[0] == lend )  outfile<<orgstop->m_id<<","<<deststop->m_id<<","<<words[0]<<","<<words[1]<<","<<link->tail->GetStopPtr()->m_id<<","<<link->head->GetStopPtr()->m_id<<","<<0<<","<<n<<endl;
							else outfile<<orgstop->m_id<<","<<deststop->m_id<<","<<words[0]<<","<<words[1]<<","<<link->tail->GetStopPtr()->m_id<<","<<link->head->GetStopPtr()->m_id<<","<<2<<","<<n<<endl;
						}
						

						lend = words[0];
					}
					node = link->head;
				}
				//cout<<"total "<<n<<" stops"<<endl;
			
			}
			//cout<<endl<<endl;;
		
		}


	}


	
}