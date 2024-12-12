#include "PTNet.h"
//#include "TNM_utility.h"
//#include "..\..\..\include\tnm\TNM_utility.h"


bool comp_c_gnode_level(const GNODE *a,const GNODE *b)
{
	return  a->m_tpLevel > b->m_tpLevel;
}

//=========those are function for enlarge capacity ==========

bool	PTNET::InitializeFeasiblePathLS(PTNode* dest)
{
	PTNode *node;
    PTLink *link;
    multimap<double, PTNode*, less<double> > Q;
    for(int i = 0;i<numOfNode;i++)
    {
        node = nodeVector[i];
		node->StgElem->cost = POS_INF_FLOAT;// this store the in feasible flow cost to the destination
		node->StgElem->vialink = NULL; // store the minimum link toward destination
        node->scanStatus = 0;
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
				//update cost
				tailNode->StgElem->cost = t;
				tailNode->StgElem->vialink = link;		
			}
		}

	}
	return true;
}


void PTNET::EnlargeLineCapacityIII()
{

	ZeroTransitLinkFlow();
	PTOrg* org;
	PTNode* node;
	PTDestination* dest;
	double M = 10000.0;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		if (link->GetTransitLinkType() == PTLink::ABOARD)
		{
			link->cost = 1.0/( M * link->cap);
		}
	}
	int n = 0;
	for (int i = 0;i<numOfPTDest;i++)
	{
		dest = PTDestVector[i];
		for (int j=0; j<dest->numOfOrg; j++)
		{	
			PTOrg* org= dest->orgVector[j];
			n++;
			while(org->assDemand!=0)
			{
				//cout<<"org:"<<org->org->id<<",dest:"<<dest->destination->id<<",demand:"<<org->assDemand<<endl;

				InitializeFeasiblePathLS(dest->destination);
				
				floatType assignflow = POS_INF_FLOAT;	
		
				node = org->org;
				while(node!=dest->destination)
				{
					PTLink* link  = node->StgElem->vialink;
					
					if (link->GetTransitLinkType() == PTLink::ENROUTE)
					{
						floatType dflow = link->cap - link->volume;
						if (dflow<assignflow) assignflow = dflow;
					}
					node = link->head;
				}	
				
				if (assignflow > 0) assignflow = min(assignflow,org->assDemand);
				else assignflow = org->assDemand;

				node = org->org;
				while(node!=dest->destination)
				{
					PTLink* link  = node->StgElem->vialink;
					link->volume += assignflow;
	
					if (link->GetTransitLinkType() == PTLink::ABOARD ) //&& link->volume>link->cap
					{
						if (link->rLink->cap>=link->rLink->volume)		link->cost = 1.0/( M * link->rLink->cap);				
						else link->cost = (link->rLink->volume - link->cap)*M; 
					}
					node = link->head;
				}
				org->assDemand -= assignflow;

			}
			if (n%100==0) 
				cout<<"Enlarged "<<n<<" O-D pairs."<<endl;
		}
	}

	string fileName  = networkName + ".largecap";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .stgsolution file to write  Information!"<<endl;
		return;
	}
	cout<<"\tWriting enlarge capacity into "<<fileName<<" !"<<endl;
	outfile<<"shapeid,capacity"<<endl;

	for(PTShapeMapIter ps = m_shapes.begin(); ps!=m_shapes.end();ps++)
	{		 
		PTShape* shp=ps->second;	
		
		floatType maxflow = -1.0;
		for(int i=0;i<numOfLink;i++)
		{
			PTLink* link  = linkVector[i];
			if (link->GetTransitLinkType()== PTLink::ENROUTE && link->volume>maxflow&&link->m_shape==shp)
			{
				maxflow = link->volume;			
			}
		
		}
		
		outfile<<shp->m_id<<","<<ceil(__max(maxflow,shp->m_cap))<<endl;
	}
	outfile.close();
}



void	PTNET::EnlargeLineCapacityII()
{
	ZeroTransitLinkFlow();
	PTOrg* org;
	PTNode* node;
	PTDestination* dest;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		{
			link->cost = 1.0/( 1000.0 * link->cap);
		}

		if (link->GetTransitLinkType() == PTLink::WALK)
		{
			link->cost = 1000;
		}

	}
	
	int n = 0;
	for (int i = 0;i<numOfPTDest;i++)
	{
		dest = PTDestVector[i];
		for (int j=0; j<dest->numOfOrg; j++)
		{	
			PTOrg* org= dest->orgVector[j];
			n++;
			while(org->assDemand!=0)
			{
				//cout<<"org:"<<org->org->id<<",dest:"<<dest->destination->id<<",demand:"<<org->assDemand<<endl;

				InitializeFeasiblePathLS(dest->destination);
				
				floatType assignflow = POS_INF_FLOAT;	
		
				node = org->org;
				while(node!=dest->destination)
				{
					PTLink* link  = node->StgElem->vialink;
					
					if (link->GetTransitLinkType() == PTLink::ENROUTE)
					{
						floatType dflow = link->cap - link->volume;
						if (dflow<assignflow) assignflow = dflow;
					}
					node = link->head;
				}	
				
				if (assignflow > 0) assignflow = min(assignflow,org->assDemand);
				else assignflow = org->assDemand;

				//if (assignflow == 0 && org->assDemand>0)		assignflow = org->assDemand;
				//else if (assignflow > 0 && org->assDemand>0)	assignflow = min(assignflow,org->assDemand);
		

				//if (assignflow<0)
				//{
				//	cout<<"assignflow:"<<assignflow<<",assDemand:"<<org->assDemand<<",infeasibleflow:"<<infeasibleflow<<endl;;
				//}

				//cout<<"assignflow:"<<assignflow<<",assDemand:"<<org->assDemand<<",infeasibleflow:"<<infeasibleflow<<endl;;
				
				node = org->org;
				while(node!=dest->destination)
				{
					PTLink* link  = node->StgElem->vialink;
					link->volume += assignflow;
					//	cout<<link->id<<"("<<link->cost<<","<<link->volume<<")->";			
					if (link->GetTransitLinkType() == PTLink::ENROUTE ) //&& link->volume>link->cap
					{
						//link->cost = link->volume - link->cap; 
						if (link->cap>=link->volume)		link->cost = 1.0/( 1000.0 * link->cap);
						//else if (link->cap == link->volume) link->cost == 0.1;
						else link->cost = link->volume - link->cap; 
					}
					node = link->head;
				}

				org->assDemand -= assignflow;

			}
			if (n%100==0) cout<<"Enlarged "<<n<<" O-D pairs."<<endl;
		}

	}

	string fileName  = networkName + ".largecap";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .stgsolution file to write  Information!"<<endl;
		return;
	}
	cout<<"\tWriting enlarge capacity into "<<fileName<<" !"<<endl;
	outfile<<"shapeid,capacity"<<endl;

	//PrintCapTransitLinkFlow();
	for(PTShapeMapIter ps = m_shapes.begin(); ps!=m_shapes.end();ps++)
	{		 
		PTShape* shp=ps->second;	
		
		floatType maxflow = -1.0;
		for(int i=0;i<numOfLink;i++)
		{
			PTLink* link  = linkVector[i];
			if (link->GetTransitLinkType()== PTLink::ENROUTE && link->volume>maxflow&&link->m_shape==shp)
			{
				maxflow = link->volume;			
			}
		
		}
		
		outfile<<shp->m_id<<","<<ceil(__max(maxflow,shp->m_cap))+5.0<<endl;
	}
	outfile.close();
}


void	PTNET::EnlargeLineCapacity()
{
	ZeroTransitLinkFlow();
	PTOrg* org;
	PTNode* node;
	PTDestination* dest;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		if (link->GetTransitLinkType() == PTLink::ABOARD)
		{
			link->cost = 1.0/( 1000.0 * link->cap);
			//cout<<link->cap<<endl;
		}
	}
	
	int n = 0;
	for (int i = 0;i<numOfPTDest;i++)
	{
		dest = PTDestVector[i];
		for (int j=0; j<dest->numOfOrg; j++)
		{	
			PTOrg* org= dest->orgVector[j];
			n++;
			while(org->assDemand!=0)
			{
				//if (n>1900) cout<<"org:"<<org->org->id<<",dest:"<<dest->destination->id<<",demand:"<<org->assDemand<<endl;

				InitializeFeasiblePathLS(dest->destination);
				//floatType infeasibleflow = org->org->StgElem->cost;
				floatType assignflow = POS_INF_FLOAT;	
		
				node = org->org;
				while(node!=dest->destination)
				{
					PTLink* link  = node->StgElem->vialink;
					
					if (link->GetTransitLinkType() == PTLink::ENROUTE)
					{
						floatType dflow = link->cap - link->volume;
						//if (dflow<0)
						//{
						//	cout<<"impossible dflow,"<<dflow<<",cap："<<link->cap<<",volume"<<link->volume<<endl;
						//}
						if (dflow<assignflow) assignflow = dflow;
					}
					node = link->head;
				}	
				//if (org->assDemand>0)
				//{
				//	if (assignflow <= 0 ) assignflow = org->assDemand;
				//	else assignflow = min(assignflow,org->assDemand);
				//
				//
				//}
				
				if (assignflow == 0 && org->assDemand>0)		assignflow = org->assDemand;
				else if (assignflow > 0 && org->assDemand>0)	assignflow = min(assignflow,org->assDemand);

		/*		if (assignflow <= 0 && org->assDemand>0)		assignflow = org->assDemand;
				else if (assignflow > 0 && org->assDemand>0)	assignflow = min(assignflow,org->assDemand);*/

				//if (assignflow<0)
				//{
				//	cout<<"impossible,"<<assignflow<<endl;
				//	system("PAUSE");
				//}
				//cout<<"assignflow:"<<assignflow<<",assDemand:"<<org->assDemand<<",infeasibleflow:"<<infeasibleflow<<endl;;
				
				node = org->org;
				while(node!=dest->destination)
				{
					PTLink* link  = node->StgElem->vialink;
					link->volume += assignflow;
					//	cout<<link->id<<"("<<link->cost<<","<<link->volume<<")->";			
					if (link->GetTransitLinkType() == PTLink::ABOARD ) //&& link->volume>link->cap
					{
						//link->cost = link->volume - link->cap; 
						if (link->cap>=link->rLink->volume)		link->cost = 1.0/( 1000.0 * link->cap);
						//else if (link->cap == link->volume) link->cost == 0.1;
						else link->cost = link->rLink->volume - link->cap; 
					}
					node = link->head;
				}

				org->assDemand -= assignflow;

			}
			if (n%100==0) cout<<"Enlarged "<<n<<" O-D pairs."<<endl;
		}

	}

	string fileName  = networkName + ".largecap";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .stgsolution file to write  Information!"<<endl;
		return;
	}
	cout<<"\tWriting enlarge capacity into "<<fileName<<" !"<<endl;
	outfile<<"shapeid,capacity"<<endl;

	//PrintCapTransitLinkFlow();
	for(PTShapeMapIter ps = m_shapes.begin(); ps!=m_shapes.end();ps++)
	{		 
		PTShape* shp=ps->second;	
		
		floatType maxflow = -1.0;
		for(int i=0;i<numOfLink;i++)
		{
			PTLink* link  = linkVector[i];
			if (link->GetTransitLinkType()== PTLink::ENROUTE && link->volume>maxflow&&link->m_shape==shp)
			{
				maxflow = link->volume;			
			}
		
		}
		
		outfile<<shp->m_id<<","<<ceil(__max(maxflow,shp->m_cap))<<endl;
	}
	outfile.close();
}



void	PTNET::UpdateLinkCapacity()
{
	cout<<"Start to update link capacity."<<endl;
	string fileName  = networkName + ".largecap";
	ifstream infile;
    if(!TNM_OpenInFile(infile, fileName))
    {
		cout<<"\t cannot read file"<<fileName<<" to read"<<endl;
        return;
    }
	vector<string> words,iwords;
	string pline;
	getline(infile, pline);
	floatType cap;
	while(getline(infile, pline))
    {
		if(!pline.empty())//skip an empty line
		{
			TNM_GetWordsFromLine(pline, words, ',', '"');
			PTShape* shp = GetShapePtr(words[0]);

			for (int i=0;i<shp->m_boardlinks.size();i++)
			{
				PTLink* link = shp->m_boardlinks[i];
				TNM_FromString(cap, words[1], std::dec);			
				link->cap = cap;
				link->rLink->cap = cap; // update the transit link

			}
			/*
			for(int i=0;i<numOfLink;i++)
		    {
				PTLink* link  = linkVector[i];	
				if (link->GetTransitLinkType()== PTLink::ENROUTE ||link->GetTransitLinkType()== PTLink::ABOARD)
				{
					if (link->m_shape->m_id  ==  words[0])
					{
						TNM_FromString(cap, words[1], std::dec);				
						//cout<<"link "<<link->id<<" with original cap "<<link->cap<<" enlarge to "<<cap<<endl;
						link->cap = cap;
					}
				}
			}
			*/
			

		}

	}
	cout<<"Update link capacity finished!"<<endl;
}

void    PTNET::ZeroTransitLinkFlow()
{
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		link->volume = 0.0;
		link->cost = 0;// this store the infeasible flow on this link
	}
}

void	PTLink::UpdateGeneralPTLink_Cost_DerCost_Link(floatType lambda)
{
	if (volume<0&&volume>-1e-10) volume = 0.0;
	if (volume<-1e-10)
	{
		cout<<"link id :"<<id<<" have negative flow:"<<volume<<endl;
	}
	switch(GetTransitLinkType())
    {
		case PTLink::FAILWALK:
			double alpha_11,wt1;
			alpha_11=pars[0];
			wt1=	pars[1];
			cost = alpha_11 * wt1;
			pfdcost = 0.0;
			g_cost = cost;	
			break;

		case PTLink::WALK:
			double alpha_1,wt;
			alpha_1=pars[0];
			wt=	pars[1];
			cost = alpha_1 * wt;
			pfdcost = 0.0;			
			g_cost = cost;	
			break;
		case PTLink::ABOARD:
			double alpha_2,beta_2,n1,cap1;
			alpha_2	=	pars[0];
			beta_2	=	pars[1];
			n1		=	pars[2];//power
			cap1		=	cap;	

			if (m_tlSym==PTLink::ASYMMTRIC) // link cost function is asymmric
			{
				if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;
				cost = alpha_2 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1);
				pfdcost = alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1 - 1) * ( (1 - beta_2) / cap1);
				rpfdcost= alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1 - 1) * (beta_2 / cap1);

				if (m_tlCap == PTLink::TL)
				{
					g_cost = cost;			
				}
				else
				{
				   g_cost = cost + max(long double(0), mu + lambda* ( rLink->volume -  rLink->cap));
				}
			}
			else
			{
				cost = alpha_2 * pow ( volume / cap1 , n1);
				pfdcost =  alpha_2 * n1 * pow( volume / cap1 , n1-1) * (1/cap1);
				rpfdcost = 0.0;

				if (m_tlCap == PTLink::TL)
				{
					g_cost = cost;
				
				}
				else
				{
				   g_cost = cost + max(long double(0),mu + lambda* ( rLink->volume -  rLink->cap));			
				}
			}
			
			break;
		case PTLink::ALIGHT:
			double alpha_4,tloss;
			alpha_4=	pars[0];
			tloss =		pars[1];
			cost  = alpha_4 * tloss;
			pfdcost = 0.0;

			//cost = alpha_4 * tloss+ 1e-6*volume;
			//pfdcost = 1e-6;
			g_cost = cost;
			
			break;
		case PTLink::ENROUTE:
			double alpha_3,beta_3,gamma_3,n2,cap2,fft;
			alpha_3	=	pars[0];
			beta_3	=	pars[1];
			gamma_3	=	pars[2];
			n2		=	pars[3];
			cap2	=	cap;
			fft		=	pars[5];

			if(m_tlSym==PTLink::ASYMMTRIC) // link cost function is asymmric
			{
				if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;
				cost = alpha_3 * fft + beta_3 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2);
				pfdcost  = beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2 - 1) * (1 / cap2);
				rpfdcost = beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2 - 1) * ((gamma_3 - 1) / cap2);
				if (m_tlCap == PTLink::TL)
				{
					g_cost = cost + max(long double(0),mu + lambda* (volume - cap));		
				}
				else
				{
				   g_cost = cost;		
				}
			}
			else 
			{
				//if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;
				cost = alpha_3 * fft + beta_3 * pow (volume / cap2, n2);
				pfdcost  =  beta_3 * n2 * pow(volume/ cap2,n2-1)*(1/cap2);
				rpfdcost = 0.0;
				if (m_tlCap == PTLink::TL)
				{
					g_cost = cost + max(long double(0),mu + lambda* (volume - cap));				
				}
				else
				{
				  g_cost = cost;		
				}
			}
			break;
	}
	
}


void	PTLink::UpdateGeneralPTLink_Cost_DerCost(floatType lambda)
{
	if (volume<0&&volume>-1e-10) volume = 0.0;
	if (volume<-1e-10)
	{
		cout<<"link id :"<<id<<" have negative flow:"<<volume<<endl;
	}
	switch(GetTransitLinkType())
    {
		case PTLink::FAILWALK:
			double alpha_11,wt1;
			alpha_11=pars[0];
			wt1=	pars[1];
			cost = alpha_11 * wt1;
			pfdcost = 0.0;
			g_cost = cost;	
			break;

		case PTLink::WALK:
			double alpha_1,wt;
			alpha_1=pars[0];
			wt=	pars[1];
			cost = alpha_1 * wt;
			pfdcost = 0.0;			
			g_cost = cost;	
			break;
		case PTLink::ABOARD:
			double alpha_2,beta_2,n1,cap1;
			alpha_2	=	pars[0];
			beta_2	=	pars[1];
			n1		=	pars[2];//power
			cap1		=	cap;	

			if (m_tlSym==PTLink::ASYMMTRIC) // link cost function is asymmric
			{
				if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;
				cost = alpha_2 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1);
				pfdcost = alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1 - 1) * ( (1 - beta_2) / cap1);
				rpfdcost= alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1 - 1) * (beta_2 / cap1);

				if (m_tlCap == PTLink::TL)
				{
					g_cost = cost;			
				}
				else
				{
				   g_cost = cost + max(long double(0), mu + lambda* ( rLink->volume -  rLink->cap));
				}
			}
			else
			{
				cost = alpha_2 * pow ( volume / cap1 , n1);
				pfdcost =  alpha_2 * n1 * pow( volume / cap1 , n1-1) * (1/cap1);
				rpfdcost = 0.0;

				if (m_tlCap == PTLink::TL)
				{
					g_cost = cost;
				
				}
				else
				{
				   g_cost = cost + max(long double(0),mu + lambda* ( rLink->volume -  rLink->cap));			
				}
			}

			break;
		case PTLink::ALIGHT:
			double alpha_4,tloss;
			alpha_4=	pars[0];
			tloss =		pars[1];
			cost  = alpha_4 * tloss;
			pfdcost = 0.0;

			//cost = alpha_4 * tloss+ 1e-6*volume;
			//pfdcost = 1e-6;
			g_cost = cost;
			
			break;
		case PTLink::ENROUTE:
			double alpha_3,beta_3,gamma_3,n2,cap2,fft;
			alpha_3	=	pars[0];
			beta_3	=	pars[1];
			gamma_3	=	pars[2];
			n2		=	pars[3];
			cap2	=	cap;
			fft		=	pars[5];

			if(m_tlSym==PTLink::ASYMMTRIC) // link cost function is asymmric
			{
				if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;
				cost = alpha_3 * fft + beta_3 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2);
				pfdcost  = beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2 - 1) * (1 / cap2);
				rpfdcost = beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2 - 1) * ((gamma_3 - 1) / cap2);
				if (m_tlCap == PTLink::TL)
				{
					g_cost = cost + max(long double(0),mu + lambda* (volume - cap));		
				}
				else
				{
				   g_cost = cost;		
				}
			}
			else 
			{
				//if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;
				cost = alpha_3 * fft + beta_3 * pow (volume / cap2, n2);
				pfdcost  =  beta_3 * n2 * pow(volume/ cap2,n2-1)*(1/cap2);
				rpfdcost = 0.0;
				if (m_tlCap == PTLink::TL)
				{
					g_cost = cost + max(long double(0),mu + lambda* (volume - cap));				
				}
				else
				{
				  g_cost = cost;		
				}
			}
			break;
	}
}


void	PTLink::BL_UpdateGeneralPTLink_Cost_DerCost(floatType lambda)
{
	floatType tvol,rtvol = 0.0;
	if(volume<0.0) 
	{
		tvol = 0.0;
		if (fabs(volume)>1e-10)
		{
			cout<<"link id :"<<id<<" have negative flow:"<<volume<<endl;
		}
		else
		{
			volume = 0.0;
		}
	}
	else tvol = volume;
	if (rLink)
	{
		if (rLink->volume<0.0) 
		{
			//cout<<"link id :"<<rLink->id<<" have negative flow:"<<rLink->volume<<endl;
			rtvol = 0.0;
			if (fabs(rLink->volume)>1e-10)
			{
				cout<<"link id :"<<rLink->id<<" have negative flow:"<<rLink->volume<<endl;
			}
			else
			{
				rLink->volume = 0.0;
			}
		}
		else  rtvol = rLink ->volume;
	}
	switch(GetTransitLinkType())
    {
		case PTLink::WALK:
			double alpha_1,wt;
			alpha_1=pars[0];
			wt=	pars[1];
			cost = alpha_1 * wt;
			g_cost = cost;
			pfdcost = 0.0;
			g_pfdcost = pfdcost;
			break;
		case PTLink::ABOARD:
			double alpha_2,beta_2,n1,cap1;
			alpha_2	=	pars[0];
			beta_2	=	pars[1];
			n1		=	pars[2];//power
			cap1		=	cap;			
			cost = alpha_2 * pow (((1 - beta_2) * tvol + beta_2 * rtvol) / cap1 , n1);			
			g_cost = cost + max(long double(0),mu + lambda* (rLink->volume - cap));
			//derivative cost
			pfdcost = alpha_2 * n1 * pow (((1 - beta_2) * tvol + beta_2 * rtvol) / cap1 , n1 - 1) * ( (1 - beta_2) / cap1);
			rpfdcost= alpha_2 * n1 * pow (((1 - beta_2) * tvol + beta_2 * rtvol) / cap1 , n1 - 1) * (beta_2 / cap1);
				
			break;
		case PTLink::ALIGHT:
			double alpha_4,tloss;
			alpha_4=	pars[0];
			tloss =		pars[1];
			cost  = alpha_4 * tloss;
			g_cost = cost;
			pfdcost = 0.0;
			break;
		case PTLink::ENROUTE:
			double alpha_3,beta_3,gamma_3,n2,cap2,fft;
			alpha_3	=	pars[0];
			beta_3	=	pars[1];
			gamma_3	=	pars[2];
			n2		=	pars[3];//power
			cap2	=	cap;
			fft		=	pars[5];
			cost = alpha_3 * fft + beta_3 * pow ((tvol + (gamma_3 - 1) * rtvol) / cap2, n2);
			g_cost = cost;
			//derivative cost
			pfdcost  = beta_3 * n2 * pow ((tvol + (gamma_3 - 1) * rtvol) / cap2, n2 - 1) * (1 / cap2);
			rpfdcost = beta_3 * n2 * pow ((tvol + (gamma_3 - 1) * rtvol) / cap2, n2 - 1) * ((gamma_3 - 1) / cap2);
			break;

	}
	if (fabs(cost)<1e-10) cost = 0.0;
	if (fabs(g_cost)<1e-10) g_cost = 0.0;




}

void	PTLink::TL_UpdateGeneralPTLink_Cost_DerCost(floatType lambda)
{
	floatType tvol,rtvol = 0.0;
	if(volume<0.0) 
	{
		tvol = 0.0;
		if (fabs(volume)>1e-10)
		{
			cout<<"link id :"<<id<<" have negative flow:"<<volume<<endl;
		}
		else
		{
			volume = 0.0;
		}
	}
	else tvol = volume;
	if (rLink)
	{
		if (rLink->volume<0.0) 
		{
			//cout<<"link id :"<<rLink->id<<" have negative flow:"<<rLink->volume<<endl;
			rtvol = 0.0;
			if (fabs(rLink->volume)>1e-10)
			{
				cout<<"link id :"<<rLink->id<<" have negative flow:"<<rLink->volume<<endl;
			}
			else
			{
				rLink->volume = 0.0;
			}
		}
		else  rtvol = rLink ->volume;
	}
	switch(GetTransitLinkType())
    {
		case PTLink::WALK:
			double alpha_1,wt;
			alpha_1=pars[0];
			wt=	pars[1];
			cost = alpha_1 * wt;
			g_cost = cost;
			pfdcost = 0.0;
			g_pfdcost = pfdcost;
			break;
		case PTLink::ABOARD:
			double alpha_2,beta_2,n1,cap1;
			alpha_2	=	pars[0];
			beta_2	=	pars[1];
			n1		=	pars[2];//power
			cap1		=	cap;				
			cost = alpha_2 * pow (((1 - beta_2) * tvol + beta_2 * rtvol) / cap1 , n1);
			g_cost = cost;
			//derivative cost
			pfdcost = alpha_2 * n1 * pow (((1 - beta_2) * tvol + beta_2 * rtvol) / cap1 , n1 - 1) * ( (1 - beta_2) / cap1);
			rpfdcost= alpha_2 * n1 * pow (((1 - beta_2) * tvol + beta_2 * rtvol) / cap1 , n1 - 1) * (beta_2 / cap1);
			g_pfdcost = pfdcost;
			if (rLink)// it is a transit link
			{
				if (rtvol>= (rLink->cap - rLink->mu/lambda)) g_rpfdcost = rpfdcost + lambda;
				else g_rpfdcost = rpfdcost;		
			}

			if (g_rpfdcost<0||g_pfdcost<0)
			{
				cout<<"g_pfdcost:"<<g_pfdcost<<",g_rpfdcost:"<<g_rpfdcost<<endl;
				system("PAUSE");
			}

			break;
		case PTLink::ALIGHT:
			double alpha_4,tloss;
			alpha_4=	pars[0];
			tloss =		pars[1];
			cost  = alpha_4 * tloss;
			g_cost = cost;
			pfdcost = 0.0;
			g_pfdcost = pfdcost;
			break;
		case PTLink::ENROUTE:
			double alpha_3,beta_3,gamma_3,n2,cap2,fft;
			alpha_3	=	pars[0];
			beta_3	=	pars[1];
			gamma_3	=	pars[2];
			n2		=	pars[3];//power
			cap2	=	cap;
			fft		=	pars[5];
			cost = alpha_3 * fft + beta_3 * pow ((tvol + (gamma_3 - 1) * rtvol) / cap2, n2);
			g_cost = cost + max(long double(0),mu + lambda* (tvol - cap));
			//derivative cost
			pfdcost  = beta_3 * n2 * pow ((tvol + (gamma_3 - 1) * rtvol) / cap2, n2 - 1) * (1 / cap2);
			rpfdcost = beta_3 * n2 * pow ((tvol + (gamma_3 - 1) * rtvol) / cap2, n2 - 1) * ((gamma_3 - 1) / cap2);
			if (tvol>= (cap - mu/lambda))  g_pfdcost = pfdcost + lambda;		
			else g_pfdcost = pfdcost;
			if (rLink)        
			{
				g_rpfdcost = rpfdcost; 
			}

			if (g_rpfdcost<0||g_pfdcost<0)
			{
				cout<<"g_pfdcost:"<<g_pfdcost<<",g_rpfdcost:"<<g_rpfdcost<<endl;
				system("PAUSE");
			}
			break;

	}
	if (fabs(cost)<1e-10) cost = 0.0;
	if (fabs(g_cost)<1e-10) g_cost = 0.0;

}


void	PTNET::UpdatePTNetGeneralCost_Dercost()
{
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];

		link->UpdateGeneralPTLink_Cost_DerCost(cb_lambda);


			
			//if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL ||PCTAE_ALG ==PCTAE_algorithm::CAP_PCTAE_B_TL) 
		//	link->TL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);	
		//else if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_BL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_BL||PCTAE_ALG ==PCTAE_algorithm::CAP_PCTAE_B_BL  ||PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy )  
		//	link->BL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);	
	}
}

int		PTNET::IPFInitializeGeneralHyperpathLS(PTNode* dest)

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
		//cout<<niter<<endl;
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


int		PTNET::InitializeGeneralHyperpathLS(PTNode* dest)
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
		//cout<<niter<<endl;
        firstElem = Q.begin();
        node = firstElem->second;
        node->scanStatus = 0;
		Q.erase(firstElem);

		for(PTRTRACE pv = node->backStar.begin(); pv!= node->backStar.end(); pv++)
		{
			link = *pv;
            PTNode* tailNode = link->tail;
			
			double t = node->StgElem->cost + link->g_cost;
			
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
							vlink->tmpuse = vlink->head->StgElem->cost + vlink->g_cost;
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

void PTNET::IniTransitLinkMultiplier()
{
	//double ff=0;
	double tn = 0,tvalue = 0;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];	
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		{
			floatType tmp = link->volume;
			link->volume = link->cap;
			floatType cap_cost = link->GetPTLinkCost(); 
			link->volume = tmp;

			link->mu = max(long double(0), link->cost  - cap_cost );
			if (link->volume>link->cap)
			{
				tn++;
				tvalue += cap_cost/(link->volume - link->cap);
			}
		}
		else link->mu = 0;	
	}
	cb_lambda = tvalue/tn;
	//cout<<"if::"<<ff<<endl;
	cout<<"calulate iniLamda="<<cb_lambda<<endl;
}



void PTNET::IniBoardLinkMultiplier()
{
	double ff=0;
	double tn = 0,tvalue = 0;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
	
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		{
			floatType tmp = link->volume;
			link->volume = link->cap;
			floatType cap_cost = link->GetPTLinkCost(); 
			link->volume = tmp;
			link->mu = 0;	
			link->rLink->mu = max(long double(0), link->cost  - cap_cost ); //impose such a delay on board links
			if (link->volume>link->cap)
			{
				tn++;
				tvalue += cap_cost/(link->volume - link->cap);
			}
		}
		else if (link->GetTransitLinkType() == PTLink::WALK || link->GetTransitLinkType() ==PTLink::ALIGHT || link->GetTransitLinkType() == PTLink::FAILWALK) link->mu = 0;	



		//if (link->GetTransitLinkType() == PTLink::ABOARD)
		//{

			//floatType tmp = link->volume;
			//double newboardcap =   max(long double(0),  link->volume - ( link->rLink->cap - link->rLink->volume ) );  // the capacity for the boarding link: current flow + residual capacity for rlink
			//link->volume =  newboardcap;
			//
			//floatType cap_cost = link->GetPTLinkCost(); 
			//link->volume = tmp;
			//link->mu = max(long double(0), link->cost  - cap_cost );
			//if (link->volume>newboardcap)
			//{
			//	tn++;
			//	tvalue += cap_cost/(link->volume - newboardcap);
			//}

			//floatType alightflow = 0.0, boardflow = 0.0;
			//for (int i = 0; i<=link->seq-1;i++) alightflow += link->m_shape->m_alightlinks[i]->volume;
			
			//for (int i=0;i<=link->seq;i++) boardflow += link->m_shape->m_boardlinks[i]->volume;

			//floatType tmp = link->rLink->volume;

			//cout<<link->seq<<","<<alightflow - boardflow<<","<<link->rLink->volume<<endl;
			//link->volume = link->cap + alightflow - boardflow;
			//link->rLink->volume = link->cap;
			//floatType cap_cost = link->rLink->GetPTLinkCost(); 
			//link->rLink->volume = tmp;
			//link->mu = max(long double(0), link->rLink->cost  - cap_cost );
			//cout<<"lid:"<<link->id<<",mu:"<<link->mu<<endl;

			//if (link->rLink->volume>link->cap) //infeasible flow > 0
			//{
			//	tn++;
			//	//tvalue += cap_cost/(link->volume - link->cap - alightflow + boardflow);
			//	tvalue += cap_cost/(link->rLink->volume - link->cap);
			//}
			//ff += max(long double(0), link->volume   - link->cap - alightflow + boardflow);
	//	}
	//	else link->mu = 0;	
	}
	cb_lambda = tvalue/tn;
	//cout<<"if::"<<ff<<endl;
	//cout<<"ini cb_lambda:"<<cb_lambda<<endl;

}


void PTNET::IniCapacityPars()
{
	//cb_lambda = 1e-1;
	cb_sigma = 0.8;
	cb_xi = 2.0;
	//SubIter = 5;
	MaxSubIter = 25;
	SubRG = 1.0;
	MaxFlowInfeasibility = 1.0;  //1e-8;
	//infeasibleflow = 10.0;
	SubRGCriterion = 1e-4;
}


void PTNET::IniCapLambda()
{
	floatType tmp = 0.0; 
	int tn = 0;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		{
			if (link->volume>link->cap)
			{
				tmp += (link->volume - link->cap);
				tn++;
			
				
			}
		
}
	}
}


floatType	PTNET::IPFComputeNormFlowInfeasibility(bool islastflow)
{
	floatType norm = 0.0;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		{
			if (!islastflow&& link->volume>link->cap)  norm+=pow((link->volume-link->cap),2);
			if (islastflow && link->volume>link->cap)  norm+=pow((link->buffer[1]-link->cap),2);
		}
	}
	return pow(norm,0.5);
}

floatType	PTNET::ComputeNormFlowInfeasibility(bool islastflow)
{
	floatType norm = 0.0;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
	
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		{
			if (!islastflow && link->volume>link->cap)
			{
				norm+=pow((link->volume-link->cap),2);
				//norm+=fabs((link->volume-link->cap));
				//cout<<"id:"<<link->id<<",flow:"<<link->volume<<",cap:"<<link->cap<<endl;
			}
			if (islastflow&& link->SCvolume>link->cap)
			{
				norm+=pow((link->SCvolume-link->cap),2);
			}

		}
	}
	return pow(norm,0.5);
}

floatType PTNET::ComputeBoundGAP()
{

	floatType gap = 0.0;	
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
	
		if (link->GetTransitLinkType() == PTLink::ENROUTE &&link->GetTransitLinkCapType()==PTLink::TL)
		{
			//gap += pow(max(long double(0),-cb_lambda*(link->cap - link->volume)),2);
			//gap += pow(max(long double(0), link->mu + cb_lambda *(link->volume - link->cap)),2) - pow(link->mu, 2); //原问题目标值最优解的上下界
			gap += max(long double(0), pow(max(long double(0), link->mu + cb_lambda *(link->volume - link->cap)),2) - pow(link->mu, 2) ); 
		}

		if (link->GetTransitLinkType() == PTLink::ABOARD &&link->GetTransitLinkCapType()==PTLink::BL)
		{
			gap += pow(max(long double(0),-cb_lambda*(link->rLink->cap - link->rLink->volume)),2);
		}
	}
	
	gap = 1.0/(2.0*cb_lambda)*gap;
	//gap = 1.0/(2.0*cb_lambda)*pow(gap,0.5);
	//return  1.0/(2.0*cb_lambda)*pow(gap, 0.5);
	//cout<<"multiConv ="<< gap<<endl;
	return  gap;
}

void	PTNET::IPFComputeCapSubConvGap()
{
	floatType gap, tt = 0.0, minimumTcost = 0.0;
	PTLink *link;
	PTDestination* dest;
	PTOrg* org;
	numaOfHyperpath = 0;
	for (int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		tt += link->volume * link->cost;// summarize generalized cost
	}

	if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL
		)
	{
		netTTwaitcost = 0;
	}
	for (int i = 0;i<numOfPTDest;i++)
	{
		dest = PTDestVector[i];

		IPFInitializeGeneralHyperpathLS(dest->destination);

		for (int j=0; j<dest->numOfOrg; j++)
		{
			org= dest->orgVector[j];
			minimumTcost += org->org->StgElem->cost * org->assDemand;

			if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL|| PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL )
			{
				int k = 0;

				for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();) 
				{
					TNM_HyperPath* path = *it;
					if (path->flow == 0.0) // column dropping
					{
						it = org->pathSet.erase(it);			
					}
					else
					{			
						netTTwaitcost += path->WaitCost * path->flow;	

						it++;
						k++;
					}
				}	
				numaOfHyperpath += k;
				org->currentRelativeGap = 1.1;
			}
		}
	}
	tt += netTTwaitcost;
	gap = tt - minimumTcost;
	SubRG = fabs(gap/tt);
	netTTcost = tt;
}


void	PTNET::ComputeCapSubConvGap()
{
	floatType gap, tt = 0.0, minimumTcost = 0.0;
	PTLink *link;
	PTDestination* dest;
	PTOrg* org;
	numaOfHyperpath = 0;
	for (int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		tt += link->volume * link->g_cost;// summarize generalized cost
	}

	if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL)
	{
		netTTwaitcost = 0;
	}
	for (int i = 0;i<numOfPTDest;i++)
	{
		dest = PTDestVector[i];

		InitializeGeneralHyperpathLS(dest->destination);

		for (int j=0; j<dest->numOfOrg; j++)
		{
			org= dest->orgVector[j];
			minimumTcost += org->org->StgElem->cost * org->assDemand;

			if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL )
			{
				int k = 0;

				for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();) 
				{
					TNM_HyperPath* path = *it;
					if (path->flow == 0.0) // column dropping
					{
						it = org->pathSet.erase(it);			
					}
					else
					{			
						netTTwaitcost += path->WaitCost * path->flow;	

						it++;
						k++;
					}
				}	
				numaOfHyperpath += k;
				org->currentRelativeGap = 1.1;
			}
		}
	}
	
	tt += netTTwaitcost;
	gap = tt - minimumTcost;
	SubRG = fabs(gap/tt);
	//cout<<"Now: netTTwaitcost="<<netTTwaitcost<<"  netTTcost="<<tt<<"  minimumTcost="<<minimumTcost<<"  gap="<<gap<<"  SubRG"<<SubRG<<endl;
}





void	PTNET::ReportWalkflowRation()
{
	string WalkinfoName  = networkName +"-" + GetAlgorithmName() + ".walk";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, WalkinfoName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .pathinfo file to write hyperpath Information!"<<endl;
		return;
	}
	
	cout<<"\tWriting net walk information into "<<WalkinfoName<<" for covergence info!"<<endl;
	outfile<<"Origin,Destination,numberofpaths,demand,Newwalkflow,Orgwalkflow,Walkflow"<<endl<<endl;
	
	PTDestination* dest;
	PTOrg* org;
	float ttflow = 0.0;
	float ttwalkflow = 0.0, ttnewwalkflow = 0.0, ttorgwalkflow = 0.0;

	//由超路径计算广义费用
	
	for (int i = 0;i<numOfPTDest;i++)
	{
		dest = PTDestVector[i];
		for (int j=0; j<dest->numOfOrg; j++) //OD对
		{
			org= dest->orgVector[j];
			float odwalkflow = 0.0, odnewwalkflow = 0.0, odorgwalkflow = 0.0;
			//ttflow+=org->assDemand;
			outfile<<"("<<org->org->id<<","<<dest->destination->id<<")"<<org->pathSet.size()<<","<<org->assDemand<<",";
			for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();it++)  //超路径集中的超路径
			{
				TNM_HyperPath* path = *it; 
				vector<GLINK*> glinks= path->GetGlinks();
				
				ttflow += path->flow;
				float odhpwalkflow = 0.0; //这一条超路径的步行客流

				for(int k=0;k<glinks.size();k++)  //超路径中的各段弧
				{
					if(glinks[k]->m_linkPtr->GetTransitLinkType()==PTLink::WALK)
					{
						if(glinks[k]->m_linkPtr->odwalk==true) {odnewwalkflow += path->flow;} //O-D之间原本在步行阈值范围内构建的换乘步行弧 流量
						else {odorgwalkflow += path->flow;}  //O-D之间在步行阈值范围外新增的起终点步行弧 流量
						odwalkflow += path->flow; //这一对O-D的步行客流
						break;
					}
				}
			}
			ttwalkflow += odwalkflow; //网络总的步行客流
			ttnewwalkflow += odnewwalkflow;
			ttorgwalkflow += odorgwalkflow;
			outfile<<odnewwalkflow<<","<<odorgwalkflow<<","<<odwalkflow<<endl;
		}
	}


	WalkflowRation  = ttwalkflow/ttflow;
	cout<<"ttwalkflow="<<ttwalkflow<<", ttflow="<<ttflow<<", WalkflowRation="<<WalkflowRation<<endl;


	outfile<<"Number of O-Dpairs:"<<numOfPTOD<<", ttorgwalkflow="<<ttorgwalkflow<<", ttnewwalkflow="<<ttnewwalkflow<<", ttwalkflow="<<ttwalkflow<<", ttflow="<<ttflow<<endl;
	outfile<<"WalkflowRation="<<WalkflowRation<<endl;

	outfile.close();


}



void PTNET::UpdateMultiplier()
{
	floatType norm = 0.0;

	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
	
		if (link->GetTransitLinkType() == PTLink::ENROUTE &&link->GetTransitLinkCapType()==PTLink::TL)
		{
			link->mu = max(long double(0), link->mu + cb_lambda *(link->volume - link->cap) );
		}

		if (link->GetTransitLinkType() == PTLink::ABOARD &&link->GetTransitLinkCapType()==PTLink::BL)
		{
			link->mu = max(long double(0), link->mu + cb_lambda *(link->rLink->volume - link->rLink->cap) );
		}
	}

}



void PTNET::PrintCapTransitLinkFlow()
{
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
	
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		{
			cout<<"link id:"<<link->id<<",flow:"<<link->volume<<",cap:"<<link->cap<<",mu:"<<link->mu<<",cost:"<<link->cost<<",g_cost:"<<link->g_cost<<endl;
		}
	}


}



floatType PTNET::AssignCapFeasibleFlow(PTDestination* dest,PTOrg* org)
{
	PTNode* node;
	for(int i=0;i<numOfNode;i++)
	{
		node = nodeVector[i];
		node->StgElem->vialink = NULL;//this store the forward maximum residual capacity on the bush
		node->StgElem->cost = -POS_INF_FLOAT;
		node->tmpNumOfIn = node->NumOfDestOutLink();
		if (node->m_stgNode)
		{
			for(PTRTRACE pv = node->forwStar.begin(); pv!= node->forwStar.end(); pv++)
			{
				PTLink* link = *pv;
				if (link->markStatus>0 && link->GetTransitLinkType()!=PTLink::WALK)
				{
					floatType rflow = link->cap - link->volume;
					//cout<<"node:"<<node->id<<",vialink:"<<link->id<<",rflow:"<<rflow<<", link volume:"<< link->volume<<",cap:"<<link->cap<<endl;
					if (rflow>node->StgElem->cost)
					{
						node->StgElem->vialink = link;
						node->StgElem->cost = rflow;
						
					}	
				}	
			}
		
		}
	
	}
	dest->destination->StgElem->cost = POS_INF_FLOAT;
	vector<GNODE*>::iterator pg;
	GNODE  *gnode;
	for (pg = dest->tplNodeVec.begin(); pg!=dest->tplNodeVec.end(); pg++)
	{
		gnode = *pg;
		for(PTRTRACE pv = gnode->m_ptnodePtr->forwStar.begin(); pv!= gnode->m_ptnodePtr->forwStar.end(); pv++)
		{
			PTLink* link = *pv;
			if(link->markStatus>0 && link->GetTransitLinkType()==PTLink::WALK)
			{
				if (link->head->StgElem->cost > gnode->m_ptnodePtr->StgElem->cost)
				{
					gnode->m_ptnodePtr->StgElem->cost = link->head->StgElem->cost;
					gnode->m_ptnodePtr->StgElem->vialink = link;
				}
			
			}
		}

	}

	node = org->org;
	floatType assignflow = POS_INF_FLOAT;
	while(node!=dest->destination)
	{
		PTLink* link  = node->StgElem->vialink;
		if (node->StgElem->cost<assignflow) assignflow = node->StgElem->cost;
		cout<<link->id<<"("<<node->StgElem->cost<<")->";
		if (!link)
		{
			cout<<link->id<<"do not have outgoing link!!"<<endl;
			system("PAUSE");
		}
		node = link->head;
	}
	cout<<endl;
	//assign the feasible flow 
	

	
	
	if (org->assDemand>0 && assignflow>0)// assign the minimum flow along the path
	{
		assignflow = min(assignflow,org->assDemand);	
	}

	if(assignflow==0.0 && org->assDemand>0) //assign the residual demand to this path
	{
		assignflow = org->assDemand;	
	}
	cout<<"assignflow:"<<assignflow<<",assDemand:"<<org->assDemand<<endl;;
	node = org->org;
	while(node!=dest->destination)
	{
		PTLink* link  = node->StgElem->vialink;
		link->volume += assignflow;
		node = link->head;
	}
	org->assDemand -= assignflow;


	return 0.0;
}










void PTNET::CheckWalkingHyperPath()
{

	PTDestination* dest;

	for (int i = 0;i<numOfPTDest; i++)
	{
		dest = PTDestVector[i];

		for (int j=0; j<dest->numOfOrg; j++)
		{
			PTOrg* org= dest->orgVector[j];

			for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();) 
			{
				TNM_HyperPath* path = *it;
				bool iswalk = true;
				for(int n=0;n<path->m_links.size();++n)
				{
					PTLink* link =  path->m_links[n]->m_linkPtr;

					if (link->GetTransitLinkType()!=PTLink::WALK)
					{
						iswalk = false;
						break;
					}
				
				}

				if (iswalk)
				{
					cout<<"warnning walking path, O-D:"<< org->org->id<<","<<dest->destination->id<<",pathname:"<<path->name<<endl;
				}

			}
		}
	}
	
}



int	 PTNET::SolveCapacityHyperpathIPF()
{
//Step 0––Initialization: 使用GP求解一次完整TEAP(no capacity constraints)达到收敛指标（RG），得到初始流量解v^0 和h^0 ，令惩罚因子初始值 γ^0=1. 
	m_startRunTime = clock();  //初始化一些参数
	curIter = 0;
	kapa = 0.8;  //惩罚因子的调整系数  //for Gentile: demand(a->d)=350 kapa=0.6
	alpha = 1;
	beta = 0.8;
	gamma = 1; //惩罚因子初始值 γ^0=1 
	gapConv = 1; //惩罚项收敛精度
	float gap=1.0; //惩罚因子收敛初始值
	MaxFlowInfeasible=0.1; //IF收敛精度
	MaxSubConv = 1e-4;  //subproblem收敛精度
	MaxSubIter = 25;    //subproblem最大循环次数  //for Gentile: demand(a->d)=350 MaxSubIter=20
	AllocateLinkBuffer(7);  //link->buffer[j]:0 travel time,1 last flow;2 pseudo capacity;3 gamma;4 waiting time
							// store the probability(m_data) for referance(shortest) [5] and non-referance hyperpath [6] pass the link

	//使用GP求解一次完整TEAP(no capacity constraints)达到收敛指标（IniRG），得到初始流量解v^0 和h^0 
	IniSolveCapacityHyperpathIPF();

	//ReportPTlinkflow(); 
	//ReportPTHyperpaths();
	(clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0);
	cout<<"============= End initializtion. Start mainLoop============="<<endl;
	cout<<"Iter= 0"<<", RGgap="<<TNM_FloatFormat(RGapIndicator,11,7)<<" ,flowInfeasible="<<TNM_FloatFormat(FlowInfeasible,7,5)<<", subIter="<<curIter<<", time="<<(clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0)<<endl;
//Step 1––Check feasibility:若流量可行（v^0满足 capacity constraints）停止迭代，得到CTAP最优解v^*= v^0 ；否则初始化link->buffer参数，更新epsilon，构造pseudo-feasible set，初始化gamma
	if (FlowInfeasible<MaxFlowInfeasible)
	{
		cout<<"No overflow. Stop Iter."<<endl;
		return 1;
	}
	IniLinkBuffer(); //初始化link->buffer参数
	cout.setf(ios::fixed,ios::floatfield);
	cout<<setprecision(7);
	CTEAPUpdateEpsilon(); //初始化epsilon0 (此时gamma0=1) 
	cout<<"Ini Epsilon="<<epsilon<<endl;
	CTEAPUpdatepseCap();  //对于行驶弧构造pseudo-feasible set
	CTEAPSetiniGamma();   //初始化gamma0 
	curIter = 0;  //令主循环迭代次数 n=1，转入Step 2.

//Step 2––MainLoop
	if(!((FlowInfeasible<=MaxFlowInfeasible) || (curIter>=maxMainIter) || (clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0)>=maxIterTime)) //判定收敛指标
	{
		do  //外循环
		{
			gap = GammaConvTest(gamma);
			cout<<"Iter="<<curIter<<", penaltyConv="<<TNM_FloatFormat(gap,7,7)<<endl;
			//2-1:使用GP-AIL求解一次完整CTEAP(no capacity constraints)，得到流量解v^n 和h^n，计算收敛指标（subGap），
			CTEAPIPFUpdateLinkgCost();//更新link travel time buffer[0], waiting time buffer[4], cost, pfdcost)
			PTLink* link;
			for(int a=0;a<numOfLink;a++) //记录上一次mainloop的link flow
			{
				link=linkVector[a];
				link->buffer[1]=link->volume;
			}
			SubIter=0;
			SubRG = 1.0;
			//sub-problem: 
			//cout<<"Start to solve sub-problem."<<endl;
			do
			{
				SubIter++;//子循环次数
				CTEAPSubLoop_IPF();  
				//2-2:更新 内罚因子的值
				CTEAPUpdateEpsilon(); //更新epsilon
			}while(SubRG>=MaxSubConv && SubIter<=MaxSubIter);
			//2-3:收敛判定，若subGap满足收敛指标，转入Step 3；否则重新循环本步骤(Step 2)
			temp_gamma=gamma;
			MainRG = SubRG;
			FlowInfeasible = IPFComputeNormFlowInfeasibility(0);

			//if(FlowInfeasible<MaxFlowInfeasible && MainRG<MaxMainConv && gammaTest<gammaConv)  maxMainIter=1;    //已经达到收敛标志
			//Step 3––Check feasibility:
			if(FlowInfeasible<MaxFlowInfeasible) //判定可行性：若流量可行，更新罚因子γ^n=κγ^(n-1) 
			{
				//if(MainRG<MaxMainConv)  
				gamma=kapa*gamma; //if flow is feasible
			}
			else  //否则（流量不可行）更新罚因子 γ^n=γ^(n-1)/κ 
				CTEAPUpdateGamma(kapa);  // CTEAPUpdateGamma(floatType ly_kapa)
			//判定当前pseudo cap和real capacity的关系
			EndUpdC = false;
			if(!EndUpdC)
			{
				bool upd=false;
				for(int a=0;a<numOfLink;a++)
				{
					PTLink* link=linkVector[a];
					if (link->GetTransitLinkType() == PTLink::ENROUTE)
					{
						if(link->buffer[2]!=link->cap) //存在 link的 pseudo capacity 不等于 real capacity, upd=true
							upd=true;
					}
				}
				if(!upd) //if upd=false, EndUpdC=true, 即 每一条link的 pseudo capacity 均等于 real capacity
					EndUpdC=true;
			}
			if(!EndUpdC) //if EndUpdC=false, 即存在 link的 pseudo capacity 不等于 real capacity, 即flow is not feasible
			{//When isfeasible is true, that is the second iteration which link flow is feasible, so pseduo capacity is equal to original capacity.
				CTEAPUpdateEpsilon();//更新epsilon
				CTEAPUpdatepseCap(); //扩容 pseudo capacity
			}
			gap = GammaConvTest(gamma); //惩罚项的收敛指标值

			RecordCapTEAPCurrentIterIPF(gap, FlowInfeasible); //记录当前迭代信息至iterRecord
			cout<<"Iter="<<curIter<<", RG="<<TNM_FloatFormat(MainRG,11,7)<<", penaltyConv="<<TNM_FloatFormat(gap,7,7)
				<<" ,flowInfeasible="<<TNM_FloatFormat(FlowInfeasible,7,5)<<", penalty="<<TNM_FloatFormat(gamma,7,5)<<", epsl="<<TNM_FloatFormat(epsilon,7,5)<<", subIter="<<SubIter
				<<", time="<<(clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0)<<endl<<endl;
				//输出：当前主循环次数，RG，Gap，flowInfeasible，gamma，epsl，其中的子循环次数
			curIter ++;
		}while (!((  (FlowInfeasible<=MaxFlowInfeasible) && (gap<=gapConv) )|| (curIter>=maxMainIter) || (clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0)>=maxIterTime));  //判定
		//while (!((  (FlowInfeasible<=MaxFlowInfeasible) && (gamma<=gammaConv) )|| (curIter>=maxMainIter) || (clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0)>=maxIterTime));  //判定
		
	}

	cout<<"\n\n\tSolution process terminated"<<endl;
	cout<<"CPU Time="<<1.0*(clock() - m_startRunTime)/(CLOCKS_PER_SEC*60);
	cout<<"\tThe number of iteration is "<<curIter<<endl;

	
	//输出所有弧流量、容量、乘子、费用
	//for(int i = 0;i<numOfLink;i++)
	//{
	//	PTLink* link = linkVector[i];
	//	//if(link->GetTransitLinkTypeName()=="ENROUTE")	
	//	//{cout<<"id:"<<link->id<<"("<<link->GetTransitLinkTypeName()<<"), cap:"<<TNM_FloatFormat(link->cap,7,2)<<", volume:"<<TNM_FloatFormat(link->volume,6,2)<<", mu:"<<TNM_FloatFormat(link->mu,6,2)<<", cost:"<<TNM_FloatFormat(link->cost,6,2)<<",  gcost:"<<TNM_FloatFormat(link->g_cost,7,2)<<endl;}
	//	cout<<"id:"<<link->id<<"("<<link->GetTransitLinkTypeName()<<"), cap:"<<TNM_FloatFormat(link->cap,7,2)<<", volume:"<<TNM_FloatFormat(link->volume,6,2)<<", mu:"<<TNM_FloatFormat(link->buffer[4],6,2)<<", cost:"<<TNM_FloatFormat(link->cost,6,2)<<",  gcost:"<<TNM_FloatFormat(link->g_cost,7,2)<<endl;
	//}
	ReportWalkflowRation();
	
	
	cout<<endl;
	


	return 0;
}


void PTNET::IniSolveCapacityHyperpathIPF()
{
	cout<<"============= Start to Initialize CTEAP-IPF============="<<endl;
	int IniIter=0;    //内循环参数
	float MaxIniRG=1e-4; //内循环RG（收敛精度） //for Gentile  MaxIniRG=1e-7
	float MaxIniGap=1e-5;
	UpatePTNetworkLinkCost();
	InitialHyperpathNetFlow();
	UpatePTNetworkLinkCost();
	float IniGap=1e7;
	float IniRG=1.0;
	convCriterion = MaxIniRG;
	//cout<<"iter:"<<IniIter<<",\tcurrent gap:"<<TNM_FloatFormat(IniGap,5,15)<<", relative gap:"<<TNM_FloatFormat(IniRG,5,15)<<
		//", hyperpath num:"<<numaOfHyperpath<<",inneriters:"<<InnerIters<<endl<<endl;
	IniIter=curIter;
	IniGap=GapIndicator;
	IniRG=RGapIndicator;
	//计算IF
	float norm = 0.0;
	satuationLink=0;
	numOfTransitLink=0;
	for(int i=0;i<numOfLink;i++)
	{   
		PTLink* link = linkVector[i];
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		{ 
			numOfTransitLink++;
			if (link->volume>=link->cap)
			{
				norm+=pow((link->volume-link->cap),2);
				satuationLink++;
				//cout<<"id:"<<link->id<<",flow:"<<link->volume<<",cap:"<<link->cap<<endl;
			}
		}
	}
	FlowInfeasible = sqrt(norm);
	cout<<"============= End Initialize CTEAP-IPF============="<<endl;
}

void PTNET::IniLinkBuffer()	
{
	for(int ini=0;ini<numOfLink;ini++)  //初始化link->buffer参数
	{
		PTLink* link = linkVector[ini];
		link->buffer[0]=link->cost;  //travel time
		link->buffer[1]=link->volume;  //last flow
		link->buffer[2]=link->cap; //pseudo capacity
		if(link->GetTransitLinkTypeName()=="ENROUTE")	link->buffer[3]=1; //初始化行驶弧gamma0=1
		else    link->buffer[3]=0;
		link->buffer[4]=0; //waiting time
		//cout<<"id:"<<link->id<<"("<<link->GetTransitLinkTypeName()<<"), cap:"<<TNM_FloatFormat(link->cap,7,2)  <<",flow="<<TNM_FloatFormat(link->volume,7,2)<<",mu="<<TNM_FloatFormat(link->buffer[3],7,2)<<endl;
	}
}

void PTNET::CTEAPUpdateGamma(floatType ly_kapa)
{
	float gamma2;
	floatType ugnorm1=IPFComputeNormFlowInfeasibility(0); //current iter infeasibleflow
	floatType ugnorm0=IPFComputeNormFlowInfeasibility(1); //last iter infeasibleflow
	if(ugnorm1>=beta*ugnorm0)
		gamma=gamma/ly_kapa;//fomulate(63) ly_gamma=ly_gamma*ly_kapa; at ConvTestIPF()
	if(gamma>1e14)
		gamma=1e10;
}



void PTNET::CTEAPSubLoop_IPF()
{
	PTDestination* dest;
	for (int i = 0;i<numOfPTDest; i++)
	{
		dest = PTDestVector[i];
		if (!IPFInitializeGeneralHyperpathLS(dest->destination)) //找最短路树IPF
		{
			for (int j=0; j<dest->numOfOrg; j++)
			{													
				PTOrg* org= dest->orgVector[j];
				ColumnGeneration(dest,org) ;
				org->CTEAPIPFUpdatePathSetCost(gamma);
				//org->UpdatePathSetCost(); //这里需要改？
				if (org->pathSet.size()>1)
				{
					if (PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iGreedy || PCTAE_ALG == PCTAE_algorithm::PCTAE_P_Greedy)	UpdateHyperPathGreedyFlow(dest,org);	
					else if (PCTAE_ALG == PCTAE_algorithm::PCTAE_P_NGP || PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iNGP || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL)	
						CTEAPIPFUpdateHyperpathGPFlow(dest,org);
					//else if (PCTAE_ALG == PCTAE_algorithm::PCTAE_P_NGP || PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iNGP || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL)	UpdateHyperpathGPFlow(dest,org);
				}
			}

		}
		
	}
	IPFHyperpathCapInnerLoop(200);
	//HyperpathCapInnerLoop(200);

	//ComputeConvGap();//得到当前RGapIndicator
	IPFComputeCapSubConvGap();
	//ComputeCapSubConvGap();
	//RecordTEAPCurrentIter();

	cout<<"  subiter:"<<SubIter<<", relative gap:"<<TNM_FloatFormat(SubRG,5,10)<<",inneriters:"<<InnerIters<<",time:"<< (clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0)<<endl;
		//输出：当前的子循环次数，子循环RG，当中的内循环次数inneriters
}

void PTNET::CTEAPIPFUpdateHyperpathGPFlow(PTDestination* dest,PTOrg* org)
{
	TNM_HyperPath* MinCostpath=org->pathSet[org->minIx];
	vector<GLINK*> mglinks=MinCostpath->GetGlinks(); //MinCostpath的各条路段 mglinks
	PTLink *link,*rlink;

	for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();it++) 
	{
		TNM_HyperPath* path = *it;
		path->Preflow = path->flow;
		MinCostpath->Preflow = MinCostpath->flow;
		if (path != MinCostpath)
		{
			path->CTEAPIPFUpdateHyperpathCost(gamma);
			MinCostpath->CTEAPIPFUpdateHyperpathCost(gamma);
			//cout<<"before update, pathcost:"<<path->cost<<",minpathcost:"<<MinCostpath->cost<<",gap:"<<fabs(path->cost-MinCostpath->cost)<<endl;
			//cout<<"               pathfdcost = "<<path->fdcost<<",minpathfdcost:"<<MinCostpath->fdcost<<endl;
			vector<GLINK*> glinks= path->GetGlinks();  //path的各条路段 glinks
			floatType DerSum = 0.0;	
			//shift flow from path -> minpath
			for(int i=0;i<mglinks.size();i++) //遍历MinCostpath的各条路段 mglinks[i]
			{
				link = mglinks[i]->m_linkPtr;
				link->buffer[5] = mglinks[i]->m_data; //buffer[5]为路段经过参考路径（最短路）的概率m_data(sp)
				link->markStatus = 1;// tag 1 for arcs on the shortest path
			}
			for(int i=0;i<glinks.size();i++) //遍历path的各条路段 glinks[i]
			{
				glinks[i]->m_linkPtr->buffer[6] = glinks[i]->m_data; //buffer[6]为路段经过非参考路径的概率m_data(p)
			}
			// calculate derivative for shortest links	
			for(int i=0;i<mglinks.size();i++)  //遍历参考路径（最短路）MinCostpath的各条路段 mglinks[i]
			{   
				link = mglinks[i]->m_linkPtr; 
				double tmp = link->pfdcost * (link->buffer[6] - link->buffer[5]); 
				        //路段traveltime一阶导[dt(i,j)/dv(i,j)] * 路段经过非参考路径和参考路径的概率差[m_data(p) - m_data(sp)]
				if ( link->GetTransitLinkType() == PTLink::ENROUTE )
				{
					tmp  += gamma/pow((link->buffer[2]-link->volume), 2) * (link->buffer[6] - link->buffer[5]);	 
					    //路段waiting time一阶导[dw(i,j)/dv(i,j)] * 该路段经过非参考路径和参考路径的概率[m_data(p) - m_data(sp)]
				}
				if (link->rLink && link->GetTransitLinkSymmetry() == PTLink::ASYMMTRIC)
				{
					tmp += link->rpfdcost * (link->rLink->buffer[6] - link->rLink->buffer[5]); 
						//相互作用的路段traveltime一阶导[dt(i,j)/dv(i',j')] * 该路段经过非参考路径和参考路径的概率差[m_data[i',j'](p) - m_data[i',j'](sp)]
				}
				DerSum += tmp *(link->buffer[6] - link->buffer[5]); 
						// 最后，乘以该路段经过非参考路径和参考路径的概率差[m_data(p) - m_data(sp)]
			}
			// calculate derivative for non-shortest links
			for(int i=0;i<glinks.size();i++)
			{
				link = glinks[i]->m_linkPtr;  //遍历对于仅经过非参考超路径而不经过参考超路径的路段
				if (link->markStatus != 1) // arcs on the non-shortest path but not on the shortest path 
				{
					double tmp = link->pfdcost * (link->buffer[6] - link->buffer[5]);
					if (  link->GetTransitLinkType() == PTLink::ENROUTE )
					{
						tmp  += gamma/pow(link->buffer[2]-link->volume, 2) * (link->buffer[6] - link->buffer[5]);
					}
					if (link->rLink  && link->GetTransitLinkSymmetry() == PTLink::ASYMMTRIC)
					{
						tmp += link->rpfdcost * (link->rLink->buffer[6] - link->rLink->buffer[5]);
						//cout<<"rpfdcost="<<link->rpfdcost<<","<<link->rLink->buffer[0]<<","<<link->rLink->buffer[1]<<endl;
					}
					DerSum += tmp *(link->buffer[6] - link->buffer[5]);
					//gk=DerSum 这里表示经过非参考和参考超路径的所有路段（不重复计算）
				}
			}


			if (DerSum ==0)	DerSum = 1e-8;//no need to shit flow from current path to the shortest path 
			double dev = path->cost - MinCostpath->cost; //~Ck
			//cout<<"path cost="<<path->cost<<" MinCostpath cost="<<MinCostpath->cost<<endl;
			double dflow;
			if (DerSum>=0) //(gk>=0)
			{
				if (dev>=0)  //如果当前non-reference path cost > reference path cost，即需要把流量从non-reference path转移至reference path
				{
					dflow =  __min(1.0 * dev /DerSum, path->flow);			//转移  min{ 计算的转移量(+), 当前non-reference path的流量(+)}	
					//cout<<"detaC="<<dev<<" g="<<DerSum<<" CalTransFlow="<<1.0 * dev /DerSum<<", pathflow="<< path->flow<<",dflow"<<dflow<<endl;
				}
				else		 //如果当前non-reference path cost <= reference path cost，即需要把流量从reference path转移至non-reference path
				{
					dflow =  __max(1.0 * dev /DerSum, -MinCostpath->flow);	 //转移  max{ 计算的转移量(-), 当前reference path的流量的负值(-)}
					//cout<<1.0 * dev /DerSum<<","<< MinCostpath->flow<<",dflow"<<dflow<<endl;
					//cout<<"detaC="<<dev<<" g="<<DerSum<<" CalTransFlow="<<1.0 * dev /DerSum<<", minflow="<< MinCostpath->flow<<",dflow"<<dflow<<endl;
				}
				//cout<<"org:"<<org->org->id<<",dest:"<<dest->destination->id<<",dflow:"<<dflow<<",DerSum:"<<DerSum<<",pathcost:"<<path->cost<<",minpathcost:"<<MinCostpath->cost<<endl;
				MinCostpath->flow = MinCostpath->flow + dflow; 	
				path->flow = path->flow - dflow;
			}
			else  //(gk<0)
			{
				if (dev<0)  //如果当前non-reference path cost <= reference path cost，即需要把流量从reference path转移至non-reference path
				{
					dflow =  __min(1.0 * dev /DerSum, MinCostpath->flow);   //转移  min{计算的转移量(+), 当前non-reference path的流量(+)}
					cout<<"detaC="<<dev<<" g="<<DerSum<<" CalTransFlow="<<1.0 * dev /DerSum<<", pathflow="<< path->flow<<",dflow"<<dflow<<endl;
				
				}
				else		//如果当前non-reference path cost >= reference path cost，即需要把流量从non-reference path转移至reference path
				{
					dflow =  __max(1.0 * dev /DerSum, path->flow);  //转移量(-)  max{ 计算的转移量(-), 当前reference path的流量的负值(-)}
					cout<<"detaC="<<dev<<" g="<<DerSum<<" CalTransFlow="<<1.0 * dev /DerSum<<", minflow="<< MinCostpath->flow<<",dflow"<<dflow<<endl;
				}
				cout<<"org:"<<org->org->id<<",dest:"<<dest->destination->id<<",dflow:"<<dflow<<",DerSum:"<<DerSum<<",pathcost:"<<path->cost<<",minpathcost:"<<MinCostpath->cost<<endl;
				MinCostpath->flow = MinCostpath->flow - dflow; 	
				path->flow = path->flow + dflow;
			}


			

			if (path->flow<0||MinCostpath->flow<0)
			{
				cout<<"path flow:"<<path->flow<<",minpath flow:"<<MinCostpath->flow<<",dev:"<<dev<<",DerSum:"<<DerSum<<",dflow:"<<dflow<<endl;
				system("PAUSE");
			}
			for(int lj=0;lj<glinks.size();lj++)
			{
				link = glinks[lj]->m_linkPtr;
				link ->volume += glinks[lj]->m_data* (path->flow - path->Preflow);
				if (abs(link->volume) < 1e-8) link ->volume = 0.0;
				link->IPFUpdatePTLinkCost(gamma); //更新link cost fdcost in CTEAP-IPF
				if (link ->rLink)
				{
					link ->rLink ->IPFUpdatePTLinkCost(gamma); //更新rlink cost fdcost in CTEAP-IPF （current link flow variation influnce the related link cost and derivative cost）
				}
						
				link->buffer[6]=0.0;// reset the link prob
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
				link->buffer[5]=0.0;// reset the link prob
				link->markStatus =0;
				link->IPFUpdatePTLinkCost(gamma); //更新link cost fdcost in CTEAP-IPF
				if (link ->rLink&& link->GetTransitLinkSymmetry()==PTLink::ASYMMTRIC)
				{
					link ->rLink ->IPFUpdatePTLinkCost(gamma); //更新rlink cost fdcost in CTEAP-IPF （current link flow variation influnce the related link cost and derivative cost）
				}
			}
			path->CTEAPIPFUpdateHyperpathCost(gamma);
			MinCostpath->CTEAPIPFUpdateHyperpathCost(gamma);
			//cout<<"after update, pathcost:"<<path->cost<<",minpathcost:"<<MinCostpath->cost<<",gap:"<<fabs(path->cost-MinCostpath->cost)<<endl;
			//cout<<"              pathfdcost = "<<path->fdcost<<",minpathfdcost:"<<MinCostpath->fdcost<<endl;
		}
	}
}


void PTOrg::CTEAPIPFUpdatePathSetCost(floatType gamma)
{
	double maxCost= - 1.0 , minCost=POS_INF_FLOAT;
	currentTotalCost = 0.0;

	for (int i =0; i < pathSet.size();i++)
	{		
		TNM_HyperPath* path= pathSet[i]; //遍历该起点org的超路径集合中的超路径path
		//compute hyperpath cost
		path->CTEAPIPFUpdateHyperpathCost(gamma);    //更新超路径path费用和费用一阶导
		//path->UpdateHyperpathCost();  
		if (path->cost> maxCost)	maxCost =   path->cost; //求当前超路径集合中超路径的maxCost、minCost和对应索引minIx

		if (path->cost< minCost)    
		{
			minCost = path->cost; 
			minIx = i; 
		}
		currentTotalCost += path->cost * path->flow; //计算当前超路径集中超路径的费用之和currentTotalCost
	}

	maxPathGap = maxCost - minCost;
	currentRelativeGap = fabs( 1 - minCost * assDemand / currentTotalCost);
}

void PTNET::CTEAPIPFUpdateLinkgCost() //更新link cost fdcost in CTEAP-IPF
{
	PTLink *link;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		link->IPFUpdatePTLinkCost(gamma); //IPF中计算link cost, pfdcost
		//link->IPFUpdatePTDerLinkCost();	
	}
	/*
	for(int i = 0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		//if(link->GetTransitLinkTypeName()=="ENROUTE")	
		//{cout<<"id:"<<link->id<<"("<<link->GetTransitLinkTypeName()<<"), cap:"<<TNM_FloatFormat(link->cap,7,2)<<", volume:"<<TNM_FloatFormat(link->volume,6,2)<<", mu:"<<TNM_FloatFormat(link->mu,6,2)<<", cost:"<<TNM_FloatFormat(link->cost,6,2)<<",  gcost:"<<TNM_FloatFormat(link->g_cost,7,2)<<endl;}

		cout<<"id:"<<link->id<<"("<<link->GetTransitLinkTypeName()<<"), cap:"<<TNM_FloatFormat(link->cap,7,2)<<", volume:"<<TNM_FloatFormat(link->volume,6,2)
			<<", pseduocap:"<<link->buffer[2]<<", traveltime:"<<link->buffer[0]<<", waitingtime:"<<link->buffer[4]
			<<", cost:"<<TNM_FloatFormat(link->cost,6,2)<<",  fdcost:"<<TNM_FloatFormat(link->pfdcost,7,2)<<endl;
	}
	*/
}



void PTLink::IPFUpdatePTLinkCost(floatType netgamma) //更新link cost fdcost in CTEAP-IPF
{
	if (volume<0&&volume>-1e-10) volume = 0.0;
	if (volume<-1e-10)
	{
		cout<<"link id :"<<id<<" have negative flow:"<<volume<<endl;
	}
	switch(GetTransitLinkType())
	{
	case PTLink::FAILWALK: //add for failing walk link
		double alpha_11,wt1;
		alpha_11=pars[0];
		wt1=	pars[1];
		buffer[0] = alpha_11 * wt1; //travel time 
		cost = buffer[0]; 
		pfdcost = 0.0;
		break;
	case PTLink::WALK:
		double alpha_1,wt;
		alpha_1=pars[0];
		wt=	pars[1];
		buffer[0] = alpha_1 * wt;
		cost = buffer[0];  
		pfdcost = 0.0;
		break;
	case PTLink::ABOARD:
		double alpha_2,beta_2,n1,cap1;
		alpha_2	=	pars[0];
		beta_2	=	pars[1];
		n1		=	pars[2];//power
		cap1	=	buffer[2]; //这里是对应行驶弧的pseudo cap？还是上车弧自身的real capacity？		
		if(m_tlSym==PTLink::ASYMMTRIC) // link cost function is asymmric
		{
			if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;
			buffer[0] = alpha_2 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1);
			cost = buffer[0] ;
			pfdcost = alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1 - 1) * ( (1 - beta_2) / cap1);
			rpfdcost= alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1 - 1) * (beta_2 / cap1);
		}
		
		break;
	case PTLink::ALIGHT:
		double alpha_4,tloss;
		alpha_4=	pars[0];
		tloss =		pars[1];
		buffer[0] = alpha_4 * tloss;
		cost = buffer[0] ;
		pfdcost = 0.0;
		break;
	case PTLink::ENROUTE:
		double alpha_3,beta_3,gamma_3,n2,cap2,fft;
		alpha_3	=	pars[0];
		beta_3	=	pars[1];
		gamma_3	=	pars[2];
		n2		=	pars[3];//power
		cap2	=	buffer[2]; //pseudo cap
		fft		=	pars[5];
		
		if(m_tlSym==PTLink::ASYMMTRIC)
		{
			if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;	
			buffer[0] = alpha_3 * fft + beta_3 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2); //travel time
			buffer[4] = netgamma/(cap2-volume);  //waiting time
			cost = buffer[0]+buffer[4];
			pfdcost =  beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2 - 1) * (1 / cap2); //+ netgamma/pow(cap2-volume,2);
			rpfdcost = beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2 - 1) * ((gamma_3 - 1) / cap2);
		}
		
		break;
	}
	if (fabs(cost)<1e-10) cost = 0.0;


}

void PTLink::IPFUpdatePTDerLinkCost()
{
	switch(GetTransitLinkType())
	{
	case PTLink::FAILWALK:
		pfdcost = 0.0;
		break;
	case PTLink::WALK:
		pfdcost = 0.0;
		//rpfdcost = 0.0;
		break;
	case PTLink::ABOARD: //link=上车弧(i1,j1) rlink=节点j1出发行驶弧(j1,j2)
		double alpha_2,beta_2,n1,cap1;
		alpha_2	=	pars[0];
		beta_2	=	pars[1];
		n1		=	pars[2];//power
		cap1	=	rLink->buffer[2]; //pseudo cap
		if(m_tlSym==PTLink::ASYMMTRIC)
		{
			pfdcost = alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1 - 1) * ( (1 - beta_2) / cap1); //上车弧cost_func对上车弧link流量v(i1,j1)一阶导
			rpfdcost= alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1 - 1) * (beta_2 / cap1);        //上车弧cost_func对下一段行驶弧rlink流量v(j1,j2)一阶导
		}
		else
		{
			pfdcost =  alpha_2 * n1 * pow( volume / cap1 , n1-1) * (1/cap1);
			rpfdcost = 0.0;
		}
		break;
	case PTLink::ALIGHT:
		pfdcost = 0.0;
		//rpfdcost = 0.0;
		break;
	case PTLink::ENROUTE: //link=行驶弧(j1,j2) rlink=节点j1上车弧(i1,j1)
		double alpha_3,beta_3,gamma_3,n2,cap2,fft;
		alpha_3	=	pars[0];
		beta_3	=	pars[1];
		gamma_3	=	pars[2];
		n2		=	pars[3];//power
		cap2	=	buffer[2]; //pseudo cap
		fft		=	pars[5];
		if(m_tlSym==PTLink::ASYMMTRIC)
		{
			pfdcost  = beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2 - 1) * (1 / cap2);   //行驶弧cost_func对行驶弧link流量v(j1,j2)一阶导
			rpfdcost = beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2 - 1) * ((gamma_3 - 1) / cap2); //行驶弧cost_func对节点j1上车弧rlink流量v(i1,j1)一阶导
		}
		else
		{
			pfdcost  =  beta_3 * n2 * pow(volume/ cap2,n2-1)*(1/cap2);
			rpfdcost = 0.0;
		}
		break;
	}
}


void PTNET::CTEAPSetiniGamma()
{
	float inigamma=0.0;

	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
	
		if (link->GetTransitLinkType() == PTLink::ENROUTE &&link->GetTransitLinkCapType()==PTLink::TL)
		{
			inigamma += log(link->cap/(link->buffer[2] - link->volume));
		}
	}
	gamma = inigamma;
	//gamma=1;
	cout<<"initial gamma="<<gamma<<endl;
}

void PTNET::CTEAPUpdatepseCap() //仅对行驶弧构建pseudo capacity，其他弧的buffer[2]（pseudo capacity）= link->cap（Real capacity）
{
	int bigchange=0;
	for(int upci=0;upci<numOfLink;upci++)
	{
		PTLink* link=linkVector[upci];
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		{
			if(link->volume>=link->cap) //达到饱和
			{	
				if((link->buffer[2]-link->volume)>1) bigchange++;
				link->buffer[2]=link->volume+epsilon; //pseudo capacity
				//cout<<"link "<<link->id<<"("<<link->GetTransitLinkTypeName()<<")"<<" is saturated.";
				//cout<<"Flow:"<<link->volume<<" Real capacity:"<<link->cap<<" Epsilon:"<<epsilon<<" Pseudo capacity:"<<link->buffer[2]<<endl;
			}
			else
				link->buffer[2]=link->cap;
		}
		//else link->buffer[2]=link->cap;
		
	}
	//cout<<"Update capacity have "<<bigchange<<" big change."<<endl;
}

floatType PTNET::GammaConvTest(floatType gamma)
{
	//这里计算网络所有行驶弧
	floatType gap = 0.0;	
	//netTTcost = netTTwaitcost;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
	
		if (link->GetTransitLinkType() == PTLink::ENROUTE &&link->GetTransitLinkCapType()==PTLink::TL)
		{
			//gap += pow(max(long double(0),-cb_lambda*(link->cap - link->volume)),2);
			gap += gamma*log(link->buffer[2]/(link->buffer[2]-link->volume));

			//gap += gamma*(1/(link->buffer[2]-link->volume));
		}
	}

	//return  1.0/(2.0*cb_lambda)*gap;
	cout<<"惩罚项：gapCon= "<<gap<<" ttCost="<<netTTcost<<" gapRG="<<gap/netTTcost<<endl;
	//return gap;
	return gap;///netTTcost;


	/* //这里是计算所有非最短路的行驶弧
	floatType gap=0.0;
	PTLink *link;
	PTDestination* dest;
	for (int i = 0;i<numOfPTDest;i++)
	{
		dest = PTDestVector[i];
		InitializeGeneralHyperpathLS(dest->destination);
		for (int j=0; j<dest->numOfOrg; j++)
		{
			PTOrg* org= dest->orgVector[j];
			//double mincost = org->pathSet[org->minIx]->cost;
			TNM_HyperPath* MinCostpath=org->pathSet[org->minIx];
			vector<GLINK*> mglinks=MinCostpath->GetGlinks();
			PTLink *link,*rlink;
			for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();it++) 
			{
				TNM_HyperPath* path = *it;
				if (path != MinCostpath) //非最短路
				{
					vector<GLINK*> glinks= path->GetGlinks();
					for(int i=0;i<glinks.size();i++)
					{
						if(glinks[i]->m_linkPtr->GetTransitLinkType()==PTLink::ENROUTE)
							//a += gamma*(glinks[i]->m_linkPtr->buffer[2]/(glinks[i]->m_linkPtr->buffer[2]-glinks[i]->m_linkPtr->volume)); //这里是导数
							gap += gamma*log(glinks[i]->m_linkPtr->buffer[2]/(glinks[i]->m_linkPtr->buffer[2]-glinks[i]->m_linkPtr->volume)); //这里是原函数
							//a += gamma*log(glinks[i]->m_linkPtr->cap/(glinks[i]->m_linkPtr->cap-glinks[i]->m_linkPtr->volume));
					}
				}
			}
		}
	}
	//cout<<"gap= "<<a<<endl;
	return gap;
	*/

}

void PTNET::CTEAPUpdateEpsilon()
{
	floatType uegcost=0.0,uegcost1;
	for(int uei=0;uei<numOfLink;uei++)
	{
		PTLink* link=linkVector[uei];
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		//uegcost1=link->cost+ly_gamma/(link->buffer[2]-link->volume);
		{
			if(uegcost<link->cost) uegcost=link->cost; //max linkcost
		}
	}
	epsilon=gamma/(alpha*uegcost); //gamma
	//cout<<"gamma="<<gamma<<", alpha="<<alpha<<", max linkcost="<<uegcost<<", epsilon="<<epsilon<<endl; //输出当前epsilon的值
	if(epsilon<1e-7)
		epsilon=1e-7;
}

int PTNET::PTAllOrNothingCAP(PTDestination* dest)
{
	PTOrg* org;
	//InitializeHyperpathLS(dest->destination); 
	InitializeGeneralHyperpathLS(dest->destination); //根据generalized cost找最短路
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
	//CoutLinkInoCAP();
	
	return 0;
}




int	 PTNET::SolveCapacityHyperpath()
{

	UpatePTNetworkLinkCost();
	if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL ||
		PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GFW_TL)			
		InitialHyperpathNetFlow();
	UpatePTNetworkLinkCost();
	//cout<<"iter:"<<0<<",\t current gap:"<<TNM_FloatFormat(GapIndicator,5,15)<<", relative gap:"<<TNM_FloatFormat(RGapIndicator,5,15)<<endl<<endl;
	if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GFW_TL)		
		IniTransitLinkMultiplier();
	floatType lastSubRG;
	IniCapacityPars(); 
	//MaxSubIter = 50; //for shenzhannet 20
	cb_lambda = 0.005; //initial lambda 
	if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL)		
		AllocateLinkBuffer(2);   //store prob for longest and shortest hyperpath pass the link
	if ( PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GFW_TL)		
	{
		AllocateLinkBuffer(1);  //store the link volume
		AllocateNetBuffer(1);   //store the netTTwaitcost
		ComputeNetWait();
		SaveLinkSolution(1);  //save current link volume and netTTwaitcost
		//cout<<"nownetTTwait="<<netTTwaitcost<<"  buffer[0]="<<buffer[0]<<endl;
	}
	UpdatePTNetGeneralCost_Dercost();
	m_startRunTime = clock();
	curIter = 0;
	infeasibleflow = ComputeNormFlowInfeasibility(false);
	SetCurrentSolution();
	cout<<"curIter:"<<0<<",initial infeasibleflow:"<<infeasibleflow<<",initial lambda:"<<cb_lambda<<endl<<endl;
	MaxFlowInfeasibility =  1e-7 ; //for shenzhannet  0.2 gentile/sf 1e-7  winniebeg 0.05
	//CoutLinkInoCAP();
	if(infeasibleflow>MaxFlowInfeasibility && !ReachMaxTime((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0)) && !ReachMaxIter())
	{
		floatType gap = ComputeBoundGAP();
		cout<<"IniGap="<<gap<<endl;
		do 
		{
			double M = 1e4; //Shenzhen 1e4
			SubRGCriterion = 1e-5; //Shenzhen 1e-4    gentile and sf 1e-7   winniebeg 1e-5
			if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL||PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL)
				CapacityHyperpathSubProblem();
			else if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GFW_TL) //link-based solver for suproblem
				CapacityLinkSubProblem();
			gap = ComputeBoundGAP();

			floatType IF = ComputeNormFlowInfeasibility(false);
			floatType sn = ComputeNormSolution(); //solution norm
			SetCurrentSolution();
			curIter++;
			RecordCapTEAPCurrentIter(gap, IF,sn);

			if (curIter>1)
			{
				if (infeasibleflow/SubRG > M) //subproblem is good enough
				{
				if (IF > cb_sigma*infeasibleflow)
					{
						cb_lambda *= 3; 
					}
				}
				else
				{
					cb_lambda /= 3;			
				}
			}
			UpdateMultiplier();
			infeasibleflow = IF;
			lastSubRG = SubRG;
			UpdatePTNetGeneralCost_Dercost();
			//CoutLinkInoCAP();
			cout<<"Main problem: curIter:"<<curIter<<",infeasibleflow:"<<infeasibleflow<<",lambda:"<<cb_lambda<<",gap:"<<gap<<",solution-norm:"<<sn
				<<", time="<<TNM_FloatFormat((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0),5,15)<<endl<<endl;
			//ReportAvgpaxinfo();
		}while ( infeasibleflow>MaxFlowInfeasibility && !ReachMaxTime((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC*60.0)) && !ReachMaxIter()); //
	}
	///* 
	//输出行驶弧/上车弧流量、容量、乘子、费用
	for(int i = 0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		if (link->GetTransitLinkType() == PTLink::ENROUTE &&link->GetTransitLinkCapType()==PTLink::TL )
		{
			cout<<"id:"<<link->id<<"("<<link->tail->GetStopPtr()->m_id<<","<<link->head->GetStopPtr()->m_id<<")"<<"cap:"<<link->cap<<",volume:"<<link->volume<<",mu:"<<link->mu<<",cost:"<<link->cost<<",gcost:"<<link->g_cost<<endl<<endl;
		}
	}
	//*/
	//CoutLinkInoCAP();
	ReportWalkflowRation();
	cout<<"netwait:"<<netTTwaitcost<<endl;
	ReportPTHyperpaths();
	return 0;
}



floatType PTNET::ComputePTDzCAP()
{
	floatType dz = 0, 
		diff, tmpVol,rtemVol,rdiff;
	PTLink* plink;
	//cout<<" cal: dz="<<dz;
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
			//dz     += diff* plink->GetPTLinkCost();
			plink->UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
			dz     += diff* plink->g_cost;
			plink->volume = tmpVol;
			plink->rLink->volume = rtemVol;
					//cout<<" 1 plink->volume="<<tmpVol<<"  rLink->volume="<<rtemVol<<endl;
					//cout<<" 2 diff="<<diff<<"  plink->g_cost="<<plink->g_cost<<"  multi="<<diff* plink->g_cost<<endl;
					//cout<<" 3 dznow="<<dz<<endl;
			//cout<<"+"<<diff* plink->g_cost;
		}
		else
		{
			diff    = plink->volume - plink->buffer[0];
			tmpVol  = plink->volume;
			plink->volume = plink->buffer[0] + diff*stepSize;
			//dz  += diff * plink->GetPTLinkCost();
			plink->UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
			dz     += diff* plink->g_cost;
			plink->volume = tmpVol;	
					//cout<<" 1 plink->volume="<<tmpVol<<endl;
					//cout<<" 2 diff="<<diff<<"  plink->g_cost="<<plink->g_cost<<"  multi="<<diff* plink->g_cost<<endl;
					//cout<<" 3 dznow="<<dz<<endl;
			//cout<<"+"<<diff* plink->g_cost;
		}
	}
	//cout<<"+ (devwait)"<<netTTwaitcost - buffer[0];
	dz += netTTwaitcost - buffer[0];
	//cout<<"="<<dz<<endl;

	return dz;

}



int PTNET::BisecSearchCAP(floatType maxStep)
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
		//dz = ComputePTDz();
		dz = ComputePTDzCAP(); //link->UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
		//cout<<"lineiter:"<<iter<<",dz:"<<TNM_FloatFormat(dz,10, 8)<<",(b-a):"<<TNM_FloatFormat(b-a,10,8)<<" step size = "<<stepSize<<endl;
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


int PTNET::PTAllOrNothingCAP()
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
		//if (PTAllOrNothing(PTDestVector[i])!=0) return 1;
		if (PTAllOrNothingCAP(PTDestVector[i])!=0) return 1; //if not correctly, return 1;
	
	}
	//cout<<"end all-or nothong, auxilury solution: netTTwaitcost="<<netTTwaitcost<<endl;
	return 0;
}


void PTNET::CapacityLinkSubProblem()
{
	SubIter = 0;
	SubRG = 1.0;
	cout<<"Start to solve sub-problem."<<endl;
	do
	{
		SubIter++;
		clock_t sclock = clock();
		SaveLinkWaitVariables(); //令link->buffer[0]=当前volume；网络整体buffer[0] = netTTwaitcost，即当前解x
		PTAllOrNothingCAP();    //all-or-nothing assignment, get netTTwaitcost，即辅助解y
		//CoutLinkInoCAP();
		
		if (PCTAE_ALG == PCTAE_algorithm::PCTAE_A_GFW)	BisecSearch();  //line search
		else if (PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_GFW_TL)  BisecSearchCAP(); 
		//cout<<"Cal: stepSize="<<stepSize<<", netTTwaitcost(x)="<<buffer[0]<<", netTTwaitcost(y)="<<netTTwaitcost<<endl;
		NewSolution(1);   //update solution
		//cout<<"Update: new-netTTwaitcost="<<netTTwaitcost<<endl; //[x+(y-x)*stepSize]
		UpdatePTNetGeneralCost_Dercost();
		IterMainlooptime = 1.0 * (clock() - sclock)/CLOCKS_PER_SEC;
		ComputeCapSubConvGap();
		cout<<"Sub-problem: subiter:"<<SubIter<<",\t relative gap:"<<TNM_FloatFormat(SubRG,5,15)
			<<", time:"<<TNM_FloatFormat((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC),5,15)<<endl;
	}while((SubRG>SubRGCriterion) && (SubIter<MaxSubIter));
	
}





void PTNET::CapacityHyperpathSubProblem()
{
	SubIter = 0;
	SubRG = 1.0;
	cout<<"Start to solve sub-problem."<<endl;
	do
	{

		SubIter++;

		PTDestination* dest;
		for (int i = 0;i<numOfPTDest; i++)
		{
			dest = PTDestVector[i];
			if (!InitializeGeneralHyperpathLS(dest->destination))
			{
				for (int j=0; j<dest->numOfOrg; j++)
				{													
					PTOrg* org= dest->orgVector[j];
					ColumnGeneration(dest,org) ;
					org->TL_UpdateCapPathSetCost(cb_lambda);
					if (org->pathSet.size()>1)
					{
						if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL)		
							UpdateCapHyperPathGreedyFlow(dest,org);	
						else if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL)			
							UpdateCapHyperpathGPFlow(dest,org);
					} 					
				}
			}
		}
		HyperpathCapInnerLoop(200); //shenzhen 100
		ComputeCapSubConvGap();
		cout<<"Sub-problem: subiter:"<<SubIter<<",\t relative gap:"<<TNM_FloatFormat(SubRG,5,15)
			<<", time:"<<TNM_FloatFormat((clock() - m_startRunTime)*1.0 / (CLOCKS_PER_SEC),5,15)<<endl;
	}while((SubRG>SubRGCriterion) && (SubIter<MaxSubIter) );
	InnerIters = SubIter;
}


void PTNET::IniCapacityHyperpathProblem() 
{
	int Iter = 0;
	float RG = 1.0;
	int MaxIter = 1000;
	float RGCriterion = 1e-7;
	cout<<"Start to initial CTEAP problem."<<endl;
	do
	{
		SubIter++;

		PTDestination* dest;

		for (int i = 0;i<numOfPTDest; i++)
		{
			dest = PTDestVector[i];
		}

	}while((RG>RGCriterion) && (Iter<MaxIter) );
	int iniIters = Iter;
}

void	PTNET::UpdateCapHyperPathGreedyFlow(PTDestination* dest,PTOrg* org)
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

		if (path->s1<0)
		{
			cout<<"negative deravitive:"<<path->s1<<endl;
			system("PAUSE");
		}

		Orderpaths.insert(pair<double, TNM_HyperPath*>(path->s2, path));//key is the current label value of  hyperpath  'avg cost'

		//if (path->s1<0) cout<<"name:"<<path->name<<",cost"<<path->cost<<",s1"<<path->s1<<",flow:"<<path->flow<<endl;
	}
	//cout<<endl;
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
			//if (path->s1<0) cout<<"update path:"<<path->name<<", flow:"<<path->flow<<endl;
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
			plink->volume += lprob*dflow;
			if (abs(plink->volume) < 1e-10)
			{
				plink->volume = 0.0;
			}
			
			plink->UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
			if (plink->rLink )
			{
				plink->rLink->UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
			}

			//if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL )
			//{
			//	plink->TL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
			//	if (plink->rLink) plink->rLink->TL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
			//}
			//else if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_BL||PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy)
			//{
			//	plink->BL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
			//	if (plink->rLink) plink->rLink->BL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);	
			//}
		}			
	}
	/*
	cout<<"after equi"<<endl;
	org->BL_UpdateCapPathSetCost(cb_lambda);
	for(int i = 0;i<org->pathSet.size();i++)
	{				
		TNM_HyperPath* path = org->pathSet[i];  
		cout<<path->name<<","<<path->cost<<endl;
	}*/
	//cout<<endl;

}

void	PTNET::UpdateCapHyperpathGPFlow(PTDestination* dest,PTOrg* org)
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
			//if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_BL ||PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_BL)
			//{
			//	path->TL_UpdateCapHyperpathCost(net->cb_lambda);
			//	MinCostpath->TL_UpdateCapHyperpathCost(net->cb_lambda);
			//}
						
			path->TL_UpdateCapHyperpathCost(cb_lambda);
			MinCostpath->TL_UpdateCapHyperpathCost(cb_lambda);
			//cout<<"before update, pathcost:"<<path->cost<<",minpathcoat:"<<MinCostpath->cost<<",gap:"<<fabs(path->cost-MinCostpath->cost)<<endl;

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
				//if (link->GetTransitLinkType()==PTLink::ENROUTE && link->volume>(link->cap- link->mu/cb_lambda))
				//{
				//	tmp  += cb_lambda * (link->buffer[1] - link->buffer[0]);		
				//}

				if (  link->GetTransitLinkType() == PTLink::ENROUTE && link->volume >= (link->cap - link->mu/cb_lambda) &&link->GetTransitLinkCapType()==PTLink::TL)
				{
					tmp  += cb_lambda * (link->buffer[1] - link->buffer[0]);		
				}

				if (  link->GetTransitLinkType() == PTLink::ABOARD && link->rLink->volume >= (link->rLink->cap - link->mu/cb_lambda) &&link->GetTransitLinkCapType()==PTLink::BL)
				{
					tmp  += cb_lambda * (link->buffer[1] - link->buffer[0]);		// BE CAREFUL!!!
				}

	


				if (link->rLink && link->GetTransitLinkSymmetry() == PTLink::ASYMMTRIC)
				{
					tmp += link->rpfdcost * (link->rLink->buffer[1] - link->rLink->buffer[0]);
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
					//if (link->GetTransitLinkType()==PTLink::ENROUTE && link->volume>(link->cap- link->mu/cb_lambda))
					//{
					//	tmp  += cb_lambda * (link->buffer[1] - link->buffer[0]);		
					//}
					if (  link->GetTransitLinkType() == PTLink::ENROUTE && link->volume >= (link->cap - link->mu/cb_lambda) &&link->GetTransitLinkCapType()==PTLink::TL)
					{
						tmp  += cb_lambda * (link->buffer[1] - link->buffer[0]);		
					}
					if (  link->GetTransitLinkType() == PTLink::ABOARD && link->rLink->volume >= (link->rLink->cap - link->mu/cb_lambda) &&link->GetTransitLinkCapType()==PTLink::BL)
					{
						tmp  += cb_lambda * (link->buffer[1] - link->buffer[0]);		
					}

					if (link->rLink  && link->GetTransitLinkSymmetry() == PTLink::ASYMMTRIC)
					{
						tmp += link->rpfdcost * (link->rLink->buffer[1] - link->rLink->buffer[0]);
						//cout<<"rpfdcost="<<link->rpfdcost<<","<<link->rLink->buffer[0]<<","<<link->rLink->buffer[1]<<endl;
					}
					DerSum += tmp *(link->buffer[1] - link->buffer[0]);	
				
				}
			}


				
			//if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL)
			//{
			//	for(int i=0;i<mglinks.size();i++)
			//	{
			//		link = mglinks[i]->m_linkPtr;
			//		double tmp = link->g_pfdcost * (link->buffer[1] - link->buffer[0]);
			//		if (link->rLink)
			//		{
			//			tmp += link->g_rpfdcost * (link->rLink->buffer[1] - link->rLink->buffer[0]);
			//			//cout<<"rpfdcost="<<link->rpfdcost<<","<<link->rLink->buffer[0]<<","<<link->rLink->buffer[1]<<endl;
			//		}
			//		DerSum += tmp *(link->buffer[1] - link->buffer[0]);				
			//	}
			//	// calculate derivative for non-shortest links
			//	for(int i=0;i<glinks.size();i++)
			//	{
			//		link = glinks[i]->m_linkPtr;
			//		if (link->markStatus != 1)
			//		{
			//			double tmp = link->g_pfdcost * (link->buffer[1] - link->buffer[0]);
			//			if (link->rLink)
			//			{
			//				tmp += link->g_rpfdcost * (link->rLink->buffer[1] - link->rLink->buffer[0]);
			//				//cout<<"rpfdcost="<<link->rpfdcost<<","<<link->rLink->buffer[0]<<","<<link->rLink->buffer[1]<<endl;
			//			}
			//			DerSum += tmp *(link->buffer[1] - link->buffer[0]);	
			//	
			//		}
			//	}
			//}


			/*
		    if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_BL)
			{
				//link on the shortest hyperpath
				for(int i=0;i<mglinks.size();i++)
				{
					link = mglinks[i]->m_linkPtr;
					double tmp = link->pfdcost * (link->buffer[1] - link->buffer[0]);
					if (link->rLink)
					{
						tmp += link->rpfdcost * (link->rLink->buffer[1] - link->rLink->buffer[0]);
					}
					DerSum += tmp *(link->buffer[1] - link->buffer[0]);				

					if (link->GetTransitLinkType()==PTLink::ABOARD )
					{
						if (link->rLink->volume>(link->cap-link->mu/cb_lambda))
						{
							for (int j=0;j<=link->seq;j++) 
							{
								PTLink* blink = link->m_shape->m_boardlinks[j];
								DerSum += (link->buffer[1] - link->buffer[0]) * cb_lambda * (blink->buffer[1] - blink->buffer[0]);
							}			
							for (int j = 0; j<=link->seq-1;j++)
							{
								PTLink* alink = link->m_shape->m_alightlinks[j];
								DerSum -= (link->buffer[1] - link->buffer[0]) * cb_lambda * (alink->buffer[1] - alink->buffer[0]);
							}					
						}
					}
				}
				//link on the non-shortest hyperpath
				for(int i=0;i<glinks.size();i++)
				{
					link = glinks[i]->m_linkPtr;
					if (link->markStatus != 1)
					{
						double tmp = link->pfdcost * (link->buffer[1] - link->buffer[0]);
						if (link->rLink)
						{
							tmp += link->rpfdcost * (link->rLink->buffer[1] - link->rLink->buffer[0]);
						}
						DerSum += tmp *(link->buffer[1] - link->buffer[0]);	
				
						if (link->GetTransitLinkType()==PTLink::ABOARD)
						{
							if (link->rLink->volume>(link->cap-link->mu/cb_lambda))
							{
								for (int j=0;j<=link->seq;j++) 
								{
									PTLink* blink = link->m_shape->m_boardlinks[j];
									DerSum += (link->buffer[1] - link->buffer[0]) * cb_lambda * (blink->buffer[1] - blink->buffer[0]);
								}			
								for (int j = 0; j<=link->seq-1;j++)
								{
									PTLink* alink = link->m_shape->m_alightlinks[j];
									DerSum -= (link->buffer[1] - link->buffer[0]) * cb_lambda * (alink->buffer[1] - alink->buffer[0]);
								}				
							}
						}
					}
				}
			
			}
			*/



			if (DerSum ==0)	DerSum = 1e-8;//no need to shit flow from current path to the shortest path 



			double dev = path->cost - MinCostpath->cost;
			double dflow;

			if (DerSum>=0) 
			{
				if (dev>=0)  //如果当前non-reference path cost > reference path cost，即需要把流量从non-reference path转移至reference path
				{
					dflow =  __min(1.0 * dev /DerSum, path->flow);			//转移  min{ 计算的转移量(+), 当前non-reference path的流量(+)}	
					//cout<<"detaC="<<dev<<" g="<<DerSum<<" CalTransFlow="<<1.0 * dev /DerSum<<", pathflow="<< path->flow<<",dflow"<<dflow<<endl;
				}
				else		 //如果当前non-reference path cost <= reference path cost，即需要把流量从reference path转移至non-reference path
				{
					dflow =  __max(1.0 * dev /DerSum, -MinCostpath->flow);	 //转移  max{ 计算的转移量(-), 当前reference path的流量的负值(-)}
					//cout<<1.0 * dev /DerSum<<","<< MinCostpath->flow<<",dflow"<<dflow<<endl;
					//cout<<"detaC="<<dev<<" g="<<DerSum<<" CalTransFlow="<<1.0 * dev /DerSum<<", minflow="<< MinCostpath->flow<<",dflow"<<dflow<<endl;
				}
				//cout<<"org:"<<org->org->id<<",dest:"<<dest->destination->id<<",dflow:"<<dflow<<",DerSum:"<<DerSum<<",pathcost:"<<path->cost<<",minpathcost:"<<MinCostpath->cost<<endl;
				MinCostpath->flow = MinCostpath->flow + dflow; 	
				path->flow = path->flow - dflow;
			}
			else  //(gk<0)
			{
				if (dev<0)  //如果当前non-reference path cost <= reference path cost，即需要把流量从reference path转移至non-reference path
				{
					dflow =  __min(1.0 * dev /DerSum, MinCostpath->flow);   //转移  min{计算的转移量(+), 当前non-reference path的流量(+)}
					//cout<<"detaC="<<dev<<" g="<<DerSum<<" CalTransFlow="<<1.0 * dev /DerSum<<", pathflow="<< path->flow<<",dflow"<<dflow<<endl;
				
				}
				else		//如果当前non-reference path cost >= reference path cost，即需要把流量从non-reference path转移至reference path
				{
					dflow =  __max(1.0 * dev /DerSum, path->flow);  //转移量(-)  max{ 计算的转移量(-), 当前reference path的流量的负值(-)}
					//cout<<"detaC="<<dev<<" g="<<DerSum<<" CalTransFlow="<<1.0 * dev /DerSum<<", minflow="<< MinCostpath->flow<<",dflow"<<dflow<<endl;
				}
				//cout<<"org:"<<org->org->id<<",dest:"<<dest->destination->id<<",dflow:"<<dflow<<",DerSum:"<<DerSum<<",pathcost:"<<path->cost<<",minpathcost:"<<MinCostpath->cost<<endl;
				MinCostpath->flow = MinCostpath->flow - dflow; 	
				path->flow = path->flow + dflow;
			}

			if (path->flow<0||MinCostpath->flow<0)
			{
				cout<<"path flow:"<<path->flow<<",minpath flow:"<<MinCostpath->flow<<",dev:"<<dev<<",DerSum:"<<DerSum<<",dflow:"<<dflow<<endl;
				system("PAUSE");
			}
			for(int lj=0;lj<glinks.size();lj++)
			{
				link = glinks[lj]->m_linkPtr;
				link ->volume += glinks[lj]->m_data* (path->flow - path->Preflow);

				if (abs(link->volume) < 1e-8) link ->volume = 0.0;
				
				link->UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
				if (link->rLink)
				{
					link->rLink->UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
				}

				/*if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL )
				{
					link->TL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
					if (link->rLink) link->rLink->TL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
				}
				else if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_BL)
				{
					link->BL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
					if (link->rLink) link->rLink->BL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);	
				}*/
				
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
				//if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL )
				//{
				//	link->TL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
				//	if (link->rLink) link->rLink->TL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
				//}
				//else if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_BL)
				//{
				//	link->BL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
				//	if (link->rLink) link->rLink->BL_UpdateGeneralPTLink_Cost_DerCost(cb_lambda);	
				//}
				link->UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
				if (link->rLink && link->GetTransitLinkSymmetry()==PTLink::ASYMMTRIC)
				{
					link->rLink->UpdateGeneralPTLink_Cost_DerCost(cb_lambda);
				}
			}
			
			path->TL_UpdateCapHyperpathCost(cb_lambda);
			MinCostpath->TL_UpdateCapHyperpathCost(cb_lambda);
			//cout<<"after update, pathcost:"<<path->cost<<",minpathcoat:"<<MinCostpath->cost<<",gap:"<<fabs(path->cost-MinCostpath->cost)<<endl<<endl;

			//if (DerSum<0)
			//{
			//	path->UpdateCapHyperpathCost();
			//	MinCostpath->UpdateCapHyperpathCost();
			//	cout<<"cost diff:"<<path->cost-MinCostpath->cost<<endl;
			//
			//}

		}
	}



}

void TNM_HyperPath::UpdateCapHyperpathCost(floatType lambda)
{
	cost = WaitCost;
	fdcost = 0.0;
	vector<GLINK*>::iterator it;
	for(int i=0;i<m_links.size();++i)
	{
		PTLink* link =   m_links[i]->m_linkPtr;
		cost += m_links[i]->m_data * link ->g_cost;
		fdcost += m_links[i]->m_data * link ->pfdcost * m_links[i]->m_data;	
		if (link->GetTransitLinkType() == PTLink::ABOARD  )
		{
			if (link->rLink->volume>=(link->cap - link->mu/lambda))
			{
				int seq = link->seq;
				
				if (seq>0)// 
				{
					PTLink* tlink = link->m_shape->m_boardlinks[seq-1]->rLink;
					it = find_if(m_links.begin(), m_links.end(), finder_t(tlink->id)); 
					if (it != m_links.end())
					{
						fdcost += m_links[i]->m_data *  lambda * (*it)->m_data;
					}

					PTLink* alink = link->m_shape->m_alightlinks[seq-1];
					it = find_if(m_links.begin(), m_links.end(), finder_t(alink->id)); 
					if (it != m_links.end())
					{
						fdcost -= m_links[i]->m_data *  lambda * (*it)->m_data;
					}
				}

				fdcost += m_links[i]->m_data *  lambda *m_links[i]->m_data;
			}

		}
		if (link->rLink)
		{
			it = find_if(m_links.begin(), m_links.end(), finder_t(link->rLink->id)); 
			if (it != m_links.end())
			{
				fdcost += m_links[i]->m_data *  link->rpfdcost * (*it)->m_data;
			}
		}
	}
	if (fdcost<0) 
	{
		cout<<"Warning! The derivative cost for path:"<<name<<"is negtive:"<<fdcost<<endl;
		system("PAUSE");
	}
}

void TNM_HyperPath::TL_UpdateCapHyperpathCost(floatType lambda)
{
	cost = WaitCost;
	fdcost = 0.0;

	for(int i=0;i<m_links.size();++i)
	{
		PTLink* link =   m_links[i]->m_linkPtr;
		cost += m_links[i]->m_data * link ->g_cost;
		fdcost += m_links[i]->m_data * link ->pfdcost * m_links[i]->m_data;	
		if (link->rLink && link->GetTransitLinkSymmetry() == PTLink::ASYMMTRIC)
		{
			vector<GLINK*>::iterator it = find_if(m_links.begin(), m_links.end(), finder_t(link->rLink->id)); 			
			if (it != m_links.end())
			{
				fdcost += m_links[i]->m_data *  link->rpfdcost * (*it)->m_data;
			}
		}


		if (  link->GetTransitLinkType() == PTLink::ENROUTE && link->volume >= (link->cap - link->mu/lambda) &&link->GetTransitLinkCapType()==PTLink::TL)
		{
			fdcost +=  m_links[i]->m_data *  lambda * m_links[i]->m_data;
		}

		if (  link->GetTransitLinkType() == PTLink::ABOARD && link->rLink->volume >= (link->rLink->cap - link->mu/lambda) &&link->GetTransitLinkCapType()==PTLink::BL)
		{
			fdcost +=  m_links[i]->m_data *  lambda * m_links[i]->m_data;
		}
	}

	if (fdcost<0) 
	{
		cout<<"Warning! The derivative cost for path:"<<name<<"is negtive:"<<fdcost<<endl;
		system("PAUSE");
	}
}


void TNM_HyperPath::BL_UpdateCapHyperpathCost(floatType lambda)
{
	cost = WaitCost;
	fdcost = 0.0;

	for(int i=0;i<m_links.size();++i)
	{
		PTLink* link =   m_links[i]->m_linkPtr;
		//cout<<"1"<<endl;
		cost += m_links[i]->m_data * link ->g_cost;
		
		fdcost += m_links[i]->m_data * link ->pfdcost * m_links[i]->m_data;	
		//cout<<"2"<<endl;

		if (link->rLink && link->GetTransitLinkSymmetry() == PTLink::ASYMMTRIC   )
		{
			vector<GLINK*>::iterator it = find_if(m_links.begin(), m_links.end(), finder_t(link->rLink->id)); 
			if (it != m_links.end())
			{
				fdcost += m_links[i]->m_data *  link->rpfdcost * (*it)->m_data;
			}
		}


		if (link->GetTransitLinkType() == PTLink::ABOARD  )
		{
			
			if (link->rLink->volume>=(link->cap - link->mu/lambda))
			{
				//vector<GLINK*>::iterator mit = find_if(m_links.begin(), m_links.end(), finder_t(link->rLink->id)); 
				//if (mit != m_links.end())
				//{
				//	fdcost += m_links[i]->m_data * lambda * (*mit)->m_data;
				//
				//}

				for (int j=0;j<=link->seq;j++) 
				{
					PTLink* blink = link->m_shape->m_boardlinks[j];
					vector<GLINK*>::iterator it = find_if(m_links.begin(), m_links.end(), finder_t(blink->id)); 
					if (it != m_links.end())
					{
						fdcost +=  m_links[i]->m_data *  lambda * (*it)->m_data;
					}
				}			
				for (int j = 0; j<=link->seq-1;j++)
				{
					PTLink* alink = link->m_shape->m_alightlinks[j];
					vector<GLINK*>::iterator it = find_if(m_links.begin(), m_links.end(), finder_t(alink->id)); 
					if (it != m_links.end())
					{
						fdcost -=  m_links[i]->m_data *  lambda * (*it)->m_data;
					}				
				}
			
			}
		}
			
		
		//cout<<"7"<<endl;
	}
	if (fdcost<0) 
	{
		cout<<"Warning! The derivative cost for path:"<<name<<"is negtive:"<<fdcost<<endl;
		system("PAUSE");
	}

}

void  PTOrg::UpdateCapPathSetCost(floatType lambda)
{
	floatType maxCost= - 1.0 , minCost=POS_INF_FLOAT;
	currentTotalCost = 0.0;

	for (int i =0; i < pathSet.size();i++)
	{		
		TNM_HyperPath* path= pathSet[i];
		path->UpdateCapHyperpathCost(lambda);
		if (path->cost> maxCost)	maxCost =   path->cost;

		if (path->cost< minCost)    
		{
			minCost = path->cost; 
			minIx = i; 
		}
		currentTotalCost += path->cost * path->flow;
	}
	maxPathGap = maxCost - minCost;
	currentRelativeGap = fabs( 1 - minCost * assDemand / currentTotalCost);

}


void	PTOrg::BL_UpdateCapPathSetCost(floatType lambda)
{
	floatType maxCost= - 1.0 , minCost=POS_INF_FLOAT;
	currentTotalCost = 0.0;

	for (int i =0; i < pathSet.size();i++)
	{		
		TNM_HyperPath* path= pathSet[i];
		path->BL_UpdateCapHyperpathCost(lambda);


		if (path->cost> maxCost)	maxCost =   path->cost;

		if (path->cost< minCost)    
		{
			minCost = path->cost; 
			minIx = i; 
		}
		currentTotalCost += path->cost * path->flow;
	}

	maxPathGap = maxCost - minCost;
	currentRelativeGap = fabs( 1 - minCost * assDemand / currentTotalCost);


}



void	PTOrg::TL_UpdateCapPathSetCost(floatType lambda)
{
	double maxCost= - 1.0 , minCost=POS_INF_FLOAT;
	currentTotalCost = 0.0;

	for (int i =0; i < pathSet.size();i++)
	{		
		TNM_HyperPath* path= pathSet[i];
		path->TL_UpdateCapHyperpathCost(lambda);

		if (path->cost> maxCost)	maxCost =   path->cost;

		if (path->cost< minCost)    
		{
			minCost = path->cost; 
			minIx = i; 
		}
		currentTotalCost += path->cost * path->flow;
	}

	maxPathGap = maxCost - minCost;
	currentRelativeGap = fabs( 1 - minCost * assDemand / currentTotalCost);


}

void	PTNET::IPFHyperpathCapInnerLoop(int maxiters )
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
				org->CTEAPIPFUpdatePathSetCost(gamma);
			
				double mincost = org->pathSet[org->minIx]->cost;
				double rg = fabs( 1 - mincost * org->assDemand / org->currentTotalCost);		
				if(rg > inner_indicator && org->pathSet.size() > 1)
				{				
					if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL)		
							UpdateCapHyperPathGreedyFlow(dest,org);	
					else if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL)			
							UpdateCapHyperpathGPFlow(dest,org);
					else if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL)
							CTEAPIPFUpdateHyperpathGPFlow(dest,org);
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
	InnerIters = n;

}

void	PTNET::HyperpathCapInnerLoop(int maxiters )
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
				org->TL_UpdateCapPathSetCost(cb_lambda);
				
				double mincost = org->pathSet[org->minIx]->cost;
				double rg = fabs( 1 - mincost * org->assDemand / org->currentTotalCost);		
				if(rg > inner_indicator && org->pathSet.size() > 1)
				{				
					if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL)		
							UpdateCapHyperPathGreedyFlow(dest,org);	
						else if (PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL)			
							UpdateCapHyperpathGPFlow(dest,org);
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


}

void	PTNET::RecordCapTEAPCurrentIterIPF(float GAP,float IF)
{
	PTITERELEM *iterElem = new PTITERELEM;
	iterElem->iter  = curIter;
	iterElem->time  = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	iterElem->convRGap = SubRG;
	iterElem->convIF = IF;
	iterElem->multiplierconv = GAP;
	iterElem->penalty = gamma;
	iterElem->epsi = epsilon;
	iterElem ->innerIters =SubIter;
	iterElem->numberofhyperpaths=numaOfHyperpath;
	iterRecord.push_back(iterElem); 
}


void	PTNET::RecordCapTEAPCurrentIter(float GAP, float IF, float SN)
{
	PTITERELEM *iterElem = new PTITERELEM;
	iterElem->iter  = curIter;
	iterElem->time  = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC;
	iterElem->convGap  = SN;
	iterElem->convRGap = SubRG;
	iterElem->convIF = IF;
	iterElem->multiplierconv = GAP;
	iterElem->penalty = cb_lambda;
	iterElem ->innerIters =SubIter;
	iterRecord.push_back(iterElem); 
}

void	PTNET::ReportCapIter()
{
	string IterConvName  = networkName +"-" + GetAlgorithmName() + ".conv";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, IterConvName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .conv file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting iterative information into "<<IterConvName<<" for cap covergence info!"<<endl;
	outfile<<"Iteration,Time,IF,SubGap,SN,MuGAP,penalty,subiter"<<endl;
	//outfile<<"0,1,,0,,,,"<<endl;
	for(int i=0;i<iterRecord.size();i++)
	{
		PTITERELEM* it = iterRecord[i];
		outfile<<TNM_IntFormat(it->iter)<<","<<TNM_FloatFormat(it->time,6,3)<<","<<TNM_FloatFormat(it->convIF,20,18)<<","<<TNM_FloatFormat(it->convRGap,20,18)<<","<<TNM_FloatFormat(it->convGap,20,18)<<","<<TNM_FloatFormat(it->multiplierconv,20,18)<<","<<TNM_FloatFormat(it->penalty,12,6)<<","<<TNM_IntFormat(it->innerIters)<<endl;
		//outfile<<TNM_IntFormat(it->iter)<<","<<TNM_FloatFormat(it->time,6,3)<<","<<TNM_FloatFormat(it->convGap,20,18)<<","<<TNM_FloatFormat(it->convRGap,20,18)<<","<<TNM_FloatFormat(it->multiplierconv,20,18)<<","<<TNM_FloatFormat(it->penalty,12,6)<<","<<TNM_IntFormat(it->innerIters)<<endl;
	}
	outfile.close();


}


floatType PTNET::ComputeNormSolution()
{
	floatType norm = 0.0;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];

		norm+=pow((link->SCvolume-link->volume),2);

	}
	norm += pow(SCnetTTwaitcost-netTTwaitcost,2);
	return norm;

}

void PTNET::SetCurrentSolution()
{
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		link->SCvolume = link->volume;
	}
	SCnetTTwaitcost = netTTwaitcost;
}

bool	PTNET::InitialCapHyperpathNetFlow()
{

	PTDestination* dest;

	for (int i = 0;i<numOfPTDest; i++)
	{
		dest = PTDestVector[i];
		//InitializeHyperpathLS(dest->destination);
		InitializeGeneralHyperpathLS(dest->destination);
		for (int j=0; j<dest->numOfOrg; j++)
		{		
			TNM_HyperPath* path = new TNM_HyperPath();
			PTOrg* org= dest->orgVector[j];

			if (path->InitializeHP(org->org,dest->destination))
			{
				double dmd = org->assDemand;//total demand.
				//cout<<"od:"<<org->org->id<<"->"<<dest->destination->id<<",waitcost:"<<path->WaitCost<<",dmd:"<<dmd<<endl;
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

void PTNET::GenerateODWalkFile()
{
	string fileName  = networkName + "_odwalk.txt";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\t Fail to prepare report: Cannot open .hyperpathue file to write information!"<<endl;
		return;
	}
	outfile<<"ORGSTOP,DESTSTOP,FFT"<<endl;
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
	
		if (link->GetTransitLinkType() ==  PTLink::ABOARD) link->cost = link->m_hwmean * 0.5;
		else if (link->GetTransitLinkType() ==  PTLink::ALIGHT) link->cost = 0.01;
		else link->cost = link->fft;
	}

	
	PTDestination* dest;
	for (int i = 0;i<numOfPTDest; i++)
	{
		dest = PTDestVector[i];
		if (!InitializeHyperpathLS(dest->destination))
		{
			for (int j=0; j<dest->numOfOrg; j++)
			{													
				PTOrg* org= dest->orgVector[j];
				int c = 3*org->org->StgElem->cost;
				outfile<<org->org->id<<","<<dest->destination->id<<","<<c<<endl;
			}
			
		}
	}
	outfile.close();
	cout<<endl;
}


int   PTNET::ReadODWALK()
{
	string infilename = networkName + "_odwalk.txt";
    ifstream infile;
    if(!TNM_OpenInFile(infile, infilename))
    {
        cout<<"\tCannot open file "<<infilename<<" to read"<<endl;
        return 1;
    }
    cout<<"\tReading od walking information "<<endl;
    string line;
    vector<string> words;
    getline(infile, line);
	double ttfreq = 0.0;
    while(getline(infile, line))
    {
		TNM_GetWordsFromLine(line, words,',');
		if(words.size() < 3)
        {
            cout<<"\tExpected at lest six columns, found "<<words.size()<<" on "<<line<<endl;
            return 1;
        }
		int t,h;
		double c;
		TNM_FromString(t, words[0], std::dec);
		TNM_FromString(h, words[1], std::dec);
		TNM_FromString(c, words[2], std::dec);

		PTNode* tail = CatchNodePtr(t);
		PTNode* head = CatchNodePtr(h);
		PTLink* link = CatchLinkPtr(tail,head);
		if (!link)
		{
			cout<<"\t Can not find od walk link for line "<<line<<endl;
		}
		link->pars.push_back(1.0);//alpha_1
		link->pars.push_back(c);
		link->fft =c;
	}

	return 0;
}