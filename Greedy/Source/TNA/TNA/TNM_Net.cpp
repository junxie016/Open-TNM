#include "header\stdafx.h"
#include <iostream>
#include <iomanip>
#include <string.h>
#include <math.h>
#include <cassert>
#include <cstdlib>
#include <stack>
#include <algorithm>
//#include <my_predicate.h>
using namespace std;

smallInt TNM_SPATH::pathBufferSize = 0;
IDManager TNM_SPATH::idManager;

TNM_SNODE::TNM_SNODE()
{
	id        =  -1;
	type      =  BASND;
	xCord      = 0;
	yCord      = 0;
	scanStatus = 0;
	pathElem   = new PATHELEM;
	rPathElem  = new PATHELEM;
	buffer     = NULL;
	//pthHeap    = NULL;
	//dummy      = false;
	//guiNode    = NULL;
	//m_spTree   = NULL;
	//kspPEvector = NULL;
	//kspPathElem = NULL;
    attachedOrg= 0;
	attachedDest=0;
	m_isThrough = true;
	is_centroid =false;
	SkipCentroid=false;
}

TNM_SNODE::~TNM_SNODE()
{
	
	for(PTRTRACE pv = forwStar.begin(); pv!= forwStar.end(); pv++)  
		if(*pv!=NULL) 
		{
			delete *pv;
			*pv = NULL;
		}
   for(PTRTRACE pv = backStar.begin(); pv!= backStar.end(); pv++)  
	   if(*pv!=NULL) 
	   {
		   delete *pv;
		   *pv = NULL;
	   }
	   if(!forwStar.empty()) forwStar.clear();
	   if(!backStar.empty()) backStar.clear();
  // cout<<"finish deleting links for node "<<id<<endl;
   if(buffer!=NULL)
   {
	   delete [] buffer;
	   buffer = NULL;
   }

   //cout<<"finish delete gui for node "<<id<<endl;
   delete pathElem;
   delete rPathElem;
  // cout<<"finish delete node "<<id<<endl;
}

void TNM_SNODE::InitPathElem()
{
	pathElem->cost = POS_INF_FLOAT;
	pathElem->via  = NULL;
}

bool TNM_SNODE::Initialize(const NODE_VALCONTAINER &cont)
{
	id    = cont.id;
	dummy = cont.dummy;
	xCord = cont.xCord;
	yCord = cont.yCord;
	return true;
}

void TNM_SNODE::SearchMinInLink(SCANLIST *list)
{
	LINK_COST_TYPE dp, curTT;
	vector<TNM_SLINK *>::iterator pv;
	TNM_SNODE *scanNode;
	for (pv = backStar.begin();pv!=backStar.end();pv++)
	{
		if(*pv!=NULL)
		{
			//cout<<"\tsearch link "<<(*pv)->id<<endl;
			scanNode = (*pv)->tail; // get upstream node
			curTT    = (*pv)->cost; //get link cost
            dp       = pathElem->cost; //get the label of current node
		/*	cout<<"cost of its tail node "<<scanNode->id<<" = "<<scanNode->pathElem->cost
				<<"\t curTT + previous cost = "<<curTT + dp<<endl;*/
				if(scanNode->pathElem->cost > curTT + dp)
				{ // if so, update the routing table
                    scanNode->pathElem->cost = curTT + dp;
					scanNode->pathElem->via  = *pv;
    				list->InsertANode(scanNode);
				}// end if
		}
	}
}

void TNM_SNODE::SearchMinOutLink(SCANLIST *list)
{
	LINK_COST_TYPE dp, curTT;
	vector<TNM_SLINK *>::iterator pv;
	TNM_SNODE *scanNode;
	for (pv = forwStar.begin();pv!=forwStar.end();pv++)
	{
		if(*pv!=NULL)
		{
			scanNode = (*pv)->head; // get upstream node
			curTT    = (*pv)->cost; //get link cost
            dp       = pathElem->cost; //get the label of current node
				if(scanNode->pathElem->cost > curTT + dp)
				{ // if so, update the routing table
                    scanNode->pathElem->cost = curTT + dp;
					scanNode->pathElem->via  = *pv;
    				list->InsertANode(scanNode);
				}// end if
		}
	}
}

TNM_SLINK::TNM_SLINK()
{
	id         =  0;
	orderID    = 0;
	type       =  BASLK;
	head       =  NULL;         /*starting node of the link*/
	tail       =  NULL;         /*ending node of the link */
	capacity   =  0.0;          /*link capacity*/
	volume     =  0.0;          /*link volume*/
	length     =  0.0;          /*link length*/
	ffs        =  0.0;          /*free flow speed*/
	fft        =  0.0;          /*free flow travel time*/
	cost       =  0;
	toll       =  0.0;
	fdCost     =  0.0;
	//m_classid  =  -1;   //no clasification. 
	markStatus =  0;
	oLinkPtr   =  NULL;
	//revLink    =  NULL;
	buffer     =  NULL;
	//dummy      =  false;
	//guiLink    =  NULL;
	//m_tlType   =  TT_NOTOLL;
	m_timeCostCoefficient = 1.0;
	m_distCostCoefficient = 0.0;

}

TNM_SLINK::~TNM_SLINK()
{
	DisconnectFW(); //remove forward connection
	DisconnectBK(); //remove backward connection
	//if (revLink!=NULL) 	revLink->revLink = NULL;//set its reverse link's revLink field as NULL;
	if (buffer!=NULL) 
	{
		delete [] buffer;
		buffer = NULL;
	}
	
}

void TNM_SLINK::DisconnectFW()
{
	//cout<<"SLINK: disconnect FW"<<endl;
	vector<TNM_SLINK *>::iterator pv;
	pv = find(tail->forwStar.begin(), tail->forwStar.end(), this); //directly compare the pointer, this is, somehow
                                                                   //dangerous, because it is possible that the
	                                                               //object has been deleted. so make sure if you want
	                                                               //to delete a link,erase its connection first.
	if(pv!=tail->forwStar.end()) 	tail->forwStar.erase(pv);
}

void TNM_SLINK::DisconnectBK()
{	
	//cout<<"SLINK: disconnect FW"<<endl;
	vector<TNM_SLINK *>::iterator pv;
	pv = find(head->backStar.begin(), head->backStar.end(), this);
	if(pv!=head->backStar.end()) 		head->backStar.erase(pv);
}

bool TNM_SLINK::Initialize(const LINK_VALCONTAINER cont)
{
    //cout<<"\tinitializing slink"<<endl;
	id         = cont.id;
	head       = cont.head;
	tail       = cont.tail;
	capacity   = cont.capacity;
	length     = cont.length;
	ffs        = cont.ffs;
	fft        = length/ffs;
	cost       = fft;
	dummy      = cont.dummy;
	if(tail == NULL)
	{
		cout<<"Creating a new link: tail node pointer is invalid"<<endl;
		return false;
	}
	if(head == NULL)
	{
		cout<<"Creating a new link: head node pointer is invalid"<<endl;
		return false;
	}
	if(capacity<0)    
	{
		cout<<"Warning: negative cap found when constructing a link."<<endl;
		return false;
	}
	if(length<0)    
	{
		cout<<"Warning: negative length found when constructing a link. length  = "<<length<<endl;
		return false;
	}
	if(ffs<0)  
	{
		cout<<"Warning: negative speed found when constructing a link. speed = "<<ffs<<endl;
		return false;
	}
	if(!CheckParallel()) return false;
	ConnectFW();
	ConnectBK(); //connect foward star and backward star
	return true;
	
}

void TNM_SLINK::ConnectFW()
{
	tail->forwStar.push_back(this);
}

void TNM_SLINK::ConnectBK()
{
	head->backStar.push_back(this);
}

bool TNM_SLINK::CheckParallel()
{
	TNM_SLINK *link;
		for (PTRTRACE pv = tail->forwStar.begin(); pv!=tail->forwStar.end(); pv++)
		{
			link = *pv;
			if(link!=NULL)
			{
				if(link->head == head) 
				{
					cout<<"Parallel links found: link "<<link->id<<" and "<<id<<endl;
					return false;
				}
				if(link->id   == id)   
				{
					cout<<"Duplicate links found: link "<<link->id<<" existed "<<endl;
					return false;
				}
			}
		}
		for (PTRTRACE pv = head->backStar.begin(); pv!=head->backStar.end(); pv++)
		{
			link = *pv;
			if(link!=NULL)
			{
				if(link->tail == tail)
				{
					cout<<"Parallel links found: link "<<link->id<<" and "<<id<<endl;
					return false;
				}
				if(link->id   == id)   
				{
					cout<<"Duplicate links found: link "<<link->id<<" existed "<<endl;
					return false;
				}
			}

		}
	return true;
}

floatType TNM_SLINK::GetCost(bool ftoll)
{
	double t, mc = 0.0;
	//double v = volume;
	//if(m_classid >0)
	//{
	//	volume += m_mcHolder.TotalFlowExceptOne(m_classid);
	//	mc = m_mcHolder.GetClassPtr(m_classid)->m_smcost;
	//}
	//switch(m_tlType)
	//{
	//case TT_NOTOLL:
		t =  GetCost_() * m_timeCostCoefficient + length * m_distCostCoefficient + mc;
	//	break;
	//case TT_MTTOLL:
	//	t =  (GetCost_() + GetDerCost_() * volume)*m_timeCostCoefficient + (length * m_distCostCoefficient + mc);
	//	break;
	//case TT_MCTOLL:
	//	if(m_classid == -1) t =  (GetCost_()  + GetDerCost_()*volume) * m_timeCostCoefficient + length * m_distCostCoefficient + mc; //sigle class case:
	//	else  t =  GetCost_()  * m_timeCostCoefficient + GetDerCost_() * m_mcHolder.WeightedFlow(m_classid, v) + length * m_distCostCoefficient + mc;
	//		
	//	break;
	//case TT_FXTOLL:
	//	t =  GetCost_() * m_timeCostCoefficient + length * m_distCostCoefficient + toll + mc;
	//	break;
	//}
	//if(m_classid > 0) volume = v;
	//return t;

	return t;
}

//for derivative, for both fixed toll and no toll case, we don't even need to consider other costs, as they are constant
//for first-best toll, they need to be included. 
floatType TNM_SLINK::GetDerCost(bool ftoll)
{
	//double t, v = volume;
	//if(m_classid >0)
	//{
	//	volume += m_mcHolder.TotalFlowExceptOne(m_classid);
	//}
	//switch(m_tlType)
	//{
	//case TT_MTTOLL:
	//	t= (2.0* GetDerCost_() + volume * GetDer2Cost_())*m_timeCostCoefficient;
	//	break;
	//case TT_MCTOLL:
	//	if(m_classid == -1) t= (2.0* GetDerCost_() + volume * GetDer2Cost_())*m_timeCostCoefficient;
	//	else         t = GetDerCost_()  * (m_timeCostCoefficient + m_mcHolder.WeightedVOT(m_classid, v)) + GetDer2Cost_() * m_mcHolder.WeightedFlow(m_classid, v);
	//	break;
	//default:
	//	t = GetDerCost_()* m_timeCostCoefficient;
	//}
	//if(m_classid > 0) volume = v;
	//return (t<1e-15? 1e-15:t);

	double t=GetDerCost_();
	return (t<1e-15? 1e-15:t);
}

floatType TNM_SLINK::GetIntCost(bool ftoll) //when you call this function, make sure volume is the total volume. 
{
	double t;
	//switch(m_classid)
	//{
	//	case -1: //single class;
	//		switch(m_tlType)
	//		{
	//			case TT_NOTOLL:
				t = GetIntCost_()*m_timeCostCoefficient  + length * m_distCostCoefficient * volume; //in NOTOLL equilibrium case, the to
	//			break;
	//		case TT_MTTOLL:
	//			//volume = v; //for first best toll, we reset volume back to just plain volume.  GetDerCost_() * total weight flow is the toll
	//			t=  (GetCost_() + GetDerCost_()*volume) * volume  + length * m_distCostCoefficient * volume/m_timeCostCoefficient; //total travel time already includes the margincal cost toll. 
	//			break;
	//		case TT_MCTOLL:
	//			t=  (GetCost_() + GetDerCost_()*volume) * volume * m_timeCostCoefficient + length * m_distCostCoefficient * volume;
	//			break;
	//		case TT_FXTOLL:
	//			t = GetIntCost_() * m_timeCostCoefficient + (length * m_distCostCoefficient + toll) * volume;
	//			break;
	//		}
	//		break;
	//	case 0: //mulitiple class wthen link is not set into a particular call state.
	//		switch(m_tlType)
	//		{
	//		case TT_NOTOLL:
	//			t = GetIntCost_() + length * m_distCostCoefficient * m_mcHolder.InverseWeightedFlow() + m_mcHolder.TotalSMTime(); //in NOTOLL equilibrium case, the to
	//			break;
	//		case TT_MTTOLL:
	//			t = (GetCost_() + GetDerCost_() * volume) * volume + length * m_distCostCoefficient * m_mcHolder.InverseWeightedFlow() + m_mcHolder.TotalSMTime();
	//			break;
	//		case TT_MCTOLL:
	//			//volume = v; //for first best toll, we reset volume back to just plain volume.  GetDerCost_() * total weight flow is the toll
	//			//t=  (GetCost_() + GetDerCost_()*volume) * volume * m_timeCostCoefficient + length * m_distCostCoefficient * volume; //total travel time already includes the margincal cost toll. 
	//			double v;
	//			v = m_mcHolder.WeightedFlow();
	//			t=  (GetCost_() + GetDerCost_()*v) *v  + length * m_distCostCoefficient * volume + m_mcHolder.TotalSMCost();
	//			break;
	//		case TT_FXTOLL:
	//			t = GetIntCost_() + (length * m_distCostCoefficient + toll) * m_mcHolder.InverseWeightedFlow() + m_mcHolder.TotalSMTime();
	//			break;
	//		}
	//		break;
	//	default: //
	//		cout<<"\tWarning, calling GetIntCost when classID > 0 is unexpected. All calls will receive 0 as return"<<endl;
	//		t = 0.0;
	//		break;
	//}
	return t;
}

double TNM_SLINK::GetToll()
{
	double t;
	//switch(m_classid)
	//{
	//	case -1: //single class;
	//		switch(m_tlType)
	//		{
	//		case TT_NOTOLL:
				t = 0.0;
	//			break;
	//		case TT_MTTOLL:
	//		case TT_MCTOLL:
	//			t = GetDerCost_() * volume* m_timeCostCoefficient; //note that volume should be the current total volume!
	//			break;
	//		case TT_FXTOLL:
	//			t = toll; //directly return the current toll;
	//			break;
	//		}
	//		break;
	//	default: //
	//		
	//		switch(m_tlType)
	//		{
	//		case TT_NOTOLL:
	//			t = 0;
	//			break;
	//		case TT_MCTOLL:		
	//			{
	//			double v = volume;
	//			volume += m_mcHolder.TotalFlowExceptOne(m_classid);
	//			t = GetDerCost_() * m_mcHolder.WeightedFlow(m_classid, v); //note that volume should be the current total volume!
	//			volume = v;
	//			break;
	//			}
	//		case TT_MTTOLL:
	//			{
	//				double v = volume;
	//				volume += m_mcHolder.TotalFlowExceptOne(m_classid);
	//				t = GetDerCost_() * volume *m_timeCostCoefficient;
	//				volume = v;
	//			break;
	//			}
	//		case TT_FXTOLL:
	//			t = toll; //directly return the current toll;
	//			break;
	//		}
	//		break;
	//}
	return t;

	
}

bool TNM_BPRLK::Initialize(const LINK_VALCONTAINER cont)
{
	if (cont.par.size()==2) //in general this is not in use.
	{
		if (cont.par[0]>=0.0 && cont.par[0]<999.0) alpha = cont.par[0];
		if (cont.par[1]>=0.0 && cont.par[1]<99.0)  beta  = cont.par[1];
	}
	return TNM_SLINK::Initialize(cont);
}

floatType TNM_BPRLK::GetCost_()
{
	if(volume <= 0.0) return fft;
	else             return fft*(1.0 + alpha * pow(volume/capacity,beta));
	
//	return fft*(1.0 + 0.15 * powf(volume/capacity,4));
}

floatType TNM_BPRLK::GetDerCost_()
{
	if (volume<= 0.0) return 0.0;
	else return (fabs(beta - 0.0) < 1e-6? 0: fft* alpha * beta * pow(volume/capacity, beta-1.0)/capacity);
	//return fft* alpha * beta * pow(volume/capacity, beta-1.0)/capacity;
	//return fft* 0.6 * pow(volume/capacity, 3)/capacity;
}

floatType TNM_BPRLK::GetIntCost_()
{
	if(volume <= 0.0) return 0.0;
	else              return fft*volume *(1.0 + alpha * pow(volume/capacity, beta)/(beta+1.0));
	//return fft*volume *(1.0 + 0.15 * pow(volume/capacity, 4)/(5.0));
}

floatType TNM_BPRLK::GetDer2Cost_()
{
	if(volume<=0.0) return 0.0;
	else            return (fabs(beta - 1.0) < 1e-6? 0.0: fft * alpha * beta * (beta - 1.0) * pow(volume/capacity,beta - 2.0) /(capacity * capacity));
}

TNM_SPATH::TNM_SPATH()
{
	//SetID();
	id = idManager.SelectANewID();
	idManager.RegisterID(id);
	flow       = 0.0; 
	cost       = 0.0; 
	//active     = true;
	//reIte      =0;
	if (pathBufferSize == 0) buffer     = NULL;
	else
	{
		buffer = new floatType[pathBufferSize];
		for (int i = 0; i<pathBufferSize;i++)
			buffer[i] = 0.0;
	}
	markStatus = 0;
	//m_refAsnElem = NULL;
	//preFlow =0.0;
	//preRatio =0.0;
	//curRatio =0.0;
	fdCost =0.0;
	estCost =0.0;
}

TNM_SPATH::~TNM_SPATH()
{
	if (buffer !=NULL) delete [] buffer;
	buffer =NULL;
	path.clear();
	idManager.UnRegisterID(id);
}

TNM_SORIGIN::TNM_SORIGIN()
{
	origin = NULL;
	numOfDest = 0;
	destVector = NULL;
	m_class    = 1; 
	m_tdmd     = 0.0;
	//m_smcost   = 0.0;
	//m_id = 0;
	//m_converged = false;
}

TNM_SORIGIN::TNM_SORIGIN(TNM_SNODE *org, int nd)
{
	origin    = org;
	org->attachedOrg++;
	numOfDest = nd;
	if(numOfDest > 0) 
	{
		destVector = new TNM_SDEST*[numOfDest];
		for (int i = 0;i<numOfDest;i++)
			destVector[i] = new TNM_SDEST;
	}
	else
		destVector = NULL;
	//m_orderSubNet = true;
	//m_trimmed = false;
	//m_expanded = false;
	m_class = 1;
	m_tdmd  = 0.0;
	//m_smcost = 0.0;
	//m_count++;
	//m_id = m_count;
}

TNM_SORIGIN::~TNM_SORIGIN()
{
	//cout<<"Deleting a static origin object"<<endl;
	//m_count--;
	//origin->attachedOrg--;
	for (int i = 0;i<numOfDest;i++)
		delete destVector[i];
	//cout<<"destinations are deleted."<<endl;
	if(destVector) delete [] destVector;
	DeleteBush();
}

void TNM_SORIGIN::DeleteBush()
{
for (vector<ORGLINK*>::iterator pv = obLinkVector.begin();
	     pv!=obLinkVector.end(); pv++)
			 delete *pv;
    obLinkVector.clear();
	for (vector<ORGNODE*>::iterator pl = tplNodeVector.begin();
	     pl!=tplNodeVector.end(); pl++)
			 delete *pl;
	tplNodeVector.clear();
}

bool TNM_SORIGIN::SetDest(int id, TNM_SNODE *node, floatType demand)
{
	if(id>numOfDest||id<=0) 
	{
		cout<<"\n\tSetDest in Origin Object: dest index exceeds the rannge!"
			<<"\n\trequired index = "<<id<<endl;;
		return false;
	}
	return destVector[id - 1]->Initialize(origin, node, demand);
}

//Get all destinatio vectors from destVector to bDestVector
void TNM_SORIGIN::LoadDestVector()
{
	if(destVector == NULL) return;
	bDestVector.clear();
	bDestVector.insert(bDestVector.begin(), destVector, destVector + numOfDest);
	if(destVector!=NULL) 
	{
		delete [] destVector;
		destVector = NULL;
	}
	numOfDest = bDestVector.size();
}

int TNM_SORIGIN::DeleteZeroOD()
{
	vector<TNM_SDEST *>::iterator pv;
	pv = bDestVector.begin();
	while(pv != bDestVector.end())
	{
		if((*pv)->assDemand <=0) 
		{
			delete *pv;
			pv = bDestVector.erase(pv);
		}
		else pv++;
		
	}
	numOfDest = bDestVector.size();
	return 0;

}

void TNM_SORIGIN::UnLoadDestVector()
{
	if(destVector!=NULL) return;
	numOfDest = bDestVector.size();
	if(numOfDest>0)
	{
		destVector = new TNM_SDEST*[numOfDest];
		for (int i = 0;i<numOfDest;i++)
		{
			destVector[i] = bDestVector[i];
		}
		bDestVector.clear();
	}
}

TNM_SDEST::TNM_SDEST()
{
	dest = NULL; 
	origin = NULL;
	assDemand = 0; 
	buffer = NULL;
	yPath = NULL;
	//exist = false;
	costDif = 100;
	shiftFlow =0.0;
}

TNM_SDEST::~TNM_SDEST()
{
	dest->attachedDest--;
	EmptyPathSet();
	if(buffer!=NULL) 
	{
		delete [] buffer;
	}
	//
	//for (int fi=0;fi<iteInfoVector.size();fi++)
	//{
	//	FWRoute* fwr = iteInfoVector[fi];
	//	delete fwr;
	//}
	//iteInfoVector.clear();
}

void TNM_SDEST::EmptyPathSet()
{
	for(PTRPATH pv = pathSet.begin(); pv!=pathSet.end(); pv++)
		delete *pv;
	//cout<<"terminatd a static dest object!"<<endl;
	pathSet.clear();
}

bool TNM_SDEST::Initialize(TNM_SNODE *org, TNM_SNODE *dt, floatType dmd)
{
	if(org == NULL) return false;
	if(dt  == NULL) return false;
	origin = org;
	dest   =   dt;
	dest->attachedDest++;
	assDemand = dmd;
	return true;

}

TNM_SNET::TNM_SNET(const string& netName)
{
	//m_regime      = RM_STATIC;
	networkName   =  netName;
	numOfNode     =        0;
	numOfLink     =        0;
	numOfOrigin   =		   0;
	numOfOD       =		   0;
	scanList      =     NULL;
	//ChooseSPAlgorithms(QUEUE);
	ChooseSPAlgorithms(DEQUE);  //initialize ScanList;
	buildStatus   = 0;
	initialStatus = 0;
	linkCostScalar = 1.0;
	//m_progressMessage = "TNM in progress";
	//m_progressIndicator = 0.0;
	centroids_blocked=false;
	timeCostCoefficient = 1.0;
	distCostCoefficient = 0.0;
}

TNM_SNET::~TNM_SNET()
{
	//cout<<"beging to delete tnm_snet object\n"<<endl;
	UnBuild();
	if(scanList) delete scanList;
	scanList = NULL;
}

void TNM_SNET::Reset()
{
	TNM_SLINK *link;
	UnInitialize();
	//EmptyPathSet();
	for (int i = 0;i<numOfLink;i++)
	{
		link             = linkVector[i];
		link->volume     = 0.0;
		link->markStatus = 0;
	}
	ClearLink2PathPtr();
	UpdateLinkCost();

}

void TNM_SNET::UpdateLinkCost(bool fToll)
{
	TNM_SLINK *link;
	for(int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		link->cost = link->GetCost(fToll);
	}

}

void TNM_SNET::UpdateLinkCostDer(bool fToll)
{
	TNM_SLINK *link;
	for(int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		link->fdCost = link->GetDerCost(fToll);
	}

}

void TNM_SNET::ClearLink2PathPtr()
{
	for (int i = 0;i<numOfLink;i++)
		linkVector[i]->pathInciPtr.clear();
}

int TNM_SNET::AllocateLinkBuffer(int size)
{
	TNM_SLINK *link;
	if (size <=0)
	{
		cout<<"\tInvalid size of link buffer array"<<endl;
		return 1;
     }
	else
	{
		linkBufferSize = size;
	}

	for (int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		if(link->buffer) delete [] link->buffer;
		link->buffer = new floatType[size];
		if(link->buffer == NULL)
		{
			cout<<"\tCannot allocate memory for link buffer!"<<endl;
			return 1;
		}
		for (int j = 0;j<size;j++)
			link->buffer[j] = 0.0;
	}
	return 0;
}

int TNM_SNET::AllocateNodeBuffer(int size)
{
	TNM_SNODE *node;
	if (size <=0)
	{
		cout<<"\tInvalid size of node buffer array"<<endl;
		return 1;
     }
	else
	{
		nodeBufferSize = size;
	}

	for (int i = 0;i<numOfNode;i++)
	{
		node = nodeVector[i];
		if(node->buffer) delete [] node->buffer;
		node->buffer = new floatType[size];
		if(node->buffer == NULL)
		{
			cout<<"\tCannot allocate memory for node buffer!"<<endl;
			return 1;
		}
			for (int j = 0;j<size;j++)
			node->buffer[j] = 0.0;
	}
	return 0;
}

int TNM_SNET::AllocatePathBuffer(int size)
{
	if (size <=0)
	{
		cout<<"\tInvalid size of path buffer array"<<endl;
		return 1;
     }
	else
	{
		pathBufferSize = size;
		TNM_SPATH::SetPathBufferSize(size);  
	}
	TNM_SORIGIN *org;
	TNM_SDEST *dest;
	PTRPATH pv;
    for (int i = 0;i<numOfOrigin;i++)
	{
		org = originVector[i];
		for (int j = 0;j<org->numOfDest;j++)
		{
			dest = org->destVector[j];
			for (pv = dest->pathSet.begin(); pv != dest->pathSet.end(); pv++)
			{
				if((*pv)->buffer) delete [] (*pv)->buffer;
				(*pv)->buffer = new floatType[size];
				if((*pv)->buffer == NULL)
				{
					cout<<"\tCannot allocate memory for link buffer!"<<endl;
					return 1;
				}
				for (int j = 0;j<size;j++)
					(*pv)->buffer[j] = 0.0;
			}
		}
	}
	return 0;
}

int TNM_SNET::AllocateDestBuffer(int size)
{
	if (size <=0)
	{
		cout<<"\tInvalid size of dest buffer array"<<endl;
		return 1;
     }
	else
	{
		destBufferSize = size;
	}
	TNM_SORIGIN *org;
	TNM_SDEST *dest;
	PTRPATH pv;
    for (int i = 0;i<numOfOrigin;i++)
	{
		org = originVector[i];
		for (int j = 0;j<org->numOfDest;j++)
		{
			dest = org->destVector[j];
			if(dest->buffer) delete [] dest->buffer;
			dest->buffer = new floatType[size];   
			if (dest->buffer == NULL)
			{
					cout<<"\tCannot allocate memory for dest buffer!"<<endl;
					return 1;
			}
			for (int k = 0;k<size;k++)
			 dest->buffer[k] = 0.0;
		}
	}
	return 0;
}


int TNM_SNET::ReleaseLinkBuffer()
{
	TNM_SLINK *link;
	for (int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		if(link->buffer != NULL)
		{
			delete[] link->buffer;
			link->buffer = NULL;
		}
	}
	linkBufferSize = 0;
	return 0;
}

int TNM_SNET::ReleaseNodeBuffer()
{
	TNM_SNODE *node;
	for (int i = 0;i<numOfNode;i++)
	{
		node = nodeVector[i];
		if(node->buffer != NULL)
		{
			delete[] node->buffer;
			node->buffer = NULL;
		}
	}
	nodeBufferSize = 0;
	return 0;
}

int TNM_SNET::ReleasePathBuffer()
{
	TNM_SORIGIN *org;
	TNM_SDEST *dest;
	PTRPATH pv;
    for (int i = 0;i<numOfOrigin;i++)
	{
		org = originVector[i];
		for (int j = 0;j<org->numOfDest;j++)
		{
			dest = org->destVector[j];
			for (pv = dest->pathSet.begin(); pv != dest->pathSet.end(); pv++)
			{
				if((*pv)->buffer != NULL)
				{
					delete[] (*pv)->buffer;
					(*pv)->buffer = NULL;
				}
			}
		}
	}
	pathBufferSize = 0;
	TNM_SPATH::SetPathBufferSize(0);
	return 0;
}

int TNM_SNET::ReleaseDestBuffer()
{
	TNM_SORIGIN *org;
	TNM_SDEST *dest;
    for (int i = 0;i<numOfOrigin;i++)
	{
		org = originVector[i];
		for (int j = 0;j<org->numOfDest;j++)
		{
			dest = org->destVector[j];
			if (dest->buffer != NULL)
			{
				delete[] dest->buffer;
				dest->buffer = NULL;
			}
		}
	}
	destBufferSize = 0;
	return 0;
}

void TNM_SNET::ChooseSPAlgorithms(DATASTRUCT type)
{
	switch (type)
	{
	case QUEUE:
		if(scanList != NULL) delete scanList;
		scanList  = new SCAN_QUEUE;
		break;
	case DEQUE:
		if (scanList != NULL) delete scanList;
		scanList = new SCAN_DEQUE;
		break;
	default:
		cout<<"Undefined data structure for shortest path search"<<endl;
		break;
	}

}

int TNM_SNET::UnBuild()
{
	//cout<<"\nUnBuilding network object..."<<endl;
	//if(CheckBuildStatus(false) ==0) return 0; //no neeed to do it should the net has not built yet.
	UnInitialize();
	vector<TNM_SORIGIN*>::iterator po;
	for (po = originVector.begin();po!=originVector.end();po++)
	{
		//cout<<"I am deleting origin "<<(*po)->origin->id_()<<endl;
		delete *po;				
	}
	vector<TNM_SLINK*>::iterator pl;
	for (pl = linkVector.begin(); pl!=linkVector.end(); pl++)
	{
		//cout<<"\tbegin to delete link "<<(*pl)->id<<" which is a "<<(*pl)->type<<endl;
	//	cout<<*linkVector[i]<<endl;
		//linkVector[i]->Print();
		delete *pl;
	}
	//cout<<"links are destroyed."<<endl;
	vector<TNM_SNODE *>::iterator pn;
	for (pn = nodeVector.begin(); pn!=nodeVector.end(); pn++)
	{
	//	cout<<"I am deleting node "<<(*pn)->id<<endl;
		delete *pn;
	}
	//cout<<"nodes are destroyed."<<endl;
	if(!linkVector.empty()) linkVector.clear();
	if(!nodeVector.empty()) nodeVector.clear();
	if(!originVector.empty()) originVector.clear();
	if(!destNodeVector.empty()) destNodeVector.clear();
	numOfNode     =        0;
	numOfLink     =        0;
	numOfOrigin   =		   0;
	numOfOD       =		   0;
	buildStatus   = 0;
	initialStatus = 0;
	return 0;
}

int TNM_SNET::UnInitialize()
 {
	 //if(CheckInitialStatus(false)==0) return 1;
	 EmptyPathSet();
	 initialStatus = 0; 
	 return 0;
 }

void TNM_SNET::EmptyPathSet()
{
	TNM_SORIGIN *origin;
	TNM_SDEST *dest;
	vector<TNM_SORIGIN*>::iterator po;
	for (po = originVector.begin();po!=originVector.end();po++)
	{
			origin =*po;
			for (int j = 0; j<origin->numOfDest;j++) {
				dest = origin->destVector[j];
				dest->EmptyPathSet();
			}
	}
}

void TNM_SNET::SetLinkCostScalar(floatType s)
{
	if(s<=0.01 || s> 99999)
	{
		cout<<"link cost scalar out of range"<<endl;
		linkCostScalar = 1.0;
	}
	linkCostScalar = s;
}

int TNM_SNET::CheckBuildStatus(bool noteBuilt)
{
	switch(buildStatus)
	{
	case 0:
		//if(noteBuilt) cout<<"\tNetwork "<<networkName<<" has not been built."<<endl;
		break;
	case 1:
		if(noteBuilt) cout<<"\tNetwork "<<networkName<<" has been built.  You are not allowed to rebuild it"<<endl;
		break;
	default:
		cout<<"\tUnknown build status!"<<endl;
	}
	return buildStatus; 
}

TNM_SLINK* TNM_SNET::CreateNewLink(const LINK_VALCONTAINER lval)
{
   // cout<<"creating a new link"<<endl;
	TNM_SLINK *link = AllocateNewLink(lval.type);
	if(link==NULL) return NULL;
    //cout<<"initailizing a new link "<<endl;
	if(!link->Initialize(lval))		
	{
		delete link;
		return NULL;
	}
	linkVector.push_back(link);
	link->orderID = linkVector.size();
	return link;
}

TNM_SLINK* TNM_SNET::AllocateNewLink(const TNM_LINKTYPE &pType)
{
	TNM_SLINK* link;
	switch(pType)
	{
		case BPRLK:
		{
			TNM_BPRLK *bprlk = new TNM_BPRLK;
			link             = (TNM_SLINK *) bprlk; //dynamic casting
			break;
		}
		//case CPBPR:
		//	{
		//	TNM_CPBPR *cpbpr = new TNM_CPBPR;
		//	link             = (TNM_SLINK *) cpbpr; //dynamic casting
		//	break;
		//	}
		//case ACHLK:
		//{
		//	TNM_ACHLK *achlk = new TNM_ACHLK;
		//	link             = (TNM_SLINK *) achlk;
		//	break;
		//}
		//case CSTLK:
		//	{
		//	TNM_CSTLK *cstlk = new TNM_CSTLK;
		//	link             = (TNM_SLINK *) cstlk;
		//	break;
		//	}
		//case LINLK:
		//	{
		//		TNM_LINLK *linlk = new TNM_LINLK;
		//		link = (TNM_SLINK*) linlk;
		//		break;				
		//	}
		//case EXPLK:
		//	{
		//		TNM_EXPLK *explk = new TNM_EXPLK;
		//		link = (TNM_SLINK*) explk;
		//		break;
		//	}
		default:
		//	cout<<"\n\tError: a "<<lval.type<<" link cannot be created. "<<endl;
			cout<<pType<<" is not a valid static link type"<<endl;
			return NULL;
	}
	return link;
}

TNM_SNODE* TNM_SNET::CreateNewNode(const NODE_VALCONTAINER nval)
{
	TNM_SNODE *node  = AllocateNewNode(nval.type);
	if(node == NULL) return NULL;
	if(!node->Initialize(nval) )
	{
		delete node;
		return NULL;
	}
	nodeVector.push_back(node);
//	numOfNode++;
	return node;
}

void TNM_SNET::ScaleDemand(floatType r)
{
	if(r< 1e-5 || r > 1e5) 
	{
		cout<<"\tDemand scalar should range between 1e-5 and 1e5! No scaling is done."<<endl;
		return;
	}
	//floatType dr = r;
	TNM_SORIGIN *org;
	TNM_SDEST   *dest;
	for (int i = 0;i<numOfOrigin;i++)
	{
		org = originVector[i];
		for (int j = 0;j<org->numOfDest;j++)
		{
			dest = org->destVector[j];
			dest->assDemand *= r;
		}
	}
}

void TNM_SNET::SetLinkTollType(TNM_TOLLTYPE tl)
{
	for(int i = 0;i<numOfLink;i++)
	{
		if(!linkVector[i]->dummy) //we do not allow you set toll type on dummy links. 
			linkVector[i]->SetTollType(tl); //override all link toll type.
	}
}

TNM_SNODE* TNM_SNET::AllocateNewNode(const TNM_NODETYPE &nType)
{
	TNM_SNODE *node;
	switch(nType)
	{
	case BASND:
		{
		TNM_SNODE *basnd = new TNM_SNODE;
		node = (TNM_SNODE*) basnd;
		break;
		}
	default:
		cout<<nType<<" is not a valid static node type"<<endl;
		return NULL;
	}
	return node;
}

int TNM_SNET::ClearZeroDemandOD()
{
	TNM_SORIGIN *org;
	vector<TNM_SORIGIN*>::iterator ov;
	for (ov = originVector.begin(); ov != originVector.end(); ov++)
	{
		org = *ov;
		org->LoadDestVector();
		org->DeleteZeroOD();
		org->UnLoadDestVector();
		if(org->numOfDest == 0)
			if(DeleteAnOrigin(org->id_())==0) ov --;
	}
	UpdateOriginNum();
	return 0;
}

int TNM_SNET::DeleteAnOrigin(int id)
{
	vector<TNM_SORIGIN *>::iterator pv;
	pv = find_if(originVector.begin(), originVector.end(), predP(&TNM_SORIGIN::id_, id));
	if(pv == originVector.end()) return 1;
	else
	{
		delete *pv;
		originVector.erase(pv);
	}
	numOfOrigin = originVector.size();
	return 0;
}

int TNM_SNET::BuildTAPAS(bool loadod, const TNM_LINKTYPE ltype)
{
	if(CheckBuildStatus(true)!=0) return 7;
	string netfilename, odfilename, nodefilename, tolfilename;
	NODE_VALCONTAINER nval;
	LINK_VALCONTAINER lval;
    netfilename = networkName + "_net.tntp";
	odfilename  = networkName + "_trips.tntp";
	nodefilename = networkName + "_node.tntp";
	tolfilename = networkName + "_tol.dat";
	int nn, nl, nz,nft;
	bool readNode = true;
	ifstream netFile, odFile, nodeFile,tolfile;
	if (!TNM_OpenInFile(netFile, netfilename))    return 1;
    if(loadod && !TNM_OpenInFile(odFile, odfilename))      return 2;
	if(!TNM_OpenInFile(nodeFile, nodefilename))
	{
		cout<<"\tCannot find node coordinates file, ignored!"<<endl;
		readNode = false;
	}

	/*Build node objects*/
	cout<<"\tReading "<<netfilename<<"..."<<endl;	
	TNM_SkipString(netFile, 3);
	netFile>>nz;
	TNM_SkipString(netFile, 3);
	netFile>>nn;
	TNM_SkipString(netFile, 3);
	netFile>>nft;
	TNM_SkipString(netFile, 3);
	netFile>>nl;
	string line;
	int tn;
	for(int i  = 0;i<nn;i++)
	{
		nval.type = BASND;
		nval.id = i + 1; 
		nval.dummy = false;
		if(CreateNewNode(nval) == NULL) return 3;
	}
	for(int i = 1;i<=nft-1 ;i++)
	{
		nodeVector[i-1]->m_isThrough = false;
	}


	/*Build link objects*/
	int tail, head;
	floatType cap, len, fft, t,ffs, B,P, spd;
	vector<string> words;
	int lid = 0;
	while(!netFile.eof())
	{
		getline(netFile, line);
		TNM_GetWordsFromLine(line, words);
		if(words.size() >=1)
		{
			if(words[0].find_first_of("~") == -1 && words.size() >= 10)
			{
				lid++;
				TNM_FromString<int>(tail, words[0], std::dec);
				TNM_FromString<int>(head, words[1], std::dec);
				TNM_FromString<floatType>(cap, words[2], std::dec);
				TNM_FromString<floatType>(len, words[3], std::dec);
				TNM_FromString<floatType>(fft, words[4], std::dec);
				TNM_FromString<floatType>(B, words[5], std::dec); 
				TNM_FromString<floatType>(P, words[6], std::dec);
				TNM_FromString<floatType>(spd, words[7], std::dec);
				lval.type     = ltype;
				lval.id       = lid;
				lval.dummy    = false;
				if(tail !=head) //if tail = head, the link is ignored.
				{
					lval.tail     = CatchNodePtr(tail, true);
					lval.head     = CatchNodePtr(head, true);
					
					if(fft> 0)
					{
						lval.length = len;
						lval.ffs    = lval.length * 60 /fft/linkCostScalar;
					}
					else
					{
						lval.length = len;
						lval.ffs    = 25.0;
					}
					
					//check if the link uses special types.
					TNM_LINKTYPE tltype;				
					if(words[9].size()>=4) //type string larger than
					{
						std::istringstream  str(words[9]); //read it into ltType.
						str>>tltype; //temparary link type
						lval.type = tltype;
					}
					//check if capacity is abnormal, often centriod connectors's capacity are set to 0.0. 
					if(cap <=1e-6)
					{
						cout<<"Warning: link "<<tail<<" - "<<head<<"'s capacity = "<<cap<<", its link performance function is forced to be constant."<<endl;					
						lval.type = ACHLK;
						B = 0.0;
						P = 0.0;
					}
					lval.par.clear();
					lval.par.push_back(B);
					lval.par.push_back(P);
					lval.capacity  = cap;	
					TNM_SLINK *link =CreateNewLink(lval);
					if(link == NULL) return 4;
					if(fft == 0) link->fft=fft;
					TNM_FromString<floatType>(link->cost, words[8], std::dec); //read in toll into cost, note that this is only temperary.
					TNM_FromString<floatType>(link->toll, words[8], std::dec);//now read toll into the permanet cost. 
					//set toll type.
					link->SetTollType(TT_NOTOLL); //inialize all to be no toll.  In most cases, toll type will be specified globablly when solving TAP.
					if(words.size() > 10 ) //allow you to specify a toll type here (note: maybe useful for small examples, in which case you can set toll type link by link. not typically used.
					{
						if(words[10].size() >= 4)
						{
							TNM_TOLLTYPE tt;
							std::istringstream  str(words[10]);
							str>>tt; //temparary link type
							link->SetTollType(tt);
						}
					}
					link->InitializeCostCoef(timeCostCoefficient,distCostCoefficient);
				}
				
			}
		}
	}

	if(readNode)
	{
		/*Read the coordinates of the node*/
		cout<<"\tReading "<<nodefilename<<", please wait..."<<endl;
		getline(nodeFile, line);
		//m_progressMessage = "Reading nodes...";
		for(int i = 0;i<nn;i++)
		{
			//m_progressIndicator = 1.0*i/nn;
			TNM_SNODE *node = nodeVector[i];
			floatType xcol, ycol;
			int id;
			nodeFile>>id>>xcol>>ycol;
			node->xCord = xcol ; //*52.8 is for philadolphia network.
			node->yCord = ycol;
		}
		nodeFile.close();
	}
	
	/*Build origin and destination objects*/
	if(loadod && nz >0)
	{
		cout<<"\tReading "<<odfilename<<", please wait..."<<endl;
		for(int i =0;i<3;i++) getline(odFile,line);
		string tStr;
		odFile>>tStr;
		//this is to pass potential comments lines.
		while(tStr.compare("Origin") !=0 && !odFile.eof())
		{
			odFile>>tStr;
		}
		
		int orgID, nd, destID;
		floatType dmd;
		TNM_SNODE* node;
		vector<int> dvec;
		vector<floatType> dmdvec;
		TNM_SORIGIN *pOrg;
		
		for(int i = 0;i<nz;i++)
		{
			odFile>>orgID;
			if((i+1)%100 ==0)
			{
				cout<<"\t"<<setw(4)<<100*i/nz<<"% completed"<<endl;
			}
		
			odFile>>tStr;
			nd = 0;
			if(!dvec.empty()) dvec.clear();
			if(!dmdvec.empty()) dmdvec.clear();
			while(tStr.compare("Origin") != 0 && !odFile.eof()) //not
			{
				if(TNM_FromString<int>(destID, tStr, std::dec))
				{
					odFile>>tStr;
					odFile>>tStr;
					TNM_FromString<floatType>(dmd, tStr, std::dec);
					odFile>>tStr;
					if(tStr == ";") odFile>>tStr; // test if ; has an space before it.
					dvec.push_back(destID);
					dmdvec.push_back(dmd);
					nd++;
				}
				else
				{
					cout<<"destID = "<<destID<<endl;
					cout<<"OD trip file includes unrecognized format!"<<endl;
					return 5;
				}
			}
			
			if((pOrg = CreateSOrigin(orgID, nd))==NULL) 
			{
				cout<<"cannot create static origin object!"<<endl;
				return 6;
			}
			pOrg->m_tdmd = 0.0;
			pOrg->origin->is_centroid=true;
			if(centroids_blocked) pOrg->origin->SkipCentroid=true;
			for(int j = 0;j<nd;j++)
			{
				node = CatchNodePtr(dvec[j], true);
				pOrg->SetDest(j + 1, node, dmdvec[j]);
				pOrg->m_tdmd += dmdvec[j];
				node->is_centroid=true;
				if(centroids_blocked) node->SkipCentroid=true;
			}
			
		}
	}
	
	UpdateLinkNum();
	UpdateNodeNum();
	if(loadod) UpdateOriginNum();
	else       numOfOrigin = nz;
	buildStatus = 1;

	if (TNM_OpenInFile(tolfile, tolfilename)) 
	{
		cout<<"\tReading toll information from "<<tolfilename<<endl;
		for(int i = 0;i<numOfLink;i++) linkVector[i]->toll = 0.0;
		int ntl, ncount;
		floatType lowb, uppb;
		tolfile>>ntl>>lowb>>uppb;
		for(int i = 0;i<ntl;i++)
		{
			int from, to;
			floatType toll;
			tolfile>>from>>to>>toll;
			TNM_SNODE *fromnode, *tonode;
			if(from >0 && from<= numOfNode && to > 0 && to <= numOfNode)
			{
			
				fromnode = CatchNodePtr(from, true);
				tonode = CatchNodePtr(to, true);
				if(from  && to) 
				{
							
					TNM_SLINK *link = CatchLinkPtr(fromnode, tonode);
					if(link)
					{
						if(toll<lowb)
						{
							link->toll = lowb;
						}
						else if (toll > uppb)
						{
							link->toll = uppb;
						}
						else
						{
							link->toll = toll;
							//ncount++;
						}

					}
					else
					{
						cout<<"\tCannot locate link "<<from<<" - "<<to<<endl;
					}
				}
				

			}
			else
			{
				cout<<"\tCannot locate link "<<from<<" - "<<to<<endl;
			}
		}
		tolfile.close();
	}
	return 0;
}

TNM_SLINK* TNM_SNET::CatchLinkPtr(TNM_SNODE* tail, TNM_SNODE *head)
{
	for (PTRTRACE pl = tail->forwStar.begin(); pl!=tail->forwStar.end(); pl++)
	{
			if((*pl))
			{
				if((*pl)->head == head)
					return *pl;
			}
	}
	return NULL;
}

void TNM_SNET::UpdateOriginNum()
{
	numOfOrigin = originVector.size();
	numOfOD = 0;
	for(int i = 0;i<numOfOrigin;i++)
	{
		numOfOD += originVector[i]->numOfDest;
	}
	for (int i = 0;i<numOfNode;i++)
		if(nodeVector[i]->attachedDest >0) destNodeVector.push_back(nodeVector[i]);
}

TNM_SORIGIN* TNM_SNET::CreateSOrigin(int nodeID, int nd)
{
	TNM_SNODE *node = CatchNodePtr(nodeID);
	if(node == NULL)
	{
		cout<<"\n\tnode "<<nodeID<<" is not a valid node object. "<<endl;
		return NULL;
	}
	else return CreateSOriginP(node, nd);
	
}

TNM_SORIGIN* TNM_SNET::CreateSOriginP(TNM_SNODE* node, int nd)
{
	if (nd <0)
	{
		cout<<"\n\tOrigin "<<node->id<<" contains none or negative destinations."<<endl;
		return NULL;
	}
	TNM_SORIGIN *org = new TNM_SORIGIN(node, nd);
	if (org == NULL)
	{
		cout<<"\n\tCannot allocate memory for new origin"<<endl;
		return NULL;
	}
	originVector.push_back(org);
	numOfOrigin ++;
	numOfOD+=nd;
	return org;
}

TNM_SNODE * TNM_SNET::CatchNodePtr(int id, bool safe)
{
	/*if(nodeVector[id - 1]->id == id) return nodeVector[id-1];
	else*/
	if(safe) return nodeVector[id - 1];
	else
	{
		vector<TNM_SNODE *>::iterator pv;
		pv = find_if(nodeVector.begin(), nodeVector.end(), predP(&TNM_SNODE::id_, id));
		if(pv == nodeVector.end()) return NULL;
		else                       return *pv;
	}
}

//Shortest path calculaton provided address of a given origin
void TNM_SNET::UpdateSP(TNM_SNODE *rootNode)
{
	TNM_SNODE *node;
//initialize node for shortest path calculation
	for (int j = 0;j<numOfNode;j++)
	{
		node = nodeVector[j];
		node->InitPathElem();            
		node->scanStatus = 0;
	}
//call scanList 's major method to compute shortest path
	if(centroids_blocked) rootNode->SkipCentroid=false;
	scanList->SPTreeO(rootNode);//compute shortest path;
	if(centroids_blocked) rootNode->SkipCentroid=true;
}

void TNM_SNET::InitialSubNet3(int kx, bool createPath)
{
	TNM_SORIGIN *origin;
	for (int i = 0;i<numOfOrigin; i++)
	{
		origin = originVector[i];
		
		UpdateSP(origin->origin);
		
		//add path
		{
		
			for (int j=0; j<origin->numOfDest; j++)
			{
				TNM_SDEST* dest = origin->destVector[j];
				if(origin->origin == dest->dest) continue;
				//test
				//cout<<"For dest "<<dest->dest->id<<endl;
				//add iteration 0
				
				//
				double dmd = dest->assDemand;
				TNM_SPATH* path= new TNM_SPATH;
				path->path.clear();
				TNM_SNODE* snode = NULL;
				TNM_SLINK* slink = NULL;
				snode = dest->dest;
				while(snode != origin->origin)
				{
					slink = snode->pathElem->via;
					slink->volume+=dmd;
					if(slink == NULL) 
					{
						cout<<"The link is null in calculating the direction for OFW algorithm"<<endl;
						break;
					}
					path->path.push_back(slink);
					//test
					//cout<<slink->tail->id<<"-->"<<slink->head->id<<endl;
					//
					snode = slink->tail;
					
					
				}
				//
				path->flow = dest->assDemand;
				dest->pathSet.push_back(path);
				//path->Print(true);
				//system("PAUSE");
			}
		}
		//getchar();
		//cout<<"1.2"<<endl;
	}

}

void SCANLIST::SPTreeO(TNM_SNODE * rootNode)//given root, perform shortest path computation.
{

//insert the root into scanList, whatever data strucrue it uses.
	TNM_SNODE *curNode;
	InitRoot(rootNode);
	InsertANode(rootNode);
//	PrintList();
	curNode = GetNextNode();
	while (curNode != NULL)
			{
				if(curNode->m_isThrough || curNode == rootNode) curNode->SearchMinOutLink(this);
				curNode = GetNextNode();
			}//end while	
}

void SCANLIST::SPTreeD(TNM_SNODE *rootNode)
{
	TNM_SNODE *curNode;
	//rootNode->tmpPathElem->cost = 0;
	InitRoot(rootNode);
	InsertANode(rootNode);
//	PrintList();
	curNode = GetNextNode();
	while (curNode != NULL)
	{
		//cout<<"Scan node "<<curNode->id<<endl;
		if(curNode->m_isThrough || curNode == rootNode) curNode->SearchMinInLink(this);
		/*else
		{
			cout<<"node "<<curNode->id<<" is neither a through node, or a root node "<<endl;
		}
	*/	curNode = GetNextNode();
	//	getchar();
	}//end while	
}

void SCANLIST::InitRoot(TNM_SNODE *root) 
{
	root->pathElem->cost = 0;
}

/*===========================================================================================
                              DEQUE 
  ===========================================================================================*/

SCAN_DEQUE::SCAN_DEQUE()
{
/*	status = new tinyInt[numOfNode];
	for (int i = 0;i<numOfNode;i++)
		status[i] = 0;*/

}

void SCAN_DEQUE::ClearList()
{
	nodeList.clear();
}

bool SCAN_DEQUE::InsertANode(TNM_SNODE *node)
{
	if(!node->SkipCentroid)
	{
	if(node->scanStatus == 0) // if never been used, insert it to the back of dq
	{
		nodeList.push_back(node); 
	    node->scanStatus = 1; // now being used
		return true;
	}
    else if (node->scanStatus == -1) // if it has ever been used, insert it to the front of dq
	{
		nodeList.push_front(node); 
	    node->scanStatus = 1; // now being used
		return true;
	}
	}
	return false;
}

TNM_SNODE *SCAN_DEQUE::GetNextNode()
{
   TNM_SNODE *node;
   if (!nodeList.empty()) //if the list is not empty
   {
   node = nodeList.front(); // get the current node
   nodeList.pop_front(); //delete it from the deque;
   node->scanStatus = -1; //mark it as been used but not in queue right now.
   return node;
   }
   else
	   return NULL;
}

SCAN_DEQUE::~SCAN_DEQUE()
{
	if(!nodeList.empty()) nodeList.clear();
	//cout<<"delete SCAN_DEQUE"<<endl;
}

void SCAN_DEQUE::PrintList()
{
	cout<<"Print List is under construction"<<endl;
}


/*===========================================================================================
                              QUEUE 
  ===========================================================================================*/

SCAN_QUEUE::SCAN_QUEUE()
{
/*	status = new tinyInt[numOfNode];
	for (int i = 0;i<numOfNode;i++)
		status[i] = 0;*/

}

void SCAN_QUEUE::ClearList()
{
	while(!nodeList.empty()) nodeList.pop();
}
bool SCAN_QUEUE::InsertANode(TNM_SNODE *node)
{
      if(node->scanStatus !=1)   
	  {
		  nodeList.push(node); 
		  node->scanStatus = 1;
		  return true;
	  }
	  else return false;
}

TNM_SNODE *SCAN_QUEUE::GetNextNode()
{
   TNM_SNODE *node;
   if (!nodeList.empty()) //if the list is not empty
   {
   node = nodeList.front(); // get the current node
   node->scanStatus = 0;
   nodeList.pop(); //delete it from the deque;
   return node;
   }
   else
	   return NULL;
}
SCAN_QUEUE::~SCAN_QUEUE()
{
	//nodeList.clear();
}
void SCAN_QUEUE::PrintList()
{
  cout<<"Print List is under costruction."<<endl;
}