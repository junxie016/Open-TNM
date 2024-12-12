//#pragma once
#include "head.h"
//#include "TNM_utility.h"
#include "My_Predicate.h"
//#include "nlopt.h"
//#include "lp_lib.h"

#include "..\trafficNet\stdafx.h"
//#include "..\..\include\tnm\TNM_utility.h"
//#include "..\..\..\include\tnm\TNM_utility.cpp"

//
//#pragma comment(lib, "liblpsolve55d")
//#pragma comment(lib, "liblpsolve55")
//#pragma comment(lib, "lpsolve55")


class PTNet_API PTRoute
{
public:
	PTRoute()
    {
		m_patternid = "unknown";
        m_id = "unknown";
        m_sname = "0";
        m_lname = "unknown";
        m_type  = -1;
    };
	string m_id;
	string m_sname;
	string m_lname;
	string m_patternid;
	int m_type;
};

class PTNode;
class PTLink;
class PTNET;
class GNODE;
class GLINK;
class PTShape;
typedef map<string, PTNode*, less<string> > PTNodeMap;//key is the ID of the route that node is associated with. 
typedef map<string, PTNode*, less<string> >::iterator PTNodeMapIter;
typedef multimap<long, int, std::less<long> > COORDMAP;
typedef COORDMAP::iterator COORDMAP_I; //iterator





class PTNet_API PTStop
{
public:
	PTStop(){;};

	bool		TAPInitialize(const string &inf,bool isPos = false);
	PTNode*		GetTransferNode() {return m_transferNode;};
	double		GetLat(){return m_pos.GetLatitude();};
    double		GetLon(){return m_pos.GetLongitude();};
	void        SetTransferNode(PTNode* nd) {m_transferNode = nd;}
	bool		AddNode(const string &shapeid, PTNode* node)
	{
		if(!node) return false;
		else
		{
			pair<PTNodeMapIter, bool> ret = m_nodes.insert(PTNodeMap::value_type(shapeid, node));
			return ret.second;
		}
	
	};
	PTNode*     GetNode(const string &shapeid)
	{
		PTNodeMapIter pi = m_nodes.find(shapeid);
		if(pi == m_nodes.end()) return NULL;
		else return pi->second;	
	};
	

	PTNode*		m_transferNode;//this node is initialized after CreateTransferNode. 
	PTNodeMap	m_nodes; //all nodes belong to this stop 
	string		m_id;
	string		m_name;
	TNM_Position m_pos;
	vector<PTShape*> mshapes;
};


class PTNet_API PTShape
{
public: 
	PTShape()
	{
		m_id = "-1";
		m_routePtr = NULL;
	};

	void   SetRoute(PTRoute* rt) {m_routePtr = rt;}
	bool   AddStop(PTStop* stop)
	{
		if(!stop) return false;
		else
		{
			m_stops.push_back(stop);
			return true;
		}
	}
	PTRoute*        m_routePtr;
	vector<PTStop*> m_stops;
	vector<PTLink*> m_boardlinks;//creater for capacitated assignment
	vector<PTLink*> m_alightlinks;
	string			m_id; 
	long double			m_cap;//capacity of this line
	double			m_freq;//frequency of this line

};




struct PTNet_API StgLinks // strategic links, not create new glink on this strategy, save creating time for glink,
{
	StgLinks(string name, floatType w, vector<int> ls,vector<floatType> lp, floatType c)
	{
		sname = name;
		waitT = w;
		StgLinkPosVec = ls;
		StgLinkProbVec = lp;
		cost = c;
		buffer = NULL;
		isboarding = false;
	}

	StgLinks()
	{
		approach=	0.0;
		sflow	=	0.0;
		waitT	=	0.0;
		cost	=	0.0;
		sname	=	"";
		buffer = NULL;
		isboarding = false;
	}

	StgLinks(int buffersize)
	{
		approach=	0.0;
		sflow	=	0.0;
		waitT	=	0.0;
		cost	=	0.0;
		sname	=	"";
		isboarding = false;
		buffer = new floatType[buffersize];
		for (int j = 0;j<buffersize;j++)	buffer[j] = 0.0;
	}

	void SetLinkProbVec(vector<floatType> v)
	{
		StgLinkProbVec = v;
		numofstglinks = v.size();
	}

	floatType				*buffer;    //this data is added as a working space for outside using.
	floatType				cost;
	string					sname;
	int						numofstglinks;
	bool					isboarding;
	floatType				waitT;//waiting time of strategy
	floatType				sflow;//strategy flow
	floatType				approach;//the ratio of the flow
	vector<int>				StgLinkPosVec;// just save the link position in linkvector
	vector<floatType>		StgLinkProbVec;// approach prob
	void  print()
	{
		cout<<"stg:"<<sname<<", flow:"<<sflow<<", wait delay="<<waitT<<endl;
	}
};

/*
struct PTNet_API StgLinks // strategic links
{
	StgLinks()
	{	
		approach=	0.0;
		sflow	=	0.0;
		waitT	=	0.0;
		cost	=	0.0;
		sname	=	"";
		StgLinkVia = NULL;
	};
	
	floatType				approach;//the ratio of the flow
	floatType				sflow;//strategy flow
	floatType				waitT;//waiting time of strategy
	string					sname;
	floatType				cost;
	GLINK*					StgLinkVia;

	void  print()
	{
		cout<<"stg:"<<sname<<", flow:"<<sflow<<", wait delay="<<waitT<<endl;
	}
};
*/

typedef vector<PTLink*>::iterator PTRTRACE;

struct PTNet_API HYPERPATHELEM
{
	HYPERPATHELEM()
	{
		 cost = 0.0;
		 via = NULL;
	}
	floatType				cost;
	StgLinks*				via; // this is created for built strategy info
	PTLink*					vialink;// this is created for link pointer on shortest hyperapth, not necessarily create a strategy
};


class PTNet_API PTNode
{
public:
	PTNode()
	{
		id = -1;
		trafficnetid = -1;
		xCord = 0;
		yCord = 0;
		scanStatus = 0;
		tmpNumOfIn = 0;
		m_tmpdata = 0.0;
		m_wait = 0.0;//this is create for storing waiting delay at transfernodes
		StgElem   = new HYPERPATHELEM;
		rStgElem  = new HYPERPATHELEM;
		//eStgElem  = new HYPERPATHELEM;
		m_stgNode = NULL;

		buffer = NULL;		
	};
	enum TNTYPE {TRANSFER, ENROUTE, DWELL, CENTROID};
	int	NumOfDestOutLink();
	int	NumOfBushOutLink();

	void		 CleanStgLinksOnHyperPath();

	void         CleanStgLinks();
	void		 SetTransitNodeType(TNTYPE t = ENROUTE) {m_tnType = t;}
	TNTYPE		 GetTransitNodeType() {return m_tnType;}

	void		 SetStopPtr(PTStop *st) {m_stop = st;}
	PTStop*		 GetStopPtr() {return m_stop;}
	void		 SetShapePtr(PTShape *sp) {m_shape = sp;}
	void		 SetRoutePtr(PTRoute *sh) {m_route = sh;}
	double		 MeasureDist(PTNode *node)
	{
		double tx, ty;
		tx = node->xCord - xCord;
		ty = node->yCord - yCord;
		return sqrt(tx * tx + ty * ty);
	}
	inline int			id_() {return id;} 
	inline int			trafficid_() {return trafficnetid;} 

	int				id;
	int				trafficnetid; // traffic net id
	int             tmpNumOfIn; //Used in topological computation to store the incoming links
	vector<PTLink*> forwStar;   //All outgoing link pointers
	vector<PTLink*> backStar;   //All incoming link pointers
	PTShape*		m_shape;
	PTRoute*		m_route;
	PTStop*			m_stop;
	floatType		xCord;
	floatType		yCord;
	floatType		m_tmpdata;
	floatType		m_wait;
	vector<floatType> m_attProb; //m_attProb[i] 记录（在吸引集R中）从节点（为上车弧的尾节点transfernode）出发的第i条上车弧所属线路l(l∊R)的车辆最先达到的概率Π[l]=freq[l]/sum(freq[k]) k∊R
	int				scanStatus;
	floatType		*buffer;  //this is a solution variable intended for temporary use. 
	TNTYPE			m_tnType; //transfer or not;
	GNODE			*m_stgNode;
	
	HYPERPATHELEM           *StgElem;  //Store the optimal routing policy. 存储最佳路径选择策略
	HYPERPATHELEM           *rStgElem;  //Store the optimal routing policy.
	//HYPERPATHELEM           *eStgElem;  //Store the efficient routing policy.
	//StgLinks*				m_StgVia;//shortest tree topology
	//StgLinks*				m_rStgVia;//longest tree topology
	//floatType				m_cost;// shortest strategy cost from the tail node of the stg to destination
	//floatType				m_rcost;// longest strategy cost from the tail node of the stg to destination
};



class PTNet_API PTLink
{
public:
	PTLink(int i, PTNode* t, PTNode* h)
	{
		id = i;
		tail = t;
		head = h;
		tail->forwStar.push_back(this);
		fID = tail->forwStar.size() - 1;
		head->backStar.push_back(this);
		volume = 0.0;
		volumet = 0.0; //用于统计换乘次数
		cost = 0.0;
		rLink = NULL;
		stglinkptr = NULL;
		pfdcost = 0;
		rpfdcost = 0.0;
		length = 0;
		fft = 0;
		buffer = NULL;
		markStatus = 0;
		m_probdata = 0;
		effpower = 0.2;
		freq = POS_INF_FLOAT;
		m_stglink = NULL;
		odwalk = false;
	
	};
	enum TLTYPE {ALIGHT, ABOARD, ENROUTE, DWELL, WALK, FAILWALK}; // fail walking is created for 
	enum TLSYM {SYMMTRIC,ASYMMTRIC};
	enum CAPTYPE{TL, BL};


	void		SetTransitLinkType(TLTYPE t = ENROUTE) {m_tlType = t;};
	void		SetTransitLinkSym(TLSYM t) {m_tlSym = t;};
	void     SetCapType(CAPTYPE t){m_tlCap=t;};

	TLTYPE		GetTransitLinkType() {return m_tlType;};
	string		GetTransitLinkTypeName() 
	{
		switch(m_tlType)
		{
			case TLTYPE::ABOARD:
				return "ABOARD";
				break;
			case TLTYPE::ALIGHT:
				return "ALIGHT";
				break;
			case TLTYPE::ENROUTE:
				return "ENROUTE";
				break;
			case TLTYPE::DWELL:
				return "DWELL";
				break;
			case TLTYPE::WALK:
				return "WALK";
				break;		
		}
	}
	TLSYM		GetTransitLinkSymmetry() {return m_tlSym;};
	CAPTYPE		GetTransitLinkCapType() {return m_tlCap;};
	void		UpdatePTLinkCost();
	void		UpdatePTDerLinkCost();
    floatType	GetPTLinkCost();
	floatType	GetPTLinkCostCAP();
	floatType	GetPTLinkDerCost();
	inline int			id_() {return id;} 
	
	// add for effective frequency-based TEAP
	void				UpdateEffectiveFreq();


	int					id;
	int					fID;// this stored the order of forward links at the tail node
	int					seq;// it indicate the sequence number of a line, for a boarding link
	PTShape*			m_shape;

	PTNode*				tail;
	PTNode*				head;
	floatType			tmpuse; // for temp use
	floatType           m_probdata;// for storing link probability, for temorporary storage on shortest hyperapath tree
	floatType           m_hwmean; // headway value
	vector<floatType>	pars;// parameters to determine link cost
	floatType			pfdcost;//first derivative link cost respective to itself
	floatType			rpfdcost;//first derivative link cost respective to the related link
	floatType			cost;//link cost
	
	floatType			volume;//link flow
	floatType			volumet; //用于统计换乘次数
	floatType			length;//km
	floatType			fft;//free flow time
	floatType			*buffer;  //this is a solution variable intended for temporary use. 
	floatType			*bufferData; 
	tinyInt				markStatus; // tag 1 for arcs on the shortest path
	floatType			SCvolume;//simplex combination link flow, only called in SD
	PTLink*				rLink;//cost related link pointer
	PTLink*				stglinkptr;   //pointer toward the link on the strategy
	TLTYPE				m_tlType; //transit link type
	TLSYM				m_tlSym; //indicate whether link cost is separable 
	CAPTYPE				m_tlCap; //indicate the capacitated link type

	GLINK*				m_stglink;

	//========capacity-related  attribute========
	floatType			g_cost;	//generalized cost
	floatType			g_pfdcost;//first generalized derivative link cost respective to itself
	floatType			g_rpfdcost;//first generalized derivative link cost respective to the related link
	floatType			cap;	//capacity
	floatType			mu;		//multiplier
	floatType			z;		//slack variable

	
	floatType			freq;
	floatType			efffreq;
	floatType			effpower;
	bool				odwalk;

	void		TL_UpdateGeneralPTLink_Cost_DerCost(floatType lambda);
	void		BL_UpdateGeneralPTLink_Cost_DerCost(floatType lambda);
	void		UpdateGeneralPTLink_Cost_DerCost(floatType lambda);
	void		UpdateGeneralPTLink_Cost_DerCost_Link(floatType lambda);
	void		IPFUpdatePTLinkCost(floatType netgamma);
	void		IPFUpdatePTDerLinkCost();
};


class PTNet_API GNODE
{
public:
	GNODE(PTNode* s)
	{
		m_ptnodePtr = s;
		m_data = 0.0;
		//m_wait = 0.0;
		m_stgname = "";
		m_ptnodeLCNPtr = NULL;
		m_tpLevel = 0;
		status = 0;
	}

	inline  StgLinks* SearchStgbyName(string name)
	{	
		for (vector<StgLinks*>::iterator it =m_StgsVec.begin();it != m_StgsVec.end();it++)
		{
			if ((*it)->sname == name)
			{			
				return *it;
			}
		}
		return NULL;
	};

	inline StgLinks* SearchBoardStgByContainLink(int fid)
	{
		for (vector<StgLinks*>::iterator it =m_StgsVec.begin();it != m_StgsVec.end();it++)
		{
			if ((*it)->isboarding)
			{
				vector<int>::iterator fit = find((*it)->StgLinkPosVec.begin(), (*it)->StgLinkPosVec.end(),fid);

				if (fit != (*it)->StgLinkPosVec.end())
					return (*it); 
			}		
		}
		return NULL;
	
	}

	void					UpdateOutBoardingLam();
	void					UpdateOutBoardingLamII();
	void					UpdateOutBoardingLamIII();

	inline int			id_() {return m_ptnodePtr->id;} 

	void					UpdateLinkProbability(floatType timescaler = 60);

	PTNode*					m_ptnodePtr;
	bool					m_mutliStart;//multi outgoing links
	bool					m_mutliStg; //multi stgs
	int						m_tpLevel; //topological level;
	PTNode*					m_ptnodeLCNPtr;//last common node pointer;
	floatType				m_data;// prob on a hyperpath
	string					m_stgname;
	vector<StgLinks*>		m_StgsVec;//store stgs
	StgLinks*               shortestvia;
	int						status;
	floatType				minShiftflow;
	floatType				maxShiftflow;
	//floatType				m_cost;// shortest strategy cost from the tail node of the stg to destination
	//floatType				m_rcost;// longest strategy cost from the tail node of the stg to destination

	floatType		prediff;
	floatType		afterdiff;
};



class PTNet_API GLINK
{
public:
	GLINK(PTLink *link, GNODE* tail, GNODE* head ) // this construction is for bush-based alg
	{
		m_linkPtr = link; 
		m_head = head; 
		m_tail = tail; 
		m_data =0.0;
		m_prob = 0.0;
		revStgLink = NULL;
		buffer = NULL;
		volume = 0;
		lam = 0;
	}

	GLINK(PTLink *link)
	{
		m_linkPtr = link; 
		m_data =0.0;
		m_prob = 0.0;
		revStgLink = NULL;
		volume = 0;
		lam = 0;
	}
	inline int	id_() {return m_linkPtr->id;} 
	void AllocateBuffer(int n)
	{
	
	
	}
	GLINK*		revStgLink;
	PTLink*		m_linkPtr;
    GNODE*		m_head;
    GNODE*		m_tail;
    floatType	m_data;// prob on a hyperpath
	floatType   m_prob;//approach probability
	floatType	volume;//store destination-based link flow
	floatType   *buffer;
	floatType	lam;//add for additional cost on boarding links
};

typedef struct finder_t
{
   finder_t(int n) : id(n) { }

	bool operator()(GLINK *p)
	{ 
		return (id == p->m_linkPtr->id); 
	}

	int id;
 } finder_t;





class PTNET;
class PTNet_API TNM_HyperPath
{
public:
	TNM_HyperPath() {flow = 0.0;Preflow=0.0;cost=0.0;fdcost=0.0;WaitCost = 0.0;name=""; }

	floatType    flow;
	floatType    cost;
	floatType	 fdcost;//first derivative cost 
	floatType	 WaitCost;
	floatType	 Preflow;
	floatType    s1;
	floatType    s2;
	string		 name;
	inline vector<GLINK*> GetGlinks() {return m_links;}
	void AddGlinks(PTNode* node) //已知节点i，更新超路径经过的弧(i,j)、计算在超路径k上使用弧(i,j)的概率Π[i,j]，总期望等待时间WaitCost
	{
		PTLink* link = node->StgElem->vialink; //link = 节点i的vialink(i,j)
		WaitCost += node->m_tmpdata * node->m_wait; //超路径k的总期望等待时间=sum(每一个节点的经过概率v[i]*吸引集R的期望等待时间)
		int i = 0;
		while (link)
		{
			GLINK* newglink = new GLINK(link);
			name += std::to_string(link->id) + "-";
			newglink->m_data = node->m_tmpdata * node->m_attProb[i];//超路径k上使用弧(i,j)的概率Π[i,j]=在超路径k上经过节点i的概率v[i]*在节点i处乘客使用弧(i,j)的概率e[i,j]
			newglink->m_prob = node->m_attProb[i]; //在节点i处乘客使用弧(i,j)的概率e[i,j]
			m_links.push_back(newglink); //其中m_data记录超路径k上使用弧(i,j)的概率Π[i,j]，m_prob记录在节点i处乘客使用弧(i,j)的概率e[i,j]
			link = link->stglinkptr; //对于换乘节点的上车弧，存在吸引集中的其他弧
			i++;
		}
		
	}

	/*
	void  AddNodeLinks(PTNode* node)
	{
		GNODE *pnode = new GNODE(node);
		m_nodes.push_back(pnode);
		pnode->m_data = node->m_tmpdata;
		StgLinks* stg = node->m_StgVia;
		if (stg)
		{
			//pnode->m_wait = stg->waitT;
			pnode->m_stgname = stg->sname;		
			WaitCost += pnode->m_data * stg->waitT;

			GLINK* glink = stg->StgLinkVia;
			while (glink)
			{
				GLINK* newglink = new GLINK(glink->m_linkPtr);
				newglink->m_prob = glink->m_prob;
				newglink->m_data = pnode->m_data * glink->m_prob;
				m_links.push_back(newglink);
				glink = glink->revStgLink;
		
			}
		}
	};
	*/


	bool IsSameHyperpath(TNM_HyperPath* cpath)
	{
		if (m_nodes.size()!=cpath->m_nodes.size()) return  false;
		else
		{
			for (int i=0;i<m_nodes.size();i++)
			{
				if (m_nodes[i]->m_stgname != cpath->m_nodes[i]->m_stgname)
				return  false;
			}		
		}
		return true;
	}

	void CTEAPIPFUpdateHyperpathCost(floatType gamma)
	{
		
		cost = WaitCost;
		//cout<<"WaitCost="<<WaitCost<<endl;
		fdcost = 0.0;
		//cout<<"path fdcost = ";
		for(int i=0;i<m_links.size();++i) //遍历该超路径路段集合m_links的各条路段link
		{
			PTLink* link =   m_links[i]->m_linkPtr;
			//cout<<"[id"<<link->id<<"]";
			cost += m_links[i]->m_data * link ->cost; //计算该超路径k的费用Ck=所有弧上的期望等待费用+经过所有换乘节点的总期望等待时间WaitCost
			fdcost += m_links[i]->m_data * link ->pfdcost * m_links[i]->m_data;	 //超路径费用对自身流量的一阶导fdcost部分
			//cout<<"(pro"<<m_links[i]->m_data<<"*"<<"pfd"<< link ->pfdcost<<"*"<<"pro"<<m_links[i]->m_data<<") + ";
			if (link->rLink && link->GetTransitLinkSymmetry() == PTLink::ASYMMTRIC)  //若link(i->j)为上车弧（或行驶弧），rlink为对应的行驶弧(尾节点j)（或上车弧(头节点i)）
			{
				vector<GLINK*>::iterator it = find_if(m_links.begin(), m_links.end(), finder_t(link->rLink->id)); 			
				if (it != m_links.end())
				{
					fdcost += m_links[i]->m_data *  link->rpfdcost * (*it)->m_data; //对对应行驶弧（或上车弧）流量求导的rfdcost部分
					//cout<<"(pro"<<m_links[i]->m_data<<"*"<<"rpfd"<<link->rpfdcost <<"*"<<"rpro"<< (*it)->m_data<<") + ";
				}
			}
			if (  link->GetTransitLinkType() == PTLink::ENROUTE )
			{
				fdcost +=  m_links[i]->m_data * gamma/pow((link->buffer[2]-link->volume),2)  * m_links[i]->m_data;
				//cout<<"(pro"<<m_links[i]->m_data<<"*"<<"pfdpenalty"<<gamma/pow((link->buffer[2]-link->volume),2) <<"*"<<"pro"<<m_links[i]->m_data<<") + ";		
			}
			if (fdcost<0) 
			{
				cout<<"Warning! The derivative cost for path:"<<name<<"is negtive:"<<fdcost<<endl;
				system("PAUSE");
			}

		}
		//cout<<"="<<fdcost<<endl;
	}

	void UpdateHyperpathCost() //计算超路径费用和费用一阶导
	{
		cost = WaitCost;
		fdcost = 0.0;
		for(int i=0;i<m_links.size();++i) //遍历该超路径路段集合m_links的各条路段link
		{
			PTLink* link =   m_links[i]->m_linkPtr;
			cost += m_links[i]->m_data * link ->cost; //计算该超路径k的费用Ck=所有弧上的期望等待费用+经过所有换乘节点的总期望等待时间WaitCost
			fdcost += m_links[i]->m_data * link ->pfdcost * m_links[i]->m_data;	    //????? 超路径费用对流量的一阶导fdcost
			if (  link->rLink &&  link->GetTransitLinkSymmetry()==PTLink::ASYMMTRIC&&link->rLink  ) //若link(i->j)为上车弧（或行驶弧），rlink为对应的行驶弧(尾节点j)（或上车弧(头节点i)）
			{
				//vector<GLINK*>::iterator it = find_if(m_links.begin(), m_links.end(), predP(&GLINK::id_, link->rLink->id)); 
				vector<GLINK*>::iterator it = find_if(m_links.begin(), m_links.end(), finder_t(link->rLink->id)); 	//在该超路径路段集合m_links中查找link对应的rlink，并使用it指向rlink
				if (it != m_links.end()) //if rlink不是m_links最后一项
				{
					fdcost += m_links[i]->m_data *  link->rpfdcost * (*it)->m_data; //????? 超路径费用一阶导
				}
			}
		}
	}
	void BL_UpdateCapHyperpathCost(floatType lambda);
	
	void UpdateCapHyperpathCost(floatType lambda);

	void TL_UpdateCapHyperpathCost(floatType lambda);
	//{
	//	cost = WaitCost;
	//	fdcost = 0.0;
	//	for(int i=0;i<m_links.size();++i)
	//	{
	//		PTLink* link =   m_links[i]->m_linkPtr;
	//		cost += m_links[i]->m_data * link ->g_cost;
	//		fdcost += m_links[i]->m_data * link ->g_pfdcost * m_links[i]->m_data;	
	//		if (link->rLink)
	//		{
	//			vector<GLINK*>::iterator it = find_if(m_links.begin(), m_links.end(), finder_t(link->rLink->id)); 
	//			
	//			if (it != m_links.end())
	//			{
	//				fdcost += m_links[i]->m_data *  link->g_rpfdcost * (*it)->m_data;
	//			}
	//
	//		}
	//	}
	//}


	std::vector<GNODE*> m_nodes;
    std::vector<GLINK*> m_links;


	bool		InitializeHP(PTNode *org, PTNode *dest);	
	//bool		InitializeHP(PTNode *org, PTNode *dest);	
	void		print()
	{
		double t = WaitCost;
		//cout<<"od:"<<org->id<<"->"<<dest->id<<endl;
		for (int i=0;i<m_links.size();i++)
		{   //超路径集合中，从起点org开始：输出第i段弧的id (超路径k上使用弧的概率Π):该弧的费用cost -> 下一段的id(概率):cost->直至终点dest
			cout<<m_links[i]->m_linkPtr->id<<"("<<m_links[i]->m_data<<"):"<<m_links[i]->m_linkPtr->cost<<"->";  //m_data记录超路径k上使用弧(i,j)的概率Π[i,j]，m_prob记录在节点i处乘客使用弧(i,j)的概率e[i,j]
			t += m_links[i]->m_linkPtr->cost * m_links[i]->m_data; //超路径费用 t = WaitCost + sum( Π(i,j)*cost(i,j) )
		}
		cout<<endl;
		cout<<"WaitCost:"<<WaitCost<<", pathcost:"<<t<<endl; //输出超路径k的总期望等待时间WaitCost和费用Ck(t)
	}
};



class PTNet_API PTOrg
{
	public:
	PTOrg() { org = NULL; dest = NULL;assDemand = 0; buffer = NULL;maxPathGap=0.0;minIx = 0;} //constructor

/*=======================================================================================
                  data members
  ======================================================================================*/
	PTNode*         dest;
	PTNode*         org;//node on which the origin destination rest	
	floatType           assDemand;//assignment O-D demand
	floatType           maxPathGap;//(max hyperpath cost - min hyperpath cost)
	floatType           *buffer;
	floatType			currentTotalCost;
	double				currentRelativeGap;
	vector<TNM_HyperPath*> pathSet;
	int					minIx;// minimum cost of path index in the pathset  
	bool                Initialize(PTNode *destnode, PTNode *orgnode, floatType ass)
	{
		if(destnode == NULL || orgnode  == NULL) return false;
		
		dest   =   destnode;
		org = orgnode;
		assDemand = ass;
		return true;
	}
	void				ComputePathSetCostdiff(double &mincost, double &maxcost, double &ttcost);
	void				UpdatePathSetCost();
	void				PathFlowConservation();
	void				CTEAPIPFPathFlowConservation(floatType gamma);
	void				CTEAPIPFUpdatePathSetCost(floatType gamma);
	void				TL_UpdateCapPathSetCost(floatType lambda); //for capacited problem 
	void				BL_UpdateCapPathSetCost(floatType lambda);
	void				UpdateCapPathSetCost(floatType lambda);
};



class PTNet_API PTDestination
{
	public:
	PTDestination();  /*constructor*/

	PTDestination(PTNode* dest, int nd)
	{
		destination = dest;
		numOfOrg = nd;

		if(numOfOrg > 0) 
		{
			orgVector = new PTOrg*[numOfOrg];
			for (int i = 0;i<numOfOrg;i++)  orgVector[i] = new PTOrg;
		}
		else
		{
			orgVector = NULL;
		}
		m_tdmd = 0.0;
		m_trimmed = false;
		m_expanded = false;
	}
	inline int			id_() {return destination->id;} //return id of the dest node 
	//add for path-based algorithm
	void		InitialSubNet(PTNET *net);
	void		DestAllOrNothing();
	bool        TopologOrder(PTNET *net);
	void		RemarkBushLinksOnNet();
	void		MarkBushLinksOnNet();
	int			ExpandSubNet(PTNET *net);
	double      UpdateNodePotential(bool iscontributing = false);
	double		NodeFlowShift(PTNode *node,PTNET *net);
	int			TrimSubNet();
	


	//add for effective frquency model
	void		EffStgSubNetAllorNothing(PTNET *net);
	void		InitialStgNodeLinks(PTNET *net);
	void		AssignEffStgFlow(PTNode* org, PTNode* dest, double dmd);
	void		EffMarkStgLinksOnNet();
	void        EffRemarkStgLinksOnNet();
	int			EffExpandSubNet(PTNET *net);
	floatType	EffEquilibriumSubNet(PTNode *node,PTNET *net);
	void		EffUpdateSubNetStgFlow();
	void		EffStgFlowShift(PTNET *net);
	GNODE*		FindNearestFPD(GNODE* onode, vector<GNODE*> &M, bool isshort, double &hyperpathcost, double &maxshiftflow );
	void		EffShiftFlowEPHS(vector<GNODE*> Q, vector<GNODE*> rQ, double shift);
	


	bool		SetOrg(int id, PTNode *orgnode, floatType demand)
	{
		if(id>numOfOrg||id<=0) 
		{
			cout<<"\n\tSetOrg in Destination Object: dest index exceeds the rannge!"
				<<"\n\trequired index = "<<id<<endl;;
			return false;
		}
		return orgVector[id - 1]->Initialize(destination, orgnode, demand);
	}

	PTNode*					destination;       //node pointer
	int						numOfOrg;
	PTOrg					**orgVector;  //origins
	double					m_tdmd;       //total demand
	double					avgCOV;
	vector<GNODE*>			tplNodeVec;// a node vector follows topological order
	vector<GLINK*>			tplLinkVec;//add to store destination-based link flow
	bool					m_trimmed; //whether or not subnetwork is trimmed since last trim operation
	bool					m_expanded; //whether or not a subnetwork is expanded since last expanded operation. 
	bool					m_reduced; //whether or not a subnetwork is reduced for a strategy


	// capacited TEAP model
	double      UpdateCapStgPotential(bool iscontributing = false);
	int         ExpandCapStgSubNet(PTNET *net);
	floatType	CapStgNodeFlowShift(PTNode *node,PTNET *net);	
	void        UpdateCapStgSPTreeOnly();
	void        TrimCapStgSubNet(PTNET *net);
};



struct PTNet_API PTITERELEM
{
	PTITERELEM(){innerlooptime = 0.0;};
	int         iter; //current iteration number
	double      convGap; //current gap
	double	   convIF;//infeasible flow
	double      convRGap; //current relative gap
	double      stepsize; //current step size
	float       time; //current cputimeprint
	float		mainlooptime;
	float		innerlooptime;
	int			innerIters;
	int			numberofhyperpaths;
	float		multiplierconv;//add for cap TEAP
	float		penalty;//add for cap TEAP
	float		epsi; //add for CTEAP-IPF
};

class PTNet_API GRIDPTNETPAR
{
public:
	GRIDPTNETPAR()
	{
		seed = 1;
		nx   = 10;
		ny   = 10;
		zRatio=0.125;
		dLevel = 0.5;
		gridLen= 0.5;
		randLen= false;
		commonLines= 1;
        loop    = true;

	}
	GRIDPTNETPAR(const GRIDPTNETPAR &rhs)
	{
		seed = rhs.seed;
		nx   = rhs.nx;
		ny   = rhs.ny;
		zRatio = rhs.zRatio;
		dLevel  = rhs.dLevel;
		gridLen = rhs.gridLen;
		randLen = rhs.randLen;
		commonLines = rhs.commonLines;
        loop     = rhs.loop;
	}
	void   Print();
	void   Input();
    int    Input(const string &fileName);
	int   Save(const string &fileName);
public:
	int    seed;
	int    nx; //x - number of stops
	int    ny; //y - number of stops
	float  zRatio; //zone ratio
	float  dLevel;//demand level between 0 and 1
	float  gridLen; //grid length in mile
	bool   randLen; //random length.
	int    commonLines; //common lines of two adjacent stop
    bool   loop;
};

typedef map<string,PTStop*, less<string> > PTStopMap;
typedef map<string,PTStop*, less<string> >::iterator PTStopMapIter;
typedef map<string,PTRoute*, less<string> > PTRouteMap;
typedef map<string,PTRoute*, less<string> >::iterator PTRouteMapIter;
typedef map<string,PTShape*, less<string> > PTShapeMap;
typedef map<string,PTShape*, less<string> >::iterator PTShapeMapIter;




class PTNet_API PTNET
{
public:
    PTNET(const string &name)
	{
		networkName = name;
		numOfNode = 0;
		numOfLink = 0;
		numOfPTDest=0;
		numOfPTOD=0;
		numOfPTTrips = 0;

		numaOfHyperpath = 0;
		curIter = 0;
		InnerIters = 0;
		IterMainlooptime = 0;
		IterInnerlooptime = 0;
		netTTwaitcost = 0;
		flowPrecision = 1e-10;
		m_capacity = 30;

		m_walkNum = 0;
		m_maxWalkTime = 5.0; //5 minutes. 
		m_alightLoss  = 0.1; //0.1 minute
		m_walkPenalty = 1.0;
		RGapIndicator = 1.0;
		buffer =NULL;

		m_symLinks = false; 
	};

	typedef enum {
     /* Naming conventions:
	 PCTAE_{A/P/B}_* = arc-based/hyperpath-based/strategic bush-based respectively
	*/
     PCTAE_A_SD, //Simplicial decomposition algorithm 
	 PCTAE_A_GFW,//General frank-wolf algorithm 
	 PCTAE_A_MSA,//MSA method, step size = 1 / numer of iters

	 PCTAE_P_iGreedy, // 'i' mean with innerloops
	 PCTAE_P_Greedy,
	 PCTAE_P_NGP,	// gradient proj with new VI formula
	 PCTAE_P_iNGP,


	 CAP_PCTAE_P_Greedy_TL,		// Capacited that solved in hyperpath-based formulation with Method of mulplier，capacity imposed on transit link
	 CAP_PCTAE_P_GP_TL, //capacity imposed on transit link
	 CAP_PCTAE_P_GFW_TL, //use GFW for uncapacitated subproblem
	 CAP_PCTAE_IPF_iGP_TL, //capacity imposed on transit link, IPF




	} PCTAE_algorithm; 

	
	typedef enum {
		FCTAE_A_MSA,//it doesn't work in the space of link flow
		FCTAE_B_MSA,
		FCTAE_B_HYPERPATH
	}FCTAE_algorithm; 




	///these are IO related to Transit assignment problem.
	int  ReadTEAPStops(bool ispos=false);
	int  ReadTEAPRoutes();
	int  ReadTEAPShapes();
	int  ReadTEAPODdemand();
	int  CreateTEAPNodeLinks();
	int  ReadTEAPTransitData();
	int  CreateTEAPWalks(bool Traj = false);
	int  ReadODWALK();//add for capacitated problem for feasibility
	void SetProjection(bool con) {m_IsPorjected=con;} 
	void ReprojectNodeCoordinates();
	void GetVicinityNodes(COORDMAP &xmap, COORDMAP &ymap, vector<PTNode *> &nvec, PTNode *node, long threshold);
	


	void UpdateNodeNum() {numOfNode = nodeVector.size();}
	void UpdateLinkNum() {numOfLink = linkVector.size();}
	void PrintNetLinks()
	{
		for (int i = 0; i<numOfLink; i++)
		{
			PTLink* link = linkVector[i];
			cout<<"link id:"<<link->id<<", tailnode:"<<link->tail->id<<",head:"<<link->head->id<<", flow:"<<link->volume<<",cost:"<<link->cost<<",eff:"<<link->efffreq<<endl;
		}
	};

	



	PTLink*	CatchLinkPtr(PTNode* tail, PTNode *head)
	{
		for (vector<PTLink*>::iterator pl = tail->forwStar.begin(); pl!=tail->forwStar.end(); pl++)
		{
			if((*pl))
			{
				if((*pl)->head == head)
					return *pl;
			}
		}
		return NULL;
	
	};

	int				    nodeBufferSize;
	int			        linkBufferSize;
	int				    pathBufferSize;
	int				    destBufferSize;
	int					AllocateLinkBuffer(int size);
	int					AllocateLinkBufferData(int size);
	int					AllocateNodeBuffer(int size);
	int				    AllocateNetBuffer(int size);

	PTShape* GetShapePtr(const string &id)
	{	
		PTShapeMapIter si = m_shapes.find(id);
		if(si == m_shapes.end()) return NULL;
		else                     return si->second;
	};
    PTRoute* GetRoutePtr(const string &id)
	{
		PTRouteMapIter si = m_routes.find(id);
		if(si == m_routes.end()) return NULL;
		else                     return si->second;
	};
    PTStop*  GetStopPtr(const string &id)
	{
		PTStopMapIter si = m_stops.find(id);
		if(si == m_stops.end()) return NULL;
		else                     return si->second;
	};

	PTLink*	CatchLinkPtr(int id)
	{
		vector<PTLink*>::iterator pl;
		pl = find_if(linkVector.begin(), linkVector.end(), predP(&PTLink::id_, id));
		if(pl == linkVector.end()) return NULL;
		else                         return *pl;
	
	};

	PTNode*	CatchNodePtr(int id)
	{
		vector<PTNode*>::iterator pv;
		pv = find_if(nodeVector.begin(), nodeVector.end(), predP(&PTNode::id_, id));
		if(pv == nodeVector.end()) return NULL;
		else                         return *pv;
	}

	PTNode*	CatchNodePtrbyTrafficid(int id)
	{
		vector<PTNode*>::iterator pv;
		pv = find_if(nodeVector.begin(), nodeVector.end(), predP(&PTNode::trafficid_, id));
		if(pv == nodeVector.end()) return NULL;
		else                         return *pv;
	}

	PTDestination*	CatchDestPtr(int id)
	{
		vector<PTDestination*>::iterator pv;
		pv = find_if(PTDestVector.begin(), PTDestVector.end(), predP(&PTDestination::id_, id));
		if(pv == PTDestVector.end()) return NULL;
		else                         return *pv;
	}


	void							PCTAE_SetAlgorithm(PCTAE_algorithm algorithm){PCTAE_ALG = algorithm;};
	void							FCTAE_SetAlgorithm(FCTAE_algorithm algorithm){FCTAE_ALG = algorithm;};
	string							GetAlgorithmName();
	int								PCTAE_Solver(PCTAE_algorithm algorithm);//partial congested model
	void							SetConv(floatType e){convCriterion=e;};
	void							SetMaxIter(int i){maxMainIter = i;};
	void							SetMaxIterTime(floatType i){maxIterTime = i;};
	void							SetInnerConv(floatType c = 0.001) {m_innerConv = c;}
	void							Settimescaler(floatType i){timescaler = i;};
	void							SetAlightLoss(floatType i){m_alightLoss = i;};
	void							SetWalkPenalty(floatType i = 1.0){m_walkPenalty = i;};
	void							SetCapcityPerLine(floatType i){m_capacity = i;};
	void							SetSymLinkType(bool a) {m_symLinks  = a;};
	inline  bool					ReachAccuracy(double g) {return g<=convCriterion;}
	inline  bool					ReachMaxIter()  {return curIter>=maxMainIter;} //test if maximum iteration is attained
	inline  bool					ReachMaxTime(floatType t) {return t>=maxIterTime;}
	void							RecordTEAPCurrentIter();
	void							ReportIter();
	void							ReportPTlinkflow();
	void							ReportPTlinkflow_IPFCTEAPIni();
	void							ReportPTHyperpaths();
	void							ReportLinkCap();
	void							ReportAvgpaxinfo();//输出乘客平均的出行成本、换乘次数、车内时间、等待时间、步行时间

	// output functions for SZ project
	void							SZ_OutputOnOffLinkFlow();
	void							SZ_OutputTransitLinkFlow();
	void							SZ_OutputWALKLinkFlow();
	void							SZ_OutputHyperpathFlow();
	//input functions
	int								SZ_BuildAN();
	int								SZ_ReadStop();
	int								SZ_ReadRoute();
	int								SZ_ReadShape();
	int								SZ_ReadWalk();
	int								SZ_ReadTEAPTransitData();
	int								SZ_ReadStopTrip();
	int								SZ_ReadNodeTrip();
	//======Functions to solve transit assignment problem//
	int								BuildAN(bool stopPos=false /*whether has gis pos*/, bool walkfile = false);//build network for assginment 
	int								GenerateRandGridAN(GRIDPTNETPAR par,string filepath);// build grid network
	void							GenerateRandGripTrip(GRIDPTNETPAR par);// write trip info, called after net has built
	void							UpatePTNetworkLinkCost();
	void							ConnectAsymmetricLinks();
	void							SetLinksAttribute(bool isSym);
	PTDestination*					CreatePTDestination(int nid, int noo);//node id, number of origins 
	PTDestination*					CreatePTDestination(PTNode* rootDest, int noo);//node id, number of origins 
	int								InitializeHyperpathLS(PTNode* rootDest);

	StgLinks*						GenerateNonBoardingStg(PTLink* link);						
	void							GenerateBoardingStg(vector<PTLink*> links, vector<PTLink*> &attractivelinks,floatType &ec/*expected cost*/,floatType &ew/*expected waiting delays*/);
	
	

	void							ComputeConvGap();
	void							ComputeNetWait();

	
	clock_t							m_startRunTime; //this is the time when SolveTAP just called.
	
	void							GenerateSPP();//Generate shortest path 
	void							GenerateODWalkFile();							
	
	//=======================Method of mulplier for capacited TEAP in hyperpath  
	void				IniTransitLinkMultiplier();
	void				IniBoardLinkMultiplier();
	void				IniCapacityPars();
	void				IniCapLambda();
	void				ComputeCapSubConvGap();
	void				UpdatePTNetGeneralCost_Dercost();
	int					InitializeGeneralHyperpathLS(PTNode* rootDest);
	void				UpdateMultiplier();
	floatType			ComputeNormFlowInfeasibility(bool islastflow);
	floatType			ComputeNormSolution();
	void				SetCurrentSolution();
	void				PrintCapTransitLinkFlow();
	floatType			ComputeBoundGAP();
	void				RecordCapTEAPCurrentIterIPF(float GAP, float IF);
	void				RecordCapTEAPCurrentIter(float GAP, float IF, float SN);
	void				ReportCapIter();
	void				ReportWalkflowRation(); //record the flow portion on walking link 

	bool				InitialCapHyperpathNetFlow();
	int					SolveCapacityHyperpath();
	int					SolveCapacityHyperpathIPF();
	void				CapacityHyperpathSubProblem();
	void				CapacityLinkSubProblem();
	void				UpdateCapHyperPathGreedyFlow(PTDestination* dest,PTOrg* org);// constraint on transit link (TL)
	void				UpdateCapHyperpathGPFlow(PTDestination* dest,PTOrg* org);
	void				BL_UpdateCapHyperpathGPFlow(PTDestination* dest,PTOrg* org);
	void				HyperpathCapInnerLoop(int maxiters = 200);
	void				ReportPTlinkflowCap();

	void				CheckWalkingHyperPath();
	void				CreateODLink();
	void				CreateWalkLink(); //为Gentile网络构建步行弧
	// initialize a feasible solution
	void				ZeroTransitLinkFlow();
	void				EnlargeLineCapacityII();//enlarge capacity on transit links
	void				EnlargeLineCapacity();//enlarge capacity on boarding links
	void				EnlargeLineCapacityIII();//enlarge the capacity once detecting a block path 

	floatType			AssignCapFeasibleFlow(PTDestination* dest,PTOrg* org);
	bool				InitializeFeasiblePathLS(PTNode* rootDest);
	void				UpdateLinkCapacity();


	//parameters
	floatType			cb_xi;		//penalty enlarge parameter, recommended as [2,10] (Bertsekas)
	floatType			cb_sigma;	// infeasible flow reduction criteria, recommended as 0.25 (Bertsekas), 0.7 (Feng), 0.8 (Nie) //不可行流量在前后两次迭代中的变化指标（用于惩罚系数调整的判定条件）
	floatType			cb_lambda;	// penalty coefficient
	long double			infeasibleflow;	// penalty coefficient
	floatType			SubRG;  //relative gap in sub-problem
	long double			SubRGCriterion;
	int					SubIter;
	int					MaxSubIter;

	floatType			MaxFlowInfeasibility;

	//=======================IPF for capacited TEAP in hyperpath 
	void				ReportIterIPF();
	void				ReportPTlinkflowCapIPF();
	floatType			GammaConvTest(floatType gamma);
	void				UpatePTNetworkLinkCost_IPF();
	void				IniLinkBuffer();
	void				IniSolveCapacityHyperpathIPF();
	void				IniCapacityHyperpathProblem();
	void				CTEAPUpdateEpsilon();
	void				CTEAPUpdatepseCap();
	void				CTEAPUpdateGamma(floatType gamma);
	void				CTEAPSetiniGamma();
	void				CTEAPIPFUpdateLinkgCost();
	void				CTEAPIPFUpdatePathSetCost();
	bool				CTEAPIPFTerminate();
	void				CTEAPSubLoop_IPF();
	void				IPFHyperpathCapInnerLoop(int maxiters = 200);
	void				IPFComputeCapSubConvGap();
	floatType			IPFComputeNormFlowInfeasibility(bool islastflow);
	void				CTEAPIPFUpdateHyperpathGPFlow(PTDestination* dest,PTOrg* org);
	int					satuationLink;
	int					numOfTransitLink;
	float				FlowInfeasible;
	float				MaxFlowInfeasible;
	//float				gammaTest; //收敛判定值
	float				gapConv;	//收敛精度值
	int					IPFInitializeGeneralHyperpathLS(PTNode* rootDest);

	//parameters
	floatType			alpha;	// 
	floatType			beta;	// 
	floatType			gamma;	// 
	floatType			kapa;
	bool				EndUpdC;//link pseduo capacity is equal to original capacity
	floatType			temp_gamma;	// 
	floatType			epsilon;	// 
	floatType			MaxMainConv;
	floatType			MaxSubConv;
	int					MaxMainIter;
	floatType			MainRG; //relative gap in main-problem
	floatType			Inilooptime; //



	//=======================fw methods
	int					SolveFWTEAP();
	int  				PTAllOrNothing();
	int  				PTAllOrNothingCAP();
	int  				PTAllOrNothing(PTDestination* dest);
	int  				PTAllOrNothingCAP(PTDestination* dest);
	int                 BisecSearchCAP(floatType maxStep = 1.0);  //line search: bisection for CTEAP
	void				SaveLinkWaitVariables(int col = 1);
	int                 BisecSearch(floatType maxStep = 1.0);  //line search: bisection
	floatType			ComputePTDz(); //overload computeDz for bisec line search
	floatType			ComputePTDzCAP();
	void				NewSolution(int col);        //volume = buffer[col - 1] + (volume - buffer[col - 1])*stepSize
	void				SaveLinkSolution(int col);

	//===================hyperpath-based methods
	int					SolvePathTEAP();
	void				CoutLinkInoCAP();
	bool				InitialHyperpathNetFlow();
	void				ColumnGeneration(PTDestination* dest,PTOrg* org);
	void				UpdateHyperPathGreedyFlow(PTDestination* dest,PTOrg* org);
	void				UpdateHyperpathGPFlow(PTDestination* dest,PTOrg* org);
	void				HyperpathInnerLoop(int maxiters = 200);
	void				FlowReAssignment();

	void				ExportHyperpathUEsolution();
	bool				ImportHyperpathUESolution();

	//==================simplicial decomposition method
	int					SolveSDTEAP();//SD algorithm with first-order approximation
	void				SDInitializedCH();// initialize convex hull to get two extreme solution
	bool				SDAddCurrentSolution2CH();
	void				SDMasterProblem();
	vector<floatType*>	c_h;//convex hull set, only called in SD-type algorithms
	floatType			SDProjGap;// the gap criterion in SD master problem

	string							networkName;
	PCTAE_algorithm					PCTAE_ALG;  
	FCTAE_algorithm					FCTAE_ALG;
	vector<PTNode*>					nodeVector;
	vector<PTLink*>					linkVector;
	int								numOfNode;
	int								numOfLink;
	vector<PTDestination*>			PTDestVector;
	int								numOfPTDest;
	int								numOfPTOD;		// number of O-D pairs
	floatType						numOfPTTrips;		// number of O-D pairs
	int								numaOfHyperpath;// number of hyperpaths
	int								numaOfShift;// number of hyperpaths
	int								m_walkNum;
    double							m_maxWalkTime; /*in minutes*/
    double							m_alightLoss;
	double                          m_walkPenalty; /*walk penalty parameter*/
	floatType						m_capacity;
	bool							m_symLinks;//add to repsent whether a link has a separable cost function
	floatType						m_innerConv;
	floatType						RGapIndicator; //convergence indicator
	floatType						convCriterion; // convergence gap
	floatType						GapIndicator; // absolute gap indicator
	floatType						IterInnerlooptime;//time for each inner loop
	floatType						IterMainlooptime;//time for each inner loop
	int								InnerIters;
	floatType						maxIterTime;   //maximum allowed iteration time in min
	int								maxMainIter;   //maximum allowed iteration number
	floatType						timescaler;
	int								curIter;       //current iteration
	vector<PTITERELEM*>				iterRecord;		//a vector record iteration history
	floatType						stepSize;      // current step size
	floatType						*buffer;    //this data is added as a working space for outside using.
	floatType						netTTwaitcost; //this data is added as the network total waiting cost
	floatType						WalkflowRation;
	floatType						SCnetTTwaitcost;//simplex combination waiting cost, only called in SD
	floatType						netTTcost; //this data is added as the network total cost
	floatType						flowPrecision;
	//floatType						tt;


	PTStopMap						m_stops;
    PTRouteMap						m_routes;
    PTShapeMap						m_shapes;

	

	bool							m_IsPorjected;// indicate if the coordinate has been projection or not
	//CSHPInterface					m_geoIF; //geo interface. initialized in ReadPTNode; check if average stop x is between -180 and 180.
};


