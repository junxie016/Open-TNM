#ifndef TNM_NET_H
#define TNM_NET_H

#include <vector>
#include <queue>
#include <map>
#include "TNM_Header.h"
#include "TNM_utility.h"
using namespace std;
class TNM_SLINK;
class ORGLINK;
class TNM_SPATH;
class TNM_SDEST;
class SCANLIST;
class TNM_SNET;
//class IDManager;

//int TNM_SORIGIN::m_count = 0;

//<summary> A structure to maintain optimal cost and optimal routing policy (next link).
//<newpara>It is used in all kinds of routing problems (e.g., shortest path problems).</newpara></summary>
struct PATHELEM
{
  PATHELEM()
  {
	  cost = 0.0;
	  pCost =0.0;
	  via  = NULL;
  }
  LINK_COST_TYPE cost; //Optimal cost to root of shortest path tree
  TNM_SLINK *via;      //Next link on shortest path leading to the root. 
  LINK_COST_TYPE pCost; //added by Jun, used in TAP_EBA
};


struct ORGNODE
{
	ORGNODE()
	{
		pathVia = NULL;
		potential = 0.0;
		shortEntro = 0.0;
		LongEntro = 0.0;
		entro = 0.0;
		fdentro = 0.0;
		Inflow = 0.0;
		sLink = NULL;
		lLink = NULL;
		status = 0;
		//opathElem = new OPATHELEM;
		//orpathElem = new OPATHELEM;
	}
	TNM_SNODE* nodePtr;
	TNM_SNODE* lcnPtr; //last common node pointer;
	bool       mutliEnd;
	int        tpLevel; //topological level;
	long double shortEntro;
	long double LongEntro;
	ORGLINK* sLink;
	ORGLINK* lLink;
	//vector<ORGLINK*> forwStar;
	//vector<ORGLINK*> backStar;


	//added by Jun
	deque<TNM_SLINK*> treeLink;//subnet topology
	long double potential;
	TNM_SLINK* pathVia;//tree topology
	int status;

	//added by ly
	long double entro;
	long double fdentro;
	long double Inflow;//sum of all incoming link flow for this bush
	//OPATHELEM	*opathElem;
	//OPATHELEM	*orpathElem;
	int			scanStatus;
	vector<ORGLINK*> forwStar;
	vector<ORGLINK*> backStar;
	int			tmpNumOfIn;
	ORGNODE* lconPtr; //last common onode pointer;
};

class TNM_EXT_CLASS TNM_SNODE 
{
public:
	TNM_SNODE();
	~TNM_SNODE();

	int                id;         //Node id
	TNM_NODETYPE       type;       //Node type.
	largeInt           xCord;      //The x coordinate
	largeInt           yCord;      //The y coordinate
	vector<TNM_SLINK*> forwStar;   //All outgoing link pointers
	vector<TNM_SLINK*> backStar;   //All incoming link pointers
	PATHELEM           *pathElem;  //Store the optimal routing policy. Normally this is reserved for shortest path tree rooted at an origin. 
	PATHELEM           *rPathElem; //This is reserved for possible all-to-one shortest path search. Only be used as necessary.
	PATHELEM           *oldPathElem;
	floatType          *buffer;    //It is a temporary working space for outside use.  TNM_SNET provide a function to allocate this memory.
	smallInt           scanStatus; //A swith variable.  Normally it is used in shortest path computation, also somewhere else.
	int                tmpNumOfIn; //Used in topological computation to store the incoming links
	bool               dummy;      //A switch variable, tell if the node is a temporary dummy node. In some applications, dummy nodes might be created.
	bool               m_isThrough;
	int                attachedOrg; //how many origins are builed on this node.
	int                attachedDest;
	bool			   is_centroid; //
	bool			   SkipCentroid;//if need, skip centroid when searching for paths
	ORGNODE        *m_orgNode;



	inline int		   id_() {return id;} //return id of this node.
	void			   InitPathElem(); //initalize path element, set pathElem.cost = +infinity, and pathElem->via = NULL. Normally used in shortest path algorithms
	virtual bool	   Initialize(const NODE_VALCONTAINER &cont);  
	void			   SearchMinInLink(SCANLIST *list); 
	void			   SearchMinOutLink(SCANLIST *list); 
	int            NumOfOrgInLink();

};


class TNM_EXT_CLASS TNM_SLINK
{
public:
	TNM_SLINK();
	~TNM_SLINK();

	int					id;        /*link id*/
	int                 orderID;   //the order of the link in the vector.
	TNM_LINKTYPE		type;
	TNM_SNODE			*head;     /*starting node of the link*/
	TNM_SNODE			*tail;     /*ending node of the link */
	floatType			capacity;  /*link capacity, vph*/
	floatType			volume;    /*link volume, vph*/
	floatType			length;    /*link length, m*/
	floatType			ffs;       /*free flow speed, mph*/
	floatType			fft;       /*free flow travel time, h*/
	floatType           toll;      //this should be interpreted as any monetoary cost associated with link
	floatType			cost;      /*this can be a general concept of link cost (a basis for shortest path computation*/	
	floatType           fdCost;    //this is the derivative of link cost;
	floatType			*buffer;    //this data is added as a working space for outside using.
	int					markStatus; //a status variable for temporary usage.
	ORGLINK				*oLinkPtr;  /*a pointer toward an obLinkPtr. It will be null if none is set. I am considering to removed it*/
	vector<TNM_SPATH*>  pathInciPtr; // a vector contains all path pointers which use the link;
	bool				dummy;      //if the link is a temporary "dummy" link.
protected:
	TNM_TOLLTYPE        m_tlType;       //type of toll
	double				m_timeCostCoefficient; //monetary value per unit travel time (the unit is defined by linkCostScalar, when link cost scalar = 1, it is hour, =60, in minutes
	double				m_distCostCoefficient; //monetary value per unit travel distance (mile)

/*============================================================================================
        Major methods.: Initialize functions
  ===========================================================================================*/
public:
	virtual bool		Initialize(const LINK_VALCONTAINER cont);
	void                SetTollType(TNM_TOLLTYPE tl) 
	{
		if(tl == TT_MXTOLL) 
		{
			cout<<"\tTry to set link toll type to mixed, use fixed toll strategy instead!"<<endl;
			m_tlType = TT_FXTOLL;
		}
		else m_tlType = tl; 
	}
	void				 InitializeCostCoef(double t, double d) 
	{
		if(t > 0) m_timeCostCoefficient = t;
		if(d >=0) m_distCostCoefficient = d;
	}
	floatType           GetCost(bool ftoll = false); 
	double              GetToll(); //this returns the actual toll depending on the m_tltype.
	floatType           GetDerCost(bool ftoll = false);
	floatType           GetIntCost(bool ftoll = false);
protected:
	virtual void		ConnectFW(); //put the link's pointer to tail node's forwStar
	virtual void		ConnectBK(); //put the link's pointer to head node's backStar
	virtual void		DisconnectFW(); //erase link's pointer from tail'node's forwStar
	virtual void		DisconnectBK(); //erase link's pointer to head node's backstar
	virtual floatType   GetDer2Cost_() {return 0.0;}
	virtual floatType   GetCost_() {return length;}
	virtual floatType   GetDerCost_() {return 0.0;}
	virtual floatType   GetIntCost_() {return 0.0;}

private:
	bool                CheckParallel();

};

class TNM_EXT_CLASS TNM_BPRLK :	public TNM_SLINK
{
public:
//constructors and destructor
	TNM_BPRLK(void) {alpha = 0.15; beta = 4; type = BPRLK;}
	virtual ~TNM_BPRLK(void) {;}

	virtual bool		Initialize(const LINK_VALCONTAINER cont); //the first of cont.par is given to alpha, the other is given to beta.
	//void                SetAlpha
	floatType  GetAlpha() {return alpha;}
	int  GetPower()  {return beta;}
protected:
	floatType			alpha; //parameters to define speed-flow relationship.
	int			beta;
	virtual floatType   GetCost_();
	virtual floatType   GetDerCost_();
	virtual floatType   GetIntCost_();
	virtual floatType   GetDer2Cost_();
};

//class defining origin-based link. it is useful for storing origin-based flows.
struct ORGLINK
{
	ORGLINK()
	{
		linkPtr = NULL;
		oFlow   = 0.0;
		markStatus = 0;
		approach   = 0.0;
		buffer     = 0.0;
		tempFlow = 0.0;
	    rc =0;
		entro = 0;
		fdentro = 0.0;
		otail=NULL;
		ohead=NULL;
	    isolated = false;
	}
	TNM_SLINK *linkPtr;
	long double oFlow; //origin based flow
	long double approach;
	long double buffer;  //this is a solution variable intended for temporary use. 
	tinyInt   markStatus;
	long double rc;//reduced cost for this link
	long double entro; // value of entropy
	bool   isolated;//added by Jun

	//added by jun
	long double tempFlow;
	//added by ly
	long double fdentro;
	ORGNODE *otail;
	ORGNODE *ohead;
	bool onUEbush;
};

class TNM_EXT_CLASS TNM_SPATH
{
public:
	TNM_SPATH();
	~TNM_SPATH();

	int                   id;
	floatType             flow;      // path flow
	floatType             preFlow; //added by Jun
	floatType             cost;      // path cost
	floatType             *buffer;   //a working space for outside using. 
    short                 markStatus; 
    vector<TNM_SLINK*>    path;      //maintain path trace by link pointers
	floatType             fdCost;
	floatType             estCost;
	floatType             preRatio;
	floatType             curRatio;

private:
	static smallInt       pathBufferSize;
	static IDManager      idManager;

public:
	static void           SetPathBufferSize(int bf) {pathBufferSize = bf;}
};

typedef vector<TNM_SLINK*>::iterator PTRTRACE;
typedef vector<TNM_SPATH*>::iterator PTRPATH;

class TNM_EXT_CLASS TNM_SORIGIN 
{
public:
	TNM_SORIGIN();  /*constructor*/
	TNM_SORIGIN(TNM_SNODE *, int nd);
	~TNM_SORIGIN(); /*destructor*/

	TNM_SNODE            *origin;       //node pointer
	int                  numOfDest;     //number of destinations
	TNM_SDEST            **destVector;  //destinations
	double               m_tdmd;       //total demand;
	short                m_class;      //we may use this identify different classes. 
	vector<ORGLINK *>    obLinkVector;  //a link vector used by the current link;
	vector<ORGNODE *>    tplNodeVector; // a node vector follows topological order
	vector<TNM_SDEST*>   bDestVector;
	int                   m_id;
	int    m_count;

	inline int			 id_() {return origin->id; } //return origin's id
	
	inline int   id() { return m_id; }
	void				 DeleteBush(); //delete the data associated with bush operation
	bool				 SetDest(int id, TNM_SNODE *, floatType demand);
	void                 LoadDestVector();
	int                  DeleteZeroOD();
	void                 UnLoadDestVector();
	void                 InitialSubNet(TNM_SNET*); //initialize sub-network, which should be acylcic
	void                 OrgAllOrNothing();
	bool                 TopologOrder(TNM_SNET*);    //check if the subnetwork is acyclic, if it is, create a toplogical order.

	void                 RemarkOBLinksOnNet(); //reset the mark on links used this origin.
	void                 MarkOBLinksOnNet(); //set a mark on links used by this origin.
	void                 UpdateCFSPTreeOnly(); //only sp tree is updated.

};

class TNM_EXT_CLASS TNM_SDEST
{
public:
	TNM_SDEST() ; //constructor
	~TNM_SDEST(); //destructor

	TNM_SNODE          *dest;    //node on which the destination rest
	TNM_SNODE          *origin;
	floatType          assDemand;//assignment O-D demand
	floatType          *buffer;
	vector<TNM_SPATH*> pathSet;
	TNM_SPATH*         yPath;//save the new generated shortest path for FWR algorithm
	floatType          costDif;
	floatType          shiftFlow;

	virtual void	   EmptyPathSet();
	bool               Initialize(TNM_SNODE *origin, TNM_SNODE *dest, floatType assDemand);
};

class TNM_EXT_CLASS TNM_SNET
{
public:
	TNM_SNET(const string& netName); /*constructor*/
	~TNM_SNET();

	string               networkName;
    int                  numOfNode;                // number of nodes
	int                  numOfLink;                // number of links
	int                  numOfOrigin;              // number of Origins pairs.
	int                  numOfOD;                  // number of O-D pairs
	vector<TNM_SLINK*>   linkVector;               // a vector stoaring link pointers
	vector<TNM_SNODE*>   nodeVector;               // a vector storing node pointers
	vector<TNM_SORIGIN*> originVector;             // a vector storing origin pointers
	vector<TNM_SNODE *>  destNodeVector;     //assemble all destinations here for convenience
	SCANLIST             *scanList;				   // a scanlist object which determines how shortest path algorithm is implemented
	int                  buildStatus;   //=0 means network has not been build yet.
	int                  initialStatus; //= 0 means netwok has not been initiated yet.
	floatType            linkCostScalar;
	bool				 centroids_blocked; //If or not the center is closed when searching for paths
	
/*==============================================================================================
                             io functions
  =============================================================================================*/
	void                 ChooseSPAlgorithms(DATASTRUCT);             //a number of SP algorithms can be chosen.
	                                                                //datastruct is enumeratio variable defined in tnm_header.h
	int                  BuildTAPAS(bool loadod = true, const TNM_LINKTYPE tt = BPRLK); //reading tapas format. 
	void                 SetLinkCostScalar(floatType);
	void                 SetLinkTollType(TNM_TOLLTYPE tl);
	int                  CheckBuildStatus(bool noteBuilt); // give a messenge if it is built already when notebuild = true;
	TNM_SLINK*			 CreateNewLink(const LINK_VALCONTAINER lval);
	TNM_SNODE*           CreateNewNode(const NODE_VALCONTAINER nval);
	TNM_SNODE*           CatchNodePtr(int id, bool safe = false); //search from nodeVector by its id
	TNM_SLINK*           CatchLinkPtr(TNM_SNODE* tail, TNM_SNODE *head);
	TNM_SORIGIN*         CreateSOrigin(int nid, int nd); //create a static origin
	TNM_SORIGIN*         CreateSOriginP(TNM_SNODE*, int nd);
	void                 UpdateLinkNum() {numOfLink = (int)linkVector.size();}
	void                 UpdateNodeNum() {numOfNode = (int)nodeVector.size();}
	void                 UpdateOriginNum();
	int                  ClearZeroDemandOD();
	int                  DeleteAnOrigin(int id);
	void				 InitializeCostCoef(double t, double d) 
	{
		if(t > 0) timeCostCoefficient = t;
		if(d >=0) distCostCoefficient = d;
	}
	TNM_SORIGIN     *CatchOrgPtr(int id);



protected:
	virtual TNM_SLINK*   AllocateNewLink(const TNM_LINKTYPE &pType);
	virtual TNM_SNODE*   AllocateNewNode(const TNM_NODETYPE &nType);
	double				 timeCostCoefficient;
	double				 distCostCoefficient;

/*==============================================================================================
                             Functional functions
  =============================================================================================*/
public:
	virtual int          UnBuild();
	void				 EmptyPathSet();//empty all paths.
	virtual int          UnInitialize();
	void                 UpdateSP(TNM_SNODE *rootOrg);   //calculate the shortest path for a given origin
	virtual void         ScaleDemand(floatType r);
	void				 Reset();	//set all link volume as zero, update its link cost as the free flow cost
									//empty path set for all O-D pair;
	void				 ClearLink2PathPtr();
	virtual void		 UpdateLinkCost(bool toll = false); //consider first-best toll or not. 
	void                 UpdateLinkCostDer(bool toll = false);
	int				     AllocateNodeBuffer(int size);
	int				     AllocateLinkBuffer(int size);
	int				     AllocatePathBuffer(int size);
	int			  	     AllocateDestBuffer(int size);
	int				     ReleaseNodeBuffer();
	int			 	     ReleaseLinkBuffer();
	int				     ReleasePathBuffer();
	int				     ReleaseDestBuffer();
	void				     InitialSubNet(int sptreeLen = 0, bool createpath = false);//
	void                 InitialSubNet3(int sptreeLen = 0, bool createpath = false);

private:
	int				    nodeBufferSize;
	int			        linkBufferSize;
	int				    pathBufferSize;
	int				    destBufferSize;
//protected:
//	string              m_progressMessage;

};

class TNM_EXT_CLASS SCANLIST  
{
public:
	SCANLIST() {;} //default is backward scan;
	~SCANLIST() {;}

	void         SPTreeO(TNM_SNODE *);//given origin, perform shortest path computation, retunr iteration number.
	void         SPTreeD(TNM_SNODE *); //given destination, perform...
	virtual bool InsertANode(TNM_SNODE *) {return false;}
protected:
	virtual      TNM_SNODE *GetNextNode() {return NULL;}
	void         InitRoot(TNM_SNODE *);
};

class TNM_EXT_CLASS SCAN_DEQUE: public SCANLIST
{
public:
	SCAN_DEQUE();
	virtual ~SCAN_DEQUE();
	virtual void       PrintList();
	virtual bool       InsertANode(TNM_SNODE *);

protected:
	deque<TNM_SNODE *> nodeList;
	virtual            TNM_SNODE *GetNextNode();
	virtual void       ClearList();
};

class TNM_EXT_CLASS SCAN_QUEUE: public SCANLIST
{
public:
	SCAN_QUEUE();
	virtual ~SCAN_QUEUE();
	virtual void	   PrintList();
	virtual bool       InsertANode(TNM_SNODE *);

protected:
	virtual TNM_SNODE  *GetNextNode();
	virtual void       ClearList();
	queue<TNM_SNODE *> nodeList;

};

#endif