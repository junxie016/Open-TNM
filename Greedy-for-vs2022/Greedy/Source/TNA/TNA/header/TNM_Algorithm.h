#ifndef TNM_ALGORITHM_H
#define TNM_ALGORITHM_H

#include "mat_header.h"
using namespace std;

class TNM_EXT_CLASS TNM_TAP
{
public:
	TNM_TAP();
	~TNM_TAP();

	string            inFileName; //this is a string that normally attached with input and output of the object-related information.
	string            outFileName;
	TNM_SNET*		  network;
	TNM_LINKTYPE	  lpf;     //this is to provide the user an interface to determine the type of the links on the network
	                           //some build functions in network class, like buildfort, builddanet2, may need this.
	floatType		  costScalar;//
	floatType         OFV; //objective function value;
	int               curIter;       //current iteration
	float             cpuTime;       //used to stored the total time of solving
	TERMFLAGS         termFlag;   //termination Flag;
	TNM_SPATH*		  yPath;
	static int        intWidth;
	static int        floatWidth; 
	bool              reportIterHistory; //determine if the Iteration history will be retained.
	bool			  reportLinkDetail;
	bool			  reportPathDetail;

private:
	bool              m_needRebuild;
protected:
	clock_t           m_startRunTime; //this is the time when SolveRec() just called.
	//bool              m_storeResult; 
	int               numLineSearch;
	MATOBJID          objectID;  //this is an unique id to tell who is this class.
	int               maxMainIter;  //maximum allowed iteration number
	int				  m_maxInnerIter;
	//float             m_maxCPUTime; // in hours;
	floatType         convIndicator; //convergence indicator
	floatType         convCriterion; // convergence criterion
	floatType		  m_innerConv; 
	floatType         stepSize;      // current step size
	vector<ITERELEM*> iterRecord; //a vector record iteration history
	int               maxLineSearchIter;   //maxallowed line search iteration
	TNM_TOLLTYPE	  m_tlType;  //tolltype: determine equilibrium type
	bool			  m_resetNetworkOnSolve;
	ofstream          iteFile;    //iteration history.
	ofstream		  lfpFile;// link detail
	ofstream		  pthFile;//path detail
/*========================================================================================
	I-O methods.
  ========================================================================================*/
public:
	void          SetLPF(TNM_LINKTYPE l)
	{
		if(l!=lpf) PostRebuild();
		lpf = l;
	}
	void			  SetCostScalar(floatType c)
	{
			if(c!=costScalar) PostRebuild();
			costScalar = c;
	}
	void              PostRebuild(bool r = true) {m_needRebuild = r;}
	//bool              IsStoreResult() {return m_storeResult;}
	inline  bool      ReachAccuracy() {return convIndicator<=convCriterion;} //test if required accuracy is attained.
	inline  bool      ReachMaxIter() {return curIter>=maxMainIter;} //test if maximum iteration is attained
	inline  bool      ReachError() {return termFlag == ErrorTerm;} //test if termFlat is set to error, which could happen in the process of Solve function
	inline  bool      ReachUser() {return termFlag == UserTerm;}
	//inline  bool      ReachMaxCPU() {if(watchTime) return GetElapsedTime() > m_maxCPUTime * HR2SEC; else return false;}
	void              SetConv(floatType);
	void			  SetInnerConv(floatType c = 0.001) {m_innerConv = c;}
	void              SetMaxLsIter(int);
	void			  SetMaxInnerIter(floatType i = 15) {m_maxInnerIter = i;}
	void              SetMaxIter(int);
	int				  SetTollType(TNM_TOLLTYPE tl);
	void              SetCentroidsBlocked(bool cb) {
		network->centroids_blocked=cb;
		if(network->centroids_blocked)
		{
			for(int i=0;i<network->numOfOrigin;i++)
			{
				TNM_SORIGIN* orgn=network->originVector[i];
				//cout<<"o"<<orgn->origin->id<<" ";
				orgn->origin->SkipCentroid=true;
				for(int j=0;j<orgn->numOfDest;j++)
				{
					TNM_SDEST* dest=orgn->destVector[j];
					dest->dest->SkipCentroid=true;
				}
			}
		}
	}//If or not the center is closed when searching for paths
	void			  SetCostCoef(double t, double d) 
	{
		if(t > 0) timeCostCoefficient = t;
		if(d >=0) distCostCoefficient = d;
	}
	virtual int       Report(); //All report functions are packed in it.
protected:
	void			  ReportIter(ofstream &out); //report iteration history to a file
	virtual void	  ReportLink(ofstream &); 
	virtual void	  ReportPath(ofstream &); 

	double			  timeCostCoefficient;
	double			  distCostCoefficient;

/*========================================================================================
	Major methods: finding optimal solutions to the given model with given algorithm.
  ========================================================================================*/
public:
	virtual int       Build(const string& inFile, const string& outFileName, MATINFORMAT);
	TERMFLAGS		  Solve();     //public function: find optimal solution 
	virtual void      PreProcess(); //this function is used for performing some operation before initialze
	virtual void      Initialize() {;} //must be overloaded in derived classes for solve
	virtual void      MainLoop() {;}   //must be overloaded in derived classes for solve
	virtual bool      Terminate();     //must be overloaded in derived classes for solve
	virtual void	  PostProcess() {;} //this function is used for performing some operation after Solve
	virtual TERMFLAGS TerminationType();         //determined termination type.
	ITERELEM*         RecordCurrentIter();  //record the information of current iterations
	//virtual void      StoreResult() {;}
	void              ClearIterRecord();
	virtual void      ColumnGeneration(TNM_SORIGIN* pOrg,TNM_SDEST* dest);
	floatType		  RelativeGap(bool scale = true);
	floatType		  RelativeGap2(bool scale = true);
	virtual void	  ComputeOFV(); // overloaded objective function computation
	double			  ComputeBeckmannObj(bool toll = false); //compute and return the classic beckmann objective function vlaue
	

};

class TNM_EXT_CLASS TAP_Greedy: public TNM_TAP
{
public:
	TAP_Greedy();
	~TAP_Greedy();

	virtual void      PreProcess();
	virtual void      Initialize();
	virtual void	  MainLoop();
	virtual void	  UpdatePathFlowGreedy(TNM_SORIGIN* pOrg,TNM_SDEST* dest);
	virtual void	  QuickSortPath(vector<TNM_SPATH*> &,int low, int high);
	virtual void	  PostProcess();

	double			  TotalFlowChange;
	int				  numOfPathChange;
	double			  totalShiftFlow;
	double			  maxPathGap;
	bool			  columnG;
	double			  innerShiftFlow;
	int				  numOfD;
	vector<TNM_SPATH*> dePathSet;
	double			  aveFlowChange;
	int				  nPath;
};

//An alternative implementation of Greedy algorithm with the shortest path being the one-to-one Dijkstra algorithm. This version of implementation better fits the sparse OD matrix

class TNM_EXT_CLASS TAP_Greedy_dijk : public TAP_Greedy
{
public:
	TAP_Greedy_dijk();
	~TAP_Greedy_dijk();

	
	virtual void      Initialize();
	virtual void	  MainLoop();
	virtual void      ColumnGeneration(TNM_SORIGIN* pOrg, TNM_SDEST* dest);
	
	


};


#endif
