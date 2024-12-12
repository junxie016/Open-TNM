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
	TNM_SNET*         network2;
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
	virtual int       Report(); //All report functions are packed in it.
protected:
	void			  ReportIter(ofstream &out); //report iteration history to a file
	virtual void	  ReportLink(ofstream &); 
	virtual void	  ReportPath(ofstream &); 

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
	void			  ColumnGeneration(TNM_SORIGIN* pOrg,TNM_SDEST* dest);
	floatType		  RelativeGap(bool scale = true);
	virtual void	  ComputeOFV(); // overloaded objective function computation
	double			  ComputeBeckmannObj(bool toll = false); //compute and return the classic beckmann objective function vlaue
	

};

class TNM_EXT_CLASS TAP_iGP: public TNM_TAP
{
public:
	TAP_iGP();
	~TAP_iGP();

	virtual void      PreProcess();
	virtual void      Initialize();
	virtual void	  MainLoop();
	virtual void	  UpdatePathFlowLazy(TNM_SORIGIN* pOrg,TNM_SDEST* dest);
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

class TNM_EXT_CLASS TAP_BCP: public TAP_iGP
{
public:
	TAP_BCP();
	~TAP_BCP();

	//methods for solving the assignment-2 problem, i.e., the problem (16) in the paper
	void  PreProcess2();
	void  Initialize2();
	void  Initialize22();
	void	  MainLoop2();
	void	  UpdatePathFlowLazy2(TNM_SORIGIN* pOrg,TNM_SDEST* dest);
	void	  PostProcess2();
	void UpdateLinkCost2();
	void UpdateLinkCostDer2();
	TERMFLAGS SolveAssingment2();
	TERMFLAGS SolveAssingment22();
	double ComputeOFV2();

	//methods for solving the assignment-1 problem, i.e., the subproblem (14) in the paper

	void  PreProcess1();
	void  Initialize1();
	void  Initialize12();
	void	  MainLoop1();
	void	  UpdatePathFlowLazy1(TNM_SORIGIN* pOrg,TNM_SDEST* dest);
	void	  PostProcess1();
	void UpdateLinkCost1();
	void UpdateLinkCostDer1();
	TERMFLAGS SolveAssignment1(); //the path-based algorithm for solving the TAP Problem (4)
	TERMFLAGS SolveAssignment12(); //the path-based algorithm for solving the TAP Problem (4) with warm start
	double ComputeOFV1();

	void AlgorithmPGBB();

	void FixedAlgorithmPGBB();

	double ComputePGBBgap();

	//methods for the whole problem

	void AlgorithmBCPCC();
	void FixedAlgorithmBCPCC();

	void PostTollLevelOptimizaiton();

	void QuickSort(vector<TNM_SLINK*> &, int  , int );
	double ComputeTotalCost();
	//bool CheckConvergence();
	//variables and paramters
	double myRow1;
	double myRow2;
	double a_min; // parameters for PGBB algorithm
	double a_max; // parameters for PGBB algorithm
	double a_stepsize;// variables for PGBB algorithm
	double eta; //parameters for the PGBB algorithm

	//parameters for the pd algorithm
	double pd_gamma;
	double pd_gamma2;
	//parameters and variables for the BCPCC algorithm
	int tolledlinknum;

	double U; //upper bound for the 
	int kappa;
	double Upsilon;
	double currentups;
	int numofAssignment1;
	int numofAssignment2;

	//
};

#endif