#ifndef TNM_ALGORITHM_H
#define TNM_ALGORITHM_H

#include "mat_header.h"
#include <cassert>
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
	vector<ITERELEM*> iterRecord; //modified for SBA a vector record iteration history
	int               badIter;
	int               maxbadIter;
	int               ziggIter;

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
	//inline  bool      ReachMaxBad() {return curIter>=maxbadIter;} //test if maximum bad iteration is attained
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

class TNM_EXT_CLASS VOT_DIST
{
public:
	VOT_DIST();
	VOT_DIST(floatType, floatType, string, int);//input min, max, name, treenumber
	virtual      ~VOT_DIST();
	floatType     vot_min;
	floatType     vot_max;
	floatType     vot_intervallength;
	int           treenumber;
	string        votdist_name;
	floatType     first_k;
	floatType     first_c0;
	floatType     first_c1;
	floatType     exp_lambda;
	floatType     rational_rho;
	floatType     rational_a;
	floatType     rational_b;
public:
	void          set_rho(floatType);
	floatType	  MMBA1_gtheta(floatType);
	floatType     MMBA1_Gtheta(floatType);
	floatType     MMBA1_gprimetheta(floatType);
	floatType     MMBA1_Ptheta(floatType);//\int \theta\cdot g(\theta)d\theta
};


class TNM_EXT_CLASS TAP_SBA: public TNM_TAP
{
public:
	TAP_SBA();
	TAP_SBA(floatType, floatType, string, int);
	virtual        ~TAP_SBA();
public:
	VOT_DIST*      votdist;
	floatType      theta_min;
	floatType      theta_max;
	int            spptreenum;
	double         shortestpathtime;
	double         truespptime;
	double         ltatime;
	double         ltachecktime;
	double         withoutgaptime;
	double         lta_specific_check_time;
	bool           useMPAstrategy;
	int            deletestrategy;
	floatType      sumo_path;
	floatType	   theta_up; //the upper bound for the current shortest path tree
	floatType      theta_low; //the lower bound for the current shortest path tree
	floatType      old_theta_low;
	floatType      aveInnerGap;
	int            typeofDistri;
	deque<TNM_SNODE*> Dset; // D set used  in T2-PTU and T2-LTA
	deque<TNM_SNODE*> Hset; // H set used  in T2-PTU and T2-LTA
	map<int, map<int, TNM_SDEST*>> node_dest;  // map node to the dest object
	vector<int>    tree_num;
	int            periter_addpathcount;
	int            total_pathset_size;
	int            max_pathset_size;
	floatType      total_shift_flow;
	int            periter_deletepathcount;
	int            likelysppcount;
	bool           delete_k_status;
	bool           delete_kplus1_status;
	int            totalDset;
	vector<TNM_SPATH*> depathset;
	bool           armijo_rule;

	// ***************common procedures************************************************************

	virtual void   Initialize();
	virtual void   MainLoop();
	// Calculate relative gap for the SBA algorithm where the parametric shortest path using the the PTU tree replacement strategy
	void           SBA_GAP(); 
	// Calculate the objective function value for SBA algorithm
	void           SBA_obj(); 

	// **************shortest path and column generation******************************************

	//Initialize the pathElemVector to generate multiple trees when needed
	void           init_pathElemVector(int);
	// Just use the shortest path label correcting algorithm to build the a shortest path tree under given parameters
	int            Generalized_cost_spp(TNM_SNODE *,vector<floatType> &);  
	// The parametric shortest path algorithm if we do not use the PTU tree replacement strategy.
	int            MPA_path_scrach(TNM_SORIGIN*, int, int);
	// The parametric shortest path proposed from Dial (1997)
	int            Dial_MPA_path(TNM_SORIGIN*, int, int);
	// Parametric shortest path tree lazy assignment
	int            Dial_LTA_path(TNM_SORIGIN*, int, int, vector<TNM_SLINK*> &, floatType);
	// The first time of the parametric shortest path tree replacement
	int            Dial_MIN_path(TNM_SORIGIN*);
	// Parametric shortest path tree replacement
	int            Dial_PTU_path(TNM_SNODE *);
	// Parametric shortest path used in link-based algorithm like FW
	int            Dial_MPA_link(TNM_SORIGIN* origin, int allornothing, int addset);
	// Parametric shortest path lazy assignment used in link-based algorithm like FW
	int            Dial_LTA_link(TNM_SORIGIN*, int, int, vector<TNM_SLINK*> , floatType);
	// Sort nodes by topology order
	void           T2_sortTopo(deque<TNM_SNODE*> &D,vector<TNM_SLINK*> const & pred);
	// Sort node recursive function
	void           quickSort1(deque<TNM_SNODE*> &order, int low , int high);
	// Constructing objects for the shortest path.
	TNM_SPATH*     revert_SPath(TNM_SNODE*, TNM_SNODE*);
	// Recover the shortest path in the tree species from the previous step
	TNM_SPATH*     revert_SPath_lasttree(TNM_SNODE *, TNM_SNODE *, vector<TNM_SLINK*> &);

	// ************************numerical operation*********************************************

	// Add a path to the path set
	bool           add_to_pathset_path(TNM_SDEST*, TNM_SPATH*);
	// Ceiling process for a number, e.g, num=2: 1.456->1.46, 1.783->1.79
	floatType      setdecimal(floatType, int);
	// Flooring process for a number, e.g, num=2: 1.456->1.45, 1.783->1.78
	floatType      setdecimal_lower(floatType, int);
	// Rounding process for a number, e.g, num=2: 1.456->1.46, 1.783->1.78
	floatType      setdecimal_round(floatType, int);

	// ************************network operation and output************************************

	// Update the flow of links that make up the path.
	void           update_pathlinkflow(TNM_SPATH*, floatType);
	// Map node to destination obj
	void           map_node_dest();
	//output the number of trees (in parametric shortest path) for each origin
	void           tree_num_output(string outname);
	// Print node id that included in path to check
	void           printpath(TNM_SPATH*);

	// **********************SBA operation*****************************************************

	// Set path boundaries after updating the path set.
	void           Set_addpathbound(TNM_SDEST*);
	// Check the toll order in the path set is decreasing.
	bool           check_money(TNM_SDEST*);
	// Boundary adjustment for a single OD pair
	void           boundadjusting_OD(TNM_SDEST*);
	// The adjustment of one boundary in the SBA algorithm
	void           adjust_one_bound(TNM_SDEST*, int, TNM_SPATH*, TNM_SPATH*);
	// Calculate the largest absolute first derivative value for an O-D pair as the inner gap.
	floatType      SBA_innerGap_OD(TNM_SDEST*);
	// Delete 0 flow path in the path set.
	void           deletenousepath_OD(TNM_SDEST*);
	// An adaptive inner loop procedure
	int            adaptive_inner(floatType, int, TNM_SDEST*);
	int            adaptive_inner_2(floatType, int, TNM_SDEST*);
	int            adaptive_inner_3(floatType, int, TNM_SDEST*);

};


class TNM_EXT_CLASS TAP_T2theta: public TAP_SBA
{
public:
	TAP_T2theta();
	TAP_T2theta(floatType, floatType, string, int);
	virtual        ~TAP_T2theta();
public:
	virtual void   Initialize();
	virtual void   MainLoop();
	void           T2theta_GAP();
	void           T2theta_Objective();
	int            T2theta_lineSearch(floatType);
	void           T2theta_updateSolution();
	floatType      L_function(floatType, int);
	floatType      GetBPRLinkTime(TNM_SLINK*, floatType);
	int            T2theta_ReadToll(const string & path);
	floatType      lineSearchAccuracy;
};

#endif