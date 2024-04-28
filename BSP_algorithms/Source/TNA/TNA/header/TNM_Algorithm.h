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
	int               ziggIter;      // iteration which does not show signifciant decrease
	int               badIter; // bad iteration, means the number of iteration in which objective is increased
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
	void			  ColumnGeneration(TNM_SORIGIN* pOrg,TNM_SDEST* dest);
	floatType		  RelativeGap(bool scale = true);
	virtual void	  ComputeOFV(); // overloaded objective function computation
	double			  ComputeBeckmannObj(bool toll = false); //compute and return the classic beckmann objective function vlaue
	

};


/*   continuous VOT   */
class TNM_EXT_CLASS  VOT_DIST
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
	floatType     vot_pdf(TNM_SDEST*);//calc gradient used in dist calculation
	floatType     MMBA1_Ptheta(floatType);//\int \theta\cdot g(\theta)d\theta
	floatType	  MMBA1_gtheta(floatType);
	floatType     MMBA1_gprimetheta(floatType);
	floatType     MMBA1_Gtheta(floatType);
	void          calc_cdf_gra(TNM_SNET*);// save in dest buffer[0]
};


/* Bicriteria Shortest Path Problem with continuous VOT.*/
class TNM_EXT_CLASS  TAP_MMBA1_o: public TNM_TAP
{
public:
	               TAP_MMBA1_o();
				   TAP_MMBA1_o(floatType, floatType, string, int);
	virtual        ~TAP_MMBA1_o();
public:
	VOT_DIST*      votdist;
	floatType      tempvot;
	floatType      theta_min;
	floatType      theta_max;
	floatType      total_shift_flow;
	floatType      sumo_path;
	floatType	   theta_up; //the upbound for the current shortest path tree
	floatType      theta_low; //the lowbound for the current shortest path tree
	floatType      old_theta_low;
	floatType      aveInnerGap;
	int            spptreenum;
	deque<TNM_SNODE*> Dset; // D set used  in T2-PTU and T2-LTA
	deque<TNM_SNODE*> Hset; // H set used  in T2-PTU and T2-LTA
	map<int, map<int, TNM_SDEST*>> node_dest;
	vector<int>    tree_num;
	void           cout_tree_num(string outname);
	int            periter_addpathcount;
	int            total_pathset_size;
	int            max_pathset_size;
	double         shortestpathtime;
	double         truespptime;
	double         ltatime;
	double         ltachecktime;
	double         withoutgaptime;
	string         algorNameCard;
	int            likelysppcount;
	int            periter_deletepathcount;
	int            usegap_columngeneration;

	virtual void   Initialize();
	virtual void   MainLoop();
	int            MPA_path(TNM_SORIGIN*, int, int);
	int            modified_T2_shortestpath(TNM_SNODE *,vector<floatType> &);
	int            modified_T2_MPA_path(TNM_SORIGIN*, int, int);
	virtual int    modified_T2_LTA_path(TNM_SORIGIN*, int, int, vector<TNM_SLINK*> &, floatType);
	int            T2_MPA_link(TNM_SORIGIN* origin, int allornothing, int addset);
	int            modified_T2_MIN_path(TNM_SORIGIN*);
	int            modified_T2_PTU_path(TNM_SNODE *);
	TNM_SPATH*     revert_SPath_lasttree(TNM_SNODE *, TNM_SNODE *, vector<TNM_SLINK*> &);
	bool           add_to_pathset_path(TNM_SDEST*, TNM_SPATH*);
	void           build_node_dest();
	floatType      setdecimal(floatType, int);
	floatType      setdecimal_lower(floatType, int);
	floatType      setdecimal_round(floatType, int);
	TNM_SPATH*     revert_SPath(TNM_SNODE*, TNM_SNODE*);
	void           update_pathlinkflow(TNM_SPATH*, floatType);
	void           MMBA1_o_GAP();
	void           MMBA1_o_obj();

	void           printpath(TNM_SPATH*);
	floatType      MMBA1_innerGap_OD(TNM_SDEST* dest);
	void           Set_addpathbound(TNM_SDEST*);
	bool           check_money(TNM_SDEST*);
	void           boundadjusting_OD(TNM_SDEST*);
	void           adjust_one_bound(TNM_SDEST*, int, TNM_SPATH*, TNM_SPATH*);
	void           deletenousepath_OD(TNM_SDEST*);
	void           init_pathElemVector(int);
	void           bi_spp_path_output(const string &path, const string &name, const string &algname, double usetime);
	int            cout_Q_node_count( const string &path, const string &name, const string &algname);
	void           cout_Q_node_count_origin( const string &path, const string &name, const string &algname);
	void           Likelypath(TNM_SORIGIN*, TNM_SDEST* ,int);
	TNM_SPATH*     updateMMSPOD(TNM_SORIGIN*, TNM_SDEST*, floatType, int);
	TNM_SPATH*     GetSPath(TNM_SNODE*, TNM_SNODE*, int);
	void           MMall_or_nothing(TNM_SDEST*);

};


#endif
