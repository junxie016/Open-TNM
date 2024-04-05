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
	string            algorNameCard; //this is used for print purpose
	int          m_orgFlowReportID; // = -1 means all.
	bool              m_reportOrgFlows; //report origin-based flows.
	bool         m_evRGap; //whether or not to evaluate relative gap.
	bool              readyToReport;
	bool              m_printVersion;
	string            modelNameCard;
	ofstream          sumFile;    //summary of comptation facts
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
	ofstream      m_orgFile;

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
		network->centroids_blocked = cb;
		if (network->centroids_blocked)
		{
			for (int i = 0; i < network->numOfOrigin; i++)
			{
				TNM_SORIGIN* orgn = network->originVector[i];
				//cout<<"o"<<orgn->origin->id<<" ";
				orgn->origin->SkipCentroid = true;
				for (int j = 0; j < orgn->numOfDest; j++)
				{
					TNM_SDEST* dest = orgn->destVector[j];
					dest->dest->SkipCentroid = true;
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
	bool              IsPrintVersion() { return m_printVersion; }
	void              EnablePrintVersion(bool v = true) { m_printVersion = v; }
	virtual int       CloseReport();
	void         EnableReportOrgFlows(bool org = true, int orgid = -1 /*-1 means all will be reported*/)
	{
		m_reportOrgFlows = org; m_orgFlowReportID = orgid;
	}
	bool         IsReportOrgFlows() { return m_reportOrgFlows; }
	void         EnableEvRGap(bool ev = true) { m_evRGap = ev; }
	bool         IsEvRGap() { return m_evRGap; }
	void              PrintVersion(ofstream&);	//utility function
	virtual      void ReportOrgFlows(ofstream& out); //origin-based flows;

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

/////////
// 
////////This construct is used to define the destinations saved in PasOrigin1
struct TNM_EXT_CLASS PasDest
{
	PasDest()
	{
		segFlowOne = 0;
		segFlowTwo = 0;
		segRatio = 0;
		dest = NULL;
	}
	TNM_SPATH* AddPathSegOne(TNM_SPATH* path)
	{
		bool ex = false;
		TNM_SPATH* red = NULL;
		for (int i = 0; i < PathSetOne.size(); i++)
		{
			TNM_SPATH* pa = PathSetOne[i];
			if (pa == path)
			{
				ex = true;
				red = pa;
			}
		}
		if (!ex)
		{
			PathSetOne.push_back(path);
			red = path;
		}
		return red;
	}
	TNM_SPATH* AddPathSegTwo(TNM_SPATH* path)
	{
		bool ex = false;
		TNM_SPATH* red = NULL;
		for (int i = 0; i < PathSetTwo.size(); i++)
		{
			TNM_SPATH* pa = PathSetTwo[i];
			if (pa == path)
			{
				ex = true;
				red = pa;
			}
		}
		if (!ex)
		{
			PathSetTwo.push_back(path);
			red = path;
		}
		return red;
	}
	//update the segflow for the two paths
	void UpdateSegmentPathFlow()
	{
		TNM_SPATH* path;
		//
		segFlowOne = 0;
		for (int i = 0; i < PathSetOne.size(); i++)
		{
			path = PathSetOne[i];
			segFlowOne += path->flow;
		}
		segFlowTwo = 0;
		for (int i = 0; i < PathSetTwo.size(); i++)
		{
			path = PathSetTwo[i];
			segFlowTwo += path->flow;
		}
	}
	//definition of variables
	TNM_SDEST* dest;
	vector<TNM_SPATH*> PathSetOne;
	vector<TNM_SPATH*> PathSetTwo;
	floatType segFlowOne;
	floatType segFlowTwo;
	floatType segRatio;
};

// ///// this construct is used to define the origins saved in a PAS
struct TNM_EXT_CLASS PasOrigin1
{
	PasOrigin1()
	{
		org = NULL;
		minFlow1 = 0;
		minFlow2 = 0;
		largerMin = 0;
		sflow = 0;
		zeroShift = 0;
		proportion = 0;
		a = 0;
		b = 0;
		c = 0;
		sum = 0;
		proDif = 0;
		boundPro = 0;

		xSeg1 = 0;
		xSeg2 = 0;
		ySeg1 = 0;
		ySeg2 = 0;
		dSeg1 = 0;
		dSeg2 = 0;
		SegFlowOne = 0;
		SegFlowTwo = 0;


	}
	~PasOrigin1()
	{
		for (int i = 0; i < DestVector.size(); i++)
		{
			PasDest* dest = DestVector[i];
			delete  dest;
		}
	}
	//
	PasDest* AddDest(TNM_SDEST* dest)
	{
		bool ex = false;
		PasDest* red = NULL;
		for (int i = 0; i < DestVector.size(); i++)
		{
			PasDest* de = DestVector[i];
			if (de->dest == dest)
			{
				ex = true;
				red = de;
				break;
			}
		}
		if (!ex)
		{
			PasDest* de = new PasDest;
			de->dest = dest;
			DestVector.push_back(de);
			red = de;
		}
		return red;
	}
	//variables
	TNM_SORIGIN* org;
	floatType minFlow1;//the minimum origin-based link flow of segment 1 related to the current origin
	floatType minFlow2;//the minimum origin-based link  flow of segment 2 related to the current origin
	floatType largerMin;
	floatType sflow;//the flows need to be shifted related to the current origin
	int zeroShift; // to record the number of shift that the flow is zero, update in function: 
	floatType proportion;
	floatType boundPro;
	floatType gs1;//flow of segment related to this origin, used for proportionality
	floatType gs2;
	floatType a;  //parameters for the quadratic approximation of origin flow redistribution
	floatType c;
	floatType b;
	floatType dp;//the deviation of flow on segment 1
	floatType sum; //sum of gs1 and gs2
	floatType proDif; // the difference of current proportion with the average value of all origins
	floatType xSeg1;
	floatType ySeg1;
	floatType xSeg2;
	floatType ySeg2;
	floatType dSeg1;
	floatType dSeg2;
	floatType SegFlowOne;
	floatType SegFlowTwo;
	vector<PasDest*> DestVector;
};
// 
class TNM_EXT_CLASS PairSegment2
{
public:
	PairSegment2();
	~PairSegment2();

	TNM_SLINK* getmlSeg1(); // return the front link of segment1
	TNM_SLINK* getmlSeg2();

	//
	int LinkSegment(TNM_SLINK*);
	bool addOrigin(TNM_SORIGIN*); // add an origin into this PAS
	PasOrigin1* addOriginNew(TNM_SORIGIN*);
	void eraseOrigin(TNM_SORIGIN*); // delete an origin from this PAS
	floatType findHigher(); //cal segment time and return the higher segment
	floatType calShiftFlow(); // compute the shift flow of this PAS
	void updatePasFlow(); // update the 
	void showPas(); // show the details of the PAS
	void ClearPAS();

public:
	int Pid;
	deque<TNM_SLINK*> seg1; // link vector of segment 1
	deque<TNM_SLINK*> seg2;
	TNM_SNODE* begin;//the begin node of pas
	TNM_SNODE* end;//the end node of pas
	floatType flow1;
	floatType flow2;
	floatType time1; // the time of segment 1
	floatType time2;
	floatType der1; //derivative of segment 1
	floatType der2;//derivative of segment 2
	floatType shiftFlow;// the flow that shift
	vector<PasOrigin1*> OriginList;
	int higher; //show the higher segment
	floatType minTotalFlow1;
	floatType minTotalFlow2;
	int zShift;//check if the shifting flow between segments are zero, if it is , then zShift ++; update in function: doshiftflow
	bool active; //check if this pas is active, default is active
	floatType proportion;
	bool equilibrated;
	floatType costDif;
	floatType prodif;
	int OriginLabel;
	bool ProStatus;
	floatType Pro2;
	floatType ta;
	floatType tb;
	floatType tc;
	bool linear;
	floatType boundPro;
	floatType bProDif;
	int newSegment;

};
// used in TAPAS, a Origin List is maintained in TAPAS, and the PasOrigin2 is the objects related to origins in this list
struct TNM_EXT_CLASS PasOrigin2
{
	PasOrigin2()
	{
		OriginPtr = NULL;
		id = -1;
		oFlow = 0.0;
	}
	//delete pas from this
	void ErasePas(PairSegment2* pas)
	{
		for (vector<PairSegment2*>::iterator iter = OPasList.begin(); iter != OPasList.end(); )
		{
			if (*iter == pas)
			{
				iter = OPasList.erase(iter);
			}
			else
				iter++;
		}
	}
	//add pas from this
	void AddPas(PairSegment2* pas)
	{
		int lag = 0;
		for (int i = 0; i < OPasList.size(); i++)
		{
			if (pas == OPasList[i])
			{
				lag = 1;
			}
		}
		if (lag == 0)
		{
			OPasList.push_back(pas);
		}
	}
	//variables
	TNM_SORIGIN* OriginPtr;
	vector<PairSegment2*> OPasList;
	vector<ORGLINK*> olinkList;
	vector<floatType> onodeflow;//origin-based node flow for this origin
	vector<bool> oLinkLable;
	//variables for OEMA
	vector<floatType> oLinkFlowVector;
	vector<floatType> oNewLinkFlowVector;
	vector<floatType> oLinkApproachVector;
	//vector<int> changedLinkVector;
	map<int, floatType> changedLinkVector;
	floatType oFlow;
	int id;

};

/////////
struct TNM_EXT_CLASS PasNode2
{
	PasNode2()
	{
		NodePtr = NULL;
	}

	//delete pas
	void ErasePas(PairSegment2* pas)
	{
		for (vector<PairSegment2*>::iterator iter = NPasList.begin(); iter != NPasList.end(); )
		{
			if (*iter == pas)
			{
				iter = NPasList.erase(iter);
			}
			else
			{
				iter++;
			}
		}
	}
	//add pas
	void AddPas(PairSegment2* pas)
	{
		int lag = 0;
		for (int i = 0; i < NPasList.size(); i++)
		{
			if (pas == NPasList[i])
			{
				lag = 1;
			}
		}
		if (lag == 0)
		{
			NPasList.push_back(pas);
		}
	}

	//////////variables
	TNM_SNODE* NodePtr;
	vector<PairSegment2*> NPasList;
};


struct TNM_EXT_CLASS DirCycle
{
	DirCycle()
	{
		cost = 0.0;
		minFlow = 0.0;
	}
	floatType UpdateCost()
	{
		floatType tc = 0;
		for (int i = 0; i < cycle.size(); i++)
		{
			TNM_SLINK* sl = cycle[i];
			tc += sl->cost;
		}
		cost = tc;
		return tc;
	}
	//

	void UpdateMinFlow()
	{
		floatType mf = 10000000000000;
		for (int i = 0; i < cycle.size(); i++)
		{
			TNM_SLINK* sl = cycle[i];
			if (mf > sl->oLinkPtr->oFlow)
			{
				mf = sl->oLinkPtr->oFlow;
			}
		}
		minFlow = mf;
	}
	//
	bool Check()
	{
		bool good = true;
		for (int i = 0; i < cycle.size(); i++)
		{
			if (cycle[i] == NULL || cycle[i]->oLinkPtr == NULL)
			{
				good = false;
			}
		}
		return good;
	}
	//
	void Print()
	{

		for (int i = 0; i < cycle.size(); i++)
		{
			cout << cycle[i]->head->id << "<-" << cycle[i]->tail->id << "  ";
		}
		cout << endl;
	}
	void SetStatus()
	{
		for (int i = 0; i < cycle.size(); i++)
		{
			TNM_SLINK* sl = cycle[i];
			sl->markStatus = 0;
		}
	}
	//variables
	deque<TNM_SLINK*> cycle;
	floatType cost;
	floatType minFlow;
};


///////////////////////////
struct TNM_EXT_CLASS CycleInfo
{
	int Iter;
	int PreCycles;
	int InCycles;
	floatType PreFlow;
	floatType InFlow;

	CycleInfo()
	{
		Iter = 0;
		PreCycles = 0;
		InCycles = 0;
		PreFlow = 0;
		InFlow = 0;
	}

};

//////////////////////////////////

struct TNM_EXT_CLASS PasInfo
{
	int Iter;
	int NumOfCreatePas;
	int NumOfMatch;
	int NumOfFirstDelete;
	int NumOfNewPas;
	int NumOfAveOrigins;
	int NumOfAvePasNode;
	int NumOfLinksSeg;
	int MaxLinkSeg;
	int MaxOriginPas;
	int NumOfPas;
	int NumOfdelete;
	int numOfShift;
	floatType TotalShiftFlow;
	floatType averageRatio;
	int TotalPasCreate;
	int NumOfMatchOrigin;

	PasInfo()
	{
		Iter = 0;
		averageRatio = 0;
		TotalPasCreate = 0;
		NumOfCreatePas = 0;
		NumOfMatch = 0;
		NumOfFirstDelete = 0;
		NumOfNewPas = 0;
		NumOfAveOrigins = 0;
		NumOfAvePasNode = 0;
		NumOfLinksSeg = 0;
		MaxLinkSeg = 0;
		MaxOriginPas = 0;
		NumOfdelete = 0;
		NumOfPas = 0;
		numOfShift = 0;
		TotalShiftFlow = 0;
		NumOfMatchOrigin = 0;
	}
};

////////////////////////ProInfo/////////////////////////////////////
struct TNM_EXT_CLASS ProInfo
{
	int iter;
	floatType Entro;
	floatType time;
	floatType proprotion;
	int consis;
	ProInfo()
	{
		iter = 0;
		Entro = 0;
		time = 0;
		proprotion = 0;
		consis = 0;

	}
};


class TNM_EXT_CLASS TAP_iTAPAS : public TNM_TAP
{
public:
	TAP_iTAPAS();
	~TAP_iTAPAS();

	virtual void Initialize();
	virtual void MainLoop();


	virtual floatType CalEntropy();
	virtual int MarkobLink(TNM_SORIGIN*, int i);//mark the POList 
	int RemoveCycle(TNM_SORIGIN*);
	virtual void DFSCycle(TNM_SNODE*, TNM_SORIGIN*);
	virtual void MarkUsedLink(TNM_SORIGIN*);//mark all used links which are in shortest path tree
	virtual int GeneratePAS(TNM_SORIGIN*);//generate PASs from the subnet
	virtual PairSegment2* matchPas(TNM_SORIGIN*, ORGLINK*);//find a exist pas for the olink
	int updatePasFlow(PairSegment2*);//update the flow in pas
	virtual floatType PasFlowShift(PairSegment2*); //equalize the pas cost
	int updatePasOrigin(PairSegment2*);
	floatType doShiftFlow(PairSegment2*);// second fucntion
	floatType flowShift(PairSegment2*, floatType);//third function :flow shift for each origin related to the pas
	virtual PairSegment2* createPas(TNM_SORIGIN*, TNM_SLINK*, floatType);
	virtual bool addSubnet(TNM_SORIGIN*, PairSegment2*, int);// add the links of pas in to the subnet 
	virtual bool ImmediatePasShift(PairSegment2*);
	virtual PairSegment2* CheckExist(PairSegment2*);
	virtual floatType LocalPasShift(TNM_SORIGIN*);
	int Random(int, int);
	virtual bool eliminatePas(PairSegment2*);
	int deletePas();
	virtual int  PrepareReport();
	void GeneratePathbyDFS();
	virtual void CalLinkApproach();
	void DFSPath(TNM_SNODE*, TNM_SORIGIN*, TNM_SDEST*);
	void SavePathtoFile();
	virtual int       Report(); //All report functions are packed in it.
	void ReportPas(ofstream& out);
	void ReportPasInfo(ofstream& out);
	void ReportCycleInfo(ofstream& out);
	void PrepareLogFile();

	//methods for proportionality
	virtual void PostProcess();
	virtual floatType ComputeConsistency();
	int PrePareProportion();
	virtual void PostProportionality();
	virtual PairSegment2* CreateProPAS(TNM_SLINK*, TNM_SORIGIN*);
	floatType UpdateSegFlow(PairSegment2*);
	floatType UnityProportion6(PairSegment2*, floatType); 
	floatType UpdateApproach(PairSegment2*);
	int ReducePAS();
	void SaveUESolution2();
	void SaveTAPASOUE2();
	void ReportProportion();

	//variables


	int numOfShift;
	floatType flowPrecision;
	floatType costPrecision;
	floatType flowPrecision2;
	bool lineSearch;
	int PasID;
	vector<PairSegment2*> PasList; //the total list of PAS in the net
	vector<PasOrigin2*> POList; //the list for Origins, size and order is equal to vectorOfOrigin
	vector<PasNode2*> PNList; //the list of pasNodes in the net
	vector<TNM_SLINK*> newAddLinks;
	floatType costfactor;
	floatType flowfactor;
	bool IsReportPAS;
	ofstream     PasFile;
	ofstream     PasInfoFile;
	ofstream    CycleInfoFile;
	bool acyclic;
	floatType TotalExcessCost;
	floatType AveExcessCost;
	int NumOfNoRegPas;
	floatType AvePasOrg;
	vector<TNM_SLINK*> BranchList;
	int RandomNum;
	int NumOfCycle;
	floatType branchFlow;
	vector<DirCycle*> BrList;
	vector<TNM_SLINK*> BadLinks;
	deque<TNM_SNODE*> CNList;
	deque<TNM_SLINK*> PotentialLinks;
	vector<PasInfo*> PasInfoList;
	vector<CycleInfo*> CycleInList;
	floatType PreCycleFlow;
	floatType BranchCycleFlow;

	bool findNewPas;
	int NumOfNewPas;
	int NumOfMatch;
	int NumofRepeat;
	int NumOfCycleBranch;
	int numofCycleCreate;
	floatType cycleFlowCreate;
	int Numofgood;
	int NumofNoBranch;
	floatType deFlow;
	int NumOfOverShift;
	int NumOfBranch;
	bool repeated;
	bool findcycle;
	int NumOfTotalShift;
	floatType TotalFlowShift;
	floatType RatioOfFlowLink;
	floatType NumOfCreatePas;
	floatType NumOfSegLink;
	floatType SegLinkFlowRatio;
	int badQua;
	bool ReadFromOUE; // check if it is readed from the OUE file
	bool DoProportionality;
	floatType convPro;
	vector<ProInfo*> ProInfoList;
	bool isReadProSolution;
	int pasNum;
	floatType entropyCostPrecision;
	floatType entropyFlowPrecision;
	ofstream logFile;
	floatType totalExcessPrecision;
	floatType zeroDeleteFlow;
	vector<PairSegment2*> LinkPasVector;
	bool headend;
	bool iscycle;
	int deCycle;

};


#endif