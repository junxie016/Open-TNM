#ifndef TNM_LEGACY_ALGORITHMS_H
#define TNM_LEGACY_ALGORITHMS_H

const double POS_LARGE_FLOAT = 1e9;


struct TNM_EXT_CLASS PairSegment_ly
{
	int Pid;
	deque<ORGLINK*> seg1;
	deque<ORGLINK*> seg2;
	deque<ORGNODE*> oNodeList1;
	deque<ORGNODE*> oNodeList2;
	TNM_SNODE* begin;
	TNM_SNODE* end;
	floatType flow1;
	floatType flow2;
	floatType time1;
	floatType time2;
	floatType der1;
	floatType der2;
	floatType shiftFlow;
	floatType minTotalFlow1;
	floatType minTotalFlow2;
	floatType costDif;
	int OriginLabel;
	int threadNo;
};

struct TNM_EXT_CLASS PairSegment_struct
{
	int Pid;
	vector<PairSegment_ly*> PASoriginList;
	TNM_SNODE* begin;
	TNM_SNODE* end;
};

struct TNM_EXT_CLASS PasNode_ly
{
	TNM_SNODE* NodePtr;
	vector<PairSegment_struct*> NPasList;
};

class TNM_EXT_CLASS TAP_iGP : public TNM_TAP
{
public:
	TAP_iGP();
	virtual ~TAP_iGP();
	virtual void Initialize();
	virtual void MainLoop();
	virtual void UpdatePathFlowLazy(TNM_SORIGIN* pOrg, TNM_SDEST* dest);
	double ComputeEntropyfromPath();
	void PrintPath(TNM_SPATH* path);
	int PrepareReport() { return 0; }
	const vector<ITERELEM*>& GetIterRecord() const { return iterRecord; }

	bool recordODPath;
	bool columnG;
	int nPath;
	double TotalFlowChange;
	double aveFlowChange;
	int numOfPathChange;
	double totalShiftFlow;
	double innerShiftFlow;
	double maxPathGap;
	int numOfD;
	vector<TNM_SPATH*> dePathSet;
	int ziggIter;
	int badIter;

protected:
	virtual void PostProcess();
};

class TNM_EXT_CLASS TAP_iOEMA : public TAP_iTAPAS
{
public:
	TAP_iOEMA();
	virtual ~TAP_iOEMA();
	virtual void Initialize();
	virtual void MainLoop();

	void FromOrigintoNode(TNM_SNODE*);
	void BFSOriginToNode(PasOrigin2*, TNM_SNODE*, floatType);
	void BFSNodeToOrigin(PasOrigin2*, TNM_SNODE*, floatType);
	void FromNodeToOrigin(TNM_SNODE*);
	virtual void CalLinkApproach();
	void UpdateOrigins(TNM_SNODE*);
	void UpdateOrigins2();
	floatType DeleteCycleFlow(TNM_SNODE*, PasOrigin2*);
	double RemoveNodeBasedCycle(TNM_SNODE*);
	double RemoveNodeBasedCycle2(TNM_SNODE*);
	void NodeDFS(TNM_SNODE*);

	void BuildAllPath(TNM_SORIGIN*, PasOrigin2*, floatType);
	void GotoNextNode(TNM_SNODE*, TNM_SORIGIN*, PasOrigin2*, floatType);
	void BuildSpecialPath(TNM_SORIGIN*, PasOrigin2*, floatType);
	void GotoNextNode_buildSpecialPath(TNM_SNODE*, TNM_SORIGIN*, PasOrigin2*, floatType);
	void SaveProSolution();
	void SaveOriginLinkFlow(const string& fileName);
	int Reportpath();

	void ReadTAPASOUE();
	void CheckLinkFlow();
	virtual floatType CalEntropy();
	virtual floatType ComputeConsistency();
	virtual int PrepareReport();
	void ReportEntropyIter(ofstream& out);
	virtual int Report();
	void SaveOUESolution();
	void SaveUESolution();

	vector<floatType> nodeFlowVector;
	vector<floatType> linkFlowVector;
	vector<floatType> linkApproachVector;
	vector<PasOrigin2*> originVector;
	int numOfChangedLink;
	floatType changedFlow;
	floatType flowChangePrecision;
	floatType aveNodeChange;
	int numOfChangedNode;
	floatType lostFlow;
	floatType deFlow;
	map<TNM_SNODE*, TNM_SDEST*> node2dest;
	int count_ofbuildpath;
	int ziggIter;

	vector<ProInfo*> EntropyInfoList;
	map<int, TNM_SORIGIN*> originMap;
	floatType entr;
	floatType preEntr;
	int OUEfileType;
	ofstream EntropyIterFile;
};

class TNM_EXT_CLASS MEUE_DOB : public TAP_iTAPAS
{
public:
	MEUE_DOB();
	virtual ~MEUE_DOB();
	virtual void PreProcess();
	virtual void PostProcess();

	void SetInipntCoe();
	void SetiniMultipler();
	void UpdateMultipler();
	void UpdateFlowInfeasibility();
	floatType GetRelativeGAP();
	void CalculateOFV();
	void InitialSolution_tapasoue();
	void ReadTAPASOUE();
	void ComplementUEBushII();
	void CheckLinkFlow();
	void ClearSolution();
	void SaveUESolution();
	void SaveMEUESolution();
	void SaveOriginLinkFlow(const string& fileName);

	void UpdateEntropyII(TNM_SORIGIN* orgn);
	floatType BuildPathTreeII(TNM_SORIGIN* orgn);
	floatType BuildPathTree_PAS(TNM_SORIGIN* orgn);
	floatType PasFlowShift(PairSegment_ly* pas);
	floatType GetOlinkEntro(TNM_SLINK*);
	floatType GetOlinkfdEntro(TNM_SLINK*);
	floatType GetOlinkEntro(ORGLINK*, ORGNODE*);
	floatType GetOlinkfdEntro(ORGLINK*, ORGNODE*);

	void OpenRecordField();
	void RecordIterResult();
	void CloseRecordFile();
	void AnalysisLinkFlowError();
	void BuildAllPath(TNM_SORIGIN* orgn);
	void GotoNextNode(TNM_SNODE* node, TNM_SORIGIN* orgn);

	void SetMaxMainGapConv(floatType e) { MaxMainGapConv = e; }
	void SetMaxMainFIConv(floatType e) { MaxMainFIConv = e; }
	void SetMaxSubConv(floatType e) { MaxSubConv = e; }
	void SetMaxSubIter(int e) { MaxSubIter = e; }

	floatType Gap;
	floatType RG;
	floatType FI;
	floatType MaxFI;
	int SubIter;
	bool reportbushflow;
	bool reportpathflow;
	int addbushlink;
	floatType MaxInnerConv;
	int iniiter;
	int totalpas;
	int numpas;
	int validpas;
	bool FIsatisfied;
	int OUEfileType;
	map<TNM_SNODE*, TNM_SDEST*> node2dest;

protected:
	floatType pntCoe;
	floatType MaxMainGapConv;
	floatType MaxMainFIConv;
	floatType MaxSubConv;
	int MaxSubIter;
	floatType alpha;
	clock_t begintime;
	clock_t currenttime;
	vector<ORGLINK*> NSPLinkVector;
	vector<string> ly_IterResult;
	vector<string> ly_BushFlowResult;
	ofstream outf;
	ofstream outBushFlowf;

	int ziggIter;
	int badIter;
	floatType ziggIndicator;
	floatType m_OOFV;
	floatType m_BOFV;
	floatType m_OConv;
	floatType m_BConv;
};

class TNM_EXT_CLASS MEUE_TAPAS : public MEUE_DOB
{
public:
	MEUE_TAPAS();
	virtual ~MEUE_TAPAS();
	virtual void PreProcess();
	virtual void Initialize();
	virtual void MainLoop();
	virtual void SubLoop_ALM();
	virtual floatType GeneratePASII(TNM_SORIGIN* orgn);
	virtual bool ExistPas(TNM_SNODE* node, PairSegment_ly* pas);

	vector<PasNode_ly*> PNList_ly;
	vector<PairSegment_struct*> PasList_ly;
	int PasStructID;
	int oldcurIter;
	int rt;
	bool pre0;
	bool pre2;
	bool pre4;
	bool pre6;
	bool pre7;
};

#endif
