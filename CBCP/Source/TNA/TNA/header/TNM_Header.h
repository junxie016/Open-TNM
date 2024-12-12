/*======================================================================================
   TNM: Transportation network modelling

   file tnm_header.h: header file in which some critical definitions are made

   Yu Nie
   UC Davis
   latest update Sep, 2004
 ======================================================================================*/
#ifndef TNM_HEADER_H
#define TNM_HEADER_H
typedef long double LINK_COST_TYPE;
typedef int    LINK_COST_INT;
typedef long double floatType;
typedef char   tinyInt;  // 0 ~ 255, one byte
typedef int    smallInt; // 0 ~ 65,535, two byte
typedef long   largeInt; // 0 ~ 4, 294,967,295, four byte
enum  COSTUNIT {CU_HOUR, CU_MINUTE, CU_SECOND};
const          LINK_COST_TYPE INF_LINK_COST = 999999999.0;
const long     POS_INF_INT					= 999999999;
const long     MAX_DEVPATH					= 10000; //the maximum permitted checked path when searching K-shortest paths.
const double   POS_INF_FLOAT				= 1e15;
const double   POS_SMALL_FLOAT				= 1e-9;
const int      MAX_LINE_BYTES				= 1024;
const double   PI                           = 3.14159265358979323846;


//const int      LINKCOST_SCALAR              = 60; //only used for static networks. 1 means link cost is measured by hour, = 60 means it is measured by minutes; = 3600 means it is measured by seconds.
enum TNM_REGIME {
				  RM_STATIC,
				  RM_STATICMC,
				  RM_MACROS,
				  RM_MICROS,
};
enum TNM_LINKTYPE {
	               BASLK,  //basic static link type
           //static link types
                   BPRLK,  //link cost are evaluated by BPR type speed-flow function. 
				   CPBPR,  //capacitated bpr link;
				   ACHLK,  //link cost tested in A. Chen's paper. this is just for test purpose
				   CSTLK,  //link cost keep constant.
				   LINLK,  //linear type link cost.
				   EXPLK,  // exponential link cost function.
		   //dynamic link types
                   LWRLK,  //cell transimission dynamics
				   SQULK,  //spatial queue model:  point queue + queue spillover
			       PQULK,  // point queue model:  no queue spillback.
                   LICLK,  // LWR link with infinity holding capacity, namely, there is no queue spillback
				   LWRNF,  // A non-fifo lwr link, this is basically added to test wenlong's model.
				   DFMLK,  //delay function model
 				   DMOLK,  // dummy origin link
				   DMDLK,   // dummy destination link

		   //microscopic link types
				   MCALK, // for celluar automata.
		   //probability link type:
		           PRBLK, //for basic probability application (TNM_PLINK).
				   GCMLK,
           //Tranist link type;
                   PTRLK, //for basic transit link;

		   //BCPCC problem link type
		           BCPLK
                   };
enum TNM_NODETYPE{
	              BASND,   //basic static node type, no other static node types being define yet.
				  //dynamic node types
				  FWJFI,   //Freeway junction with restrict FIFO.  
                  FWJCT,   //free way junction, merge/diverage, or combined
				  CTLGN,   //a new generalied control node, it is intended to replace the CTLGC node.
				  CTLSN,   //controlled node, controller type: stop sign
				  CTLPT,   //controlled node, controller type: pretimed
				  CTLAD,   //controlled node, controller type: adaptive (actuated)
				  CTLRM,   //controlled node, ramp meter
				  CTLGC,   //controlled node, general controlled node
				  DMOND,   //dummy origin node
				  DMDND,    //dummy destination node
			  //microscopic node types
				  MCAND,   // for celluar automata;
				  MSTOP,   // microspioc stop sign control
				  PASND,
				  PASND_DMA, //especially for most diverse approach in stochastic on-time arrival problem
              //transit node type;
                  PTRND //basic transit node type;
                  };

enum TNM_TOLLTYPE {
				   TT_NOTOLL, //no toll 
				   TT_MTTOLL, //first best toll for minimizing travel time.
				   TT_MCTOLL, //first best toll for minimzing travel cost.   TT_MTTOLL = TT_MCTOLL when all users have the same value of time. 
				   TT_FXTOLL,  //fixed toll. 
				   TT_MXTOLL   //mixed toll strategy (different links are allowed for different toll strategies).
};
/*control method*/
enum CONTROLMETHOD{PRETIME, VEHACT, OPAC, RHODES, ALINEA, SWARM};
enum GENETYPE {CYCLE, PHASE, MRATE, OFFSET};  //Aug 2005, 
enum LSCTYPE {ONRLSC, OFFLSC}; //Aug 2005, if the ramp is on ramp, it is ONRLSC, otherwise, OFFLSC

//datastruct defines shortest path algorithsm. basically, it states how
//the scanning list is impmented.
enum DATASTRUCT {DEQUE,   //deque structure
                 QUEUE,   //queue structure
				 TWOQUE,  //two quque
				 SLFQUE,  //smaller label first queue
				 THRESH,  //threshhold
	             DIJKST,  //dijkstra algorithm
				 BUCKET,  //bucket structure
				 BIHEAP,  //binary heap
				 FIHEAP,  //Fibonacci heap
				 RAHEAP   //radix heap
                 };       //to be extended
//for equilibrium type.
enum EQUITYPE   {DUE, DSO, SUE}; // DUE -- deterministic user equilibrium
                                 // DSO -- deterministic system optimal
                                 // SUE -- stochastic user equilibrium
#define TNM_DEBUG
#endif

