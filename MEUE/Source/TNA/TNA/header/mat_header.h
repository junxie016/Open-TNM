#pragma once

const int	 MINMAXITER = 0;
const int    MAXMAXITER = 1000000;
const double MINCONV    = 1e-16;
const double MAXCONV    = 1.0;
const double MINSTEP    = 1e-9;
const double MINLSCONV  = 1e-9;
const double MAXLSCONV  = 1.0;
const int    MINLSITER  = 1;
const int    MAXLSITER  = 1000;
const int    MINBITER   = 1;
const int    MAXBITER   = 1000;
const int    MINZITER   = 1;
const int    MAXZITER   = 1000;
const double MINZCONV   = 1e-16;
const double MAXZCONV   = 1.0;

enum TERMFLAGS {
	            InitTerm,  
                ConvergeTerm, 
				MaxIterTerm, 
				MaxBadTerm, 
				MinStepTerm, 
				ZiggTerm,
				ObjectTerm, //sometimes you may know what is the optimal OFV for a program, so you can terminate the program in this way
				UserTerm,//terminated by user. this is considered as normal termination
				ErrorTerm, //terminated due to errors.
				MaxCPUTerm
                };

//input file format:
enum MATINFORMAT {
				 NETFORT,  //The fort.1 forrt.2 format to describe a static network.
				 NETDANET2, //the danet.net, danet.nod, danet.lin, danet.odp,... to describe a dynamic network.
				 NETVISUM,//the visum format. 
				 NETVISUMS, //the visum standard.
				 NETTRANSCAD,
				 NETTAPAS
};
//methods and algorithms toolkit object ID enumeration.
enum MATOBJID 
{
			   MAT_ROOT_ID, //0 base
               GEN_IA_ID,  // 1. base object
			   TNM_IA_ID,  // 2. base object for all TNM-related object.
			   TNM_TAP_ID,   //3.  deterministic traffic assignment model.
			   TAP_BBA_ID,
			   TAP_FW_ID,   //4. solving TAP using FW,
			   TAP_FWMC_ID,
			   TAP_GP_ID,   //5. solveing TAP using GP,
			   TAP_CFGP_ID,
			   TAP_DOB_ID,
			   TAP_DOB_ME_ID,// for maximum entropy problem.
			   TAP_DOBMC_ID,
			   TAP_CFFW_ID,
			   TAP_BOB_ID,
			   TAP_TAPAS_ID,//for TAPAS 
			   TAP_NOB_ID,
			   TAP_QOB_ID,
               TAP_LUCE_ID,
               TAP_BOBX_ID,
               TAP_QOBLUCE_ID,
			   TNM_CTAP_ID,
			   TNM_MPEN_ID,  //6. maximization path entropy
			   MPEN_MC_ID,   //7. MPEN + minimization cost
			   MPEN_DC_ID,   //8. MPEN + demand constraint
			   MPEN_DC_MC_ID,   //9. MPEN + DC + MC
			   TNM_LTAP_ID,        //10. logit traffic assignment ( with fix cost and known path-link incidence
			   LTAP_RQ_ID,         //11. logit traffic assignment allowing residual queue
			   LTAP_RQ_MSA_ID,     //12. solving LTAP_RQ by MSA
			   LTAP_RQ_IBA_ID,     //13 solving LTAP_RQ by IBA
			   TNM_LPFE_ID,        //14 logit path flow estimator
			   LPFE_RQ_ID,         //15. LPFE with residual queue
			   LPFE_RQ_MSA_ID,     //16 solving LPFE_RQ by MSA
			   LPFE_RQ_IBA_ID,     //17. solving LPFE_RQ by IBA
			   TNM_LOGWRAP_ID,     //18 a wrapper for all logit models. It is higher than LOGWRAP_RMP, it performs column generation.
			   LOGWRAP_RMP_ID,     //19. a wrapper for all logit models, including LTAP, LPFE, LTAP_RQ and LPFE_RQ. it consist a layer performing iteration for cost function
			   TNM_ODE_ID,         //20. a base class for all O-D estimation models.
			   TNM_LODE_ID,        //21. a "run" class for logit O-D estimation.
			   TNM_LODE_TD_ID,    // 22. a "run" class for time-dependnet logit O-D estimation.
               TNM_STAP_ID,           //23. a base class for stochastic traffic assignmet.
			   CSTAP_B_ID,                //24. Bell's capacitated stochastic TAP
			   CSTAP_B_TD_ID,        //25. Time-depenent version of CSTAP_B.
			   TNM_DTA_ID, //26. dynamic traffic assignment
			   DTA_MSA_ID,     //27. MSA algorithm for DTA.
			   DTA_PRJ_ID,      //28. PRJ algorithm for DTA
			   DTA_HAD_ID,       //29, HAD algorithm for DTA
			   DTA_EPR_ID,        //30, EPR algorithm for DTA
			   DTA_HPR_ID,         //31, HPR algorithm for DTA
			   DTA_SWP_ID,         //32 SWAP ALGORITHM
			   DTA_PCH_ID,         // 33 PCH ALGORITHM
			   DTA_PCL_ID,         //34 PCL algorithm
			   DTA_SAS_ID,          //35 sas algorithm
			   DTA_ASD_ID,          //36 asd algorithm
			   TNM_SODE_ID,         //37 a base class for standard ODE
			   SODE_RMP_ID,         //38 RMP stanadard O-D estimation, solved by Steepest descent
			   SODE_BILEV_ID,        //39 Bilevel model for standard O-D estimation
			   SODE_RMP_CGP_ID,     //40 RMP, solved by CGP
			   SODE_RMP_PRJ_ID,       //41, RMP for standard O-DE, solved by Projection.
			   SODE_SILEV_ID,          //42 single level model for standard O-D
			   TNM_DODE_ID,            //43 dynamic O-D estimation
			   DODE_RMP_ID,            //44 restricted master problem for dode
			   DODE_RMP_EPA_ID,        //45, extraprojection method for solving RMP
			   DODE_RMP_PLS_ID,        //46, projection with line search
			   DODE_SILEV_ID,           //46 single level for dynamic O-D estimation
			   SODE_RMPB_ID,               //47. sode rmp for bilevel case
			   DTA_SSO_ID,           //48, dynamic network simplex method for system optimal DTA,  added by Wei 



};

typedef struct
{
	MATOBJID    objID; //object ID, in which Object the iteration is conducted.
	int         iter; //current iteration number
	double      ofv;  //current objective function value
	double      conv; //current convergence criterion
	double      step; //current step size
	float       time; //current cputime
}
ITERELEM;