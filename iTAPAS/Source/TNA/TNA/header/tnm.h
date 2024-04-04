#ifndef TNM_TOOLKIT_H
#define TNM_TOOLKIT_H
//#ifdef _MANAGED
//#pragma managed(push, off)
//#endif
//#pragma unmanaged
#ifndef TNMDLL_H
	#define TNMDLL_H

	#ifndef TNM_EXT_CLASS 
		#ifdef _TNM_DLL
			#define TNM_EXT_CLASS  _declspec(dllexport)
		#else
			#define TNM_EXT_CLASS  _declspec(dllimport)
		#endif
	#endif
	#if defined(_M_IX86)
		#ifndef _SHUTTNMMATLAB
			#define _ENABLE_MATLAB_ENGINE
		#endif
	#endif
	   
	#ifndef _TNM_DLL
	#ifndef _TNM_DLL_LOADED
	#define _TNM_DLL_LOADED
		#if defined(_M_IX86)
			#ifdef _DEBUG
				#pragma message("     _Adding library: tnad.lib: debug, Win32" ) 
				#pragma comment(lib, "tnad.lib")
			#else
				#pragma message("     _Adding library: tna_win32.lib: release, Win32" ) 
				#pragma comment(lib, "tna_win32.lib")
			#endif
		#else
			#ifdef _DEBUG
				#pragma message("     _Adding library: tna.lib: debug, Win64" ) 
				#pragma comment(lib, "tnad64.lib")
			#else
				#pragma message("     _Adding library: tna64.lib: release, Win64" ) 
				#pragma comment(lib, "tna_x64.lib")
			#endif
		#endif
	#endif
	#endif
#endif
	
//#define _HAS_ITERATOR_DEBUGGING 0
//#define _SECURE_SCL 0
//#ifndef TNM_SHUTMYSQL
//#include <mysql++.h> //note thtat the folders for mysql++ has been added as additional include directories.
//#include <ssqls.h>//you need to include it if you want to use custom container.
//#endif
#include <newran.h> //include random lib header
//#include "..\..\..\myutil\shapelib\shapefil.h"
//the following are tnm's headers.
//#include "dtnm_header.h"

//#include "TNM_CNODE.h"
#ifndef TNM_SHUTMYSQL
//#include "TNM_DBOP.h"
#endif
//#include "TNM_DLINK.h"
//#include "TNM_DNET.h"
//#include "TNM_DNODE.h"
//#include "TNM_DODPAIR.h"
#include "tnm_enum.io"
#include "TNM_EnumIO.h"
#include "TNM_Net.h"
#include "TNM_Header.h"
#include "mat_header.h"
#include "TNM_Algorithm.h"
//#include "TNM_INCIDENT.h"
//#include "TNM_MNET.h"
//#include "TNM_MODPAIR.h"
//#include "TNM_PHASE.h"
//#include "tnm_dma.h"
//#include "tnm_pnode.h"
//#include "TNM_ProbNet.h"
//#include "TNM_ScanList.h"
//#include "TNM_Slink.h"
//#include "TNM_SNet.h"
//#include "TNM_SNode.h"
//#include "TNM_SODpair.h"
//#include "TNM_SPATH.H"
//#include "TNM_utility_old.h"
#include "TNM_utility.h"
//#include "TNM_VEHICLE.h"
//#include "TNM_GENE.h"
//#include "tnm_ctlLSC.h"
#include "My_Predicate.h"

//#ifdef _MANAGED
//#pragma managed(pop)
//#endif
#endif