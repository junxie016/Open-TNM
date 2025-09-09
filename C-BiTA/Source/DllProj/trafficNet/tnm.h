#ifndef TNM_TOOLKIT_H
#define TNM_TOOLKIT_H
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

#endif
//#ifndef TNM_SHUTMYSQL
//#include <mysql++.h> //note thtat the folders for mysql++ has been added as additional include directories.
//#include <ssqls.h>//you need to include it if you want to use custom container.
//#endif
#include <newran.h> //include random lib header
#ifndef TNM_SHUTMYSQL
#endif

#include "tnm_enum.io"
#include "TNM_EnumIO.h"
#include "TNM_Net.h"
#include "TNM_Header.h"
#include "mat_header.h"
#include "TNM_Algorithm.h"
#include "TNM_utility.h"
#include "My_Predicate.h"

#endif