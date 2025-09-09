/*======================================================================================
   TNM: Transportation network modelling toolkit

   file tnm_enumio.h: implementation file to define io functions for all enumeration types

   Yu Nie
   UC Davis
   latest update September, 2004
 ======================================================================================*/
#include "stdafx.h"
//#include <dtnm_Header.h>
//#include "header\TNM_EnumIO.h"

using std::endl;
using std::cerr;
using std::cout;
/********************************************************************
      enum TNM_LINKTYPE {LWRLK, VQLK, HQLK, DMOLK, DMDLK} 
********************************************************************/

/* string in */
istringstream & operator>>(istringstream & in, TNM_LINKTYPE & lkType)
{

#define  LINK_TYPE_IN
#include "tnm_enum.io"
#undef   LINK_TYPE_IN

  
}

/* string out */
ostringstream & operator<<(ostringstream & out, const TNM_LINKTYPE & lkType)
{

#define  LINK_TYPE_OUT
#include "tnm_enum.io"
#undef   LINK_TYPE_OUT
	
}

/* file in */
ifstream & operator>>(ifstream & in, TNM_LINKTYPE & lkType)
{
#define  LINK_TYPE_IN
#include "tnm_enum.io"
#undef   LINK_TYPE_IN
  
}

/* file out */
ofstream & operator<<(ofstream & out, const TNM_LINKTYPE & lkType)
{

#define  LINK_TYPE_OUT
#include "tnm_enum.io"
#undef   LINK_TYPE_OUT
	
}

/* cin */
istream & operator>>(istream & in, TNM_LINKTYPE & lkType)
{
#define  LINK_TYPE_IN
#include "tnm_enum.io"
#undef   LINK_TYPE_IN
  
}

/* cout */
ostream & operator<<(ostream & out, const TNM_LINKTYPE & lkType)
{
#define  LINK_TYPE_OUT
#include"tnm_enum.io"
#undef   LINK_TYPE_OUT

}


/*******************************************************************
        enum TNM_NODETYPE {FWJCT, SGNED, UNSED, DMOND, DMDND};
********************************************************************/

/* string in */
istringstream & operator>>(istringstream & in,        TNM_NODETYPE & ndType)
{
#define NODE_TYPE_IN	
#include "tnm_enum.io"
#undef NODE_TYPE_IN
}

/* string out */
ostringstream & operator<<(ostringstream & out, const TNM_NODETYPE & ndType)
{
#define NODE_TYPE_OUT	
#include "tnm_enum.io"
#undef NODE_TYPE_OUT
}

/* file in */
ifstream & operator>>(ifstream & in,        TNM_NODETYPE & ndType)
{
#define NODE_TYPE_IN	
#include "tnm_enum.io"
#undef NODE_TYPE_IN
}

/* file out */
ofstream & operator<<(ofstream & out, const TNM_NODETYPE & ndType)
{
#define NODE_TYPE_OUT	
#include "tnm_enum.io"
#undef NODE_TYPE_OUT
}

/* cin */
istream  & operator>>(istream & in,         TNM_NODETYPE & ndType)
{
#define NODE_TYPE_IN	
#include "tnm_enum.io"
#undef NODE_TYPE_IN
}
/* cout */
ostream  & operator<<(ostream & out, const TNM_NODETYPE & ndType)
{
#define NODE_TYPE_OUT	
#include "tnm_enum.io"
#undef  NODE_TYPE_OUT
}

istringstream & operator>>(istringstream & in,        TNM_TOLLTYPE & ttType)
{
#define TNM_TOLLTYPE_IN	
#include "tnm_enum.io"
#undef  TNM_TOLLTYPE_IN
}

ostringstream & operator<<(ostringstream & out, const TNM_TOLLTYPE & ttType)
{

#define TNM_TOLLTYPE_OUT	
#include "tnm_enum.io"
#undef  TNM_TOLLTYPE_OUT	
}

/*******/
ifstream & operator>>(ifstream & in,        TNM_TOLLTYPE & ttType)
{
#define TNM_TOLLTYPE_IN	
#include "tnm_enum.io"
#undef  TNM_TOLLTYPE_IN
}
/*******/
ofstream & operator<<(ofstream & out, const TNM_TOLLTYPE & ttType)
{
#define TNM_TOLLTYPE_OUT	
#include "tnm_enum.io"
#undef  TNM_TOLLTYPE_OUT	
}
/*******/
istream  & operator>>(istream & in,         TNM_TOLLTYPE & ttType)
{
#define TNM_TOLLTYPE_IN	
#include "tnm_enum.io"
#undef  TNM_TOLLTYPE_IN
}



ostream & operator<<(ostream & out, const TNM_TOLLTYPE & ttType)
{
#define TNM_TOLLTYPE_OUT
#include "tnm_enum.io"
#undef  TNM_TOLLTYPE_OUT	
}
