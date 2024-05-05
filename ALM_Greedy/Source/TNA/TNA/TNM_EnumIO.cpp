/*======================================================================================
   TNM: Transportation network modelling toolkit

   file tnm_enumio.h: implementation file to define io functions for all enumeration types

   Yu Nie
   UC Davis
   latest update September, 2004
 ======================================================================================*/
#include "header\stdafx.h"
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
#include "header\tnm_enum.io"
#undef   LINK_TYPE_IN

  
}

/* string out */
ostringstream & operator<<(ostringstream & out, const TNM_LINKTYPE & lkType)
{

#define  LINK_TYPE_OUT
#include "header\tnm_enum.io"
#undef   LINK_TYPE_OUT
	
}

/* file in */
ifstream & operator>>(ifstream & in, TNM_LINKTYPE & lkType)
{
#define  LINK_TYPE_IN
#include "header\tnm_enum.io"
#undef   LINK_TYPE_IN
  
}

/* file out */
ofstream & operator<<(ofstream & out, const TNM_LINKTYPE & lkType)
{

#define  LINK_TYPE_OUT
#include "header\tnm_enum.io"
#undef   LINK_TYPE_OUT
	
}

/* cin */
istream & operator>>(istream & in, TNM_LINKTYPE & lkType)
{
#define  LINK_TYPE_IN
#include "header\tnm_enum.io"
#undef   LINK_TYPE_IN
  
}

/* cout */
ostream & operator<<(ostream & out, const TNM_LINKTYPE & lkType)
{
#define  LINK_TYPE_OUT
#include"header\tnm_enum.io"
#undef   LINK_TYPE_OUT

}


/*******************************************************************
        enum TNM_NODETYPE {FWJCT, SGNED, UNSED, DMOND, DMDND};
********************************************************************/

/* string in */
istringstream & operator>>(istringstream & in,        TNM_NODETYPE & ndType)
{
#define NODE_TYPE_IN	
#include "header\tnm_enum.io"
#undef NODE_TYPE_IN
}

/* string out */
ostringstream & operator<<(ostringstream & out, const TNM_NODETYPE & ndType)
{
#define NODE_TYPE_OUT	
#include "header\tnm_enum.io"
#undef NODE_TYPE_OUT
}

/* file in */
ifstream & operator>>(ifstream & in,        TNM_NODETYPE & ndType)
{
#define NODE_TYPE_IN	
#include "header\tnm_enum.io"
#undef NODE_TYPE_IN
}

/* file out */
ofstream & operator<<(ofstream & out, const TNM_NODETYPE & ndType)
{
#define NODE_TYPE_OUT	
#include "header\tnm_enum.io"
#undef NODE_TYPE_OUT
}

/* cin */
istream  & operator>>(istream & in,         TNM_NODETYPE & ndType)
{
#define NODE_TYPE_IN	
#include "header\tnm_enum.io"
#undef NODE_TYPE_IN
}
/* cout */
ostream  & operator<<(ostream & out, const TNM_NODETYPE & ndType)
{
#define NODE_TYPE_OUT	
#include "header\tnm_enum.io"
#undef  NODE_TYPE_OUT
}




///********************************************************************
//                enum RoutingType {FIXED, VARIABLE, MIXED};
//*********************************************************************/
//
//
///* string in */
//istringstream & operator>>(istringstream & in,        RoutingType & rtType)
//{
//#define ROUTING_TYPE_IN	
//#include "header\tnm_enum.io"
//#undef  ROUTING_TYPE_IN
//}
///* string out */
//ostringstream & operator<<(ostringstream & out, const RoutingType & rtType)
//{
//#define ROUTING_TYPE_OUT	
//#include "header\tnm_enum.io"
//#undef  ROUTING_TYPE_OUT
//}
///* file in */
//ifstream & operator>>(ifstream & in,        RoutingType & rtType)
//{
//#define ROUTING_TYPE_IN	
//#include "header\tnm_enum.io"
//#undef  ROUTING_TYPE_IN
//}
///* file out */
//ofstream & operator<<(ofstream & out, const RoutingType & rtType)
//{
//#define ROUTING_TYPE_OUT	
//#include "header\tnm_enum.io"
//#undef  ROUTING_TYPE_OUT
//}
///* cin */
//istream  & operator>>(istream & in,         RoutingType & rtType)
//{
//#define ROUTING_TYPE_IN	
//#include "header\tnm_enum.io"
//#undef  ROUTING_TYPE_IN
//}
///* cout */
//ostream  & operator<<(ostream & out, const RoutingType & rtType)
//{
//#define ROUTING_TYPE_OUT	
//#include "header\tnm_enum.io"
//#undef  ROUTING_TYPE_OUT
//}
//
//
//
///********************************************************************
//         enum ReactiveAssignType {DETERMINISTIC, STOCHASTIC};
//*********************************************************************/
//
//istringstream & operator>>(istringstream & in,        ReactiveAssignType & raType)
//{
//#define REACTIVE_ASSIGN_IN	
//#include "header\tnm_enum.io"
//#undef  REACTIVE_ASSIGN_IN
//}
//
///*******/
//ostringstream & operator<<(ostringstream & out, const ReactiveAssignType & raType)
//{
//
//#define REACTIVE_ASSIGN_OUT	
//#include "header\tnm_enum.io"
//#undef  REACTIVE_ASSIGN_OUT	
//}
//
///*******/
//ifstream & operator>>(ifstream & in,        ReactiveAssignType & raType)
//{
//#define REACTIVE_ASSIGN_IN	
//#include "header\tnm_enum.io"
//#undef  REACTIVE_ASSIGN_IN
//}
///*******/
//ofstream & operator<<(ofstream & out, const ReactiveAssignType & raType)
//{
//#define REACTIVE_ASSIGN_OUT	
//#include "header\tnm_enum.io"
//#undef  REACTIVE_ASSIGN_OUT	
//}
///*******/
//istream  & operator>>(istream & in,         ReactiveAssignType & raType)
//{
//#define REACTIVE_ASSIGN_IN	
//#include "header\tnm_enum.io"
//#undef  REACTIVE_ASSIGN_IN
//}
///*******/
//ostream  & operator<<(ostream & out, const ReactiveAssignType & raType)
//{
//#define REACTIVE_ASSIGN_OUT	
//#include "header\tnm_enum.io"
//#undef  REACTIVE_ASSIGN_OUT	
//}
//
//
///********************************************************************
//          enum INITASNTYPE {IAT_UNIFORM,  //uniform for each time interval
//	IAT_TRIAGNM,  //normal triangle
//	ITA_TROPZNM,  //normal tropzoid
//	ITA_CIRCLNM,  //normal circle;
//	ITA_TRIAGRV,  //reverse triangle
//	ITA_TROPZRV,  //reverse tropzoid
//	ITA_CIRCLRV  //reverse circle.};
//*********************************************************************/
//
//istringstream & operator>>(istringstream & in,        INITASNTYPE & iaType)
//{
//#define INIT_ASSIGN_IN	
//#include "header\tnm_enum.io"
//#undef  INIT_ASSIGN_IN
//}
//
///*******/
//ostringstream & operator<<(ostringstream & out, const INITASNTYPE & iaType)
//{
//
//#define INIT_ASSIGN_OUT	
//#include "header\tnm_enum.io"
//#undef  INIT_ASSIGN_OUT	
//}
//
///*******/
//ifstream & operator>>(ifstream & in,        INITASNTYPE & iaType)
//{
//#define INIT_ASSIGN_IN	
//#include "header\tnm_enum.io"
//#undef  INIT_ASSIGN_IN
//}
///*******/
//ofstream & operator<<(ofstream & out, const INITASNTYPE & iaType)
//{
//#define INIT_ASSIGN_OUT	
//#include "header\tnm_enum.io"
//#undef  INIT_ASSIGN_OUT	
//}
///*******/
//istream  & operator>>(istream & in,         INITASNTYPE & iaType)
//{
//#define INIT_ASSIGN_IN	
//#include "header\tnm_enum.io"
//#undef  INIT_ASSIGN_IN
//}
///*******/
//ostream  & operator<<(ostream & out, const INITASNTYPE & iaType)
//{
//#define INIT_ASSIGN_OUT	
//#include "header\tnm_enum.io"
//#undef  INIT_ASSIGN_OUT	
//}
//
//ostringstream & operator<<(ostringstream & out, const VEHTYPE & vType)
//{
//#define VEHTYPE_OUT
//#include "header\tnm_enum.io"
//#undef  VEHTYPE_OUT	
//}
//
//ofstream & operator<<(ofstream & out, const VEHTYPE & vType)
//{
//#define VEHTYPE_OUT	
//#include "header\tnm_enum.io"
//#undef  VEHTYPE_OUT	
//}
//
//ostream  & operator<<(ostream & out, const VEHTYPE & vType)
//{
//#define VEHTYPE_OUT	
//#include "header\tnm_enum.io"
//#undef  VEHTYPE_OUT	
//}
//
/////enumerate turn type
//ostringstream & operator<<(ostringstream & out, const TURNTYPE & tType)
//{
//#define TURNTYPE_OUT
//#include "header\tnm_enum.io"
//#undef  TURNTYPE_OUT	
//}
//
//ofstream & operator<<(ofstream & out, const TURNTYPE & tType)
//{
//#define TURNTYPE_OUT
//#include "header\tnm_enum.io"
//#undef  TURNTYPE_OUT	
//}
//
//ostream  & operator<<(ostream & out, const TURNTYPE & tType)
//{
//#define TURNTYPE_OUT
//#include "header\tnm_enum.io"
//#undef  TURNTYPE_OUT	
//}


///enumerate toll type

istringstream & operator>>(istringstream & in,        TNM_TOLLTYPE & ttType)
{
#define TNM_TOLLTYPE_IN	
#include "header\tnm_enum.io"
#undef  TNM_TOLLTYPE_IN
}

ostringstream & operator<<(ostringstream & out, const TNM_TOLLTYPE & ttType)
{

#define TNM_TOLLTYPE_OUT	
#include "header\tnm_enum.io"
#undef  TNM_TOLLTYPE_OUT	
}

/*******/
ifstream & operator>>(ifstream & in,        TNM_TOLLTYPE & ttType)
{
#define TNM_TOLLTYPE_IN	
#include "header\tnm_enum.io"
#undef  TNM_TOLLTYPE_IN
}
/*******/
ofstream & operator<<(ofstream & out, const TNM_TOLLTYPE & ttType)
{
#define TNM_TOLLTYPE_OUT	
#include "header\tnm_enum.io"
#undef  TNM_TOLLTYPE_OUT	
}
/*******/
istream  & operator>>(istream & in,         TNM_TOLLTYPE & ttType)
{
#define TNM_TOLLTYPE_IN	
#include "header\tnm_enum.io"
#undef  TNM_TOLLTYPE_IN
}



ostream & operator<<(ostream & out, const TNM_TOLLTYPE & ttType)
{
#define TNM_TOLLTYPE_OUT
#include "header\tnm_enum.io"
#undef  TNM_TOLLTYPE_OUT	
}
