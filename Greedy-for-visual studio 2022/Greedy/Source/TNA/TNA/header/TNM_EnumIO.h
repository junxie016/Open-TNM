/*======================================================================================
   TNM: Transportation network modelling toolkit

   file tnm_enumio.h: interface file to define io functions for all enumeration types

   Yu Nie
   UC Davis
   latest update September, 2004
 ======================================================================================*/

#ifndef _ENUMERATE_TYPE_IO_
#define _ENUMERATE_TYPE_IO_
#include "tnm_header.h"
//#include "dtnm_Header.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using std::string;
using std::istringstream;
using std::ostringstream;
using std::istream;
using std::ostream;
using std::ifstream;
using std::ofstream;
using std::vector;

struct TNM_EXT_CLASS NODE_VALCONTAINER
{
	TNM_NODETYPE       type;
	int                id;         //node id
	bool               dummy;
	largeInt           xCord;      //the x coordinate in foot
	largeInt           yCord;      //the y coordinate in foot

	//to be extended.
}; //We encompass all possible values of nodes in this structure.
        //any derived node class will intialize by inputing an instance of this structure.
class TNM_SNODE;
struct TNM_EXT_CLASS LINK_VALCONTAINER
{
	TNM_LINKTYPE        type;
	int					id;        /*link id*/
	bool				dummy;
	TNM_SNODE			*head;     /*starting node of the link*/
	TNM_SNODE			*tail;     /*ending node of the link */
	floatType			capacity;  /*link capacity*/
	floatType			length;    /*link length*/
	floatType			ffs;       /*free flow speed*/
	vector<floatType>	par;      //for potential parameters used in speed-flow function
	//the following for dynamics only
	floatType			laneHldCap; // holding capacity per lane: veh/mile/lane
	floatType			laneFlwCap; // flow capacity per lane: veh/hr/lane
	tinyInt				numOfLanes; // the number of lanes 
	smallInt			unitTime;   // this is not a member of any link, but will be needed when initializing 
	                             // some type of links.
	//the following for PLINK:
	string              roadname;
	string              detecttype;
};

struct TNM_EXT_CLASS DEST_VALCONTAINER
{
	TNM_SNODE  *node;
	floatType  demand;
	int        nt; //number of time intervals
} ;
/********************************************************************
      enum TNM_LINKTYPE 
********************************************************************/

TNM_EXT_CLASS istringstream & operator>>(istringstream & in,        TNM_LINKTYPE & lkType);
TNM_EXT_CLASS ostringstream & operator<<(ostringstream & out, const TNM_LINKTYPE & lkType);

TNM_EXT_CLASS ifstream & operator>>(ifstream & in,        TNM_LINKTYPE & lkType);
TNM_EXT_CLASS ofstream & operator<<(ofstream & out, const TNM_LINKTYPE & lkType);

TNM_EXT_CLASS istream  & operator>>(istream & in,         TNM_LINKTYPE & lkType);
TNM_EXT_CLASS ostream  & operator<<(ostream & out, const TNM_LINKTYPE & lkType);



/********************************************************************
        enum TNM_NODETYPE 
********************************************************************/

TNM_EXT_CLASS istringstream & operator>>(istringstream & in,        TNM_NODETYPE & ndType);
TNM_EXT_CLASS ostringstream & operator<<(ostringstream & out, const TNM_NODETYPE & ndType);

TNM_EXT_CLASS ifstream & operator>>(ifstream & in,        TNM_NODETYPE & ndType);
TNM_EXT_CLASS ofstream & operator<<(ofstream & out, const TNM_NODETYPE & ndType);

TNM_EXT_CLASS istream  & operator>>(istream & in,         TNM_NODETYPE & ndType);
TNM_EXT_CLASS ostream  & operator<<(ostream & out, const TNM_NODETYPE & ndType);



///********************************************************************
//          enum RoutingType {FIXED, VARIABLE, MIXED};
//*********************************************************************/
//
//TNM_EXT_CLASS istringstream & operator>>(istringstream & in,        RoutingType & rtType);
//TNM_EXT_CLASS ostringstream & operator<<(ostringstream & out, const RoutingType & rtType);
//
//TNM_EXT_CLASS ifstream & operator>>(ifstream & in,        RoutingType & rtType);
//TNM_EXT_CLASS ofstream & operator<<(ofstream & out, const RoutingType & rtType);
//
//TNM_EXT_CLASS istream  & operator>>(istream & in,         RoutingType & rtType);
//TNM_EXT_CLASS ostream  & operator<<(ostream & out, const RoutingType & rtType);
//
//
///********************************************************************
//         enum ReactiveAssignType {DETERMINISTIC, STOCHASTIC};
//*********************************************************************/
//
//TNM_EXT_CLASS istringstream & operator>>(istringstream & in,        ReactiveAssignType & raType);
//TNM_EXT_CLASS ostringstream & operator<<(ostringstream & out, const ReactiveAssignType & raType);
//
//TNM_EXT_CLASS ifstream & operator>>(ifstream & in,        ReactiveAssignType & raType);
//TNM_EXT_CLASS ofstream & operator<<(ofstream & out, const ReactiveAssignType & raType);
//
//TNM_EXT_CLASS istream  & operator>>(istream & in,         ReactiveAssignType & raType);
//TNM_EXT_CLASS ostream  & operator<<(ostream & out, const ReactiveAssignType & raType);
//
///********************************************************************
//         enum INITASNTYPE {IAT_UNIFORM,  //uniform for each time interval
//	IAT_TRIAGNM,  //normal triangle
//	ITA_TROPZNM,  //normal tropzoid
//	ITA_CIRCLNM,  //normal circle;
//	ITA_TRIAGRV,  //reverse triangle
//	ITA_TROPZRV,  //reverse tropzoid
//	ITA_CIRCLRV  //reverse circle.};
//*********************************************************************/
//
//TNM_EXT_CLASS istringstream & operator>>(istringstream & in,        INITASNTYPE & iaType);
//TNM_EXT_CLASS ostringstream & operator<<(ostringstream & out, const INITASNTYPE & iaType);
//
//TNM_EXT_CLASS ifstream & operator>>(ifstream & in,        INITASNTYPE & iaType);
//TNM_EXT_CLASS ofstream & operator<<(ofstream & out, const INITASNTYPE & iaType);
//
//TNM_EXT_CLASS istream  & operator>>(istream & in,         INITASNTYPE & iaType);
//TNM_EXT_CLASS ostream  & operator<<(ostream & out, const INITASNTYPE & iaType);
//
//
////// enumerate vehicle type
//TNM_EXT_CLASS ostringstream & operator<<(ostringstream & out, const VEHTYPE & vType);
//
//TNM_EXT_CLASS ofstream & operator<<(ofstream & out, const VEHTYPE & vType);
//
//TNM_EXT_CLASS ostream  & operator<<(ostream & out, const VEHTYPE & vType);
//
/////enumerate turn type
//TNM_EXT_CLASS ostringstream & operator<<(ostringstream & out, const TURNTYPE & tType);
//
//TNM_EXT_CLASS ofstream & operator<<(ofstream & out, const TURNTYPE & tType);
//
//TNM_EXT_CLASS ostream  & operator<<(ostream & out, const TURNTYPE & tType);



///enumerate turn type
TNM_EXT_CLASS istringstream & operator>>(istringstream & in,        TNM_TOLLTYPE & raType);
TNM_EXT_CLASS ostringstream & operator<<(ostringstream & out, const TNM_TOLLTYPE & raType);

TNM_EXT_CLASS ifstream & operator>>(ifstream & in,        TNM_TOLLTYPE & raType);
TNM_EXT_CLASS ofstream & operator<<(ofstream & out, const TNM_TOLLTYPE & raType);

TNM_EXT_CLASS istream  & operator>>(istream & in,         TNM_TOLLTYPE & raType);
TNM_EXT_CLASS ostream  & operator<<(ostream & out, const TNM_TOLLTYPE & raType);


#endif