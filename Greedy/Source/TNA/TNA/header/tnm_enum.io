
/*****************************************************************
                      LINK_TYPE_IN
******************************************************************/
#ifdef LINK_TYPE_IN  
  string typeStr;
  in>>typeStr; 
  //cout<<typeStr<<endl;
  strupr((char *) typeStr.c_str()); // conver the string to upper case
  if     (typeStr=="BASLK")  lkType = BASLK;
  else if(typeStr=="BPRLK")  lkType = BPRLK;
  else if(typeStr=="ACHLK")  lkType = ACHLK;
  else if(typeStr=="CSTLK")  lkType = CSTLK;
  else if(typeStr=="LINLK")  lkType = LINLK;
  else if(typeStr=="EXPLK")  lkType = EXPLK;
  else if(typeStr=="LWRLK")  lkType = LWRLK;
  else if(typeStr=="SQULK" ) lkType = SQULK;
  else if(typeStr=="LICLK")  lkType = LICLK;
  else if(typeStr=="PQULK")  lkType = PQULK;
  else if(typeStr=="LWRNF")  lkType = LWRNF;
  else if(typeStr=="DMOLK")  lkType = DMOLK;
  else if(typeStr=="DMDLK")  lkType = DMDLK;
  else if(typeStr=="DFMLK")  lkType = DFMLK;
  else if(typeStr=="MCALK")  lkType = MCALK;
  else if(typeStr=="PRBLK")  lkType = PRBLK;
  else if(typeStr=="GCMLK")  lkType = GCMLK;
  else                        { cout<<"\t"<<typeStr<<" is an unrecognized link type!"<<endl; lkType = BPRLK; }
  return in;
#endif

/*****************************************************************
                      LINK_TYPE_OUT
******************************************************************/
#ifdef LINK_TYPE_OUT
    	switch(lkType)
	{
	 case BASLK:
		  out<<"BASLK";
		  break;
	case BPRLK:
		  out<<"BPRLK";
		  break;
		  case CPBPR:
		  out<<"CPBPR";
		  break;
	case ACHLK:
		  out<<"ACHLK";
		  break;
	case CSTLK:
		  out<<"CSTLK";
		  break;
	case LINLK:
		  out<<"LINLK";
		  break;
	case EXPLK:
	      out<<"EXPLK";
	      break;
	 case LWRLK:
		  out<<"LWRLK";
		  break;
	case  LWRNF:
	      out<<"LWRNF";
	      break;
	 case SQULK:
		  out<<"SQULK";
		  break;
	 case LICLK:
	      out<<"LICLK";
	      break;
	 case PQULK:
		  out<<"PQULK";
		  break;		  
	 case DMOLK:
		  out<<"DMOLK";
		  break;
	 case DMDLK:
		  out<<"DMDLK";
		  break;
	 case DFMLK:
		  out<<"DFMLK";
		  break;
	 case MCALK:
		  out<<"MCALK";
		  break;	
	case PRBLK:	  
	      out<<"PRBLK";
	      break;
	case GCMLK:	  
	      out<<"GCMLK";
	      break;
	 default:
		  cerr<<"Bad link type in memory!"<<endl;
		  exit(1);
	}
    return out;
#endif


/*****************************************************************
                      NODE_TYPE_IN
******************************************************************/
#ifdef NODE_TYPE_IN
  string typeStr;
  in>>typeStr; 

  strupr((char *) typeStr.c_str()); // conver the string to upper case

  if     (typeStr=="BASND")   ndType = BASND;
  else if(typeStr=="FWJCT")   ndType = FWJCT;
  else if (typeStr=="FWJFI")  ndType = FWJFI;
  else if(typeStr=="CTLSN" )  ndType = CTLSN;
  else if(typeStr=="CTLPT" )  ndType = CTLPT;
  else if(typeStr=="CTLAD" )  ndType = CTLAD;
  else if(typeStr=="CTLRM" )  ndType = CTLRM;
  else if(typeStr=="CTLGC" )  ndType = CTLGC;
  else if(typeStr=="CTLGN")   ndType = CTLGN;
  else if(typeStr=="DMOND")   ndType = DMOND;
  else if(typeStr=="DMDND")   ndType = DMDND;
  else if(typeStr=="MCAND")   ndType = MCAND;
  else if(typeStr=="MSTOP")   ndType = MSTOP;
  else if(typeStr=="PASND")   ndType = PASND;
  else if(typeStr=="PASND_DMA") ndType = PASND_DMA;
  else                       { cout<<"\t"<<typeStr<<" is an unrecognized node type!"<<endl; exit(0);}
  return in;
#endif


/*****************************************************************
                      NODE_TYPE_OUT
******************************************************************/
#ifdef NODE_TYPE_OUT
switch(ndType)
	{
	 case BASND:
		  out<<"BASND";
		  break;
	 case FWJCT:
		  out<<"FWJCT";
		  break;

	 case FWJFI:
		  out<<"FWJFI";
		  break;

	 case CTLSN:
		  out<<"CTLSN";
		  break;

	 case CTLPT:
	      out<<"CTLPT";
		  break;

	 case CTLAD:
	      out<<"CTLAD";
		  break;

	 case CTLRM:
	      out<<"CTLRM";
		  break;

	 case CTLGC:
	      out<<"CTLGC";
		  break;
	 case CTLGN:
	      out<<"CTLGN";
		  break;
	 case DMOND:
		  out<<"DMOND";
		  break;

	 case DMDND:
		  out<<"DMDND";
		  break;
	 case MCAND:
		  out<<"MCAND";
		  break;
	 case MSTOP:
		  out<<"MSTOP";
		  break;
	 case PASND:
		  out<<"PASND";
		  break;
	 case PASND_DMA:
		  out<<"PASND_DMA";
		  break;
	 default:
		  cerr<<"Bad node type in memory!"<<endl;
		  exit(1);
	}
    return out;
#endif

/*****************************************************************
                      ROUTING_TYPE_IN
******************************************************************/
#ifdef ROUTING_TYPE_IN
  string typeStr;
  in>>typeStr; 

  strupr((char *) typeStr.c_str()); // conver the string to upper case

  if     (typeStr=="PREDICTIVE")      rtType = PREDICTIVE;
  else if(typeStr=="REACTIVE" )  rtType = REACTIVE;
  else if(typeStr=="MIXED")      rtType = MIXED;
  else if(typeStr=="MYOPIC")     rtType = MYOPIC;
  else                      rtType = PREDICTIVE;  // default link type
  return in;
#endif

/*****************************************************************
                      ROUTING_TYPE_OUT
******************************************************************/
#ifdef ROUTING_TYPE_OUT
switch(rtType)
	{
	 case PREDICTIVE:
		  out<<"PREDICTIVE";
		  break;
	 case REACTIVE:
		  out<<"REACTIVE";
		  break;
     case MIXED:
	      out<<"MIXED";
		  break;
	 case MYOPIC:
	      out<<"MYOPIC";
	      break;
	 default:
		  cerr<<"Bad routing type in memory!"<<endl;
		  exit(1);
	}
    return out;

#endif

/*****************************************************************
                      REACTIVE_ASSIGN_IN
******************************************************************/

#ifdef REACTIVE_ASSIGN_IN
string typeStr;
  in>>typeStr; 

  strupr((char *) typeStr.c_str()); // conver the string to upper case

  if     (typeStr=="DETERMINISTIC")      raType = DETERMINISTIC;
  else if(typeStr=="STOCHASTIC" )        raType = STOCHASTIC;
  else                                   raType = DETERMINISTIC;  // default link type
  return in;

#endif

/*****************************************************************
                      REACTIVE_ASSIGN_OUT
******************************************************************/
#ifdef REACTIVE_ASSIGN_OUT
switch(raType)
	{
	 case DETERMINISTIC:
		  out<<"DETERMINISTIC";
		  break;

	 case STOCHASTIC:
		  out<<"STOCHASTIC";
		  break;
    
	 default:
		  cerr<<"Bad routing type in memory!"<<endl;
		  exit(1);
	}
    return out;


#endif


/*****************************************************************
						INIT_ASSIGN_IN
******************************************************************/

#ifdef INIT_ASSIGN_IN
string typeStr;
  in>>typeStr; 

  strupr((char *) typeStr.c_str()); // conver the string to upper case

  if     (typeStr=="IAT_UNIFORM")         iaType = IAT_UNIFORM;
  else if(typeStr=="IAT_TRIAGNM" )        iaType = IAT_TRIAGNM;
  else if(typeStr=="IAT_TROPZNM" )        iaType = IAT_TROPZNM;
  else if(typeStr=="IAT_CIRCLNM" )        iaType = IAT_CIRCLNM;
  else if(typeStr=="IAT_TRIAGRV" )        iaType = IAT_TRIAGRV;
  else if(typeStr=="IAT_TROPZRV" )        iaType = IAT_TROPZRV;
  else if(typeStr=="IAT_CIRCLRV" )        iaType = IAT_CIRCLRV;
  else                                    iaType = IAT_UNIFORM;  // default link type
  return in;

#endif

/*****************************************************************
                      INIT_ASSIGN_OUT
******************************************************************/
#ifdef INIT_ASSIGN_OUT
switch(iaType)
	{
	 case IAT_UNIFORM:
		  out<<"IAT_UNIFORM";
		  break;
	 case IAT_TRIAGNM:
		  out<<"IAT_TRIAGNM";
		  break;
	 case IAT_TROPZNM:
		  out<<"IAT_TROPZNM";
		  break;
	 case IAT_CIRCLNM:
		  out<<"IAT_CIRCLNM";
		  break;
	 case IAT_TRIAGRV:
		  out<<"IAT_TRIAGRV";
		  break;
	 case IAT_TROPZRV:
		  out<<"IAT_TROPZRV";
		  break;
	 case IAT_CIRCLRV:
		  out<<"IAT_CIRCLRV";
		  break;
    
	 default:
		  cerr<<"Bad initial assignment type in memory!"<<endl;
		  exit(1);
	}
    return out;
#endif

#ifdef VEHTYPE_OUT
switch(vType)
{
	case VT_VQ:
	out<<"Vehicular quantum";
	break;
	case VT_CAR:
	out<<"Passenger car";
	break;
	case VT_VAN:
	out<<"Van";
	break;
	case VT_TRUCK:
	out<<"Truck";
	break;
	default:
	cerr<<"Bad vehicle type in memory!"<<endl;
	exit(1);
}
return out;
#endif

#ifdef TURNTYPE_OUT
switch(tType)
{
	case TT_Left:
	out<<"Left";
	break;
	case TT_Right:
	out<<"Right";
	break;
	case TT_Through:
	out<<"Through";
	break;
	case TT_Uturn:
	out<<"U turn";
	break;
	default:
	cerr<<"Bad turn type in memory!"<<endl;
	exit(1);
}
return out;
#endif


/*****************************************************************
                      TOLL_TYPE_IN
******************************************************************/
#ifdef TNM_TOLLTYPE_IN
  string typeStr;
  in>>typeStr; 

  strupr((char *) typeStr.c_str()); // conver the string to upper case

  if     (typeStr=="NOTOLL")      ttType = TT_NOTOLL;
  else if(typeStr=="MTTOLL" )  ttType = TT_MTTOLL;
  else if(typeStr=="MCTOLL")   ttType = TT_MCTOLL;
  else if(typeStr=="FXTOLL")      ttType = TT_FXTOLL;
  else if(typeStr=="MXTOLL")    ttType = TT_MXTOLL;
  else                     
  {
        cout<<"\t"<<typeStr<<" is an unrecongized toll type. Change it to NOTOLL"<<endl;
		ttType = TT_NOTOLL;  // default link type
   }
  return in;
#endif

/*****************************************************************
                      toll_TYPE_OUT
******************************************************************/
#ifdef TNM_TOLLTYPE_OUT
switch(ttType)
	{
	 case TT_NOTOLL:
		  out<<"NOTOLL";
		  break;
	 case TT_MTTOLL:
		  out<<"MTTOLL";
		  break;
	case TT_MCTOLL:
		 out<<"MCTOLL";
		 break;
	case TT_FXTOLL:
	      out<<"FXTOLL";      
		  break;
	case TT_MXTOLL:
	      out<<"MXTOLL";
	      break;	 
	 default:
		  cerr<<"Bad toll type in memory!"<<endl;
		  exit(1);
	}
    return out;

#endif