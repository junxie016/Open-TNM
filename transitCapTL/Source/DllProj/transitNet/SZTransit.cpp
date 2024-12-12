#include "PTNet.h"

void PTNET::SZ_OutputOnOffLinkFlow() //对于某虚拟站点节点，所对应线路的上车弧流量(onflow)和下车弧流量(offflow)
{
	string fileName  = networkName +"OnOffLinkFlow.csv";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .linkinfo file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting PT link information into "<<fileName<<" linkinfo!"<<endl;

	PTLink *link;
	outfile<<"stop_id,route_id,on,off"<<endl;

	for(PTStopMapIter pv = m_stops.begin(); pv!= m_stops.end(); pv++) 
	{
		PTStop* stop=pv->second;
		PTNode* node = stop->GetTransferNode();
		
		for (int i=0;i<stop->mshapes.size();i++)
		{
			PTShape* shp = stop->mshapes[i];
			int on=0,off=0;
			for (int j = 0; j< node->forwStar.size();j++)
			{
				link = node->forwStar[j];
				if (link->GetTransitLinkType() == PTLink::ABOARD && link->head->m_shape == shp)
				{
					on = link->volume;
					break;
				}
			}
			for (int j = 0; j< node->backStar.size();j++)
			{
				link = node->backStar[j];
				if (link->GetTransitLinkType() == PTLink::ALIGHT && link->tail->m_shape == shp)
				{
					off = link->volume;
					break;
				}
			}

			outfile<<stop->m_id<<","<<shp->m_id<<","<<on<<","<<off<<endl;
		
		}

	}
	outfile.close();

}

void PTNET::SZ_OutputTransitLinkFlow() //输出各条行驶弧(i,j)所在的线路id，尾节点i名，头节点j名，弧流量，弧费用(in_vehicle_time)
{
	string fileName  = networkName +"TransitLinkFlow.csv";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .linkinfo file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting PT link information into "<<fileName<<" linkinfo!"<<endl;

	
	outfile<<"route_id,from_stop,to_stop,transit_flow,in_vehicle_time"<<endl;


	for(int i=0;i<numOfLink;i++)
	{
		PTLink *link = linkVector[i];
		if (link->GetTransitLinkType() == PTLink::ENROUTE)
		{
			outfile<<link->tail->m_shape->m_id<<","<<link->tail->m_stop->m_id<<","<<link->head->m_stop->m_id<<","<<link->volume<<","<<link->cost<<endl;
		
		}
	
	}

	outfile.close();
}


void PTNET::SZ_OutputWALKLinkFlow()
{
	string fileName  = networkName +"WalkLinkFlow.csv";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .linkinfo file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting PT link information into "<<fileName<<" linkinfo!"<<endl;

	
	outfile<<"link_id,from_stop,to_stop,walk_flow"<<endl;


	for(int i=0;i<numOfLink;i++)
	{
		PTLink *link = linkVector[i];
		if (link->GetTransitLinkType() == PTLink::WALK)
		{
			PTNode* tail = link->tail;
			PTNode* head = link->head;
			if(tail->GetTransitNodeType() == PTNode::TRANSFER && head->GetTransitNodeType() == PTNode::TRANSFER)
			{
				outfile<<link->id<<","<<tail->m_stop->m_id<<","<<head->m_stop->m_id<<","<<link->volume<<endl;
			}
			else if(tail->GetTransitNodeType() == PTNode::TRANSFER)
			{
				outfile<<link->id<<","<<tail->m_stop->m_id<<","<<head->id<<","<<link->volume<<endl;
			}
			else if(head->GetTransitNodeType() == PTNode::TRANSFER)
			{
				outfile<<link->id<<","<<tail->id<<","<<head->m_stop->m_id<<","<<link->volume<<endl;
			}
			else
			{
				outfile<<link->id<<","<<tail->id<<","<<head->id<<","<<link->volume<<endl;
			}
		}
	}

	outfile.close();
}

void PTNET::SZ_OutputHyperpathFlow() 
	//对于每对OD：输出起点id，终点id，OD需求:超路径数量，
	//遍历该OD对间每条超路径，输出其各段弧id(type)，终点id:路径流量，路径总成本，路径总等待时间
{
	string fileName  = networkName +"HyperpathFlow.csv";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .linkinfo file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting hyperpath information into "<<fileName<<" !"<<endl;

	outfile<<"ORG,DEST,DEMAND:PATH_SIZE"<<endl; //文件第一行，用","隔开
	int numberhp=0; //计算总超路径数量
	int numofboardings=0;
	int numofboardinglines = 0;
	for (int i = 0;i<numOfPTDest;i++)
	{
		PTDestination* dest = PTDestVector[i];
		double sumRGap = 0.0;

		for (int j=0; j<dest->numOfOrg; j++) //遍历各OD对
		{
			PTOrg* org= dest->orgVector[j];
	
			outfile<<org->org->m_stop->m_id<<","<<dest->destination->m_stop->m_id<<","<< org->assDemand <<":"<<org->pathSet.size()<<endl; //输出起点id, 终点id, OD需求:超路径数量
			//outfile<<org->org->trafficnetid<<","<<dest->destination->trafficnetid<<","<< org->assDemand <<":"<<org->pathSet.size()<<endl;
			//outfile<<"OD:"<<TNM_IntFormat(org->org->id)<<","<<TNM_IntFormat(dest->destination->id)<<", path size"<<TNM_IntFormat(org->pathSet.size())<<endl;
			numberhp += org->pathSet.size(); 
			for (int k=0; k<org->pathSet.size();++k) //遍历该OD对间的各超路径，输出每条超路径经过的各段弧的linkid(type)，终点节点id:路径流量，路径总成本，路径总等待时间
			{
				TNM_HyperPath* path = org->pathSet[k];
				vector<GLINK*> glinks=path->GetGlinks();

				for(int ix=0;ix<glinks.size();++ix)
				{
					PTLink* plink=glinks[ix]->m_linkPtr;
					if (ix<glinks.size()-1) outfile<<plink->id<<"("<<plink->GetTransitLinkTypeName()<<")"<<","; //输出该超路径各段弧的id和type
					else outfile<<plink->id;
					//outfile<<TNM_IntFormat(plink->id)<<",";
				}
				outfile<<":	path flow:"<<path->flow<<",path ttcost:"<<path->cost<<",path waitcost:"<<path->WaitCost<<endl;
				//outfile<<TNM_FloatFormat(path->flow,12,6)<<TNM_FloatFormat(path->cost,12,6)<<endl;
			}
			outfile<<endl;
		}
	}
	outfile.close();

}


int PTNET::SZ_BuildAN()
{
	if(SZ_ReadStop()!=0)
    {
        cout<<"\tFailed to read stops"<<endl;
        return 1;
    }


	if(SZ_ReadRoute()!=0)
	{
		cout<<"\tFailed to read routes"<<endl;
        return 2;
	}

	if(SZ_ReadShape()!=0)
    {
        cout<<"\tFailed to read shapes"<<endl;
        return 3;
    }

	if(CreateTEAPNodeLinks()!=0)
	{
		cout<<"\tFailed to create transit nodes and links"<<endl;
        return 4;	
	}

	if(SZ_ReadTEAPTransitData()!=0)
	{
		cout<<"\tFailed to create transit nodes and links"<<endl;
        return 5;	
	}

	if(SZ_ReadWalk()!=0)
	{
		cout<<"\tFailed to create walk links"<<endl;
        return 6;	
	}

	if(SZ_ReadStopTrip()!=0)
	{
		cout<<"\tFailed to read stop-pair demand"<<endl;
        return 7;	
	}

	if(SZ_ReadNodeTrip()!=0)
	{
		cout<<"\tFailed to read centroid-pair demand"<<endl;
        return 8;	
	}

	ConnectAsymmetricLinks();



	cout<<"============= node size:"<<numOfNode<<",link size:"<<numOfLink<<",od-pair size:"<<numOfPTOD<<", trips:"<<numOfPTTrips<<",numberofdestination:"<<numOfPTDest<<"============="<<endl;
	
	cout<<"Successfully build the network"<<endl;

	return 0;

}

int PTNET::SZ_ReadStop()
{
	string stopname=networkName + "\\stop.csv";
	ifstream infile;
    if(!TNM_OpenInFile(infile, stopname))
    {
        return 1;
    }
	string pline;
    vector<string> words;
    cout<<"\tReading the stop file..."<<endl;
	if(!getline(infile, pline))//skip the first line
    {
        cout<<"Failed to read the stop informaiton from the following line: \n"<<pline<<endl;
        return 2;
    }

	 while(getline(infile, pline))
    {
        if(!pline.empty())//skip an empty line
        {
            PTStop *stop = new PTStop;

			vector<string> words;
			TNM_GetWordsFromLine(pline, words, ',', '"');
			if(words.size() !=4)
			{
				cout<<"expected at least four columns, only found "<<words.size()<<" from line: "<<pline<<endl;
				return 3;
			}
			stop->m_id = words[0];
			stop->m_name = words[1];

			double lat, lon;
			if (TNM_FromString(lon, words[2], std::dec) && TNM_FromString(lat, words[3], std::dec))
			{
				stop->m_pos.SetPos(lat,lon);
			}
			else 
			{
				cout<<"error gis format for line: "<<pline<<endl;
				return 4;
			
			}
	
			pair<PTStopMapIter,bool> ret = m_stops.insert(pair<string, PTStop*>(stop->m_id, stop));
			if(!ret.second)
            {
                cout<<"\tThe following stop may be duplicated and shall not be read: "<<endl;
                //stop->Print();
                delete stop;
            }
		}
	 }

	cout<<"\tRead in "<<m_stops.size()<<" stops"<<endl;

    infile.close();
	return 0;

}

int PTNET::SZ_ReadRoute()
{
	string routename = networkName + "\\route.csv";

	ifstream infile;
    if(!TNM_OpenInFile(infile, routename))
    {
        return 1;
    }
    string pline;
    vector<string> words;
    cout<<"\tReading the route file..."<<endl;
    if(!getline(infile, pline))
    {
        cout<<"Failed to read the route informaiton from the following line: "<<pline<<endl;
        return 2;
    }//skip the first line
    while(getline(infile, pline))
    {
        if(!pline.empty())//skip an empty line
        {
            PTRoute *route = new PTRoute;
			vector<string> words;
			TNM_GetWordsFromLine(pline, words, ',', '"');
			
			//cout<<pline<<endl;
			if(words.size() <5 || !TNM_FromString(route->m_type, words[4], std::dec))
			{
				cout<<"expected at least five columns, only found "<<words.size()<<" from line: "<<pline<<endl;
				return false;
			}
			route->m_id    = words[1];
			route->m_patternid = words[0];
			route->m_sname = words[2];
			route->m_lname = words[3];

			pair<PTRouteMapIter,bool> ret = m_routes.insert(pair<string, PTRoute*>(route->m_id, route));
			if(!ret.second)
			{
				cout<<"\tThe following route may be duplicated and shall not be read: "<<pline<<endl;
				delete route;
			}
			
        }
    }

	 cout<<"\tRead in "<<m_routes.size()<<" routes"<<endl;
    infile.close();
    return 0;

}

int PTNET::SZ_ReadShape()
{
	string infilename = networkName + "\\shape.csv";
    ifstream infile;
    if(!TNM_OpenInFile(infile, infilename))
    {
        cout<<"\tCannot open file "<<infilename<<" to read"<<endl;
        return 1;
    }
    cout<<"\tReading shape pattern information "<<endl;
    string line;
    vector<string> words;
    getline(infile, line);
	double ttfreq = 0.0;
    while(getline(infile, line))
    {
		TNM_GetWordsFromLine(line, words,',','"');
		if(words.size() < 5)
        {
            cout<<"\tExpected at least five columns, found "<<words.size()<<" on "<<line<<endl;
            return 1;
        }
		PTShape *shape = new PTShape;
		string shapeid=words[1]+"-"+words[0];
		shape->m_id = shapeid;
		PTRoute *route = GetRoutePtr(words[0]);
		if(!route)
        {
            cout<<"\tCannot find route "<<words[1]<<" on line "<<line<<endl;
            return 1;
        }
		shape->SetRoute(route);
		double f,cap;
		TNM_FromString(f, words[2], std::dec);
		TNM_FromString(cap, words[3], std::dec);
		shape->m_cap=cap;
		shape->m_freq=f;
		ttfreq += f;

		vector<string> iwords;
		TNM_GetWordsFromLine(words[4], iwords,',','"');
		if(iwords.size()<2)
		{
			cout<<"\t A transit line must have more than two stops: "<<words[4]<<" on line "<<line<<endl;
            return 2;
		}


		for(int i = 0;i<iwords.size();i++)
        {
            PTStop *stop = GetStopPtr(iwords[i]);
            if(stop)
            {
                shape->AddStop(stop);
				// add lines to stop 
				vector<PTShape*>::iterator ittt = find(stop->mshapes.begin(), stop->mshapes.end(), shape);
				if (ittt==stop->mshapes.end())
					stop->mshapes.push_back(shape);
            }
            else
            {
                cout<<"\tCannot find stop "<<words[i]<<" from line "<<line<<endl;
                return 3;
            }
        }
		pair<PTShapeMapIter,bool> ret =m_shapes.insert(PTShapeMapIter::value_type(shapeid, shape));
        if(!ret.second)
        {
            cout<<"\tThe following stop may be duplicated and shall not be read: "<<line<<endl;
            delete shape;
        }

	}
	cout<<"\tRead in total "<<m_shapes.size()<<" shapes, total freqency = "<<ttfreq<<endl;
    infile.close();
    return 0;


}

int	PTNET::SZ_ReadTEAPTransitData()
{
	string infilename = networkName + "//transit.csv";
    ifstream infile;
    if(!TNM_OpenInFile(infile, infilename))
    {
		cout<<"_transit.txt does not exist!"<<endl;
		return 1;
	}
	cout<<"\tReading transit running data from transit.txt"<<endl;

	string line;
    vector<string> words;      
	getline(infile, line);

	while(getline(infile, line))
    {
		TNM_GetWordsFromLine(line, words,',');
        if(words.size() != 6)
        {
            cout<<"\tExpected six columns, found "<<words.size()<<" on "<<line<<endl;
            return 1;
        }
		string shapeid=words[0]+'-'+words[1];
		PTShape* shp=GetShapePtr(shapeid);

		if (!shp)
		{
			cout<<"shape id "<<words[0]<<" does not exist in "<<line<<endl;
			return 3;		
		}
		PTStop* fstop=GetStopPtr(words[2]);
		PTStop* tstop=GetStopPtr(words[3]);
		if (!fstop||!tstop)
		{
			cout<<"stop id "<<words[0]<<" or "<<words[1]<<" does not exist in "<<line<<endl;
			return 4;		
		}
		if (words[2]!=words[3])
		{
			PTNode* tail;
			PTNode* head;
			tail=fstop->GetNode(shp->m_id+";ENROUTE");
			head=tstop->GetNode(shp->m_id+";ENROUTE");
			if (!tail||!head) 
			{
				cout<<"tranist link in shape "<<words[0]<<" in line"<<line<<"does not exist, check network representation!"<<endl;
				return 5;
			}
		
			PTLink *link = CatchLinkPtr(tail,head);
			if(link && link->GetTransitLinkType() == PTLink::ENROUTE) //link must be enroute
			{
				double fft,length;
				TNM_FromString(fft, words[4], std::dec);
				TNM_FromString(length, words[5], std::dec);
				link->pars.push_back(fft);//add transit free flow time to the parVector
				link->length = length;
				link->fft = fft;
			}
			else
			{
				cout<<"link must be enroute, check network representation!"<<endl;
				return 5;
			}
		}
		else
		{
			cout<<"Stop:"<<words[2]<<" to itself!"<<endl;
			return 5;
		}


	}
	infile.close();
	return 0;



}

int	PTNET::SZ_ReadWalk()
{
	vector<PTNode*> searnodevec;
	//Read traffic network
	TNM_SNET* net=new TNM_SNET(networkName);
	net->BuildSZFormatPS_T();

	TNM_SNODE* tonode = net->CatchNodePtr(3253);


	//creating centroid transfer nodes 
	int nid = nodeVector.size();
	int nn = 0;
	for(int i=0;i<net->numOfNode;i++)
	{
		TNM_SNODE* snode = net->nodeVector[i];
		if (snode->is_centroid)
		{
			PTNode* centroidnode = new PTNode();
			nid++;
			centroidnode->id = nid;
			centroidnode->SetTransitNodeType(PTNode::CENTROID);
			centroidnode->trafficnetid = snode->id;
			nodeVector.push_back(centroidnode);
			searnodevec.push_back(centroidnode);
			nn ++;
		}	
	}
	UpdateNodeNum();
	cout<<"\tCreate "<<nn<<" centroid nodes"<<endl;
	// match node id between transit and traffic network
	string odname=networkName + "//stop_node.csv"; //建议原文件stop不用重复
	ifstream infile;
    if(!TNM_OpenInFile(infile, odname))
    {
        return 1;
    }
    string pline;
    vector<string> words;

	 cout<<"\tReading the stop_node demand file..."<<endl;
	if(!getline(infile, pline))//skip the first line
    {
        cout<<"Failed to read the demand informaiton from the following line: \n"<<pline<<endl;
        return 2;
    }
	map<string,int> stopid;
	while(getline(infile, pline))
    {
        if(!pline.empty())//skip an empty line
        {
			TNM_GetWordsFromLine(pline, words, ',', '"');
			int id;
			TNM_FromString(id, words[2], std::dec);

			PTStop* stop = GetStopPtr(words[1]);
			if (!stop)
			{
				cout<<"we cannot find stop:"<<words[1]<<" from stop info"<<endl;
				return 2;
			}
			stop->GetTransferNode()->trafficnetid = id;


			vector<PTNode*>::iterator ittt = find(searnodevec.begin(), searnodevec.end(), stop->GetTransferNode());
			if (ittt==searnodevec.end())
				searnodevec.push_back(stop->GetTransferNode());


		}
	}
	infile.close();

	//creating walking links
	for(int i=0;i<net->numOfLink;i++)
	{
		TNM_SLINK* link=net->linkVector[i];
		link->cost=link->walktime;//路网link文件中有length、walk_time和ivtt三个参数，分别对应TNM_SLINK类的length、walktime和ivtt三个变量；
	}
	
	int n = 0;
	int lid = linkVector.size();
	for(vector<PTNode*>::iterator it =searnodevec.begin();it!=searnodevec.end();it++)
	{
		PTNode* node = *it;
		TNM_SNODE* fromnode = net->CatchNodePtr(node->trafficnetid);
		net->UpdateSP(fromnode);

		for(vector<PTNode*>::iterator fit =searnodevec.begin();fit!=searnodevec.end();fit++)
		{
			PTNode* tnode = *fit;
			if(tnode!=node)
			{
				TNM_SNODE* tonode = net->CatchNodePtr(tnode->trafficnetid);
				if (tonode->pathElem->via)
				{
					if (tonode->pathElem->via->cost<5)
					{
						//createing the walk link 
						lid++;
						PTLink* walklink = new PTLink(lid,node,tnode);
						walklink->SetTransitLinkType(PTLink::WALK);
					
						walklink->pars.push_back(1.0);//alpha_1
						walklink->pars.push_back(tonode->pathElem->via->cost*m_walkPenalty);  //using a random min					
						linkVector.push_back(walklink);

						n++;
				
					}
				}
			}
		}
	}
	UpdateLinkNum();

	cout<<"\tCreate "<<n<<" walking links"<<endl;


	return 0;
}


int	PTNET::SZ_ReadStopTrip()
{
	string odname=networkName + "//trip_stop_to_stop.csv";
	ifstream infile;
    if(!TNM_OpenInFile(infile, odname))
    {
        return 1;
    }
    string pline;
    vector<string> words;
	PTDestination* pDest;

	 cout<<"\tReading the stop demand file..."<<endl;
	if(!getline(infile, pline))//skip the first line
    {
        cout<<"Failed to read the demand informaiton from the following line: \n"<<pline<<endl;
        return 2;
    }

	multimap<string, string, less<string> > Dests;
	set<string> destkeys;
	while(getline(infile, pline))
    {
        if(!pline.empty())//skip an empty line
        {
			TNM_GetWordsFromLine(pline, words, ',', '"');
			PTStop *orgstop = GetStopPtr(words[0]);
			PTStop *deststop = GetStopPtr(words[1]);
			floatType d;
			TNM_FromString(d, words[2], std::dec);
			if (!orgstop||!deststop)
			{
				cout<<"cannot find stop info for line:"<<pline<<endl;
				return 3;
			}
			if (d>0&&words[0]!=words[1])
			{
				Dests.insert(pair<string, string>(words[1], words[0]+','+words[2]));
				destkeys.insert(words[1]);
			}
		}
	}
	
	for(set<string> ::iterator it=destkeys.begin();it!=destkeys.end ();it++)
	{
		string id = *it;
		multimap<string,string>::iterator beg = Dests.lower_bound(id);
		multimap<string,string>::iterator end = Dests.upper_bound(id);
		PTStop *stop = GetStopPtr(id);
		int numoforg = Dests.count(id);
		if((pDest = CreatePTDestination(stop->GetTransferNode(),numoforg)) == NULL)				
		{
			cout<<"cannot create static destination object!"<<endl;
			return 6;
		} 
		int i=0;
		while(beg != end)
		{
			string Nstr = beg->second;

			TNM_GetWordsFromLine(Nstr, words, ',', '"');

			floatType ded;
			PTStop *orgstop = GetStopPtr(words[0]);
			TNM_FromString(ded, words[1], std::dec);
			if (pDest->SetOrg(i+1,orgstop->GetTransferNode(),ded))
			{
				pDest->m_tdmd += ded;
				numOfPTTrips += ded;
			}
			else
			{
				cout<<"cannot set org flow:"<<ded<<" for dest:"<<words[1]<<endl;
				return 4;
			}
			++beg;
			i++;
		}
		//cout<<endl;
	}

	infile.close();
	return 0;
}


int	PTNET::SZ_ReadNodeTrip()
{
	string odname=networkName + "//trip_node_to_node.csv";
	ifstream infile;
    if(!TNM_OpenInFile(infile, odname))
    {
        return 1;
    }
    string pline;
    vector<string> words;
	PTDestination* pDest;

	 cout<<"\tReading the stop demand file..."<<endl;
	if(!getline(infile, pline))//skip the first line
    {
        cout<<"Failed to read the demand informaiton from the following line: \n"<<pline<<endl;
        return 2;
    }

	multimap<string, string, less<string> > Dests;
	set<string> destkeys;
	while(getline(infile, pline))
    {
        if(!pline.empty())//skip an empty line
        {
			TNM_GetWordsFromLine(pline, words, ',', '"');
			//int fid,tid;
			floatType d;
			//TNM_FromString(fid, words[0], std::dec);
			//TNM_FromString(tid, words[1], std::dec);
			TNM_FromString(d, words[2], std::dec);

			if(words[0]!=words[1] && d>0)
			{
				//
				Dests.insert(pair<string, string>(words[1], words[0]+','+words[2]));
				destkeys.insert(words[1]);

			}
		}
	}

	for(set<string> ::iterator it=destkeys.begin();it!=destkeys.end ();it++)
	{
		string id = *it;
		int destid;
		multimap<string,string>::iterator beg = Dests.lower_bound(id);
		multimap<string,string>::iterator end = Dests.upper_bound(id);
		TNM_FromString(destid, id, std::dec);
		PTNode* destnode = 	CatchNodePtrbyTrafficid(destid);
		int numoforg = Dests.count(id);

		if((pDest = CreatePTDestination(destnode,numoforg)) == NULL)				
		{
			cout<<"cannot create static destination object!"<<endl;
			return 6;
		} 

		int i=0;
		while(beg != end)
		{
			string Nstr = beg->second;
			TNM_GetWordsFromLine(Nstr, words, ',', '"');

			floatType ded;
			int orgid;
			TNM_FromString(orgid, words[0], std::dec);
			TNM_FromString(ded, words[1], std::dec);
			PTNode* orgnode = 	CatchNodePtrbyTrafficid(orgid);
			if (pDest->SetOrg(i+1,orgnode,ded))
			{
				pDest->m_tdmd += ded;
				numOfPTTrips += ded;
			}
			else
			{
				cout<<"cannot set org flow:"<<ded<<" for dest:"<<words[1]<<endl;
				return 4;
			}
			++beg;
			i++;
		}
	}
	infile.close();
	return 0;
}