#include "PTNet.h"
//#include "TNM_utility.h"
//#include "TNM_utility.cpp"
//#include "..\..\..\include\tnm\TNM_utility.h"
using namespace std;

bool PTStop::TAPInitialize(const string &inf,bool isPos)//网络初始化
{
	vector<string> words;
	TNM_GetWordsFromLine(inf, words, ',', '"');
	if(words.size() <4)
	{
		cout<<"expected at least four columns, only found "<<words.size()<<" from line: "<<inf<<endl;
		return false;
	}
	m_id=words[0];  //第0位，STOPID
	m_name = words[1];  //第1位，STOPNAME
	if (isPos) // txt info inculde stop position
	{
		double lat, lon;
		if (TNM_FromString(lat, words[2], std::dec) && TNM_FromString(lon, words[3], std::dec))
		{
			m_pos.SetPos(lat,lon);
		}
	}
	else m_pos.SetPos(0.0,0.0);

	return true;
}



int  PTNET::ReadTEAPStops(bool ispos)//公交站点(STOPID,STOPNAME,X,Y)
{
	string stopname=networkName + "_stop.txt"; //stop文件路径
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
			stop->TAPInitialize(pline,ispos);//does 		
			pair<PTStopMapIter,bool> ret = m_stops.insert(pair<string, PTStop*>(stop->m_id, stop));  //更新ret((STOPID,stop),bool)
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

int  PTNET::ReadTEAPRoutes()//公交路线(route_id,route_short_name,route_long_name,route_type)
{
	string routename = networkName + "_route.txt";

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
			if(words.size() <4 || !TNM_FromString(route->m_type, words[3], std::dec))  //route_type
			{
				cout<<"expected at least four columns, only found "<<words.size()<<" from line: "<<pline<<endl;
				return false;
			}
			route->m_id    = words[0]; //route_id
			route->m_sname = words[1]; //route_short_name
			route->m_lname = words[2]; //route_long_name

			pair<PTRouteMapIter,bool> ret = m_routes.insert(pair<string, PTRoute*>(route->m_id, route)); //更新ret((route_id,route),bool)
			if(!ret.second)
			{
				cout<<"\tThe following route may be duplicated and shall not be read: "<<endl;
				delete route;
			}

		}
	}

	cout<<"\tRead in "<<m_routes.size()<<" routes"<<endl;
	infile.close();
	return 0;


}

int	PTNET::ReadTEAPShapes()//公交线路信息(shape_id,route_id,frequency,capacity,stops)
{

	string infilename = networkName + "_shape.txt";
	ifstream infile; //读取txt文件
	if(!TNM_OpenInFile(infile, infilename))
	{
		cout<<"\tCannot open file "<<infilename<<" to read"<<endl;
		return 1;
	}
	cout<<"\tReading shape pattern information "<<endl;
	string line;
	vector<string> words;
	getline(infile, line);
	double ttfreq = 0.0;//sum of all routes frequences
	while(getline(infile, line))
	{
		TNM_GetWordsFromLine(line, words,',');//从每行中提取字符
		if(words.size() < 6)
		{
			cout<<"\tExpected at lest six columns, found "<<words.size()<<" on "<<line<<endl;
			return 1;
		}
		PTShape *shape = new PTShape;  //在栈上开新的存储空间？
		string shapeid=words[0]; //shape_id
		shape->m_id = shapeid;
		PTRoute *route = GetRoutePtr(words[1]); //route_id
		if(!route)
		{
			cout<<"\tCannot find route "<<words[1]<<" on line "<<line<<endl;
			return 1;
		}
		shape->SetRoute(route);//route_id
		double f,cap;
		TNM_FromString(f, words[2], std::dec);//frequency
		TNM_FromString(cap, words[3], std::dec);//capacity
		shape->m_cap=cap;
		//shape->m_cap=f*m_capacity; //发车频率(veh/h)*公交容量(pax/veh)=该公交线路每小时服务乘客容量(pax/h)
		shape->m_freq=f; //frequency
		ttfreq += f;//所有公交线路的频率之和
		for(int i = 4;i<words.size();i++)
		{
			PTStop *stop = GetStopPtr(words[i]); //公交沿线站点
			if(stop)
			{
				shape->AddStop(stop); //添加stop至shape中
				// add lines to stop ？
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
		pair<PTShapeMapIter,bool> ret =m_shapes.insert(PTShapeMapIter::value_type(shapeid, shape));//更新ret((shapeid, shape),bool)
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
/*
void PTNET::ReprojectNodeCoordinates()
{
int nnode = nodeVector.size();
double centralx = 0.0,  maxy = -POS_INF_FLOAT, miny = POS_INF_FLOAT;
for(int i = 0;i<nnode;i++)
{
PTNode *node = nodeVector[i];
centralx += node->GetStopPtr()->GetLon();
double ty = node->GetStopPtr()->GetLat();
if(maxy < ty) maxy = ty;
if(miny > ty) miny = ty;

}
centralx/=nnode;

//cout<<"maxy:"<<maxy<<"miny:"<<miny;
//cout<<"centralx:"<<centralx<<"  miny:"<<miny<<"  maxy:"<<maxy<<endl;
if((centralx >=-180 && centralx <=180 && maxy <= 90 && miny >=-90)
)//confirm if the coordinates was in lat/lon format. 
{

m_geoIF.InitializeOutGeoRef("NAD83", centralx, true);//geo interface. initialized in ReadPTNode; check if average stop x is between -180 and 180.
m_geoIF.InitializeInGeoRefAsLatLonOfOutRef();
m_geoIF.InitializeTransform(false);
SetProjection(true);
for(int i = 0;i<nnode;i++)
{
PTNode *node = nodeVector[i];
double tx = node->GetStopPtr()->GetLon(), 
ty = node->GetStopPtr()->GetLat();
//cout<<"id:"<<node->id<<" tx:"<<tx<<" ty:"<<ty<<endl;
m_geoIF.ReprojectPoints(1, &tx,&ty);
//cout<<"id:"<<node->id<<" tx:"<<tx<<" ty:"<<ty<<endl;
node->xCord = tx;
node->yCord = ty;
}
}
else
{
for(int i = 0;i<nnode;i++)
{
PTNode *node = nodeVector[i];
double tx = node->GetStopPtr()->GetLon(), 
ty = node->GetStopPtr()->GetLat();           
node->xCord = tx;
node->yCord = ty;
}
}


}
*/


int  PTNET::CreateTEAPNodeLinks()//点弧
{
	cout<<endl<<"============= Start to create nodes and links ============="<<endl;
	cout<<"\tCreating transfer nodes for each stop..."<<endl;
	int nid = 0,lid = 0;
	//这些符号表示决定路段成本的参数
	floatType power = 2;
	floatType alpha_2 = 1.0, beta_2 = 1.0, gama_2 = 1.2;
	floatType beta_3 = 1.0, gama_3 = 0.2;
	floatType alpha_3 = 1.0;
	for(PTStopMapIter pv = m_stops.begin(); pv!= m_stops.end(); pv++) //pv 遍历所有公交站点
	{
		PTStop* stop=pv->second;//这个second指的第二个存储的值
		nid++;
		PTNode* transfernode = new PTNode();//公交站点节点
		transfernode->id = nid;
		transfernode->xCord = stop->GetLon();//站点节点的x坐标是站点的经度
		transfernode->yCord = stop->GetLat(); //站点节点的y坐标是站点的纬度
		transfernode->SetTransitNodeType(PTNode::TRANSFER);
		transfernode->SetStopPtr(stop);//ptr是什么
		stop->SetTransferNode(transfernode);//transfernode中m_stop存储经过该站点节点的路线信息
		nodeVector.push_back(transfernode);
	}
	//ReprojectNodeCoordinates();
	cout<<"\tCreating transit nodes and links for each pattern..."<<endl;	 

	for(PTShapeMapIter ps = m_shapes.begin(); ps!=m_shapes.end();ps++) //ps 遍历各条公交路线
	{		 
		PTShape* shp=ps->second;		//shp是shape

		vector<PTStop*> pvec = shp->m_stops; //pvec 该路线经过的各站点
		//double* shpdist=shp->GetShapeDistancePtr();
		int numofstops=pvec.size(); //线路经过站点数
		double freq=shp->m_freq; //线路频率
		double cap=shp->m_cap; //线路服务人数（人/h）
		//cout<<shp->m_id<<","<<numofstops<<endl;
		if(numofstops > 1 ) //构建上车弧/下车弧/行驶弧
		{
			for (int nseq=0;nseq<numofstops;nseq++)//nseq 该路线经过的各站点的站序
			{
				PTStop *stop = pvec[nseq];
				PTNode *transfernode=stop->GetTransferNode();

				if (nseq>0 && pvec[nseq]->m_id==pvec[nseq-1]->m_id)//重复？
				{
					cout<<"exception occurs because stop "<<stop->m_id<<" in pattern "<<shp->m_id<<" visit to itself !"<<endl;
					return 4;
				}
				//上面是站点节点，下面是虚拟站节点，代码类似
				PTNode *transitnode = new PTNode();
				nid++;
				transitnode->id = nid;
				nodeVector.push_back(transitnode);


				if(!transitnode)
				{
					cout<<"\tFailed to create a new transit node for shape "<<shp->m_id<<" in sequence of "<<nseq<<" at stop "<<stop->m_id<<endl;
					return 1;
				}
				transitnode->SetTransitNodeType(PTNode::ENROUTE);//TNTYPE其中的类型
				transitnode->SetStopPtr(stop);  
				transitnode->SetShapePtr(shp);
				transitnode->SetRoutePtr(shp->m_routePtr);		
				if (stop->AddNode(shp->m_id+";ENROUTE",transitnode)) 
				{   //该站点不是地铁线路站点，无站台节点（也称换乘节点），可以创建在站点节点和虚拟站节点之间创建transit/board/alight弧
					//No transit node for the stop have ever created, we can create /transit/board/alight links
					if (nseq<numofstops-1)// last stop cannot aboard，大前提
					{
						lid ++;   
						PTLink* boardlink = new PTLink(lid,transfernode,transitnode);//上车弧(站点节点tail->虚拟站节点head)
						boardlink->SetTransitLinkType(PTLink::ABOARD);
						boardlink->m_hwmean = timescaler / freq;//set the mean headway time to the line!!!!!!再仔细看看
						linkVector.push_back(boardlink);		
						if(boardlink == NULL)
						{
							cout<<"Error: Fail to create a boarding link for "<<stop->m_id<<" in pattern "<<shp->m_id<<endl;
							return 7;
						}
						else
						{
							//boardlink->pars.push_back(freq);
							boardlink->m_shape=shp;//该上车弧所在公交线路信息
							boardlink->seq = nseq;//seq是上车路段路线的站点的顺序，nseq只是表示的一个参数
							shp->m_boardlinks.push_back(boardlink);
							boardlink->pars.push_back(beta_3);
							boardlink->pars.push_back(gama_3);
							boardlink->pars.push_back(power);
							boardlink->pars.push_back(cap);
							boardlink->cap = cap;
							boardlink->freq = freq/timescaler; //频率 (veh/min)
						}			
					}

					if(nseq>0)//first stop cannot alight，大前提
					{
						lid ++;   
						//========**initial setting for alghting link**====== //下车弧 
						PTLink* alghtinglink = new PTLink(lid,transitnode,transfernode);	
						alghtinglink->SetTransitLinkType(PTLink::ALIGHT);

						alghtinglink->m_shape=shp;
						alghtinglink->seq = nseq - 1;//seq是指下车的站序，因为下车的站点是从1开始，所以减1？
						shp->m_alightlinks.push_back(alghtinglink);

						alghtinglink->pars.push_back(alpha_3);//alpha_3
						alghtinglink->pars.push_back(m_alightLoss);	//m_alightLoss
						alghtinglink->cap = cap;
						linkVector.push_back(alghtinglink);	//弧集合

						//transit link initialize//行驶弧
						lid ++;    
						PTNode* tailnode = pvec[nseq-1]->GetNode(shp->m_id+";ENROUTE");
						PTLink* transitlink = new PTLink(lid,tailnode,transitnode);	 
						/////===========initial setting for transit link 
						transitlink->SetTransitLinkType(PTLink::ENROUTE);
						transitlink->m_shape=shp;
						transitlink->seq = nseq-1;//跟下车弧一样

						transitlink->pars.push_back(alpha_2);
						transitlink->pars.push_back(beta_2);
						transitlink->pars.push_back(gama_2);
						transitlink->pars.push_back(power);//power
						transitlink->pars.push_back(cap);
						transitlink->cap = cap;
						linkVector.push_back(transitlink);	
					}

				}
				else
				{   //该站点是地铁线路站点，还需构造站台节点（也称换乘节点），仅能在虚拟站节点之间创建transit link(行驶弧)
					//transit node must have been created,we only create transit link

					cout<<"\n  pattern "<<ps->second->m_id<<", stop "<<stop->m_id<<" is visited more than once, please make sure the connection is right!"<<endl;
					nodeVector.pop_back();
					//transit link from laststop dwell node to current stop updated transit node
					lid ++;                   
					PTNode* tail = pvec[nseq-1]->GetNode(shp->m_id+";ENROUTE");
					PTNode* head = stop->GetNode(shp->m_id+";ENROUTE");
					PTLink *transitlink = new PTLink(lid,tail,head);	 
					transitlink->SetTransitLinkType(PTLink::ENROUTE);			
					//===========initial setting for transit link===============
					if(transitlink == NULL)
					{
						cout<<"Error: Fail to create a transit link for "<<stop->m_id<<" to "<<transitnode->GetStopPtr()->m_id<<" in pattern "<<shp->m_id<<endl;						
					}		
					else
					{
						transitlink->m_shape=shp;
						transitlink->pars.push_back(alpha_2);
						transitlink->pars.push_back(beta_2);
						transitlink->pars.push_back(gama_2);
						transitlink->pars.push_back(power);//power
						transitlink->pars.push_back(cap);
						transitlink->cap = cap;
					}	
					linkVector.push_back(transitlink);	
				}
			}				
		}
		else
		{
			cout<<"\n  pattern "<<ps->second->m_id<<" has less than 2 stops"<<endl;
			return 5;
		}
	}
	UpdateNodeNum();
	UpdateLinkNum();
	return 0;
}

int	PTNET::ReadTEAPTransitData() //道路弧信息(shapeid,fromstop,tostop,fft,length)
{
	string infilename = networkName + "_transit.txt";
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
		if(words.size() != 5)
		{
			cout<<"\tExpected five columns, found "<<words.size()<<" on "<<line<<endl;
			return 1;
		}
		string shapeid=words[0];
		PTShape* shp=GetShapePtr(words[0]);//ptr是指针记录？

		if (!shp)
		{
			cout<<"shape id "<<words[0]<<" does not exist in "<<line<<endl;
			return 3;		
		}
		PTStop* fstop=GetStopPtr(words[1]); //fromstop尾节点
		PTStop* tstop=GetStopPtr(words[2]); //tostop头节点
		if (!fstop||!tstop)//或
		{
			cout<<"stop id "<<words[0]<<" or "<<words[1]<<" does not exist in "<<line<<endl;
			return 4;		
		}
		if (words[1]!=words[2])
		{
			PTNode* tail;
			PTNode* head;
			tail=fstop->GetNode(shp->m_id+";ENROUTE"); //fromstop尾节点
			head=tstop->GetNode(shp->m_id+";ENROUTE"); //tostop头节点
			if (!tail||!head) 
			{
				cout<<"tranist link in shape "<<words[0]<<" in line"<<line<<"does not exist, check network representation!"<<endl;
				return 5;
			}

			PTLink *link = CatchLinkPtr(tail,head); //公交站点ns匹配的道路弧ar
			if(link && link->GetTransitLinkType() == PTLink::ENROUTE) //link must be enroute，link的类型
			{
				double fft,length;
				TNM_FromString(fft, words[3], std::dec);
				TNM_FromString(length, words[4], std::dec);
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
			cout<<"Stop:"<<words[1]<<" to itself!"<<endl;
			return 5;
		}


	}
	infile.close();
	return 0;

}

void PTNET::GetVicinityNodes(COORDMAP &xmap, COORDMAP &ymap, vector<PTNode *> &nvec, PTNode *node, long threshold) //获取临近节点neighbor
{
	nvec.clear();
	if(!node) return;
	long xLB = node->xCord - threshold, xUB = node->xCord + threshold, yLB = node->yCord-threshold,
		yUB = node->yCord + threshold;
	COORDMAP_I xLiter, xUiter, yLiter, yUiter, pv;
	if(xLB > xmap.rbegin()->first || xUB < xmap.begin()->first || yLB > ymap.rbegin()->first || yUB < ymap.begin()->first) 
	{
		//	cout<<"no vicinity set found for node "<<node->id<<endl;
		return;
	}

	xLiter = xmap.lower_bound(xLB);
	xUiter = xmap.upper_bound(xUB);
	yLiter = ymap.lower_bound(yLB);
	yUiter = ymap.upper_bound(yUB);

	set<int> neighbor;	
	pv = xLiter;
	do 
	{
		neighbor.insert(pv->second);
		if(pv!= xUiter) pv++;
	}while(pv!= xUiter);
	if(neighbor.empty()) return;
	set<int>::const_iterator ei = neighbor.end();
	pv = yLiter;
	do
	{
		if(neighbor.find(pv->second)!= ei) //so we find it.
		{
			nvec.push_back(nodeVector[pv->second-1]);
		}
		if(pv!= yUiter) pv++;
	}while(pv!=yUiter);
}



#include <math.h>
// 角度转弧度
double rad(double d)
{
	const double PI = 3.1415926535898;
	return d * PI / 180.0;
}
// 传入两个经纬度，计算之间的大致直线距离
int CalcDistance(float fLati1, float fLong1, float fLati2, float fLong2)
{
	const float EARTH_RADIUS = 6378.137;

	double radLat1 = rad(fLati1);
	double radLat2 = rad(fLati2);
	double a = radLat1 - radLat2;
	double b = rad(fLong1) - rad(fLong2);
	double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
	s = s * EARTH_RADIUS;
	s = (int)(s * 10000000) / 10000;
	return s;
}



int  PTNET::CreateTEAPWalks(bool Traj) //步行弧
{
	int lid = numOfLink; //当前弧数量
	int n=0;
	double walklinkTTlength,walklinkTTcost = 0.0;
	if (Traj)//如果Traj=True
	{
		//without stop position, walk links comes from external file//创建步行弧
		//string walkname=networkName + "_walks_org.txt"; //sf网络的无容量分配的walklink文件
		string walkname=networkName + "_walks.txt";
		ifstream infile;
		if(!TNM_OpenInFile(infile, walkname))
		{
			return 1;
		}
		string pline;
		vector<string> words;		
		if(!getline(infile, pline))//skip the first line
		{
			cout<<"Failed to read the stop informaiton from the following line: \n"<<pline<<endl;
			return 2;
		}

		while(getline(infile, pline))
		{
			if(!pline.empty())//skip an empty line
			{
				n++;
				TNM_GetWordsFromLine(pline, words,',','"');
				PTStop *fstop = GetStopPtr(words[0]);
				PTStop *tstop = GetStopPtr(words[1]);
				if (fstop&&tstop)//都非零
				{
					lid++;
					PTLink* walklink = new PTLink(lid,fstop->GetTransferNode(),tstop->GetTransferNode());
					walklink->SetTransitLinkType(PTLink::WALK);
					TNM_FromString(walklink->length, words[3], std::dec);
					double fft;
					TNM_FromString(fft, words[2], std::dec);
					if(walklink==NULL)
					{
						cout<<"\tFailed to create a walk link from stop:"<<fstop->m_id<<" to stop:"<<tstop->m_id<<endl;
						return 2;
					}     
					else
					{
						walklink->pars.push_back(1.0);//alpha_1
						walklink->pars.push_back(fft);
						walklink->fft = fft;
					}
					linkVector.push_back(walklink);
				}
				else
				{
					cout<<"Walking from the line"<<pline<<", can not find the stop!"<<endl;
					return 1;
				}
			}
		}
		infile.close();
	}
	else//Traj=False
	{
		// create walk link according to the stop position
		double walkSpeed = 5.0;  //kn/h 5.0/60 * 1000 * 3.28084; //unit: ft per minute.  //1米(m)等于3.2808英尺(ft) 5km/h->273.4ft/min
		//double walkRadius = walkSpeed * m_maxWalkTime; //total distance one can walk in five minutes.
		double walkRadius = 300.0;
		double walkRadiusrad = walkRadius/111*0.001; //m->经纬度   // 300.0 * 3.2808; //300m->984.24ft
		//cout<<"walkSpeed="<<walkSpeed<<"km/h, walkRadius="<<walkRadius<<"度"<<endl;
		COORDMAP xm, ym;//迭代器
		for(PTStopMapIter ps = m_stops.begin(); ps!= m_stops.end(); ps++) //ps遍历各站点
		{
			PTNode* node = ps->second->GetTransferNode();  //公交站点节点的位置信息
			xm.insert(pair<long, int>(node->xCord, node->id)); //xm [id](xCord, node id)
			ym.insert(pair<long, int>(node->yCord, node->id));				
		}
		for(PTStopMapIter ps = m_stops.begin(); ps!= m_stops.end(); ps++) //ps遍历各站点
		{
			PTNode* node = ps->second->GetTransferNode();
			if(node)  //A stop may not have a transfer node if it is not used by any pattern. //只对站点节点transferNode操作
			{
				std::vector<PTNode*> vnodes;
				GetVicinityNodes(xm, ym, vnodes, node, walkRadiusrad);//获取该站点节点node在最大步行半径walkRadius内的邻近节点vnodes
				int j=0; 
				for(int i = 0;i<vnodes.size();i++)
				{
					PTNode* hnode = vnodes[i];//邻近节点里面一个具体的点
					if(hnode->GetTransitNodeType() == PTNode::TRANSFER && hnode!=node) //only if it is a transfer node. 即hnode(=transferNode&&!=源节点)，连接公交站点节点
					{
						//double dist = hnode->MeasureDist(node);
						double dist = CalcDistance(node->yCord, node->xCord, hnode->yCord, hnode->xCord); //使用经纬度计算节点距离（m）
						//cout<<"dist = "<<dist<<"m"<<endl;
						if(dist <= walkRadius)
						{
							j++; //计算满足对于当前站点node，与其距离dist <= walkRadius的点的个数（即构建的步行弧数量）
							n++;
							lid++;
							PTLink* walklink = new PTLink(lid,node,hnode); //站间换乘弧（步行弧） tail->head
							walklink->SetTransitLinkType(PTLink::WALK);  //弧类型：步行弧 m_tlType=WALK(4)
							walklink->length = dist/1000;   //km  //dist/3280.84;  //unit ft->km	步行弧长度
							if(walklink==NULL)
							{
								cout<<"\tFailed to create a walk link from node "<<node->id<<" to "<<hnode->id<<endl;
								return 2;
							}     
							else
							{
								walklink->pars.push_back(1.0);//alpha_1
								walklink->pars.push_back(walklink->length/5.0*60.0); //walktime(min)
								walklinkTTlength += walklink->length;
								walklinkTTcost += walklink->length/5.0*60.0;
								//cout<<"walklink length = "<<walklink->length<<"km"<<"  walklink time = "<<walklink->pars[1]<<"min"<<endl;
							}		
							linkVector.push_back(walklink); //插入linkVector
						}
					}
				}
				//cout<<"站点"<<node->m_stop->m_name<<"的步行范围内所构建的步行弧数量："<<j<<endl;
			}
		}

	}
	UpdateNodeNum();
	UpdateLinkNum();
	cout<<"\tCreate "<<n<<" walking links"<<endl;
	//cout<<"步行弧：平均长度="<<walklinkTTlength/n*1000<<"m, 平均时间"<<walklinkTTcost/n<<"min"<<endl;
	return 0;
}



void	PTNET::CreateWalkLink() //在站点之间构建步行弧 （for Gentile网络）
{
	int lid = numOfLink;
	//srand(10);
	PTLink* walklink;
	int a;
	//站点a->b
	walklink = new PTLink(linkVector.size()+1,nodeVector[1-1],nodeVector[2-1]); //(tailnodeid-1,headnodeid-1)
	walklink->SetTransitLinkType(PTLink::WALK);
	walklink->pars.push_back(1.0);//alpha_1
	walklink->pars.push_back(a = 35.0);  //walk link cost	
	cout<<"新增步行弧id="<<walklink->id<<"("<<walklink->tail->id<<","<<walklink->head->id<<") cost="<<a<<endl;
	linkVector.push_back(walklink);
	UpdateLinkNum();
	//站点a->c
	walklink = new PTLink(linkVector.size()+1,nodeVector[1-1],nodeVector[3-1]); //(tailnodeid-1,headnodeid-1)
	walklink->SetTransitLinkType(PTLink::WALK);
	walklink->pars.push_back(1.0);//alpha_1
	walklink->pars.push_back(a = 60.0);  //walk link cost	
	cout<<"新增步行弧id="<<walklink->id<<"("<<walklink->tail->id<<","<<walklink->head->id<<") cost="<<a<<endl;
	linkVector.push_back(walklink);
	UpdateLinkNum();
	//站点a->d
	walklink = new PTLink(linkVector.size()+1,nodeVector[1-1],nodeVector[4-1]); //(tailnodeid-1,headnodeid-1)
	walklink->SetTransitLinkType(PTLink::WALK);
	walklink->pars.push_back(1.0);//alpha_1
	walklink->pars.push_back(a = 100.0);  //walk link cost	
	cout<<"新增步行弧id="<<walklink->id<<"("<<walklink->tail->id<<","<<walklink->head->id<<") cost="<<a<<endl;
	linkVector.push_back(walklink);
	UpdateLinkNum();
	//节点b->c
	walklink = new PTLink(linkVector.size()+1,nodeVector[2-1],nodeVector[3-1]); //(tailnodeid-1,headnodeid-1)
	walklink->SetTransitLinkType(PTLink::WALK);
	walklink->pars.push_back(1.0);//alpha_1
	walklink->pars.push_back(a = 27.0);  //walk link cost	
	cout<<"新增步行弧id="<<walklink->id<<"("<<walklink->tail->id<<","<<walklink->head->id<<") cost="<<a<<endl;
	linkVector.push_back(walklink);
	UpdateLinkNum();
	//节点b->d
	walklink = new PTLink(linkVector.size()+1,nodeVector[2-1],nodeVector[4-1]); //(tailnodeid-1,headnodeid-1)
	walklink->SetTransitLinkType(PTLink::WALK);
	walklink->pars.push_back(1.0);//alpha_1
	walklink->pars.push_back(a = 60.0);  //walk link cost	
	cout<<"新增步行弧id="<<walklink->id<<"("<<walklink->tail->id<<","<<walklink->head->id<<") cost="<<a<<endl;
	linkVector.push_back(walklink);
	UpdateLinkNum();
	//站点c->d
	walklink = new PTLink(linkVector.size()+1,nodeVector[3-1],nodeVector[4-1]); //(tailnodeid-1,headnodeid-1)
	walklink->SetTransitLinkType(PTLink::WALK);
	walklink->pars.push_back(1.0);//alpha_1
	walklink->pars.push_back(a = 38.0);  //walk link cost	
	cout<<"新增步行弧id="<<walklink->id<<"("<<walklink->tail->id<<","<<walklink->head->id<<") cost="<<a<<endl;
	linkVector.push_back(walklink);
	UpdateLinkNum();
	int lidnow = numOfLink;
	cout<<"\tCreate "<<lidnow-lid<<" walking links"<<endl;
	return;
}

void	PTNET::CreateODLink() //对于每对OD 构建walklink(org,dest)
{
	int lid = numOfLink;
	int n=0;
	PTDestination* dest;
	srand(10);
	PTLink* walklink;
	double ttodwalkcost,avgodwalkcost = 0.0;
	for (int i = 0;i<numOfPTDest; i++)
	{
		dest = PTDestVector[i];
		for (int j=0; j<dest->numOfOrg; j++)
		{		
			PTOrg* org= dest->orgVector[j];
			lid++;

			walklink = CatchLinkPtr(org->org,dest->destination);

			if (!walklink) 
			{

				walklink = new PTLink(lid,org->org,dest->destination);
				walklink->SetTransitLinkType(PTLink::WALK);
				if(walklink==NULL)
				{
					cout<<"\tFailed to create a walk link from node "<<org->org->id<<" to "<<dest->destination->id<<endl;
					return ;
				}     
				else
				{
					walklink->pars.push_back(1.0);//alpha_1	
					double a= rand() % 60 + 90;
					walklink->pars.push_back(a);  //using a random min
					walklink->odwalk = true;
					ttodwalkcost += a;
				}		
				linkVector.push_back(walklink);

				n++;
			}
			else
			{
				cout<<"OD:"<<org->org->id<<"-"<<dest->destination->id<<" already have links."<<endl;
			}

		}
	}


	UpdateLinkNum();
	avgodwalkcost = ttodwalkcost/n;
	cout<<"\tAvg walking links cost = "<<avgodwalkcost<<endl;
	cout<<"\tCreate "<<n<<" walking links"<<endl;

	/*
	//PTLink *link;
	//16974,398,11594,12743,403,11599,12748,11598,401,12746,406,12751,405,12750	
	cout<<"TEAP-1,CTEAP-1:";
	cout<<linkVector[16973]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[397]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11593]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12742]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[402]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11598]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12747]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11597]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[400]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12745]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[405]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12750]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[404]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12749]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12749]->head->GetStopPtr()->m_id<<endl<<endl;

	//16974,398,11594,12743,15809,403,11599,12748,15814,11598,15813,401,12746,406,12751,405,12750
	cout<<"TEAP-2,CTEAP-4:";
	cout<<linkVector[16973]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[397]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11593]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12742]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[15808]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[402]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11598]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12747]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[15813]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11597]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[15812]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[400]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12745]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[405]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12750]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[404]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12749]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12749]->head->GetStopPtr()->m_id<<endl<<endl;

	//16974,398,11594,12743,14786,403,11599,12748,14791,11598,14790,401,12746,406,12751,405,12750	
	cout<<"CTEAP-2:";
	cout<<linkVector[16973]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[397]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11593]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12742]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[15808]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[402]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11598]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12747]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[14790]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11597]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[14789]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[400]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12745]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[405]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12750]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[404]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12749]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12749]->head->GetStopPtr()->m_id<<endl<<endl;

	//16974,398,11594,12743,14786,15809,403,11599,12748,14791,15814,11598,14790,15813,401,12746,406,12751,405,12750
	cout<<"CTEAP-3:";
	cout<<linkVector[16973]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[397]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11593]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12742]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12742]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[15808]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[402]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11598]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12747]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[14790]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[15813]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[11597]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[14789]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[15812]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[400]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12745]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[405]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12750]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[404]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12749]->tail->GetStopPtr()->m_id<<"->";
	cout<<linkVector[12749]->head->GetStopPtr()->m_id<<endl<<endl;
	
	cout<<endl;
	*/

	return;

}

PTDestination*	PTNET::CreatePTDestination(PTNode* rootDest, int noo)
{
	if (noo <0)
	{
		cout<<"\n\Dest "<<rootDest->id<<" contains none or negative orgs."<<endl;
		return NULL;
	}
	PTDestination *dest = new PTDestination(rootDest, noo);
	if (dest == NULL)
	{
		cout<<"\n\tCannot allocate memory for new dest"<<endl;
		return NULL;
	}
	PTDestVector.push_back(dest);
	numOfPTDest ++;
	numOfPTOD += noo;
	return dest;
}

PTDestination*	PTNET::CreatePTDestination(int nid, int noo)
{
	PTNode *node = nodeVector[nid-1];
	if(node == NULL)
	{
		cout<<"\n\tnode "<<nid<<" is not a valid node object. "<<endl;
		return NULL;
	}
	else 
	{
		if (noo <0)
		{
			cout<<"\n\Dest "<<node->id<<" contains none or negative orgs."<<endl;
			return NULL;
		}
		PTDestination *dest = new PTDestination(node, noo);
		if (dest == NULL)
		{
			cout<<"\n\tCannot allocate memory for new dest"<<endl;
			return NULL;
		}
		PTDestVector.push_back(dest);
		numOfPTDest ++;
		numOfPTOD += noo;
		return dest;
	}
}

int  PTNET::ReadTEAPODdemand()
{
	string odname=networkName + "_trip.txt";
	ifstream infile;
	if(!TNM_OpenInFile(infile, odname))
	{
		return 1;
	}
	string pline;
	vector<string> words,iwords;
	PTDestination* pDest;

	cout<<"\tReading the od demand file..."<<endl;
	if(!getline(infile, pline))//skip the first line
	{
		cout<<"Failed to read the demand informaiton from the following line: \n"<<pline<<endl;
		return 2;
	}
	while(getline(infile, pline))
	{
		if(!pline.empty())//skip an empty line
		{
			TNM_GetWordsFromLine(pline, words, '\t', '"');//words 记录DestinationID, number of origins
			if (words[0]=="Destination")  
			{
				int numoforg;				
				TNM_FromString(numoforg, words[2], std::dec); //number of origins
				getline(infile, pline);
				TNM_GetWordsFromLine(pline, iwords, ',', '"'); //iwords 记录该终点对应的不同起点和需求
				PTStop *stop = GetStopPtr(words[1]); //DestinationID
				if (!stop) 
				{
					cout<<"The stop:"<<words[1]<<" do not exist!"<<endl;
					return 3;
				}

				if((pDest = CreatePTDestination(stop->GetTransferNode()->id,numoforg)) == NULL)				
				{
					cout<<"cannot create static destination object!"<<endl;
					return 6;
				} 
				//				pDest->m_tdmd = 0.0;
				if (iwords.size() == numoforg) 
				{
					vector<string> dd;
					for (int i=0;i<iwords.size();++i) //i遍历该终点对应的每一个起点
					{
						TNM_GetWordsFromLine(iwords[i],dd, ':', '"');
						floatType ded;
						PTStop *dstop = GetStopPtr(dd[0]); //dstop 对应dd[0] 起点 
						TNM_FromString(ded, dd[1], std::dec); //ded 对应dd[1] OD需求
						if (dstop)
						{
							//TNM_SNODE* node = CatchNodePtr(dstop->GetTransferNode()->id);
							if (dstop->GetTransferNode()) 
							{
								if (pDest->SetOrg(i+1,dstop->GetTransferNode(),ded))
								{
									pDest->m_tdmd += ded; //该终点的total demend
									numOfPTTrips += ded; //所有OD对total demend
								}
								else
								{
									cout<<"cannot set org flow:"<<ded<<" for dest:"<<words[1]<<endl;
									return 4;
								}
							}
						}
						else
						{
							cout<<"The stop:"<<dd[0]<<" do not exist!"<<endl;
							return 3;						
						}		
					}
				}
				else
				{
					cout<<"Destination stop:"<<words[1]<<" has wrong number of org matching"<<endl;
					return 2;			
				}
			}
		}
	}
	/* //输出每个终点对应的起点和需求
	cout<<"OD对信息"<<endl;
	for(int m=0;m<PTDestVector.size();m++)
	{   
	cout<<"--Destination Nodeid:"<<PTDestVector[m]->destination->m_stop->m_id<<endl;
	for(int n=0;n<PTDestVector[m]->numOfOrg;n++)
	{
	cout<<"Orgin Nodeid:"<<PTDestVector[m]->orgVector[n]->org->m_stop->m_id<<endl;
	cout<<"ODdemand:"<<PTDestVector[m]->orgVector[n]->assDemand<<endl;
	}
	}
	*/
	

	return 0;




}

int	PTNET::BuildAN(bool stopPos, bool walkfile)
{
	//m_stops 公交网络站点(m_id=stop_id)，来自network_stop.txt(STOPID,STOPNAME,X,Y) 
	if(ReadTEAPStops(stopPos)!=0) 
	{
		cout<<"\tFailed to read stops"<<endl;
		return 1;
	}

	//m_routes 公交网络线路(m_id=route_id)，来自network_route.txt(route_id,route_short_name,route_long_name,route_type)
	if(ReadTEAPRoutes()!=0)  
	{
		cout<<"\tFailed to read routes"<<endl;
		return 2;
	}

	//m_shapes 公交网络线路信息(m_id=shape_id)，来自network_shape.txt(shape_id,route_id,frequency,capacity,stops)
	//m_stops 各条线路经过的站点，其中的mshapes记录经过该站点的线路
	//m_cap 各条线路每小时最高服务人数-通过容量(人/h)
	//m_freq 各条线路频率(veh/h)
	if(ReadTEAPShapes()!=0)  
	{
		cout<<"\tFailed to read shapes"<<endl;
		return 3;
	}

	//构建常规公交和地铁的节点和弧（插入nodeVector和linkVector；计数numOfLink和numOfNode）
	//节点：公交站点节点、虚拟站点节点  (若为地铁，还需要构建站台节点，此处无)
	//m_tnType: TRANSFER(0); ENROUTE(1)
	//forwStar从节点i出发的前向弧集；backStar到达节点i的后向弧集；
	//m_stop 节点i信息（m_transferNode节点i对应站点节点信息；m_nodes节点i对应虚拟站点节点信息；m_id节点ID；mshapes经过节点的路线）
	//弧：上车弧(站点节点->虚拟站点节点)；下车弧(虚拟站点节点->站点节点)；行驶弧(虚拟站点节点->虚拟站点节点)
	//mlType: ABOARD(1); ALIGHT(0); ENROUTE(2)
	//tail尾节点; head头节点
	//m_hwmean平均车头时距(60/freq); pars弧参数; freq弧频率(veh/min); m_shape弧所在线路的信息
	if(CreateTEAPNodeLinks()!=0)
	{
		cout<<"\tFailed to create transit nodes and links"<<endl;
		return 4;	
	}
	//新增道路弧信息，来自network_transit.txt(shapeid,fromstop,tostop,fft,length)
	//连接行驶弧两端的虚拟站点节点m_tnType=ENROUTE(1) (tail->head)
	//插入fft自由流时间; length线路长度; 
	if(ReadTEAPTransitData()!=0)
	{
		cout<<"\tFailed to create transit nodes and links"<<endl;
		return 5;	
	}
	//构建步行弧（不同公交站点节点之间的步行换乘）
	if(CreateTEAPWalks(walkfile)!=0)
	{
		cout<<"\tFailed to create walk links"<<endl;
		return 6;	
	}
	//PTDestVector OD对信息，来自network_trip.txt(destination,number of origins. org id:demand)
	//其中，destination终点；numOdOrg对应起点数；m_tdmd该终点所属的总OD需求；
	//numOfPTDest 终点数量；numOfPTOD OD对数量；numOfPTTrips 总需求量；
	if(ReadTEAPODdemand()!=0)
	{
		cout<<"\tFailed to read od demand"<<endl;
		return 7;	
	}

	//记录各条上车弧（或行驶弧）link所在公交线路对应的行驶弧（或上车弧）rlink
	ConnectAsymmetricLinks();

	cout<<"============= node size:"<<numOfNode<<",link size:"<<numOfLink<<",od-pair size:"<<numOfPTOD<<", trips:"<<numOfPTTrips<<",numberofdestination:"<<numOfPTDest<<"============="<<endl;

	cout<<"Successfully build the network"<<endl;

	//PrintNetLinks();

	return 0;
}

int	PTNET::AllocateLinkBufferData(int size) //令linkBufferSize = size；同时对于每一条link设置size个buffer，初始化各link->buffer[0]至[size-1]=0.0
{
	PTLink *link;
	if (size <=0)
	{
		cout<<"\tInvalid size of link buffer array"<<endl;
		return 1;
	}
	else
	{
		linkBufferSize = size;
	}

	for (int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		if(link->bufferData) delete [] link->bufferData;
		link->bufferData = new floatType[size];
		if(link->bufferData == NULL)
		{
			cout<<"\tCannot allocate memory for link buffer!"<<endl;
			return 1;
		}
		for (int j = 0;j<size;j++)
			link->bufferData[j] = 0.0;
	}
	return 0;

}

int	PTNET::AllocateLinkBuffer(int size) //令linkBufferSize = size；同时对于每一条link设置size个buffer，初始化各link->buffer[0]至[size-1]=0.0
{
	PTLink *link;
	if (size <=0)
	{
		cout<<"\tInvalid size of link buffer array"<<endl;
		return 1;
	}
	else
	{
		linkBufferSize = size;
	}

	for (int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		if(link->buffer) delete [] link->buffer;
		link->buffer = new floatType[size];
		if(link->buffer == NULL)
		{
			cout<<"\tCannot allocate memory for link buffer!"<<endl;
			return 1;
		}
		for (int j = 0;j<size;j++)
			link->buffer[j] = 0.0;
	}
	return 0;

}

int	PTNET::AllocateNetBuffer(int size)
{
	if (size <=0)
	{
		cout<<"\tInvalid size of network buffer array"<<endl;
		return 1;
	}

	//if (buffer) delete [] buffer;
	buffer = new floatType[size];
	if(buffer == NULL)
	{
		cout<<"\tCannot allocate memory for network buffer!"<<endl;
		return 1;
	}
	for (int j = 0;j<size;j++) buffer[j] = 0.0;
	return 0;


}

int	PTNET::AllocateNodeBuffer(int size)
{
	PTNode *node;
	if (size <=0)
	{
		cout<<"\tInvalid size of network buffer array"<<endl;
		return 1;
	}
	else
	{
		nodeBufferSize = size;
	}

	for (int i= 0;i<numOfNode;i++)
	{
		node = nodeVector[i];
		if(node->buffer) delete [] node->buffer;
		node->buffer = new floatType[size];
		if(node->buffer == NULL)
		{
			cout<<"\tCannot allocate memory for link buffer!"<<endl;
			return 1;
		}
		for (int j = 0;j<size;j++)
			node->buffer[j] = 0.0;

	}
	return 0;
}




void PTNET::UpatePTNetworkLinkCost() //利用公式计算link cost和link cost一阶导
{
	PTLink *link,*rlink;
	for(int i = 0;i<numOfLink;i++)
	{
		link =  linkVector[i];


		link->UpdatePTLinkCost(); //更新link cost
		link->UpdatePTDerLinkCost(); //更新link cost一阶导
		if (link->rLink && link->GetTransitLinkSymmetry()==PTLink::ASYMMTRIC) //if该弧为上车弧（或行驶弧），更新对应行驶弧（或上车弧）的cost和cost一阶导
		{
			link->rLink->UpdatePTLinkCost(); //更新rlink cost
			link->rLink->UpdatePTDerLinkCost(); //更新rlink cost一阶导
		}			
	}
}



floatType	PTLink::GetPTLinkCost()
{
	floatType tc = 0.0, tmpcost;
	tmpcost = cost;
	UpdatePTLinkCost();
	tc = cost;
	cost = tmpcost;
	//switch(GetTransitLinkType())
	//       {
	//		case PTLink::WALK:
	//			double alpha_1,wt;
	//			alpha_1=pars[0];
	//			wt=	pars[1];
	//			tc = alpha_1 * wt;
	//			break;
	//		case PTLink::ABOARD:
	//			double alpha_2,beta_2,n1,cap1;
	//			alpha_2	=	pars[0];
	//			beta_2	=	pars[1];
	//			n1		=	pars[2];//power
	//			cap1		=	cap;
	//			tc = alpha_2 * pow (((1 - beta_2) * volume + beta_2 * rLink ->volume) / cap1 , n1);
	//			break;
	//		case PTLink::ALIGHT:
	//			double alpha_4,tloss;
	//			alpha_4=	pars[0];
	//			tloss =		pars[1];
	//			tc  = alpha_4 * tloss;
	//			break;
	//		case PTLink::ENROUTE:
	//			double alpha_3,beta_3,gamma_3,n2,cap2,fft;
	//			alpha_3	=	pars[0];
	//			beta_3	=	pars[1];
	//			gamma_3	=	pars[2];
	//			n2		=	pars[3];//power
	//			cap2	=	cap;
	//			fft		=	pars[5];
	//			tc = alpha_3 * fft + beta_3 * pow ((volume + (gamma_3 - 1) * rLink ->volume) / cap2, n2);
	//			break;
	//       }

	return tc;


}

floatType	PTLink::GetPTLinkDerCost()
{
	floatType tc = 0.0, tmpcost;
	tmpcost = pfdcost;
	UpdatePTDerLinkCost();
	tc = pfdcost;
	pfdcost = tmpcost;

	//switch(GetTransitLinkType())
	//{
	//	case PTLink::WALK:
	//		tc = 0.0;
	//		break;
	//	case PTLink::ABOARD:
	//		double alpha_2,beta_2,n1,cap1;
	//		alpha_2	=	pars[0];
	//		beta_2	=	pars[1];
	//		n1		=	pars[2];//power
	//		cap1		=	cap;
	//		tc = alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink ->volume) / cap1 , n1 - 1) * ( (1 - beta_2) / cap1);
	//		break;
	//	case PTLink::ALIGHT:
	//		tc = 0.0;
	//		break;
	//	case PTLink::ENROUTE:
	//		double alpha_3,beta_3,gamma_3,n2,cap2,fft;
	//		alpha_3	=	pars[0];
	//		beta_3	=	pars[1];
	//		gamma_3	=	pars[2];
	//		n2		=	pars[3];//power
	//		cap2	=	cap;
	//		fft		=	pars[5];
	//		tc  =   beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink ->volume) / cap2, n2 - 1) * (1 / cap2);
	//		break;
	//}

	return tc;


}




void PTLink::UpdatePTLinkCost()
{
	//floatType tvol,rtvol = 0.0;
	//if(volume<0.0) 
	//{
	//	tvol = 0.0;
	//	if (fabs(volume)>1e-10)
	//	{
	//		cout<<"link id :"<<id<<" have negative flow:"<<volume<<endl;
	//	}
	//	else
	//	{
	//		volume = 0.0;
	//	}
	//}
	//else tvol = volume;
	//if (rLink)
	//{
	//	if (rLink->volume<0.0) 
	//	{
	//		//cout<<"link id :"<<rLink->id<<" have negative flow:"<<rLink->volume<<endl;
	//		rtvol = 0.0;
	//		if (fabs(rLink->volume)>1e-10)
	//		{
	//			cout<<"link id :"<<rLink->id<<" have negative flow:"<<rLink->volume<<endl;
	//		}
	//		else
	//		{
	//			rLink->volume = 0.0;
	//		}
	//	}
	//	else  rtvol = rLink ->volume;
	//}

	if (volume<0&&volume>-1e-10) volume = 0.0;
	if (volume<-1e-10)
	{
		cout<<"link id :"<<id<<" have negative flow:"<<volume<<endl;
	}
	if (rLink)
	{
		if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;
		if (rLink->volume<-1e-10)
		{
			cout<<"link id :"<<rLink->id<<" have negative flow:"<<volume<<endl;
		}

	}


	switch(GetTransitLinkType())
	{
	case PTLink::FAILWALK: //add for failing walk link
		double alpha_11,wt1;
		alpha_11=pars[0];
		wt1=	pars[1];
		cost = alpha_11 * wt1;
		break;
	case PTLink::WALK:
		double alpha_1,wt;
		alpha_1=pars[0];
		wt=	pars[1];
		cost = alpha_1 * wt;
		break;
	case PTLink::ABOARD:
		double alpha_2,beta_2,n1,cap1;
		alpha_2	=	pars[0];
		beta_2	=	pars[1];
		n1		=	pars[2];//power
		cap1		=	cap;				
		//cost = alpha_2 * pow (((1 - beta_2) * volume + beta_2 * rLink ->volume) / cap1 , n1);

		if(m_tlSym==PTLink::ASYMMTRIC)
		{
			//if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;
			cost = alpha_2 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1);
		}
		else
		{
			cost = alpha_2 * pow ( volume / cap1 , n1);
		}
		break;
	case PTLink::ALIGHT:
		double alpha_4,tloss;
		alpha_4=	pars[0];
		tloss =		pars[1];
		cost  = alpha_4 * tloss;
		break;
	case PTLink::ENROUTE:
		double alpha_3,beta_3,gamma_3,n2,cap2,fft;
		alpha_3	=	pars[0];
		beta_3	=	pars[1];
		gamma_3	=	pars[2];
		n2		=	pars[3];//power
		cap2	=	cap;
		fft		=	pars[5];

		if(m_tlSym==PTLink::ASYMMTRIC)
		{
			if (rLink->volume<0&&rLink->volume>-1e-10) rLink->volume = 0.0;
			cost = alpha_3 * fft + beta_3 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2);
		}
		else
		{
			cost =  alpha_3 * fft + beta_3 * pow (volume/ cap2, n2);
		}
		//cout<<alpha_3<<","<<fft<<","<<beta_3<<","<<gamma_3<<","<<cap2<<endl;
		//cost = alpha_3 * fft + beta_3 * pow ((volume + (gamma_3 - 1) * rLink ->volume) / cap2, n2);

		break;
	}
	if (fabs(cost)<1e-10) cost = 0.0;


}

void PTLink::UpdatePTDerLinkCost()
{
	//floatType tvol,rtvol = 0.0;
	//if(volume<0.0) tvol = 0.0;
	//else tvol = volume;
	////cout<<rtvol<<endl;
	//if (rLink)
	//{
	//	if (rLink->volume<0.0) rtvol = 0.0;
	//	else  rtvol = rLink ->volume;
	//}

	switch(GetTransitLinkType())
	{
	case PTLink::FAILWALK:
		pfdcost = 0.0;
		break;
	case PTLink::WALK:
		pfdcost = 0.0;
		//rpfdcost = 0.0;
		break;
	case PTLink::ABOARD: //link=上车弧(i1,j1) rlink=节点j1出发行驶弧(j1,j2)
		double alpha_2,beta_2,n1,cap1;
		alpha_2	=	pars[0];
		beta_2	=	pars[1];
		n1		=	pars[2];//power
		cap1	=	cap;
		if(m_tlSym==PTLink::ASYMMTRIC)
		{
			pfdcost = alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1 - 1) * ( (1 - beta_2) / cap1); //上车弧cost_func对上车弧link流量v(i1,j1)一阶导
			rpfdcost= alpha_2 * n1 * pow (((1 - beta_2) * volume + beta_2 * rLink->volume) / cap1 , n1 - 1) * (beta_2 / cap1);        //上车弧cost_func对下一段行驶弧rlink流量v(j1,j2)一阶导
		}
		else
		{
			pfdcost =  alpha_2 * n1 * pow( volume / cap1 , n1-1) * (1/cap1);
			rpfdcost = 0.0;
		}
		break;
	case PTLink::ALIGHT:
		pfdcost = 0.0;
		//rpfdcost = 0.0;
		break;
	case PTLink::ENROUTE: //link=行驶弧(j1,j2) rlink=节点j1上车弧(i1,j1)
		double alpha_3,beta_3,gamma_3,n2,cap2,fft;
		alpha_3	=	pars[0];
		beta_3	=	pars[1];
		gamma_3	=	pars[2];
		n2		=	pars[3];//power
		cap2	=	cap;
		fft		=	pars[5];
		if(m_tlSym==PTLink::ASYMMTRIC)
		{
			pfdcost  = beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2 - 1) * (1 / cap2);   //行驶弧cost_func对行驶弧link流量v(j1,j2)一阶导
			rpfdcost = beta_3 * n2 * pow ((volume + (gamma_3 - 1) * rLink->volume) / cap2, n2 - 1) * ((gamma_3 - 1) / cap2); //行驶弧cost_func对节点j1上车弧rlink流量v(i1,j1)一阶导
		}
		else
		{
			pfdcost  =  beta_3 * n2 * pow(volume/ cap2,n2-1)*(1/cap2);
			rpfdcost = 0.0;
		}
		break;
	}
}
void PTNET::SetLinksAttribute(bool isSym)
{
	for(int i=0;i<numOfLink;i++)
	{
		PTLink* link = linkVector[i];
		if (isSym)
			link->SetTransitLinkSym(PTLink::SYMMTRIC);
		else
			link->SetTransitLinkSym(PTLink::ASYMMTRIC);//link->m_tlSym = ASYMMTRIC(1) //设置路段属性 非对称
		if (PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_GP_TL || PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL || PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_GFW_TL || PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL)
			link->SetCapType(PTLink::TL); //考虑transit link容量约束，设置link->m_tlCap=TL(0)
	}

}
void PTNET::ConnectAsymmetricLinks() //记录各条上车弧（或行驶弧）所在公交线路对应的行驶弧（或上车弧）link->rLink
{
	PTLink *link,*rlink;
	string sid;
	PTNode* node;
	for(int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		rlink=NULL;

		switch(link->GetTransitLinkType())
		{
		case PTLink::ABOARD: //上车弧
			sid = link->m_shape->m_id; //该上车弧所在的公交线路sid
			node=link->head->GetStopPtr()->GetNode(sid+";ENROUTE"); //上车弧头节点head在本公交线路sid对应的行驶弧尾节点node
			for (vector<PTLink*>::iterator it=node->forwStar.begin();it!=node->forwStar.end();++it) //遍历node的前向弧，取出在该公交线路的行驶弧的rlink
			{
				if ((*it)->GetTransitLinkType() == PTLink::ENROUTE) rlink= (*it);			
			}			
			if (!rlink)
			{
				cout<<"shpid:"<<sid<<" have wrong connection without outgoing transit links!"<<endl;
				return;
			}
			else link->rLink = rlink; //记录该上车弧头节点（所在的公交线路）对应的一段行驶弧rlink
			break;
		case PTLink::ENROUTE: //行驶弧
			node =  link->tail->GetStopPtr()->GetTransferNode(); //该段行驶弧尾节点tail对应的上车弧的尾节点node
			for (vector<PTLink*>::iterator it=node->forwStar.begin();it!=node->forwStar.end();++it) //遍历node的前向弧，取出在该公交线路的上车弧的rlink
			{
				//cout<<sid<< (*it)->m_shape->m_id;
				if ( (*it)->m_shape == link->m_shape) rlink= (*it);		
				//rlink = link->m_shape->m_boardlinks[link->seq];
			}
			if (!rlink)
			{
				cout<<"shpid:"<<sid<<" have no outgoing incoming board links!"<<endl;
				return;
			}
			else link->rLink = rlink; //记录该行驶弧尾节点（所在的公交线路）对应的一段上车弧rlink


			break;
		}
	}



}

int	PTNode::NumOfBushOutLink()
{
	int n =0;
	for(PTRTRACE pv = forwStar.begin(); pv!= forwStar.end(); pv++)
	{
		if((*pv)->m_stglink) n++;
	}
	return n;


}
int	PTNode::NumOfDestOutLink() //记录当前节点属于最短路上的出发弧数量
{
	int n =0;
	for(PTRTRACE pv = forwStar.begin(); pv!= forwStar.end(); pv++)
	{
		if((*pv)->markStatus > 0) n++;
	}
	return n;
}

void	PTNET::ComputeConvGap()
{
	floatType gap, tt =0.0, tmd = 0.0, minimumTcost = 0.0, assflowdiff = 0.0; //初始化
	numaOfHyperpath = 0;
	netTTcost = 0.0;
	PTDestination* dest;
	PTOrg* org;
	PTLink *link;
	for (int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		tt += link->volume * link->cost; //累计所有路段总费用（路段流量*路段费用之和）
	}

	if (PCTAE_ALG == PCTAE_algorithm::PCTAE_A_SD) //store the extreme link flow when implementing SPP
	{
		SCnetTTwaitcost = 0.0;
		for (int i = 0;i<numOfLink;i++)
		{
			link = linkVector[i];
			link->volume = 0.0;
		}
	}

	if (PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iGreedy || PCTAE_ALG == PCTAE_algorithm::PCTAE_P_Greedy	||
		PCTAE_ALG == PCTAE_algorithm::PCTAE_P_NGP || PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iNGP ||	//path-based alg, recompute netTTwaitcost 
		PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL||
		PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL
		)
	{
		netTTwaitcost = 0;
	}

	for (int i = 0;i<numOfPTDest;i++)
	{   //遍历各目的地dest
		dest = PTDestVector[i]; 
		//cout<<dest->destination->m_stop->m_id<<endl; 
		InitializeHyperpathLS(dest->destination);  //计算终点dest下的最短超路径
		for (int j=0; j<dest->numOfOrg; j++)
		{   //对目的地dest的每一个起点org (org,dest)
			org= dest->orgVector[j];  
			//cout<<org->org->m_stop->m_id<<endl; 
			minimumTcost += org->org->StgElem->cost * org->assDemand; //minimumTcost累计所有OD对的最小总成本 cost*ODdemand

			if (PCTAE_ALG == PCTAE_algorithm::PCTAE_A_SD)
			{
				TNM_HyperPath* path = new TNM_HyperPath();
				if (path->InitializeHP(org->org,dest->destination))
				{
					double dmd = org->assDemand;
					SCnetTTwaitcost += path->WaitCost * dmd;
					vector<GLINK*> glinks=path->GetGlinks();
					for(int i=0;i<glinks.size();++i)
					{
						glinks[i]->m_linkPtr->volume += glinks[i]->m_data * dmd;
					}
				}
			}

			if (PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iGreedy || PCTAE_ALG == PCTAE_algorithm::PCTAE_P_Greedy	||
				PCTAE_ALG == PCTAE_algorithm::PCTAE_P_NGP || PCTAE_ALG == PCTAE_algorithm::PCTAE_P_iNGP	||	//path-based alg, recompute netTTwaitcost
				PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_GP_TL || PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL||
				PCTAE_ALG == PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL
				)
			{
				int k = 0;
				floatType rdemand = 0.0;
				for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();)  //遍历该od对路径集pathSet的每一条路径
				{
					TNM_HyperPath* path = *it;
					rdemand += path->flow; //累计超路径集中各条超路径流量之和rdemand
					if (path->flow == 0.0) // column dropping //if 该路径流量为0，在超路径集中删除该路径
					{
						it = org->pathSet.erase(it);			
					}
					else
					{			
						netTTwaitcost += path->WaitCost * path->flow;	//if 该路径流量不为0，计算累计的路径等待时间*路径流量之和netTTwaitcost
						it++;
						k++;
					}
				}
				assflowdiff += fabs(org->assDemand - rdemand); //计算 OD对间需求 与 累计超路径集中各条超路径流量之和rdemand 的差值 assflowdiff
				numaOfHyperpath += k;           //计算 该OD对超路径集中的超路径数量numaOfHyperpath
				org->currentRelativeGap = 1.1;  //令org对应的currentRelativeGap=1.1
			}
		}
	}

	tt += netTTwaitcost; //令tt= （累计所有路段的流量*路段费用之和）+（累计所有路径的流量*路径等待时间之和）
	netTTcost = tt;
	if (PCTAE_ALG == PCTAE_algorithm::PCTAE_A_SD) 
		netTTwaitcost = SCnetTTwaitcost;
	//cout<<tt<<","<<minimumTcost<<endl;
	//cout<<"assflowdiff:"<<assflowdiff<<endl;
	gap = tt - minimumTcost; //计算gap = （累计所有路段的流量*路段费用之和 + 累计所有路径的流量*路径等待时间之和）- 累计所有OD对的最小总成本
	GapIndicator = fabs(gap);      //令GapIndicator = fabs(gap)
	RGapIndicator = fabs(gap/tt);  //令RGapIndicator = fabs(gap/tt)
}

void	PTNET::ComputeNetWait()
{
	floatType gap, tt =0.0, tmd = 0.0, minimumTcost = 0.0, assflowdiff = 0.0; //初始化
	numaOfHyperpath = 0;
	netTTcost = 0.0;
	PTDestination* dest;
	PTOrg* org;
	PTLink *link;
	for (int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		tt += link->volume * link->cost; //累计所有路段总费用（路段流量*路段费用之和）
	}
	netTTwaitcost = 0;
	for (int i = 0;i<numOfPTDest;i++)
	{   //遍历各目的地dest
		dest = PTDestVector[i]; 
		//cout<<dest->destination->m_stop->m_id<<endl; 
		InitializeHyperpathLS(dest->destination);  //计算终点dest下的最短超路径
		for (int j=0; j<dest->numOfOrg; j++)
		{   //对目的地dest的每一个起点org (org,dest)
			org= dest->orgVector[j];  
			//cout<<org->org->m_stop->m_id<<endl; 
			minimumTcost += org->org->StgElem->cost * org->assDemand; //minimumTcost累计所有OD对的最小总成本 cost*ODdemand

			int k = 0;
			floatType rdemand = 0.0;
			for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();)  //遍历该od对路径集pathSet的每一条路径
			{
				TNM_HyperPath* path = *it;
				rdemand += path->flow; //累计超路径集中各条超路径流量之和rdemand
				if (path->flow == 0.0) // column dropping //if 该路径流量为0，在超路径集中删除该路径
				{
					it = org->pathSet.erase(it);			
				}
				else
				{			
					netTTwaitcost += path->WaitCost * path->flow;	//if 该路径流量不为0，计算累计的路径等待时间*路径流量之和netTTwaitcost
					it++;
					k++;
				}
			}
			assflowdiff += fabs(org->assDemand - rdemand); //计算 OD对间需求 与 累计超路径集中各条超路径流量之和rdemand 的差值 assflowdiff
			numaOfHyperpath += k;           //计算 该OD对超路径集中的超路径数量numaOfHyperpath
			org->currentRelativeGap = 1.1;  //令org对应的currentRelativeGap=1.1	
		}
	}
	tt += netTTwaitcost; //令tt= （累计所有路段的流量*路段费用之和）+（累计所有路径的流量*路径等待时间之和）
	netTTcost = tt;
}

void	PTNET::RecordTEAPCurrentIter()
{
	PTITERELEM *iterElem = new PTITERELEM;
	iterElem->iter  = curIter;          //迭代次数
	iterElem->convGap  = GapIndicator;  //收敛指标（差值）
	iterElem->convRGap = RGapIndicator; //收敛指标（比值）
	iterElem->time  = 1.0*(clock() - m_startRunTime)/CLOCKS_PER_SEC; //大循环用时
	iterElem->mainlooptime = IterMainlooptime ;    //主循环用时
	iterElem->innerlooptime = IterInnerlooptime;   //内循环用时
	iterElem->numberofhyperpaths=numaOfHyperpath;  //超路径集中的路径数量
	iterElem ->innerIters =InnerIters;  //内循环迭代次数
	iterRecord.push_back(iterElem);  //在iterRecord插入本次循环的参数
	//return iterElem;
}

string PTNET::GetAlgorithmName()
{
	string alg = "";
	switch(PCTAE_ALG)
	{
	case PCTAE_algorithm::PCTAE_A_GFW:
		alg = "fw";
		break;
	case PCTAE_algorithm::PCTAE_P_Greedy:
		alg = "greedy";
		break;
	case PCTAE_algorithm::PCTAE_P_NGP:
		alg = "gp";
		break;
	case PCTAE_algorithm::PCTAE_A_SD:
		alg = "sd";
		break;
	case PCTAE_algorithm::PCTAE_P_iGreedy:
		alg = "igreedy";
		break;
	case PCTAE_algorithm::PCTAE_P_iNGP:
		alg = "igp";
		break;
	case PCTAE_algorithm::CAP_PCTAE_P_GP_TL:
		alg = "c_gp_tl";
		break;
	case PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL:
		alg = "c_greedy_tl";
		break;
	case PCTAE_algorithm::CAP_PCTAE_P_GFW_TL:
		alg = "c_gfw_tl";
		break;
	case PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL:
		alg = "c_ipf_gp_tl";
		break;
	default:
		cout << "No alg defined in the solver, please check solve!" << endl;
		return "";
		break;
	}
	return alg;
}

void PTNET::ReportIter()
{
	string IterConvName  = networkName +"-" + GetAlgorithmName() + ".conv";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, IterConvName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .conv file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting iterative information into "<<IterConvName<<" for covergence info!"<<endl;
	//outfile<<"Iteration,Relative Gap,Gap,Time,Mainlooptime,IterInnerlooptime,inneriters,numofhyperpath"<<endl;
	//outfile<<"0,1,,0,,,,"<<endl;
	outfile<<"Iteration,SubRG,solutiongap,Time,Mainlooptime,IterInnerlooptime,inneriters,numofhyperpath,infeasibleflow,multiplierconv,lambda"<<endl;
	outfile<<"0,1,,0,,,,,,,"<<endl;
	for(int i=0;i<iterRecord.size();i++)
	{
		PTITERELEM* it = iterRecord[i];
		//outfile<<TNM_IntFormat(it->iter)<<","<<TNM_FloatFormat(it->convRGap,20,18)<<","<<TNM_FloatFormat(it->convGap,20,18)<<","<<TNM_FloatFormat(it->time,6,3)<<","<<TNM_FloatFormat(it->mainlooptime,6,3)<<","<<TNM_FloatFormat(it->innerlooptime,6,3)<<","<<TNM_IntFormat(it->innerIters)<<","<<TNM_IntFormat(it->numberofhyperpaths)<<endl;
		outfile<<TNM_IntFormat(it->iter)<<","<<TNM_FloatFormat(it->convRGap,20,18)<<","<<TNM_FloatFormat(it->convGap,20,18)<<","<<TNM_FloatFormat(it->time,6,3)<<","<<TNM_FloatFormat(it->mainlooptime,6,3)<<","<<TNM_FloatFormat(it->innerlooptime,6,3)<<","<<TNM_IntFormat(it->innerIters)<<","<<TNM_IntFormat(it->numberofhyperpaths)<<","
				<<TNM_FloatFormat(it->convIF,20,18)<<","<<TNM_FloatFormat(it->multiplierconv,20,18)<<","<<TNM_FloatFormat(it->penalty,20,18)<<endl;
		//convGap  = SN;  convRGap = SubRG;  convIF = IF;  multiplierconv = GAP;  penalty = cb_lambda;
	}
	outfile.close();

}

void PTNET::ReportIterIPF()
{
	string IterConvName  = networkName +"-" + GetAlgorithmName() + ".conv";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, IterConvName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .conv file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting iterative information into "<<IterConvName<<" for covergence info!"<<endl;
	//outfile<<"Iteration,Relative Gap,Gap,Time,Mainlooptime,IterInnerlooptime,inneriters,numofhyperpath"<<endl;
	//outfile<<"0,1,,0,,,,"<<endl;
	outfile<<"Iteration,RG,PenaltyGap,Time,inneriters,numofhyperpath,infeasibleflow,gamma,epsl"<<endl;
	outfile<<"0,,,,,,,,,"<<endl;
	for(int i=0;i<iterRecord.size();i++)
	{
		PTITERELEM* it = iterRecord[i];
		//outfile<<TNM_IntFormat(it->iter)<<","<<TNM_FloatFormat(it->convRGap,20,18)<<","<<TNM_FloatFormat(it->convGap,20,18)<<","<<TNM_FloatFormat(it->time,6,3)<<","<<TNM_FloatFormat(it->mainlooptime,6,3)<<","<<TNM_FloatFormat(it->innerlooptime,6,3)<<","<<TNM_IntFormat(it->innerIters)<<","<<TNM_IntFormat(it->numberofhyperpaths)<<endl;
		outfile<<TNM_IntFormat(it->iter)<<","<<TNM_FloatFormat(it->convRGap,20,18)<<","<<TNM_FloatFormat(it->multiplierconv,20,18)<<","<<TNM_FloatFormat(it->time,6,3)<<","<<TNM_IntFormat(it->innerIters)<<","<<TNM_IntFormat(it->numberofhyperpaths)<<","<<TNM_FloatFormat(it->convIF,20,18)<<","<<TNM_FloatFormat(it->penalty,20,18)<<","<<TNM_FloatFormat(it->epsi,20,18)<<endl;
		//it->multiplierconv = PenaltyGap
	}
	outfile.close();

}

void PTNET::ReportAvgpaxinfo() //Generalized cost (min) ；Number of transfers ；Waiting time (min) ；In-vehicle time (min) ；Walking time (min) 
{
	//PTLink *link;

	floatType netTTwalkcost = 0.0, netTTinvehcost = 0.0, netTTtransfer = 0.0, netTTgcost=0.0, netTTmu=0.0, netTTaboardcost =0.0, netTTalightcost =0.0;
	cout<<"------平均信息输出-------"<<endl;
	//等待时间
	//netTTgcost = netTTwaitcost;
	for(int i=0;i<numOfLink;i++)
	{
		//算（广义）时间和乘子
		if (PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_GP_TL || PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL)	
		{   
			netTTgcost += linkVector[i]->g_cost * linkVector[i]->volume;
			netTTmu += linkVector[i]->mu * linkVector[i]->volume;
		}
		else
		{
			netTTgcost += linkVector[i]->cost * linkVector[i]->volume;
		}
		
		
		//算车内时间 步行时间
		PTLink *link= linkVector[i];
		if (PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_GP_TL || PCTAE_ALG==PTNET::PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL)	
		{
			switch(link->GetTransitLinkType())
				{
					case PTLink::WALK:
						netTTwalkcost += link->cost*link->volume;
						break;
					case PTLink::ENROUTE:
						netTTinvehcost += link->cost*link->volume;
						break;
					case PTLink::ABOARD:
						netTTaboardcost += link->cost*link->volume;
						break;
					case PTLink::ALIGHT:
						netTTalightcost += link->cost*link->volume;
						break;
				}
		}
		else
		{
			switch(link->GetTransitLinkType())
				{
					case PTLink::WALK:
						netTTwalkcost += link->cost*link->volume;
						break;
					case PTLink::ENROUTE:
						netTTinvehcost += link->cost*link->volume;
						break;
					case PTLink::ABOARD:
						netTTaboardcost += link->cost*link->volume;
						break;
					case PTLink::ALIGHT:
						netTTalightcost += link->cost*link->volume;
						break;
				}
		}
	}

	//cout<<netTTwaitcost<<","<<netTTcost<<","<<netTTgcost<<endl;

	//for(int i=0;i<numOfLink;i++)
	//{
	//	PTLink *link= linkVector[i];
	//	//link->UpdatePTLinkCost();
	//	//cout<<"linkid:"<<link->id<<"("<<link->GetTransitLinkTypeName()<<")"<<" flow:"<<link->volume<<", cost:"<<link->cost<<endl;
	//	switch(link->GetTransitLinkType())
	//	{
	//		case PTLink::WALK:
	//			netTTwalkcost += link->cost*link->volume;
	//			break;
	//		case PTLink::ENROUTE:
	//			netTTinvehcost += link->g_cost*link->volume;
	//			break;
	//	}
	//}
	PTDestination* dest;
	PTOrg* org;
	//由超路径计算广义费用
	floatType netTTpathgcost =0.0;
	for (int i = 0;i<numOfPTDest;i++)
	{
		dest = PTDestVector[i];
		for (int j=0; j<dest->numOfOrg; j++) //OD对
		{
			org= dest->orgVector[j];
			for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();it++)  //超路径集中的超路径
			{
				TNM_HyperPath* path = *it;
				//cout<<path->cost<<"*"<<path->flow<<"="<<path->cost*path->flow<<endl;
				netTTpathgcost += path->cost*path->flow;
			}
		}
	}
	//cout<<"TTwaitcost:"<<netTTwaitcost<<endl;
	//cout<<"TTlinkcost:"<<netTTgcost<<endl;
	//cout<<"TTpathcost:"<<netTTpathgcost<<endl; //=netTTwaitcost+netTTgcost

	//计算换乘
	for (int i = 0;i<numOfPTDest;i++)
	{
		dest = PTDestVector[i];
		for (int j=0; j<dest->numOfOrg; j++) //OD对
		{
			floatType odTTtransfer = 0.0;
			org= dest->orgVector[j];
			for (vector<TNM_HyperPath*>::iterator it = org->pathSet.begin(); it!=org->pathSet.end();it++)  //超路径集中的超路径
			{
				TNM_HyperPath* path = *it; 
				floatType pathTTtransfer = 0.0;
				//cout<<"------hyperpath flow:"<<path->flow<<endl;
				vector<GLINK*> glinks= path->GetGlinks();
				for(int k=0;k<glinks.size();k++)
				{
					glinks[k]->m_linkPtr->volumet = 0.0;
				}
				double dmd = org->assDemand;//total demand.	
				for(int k=0;k<glinks.size();k++)
				{
					glinks[k]->m_linkPtr->volumet += glinks[k]->m_data * path->flow;
					//cout<<"弧"<<glinks[k]->m_linkPtr->id<<", 概率："<<glinks[k]->m_data<<", 流量："<<glinks[k]->m_linkPtr->volumet<<endl;
				}
				for(int k=0;k<glinks.size();k++)  //超路径中的各段弧
				{
					switch(glinks[k]->m_linkPtr->GetTransitLinkType())
					{
						case PTLink::ABOARD:
							//cout<<"头节点和起点："<<glinks[k]->m_linkPtr->tail<<","<<org->org<<endl;
							//cout<<"弧流量："<<glinks[k]->m_linkPtr->volumet<<",  弧概率："<<glinks[k]->m_data<<endl;
							if (glinks[k]->m_linkPtr->tail != org->org)
							{
								pathTTtransfer += glinks[k]->m_linkPtr->volumet;
								//cout<<"弧流量："<<glinks[k]->m_linkPtr->volumet<<",  弧概率："<<glinks[k]->m_data<<", 换乘次数："<<glinks[k]->m_linkPtr->volumet<<endl;
							}
							break;
					}
				}
				//cout<<"这条超路径的换乘次数："<<pathTTtransfer<<", 乘客人数"<<path->flow<<endl;
				odTTtransfer += pathTTtransfer;
			}
			//cout<<"OD对("<<org->org->id<<","<<dest->destination->id<<") 总换乘次数："<<odTTtransfer<<endl;
			netTTtransfer += odTTtransfer;
			//cout<<netTTtransfer<<endl;
		}
			
	}
	//cout<<netTTwalkcost<<endl;
	//cout<<netTTinvehcost<<endl;
	//cout<<netTTtransfer<<endl;
	//cout<<"trips:"<<numOfPTTrips<<endl;	
	//cout<<"total netcost:"<<netTTpathgcost<<",   avg netcost:"<<netTTpathgcost/numOfPTTrips<<endl; //net total cost
	//cout<<"total linkgcost:"<<netTTgcost<<",   avg gcost:"<<netTTgcost/numOfPTTrips<<endl;  //link general cost(not include waitcost)
	//cout<<"total waitcost:"<<netTTwaitcost<<",   avg wcost:"<<netTTwaitcost/numOfPTTrips<<endl; //wait cost
	//cout<<"total penalty:"<<netTTmu<<",   avg penalty:"<<netTTmu/numOfPTTrips<<endl;
	//cout<<"total transfer number:"<<netTTtransfer<<",   avg transfer num:"<<netTTtransfer/numOfPTTrips<<endl;
	////cout<<"total waiting time:"<<netTTwaitcost<<",   avg waiting time:"<<netTTwaitcost/numOfPTTrips<<endl;
	//cout<<"total in-vehicle time:"<<netTTinvehcost<<",   avg in-veh time:"<<netTTinvehcost/numOfPTTrips<<endl;
	//cout<<"total walking time:"<<netTTwalkcost<<",   avg walking time:"<<netTTwalkcost/numOfPTTrips<<endl;
	//cout<<"-------------------------------------"<<endl;
	
	cout<<"TTtrips:"<<numOfPTTrips<<endl;	
	cout<<"avgNetcost:"<<netTTpathgcost/numOfPTTrips<<endl; //net total cost
	//cout<<netTTgcost/numOfPTTrips<<endl;  //link general cost(not include waitcost)
	cout<<"avgMu:"<<netTTmu/numOfPTTrips<<endl;
	cout<<"avgWaitcost:"<<netTTwaitcost/numOfPTTrips<<endl; //wait cost
	//cout<<netTTtransfer/numOfPTTrips<<endl;
	//cout<<"total waiting time:"<<netTTwaitcost<<",   avg waiting time:"<<netTTwaitcost/numOfPTTrips<<endl;
	cout<<"avgInvehcost:"<<netTTinvehcost/numOfPTTrips<<endl;
	cout<<"avgWalkcost:"<<netTTwalkcost/numOfPTTrips<<endl;
	cout<<"avgAboardcost:"<<netTTaboardcost/numOfPTTrips<<endl;
	cout<<"avgAlightcost:"<<netTTalightcost/numOfPTTrips<<endl;
	cout<<"-------------------------------------"<<endl;


	return;

}

void PTNET::ReportPTlinkflow_IPFCTEAPIni()
{
	string fileName  = networkName +"-" + GetAlgorithmName() + ".Initiallinkinfo";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .linkinfo file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting PT link information into "<<fileName<<" linkinfo!"<<endl;

	PTLink *link;
	outfile<<"link id (link type),link flow,link cost,fcost,capacity"<<endl; 
	for(int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		outfile<<link->id<<"("<<link->GetTransitLinkTypeName()<<")"<<","<<link->volume<<","<<link->cost<<","<<link->pfdcost<<",";
		//outfile<<TNM_IntFormat(link->id)<<","<<TNM_FloatFormat(link->volume,6,3)<<","<<link->cost<<","<<link->pfdcost<<",";
		if (link->GetTransitLinkType()==PTLink::ABOARD) //仅对上车弧和行驶弧有容量约束的可能
		{
			double cap = link->pars[link->pars.size() - 1] ;
			outfile<<cap<<endl;
			//outfile<<TNM_FloatFormat(cap,15,10)<<endl;
		}
		else if ( link->GetTransitLinkType()==PTLink::ENROUTE)
		{
			double cap = link->pars[link->pars.size() - 2] ;
			outfile<<cap<<endl;
			//outfile<<TNM_FloatFormat(cap,15,10)<<endl;
		}
		else
		{
			outfile<<"-"<<endl;
		}
	}
	outfile<<"netTTwaitcost:"<<netTTwaitcost<<endl;
	outfile.close();
}

void PTNET::ReportPTlinkflowCapIPF()
{
	string fileName  = networkName +"-" + GetAlgorithmName() + ".linkinfo";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .linkinfo file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting PT link information into "<<fileName<<" linkinfo!"<<endl;

	PTLink *link;
	outfile<<"link id (link type), flow, traveltime, delaytime, cost, fcost, psecap, realcap, "<<endl; 
	for(int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		outfile<<link->id<<"("<<link->GetTransitLinkTypeName()<<")"<<","<<link->volume<<","<<link->buffer[0]<<","<<link->buffer[4]<<","<<link->cost<<","<<link->pfdcost<<",";
		//outfile<<TNM_IntFormat(link->id)<<","<<TNM_FloatFormat(link->volume,6,3)<<","<<link->cost<<","<<link->pfdcost<<",";
		if (link->GetTransitLinkType()==PTLink::ABOARD) //仅对上车弧和行驶弧有容量约束的可能
		{
			double cap = link->pars[link->pars.size() - 1] ;
			outfile<<"-,"<<cap<<endl;
			//outfile<<TNM_FloatFormat(cap,15,10)<<endl;
		}
		else if ( link->GetTransitLinkType()==PTLink::ENROUTE)
		{
			double cap = link->pars[link->pars.size() - 2] ;
			outfile<<link->buffer[2]<<","<<cap<<endl;
			//outfile<<TNM_FloatFormat(cap,15,10)<<endl;
		}
		else
		{
			outfile<<"-,-"<<endl;
		}
	}



	outfile<<"netTTwaitcost:"<<netTTwaitcost<<endl;
	outfile.close();
}

void PTNET::ReportPTlinkflowCap()
{
	string fileName  = networkName +"-" + GetAlgorithmName() + ".linkinfo";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .linkinfo file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting PT link information into "<<fileName<<" linkinfo!"<<endl;

	PTLink *link;
	outfile<<"link id (link type),flow,delaytime,cost,gcost,fcost,capacity"<<endl; 
	for(int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		outfile<<link->id<<"("<<link->GetTransitLinkTypeName()<<")"<<","<<link->volume<<","<<link->mu<<","<<link->cost<<","<<link->g_cost<<","<<link->pfdcost<<",";
		//outfile<<TNM_IntFormat(link->id)<<","<<TNM_FloatFormat(link->volume,6,3)<<","<<link->cost<<","<<link->pfdcost<<",";
		if (link->GetTransitLinkType()==PTLink::ABOARD) //仅对上车弧和行驶弧有容量约束的可能
		{
			double cap = link->pars[link->pars.size() - 1] ;
			outfile<<cap<<endl;
			//outfile<<TNM_FloatFormat(cap,15,10)<<endl;
		}
		else if ( link->GetTransitLinkType()==PTLink::ENROUTE)
		{
			double cap = link->pars[link->pars.size() - 2] ;
			outfile<<cap<<endl;
			//outfile<<TNM_FloatFormat(cap,15,10)<<endl;
		}
		else
		{
			outfile<<"-"<<endl;
		}
	}
	outfile<<"netTTwaitcost:"<<netTTwaitcost<<endl;
	outfile.close();
}

void PTNET::ReportPTlinkflow()
{
	string fileName  = networkName +"-" + GetAlgorithmName() + ".linkinfo";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .linkinfo file to write Pas Origin Information!"<<endl;
		return;
	}
	cout<<"\tWriting PT link information into "<<fileName<<" linkinfo!"<<endl;

	PTLink *link;
	outfile<<"link id (link type),link flow,link cost,fcost,capacity"<<endl; 
	for(int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		outfile<<link->id<<"("<<link->GetTransitLinkTypeName()<<")"<<","<<link->volume<<","<<link->cost<<","<<link->pfdcost<<",";
		//outfile<<TNM_IntFormat(link->id)<<","<<TNM_FloatFormat(link->volume,6,3)<<","<<link->cost<<","<<link->pfdcost<<",";
		if (link->GetTransitLinkType()==PTLink::ABOARD) //仅对上车弧和行驶弧有容量约束的可能
		{
			double cap = link->pars[link->pars.size() - 1] ;
			outfile<<cap<<endl;
			//outfile<<TNM_FloatFormat(cap,15,10)<<endl;
		}
		else if ( link->GetTransitLinkType()==PTLink::ENROUTE)
		{
			double cap = link->pars[link->pars.size() - 2] ;
			outfile<<cap<<endl;
			//outfile<<TNM_FloatFormat(cap,15,10)<<endl;
		}
		else
		{
			outfile<<"-"<<endl;
		}
	}
	outfile<<"netTTwaitcost:"<<netTTwaitcost<<endl;
	outfile.close();
}

void PTNET::ReportLinkCap()
{
	string fileName  = networkName +"-" + GetAlgorithmName() + ".cap";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, fileName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .cap file to write  Information!"<<endl;
		return;
	}
	cout<<"\tWriting link capacity information into "<<fileName<<" linkinfo!"<<endl;

	PTLink *link;
	outfile<<"id,flow,capacity"<<endl;
	floatType avg = 0.0, n = 0.0;
	floatType positive_avg = 0.0, positive_n = 0.0;
	for(int i = 0;i<numOfLink;i++)
	{
		link = linkVector[i];
		switch(link->GetTransitLinkType())
		{
		case PTLink::ABOARD:
			double alpha_2,beta_2,n1,cap1;
			alpha_2	=	link->pars[0];
			beta_2	=	link->pars[1];
			n1		=	link->pars[2];//power
			cap1		=	link->pars[3];		
			outfile<<link->id<<","<<(1 - beta_2) * link->volume +  beta_2 *link->rLink->volume<<","<<cap1<<endl;
			avg += ((1 - beta_2) * link->volume +  beta_2 *link->rLink->volume)/cap1;
			n++;
			if (link->volume>0||link->rLink->volume>0)
			{
				positive_avg += ((1 - beta_2) * link->volume +  beta_2 *link->rLink->volume)/cap1;
				positive_n++;
			}

			break;

		case PTLink::ENROUTE:
			double alpha_3,beta_3,gamma_3,n2,cap2,fft;
			alpha_3	=	link->pars[0];
			beta_3	=	link->pars[1];
			gamma_3	=	link->pars[2];
			n2		=	link->pars[3];//power
			cap2	=	link->pars[4];
			fft		=	link->pars[5];
			outfile<<link->id<<","<< link->volume +  (gamma_3 - 1) *link->rLink->volume<<","<<cap2<<endl;
			avg += (link->volume +  (gamma_3 - 1) * link->rLink->volume)/cap2;
			n++;
			if (link->volume>0||link->rLink->volume>0)
			{
				positive_avg += (link->volume +  (gamma_3 - 1) * link->rLink->volume)/cap2;
				positive_n++;
			}
			break;
		}
	}
	outfile<<"Avg: V/C: "<<avg / n<<", Pos Avg:"<<positive_avg/positive_n<<endl;
	outfile.close();
}


void PTNET::ReportPTHyperpaths()
{
	string PathinfoName  = networkName +"-" + GetAlgorithmName() + ".pathinfo";
	ofstream outfile;
	if (!TNM_OpenOutFile(outfile, PathinfoName))
	{
		cout<<"\n\tFail to prepare report: Cannot open .pathinfo file to write hyperpath Information!"<<endl;
		return;
	}
	cout<<"\tWriting net hyperpath information into "<<PathinfoName<<" for covergence info!"<<endl;
	outfile<<"Origin,Destination,numberofpaths,(link sequence,path flow,path cost)"<<endl<<endl;

	int numberhp=0;
	int numofboardings=0;
	int numofboardinglines = 0;
	for (int i = 0;i<numOfPTDest;i++)
	{
		PTDestination* dest = PTDestVector[i];
		double sumRGap = 0.0;

		for (int j=0; j<dest->numOfOrg; j++)
		{
			PTOrg* org= dest->orgVector[j];
			//int n1 =0, n2 =0;
			//for (int k=0; k<org->pathSet.size();++k) 
			//{
			//	n1 += org->pathSet[k]->Boardings;
			//	n2 += org->pathSet[k]->TotalBoardlines;
			//}
			outfile<<"OD:"<<org->org->id<<","<<dest->destination->id<<":"<<org->pathSet.size()<<endl;
			//outfile<<"OD:"<<TNM_IntFormat(org->org->id)<<","<<TNM_IntFormat(dest->destination->id)<<", path size"<<TNM_IntFormat(org->pathSet.size())<<endl;
			numberhp += org->pathSet.size();
			for (int k=0; k<org->pathSet.size();++k)
			{
				TNM_HyperPath* path = org->pathSet[k];
				vector<GLINK*> glinks=path->GetGlinks();

				for(int ix=0;ix<glinks.size();++ix)
				{
					PTLink* plink=glinks[ix]->m_linkPtr;
					if (ix<glinks.size()-1) outfile<<plink->id<<",";
					else outfile<<plink->id;
					//outfile<<TNM_IntFormat(plink->id)<<",";
				}
				outfile<<":	path flow:"<<path->flow<<",path ttcost:"<<path->cost<<",path waitcost:"<<path->WaitCost<<endl;
				//outfile<<TNM_FloatFormat(path->flow,12,6)<<TNM_FloatFormat(path->cost,12,6)<<endl;
			}
			outfile<<endl;
		}
	}
	outfile<<"Number of O-Dpairs:"<<numOfPTOD<<",Total number of hyperpath:"<<numberhp<<endl;
	outfile.close();




}



void PTOrg::CTEAPIPFPathFlowConservation(floatType gamma) 
	//获取当前超路径集的最大流量(maxdemand)路径maxpath
	//计算OD对需求assDemand 与 当前计算出的总需求ttdemand 是否相等，若不相等（存在差值），则将流量差值均分在路径maxpath的各段弧之上，并更新当前路段费用和费用一阶导 
{
	floatType ttdemand = 0.0, maxdemand = 0.0;
	TNM_HyperPath* maxpath;
	for (vector<TNM_HyperPath*>::iterator it = pathSet.begin(); it!=pathSet.end();it++) 
	{
		TNM_HyperPath* path = *it;    //遍历OD对的超路径集pathSet
		ttdemand += path->flow;       //计算OD对之间总需求ttdemand、超路径集的最大流量(maxdemand)路径maxpath
		if ( path->flow > maxdemand)
		{
			maxdemand = path->flow;
			maxpath = path;
		}
	}
	floatType dflow = assDemand -  ttdemand; //OD对之间总需求assDemand与当前计算出的总需求ttdemand的差dflow
	if (dflow!=0) //if OD对之间总需求assDemand 与 当前计算出的总需求ttdemand 不相等，存在差值dflow
	{
		maxpath->flow += dflow; //令超路径集中的最大流量超路径maxpath在原有流量上加上这个差值
		vector<GLINK*> glinks=maxpath->GetGlinks(); //获取最大流量超路径maxpath的弧集glinks
		for(int i=0;i<glinks.size();++i)
		{
			PTLink* plink= glinks[i]->m_linkPtr; //遍历最大流量超路径maxpath上的弧plink
			double lprob= glinks[i]->m_data;     //获取对应的在超路径maxpath上使用弧plink的概率lprob
			plink ->volume += lprob*dflow;  //令超路径maxpath的各段弧在原有弧流量上+lprob*dflow
			if (abs(plink->volume) < 1e-8)
			{
				plink->volume = 0.0;
			}
			plink ->IPFUpdatePTLinkCost(gamma); //更新超路径maxpath的各段弧（和相关弧）的费用/费用一阶导   
			plink ->IPFUpdatePTDerLinkCost(); 
			if (plink ->rLink)
			{
				plink ->rLink ->IPFUpdatePTLinkCost(gamma);
				plink ->rLink ->IPFUpdatePTDerLinkCost();//current link flow variation influnce the related link cost and derivative cost
			}
		}			

	}


}

void PTOrg::PathFlowConservation() 
	//获取当前超路径集的最大流量(maxdemand)路径maxpath
	//计算OD对需求assDemand 与 当前计算出的总需求ttdemand 是否相等，若不相等（存在差值），则将流量差值均分在路径maxpath的各段弧之上，并更新当前路段费用和费用一阶导 
{
	floatType ttdemand = 0.0, maxdemand = 0.0;
	TNM_HyperPath* maxpath;
	for (vector<TNM_HyperPath*>::iterator it = pathSet.begin(); it!=pathSet.end();it++) 
	{
		TNM_HyperPath* path = *it;    //遍历OD对的超路径集pathSet
		ttdemand += path->flow;       //计算OD对之间总需求ttdemand、超路径集的最大流量(maxdemand)路径maxpath
		if ( path->flow > maxdemand)
		{
			maxdemand = path->flow;
			maxpath = path;
		}
	}
	floatType dflow = assDemand -  ttdemand; //OD对之间总需求assDemand与当前计算出的总需求ttdemand的差dflow
	if (dflow!=0) //if OD对之间总需求assDemand 与 当前计算出的总需求ttdemand 不相等，存在差值dflow
	{
		maxpath->flow += dflow; //令超路径集中的最大流量超路径maxpath在原有流量上加上这个差值
		vector<GLINK*> glinks=maxpath->GetGlinks(); //获取最大流量超路径maxpath的弧集glinks
		for(int i=0;i<glinks.size();++i)
		{
			PTLink* plink= glinks[i]->m_linkPtr; //遍历最大流量超路径maxpath上的弧plink
			double lprob= glinks[i]->m_data;     //获取对应的在超路径maxpath上使用弧plink的概率lprob
			plink ->volume += lprob*dflow;  //令超路径maxpath的各段弧在原有弧流量上+lprob*dflow
			if (abs(plink->volume) < 1e-8)
			{
				plink->volume = 0.0;
			}
			plink ->UpdatePTLinkCost(); //更新超路径maxpath的各段弧（和相关弧）的费用/费用一阶导   
			plink ->UpdatePTDerLinkCost(); 
			if (plink ->rLink)
			{
				plink ->rLink ->UpdatePTLinkCost();
				plink ->rLink ->UpdatePTDerLinkCost();//current link flow variation influnce the related link cost and derivative cost
			}
		}			

	}


}


void PTOrg::UpdatePathSetCost() //求解OD对w子问题的收敛指标 相对误差currentRelativeGap
{
	double maxCost= - 1.0 , minCost=POS_INF_FLOAT;
	currentTotalCost = 0.0;

	for (int i =0; i < pathSet.size();i++)
	{		
		TNM_HyperPath* path= pathSet[i]; //遍历该起点org的超路径集合中的超路径path
		path->UpdateHyperpathCost();     //更新超路径path费用和费用一阶导
		if (path->cost> maxCost)	maxCost =   path->cost; //求当前超路径集合中超路径的maxCost、minCost和对应索引minIx

		if (path->cost< minCost)    
		{
			minCost = path->cost; 
			minIx = i; 
		}
		currentTotalCost += path->cost * path->flow; //计算当前超路径集中超路径的费用之和currentTotalCost
	}

	maxPathGap = maxCost - minCost;
	currentRelativeGap = fabs( 1 - minCost * assDemand / currentTotalCost);
}

int	PTNET::GenerateRandGridAN(GRIDPTNETPAR par,string filepath)
{
	ofstream outfile,toutfile;
	srand(par.seed);
	string routefilename = filepath + networkName + "_route.txt";
	cout<<"\tWriting route information into "<<routefilename<<endl;
	if(!TNM_OpenOutFile(outfile, routefilename))
	{
		cout<<"\tCannot open file "<<routefilename<<" to write"<<endl;
		return 1;
	}
	outfile<<"route_id,route_short_name,route_long_name,route_type"<<endl;
	int rid = 1;
	for(int i = 1;i<=par.nx;i++)
	{
		outfile<<""<<rid<<","<<rid<<","<<rid<<","<<"3"<<endl;
		rid++;
		outfile<<""<<rid<<","<<rid<<","<<rid<<","<<"3"<<endl;
		rid++;
	}
	for(int i = 1;i<=par.ny;i++)
	{
		outfile<<""<<rid<<","<<rid<<","<<rid<<","<<"3"<<endl;
		rid++;
		outfile<<""<<rid<<","<<rid<<","<<rid<<","<<"3"<<endl;
		rid++;
	}
	outfile.close();

	string stopfilename = filepath + networkName + "_stop.txt";
	cout<<"\tWriting stop information into "<<stopfilename<<endl;
	if(!TNM_OpenOutFile(outfile, stopfilename))
	{
		cout<<"\tCannot open file "<<stopfilename<<" to write"<<endl;
		return 1;
	}
	outfile<<"STOPID,STOPNAME,X,Y"<<endl;
	double dist = par.gridLen;
	for(int i = 1;i<=par.nx;i++)
	{
		for (int j = 1;j<=par.ny;j++)
		{
			outfile<<""<<(i-1) * par.nx   + j <<","<<(i-1) * par.nx   + j <<","<<dist * i<<","<<dist * j<<endl;
		}
	}
	outfile.close();

	string shapefilename = filepath + networkName + "_shape.txt";
	cout<<"\tWriting shape information into "<<shapefilename<<endl;
	if(!TNM_OpenOutFile(outfile, shapefilename))
	{
		cout<<"\tCannot open file "<<shapefilename<<" to write"<<endl;
		return 1;
	}
	outfile<<"shape_id,route_id,frequency,capacity,stops"<<endl;

	string transitfilename = filepath + networkName + "_transit.txt";
	cout<<"\tWriting transit information into "<<transitfilename<<endl;
	if(!TNM_OpenOutFile(toutfile, transitfilename))
	{
		cout<<"\tCannot open file "<<routefilename<<" to write"<<endl;
		return 1;
	}
	toutfile<<"shapeid,fromstop,tostop,fft,length"<<endl;

	int a = 45, b = 5;
	// generate horizonal lines
	for(int i = 1;i<=par.nx;i++)
	{
		for (int rj = 1; rj<=2; rj++)
		{
			int routeid = 2 * i - 2 + rj;
			for (int k = 1; k<=par.commonLines;k++)
			{
				int frq = rand() % a + b;
				int cap = m_capacity * frq;
				string shapeid = to_string(routeid) + "-" + to_string(k);
				if (rj == 1)
				{
					outfile<<""<<shapeid<<","<<routeid<<","<<frq<<","<<cap<<",";
					for (int j = 1;j<par.nx;j++)
					{
						outfile<<""<<(i-1) * par.nx + j<<",";	
						double t = dist / ( 20 + rand() % 20 ) * 60.0 ;//conver to min
						//cout<<dist<<","<<t<<endl;
						toutfile<<""<<shapeid<<","<<(i-1) * par.nx + j<<","<<(i-1) * par.nx + j + 1<<","<<TNM_FloatFormat(t,4,2)<<","<<dist<<endl;
					}
					outfile<<""<<(i-1) * par.nx + par.nx<<endl;
				}
				else
				{
					outfile<<""<<shapeid<<","<<routeid<<","<<frq<<","<<cap<<",";
					for (int j = par.nx;j > 1;j--)
					{
						outfile<<""<<(i-1) * par.nx + j<<",";
						double t = dist / ( 20 + rand() % 20 ) * 60.0 ;//conver to min
						toutfile<<""<<shapeid<<","<<(i-1) * par.nx + j<<","<<(i-1) * par.nx + j - 1<<","<<TNM_FloatFormat(t,4,2)<<","<<dist<<endl;
					}
					outfile<<""<<(i-1) * par.nx + 1<<endl;
				}
			}
		}
	}
	// generate vertical lines
	for(int i = 1;i<=par.ny;i++)
	{
		for (int rj = 1; rj<=2; rj++)
		{
			int routeid = 2 * i - 2 + rj + 2 * par.nx;
			for (int k = 1; k<=par.commonLines;k++)
			{
				int frq = rand() % a + b;
				int cap = m_capacity * frq;
				string shapeid = to_string(routeid) + "-" + to_string(k);
				if (rj == 1)
				{
					outfile<<""<<shapeid<<","<<routeid<<","<<frq<<","<<cap<<",";
					for (int j = 1;j<par.ny;j++)
					{
						outfile<<""<<(j-1) * par.nx + i<<",";		
						double t = dist / ( 20 + rand() % 20 ) * 60.0 ;//conver to min
						toutfile<<""<<shapeid<<","<<(j-1) * par.nx + i<<","<<j * par.nx + i<<","<<TNM_FloatFormat(t,4,2)<<","<<dist<<endl;
					}
					outfile<<""<<(par.ny - 1) * par.nx + i<<endl;
				}
				else
				{
					outfile<<""<<shapeid<<","<<routeid<<","<<frq<<","<<cap<<",";
					for (int j = par.ny;j > 1;j--)
					{
						outfile<<""<<(j-1) * par.nx + i<<",";	
						double t = dist / ( 20 + rand() % 20 ) * 60.0 ;//conver to min
						toutfile<<""<<shapeid<<","<<(j-1) * par.nx + i<<","<<(j-2) * par.nx + i<<","<<TNM_FloatFormat(t,4,2)<<","<<dist<<endl;
					}
					outfile<<""<<i<<endl;
				}
			}
		}
	}
	outfile.close();
	toutfile.close();

	string tripfilename = filepath + networkName + "_trip.txt";
	cout<<"\tWriting trip information into "<<tripfilename<<endl;
	if(!TNM_OpenOutFile(outfile, tripfilename))
	{
		cout<<"\tCannot open file "<<tripfilename<<" to write"<<endl;
		return 1;
	}
	outfile<<"=================destination,number of origins. org id:demand==========="<<endl;
	outfile.close();

	string walkfilename = filepath + networkName + "_walks.txt";
	cout<<"\tWriting walk information into "<<tripfilename<<endl;
	if(!TNM_OpenOutFile(outfile, walkfilename))
	{
		cout<<"\tCannot open file "<<walkfilename<<" to write"<<endl;
		return 1;
	}
	outfile<<"fromstop,tostop,fft,length,capacity"<<endl;
	outfile.close();
	return 0;

}

void PTNET::GenerateRandGripTrip(GRIDPTNETPAR par)
{
	ofstream outfile;
	string tripfilename = networkName + "_trip.txt";
	floatType ttdemand = 0.0;
	if(!TNM_OpenOutFile(outfile, tripfilename))
	{
		cout<<"\tCannot open file "<<tripfilename<<" to write"<<endl;
		return;
	}
	outfile<<"=================destination,number of origins. org id:demand==========="<<endl;
	int nzones = par.nx * par.ny * par.zRatio;
	//cout<<par.zRatio<<endl;
	if (nzones > 2)
	{
		vector<PTStop*> stops, zonestop;
		for(PTStopMapIter ps = m_stops.begin(); ps!=m_stops.end();ps++)
		{
			stops.push_back(ps->second);
		}

		while (zonestop.size() < nzones)
		{
			int id = (rand() % ((stops.size() - 1) - 0 +1))+ 0;
			PTStop* sstop = stops[id];
			vector<PTStop*>::iterator fit = find(zonestop.begin(),zonestop.end(),sstop);		
			if (fit ==zonestop.end() )   zonestop.push_back(sstop);			
		}


		//floatType sz = 0.28 ;
		floatType sz = 1.0 / pow(nzones,0.36);


		for(int i=0;i<zonestop.size();i++)
		{
			PTStop* dstop = stops[i];
			floatType zdmd =0.0;
			for(PTShapeMapIter it = m_shapes.begin();it!= m_shapes.end();it++)
			{
				PTShape* shp = it->second;
				vector<PTStop*>  ms = shp->m_stops;
				vector<PTStop*>::iterator fit = find(ms.begin(),ms.end(),dstop);		
				if (fit !=ms.end() ) 
				{
					//cout<<shp->m_id<<","<<shp->m_cap<<endl;
					zdmd += shp->m_cap;	
				}
			}
			//cout<<sz<<","<<zdmd<<endl;
			zdmd  *= par.dLevel*sz;

			ttdemand += zdmd;
			floatType tdist = 0.0;
			for(int j=0;j<zonestop.size();j++)
			{
				if(j!=i)
				{
					PTStop* ostop = stops[j];
					tdist += 1.0 / sqrt ( pow(ostop->GetLat() - dstop->GetLat() , 2) + pow(ostop->GetLon() - dstop->GetLon() , 2) ) ;
				}	
			}

			outfile<<"Destination\t"<<dstop->m_id<<"\t"<<zonestop.size() - 1<<endl;
			int k = 0;
			for(int j=0;j<zonestop.size();j++)
			{
				if(j!=i)
				{
					k++;
					PTStop* ostop = stops[j];
					double pdist = 1.0 / sqrt ( pow(ostop->GetLat() - dstop->GetLat() , 2) + pow(ostop->GetLon() - dstop->GetLon() , 2));
					double demand = zdmd * pdist / tdist;

					if (k == zonestop.size() - 1)
					{
						outfile<<ostop->m_id<<":"<<demand<<endl;
					}
					else outfile<<ostop->m_id<<":"<<demand<<",";
				}	
			}
		}
	}
	cout<<"Generate total demand:"<<ttdemand<<endl;
	outfile.close();



}