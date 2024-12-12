#include "..\..\DllProj\trafficNet\stdafx.h"
#include <algorithm>
#include <math.h>
#include <iostream>
#include <tchar.h>

using namespace std;

int TestiGP( const string &path, const string &name)
{
	//读取配置文件
	ifstream conf;
	conf.open(path+"\\conf.csv");
	if(!conf.is_open())
	{
		cout<<"Can't open the configuration file!"<<endl;
		return 1;
	}
	int max_iter=100;
	floatType max_relgap=1e-10;
	floatType max_time=500;
	bool centroids_blocked=false;
	floatType vot=10;

	int count=0;
	while(!conf.eof())
	{
		count++;
		string line;
		getline(conf,line);
		const int strEnd = line.find_last_not_of(",");
		line=line.substr(0,strEnd+1);
		vector<string> words;
		TNM_GetWordsFromLine(line, words, ',');
		if(words.size()>1)
		{
			if(count==1)
			{
				TNM_FromString<int>(max_iter, words[1], std::dec);
			}
			if(count==2)
			{
				TNM_FromString<floatType>(max_relgap, words[1], std::dec);
			}
			if(count==3)
			{
				TNM_FromString<floatType>(max_time, words[1], std::dec);
			}
			if(count==4)
			{
				if(words[1]=="T")
					centroids_blocked=true;
			}
			if(count==5)
			{
				TNM_FromString<floatType>(vot, words[1], std::dec);
			}

		}
	}
	cout<<"max_iter "<<max_iter<<endl;
	cout<<"max_relgap "<<max_relgap<<endl;
	cout<<"max_time "<<max_time<<endl;
	cout<<"centroids_blocked "<<centroids_blocked<<endl;
	cout<<"vot "<<vot<<endl;
	getchar();


	TNM_FloatFormat::SetFormat(18,6);
	TAP_iGP *ofw = new TAP_iGP;
	//tapas->SetCostScalar(60);
	ofw->SetConv(max_relgap);
	ofw->SetInnerConv(1e-10);
	ofw->SetMaxLsIter(15);
	ofw->SetMaxInnerIter(20);
	ofw->SetMaxIter(max_iter);
	ofw->SetMaxCPUTime(max_time);
	//ofw->SetLPF(BPRLK);

	floatType per=1;
	cout<<"是否存在ackelik类型的路段(T/F)：";
	string ack;
	cin>>ack;
	if(ack=="T")
	{
		cout<<"若使用了Akcelik函数，请输入分配的时段长度：";
		string ti;
		cin>>ti;
		TNM_FromString<floatType>(per,ti,std::dec);
	}
	

	string input, output;
	input = path +   name;
	output = path  + name + "_iGP";

	if(ofw->Build(input, output,NETSZFAS)!=0)
	{
		cout<<"\tFail to build network object"<<endl;
		return 0;
	}
	ofw->network->SetPeriodOfAlkLink(per);

	ofw->network->centroids_blocked=centroids_blocked;////搜索路径时是否封闭形心；
	if(ofw->network->centroids_blocked)
	{
		for(int a=0;a<ofw->network->centroidSet.size();a++)
		{
			TNM_SNODE* node=ofw->network->centroidSet[a];
			node->is_centroid=true;
			node->SkipCentroid=true;
		}
		//ofw->network->nodeVector[425]->SkipCentroid=true;
	}
	

	ofw->SetTollType(TT_FXTOLL); //有toll设置为TT_FXTOLL；无toll设置为TT_NOTOLL
	for(int a=0;a<ofw->network->numOfLink;a++)
	{
		TNM_SLINK* link=ofw->network->linkVector[a];
		link->vot=vot;
	}
	//for(int a=0;a<ofw->network->numOfLink;a++)
	//{
	//	TNM_SLINK* link=ofw->network->linkVector[a];
	//	link->toll=2;
	//	link->vot=0.2;
	//	if(a<100)
	//	link->beckFlow=356;
	//	else if(a>200 && a<260)
	//		link->beckFlow=652;
	//	else
	//		link->beckFlow=462;

	//}

	if(ofw->Solve()== ErrorTerm)  //solve traffic assignment problem
	{
		cout<<"something wrong happend in solving the problem"<<endl;
		return	0;
	}

	//vector<TNM_SLINK*> linkv = ofw->network->linkVector;
	//for(int llcount=0;llcount<=20;llcount++)//linkv.size()
	//{
	//	cout<<"ID"<<linkv[llcount]->id<<"head"<<linkv[llcount]->head->id<<"tail"<<linkv[llcount]->tail->id<<"flow"<<linkv[llcount]->volume<<endl;
	//}

	cout<<"\tTerminate flag="<<ofw->termFlag<<endl;
	cout<<"OFV="<<ofw->OFV<<endl;
	//for(int a=0;a<ofw->network->numOfLink;a++)
	//{
	//	TNM_SLINK* link=ofw->network->linkVector[a];
	//	if(link->volume<link->beckFlow)
	//		cout<<link->tail->id<<"->"<<link->head->id<<" "<<link->volume<<endl;
	//}
	//for(int i=0;i<ofw->network->numOfOrigin;i++)
	//{
	//	TNM_SORIGIN* orgn=ofw->network->originVector[i];
	//	for(int j=0;j<orgn->numOfDest;j++)
	//	{
	//		TNM_SDEST* dest=orgn->destVector[j];
	//		for(int k=0;k<dest->pathSet.size();k++)
	//		{
	//			TNM_SPATH* path=dest->pathSet[k];
	//			for(int a=0;a<path->path.size();a++)
	//			{
	//				TNM_SLINK* link=path->path[a];
	//				//if(link->head->SkipCentroid && link->head->id!=dest->dest->id)
	//				if(link->head->id == 426)
	//				{
	//					cout<<"From "<<orgn->origin->id<<" to "<<dest->dest->id <<"path id "<<path->id<<" through node "<<link->head->id<<endl;
	//					break;
	//				}
	//			}
	//		}
	//	}
	//}

	ofw->reportIterHistory=true;
	ofw->reportLinkDetail=true;
	ofw->reportPathDetail=true;
	//ofw->Report();
	ofw->ReportSZFormat();

	delete ofw;
	return 0;
}

int TestMCiGP( const string &path, const string &name)
{
	TNM_FloatFormat::SetFormat(18,6);
	MC_iGP *ofw = new MC_iGP;
	//tapas->SetCostScalar(60);
	ofw->SetConv(1e-10);
	ofw->SetInnerConv(1e-10);
	ofw->SetMaxLsIter(15);
	ofw->SetMaxInnerIter(20);
	ofw->SetMaxIter(100);
	ofw->SetLPF(BPRLK);
	ofw->SetMaxCPUTime(3600);


	string input, output;
	input = path +   name;
	output = path  + name + "_iGP";

	if(ofw->Build(input, output,NETSZFMC)!=0)
	{
		cout<<"\tFail to build network object"<<endl;
		return 0;
	}
	//若使用了Akcelik函数，请输入分配的时段长度
	floatType per=1;
	ofw->network->SetPeriodOfAlkLink(per);

	ofw->network->centroids_blocked=true;////搜索路径时是否封闭形心；
	if(ofw->network->centroids_blocked)
	{
		for(int a=0;a<ofw->network->numOfOrigin;a++)
		{
			TNM_SORIGIN* orgn=ofw->network->originVector[a];
			orgn->origin->is_centroid=true;
			orgn->origin->SkipCentroid=true;
		}
		//ofw->network->nodeVector[425]->SkipCentroid=true;
	}
	

	ofw->SetTollType(TT_FXTOLL); //有toll设置为TT_FXTOLL；无toll设置为TT_NOTOLL
	//for(int a=0;a<ofw->network->numOfLink;a++)
	//{
	//	TNM_SLINK* link=ofw->network->linkVector[a];
	//	link->toll=2;
	//	link->vot=0.2;
	//	if(a<100)
	//	link->beckFlow=356;
	//	else if(a>200 && a<260)
	//		link->beckFlow=652;
	//	else
	//		link->beckFlow=462;

	//}

	if(ofw->Solve()== ErrorTerm)  //solve traffic assignment problem
	{
		cout<<"something wrong happend in solving the problem"<<endl;
		return	0;
	}

	cout<<"\tTerminate flag="<<ofw->termFlag<<endl;

	//vector<TNM_SLINK*> linkv = ofw->network->linkVector;
	//for(int llcount=0;llcount<=20;llcount++)//linkv.size()
	//{
	//	cout<<"ID"<<linkv[llcount]->id<<"head"<<linkv[llcount]->head->id<<"tail"<<linkv[llcount]->tail->id<<"flow"<<linkv[llcount]->volume<<endl;
	//}


	//cout<<"OFV="<<ofw->OFV<<endl;
	//for(int a=0;a<ofw->network->numOfLink;a++)
	//{
	//	TNM_SLINK* link=ofw->network->linkVector[a];
	//	if(link->volume<link->beckFlow)
	//		cout<<link->tail->id<<"->"<<link->head->id<<" "<<link->volume<<endl;
	//}
	//for(int i=0;i<ofw->network->numOfOrigin;i++)
	//{
	//	TNM_SORIGIN* orgn=ofw->network->originVector[i];
	//	for(int j=0;j<orgn->numOfDest;j++)
	//	{
	//		TNM_SDEST* dest=orgn->destVector[j];
	//		for(int k=0;k<dest->pathSet.size();k++)
	//		{
	//			TNM_SPATH* path=dest->pathSet[k];
	//			for(int a=0;a<path->path.size();a++)
	//			{
	//				TNM_SLINK* link=path->path[a];
	//				//if(link->head->SkipCentroid && link->head->id!=dest->dest->id)
	//				if(link->head->id == 426)
	//				{
	//					cout<<"From "<<orgn->origin->id<<" to "<<dest->dest->id <<"path id "<<path->id<<" through node "<<link->head->id<<endl;
	//					break;
	//				}
	//			}
	//		}
	//	}
	//}

	ofw->reportIterHistory=true;
	ofw->reportLinkDetail=true;
	ofw->reportPathDetail=true;
	//ofw->Report();
	ofw->ReportSZFormat();

	delete ofw;

	return 0;
}

int TestRoadPathSearch(const string &path)
{
	//读取配置文件
	ifstream conf;
	conf.open(path+"\\conf.csv");
	if(!conf.is_open())
	{
		cout<<"Can't open the configuration file!"<<endl;
		return 1;
	}
	string minimize_field="fft";
	vector<string> export_field_list;
	export_field_list.clear();
	vector<int> from_node_list;
	from_node_list.clear();
	vector<int> to_node_list;
	to_node_list.clear();
	bool centroids_blocked=false;
	int export_path_num=1;

	int count=0;
	while(!conf.eof())
	{
		count++;
		string line;
		getline(conf,line);
		vector<string> words;
		const int strEnd = line.find_last_not_of(",");
		line=line.substr(0,strEnd+1);//删除多余的逗号
		//cout<<line<<endl;
		TNM_GetWordsFromLine(line, words, ',');
		
		//cout<<words.size()<<endl;
		if(words.size()>1)
		{
			if(count==1)
				minimize_field=words[1];
			if(count==2)
			{
				for(int i=1;i<words.size();i++)
				{
					export_field_list.push_back(words[i]);
				}
			}
			if(count==3)
			{
				for(int i=1;i<words.size();i++)
				{
					int nodeid;
					TNM_FromString<int>(nodeid, words[i], std::dec);
					from_node_list.push_back(nodeid);
				}
			}
			if(count==4)
			{
				for(int i=1;i<words.size();i++)
				{
					int nodeid;
					TNM_FromString<int>(nodeid, words[i], std::dec);
					to_node_list.push_back(nodeid);
				}
			}
			if(count==5)
			{
				if(words[1]=="T")
					centroids_blocked=true;
			}
			if(count==6)
			{
				TNM_FromString<int>(export_path_num, words[1], std::dec);
			}

		}
	}
	cout<<"minimize_field "<<minimize_field<<endl;
	cout<<"export_field_list ";
	for(int i=0;i<export_field_list.size();i++)
		cout<<export_field_list[i]<<" ";
	cout<<endl;
	cout<<"from_node_list ";
	for(int i=0;i<from_node_list.size();i++)
		cout<<from_node_list[i]<<" ";
	cout<<endl;
	cout<<"to_node_list ";
	for(int i=0;i<to_node_list.size();i++)
		cout<<to_node_list[i]<<" ";
	cout<<endl;
	cout<<"centroids_blocked "<<centroids_blocked<<endl;
	cout<<"export_path_num "<<export_path_num<<endl;
	//getchar();

	//构建网络
	TNM_SNET* net=new TNM_SNET(path);
	//centroid blocked
	net->centroids_blocked=centroids_blocked;
	net->BuildSZFormatPS();
	//minimize field
	
	for(int a=0;a<net->numOfLink;a++)
	{
		TNM_SLINK* link=net->linkVector[a];
		if(minimize_field=="fft")
			link->cost=link->fft;
		else if(minimize_field=="distance")
			link->cost=link->length;
		else if(minimize_field=="toll")
			link->cost=link->toll;
		//cout<<"link "<<link->id<<" cost "<<link->cost<<" "<<link->head->forwStar.size()<<endl;
		if(link->cost<0.0)
		{
			cout<<"Error! cost of link "<<link->id<<" is negative!"<<endl;
			return 1;
		}
	}
	
	//bool centroids_blocked=false;
	//if(centroids_blocked)
	//{
	//	for(int i=0;i<net->numOfNode;i++)
	//	{
	//		TNM_SNODE* node=net->nodeVector[i];
	//		if(node->is_centroid)
	//			node->SkipCentroid=true;
	//	}
	//}

	//搜索最短路与结果输出
	//int exportpathnum=10;
	//cout<<"num of node "<<net->numOfNode<<endl;
	//cout<<net->nodeVector[0]->id<<" "<<net->nodeVector[10]->id;
	//queue<TNM_SPATH*> pathset=net->DIJK_KSP(net->nodeVector[0]->id,net->nodeVector[10]->id,5,-1,true,false);

	string FileName  = path + "result.csv";
	ofstream resfile;
	if (!TNM_OpenOutFile(resfile, FileName))
	{
		cout<<"\nCannot open result file to write!"<<endl;
	}
	resfile<<"from_node,to_node,path";
	for(int k=0;k<export_field_list.size();k++)
	{
		resfile<<","<<export_field_list[k];
	}
	resfile<<endl;
	
	for(int i=0;i<from_node_list.size();i++)
	{
		//TNM_SNODE* from=net->CatchNodePtr(from_node_list[i], false);
		for(int j=0;j<to_node_list.size();j++)
		{
			if(from_node_list[i]==to_node_list[j])
				continue;
			//TNM_SNODE* to=net->CatchNodePtr(to_node_list[j], false);
			queue<TNM_SPATH*> pathset=net->BFM_KSP(from_node_list[i],to_node_list[j],export_path_num,false);//loop=false时排除环路径
			//结果输出
			//cout<<"path size="<<pathset.size()<<endl;
			//getchar();
			while (!pathset.empty())
			{
				TNM_SPATH* path=pathset.front();
				char res[200];
				sprintf(res,"%d",from_node_list[i]);
				string strr=res;
				resfile<<from_node_list[i]<<","<<to_node_list[j]<<","<<strr;
				
				floatType fft=0.0, distance=0.0, toll=0.0;
				for(int a=0;a<(path->path.size());a++)
				{
					strr="_";
					sprintf(res,"%d",path->path[a]->head->id);
					strr+=res;
					resfile<<strr;

					fft+=path->path[a]->fft;
					distance+=path->path[a]->length;
					toll+=path->path[a]->toll;
					//cout<<path->path[a]->head->id<<" ";
					//getchar();
					//cost+=path->path[a]->cost;
				}
				//cout<<" cost="<<cost<<endl;

				for(int k=0;k<export_field_list.size();k++)
				{
					if(export_field_list[k]=="fft")
						resfile<<","<<fft;
					if(export_field_list[k]=="distance")
						resfile<<","<<distance;
					if(export_field_list[k]=="toll")
						resfile<<","<<toll;
				}
				resfile<<endl;
				//path->Print(true);
				delete path;
				pathset.pop();  // Delete the 1st element
			}
		}
	}
	resfile.close();
	
	getchar();
	delete net;
	return 0;
}

int TestRoadSkim(const string &path)
{
	TNM_SNET* net=new TNM_SNET(path);
	net->BuildSZFormatPS();
	//minimize field
	string minimize_field="fft";
	for(int a=0;a<net->numOfLink;a++)
	{
		TNM_SLINK* link=net->linkVector[a];
		if(minimize_field=="fft")
			link->cost=link->fft;
		else if(minimize_field=="distance")
			link->cost=link->length;
		else if(minimize_field=="toll")
			link->cost=link->toll;
		cout<<"link "<<link->id<<" cost "<<link->cost<<" "<<link->head->forwStar.size()<<endl;
		if(link->cost<0.0)
		{
			cout<<"Error! cost of link "<<link->id<<" is negative!"<<endl;
			return 1;
		}
	}
	//centroid blocked
	net->centroids_blocked=false;
	//bool centroids_blocked=false;
	//if(centroids_blocked)
	//{
	//	for(int i=0;i<net->numOfNode;i++)
	//	{
	//		TNM_SNODE* node=net->nodeVector[i];
	//		if(node->is_centroid)
	//			node->SkipCentroid=true;
	//	}
	//}

	vector<TNM_SNODE*> centroidSet;
	centroidSet.clear();
	for(int i=0;i<net->numOfNode;i++)
	{
		TNM_SNODE* node=net->nodeVector[i];
		if(node->is_centroid)
			centroidSet.push_back(node);
	}

	//Path search
	int exportpathnum=10;
	for(int i=0;i<centroidSet.size();i++)
	{
		TNM_SNODE* start=centroidSet[i];
		for(int j=0;j<centroidSet.size();j++)
		{
			TNM_SNODE* end=centroidSet[j];
			if(i!=j)
			{
				queue<TNM_SPATH*> pathset=net->BFM_KSP(start->id,end->id,exportpathnum,false);//loop=false时排除环路径
			}
		}
	}

	delete net;
	return 0;
}

int main()
{
	string cs="chicagosketch";
	string an="Anaheim";
	string cr="chi";
	string pr = "PRISM";
	string sf = "sf";
	string win="Winnipeg";
	string ba="Barcelona";
	string phi="Philadelphia";
	string aus = "Austin";
	string mag = "mag";
	string en = "en";
	string hearn = "hearn";

	//TestiGP("D:\\项目文件\\深圳交通中心-分配算法项目\\算法输入输出格式\\xie_test_file_bpr\\","");
	//TestiGP("C:\\Users\\82212\\Desktop\\单用户分配网络\\","");
	//TestMCiGP("D:\\项目文件\\深圳交通中心-分配算法项目\\算法输入输出格式\\small_net\\","");
	//TestRoadSkim("D:\\项目文件\\深圳交通中心-分配算法项目\\算法输入输出格式\\道路交通分配输入输出设计_20210611\\M2-1_road_path_search\\");

	string fun, add;
	cout<<"请输入需要测试的功能（1=路径搜索、2=单用户分配）：";
	cin>>fun;
	cout<<"\n请输入网络文件所在目录（例如：D:\\Data\\network\\）：";
	cin>>add;
	cout<<"\n";

	if(fun=="2")
		TestiGP(add,"");

	if(fun=="1")
		TestRoadPathSearch(add);

	cout<<"\n\n\nRun finished!";
	system("PAUSE");
};