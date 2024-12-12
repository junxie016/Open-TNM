#include"stdafx.h"
string basefile = "E:\\CTEAP代码\\transitCapTL";  

void GentileNet()
{
	PTNET* Pnet=new PTNET(basefile+"\\transitNet\\GentileNet\\Gentile");
	Pnet->Settimescaler(60.0);
	Pnet->SetAlightLoss(0.1);
	Pnet->SetCapcityPerLine(30);
	Pnet->SetSymLinkType(false);
	Pnet->BuildAN(false,true);
	Pnet->SetConv(1e-14);
	Pnet->SetMaxIter(200);
	Pnet->SetMaxIterTime(60*1);
	Pnet->SetInnerConv(1e-12);
	Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::PCTAE_P_iGreedy); //no capacity: PCTAE_A_GFW / PCTAE_P_iGreedy / PCTAE_P_iNGP 
	Pnet->ReportAvgpaxinfo();
	Pnet->ReportPTlinkflow();
	Pnet->ExportHyperpathUEsolution();
}



void CapGentileNet()
{
	PTNET* Pnet=new PTNET(basefile+"\\transitNet\\GentileNet\\Gentile");
	Pnet->Settimescaler(60.0);
	Pnet->SetAlightLoss(0.1);
	Pnet->SetCapcityPerLine(30);
	Pnet->SetSymLinkType(false);
	Pnet->BuildAN(false,true);
	Pnet->SetConv(1e-14);
	Pnet->SetMaxIter(200);
	Pnet->SetMaxIterTime(60*1);
	Pnet->SetInnerConv(1e-12);

	Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::CAP_PCTAE_P_GP_TL);  //CTEAP-MOM: CAP_PCTAE_P_GFW_TL (link-based)   CAP_PCTAE_P_Greedy_TL / CAP_PCTAE_P_GP_TL (hyperpath-based)
	//Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL);  //CTEAP-IPF
	
	Pnet->ReportAvgpaxinfo(); //for MOM/IPF
	Pnet->ReportPTHyperpaths(); //for MOM/IPF
	Pnet->ReportWalkflowRation();  //for MOM/IPF
	Pnet->ReportIter();     //MOM
	Pnet->ReportPTlinkflowCap(); //for MOM
	//Pnet->ReportIterIPF();       //for IPF
	//Pnet->ReportPTlinkflowCapIPF(); //for IPF
}


void SiousfallsNet()
{
	PTNET* Pnet=new PTNET(basefile+"\\transitNet\\SiouxFallsnet_cap\\siouxfalls");
	Pnet->Settimescaler(60.0);
	Pnet->SetAlightLoss(0.1);
	Pnet->SetCapcityPerLine(30);
	Pnet->BuildAN(false,true);
	Pnet->SetConv(1e-14);
	Pnet->SetMaxIter(2000);
	Pnet->SetMaxIterTime(60);
	Pnet->SetInnerConv(1e-12);
	Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::PCTAE_P_iNGP); //no capacity:  PCTAE_A_GFW / PCTAE_P_iGreedy / PCTAE_P_iNGP 
	Pnet->ReportIter();
	Pnet->ReportPTlinkflow();
	Pnet->ReportLinkCap();
}



void CapSiousfallsnet()
{
	PTNET* Pnet=new PTNET(basefile+"\\transitNet\\SiouxFallsnet_cap\\siouxfalls");
	Pnet->Settimescaler(60.0);
	Pnet->SetAlightLoss(0.1);
	Pnet->SetCapcityPerLine(30);
	Pnet->SetSymLinkType(false);
	Pnet->BuildAN(false,true);
	Pnet->SetConv(1e-14);
	Pnet->SetMaxIter(300);
	Pnet->SetMaxIterTime(60);
	Pnet->SetInnerConv(1e-12);

	Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL);  //CTEAP-MOM: CAP_PCTAE_P_GFW_TL (link-based)   CAP_PCTAE_P_Greedy_TL / CAP_PCTAE_P_GP_TL (hyperpath-based)
	//Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL);  //CTEAP-IPF
	
	Pnet->ReportAvgpaxinfo(); //for MOM/IPF
	Pnet->ReportPTHyperpaths(); //for MOM/IPF
	Pnet->ReportWalkflowRation();  //for MOM/IPF
	Pnet->ReportIter();     //MOM
	Pnet->ReportPTlinkflowCap(); //for MOM
	//Pnet->ReportIterIPF();       //for IPF
	//Pnet->ReportPTlinkflowCapIPF(); //for IPF
}




void WinnipegNet()
{
	PTNET* Pnet=new PTNET(basefile+"\\transitNet\\winnipeg\\winnipeg");
	Pnet->Settimescaler(60.0);
	Pnet->SetAlightLoss(0.1);
	Pnet->SetSymLinkType(false);
	Pnet->BuildAN(false,true);
	Pnet->SetConv(1e-8);
	Pnet->SetMaxIter(1000);
	Pnet->SetMaxIterTime(60);
	Pnet->SetInnerConv(1e-10);
	Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::PCTAE_P_iNGP); //no capacity:  PCTAE_A_GFW / PCTAE_P_iGreedy / PCTAE_P_iNGP
	Pnet->ReportIter();
	Pnet->ReportPTlinkflow();
	Pnet->ReportLinkCap();
}




void  CapWinnipegNet()
{
	PTNET* Pnet=new PTNET(basefile+"\\transitNet\\winnipeg\\winnipeg");
	Pnet->Settimescaler(60.0);
	Pnet->SetAlightLoss(0.1);
	Pnet->SetCapcityPerLine(30);
	Pnet->SetSymLinkType(false);
	Pnet->BuildAN(false,true);
	Pnet->SetConv(1e-10);
	Pnet->SetMaxIter(1000);
	Pnet->SetMaxIterTime(60*1);
	Pnet->SetInnerConv(1e-12);

	Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::CAP_PCTAE_P_GP_TL);  //CTEAP-MOM: CAP_PCTAE_P_GFW_TL (link-based)   CAP_PCTAE_P_Greedy_TL / CAP_PCTAE_P_GP_TL (hyperpath-based)
	//Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL);  //CTEAP-IPF
	
	Pnet->ReportAvgpaxinfo(); //for MOM/IPF
	Pnet->ReportPTHyperpaths(); //for MOM/IPF
	Pnet->ReportWalkflowRation();  //for MOM/IPF
	Pnet->ReportIter();     //MOM
	Pnet->ReportPTlinkflowCap(); //for MOM
	//Pnet->ReportIterIPF();       //for IPF
	//Pnet->ReportPTlinkflowCapIPF(); //for IPF
}




void ShenzhenCenterNet()
{
	PTNET* Pnet=new PTNET(basefile+"\\transitNet\\shenzhennet\\shenzhencenter");
	Pnet->Settimescaler(60.0);
	Pnet->SetAlightLoss(0.1);
	Pnet->BuildAN(true,false);
	Pnet->SetConv(1e-4);     //convCriterion=e
	Pnet->SetMaxIter(200);      //maxMainIter = i 
	Pnet->SetMaxIterTime(60*0.75);   //maxIterTime = i /min  shenzhen 60*1.5
	Pnet->SetInnerConv(1e-4);    //m_innerConv = c

	Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::PCTAE_P_iNGP); //TEAP: PCTAE_A_GFW /  PCTAE_P_iGreedy / PCTAE_P_iNGP		
	Pnet->ReportIter();
	Pnet->ReportPTlinkflow();
	Pnet->ReportLinkCap();
}

void CapShenzhenCenterNet()
{
	PTNET* Pnet=new PTNET(basefile+"\\transitNet\\shenzhennet\\shenzhencenter");
	Pnet->Settimescaler(60.0);
	Pnet->SetAlightLoss(0.1);
	Pnet->BuildAN(true,false);
	Pnet->SetConv(1e-4);     //convCriterion=e
	Pnet->SetMaxIter(200);      //maxMainIter = i 
	Pnet->SetMaxIterTime(60*0.75);   //maxIterTime = i /min  shenzhen 60*1.5
	Pnet->SetInnerConv(1e-4);    //m_innerConv = c

	Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::CAP_PCTAE_P_Greedy_TL);  //CTEAP-MOM: CAP_PCTAE_P_GFW_TL (link-based)   CAP_PCTAE_P_Greedy_TL / CAP_PCTAE_P_GP_TL (hyperpath-based)
	//Pnet->PCTAE_Solver(PTNET::PCTAE_algorithm::CAP_PCTAE_IPF_iGP_TL);  //CTEAP-IPF
	
	Pnet->ReportAvgpaxinfo(); //for MOM/IPF
	Pnet->ReportPTHyperpaths(); //for MOM/IPF
	Pnet->ReportWalkflowRation();  //for MOM/IPF
	Pnet->ReportIter();     //MOM
	Pnet->ReportPTlinkflowCap(); //for MOM
	//Pnet->ReportIterIPF();       //for IPF
	//Pnet->ReportPTlinkflowCapIPF(); //for IPF
}





void TestSZTransitNet()
{
	PTNET* Pnet=new PTNET("D:\\MyDrive\\项目\\交通中心算法项目\\公交网络_步行网络_中等规模\\公交网络");

	Pnet->SZ_BuildAN();


}



int main(int argc, char **argv)
{
	//TEAP
	//GentileNet();			
	SiousfallsNet();		
	//WinnipegNet();		
	//ShenzhenCenterNet();  


	//CTEAP
	//CapGentileNet();		
	//CapSiousfallsnet();   
	//CapWinnipegNet();     
	//CapShenzhenCenterNet();
	
	cout<<"Run finished!"<<endl;
	getchar();

}
