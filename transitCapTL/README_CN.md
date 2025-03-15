### 公交均衡分配问题
#### 算法介绍
该算法是一种基于超路径的算法，旨在解决公交均衡分配问题（TEAP）。即使在大规模网络中，它也能以令人满意的精度高效求解TEAP。我们量身定制了两种改进的基于牛顿型超路径的算法，即带有自适应内循环的贪心算法和梯度投影算法，分别简称为Greedy - AIL和GP - AIL。该算法主要由两部分组成：列生成和自适应内循环。前者利用标号设定最短超路径算法在需要时引入新的超路径，而后者则通过求解基于超路径的子问题来进行自适应均衡。

#### 算法运行方法
- 使用Visual Studio 2012打开“./transitCapTL/Source/Solution/mySolution.sln”中的解决方案。
- 主函数位于“transitDriver”项目的“TEAPDriver.cpp”文件中。
- 提供了四个测试网络，包括：詹蒂莱（Gentile）、锡奥克斯 - 福尔斯（Sious - Falls）、温尼伯（Winnipeg）和深圳中心公交网络。测试网络需要运行相应的函数，例如，詹蒂莱网络对应的函数是“GentileNet()”。
- 修改不同网络函数中的参数。
- 使用“Ctrl + F5”组合键编译并执行代码。

#### 出版物信息
Xu, Z., J. Xie, X. Liu, and Y. M. Nie. 2020. 基于超路径的公交均衡分配问题算法. 《交通研究E辑：物流与交通评论》143: 102102. https://doi.org/10.1016/j.tre.2020.102102.

### 容量受限公交均衡分配问题
#### 算法介绍
transitCapTL算法是一种基于超路径的算法，用于解决容量受限公交均衡分配问题（CTEAP）。即使在大规模网络中，它也能以令人满意的精度高效求解CTEAP。该算法由两部分组成：
第一部分（主问题）通过基于乘子法（MOM）或内部惩罚函数法（IPF）的算法框架来求解CTEAP。
第二部分（子问题）通过基于超路径的贪心算法或梯度投影算法来求解无容量限制的TEAP子问题。

#### 算法运行方法
- 使用Visual Studio 2012打开“./transitCapTL/Source/Solution/mySolution.sln”中的解决方案。
- 主函数位于“transitDriver”项目的“TEAPDriver.cpp”文件中。
- 提供了四个测试网络，包括：詹蒂莱（Gentile）、锡奥克斯 - 福尔斯（Sious - Falls）、温尼伯（Winnipeg）和深圳中心公交网络。测试网络需要运行相应的函数，例如，詹蒂莱网络对应的函数是“CapGentileNet()”。
- 修改不同网络函数中的参数。
- 使用“Ctrl + F5”组合键编译并执行代码。

#### 如何构建起讫点（O - D）对之间的步行路段
- 在詹蒂莱公交网络上测试CTEAP时，在“PCTAE_Solver(PCTAE_algorithm alg)”函数中运行“CreateWalkLink()”。
- 在锡奥克斯 - 福尔斯、温尼伯和深圳中心公交网络上测试CTEAP时，在“PCTAE_Solver(PCTAE_algorithm alg)”函数中运行“CreateODLink()”。

#### 出版物信息
Liang, W., Xu, Z., Xie, J., Liu, X., and Feng, Y. 2024. 一种高效的基于超路径的容量受限公交均衡分配问题算法. 《运输计量学A：运输科学》. https://doi.org/10.1080/23249935.2024.2326819 