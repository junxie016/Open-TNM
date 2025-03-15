# Open-TNM

The open-TNM package is an open-source Toolkit of Network Modeling package written in C++ for solving different transportation network problems, including path-based and origin-based algorithms for the user-equilibrium traffic assignment problems,  traffic assignment problems with side constraints, maximum entropy user equilibrium traffic assignment problems, frequency-based transit assignment problems, continuous bi-criteria traffic assignment problems and so on. This package is written and maintained by Yu (Marco) Nie from Northwestern University and Jun Xie from Southwest Jiaotong University. As of now, it contains just one traffic assignment algorithm, but the plan is to gradually release many more in the coming months. You might also be interested in reading a preface blog [here](https://sites.northwestern.edu/marconie/2024/01/14/open-tnm/).

# Classes Structure

Please refer to the "TNM_Manual.pdf" for a detailed description of the TNM package. A concise introduction to its class structure is as follows: 

The network classes are defined as TNM SNET classes in TNM (‘TNM NET.h’ ). The main network elements include:

• TNM SLINK: It is a basic class used to represent network links. Derived classes like TNM BPRLK support evaluating link travel time using a BPR-type link performance function. Key attributes include tail node, head node, volume, fft (free flow time), capacity, toll, cost (travel time), etc.

• TNM SNODE: This class represents network nodes which include attributes like forwStar (in-node links), backStar (out-node links), pathElem (shortest path element to the node), etc.

• TNM SORIGIN: All origin-destination (O-D) pairs are organized using origins as roots, i.e., each origin stores a vector of destinations associated with positive demand.

• TNM SDEST: The dest (destination node), assDemand (O-D pair demand), origin (the corresponding origin node), pathSet (paths used by each O-D pair), pathSetbound (path time equivalence of money (TEM) boundaries) are stored in TNM SDEST object.

• TNM SPATH: The path (the collection of links that make up the path) and some path cost evaluation functions are defined in the TNM SPATH class.

• SCANLIST: Scanlist is introduced into TNM to accommodate different labeling shortest path problem algorithms.

The buffer attributes of each class serve as repositories for data pertinent to the object, essential for algorithmic calculations. The traffic assignment algorithms are derived from the TNM_TAP algorithm class, as defined in the 'TNM Algorithm.h' file.
This head file also encompasses general attributes and processes crucial for executing traffic assignment algorithms, including convergence accuracy, maximum iteration, and objective function values. In the test functions found in 'xxDriver.cpp,' we instantiate the algorithm, specifying relevant algorithm and network attributes. Subsequently, the network is constructed using the Build() method.

The Solve() process involves executing Preprocess(), Initialization(), Mainloop(), and Postprocess() successively. It also checks Terminate() when necessary. Finally, the Report() method is utilized to output iteration and equilibrium results.

### 类结构

有关 TNM 包的详细描述，请参考《TNM_手册.pdf》。以下是其类结构的简要介绍：

网络类在 TNM（《TNM NET.h》）中被定义为 TNM SNET 类。主要的网络元素包括：
- **TNM SLINK**：这是一个用于表示网络链路的基础类。像 TNM BPRLK 这样的派生类支持使用 BPR 型链路性能函数来评估链路行程时间。关键属性包括尾节点、头节点、流量、自由流时间（fft）、通行能力、收费、成本（行程时间）等。
- **TNM SNODE**：该类表示网络节点，其属性包括前向星（入节点链路）、后向星（出节点链路）、路径元素（到该节点的最短路径元素）等。
- **TNM SORIGIN**：所有的起讫点（O - D）对都以起点为根进行组织，即每个起点存储一个与之相关且有正需求的终点向量。
- **TNM SDEST**：TNM SDEST 对象中存储了终点（目的节点）、分配需求（O - D 对需求）、起点（对应的起点节点）、路径集（每个 O - D 对使用的路径）、路径集边界（路径时间货币等价（TEM）边界）等信息。
- **TNM SPATH**：TNM SPATH 类中定义了路径（构成该路径的链路集合）以及一些路径成本评估函数。
- **SCANLIST**：TNM 中引入了扫描列表，以适应不同的标号最短路径问题算法。

每个类的缓冲区属性作为与对象相关数据的存储库，对算法计算至关重要。交通分配算法派生自 TNM_TAP 算法类，该类在《TNM 算法.h》文件中定义。这个头文件还包含了执行交通分配算法所需的通用属性和流程，包括收敛精度、最大迭代次数和目标函数值等。在《xxDriver.cpp》的测试函数中，我们实例化该算法，并指定相关的算法和网络属性。随后，使用 Build() 方法构建网络。


Solve() 过程依次执行预处理（Preprocess()）、初始化（Initialization()）、主循环（Mainloop()）和后处理（Postprocess()），必要时还会检查终止条件（Terminate()）。最后，使用报告（Report()）方法输出迭代和均衡结果。 
# License
Copyright (c) 2023 Yu (Marco) Nie and Jun Xie.  All rights reserved.

This open Toolkit of Network Modeling (open-TNM) is provided "as is," without warranty of any kind, express or implied.  
In no event shall the author or contributors be held liable for any damages arising in any way from the use of this toolkit.

The contents of this file are DUAL-LICENSED. You may modify and/or redistribute these codes according to the following restrictions:

1. Personal and Academic Use License (GNU GPL v2 or later)

Individuals and academic users are granted the free right to obtain, use, copy, modify, and distribute these codes and its documentation, provided that all copies or substantial portions of the Software must include the notice of copyright and this license agreement.

2. Commercial Use Additional License

Commercial users must obtain an additional written license before obtaining, using, copying, modifying, or distributing the codes in this toolkit. Please contact Yu (Marco) Nie (y-nie@northwestern.edu) or Jun Xie (jun.xie@swjtu.edu.cn) to obtain a commercial use license.
      
Yu (Marco) Nie and Jun Xie reserve the right to the final interpretation of this license agreement.
      

# Core Teams
This package is written and maintained by the Open-TNM Core Team. The current members are:

  Yu (Marco) Nie   (Northwestern University)
  
  Jun Xie         (Southwest Jiaotong University)
  
  Zhandong Xu     (Southwest Jiaotong University)
  
  Liyang Feng     (Southwest Jiaotong University)
  
  Qianni Wang     (Northwestern University)



