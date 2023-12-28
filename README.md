# open-TNM
The open-TNM package is an open-source Toolkit of Network Modeling package written in C++ for solving different transportation network problems, including path-based and origin-based algorithms for the user-equilibrium traffic assignment problems,  traffic assignment problems with side constraints, maximum entropy user equilibrium traffic assignment problems, frequency-based transit assignment problems, continuous bi-criteria traffic assignment problems and so on. This package is written and maintained by Yu (Marco) Nie from Northwestern University and Jun Xie from Southwest Jiaotong University.

# Class Structures
The network classes are defined as TNM SNET classes in TNM (‘TNM NET.h’ ). The main network elements include:

• TNM SLINK: It is a basic class used to represent network links. Derived classes like TNM BPRLK support evaluating link travel time using a BPR-type link performance function. Key attributes include tail node, head node, volume, fft (free flow time), capacity, toll, cost (travel time), etc.

• TNM SNODE: This class represents network nodes which include attributes like forwStar (in-node links), backStar (out-node links), pathElem (shortest path element to the node), etc.

• TNM SORIGIN: All origin-destination (O-D) pairs are organized using origins as roots, i.e., each origin stores a vector of destinations associated with positive demand.

• TNM SDEST: The dest (destination node), assDemand (O-D pair demand), origin (the corresponding origin node), pathSet (paths used by each O-D pair), pathSetbound (path time equivalence of money (TEM) boundaries) are stored in TNM SDEST object.

• TNM SPATH: The path (the collection of links that make up the path) and some path cost evaluation functions are defined in the TNM SPATH class.

• SCANLIST: Scanlist is introduced into TNM to accommodate different labeling shortest path problem algorithms.

The buffer attributes of each class serve as repositories for data pertinent to the object, essential for algorithmic calculations. The traffic assignment algorithms are derived from the TNM TAP algorithm class, as defined in the 'TNM Algorithm.h' file.
This head file also encompasses general attributes and processes crucial for executing traffic assignment algorithms, including convergence accuracy, maximum iteration, and objective function values. In the test functions found in 'xxDriver.cpp,' we instantiate the algorithm, specifying relevant algorithm and network attributes. Subsequently, the network is constructed using the Build() method.

The Solve() process involves executing Preprocess(), Initialization(), Mainloop(), and Postprocess() successively. It also checks Terminate() when necessary. Finally, the Report() method is utilized to output iteration and equilibrium results.

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
This package is written and maintained by the open-TNM Core Team. The current members are:

  Yu(Marco) Nie   Northwestern University
  
  Jun Xie         Southwest Jiaotong University
  
  Zhandong Xu     Southwest Jiaotong University
  
  Liyang Feng     Southwest Jiaotong University
  
  Qianni Wang     Northwestern University



