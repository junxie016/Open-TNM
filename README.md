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

# License
Copyright (c) 2023 Yu (Marco) Nie and Jun Xie.  All rights reserved.

This open Toolkit of Network Modeling (open-TNM) is provided "as is," without warranty of any kind, express or implied.  
In no event shall the author or contributors be held liable for any damages arising in any way from the use of this toolkit.

Individuals, academic users, and commercial users are granted the free right to obtain, use,
copy, modify, and distribute these codes and their documentation, provided that all copies
or substantial portions of the codes must include the notice of copyright and this license
agreement, and must explicitly acknowledge that the Software or parts of it were used.

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.
      

# Core Teams
This package is written and maintained by the Open-TNM Core Team. The current members are:

  Yu (Marco) Nie   (Northwestern University)
  
  Jun Xie         (Southwest Jiaotong University)
  
  Zhandong Xu     (Southwest Jiaotong University)
  
  Liyang Feng     (Southwest Jiaotong University)
  
  Qianni Wang     (Northwestern University)



