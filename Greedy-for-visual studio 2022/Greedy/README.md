# About the Algorithm

The greedy algorithm is a path-based algorithm for the static user equilibrium traffic assignment problem, honored with the 2018 Stalla Dafermos Best Paper Award. Path-based algorithms
are generally considered less efficient than bush-based counterparts, such as Algorithm B, traffic assignment by
paired alternative segments (TAPAS), and iTAPAS, an improved version of TAPAS, because explicitly storing and manipulating
paths appears wasteful. However, our numerical experiments indicate that the greedy algorithm can outperform
TAPAS or iTAPAS by a wide margin. It shares the same Gauss-Seidel decomposition
scheme with existing path-based algorithms, and delivers a surprising performance, most likely due to its two main features.
First, it adopts a greedy method to solve the restricted subproblem defined on each origin–destination (O-D) pair. Second,
instead of sequentially visiting every O-D pair in each iteration, it introduces an intelligent scheme to determine which OD
pairs need more or less work. 

# How to Run the Algorithm
• Open the solution in ‘./Greedy/Source/TNA/TNA.sln’ using Visual Studio 2022.

• The main function is in ‘TNADriver.cpp’.

• Modify the parameters in the TestGreedy function.

• Use "Ctrl + F5 " to compile and execute the code.

# How to Run Other Test Problems

• You can download more transportation test networks from https://github.com/bstabler/TransportationNetworks and put them in ‘./Greedy/Network/’.

• Please note that different networks feature distinct 'Generalized Cost Weights'. If you aim to reproduce the solution reported in the network, ensure to adjust the parameters in the SetCostCoef() function accordingly.

# How to Cite

Jun Xie, Yu (Marco) Nie and Xiaobo Liu. An implementation of the greedy path-based algorithm for traffic assignment. https://github.com/junxie016/open-TNM/tree/main/Greedy. December, 28, 2023.

# Publication

Xie J, Nie Y, Liu X. A greedy path-based algorithm for traffic assignment. Transportation Research Record. 2018 Dec;2672(48):36-44.
