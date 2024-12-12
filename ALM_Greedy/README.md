# About the Algorithm

The ALM-Greedy algorithm is a path-based algorithm designed for solving the Capacitated Traffic Assignment Problem (CTAP). 
It can efficiently solve CTAP to satisfactory precision even in large-scale networks. 
This efficiency can be attributed to two features. 
First, it applies the [Greedy solver](https://github.com/junxie016/open-TNM/tree/main/Greedy) to address the subproblems dualized by the the augmented Lagrangian multiplier method.
Second, rather than aiming for precise solutions to each subproblem, it introduces an adaptive precision criterion for subproblems. 
This approach maintains a balance between precision and efficiency.

# How to Run the Algorithm
• Open the solution in ‘./ALM_Greedy/Source/TNA/TNA.sln’ using Visual Studio 2019.

• The main function is in ‘TNADriver.cpp’.

• Modify the parameters in the TestALM_Greedy function.

• Use "Ctrl + F5 " to compile and execute the code.

# How to Run Other Test Problems

• You can download more transportation test networks from https://github.com/bstabler/TransportationNetworks and put them in ‘./ALM_Greedy/Network/’.

• Since some of the networks in bstabler's GitHub do not have feasible CTAP solutions, you should increase the link capacity or decrease the demand to ensure a non-empty solution set.

• We provide an implementation of the CUP algorithm ([Nie et al. 2004](https://doi.org/10.1016/S0191-2615(03)00010-9)) to increase the link capacity. You can try it by running the TestCUP function in ‘TNADriver.cpp’, while commenting out the TestALM_Greedy function.

# How to Cite

Liyang Feng, Jun Xie, Yu (Marco) Nie and Xiaobo Liu. An implementation of the ALM-Greedy algorithm for capacitated traffic assignment. https://github.com/junxie016/open-TNM/tree/main/ALM_Greedy. Accessed Month, Day, Year.

# Publication

Feng, L., Xie, J., Nie, Y., & Liu, X. (2020). Efficient algorithm for the traffic assignment problem with side constraints. Transportation research record, 2674(4), 129-139.
