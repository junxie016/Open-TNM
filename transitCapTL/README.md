# Transit Equilibrium Assignment Problem
## About the Algorithm
The algorithm is a hyeprpath-based algorithm designed for solving the Transit Equilibrium Assignment Problem (TEAP). It can efficiently solve TEAP to satisfactory precision even in large-scale networks. We tailor two modified Newton-type hyperpath-based algorithms, namely the greedy and gradient projection algorithms with the adaptive inner loop, referred to as Greedy-AIL and GP-AIL respectively. It consists of two primary components: column generation and adaptive inner loop. The former one utilises the label-setting shortest hyperpath algorithm to include new hyperpaths as needed, whereas the latter is applied to perform adaptive equilibration by solving hyperpath-based subproblems. 

## How to Run the Algorithm
•	Open the solution in ‘./transitCapTL/Source/Solution/mySolution.sln’ using Visual Studio 2012.

•	The main function is located in ‘TEAPDriver.cpp’ in the ‘transitDriver’ project.

•	Four test networks are provided, including: Gentile, Sious-Falls, Winnipeg and Shenzhen Center transit networks. The test networks need to run the corresponding function, for example, the corresponding function for Gentile network is "GentileNet()".

•	Modify the parameters in the different network functions.

•	Use "Ctrl + F5 " to compile and execute the code.

## Publication
Xu, Z., J. Xie, X. Liu, and Y. M. Nie. 2020. Hyperpath-Based Algorithms for the Transit Equilibrium Assignment Problem. Transportation Research Part E: Logistics and Transportation Review 143:102102. https://doi.org/10.1016/j.tre.2020.102102.

# Capacitated Transit Equilibrium Assignment Problem
## About the Algorithm
The transitCapTL algorithm is a hyeprpath-based algorithm designed for solving the Capacitated Transit Equilibrium Assignment Problem (CTEAP). It can efficiently solve CTEAP to satisfactory precision even in large-scale networks. The algorithm consists of two parts:
The first part (main problem) solves the CTEAP through the algorithmic frameworks based on the method of multipliers (MOM) or the internal penalty functions (IPF) method.
The second part (subproblem) solves the uncapacitated TEAP subproblem by the hyperpath-based Greedy or GP algorithms.

### How to Run the Algorithm
•	Open the solution in ‘./transitCapTL/Source/Solution/mySolution.sln’ using Visual Studio 2012.

•	The main function is located in ‘TEAPDriver.cpp’ in the ‘transitDriver’ project.

•	Four test networks are provided, including: Gentile, Sious-Falls, Winnipeg and Shenzhen Center transit networks. The test networks need to run the corresponding function, for example, the corresponding function for Gentile network is "CapGentileNet()".

•	Modify the parameters in the different network functions.

•	Use "Ctrl + F5 " to compile and execute the code.

## How to Construct the Walking Link between O-D pairs
•	When testing the CTEAP on the Gentile transit network, run CreateWalkLink() within the function“PCTAE_Solver(PCTAE_algorithm alg)”.

•	When testing the CTEAP on the Sious-Falls, Winnipeg, and Shenzhen Center transit networks, run CreateODLink() within the function “PCTAE_Solver(PCTAE_algorithm alg)”.

## Publication
Liang, W., Xu, Z., Xie, J., Liu, X., and Feng, Y. 2024. An efficient hyperpath-based algorithm for the capacitated transit equilibrium assignment problem. Transportmetrica A: Transport Science. https://doi.org/10.1080/23249935.2024.2326819

