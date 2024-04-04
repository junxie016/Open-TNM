# About the Algorithm

iTAPAS is
an improved TAPAS algorithm (termed iTAPAS) that enhances
the original TAPAS mainly in two aspects: (1) a new PAS
identification method is proposed; (2) each PAS is only
associated with one origin. iTAPAS not only accelerates the
solution convergence performance by improving the efficiency
of the PAS identification and flow shift operations, but also
simplifies the algorithm implementation by removing the PAS
matching and branch shift steps. 

# How to Run the Algorithm
• Open the solution in ‘./iTAPAS/Source/TNA/TNA.sln’ using Visual Studio 2019.

• The main function is in ‘TNADriver.cpp’.

• Modify the parameters in the TestiTAPAS function.

• Use "Ctrl + F5 " to compile and execute the code.

# How to Run Other Test Problems

• You can download more transportation test networks from https://github.com/bstabler/TransportationNetworks and put them in ‘./iTAPAS/Network/’.

• Please note that different networks feature distinct 'Generalized Cost Weights'. If you aim to reproduce the solution reported in the network, ensure to adjust the parameters in the SetCostCoef() function accordingly.

# How to Cite

Jun Xie, Chi Xie, Yu (Marco) Nie and Xiaobo Liu. An implementation of iTAPAS algorithm for traffic assignment. https://github.com/junxie016/open-TNM/tree/main/iTAPAS. April, 4, 2024.

# Publication

Xie, J. and Xie, C., 2016. New insights and improvements of using paired alternative segments for traffic assignment. Transportation Research Part B: Methodological, 93, pp.406-424.
