# About the Algorithm

Combinatorial bilevel congestion pricing (CBCP), a variant of the discrete network design
problem, seeks to minimize the total travel time experienced by all travelers in a road network,
by strategically selecting toll locations and determining the corresponding charges. Conventional
wisdom suggests that these problems are intractable since they have to be formulated and solved
with a significant number of integer variables. Here, we devise a scalable local algorithm for the
CBCP problem that guarantees convergence to a Kuhn-Tucker-Karush point. Our approach is
novel in that it eliminates the use of integer variables altogether, instead introducing a cardinality
constraint that limits the number of toll locations to a user-specified upper bound. The resulting
bilevel program with the cardinality constraint is then transformed into a block-separable, singlelevel
optimization problem that can be solved efficiently after penalization and decomposition.
We are able to apply the algorithm to solve, in about 20 minutes, a CBCP instance with up to
3,000 links, of which hundreds can be tolled. To the best of our knowledge, no existing algorithm
can solve CBCP problems at such a scale while providing any assurance of convergence.

# How to Run the Algorithm
• Open the solution in ‘./CBCP/Source/TNA/TNA.sln’ using Visual Studio 2012.

• The main function is in ‘TNADriver.cpp’.

• Modify the parameters in the TestBCPCC function.

• Use "Ctrl + F5 " to compile and execute the code.

# How to Run Other Test Problems

• You can download more transportation test networks from https://github.com/bstabler/TransportationNetworks and put them in ‘./CBCP/Network/’.

• Modify the following parameters accordingly:

 (1) The network file name in the main() function in ‘TNADriver.cpp’;
 
 (2) Set a proper cardinality constraint variable "kappa" in the TAP_BCP::AlgorithmBCPCC() function in 'TNM_Algorithm.cpp';
 
 (3) Set a proper initial z value (saved in 'network->linkVector[i]->buffer[2]') in the TAP_BCP::AlgorithmBCPCC() function in 'TNM_Algorithm.cpp';


# Publication

Guo, Lei, Jiayang Li, Yu Marco Nie, and Jun Xie. "A Cardinality-Constrained Approach to Combinatorial Bilevel Congestion Pricing." arXiv preprint arXiv:2412.06482 (2024).
