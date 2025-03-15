# About the Algorithm

iTAPAS is
an improved TAPAS algorithm  that enhances
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

• Enable the proportionality computation by setting "itapas->DoProportionality = true".

• Use "Ctrl + F5 " to compile and execute the code.

# Convergence performance of iTAPAS

The convergence of iTAPAS for computing the Chicago Regional running on a personal laptop is as follows: 

|Iter|OFV|ConvIndc|Time(seconds)|
|:----|:----|:----|:----|
|0|69097772.83|1000|25.507|
|1|27400896.84|0.1108|87.208|
|2|26329660.14|0.0381|144.273|
|3|26031597.05|0.0131|182.073|
|4|25920564.43|0.0048|212.813|
|5|25876155.47|0.0018|239.845|
|6|25857749.98|0.0008|264.447|
|7|25850494.85|0.0003|287.494|
|8|25847588.73|0.0001|306.754|
|9|25846423.02|0.0001|323.923|
|10|25845995.54|0|341.054|
|11|25845835.31|8.70E-06|357.778|
|12|25845775.56|4.94E-06|374.685|
|13|25845710.26|6.09E-06|392.952|
|14|25845700.09|1.27E-06|414.415|
|15|25845695.25|5.85E-07|438.017|
|16|25845693.06|3.56E-07|459.752|
|17|25845691.6|2.52E-07|482.42|
|18|25845690.87|1.64E-07|504.94|
|19|25845690.34|1.48E-07|526.762|
|20|25845689.99|1.03E-07|550.042|
|21|25845689.64|8.45E-08|571.477|
|22|25845689.43|5.37E-08|589.717|
|23|25845689.34|1.66E-08|608.485|
|24|25845689.33|1.74E-09|630.06|
|25|25845689.33|7.56E-10|651.87|
|26|25845689.33|5.80E-10|675.714|
|27|25845689.33|3.96E-10|698.573|
|28|25845689.33|2.84E-10|722.224|
|29|25845689.33|9.66E-11|746.453|
|30|25845689.33|3.81E-10|767.764|
|31|25845689.33|4.52E-11|786.29|
|32|25845689.33|7.29E-12|805.091|
|33|25845689.33|1.63E-12|825.379|
|34|25845689.33|5.39E-13|852.073|



# How to Run Other Test Problems

• You can download more transportation test networks from https://github.com/bstabler/TransportationNetworks and put them in ‘./iTAPAS/Network/’.

• Please note that different networks feature distinct 'Generalized Cost Weights'. If you aim to reproduce the solution reported in the network, ensure to adjust the parameters in the SetCostCoef() function accordingly.

# How to Cite

Jun Xie, Chi Xie and Yu (Marco) Nie. An implementation of the iTAPAS algorithm for traffic assignment. https://github.com/junxie016/open-TNM/tree/main/iTAPAS. April, 4, 2024.

# Publication

Xie, J. and Xie, C., 2016. New insights and improvements of using paired alternative segments for traffic assignment. Transportation Research Part B: Methodological, 93, pp.406-424.


