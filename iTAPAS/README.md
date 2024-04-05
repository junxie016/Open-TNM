# About the Algorithm

iTAPAS is
an improved TAPAS algorithm  that enhances
the original TAPAS mainly in two aspects: (1) a new PAS
the identification method is proposed; (2) each PAS is only
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
  Iter               OFV          ConvIndc              Time (seconds)
       0     69097772.8321         1000.0000           25.5070
       1     27400896.8365            0.1108           87.2080
       2     26329660.1413            0.0381          144.2730
       3     26031597.0469            0.0131          182.0730
       4     25920564.4273            0.0048          212.8130
       5     25876155.4720            0.0018          239.8450
       6     25857749.9814            0.0008          264.4470
       7     25850494.8469            0.0003          287.4940
       8     25847588.7255            0.0001          306.7540
       9     25846423.0202            0.0001          323.9230
      10     25845995.5388            0.0000          341.0540
      11     25845835.3145        8.7030e-06          357.7780
      12     25845775.5627        4.9408e-06          374.6850
      13     25845710.2636        6.0866e-06          392.9520
      14     25845700.0853        1.2653e-06          414.4150
      15     25845695.2498        5.8505e-07          438.0170
      16     25845693.0611        3.5607e-07          459.7520
      17     25845691.5976        2.5154e-07          482.4200
      18     25845690.8662        1.6447e-07          504.9400
      19     25845690.3371        1.4782e-07          526.7620
      20     25845689.9868        1.0314e-07          550.0420
      21     25845689.6410        8.4507e-08          571.4770
      22     25845689.4339        5.3670e-08          589.7170
      23     25845689.3446        1.6644e-08          608.4850
      24     25845689.3332        1.7430e-09          630.0600
      25     25845689.3328        7.5605e-10          651.8700
      26     25845689.3326        5.8018e-10          675.7140
      27     25845689.3325        3.9647e-10          698.5730
      28     25845689.3325        2.8430e-10          722.2240
      29     25845689.3325        9.6643e-11          746.4530
      30     25845689.3325        3.8116e-10          767.7640
      31     25845689.3325        4.5173e-11          786.2900
      32     25845689.3325        7.2921e-12          805.0910
      33     25845689.3325        1.6349e-12          825.3790
      34     25845689.3325        5.3863e-13          852.0730


# How to Run Other Test Problems

• You can download more transportation test networks from https://github.com/bstabler/TransportationNetworks and put them in ‘./iTAPAS/Network/’.

• Please note that different networks feature distinct 'Generalized Cost Weights'. If you aim to reproduce the solution reported in the network, ensure to adjust the parameters in the SetCostCoef() function accordingly.

# How to Cite

Jun Xie, Chi Xie and Yu (Marco) Nie. An implementation of the iTAPAS algorithm for traffic assignment. https://github.com/junxie016/open-TNM/tree/main/iTAPAS. April, 4, 2024.

# Publication

Xie, J. and Xie, C., 2016. New insights and improvements of using paired alternative segments for traffic assignment. Transportation Research Part B: Methodological, 93, pp.406-424.
