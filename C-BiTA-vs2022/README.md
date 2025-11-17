# **User Manual for the C-BiTA Project**

This document is intended to help readers quickly understand and make effective use of our algorithms.

 Hardware and Software Requirements:
- **Operating System**: Windows 11 (64-bit)  
- **Compiler/IDE**: Microsoft Visual Studio 2022 (v143), ISO C++14 standard  
- **CPU**: Experiments were conducted on on 12th Gen Intel(R) Core(TM) i7-1260P CPU @ 2.10 GHz.  A modern 64-bit processor with at least 4 cores is recommended.  
- **RAM**: The laptop used was equipped with 16 GB RAM.  For typical problem instances, at least 8 GB RAM is recommended.  
- **Additional Software**: Python 3.x with NumPy (for scientific computing); Matplotlib, Seaborn (for reproducing figures); Pandas (for data analysis and manipulation).

## 01 How to Run the Source Code
 
 -  Open the solution file located at `..\C-BiTA\Source\Solution\mySolution.sln` using Visual Studio 2022. 
 
 -  Locate the `main` function in *trafficDriver.cpp*. The *trafficDriver.cpp* file is included in the source files of the `trafficDriver` solution within `mySolution`.
 
 -  Run the SBA (or FW) algorithm by invoking the `TestSBA()` (or `TestT2theta()`) function. An example of the call is: `TestSBA(''..\\..\\..\\data\\TestNetwork\\sf\\'', sf);`

 -  All networks tested in our experiments are provided in `..\C-BiTA\data\TestNetwork`. If users want to test their own networks, additional networks can be downloaded from [[TransportationNetworks](https://github.com/bstabler/TransportationNetworks)]  and placed in the `data` folder. To ensure the proper execution of the algorithm, each network must at least include the following three network files:
	 - `name_trips.tntp`: A file for OD demand
	 - `name_tol.tntp`: A file for constant link tolls.
	 - `name_net.tntp`: A file for network link parameters.

 - Use `Ctrl + F5` to compile and execute the code.

 -  Output files will be generated in the same folder as the inputs. 
    The first two types of files are produced by both algorithms, while the last three are specific to SBA:

	 - `name_alg.ite`: Iterative convergence results, including iteration number (*Iter*), objective function value (*OFV*), relative gap (*ConvIndc*), and elapsed time (*Time*). 
	 - `name_alg.lfp`: Final link flow pattern, reported in the column *Flow*.
	 - `name_alg.pfp`: Final path flow pattern, showing the geometry of used paths and their associated flows.
	 - `name_alg.info`: Final number of efficient paths for each OD pair, along with the partitioning of the VOT distribution.
	 - `name_alg.tnum`: Number of shortest path trees per iteration (one line per iteration, one number per origin).



## 02 How to Run the .exe Console
In addition to the source code compilation method, we also offer a way to quickly utilize and test the algorithm via an interactive console. The following steps detail the process:

Open and run the execution file: `..\C-BiTA\exe concole\trafficDriver.exe`. Follow the instructions and input the network file path, network name, and the algorithm name.  If you place the C-BiTA folder in the C drive, you'll be able to find the sf network files at `C:\C-BiTA\data\TestNetwork\sf\`. Accordingly, an appropriate interactive process using the exe console is as follows:

\begin{quote}
Please input the path of your network files: (For example, D:\Data\network\sf\)

> C:\C-BiTA\data\TestNetwork\sf\

Please input the network name: (For example sf, Anaheim, Barcelona, Winnipeg, chicagosketch, chi)

> sf

Please input the algorithm name you want to test: (SBA or FW)

> SBA

\end{quote}

The executables compiled with Visual Studio 2022 (v143 toolset) depend on the 
Microsoft Visual C++ Redistributable for Visual Studio 2022.  On most Windows 10/11 systems this package is already installed. If, however,  you encounter an error message indicating that a Microsoft Visual C++ runtime DLL (e.g., `MSVCP140.dll`, `VCRUNTIME140.dll`, or `ucrtbase.dll`) is missing, please download and install the latest supported Visual C++ Redistributable from the official Microsoft website:
https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist




## 03 How to Reproduce the Numerical Experiments
All raw results that are used in the numerical experiments of the paper are provided in the folder `..\C-BiTA\results\raw_results`. The subfolders correspond to different sections of the paper as follows:

 - `convergence_test`: results for Section 6.1,  
 - `num_of_tolled_links`: results for Section 6.2,  
 - `sens_toll_mag`, `sens_congestion`, `sens_dist`: results for Sections 6.3.1, 6.3.2, and 6.3.3, respectively.  


In addition, Python scripts with the same names (located under 
`..\C-BiTA\results`) can be used to process the raw results and generate the figures included in the paper. 
The figures will be automatically saved in the corresponding subfolder under 
`..\C-BiTA\results\figures`.

All input files for reproducing the experiments through the source code are stored in 
`..\C-BiTA\data\TestNetwork`.  
Each subfolder (named after the corresponding network) contains the input data used in Section 6.1.  
The `cs` subfolder also contains the input files for the experiments in Sections 6.2--6.3, organized in its subdirectories. 

To reproduce these experiments through the source code:

 - Open the solution file located at `..\C-BiTA\Source\mySolution.sln` using Visual Studio 2022. 
 - Locate the `main` function in *trafficDriver.cpp*.
 - Uncomment the line following the `base_file_path` string variable that corresponds to your target experiment (note that scripts are organized according to the paper's sections), then compile and execute using `Ctrl + F5`.
 - The output results will be generated in the same folder as the input files and can be opened, inspected, renamed, or relocated as required.