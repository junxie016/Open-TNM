//#pragma once
#include <string>
#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>

#include <cmath>
#include <limits.h>
#include <CString>
#include <iterator>
#include <iomanip>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <stack>
#include <queue>
#include <list>   

using namespace std;

//typedef long double floatType;
//typedef char   tinyInt;  // 0 ~ 255, one byte
//typedef int    smallInt; // 0 ~ 65,535, two byte
//typedef long   largeInt; // 0 ~ 4, 294,967,295, four byte

#pragma once
//const double	POS_INF_FLOAT = 1e15;
//const double	PI = 3.14159265358979323846;


#ifdef TRANSITNET_EXPORTS  
 
#define PTNet_API _declspec(dllexport) //表明标有此宏定义的函数和类是dll文件的导出函数和类，是dll文件的对外接口
 
#else
 
#define PTNet_API _declspec(dllimport) //表明标有此宏定义的函数和类的定义在dll文件中
 
#endif


//#include "..\..\include\Shapelib\shpplus.h" 

