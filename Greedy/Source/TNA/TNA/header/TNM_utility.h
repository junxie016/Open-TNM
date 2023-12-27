#pragma once
#include "stdafx.h"

template <class T>
bool TNM_FromString(T& t, 
					const std::string& s, 
					std::ios_base& (*f)(std::ios_base&))
						{
	std::istringstream iss(s);
	return !(iss >> f >> t).fail();
	};

template <class T>
 string to_string(T t, std::ios_base & (*f)(std::ios_base&))
{
  std::ostringstream oss;
  oss << f << t;
  return oss.str();
}

TNM_EXT_CLASS bool     TNM_OpenInFile(ifstream &, const string &);    //open a ascii file to read command
TNM_EXT_CLASS bool     TNM_OpenOutFile(ofstream &, const string &);   //open a ascii file to write command
TNM_EXT_CLASS ifstream &TNM_SkipString(ifstream &, int count); //frequently used function in io function.
TNM_EXT_CLASS void		TNM_GetWordsFromLine(const string &pstr, vector<string> &words, const char dimi, const char exception = ' ');
TNM_EXT_CLASS void		TNM_GetWordsFromLine(string &pstr, vector<string> &words);
TNM_EXT_CLASS string	GetSourcePath();

/*============================================================================================
            parameterized stream operator
  ===========================================================================================*/
struct TNM_EXT_CLASS TNM_FloatFormat 
{
TNM_FloatFormat(const floatType x, int w, int d); // constructor
TNM_FloatFormat(const floatType x, int w);  // constructor 
TNM_FloatFormat(const floatType x);

ostream& print(ostream&) const; // output
// some data
floatType num;
int width;
int digit;
static int sWidth;
static int sDigit;
public:
	static void SetFormat(int w, int d) {sWidth = w; sDigit = d;} 
	static int  GetWidth() {return sWidth;}
};

TNM_EXT_CLASS ostream& operator <<(ostream& os, const TNM_FloatFormat& m); 

struct TNM_EXT_CLASS TNM_IntFormat 
{
TNM_IntFormat(const int x, int w); // constructor
TNM_IntFormat(const int x);
ostream& print(ostream&) const; // output
// some data
floatType num;
int width;
static int sWidth;
public:
	static void SetFormat(int w) {sWidth = w;}
	static int  GetWidth() {return sWidth;}
};

TNM_EXT_CLASS ostream& operator <<(ostream& os, const TNM_IntFormat& m); 

typedef std::set<int, std::less<int> > INT_SET;
class  TNM_EXT_CLASS IDManager
{
public:
	IDManager();
	~IDManager(){;}
private:
	INT_SET idSet;
	INT_SET idCache;
public:
	void RegisterID(int id);
	int  SelectANewID();
	void UnRegisterID(int id);
	bool FindID(int id);
	void Reset() {idSet.clear();idCache.clear();}
};

//MyDateTime is a wraper of struct tm defined in time.h. It provides an easyway to convert between string and struct tm. 

class TNM_EXT_CLASS TNM_MyDateTime
{
public:
    TNM_MyDateTime();    
    TNM_MyDateTime(TNM_MyDateTime &rhs); //copy constructor.
    virtual ~TNM_MyDateTime();    
    bool InitializeCompact(const string &inf, bool date = true); //format:    20120212 for date, or 032355 for time
    bool InitializeExpand(const string &inf, bool date = true);//format: 2012-02-12 for date or 03:24:55 for time;
    bool InitializeCompactAll(const string &inf);  //formaat: 20120212032455.
    bool InitializeExpandAll(const string &inf); //initialize from the following format: 2012-02-12 03:24:55
    bool Initialize(time_t t); //initialize directly from a time_t object;
    bool ChangTime(long seconds);  //change time by seconds;
    long DiffTime(TNM_MyDateTime *rhs); //current time - given time, and measured in seconds. 
    int  GetYear() {return m_tm.tm_year + 1900;}
    int  GetMonth() {return m_tm.tm_mon + 1;}
    int  GetMonthDay() {return m_tm.tm_mday;}
    int  GetWeekDay() {return m_tm.tm_wday;}
    int  GetYearDay() {return m_tm.tm_yday;}
    int  GetHour() {return m_tm.tm_hour;}
    int  GetMin() {return m_tm.tm_min;}
    int  GetSec() {return m_tm.tm_sec;}
    int  GetSecSinceCurrentDay() {return GetHour()* 3600 + GetMin()*60 + GetSec();}
    bool IsDayLight() {return m_tm.tm_isdst!=0;}
    string GetDateString(bool shortformat = true);
    string GetDateStringMonthFirst();
    string GetTimeString(bool shortformat = true);
    string GetDateTimeString(bool shortformat = true);
    string GetCompactString(bool dateonly = true);
    struct tm* GetTime() {return &m_tm;}
    inline time_t  GetUnixTime() {return mktime(&m_tm);}
   // trim from both ends
	static inline string &trim(string &s) 
	{
        return ltrim(rtrim(s));
	}

	// trim from start
	static inline string &ltrim(string &s) 
	{
        size_t endpos = s.find_last_not_of(" ");
        if( string::npos != endpos )
        {
            s = s.substr( 0, endpos+1 );
        }
        return s;        
	}

	// trim from end
	static inline std::string &rtrim(std::string &s) 
	{

        size_t startpos = s.find_first_not_of(" ");
        if( string::npos != startpos )
        {
            s = s.substr( startpos );    
        }
        return s;

	}

protected:
		struct tm     m_tm;     //time structure;
};


class TNM_EXT_CLASS TNM_Position
{
public:
	TNM_Position(floatType lat, floatType lon) {SetPos(lat, lon);}
	TNM_Position() {SetPos(0,0);}
	void SetPos(floatType lat, floatType lon) {m_lat = lat, m_lon = lon;}
	floatType GetLatitude() const {return m_lat;}
	floatType GetLongitude() const {return m_lon;}
    floatType GetDist(TNM_Position *pos, char unit = 'M'/*miles*/);
    static floatType deg2rad(floatType deg) 
    {
		return (deg * PI / 180);
    }
    static floatType rad2deg(floatType rad) 
    {
        return (rad * 180 / PI);
    }
	virtual ~TNM_Position() {;}
protected:
	floatType m_lat;
	floatType m_lon;
};