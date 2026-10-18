#include "header\stdafx.h"
#include <direct.h>
#include <atlstr.h>

int TNM_FloatFormat::sDigit   = 2;
int TNM_FloatFormat::sWidth   = 10;
int TNM_IntFormat::sWidth     = 8;

ifstream &TNM_SkipString(ifstream &in, int count)
{
	string skip;
	for (int i = 0;i<count;i++)
		in>>skip;
	return in;
}

void TNM_GetWordsFromLine(const string &line, vector<string> &words, const char dim, const char exception)
{
    int count = 0;
    for (int i = 0; i < line.size(); i++)
        if (line[i] == dim) count++;
    //clase worsd
    if(!words.empty()) words.clear();
    string curWord;
    istringstream pstr(line); 
    if(exception == ' ')
    {  
        //first, we check how many dim are there. the number of expected columns should be count + 1;
        
        while(getline(pstr, curWord, dim))
        {
			//first delete all leading space. 
			const int strBegin = curWord.find_first_not_of(" \t");
			if (strBegin == std::string::npos)
				curWord = " "; // no content

			const int strEnd = curWord.find_last_not_of(" \t");
			const int strRange = strEnd - strBegin + 1;
          words.push_back(curWord.substr(strBegin, strRange));
        }
        
    }
    else
    {
        bool readbefore = false;
        while(getline(pstr, curWord,exception))
        {
            //cout<<curWord<<endl;
           // if(curWord[0] == dim) curWord[0] = ' '; //if the first of the word is a dim, we replace it. 
            if(!readbefore)
            {
                readbefore = true;
                if(!curWord.empty())
                {
                    istringstream tstr(curWord);
                    string pword;
                    //we need to check if the first word is empty, it is artificial. 
                    if(getline(tstr, pword, dim))
                    {
                        if(!pword.empty())
                        {
                            words.push_back(pword);
                           // cout<<pword<<endl;
                        }
                    }
                    while(getline(tstr, pword, dim))
                    {
                        words.push_back(pword);
                        //cout<<pword<<endl;
                    }
                }

            }
            else
            {
                words.push_back(curWord);
                //cout<<curWord<<endl;
                 for (int i = 0; i < curWord.size(); i++)
                    if (curWord[i] == dim) count--;
                
                readbefore = false;
            }
        }

    }
   if(words.size() == count) //this could happen if nohign is written after the last dim. in this case, we simply throw in an empty string to avoid problems
            words.push_back("  ");
 //  for(int i = 0;i<words.size();i++) cout<<words[i]<<endl;
}

void TNM_GetWordsFromLine(string &pstr, vector<string> &words)
{
	if(!words.empty()) words.clear();
	istringstream x(pstr.c_str());
	copy( istream_iterator< string >( x ), istream_iterator<string>(),  back_inserter( words ) ); 
}

string GetSourcePath()
{
	CString a = getcwd(NULL, 0);

	a.Replace("\\","\\\\");
	string b=a.GetString();
	for (int i=1;i<=2;i++) b=b.substr(0,b.find_last_of("\\\\")-1);
	return b;
}



floatType TNM_Position::GetDist(TNM_Position *pos, char unit)
{
    floatType theta, dist;
    theta = m_lon - pos->GetLongitude();
    dist = sin(deg2rad(m_lat)) * sin(deg2rad(pos->GetLatitude())) + cos(deg2rad(m_lat)) * cos(deg2rad(pos->GetLatitude())) * cos(deg2rad(theta));
	if (dist<-1||dist>1)	dist = 0;
	else dist = acos(dist);
    
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    switch(unit) {
    case 'M':
      break;
    case 'K':
      dist = dist * 1.609344;
      break;
    case 'N':
      dist = dist * 0.8684;
      break;
  }
  return (dist);
}

ostream& operator <<(ostream& os, const TNM_FloatFormat& m)
{
	return m.print(os);
}

TNM_FloatFormat::TNM_FloatFormat(const floatType x, int w, int d)
{
	num   = x;
	width = w;
	digit = d;
}
TNM_FloatFormat::TNM_FloatFormat(const floatType x, int w)
{
	num   = x;
	width = w;
	digit = sDigit;
}
TNM_FloatFormat::TNM_FloatFormat(const floatType x)
{
	num   = x;
	width = sWidth;
	digit = sDigit;
}

ostream& TNM_FloatFormat::print(ostream& os) const {
// do something to the stream to output what you need
// in a special way
	int ad;
	if (num >=0)  ad = 2;// 1 digit point + 1 space before it.
	else          ad = 3;// 1 digit point + 1 space + 1 minus.
	//cout<<"digit = "<<digit<<" width = "<<width<<endl;
	//getchar();
	//cout<<"\t num = "<<num<<endl;
	if (width - ad - digit >0)
	{
		//cout<<" the upper limit "<<pow(10.0, 1.0*(width-ad-digit))<<endl;
		if(fabs(num)<pow(10.0, 1.0*(width-ad-digit)))
		{
			//cout<<" output fixed: ";
			if(fabs(num) > pow(10.0, -digit-1.0)) os<<setw(width)<<setprecision(digit)<<setiosflags(ios::fixed)<<num;
			else     	
			{
			//	cout<<"width = "<<width<<" digit = "<<digit<<endl;
				os.unsetf(ios::fixed);
				os<<setw(width)<<setprecision(digit)<<setiosflags(ios::scientific)<<num;
				//os<<setw(width)<<num;
				os.unsetf(ios::scientific);
			}
		}
		else
		{
		//	cout<<" now try another left width = "<<width - digit - ad -5<<endl;
			if (width - digit - 5 - ad >0)
			{
				//cout<<" output scientific: ";
				os.unsetf(ios::fixed);
				os<<setw(width)<<setprecision(digit)<<setiosflags(ios::scientific)<<num;
				os.unsetf(ios::scientific);
			}
			else
				os<<" "<<setprecision(digit)<<setiosflags(ios::fixed)<<num;
		}
	}
	else
	{
		    if(fabs(num) > pow(10.0, -digit-1.0)) os<<" "<<setprecision(digit)<<setiosflags(ios::fixed)<<num;
			else     	
			{
				os.unsetf(ios::fixed);
				os<<setw(width)<<setprecision(digit)<<setiosflags(ios::scientific)<<num;
				os.unsetf(ios::scientific);
			}
			
	}
	return os;
}


ostream& operator <<(ostream& os, const TNM_IntFormat& m)
{
return m.print(os);
}

TNM_IntFormat::TNM_IntFormat(const int x, int w)
{
	num   = x;
	width = w;
}


TNM_IntFormat::TNM_IntFormat(const int x)
{
	num   = x;
	width = sWidth;
}
ostream& TNM_IntFormat::print(ostream& os) const {
// do something to the stream to output what you need
// in a special way
	int ad;
	if (num >=0)  ad = 1;// 1 digit point + 1 space before it.
	else          ad = 2;// 1 digit point + 1 space + 1 minus.
	//cout<<"digit = "<<digit<<" width = "<<width<<endl;
	//getchar();
	if (width - ad >0)
	{
	//	cout<<" the upper limit "<<pow(10.0, 1.0*(width-ad-digit))<<endl;
		if(fabs(num)<pow(10.0, 1.0*(width-ad)))
		{

			os<<setw(width)<<setiosflags(ios::fixed)<<setprecision(0)<<num;
		}
		else
		{
			os<<" "<<setiosflags(ios::fixed)<<setprecision(0)<<num;
		}
	}
	else
	{
			os<<" "<<setiosflags(ios::fixed)<<setprecision(0)<<num;
	}
	return os;
}


bool TNM_OpenInFile(ifstream &in, const string &file)
{
	in.open(file.c_str(),ios::in);
	if(!in) 
	{
		cout<<"\n\tCannot open file "<<file<<" to read."<<endl;
		return false;
	}
	return true;
}

bool TNM_OpenOutFile(ofstream &out, const string &file)
{
	out.open(file.c_str(),ios::out);
	if(!out) 
	{
		cout<<"\n\tCannot open file "<<file<<" to write."<<endl;
		return false;
	}
	return true;
}



//implementation of TNM_MyDate, by default the time is the current system time. 
TNM_MyDateTime::TNM_MyDateTime()
{
   time_t ct = time(NULL);
   Initialize(ct);   
}


TNM_MyDateTime::TNM_MyDateTime(TNM_MyDateTime &rhs)
{
    struct tm* ptm = rhs.GetTime();
    m_tm = *ptm;

}


TNM_MyDateTime::~TNM_MyDateTime()
{
}


bool TNM_MyDateTime::ChangTime(long seconds)
{
    time_t pt = mktime(&m_tm);
    pt+= seconds;
    return Initialize(pt);
    
}

long TNM_MyDateTime::DiffTime(TNM_MyDateTime *rhs)
{
    return GetUnixTime() - rhs->GetUnixTime();
}
bool TNM_MyDateTime::Initialize(time_t t)
{    
    if(localtime_s(&m_tm, &t) !=0)   
    {
        cout<<"\tFailed to initialize MyDateTime object."<<endl;
        return false;
    }
    else    return true;
    
}

bool TNM_MyDateTime::InitializeCompactAll(const string &inf)
{
    //string inf(str);
   // trim(inf);
    if(inf.size()!=14)
    {
        cout<<"\tinvalid format: 14 digit expected, "<<inf.size()<<" found"<<endl;
        return false;
    }
    else
    {
        string string1 = inf.substr(0,8);
        string string2 = inf.substr(8, 6);
        if(!InitializeCompact(string1, true)) return false;
        return InitializeCompact(string2, false);
    }
}
bool TNM_MyDateTime::InitializeExpandAll(const string &inf)
{
    //string inf(str);
    //trim(inf);
    if(inf.size()!=19 && inf.size()!=18) //allows the format 06:00:00 and 6:00:00.
    {
        cout<<"\tinvalide format: 18 or 19 digit expected, "<<inf.size()<<" found"<<endl;
        return false;
    }
    else
    {
        string string1 = inf.substr(0,10);
        //cout<<string1<<endl;
        string string2 = inf.substr(11, inf.size() - 11);//
        if(string2.size() == 7) string2 = "0" + string2;
       // cout<<string2<<endl;
        if(!InitializeExpand(string1, true)) return false;
        return InitializeExpand(string2, false);
    }
}

string TNM_MyDateTime::GetDateStringMonthFirst()
{
    char buffer [12];
    strftime (buffer,12,"%m/%d/%Y",&m_tm);
    string pout(buffer);
    return pout;
}
string TNM_MyDateTime::GetDateString(bool shortformat)
{
    if(shortformat)
    {
        char buffer [12];
        strftime (buffer,12,"%Y-%m-%d",&m_tm);
        string pout(buffer);
        return pout;
    }
    else
    {
        char buffer[40];
        strftime (buffer,40,"%B %d, %Y, %A",&m_tm);
        string pout(buffer);
        return pout;
    }
}
string TNM_MyDateTime::GetDateTimeString(bool shortformat)
{
    ostringstream pstr;
    pstr<<GetDateString(shortformat)<<" "<<GetTimeString(shortformat);
    return pstr.str();
}
string TNM_MyDateTime::GetTimeString(bool shortformat)
{
//    %a	Abbreviated weekday name *	Thu
//%A	Full weekday name * 	Thursday
//%b	Abbreviated month name *	Aug
//%B	Full month name *	August
//%c	Date and time representation *	Thu Aug 23 14:55:02 2001
//%d	Day of the month (01-31)	23
//%H	Hour in 24h format (00-23)	14
//%I	Hour in 12h format (01-12)	02
//%j	Day of the year (001-366)	235
//%m	Month as a decimal number (01-12)	08
//%M	Minute (00-59)	55
//%p	AM or PM designation	PM
//%S	Second (00-61)	02
//%U	Week number with the first Sunday as the first day of week one (00-53)	33
//%w	Weekday as a decimal number with Sunday as 0 (0-6)	4
//%W	Week number with the first Monday as the first day of week one (00-53)	34
//%x	Date representation *	08/23/01
//%X	Time representation *	14:55:02
//%y	Year, last two digits (00-99)	01
//%Y	Year	2001
//%Z	Timezone name or abbreviation	CDT
//%%	A % sign	%
    if(shortformat)
    {
        char buffer [12];
        strftime (buffer,12,"%H:%M:%S",&m_tm);
        string pout(buffer);
        return pout;
    }
    else
    {
        char buffer[20];
        strftime (buffer,20,"%I:%M:%S %p",&m_tm);
        string pout(buffer);
        return pout;
    }
}


bool TNM_MyDateTime::InitializeExpand(const string &inf, bool date)
{
    //string inf(str);
    //trim(inf);
    if(date)
    {
        if(inf.size()!=10) 
        {
            cout<<"\tinvalide format: 10 digit expected, "<<inf.size()<<" found"<<endl;
            return false;
        }
    }
    else
    {
        if(inf.size()!=8 ) 
        {
            cout<<"\tinvalide format:  8 digit expected, "<<inf.size()<<" found"<<endl;
            return false;
        }
    }

    vector<string> words;
     string newinf;

    if(date)
    {
        TNM_GetWordsFromLine(inf,words,'-');            
    }
    else
    {
        TNM_GetWordsFromLine(inf,words,':');

    }
    int formatStatus = 0;
    if(words.size()!=3)
    {
        if(date)
        {
            TNM_GetWordsFromLine(inf, words, '/');
           // cout<<inf<<endl;
            if(words.size()==3)
            {
                formatStatus = 1;
            }
            else
                formatStatus = 2;
        }
        else
        {
                formatStatus = 3;
        }
        
    }
 //   cout<<"invalid DateTime input."<<endl;
   // return false;
    switch(formatStatus)
    {
    case 0:
        for(int i = 0 ;i < 3;i++) 
        {
           newinf+=words[i];
        }
        break;
    case 1:
        newinf+=words[2];
        for(int i = 0;i<=1;i++) newinf+=words[i];
        break;
    case 2:        
    case 3:
        cout<<"invalid DateTime input. Format status = "<<formatStatus<<endl;
        return false;

    }

    

    return InitializeCompact(newinf, date);
}


bool TNM_MyDateTime::InitializeCompact(const string &str, bool date)
{
   // string str(inf);
   // trim(str);
    if(date)
    {
        if(str.size()!=8) 
        {
            cout<<"\tinvalid format: 8 digit expected, "<<str.size()<<" found"<<endl;
            return false;
        }
    }
   
    else 
    {
        if(str.size()!=6) 
        {
            cout<<"\tinvalid format: 6 digit expected, "<<str.size()<<" found"<<endl;
            return false;
        }
    }
       /*tm_sec	seconds after the minute	0-61*
        tm_min	minutes after the hour	0-59
        tm_hour	hours since midnight	0-23
        tm_mday	day of the month	1-31
        tm_mon	months since January	0-11
        tm_year	years since 1900	
        tm_wday	days since Sunday	0-6
        tm_yday	days since January 1	0-365
        tm_isdst	Daylight Saving Time flag*/
        int a, b, c;              
        if(date)
        {
            if(!TNM_FromString(a, str.substr(0,4), std::dec)) return false;
            if(!TNM_FromString(b, str.substr(4,2), std::dec)) return false;
            if(!TNM_FromString(c, str.substr(6,2), std::dec)) return false;  
            //cout<<"year = "<<a<<" month = "<<b<<" day = "<<c<<endl;
            m_tm.tm_year  = a  - 1900;
	        m_tm.tm_mon   = b - 1; //month range between 
	        m_tm.tm_mday  = c;
        }
        else
        {
            if(!TNM_FromString(a, str.substr(0,2), std::dec)) return false;
            if(!TNM_FromString(b, str.substr(2,2), std::dec)) return false;
            if(!TNM_FromString(c, str.substr(4,2), std::dec)) return false;  
            m_tm.tm_hour = a;
            m_tm.tm_min  = b;
            m_tm.tm_sec  = c;
        }
        mktime(&m_tm);//this function set tm_wday, tm_yday. 
       // cout<<this->GetDateString(false)<<endl;
        return true;

}

string TNM_MyDateTime::GetCompactString(bool dateonly)
{
    if(dateonly)
    {
        char buffer [10];
        strftime (buffer,10,"%Y%m%d",&m_tm);
        string pout(buffer);
        return pout;   
    }
    else
    {
        char buffer [16];
        strftime (buffer,16,"%Y%m%d%H%M%S",&m_tm);
        string pout(buffer);
        return pout;

    }
}

//implementation of IDManager
IDManager::IDManager()
{
}

void IDManager::RegisterID(int id)
{
	idSet.insert(id);
	 
	//if(result.second
	if(!idCache.empty()) idCache.erase(id);
}


bool IDManager::FindID(int id)
{
	INT_SET::const_iterator p = idSet.find(id);
	if(p!=idSet.end()) return true;
	else return false;
}
void IDManager::UnRegisterID(int id)
{
	if(!idSet.empty()) idSet.erase(id);
	//idCache.insert(id);
}

int IDManager::SelectANewID()
{
	if(idSet.empty()) return 1;
	int maxVal = *(idSet.rbegin()), minVal = *(idSet.begin());
	int bandWidth =  maxVal - minVal + 1;
	if (bandWidth == idSet.size())
		return maxVal + 1;
	else
	{
		INT_SET::const_iterator p;
		if(!idCache.empty())
		{
			p = idCache.begin();
			return *p;
		}
		{
			int curVal = minVal;
			for (p = idSet.begin(); p!=idSet.end(); p++)
			{
				if(curVal != *p) 
				{
					idCache.insert(curVal);
					p--;
				}
				curVal++;
			}
		}
		if(!idCache.empty())
		{
			p = idCache.begin();
			return *p;
		}
		else
		{
			return -999999;
		}

	}
}