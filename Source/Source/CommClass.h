#ifndef CommClass_H
#define CommClass_H

//#include "stdafx.h"
#include <sstream>
#include <fstream>
#include <strstream>
#include <string>
#include <iomanip>
#include <limits>
#include <math.h>
#include <iostream>
#include <vector>
#include <stack>
#include <map>
#include <time.h>
#include <sstream>
#include <iomanip>
#include <chrono>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <sensor_msgs/LaserScan.h>

#include <pcl/console/parse.h>
#include <Eigen/src/Eigenvalues/EigenSolver.h>
#include <pcl/filters/project_inliers.h>
#include <boost/function.hpp>
#include "Commdefinitions.h"

using namespace std;

template <class T>
class CalcBase
{
public:
	std::string ConvertToString(T Value)
	{
		int prec = numeric_limits<T>::digits10;
		//std::stringstream ss;	
		std::ostringstream ss;		
		ss.precision(prec);
		ss.str("");
		//ss.precision(12);
		ss << Value;
		return ss.str();
	}

	bool IsNan(T Value) //判断是否溢出	
	{
		if (Value != Value)
			return true;
		else
			return false;
	}
};

//

struct Area	//定义每一个分区的 X，Y起始范围，
{
	int PointIndex;
	double XMin, XMax, YMin, YMax;
};


//class DateTimeStr 
//{
//public:
//	static string ShowDateTime(const tm& t, const string& format)
//	{
//		char s[100];
//		strftime(s, sizeof(s), format.c_str(), &t);
//		return string(s);
//	}
//
//
//	static string ShowDateTime(const time_t& t, const string& format)
//	{
//		tm _tm;
//		gmtime_s(&_tm, &t);
//
//		return ShowDateTime(_tm, format);
//	}
//
//	static string ShowDateTime(const tm& t, const char& dateDiv = '-', const char& timeDiv = ':')
//	{
//		ostringstream format;
//		format << "%Y" << dateDiv << "%m" << dateDiv << "%d" << ' ';
//		format << "%H" << timeDiv << "%M" << timeDiv << "%S";
//
//		return ShowDateTime(t, format.str());
//	}
//
//	static string ShowDateTime(const time_t& t, const char& dateDiv = '-', const char& timeDiv = ':')
//	{
//		ostringstream format;
//		format << "%Y" << dateDiv << "%m" << dateDiv << "%d" << ' ';
//		format << "%H" << timeDiv << "%M" << timeDiv << "%S";
//
//		return ShowDateTime(t, format.str());
//	}
//
//	static string ShowYMD(const time_t& t, const char& dateDiv = '-')
//	{
//		ostringstream format;
//		format << "%Y" << dateDiv << "%m" << dateDiv << "%d";
//
//		return ShowDateTime(t, format.str());
//	}
//
//	static string ShowHMS(const time_t& t, const char& timeDiv = ':')
//	{
//		ostringstream format;
//		format << "%H" << timeDiv << "%M" << timeDiv << "%S";
//
//		return ShowDateTime(t, format.str());
//	}
//
//	static string ShowHM(const time_t& t, const char& timeDiv = ':')
//	{
//		ostringstream format;
//		format << "%H" << timeDiv << "%M";
//
//		return ShowDateTime(t, format.str());
//	}
//
//	// 格式化字符串转time=====================================
//
//	static time_t mkgmtime(tm * pTm)
//	{
//		unsigned int year = pTm->tm_year + 1900;
//		unsigned int mon = pTm->tm_mon + 1;
//		unsigned int day = pTm->tm_mday;
//		unsigned int hour = pTm->tm_hour;
//		unsigned int min = pTm->tm_min;
//		unsigned int sec = pTm->tm_sec;
//
//		if (0 >= (int)(mon -= 2)) {    /* 1..12 -> 11,12,1..10 */
//			mon += 12;      /* Puts Feb last since it has leap day */
//			year -= 1;
//		}
//
//		return (((
//			(unsigned long)(year / 4 - year / 100 + year / 400 + 367 * mon / 12 + day) +
//			year * 365 - 719499
//			) * 24 + hour /* now have hours */
//			) * 60 + min /* now have minutes */
//			) * 60 + sec; /* finally seconds */
//	}
//
//	//static time_t str2time(const string& dateStr, const string& format)
//	//{
//	//	tm t;
//	//	memset(&t, 0, sizeof(tm));
//
//	//	strptime(dateStr.c_str(), format.c_str(), &t);// windows下用不了
//
//	//	return mkgmtime(&t);
//	//}
//
//	//static time_t str2time(const string& dateStr, const char& dateDiv = '-', const char& timeDiv = ':')
//	//{
//	//	string format = "%Y-%m-%d %H:%M:%S";
//	//	if (dateDiv != '-')
//	//	{
//	//		format[2] = format[5] = dateDiv;
//	//	}
//	//	if (timeDiv != ':')
//	//	{
//	//		format[11] = format[14] = timeDiv;
//	//	}
//
//	//	return str2time(dateStr.c_str(), format);
//	//}
//
//	static time_t str2date(const string& dateStr, const char& dateDiv = '-')
//	{
//		string format = "%Y-%m-%d";
//		if (dateDiv != '-')
//		{
//			format[2] = format[5] = dateDiv;
//		}
//
//		return str2time(dateStr.c_str(), format);
//	}
//};

class StringBase
{

public:
	//获取重复的几个字符,重复字符间的 字符 是MiddStr
	static string GetRepeatedStr(string Str, int Number, string MiddStr = "")
	{
		string ResultStr = "";	
		for(int i = 0; i < Number; i++)
		{
			if (i == 0)
				ResultStr = Str;
			else
				ResultStr = ResultStr + MiddStr + Str;	
		}
		return ResultStr;
	}

	static string ReplaceStr(string FormulaValue, string OldStr, string NewStr)
	{
		int Index = FormulaValue.find(OldStr);
		int LastIndex = Index;
		while(Index >= 0)
		{
			LastIndex = Index;
			string Left = FormulaValue.substr(0, Index);
			string Right = FormulaValue.substr(Index + OldStr.size(), 
					FormulaValue.size() - Index - OldStr.size()); 
			FormulaValue = Left + NewStr + Right;
			Index = FormulaValue.find(OldStr);
			if (Index == LastIndex)
				break;		
		}
		return FormulaValue;
	}

	//从字符串 Formula 中返回 CharS 中的第一个字符 赋值给FirstChar，返回索引值
	static int FindFirstChar(string Formula, vector<string> CharS, string & FirstChar)
	{
		int Index = Formula.size();
		for(int i = 0 ;i < CharS.size(); i++)
		{
			int TempIndex = Formula.find(CharS[i]);
			if (TempIndex < Index)
			{	
				Index = TempIndex;
				FirstChar = CharS[i];
			}
		}
		return Index;
	}

	static string ClockValue()
	{
		CalcBase<float> CalcBasefloat;
		return CalcBasefloat.ConvertToString(clock());
	}

	static string IntToStr(int i)
	{
		CalcBase<int> CalcBasefloat;
		return CalcBasefloat.ConvertToString(i);
	}

	static string FloatToStr(float i)
	{
		CalcBase<float> CalcBasefloat;
		return CalcBasefloat.ConvertToString(i);
	}

	static string FloatToStr(double i)
	{
		CalcBase<double> CalcBasefloat;
		return CalcBasefloat.ConvertToString(i);
	}

	static string DateStr()
	{
		auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				
		stringstream ss;
		ss << std::put_time(std::localtime(&t), "%Y-%m-%d");
		return ss.str();
	}

	static string DateTimeStr()
	{
		auto t = chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				
		stringstream ss;
		ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
		return ss.str();
	}
};


////##############################*****************

class Math
{
public:
//计算从n个数中取m个数的取法个数 即组合数
static double CombinatorialNumber(int n, int m)
{
	if ( m == 0 || m == n)
		return 1.0;
	else
	{
		return MultipleMultiplication(n, m) / factorial(m);
	}	
}

//计算n的阶乘
static int factorial(int n)
{
	if (n == 1)
		return 1;
	else
		return n * factorial(n-1);
}

//计算从n*(n-1)*...*(n-k+1)
static int MultipleMultiplication(int n, int k)
{
	int Result = 1;
	for(int i = n; i >= n - k + 1; i--)
	{
		Result = Result * i;	
	}
	return Result;
}

//LU分解 解线性方程组 AX = B
static Eigen::MatrixXd ResloveLinearEquationsByLUDecomposition(
	Eigen::MatrixXd A, Eigen::MatrixXd B)
{	
	//return A.fullPivLu().solve(B);

	//Eigen::MatrixXd inverse;	
	//double determinant;

	//A.computeInverseAndDetWithCheck(inverse, determinant,invertible);	此步只适用于
	//return A.jacobiSvd().solve(B);
	// LU分解 
	return A.lu().solve(B);	
}

};

class FileBase
{
public:

	static bool FindFileExists(string FileName)
	{
		bool Find = true;

		if( (_access(FileName.c_str(), 0 )) == -1 )
		{
			Find = false;
		}

		return Find;
	}

};

class Arithmetic
{

private:
/*  中缀表达式转换成后缀表达式
    输入中缀表达式字符串infix，转化后返回后缀表达式字符串postfix
    输出串中，每一个数字后面放置一个#作为标识。因为数字使用char表达，234即三个char，“#”标识数字结束,
    做好数字完结标识后，即写入输出字符串中。
    遇到符号，左括号直接入栈；右括号时，将栈顶符号逐个取出写入输出字符串中，直到遇见左括号；
    运算符需循环与栈顶符号优先级进行比较：如果小于或者等于，取出栈顶符号写入输出字符串中，
    如果是左括号，直接入栈，如果优先级高于栈顶符号，入栈。
*/
static std::string InfixToPostfix(std::string infix)
{
    char current = 0;
    std::string postfix;//后缀表达式

    std::stack<char> mark;//符号栈

    std::map<char,int> priority;//符号优先级
    priority['+'] = 0;
    priority['-'] = 0;
    priority['*'] = 1;
    priority['/'] = 1;

    for(int i = 0;i < infix.size(); ++i)
    {
        current = infix[i];
        switch(current)
        {
            case '0':case '1':case '2':case '3':case '4':case '5':
			case '6':case '7':case '8':case '9':case '.':case 'e':
                postfix.push_back(current);//数字直接写入
                break;
            case '+':case '-':case '*':case '/':
                //如果运算符的前一项不是右括号即说明前一个数字输入完毕，用#标识

				////对于第一个数字是负数的情况
				//if (infix[i] == '-' && i == 0)
				//{
				//	postfix.push_back(current);
				//	break;
				//}
                if(infix[i-1] == 'e' && (infix[i] == '-' || infix[i] == '+'))
                {
					postfix.push_back(current);
					break;
				}

                if(infix[i-1] != ')')
                    postfix.push_back('#');
                //如果符号栈非空，即比较目前符号与栈顶符号优先级，低于等于出栈(并写入输出字符串)，
                //直至符号全部出栈或者遇到了'('或者大于栈顶符号的优先级
                if(!mark.empty())
                {
                    char tempTop = mark.top();
                    while(tempTop != '(' && priority[current] <= priority[tempTop])
                    {
                        postfix.push_back(tempTop);
                        mark.pop();
                        if(mark.empty())
                            break;
                        tempTop = mark.top();
                    }
                }
                mark.push(current);//新符号入栈
                break;
            case '(':
                if(infix[i-1] >= '0' && infix[i-1] <= '9')// for expression 2-5*2(6/2)
                {
                    postfix.push_back('#');
                    mark.push('*');
                }
                mark.push(current);
                break;
            case ')':
                postfix.push_back('#');//右括号说明前方数字输入完成，标识一下
                while(mark.top() != '(')
                {
                    postfix.push_back(mark.top());
                    mark.pop();
                }
                mark.pop();//左括号出栈
                break;
            default:
                break;//忽略其他字符
        }
    }
    if(infix[infix.size()-1] != ')')
        postfix.push_back('#');//中缀表达式最后一个是数字需要加上#。
    while(!mark.empty())//如果栈非空，全部出栈并写入输出字符串
    {
        postfix.push_back(mark.top());
        mark.pop();
    }
    return postfix;
}

/*  计算后缀表达式结果
    输入为后缀表达式s，逐个读取字符，如果是数字即放入存放当前数字的字符串中，
    遇到“#”即将此字符串转换为float，
    使用atof()，参数接受const char*类型，字符串需使用.c_str()转化。
    完成数字识别转化后入栈，遇到符号即取出栈顶的两个数字计算，结果入栈。
    栈中最后的元素即为结果。
*/
static double posfixCompute(std::string s)
{
    std::stack<double> tempResult;

    std::string strNum;
    double currNum = 0;
	char PriorChar;
    double tempNum = 0;
    for(std::string::const_iterator i = s.begin(); i != s.end(); ++i)
    {
        switch(*i)
        {
            case '0':case '1':case '2':case '3':case '4':case '5':
			case '6':case '7':case '8':case '9':case '.':case 'e':
                strNum.push_back(*i);
                break;
            case '+':
				if (PriorChar == 'e')
				{
					strNum.push_back(*i);
					break;
				}
                tempNum = tempResult.top();
                tempResult.pop();
                tempNum += tempResult.top();
                tempResult.pop();
                tempResult.push(tempNum);
                break;
            case '-':
				if (PriorChar == 'e')
				{
					strNum.push_back(*i);
					break;
				}

                tempNum = tempResult.top();
                tempResult.pop();
                tempNum = tempResult.top() - tempNum;
                tempResult.pop();
                tempResult.push(tempNum);
                break;
            case '*':
                tempNum = tempResult.top();
                tempResult.pop();
                tempNum *= tempResult.top();
                tempResult.pop();
                tempResult.push(tempNum);
                break;
            case '/':
                tempNum = tempResult.top();
                tempResult.pop();
                tempNum = tempResult.top() / tempNum;
                tempResult.pop();
                tempResult.push(tempNum);
                break;
            case '#':
                currNum = atof(strNum.c_str());//in c++11, use currNum = std::stof(strNUm);
                strNum.clear();
                tempResult.push(currNum);
                break;
        }
		PriorChar = *i;
    }
    return tempResult.top();
}

public:

	//对于第一个
static double expressionCalculate(std::string s)
{
    return posfixCompute(InfixToPostfix(s));
}

};

#endif