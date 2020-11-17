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

	bool IsNan(T Value) //�ж��Ƿ����	
	{
		if (Value != Value)
			return true;
		else
			return false;
	}
};

//

struct Area	//����ÿһ�������� X��Y��ʼ��Χ��
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
//	// ��ʽ���ַ���תtime=====================================
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
//	//	strptime(dateStr.c_str(), format.c_str(), &t);// windows���ò���
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
	//��ȡ�ظ��ļ����ַ�,�ظ��ַ���� �ַ� ��MiddStr
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

	//���ַ��� Formula �з��� CharS �еĵ�һ���ַ� ��ֵ��FirstChar����������ֵ
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
//�����n������ȡm������ȡ������ �������
static double CombinatorialNumber(int n, int m)
{
	if ( m == 0 || m == n)
		return 1.0;
	else
	{
		return MultipleMultiplication(n, m) / factorial(m);
	}	
}

//����n�Ľ׳�
static int factorial(int n)
{
	if (n == 1)
		return 1;
	else
		return n * factorial(n-1);
}

//�����n*(n-1)*...*(n-k+1)
static int MultipleMultiplication(int n, int k)
{
	int Result = 1;
	for(int i = n; i >= n - k + 1; i--)
	{
		Result = Result * i;	
	}
	return Result;
}

//LU�ֽ� �����Է����� AX = B
static Eigen::MatrixXd ResloveLinearEquationsByLUDecomposition(
	Eigen::MatrixXd A, Eigen::MatrixXd B)
{	
	//return A.fullPivLu().solve(B);

	//Eigen::MatrixXd inverse;	
	//double determinant;

	//A.computeInverseAndDetWithCheck(inverse, determinant,invertible);	�˲�ֻ������
	//return A.jacobiSvd().solve(B);
	// LU�ֽ� 
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
/*  ��׺���ʽת���ɺ�׺���ʽ
    ������׺���ʽ�ַ���infix��ת���󷵻غ�׺���ʽ�ַ���postfix
    ������У�ÿһ�����ֺ������һ��#��Ϊ��ʶ����Ϊ����ʹ��char��234������char����#����ʶ���ֽ���,
    ������������ʶ�󣬼�д������ַ����С�
    �������ţ�������ֱ����ջ��������ʱ����ջ���������ȡ��д������ַ����У�ֱ�����������ţ�
    �������ѭ����ջ���������ȼ����бȽϣ����С�ڻ��ߵ��ڣ�ȡ��ջ������д������ַ����У�
    ����������ţ�ֱ����ջ��������ȼ�����ջ�����ţ���ջ��
*/
static std::string InfixToPostfix(std::string infix)
{
    char current = 0;
    std::string postfix;//��׺���ʽ

    std::stack<char> mark;//����ջ

    std::map<char,int> priority;//�������ȼ�
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
                postfix.push_back(current);//����ֱ��д��
                break;
            case '+':case '-':case '*':case '/':
                //����������ǰһ��������ż�˵��ǰһ������������ϣ���#��ʶ

				////���ڵ�һ�������Ǹ��������
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
                //�������ջ�ǿգ����Ƚ�Ŀǰ������ջ���������ȼ������ڵ��ڳ�ջ(��д������ַ���)��
                //ֱ������ȫ����ջ����������'('���ߴ���ջ�����ŵ����ȼ�
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
                mark.push(current);//�·�����ջ
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
                postfix.push_back('#');//������˵��ǰ������������ɣ���ʶһ��
                while(mark.top() != '(')
                {
                    postfix.push_back(mark.top());
                    mark.pop();
                }
                mark.pop();//�����ų�ջ
                break;
            default:
                break;//���������ַ�
        }
    }
    if(infix[infix.size()-1] != ')')
        postfix.push_back('#');//��׺���ʽ���һ����������Ҫ����#��
    while(!mark.empty())//���ջ�ǿգ�ȫ����ջ��д������ַ���
    {
        postfix.push_back(mark.top());
        mark.pop();
    }
    return postfix;
}

/*  �����׺���ʽ���
    ����Ϊ��׺���ʽs�������ȡ�ַ�����������ּ������ŵ�ǰ���ֵ��ַ����У�
    ������#���������ַ���ת��Ϊfloat��
    ʹ��atof()����������const char*���ͣ��ַ�����ʹ��.c_str()ת����
    �������ʶ��ת������ջ���������ż�ȡ��ջ�����������ּ��㣬�����ջ��
    ջ������Ԫ�ؼ�Ϊ�����
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

	//���ڵ�һ��
static double expressionCalculate(std::string s)
{
    return posfixCompute(InfixToPostfix(s));
}

};

#endif