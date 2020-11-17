#ifndef CommVector_H
#define CommVector_H

#include<fstream>
#include<iostream>
#include <string>
#include <vector>
#include "Commdefinitions.h"

using namespace std;

//2016.01.15
//2018.12.24, modified by LeiYou
//The implementation of template class function must be written header file. 
template<class T>
class VectorBase
{
public:
	// 对整数型 容器排序 默认升序
	void SortVector(vector<T> & Vector, string Type = "Asc");

	//2016.01.15 将一个索引值插入到索引值容器中
	void InsertIndexToVector(vector<T> & Vector, T Index);	

	//2016.01.15 因最近邻获取的邻域点有时不包含自己，做一个是否包含自己的检测
	int FindIndexInVector(vector<T> NeighbourIndex, T Index);	

	//计算均值
	double CalcMeans(vector<T> floatVec);	

	//计算最大值
	double CalcMax(vector<T> floatVec);

	//计算最小值
	double CalcMin(vector<T> floatVec);

	//计算容器floatVec内的方差
	double CalcVariances(vector<T> floatVec);	
	
	//计算容器floatVec内的均值与方差	
	double CalcVariances(vector<T> floatVec, double & MeanValue);	

	//计算截尾平均值与方差，返回方差
	double CalcVariancesByTrimMaxMin(vector<T> floatVec, double & MeanValue);	

	int FindValueByBinarySearch(vector<T> & Values, T TempValue);

	//Calc Quartile, return is the IRQ
	double CalcQuartile(vector<T> & Values, double & UpperLimit, double & LowerLimit);

	int GetMaxIndex(vector<T> Vector);

	int GetMinIndex(vector<T> Vector);

	void SaveToFile(vector<T> Vector, string FileName);
};


//Asc Or Desc
template<class T>
void VectorBase<T>::SortVector(vector<T> & Vector, string Type = "Asc")
{
	for (int i = 0; i < Vector.size(); i++)
	{
		for (int j = i + 1; j < Vector.size(); j++)
		{
			if (Type == "Asc")
			{
				if (Vector[i] > Vector[j])
					swap(Vector[i], Vector[j]);
			}
			else
			{
				if (Vector[i] < Vector[j])
					swap(Vector[i], Vector[j]);
			}
		}
	}
}

//2016.01.15 将一个索引值插入到索引值容器中
template<class T>
void VectorBase<T>::InsertIndexToVector(vector<T> & Vector, T Index)
{
	bool Find = false;
	for (int i = 0; i < Vector.size(); i++)
	{
		if (abs(Vector[i] - Index) < EPSM9)
		{
			Find = true;
			break;
		}
	}
	if (!Find)
		Vector.push_back(Index);
}

template<class T>
int VectorBase<T>::GetMaxIndex(vector<T> Vector)
{
	if (Vector.size() == 0) return -1;
	int Max_Index = 0;
	double Max_Value = Vector[Max_Index];

	for (int i = 1; i < Vector.size(); i++)
	{
		if (Vector[i] > Max_Value)
		{
			Max_Value = Vector[i];
			Max_Index = i;
		}
	}
	return Max_Index;
}

template<class T>
int VectorBase<T>::GetMinIndex(vector<T> Vector)
{
	if (Vector.size() == 0) return -1;
	int Min_Index = 0;
	double Min_Value = Vector[Min_Index];

	for (int i = 1; i < Vector.size(); i++)
	{
		if (Vector[i] < Min_Value)
		{
			Min_Value = Vector[i];
			Min_Index = i;
		}
	}
	return Min_Index;
}

//2016.01.15 因最近邻获取的邻域点有时不包含自己，做一个是否包含自己的检测
template<class T>
int VectorBase<T>::FindIndexInVector(vector<T> NeighbourIndex, T Index)
{
	int FindIndex = -1;
	for (int i = 0; i < NeighbourIndex.size(); i++)
	{
		if (abs(NeighbourIndex[i] - Index) < EPSM9)
		{
			FindIndex = i;
			break;
		}
	}
	return FindIndex;
}

template<class T>
double VectorBase<T>::CalcMeans(vector<T> floatVec)
{//计算容器floatVec内的均值
	double Means = 0;
	int i = 0;

	if (floatVec.size() > 0)
	{
		for (i = 0; i < floatVec.size(); i++)
		{	//求平均值
			Means = Means + floatVec[i] * 1.0 / floatVec.size();
		}
	}
	else
		Means = 0;
	return Means;
}

template<class T>
double VectorBase<T>::CalcMax(vector<T> floatVec)
{//计算容器floatVec内的最大值
	if (floatVec.size() == 0) return 0;

	double Max = floatVec[0];
	int i = 0;

	for (i = 1; i < floatVec.size(); i++)
	{	//求最大值
		if (Max < floatVec[i])
			Max =  floatVec[i];
	}		
	return Max;
}

template<class T>
double VectorBase<T>::CalcMin(vector<T> floatVec)
{//计算容器floatVec内的最小值
	if (floatVec.size() == 0) return 0;

	double Min = floatVec[0];
	int i = 0;

	for (i = 1; i < floatVec.size(); i++)
	{	//求最小值
		if (Min > floatVec[i])
			Min = floatVec[i];
	}
	return Min;
}

template<class T>
double VectorBase<T>::CalcVariances(vector<T> floatVec)
//计算容器floatVec内的方差
{
	double Varinces = 0;

	double Means = CalcMeans(floatVec);
	if (floatVec.size() > 0)
	{
		for (int i = 0; i < floatVec.size(); i++)
		{	//求方差			
			Varinces = Varinces + pow(floatVec[i] - Means, 2) / floatVec.size();
			//Varinces = Varinces + pow(floatVec[i] - Means, 2);
		}
	}
	else
		Varinces = 0;

	return Varinces;
}

template<class T>
double VectorBase<T>::CalcVariances(vector<T> floatVec, double & MeanValue)
//计算容器floatVec内的均值与方差
{
	double Varinces = 0;

	double Means = CalcMeans(floatVec);

	if (floatVec.size() > 0)
	{
		for (int i = 0; i < floatVec.size(); i++)
		{	//求方差			
			Varinces = Varinces + pow(floatVec[i] - Means, 2) / floatVec.size();
			//Varinces = Varinces + pow(floatVec[i] - Means, 2);
		}
	}
	else
		Varinces = 0;

	MeanValue = Means;
	return Varinces;
}

//计算截尾平均值与方差，返回方差
template<class T>
double VectorBase<T>::CalcVariancesByTrimMaxMin(vector<T> floatVec, double & MeanValue)
{
	int MaxIndex, MinIndex;
	double MaxValue, MinValue;

	MaxValue = floatVec[0];
	MinValue = MaxValue;
	MaxIndex = 0;
	MinIndex = 0;

	for (int i = 1; i < floatVec.size(); i++)
	{
		if (MaxValue < floatVec[i])
		{
			MaxValue = floatVec[i];
			MaxIndex = i;
		}
		if (MinValue > floatVec[i])
		{
			MinValue = floatVec[i];
			MinIndex = i;
		}
	}

	if (MaxIndex > MinIndex)
	{
		floatVec.erase(floatVec.begin() + MaxIndex);
		floatVec.erase(floatVec.begin() + MinIndex);
	}
	else
	{
		floatVec.erase(floatVec.begin() + MinIndex);
		floatVec.erase(floatVec.begin() + MaxIndex);
	}
	return CalcVariances(floatVec, MeanValue);
}

template<class T>
int VectorBase<T>::FindValueByBinarySearch(vector<T> & Values, T TempValue)
{
	if (Values.size() == 0) return -1;
	if (TempValue < Values[0] || TempValue > Values[Values.size() - 1]) return -1;

	int low = 0;
	int LastIndex = Values.size() - 1;

	if (abs(TempValue - Values[LastIndex]) <= EPSM6) return LastIndex;
	int high = LastIndex;
	int Index = (low + high) / 2;
	while ((TempValue < Values[Index] || TempValue >= Values[Index + 1]) && (low != high))
	{
		if (TempValue < Values[Index]) high = Index;
		else low = Index;
		Index = (low + high) / 2;
	}
	return Index;
}

//Calc Quartile, return is the IRQ
template<class T>
double VectorBase<T>::CalcQuartile(vector<T> & Values, double & UpperLimit, double & LowerLimit)
{
	double Q1Index = 0, Q2Index = 0, Q3Index = 0;
	double Q1Value = 0, Q2Value = 0, Q3Value = 0;
	double IQR = 0;

	int n = Values.size();
	Q1Index = 1.0 * (n + 1) / 4.0;
	Q2Index = 2.0 * (n + 1) / 4.0;
	Q3Index = 3.0 * (n + 1) / 4.0;

	SortVector(Values);

	int Q1Lower, Q1Upper, Q2Lower, Q2Upper, Q3Lower, Q3Upper;

	//The start index is zero.
	Q1Lower = floor(Q1Index) - 1, Q1Upper = ceil(Q1Index) - 1;
	Q2Lower = floor(Q2Index) - 1, Q2Upper = ceil(Q2Index) - 1;
	Q3Lower = floor(Q3Index) - 1, Q3Upper = ceil(Q3Index) - 1;

	//if (Q3Upper >= n)
	//{
	//	LowerLimit = 0;
	//	return 0;
	//}

	Q1Value = Values[Q1Lower] * 0.25 + Values[Q1Upper] * 0.75;
	Q2Value = Values[Q2Lower] * 0.5 + Values[Q2Upper] * 0.5;
	Q3Value = Values[Q3Lower] * 0.75 + Values[Q3Upper] * 0.25;

	IQR = (Q3Value - Q1Value) / 2.0;

	UpperLimit = Q3Value + 1.5 * IQR;
	LowerLimit = Q1Value - 1.5 * IQR;

	return IQR;
}

template<class T>
void VectorBase<T>::SaveToFile(vector<T> Vector, string FileName)
{
	ofstream OutFile(FileName);
	for (int i = 0; i < Vector.size(); i++)
		OutFile << Vector[i] << endl;

	OutFile.close();
}

#endif