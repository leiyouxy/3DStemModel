#ifndef BezierSubsectionG2_H
#define BezierSubsectionG2_H

///2015.12.11分段构建二次光滑的闭合Bezier曲线

#include "CommClass.h"
#include "BezierCurve.h"
//#include "AnglePartition.h"
//#include "FormulaAnalytic.h"
#include "CommVector.h"

class CBezierSubsectionG2
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPoints;

	vector<long double> MaxCoefficientValue;

	vector<long double> CoefficientValue;

	vector<long double> CoefficientRate0fOneAndFour;

	//计算
	void CalcCoefficientRateAndMaxCoefficientValue();
	long double Size;

	//计算第PointIndex个点的第Order阶导数 返回向量解
	pcl::PointXYZRGB CalcPointDerivative(int PointIndex, int Order, int Position);
	
	//返回导数的数值解
	double CalcPointDerivativeValue(int PointIndex, int Order, int Position);

	//计算控制系数 根据方程组
	void ResolveCoefficientByEquations();

	///×××××××以上为之前老的程序 下为2015.12.31编写程序 唐老师的想法

	long double U;

	vector<long double> UU;	//对应于 minu
	vector<long double> AA;	//对应于
	vector<long double> BB;	//对应于 beita
	vector<long double> EE;	//对应于e
	vector<long double> FF;	//对应于f
	vector<long double> KK;	//对应于k
	vector<long double> GG;	//对应于g
	vector<long double> RR;	//对应于 可以不使用

	//点之间构成的向量 VectorofQPoints[i] = p(i+1) - p(i)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr VectorofQPoints;
	
	//构建节点的弦向量  及其它参变量	 2015.12.31 
	//2015.01.26 修改为 动态判断 EE 值是否为0 如果为0 则删除对应的 i+1 的节点
	bool GetVectorofQPointsAndOthers();

	//根据上述计算结果计算控制点 2015.12.31
	void ResolveControlPoints();	

	//所有点对应的U值 //2016.01.05
	vector<double> AllPointUValues;

	//每段开始的节点索引 //2016.01.05
	vector<int> SectionStartPointIndex;

	////判断此点是否是凸曲线上的点 	//2016.01.05
	//bool IsConvexHullPoint(int AngleIndex, pcl::PointXYZRGB TangentLinePoint,
	//	vector<AnglePartitionStruct> & SectionAngle);

	//2016.01.05
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalBezierCurvePoints;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr NoConvexhullPoints;

	//计算样条曲线的切线 	//2016.01.05
	pcl::PointXYZRGB ResolveTangentLine(int PointIndex);

	//避免(Li) X (Li-1)等于0的问题
	void FilterPoints();
public:
	CBezierSubsectionG2()
	{
		QPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		ControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		VectorofQPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		//NoConvexhullPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints;

	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, double SizeValue = 0.8);

	//计算控制点
	void ResolveControlPoints(double UValue = BezierMiu);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr DrawBroderPoints();

	double DrawBezierCurve(pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierCurvePoints, int PointNum = -1);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;

	void ShowPointDerivative();

	////计算点构成曲线的凸性
	//void ComputeConvexPropertyOfCurve();

	//获取特定点的曲率
	double GetCurvature(int PointIndex);

	double GetBezierLength();

	double GetBezierArea();
};

#endif