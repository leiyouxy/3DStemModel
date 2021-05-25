#pragma once

#include <thread>
#include <future>
#include <omp.h>

#include "CommClass.h"
#include "Spline.h"
#include "SplineInterpolation.h"
#include "StemSkeleton.h"
#include "AnglePartition.h"
#include "TreeBase.h"

/*

add by yleiou Construction of stem surface and and stem volume

*/

//曲面插值的类
class CSurfaceInterpolation : public CTreeBase
{
private:
	//InterpolationPoints 中存储的是（）×（）个数据点
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> InterpolationPoints;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> RowControlPoints;

	//闭合曲线节点 根据插值点节点集合的第一个点构建的样条曲线
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ClosedReferenceCurvePoint;

	//求解 一组全局控制点，
	bool ResolveOneGlobalControlPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue,
		vector<double> ParameterValues, vector<double> KnotValues,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints);

	int k, l; //k行插值点 and //l列插值点

	//根据 RowIndex和 ColIndex 索引 与 节点向量 RowU 和 ColV 得到曲面上的点
	pcl::PointXYZRGB GetSurfacePoint(int RowIndex, int ColIndex, double RowU, double ColV);

	double RowStep, ColStep, HStep; //Step show surface points and calculation precision

	int dSpline;

	bool HorizontalIsClosed;

	double DisError;
	double eps;

	std::mutex g_lock;
	const int MaxThreadNum = 60;
	int ExecuteThreadNum;
	double StemVolume;

	//double ReferenceHeight;

	//搜索节点向量值所在的索引
	int FindUIndex(double u, vector<double> KnotValues, int StartIndex, int EndIndex);

	//根据参数值 在 参数空间中 寻找 索引 此种方法不准确
	void FindUVIndex(double RowU, double ColV, int & RowIndex, int & ColIndex);

	//根据插值点集合的第一个节点与 ColStep 创建的闭合曲线的控制点
	void ConstructClosedCurvePoint();

	//在构建的闭合曲线的控制点上找 距离 ColU 所控制节点高度位置处的 最近点
	pcl::PointXYZRGB FindHNearestPoint(double RowU, double ColU);

	//给定列的节点向量，寻找在该列位置处 构建闭合曲线的 起始 StartU 与 EndU；
	void FindStartUandEndU(int RowIndex, double ColU,
		double & StartRowU, double & EndRowU, double DisError = 0.001, double UStep = 0.000001);

	//根据高度Z值获取对应位置处的 横断面轮廓曲线对应的面积
	double CalcAreaByHeight(double Z, int RowIndex, double DisError = EPSM3);
	
//	double CalcAreaByHeightByThread(double Z, int RowIndex, double DisError = 0.001);

	//根据高度Z值获取对应位置处的 横断面轮廓曲线
	void GetCrossSectionalCurveByHeight(double Z, pcl::PointCloud<pcl::PointXYZRGB>::Ptr HeightPoints,
		double DisError = 0.001, double UStep = 0.001);

	//根据u值返回 u 的字符串公式表达式 2016.12.13 根据具体的U值 与 VValue 值计算曲面在特定VValue值下的表达式
	virtual vector<string> GetUVFormula(double RowU, double ColV);

	double CalcZoneValueByThread(vector<string> FormulaS, double TempUStart, double TempUEnd);
public:
	CSurfaceInterpolation()
	{
		//GlobalControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		//RowControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);	
		ClosedReferenceCurvePoint.reset(new pcl::PointCloud<pcl::PointXYZRGB>);		
		dSpline = 3;
		HorizontalIsClosed = true;
		DisError = 0.001;
		eps = 1.0e-8;
	}
	~CSurfaceInterpolation();

	//根据给定高度寻找 ColU 的值 2019.07.10
	double FindColUByHeight(double Z);

	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> SurfacePoints;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SplineSurfacePoints;

	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> GlobalControlPoints;

	vector<vector<double>> KnotValuesCol;
	//节点参数值
	vector<vector<double>> ParametersCol;	

	vector<vector<double>> KnotValuesRow;
	//节点参数值
	vector<vector<double>> ParametersRow;	

	//构建全局曲面插值
	void InterpolationSurface();

	//根据定义画插值曲面
	void DrawSurfaceByDefinition(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		double UStep = 0.001, double VStep = 0.001);

	void GetSurfacePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutSurfacePoints, 
		double UStep = 0.001, double VStep = 0.001);

	double CalcVolume(double StartZ, double EndZ);

	//The input is the formatted Interpolation points 
	void SetInputInterpolationPoints(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> InterpolationPointsValue,
		double HSetpValue = 0.05, double RowStepValue = 0.05, double ColStepValue = 0.05);
	
	////The input is the stem points that should be formatted to Interpolation points
	void SetInputStemPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemPointPtr, 
		float SectionThickNessValue, int CalcSectionNumbersValue, double AnlgeValue,
		double ZMax,
		double HSetpValue = 0.05, double RowStepValue = 0.05, double ColStepValue = 0.05);

	void ShowInterpolationPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer);

};
