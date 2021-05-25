#ifndef StemSkeleton_H
#define StemSkeleton_H

//2016.04.05  迭代获取与树干生长方向垂直的分段及分段的质心点，

#include "CommClass.h"
#include "HorizontalPartition.h"
#include "AnglePartition.h"
#include "Commdefinitions.h"
#include "SplineInterpolation.h"
#include "Spline.h"
#include "StemDiameter.h"

using namespace std;

class CStemSkeleton
{
private:	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemCloud;		//树干点云
	float SectionThickNess;									//分段厚度
	//float MiniError;										//迭代满足最小的误差
	int CalcSectionNumbers;									//多少个分段一起计算
	int StartIndex;
	int TheIncludedAngle;
	bool IsOptimized;

	double CurrentHeightShow;								//2018.10.31 为保存正在显示的横断面点云文件而设定的高度
	
	//计算最大值与最小值
	void GetMaxAndMinValueFromPointsAndNormal(pcl::PointCloud<PointXYZRGBIndex>::Ptr TempCloud,
		pcl::PointXYZRGB InitialNormal, float & MinValue, float & MaxValue);
public:
	//计算初始的树干生长向量
	pcl::PointXYZRGB GetInitialVector();

	//返回另一个平面上的一个点
	pcl::PointXYZRGB GetSlicePoints(pcl::PointXYZRGB NormalPoint, pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints,
		pcl::PointXYZRGB & SliceCentroidPoint, bool IsUp = true, vector<int> * Indexs = NULL);

	//2019.06.15 当前位置的前一个Slice
	void GetPriorSlicePoints(pcl::PointXYZRGB NormalPoint, pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints,
		pcl::PointXYZRGB & SliceCentroidPoint, bool IsUp = true, vector<int> * Indexs = NULL, int SliceNum = 1);

	//获取点集的质心点
	pcl::PointXYZRGB GetCentroidOfPointsInSpaces(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
		pcl::PointXYZRGB NormalPoint, bool & IsCircle);
private:
	//根据质心点判断当前平面点集是否是圆形
	bool PlanePointsIsCircle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints, 
		pcl::PointXYZRGB Centriod);

//渐进式获取树干骨架
	void GetSkeletonPointS();

	void UseSlicesSkeleton();

	//void AngleSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints, double Angle = 1);

	//double GetCrossSectionalProfileAndArea(
	//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CrossSectionalProfilePtr,
	//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurve, bool IsArea = false);	//获取横断面轮廓曲线 并 计算断面积

	//void ShowCrossSectionalPlaneByCircle(pcl::PointXYZRGB CenterPoint, pcl::PointXYZRGB NormalPoint, 
	//	double Radius, int PointSize = 1,
	//	int r = 255, int g = 0, int b = 0 );

public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CentroidPoints;		//树干的骨架点
private:
	double MaximumHeight;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OptimizedCentroidPoints;		//树干的骨架点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemSkeletonPoints;	//树干骨架的样条点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AllProfilecurvePoints;	//树干骨架的样条点
	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;
	
	//SectionVector SectionsVector;

	//is the input cloud has points from stem tip, if not, 2019.1.23
	bool IsHaveTip;
	void IsHaveTipPoints();
public:	
	CStemSkeleton()
	{
		IsHaveTip = true;
		StemCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		CentroidPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		OptimizedCentroidPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		StemSkeletonPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		AllProfilecurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		//UnPartitedCloud.reset(new pcl::PointCloud<PointXYZRGBIndex>);
	}
	string ResultStr;
	CSpline StemSkeletonSpline;
	CHorizontalPartition HorizontalPartition;

	// IsOptimizedValue 表示 是否对几何中心点进行优化，
	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemCloudValue,
		float SectionThickNessValue = 0.5, bool IsOptimizedValue = true, int CalcSectionNumbersValue = 10,
		int StartIndexValue = 0, int TheIncludedAngleValue = 60, double MaximumHeightValue = 0);
	
	//构建骨架的样条曲线
	void ConstructStemSplineCurve(double StartHeight = 100, bool IsDirect = false);

	////在指定的位置处画横断面  这个函数应在树干骨架移动前完成
	//void DrawCrossSectionalPlaneByHeight(float Height, int LineWidth = 10, float PointSize = 3, 
	//	bool ShowProfileCurve = false, bool DrawUpper = false);

////从开始位置StartHeight处 至 EndHeight处 画 PlaneNumbers 个横断面 2017.05.27
//	void DrawCrossSectionalPlane(float StartHeight, float EndHeight, float PlaneNumbers = 20, 
//		float PointSize = 3,
//		bool ShowProfileCurve = false, bool DrawUpper = false);

	bool IsSuitForConstruct();

	//在给定Height值处显示横断面
	void DrawCrossSectionalCurve(float Height, float PointSize = 3);

	//在给定UValue值处显示横断面
	void DrawCrossSectionalCurveAtU(float UValue, float PointSize = 3);

	void SaveInclinationCurvatureTorsionToFile(double StartLength, double EndLength, double Interval);
};

#endif