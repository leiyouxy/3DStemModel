#pragma once

#ifndef StemGridPointsRefine_H
#define StemGridPointsRefine_H

#include "Commdefinitions.h"
#include "HorizontalPartition.h"
#include "AnglePartition.h"
#include "Spline.h"
#include "SplineInterpolation.h"
#include "StemSkeleton.h"
#include "nlopt.hpp"

/*

refining the stem grid points for calculation(stem volume, the interpolation for stem surface interpolation)

 2018.12.31 by leiyou

*/

class CStemGridPointsRefine : public CTreeBase
{

private:
	CHorizontalPartition * p_HorizontalPartition;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionsCenterPoints;	

	//Angle Value for anlge Partition and for refine
	double Angle;

	void GetOriginalGirdPoints();

	// filling the hole anlge by using the same anlge partition in the upper section 
	void RefineDownToUpByDirectlyFilling();

	//not used Now 2019.01.07
	bool CheckRowIsNeedRefine(int RowIndex);

	pcl::PointXYZRGB FindAngleCenterPointInSplineCurve(CSpline & SplineCurve,
		pcl::PointXYZRGB CenterPoint, int AngleIndex);
	
	//at the same height with different anlge Index
	void RefineRow(int RowIndex, bool DrawCurve = false);
	void RefineRowByThread(int RowIndex, bool DrawCurve = false);

	bool CheckColIsNeedRefine(int ColIndex);
	
	//at the same anlge Index with different sections
	void RefineCol(int ColIndex, bool DrawCurve = false);
	void RefineColByThread(int ColIndex, bool DrawCurve = false);
	
	std::mutex g_lock;
	std::thread ThreadObj;
	const int MaxThreadNum = 20;
	//int ExecuteThreadNum;

	//2019.07.16 根据角度与距离的线性关系对多个连续缺失的角度分区进行修复
	void RefineSlicesByLinear(int Index);

	//2019.07.21 对当前垂直分段中的轮廓点进行优化，优化准则是当前轮廓点到当前角度分区中的点的距离最小，且相邻角度分区间的角度也越均衡
	void RefineSlincesByOptimal(int Index);

	//2019.07.18 确保相邻三个点的距离不小于90度，如果有，则向中心点移动
	void RefineAngleRule(int Index);

	//
	pcl::PointXYZRGB GetMiddlePointAlongLineBetweenTwoPoints(
		pcl::PointXYZRGB DirectionA, pcl::PointXYZRGB DirectionB,
		pcl::PointXYZRGB PointA, pcl::PointXYZRGB PointB, double Angle);

	void RefineHeadAndRearByStemAxisCurve(int Index);
	void RefineMiddleByStemAxisCurve(int Index);

	double const RepairLength = 20;
public:
	CStemGridPointsRefine();
	~CStemGridPointsRefine();
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BeforeRefinePoints;
	CAnglePartition AnglePartitionInstance;

	void SetInput(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Input,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionsCenterPointsValue,
		CHorizontalPartition * p_HorizontalPartitionValue, double AngleValue);

	//Refine  
	//RefineType = 0 be Refined By Spline Curve; RefineType = 1 be Refined By Stem axis curve and get
	void RefineStemGridPointsBySplineCurve(bool ShowFittingPoints = false);

	void RefineStemGridPointsByStemAxisCurve(bool ShowFittingPoints = false);

	//output
	void GetRefinedPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Output);

	void GetRefinedPointsBySliceIndex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Output, int SliceIndex = 0);

	void GetBeforeRefinedPointsBySliceIndex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Output, int SliceIndex = 0);

	void GetRefinePointsForSurfaceInterpolation(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & Output);

	void GetSliceRefinePoints(int SliceIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr SliceRefinedPoints);
};

#endif
