#pragma once
#ifndef BranchRemoval_H
#define BranchRemoval_H

#include "TreeBase.h"
#include "HorizontalPartition.h"
#include "AnglePartition.h"

#include "GeneratedFiles/ui_BranchRemovalByTangentPlane.h"

//每个角度分区的切平面参数数据
typedef struct AnglePartitionTargentInfo
{
	bool Calculated;	// is calculated 
	int AngleIndex;		//角度分区的索引	
	double Symbol; //定义符号，符号的正负由切平面的参数及几何中心点确定，然后在相反的符号测移除树杈点
	double a;
	double b;
	double c;
	double d;	
			
	double LineParameterSlope;			//直线参数方程的参数，此直线是过几何中心点与角度分区重心点的直线方程
	double DisFromCenterToGeoCenter;	//2015.08.15 几何中心点与角度分区重心点的距离	
	double ExtendValue;					//2016.03.17 计算切平面位置时使用的延长点的距离值
	pcl::PointXYZRGB CenterPoint;		//计算切平面时用到的重心点
	pcl::PointXYZRGB TargentPoint;		//定位切平面时用到的点（延长点），此点与重心点在同一条直线上，且距离重心点距离为L（程序设定）
			//此直线由重心点与中心线的xy与重心点的z值确定	
};

//每个分区的角度切平面数据
typedef struct SectionTargentInfo
{
	int SectionIndex;
	double AvgDis;					//2015.08.15 统计 计算切平面时用到的重心点 到 几何中心点 的平均距离
	double DisVariance;				//2015.08.15 上述距离的方差
	vector<AnglePartitionTargentInfo> AngleSTangentInfo;
};


class CBranchRemovalByTangentPlane : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::BranchRemovalByTangentPlaneForm BranchRemovalForm;
	const string BranchPointsStr = " BranchPoints";
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints;
	const string HeightPlanePointsStr = "HeightPlanePoints";

	const string CenterPointsPointsStr = "CenterPointsPoints";

	//多个分区的角度划分
	CAnglePartition AnglePartition;
	CHorizontalPartition HorizontalPartition;

	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> * Octree;

	//多个垂直分区的角度分区的切平面数据
	vector<SectionTargentInfo> SectionSTangentInfo;
	int SearchStartIndex;
	int SuccessiveSectionNumber;
	double Thickness;
	double Angle;	
	double AlongDistance;
	bool IsUp;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CenterPointsPtr;
	const string CenterPointsStr = "CenterPointsStr";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TargentPointsPtr;
	const string TargentPointsStr = "TargentPointsStr";

	//Except for the stem points, all other points are outliers points
	int OutliersColor;

	void GetParameters();

	void Initial();

	//2015.07.30 移除当前分区的树杈信息，	
	void MarkSectionsBranches(int SectionIndex);

	//2015.07.31 计算 SectionIndex 分区 的切平面参数
	void CalcSectionSTangent(int SectionIndex);

	void CalcSectionSAnglePartitionTargent(int SectionIndex,
		int AnglePartitionIndex);

	void GetPoint(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr,
		int SectionIndex,
		int AnglePartitionIndex);

	//获取计算切平面的点云数据集
	void GetTargentPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr AnglePartitionIndexPointPtr,	//当前角度分区的点云
		int SectionIndex,
		int AnglePartitionIndex,
		int AnglePartitions = 3,	//使用角度分区的个数，此参数应该为奇数，
		int Sections = 10,			//使用垂直分区的个数，
		int SectionSpace = 10);		//与现分区的间隔数
	
	//void UpdateGeometricalCenterAndReAnglePartition(int SectionIndex);
	void ShowAngularCenterPoints();

signals:

public Q_SLOTS:
	void ShowSlicesPoints(int CheckValue);

	void ShowHeightPlane(int Value);

	void CheckBranches();

	void Redo();

	void RemoveBranches();
public:	
	CBranchRemovalByTangentPlane(QGroupBox * ParentWin);
	~CBranchRemovalByTangentPlane();

	void SetViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue);
	void RefreshData();
};

#endif