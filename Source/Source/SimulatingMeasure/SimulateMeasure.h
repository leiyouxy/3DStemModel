#pragma once

#include "../TreeBase.h"

#include "Source/HorizontalPartition.h"
#include "Source/AnglePartition.h"
#include "Source/CylinerFitting.h"
#include "Source/ContourAndConvexHull.h"
#include "Source/BezierSubsectionG2.h"
#include "Source/SplineInterpolation.h"
#include "Source/StemSkeleton.h"

#include "GeneratedFiles/ui_SimulatingMeasure.h"

class CSimulateMeasure : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	bool IsBat;
	string ResultFileName;
	Ui::FormSimulating SimulatingForm;
	const string SimulatingPointsStr = "SimulatingPoints";
	const string PlanarPointsStr = "PlanarPoints";
	const string CircleFittingPointsStr = "CircleFittingPoints";
	const string CylinderFittingPointsStr = "CylinderFittingPoints";
	const string ConvexHullLineStr = "ConvexHullLine";
	const string SplineFittingStr = "SplineFitting";
	const string BezierFittingStr = "BezierFitting";
	const string ConvexHullPointsStr = "ConvexHullPoints";	

	//CHorizontalPartition HorizontalPartition;
	pcl::PointXYZRGB CylinderCenterPoint, CircleCenter, D3Center, D2Center;
	//计算的点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SimulatingPoints;
	//计算的平面点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr D3PlanarPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr D2PlanarPoints;

	//计算的平面点云的凸包点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexHullPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr D2ConvexHullPoints;

	//Cylinder曲线点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CylinderFittingPoints;

	//Circle曲线点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CircleFittingPoints;

	//BSpline曲线点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BSplineFittingPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BSplineProfilePoints;

	//Bezier曲线点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierCurvePoints;

	//树干所在方向, AnchorPoint是获取计算点云的 锚点
	pcl::PointXYZRGB GrownDirection, AnchorPoint;

	//计算点云所在树干的生长方向
	void CalculateDriection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempStemPoints, 
		pcl::PointXYZRGB & TempStemGrowth);

	//模拟卡尺测量的多个直径值
	vector<double> CaliperDs;
	
	double AngleSpace = 15;
	double Height, MeasureLength, HCylinder;
	double DCaliper, DCaliperMax, DCaliperMin, DCaliperOvality, DCylinder, DCircle;
	double DConvexLine, DBSpline, DBezierConvex;
	double BACylinder, BACircle, BAConvexLine, BASpline, BAezierConvex;
	double ProfileArea;

	//根据基础断面轮廓点，采用 BSpline样条生成断面轮廓曲线，然后再生成一段树干点云 2020.08.11
	void GenerateSimulatePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr BaseProfilePoints, 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutStemPoints,
		pcl::PointXYZRGB AxisDirection, double Height, double HeightInterval = 0.5, 
		double AngleSpace = 0.5, bool HaveGaussianNoise = false, 
		float NoiseMean = 0, float NoiseStddev = 0);

	//模拟卡尺测量 2020.08.13
	void CaliperMeasure();

	void SaveValueToFile(string ProcFileName);

	//寻找距离 CurPoint 点最近的一个凸包点 2020.08.13
	int FindNearestConvexHull(pcl::PointXYZRGB CurPoint);
signals:

public Q_SLOTS:
	void ShowModelPoints(int CheckValue);

	void StemDividingByStemAxisCurve();

	//批处理每一个树干得到若干树干段 2020.11.09
	void BatStemDividing();

	//2020.11.09 废弃不用
	//void StemDividingByHorizontalPartition();

	void SimulateMeasure();

	void BatSimulateMeasure();
public:	
	CSimulateMeasure(QGroupBox * ParentWin);
	~CSimulateMeasure();
	void RefreshData();
};