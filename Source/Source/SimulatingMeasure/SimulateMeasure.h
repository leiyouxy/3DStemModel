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
	//����ĵ���
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SimulatingPoints;
	//�����ƽ�����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr D3PlanarPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr D2PlanarPoints;

	//�����ƽ����Ƶ�͹����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexHullPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr D2ConvexHullPoints;

	//Cylinder���ߵ���
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CylinderFittingPoints;

	//Circle���ߵ���
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CircleFittingPoints;

	//BSpline���ߵ���
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BSplineFittingPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BSplineProfilePoints;

	//Bezier���ߵ���
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierCurvePoints;

	//�������ڷ���, AnchorPoint�ǻ�ȡ������Ƶ� ê��
	pcl::PointXYZRGB GrownDirection, AnchorPoint;

	//��������������ɵ���������
	void CalculateDriection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempStemPoints, 
		pcl::PointXYZRGB & TempStemGrowth);

	//ģ�⿨�߲����Ķ��ֱ��ֵ
	vector<double> CaliperDs;
	
	double AngleSpace = 15;
	double Height, MeasureLength, HCylinder;
	double DCaliper, DCaliperMax, DCaliperMin, DCaliperOvality, DCylinder, DCircle;
	double DConvexLine, DBSpline, DBezierConvex;
	double BACylinder, BACircle, BAConvexLine, BASpline, BAezierConvex;
	double ProfileArea;

	//���ݻ������������㣬���� BSpline�������ɶ����������ߣ�Ȼ��������һ�����ɵ��� 2020.08.11
	void GenerateSimulatePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr BaseProfilePoints, 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutStemPoints,
		pcl::PointXYZRGB AxisDirection, double Height, double HeightInterval = 0.5, 
		double AngleSpace = 0.5, bool HaveGaussianNoise = false, 
		float NoiseMean = 0, float NoiseStddev = 0);

	//ģ�⿨�߲��� 2020.08.13
	void CaliperMeasure();

	void SaveValueToFile(string ProcFileName);

	//Ѱ�Ҿ��� CurPoint �������һ��͹���� 2020.08.13
	int FindNearestConvexHull(pcl::PointXYZRGB CurPoint);
signals:

public Q_SLOTS:
	void ShowModelPoints(int CheckValue);

	void StemDividingByStemAxisCurve();

	//������ÿһ�����ɵõ��������ɶ� 2020.11.09
	void BatStemDividing();

	//2020.11.09 ��������
	//void StemDividingByHorizontalPartition();

	void SimulateMeasure();

	void BatSimulateMeasure();
public:	
	CSimulateMeasure(QGroupBox * ParentWin);
	~CSimulateMeasure();
	void RefreshData();
};