#pragma once

#include "TreeBase.h"

#include <QFiledialog>

#include "GeneratedFiles/ui_DiameterAndBasalRetrieval.h"
#include "HorizontalPartition.h"
#include "StemSkeleton.h"
#include "StemDiameter.h"

class CStemDiameterAndBasalRetrieval : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::StemDiameterAndBasalRetrieval StemDiameterAndBasalRetrievalForm;
	const string SlicesPointsStr = "SlicesPoints";
	const string StemAxisCurvePointsStr = "StemAxisCurvePoints";
	const string StemCrossSectionPointsStr = "StemCrossSectionPoints";
	const string CalcingStemPointsStr = "CalcingStemPoints";
	const string SplineFittingPointsStr = "SplineFittingPoints";
	const string BezierFittingPointsStr = "BezierFittingPoints";
	const string ProfileCurvePointsStr = "ProfileCurvePoints";

	CHorizontalPartition HorizontalPartition;
	CStemSkeleton StemSkeleton;
		
	double StemHeight, StemLength;
	double CurrentPointU;
	pcl::PointXYZRGB CurrentPoint;
	pcl::PointXYZRGB CurrentUpperPoint;
	pcl::PointXYZRGB CurrentNormal;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CalcingStemPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CopyCalcingStemPoints;
		
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SplineFittingPoints;	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierFittingPoints;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurvePoints;

	//CStemDiameter StemDiameter;
	CStemDiameterAndProfileCurveBasalArea StemDiameter;
	bool IsBat;
signals:
	 
public Q_SLOTS:
	
	void ResetForm();
	void ShowSlicesPoints(int CheckValue);	
	void ShowSlicesPoints(double thickNess);

	void ShowStemAxisCurvePoints(int CheckValue);
	void ShowCrossSectionalPoints(int CheckValue);
	void ShowCalcingStemPoints(int CheckValue);

	void ShowSplineFittingPoints(int CheckValue);
	void ShowBezierFittingPoints(int CheckValue);

	void ShowProfileCurvePoints(int CheckValue);

	void SaveCalcingStemPoints();

	void StemAxisCurveConstruct();

	void ChangeCalcPosition();

	void RadioPositionCheck(bool Checked);

	//U Value is determined by stem height or length. 2019.01.23 
	void GetPointUValue();
	void CalcingStemParameters();

	void CalcingBasalArea();

	//2021.02.02 Bat 方式获取Slice
	void BatGetSlice();
public:
	CStemDiameterAndBasalRetrieval();
	CStemDiameterAndBasalRetrieval(QGroupBox * ParentWin);
	~CStemDiameterAndBasalRetrieval();

	void RefreshData();
};
