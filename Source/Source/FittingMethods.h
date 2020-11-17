#pragma once
#ifndef FittingMethods_H
#define FittingMethods_H

/*
2019.01.11

For rendering curve fitting or surface fitting effectiveness

Leiyou 
*/
#include <QMessageBox>

#include "TreeBase.h"
#include "CommGeometry.h"
#include "CylinerFitting.h"
#include "SplineInterpolation.h"
#include "BezierSubsectionG2.h"

#include "GeneratedFiles/ui_FittingMethods.h"

class CFittingMethods : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::FittingMethodsForm FittingMethodsForm;
	const string CirclePointsStr = "CirclePoints";
	const string CylinderPointsStr = "CylinderPoints";
	const string SplinePointsStr = "SplinePoints";
	const string BezierPointsStr = "BezierPoints";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CirclePoints;
	double CylinderR, CylinderH;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CylinderPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SplinePoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierPoints;

	bool SplineIsClosure, BezierIsClosure;	
signals:

public Q_SLOTS:
	void CircleFitting();
	void ShowCirclePoints(int checkState);

	void CylinderFitting();
	void ShowCylinderPoints(int checkState);

	void SplineFitting();
	void SplineClosureChange(int checkState);
	void ShowSplinePoints(int checkState);

	void BeizerFitting();
	void BezierClosureChange(int checkState);
	void ShowBezierPoints(int checkState);
	
public:
	CFittingMethods();
	CFittingMethods(QGroupBox * ParentWin);
	~CFittingMethods();

	void RefreshData();
};

#endif