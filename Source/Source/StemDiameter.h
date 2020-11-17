#pragma once

#ifndef StemDiameter_H
#define StemDiameter_H

#include "TreeBase.h"
#include "BezierSubsectionG2.h"
#include "Spline.h"
#include "SplineInterpolation.h"
#include "StemSkeleton.h"
#include "CylinerFitting.h"

class CStemDiameter : public CTreeBase
{
protected:
	pcl::PointXYZRGB CenterPoint;
	pcl::PointXYZRGB XYCenterPoint;
	pcl::PointXYZRGB NormalPoint;
protected:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYPoints;
public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYConvexhullPoints;

public:
	CStemDiameter();

	~CStemDiameter();

	//The output points should be transformed to themselves's original position.
	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr MeasurePointsValue,
		pcl::PointXYZRGB CenterPointValue = pcl::PointXYZRGB(), 
		pcl::PointXYZRGB NormalPointValue = pcl::PointXYZRGB());	

	//Return value is D value, not radius value
	double DRetrievalByCircleFittingIn2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePoints = NULL);

	////Return value is D value, not radius value
	//double DRetrievalByBezierCurve();

	//Return value is D value, not radius value
	double DRetrievalByCircleFittingIn2D(pcl::PointXYZRGB & CircleCenter);

	//
	double DRetrievalByConvexHullLine();

	//Return value is D value, not radius value
	double DRetrievalByBezierCurve(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePoints = NULL);

	//Return value is D value, not radius value
	double DRetrievalBySpline(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePoints = NULL);

	//Return value is D value, not radius value
	double DRetrievalByCylinerFittingIn3D(double & R, double & H, 
		pcl::PointXYZRGB Direction = (pcl::PointXYZRGB()));

	double DRetrievalByCylinerFittingIn3D(pcl::PointXYZRGB Direction = (pcl::PointXYZRGB()),
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePoints = NULL);	
};

/*

Class for calc Basal area by profile curve
leiyou
2019.01.17

*/

class CStemDiameterAndProfileCurveBasalArea : public CStemDiameter
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfilePoints;	
public:
	CStemDiameterAndProfileCurveBasalArea();
	~CStemDiameterAndProfileCurveBasalArea();

	//CRationalSpline RationalSpline;
	CSpline Spline;
	
	//Input is Profile points for profile curve and return value is basal area
	double GetProfileCurveAndArea(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfilePointsValue);

	//Input is Profile curve points pointer, and return value is basal area
	double GetProfileCurveAndArea(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempXYPoints,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurvePoints, double Angle = 5);

	double GetProfileCurveAndArea(double Angle = 5, 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurvePoints = NULL);
};

#endif 
