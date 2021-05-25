#pragma once
#ifndef StemVolumeVerticalSlices_H
#define StemVolumeVerticalSlices_H

/*
stem volume calculation by vertical slices
by leiyou 2018.12.28
*/

#include <thread>

#include "Commdefinitions.h"
#include "CommPointBase.h"
#include "CommVector.h"
#include "CommGeometry.h"

#include <QMainWindow>
#include <QMessageBox>
#include <QGroupBox>

#include "TreeBase.h"

#include "GeneratedFiles/ui_StemVolumeVerticalSlices.h"

#include "HorizontalPartition.h"
#include "StemGridPointsRefine.h"
#include "StemDiameter.h"
#include "SplineSurfaceInterpolation.h"

#include  "CrustSurfaceReconstruction.h"

struct VolumeThread
{
	bool SliceDone = false;
	double BezierD = 0;
	double SliceVolumeBeforeCircle = 0;
	double SliceVolumeAfterCircle = 0;
	double SliceVolumeBeforeCylinder = 0;
	double SliceVolumeAfterCylinder = 0;

	double SliceVolumeBeforeBezier = 0;
	double SliceVolumeAfterBezier = 0;

	double SliceVolumeBeforeProfileCurve = 0;
	double SliceVolumeAfterProfileCurve = 0;
	double SliceVolumeBeforeSurface = 0;
	double SliceVolumeAfterSurface = 0;
	
	//double SliceVolumeBeforeTetra = 0;
	//double SliceVolumeAfterTetra = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BeforeBezierCurvePoints = NULL;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AfterBezierCurvePoints = NULL;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BeforeProfileCurvePoints = NULL;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AfterProfileCurvePoints = NULL;

	//Need for Surface Volumen Calculation	2019.06.03
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> BeforeSlicesProfilePoints;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> AfterSlicesProfilePoints;
};

class CStemVolumeVerticalSlices : public CTreeBase
{
	Q_OBJECT
public Q_SLOTS:
	void ShowSlicesPoints(int CheckValue);

	void AdjustAngle(int Value);
	void AdjustThickNess(int Value);

	//Repair holes of point cloud
	void FillRepairBySpline();

	void ShowRepairedPoints(int CheckValue);

	void ShowOriginalPoints(int CheckValue);

	void CalcStemByMethods();	

	void ShowProfileAfter(int CheckValue);
	void ShowProfileBefore(int CheckValue);
	void ShowBezierAfter(int CheckValue);
	void ShowBezierBefore(int CheckValue);
	void ShowSplineSurfacePoints(int CheckValue);

	void Bat();

	void SurfaceReconstruction();
	 
private:
	bool IsRepaired;
	bool IsBat;
	Ui::StemVolumeVerticalSlicesForm StemVolumeVerticalSlices;

	//double TempD;
	const string SlicesPointsStr = "SlicesPoints";
	const string RefinedStemGridPointsStr = "RefinedStemGridPoints";
	const string BeforeRefinedPointsStr = "BeforeRefinedPoints";
	const string BeforeRefineProfileCurvePointsStr = "BeforeRefineProfileCurvePoints";
	const string AfterRefineProfileCurvePointsStr = "AfterRefineProfileCurvePoints";
	const string BeforeRefineBezierCurvePointsStr = "BeforeRefineBezierCurvePoints";
	const string AfterRefineBezierCurvePointsStr = "AfterRefineBezierCurvePoints";
	const string AfterSplineSurfaceStr = "AfterSplineSurfaceStr";

	CHorizontalPartition HorizontalPartition;
	CStemGridPointsRefine StemGridPointsRefine;
	CSurfaceInterpolation SurfaceInterpolation;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AllRefinedStemGridPoints;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> InterpolationPointsForSurface;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RefinedStemGridPoints;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BeforeRefineProfileCurvePoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AfterRefineProfileCurvePoints;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BeforeRefineBezierCurvePoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AfterRefineBezierCurvePoints;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AfterSplineSurfacePoints;
	
	double CircleStemVolumeBefore, CylinderStemVolumeBefore, BezierStemVolumeBefore,
		ProfileCurveStemVolumeBefore, SliceSurfaceStemVolumeBefore;

	double CircleStemVolumeAfter, CylinderStemVolumeAfter, BezierStemVolumeAfter,
		ProfileCurveStemVolumeAfter, SliceSurfaceStemVolumeAfter;

	CStemDiameterAndProfileCurveBasalArea ProfileCurveBasalArea;

	double DRetrievalByCircleFittingFromRefinedPoints(int SliceIndex = 0);

	double DRetrievalByBezierFromRefinedPoints(int SliceIndex = 0);

	double DRetrievalByCircleFittingFromSlice(int SliceIndex = 0);
	
	//double DRetrievalByBezierFittingFromSlice(int SliceIndex = 0);

	void CalcStemDiameterErrorsAfterRefine();

	void FormReset();
	
	std::mutex g_lock;
	const int MaxThreadNum = 60;
	//int ExecuteThreadNum;
	vector<VolumeThread> VolumeThreads;
	//std::thread ThreadObj;
	//vector<std::thread> ExecuteThreads;

	void CalcStemByMethodsThread(int SliceIndex);

	double CalcVolumeByStemSplineSurface(double HStep = 0.05, double UStep = 0.05, double VStep = 0.05);
		
	string ResultFileName;
	void SaveToFile(string ProcFileName);	

	//获取用于表面重建的插值点 2020.12.26
	void GetInterpolationPointsForSurface();

	//创建Delaunay Surface 2020.12.26
	void ConstructeDelaunayTrigualtionSurface();

public:
	void OnShow();// override;
	CStemVolumeVerticalSlices(QGroupBox * ParentWin, string Type);	
	~CStemVolumeVerticalSlices();	

	void RefreshData();
};

#endif