#pragma once

#ifndef TreePclQtGui_H
#define TreePclQtGui_H

#include <QtWidgets/QMainWindow>
#include "ui_TreePclQtGui.h"

#include <iostream>
#include <vector>
#include <string>

// Qt
#include <QMainWindow>
#include <QFiledialog>
//#include <QJsonDocument>
//#include <QJsonObject>
//#include <qDebug>
//#include <QSignalMapper>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

////UI Class
//#include "Source/WidgetForOutliersRemovalByQuantile.h"
//#include "GeneratedFiles/ui_OutliersRemovalByNeighbour.h"

//Defined Class
#include <Source/CommClass.h>
#include <Source/CommPointBase.h>
#include <Source/Commdefinitions.h>

#include <Source/PointRenderingSetting.h>
#include <Source/OutliersRemovalByRadius.h>
#include <Source/StemPointsRemovalByPlane.h>
#include <Source/StemVolumeVerticalSlices.h>
#include <Source/FittingMethods.h>
#include <Source/StemDiameterAndBasalRetrieval.h>
#include <Source/PointUnit.h>
#include <Source/GeneratePoints.h>
#include <Source/TreeStubDetection.h>
#include <Source/BranchRemovalByTangentPlane.h>
#include <Source/PointDownSample.h>
#include <Source/BranchRemovalByTangentPlaneAndStemAxisCurve.h>
#include "Source/BranchRemovalByClusterAndCurvature.h"
#include "Source/BranchesClassficationByDBScan.h"
#include "Source/DelaunayParallel.h"
#include "Source/SimulatingMeasure/SimulateMeasure.h"

#include <direct.h>

#include <Source/Demo.h>
#include <Source/Cluster.h>

//pcl_plotter draw curve, may be used for draw circle Bezier and spline.

//The event for pcl  viewer point picking

class CustomInteractor : public
	pcl::visualization::PCLVisualizerInteractorStyle
{
	void OnChar() override {}
	void OnKeyDown() override {}
	void OnKeyUp() override {}
};

namespace Ui
{
	class TreePclQtGui;
}

class TreePclQtGui : public QMainWindow
{
	Q_OBJECT

	//当前视图窗体中显示的点云名称
	std::vector<string> PointCloudStrs;

public:
	TreePclQtGui(QWidget *parent = Q_NULLPTR);
	~TreePclQtGui();
	
	int PointSize;
signals:
	void UpdateUI();
	void UpdateStatus(QString, int);
	void UpdateTitle(string FileName);
	void ShowMsg(string Msg);
	void UpdateResetCamera();
public Q_SLOTS:
	//void randomButtonPressed();

	//void RGBsliderReleased();

	//void pSliderValueChanged(int value);

	//open point cloud file
	void OpenCloudByFile(QString FileName = "");

	void Reload();

	void SaveCloudtoFile();

	void SaveAsCloudtoFile();

	//save current view to picture
	void SaveViewerAsPicture();

	void Reset_Camera();

	void UpdateCloudAndUI();	

	void ResetCamera();

	void UpdateAppTitle(string FileName);

	void ShowMessage(string Msg);

	//2019.03.17
	void TreeStubRemoval();

	//2019.03.26
	void BranchRemovalByTangentPlane();

	//2019.06.13
	void BranchRemovalByTangentPlaneAndStemAxisCurve();

	//2019.09.23
	void BranchesRemovalClusterAndCurvature();

	//Open the form for Outliers Removal
	void OutliersRemoval();	

	//2019.01.21 Retrieval stem Form Parameters
	void StemDiameterAndBasalRetrieval();

	//2020.02.12 Slice DBScan 
	void BranchesClassficationSliceDBScan();

	//remove points by plane Equation
	void StemPointsRemovalByPlaneEquation();

	//Stem Volume Calculation by Circle Fitting (profile curve) of vertical slices
	void StemVolumeCalculationByVerticalSlices();

	void TestMsg();

	void SetStatusText(QString StrValue, int TimeOut);

	void ShowPointRenderingSetting();

	void DownSample();

	void ShowFittingMethodsForm();

	void ShowDemoForm();

	void ShowDelaunayParallelForm();

	void ShowSimulateMeasureForm();

	void ShowClusterForm();

	void GeneratePoints();

	void OpenAndInsert_And_Move();
protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;
	
	//Main Data For Processing
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreeStemCloud;
private:
	QLabel * VersionLabel;
	QLabel * FirstLabelofStatus;
	QLabel * SecondLabelofStatus;
	
	static TreePclQtGui * p_Self;
	Ui::TreePclQtGuiClass Mainui;
	const string TreeStemCloudStr = "TreeStemCloud";
	const string AppTitle = "3D Stem Modelling and Measurement for a Single Tree";
	
	string AppPath;
	string AppFormTitle;
	string OpenedFilePath;

	void parseArguments();

	string PointCloudFile;	

	//All forms used in this class shoule be definited here
	Ui::OutliersRemovalByNeighbourForm SubuiOfOutliersRemoval;	

	//CTreeBase * p_TreeBase;
	boost::shared_ptr<CTreeBase> p_TreeBase;
	
	//remove all components of groupBox
	void RemoveGroupBoxItems();

	int SelectedPointIndex;

	void static mousePointPickingOccurred(const pcl::visualization::PointPickingEvent &event, void* Viewer_void);
	void static mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* Viewer_void);
	void static keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* Viewer_void);
};

#endif