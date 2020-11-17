#pragma once
#ifndef TreeBase_H
#define TreeBase_H
/*

The base class for tree point cloud processing 2018.12.15 by leiyou

*/

#include <QMainWindow>
#include <QMessageBox>
#include <QGroupBox>

#include "Commdefinitions.h"

extern class TreePclQtGui;

class CTreeBase : public QObject
{
	Q_OBJECT
protected:
	//The main point cloud data for data processing and used in all the inherited class
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RedoCloud;

	//The viewer form for point cloud rendering
	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;
	double ZMax, ZMin, YMax, YMin, XMax, XMin;	
public:
	//The main form pointer
	TreePclQtGui * p_TreePclQtGui;
	int PointSize;
	string OpenedFilePath;
	string FileName;
	string AppPath;
	string ClassName;

	CTreeBase();
	~CTreeBase();

	void SaveToRedo();

	//Using external data pointer directly.
	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreePointsValue);		
	
	virtual void SetViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue);
	

	//control the main form 
	void emitUpdateUI();
	void emitUpdateStatusBar(QString StrValue, int TimeOut = 5000);
	void emitUpdateAppTitle(string FileName);
	void emitResetCamera();
	void emitShowMsg(string Msg);
	virtual void RefreshData();//procedure for open new point cloud file	
signals:	
	void PointIndex(int);	

public Q_SLOTS:	
	void GetPointIndex(int IndexValue);
	void ShowPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, string PointsStr, 
		int PointSize = 2, bool ShowIndex = false, double IndexScale = 0.01);
	void ShowNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, 
			pcl::PointCloud<pcl::Normal>::Ptr Normals, string PointsStr, int Level = 1, float Scale = 1);
	void ShowPrincipalCurvatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
		pcl::PointCloud<pcl::Normal>::Ptr Normals,
		pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr PrincipalCurvatures, 
			string PointsStr, int Level = 1, float Scale = 1);
	virtual void Redo();
private:
	int SelectedPointIndex;
};

#endif