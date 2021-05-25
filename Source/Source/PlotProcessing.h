#pragma once

#include <qfiledialog.h>

#include "TreeBase.h"

#include "GeneratedFiles/ui_PlotProcessing.h"

#include "ContourAndConvexHull.h"
#include "CommGeometry.h"
#include "HorizontalPartition.h"
#include "DBScanCluster.h"


/*
2021.01.20 处理样地数据集



*/
typedef struct TreeStruct
{
	int TreeID;
	pcl::PointXYZRGB CenterPoint;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreeClouds;
};

class CPlotProcessing : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::FormPlotProcessing PlotProcessingForm;
	const string PlotProcessingPointsStr = "PlotProcessingPoints";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreeCoordinatePoints; //树木的坐标
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlotTreePoints; //树木的坐标
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreeConvexHullPoints; //树木的坐标的凸包点
	vector<int> ConvexHullIndexs;
	pcl::PointXYZRGB ConvexHullCentroidPoint;

	bool CurrentPointIsInConvexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
		TreeCoordinatePoints, pcl::PointXYZRGB CurPoint);
	
	vector<TreeStruct> TreeStructs;
signals:

public Q_SLOTS:
	//打开每棵树木对应的中心位置的坐标
	void OpenCoordinateFile();

	//将整个Plot点云垂直分段，并逐段保存，然后选择一个较好的段来寻找树木 2021.01.24
	void PlotVecSlice();

	//对某一个分段进行DBScan聚类，并输出其的中心位置 2020.01.25
	void ClusterForASlice();
public:	
	CPlotProcessing(QGroupBox * ParentWin);
	~CPlotProcessing();
	
	void RefreshData();
};