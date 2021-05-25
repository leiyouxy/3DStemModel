#pragma once

#include <qfiledialog.h>

#include "TreeBase.h"

#include "GeneratedFiles/ui_PlotProcessing.h"

#include "ContourAndConvexHull.h"
#include "CommGeometry.h"
#include "HorizontalPartition.h"
#include "DBScanCluster.h"


/*
2021.01.20 �����������ݼ�



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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreeCoordinatePoints; //��ľ������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlotTreePoints; //��ľ������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreeConvexHullPoints; //��ľ�������͹����
	vector<int> ConvexHullIndexs;
	pcl::PointXYZRGB ConvexHullCentroidPoint;

	bool CurrentPointIsInConvexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
		TreeCoordinatePoints, pcl::PointXYZRGB CurPoint);
	
	vector<TreeStruct> TreeStructs;
signals:

public Q_SLOTS:
	//��ÿ����ľ��Ӧ������λ�õ�����
	void OpenCoordinateFile();

	//������Plot���ƴ�ֱ�ֶΣ�����α��棬Ȼ��ѡ��һ���ϺõĶ���Ѱ����ľ 2021.01.24
	void PlotVecSlice();

	//��ĳһ���ֶν���DBScan���࣬������������λ�� 2020.01.25
	void ClusterForASlice();
public:	
	CPlotProcessing(QGroupBox * ParentWin);
	~CPlotProcessing();
	
	void RefreshData();
};