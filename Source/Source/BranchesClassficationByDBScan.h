#pragma once

#include "TreeBase.h"

#include "GeneratedFiles/ui_BranchClassificationByDBScan.h"

#include "DBScanCluster.h"
#include "HorizontalPartition.h"
#include "DBScanTreeSkeleton.h"

//typedef struct SliceClusterIndex
//{
//	int SliceIndex;
//	int ClusterIndex;
//};


class CBranchesClassficationByDBScan : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::FormBranchClassficationByDBScan Form;

	CDBScanCluster DBScanCluster;
	CDBScanTreeSkeleton DBScanTreeSkeleton;
	//bool HaveDone;

	//多个Slice的DBScanCluster；
	vector<SliceCluster> SliceDBScanClusters;

	double SliceHeight;

	CHorizontalPartition HorizontalPartition;
	const string MainBranchesPointsStr = "MainBranchesPoints";
	const string SliceClusterPointsStr = "SliceClusterPoints";
	const string NodePointsStr = "NodePoints";
	//const string NodePointsNormalStr = "NodePointsNormal";
		
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutliersPoints;
	const string OutliersPointsStr = "OutliersPointsStr";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr NodePoints;

	//重新设置 SliceDBScanClusters 的数据
	void ResetSliceDBScanClusters();

	//Calc SliceNum  by thickness and show in form
	void CalcSliceNumAndShow();

	//根据Label的值Value获取最大的MaxID
	int FindMaxID(string Value);

	//根据不同层级设置不同的颜色
	void SetColors();
		
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *OctreeCloudSearch;
	
	////根据点集的连续性获取父节点
	//bool GetParentSliceCluster(int SliceIndex, int ClusterIndex, double SearchRadius, bool Bottom = true);

	////如果GetParentSliceCluster 获取不到Parent，只好采用最近距离来获取 最后的方法
	//void GetParentSliceClusterByNearestDis();

	void Redo();
signals:

public Q_SLOTS:
	void MainTreeBranchesDetection();

	//retrieval method of Main branches
	void RetrievalMainTree();
		
	void ShowBranchesPoints(int State);

	//void ShowMainClusterPoints(int State);
	void ShowSlicePoint(int State);

	void ShowShowNormal(int State);

	//sHow slice cluster after slice clustering
	void ShowSliceClusters(int State);

	//Show skeleton points, skeleton or both.
	void ShowSkeleton(int State);

	//改变Slice Num时 SliceThickNess跟着调整
	void SliceNumChange(int Num);

	//在各个垂直分段Slice上执行DBScan算法得到一个垂直分段的1个或多个Cluster	
	void GetSlicesClusters();

	void SlicesCluster();
		
	void SkeletonGenerate();

	void ReSlice(double Height);

	void SuccessiveAngleChange(double AngleValue);
	void SuccessiveNumChange(int NumValue);

	void OutliersDetection();

	void OutliersRemoval();

	//根据树干的连续性修改树干骨架中的点，并明确每个点所在树干的层次关系
	void SkeletonPointRepair();

	void NormalComputation();
public:
	CBranchesClassficationByDBScan(QGroupBox * ParentWin);
	~CBranchesClassficationByDBScan();

	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreePointsValue); 

	void RefreshData();	
};