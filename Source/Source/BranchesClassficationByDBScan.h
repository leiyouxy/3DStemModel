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

	//���Slice��DBScanCluster��
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

	//�������� SliceDBScanClusters ������
	void ResetSliceDBScanClusters();

	//Calc SliceNum  by thickness and show in form
	void CalcSliceNumAndShow();

	//����Label��ֵValue��ȡ����MaxID
	int FindMaxID(string Value);

	//���ݲ�ͬ�㼶���ò�ͬ����ɫ
	void SetColors();
		
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *OctreeCloudSearch;
	
	////���ݵ㼯�������Ի�ȡ���ڵ�
	//bool GetParentSliceCluster(int SliceIndex, int ClusterIndex, double SearchRadius, bool Bottom = true);

	////���GetParentSliceCluster ��ȡ����Parent��ֻ�ò��������������ȡ ���ķ���
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

	//�ı�Slice Numʱ SliceThickNess���ŵ���
	void SliceNumChange(int Num);

	//�ڸ�����ֱ�ֶ�Slice��ִ��DBScan�㷨�õ�һ����ֱ�ֶε�1������Cluster	
	void GetSlicesClusters();

	void SlicesCluster();
		
	void SkeletonGenerate();

	void ReSlice(double Height);

	void SuccessiveAngleChange(double AngleValue);
	void SuccessiveNumChange(int NumValue);

	void OutliersDetection();

	void OutliersRemoval();

	//�������ɵ��������޸����ɹǼ��еĵ㣬����ȷÿ�����������ɵĲ�ι�ϵ
	void SkeletonPointRepair();

	void NormalComputation();
public:
	CBranchesClassficationByDBScan(QGroupBox * ParentWin);
	~CBranchesClassficationByDBScan();

	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreePointsValue); 

	void RefreshData();	
};