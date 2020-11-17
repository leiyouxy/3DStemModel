#pragma once

/*

*/

#include "CommPointBase.h"
#include "DBScanCluster.h"
#include "HorizontalPartition.h"
#include "PointGeometry.h"
#include "BezierSubsectionG2.h"

typedef struct SliceCluster
{
	CDBScanCluster * pCluster;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints;
};

class CDBScanTreeSkeleton
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SkeletonPoints;

	const string SliceClusterPointsStr = "SliceClusterPoints";

	vector<SliceCluster> SliceDBScanClusters;	
	//是否使用 ConvexCentroid 为中心点，如果是则使用，否则使用 Centroid
	bool UseConvexCentroid;

	double SliceHeight;
	double Radius;
	int MinPts;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;

	CHorizontalPartition HorizontalPartition;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *OctreeCloudSearch;
	

	//当前正在使用的函数  //根据点集的连续性获取父节点 2020.04.11
	bool GetParentSliceCluster(int SliceIndex, int ClusterIndex, double SearchRadius, bool Bottom = true);

	//By continuity of the direction 2020.02.25
	bool GetParentSliceClusterByDirectionContinuity(int SliceIndex, int ClusterIndex);

	//如果GetParentSliceCluster 获取不到Parent，只好采用最近距离来获取 最后的方法 
	//此方法暂未使用
	void GetParentSliceClusterByNearestDis();	

	//根据树干的连续性获取树干的连续部分 2020.04.07
	//AllowAngle 为判断 聚类点 连续性的最小角度值
	void FindSuccessivePart(int SuccessiveNum, double AllowAngle);

	////使用分割树枝凸包多边形分割树木，使用SliceIndex的ClusterIndex的父节点的信息 
	////对 SliceIndex的ClusterIndex 进行凸包多边形分割 2020.04.08
	void SegmentationByConvexPolygon(double Thick, int SuccessiveNum, 
		int SliceIndex, int ClusterIndex, double AllowDis);

	//根据过滤掉的树杈点做聚类，并添加到之前的聚类容器中
	void CreateNewClusterForSliceByBranchesPoints(int SliceIndex, int ClusterIndex,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr BranchesPoints,
		vector<int> BranchesPointsIndexS);

	//使用凸包多边形分割点集，AllowDis 为凸包多边形的膨胀距离 返回需要不在凸包多边形内的点集索引 2020.04.08
	vector<int> SegmentationPointsByConvexPolygon(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexPolygon3DPoints,		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ZonePoints,
		vector<int> ZonePointsIndexs, 
		pcl::PointXYZRGB Direction,
		pcl::PointXYZRGB BottomPoint, double AllowDis);

	//获取SliceIndex中ClusterIndex的点集
	void GetSliceClusterPoints(int SliceIndex, int ClusterIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr SliceClusterPoints);

	//计算当前位置处的生长方向 2020.04.09
	bool CalcCurDirection(int SuccessiveNum, int SliceIndex, 
		int ClusterIndex, pcl::PointXYZRGB & CurDirection);

	//移除在修复过程中产生的Cluster 2020.04.09	 
	void RemoveRepairdCluster();

	//移除分支处的连接关系 2020.04.11
	void RemoveBrancheRoles();
public:
	CDBScanTreeSkeleton();
	~CDBScanTreeSkeleton();

	CPointGeometry PointGeometry;

	//噪声点的索引 2020.02.24
	vector<int> OutliersIndexs;

	bool SkeletonIsGenerate;

	void ResetSliceDBScanClusters();

	int GetSlicesClustersCount();
	
	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud,
		boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue);

	//探测树木相互连通的若干主要部分
	void MainTreeBranchesDetection(double RadiusValue, int MinPtsValue);

	//提取树木相互连通的 Top 个主要部分，一个部分是一个DBScan的数据点集
	void MainTreeBranchesRetrieval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud, int Top);

	//在不同高度Slice上聚类
	void SlicesClusters(double SliceHeightValue, double RadiusValue, int MinPtsValue, 
		bool Show = true, int PointSize = 2);

	//根据SlicesClusters间的相邻关系构建树的Skeleton
	void SkeletonGenerate(double SliceHeightValue, double RadiusValue, int MinPtsValue, bool IsUseConvexCentroid = true);

	//标注Skeleton点分别属几级枝
	void LableSkeleton();

	//显示Skeleton节点及节点间的连接关系
	void ShowSkeleton(bool ShowSkeletonPoint = true, bool ShowSkeleton = true, int PointSize = 2,
		string SkeletonPointsStr = "SkeletonPointsStr");

	void ShowSliceClusters(bool ShowPoints = true, bool IsByID = false, int PointSize = 2);
	
	//检测包含的噪声点
	void OutliersDetection(double SliceHeightValue, double RadiusValue, int MinPtsValue);

	void OutliersRemoval();

	void NormalComputation();

	//根据树干的连续性修改树干骨架中的点，并明确每个点所在树干的层次关系 2020.02.26
	//AllowAngle 为判断 聚类点 连续性的最小角度值
	void SkeletonPointRepair(double Thick, int SuccessiveNum, double AllowAngle, double AllowDis);
};
//2020.04.09 先从分支的顶端入手，逐个寻找每个分支的点集，