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
	//�Ƿ�ʹ�� ConvexCentroid Ϊ���ĵ㣬�������ʹ�ã�����ʹ�� Centroid
	bool UseConvexCentroid;

	double SliceHeight;
	double Radius;
	int MinPts;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;

	CHorizontalPartition HorizontalPartition;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *OctreeCloudSearch;
	

	//��ǰ����ʹ�õĺ���  //���ݵ㼯�������Ի�ȡ���ڵ� 2020.04.11
	bool GetParentSliceCluster(int SliceIndex, int ClusterIndex, double SearchRadius, bool Bottom = true);

	//By continuity of the direction 2020.02.25
	bool GetParentSliceClusterByDirectionContinuity(int SliceIndex, int ClusterIndex);

	//���GetParentSliceCluster ��ȡ����Parent��ֻ�ò��������������ȡ ���ķ��� 
	//�˷�����δʹ��
	void GetParentSliceClusterByNearestDis();	

	//�������ɵ������Ի�ȡ���ɵ��������� 2020.04.07
	//AllowAngle Ϊ�ж� ����� �����Ե���С�Ƕ�ֵ
	void FindSuccessivePart(int SuccessiveNum, double AllowAngle);

	////ʹ�÷ָ���֦͹������ηָ���ľ��ʹ��SliceIndex��ClusterIndex�ĸ��ڵ����Ϣ 
	////�� SliceIndex��ClusterIndex ����͹������ηָ� 2020.04.08
	void SegmentationByConvexPolygon(double Thick, int SuccessiveNum, 
		int SliceIndex, int ClusterIndex, double AllowDis);

	//���ݹ��˵�����辵������࣬����ӵ�֮ǰ�ľ���������
	void CreateNewClusterForSliceByBranchesPoints(int SliceIndex, int ClusterIndex,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr BranchesPoints,
		vector<int> BranchesPointsIndexS);

	//ʹ��͹������ηָ�㼯��AllowDis Ϊ͹������ε����;��� ������Ҫ����͹��������ڵĵ㼯���� 2020.04.08
	vector<int> SegmentationPointsByConvexPolygon(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexPolygon3DPoints,		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ZonePoints,
		vector<int> ZonePointsIndexs, 
		pcl::PointXYZRGB Direction,
		pcl::PointXYZRGB BottomPoint, double AllowDis);

	//��ȡSliceIndex��ClusterIndex�ĵ㼯
	void GetSliceClusterPoints(int SliceIndex, int ClusterIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr SliceClusterPoints);

	//���㵱ǰλ�ô����������� 2020.04.09
	bool CalcCurDirection(int SuccessiveNum, int SliceIndex, 
		int ClusterIndex, pcl::PointXYZRGB & CurDirection);

	//�Ƴ����޸������в�����Cluster 2020.04.09	 
	void RemoveRepairdCluster();

	//�Ƴ���֧�������ӹ�ϵ 2020.04.11
	void RemoveBrancheRoles();
public:
	CDBScanTreeSkeleton();
	~CDBScanTreeSkeleton();

	CPointGeometry PointGeometry;

	//����������� 2020.02.24
	vector<int> OutliersIndexs;

	bool SkeletonIsGenerate;

	void ResetSliceDBScanClusters();

	int GetSlicesClustersCount();
	
	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud,
		boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue);

	//̽����ľ�໥��ͨ��������Ҫ����
	void MainTreeBranchesDetection(double RadiusValue, int MinPtsValue);

	//��ȡ��ľ�໥��ͨ�� Top ����Ҫ���֣�һ��������һ��DBScan�����ݵ㼯
	void MainTreeBranchesRetrieval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud, int Top);

	//�ڲ�ͬ�߶�Slice�Ͼ���
	void SlicesClusters(double SliceHeightValue, double RadiusValue, int MinPtsValue, 
		bool Show = true, int PointSize = 2);

	//����SlicesClusters������ڹ�ϵ��������Skeleton
	void SkeletonGenerate(double SliceHeightValue, double RadiusValue, int MinPtsValue, bool IsUseConvexCentroid = true);

	//��עSkeleton��ֱ�������֦
	void LableSkeleton();

	//��ʾSkeleton�ڵ㼰�ڵ������ӹ�ϵ
	void ShowSkeleton(bool ShowSkeletonPoint = true, bool ShowSkeleton = true, int PointSize = 2,
		string SkeletonPointsStr = "SkeletonPointsStr");

	void ShowSliceClusters(bool ShowPoints = true, bool IsByID = false, int PointSize = 2);
	
	//��������������
	void OutliersDetection(double SliceHeightValue, double RadiusValue, int MinPtsValue);

	void OutliersRemoval();

	void NormalComputation();

	//�������ɵ��������޸����ɹǼ��еĵ㣬����ȷÿ�����������ɵĲ�ι�ϵ 2020.02.26
	//AllowAngle Ϊ�ж� ����� �����Ե���С�Ƕ�ֵ
	void SkeletonPointRepair(double Thick, int SuccessiveNum, double AllowAngle, double AllowDis);
};
//2020.04.09 �ȴӷ�֧�Ķ������֣����Ѱ��ÿ����֧�ĵ㼯��