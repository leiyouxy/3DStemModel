#pragma once

/*
add by leiyou 2019.09.05

Ϊ���� dbScan ����������

dbScan �� Paper ��DBSCAN modicado con Octrees para agrupar nubes de puntos en tiempo real���е�Source Code

Ȼ�� �ó���������ֻ�е�����꣬�����ǵ����������Ϊ������������ܶ��鷳�������Լ�����дһ�� 2019.09.05

*/
#include "CommPointBase.h"
#include "CommGeometry.h"
#include "PointGeometry.h"
#include "HorizontalPartition.h"

////�ڵ��Ĺ�����ϵ RelationShip

//�ڵ�Ĺ�ϵ
typedef struct Role
{
	int SliceIndex;
	int ClusterIndex;
};

typedef struct NodeRole
{
	Role ParentNode;		//Parent Node
	//int ParentSliceIndex;		//���ڵ����ڵ�Slice, �п��ܲ���ֱ�ӵ���һ��
	//int ParentClusterIndex;			//��һ��Slice��ĳһ����Index��Ϊ�丸�ڵ�
	vector<Role> ChildNodes;	//
};

typedef struct 
{	
	int ID;							//����
	int IsOutliers;					//�Ƿ���������
	//int IsJoint;					// is the joint point between the branches and stem, 
	//					can be judged by NodeRoles.ChildNodes.count();
	vector<int> Indexs;				//�������
	vector<int> GlobalIndexs;		//���ȫ��������ʹ���ڶ��Slice�����ʱ��
	string Label;					//�ַ�����ǩ
	NodeRole NodeRoles;				//�洢����Slice�䲻ͬCluster�ĸ��ӹ�ϵ
	pcl::PointXYZRGB Centroid;			//���ĵ�
	pcl::PointXYZRGB ConvexCentroid;	//͹������ε����ĵ�
	pcl::PointXYZRGB CurDirection;	//��ǰλ�ô��ķ���
} Cluster;

//typedef struct SameCluster
//{
//	int SmallIndex;
//	int BigIndex;
//};

class CDBScanCluster	
{
public:
	CDBScanCluster();
	~CDBScanCluster();
	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue);	
	void RunCluster(double RadiusValue, int MinPtsValue, bool IsUseOutliers = true);

	//////һ��ĵ�������������⣬2020.02.21
	//void RunClusterTwo(double RadiusValue, int MinPtsValue);
	
	//���м���ķ��� 2020.02.20
	//void RunClusterParallel(double RadiusValue, int MinPtsValue);
	
	int GetClusterNumbers();
	void GetCluster(int Index, vector<int> & Indexs);
	Cluster GetCluster(int Index);
	void SetClusterColors(bool IsByID = false);

	//2019.09.23 ��ȡ�������ĵ��ƴص�����
	void GetMainCluster(vector<int> & Indexs);
	int GetMainClusterIndex();
	void GetClusterPoints(int ClusterIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsPtr);

	//����һ�ݵ��Ʋ���ʾ
	void ShowCluster(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer, int PointSize,
		string PointsStr = "");

	//����ÿһ�صĲ���
	void CalcClusterParameters();

	vector<Cluster> Clusters;	//������	
protected:
	double Radius;	//�����뾶 Eps
	int MinPts;		//���е��Ƶ���С���������С�ڴ�ֵ���עΪ������
	int MainIndex;

	vector<bool> PointVisited;	//��¼�õ��Ƿ񱻷���	
	
	////ʹ�õ�Cluster��ID��2020.02.20
	int ClusterID;
	////��¼������ 2020.02.20
	//vector<int> IndexsForParallel;
	////Ӧ����ͬһ�����࣬������Ҫ�ϲ���ʱ�� 2020.02.20
	//vector<SameCluster> SameClusters;
	////����SameClusters �ϲ�������� 2020.02.20
	//void MergeClusterS();
	//bool FindSameClusters(SameCluster TempSameCluster);

	vector<int> OutliersClusters;	//�쳣��Ľ��
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree;	//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;

	virtual void ExpandCluster(vector<int> PointNeighbourS, int ClusterIndex);
	//virtual void ExpandClusterTwo(vector<int> PointNeighbourS, int ClusterIndex);
	//virtual void ExpandClusterParallel(vector<int> PointNeighbourS, int ClusterIndex);
};

/*
2020.02.21 ��DBScan�Ļ����ϼ��Ϸֲ���� DBScan���ٶ�������ϲ����Բ��ò��з�ʽʵ�ֶԴ��������ĵ��Ʒָ�

*/
//
//class CDBScanClusterWithSlice : public CDBScanCluster
//{
//private:
//	
//	//HorizontalPartition ��ֱ�ֲ㣬ÿ��ĸ߶�
//	double SliceHeight;
//
//	CHorizontalPartition HorizontalPartition;
//
//	vector<SliceCluster> SliceClusters;	//������	
//		
//public:
//	CDBScanClusterWithSlice();
//	~CDBScanClusterWithSlice();
//
//	//ִ�з�����̣�
//	void RunCluster(double RadiusValue, int MinPtsValue, double SliceH);
//	
//	void SlicesClusterMerge();
//};


/*
2019.09.27 ��DBScan�Ļ����ϼ��Ϸֲ���� DBScan���Էָ�һ���߶Ȳ��ϵĶ�����
��ϧ����ڵ�ʱû�м�֣����뷨ʱһ��Ҫ���ñ�д����
*/
/*
class CDBScanClusterWithSlice : public CDBScanCluster
{
public:
	CDBScanClusterWithSlice()
	{	
	}

	~CDBScanClusterWithSlice()
	{
		for (int i = 0; i < SlicesClusterS.size(); i++)
		{
			SlicesClusterS[i].clear();
		}
		SlicesClusterS.clear();
	}

private:
	int ID;
	double SliceHeight;
	CHorizontalPartition HorizontalPartition;
	//ÿһ������Ҫһ��
	vector<vector<Cluster>> SlicesClusterS;
public:
	void RunSliceCluster(double SliceHeightValue, double RadiusValue, int MinPtsValue);

	//�ڶ����֮�佫��֮������ɼ���֦λ��ӳ��
	void SlicesMappingByAngle();

	//�������ɾ��� �������� ����ע
	void SlicesMappingFromDownToUp();

	void SetColorsByID();
};
*/

/*
2019.09.18 ��DBScan�Ļ����ϼ������ʵ��жϣ�������С�ڸ�����ֵ����������죬����ֹͣ 

2019.09.23 �˷�����Щ���⣬��Ȼ�����ʴ��ֹͣ��
��������ȷ�������ʴ�����еĵ㶼�����ʴ�ĵ㣬�п��ܳ���һ��С���ƹ����ʴ������

*/

//2019.09.30 �ڴ�ֱ�ֶ�Slice�Ļ����ϣ���Slice�ļ������ʾ�ֵ���׼�
//��ĳ������ʴ��� ���ʾ�ֵ+3*��׼�� ����һ���쳣�㣬
//��ʱѰ��һ���Ӹõ㵽ƽ�����ʵ����·��(�������ʵ����ֵ��Ϊ��һ���㣬������ȥ)�������
//�ֲ�����DBScan�㷨������ �����쳣�� ��·����ʱ���������죬
// ͬʱҪ���ݵ�ǰ��Ĳ�����������жϣ�����������㣬��Ϊ���ɵ���
// CDBScanClusterWithCurvatrue ���������������Ϳ�����

class CDBScanClusterWithCurvatrue : public CDBScanCluster
{
private:	
	CPointGeometry * p_PointGeometry;	
	pcl::PointXYZRGB MainDirection;
	double CurvatureThreshold0;

	double SliceHeigth;
	vector<double> MeanCurvature, StdCurvature;	
	CHorizontalPartition HorizontalPartition;
	
	//RedIndexs�����ʴ���3�������ʵ㣬BlueIndexs�Ǵ� RedIndexs �仯Ϊ ���ʾ�ֵ���� ��������·��
	vector<int> RedIndexs, BlueIndexs;

	void CalcSliceCurvature();
public:
	CDBScanClusterWithCurvatrue()
	{

	}

	~CDBScanClusterWithCurvatrue()
	{

	}

	//2019.09.18 With Condition, 
	//��һ�����Curvature ���� CurvatureThresholdʱ��
	//��ĳһ����������� �� ���Curvature ���� CurvatureThreshold ��ֹͣ�Դ�Ϊ���Ľ��е���
	void RunClusterWithCurvature(double RadiusValue, int MinPtsValue,
		CPointGeometry * p_PointGeometryValue, double CurvatureThresholdValue = 0.001);

	virtual void ExpandCluster(int PointIndex, vector<int> PointNeighbourS, int ClusterIndex);

	//��������������������С����������������������ϲ�
	//�ϲ�����������ĽǶ�����
	void ClusterMerge(double Anlge);

	void RunClusterWithSliceCurvature(double SliceHeigthValue, double RadiusValue, int MinPtsValue,
		CPointGeometry * p_PointGeometryValue);

	void ExpandClusterWithSlice(int PointIndex, vector<int> PointNeighbourS, int ClusterIndex);
};


//2019.09.30 �ڴ�ֱ�ֶ�Slice�Ļ����ϣ���Slice�ļ������ʾ�ֵ���׼�
//��ĳ������ʴ��� ���ʾ�ֵ+3*��׼�� ����һ���쳣�㣬
//��ʱѰ��һ���Ӹõ㵽ƽ�����ʵ����·��(�������ʵ����ֵ��Ϊ��һ���㣬������ȥ)�������
//�ֲ�����DBScan�㷨������ �����쳣�� ��·����ʱ���������죬
// ͬʱҪ���ݵ�ǰ��Ĳ�����������жϣ�����������㣬��Ϊ���ɵ���
//
//class CDBScanWithCurvatruePath : public CDBScanCluster
//{
//private:
//	double SliceHeight;
//	vector<double> MeanCurvature, StdCurvature;
//	CPointGeometry * p_PointGeometry;
//	CHorizontalPartition HorizontalPartition;	
//
//	void CalcSliceCurvature();
//public:
//	CDBScanWithCurvatruePath()
//	{
//	
//	}
//
//	~CDBScanWithCurvatruePath()
//	{
//	
//	}
//
//	void SetParameters(double SliceHeightValue, double RadiusValue, int MinPtsValue, CPointGeometry * pPointGeometry);
//
//	void RunClusterCurvatruePath();
//
//	void RunClusterSliceCurvatruePath(int SliceIndex);
//
//	virtual void ExpandCluster(int SliceIndex, int PointIndex, vector<int> PointNeighbourS, int ClusterIndex);
//};
