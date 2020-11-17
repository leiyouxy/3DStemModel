#pragma once

/*
add by leiyou 2019.09.05

为调用 dbScan 而构建的类

dbScan 是 Paper “DBSCAN modicado con Octrees para agrupar nubes de puntos en tiempo real”中的Source Code

然而 该程序输出结果只有点的坐标，而不是点的索引，这为后续计算带来很多麻烦，索性自己重新写一个 2019.09.05

*/
#include "CommPointBase.h"
#include "CommGeometry.h"
#include "PointGeometry.h"
#include "HorizontalPartition.h"

////节点间的关联关系 RelationShip

//节点的关系
typedef struct Role
{
	int SliceIndex;
	int ClusterIndex;
};

typedef struct NodeRole
{
	Role ParentNode;		//Parent Node
	//int ParentSliceIndex;		//父节点所在的Slice, 有可能不在直接的上一级
	//int ParentClusterIndex;			//上一级Slice的某一个簇Index，为其父节点
	vector<Role> ChildNodes;	//
};

typedef struct 
{	
	int ID;							//类编号
	int IsOutliers;					//是否是噪声点
	//int IsJoint;					// is the joint point between the branches and stem, 
	//					can be judged by NodeRoles.ChildNodes.count();
	vector<int> Indexs;				//点的索引
	vector<int> GlobalIndexs;		//点的全局索引，使用于多个Slice聚类的时候
	string Label;					//字符串标签
	NodeRole NodeRoles;				//存储上下Slice间不同Cluster的父子关系
	pcl::PointXYZRGB Centroid;			//质心点
	pcl::PointXYZRGB ConvexCentroid;	//凸包多边形的质心点
	pcl::PointXYZRGB CurDirection;	//当前位置处的方向
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

	//////一半的点进来还是有问题，2020.02.21
	//void RunClusterTwo(double RadiusValue, int MinPtsValue);
	
	//并行计算的方法 2020.02.20
	//void RunClusterParallel(double RadiusValue, int MinPtsValue);
	
	int GetClusterNumbers();
	void GetCluster(int Index, vector<int> & Indexs);
	Cluster GetCluster(int Index);
	void SetClusterColors(bool IsByID = false);

	//2019.09.23 获取数量最多的点云簇的索引
	void GetMainCluster(vector<int> & Indexs);
	int GetMainClusterIndex();
	void GetClusterPoints(int ClusterIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsPtr);

	//复制一份点云并显示
	void ShowCluster(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer, int PointSize,
		string PointsStr = "");

	//计算每一簇的参数
	void CalcClusterParameters();

	vector<Cluster> Clusters;	//聚类结果	
protected:
	double Radius;	//搜索半径 Eps
	int MinPts;		//类中点云的最小个数，如果小于此值则标注为噪声点
	int MainIndex;

	vector<bool> PointVisited;	//记录该店是否被访问	
	
	////使用的Cluster的ID，2020.02.20
	int ClusterID;
	////记录的类编号 2020.02.20
	//vector<int> IndexsForParallel;
	////应属于同一个的类，后续需要合并的时间 2020.02.20
	//vector<SameCluster> SameClusters;
	////根据SameClusters 合并多个聚类 2020.02.20
	//void MergeClusterS();
	//bool FindSameClusters(SameCluster TempSameCluster);

	vector<int> OutliersClusters;	//异常点的结果
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree;	//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;

	virtual void ExpandCluster(vector<int> PointNeighbourS, int ClusterIndex);
	//virtual void ExpandClusterTwo(vector<int> PointNeighbourS, int ClusterIndex);
	//virtual void ExpandClusterParallel(vector<int> PointNeighbourS, int ClusterIndex);
};

/*
2020.02.21 在DBScan的基础上加上分层逐层 DBScan，再对逐层结果合并，以采用并行方式实现对大数据量的点云分割

*/
//
//class CDBScanClusterWithSlice : public CDBScanCluster
//{
//private:
//	
//	//HorizontalPartition 垂直分层，每层的高度
//	double SliceHeight;
//
//	CHorizontalPartition HorizontalPartition;
//
//	vector<SliceCluster> SliceClusters;	//聚类结果	
//		
//public:
//	CDBScanClusterWithSlice();
//	~CDBScanClusterWithSlice();
//
//	//执行分类过程，
//	void RunCluster(double RadiusValue, int MinPtsValue, double SliceH);
//	
//	void SlicesClusterMerge();
//};


/*
2019.09.27 在DBScan的基础上加上分层逐层 DBScan，以分割一个高度层上的多个树杈
可惜这个在当时没有坚持，有想法时一定要采用笔写下来
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
	//每一个层需要一个
	vector<vector<Cluster>> SlicesClusterS;
public:
	void RunSliceCluster(double SliceHeightValue, double RadiusValue, int MinPtsValue);

	//在多个层之间将层之间的树干及树枝位置映射
	void SlicesMappingByAngle();

	//根据树干聚类 从下至上 逐层标注
	void SlicesMappingFromDownToUp();

	void SetColorsByID();
};
*/

/*
2019.09.18 在DBScan的基础上加上曲率的判断，若曲率小于给定阈值，则继续延伸，否则停止 

2019.09.23 此方法有些问题，虽然到曲率大点停止，
但并不能确保，曲率大点所有的点都是曲率大的点，有可能出现一条小道绕过曲率大点的情况

*/

//2019.09.30 在垂直分段Slice的基础上，逐Slice的计算曲率均值与标准差，
//若某点的曲率大于 曲率均值+3*标准差 则是一个异常点，
//此时寻找一条从该点到平均曲率的最大路径(邻域曲率的最大值点为下一个点，逐渐找下去)，并标记
//分层运行DBScan算法，遇到 曲率异常点 的路径点时，则不再延伸，
// 同时要根据当前点的不规则情况做判断，如果是正常点，则为树干点云
// CDBScanClusterWithCurvatrue 在这个类基础上做就可以了

class CDBScanClusterWithCurvatrue : public CDBScanCluster
{
private:	
	CPointGeometry * p_PointGeometry;	
	pcl::PointXYZRGB MainDirection;
	double CurvatureThreshold0;

	double SliceHeigth;
	vector<double> MeanCurvature, StdCurvature;	
	CHorizontalPartition HorizontalPartition;
	
	//RedIndexs是曲率大于3倍的曲率点，BlueIndexs是从 RedIndexs 变化为 曲率均值点所 经历过的路径
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
	//若一个点的Curvature 大于 CurvatureThreshold时，
	//或某一个点的邻域中 有 点的Curvature 大于 CurvatureThreshold 则停止以此为核心进行迭代
	void RunClusterWithCurvature(double RadiusValue, int MinPtsValue,
		CPointGeometry * p_PointGeometryValue, double CurvatureThresholdValue = 0.001);

	virtual void ExpandCluster(int PointIndex, vector<int> PointNeighbourS, int ClusterIndex);

	//根据最大类的最大方向，如果有小类与最大类的最大方向相近，则合并
	//合并的两个方向的角度门限
	void ClusterMerge(double Anlge);

	void RunClusterWithSliceCurvature(double SliceHeigthValue, double RadiusValue, int MinPtsValue,
		CPointGeometry * p_PointGeometryValue);

	void ExpandClusterWithSlice(int PointIndex, vector<int> PointNeighbourS, int ClusterIndex);
};


//2019.09.30 在垂直分段Slice的基础上，逐Slice的计算曲率均值与标准差，
//若某点的曲率大于 曲率均值+3*标准差 则是一个异常点，
//此时寻找一条从该点到平均曲率的最大路径(邻域曲率的最大值点为下一个点，逐渐找下去)，并标记
//分层运行DBScan算法，遇到 曲率异常点 的路径点时，则不再延伸，
// 同时要根据当前点的不规则情况做判断，如果是正常点，则为树干点云
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
