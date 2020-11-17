#pragma once

/*

2019.09.20

为便于处理，以点的形式做为输入数据

K-Means Cluster 聚类


*/

#include "CommPointBase.h"
#include "CommGeometry.h"

struct MeansCluster				//聚类的数据结构
{
	int ClusterIndex;				//聚类编号
	double CurSquareErrorOfDis;		//当前聚类中点到中心点的距离
	pcl::PointXYZRGB ClusterCenter;	//聚类中心，可以用颜色表示
	vector<int> PointIndexS;		//聚类包含的点索引
};

struct DisOfPointToCluster
{
	int PointIndex;			//点的索引		
	int ClusterIndex;		//簇的索引
	double Dis;
};

class CK_MeansCluster 
{
public:
	CK_MeansCluster();
	~CK_MeansCluster();

private:	
	int ClusterNumber;						//类的个数
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;
	vector<MeansCluster> ClusterResults;	//聚类结果的容器

	double DisDIMENSION;					//量纲，为统一不同数值单位下的量纲
	double GetDisDIMENSION();				//有待于改进，并不是很好 2019.09.20
	
	void PreProcessingForCluster();			//聚类前的预处理，以选择合适的聚类开始点

	//寻找与 第 ClusterIndex 个聚类距离最近的点 主要用于聚类中心的初始化环节
	DisOfPointToCluster GetMaxDisBetweenPointAndCluster(int ClusterIndex);

	//寻找到聚类中心距离最小的点
	int GetMinDis(vector<DisOfPointToCluster> DisSOfPointToCluster);

	double ClusterIteration();				//每一次聚类迭代
	
	double RefreshClusterData(int ClusterIndex);	//更新 ClusterIndex 类的数据，并返回新旧中心点的距离
public:
	void SetInput(vector <double> Values);
	void SetInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue);
	void RunCluster(int Number, double Threshold = 0.01);			//讲聚类分为 Number 类
	void SetClusterColors();
	void ShowCluster(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer, int PointSize,
		string PointsStr = "");
	//返回某一个类的点索引
	void GetClusterPoints(int Index, vector<int> & Indexs);
	void GetClusterInfo(int Index, MeansCluster & Cluster);
	MeansCluster GetClusterInfo(int Index);
	int GetClusterNumbers();
};
