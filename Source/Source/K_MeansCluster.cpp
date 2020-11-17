#include "K_MeansCluster.h"

CK_MeansCluster::CK_MeansCluster()
{
	InputCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
}

CK_MeansCluster::~CK_MeansCluster()
{
	InputCloud->points.clear();
}

//未使用，当初是想获取数据的量纲，但迭代条件调整为无量纲形式的
double CK_MeansCluster::GetDisDIMENSION()
{
	double MaxDis = EPSM6, MinDis = EPSP6;
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		double TempDis = sqrt(pow(InputCloud->points[i].x, 2)
			+ pow(InputCloud->points[i].y, 2) + pow(InputCloud->points[i].z, 2));

		if (TempDis > MaxDis) MaxDis = TempDis;
		if (TempDis < MinDis) MinDis = TempDis;
	}

	return (MaxDis - MinDis);
}

void CK_MeansCluster::PreProcessingForCluster()
{
	for (int i = 0; i < ClusterNumber; i++)
	{
		MeansCluster TempClusterResult;
		
		TempClusterResult.ClusterIndex = i;
		TempClusterResult.ClusterCenter.x = 0, TempClusterResult.ClusterCenter.y = 0, TempClusterResult.ClusterCenter.z = 0;
		TempClusterResult.CurSquareErrorOfDis = 0;

		//if (i == 0)	//如果是第一个，就先把第一个点赋值给一个簇的中心点，然后计算初次聚类结果
		//{
		//	TempClusterResult.ClusterCenter = InputCloud->points[i];
		//	//此时并不需要把点放到其中
		//	//for(int ii =0; ii < InputCloud->points.size(); ii++)
		//	//{
		//	//	TempClusterResult.PointIndexS.push_back(ii);
		//	//}
		//}

		ClusterResults.push_back(TempClusterResult);
	}

	//以第一个点作为第一个聚类的中心点
	ClusterResults[0].ClusterCenter = InputCloud->points[0];

	for (int i = 1; i < ClusterNumber; i++)
	{
		vector<DisOfPointToCluster> ClusterDisS;
		
		//计算点集中的点到当前已经分配聚类的中心点的最小距离
		for (int ii = 0; ii < i; ii++)
		{
			DisOfPointToCluster TempDis = GetMaxDisBetweenPointAndCluster(ii);
			ClusterDisS.push_back(TempDis);
		}

		//寻找上述中的到各个聚类的最小距离的最大值对应的点 作为下一个聚类的中心点
		double MaxDis = 0;
		int MaxIndex = -1;
		for (int ii = 0; ii < ClusterDisS.size(); ii++)
		{
			if (ClusterDisS[ii].Dis > MaxDis)
			{
				MaxDis = ClusterDisS[ii].Dis;
				MaxIndex = ClusterDisS[ii].PointIndex;
			}
		}

		ClusterResults[i].ClusterCenter = InputCloud->points[MaxIndex];
	}	 
}

//寻找与 第 ClusterIndex 个聚类距离最近的点 主要用于聚类中心的初始化环节
DisOfPointToCluster CK_MeansCluster::GetMaxDisBetweenPointAndCluster(int ClusterIndex)
{
	double MinDis = EPSP6;
	int MinIndex = -1;	

	DisOfPointToCluster TempDis;
	   
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		double TempDis = PointDis(InputCloud->points[i], ClusterResults[ClusterIndex].ClusterCenter);
		if (TempDis > 0 && TempDis < MinDis)
		{
			MinDis = TempDis;
			MinIndex = i;
		}
	}
	TempDis.ClusterIndex = ClusterIndex;
	TempDis.Dis = MinDis;
	TempDis.PointIndex = MinIndex;

	return TempDis;
}

int CK_MeansCluster::GetMinDis(vector<DisOfPointToCluster> DisSOfPointToCluster)
{
	double MinDis = EPSP6;
	int MinIndex = -1;
	for (int i = 0; i < DisSOfPointToCluster.size(); i++)
	{
		if (DisSOfPointToCluster[i].Dis < MinDis)
		{
			MinDis = DisSOfPointToCluster[i].Dis;
			MinIndex = DisSOfPointToCluster[i].ClusterIndex;
		}
	}
	return MinIndex;
}

double CK_MeansCluster::ClusterIteration()
{
	for (int j = 0; j < ClusterResults.size(); j++)
	{
		ClusterResults[j].PointIndexS.clear();
	}

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		vector< DisOfPointToCluster> DisSofPointToCluster;

		for (int j = 0; j < ClusterResults.size(); j++)
		{
			DisOfPointToCluster TempDis;
			TempDis.ClusterIndex = j;
			TempDis.PointIndex = i;
			TempDis.Dis = PointDis(InputCloud->points[i], ClusterResults[j].ClusterCenter);
			DisSofPointToCluster.push_back(TempDis);
		}

		//找到距离最小的聚类加入
		int TempClusterIndex = GetMinDis(DisSofPointToCluster);
		ClusterResults[TempClusterIndex].PointIndexS.push_back(i);
	}

	//更新聚类信息
	double SumError = 0;
	for (int j = 0; j < ClusterResults.size(); j++)
	{
		SumError += RefreshClusterData(j);
	}
	return SumError;
}

double CK_MeansCluster::RefreshClusterData(int ClusterIndex)
{
	if (ClusterResults[ClusterIndex].PointIndexS.size() == 0) return -1;

	pcl::PointXYZRGB OldClusterCenter = ClusterResults[ClusterIndex].ClusterCenter;
	ClusterResults[ClusterIndex].ClusterCenter.x = 0, ClusterResults[ClusterIndex].ClusterCenter.y = 0, 
		ClusterResults[ClusterIndex].ClusterCenter.z = 0;

	for (int i = 0; i < ClusterResults[ClusterIndex].PointIndexS.size(); i++)
	{
		ClusterResults[ClusterIndex].ClusterCenter.x += InputCloud->points[ClusterResults[ClusterIndex].PointIndexS[i]].x;
		ClusterResults[ClusterIndex].ClusterCenter.y += InputCloud->points[ClusterResults[ClusterIndex].PointIndexS[i]].y;
		ClusterResults[ClusterIndex].ClusterCenter.z += InputCloud->points[ClusterResults[ClusterIndex].PointIndexS[i]].z;
	}

	ClusterResults[ClusterIndex].ClusterCenter.x /= ClusterResults[ClusterIndex].PointIndexS.size();
	ClusterResults[ClusterIndex].ClusterCenter.y /= ClusterResults[ClusterIndex].PointIndexS.size();
	ClusterResults[ClusterIndex].ClusterCenter.z /= ClusterResults[ClusterIndex].PointIndexS.size();

	
	double SumError = 0;
	for (int i = 0; i < ClusterResults[ClusterIndex].PointIndexS.size(); i++)
	{
		SumError += PointDis(ClusterResults[ClusterIndex].ClusterCenter,
			InputCloud->points[ClusterResults[ClusterIndex].PointIndexS[i]]);
	}
	return SumError;
	//return PointDis(OldClusterCenter, ClusterResults[ClusterIndex].ClusterCenter);
}

void CK_MeansCluster::SetInput(vector<double> Values)
{
	InputCloud->points.clear();
	for(int i = 0; i < Values.size(); i++)
	{
		pcl::PointXYZRGB TempPoint;
		TempPoint.x = Values[i], TempPoint.y = 0, TempPoint.z = 0;
		InputCloud->points.push_back(TempPoint);
	}
}

void CK_MeansCluster::SetInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue)
{
	InputCloud->points.clear();

	InputCloud->points.insert(InputCloud->points.begin(),
		InputCloudValue->points.begin(), InputCloudValue->points.end());	
}

void CK_MeansCluster::RunCluster(int Number, double Threshold)
{
	ClusterNumber = Number;
	ClusterResults.clear();

	//初始化聚类中心点
	PreProcessingForCluster();

	//首先迭代两次
	double OldSumError = ClusterIteration();
	double NewSumError = ClusterIteration();

	//应考虑到量纲的差异，迭代至满足最小条件
	while (abs(OldSumError - NewSumError) / NewSumError > Threshold)
	{
		OldSumError = NewSumError;
		NewSumError = ClusterIteration();
	}
}

//返回某一个类的点索引
void CK_MeansCluster::GetClusterPoints(int Index, vector<int> & Indexs)
{
	Indexs.clear();
	if (Index < ClusterResults.size())
		Indexs.insert(Indexs.begin(), ClusterResults[Index].PointIndexS.begin(), ClusterResults[Index].PointIndexS.end());
}

void CK_MeansCluster::GetClusterInfo(int Index, MeansCluster & Cluster)
{
	if (Index < ClusterResults.size())
	{
		Cluster.ClusterCenter = ClusterResults[Index].ClusterCenter;
		Cluster.ClusterIndex = ClusterResults[Index].ClusterIndex;
		Cluster.CurSquareErrorOfDis = ClusterResults[Index].CurSquareErrorOfDis;
		Cluster.PointIndexS.clear();
		Cluster.PointIndexS.insert(Cluster.PointIndexS.begin(), 
			ClusterResults[Index].PointIndexS.begin(), ClusterResults[Index].PointIndexS.end());
	}
}

MeansCluster CK_MeansCluster::GetClusterInfo(int Index)
{
	if (Index < ClusterResults.size())
	{
		return ClusterResults[Index];
	}		
}

int CK_MeansCluster::GetClusterNumbers()
{
	return ClusterResults.size();
}

void CK_MeansCluster::SetClusterColors()
{
	for (int i = 0; i < ClusterResults.size(); i++)
	{
		int Color = ColorBaseS[i % 29 + 1];
		for(int j = 0; j < ClusterResults[i].PointIndexS.size(); j++)
		{
			InputCloud->points[ClusterResults[i].PointIndexS[j]].rgba = Color;
		}
	}
}

void CK_MeansCluster::ShowCluster(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer, int PointSize, string PointsStr)
{
	if (PointsStr == "")
		PointsStr = StringBase::ClockValue();
	Viewer->addPointCloud(InputCloud, PointsStr);
	Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, PointsStr);
}
