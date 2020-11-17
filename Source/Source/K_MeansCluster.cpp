#include "K_MeansCluster.h"

CK_MeansCluster::CK_MeansCluster()
{
	InputCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
}

CK_MeansCluster::~CK_MeansCluster()
{
	InputCloud->points.clear();
}

//δʹ�ã����������ȡ���ݵ����٣���������������Ϊ��������ʽ��
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

		//if (i == 0)	//����ǵ�һ�������Ȱѵ�һ���㸳ֵ��һ���ص����ĵ㣬Ȼ�������ξ�����
		//{
		//	TempClusterResult.ClusterCenter = InputCloud->points[i];
		//	//��ʱ������Ҫ�ѵ�ŵ�����
		//	//for(int ii =0; ii < InputCloud->points.size(); ii++)
		//	//{
		//	//	TempClusterResult.PointIndexS.push_back(ii);
		//	//}
		//}

		ClusterResults.push_back(TempClusterResult);
	}

	//�Ե�һ������Ϊ��һ����������ĵ�
	ClusterResults[0].ClusterCenter = InputCloud->points[0];

	for (int i = 1; i < ClusterNumber; i++)
	{
		vector<DisOfPointToCluster> ClusterDisS;
		
		//����㼯�еĵ㵽��ǰ�Ѿ������������ĵ����С����
		for (int ii = 0; ii < i; ii++)
		{
			DisOfPointToCluster TempDis = GetMaxDisBetweenPointAndCluster(ii);
			ClusterDisS.push_back(TempDis);
		}

		//Ѱ�������еĵ������������С��������ֵ��Ӧ�ĵ� ��Ϊ��һ����������ĵ�
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

//Ѱ���� �� ClusterIndex �������������ĵ� ��Ҫ���ھ������ĵĳ�ʼ������
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

		//�ҵ�������С�ľ������
		int TempClusterIndex = GetMinDis(DisSofPointToCluster);
		ClusterResults[TempClusterIndex].PointIndexS.push_back(i);
	}

	//���¾�����Ϣ
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

	//��ʼ���������ĵ�
	PreProcessingForCluster();

	//���ȵ�������
	double OldSumError = ClusterIteration();
	double NewSumError = ClusterIteration();

	//Ӧ���ǵ����ٵĲ��죬������������С����
	while (abs(OldSumError - NewSumError) / NewSumError > Threshold)
	{
		OldSumError = NewSumError;
		NewSumError = ClusterIteration();
	}
}

//����ĳһ����ĵ�����
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
