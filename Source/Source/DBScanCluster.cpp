#include"DBScanCluster.h"

CDBScanCluster::CDBScanCluster()
{
	Octree = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	InputCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
}

CDBScanCluster::~CDBScanCluster()
{
	free(Octree);
	Octree = NULL;
}

void CDBScanCluster::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue)
{		
	InputCloud->points.clear();
	InputCloud->points.insert(InputCloud->points.begin(), InputCloudValue->points.begin(),
		InputCloudValue->points.end());

	Octree->setInputCloud(InputCloud);
	Octree->addPointsFromInputCloud();

	Clusters.clear();
	OutliersClusters.clear();

	PointVisited.clear();
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		PointVisited.push_back(false);
	}
}

void CDBScanCluster::RunCluster(double RadiusValue, int MinPtsValue, bool IsUseOutliers)
{
	Radius = RadiusValue;
	MinPts = MinPtsValue;

	Clusters.clear();
	OutliersClusters.clear();
	//cout<<"RunCluster is doing, please waiting!"<<endl;
	//#pragma omp parallel for �˴�������м������ֱ�Ӧ����һ����ĵ����ڶ���࣬
	for (int i = 0; i < InputCloud->points.size(); i++)
	{		
		if (!PointVisited[i])	//unvisited
		{
			//cout << "Size:" << InputCloud->points.size() << ", Current:" << i << endl;
			vector<int> NeighbourIndexS;
			vector<float> NeighbourDisS;

			GeometryBase::GetNeighbourInRadius(Octree, Radius, i, &NeighbourIndexS, &NeighbourDisS);

			PointVisited[i] = true;
			if (NeighbourIndexS.size() >= MinPts)	//Core Point
			{
				Cluster NewCluster;
				NewCluster.Indexs.push_back(i);				
				NewCluster.Label = "";
				Clusters.push_back(NewCluster);
				
				int ClusterIndex = Clusters.size() - 1;	//Index Of Current Cluster
				ExpandCluster(NeighbourIndexS, ClusterIndex);				
			}
			else if (!IsUseOutliers)	//��ʹ���쳣ֵ
			{
				Cluster NewCluster;
				NewCluster.Indexs.push_back(i);
				NewCluster.Label = "";			
				NewCluster.IsOutliers = true;
				//for (int ii = 0; ii < NeighbourIndexS.size(); ii++)
				//{
				//	PointVisited[NeighbourIndexS[ii]] = true;
				//	NewCluster.Indexs.push_back(NeighbourIndexS[ii]);
				//}
				Clusters.push_back(NewCluster);
			}
			else// Outliers ʹ���쳣ֵ
			{
				OutliersClusters.push_back(i);
			}
		}
	}
	
	cout<<"DBScan Cluster has finished. The Number of Cluster is "<< Clusters.size() <<endl;
}

//һ��ĵ�������������⣬
//void CDBScanCluster::RunClusterTwo(double RadiusValue, int MinPtsValue)
//{
//	Radius = RadiusValue;
//	MinPts = MinPtsValue;
//
//	Clusters.clear();
//	OutliersClusters.clear();
//	//cout<<"RunCluster is doing, please waiting!"<<endl;
//	//#pragma omp parallel for �˴�������м������ֱ�Ӧ����һ����ĵ����ڶ���࣬
//	for (int i = 0; i < InputCloud->points.size(); i++)
//	{
//		if (!PointVisited[i])	//unvisited
//		{
//			cout << "Size:" << InputCloud->points.size() << ", Current:" << i << endl;
//			vector<int> NeighbourIndexS;
//			vector<float> NeighbourDisS;
//
//			if (i == 68)
//				cout << endl;
//
//			GeometryBase::GetNeighbourInRadius(Octree, Radius, i, &NeighbourIndexS, &NeighbourDisS);
//
//			PointVisited[i] = true;
//			if (NeighbourIndexS.size() >= MinPts)	//Core Point
//			{
//				Cluster NewCluster;
//				NewCluster.Indexs.push_back(i);
//				NewCluster.Label = "";
//				Clusters.push_back(NewCluster);
//
//				int ClusterIndex = Clusters.size() - 1;	//Index Of Current Cluster
//				//ExpandCluster(i, NeighbourIndexS, ClusterIndex);
//				{
//					vector<int> ExpandNeighbourIndexS;
//
//					for (int ii = 0; ii < NeighbourIndexS.size(); ii++)
//					{
//						if (ii < NeighbourIndexS.size() / 2.0)
//						{
//							Clusters[ClusterIndex].Indexs.push_back(NeighbourIndexS[ii]);
//							PointVisited[NeighbourIndexS[ii]] = true;
//						}
//						else
//						{
//							ExpandNeighbourIndexS.push_back(NeighbourIndexS[ii]);
//						}
//					}
//					ExpandClusterTwo(ExpandNeighbourIndexS, ClusterIndex);
//				}
//			}
//			else// Outliers
//			{
//				OutliersClusters.push_back(i);
//			}
//		}
//	}
//
//	cout << "DBScan Cluster has finished. The Number of Cluster is " << Clusters.size() << endl;
//}

/*
void CDBScanCluster::MergeClusterS()
{
	for (int i = 0; i <= ClusterID; i++)
	{
		Cluster TempCluster;
		TempCluster.ID = i;
		Clusters.push_back(TempCluster);
	}

	for (int i = 0; i <= InputCloud->points.size(); i++)
	{
		if (IndexsForParallel[i] >= 0)
			Clusters[IndexsForParallel[i]].Indexs.push_back(i);
		else if (IndexsForParallel[i] == -1)
			OutliersClusters.push_back(i);
	}
	SetClusterColors();
}

bool CDBScanCluster::FindSameClusters(SameCluster TempSameCluster)
{
	for (int i = 0; i < SameClusters.size(); i++)
	{
		if (SameClusters[i].SmallIndex == TempSameCluster.SmallIndex
			&& SameClusters[i].BigIndex == TempSameCluster.BigIndex)
		{
			return true;
		}
	}
	return false;
}
*/

//����㲢�����������뵽����
void CDBScanCluster::ExpandCluster(vector<int> PointNeighbourS, int ClusterIndex)
{
	//cout << "PointNeighbourS.size()" << PointNeighbourS.size()<< endl;
	VectorBase<int> VectorBaseInt;
	for (int i = 0; i < PointNeighbourS.size(); i++)
	{
		//cout << "New PointNeighbourS.size()" << PointNeighbourS.size() << endl;
		//cout << "i" << i << endl;
		int TempPointIndex = PointNeighbourS[i];
		if (!PointVisited[TempPointIndex])		{
			
			PointVisited[TempPointIndex] = true;

			vector<int> NeighbourIndexS;
			vector<float> NeighbourDisS;
			GeometryBase::GetNeighbourInRadius(Octree, Radius, TempPointIndex,
				&NeighbourIndexS, &NeighbourDisS);

			if (NeighbourIndexS.size() >= MinPts)	//Core Point
			{				
				Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);
								
				PointNeighbourS.insert(PointNeighbourS.end(), 
					NeighbourIndexS.begin(), NeighbourIndexS.end());
			}
			else //��PointIndex�����򣬵�����Core Point�����Ҳ�����쳣�㣬�������ClusterIndex��
			{
				VectorBase<int> VectorBaseInt;
				int FindIndex = VectorBaseInt.FindIndexInVector(OutliersClusters, TempPointIndex);
				if (FindIndex == -1)
				{
					Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);
				}
			}
		}
	}	
}


////һ��ĵ�������������⣬2020.02.21
//void CDBScanCluster::ExpandClusterTwo(vector<int> PointNeighbourS, int ClusterIndex)
//{
//	//cout << "PointNeighbourS.size()" << PointNeighbourS.size()<< endl;
//	VectorBase<int> VectorBaseInt;
//	for (int i = 0; i < PointNeighbourS.size(); i++)
//	{
//		//cout << "New PointNeighbourS.size():" << PointNeighbourS.size() << endl;
//		//cout << "i:" << i << endl;
//		int TempPointIndex = PointNeighbourS[i];
//		if (!PointVisited[TempPointIndex]) 
//		{
//			PointVisited[TempPointIndex] = true;
//
//			vector<int> NeighbourIndexS;
//			vector<float> NeighbourDisS;
//			GeometryBase::GetNeighbourInRadius(Octree, Radius, TempPointIndex,
//				&NeighbourIndexS, &NeighbourDisS);
//
//			if (NeighbourIndexS.size() >= MinPts)	//Core Point
//			{
//				Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);
//
//				//PointNeighbourS.insert(PointNeighbourS.end(),
//				//	NeighbourIndexS.begin(), NeighbourIndexS.end());
//
//				vector<int> ExpandIndexS;
//				for (int ii = 0; ii < NeighbourIndexS.size(); ii++)
//				{
//					if (ii < NeighbourIndexS.size() / 2.0)
//					{
//						Clusters[ClusterIndex].Indexs.push_back(NeighbourIndexS[ii]);
//						PointVisited[NeighbourIndexS[ii]] = true;
//					}
//					else
//					{
//						ExpandIndexS.push_back(NeighbourIndexS[ii]);
//					}
//				}
//				PointNeighbourS.insert(PointNeighbourS.end(),
//					ExpandIndexS.begin(), ExpandIndexS.end());
//			}
//			else //��PointIndex�����򣬵�����Core Point�����Ҳ�����쳣�㣬�������ClusterIndex��
//			{
//				VectorBase<int> VectorBaseInt;
//				int FindIndex = VectorBaseInt.FindIndexInVector(OutliersClusters, TempPointIndex);
//				if (FindIndex == -1)
//				{
//					Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);
//				}
//			}
//		}
//	}
//}

/*
void CDBScanCluster::ExpandClusterParallel(vector<int> PointNeighbourS, int ClusterIndex)
{
	//cout << "PointNeighbourS.size()" << PointNeighbourS.size()<< endl;
	VectorBase<int> VectorBaseInt;
	while(PointNeighbourS.size() > 0)	
	{		
		int TempPointIndex = PointNeighbourS[0];		
		cout << "PointNeighbourS.size()" << PointNeighbourS.size() <<", TempPointIndex:"<< TempPointIndex << endl;
		PointNeighbourS.erase(PointNeighbourS.begin());

		if (IndexsForParallel[TempPointIndex] == -2)	//���û�д�������
		{
			vector<int> NeighbourIndexS;
			vector<int> ExpandNeighbourIndexS;
			vector<float> NeighbourDisS;
			GeometryBase::GetNeighbourInRadius(Octree, Radius, TempPointIndex,
				&NeighbourIndexS, &NeighbourDisS);

			if (NeighbourIndexS.size() >= MinPts)	//Core Point
			{			
				for (int i = 0; i < NeighbourIndexS.size(); i++)
				{
					if (((IndexsForParallel[NeighbourIndexS[i]] == -2 ||
						IndexsForParallel[NeighbourIndexS[i]] == -1)))
					{
						IndexsForParallel[NeighbourIndexS[i]] = ClusterIndex;
					}
					else if (IndexsForParallel[NeighbourIndexS[i]] != ClusterIndex)
					{
						SameCluster TempSameCluster;

						//����˳��洢
						if (IndexsForParallel[NeighbourIndexS[i]] > ClusterIndex)
						{
							TempSameCluster.BigIndex = IndexsForParallel[NeighbourIndexS[i]];
							TempSameCluster.SmallIndex = ClusterIndex;
						}
						else
						{
							TempSameCluster.BigIndex = ClusterIndex;
							TempSameCluster.SmallIndex = IndexsForParallel[NeighbourIndexS[i]];
						}
						if (!FindSameClusters(TempSameCluster))
							SameClusters.push_back(TempSameCluster);
					}

					if (i > NeighbourIndexS.size() / 2)
					{
						if(!VectorBaseInt.FindIndexInVector(PointNeighbourS, NeighbourIndexS[i]))
							PointNeighbourS.push_back(NeighbourIndexS[i]);
					}					
				}
			}
			else //��PointIndex�����򣬵�����Core Point�����Ҳ�����쳣�㣬�������ClusterIndex��
			{
				for (int i = 0; i < NeighbourIndexS.size(); i++)
				{
					IndexsForParallel[NeighbourIndexS[i]] = ClusterIndex;
				}
			}
		}
	}
}
*/

void CDBScanCluster::ShowCluster(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer, int PointSize, string PointsStr)
{
	if (PointsStr == "")
		PointsStr = StringBase::ClockValue();
	Viewer->addPointCloud(InputCloud, PointsStr);
	Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, PointsStr);
}

void CDBScanCluster::CalcClusterParameters()
{
	double XMin, XMax, YMin, YMax, ZMin, ZMax;
	PointBase::GetPointsMaxAndMin(InputCloud, XMax, XMin, YMax, YMin, ZMax, ZMin);

	for (int i = 0; i < Clusters.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurClusterPointsPtr (new pcl::PointCloud<pcl::PointXYZRGB>());
		GetClusterPoints(i, CurClusterPointsPtr);

		double TempMinx, TempMaxx, TempMiny, TempMaxy, TempMinz, TempMaxz;
		PointBase::GetPointsMaxAndMin(CurClusterPointsPtr, 
			TempMaxx, TempMinx, TempMaxy, TempMiny, TempMaxz, TempMinz);
		//Clusters[i].MaxDirection = GeometryBase::GetMaxDirectionVector(CurClusterPointsPtr);
		
		//2020.04.11 ʹ�����ĵ���Ϊ
		PointBase::GetPointsCenterXYZ(CurClusterPointsPtr,
			Clusters[i].Centroid.x, Clusters[i].Centroid.y, Clusters[i].Centroid.z);		
		//Clusters[i].Centroid.z = TempMinz;
		//Clusters[i].Centroid.z = ZMin;		

		//2020.02.14 ������ڵ���3����
		if (CurClusterPointsPtr->points.size() > 2)
			Clusters[i].ConvexCentroid = 
				GeometryBase::GetConvexHullCentroidOfPointsInXY(CurClusterPointsPtr);
		else
			Clusters[i].ConvexCentroid = Clusters[i].Centroid;

		Clusters[i].ConvexCentroid.z = Clusters[i].Centroid.z;

		Clusters[i].Centroid.rgba = ColorBase::BlueColor;
		Clusters[i].ConvexCentroid.rgba = ColorBase::BlueColor;

		Clusters[i].ConvexCentroid.z = Clusters[i].Centroid.z;
		Clusters[i].NodeRoles.ParentNode.SliceIndex = -1;
		Clusters[i].NodeRoles.ParentNode.ClusterIndex = -1;

		Clusters[i].CurDirection.x = 0, Clusters[i].CurDirection.y = 0, Clusters[i].CurDirection.z = 0;
	}
}

/*
//���м���ķ��� 2020.02.20
void CDBScanCluster::RunClusterParallel(double RadiusValue, int MinPtsValue)
{
	Radius = RadiusValue;
	MinPts = MinPtsValue;

	Clusters.clear();
	OutliersClusters.clear();

	//��ʼ��
	IndexsForParallel.clear();
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		//-2Ϊδʹ�ã�-1Ϊ�����㣬��0��ʼΪ���
		IndexsForParallel.push_back(-2);
		PointVisited[i] = false;
	}
	ClusterID = -1;

	//cout<<"RunCluster is doing, please waiting!"<<endl;
	//#pragma omp parallel for //�˴�������м������ֱ�Ӧ����һ����ĵ����ڶ���࣬
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		if (IndexsForParallel[i] == -2)	//unvisited
		{
			cout << "Size:" << InputCloud->points.size() << ", Current:" << i << endl;
			vector<int> NeighbourIndexS;
			vector<int> ExpandNeighbourIndexS;
			vector<float> NeighbourDisS;

			GeometryBase::GetNeighbourInRadius(Octree, Radius, i, &NeighbourIndexS, &NeighbourDisS);

			//#pragma omp critical
			if (NeighbourIndexS.size() >= MinPts)	//Core Point
			{
				ClusterID = ClusterID + 1;
				IndexsForParallel[i] = ClusterID;
								
				for (int j = 0; j < NeighbourIndexS.size(); j++)
				{
					//�Ѿ���ֵ�����������㣬����Ҫ��¼��������ͬ�Ĵصı��
					if (
						(IndexsForParallel[NeighbourIndexS[j]] == -2 ||
							IndexsForParallel[NeighbourIndexS[j]] == -1))
					{
						IndexsForParallel[NeighbourIndexS[j]] = ClusterID;
					}
					else if (IndexsForParallel[NeighbourIndexS[j]] != ClusterID)
					{
						SameCluster TempSameCluster;

						//����˳��洢
						if (IndexsForParallel[NeighbourIndexS[j]] > ClusterID)
						{
							TempSameCluster.BigIndex = IndexsForParallel[NeighbourIndexS[j]];
							TempSameCluster.SmallIndex = ClusterID;
						}
						else
						{
							TempSameCluster.BigIndex = ClusterID;
							TempSameCluster.SmallIndex = IndexsForParallel[NeighbourIndexS[j]];
						}

						if (!FindSameClusters(TempSameCluster))
							SameClusters.push_back(TempSameCluster);						
					}
					if (j > NeighbourIndexS.size() / 2)
					{
						//if (IndexsForParallel[NeighbourIndexS[j]] == -2)
							ExpandNeighbourIndexS.push_back(NeighbourIndexS[j]);						
					}
				}	
			}
			else// Outliers
			{
				IndexsForParallel[i] = -1;
			}
			ExpandClusterParallel(ExpandNeighbourIndexS, ClusterID);
		}
	}

	MergeClusterS();

	cout << "DBScan Cluster has finished. The Number of Cluster is " << Clusters.size() << endl;
}
*/

int CDBScanCluster::GetClusterNumbers()
{
	return Clusters.size();
}

void CDBScanCluster::GetCluster(int Index, vector<int>  & ClusterIndexs)
{
	if (Index < 0 || Index >= Clusters.size()) return;

	ClusterIndexs.clear();
	ClusterIndexs.insert(ClusterIndexs.end(), Clusters[Index].Indexs.begin(), Clusters[Index].Indexs.end());
}

Cluster CDBScanCluster::GetCluster(int Index)
{
	//if (Index < 0 || Index >= Clusters.size()) return;	
	return Clusters[Index];
}

void CDBScanCluster::SetClusterColors(bool IsByID)
{
	for (int i = 0; i < OutliersClusters.size(); i++)
	{
		InputCloud->points[OutliersClusters[i]].rgba = ColorBase::RedColor;
	}
	
	int ClusterCount = GetClusterNumbers();	

	int ColorRStep = 0 , ColorGStep = 0, ColorBStep = 0;

	for (int i = 0; i < ClusterCount; i++)
	{		
		int Color;
	
		if (IsByID) 
			Color = ColorBaseS[Clusters[i].ID % 29 + 1];
		else
			Color = ColorBaseS[i % 29 + 1];

		for (int j = 0; j < Clusters[i].Indexs.size(); j++)
		{
			InputCloud->points[Clusters[i].Indexs[j]].rgba = Color;
		}
	}
}

void CDBScanCluster::GetMainCluster(vector<int> & Indexs)
{
	if (Clusters.size() == 0) return;

	MainIndex = GetMainClusterIndex();

	GetCluster(MainIndex, Indexs);
}

void CDBScanCluster::GetClusterPoints(int ClusterIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsPtr)
{
	if (ClusterIndex >= Clusters.size()) return;

	PointsPtr->points.clear();
	for (int i = 0; i < Clusters[ClusterIndex].Indexs.size(); i++)
	{
		PointsPtr->points.push_back(InputCloud->points[Clusters[ClusterIndex].Indexs[i]]);
	}
}

int CDBScanCluster::GetMainClusterIndex()
{
	int MaxNumber = Clusters[0].Indexs.size();
	MainIndex = 0;

	for (int i = 1; i < Clusters.size(); i++)
	{
		if (Clusters[i].Indexs.size() > MaxNumber)
		{
			MaxNumber = Clusters[i].Indexs.size();
			MainIndex = i;
		}
	}
	return MainIndex;
}


//{CDBScanClusterWithCurvatrue.......................CDBScanClusterWithCurvatrue}

void CDBScanClusterWithCurvatrue::RunClusterWithCurvature(double RadiusValue, int MinPtsValue,
	CPointGeometry * p_PointGeometryValue, double CurvatureThresholdValue)
{
	Radius = RadiusValue;
	MinPts = MinPtsValue;
	p_PointGeometry = p_PointGeometryValue;

	Clusters.clear();
	OutliersClusters.clear();

	int i = 0;
	bool IsAllVisited = false;
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		if (!PointVisited[i])	//unvisited
		{
			IsAllVisited = true;

			vector<int> NeighbourIndexS;
			vector<float> NeighbourDisS;

			PointVisited[i] = true;

			//��ǰ��������ֵ�Ƚϴ�ĵ�
			if (p_PointGeometry->GetGuassCurvature(i) > CurvatureThreshold0)
				continue;

			GeometryBase::GetNeighbourInRadius(Octree, Radius, i, &NeighbourIndexS, &NeighbourDisS);

			if (NeighbourIndexS.size() >= MinPts)	//Core Point
			{
				Cluster NewCluster;
				NewCluster.Indexs.push_back(i);
				Clusters.push_back(NewCluster);

				bool Find = false;

				for (int j = 0; j < NeighbourIndexS.size(); j++)
				{
					if (p_PointGeometry->GetGuassCurvature(NeighbourIndexS[j]) > CurvatureThreshold0)
					{
						Find = true;
						break;
					}
				}

				//����������е�����ʴ�����ֵ����ô�Ҳ������
				if (Find) continue;

				int ClusterIndex = Clusters.size() - 1;	//Index Of Current Cluster
				ExpandCluster(i, NeighbourIndexS, ClusterIndex);
			}
			else// Outliers
			{
				OutliersClusters.push_back(i);
			}
		}
		i++;
	}

	cout << "DBScan Cluster has finished. The Number of Cluster is " << Clusters.size() << endl;
}

void CDBScanClusterWithCurvatrue::ExpandCluster(int PointIndex, vector<int> PointNeighbourS, int ClusterIndex)
{
	for (int i = 0; i < PointNeighbourS.size(); i++)
	{
		int TempPointIndex = PointNeighbourS[i];
		if (!PointVisited[TempPointIndex]) {

			PointVisited[TempPointIndex] = true;

			//��ǰ��������ֵ�Ƚϴ�ĵ�
			if (p_PointGeometry->GetGuassCurvature(TempPointIndex) > CurvatureThreshold0)
				continue;

			vector<int> NeighbourIndexS;
			vector<float> NeighbourDisS;
			GeometryBase::GetNeighbourInRadius(Octree, Radius, TempPointIndex,
				&NeighbourIndexS, &NeighbourDisS);

			if (NeighbourIndexS.size() >= MinPts)	//Ҳ�� Core Point
			{
				Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);

				bool Find = false;

				for (int j = 0; j < NeighbourIndexS.size(); j++)
				{
					if (p_PointGeometry->GetGuassCurvature(NeighbourIndexS[j]) > CurvatureThreshold0)
					{
						Find = true;
						break;
					}
				}

				//����������е�����ʴ�����ֵ����ô�Ҳ������
				if (Find) continue;

				//int ClusterIndex = Clusters.size() - 1;	//Index Of Current Cluster
				PointNeighbourS.insert(PointNeighbourS.end(),
					NeighbourIndexS.begin(), NeighbourIndexS.end());
			}
			else// //��PointIndex�����򣬵�����Core Point�����Ҳ�����쳣�㣬�������ClusterIndex��
			{
				VectorBase<int> VectorBaseInt;
				int FindIndex = VectorBaseInt.FindIndexInVector(OutliersClusters, TempPointIndex);
				if (FindIndex == -1)
				{
					Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);
				}
			}

			////��ǰ��������������ֵ�Ƚϴ�ĵ�
			//for (int ii = NeighbourIndexS.size() - 1; ii >= 0; ii--)
			//{
			//	if (p_PointGeometry->GetGuassCurvature(NeighbourIndexS[ii]) > CurvatureThreshold)
			//	{
			//		NeighbourIndexS.erase(NeighbourIndexS.begin() + ii);
			//	}
			//}
			////ȥ�����ʴ������޺�ĵ�����
			//if (NeighbourIndexS.size() >= MinPts)	//Core Point
			//{
			//	Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);
			//	PointNeighbourS.insert(PointNeighbourS.end(),
			//		NeighbourIndexS.begin(), NeighbourIndexS.end());
			//}
			//else //��PointIndex�����򣬵�����Core Point�����Ҳ�����쳣�㣬�������ClusterIndex��
			//{
			//	VectorBase<int> VectorBaseInt;
			//	int FindIndex = VectorBaseInt.FindIndexInVector(OutliersClusters, TempPointIndex);
			//	if (FindIndex == -1)
			//	{
			//		Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);
			//	}
			//}
		}
	}
}

//��������������������С����������������������ϲ�
//�ϲ�����������ĽǶ�����
void CDBScanClusterWithCurvatrue::ClusterMerge(double Anlge)
{
	vector<int> MainIndexS;
	GetMainCluster(MainIndexS);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < MainIndexS.size(); i++)
	{
		PointsPtr->points.push_back(InputCloud->points[MainIndexS[i]]);
	}

	MainDirection = GeometryBase::GetMaxDirectionVector(PointsPtr);

	for (int i = 0; i < Clusters.size(); i++)
	{
		if (i == MainIndex) continue;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurrentPointsPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int j = 0; j < Clusters[i].Indexs.size(); j++)
		{
			CurrentPointsPtr->points.push_back(InputCloud->points[Clusters[i].Indexs[j]]);
		}
		pcl::PointXYZRGB CurrentClusterDirection = GeometryBase::GetMaxDirectionVector(CurrentPointsPtr);

		double TempAngle = GeometryBase::RadianToAngle(
			GeometryBase::AngleOfTwoVector(MainDirection, CurrentClusterDirection));
		if (TempAngle > 90) TempAngle = 180 - TempAngle;

		if (TempAngle < Anlge)	//���Ժϲ�
		{
			cout << "�ϲ�ʱ�ĽǶ� " << TempAngle << endl;

			Clusters[MainIndex].Indexs.insert(Clusters[MainIndex].Indexs.begin(),
				Clusters[i].Indexs.begin(), Clusters[i].Indexs.end());

			Clusters[i].Indexs.clear();
		}
	}
}

//�˺���ʹ��ʱ��Ҫ�ٴδ���
void CDBScanClusterWithCurvatrue::RunClusterWithSliceCurvature(double SliceHeigthValue,
	double RadiusValue, int MinPtsValue, CPointGeometry * p_PointGeometryValue)
{
	SliceHeigth = SliceHeigthValue;
	Radius = RadiusValue;
	MinPts = MinPtsValue;
	p_PointGeometry = p_PointGeometryValue;

	Clusters.clear();
	OutliersClusters.clear();

	CalcSliceCurvature();

	int i = 0;
	bool IsAllVisited = false;
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		if (!PointVisited[i])	//unvisited
		{
			IsAllVisited = true;

			vector<int> NeighbourIndexS;
			vector<float> NeighbourDisS;

			PointVisited[i] = true;

			int PointSliceIndex = HorizontalPartition.FindSectionIndex(InputCloud->points[i].z);
			double CurCurvatureThreshold = MeanCurvature[PointSliceIndex] + 3 * StdCurvature[PointSliceIndex];

			//��ǰ��������ֵ�Ƚϴ�ĵ�
			if (p_PointGeometry->GetGuassCurvature(i) > CurCurvatureThreshold)
				continue;

			GeometryBase::GetNeighbourInRadius(Octree, Radius, i, &NeighbourIndexS, &NeighbourDisS);

			if (NeighbourIndexS.size() >= MinPts)	//Core Point
			{
				Cluster NewCluster;
				NewCluster.Indexs.push_back(i);
				Clusters.push_back(NewCluster);

				bool Find = false;

				for (int j = 0; j < NeighbourIndexS.size(); j++)
				{
					if (p_PointGeometry->GetGuassCurvature(NeighbourIndexS[j]) > CurCurvatureThreshold)
					{
						Find = true;
						break;
					}
				}

				//����������е�����ʴ�����ֵ����ô�Ҳ������
				if (Find) continue;

				int ClusterIndex = Clusters.size() - 1;	//Index Of Current Cluster
				ExpandClusterWithSlice(i, NeighbourIndexS, ClusterIndex);
			}
			else// Outliers
			{
				OutliersClusters.push_back(i);
			}
		}
		i++;
	}

}

void CDBScanClusterWithCurvatrue::ExpandClusterWithSlice(int PointIndex,
	vector<int> PointNeighbourS, int ClusterIndex)
{
	for (int i = 0; i < PointNeighbourS.size(); i++)
	{
		int TempPointIndex = PointNeighbourS[i];
		if (!PointVisited[TempPointIndex])
		{
			PointVisited[TempPointIndex] = true;

			int PointSliceIndex = HorizontalPartition.FindSectionIndex(InputCloud->points[TempPointIndex].z);
			double CurCurvatureThreshold = MeanCurvature[PointSliceIndex] + 3 * StdCurvature[PointSliceIndex];

			//��ǰ��������ֵ�Ƚϴ�ĵ�
			if (p_PointGeometry->GetGuassCurvature(TempPointIndex) > CurCurvatureThreshold)
				continue;

			vector<int> NeighbourIndexS;
			vector<float> NeighbourDisS;
			GeometryBase::GetNeighbourInRadius(Octree, Radius, TempPointIndex,
				&NeighbourIndexS, &NeighbourDisS);

			if (NeighbourIndexS.size() >= MinPts)	//Ҳ�� Core Point
			{
				Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);

				bool Find = false;

				for (int j = 0; j < NeighbourIndexS.size(); j++)
				{
					if (p_PointGeometry->GetGuassCurvature(NeighbourIndexS[j]) > CurCurvatureThreshold)
					{
						Find = true;
						break;
					}
				}

				//����������е�����ʴ�����ֵ����ô�Ҳ������
				if (Find) continue;

				//int ClusterIndex = Clusters.size() - 1;	//Index Of Current Cluster
				PointNeighbourS.insert(PointNeighbourS.end(),
					NeighbourIndexS.begin(), NeighbourIndexS.end());
			}
			else// //��PointIndex�����򣬵�����Core Point�����Ҳ�����쳣�㣬�������ClusterIndex��
			{
				VectorBase<int> VectorBaseInt;
				int FindIndex = VectorBaseInt.FindIndexInVector(OutliersClusters, TempPointIndex);
				if (FindIndex == -1)
				{
					Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);
				}
			}
		}
	}
}

void CDBScanClusterWithCurvatrue::CalcSliceCurvature()
{
	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetThickNess(SliceHeigth);
	HorizontalPartition.PatitionSection();

	RedIndexs.clear();
	BlueIndexs.clear();

	///*/// ����ÿһ������ʾ�ֵ���׼��
	VectorBase <double> VectorBasedouble;
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		double TempMeanCurvature, TempStdCurvature;

		vector<double> CurvatureS;
		for (int j = 0; j < HorizontalPartition.SectionsVector[i].Indexs.size(); j++)
		{
			int TempPointIndex = HorizontalPartition.SectionsVector[i].Indexs[j];

			CurvatureS.push_back(p_PointGeometry->GetGuassCurvature(TempPointIndex));
		}
		TempStdCurvature = VectorBasedouble.CalcVariances(CurvatureS, TempMeanCurvature);

		cout << "��" << i << "��Slice�����ʾ�ֵΪ��" << TempMeanCurvature << endl;
		cout << "��" << i << "��Slice�����ʷ���Ϊ��" << TempStdCurvature << endl;

		MeanCurvature.push_back(TempMeanCurvature);
		StdCurvature.push_back(TempStdCurvature);

		vector<int> TempRedIndexs, TempBlueIndexs;
		p_PointGeometry->SearchCurvaturePathPoints(HorizontalPartition.SectionsVector[i].Indexs,
			TempRedIndexs, TempBlueIndexs);

		RedIndexs.insert(RedIndexs.end(), TempRedIndexs.begin(), TempRedIndexs.end());
		BlueIndexs.insert(BlueIndexs.end(), TempBlueIndexs.begin(), TempBlueIndexs.end());
	}
	//*/
}


/*
2019.09.27 ��DBScan�Ļ����ϼ��Ϸֲ���� DBScan���Էָ�һ���߶Ȳ��ϵĶ�����


*/
/*
void CDBScanClusterWithSlice::RunSliceCluster(double SliceHeightValue, double RadiusValue, int MinPtsValue)
{
	SliceHeight = SliceHeightValue;
	Radius = RadiusValue;
	MinPts = MinPtsValue;

	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetParameters(SliceHeight);
	HorizontalPartition.PatitionSection();

	for (int i = 0; i < SlicesClusterS.size(); i++)
	{
		SlicesClusterS[i].clear();
	}
	SlicesClusterS.clear();

	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		//cout<<"���ڶԵ�"<<i<<"��Slice����DBScan����"<<endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSectionPoints (new pcl::PointCloud<pcl::PointXYZRGB>());
		HorizontalPartition.GetSectionPoints(i, CurSectionPoints);

		CDBScanCluster CurDBScanCluster;
		CurDBScanCluster.SetInputCloud(CurSectionPoints);
		CurDBScanCluster.RunCluster(Radius, MinPts);
		
		CurDBScanCluster.CalcClusterParameters();

		for (int k = 0; k < CurDBScanCluster.Clusters.size(); k++)
		{
			CurDBScanCluster.Clusters[k].ID = -1;

			for (int ii = 0; ii < CurDBScanCluster.Clusters[k].Indexs.size(); ii++)
			{
				int TempIndex = CurDBScanCluster.Clusters[k].Indexs[ii];
				CurDBScanCluster.Clusters[k].Indexs[ii] = HorizontalPartition.SectionsVector[i].Indexs[TempIndex];
			}
		}

		//�˴�Ҫ�Ե�ǰSlice���о���ĵ������滻Ϊȫ������		

		SlicesClusterS.push_back(CurDBScanCluster.Clusters);
		//cout << "��" << i << "��Slice����DBScan�����ѽ���" << endl;
	}
	
	//���²��໥ӳ��
	//SlicesMappingByAngle();
	SlicesMappingFromDownToUp();
}

//�ڶ����֮�佫��֮������ɼ���֦λ��ӳ�� 2019.09.27 ��д����
//���ɵ�������
void CDBScanClusterWithSlice::SlicesMappingByAngle()
{
	ID = 0;
	for (int i = 0; i < SlicesClusterS.size(); i++)
	{		
		//cout << "���ڶԵ�" << i << "��Slice����SlicesMapping" << endl;
		if (i == 0)
		{
			SlicesClusterS[0][0].ID = 0;
		}

		//�д���1���ľ��࣬��Ҫ��ǰһ��Slice�ľ���ϲ���Ĭ�ϴ�����λ�ÿ�ʼ�����Ե�һ��Slice�ض���һ������
		if (SlicesClusterS[i].size() > 0 && i > 0)
		{	
			////��һ�Ƚϣ�����һ�����һ���ĸ�������������������Ϊͬһ�����
			//int LowIndex = i - 1, UpperIndex = i;

			////���ǰһ��ľ������С�ں�һ��ľ������ ���Ժ�һ��ľ���Ϊ��׼ 
			//if (SlicesClusterS[LowIndex].size > SlicesClusterS[UpperIndex].size)
			//{

			//}
			//���ǰһSlice(i - 1)�ľ������С�ں�һSlice(i)�ľ������,����ǰ���Ϊ��׼���պ����
			//if (SlicesClusterS[i - 1].size() <= SlicesClusterS[i].size())
			{
				int FristSmallIndex = -1;
				for (int j = 0; j < SlicesClusterS[i - 1].size(); j++)
				{
					double SmallAngle = 90;
					int SmallIndex = -1;

					for (int k = 0; k < SlicesClusterS[i].size(); k++)
					{
						//����Ѿ������, �ͼ���ִ����һ��ѭ��
						if (SlicesClusterS[i][k].ID != -1) 
							continue;

						double Angle = GeometryBase::RadianToAngle(
							GeometryBase::AngleOfTwoVector(
								SlicesClusterS[i - 1][j].MaxDirection,
								SlicesClusterS[i][k].MaxDirection));
						if (Angle > 90)
							Angle = 180 - Angle;

						cout << "��" << i << "��Slice�ĵ�" << k << "����" <<
							"��" << i - 1 << "��Slice�ĵ�" << j << "�ص������Ƕ�ֵΪ:"<< Angle << endl;

						if (Angle < SmallAngle)
						{
							SmallAngle = Angle;
							SmallIndex = k;
						}
					}

					if (SmallIndex != -1)
					{
						cout << "��" << i << "��Slice�ĵ�" << SmallIndex <<"����"<<
							"��" << i - 1 << "��Slice�ĵ�" << j << "��������"<<endl;

						SlicesClusterS[i][SmallIndex].ID =
							SlicesClusterS[i - 1][j].ID;
					}

					//ǰһ��������,����������ɲ��ڵ�һ����Ԫ �����
					if (j == 0 && SmallIndex != 0)
					{
						FristSmallIndex = SmallIndex;
					}
				}

				if (FristSmallIndex != -1)
				{
					//����������Ԫ��,ȷ�����ɵ�����ÿ��0��λ��
					swap(SlicesClusterS[i][0], SlicesClusterS[i][FristSmallIndex]);
				}

				for (int k = 0; k < SlicesClusterS[i].size(); k++)
				{
					if (SlicesClusterS[i][k].ID == -1)
					{
						SlicesClusterS[i][k].ID = ID++;
					}
				}
			}
			// ���ǰһSlice(i - 1)�ľ���������ں�һSlice(i)�ľ������,���Ժ�һSlice(i)��Ϊ��׼����ǰһSlice(i - 1)��
			//else if (SlicesClusterS[i - 1].size() > SlicesClusterS[i].size())
			//{
			//	// 
			//	for (int j = 0; j < SlicesClusterS[i].size(); j++)
			//	{
			//		//����Ѿ������, �ͼ���ִ����һ��ѭ��
			//		if (SlicesClusterS[i][j].ID != -1)
			//			continue;

			//		double SmallAngle = 90;
			//		int SmallIndex = -1;

			//		for (int k = 0; k < SlicesClusterS[i - 1].size(); k++)
			//		{
			//			double Angle = GeometryBase::RadianToAngle(
			//				GeometryBase::AngleOfTwoVector(
			//					SlicesClusterS[i - 1][k].MaxDirection,
			//					SlicesClusterS[i][j].MaxDirection));
			//			if (Angle > 90)
			//				Angle = 180 - Angle;

			//			cout << "��" << i << "��Slice�ĵ�" << j << "����" <<
			//				"��" << i - 1 << "��Slice�ĵ�" << k << "�ص������Ƕ�ֵΪ:" << Angle << endl;

			//			if (Angle < SmallAngle)
			//			{
			//				SmallAngle = Angle;
			//				SmallIndex = k;
			//			}
			//		}

			//		if (SmallIndex != -1)
			//		{
			//			cout << "��" << i << "��Slice�ĵ�" << j << "����" <<
			//				"��" << i - 1 << "��Slice�ĵ�" << SmallIndex << "��������" << endl;

			//			SlicesClusterS[i][j].ID =
			//				SlicesClusterS[i - 1][SmallIndex].ID;
			//		}
			//	}

			//	for (int k = 0; k < SlicesClusterS[i].size(); k++)
			//	{
			//		if (SlicesClusterS[i][k].ID == -1)
			//		{
			//			SlicesClusterS[i][k].ID = ID++;
			//		}
			//	}
			//}
		}
		//cout << "��" << i << "��Slice��SlicesMapping�Ѿ�����" << endl;
	}
}

//�������ɾ��� �������� ����ע 
void CDBScanClusterWithSlice::SlicesMappingFromDownToUp()
{
	ID = 0;
	for (int i = 0; i < SlicesClusterS.size(); i++)
	{
		//cout << "���ڶԵ�" << i << "��Slice����SlicesMapping" << endl;
		//Ĭ��ǰ2��Slice��ֻ��һ������,
		if (i < 2)
		{
			SlicesClusterS[i][0].ID = 0;
			continue;
		}	

		//�д���1���ľ��࣬��Ҫ��ǰһ��Slice�ľ���ϲ���Ĭ�ϴ�����λ�ÿ�ʼ�����Ե�һ��Slice�ض���һ������
		if (SlicesClusterS[i].size() > 0 && i > 1)
		{
			pcl::PointXYZRGB StemDirection;
			
			StemDirection.x = SlicesClusterS[i - 1][0].Centroid.x - SlicesClusterS[i - 2][0].Centroid.x;
			StemDirection.y = SlicesClusterS[i - 1][0].Centroid.y - SlicesClusterS[i - 2][0].Centroid.y;
			StemDirection.z = SlicesClusterS[i - 1][0].Centroid.z - SlicesClusterS[i - 2][0].Centroid.z;
			
			double MinAnlge = 90;
			int MinIndex = -1;
			for (int j = 0; j < SlicesClusterS[i].size(); j++)
			{
				pcl::PointXYZRGB CurDirection;

				CurDirection.x = SlicesClusterS[i][j].Centroid.x - SlicesClusterS[i - 1][0].Centroid.x;
				CurDirection.y = SlicesClusterS[i][j].Centroid.y - SlicesClusterS[i - 1][0].Centroid.y;
				CurDirection.z = SlicesClusterS[i][j].Centroid.z - SlicesClusterS[i - 1][0].Centroid.z;

				double Angle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
												StemDirection, CurDirection));
				//if (Angle < 90) Angle = 180 - Angle;

				if (MinAnlge > Angle)
				{
					MinAnlge = Angle;
					MinIndex = j;
				}
			}
			if (MinIndex != -1)
			{
				cout << "��" << i << "��Slice�ĵ�" << MinIndex << "����" <<
					"��" << i - 1 << "��Slice�ĵ�" << 0 << "��������,��Ƕ�Ϊ��" << MinAnlge << endl;

				SlicesClusterS[i][MinIndex].ID = SlicesClusterS[i - 1][0].ID;

				//��������0��Ԫ��
				swap(SlicesClusterS[i][MinIndex], SlicesClusterS[i][0]);
			}
			else
				cout<<"û���ҵ����ʵ�ƥ����"<<endl;

			for (int k = 0; k < SlicesClusterS[i].size(); k++)
			{
				if (SlicesClusterS[i][k].ID == -1)
				{
					SlicesClusterS[i][k].ID = 1;
				}
			}
		}		
	}
}

void CDBScanClusterWithSlice::SetColorsByID()
{
	for (int i = 0; i < SlicesClusterS.size(); i++)
	{
		//cout << "��" << i << "��Slice�ľ������Ϊ" << SlicesClusterS[i].size() << endl;
		for (int j = 0; j < SlicesClusterS[i].size(); j++)
		{
			//cout << "��" << i << "��Slice�ĵ�" << j <<"�����е�ĸ���Ϊ:" << SlicesClusterS[i][j].Indexs.size() << endl;
			for (int k = 0; k < SlicesClusterS[i][j].Indexs.size(); k++)
			{
				int PointIndex = SlicesClusterS[i][j].Indexs[k];
				int Color = ColorBaseS[SlicesClusterS[i][j].ID % 29 + 1];
				//cout << "��" << PointIndex << "������ɫΪ"<< Color << endl;
				InputCloud->points[PointIndex].rgba = Color;
			}
		}
	}
}

*/
/*
2019.09.18 ��DBScan�Ļ����ϼ������ʵ��жϣ�������С�ڸ�����ֵ����������죬����ֹͣ
*/


//
//void CDBScanWithCurvatruePath::SetParameters(double SliceHeightValue, 
//	double RadiusValue, int MinPtsValue, CPointGeometry * pPointGeometry)
//{
//	p_PointGeometry = p_PointGeometry;
//	SliceHeight = SliceHeightValue;
//	Radius = RadiusValue;
//	MinPts = MinPtsValue;
//}
//
//void CDBScanWithCurvatruePath::RunClusterCurvatruePath()
//{
//	CalcSliceCurvature();
//	for (int i = 1; i < HorizontalPartition.SectionsCount; i++)
//	{
//		RunClusterSliceCurvatruePath(i);
//	}
//}
//
//void CDBScanWithCurvatruePath::RunClusterSliceCurvatruePath(int SliceIndex)
//{	
//	vector<int> RedIndexs, BlueIndexs;
//	p_PointGeometry->SearchCurvaturePathPoints(
//		HorizontalPartition.SectionsVector[SliceIndex].Indexs, RedIndexs, BlueIndexs);
//}
//
//void CDBScanWithCurvatruePath::ExpandCluster(int SliceIndex, int PointIndex, 
//	vector<int> PointNeighbourS, int ClusterIndex)
//{
//
//}

//CDBScanClusterWithSlice::CDBScanClusterWithSlice()
//{
//
//}
//
//CDBScanClusterWithSlice::~CDBScanClusterWithSlice()
//{
//
//}
