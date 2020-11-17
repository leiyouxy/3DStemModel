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
	//#pragma omp parallel for 此处如果并行计算会出现本应该在一个类的点属于多个类，
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
			else if (!IsUseOutliers)	//不使用异常值
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
			else// Outliers 使用异常值
			{
				OutliersClusters.push_back(i);
			}
		}
	}
	
	cout<<"DBScan Cluster has finished. The Number of Cluster is "<< Clusters.size() <<endl;
}

//一半的点进来还是有问题，
//void CDBScanCluster::RunClusterTwo(double RadiusValue, int MinPtsValue)
//{
//	Radius = RadiusValue;
//	MinPts = MinPtsValue;
//
//	Clusters.clear();
//	OutliersClusters.clear();
//	//cout<<"RunCluster is doing, please waiting!"<<endl;
//	//#pragma omp parallel for 此处如果并行计算会出现本应该在一个类的点属于多个类，
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

//扩充点并将点的邻域加入到类中
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
			else //是PointIndex的邻域，但不是Core Point，如果也不是异常点，则加入类ClusterIndex中
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


////一半的点进来还是有问题，2020.02.21
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
//			else //是PointIndex的邻域，但不是Core Point，如果也不是异常点，则加入类ClusterIndex中
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

		if (IndexsForParallel[TempPointIndex] == -2)	//如果没有处理则处理
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

						//按照顺序存储
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
			else //是PointIndex的邻域，但不是Core Point，如果也不是异常点，则加入类ClusterIndex中
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
		
		//2020.04.11 使用中心点作为
		PointBase::GetPointsCenterXYZ(CurClusterPointsPtr,
			Clusters[i].Centroid.x, Clusters[i].Centroid.y, Clusters[i].Centroid.z);		
		//Clusters[i].Centroid.z = TempMinz;
		//Clusters[i].Centroid.z = ZMin;		

		//2020.02.14 如果大于等于3个点
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
//并行计算的方法 2020.02.20
void CDBScanCluster::RunClusterParallel(double RadiusValue, int MinPtsValue)
{
	Radius = RadiusValue;
	MinPts = MinPtsValue;

	Clusters.clear();
	OutliersClusters.clear();

	//初始化
	IndexsForParallel.clear();
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		//-2为未使用，-1为噪声点，从0开始为类号
		IndexsForParallel.push_back(-2);
		PointVisited[i] = false;
	}
	ClusterID = -1;

	//cout<<"RunCluster is doing, please waiting!"<<endl;
	//#pragma omp parallel for //此处如果并行计算会出现本应该在一个类的点属于多个类，
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
					//已经赋值，不是噪声点，则需要记录下两个相同的簇的编号
					if (
						(IndexsForParallel[NeighbourIndexS[j]] == -2 ||
							IndexsForParallel[NeighbourIndexS[j]] == -1))
					{
						IndexsForParallel[NeighbourIndexS[j]] = ClusterID;
					}
					else if (IndexsForParallel[NeighbourIndexS[j]] != ClusterID)
					{
						SameCluster TempSameCluster;

						//按照顺序存储
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

			//当前点是曲率值比较大的点
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

				//如果邻域内有点的曲率大于阈值，则该处也不处理
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

			//当前点是曲率值比较大的点
			if (p_PointGeometry->GetGuassCurvature(TempPointIndex) > CurvatureThreshold0)
				continue;

			vector<int> NeighbourIndexS;
			vector<float> NeighbourDisS;
			GeometryBase::GetNeighbourInRadius(Octree, Radius, TempPointIndex,
				&NeighbourIndexS, &NeighbourDisS);

			if (NeighbourIndexS.size() >= MinPts)	//也是 Core Point
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

				//如果邻域内有点的曲率大于阈值，则该处也不处理
				if (Find) continue;

				//int ClusterIndex = Clusters.size() - 1;	//Index Of Current Cluster
				PointNeighbourS.insert(PointNeighbourS.end(),
					NeighbourIndexS.begin(), NeighbourIndexS.end());
			}
			else// //是PointIndex的邻域，但不是Core Point，如果也不是异常点，则加入类ClusterIndex中
			{
				VectorBase<int> VectorBaseInt;
				int FindIndex = VectorBaseInt.FindIndexInVector(OutliersClusters, TempPointIndex);
				if (FindIndex == -1)
				{
					Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);
				}
			}

			////当前点的邻域包括曲率值比较大的点
			//for (int ii = NeighbourIndexS.size() - 1; ii >= 0; ii--)
			//{
			//	if (p_PointGeometry->GetGuassCurvature(NeighbourIndexS[ii]) > CurvatureThreshold)
			//	{
			//		NeighbourIndexS.erase(NeighbourIndexS.begin() + ii);
			//	}
			//}
			////去掉曲率大于门限后的点的情况
			//if (NeighbourIndexS.size() >= MinPts)	//Core Point
			//{
			//	Clusters[ClusterIndex].Indexs.push_back(TempPointIndex);
			//	PointNeighbourS.insert(PointNeighbourS.end(),
			//		NeighbourIndexS.begin(), NeighbourIndexS.end());
			//}
			//else //是PointIndex的邻域，但不是Core Point，如果也不是异常点，则加入类ClusterIndex中
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

//根据最大类的最大方向，如果有小类与最大类的最大方向相近，则合并
//合并的两个方向的角度门限
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

		if (TempAngle < Anlge)	//可以合并
		{
			cout << "合并时的角度 " << TempAngle << endl;

			Clusters[MainIndex].Indexs.insert(Clusters[MainIndex].Indexs.begin(),
				Clusters[i].Indexs.begin(), Clusters[i].Indexs.end());

			Clusters[i].Indexs.clear();
		}
	}
}

//此函数使用时需要再次处理
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

			//当前点是曲率值比较大的点
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

				//如果邻域内有点的曲率大于阈值，则该处也不处理
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

			//当前点是曲率值比较大的点
			if (p_PointGeometry->GetGuassCurvature(TempPointIndex) > CurCurvatureThreshold)
				continue;

			vector<int> NeighbourIndexS;
			vector<float> NeighbourDisS;
			GeometryBase::GetNeighbourInRadius(Octree, Radius, TempPointIndex,
				&NeighbourIndexS, &NeighbourDisS);

			if (NeighbourIndexS.size() >= MinPts)	//也是 Core Point
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

				//如果邻域内有点的曲率大于阈值，则该处也不处理
				if (Find) continue;

				//int ClusterIndex = Clusters.size() - 1;	//Index Of Current Cluster
				PointNeighbourS.insert(PointNeighbourS.end(),
					NeighbourIndexS.begin(), NeighbourIndexS.end());
			}
			else// //是PointIndex的邻域，但不是Core Point，如果也不是异常点，则加入类ClusterIndex中
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

	///*/// 计算每一层的曲率均值与标准差
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

		cout << "第" << i << "个Slice的曲率均值为：" << TempMeanCurvature << endl;
		cout << "第" << i << "个Slice的曲率方差为：" << TempStdCurvature << endl;

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
2019.09.27 在DBScan的基础上加上分层逐层 DBScan，以分割一个高度层上的多个树杈


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
		//cout<<"正在对第"<<i<<"个Slice进行DBScan分类"<<endl;
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

		//此处要对当前Slice所有聚类的点索引替换为全局索引		

		SlicesClusterS.push_back(CurDBScanCluster.Clusters);
		//cout << "第" << i << "个Slice进行DBScan分类已结束" << endl;
	}
	
	//上下层相互映射
	//SlicesMappingByAngle();
	SlicesMappingFromDownToUp();
}

//在多个层之间将层之间的树干及树枝位置映射 2019.09.27 拟写程序
//树干的类别均在
void CDBScanClusterWithSlice::SlicesMappingByAngle()
{
	ID = 0;
	for (int i = 0; i < SlicesClusterS.size(); i++)
	{		
		//cout << "正在对第" << i << "个Slice进行SlicesMapping" << endl;
		if (i == 0)
		{
			SlicesClusterS[0][0].ID = 0;
		}

		//有大于1个的聚类，需要向前一个Slice的聚类合并，默认从树干位置开始，所以第一个Slice必定是一个聚类
		if (SlicesClusterS[i].size() > 0 && i > 0)
		{	
			////逐一比较，看上一层和下一层哪个方向最近，方向最近的为同一个类别
			//int LowIndex = i - 1, UpperIndex = i;

			////如果前一层的聚类个数小于后一层的聚类个数 则以后一层的聚类为标准 
			//if (SlicesClusterS[LowIndex].size > SlicesClusterS[UpperIndex].size)
			//{

			//}
			//如果前一Slice(i - 1)的聚类个数小于后一Slice(i)的聚类个数,则以前面的为基准对照后面的
			//if (SlicesClusterS[i - 1].size() <= SlicesClusterS[i].size())
			{
				int FristSmallIndex = -1;
				for (int j = 0; j < SlicesClusterS[i - 1].size(); j++)
				{
					double SmallAngle = 90;
					int SmallIndex = -1;

					for (int k = 0; k < SlicesClusterS[i].size(); k++)
					{
						//如果已经定标过, 就继续执行下一个循环
						if (SlicesClusterS[i][k].ID != -1) 
							continue;

						double Angle = GeometryBase::RadianToAngle(
							GeometryBase::AngleOfTwoVector(
								SlicesClusterS[i - 1][j].MaxDirection,
								SlicesClusterS[i][k].MaxDirection));
						if (Angle > 90)
							Angle = 180 - Angle;

						cout << "第" << i << "个Slice的第" << k << "簇与" <<
							"第" << i - 1 << "个Slice的第" << j << "簇的最大方向角度值为:"<< Angle << endl;

						if (Angle < SmallAngle)
						{
							SmallAngle = Angle;
							SmallIndex = k;
						}
					}

					if (SmallIndex != -1)
					{
						cout << "第" << i << "个Slice的第" << SmallIndex <<"簇与"<<
							"第" << i - 1 << "个Slice的第" << j << "簇相连接"<<endl;

						SlicesClusterS[i][SmallIndex].ID =
							SlicesClusterS[i - 1][j].ID;
					}

					//前一层是树干,而后面的树干不在第一个单元 则调换
					if (j == 0 && SmallIndex != 0)
					{
						FristSmallIndex = SmallIndex;
					}
				}

				if (FristSmallIndex != -1)
				{
					//交换这两个元素,确保树干点云在每层0号位置
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
			// 如果前一Slice(i - 1)的聚类个数大于后一Slice(i)的聚类个数,则以后一Slice(i)的为基准对照前一Slice(i - 1)的
			//else if (SlicesClusterS[i - 1].size() > SlicesClusterS[i].size())
			//{
			//	// 
			//	for (int j = 0; j < SlicesClusterS[i].size(); j++)
			//	{
			//		//如果已经定标过, 就继续执行下一个循环
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

			//			cout << "第" << i << "个Slice的第" << j << "簇与" <<
			//				"第" << i - 1 << "个Slice的第" << k << "簇的最大方向角度值为:" << Angle << endl;

			//			if (Angle < SmallAngle)
			//			{
			//				SmallAngle = Angle;
			//				SmallIndex = k;
			//			}
			//		}

			//		if (SmallIndex != -1)
			//		{
			//			cout << "第" << i << "个Slice的第" << j << "簇与" <<
			//				"第" << i - 1 << "个Slice的第" << SmallIndex << "簇相连接" << endl;

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
		//cout << "第" << i << "个Slice的SlicesMapping已经结束" << endl;
	}
}

//根据树干聚类 从下至上 逐层标注 
void CDBScanClusterWithSlice::SlicesMappingFromDownToUp()
{
	ID = 0;
	for (int i = 0; i < SlicesClusterS.size(); i++)
	{
		//cout << "正在对第" << i << "个Slice进行SlicesMapping" << endl;
		//默认前2个Slice都只有一个聚类,
		if (i < 2)
		{
			SlicesClusterS[i][0].ID = 0;
			continue;
		}	

		//有大于1个的聚类，需要向前一个Slice的聚类合并，默认从树干位置开始，所以第一个Slice必定是一个聚类
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
				cout << "第" << i << "个Slice的第" << MinIndex << "簇与" <<
					"第" << i - 1 << "个Slice的第" << 0 << "簇相连接,其角度为：" << MinAnlge << endl;

				SlicesClusterS[i][MinIndex].ID = SlicesClusterS[i - 1][0].ID;

				//交换到第0号元素
				swap(SlicesClusterS[i][MinIndex], SlicesClusterS[i][0]);
			}
			else
				cout<<"没有找到合适的匹配项"<<endl;

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
		//cout << "第" << i << "个Slice的聚类个数为" << SlicesClusterS[i].size() << endl;
		for (int j = 0; j < SlicesClusterS[i].size(); j++)
		{
			//cout << "第" << i << "个Slice的第" << j <<"个类中点的个数为:" << SlicesClusterS[i][j].Indexs.size() << endl;
			for (int k = 0; k < SlicesClusterS[i][j].Indexs.size(); k++)
			{
				int PointIndex = SlicesClusterS[i][j].Indexs[k];
				int Color = ColorBaseS[SlicesClusterS[i][j].ID % 29 + 1];
				//cout << "第" << PointIndex << "个的颜色为"<< Color << endl;
				InputCloud->points[PointIndex].rgba = Color;
			}
		}
	}
}

*/
/*
2019.09.18 在DBScan的基础上加上曲率的判断，若曲率小于给定阈值，则继续延伸，否则停止
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
