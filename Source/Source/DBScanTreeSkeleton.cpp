#include "DBScanTreeSkeleton.h"

void CDBScanTreeSkeleton::ResetSliceDBScanClusters()
{
	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		delete SliceDBScanClusters[i].pCluster;
		SliceDBScanClusters[i].pCluster = NULL;
		SliceDBScanClusters[i].SectionPoints->points.clear();
	}
	SliceDBScanClusters.clear();
}

//当前正在使用的函数  //根据点集的连续性获取父节点 2020.04.11
bool CDBScanTreeSkeleton::GetParentSliceCluster(int SliceIndex, int ClusterIndex, double SearchRadius, bool Bottom)
{
	double ZValue = EPSP6;
	int ZPointIndex = -1;

	if (Bottom)
		ZValue = EPSP6;
	else
		ZValue = EPSM6;

	//cout << endl;
	//cout << "SliceIndex:" << SliceIndex
	//	<< ", ClusterIndex:" << ClusterIndex
	//	<< ", Indexs.size:" << SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Indexs.size()	
	//	<< ", GlobalIndexs.size:" 
	//	<< SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size() << endl;

	for (int i = 0; i < SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size(); i++)
	{
		//SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Indexs 只是
		// 一个HorizontalPartition中点集的索引，还需要转换到全局索引中
		int TempGloablIndex = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs[i];

		if (Bottom)
		{
			if (InputCloud->points[TempGloablIndex].z < ZValue)
			{
				ZValue = InputCloud->points[TempGloablIndex].z;
				ZPointIndex = TempGloablIndex;
			}
		}
		else
		{
			if (InputCloud->points[TempGloablIndex].z > ZValue)
			{
				ZValue = InputCloud->points[TempGloablIndex].z;
				ZPointIndex = TempGloablIndex;
			}
		}
	}

	//cout << "ZMin:" << ZMin << endl;
	//cout<<"ZMinPointIndex:"<< ZMinPointIndex <<endl;
	if (ZPointIndex == -1)
	{
		cout << "ZMinPointIndex:" << ZPointIndex << endl;
		cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:" << ClusterIndex << "未找到父节点！" << endl;
		return false;
	}

	//if (SliceIndex == 221 && (ClusterIndex == 0)) 
	//{
	//	cout << "SliceIndex:" << SliceIndex << ", ClusterIndex:" << ClusterIndex
	//		<< ", Bottom:" << Bottom << endl;
	//}

	vector<int> NeighbourIndexS;
	vector<float> NeighbourDisS;

	//Find neighbor points for ZPointIndex
	GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, SearchRadius, ZPointIndex, &NeighbourIndexS, &NeighbourDisS);

	//cout << "NeighbourIndexS, Size:" << NeighbourIndexS.size() << endl;

	//邻域点在当前聚类中没有发现，则是另外一个聚类的点
	//int FindPointIndex = -1;
	VectorBase<int> VectorBaseInt;

	//属于其他不同聚类的一些点的索引
	vector<int> FindPointIndexs;
	
	//Find UNBelongs Points not belong to the current cluster in the current slice
	for (int i = 0; i < NeighbourIndexS.size(); i++)
	{
		//cout << "NeighbourIndexS[i]:" << NeighbourIndexS[i] << endl;
		if (VectorBaseInt.FindIndexInVector(SliceDBScanClusters[SliceIndex].
			pCluster->Clusters[ClusterIndex].GlobalIndexs, NeighbourIndexS[i]) == -1)
		{
			//FindPointIndex = NeighbourIndexS[i];
			FindPointIndexs.push_back(NeighbourIndexS[i]);
			//break;
		}
	}

	//cout << "FindPointIndex:" << FindPointIndex << endl;
	//cin >> FindPointIndex;
	if (FindPointIndexs.size() == 0)
	{
		cout << "FindPointIndex:" << FindPointIndexs.size() << endl;
		cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:" << ClusterIndex << "未找到父节点！" << endl;
		return false;
	}

	vector<int> ParentSliceIndexs, ParentClusterIndexs;
	//find ParentSliceIndexs and ParentClusterIndexs indexs for the UNBelongs Points
	for (int k = 0; k < FindPointIndexs.size(); k++)
	{
		int FindPointIndex = FindPointIndexs[k];

		//for (int i = SliceIndex - 1; i >= 0; i--)
		for (int i = SliceIndex - 1; i >= SliceIndex - 1; i--) //在其下一层寻找即可
		{
			for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
			{
				if (VectorBaseInt.FindIndexInVector(SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs,
					FindPointIndex) != -1
					//确保父节点所在簇的高度小于等于当前簇的高度
					&& ((SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid.z
						<= SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.z && Bottom)
						//|| (SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid.z
						//	>= SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.z && !Bottom)))
						|| (!Bottom)))
				{
			
					//如果簇索引已经存在，则不添加
					bool Find = false;
					for (int ii = 0; ii < ParentSliceIndexs.size(); ii++)
					{
						if (ParentSliceIndexs[ii] == i && ParentClusterIndexs[ii] == j)
						{
							Find = true;
							break;
						}
					}
					//if (SliceIndex == 222 && (ClusterIndex == 7 || ClusterIndex == 9))
					//{
					//	cout << "Bottom:" << Bottom <<", Find"<< Find << endl;
					//}

					if (!Find)
					{
						ParentSliceIndexs.push_back(i);
						ParentClusterIndexs.push_back(j);
					}
				}
			}
		}
	}

	//if (SliceIndex == 246 && (ClusterIndex == 8 || ClusterIndex == 9))
	//if (SliceIndex == 222 && (ClusterIndex == 10))
	//{
	//	cout << "SliceIndex:" << SliceIndex << ", ClusterIndex:" << ClusterIndex 
	//	<< ", ParentSliceIndexs.size():" << ParentSliceIndexs.size()
	//	<< ", ParentClusterIndexs.size():" << ParentClusterIndexs.size()  << endl;

	//	//for (int i = 0; i < ParentSliceIndexs.size(); i++)
	//	//{
	//	//	cout <<"ParentSliceIndexs:"<<i<<":"<< ParentSliceIndexs[i]
	//	//		<<", ParentClusterIndexs:"<<i<<":"<< ParentClusterIndexs[i]<< endl;
	//	//}
	//}

	if (ParentSliceIndexs.size() == 0)
	{
		//cout << "SearchRadius:" << SearchRadius << endl;
		//cout <<"FindPointIndexs.size():" << FindPointIndexs.size() << endl;
		//cout<<"Bottom:"<< Bottom <<endl;
		//cout<< "ParentSliceIndexs.size():" << ParentSliceIndexs.size() << endl;
		//cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:" << ClusterIndex << "未找到父节点！" << endl;
		return false;
	}
	else if (ParentSliceIndexs.size() == 1)
	{
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.SliceIndex
			= ParentSliceIndexs[0];
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.ClusterIndex
			= ParentClusterIndexs[0];

		//Add child node to the parent Node
		Role TempNode;
		TempNode.SliceIndex = SliceIndex;
		TempNode.ClusterIndex = ClusterIndex;
		SliceDBScanClusters[ParentSliceIndexs[0]].
			pCluster->Clusters[ParentClusterIndexs[0]].NodeRoles.ChildNodes.push_back(TempNode);

		return true;
	}
	else
	{
		pcl::PointXYZRGB CurrentPoint, ParentPoint, GrandpaPoint;

		if (!UseConvexCentroid)
		{
			CurrentPoint = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid;
		}
		else if (UseConvexCentroid)
		{
			CurrentPoint = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid;
		}

		//Find the optimal parent cluster 
		double SmallDis = EPSP6, SmallDisAngle, MaxAngle = EPSM6, MaxAngleDis;
		int SmallDisIndex = -1, MaxAngleIndex = -1;

		for (int k = 0; k < ParentClusterIndexs.size(); k++)
		{
			int ParentSliceIndex = ParentSliceIndexs[k];
			int ParentClusterIndex = ParentClusterIndexs[k];

			//cout << "ParentSliceIndexs:" << k << ":" << ParentSliceIndex
			//	<< ", ParentClusterIndexs:" << k << ":" << ParentClusterIndex << endl;

			int GrandpaSliceIndex = -1, GrandpaClusterIndex = -1;

			GrandpaSliceIndex = SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].
				NodeRoles.ParentNode.SliceIndex;
			GrandpaClusterIndex = SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].
				NodeRoles.ParentNode.ClusterIndex;

			if (GrandpaSliceIndex == -1 || GrandpaClusterIndex == -1)
				continue;

			//if (SliceIndex == 222 && (ClusterIndex == 10))
			//{
			//	cout<< "ParentSliceIndexs.size():" << ParentSliceIndexs.size()<<
			//		",ParentSliceIndex:" << ParentSliceIndex <<
			//		",ParentClusterIndex:" << ParentClusterIndex <<
			//		",GrandpaSliceIndex:" << GrandpaSliceIndex<<
			//		",GrandpaClusterIndex:" << GrandpaClusterIndex					
			//		<< endl;
			//}

			if (!UseConvexCentroid)
			{
				ParentPoint = SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].Centroid;
				GrandpaPoint = SliceDBScanClusters[GrandpaSliceIndex].pCluster->Clusters[GrandpaClusterIndex].Centroid;
			}
			else
			{
				ParentPoint = SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].ConvexCentroid;
				GrandpaPoint = SliceDBScanClusters[GrandpaSliceIndex].pCluster->Clusters[GrandpaClusterIndex].ConvexCentroid;
			}

			double TempDis = PointDis(CurrentPoint, ParentPoint);
			double TempAngle = GeometryBase::RadianToAngle(
				GeometryBase::AngleValueOfThreePoints(CurrentPoint, ParentPoint, GrandpaPoint));

			if (TempDis < SmallDis)
			{
				SmallDis = TempDis;
				SmallDisIndex = k;
				SmallDisAngle = TempAngle;
			}

			if (TempAngle > MaxAngle)
			{
				MaxAngle = TempAngle;
				MaxAngleIndex = k;
				MaxAngleDis = TempDis;
			}
		}

		//if (SliceIndex == 222 && (ClusterIndex == 10))
		//{
		//	cout << "SliceIndex:" << SliceIndex << ", ClusterIndex:" << ClusterIndex
		//		<< ", ParentSliceIndexs.size():" << ParentSliceIndexs.size()
		//		<< ", ParentClusterIndexs.size():" << ParentClusterIndexs.size()
		//		<< ", SmallDisIndex:" << SmallDisIndex
		//		<< ", MaxAngleIndex" << MaxAngleIndex << endl;
		//}

		//距离最近 角度最大
		if (SmallDisIndex == MaxAngleIndex && SmallDisIndex != -1 && MaxAngleDis < 2 * SliceHeight)
		{
			SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.SliceIndex
				= ParentSliceIndexs[SmallDisIndex];
			SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.ClusterIndex
				= ParentClusterIndexs[SmallDisIndex];

			//Add child node to the parent Node
			Role TempNode;
			TempNode.SliceIndex = SliceIndex;
			TempNode.ClusterIndex = ClusterIndex;
			SliceDBScanClusters[ParentSliceIndexs[SmallDisIndex]].
				pCluster->Clusters[ParentClusterIndexs[SmallDisIndex]].NodeRoles.ChildNodes.push_back(TempNode);

			return true;
		}
		else if (SmallDisIndex != -1 && MaxAngleIndex != -1)
		{
			//如果最小距离的角度合适就作为父节点
			if (SmallDisAngle > 150 && SmallDisIndex != -1 && SmallDis < 2 * SliceHeight)
			{
				//cout<<"SmallDisIndex:"<< SmallDisIndex <<endl;
				SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.SliceIndex
					= ParentSliceIndexs[SmallDisIndex];
				SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.ClusterIndex
					= ParentClusterIndexs[SmallDisIndex];

				//Add child node to the parent Node
				Role TempNode;
				TempNode.SliceIndex = SliceIndex;
				TempNode.ClusterIndex = ClusterIndex;
				SliceDBScanClusters[ParentSliceIndexs[SmallDisIndex]].
					pCluster->Clusters[ParentClusterIndexs[SmallDisIndex]].NodeRoles.ChildNodes.push_back(TempNode);

				return true;
			}
			//如果最大角度对应的距离小于2倍的 SliceHeight
			else if (MaxAngleDis < 2 * SliceHeight && MaxAngleIndex != -1) 
			{
				//cout << "MaxAngleIndex:" << MaxAngleIndex << endl;
				SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.SliceIndex
					= ParentSliceIndexs[MaxAngleIndex];
				SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.ClusterIndex
					= ParentClusterIndexs[MaxAngleIndex];

				//Add child node to the parent Node
				Role TempNode;
				TempNode.SliceIndex = SliceIndex;
				TempNode.ClusterIndex = ClusterIndex;
				SliceDBScanClusters[ParentSliceIndexs[MaxAngleIndex]].
					pCluster->Clusters[ParentClusterIndexs[MaxAngleIndex]].NodeRoles.ChildNodes.push_back(TempNode);

				return true;
			}
			//最后的选择只好根据距离选择
			else if (SmallDisIndex != -1 && SmallDis < 2 * SliceHeight) 
			{
				SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.SliceIndex
					= ParentSliceIndexs[SmallDisIndex];
				SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.ClusterIndex
					= ParentClusterIndexs[SmallDisIndex];

				//Add child node to the parent Node
				Role TempNode;
				TempNode.SliceIndex = SliceIndex;
				TempNode.ClusterIndex = ClusterIndex;
				SliceDBScanClusters[ParentSliceIndexs[SmallDisIndex]].
					pCluster->Clusters[ParentClusterIndexs[SmallDisIndex]].NodeRoles.ChildNodes.push_back(TempNode);

				return true;
			}
			else
			{
				cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:" 
					<< ClusterIndex << "未找到父节点！Bottom:"<<Bottom << endl;
				return false;
			}
		}
		else
			return false;
	}
}

//By continuity of the direction 2020.02.25
bool CDBScanTreeSkeleton::GetParentSliceClusterByDirectionContinuity(int SliceIndex, int ClusterIndex)
{
	//No Child Node to Calculate the Direction;
	if (SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ChildNodes.size() == 0)
		return false;

	cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
		<< ClusterIndex << "寻找父节点By DirectionContinuity" << endl;

	pcl::PointXYZRGB CurrentNode, OptimalChildNode, ChildNode, ZAxisPoint, ParentNode;
	
	int ChildSliceIndex, ChildClusterIndex;

	int OptimalChildIndex = -1;
	double MaxAngle = EPSM3;

	// Find the optimal child point, which, current point and the projection of the current point in xy plane formed the three points that 
	//formed the biggest angle.
	for(int i = 0; i < SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ChildNodes.size(); i++)
	{
		ChildSliceIndex = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ChildNodes[i].SliceIndex;
		ChildClusterIndex = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ChildNodes[i].ClusterIndex;

		if (!UseConvexCentroid)
		{
			CurrentNode = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid;
			ChildNode = SliceDBScanClusters[ChildSliceIndex].pCluster->Clusters[ChildClusterIndex].Centroid;
		}
		else
		{
			CurrentNode = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid;
			ChildNode = SliceDBScanClusters[ChildSliceIndex].pCluster->Clusters[ChildClusterIndex].ConvexCentroid;
		}
		
		ZAxisPoint.x = CurrentNode.x, ZAxisPoint.y = CurrentNode.y, ZAxisPoint.z = 0;
		//CurrentDirection = PointMinus(CurrentNode, ChildNode);
		double TempAnlge = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
			ChildNode, CurrentNode, ZAxisPoint));
		
		if (TempAnlge > MaxAngle)
		{
			MaxAngle = TempAnlge;
			OptimalChildIndex = i;
			OptimalChildNode = ChildNode;
		}
	}

	cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
		<< ClusterIndex << "寻找父节点By OptimalChildIndex:"<< OptimalChildIndex << endl;

	if (OptimalChildIndex == -1) return false;

	int ParentSliceIndex = -1, ParentClusterIndex = -1;

	MaxAngle = EPSM3;
	//find the parent slice in the serveral successive slices, 
	//that the height of the successive slices is not bigger than quarter of the tree height	
	for (int i = SliceIndex - 1; i >= SliceIndex - SliceDBScanClusters.size() / 4 && i >= 0; i--)
	{
		cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
			<< ClusterIndex << "寻找父节点By i:" << i << endl;

		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			//if a pointn has a child, then continue;
			if (SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ChildNodes.size() > 0)
				continue;									

			if (!UseConvexCentroid)
			{
				ParentNode = SliceDBScanClusters[i].pCluster->Clusters[j].Centroid;				
			}
			else
			{
				ParentNode = SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid;				
			}

			double TempAnlge = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
				OptimalChildNode, CurrentNode, ParentNode));

			//cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
			//	<< ClusterIndex << "寻找父节点By OptimalChildNode:" << OptimalChildNode << endl;

			//cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
			//	<< ClusterIndex << "寻找父节点By CurrentNode:" << CurrentNode << endl;

			//cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
			//	<< ClusterIndex << "寻找父节点By ParentNode:" << ParentNode << endl;

			//cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
			//	<< ClusterIndex << "寻找父节点By TempAnlge:" << TempAnlge << endl;

			if (TempAnlge > MaxAngle)
			{
				MaxAngle = TempAnlge;
				ParentSliceIndex = i;
				ParentClusterIndex = j;
			}
		}
	}

	cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
		<< ClusterIndex << "寻找父节点By OptimalChildIndex, MaxAngle:" << MaxAngle << endl;

	if (MaxAngle > 150 && ParentSliceIndex != -1)
	{
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.SliceIndex = ParentSliceIndex;
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.ClusterIndex = ParentClusterIndex;

		//Add child node to the parent Node
		Role TempNode;
		TempNode.SliceIndex = SliceIndex;
		TempNode.ClusterIndex = ClusterIndex;
		SliceDBScanClusters[ParentSliceIndex].
			pCluster->Clusters[ParentClusterIndex].NodeRoles.ChildNodes.push_back(TempNode);

		return true;
	}

	return false;
}

void CDBScanTreeSkeleton::GetParentSliceClusterByNearestDis()
{
	//从 1 开始，最低的 0 层 没有父节点
	for (int i = 1; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			//仍然没找到Parent的聚类 找最近的点
			if (SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex == -1)
			{
				pcl::PointXYZRGB CurrentPoint, SmallPoint;

				if (!UseConvexCentroid)
					CurrentPoint = SliceDBScanClusters[i].pCluster->Clusters[j].Centroid;
				else
					CurrentPoint = SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid;

				double SmallDis = EPSP6;

				for (int k = 0; k < SkeletonPoints->points.size(); k++)
				{
					double TempDis = PointDis(SkeletonPoints->points[k], CurrentPoint);

					if (TempDis > 0.1 && TempDis < SmallDis)
					{
						SmallDis = TempDis;
						SmallPoint = SkeletonPoints->points[k];
					}
				}

				//最近的点与骨架中的点匹配
				int ParentSliceIndex = -1, ParentClusterIndex = -1;
				for (int ii = 0; ii < SliceDBScanClusters.size(); ii++)
				{
					for (int jj = 0; jj < SliceDBScanClusters[ii].pCluster->Clusters.size(); jj++)
					{
						double TempDis;
						if (!UseConvexCentroid)
							TempDis = PointDis(SmallPoint, SliceDBScanClusters[ii].pCluster->Clusters[jj].Centroid);
						else
							TempDis = PointDis(SmallPoint, SliceDBScanClusters[ii].pCluster->Clusters[jj].ConvexCentroid);

						if (TempDis < EPSM3)
						{
							ParentSliceIndex = ii;
							ParentClusterIndex = jj;
							cout<<"Find Nearest Point"<<endl;
						}
					}
				}

				if (ParentSliceIndex != -1)
				{
					SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex = ParentSliceIndex;
					SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex = ParentClusterIndex;
				}
			}
		}
	}
}

CDBScanTreeSkeleton::CDBScanTreeSkeleton()
{
	SkeletonPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	OctreeCloudSearch = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	UseConvexCentroid = true;
	SkeletonIsGenerate = false;
	Viewer = NULL;
}

CDBScanTreeSkeleton::~CDBScanTreeSkeleton()
{
	ResetSliceDBScanClusters();
}

int CDBScanTreeSkeleton::GetSlicesClustersCount()
{
	return SliceDBScanClusters.size();
}

void CDBScanTreeSkeleton::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud,
	boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue)
{
	InputCloud = pCloud;
	Viewer = ViewerValue;

	ResetSliceDBScanClusters();
}

//在相互Slice上根据DBScan规则合并簇
void CDBScanTreeSkeleton::MainTreeBranchesDetection(double RadiusValue, int MinPtsValue)
{
	//构建
	OctreeCloudSearch->deleteTree();
	OctreeCloudSearch->setInputCloud(InputCloud);
	OctreeCloudSearch->addPointsFromInputCloud();

	//cout<<"SliceDBScanClusters.size():"<< SliceDBScanClusters.size() <<endl;
	#pragma omp parallel for //先给每个聚类的ID赋值，以区分不同的聚类，然后好合并
	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		#pragma omp parallel for
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			SliceDBScanClusters[i].pCluster->Clusters[j].ID = (i+1) * 100 + j;
		}
	}

	for (int i = 1; i < SliceDBScanClusters.size(); i++)
	{		
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{				
			//vector<int> MinZIndexS;

			//获取一些与下层Slice距离较近的点
			//for (int k = 0; k < SliceDBScanClusters[i].pCluster->Clusters[j].Indexs.size(); k++)
			//{
			//	int TempPointIndex = SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs[k];

			//	//比最大值小一个半径的值
			//	if (InputCloud->points[TempPointIndex].z <= HorizontalPartition.SectionsVector[i].ZMin + RadiusValue)
			//	{
			//		MinZIndexS.push_back(TempPointIndex);
			//	}
			//}

			//if (MinZIndexS.size() == 0)
			//	cout << "j, MinZIndexS.size(),GlobalIndexs.size():"<<j<<"," << MinZIndexS.size() 
			//	<<"," << SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.size() << endl;

			int Find = -1;
			int FindClusterIndex = -1;
			
			//遍历上一步获取点的近邻，如果近邻点是一个Core Point 且位于在上层某一个簇的Core Point的邻域中，则二者是同一个簇
			//for (int k = 0; k < MinZIndexS.size(); k++)
			for (int k = 0; k < SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.size(); k++)
			{
				int UpperPointIndex = SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs[k];
				vector<int> NearestIndexs;
				vector<float> NearestDiss;
				GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue,
					//MinZIndexS[k], &NearestIndexs, &NearestDiss);
					UpperPointIndex, &NearestIndexs, &NearestDiss);
				
				//不是核心点
				if (NearestIndexs.size() < MinPtsValue)
					continue;

				for (int ii = 0; ii < NearestIndexs.size(); ii++)
				{
					VectorBase<int> VectorBaseInt;
					//点的邻域点集是否在下一Slice的某一个簇中，如果是，说明二者是同一个簇
					for (int kk = 0; kk < SliceDBScanClusters[i - 1].pCluster->Clusters.size(); kk++)
					{
						Find = VectorBaseInt.FindIndexInVector(
							SliceDBScanClusters[i - 1].pCluster->Clusters[kk].GlobalIndexs, NearestIndexs[ii]);

						if (Find > 0)
						{
							//遍历相邻簇，是否有Core Point的邻域包括 UpperPointIndex 点 
							for (int jj = 0; jj < SliceDBScanClusters[i - 1].pCluster->Clusters[kk].GlobalIndexs.size(); jj++)
							{
								int LowerPointIndex = SliceDBScanClusters[i - 1].pCluster->Clusters[kk].GlobalIndexs[jj];

								vector<int> kkNearestIndexs;
								vector<float> kkNearestDiss;
								GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue,
								//MinZIndexS[k], &NearestIndexs, &NearestDiss);
									LowerPointIndex, &kkNearestIndexs, &kkNearestDiss);
								
									if (kkNearestIndexs.size() < MinPtsValue)
										continue;

									int FindInLower = VectorBaseInt.FindIndexInVector(
										kkNearestIndexs, UpperPointIndex);
									
									if (FindInLower != -1)
									{
										FindClusterIndex = kk;
										break;
									}								
							}	
							if (FindClusterIndex != -1)
								break;
						}
					}
					if (FindClusterIndex != -1)
						break;
				}
				if (FindClusterIndex != -1)
					break;
			}

			if (FindClusterIndex != -1)
			{
				SliceDBScanClusters[i].pCluster->Clusters[j].ID =
					SliceDBScanClusters[i - 1].pCluster->Clusters[FindClusterIndex].ID;

				//cout << "Slice:" << i << ", Cluster Index:" << j << " will be replaced with Slice:" <<
				//	i - 1 << ", Cluster Index:" << FindClusterIndex <<
				//	", ID:" << SliceDBScanClusters[i - 1].pCluster->Clusters[FindClusterIndex].ID << endl;
			}
			//else
			//	cout << "Slice:" << i << ", Cluster Index:" << j << " not find" << endl;
		}		
		SliceDBScanClusters[i].pCluster->SetClusterColors();
	}
}

void CDBScanTreeSkeleton::MainTreeBranchesRetrieval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud, int Top)
{
	pCloud->points.clear();
	VectorBase<int> VectorBaseInt;
	
	vector<int> ClusterIDs, ClusterPointsNums;

	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			int TempID = SliceDBScanClusters[i].pCluster->Clusters[j].ID;

			int TempIDIndex = VectorBaseInt.FindIndexInVector(ClusterIDs, TempID);
			
			if (TempIDIndex != -1)	//如果找到则累加			
				ClusterPointsNums[TempIDIndex] += SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.size();			
			else 
			{
				ClusterIDs.push_back(TempID);
				ClusterPointsNums.push_back(SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.size());
			}
		}
	}

	//bubbling sort desc by the cluster numbers
	for (int i = 0; i < ClusterIDs.size() - 1; i++)
	{
		for (int j = i + 1; j < ClusterIDs.size(); j++)
		{
			if (ClusterPointsNums[i] < ClusterPointsNums[j])
			{
				swap(ClusterIDs[i], ClusterIDs[j]);
				swap(ClusterPointsNums[i], ClusterPointsNums[j]);
			}
		}
	}

	// 删除 Top 后面的点
	if (ClusterIDs.size() > Top)
	{
		ClusterIDs.erase(ClusterIDs.begin() + Top, ClusterIDs.end());
		ClusterPointsNums.erase(ClusterPointsNums.begin() + Top, ClusterPointsNums.end());
	}
	cout <<"ClusterIDs.size()"<< ClusterIDs.size() <<endl;
	cout << "ClusterPointsNums.size()" << ClusterPointsNums.size() << endl;

	for (int i = 0; i < ClusterPointsNums.size(); i++)
	{
		cout <<"i:"<< ClusterPointsNums[i] <<endl;
	}

	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			int TempID = SliceDBScanClusters[i].pCluster->Clusters[j].ID;
			int TempIDIndex = VectorBaseInt.FindIndexInVector(ClusterIDs, TempID);
			
			//Find 
			if (TempIDIndex != -1)
			{
				for (int k = 0; k < SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.size(); k++)
				{
					int TempPointIndex = SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs[k];

					pCloud->points.push_back(InputCloud->points[TempPointIndex]);
				}
			}
		}
	}
}

//
void CDBScanTreeSkeleton::SlicesClusters(double SliceHeightValue, double RadiusValue, int MinPtsValue, 
	bool Show, int PointSize)
{
	Radius = RadiusValue;
	MinPts = MinPtsValue;

	ResetSliceDBScanClusters();
		
	HorizontalPartition.SetInputCloud(InputCloud);	
	HorizontalPartition.SetThickNess(SliceHeightValue);
	HorizontalPartition.PatitionSection();

	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		SliceCluster TempSliceCluster;
		TempSliceCluster.pCluster = new CDBScanCluster();
		TempSliceCluster.SectionPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
		HorizontalPartition.GetSectionPoints(i, TempSliceCluster.SectionPoints);
		TempSliceCluster.pCluster->SetInputCloud(TempSliceCluster.SectionPoints);
		SliceDBScanClusters.push_back(TempSliceCluster);
	}

	//此处并行计算 2020.02.18
	#pragma omp parallel for
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		SliceDBScanClusters[i].pCluster->RunCluster(RadiusValue, MinPtsValue, false);

		SliceDBScanClusters[i].pCluster->SetClusterColors();
		
		//计算每个簇的质心点
		SliceDBScanClusters[i].pCluster->CalcClusterParameters();

		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.clear();
			for (int k = 0; k < SliceDBScanClusters[i].pCluster->Clusters[j].Indexs.size(); k++)
			{
				int TempSectionIndex = SliceDBScanClusters[i].pCluster->Clusters[j].Indexs[k];
				int TempGlobalIndex = HorizontalPartition.SectionsVector[i].Indexs[TempSectionIndex];

				SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.push_back(TempGlobalIndex);
			}
		}
	}

	if (Show)
	{
		ShowSliceClusters(false, PointSize);
	}
}

void CDBScanTreeSkeleton::SkeletonGenerate(double SliceHeightValue, double RadiusValue, int MinPtsValue, bool IsUseConvexCentroid)
{
	SkeletonIsGenerate = false;
	UseConvexCentroid = IsUseConvexCentroid;
	SliceHeight = SliceHeightValue;
	if (SliceDBScanClusters.size() == 0)
		SlicesClusters(SliceHeightValue, RadiusValue, MinPtsValue);
	cout<<"Skeleton Generating, please waiting!"<<endl;

	OctreeCloudSearch->deleteTree();
	OctreeCloudSearch->setInputCloud(InputCloud);
	OctreeCloudSearch->addPointsFromInputCloud();

	RemoveRepairdCluster();

	#pragma omp parallel for
	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			SliceDBScanClusters[i].pCluster->Clusters[j].Label = "";
			SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex = -1;
			SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex = -1;
			SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ChildNodes.clear();
		}
	}

	//0 是最低的簇，其没有父节点，则不运行
	//#pragma omp parallel for  并行会出现错误
	for (int i = 1; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			//cout << "Find Parent for SliceIndex:" << i << ",ClusterIndex:"<< j <<endl;
			if (!GetParentSliceCluster(i, j, SliceHeightValue))
			{
				//cout << "SliceIndex:" << i << ",ClusterIndex:" << j << "未找到父节点！准备在上部运行!" << endl;
				if (!GetParentSliceCluster(i, j, SliceHeightValue * 2))
					GetParentSliceCluster(i, j, SliceHeightValue * 2, false);
			}
			cout << "Parent for SliceIndex:" << i << ",ClusterIndex:" << j << 
				" is SliceIndex:" << SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex <<
				"  ClusterIndex:" << SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex <<endl << endl;
		}
	}

	//不连续的树枝导致一个树枝在相邻Slice上的

	/* //效果不理想 2020.02.26
	for (int i = SliceDBScanClusters.size() - 1; i > 0; i--)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			if (SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex != -1)
				continue;

			GetParentSliceClusterByDirectionContinuity(i, j);
		}
	}
	*/

	//cout << "SkeletonPoints size:" << SkeletonPoints->points.size() << endl;
	//GetParentSliceClusterByNearestDis();
	SkeletonIsGenerate = true;
	//SetColors();
	cout << "Tree Skeleton has been finished!" << endl;
}

//标注Skeleton点分别属几级枝
void CDBScanTreeSkeleton::LableSkeleton()
{

}

void CDBScanTreeSkeleton::ShowSkeleton(bool ShowSkeletonPoint, bool ShowSkeleton, 
	int PointSize, string SkeletonPointsStr)
{
	if (Viewer == NULL) return;

	Viewer->removePointCloud(SkeletonPointsStr);
	Viewer->removeAllShapes();

	SkeletonPoints->points.clear();
		
	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{

		if (i >= 118 && i < 120)
			cout << "Slice Index:" << i << ",Cluster 数量:" << SliceDBScanClusters[i].pCluster->Clusters.size() << endl;

		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			if (!UseConvexCentroid)
				SkeletonPoints->points.push_back(SliceDBScanClusters[i].pCluster->Clusters[j].Centroid);
			else if (UseConvexCentroid)
				SkeletonPoints->points.push_back(SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid);

			if (SliceDBScanClusters[i].pCluster->Clusters[j].Label == "TempBranches")
				continue;

			int ParentSliceIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex;
			int ParentClusterIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex;

			if ((i >= 0) && ParentSliceIndex != -1 && ParentClusterIndex != -1)
			{
				//Viewer->removeShape(StringBase::IntToStr(i) + "_" + StringBase::IntToStr(j));
				//PointBase::ShowPointXYZRGBText(Viewer, SkeletonPoints, 
				//	StringBase::IntToStr(i) + "_" + StringBase::IntToStr(j), 1, 0, false, 
				//	StringBase::IntToStr(i) + "_" + StringBase::IntToStr(j));

				if (ShowSkeleton)
					Viewer->addLine(SliceDBScanClusters[ParentSliceIndex].pCluster->
						Clusters[ParentClusterIndex].ConvexCentroid,
						SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid, 0, 255, 0,
						StringBase::IntToStr(i) + "_" + StringBase::IntToStr(j));
			}
		}
	}

	if (ShowSkeletonPoint)
	{
		//PointsMove(SkeletonPoints, 50, 0, 0);
		Viewer->removePointCloud(SkeletonPointsStr);
		PointBase::ShowPointXYZRGB(Viewer, SkeletonPoints, SkeletonPointsStr, PointSize * 2);
		//PointBase::ShowPointXYZRGBText(Viewer, SkeletonPoints, SkeletonPointsStr + "_Point", 1);
	}
	//else
		//PointsMove(InputCloud, -300, 0, 0);

	/*
	cout << "88-89-90:" << GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints
	(SkeletonPoints->points[88], SkeletonPoints->points[89], SkeletonPoints->points[90])) << endl;

	cout << "89-90-91:" << GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints
	(SkeletonPoints->points[89], SkeletonPoints->points[90], SkeletonPoints->points[91])) << endl;

	cout << "89:" << SkeletonPoints->points[89] << endl;
	cout << "90:" << SkeletonPoints->points[90] << endl;
	cout << "91:" << SkeletonPoints->points[91] << endl;

	Viewer->addLine(SkeletonPoints->points[88],
		SkeletonPoints->points[89],0, 255, 0,
		StringBase::IntToStr(88) + "___" + StringBase::IntToStr(89));	
	Viewer->addLine(SkeletonPoints->points[89],
		SkeletonPoints->points[90], 0, 255, 0,
		StringBase::IntToStr(89) + "___" + StringBase::IntToStr(90));

	cout << "90-91-92:" << GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints
	(SkeletonPoints->points[90], SkeletonPoints->points[91], SkeletonPoints->points[92])) << endl;
	//*/

}

void CDBScanTreeSkeleton::ShowSliceClusters(bool ShowPoints, bool IsByID, int PointSize)
{
	if (Viewer == NULL)
		return;

	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		Viewer->removePointCloud(
			SliceClusterPointsStr + QString::number(i).toStdString());
		
		if (ShowPoints)
		{
			SliceDBScanClusters[i].pCluster->SetClusterColors(IsByID);

			SliceDBScanClusters[i].pCluster->ShowCluster(Viewer, PointSize,
					SliceClusterPointsStr + QString::number(i).toStdString());
		}
	}
}

void CDBScanTreeSkeleton::OutliersDetection(double SliceHeightValue, double RadiusValue, int MinPtsValue)
{
	OutliersIndexs.clear();

	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetThickNess(SliceHeightValue);
	HorizontalPartition.PatitionSection();

	OctreeCloudSearch->deleteTree();
	OctreeCloudSearch->setInputCloud(InputCloud);
	OctreeCloudSearch->addPointsFromInputCloud();

	#pragma omp parallel for
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		for (int j = 0; j < HorizontalPartition.SectionsVector[i].Indexs.size(); j++)
		{
			int GlobalIndex = HorizontalPartition.SectionsVector[i].Indexs[j];
			//cout<<"GlobalIndex:"<< GlobalIndex <<endl;
			//计算每一个点的领域
			vector<int> IndexS;
			vector<float> DisS;

			GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue, GlobalIndex, &IndexS, &DisS);

			#pragma omp critical
			{
				if (IndexS.size() < MinPtsValue)			
					OutliersIndexs.push_back(GlobalIndex);
			}
		}
	}	
}

void CDBScanTreeSkeleton::OutliersRemoval()
{
	if (OutliersIndexs.size() == 0) return;

	VectorBase<int> VectorBaseInt;
	VectorBaseInt.SortVector(OutliersIndexs, "Asc");

	//cout <<"OutliersIndexs.size():"<< OutliersIndexs.size()<< endl;
	for (int i = OutliersIndexs.size() - 1; i >= 0; i--)
	{
		cout << "OutliersIndexs[i]:"<<i <<","<< OutliersIndexs[i] << endl;
		InputCloud->points.erase(InputCloud->points.begin() + OutliersIndexs[i]);
	}

	OutliersIndexs.clear();
	cout << "OutliersIndexs.size():" << OutliersIndexs.size() << endl;
}

void CDBScanTreeSkeleton::NormalComputation()
{
	PointGeometry.SetInputCloud(InputCloud);
	PointGeometry.CalcOptimalNormal();
}

//移除在修复过程中产生的Cluster 2020.04.09	 
void CDBScanTreeSkeleton::RemoveRepairdCluster()
{
	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = SliceDBScanClusters[i].pCluster->Clusters.size() - 1;
			j >= 0; j--)
		{
			if (SliceDBScanClusters[i].pCluster->Clusters[j].Label == "TempBranches")
				SliceDBScanClusters[i].pCluster->Clusters.erase(
					SliceDBScanClusters[i].pCluster->Clusters.begin() + j);
		}
	}
}
//根据树干的连续性获取树干的连续部分 2020.04.07
//AllowAngle 为判断 聚类点 连续性的最小角度值
void CDBScanTreeSkeleton::FindSuccessivePart(int SuccessiveNum, double AllowAngle)
{
	cout << "SliceDBScanClusters Size " << SliceDBScanClusters.size() << endl;
	//2020.04.07 首先计算可信区域
	for (int i = 2; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			//if (i == 236 && j == 2)
			//	cout <<"GlobalIndexs.size:"<<SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.size()<< endl;

			//cout << "第 " << i << " 个Slice的第 " << j << " 个聚类点正在处理！" << endl;
			int ParentSliceIndex, ParentClusterIndex;
			int GrandpaSliceIndex, GrandpaClusterIndex;

			if (SliceDBScanClusters[i].pCluster->Clusters[j].Label == "TempBranches" 
				|| SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.size() == 0)
				continue;

			//设置当前的颜色，不会影响此段的目的，设置其父节点的颜色
			SliceDBScanClusters[i].pCluster->Clusters[j].Centroid.rgba = ColorBase::BlueColor;
			SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid.rgba = ColorBase::BlueColor;

			ParentSliceIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex;
			ParentClusterIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex;

			if (ParentSliceIndex == -1 || ParentClusterIndex == -1)
				continue;

			GrandpaSliceIndex = SliceDBScanClusters[ParentSliceIndex].pCluster->
				Clusters[ParentClusterIndex].NodeRoles.ParentNode.SliceIndex;
			GrandpaClusterIndex = SliceDBScanClusters[ParentSliceIndex].pCluster->
				Clusters[ParentClusterIndex].NodeRoles.ParentNode.ClusterIndex;

			if (GrandpaSliceIndex == -1 || GrandpaClusterIndex == -1)
				continue;

			pcl::PointXYZRGB CurrentPoint, ParentPoint, GrandpaPoint;

			if (UseConvexCentroid)
			{
				CurrentPoint = SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid;
				ParentPoint = SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].ConvexCentroid;
				GrandpaPoint = SliceDBScanClusters[GrandpaSliceIndex].pCluster->Clusters[GrandpaClusterIndex].ConvexCentroid;
			}
			else
			{
				CurrentPoint = SliceDBScanClusters[i].pCluster->Clusters[j].Centroid;
				ParentPoint = SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].Centroid;
				GrandpaPoint = SliceDBScanClusters[GrandpaSliceIndex].pCluster->Clusters[GrandpaClusterIndex].Centroid;
			}

			double TempAngle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints
				(CurrentPoint, ParentPoint, GrandpaPoint));

			if (TempAngle < AllowAngle)
			{
				//cout << "第 " << ParentSliceIndex << " 个Slice的第 " << ParentClusterIndex << " 个聚类点不符合要求！" << endl;
				//SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].ConvexCentroid.rgba = ColorBase::RedColor;
				//SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].Centroid.rgba = ColorBase::RedColor;
				//SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].Label = "NeedFixed";

				//cout << "第 " << i << " 个Slice的第 " << j << " 个聚类点不符合要求！" << endl;
				SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid.rgba = ColorBase::RedColor;
				SliceDBScanClusters[i].pCluster->Clusters[j].Centroid.rgba = ColorBase::RedColor;
				SliceDBScanClusters[i].pCluster->Clusters[j].Label = "NeedFixed";
			}
		}
	}

	///* //若 连续的 SuccessiveNum 都是可信区域，则最后一个点的法向即为 连续可信区域点集 构成的最大方向
	for (int i = SuccessiveNum; i < SliceDBScanClusters.size(); i++)
	{		
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			/*
			int UpCount = SuccessiveNum - 1;
			int ParentSliceIndex = -1, ParentClusterIndex = -1;

			ParentSliceIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex;
			ParentClusterIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex;

			if (ParentSliceIndex == -1 && ParentClusterIndex == -1)
				continue;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

			if (UseConvexCentroid)
				TempPoints->points.push_back(SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid);
			else
				TempPoints->points.push_back(SliceDBScanClusters[i].pCluster->Clusters[j].Centroid);

			//向上逐层获得父节点
			while (UpCount > 0 && ParentSliceIndex != -1 && ParentClusterIndex != -1)
			{
				//中间有不是连续的点
				if (SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].ConvexCentroid.rgba
					!= ColorBase::BlueColor)
					break;

				if (UseConvexCentroid)
					TempPoints->points.push_back(
						SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].ConvexCentroid);
				else
					TempPoints->points.push_back(
						SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].Centroid);

				int TempIndex = ParentSliceIndex;
				ParentSliceIndex = SliceDBScanClusters[TempIndex].pCluster->Clusters[ParentClusterIndex].NodeRoles.ParentNode.SliceIndex;
				ParentClusterIndex = SliceDBScanClusters[TempIndex].pCluster->Clusters[ParentClusterIndex].NodeRoles.ParentNode.ClusterIndex;
				UpCount--;

				if (ParentSliceIndex == -1 || ParentClusterIndex == -1)
					break;
			}

			//获取的总体个数较少
			if (TempPoints->points.size() < SuccessiveNum)
				continue;
			
			SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection =	GeometryBase::GetMaxDirectionVector(TempPoints);
			//*/

			pcl::PointXYZRGB Direction;
			bool IsHave = CalcCurDirection(SuccessiveNum, i, j, Direction);
			if (IsHave)
				SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection = Direction;
			//cout << "第 " << ParentSliceIndex << " 个Slice的第 " << ParentClusterIndex << " 个点的方向为："
			//	<< SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection << endl;

			////显示已有方向点的方向
			//PointBase::ShowDirection(Viewer, SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid,
			//	SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection, 3);
		}
	}

	/*
	for (int i = SuccessiveNum + 1; i < SliceDBScanClusters.size(); i++)
	{
		//计算每一个点的生长方向，如果 SuccessiveNum 都在一个分支上，则按照SuccessiveNum个数目计算，否则
		//按照少的个数计算
		//应该分别计算每个分支的生长方向，同时确保不会受到树杈的影响，即先计算相近区域的进行计算。
		pcl::PointXYZRGB CurrentDirection;		
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			//cout<<"正在检查第 "<<i<<" 个Slice的第 "<<j<<" 个聚类是否连续"<<endl;

			int ParentSliceIndex = -1, ParentClusterIndex = -1;
			int UpCount = SuccessiveNum - 1;
			ParentSliceIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex;
			ParentClusterIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex;

			if (ParentSliceIndex == -1 && ParentClusterIndex == -1)
				continue;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

			SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid.rgba = ColorBase::RedColor;
			SliceDBScanClusters[i].pCluster->Clusters[j].Centroid.rgba = ColorBase::RedColor;

			if (UseConvexCentroid)
				TempPoints->points.push_back(SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid);
			else
				TempPoints->points.push_back(SliceDBScanClusters[i].pCluster->Clusters[j].Centroid);

			//向上逐层获得父节点
			while (UpCount > 0 && ParentSliceIndex != -1 && ParentClusterIndex != -1)
			{
				if (UseConvexCentroid)
					TempPoints->points.push_back(
						SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].ConvexCentroid);
				else
					TempPoints->points.push_back(
						SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].Centroid);

				int TempIndex = ParentSliceIndex;
				ParentSliceIndex = SliceDBScanClusters[TempIndex].pCluster->Clusters[ParentClusterIndex].NodeRoles.ParentNode.SliceIndex;
				ParentClusterIndex = SliceDBScanClusters[TempIndex].pCluster->Clusters[ParentClusterIndex].NodeRoles.ParentNode.ClusterIndex;
				UpCount--;

				if (ParentSliceIndex == -1 || ParentClusterIndex == -1)
					break;
				//if (i > 5 && (ParentSliceIndex == 0 || ParentClusterIndex == 0))
				//	break;
			}
			//获取的总体个数较少
			if (TempPoints->points.size() < SuccessiveNum)
				continue;

			//再次直接获得上一级的节点信息
			ParentSliceIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex;
			ParentClusterIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex;

			if (ParentSliceIndex == -1 || ParentClusterIndex == -1)
				continue;

			pcl::PointXYZRGB ParentDirection =
				SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].CurDirection;

			if (ParentDirection.x != 0 && ParentDirection.y != 0 && ParentDirection.z != 0) //上级的方向已经计算
			{
				double CurrentAngle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
					TempPoints->points[0], TempPoints->points[1], TempPoints->points[2]));

				//cout << "第 " << i << " 个Slice的第 " << j << " 个聚类CurrentAngle:"<< CurrentAngle << endl;

				if (CurrentAngle >= AllowAngle)
				{
					SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection =
						GeometryBase::GetMaxDirectionVector(TempPoints);

					cout << "第 " << i << " 个Slice的第 " << j << " 个聚类是连续状态" << endl;
					SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid.rgba = ColorBase::YellowColor;
					SliceDBScanClusters[i].pCluster->Clusters[j].Centroid.rgba = ColorBase::YellowColor;
				}
			}
			else
			{
				bool IsCountinue = true;
				for (int k = 0; k < TempPoints->points.size(); k++)
				{
					if (k + 2 < TempPoints->points.size())
					{
						double CurrentAngle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
							TempPoints->points[k], TempPoints->points[k + 1], TempPoints->points[k + 2]));

						//cout << "第 " << i << " 个Slice的第 " << j << " 个聚类,K"<<k<<",CurrentAngle:" << CurrentAngle << endl;

						if (CurrentAngle < AllowAngle)
						{
							IsCountinue = false;
							break;
						}
					}
				}

				if (IsCountinue)	//都是连续状态
				{
					cout << "第 " << i << " 个Slice的第 " << j << " 个聚类是连续状态" << endl;
					SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection =
						GeometryBase::GetMaxDirectionVector(TempPoints);
					SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid.rgba = ColorBase::YellowColor;
					SliceDBScanClusters[i].pCluster->Clusters[j].Centroid.rgba = ColorBase::YellowColor;
				}
			}
		}
	}
	//*/	
}

//计算当前位置处的生长方向 2020.04.09
bool CDBScanTreeSkeleton::CalcCurDirection(int SuccessiveNum, int SliceIndex,
	int ClusterIndex, pcl::PointXYZRGB & CurDirection)
{
	int UpCount = SuccessiveNum - 1;
	int ParentSliceIndex = -1, ParentClusterIndex = -1;

	ParentSliceIndex = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.SliceIndex;
	ParentClusterIndex = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.ClusterIndex;

	if (ParentSliceIndex == -1 && ParentClusterIndex == -1)
		return false;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	if (UseConvexCentroid)
		TempPoints->points.push_back(SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid);
	else
		TempPoints->points.push_back(SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid);

	//向上逐层获得父节点
	while (UpCount > 0 && ParentSliceIndex != -1 && ParentClusterIndex != -1)
	{
		//中间有不是连续的点
		if (SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].ConvexCentroid.rgba
			!= ColorBase::BlueColor)
			break;

		if (UseConvexCentroid)
			TempPoints->points.push_back(
				SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].ConvexCentroid);
		else
			TempPoints->points.push_back(
				SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].Centroid);

		int TempIndex = ParentSliceIndex;
		ParentSliceIndex = SliceDBScanClusters[TempIndex].pCluster->Clusters[ParentClusterIndex].NodeRoles.ParentNode.SliceIndex;
		ParentClusterIndex = SliceDBScanClusters[TempIndex].pCluster->Clusters[ParentClusterIndex].NodeRoles.ParentNode.ClusterIndex;
		UpCount--;

		if (ParentSliceIndex == -1 || ParentClusterIndex == -1)
			break;
	}

	//获取的总体个数较少
	if (TempPoints->points.size() < SuccessiveNum)
		return false;
	
	CurDirection = GeometryBase::GetMaxDirectionVector(TempPoints);
	return true;
}

//根据过滤掉的树杈点做聚类，并添加到之前的聚类容器中
void CDBScanTreeSkeleton::CreateNewClusterForSliceByBranchesPoints(int SliceIndex, int ClusterIndex,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BranchesPoints,
	vector<int> BranchesPointsIndexS)
{	
	SliceCluster TempSliceCluster;
	TempSliceCluster.pCluster = new CDBScanCluster();
	TempSliceCluster.SectionPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());	
	TempSliceCluster.SectionPoints->points.insert(TempSliceCluster.SectionPoints->points.end(),
		BranchesPoints->points.begin(), BranchesPoints->points.end());
	TempSliceCluster.pCluster->SetInputCloud(TempSliceCluster.SectionPoints);
	TempSliceCluster.pCluster->RunCluster(Radius, MinPts, false);

	TempSliceCluster.pCluster->SetClusterColors();

	//计算每个簇的质心点
	TempSliceCluster.pCluster->CalcClusterParameters();

	for (int j = 0; j < TempSliceCluster.pCluster->Clusters.size(); j++)
	{
		TempSliceCluster.pCluster->Clusters[j].GlobalIndexs.clear();
		for (int k = 0; k < TempSliceCluster.pCluster->Clusters[j].Indexs.size(); k++)
		{
			int TempSectionIndex = TempSliceCluster.pCluster->Clusters[j].Indexs[k];
			int TempGlobalIndex = BranchesPointsIndexS[TempSectionIndex];

			TempSliceCluster.pCluster->Clusters[j].GlobalIndexs.push_back(TempGlobalIndex);
		}
		//如果有新建Cluster的个数小于3，则合并到之前的聚类中
		if (TempSliceCluster.pCluster->Clusters[j].Indexs.size() <= 3)
		{
			SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.insert(
				SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.end(),
				TempSliceCluster.pCluster->Clusters[j].GlobalIndexs.begin(),
				TempSliceCluster.pCluster->Clusters[j].GlobalIndexs.end());
			continue;
		}

		TempSliceCluster.pCluster->Clusters[j].Centroid.rgba = ColorBase::YellowColor;
		TempSliceCluster.pCluster->Clusters[j].ConvexCentroid.rgba = ColorBase::YellowColor;		
		cout<<"SliceIndex:"<< SliceIndex<<",新增一个聚类，点的数量是:"
			<< TempSliceCluster.pCluster->Clusters[j].GlobalIndexs.size()<<endl;
		SliceDBScanClusters[SliceIndex].pCluster->Clusters.push_back(TempSliceCluster.pCluster->Clusters[j]);
	}

	PointBase::SetPointColor(BranchesPoints, ColorBase::RedColor);
	PointBase::ShowPointXYZRGB(Viewer, BranchesPoints, StringBase::ClockValue(), 2);

	//Cluster TempCluster;
	//TempCluster.GlobalIndexs.insert(TempCluster.GlobalIndexs.end(),
	//	BranchesPointsIndexS.begin(), BranchesPointsIndexS.end());

	//TempCluster.Label = "TempBranches";
	//PointBase::GetPointsCenterXYZ(BranchesPoints,
	//	TempCluster.Centroid.x,
	//	TempCluster.Centroid.y,
	//	TempCluster.Centroid.z);
	////TempCluster.Centroid.z = ZMin;
	//TempCluster.Centroid.rgba = ColorBase::BlueColor;

	//TempCluster.ConvexCentroid = TempCluster.Centroid;
	//if (BranchesPoints->points.size() > 2)
	//	TempCluster.ConvexCentroid =
	//	GeometryBase::GetConvexHullCentroidOfPointsInXY(BranchesPoints);

	////if (SliceIndex == 56)
	////	PointBase::SavePCDToFileName(TempBranchesClusterPoints, "I:\\Test.pcd");

	//TempCluster.ConvexCentroid.z = TempCluster.Centroid.z;
	//TempCluster.ConvexCentroid.rgba = ColorBase::YellowColor;
	//TempCluster.Centroid.rgba = ColorBase::YellowColor;

	////if (PointDis(TempCluster.ConvexCentroid,
	////	SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid) > Thick * 2)
	////{
	////	cout << "新生长聚类的几何中心点有问题，SliceIndex:" << SliceIndex << ", ClusterIndex:" << ClusterIndex << endl;
	////}

	//SliceDBScanClusters[SliceIndex].pCluster->Clusters.push_back(TempCluster);
	//移除重新计算质心点 并新建一个簇
}


////使用分割树枝凸包多边形分割树木，使用SliceIndex的ClusterIndex的父节点的信息 
////对 SliceIndex的ClusterIndex 进行凸包多边形分割 2020.04.08
void CDBScanTreeSkeleton::SegmentationByConvexPolygon(double Thick, int SuccessiveNum, 
	int SliceIndex, int ClusterIndex, double AllowDis)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ZonePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempClusterPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	vector<int> ZonePointsIndexs;
	vector<int> ConvexPolygonPointsIndexs;

	cout << "尝试修复，SliceIndex：" << SliceIndex << ", ClusterIndex:" << ClusterIndex << endl;

	//取SliceIndex的ClusterIndex 前后 SuccessiveNum /2 个相邻点作为点集获取 
	ZonePointsIndexs.insert(ZonePointsIndexs.end(),
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.begin(),
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.end());

	GetSliceClusterPoints(SliceIndex, ClusterIndex, TempClusterPoints);
	ZonePoints->points.insert(ZonePoints->points.end(), TempClusterPoints->points.begin(),
		TempClusterPoints->points.end());

	int TempParentSlice = SliceIndex;
	int TempParentCluster = ClusterIndex;
	
	//寻找父级Slice的点云
	//for (int i = 0; i < SuccessiveNum / 2; i++)
	for (int i = 0; i < 2; i++)
	{
		int TempSliceIndex = TempParentSlice;
		if (SliceDBScanClusters[TempSliceIndex].pCluster->Clusters[TempParentCluster].NodeRoles.ParentNode.SliceIndex == -1 ||
			SliceDBScanClusters[TempSliceIndex].pCluster->Clusters[TempParentCluster].NodeRoles.ParentNode.ClusterIndex == -1)
			break;

		TempParentSlice = SliceDBScanClusters[TempSliceIndex].pCluster->Clusters[TempParentCluster].NodeRoles.ParentNode.SliceIndex;
		TempParentCluster = SliceDBScanClusters[TempSliceIndex].pCluster->Clusters[TempParentCluster].NodeRoles.ParentNode.ClusterIndex;

		if (TempParentSlice == -1 || TempParentCluster == -1)
			break;

		ZonePointsIndexs.insert(ZonePointsIndexs.end(),
			SliceDBScanClusters[TempParentSlice].pCluster->Clusters[TempParentCluster].GlobalIndexs.begin(),
			SliceDBScanClusters[TempParentSlice].pCluster->Clusters[TempParentCluster].GlobalIndexs.end());

		GetSliceClusterPoints(TempParentSlice, TempParentCluster, TempClusterPoints);
		ZonePoints->points.insert(ZonePoints->points.end(), TempClusterPoints->points.begin(),
			TempClusterPoints->points.end());
	}

	//寻找子级Slice的点云
	vector<Role> ChildS;
	ChildS.insert(ChildS.end(),
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ChildNodes.begin(),
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ChildNodes.end());

	int StartIndex = 0;
	int EndIndex = ChildS.size();
	//for (int i = 0; i < SuccessiveNum / 2; i++)
	for (int i = 0; i < 2; i++)
	{		
		for (int j = StartIndex; j < EndIndex; j++)
		{
			if (j >= ChildS.size()) break;

			if (ChildS[j].SliceIndex == -1 || ChildS[j].ClusterIndex == -1)
				continue;
			
			//添加点的索引
			ZonePointsIndexs.insert(ZonePointsIndexs.end(),
				SliceDBScanClusters[ChildS[j].SliceIndex].pCluster->Clusters[ChildS[j].ClusterIndex].GlobalIndexs.begin(),
				SliceDBScanClusters[ChildS[j].SliceIndex].pCluster->Clusters[ChildS[j].ClusterIndex].GlobalIndexs.end());

			GetSliceClusterPoints(ChildS[j].SliceIndex, ChildS[j].ClusterIndex, TempClusterPoints);
			ZonePoints->points.insert(ZonePoints->points.end(), TempClusterPoints->points.begin(),
				TempClusterPoints->points.end());

			ChildS.insert(ChildS.end(),
				SliceDBScanClusters[ChildS[j].SliceIndex].pCluster->Clusters[ChildS[j].ClusterIndex].NodeRoles.ChildNodes.begin(),
				SliceDBScanClusters[ChildS[j].SliceIndex].pCluster->Clusters[ChildS[j].ClusterIndex].NodeRoles.ChildNodes.end());
		}
		StartIndex = EndIndex;
		EndIndex = ChildS.size();
	}

	//从上面获取的点中获取沿着生长方向的一个用户构建ConvexPolygonPoints
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexPolygonPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	int ParentSlice = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.SliceIndex;
	int ParentCluster = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.ClusterIndex;

	if (ParentSlice == -1 || ParentCluster == -1)
		return;

	if (SliceDBScanClusters[ParentSlice].pCluster->Clusters[ParentCluster].Label == "NeedFixed")
	{
		cout <<"父级数据需要修复，ParentSlice："<< ParentSlice <<", ParentCluster:"<< ParentCluster << endl;
		//int M;
		//cin >>M;
		return;
	}	
	pcl::PointXYZRGB CurGrowth = SliceDBScanClusters[ParentSlice].pCluster->Clusters[ParentCluster].CurDirection;
	if (CurGrowth.x == 0 || CurGrowth.y == 0 || CurGrowth.z == 0)
	{
		pcl::PointXYZRGB Direction;
		bool IsHave = CalcCurDirection(SuccessiveNum, SliceIndex, ClusterIndex, Direction);
		if (IsHave)
		{
			SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].CurDirection = Direction;
			SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.rgba = ColorBase::BlueColor;
			SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Label = "";
		}
		else
		{
			cout << "尝试修复，SliceIndex：" << SliceIndex << ", ClusterIndex:" << ClusterIndex
				<< " 时未能正确计算该位置处的生长方向"<< endl;
		}
	}

	//if (SliceIndex == 54)
	//	cout << "CurGrowth:"<< CurGrowth << endl;

	pcl::PointXYZRGB UpDirection;
	UpDirection.x = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.x -
		SliceDBScanClusters[ParentSlice].pCluster->Clusters[ParentCluster].ConvexCentroid.x;
	UpDirection.y = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.y -
		SliceDBScanClusters[ParentSlice].pCluster->Clusters[ParentCluster].ConvexCentroid.y;
	UpDirection.z = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.z -
		SliceDBScanClusters[ParentSlice].pCluster->Clusters[ParentCluster].ConvexCentroid.z;

	//生长方向如果和质心点间的方向夹角大于90度，则调整生长方向
	double TempAnlge = GeometryBase::AngleOfTwoVector(CurGrowth, UpDirection);
	if (TempAnlge > M_PI_2)
		CurGrowth.x = -CurGrowth.x, CurGrowth.y = -CurGrowth.y, CurGrowth.z = -CurGrowth.z;
	
	pcl::PointXYZRGB BottomPoint = SliceDBScanClusters[TempParentSlice].pCluster->
		Clusters[TempParentCluster].ConvexCentroid;
	pcl::PointXYZRGB TopPoint = GeometryBase::GetPointAlongLine(CurGrowth, BottomPoint, Thick);	

	GeometryBase::GetPointsBetweenTwoPlanes(ZonePoints, CurGrowth.x, CurGrowth.y, CurGrowth.z,
		-(CurGrowth.x * BottomPoint.x + CurGrowth.y * BottomPoint.y + CurGrowth.z * BottomPoint.z),
		CurGrowth.x, CurGrowth.y, CurGrowth.z,
		-(CurGrowth.x * TopPoint.x + CurGrowth.y * TopPoint.y + CurGrowth.z * TopPoint.z),
		ConvexPolygonPoints, 1, true, false, false, &ConvexPolygonPointsIndexs);

	//cout<<"ConvexPolygonPoints Points Count:"<< ConvexPolygonPoints->points.size() <<endl;
	if (ConvexPolygonPoints->points.size() <= 0)
	{
		cout << "尝试修复，SliceIndex：" << SliceIndex << ", ClusterIndex:" << ClusterIndex 
			<<" 时竟然没有找到凸包点集！此时的生长方向是："<< CurGrowth<< endl;
		return;
	}
	//将 ZonePoints 替换为当前Slice当前Cluster的点集
	GetSliceClusterPoints(SliceIndex, ClusterIndex, ZonePoints);
	cout << "ZonePoints Points Count:" << ZonePoints->points.size()  <<", "
		<< SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size() << endl;
	if (ZonePoints->points.size() == 0) return;

	vector<int> TempBranchesIndexs = SegmentationPointsByConvexPolygon(ConvexPolygonPoints,
			ZonePoints, SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs, 
			CurGrowth, BottomPoint, AllowDis);
	//如果没有树枝节点，说明之前受其他地方影响导致此处也受影响
	if (TempBranchesIndexs.size() <= 0)
	{
		pcl::PointXYZRGB Direction;
		bool IsHave = CalcCurDirection(SuccessiveNum, SliceIndex, ClusterIndex, Direction);
		if (IsHave)
		{
			SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].CurDirection = Direction;
			SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.rgba = ColorBase::BlueColor;
			SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Label = "";
		}	

		return;
	}

	//移除重新计算质心点 	
	VectorBase<int> VectorBaseInt;
	//cout<<"TempBranches Points Count:"<< TempBranchesIndexs.size() <<endl;

	vector<int> TempSliceIndexs;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempSliceClusterPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempBranchesClusterPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	//cout << "GlobalIndexs Count:" << SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size() << endl;

	for (int i = 0; i < SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size(); i++)
	{
		int TempIndex = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs[i];

		//当前分区索引不在移除的索引容器中，则需要保留位原始索引
		int ResultIndex = VectorBaseInt.FindIndexInVector(TempBranchesIndexs, TempIndex);
		if (ResultIndex >= 0)
			TempBranchesClusterPoints->points.push_back(InputCloud->points[TempIndex]);
		else
		{
			TempSliceIndexs.push_back(TempIndex);
			TempSliceClusterPoints->points.push_back(InputCloud->points[TempIndex]);
		}	
	}

	//if (TempBranchesClusterPoints->points.size() != TempBranchesIndexs.size())
	//{
	//	cout<<"SliceIndex:"<< SliceIndex <<", ClusterIndex:"<< ClusterIndex <<"出现错误!"<<endl;
	//	cout << "SliceIndex,Size:" << SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size()
	//		<< ", TempBranchesIndexs,size:" << TempBranchesIndexs.size() << ", BranchesClusterPoints，Size:" 
	//		<< TempBranchesClusterPoints->points.size() <<",TempSliceIndexs:"<< TempSliceIndexs.size()<<endl;
	//}

	//cout << "TempBranchesClusterPoints Count:" << TempBranchesClusterPoints->points.size() << endl;

	//cout << "TempSliceClusterPoints Points Count:" << TempSliceClusterPoints->points.size() << endl;	

	SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.clear();
	SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.insert(
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.end(),
		TempSliceIndexs.begin(), TempSliceIndexs.end());

	//cout << "Original Centroid" << SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid <<endl;
	//cout << "Original ConvexCentroid" << SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid << endl;
	double ZMin = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid.z;
	PointBase::GetPointsCenterXYZ(TempSliceClusterPoints,
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid.x, 
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid.y, 
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid.z);
	SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid.rgba = ColorBase::BlueColor;

	SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid.z = ZMin;

	//cout << "Centroid" << SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Centroid << endl;
	//2020.02.14 如果大于等于3个点
	if (TempSliceClusterPoints->points.size() > 2)
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid =
		GeometryBase::GetConvexHullCentroidOfPointsInXY(TempSliceClusterPoints);

	SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.rgba = ColorBase::BlueColor;
	SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.z = ZMin;
	SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Label = "";	

	pcl::PointXYZRGB Direction;
	bool IsHave = CalcCurDirection(SuccessiveNum, SliceIndex, ClusterIndex, Direction);	
	if (IsHave)
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].CurDirection = Direction;

	//cout << "ConvexCentroid" << SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid << endl;

	//并新建一个簇
	//cout << "TempBranchesIndexs Count:" << TempBranchesIndexs.size() << endl;
	CreateNewClusterForSliceByBranchesPoints(SliceIndex, ClusterIndex, TempBranchesClusterPoints, TempBranchesIndexs);	

	//cout<<"Dis:"<<PointDis(BottomPoint, TopPoint);

	//cout << "ZonePoints Size:" << ZonePoints->points.size() << endl;
	//cout << "ZonePointsIndexs Size:" << ZonePointsIndexs.size() << endl;

	//PointBase::ShowPlane(Viewer, CurGrowth.x, CurGrowth.y, CurGrowth.z, TopPoint, "Top");
	//PointBase::ShowPlane(Viewer, CurGrowth.x, CurGrowth.y, CurGrowth.z, BottomPoint, "Bottom");

	//PointBase::SetPointColor(ZonePoints, ColorBase::BlueColor);
	//PointBase::ShowPointXYZRGB(Viewer, ZonePoints, "ZonePoints", 2);

	//PointBase::SetPointColor(ConvexPolygonPoints, ColorBase::RedColor);
	//PointBase::ShowPointXYZRGB(Viewer, ConvexPolygonPoints, "ConvexPolygonPoints", 2);
}

//使用凸包多边形分割点集，AllowDis 为凸包多边形的膨胀距离, 返回不在凸包多边形内的点集的索引 2020.04.08
vector<int> CDBScanTreeSkeleton::SegmentationPointsByConvexPolygon(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexPolygon3DPoints,	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ZonePoints, 
	vector<int> ZonePointsIndexs, 
	pcl::PointXYZRGB Direction,
	pcl::PointXYZRGB BottomPoint, double AllowDis)
{
	vector<int> ResultIndexS;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexPolygon2DPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexPolygon2DHullPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr LowerCovexPologonCurve(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr UpperCovexPologonCurve(new pcl::PointCloud<pcl::PointXYZRGB>());

	GeometryBase::ProjectPointsToPlane(ConvexPolygon3DPoints, ConvexPolygon2DPoints,
		Direction, BottomPoint);

	GeometryBase::GetCentroidAndConvexHullOfPointsInSpaces(ConvexPolygon2DPoints,
		ConvexPolygon2DHullPoints, Direction, AllowDis);

	CBezierSubsectionG2 BezierSubsectionG2;
	BezierSubsectionG2.SetInputs(ConvexPolygon2DHullPoints);
	BezierSubsectionG2.ResolveControlPoints(BezierMiu);
	BezierSubsectionG2.DrawBezierCurve(LowerCovexPologonCurve, 360);

	pcl::PointXYZRGB MovePoint = GeometryBase::GetPointAlongLine(Direction,
		LowerCovexPologonCurve->points[0], 2);

	PointBase::PointCopy(LowerCovexPologonCurve, UpperCovexPologonCurve);
	PointsMove(UpperCovexPologonCurve, MovePoint.x - LowerCovexPologonCurve->points[0].x,
		MovePoint.y - LowerCovexPologonCurve->points[0].y,
		MovePoint.z - LowerCovexPologonCurve->points[0].z);

	//PointBase::SetPointColor(UpperCovexPologonCurve, ColorBase::YellowColor);
	//PointBase::ShowPointXYZRGB(Viewer, UpperCovexPologonCurve, "UpperCovexPologonCurve", 2);

	//PointBase::SetPointColor(LowerCovexPologonCurve, ColorBase::YellowColor);
	//PointBase::ShowPointXYZRGB(Viewer, LowerCovexPologonCurve, "LowerCovexPologonCurve", 2);

	vector<PlaneCoeff> CoeffValues;	
	for (int i = 0; i < UpperCovexPologonCurve->points.size(); i++)
	{
		PlaneCoeff TempPlaneCoeff;

		int NextPointIndex = (i + 1) % UpperCovexPologonCurve->points.size();
		double a, b, c, d;
		GeometryBase::GetPlaneByThreePoints(LowerCovexPologonCurve->points[i],
			UpperCovexPologonCurve->points[i],
			UpperCovexPologonCurve->points[NextPointIndex], a, b, c, d);

		TempPlaneCoeff.a = a, TempPlaneCoeff.b = b, TempPlaneCoeff.c = c, TempPlaneCoeff.d = d;
		TempPlaneCoeff.PlaneValue = a * BottomPoint.x + b * BottomPoint.y + c * BottomPoint.z + d;

		CoeffValues.push_back(TempPlaneCoeff);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ResultPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	for (int i = 0; i < ZonePoints->points.size(); i++)
	{
		for (int j = 0; j < UpperCovexPologonCurve->points.size(); j++)
		{
			double TempValue = CoeffValues[j].a * InputCloud->points[ZonePointsIndexs[i]].x
				+ CoeffValues[j].b * InputCloud->points[ZonePointsIndexs[i]].y
				+ CoeffValues[j].c * InputCloud->points[ZonePointsIndexs[i]].z
				+ CoeffValues[j].d;

			if (TempValue * CoeffValues[j].PlaneValue < 0) //位于平面两侧
			{
				//InputCloud->points[ZonePointsIndexs[i]].rgba = ColorBase::RedColor;
				ResultPoints->points.push_back(InputCloud->points[ZonePointsIndexs[i]]);
				ResultIndexS.push_back(ZonePointsIndexs[i]);
				break;
			}
		}
	}

	//cout<<"ConvexPolygon3DPoints Size:"<< ConvexPolygon3DPoints->points.size() <<endl;
	//cout << "ConvexPolygon2DPoints Size:" << ConvexPolygon2DPoints->points.size() << endl;
	//PointBase::SetPointColor(ConvexPolygon2DPoints, ColorBase::GreenColor);
	//PointBase::ShowPointXYZRGB(Viewer, ConvexPolygon2DPoints, "ConvexPolygon2DPoints", 2);
		
	PointBase::SetPointColor(ZonePoints, ColorBase::GreenColor);
	Viewer->removePointCloud("ZonePoints");
	PointBase::ShowPointXYZRGB(Viewer, ZonePoints, "ZonePoints", 2);
		
	PointBase::SetPointColor(ResultPoints, ColorBase::RedColor);	
	Viewer->removePointCloud("ResultPoints");
	PointBase::ShowPointXYZRGB(Viewer, ResultPoints, "ResultPoints", 2);
	
	Viewer->updateCamera();

	return ResultIndexS;
}

//获取SliceIndex中ClusterIndex的点集
void CDBScanTreeSkeleton::GetSliceClusterPoints(int SliceIndex, int ClusterIndex,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SliceClusterPoints)
{
	SliceClusterPoints->points.clear();

	for (int i = 0; i < SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size(); i++)
	{
		SliceClusterPoints->points.push_back(InputCloud->points[
			SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs[i]]);
	}
}

//移除分支处的连接关系 2020.04.11
void CDBScanTreeSkeleton::RemoveBrancheRoles()
{
	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			if (SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ChildNodes.size() > 1)
			{
				for (int k = 0; k < SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ChildNodes.size(); k++)
				{
					int ChildSliceIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ChildNodes[k].SliceIndex;
					int ChildClusterIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ChildNodes[k].ClusterIndex;

					SliceDBScanClusters[ChildSliceIndex].pCluster->Clusters[ChildClusterIndex].NodeRoles.ParentNode.SliceIndex = -1;
					SliceDBScanClusters[ChildSliceIndex].pCluster->Clusters[ChildClusterIndex].NodeRoles.ParentNode.ClusterIndex = -1;
				}
				SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ChildNodes.clear();
			}
		}
	}
}


//根据树干的连续性修改树干骨架中的点，并明确每个点所在树干的层次关系 2020.02.26
void CDBScanTreeSkeleton::SkeletonPointRepair(double Thick,
	int SuccessiveNum, double AllowAngle, double AllowDis)
{
	Viewer->removePointCloud("ConvexPolygon2DPoints");
	Viewer->removePointCloud("ResultPoints");

	RemoveRepairdCluster();

	cout << "FindSuccessivePart starting" << endl;

	FindSuccessivePart(SuccessiveNum, AllowAngle);

	cout<<"FindSuccessivePart has been done"<<endl;

	int k;

	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			if (SliceDBScanClusters[i].pCluster->Clusters[j].Label == "NeedFixed")
			{
				int ParentSliceIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex;
				int ParentClusterIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex;

				if (ParentSliceIndex != -1 && ParentClusterIndex != -1)
					SegmentationByConvexPolygon(Thick, SuccessiveNum, ParentSliceIndex, 
							ParentClusterIndex, AllowDis);
				
				SegmentationByConvexPolygon(Thick, SuccessiveNum, i, j, AllowDis);
				
				//break;
				//cin >> k;
			}
			else if (SliceDBScanClusters[i].pCluster->Clusters[j].Label != "TempBranches")
			{
				pcl::PointXYZRGB CurDirection = SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection;

				if (CurDirection.x == 0 && CurDirection.y == 0 && CurDirection.z == 0)
				{
					bool IsHave = CalcCurDirection(SuccessiveNum, i, j, CurDirection);
					if (IsHave)
						SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection = CurDirection;
					else 
						cout << "不需要修复但该处方向无法计算 I:" << i << ", J:" << j << endl;
				}
			}			
		}
		//if (i == 56)
		//	break;
	}
	FindSuccessivePart(SuccessiveNum, AllowAngle);
	RemoveBrancheRoles();
}