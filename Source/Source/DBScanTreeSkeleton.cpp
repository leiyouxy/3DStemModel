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

//��ǰ����ʹ�õĺ���  //���ݵ㼯�������Ի�ȡ���ڵ� 2020.04.11
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
		//SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].Indexs ֻ��
		// һ��HorizontalPartition�е㼯������������Ҫת����ȫ��������
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
		cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:" << ClusterIndex << "δ�ҵ����ڵ㣡" << endl;
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

	//������ڵ�ǰ������û�з��֣���������һ������ĵ�
	//int FindPointIndex = -1;
	VectorBase<int> VectorBaseInt;

	//����������ͬ�����һЩ�������
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
		cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:" << ClusterIndex << "δ�ҵ����ڵ㣡" << endl;
		return false;
	}

	vector<int> ParentSliceIndexs, ParentClusterIndexs;
	//find ParentSliceIndexs and ParentClusterIndexs indexs for the UNBelongs Points
	for (int k = 0; k < FindPointIndexs.size(); k++)
	{
		int FindPointIndex = FindPointIndexs[k];

		//for (int i = SliceIndex - 1; i >= 0; i--)
		for (int i = SliceIndex - 1; i >= SliceIndex - 1; i--) //������һ��Ѱ�Ҽ���
		{
			for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
			{
				if (VectorBaseInt.FindIndexInVector(SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs,
					FindPointIndex) != -1
					//ȷ�����ڵ����ڴصĸ߶�С�ڵ��ڵ�ǰ�صĸ߶�
					&& ((SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid.z
						<= SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.z && Bottom)
						//|| (SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid.z
						//	>= SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].ConvexCentroid.z && !Bottom)))
						|| (!Bottom)))
				{
			
					//����������Ѿ����ڣ������
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
		//cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:" << ClusterIndex << "δ�ҵ����ڵ㣡" << endl;
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

		//������� �Ƕ����
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
			//�����С����ĽǶȺ��ʾ���Ϊ���ڵ�
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
			//������Ƕȶ�Ӧ�ľ���С��2���� SliceHeight
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
			//����ѡ��ֻ�ø��ݾ���ѡ��
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
					<< ClusterIndex << "δ�ҵ����ڵ㣡Bottom:"<<Bottom << endl;
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
		<< ClusterIndex << "Ѱ�Ҹ��ڵ�By DirectionContinuity" << endl;

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
		<< ClusterIndex << "Ѱ�Ҹ��ڵ�By OptimalChildIndex:"<< OptimalChildIndex << endl;

	if (OptimalChildIndex == -1) return false;

	int ParentSliceIndex = -1, ParentClusterIndex = -1;

	MaxAngle = EPSM3;
	//find the parent slice in the serveral successive slices, 
	//that the height of the successive slices is not bigger than quarter of the tree height	
	for (int i = SliceIndex - 1; i >= SliceIndex - SliceDBScanClusters.size() / 4 && i >= 0; i--)
	{
		cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
			<< ClusterIndex << "Ѱ�Ҹ��ڵ�By i:" << i << endl;

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
			//	<< ClusterIndex << "Ѱ�Ҹ��ڵ�By OptimalChildNode:" << OptimalChildNode << endl;

			//cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
			//	<< ClusterIndex << "Ѱ�Ҹ��ڵ�By CurrentNode:" << CurrentNode << endl;

			//cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
			//	<< ClusterIndex << "Ѱ�Ҹ��ڵ�By ParentNode:" << ParentNode << endl;

			//cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
			//	<< ClusterIndex << "Ѱ�Ҹ��ڵ�By TempAnlge:" << TempAnlge << endl;

			if (TempAnlge > MaxAngle)
			{
				MaxAngle = TempAnlge;
				ParentSliceIndex = i;
				ParentClusterIndex = j;
			}
		}
	}

	cout << "SliceIndex:" << SliceIndex << ",ClusterIndex:"
		<< ClusterIndex << "Ѱ�Ҹ��ڵ�By OptimalChildIndex, MaxAngle:" << MaxAngle << endl;

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
	//�� 1 ��ʼ����͵� 0 �� û�и��ڵ�
	for (int i = 1; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			//��Ȼû�ҵ�Parent�ľ��� ������ĵ�
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

				//����ĵ���Ǽ��еĵ�ƥ��
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

//���໥Slice�ϸ���DBScan����ϲ���
void CDBScanTreeSkeleton::MainTreeBranchesDetection(double RadiusValue, int MinPtsValue)
{
	//����
	OctreeCloudSearch->deleteTree();
	OctreeCloudSearch->setInputCloud(InputCloud);
	OctreeCloudSearch->addPointsFromInputCloud();

	//cout<<"SliceDBScanClusters.size():"<< SliceDBScanClusters.size() <<endl;
	#pragma omp parallel for //�ȸ�ÿ�������ID��ֵ�������ֲ�ͬ�ľ��࣬Ȼ��úϲ�
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

			//��ȡһЩ���²�Slice����Ͻ��ĵ�
			//for (int k = 0; k < SliceDBScanClusters[i].pCluster->Clusters[j].Indexs.size(); k++)
			//{
			//	int TempPointIndex = SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs[k];

			//	//�����ֵСһ���뾶��ֵ
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
			
			//������һ����ȡ��Ľ��ڣ�������ڵ���һ��Core Point ��λ�����ϲ�ĳһ���ص�Core Point�������У��������ͬһ����
			//for (int k = 0; k < MinZIndexS.size(); k++)
			for (int k = 0; k < SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.size(); k++)
			{
				int UpperPointIndex = SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs[k];
				vector<int> NearestIndexs;
				vector<float> NearestDiss;
				GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue,
					//MinZIndexS[k], &NearestIndexs, &NearestDiss);
					UpperPointIndex, &NearestIndexs, &NearestDiss);
				
				//���Ǻ��ĵ�
				if (NearestIndexs.size() < MinPtsValue)
					continue;

				for (int ii = 0; ii < NearestIndexs.size(); ii++)
				{
					VectorBase<int> VectorBaseInt;
					//�������㼯�Ƿ�����һSlice��ĳһ�����У�����ǣ�˵��������ͬһ����
					for (int kk = 0; kk < SliceDBScanClusters[i - 1].pCluster->Clusters.size(); kk++)
					{
						Find = VectorBaseInt.FindIndexInVector(
							SliceDBScanClusters[i - 1].pCluster->Clusters[kk].GlobalIndexs, NearestIndexs[ii]);

						if (Find > 0)
						{
							//�������ڴأ��Ƿ���Core Point��������� UpperPointIndex �� 
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
			
			if (TempIDIndex != -1)	//����ҵ����ۼ�			
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

	// ɾ�� Top ����ĵ�
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

	//�˴����м��� 2020.02.18
	#pragma omp parallel for
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		SliceDBScanClusters[i].pCluster->RunCluster(RadiusValue, MinPtsValue, false);

		SliceDBScanClusters[i].pCluster->SetClusterColors();
		
		//����ÿ���ص����ĵ�
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

	//0 ����͵Ĵأ���û�и��ڵ㣬������
	//#pragma omp parallel for  ���л���ִ���
	for (int i = 1; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			//cout << "Find Parent for SliceIndex:" << i << ",ClusterIndex:"<< j <<endl;
			if (!GetParentSliceCluster(i, j, SliceHeightValue))
			{
				//cout << "SliceIndex:" << i << ",ClusterIndex:" << j << "δ�ҵ����ڵ㣡׼�����ϲ�����!" << endl;
				if (!GetParentSliceCluster(i, j, SliceHeightValue * 2))
					GetParentSliceCluster(i, j, SliceHeightValue * 2, false);
			}
			cout << "Parent for SliceIndex:" << i << ",ClusterIndex:" << j << 
				" is SliceIndex:" << SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex <<
				"  ClusterIndex:" << SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex <<endl << endl;
		}
	}

	//����������֦����һ����֦������Slice�ϵ�

	/* //Ч�������� 2020.02.26
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

//��עSkeleton��ֱ�������֦
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
			cout << "Slice Index:" << i << ",Cluster ����:" << SliceDBScanClusters[i].pCluster->Clusters.size() << endl;

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
			//����ÿһ���������
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

//�Ƴ����޸������в�����Cluster 2020.04.09	 
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
//�������ɵ������Ի�ȡ���ɵ��������� 2020.04.07
//AllowAngle Ϊ�ж� ����� �����Ե���С�Ƕ�ֵ
void CDBScanTreeSkeleton::FindSuccessivePart(int SuccessiveNum, double AllowAngle)
{
	cout << "SliceDBScanClusters Size " << SliceDBScanClusters.size() << endl;
	//2020.04.07 ���ȼ����������
	for (int i = 2; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			//if (i == 236 && j == 2)
			//	cout <<"GlobalIndexs.size:"<<SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.size()<< endl;

			//cout << "�� " << i << " ��Slice�ĵ� " << j << " ����������ڴ���" << endl;
			int ParentSliceIndex, ParentClusterIndex;
			int GrandpaSliceIndex, GrandpaClusterIndex;

			if (SliceDBScanClusters[i].pCluster->Clusters[j].Label == "TempBranches" 
				|| SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.size() == 0)
				continue;

			//���õ�ǰ����ɫ������Ӱ��˶ε�Ŀ�ģ������丸�ڵ����ɫ
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
				//cout << "�� " << ParentSliceIndex << " ��Slice�ĵ� " << ParentClusterIndex << " ������㲻����Ҫ��" << endl;
				//SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].ConvexCentroid.rgba = ColorBase::RedColor;
				//SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].Centroid.rgba = ColorBase::RedColor;
				//SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].Label = "NeedFixed";

				//cout << "�� " << i << " ��Slice�ĵ� " << j << " ������㲻����Ҫ��" << endl;
				SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid.rgba = ColorBase::RedColor;
				SliceDBScanClusters[i].pCluster->Clusters[j].Centroid.rgba = ColorBase::RedColor;
				SliceDBScanClusters[i].pCluster->Clusters[j].Label = "NeedFixed";
			}
		}
	}

	///* //�� ������ SuccessiveNum ���ǿ������������һ����ķ���Ϊ ������������㼯 ���ɵ������
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

			//��������ø��ڵ�
			while (UpCount > 0 && ParentSliceIndex != -1 && ParentClusterIndex != -1)
			{
				//�м��в��������ĵ�
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

			//��ȡ�������������
			if (TempPoints->points.size() < SuccessiveNum)
				continue;
			
			SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection =	GeometryBase::GetMaxDirectionVector(TempPoints);
			//*/

			pcl::PointXYZRGB Direction;
			bool IsHave = CalcCurDirection(SuccessiveNum, i, j, Direction);
			if (IsHave)
				SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection = Direction;
			//cout << "�� " << ParentSliceIndex << " ��Slice�ĵ� " << ParentClusterIndex << " ����ķ���Ϊ��"
			//	<< SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection << endl;

			////��ʾ���з����ķ���
			//PointBase::ShowDirection(Viewer, SliceDBScanClusters[i].pCluster->Clusters[j].ConvexCentroid,
			//	SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection, 3);
		}
	}

	/*
	for (int i = SuccessiveNum + 1; i < SliceDBScanClusters.size(); i++)
	{
		//����ÿһ���������������� SuccessiveNum ����һ����֧�ϣ�����SuccessiveNum����Ŀ���㣬����
		//�����ٵĸ�������
		//Ӧ�÷ֱ����ÿ����֧����������ͬʱȷ�������ܵ���辵�Ӱ�죬���ȼ����������Ľ��м��㡣
		pcl::PointXYZRGB CurrentDirection;		
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			//cout<<"���ڼ��� "<<i<<" ��Slice�ĵ� "<<j<<" �������Ƿ�����"<<endl;

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

			//��������ø��ڵ�
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
			//��ȡ�������������
			if (TempPoints->points.size() < SuccessiveNum)
				continue;

			//�ٴ�ֱ�ӻ����һ���Ľڵ���Ϣ
			ParentSliceIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.SliceIndex;
			ParentClusterIndex = SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ParentNode.ClusterIndex;

			if (ParentSliceIndex == -1 || ParentClusterIndex == -1)
				continue;

			pcl::PointXYZRGB ParentDirection =
				SliceDBScanClusters[ParentSliceIndex].pCluster->Clusters[ParentClusterIndex].CurDirection;

			if (ParentDirection.x != 0 && ParentDirection.y != 0 && ParentDirection.z != 0) //�ϼ��ķ����Ѿ�����
			{
				double CurrentAngle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
					TempPoints->points[0], TempPoints->points[1], TempPoints->points[2]));

				//cout << "�� " << i << " ��Slice�ĵ� " << j << " ������CurrentAngle:"<< CurrentAngle << endl;

				if (CurrentAngle >= AllowAngle)
				{
					SliceDBScanClusters[i].pCluster->Clusters[j].CurDirection =
						GeometryBase::GetMaxDirectionVector(TempPoints);

					cout << "�� " << i << " ��Slice�ĵ� " << j << " ������������״̬" << endl;
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

						//cout << "�� " << i << " ��Slice�ĵ� " << j << " ������,K"<<k<<",CurrentAngle:" << CurrentAngle << endl;

						if (CurrentAngle < AllowAngle)
						{
							IsCountinue = false;
							break;
						}
					}
				}

				if (IsCountinue)	//��������״̬
				{
					cout << "�� " << i << " ��Slice�ĵ� " << j << " ������������״̬" << endl;
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

//���㵱ǰλ�ô����������� 2020.04.09
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

	//��������ø��ڵ�
	while (UpCount > 0 && ParentSliceIndex != -1 && ParentClusterIndex != -1)
	{
		//�м��в��������ĵ�
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

	//��ȡ�������������
	if (TempPoints->points.size() < SuccessiveNum)
		return false;
	
	CurDirection = GeometryBase::GetMaxDirectionVector(TempPoints);
	return true;
}

//���ݹ��˵�����辵������࣬����ӵ�֮ǰ�ľ���������
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

	//����ÿ���ص����ĵ�
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
		//������½�Cluster�ĸ���С��3����ϲ���֮ǰ�ľ�����
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
		cout<<"SliceIndex:"<< SliceIndex<<",����һ�����࣬���������:"
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
	////	cout << "����������ļ������ĵ������⣬SliceIndex:" << SliceIndex << ", ClusterIndex:" << ClusterIndex << endl;
	////}

	//SliceDBScanClusters[SliceIndex].pCluster->Clusters.push_back(TempCluster);
	//�Ƴ����¼������ĵ� ���½�һ����
}


////ʹ�÷ָ���֦͹������ηָ���ľ��ʹ��SliceIndex��ClusterIndex�ĸ��ڵ����Ϣ 
////�� SliceIndex��ClusterIndex ����͹������ηָ� 2020.04.08
void CDBScanTreeSkeleton::SegmentationByConvexPolygon(double Thick, int SuccessiveNum, 
	int SliceIndex, int ClusterIndex, double AllowDis)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ZonePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempClusterPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	vector<int> ZonePointsIndexs;
	vector<int> ConvexPolygonPointsIndexs;

	cout << "�����޸���SliceIndex��" << SliceIndex << ", ClusterIndex:" << ClusterIndex << endl;

	//ȡSliceIndex��ClusterIndex ǰ�� SuccessiveNum /2 �����ڵ���Ϊ�㼯��ȡ 
	ZonePointsIndexs.insert(ZonePointsIndexs.end(),
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.begin(),
		SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.end());

	GetSliceClusterPoints(SliceIndex, ClusterIndex, TempClusterPoints);
	ZonePoints->points.insert(ZonePoints->points.end(), TempClusterPoints->points.begin(),
		TempClusterPoints->points.end());

	int TempParentSlice = SliceIndex;
	int TempParentCluster = ClusterIndex;
	
	//Ѱ�Ҹ���Slice�ĵ���
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

	//Ѱ���Ӽ�Slice�ĵ���
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
			
			//��ӵ������
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

	//�������ȡ�ĵ��л�ȡ�������������һ���û�����ConvexPolygonPoints
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexPolygonPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	int ParentSlice = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.SliceIndex;
	int ParentCluster = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].NodeRoles.ParentNode.ClusterIndex;

	if (ParentSlice == -1 || ParentCluster == -1)
		return;

	if (SliceDBScanClusters[ParentSlice].pCluster->Clusters[ParentCluster].Label == "NeedFixed")
	{
		cout <<"����������Ҫ�޸���ParentSlice��"<< ParentSlice <<", ParentCluster:"<< ParentCluster << endl;
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
			cout << "�����޸���SliceIndex��" << SliceIndex << ", ClusterIndex:" << ClusterIndex
				<< " ʱδ����ȷ�����λ�ô�����������"<< endl;
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

	//����������������ĵ��ķ���нǴ���90�ȣ��������������
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
		cout << "�����޸���SliceIndex��" << SliceIndex << ", ClusterIndex:" << ClusterIndex 
			<<" ʱ��Ȼû���ҵ�͹���㼯����ʱ�����������ǣ�"<< CurGrowth<< endl;
		return;
	}
	//�� ZonePoints �滻Ϊ��ǰSlice��ǰCluster�ĵ㼯
	GetSliceClusterPoints(SliceIndex, ClusterIndex, ZonePoints);
	cout << "ZonePoints Points Count:" << ZonePoints->points.size()  <<", "
		<< SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size() << endl;
	if (ZonePoints->points.size() == 0) return;

	vector<int> TempBranchesIndexs = SegmentationPointsByConvexPolygon(ConvexPolygonPoints,
			ZonePoints, SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs, 
			CurGrowth, BottomPoint, AllowDis);
	//���û����֦�ڵ㣬˵��֮ǰ�������ط�Ӱ�쵼�´˴�Ҳ��Ӱ��
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

	//�Ƴ����¼������ĵ� 	
	VectorBase<int> VectorBaseInt;
	//cout<<"TempBranches Points Count:"<< TempBranchesIndexs.size() <<endl;

	vector<int> TempSliceIndexs;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempSliceClusterPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempBranchesClusterPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	//cout << "GlobalIndexs Count:" << SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size() << endl;

	for (int i = 0; i < SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size(); i++)
	{
		int TempIndex = SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs[i];

		//��ǰ�������������Ƴ������������У�����Ҫ����λԭʼ����
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
	//	cout<<"SliceIndex:"<< SliceIndex <<", ClusterIndex:"<< ClusterIndex <<"���ִ���!"<<endl;
	//	cout << "SliceIndex,Size:" << SliceDBScanClusters[SliceIndex].pCluster->Clusters[ClusterIndex].GlobalIndexs.size()
	//		<< ", TempBranchesIndexs,size:" << TempBranchesIndexs.size() << ", BranchesClusterPoints��Size:" 
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
	//2020.02.14 ������ڵ���3����
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

	//���½�һ����
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

//ʹ��͹������ηָ�㼯��AllowDis Ϊ͹������ε����;���, ���ز���͹��������ڵĵ㼯������ 2020.04.08
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

			if (TempValue * CoeffValues[j].PlaneValue < 0) //λ��ƽ������
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

//��ȡSliceIndex��ClusterIndex�ĵ㼯
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

//�Ƴ���֧�������ӹ�ϵ 2020.04.11
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


//�������ɵ��������޸����ɹǼ��еĵ㣬����ȷÿ�����������ɵĲ�ι�ϵ 2020.02.26
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
						cout << "����Ҫ�޸����ô������޷����� I:" << i << ", J:" << j << endl;
				}
			}			
		}
		//if (i == 56)
		//	break;
	}
	FindSuccessivePart(SuccessiveNum, AllowAngle);
	RemoveBrancheRoles();
}