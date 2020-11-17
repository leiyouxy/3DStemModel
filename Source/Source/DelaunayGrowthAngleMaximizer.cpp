#include "DelaunayGrowthAngleMaximizer.h"

void CDelaunayGrowthAngleMaximizer::SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
	CloudPtrValue)
{
	//CloudPtr = CloudPtrValue;	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointXYZRGB NormalPoint;
		
	GeometryBase::GetPointsTangentPlane(CloudPtrValue, NormalPoint);
	
	GeometryBase::RotateNormalToVertical(CloudPtrValue, PlanePoints, NormalPoint);

	CloudPtr.reset((new pcl::PointCloud<pcl::PointXYZRGB>));
	CloudPtr->points.insert(CloudPtr->points.begin(),
		PlanePoints->points.begin(), PlanePoints->points.end());

	//PointBase::SavePCDToFileName(CloudPtr, "I:\\PlanePoints\\PlanePoints20200525.pcd");
}

//2015.07.23 �ж��������Ƿ��ظ�
bool CDelaunayGrowthAngleMaximizer::FindTriangleIsExists(pcl::PolygonMesh & output, 
	int OneIndex, int TwoIndex, int ThreeIndex)
{
	bool Finded = false;

	for (int i = 0; i < output.polygons.size(); i++)
	{
		if ((output.polygons[i].vertices[0] == OneIndex || output.polygons[i].vertices[0] == TwoIndex 
			|| output.polygons[i].vertices[0] == ThreeIndex)
			&& (output.polygons[i].vertices[1] == OneIndex || output.polygons[i].vertices[1] == TwoIndex 
			|| output.polygons[i].vertices[1] == ThreeIndex)
			&& (output.polygons[i].vertices[2] == OneIndex || output.polygons[i].vertices[2] == TwoIndex 
			|| output.polygons[i].vertices[2] == ThreeIndex))
		{
			Finded = true;
			break;
		}
	}

	return (Finded);	//�����߶����ڵ�ʱ����������δ���
}

//2015.07.23 �жϱ��Ƿ���Ҫ��չ
bool CDelaunayGrowthAngleMaximizer::FindEdgeNeedExpand(pcl::PolygonMesh & output, 
	int OneIndex, int TwoIndex)
{
	int Count = 0;
	for(int i = 0; i < output.polygons.size(); i++)
	{
		if ((output.polygons[i].vertices[0] == OneIndex && output.polygons[i].vertices[1] == TwoIndex) 
			|| (output.polygons[i].vertices[1] == OneIndex && output.polygons[i].vertices[0] == TwoIndex)
			|| (output.polygons[i].vertices[1] == OneIndex && output.polygons[i].vertices[2] == TwoIndex)
			|| (output.polygons[i].vertices[2] == OneIndex && output.polygons[i].vertices[1] == TwoIndex)
			|| (output.polygons[i].vertices[0] == OneIndex && output.polygons[i].vertices[2] == TwoIndex)
			|| (output.polygons[i].vertices[2] == OneIndex && output.polygons[i].vertices[0] == TwoIndex))
		{
			Count++;
			if (2 == Count) return false;
		}
	}
	return true;
}

//ʹ�þ����㷨���� �Ե㼯��һ�� �� ������� ���� ��
void CDelaunayGrowthAngleMaximizer::performReconstructionRandomBaseEdge(
	pcl::PolygonMesh & output, bool UsePointSumAngleValue, bool FindTriangleValue)
{
	if (FindTriangleValue) 
	{	
		FindTriangle = true;
		FindEdge = false;
	}
	else 
	{
		FindTriangle = false;
		FindEdge = true;
	}
	
	output.polygons.clear();	
	NeedExpandEdges.clear();

	if (CloudPtr->points.size() < 3) return;

	UsePointSumAngle = UsePointSumAngleValue;
	pcl::PCLPointCloud2 TempPointCloud2;
	pcl::toPCLPointCloud2(*CloudPtr,TempPointCloud2);
	output.header = CloudPtr->header;
	output.cloud = TempPointCloud2;

	//////��ʼ��ʱ
	time_t c_start, c_end;
    c_start = clock(); 

	int OneIndex = 0;
	int TwoIndex = -1;
	double MinDis = 10000000;

	CalcBase<double> CalcBasefloat;

	//�������
	for(int i = 1; i < CloudPtr->points.size(); i++)
	{		
		if (i != OneIndex)
		{
			double TempDis = PointDis(CloudPtr->points[OneIndex].x, CloudPtr->points[OneIndex].y,
				CloudPtr->points[OneIndex].z, CloudPtr->points[i].x, 
				CloudPtr->points[i].y, CloudPtr->points[i].z, true);
			if (TempDis < MinDis)
			{
				MinDis = TempDis;
				TwoIndex = i;
			}
		}
	}

	int ThirdIndex = FindNextPointByAngleMax(OneIndex, TwoIndex);
	
	//if (FindTriangle)
	//	if (!FindTriangle(output, OneIndex, TwoIndex, ThirdIndex))	
			AddTriangle(output, OneIndex, TwoIndex, ThirdIndex, false);	

	int MaxNeedExpandNum = 0;

	while (NeedExpandEdges.size() > 0)
	{
		//if (output.polygons.size() > 3000)
		//	break;

		EdgeNodes TempEdgeNodes;		
		
		if (NeedExpandEdges.size() > MaxNeedExpandNum)
		{
			MaxNeedExpandNum = NeedExpandEdges.size();
		}

		//�Ƚ��ȳ�
		TempEdgeNodes = NeedExpandEdges[0];
		NeedExpandEdges.erase(NeedExpandEdges.begin());
		
		//����ȳ�
		//TempEdgeNodes = NeedExpandEdges[NeedExpandEdges.size() - 1];
		//NeedExpandEdges.pop_back();		

		if (FindEdge)
		{
			if (FindEdgeNeedExpand(output, TempEdgeNodes.EdgeIndexOne, TempEdgeNodes.EdgeIndexTwo))
			{
				ExpandEdge(output, TempEdgeNodes.ConstantIndex, 
					TempEdgeNodes.EdgeIndexOne, TempEdgeNodes.EdgeIndexTwo, false);				
			}
		}
		else
			ExpandEdge(output, TempEdgeNodes.ConstantIndex, 
				TempEdgeNodes.EdgeIndexOne, TempEdgeNodes.EdgeIndexTwo, false);
	}

    c_end = clock(); 

	//cout<<"������չ����"<<MaxNeedExpandNum<<endl;

	//if (FindEdge)
	//	cout<<"The �����㷨 FindEdge: "
	//		<<" times: "<<difftime(c_end,c_start)<<" ms"<<endl;
	//else
	//	cout<<"The �����㷨 FindTriangle: "
	//		<<" times: "<<difftime(c_end,c_start)<<" ms"<<endl;
}

//2015.07.23 ��ĽǶȺͳ�ʼ��
void CDelaunayGrowthAngleMaximizer::SumAngleInitial()
{
	SumAngle.clear(); //2016.01.23 �����������
	CopyIndexCloud.reset(new pcl::PointCloud<PointXYZRGBIndex>);
	PointBase::PointXYZRGBToPointXYZRGBIndex(CloudPtr, CopyIndexCloud);
		
	for(int i = 0; i < CloudPtr->points.size(); i++)
	{
		SumAngle.push_back(2 * M_PI);
		CopyIndexCloud->points[i].Category = 1;
	}
	
	//͹����ĽǶȺͲ�һ��
	//SumAngle[HullPoints[0]] = GeometryBase::AngleValueOfThreePoints(
	//		CloudPtr->points[HullPoints[HullPoints.size() - 2]].x, 
	//		CloudPtr->points[HullPoints[HullPoints.size() - 2]].y,
	//		CloudPtr->points[HullPoints[0]].x, CloudPtr->points[HullPoints[0]].y,
	//		CloudPtr->points[HullPoints[1]].x, CloudPtr->points[HullPoints[1]].y);
	
	//for(int i = 0; i < HullPoints.size() - 2; i++)
	//{
	//	SumAngle[HullPoints[i+1]] = GeometryBase::AngleValueOfThreePoints(
	//		CloudPtr->points[HullPoints[i]].x, CloudPtr->points[HullPoints[i]].y,
	//		CloudPtr->points[HullPoints[i+1]].x, CloudPtr->points[HullPoints[i+1]].y,
	//		CloudPtr->points[HullPoints[i+2]].x, CloudPtr->points[HullPoints[i+2]].y);
	//}

	//2016.01.29 ����Ϊȡ��ķ�ʽ����
	//HullPoints.pop_back(); //��͹��������β��ӣ�����ȥ��β���ڵ�
	int n = HullPoints.size();
	for(int i = 0; i < n; i++)
	{
		SumAngle[HullPoints[i]] = GeometryBase::AngleValueOfThreePoints(
			CloudPtr->points[HullPoints[(i+n-1)%n]].x, CloudPtr->points[HullPoints[(i+n-1)%n]].y,
			CloudPtr->points[HullPoints[i]].x, CloudPtr->points[HullPoints[i]].y,
			CloudPtr->points[HullPoints[(i+1)%n]].x, CloudPtr->points[HullPoints[(i+1)%n]].y);
	}
}

void CDelaunayGrowthAngleMaximizer::performReconstruction(pcl::PolygonMesh & output, 
	//  ʣ�µ�����������Ҫ������ѡ�����������
		bool UsePointSumAngleValue)		//�Ƿ�ʹ�õ���ǶȺ�Ϊ360		 
{	
	output.polygons.clear();

	UsePointSumAngle = UsePointSumAngleValue;

	if (CloudPtr->points.size() < 3) return;

	pcl::PCLPointCloud2 TempPointCloud2;
	pcl::toPCLPointCloud2(*CloudPtr,TempPointCloud2);
	output.header = CloudPtr->header;
	output.cloud = TempPointCloud2;
		
	//////��ʼ��ʱ
	time_t c_start, c_end;
    c_start = clock(); 

	//��ȡ͹���� ///
	CContourAndConvexHull<pcl::PointXYZRGB> InPutCloudHull;
	InPutCloudHull.SetInputs(CloudPtr);
	//PointBase::SavePCDToFileName(CloudPtr, "I:\\ProcessedPoints\\TestConvexHull.pcd");

	InPutCloudHull.GetPointsConvexHull(HullPoints);
	//HullPoints.push_back(HullPoints[0]);//ʹ͹����β����  2016.01.29 ע�͵�
	
	if (UsePointSumAngleValue)
	{
		SumAngleInitial();	

		c_end = clock();
		cout << "The Initial of CDelaunayGrowthAngleMaximizer: "
			<< " times: " << difftime(c_end, c_start) << " ms" << endl;
	}		
	
	int OneIndex = HullPoints[0];	
	int TwoIndex = HullPoints[1];
	int ThirdIndex = FindNextPointByAngleMax(OneIndex, TwoIndex);

	PriorityIndex = OneIndex; 
	AddTriangle(output, OneIndex, TwoIndex, ThirdIndex);

	int MaxNeedExpandNum = 0;
	while (NeedExpandEdges.size() > 0)
	{		
		EdgeNodes TempEdgeNodes;		
		
		if (NeedExpandEdges.size() > MaxNeedExpandNum)
		{
			MaxNeedExpandNum = NeedExpandEdges.size();
		}

		//�Ƚ��ȳ�
		TempEdgeNodes = NeedExpandEdges[0];
		NeedExpandEdges.erase(NeedExpandEdges.begin());
		
		//����ȳ�
		//TempEdgeNodes = NeedExpandEdges[NeedExpandEdges.size() - 1];
		//NeedExpandEdges.pop_back();		

		if (UsePointSumAngleValue && !(PriorityIndex == TempEdgeNodes.EdgeIndexOne 
										|| PriorityIndex == TempEdgeNodes.EdgeIndexTwo))
		{
			//RemovePointsFromCopyIndexCloud(PriorityIndex);
			PriorityIndex = TempEdgeNodes.EdgeIndexOne;
			ForwardMove();
		}

		ExpandEdge(output, TempEdgeNodes.ConstantIndex, 
			TempEdgeNodes.EdgeIndexOne, TempEdgeNodes.EdgeIndexTwo);
	}

    c_end = clock(); 

	//cout<<"������չ����"<<MaxNeedExpandNum<<endl;

	//if (UsePointSumAngle)
	//	cout<<"The CDelaunayGrowthAngleMaximizer UsePointSumAngle: "
	//		<<" times: "<<difftime(c_end,c_start)<<" ms"<<endl;
	//else
	cout<<"The CDelaunayGrowthAngleMaximizer: "
			<<" times: "<<difftime(c_end,c_start)<<" ms"<<endl;
}

//ͨ��������Ƕ����ķ�ʽ����һ���� ���صĵ����������� OneIndex->TwoIndex ���������
//�� OneIndex->TwoIndex TwoIndex->ThreeIndex ThreeIndex->OneIndex ����ʱ�뷽��
int CDelaunayGrowthAngleMaximizer::FindNextPointByAngleMax(int OneIndex, int TwoIndex)
{
	int MinIndex = -1;
	double X1 = CloudPtr->points[OneIndex].x;
	double Y1 = CloudPtr->points[OneIndex].y;
	double X2 = CloudPtr->points[TwoIndex].x;
	double Y2 = CloudPtr->points[TwoIndex].y;

	long double MinCos = 10, CosA;

	if (UsePointSumAngle)	//���ʹ�ýǶȺ�
	{
		for(int i = 0; i < CopyIndexCloud->points.size(); i++)
		{	//ͬʱ����㻹����ʹ��   //
			int TempIndex = CopyIndexCloud->points[i].Index;
			//if (OneIndex != i && TwoIndex != i && CopyIndexCloud->points[i].Category == 1)
			if (OneIndex != TempIndex && TwoIndex != TempIndex && CopyIndexCloud->points[i].Category == 1)
			{
				double X3 = CloudPtr->points[TempIndex].x;
				double Y3 = CloudPtr->points[TempIndex].y;

				double Area = GeometryBase::AreaOfThreePointsIn2D(X1, Y1, X2, Y2, X3, Y3);				
				if ( Area > 0)	//�������0 �Ѿ����� ���������㹲��
				{
					CosA = GeometryBase::CosOfThreePoints(X1,Y1, X3,Y3, X2,Y2);
					if (CosA < MinCos)
					{
						MinCos = CosA;
						MinIndex = TempIndex;
					}
				}
			}
		}
	}
	else	//��ʹ�ýǶȺ�
	{
		for(int i = 0; i < CloudPtr->points.size(); i++)
		{
			if ( OneIndex != i && TwoIndex != i)
			{
				double X3 = CloudPtr->points[i].x;
				double Y3 = CloudPtr->points[i].y;

				double Area = GeometryBase::AreaOfThreePointsIn2D(X1, Y1, X2, Y2, X3, Y3);
				//if ( Area > 0.000001)
				if ( Area > 0 )
				{
					CosA = GeometryBase::CosOfThreePoints(X1,Y1, X3,Y3, X2,Y2);

					if (CosA < MinCos)
					{
 						MinCos = CosA;
						MinIndex = i;
					}					
				}
			}
		}
	}	
	return MinIndex;
}

//���������
void CDelaunayGrowthAngleMaximizer::AddTriangle(pcl::PolygonMesh & output,
	int OneIndex, int TwoIndex, int ThreeIndex, bool NeedFindEdge)
{
	//��������ε����������
	pcl::Vertices TempVertices;
	TempVertices.vertices.push_back(OneIndex);
	TempVertices.vertices.push_back(TwoIndex);
	TempVertices.vertices.push_back(ThreeIndex);
	output.polygons.push_back(TempVertices);	

	if (UsePointSumAngle)
	{
		SumAngle[OneIndex] = SumAngle[OneIndex] - 
			GeometryBase::AngleValueOfThreePoints(CloudPtr->points[TwoIndex].x, CloudPtr->points[TwoIndex].y,
			CloudPtr->points[OneIndex].x, CloudPtr->points[OneIndex].y,
			CloudPtr->points[ThreeIndex].x, CloudPtr->points[ThreeIndex].y);
		SumAngle[TwoIndex] = SumAngle[TwoIndex] - 
			GeometryBase::AngleValueOfThreePoints(CloudPtr->points[OneIndex].x, CloudPtr->points[OneIndex].y,
			CloudPtr->points[TwoIndex].x, CloudPtr->points[TwoIndex].y,
			CloudPtr->points[ThreeIndex].x, CloudPtr->points[ThreeIndex].y);
		SumAngle[ThreeIndex] = SumAngle[ThreeIndex] - 
			GeometryBase::AngleValueOfThreePoints(CloudPtr->points[OneIndex].x, CloudPtr->points[OneIndex].y,
			CloudPtr->points[ThreeIndex].x, CloudPtr->points[ThreeIndex].y,
			CloudPtr->points[TwoIndex].x, CloudPtr->points[TwoIndex].y);
		
		//if (abs(SumAngle[OneIndex]) < 0.000001)
		if ((SumAngle[OneIndex]) < 0.000001)
		//if (abs(SumAngle[OneIndex]) == 0)
		{	//���ɾ������㣬�ͻ�ʹ�ö�λ��ʱ������
			//CopyIndexCloud->points[OneIndex].Category = 0;

			int TempIndex = GetTempIndexFromCopyIndexCloud(OneIndex);
			if (-1 != TempIndex)
				CopyIndexCloud->points.erase(CopyIndexCloud->points.begin() + TempIndex);
			//CopyIndexCloud->points[TempIndex].Category = 0;							
		}
		//if (abs(SumAngle[TwoIndex]) < 0.000001)
		if ((SumAngle[TwoIndex]) < 0.000001)
		//if (abs(SumAngle[TwoIndex]) == 0)
		{
			//CopyIndexCloud->points[TwoIndex].Category = 0;

			int TempIndex = GetTempIndexFromCopyIndexCloud(TwoIndex);
			if (-1 != TempIndex)
				CopyIndexCloud->points.erase(CopyIndexCloud->points.begin() + TempIndex);
		}
		//if (abs(SumAngle[ThreeIndex]) < 0.000001)
		if ((SumAngle[ThreeIndex]) < 0.000001) //2015.10.29 ���޸� ������ֵ�����׳���
		//if (abs(SumAngle[ThreeIndex]) == 0)
		{
			//CopyIndexCloud->points[ThreeIndex].Category = 0;

			int TempIndex = GetTempIndexFromCopyIndexCloud(ThreeIndex);
			if (-1 != TempIndex)
				CopyIndexCloud->points.erase(CopyIndexCloud->points.begin() + TempIndex);
		}
	}

	if (!NeedFindEdge)	//ֱ�Ӳ���˱�
	{
		if (FindEdge)
		{
			if (FindEdgeNeedExpand(output, OneIndex, ThreeIndex)) 
			{
				EdgeNodes TempEdgeNodesOne; 		
				TempEdgeNodesOne.ConstantIndex = TwoIndex;		
				TempEdgeNodesOne.EdgeIndexOne = OneIndex;	//��ʱ������ԭ�����δ����෴
				TempEdgeNodesOne.EdgeIndexTwo = ThreeIndex;		
				NeedExpandEdges.push_back(TempEdgeNodesOne);				
			}

			if (FindEdgeNeedExpand(output, ThreeIndex, TwoIndex)) 
			{
				EdgeNodes TempEdgeNodesOne; 
				TempEdgeNodesOne.ConstantIndex = OneIndex;		
				TempEdgeNodesOne.EdgeIndexOne = ThreeIndex;
				TempEdgeNodesOne.EdgeIndexTwo = TwoIndex;
				NeedExpandEdges.push_back(TempEdgeNodesOne);			
			}
		}
		else
		{
			EdgeNodes TempEdgeNodesOne; 		
			TempEdgeNodesOne.ConstantIndex = TwoIndex;		
			TempEdgeNodesOne.EdgeIndexOne = OneIndex;	//��ʱ������ԭ�����δ����෴
			TempEdgeNodesOne.EdgeIndexTwo = ThreeIndex;		
			NeedExpandEdges.push_back(TempEdgeNodesOne);		
			
			TempEdgeNodesOne.ConstantIndex = OneIndex;		
			TempEdgeNodesOne.EdgeIndexOne = ThreeIndex;
			TempEdgeNodesOne.EdgeIndexTwo = TwoIndex;
			NeedExpandEdges.push_back(TempEdgeNodesOne);	
		}
	}


	if (NeedFindEdge && !IsBorderEdge(ThreeIndex, OneIndex))	//�����OneIndex�Ľڵ�
	{
		if (!EdgeIsInNeedExpaned(ThreeIndex, OneIndex))
		{
			EdgeNodes TempEdgeNodesOne; 		
			TempEdgeNodesOne.ConstantIndex = TwoIndex;		
			TempEdgeNodesOne.EdgeIndexOne = OneIndex;	//��ʱ������ԭ�����δ����෴
			TempEdgeNodesOne.EdgeIndexTwo = ThreeIndex;

			if (UsePointSumAngle && (ThreeIndex == PriorityIndex || OneIndex == PriorityIndex))
				NeedExpandEdges.insert(NeedExpandEdges.begin(), TempEdgeNodesOne);
			else
				NeedExpandEdges.push_back(TempEdgeNodesOne);
		}
	}

	if (NeedFindEdge && !IsBorderEdge(TwoIndex, ThreeIndex))
	{
		if (!EdgeIsInNeedExpaned(TwoIndex, ThreeIndex))
		{
			EdgeNodes TempEdgeNodesOne; 		
			TempEdgeNodesOne.ConstantIndex = OneIndex;		
			TempEdgeNodesOne.EdgeIndexOne = ThreeIndex;
			TempEdgeNodesOne.EdgeIndexTwo = TwoIndex;

			if (UsePointSumAngle && (ThreeIndex == PriorityIndex || TwoIndex == PriorityIndex))
				NeedExpandEdges.insert(NeedExpandEdges.begin(), TempEdgeNodesOne);
			else
				NeedExpandEdges.push_back(TempEdgeNodesOne);
		}
	}
}

void CDelaunayGrowthAngleMaximizer::ExpandEdge(pcl::PolygonMesh & output, 
	int ConstantIndex, //�Ѿ��̶���һ������
	int OneIndex, int TwoIndex, bool NeedFindEdge) //����չ����������
{
	int ThreeIndex = FindNextPointByAngleMax(OneIndex, TwoIndex);
		
	if (-1 == ThreeIndex)
	{
		//cout<<"��"<<OneIndex<<","<<TwoIndex<<"��չʧ��"<<endl;
	}
	else
	{
		if (!NeedFindEdge)
		{
			if (FindTriangle)	//��������
			{
				if (!FindTriangleIsExists(output, OneIndex, TwoIndex, ThreeIndex))
					AddTriangle(output, OneIndex, TwoIndex, ThreeIndex, NeedFindEdge);
			}
			else				//�ұߵ�����¾�ֱ�Ӳ���������
			{
				AddTriangle(output, OneIndex, TwoIndex, ThreeIndex, NeedFindEdge);
			}
		}
		else
		{
			AddTriangle(output, OneIndex, TwoIndex, ThreeIndex, NeedFindEdge);
		}
	}
}

//�Ƿ���ȫ�ֵı߽��
bool CDelaunayGrowthAngleMaximizer::IsBorderEdge(int OneIndex, int TwoIndex)
{
	for(int i = 0; i < HullPoints.size() - 1; i++)
	{
		if ((HullPoints[i] == OneIndex && HullPoints[i+1] == TwoIndex)
			|| (HullPoints[i] == TwoIndex && HullPoints[i+1] == OneIndex))
		{
			return true;
		}
	}
	return false;
}

//�жϱ��Ƿ�����Ҫ��չ�ı��� ����о�ɾ��
bool CDelaunayGrowthAngleMaximizer::EdgeIsInNeedExpaned(int OneIndex, int TwoIndex)
{
	for(int i = 0; i < NeedExpandEdges.size(); i++)
	{
		if ((NeedExpandEdges[i].EdgeIndexOne == OneIndex && NeedExpandEdges[i].EdgeIndexTwo == TwoIndex)
			|| (NeedExpandEdges[i].EdgeIndexOne == TwoIndex && NeedExpandEdges[i].EdgeIndexTwo == OneIndex))
		{
			NeedExpandEdges.erase(NeedExpandEdges.begin() + i);
			return true;
		}
	}

	return false;
}

int CDelaunayGrowthAngleMaximizer::FindNearestPoint(int OneIndex)
{
	double MinDis = 100000;
	CalcBase<double> CalcBaseDouble;
	int NearestIndex = -1;

	for(int i = 0; i < CloudPtr->points.size(); i++)
	{
		if (i != OneIndex)
		{
			double Dis = PointDis(
				CloudPtr->points[i].x, CloudPtr->points[i].y, CloudPtr->points[i].z,
				CloudPtr->points[OneIndex].x, CloudPtr->points[OneIndex].y, CloudPtr->points[OneIndex].z);
			if (MinDis > Dis)
			{
				MinDis = Dis;
				NearestIndex = i;
			}
		}
	}
	return NearestIndex;
}

//��CopyIndexCloud������Ѱ��IndexΪIndex�ĵ���
int CDelaunayGrowthAngleMaximizer::GetTempIndexFromCopyIndexCloud(int Index) 
{
	int ReturnIndex = -1;
	for(int i = 0; i < CopyIndexCloud->points.size(); i++)
	{
		if(CopyIndexCloud->points[i].Index == Index)
		{
			ReturnIndex = i;
			break;
		}
	}
	//if (-1 == ReturnIndex)
	//	cout<<endl;
	return ReturnIndex;
}


//����������Ȩ�ı��ƶ���ǰ����
void CDelaunayGrowthAngleMaximizer::ForwardMove()
{
	int j = 0;

	for(int i = 1; i < NeedExpandEdges.size(); i++)
	{
		if(NeedExpandEdges[i].EdgeIndexOne == PriorityIndex || NeedExpandEdges[i].EdgeIndexTwo == PriorityIndex)
		{
			swap(NeedExpandEdges[j], NeedExpandEdges[i]);
			j++;
		}
	}
}


//2015.07.15 ��CopyIndexCloudɾ��IndexΪIndex�Ľڵ�
void CDelaunayGrowthAngleMaximizer::RemovePointsFromCopyIndexCloud(int Index)
{
	for(int i = 0; i < CopyIndexCloud->points.size(); i++)
	{
		if (CopyIndexCloud->points[i].Index == Index)
		{
			CopyIndexCloud->points.erase(CopyIndexCloud->points.begin() + i);
			break;
		}
	}
}

//���� Index ����� �Ƿ��Ѿ��Ƿ�յ�
bool CDelaunayGrowthAngleMaximizer::GetPointIsClosed(
	int PointIndex, vector<int> & NeighborIndexs)
{
	if (!UsePointSumAngle)
		return false;

	int TempIndex = -1;
	for(int i = 0; i < NeighborIndexs.size(); i++)
	{
		if (PointIndex == NeighborIndexs[i])
		{
			TempIndex = i;
			break;
		}
	}
		
	if (TempIndex == -1) return false;
	else if((SumAngle[TempIndex]) < 0.000001)
		return true;
	else return false;
}