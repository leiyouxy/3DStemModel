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

//2015.07.23 判断三角形是否重复
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

	return (Finded);	//三条边都存在的时候表面三角形存在
}

//2015.07.23 判断边是否还需要扩展
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

//使用经典算法构建 以点集中一点 及 最近点做 基边 且
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

	//////开始计时
	time_t c_start, c_end;
    c_start = clock(); 

	int OneIndex = 0;
	int TwoIndex = -1;
	double MinDis = 10000000;

	CalcBase<double> CalcBasefloat;

	//找最近点
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

		//先进先出
		TempEdgeNodes = NeedExpandEdges[0];
		NeedExpandEdges.erase(NeedExpandEdges.begin());
		
		//后进先出
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

	//cout<<"最大待扩展边数"<<MaxNeedExpandNum<<endl;

	//if (FindEdge)
	//	cout<<"The 经典算法 FindEdge: "
	//		<<" times: "<<difftime(c_end,c_start)<<" ms"<<endl;
	//else
	//	cout<<"The 经典算法 FindTriangle: "
	//		<<" times: "<<difftime(c_end,c_start)<<" ms"<<endl;
}

//2015.07.23 点的角度和初始化
void CDelaunayGrowthAngleMaximizer::SumAngleInitial()
{
	SumAngle.clear(); //2016.01.23 首先清空所有
	CopyIndexCloud.reset(new pcl::PointCloud<PointXYZRGBIndex>);
	PointBase::PointXYZRGBToPointXYZRGBIndex(CloudPtr, CopyIndexCloud);
		
	for(int i = 0; i < CloudPtr->points.size(); i++)
	{
		SumAngle.push_back(2 * M_PI);
		CopyIndexCloud->points[i].Category = 1;
	}
	
	//凸包点的角度和不一样
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

	//2016.01.29 调整为取余的方式进行
	//HullPoints.pop_back(); //因凸包点是首尾相接，所以去掉尾部节点
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
	//  剩下的三个变量主要体现在选择第三个点上
		bool UsePointSumAngleValue)		//是否使用单点角度和为360		 
{	
	output.polygons.clear();

	UsePointSumAngle = UsePointSumAngleValue;

	if (CloudPtr->points.size() < 3) return;

	pcl::PCLPointCloud2 TempPointCloud2;
	pcl::toPCLPointCloud2(*CloudPtr,TempPointCloud2);
	output.header = CloudPtr->header;
	output.cloud = TempPointCloud2;
		
	//////开始计时
	time_t c_start, c_end;
    c_start = clock(); 

	//获取凸包点 ///
	CContourAndConvexHull<pcl::PointXYZRGB> InPutCloudHull;
	InPutCloudHull.SetInputs(CloudPtr);
	//PointBase::SavePCDToFileName(CloudPtr, "I:\\ProcessedPoints\\TestConvexHull.pcd");

	InPutCloudHull.GetPointsConvexHull(HullPoints);
	//HullPoints.push_back(HullPoints[0]);//使凸包首尾相连  2016.01.29 注释掉
	
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

		//先进先出
		TempEdgeNodes = NeedExpandEdges[0];
		NeedExpandEdges.erase(NeedExpandEdges.begin());
		
		//后进先出
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

	//cout<<"最大待扩展边数"<<MaxNeedExpandNum<<endl;

	//if (UsePointSumAngle)
	//	cout<<"The CDelaunayGrowthAngleMaximizer UsePointSumAngle: "
	//		<<" times: "<<difftime(c_end,c_start)<<" ms"<<endl;
	//else
	cout<<"The CDelaunayGrowthAngleMaximizer: "
			<<" times: "<<difftime(c_end,c_start)<<" ms"<<endl;
}

//通过第三点角度最大的方式找下一个点 返回的第三个点总是 OneIndex->TwoIndex 方向的正向
//即 OneIndex->TwoIndex TwoIndex->ThreeIndex ThreeIndex->OneIndex 是逆时针方向
int CDelaunayGrowthAngleMaximizer::FindNextPointByAngleMax(int OneIndex, int TwoIndex)
{
	int MinIndex = -1;
	double X1 = CloudPtr->points[OneIndex].x;
	double Y1 = CloudPtr->points[OneIndex].y;
	double X2 = CloudPtr->points[TwoIndex].x;
	double Y2 = CloudPtr->points[TwoIndex].y;

	long double MinCos = 10, CosA;

	if (UsePointSumAngle)	//如果使用角度和
	{
		for(int i = 0; i < CopyIndexCloud->points.size(); i++)
		{	//同时这个点还可以使用   //
			int TempIndex = CopyIndexCloud->points[i].Index;
			//if (OneIndex != i && TwoIndex != i && CopyIndexCloud->points[i].Category == 1)
			if (OneIndex != TempIndex && TwoIndex != TempIndex && CopyIndexCloud->points[i].Category == 1)
			{
				double X3 = CloudPtr->points[TempIndex].x;
				double Y3 = CloudPtr->points[TempIndex].y;

				double Area = GeometryBase::AreaOfThreePointsIn2D(X1, Y1, X2, Y2, X3, Y3);				
				if ( Area > 0)	//面积大于0 已经限制 不可能三点共线
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
	else	//不使用角度和
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

//添加三角形
void CDelaunayGrowthAngleMaximizer::AddTriangle(pcl::PolygonMesh & output,
	int OneIndex, int TwoIndex, int ThreeIndex, bool NeedFindEdge)
{
	//添加三角形到最终输出中
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
		{	//如果删除这个点，就会使得定位的时候搜索
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
		if ((SumAngle[ThreeIndex]) < 0.000001) //2015.10.29 日修改 带绝对值后容易出错
		//if (abs(SumAngle[ThreeIndex]) == 0)
		{
			//CopyIndexCloud->points[ThreeIndex].Category = 0;

			int TempIndex = GetTempIndexFromCopyIndexCloud(ThreeIndex);
			if (-1 != TempIndex)
				CopyIndexCloud->points.erase(CopyIndexCloud->points.begin() + TempIndex);
		}
	}

	if (!NeedFindEdge)	//直接插入此边
	{
		if (FindEdge)
		{
			if (FindEdgeNeedExpand(output, OneIndex, ThreeIndex)) 
			{
				EdgeNodes TempEdgeNodesOne; 		
				TempEdgeNodesOne.ConstantIndex = TwoIndex;		
				TempEdgeNodesOne.EdgeIndexOne = OneIndex;	//此时次序与原三角形次序相反
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
			TempEdgeNodesOne.EdgeIndexOne = OneIndex;	//此时次序与原三角形次序相反
			TempEdgeNodesOne.EdgeIndexTwo = ThreeIndex;		
			NeedExpandEdges.push_back(TempEdgeNodesOne);		
			
			TempEdgeNodesOne.ConstantIndex = OneIndex;		
			TempEdgeNodesOne.EdgeIndexOne = ThreeIndex;
			TempEdgeNodesOne.EdgeIndexTwo = TwoIndex;
			NeedExpandEdges.push_back(TempEdgeNodesOne);	
		}
	}


	if (NeedFindEdge && !IsBorderEdge(ThreeIndex, OneIndex))	//先添加OneIndex的节点
	{
		if (!EdgeIsInNeedExpaned(ThreeIndex, OneIndex))
		{
			EdgeNodes TempEdgeNodesOne; 		
			TempEdgeNodesOne.ConstantIndex = TwoIndex;		
			TempEdgeNodesOne.EdgeIndexOne = OneIndex;	//此时次序与原三角形次序相反
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
	int ConstantIndex, //已经固定的一个顶点
	int OneIndex, int TwoIndex, bool NeedFindEdge) //待扩展到两个顶点
{
	int ThreeIndex = FindNextPointByAngleMax(OneIndex, TwoIndex);
		
	if (-1 == ThreeIndex)
	{
		//cout<<"边"<<OneIndex<<","<<TwoIndex<<"拓展失败"<<endl;
	}
	else
	{
		if (!NeedFindEdge)
		{
			if (FindTriangle)	//找三角形
			{
				if (!FindTriangleIsExists(output, OneIndex, TwoIndex, ThreeIndex))
					AddTriangle(output, OneIndex, TwoIndex, ThreeIndex, NeedFindEdge);
			}
			else				//找边的情况下就直接插入三角形
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

//是否是全局的边界边
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

//判断边是否在需要扩展的边中 如果有就删除
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

//在CopyIndexCloud点云中寻找Index为Index的点云
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


//将具有优先权的边移动到前面来
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


//2015.07.15 从CopyIndexCloud删除Index为Index的节点
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

//返回 Index 这个点 是否已经是封闭点
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