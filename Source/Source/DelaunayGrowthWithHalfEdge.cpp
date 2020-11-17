#include "DelaunayGrowthWithHalfEdge.h"

int CDelaunayGrowthWithHalfEdge::FindNextPointByAngleMax(HE_Edge Base_Edge)
{
	int MinIndex = -1;
	double X1 = InputCloud->points[Base_Edge.StartIndex].x;
	double Y1 = InputCloud->points[Base_Edge.StartIndex].y;
	double X2 = InputCloud->points[Base_Edge.EndIndex].x;
	double Y2 = InputCloud->points[Base_Edge.EndIndex].y;

	long double MinCos = 10, CosA;

	//if (UsePointSumAngle)	//如果使用角度和
	{
		for (int i = 0; i < CopyIndexCloud->points.size(); i++)
		{	//同时这个点还可以使用   //
			int TempIndex = CopyIndexCloud->points[i].Index;
			//if (OneIndex != i && TwoIndex != i && CopyIndexCloud->points[i].Category == 1)
			if (Base_Edge.StartIndex != TempIndex && Base_Edge.EndIndex != TempIndex)
			{
				double X3 = InputCloud->points[TempIndex].x;
				double Y3 = InputCloud->points[TempIndex].y;

				double Area = GeometryBase::AreaOfThreePointsIn2D(X1, Y1, X2, Y2, X3, Y3);
				if (Area > 0)	//面积大于0 已经限制 不可能三点共线 且已经确保是逆时针次序
				{
					CosA = GeometryBase::CosOfThreePoints(X1, Y1, X3, Y3, X2, Y2);
					if (CosA < MinCos)
					{
						MinCos = CosA;
						MinIndex = TempIndex;
					}
				}
			}
		}
	}
	//else	//不使用角度和
	//{
	//	for (int i = 0; i < CloudPtr->points.size(); i++)
	//	{
	//		if (Base_Edge.StartIndex != i && Base_Edge.EndIndex != i)
	//		{
	//			double X3 = CloudPtr->points[i].x;
	//			double Y3 = CloudPtr->points[i].y;

	//			double Area = GeometryBase::AreaOfThreePointsIn2D(X1, Y1, X2, Y2, X3, Y3);
	//			//if ( Area > EPSM6)
	//			if (Area > 0)
	//			{
	//				CosA = GeometryBase::CosOfThreePoints(X1, Y1, X3, Y3, X2, Y2);

	//				if (CosA < MinCos)
	//				{
	//					MinCos = CosA;
	//					MinIndex = i;
	//				}
	//			}
	//		}
	//	}
	//}
	return MinIndex;
}

//添加到Face中，并添加待扩展列表，添加一个三角形
void CDelaunayGrowthWithHalfEdge::AddFace(HE_Edge Base_Edge, int ThirdPointIndex, 
	int GuidLineIndex, bool IsExpandReverseEdge)
{
	/*
	半边规则是一条边只能出现一次，如果该边是边界边，则该边的对边不应该出现
	*/

	int BaseEdgeIndex = -1, SecondEdgeIndex = -1, ThirdEdgeIndex = -1;

	//if (InputCloud->points.size() == 697 && ((Base_Edge.StartIndex == 38 && Base_Edge.EndIndex == 39) ||
	//	(Base_Edge.StartIndex == 39 && Base_Edge.EndIndex == 38)))
	//{
	//	Base_Edge.StartIndex = Base_Edge.StartIndex;
	//}

	//如果新生成的三条边已经在半边容器中，则说明该三角形已经生成了，
	int TempIndex = FindEdgeinEdges(Point_HE_edges[Base_Edge.StartIndex].V_HE_Edges, Base_Edge);	
	if (-1 != TempIndex)
		return;
	if (-1 != FindEdgeinEdges(Point_HE_edges[Base_Edge.EndIndex].V_HE_Edges, Base_Edge.EndIndex, ThirdPointIndex))
		return;
	if (-1 != FindEdgeinEdges(Point_HE_edges[ThirdPointIndex].V_HE_Edges, ThirdPointIndex, Base_Edge.StartIndex))
		return;

	//如果该边没有存在则存入到HE_edges
	//if (Base_Edge.EdgeIndex == -1)
	{
		Point_HE_edges[Base_Edge.StartIndex].V_HE_Edges.push_back(Base_Edge);
		BaseEdgeIndex = Point_HE_edges[Base_Edge.StartIndex].V_HE_Edges.size() - 1;
		Point_HE_edges[Base_Edge.StartIndex].V_HE_Edges[BaseEdgeIndex].EdgeIndex = BaseEdgeIndex;
		Base_Edge.EdgeIndex = BaseEdgeIndex;
	}
	//else
	//{
	//	BaseEdgeIndex = Base_Edge.EdgeIndex;
	//	cout<<"Not Possible"<<endl;
	//}
		
	//构建第二个新边
	HE_Edge SecondEdge, ThirdEdge;
	SecondEdge.StartIndex = Base_Edge.EndIndex;
	SecondEdge.EndIndex = ThirdPointIndex;	
	SecondEdge.OppositeEdgeIndex = -1;
	//SecondEdge.FaceIndex = -1;
	SecondEdge.EdgeIndex = -1;
	
	//在待扩展列中寻找，如果找到则 对 OppositeEdgeIndex 赋值，并从 NeedExpandEdges 中删除
	int FindSecondNeedIndex = FindEdgeinEdges(NeedExpandEdges, SecondEdge);	
	if (FindSecondNeedIndex != -1)
	{		
		SecondEdge.OppositeEdgeIndex = NeedExpandEdges[FindSecondNeedIndex].OppositeEdgeIndex;		
		NeedExpandEdges.erase(NeedExpandEdges.begin() + FindSecondNeedIndex);
	}
	
	//将第二个新边插入到容器中并设置索引	
	Point_HE_edges[SecondEdge.StartIndex].V_HE_Edges.push_back(SecondEdge);
	SecondEdgeIndex = Point_HE_edges[SecondEdge.StartIndex].V_HE_Edges.size() - 1;
	Point_HE_edges[SecondEdge.StartIndex].V_HE_Edges[SecondEdgeIndex].EdgeIndex = SecondEdgeIndex;
	SecondEdge.EdgeIndex = SecondEdgeIndex;
	
	//构建第三个新边
	ThirdEdge.StartIndex = ThirdPointIndex;
	ThirdEdge.EndIndex = Base_Edge.StartIndex;	
	ThirdEdge.OppositeEdgeIndex = -1;
	//ThirdEdge.FaceIndex = -1;
	ThirdEdge.EdgeIndex = -1;

	int FindThirdNeedIndex = FindEdgeinEdges(NeedExpandEdges, ThirdEdge);
	if (FindThirdNeedIndex != -1)
	{
		ThirdEdge.OppositeEdgeIndex = NeedExpandEdges[FindThirdNeedIndex].OppositeEdgeIndex;
		NeedExpandEdges.erase(NeedExpandEdges.begin() + FindThirdNeedIndex);
	}

	//将第三个新边插入到容器中并设置索引
	Point_HE_edges[ThirdEdge.StartIndex].V_HE_Edges.push_back(ThirdEdge);
	ThirdEdgeIndex = Point_HE_edges[ThirdEdge.StartIndex].V_HE_Edges.size() - 1;
	Point_HE_edges[ThirdEdge.StartIndex].V_HE_Edges[ThirdEdgeIndex].EdgeIndex = ThirdEdgeIndex;
	ThirdEdge.EdgeIndex = ThirdEdgeIndex;

	//构建一个新的面片
	HE_Face TempFace;
	TempFace.EdgeIndexs[0] = Base_Edge.StartIndex;
	TempFace.EdgeIndexs[1] = Base_Edge.EndIndex;
	TempFace.EdgeIndexs[2] = ThirdPointIndex;

#ifdef ShowViewer
	//if (InputCloud->points.size() == 115)
	//{
	//	Viewer->addLine(InputCloud->points[TempFace.EdgeIndexs[0]],
	//		InputCloud->points[TempFace.EdgeIndexs[1]], 0, 255, 0,
	//		"T" + StringBase::IntToStr(TempFace.EdgeIndexs[1]) + StringBase::ClockValue());
	//	Viewer->addLine(InputCloud->points[TempFace.EdgeIndexs[1]],
	//		InputCloud->points[TempFace.EdgeIndexs[2]], 0, 255, 0,
	//		"T" + StringBase::ClockValue() + StringBase::IntToStr(TempFace.EdgeIndexs[2]));
	//	Viewer->addLine(InputCloud->points[TempFace.EdgeIndexs[2]],
	//		InputCloud->points[TempFace.EdgeIndexs[0]], 0, 255, 0,
	//		StringBase::ClockValue() + "T" + StringBase::IntToStr(TempFace.EdgeIndexs[0]));
	//}
#endif

	//添加新的面片
	T_Faces.push_back(TempFace);
	int TempFaceIndex = T_Faces.size() - 1;

	/* 对三角形的每一个顶点的角度更新赋值，*/
	//if (UsePointSumAngle)
	{
		//SumAngle[TempFace.EdgeIndexs[0]] = SumAngle[TempFace.EdgeIndexs[0]] -
		Point_HE_edges[TempFace.EdgeIndexs[0]].SumAnlge = Point_HE_edges[TempFace.EdgeIndexs[0]].SumAnlge -
			GeometryBase::AngleValueOfThreePoints(
				InputCloud->points[TempFace.EdgeIndexs[1]].x, InputCloud->points[TempFace.EdgeIndexs[1]].y,
				InputCloud->points[TempFace.EdgeIndexs[0]].x, InputCloud->points[TempFace.EdgeIndexs[0]].y,
				InputCloud->points[TempFace.EdgeIndexs[2]].x, InputCloud->points[TempFace.EdgeIndexs[2]].y);
		//SumAngle[TempFace.EdgeIndexs[1]] = SumAngle[TempFace.EdgeIndexs[1]] -
		Point_HE_edges[TempFace.EdgeIndexs[1]].SumAnlge = Point_HE_edges[TempFace.EdgeIndexs[1]].SumAnlge -
			GeometryBase::AngleValueOfThreePoints(
				InputCloud->points[TempFace.EdgeIndexs[0]].x, InputCloud->points[TempFace.EdgeIndexs[0]].y,
				InputCloud->points[TempFace.EdgeIndexs[1]].x, InputCloud->points[TempFace.EdgeIndexs[1]].y,
 				InputCloud->points[TempFace.EdgeIndexs[2]].x, InputCloud->points[TempFace.EdgeIndexs[2]].y);
		
		Point_HE_edges[TempFace.EdgeIndexs[2]].SumAnlge = Point_HE_edges[TempFace.EdgeIndexs[2]].SumAnlge -
			GeometryBase::AngleValueOfThreePoints(
				InputCloud->points[TempFace.EdgeIndexs[0]].x, InputCloud->points[TempFace.EdgeIndexs[0]].y,
				InputCloud->points[TempFace.EdgeIndexs[2]].x, InputCloud->points[TempFace.EdgeIndexs[2]].y,
				InputCloud->points[TempFace.EdgeIndexs[1]].x, InputCloud->points[TempFace.EdgeIndexs[1]].y);

		//if (TempFace.EdgeIndexs[0] == 94 || TempFace.EdgeIndexs[1] == 94
		//	|| TempFace.EdgeIndexs[2] == 94)
		//{
		//	cout << TempFace.EdgeIndexs[0] <<" SumAnlge:"<< Point_HE_edges[TempFace.EdgeIndexs[0]].SumAnlge<<endl;
		//	cout << TempFace.EdgeIndexs[1] << " SumAnlge:" << Point_HE_edges[TempFace.EdgeIndexs[1]].SumAnlge << endl;
		//	cout << TempFace.EdgeIndexs[2] << " SumAnlge:" << Point_HE_edges[TempFace.EdgeIndexs[2]].SumAnlge << endl <<endl;
		//}
		
		if ((Point_HE_edges[TempFace.EdgeIndexs[0]].SumAnlge) < EPSM6)
			//if (abs(SumAngle[OneIndex]) == 0)
		{	//如果删除这个点，就会使得定位的时候搜索
			//CopyIndexCloud->points[OneIndex].Category = 0;			
			int TempIndex = GetTempIndexFromCopyIndexCloud(TempFace.EdgeIndexs[0]);
			if (-1 != TempIndex)
				CopyIndexCloud->points.erase(CopyIndexCloud->points.begin() + TempIndex);
			//CopyIndexCloud->points[TempIndex].Category = 0;							
		}
				
		if ((Point_HE_edges[TempFace.EdgeIndexs[1]].SumAnlge) < EPSM6)			
		{
			//CopyIndexCloud->points[TwoIndex].Category = 0;			
			int TempIndex = GetTempIndexFromCopyIndexCloud(TempFace.EdgeIndexs[1]);
			if (-1 != TempIndex)
				CopyIndexCloud->points.erase(CopyIndexCloud->points.begin() + TempIndex);
		}
		
		//if ((SumAngle[TempFace.EdgeIndexs[2]]) < EPSM6) //2015.10.29 日修改 带绝对值后容易出错		
		if ((Point_HE_edges[TempFace.EdgeIndexs[2]].SumAnlge) < EPSM6)
		{
			//CopyIndexCloud->points[ThreeIndex].Category = 0;			
			int TempIndex = GetTempIndexFromCopyIndexCloud(TempFace.EdgeIndexs[2]);
			if (-1 != TempIndex)
				CopyIndexCloud->points.erase(CopyIndexCloud->points.begin() + TempIndex);
		}
	}

	///*
	//边的面索引赋值
	//Point_HE_edges[Base_Edge.StartIndex].V_HE_Edges[BaseEdgeIndex].FaceIndex = TempFaceIndex;

	//边的逆向边索引赋值
	if (Point_HE_edges[Base_Edge.StartIndex].V_HE_Edges[BaseEdgeIndex].OppositeEdgeIndex != -1)
	{	
		int TempIndex = Point_HE_edges[Base_Edge.StartIndex].V_HE_Edges[BaseEdgeIndex].OppositeEdgeIndex;
		Point_HE_edges[Base_Edge.EndIndex].V_HE_Edges[TempIndex].OppositeEdgeIndex = BaseEdgeIndex;
	}
	
	//Point_HE_edges[SecondEdge.StartIndex].V_HE_Edges[SecondEdgeIndex].FaceIndex = TempFaceIndex;
	//Point_HE_edges[ThirdEdge.StartIndex].V_HE_Edges[ThirdEdgeIndex].FaceIndex = TempFaceIndex;

	//边次序赋值
	Point_HE_edges[Base_Edge.StartIndex].V_HE_Edges[BaseEdgeIndex].NextEdgeIndex = SecondEdgeIndex;
	Point_HE_edges[SecondEdge.StartIndex].V_HE_Edges[SecondEdgeIndex].NextEdgeIndex = ThirdEdgeIndex;
	Point_HE_edges[ThirdEdge.StartIndex].V_HE_Edges[ThirdEdgeIndex].NextEdgeIndex = BaseEdgeIndex;
	//*/

	//构建两个待扩展边并写入

	if (Point_HE_edges[ThirdPointIndex].SumAnlge < EPSM6)
		return;

	//构建第二个新边 对应的 待扩展边
	//如果第二个新边是边界边，则其反向边不拓展 或者已经在待扩展列表中出现，则其反向边已经生成了三角形，或者该点角度和已经用尽
	//if (((-1 == IsBorderEdge(SecondEdge)) || FindSecondNeedIndex == -1) &&
	if (((!IsBorderEdge(SecondEdge))) && FindSecondNeedIndex == -1 &&
		!(Point_HE_edges[ThirdPointIndex].SumAnlge < EPSM6 ||
			Point_HE_edges[Base_Edge.EndIndex].SumAnlge < EPSM6))
	{
		HE_Edge SecondNeedEdge;
		SecondNeedEdge.StartIndex = ThirdPointIndex;
		SecondNeedEdge.EndIndex = Base_Edge.EndIndex;
		SecondNeedEdge.EdgeIndex = -1;
		//SecondNeedEdge.FaceIndex = -1;
		SecondNeedEdge.OppositeEdgeIndex = SecondEdgeIndex;

		//若已构成三角形，则无需扩展
		int FindNeedIndex = FindEdgeinEdges(Point_HE_edges[SecondEdge.EndIndex].V_HE_Edges, SecondNeedEdge);
		if (FindNeedIndex == -1)
		{
			if (PriorityIndex == SecondNeedEdge.StartIndex || PriorityIndex == SecondNeedEdge.EndIndex)
				NeedExpandEdges.insert(NeedExpandEdges.begin(), SecondNeedEdge);
			else
				NeedExpandEdges.push_back(SecondNeedEdge);

		}
		else//若已经构成三角形，则复制相反边
		{
			Point_HE_edges[SecondEdge.EndIndex].V_HE_Edges[FindNeedIndex].OppositeEdgeIndex = SecondEdgeIndex;
			Point_HE_edges[SecondEdge.StartIndex].V_HE_Edges[SecondEdgeIndex].OppositeEdgeIndex = FindNeedIndex;
		}
	} 

	//构建第三个新边 对应的 待扩展边 
	
	//如果第二个新边是边界边，则其反向边不拓展 或者已经在待扩展列表中出现，则其反向边已经生成了三角形，或者该点角度和已经用尽
	//if ((-1 == IsBorderEdge(ThirdEdge) || FindThirdNeedIndex == -1) && 
	if ((!IsBorderEdge(ThirdEdge)) && FindThirdNeedIndex == -1 &&
		!(Point_HE_edges[Base_Edge.StartIndex].SumAnlge < EPSM6 ||
		Point_HE_edges[ThirdPointIndex].SumAnlge < EPSM6) )
	{
		HE_Edge ThirdNeedEdge;
		ThirdNeedEdge.StartIndex = Base_Edge.StartIndex;
		ThirdNeedEdge.EndIndex = ThirdPointIndex;
		ThirdNeedEdge.EdgeIndex = -1;
		//ThirdNeedEdge.FaceIndex = -1;
		ThirdNeedEdge.OppositeEdgeIndex = ThirdEdgeIndex;

		//若已构成三角形，则无需扩展
		//int FindNeedIndex = FindEdgeinEdges(Point_HE_edges[ThirdEdge.EndIndex].V_HE_Edges, ThirdNeedEdge);
		int FindNeedIndex = FindEdgeinEdges(Point_HE_edges[ThirdNeedEdge.StartIndex].V_HE_Edges, 
											ThirdNeedEdge);
		if (FindNeedIndex == -1)
		{
			//如果有则删除，无则添加
			//FindNeedIndex = FindEdgeinEdges(NeedExpandEdges, ThirdNeedEdge);
			//if (FindNeedIndex == -1)
			{
				//cout << "新增扩展边！" << endl;
				if (PriorityIndex == ThirdNeedEdge.StartIndex
					|| PriorityIndex == ThirdNeedEdge.EndIndex)
					NeedExpandEdges.insert(NeedExpandEdges.begin(), ThirdNeedEdge);
				else		
					NeedExpandEdges.push_back(ThirdNeedEdge);
			}
			//else
			//{
			//	//需要补充
			//	cout << "在待扩展边出现！" << endl;
			//	NeedExpandEdges.erase(NeedExpandEdges.begin() + FindNeedIndex);
			//}
		} 
		else//若已经构成三角形，则复制相反边
		{
			Point_HE_edges[ThirdNeedEdge.StartIndex].V_HE_Edges[FindNeedIndex].OppositeEdgeIndex = ThirdEdgeIndex;
			Point_HE_edges[ThirdNeedEdge.EndIndex].V_HE_Edges[ThirdEdgeIndex].OppositeEdgeIndex = FindNeedIndex;
		}
	}

	//InitialPriorityIndex
	//如果不是从凸包边开始，则第一个优先点形成的边还需要逆向扩展
	if (InitialPriorityIndex == PriorityIndex && IsExpandReverseEdge)
	{
		HE_Edge Base_Reverse_Edge;
		Base_Reverse_Edge.StartIndex = Base_Edge.EndIndex;
		Base_Reverse_Edge.EndIndex = Base_Edge.StartIndex;
		Base_Reverse_Edge.OppositeEdgeIndex = Base_Edge.EdgeIndex;
		Base_Reverse_Edge.EdgeIndex = -1;
		//Base_Reverse_Edge.FaceIndex = -1;

		//若已构成三角形，则无需扩展
		int FindNeedIndex = FindEdgeinEdges(Point_HE_edges[Base_Reverse_Edge.StartIndex].V_HE_Edges,
			Base_Reverse_Edge);
		if (FindNeedIndex == -1)
		{
			//如果有则删除，无则添加
			//FindNeedIndex = FindEdgeinEdges(NeedExpandEdges, Base_Reverse_Edge);
			//if (FindNeedIndex == -1)
			{
				//cout << "新增扩展边！" << endl;
				NeedExpandEdges.insert(NeedExpandEdges.begin(), Base_Reverse_Edge);
			}
			//else
			//{
			//	cout << "在待扩展边出现！" << endl;
			//	NeedExpandEdges.erase(NeedExpandEdges.begin() + FindNeedIndex);
			//}
		}
		else//若已经构成三角形，则复制相反边
		{
			Point_HE_edges[Base_Reverse_Edge.StartIndex].V_HE_Edges[FindNeedIndex].OppositeEdgeIndex = Base_Edge.EdgeIndex;
			Point_HE_edges[Base_Reverse_Edge.EndIndex].V_HE_Edges[Base_Edge.EdgeIndex].OppositeEdgeIndex = FindNeedIndex;
		}
	}
}


int CDelaunayGrowthWithHalfEdge::GetTempIndexFromCopyIndexCloud(int Index)
{
	int ReturnIndex = -1;
	for (int i = 0; i < CopyIndexCloud->points.size(); i++)
	{
		if (CopyIndexCloud->points[i].Index == Index)
		{
			ReturnIndex = i;
			break;
		}
	}
	return ReturnIndex;
}

//2015.07.23 点的角度和初始化
void CDelaunayGrowthWithHalfEdge::Initial()
{
	SumAngle.clear(); //2016.01.23 首先清空所有
	Point_HE_edges.clear();
	CopyIndexCloud.reset(new pcl::PointCloud<PointXYZRGBIndex>);
	PointBase::PointXYZRGBToPointXYZRGBIndex(InputCloud, CopyIndexCloud);
	
	//凸包还是需要计算，因为需要根据凸包来确定凸包点的角度和，以移除凸包点
	if (HullPointsIndex.size() == 0)
		CalcConvexHull();

	if (UseCout)
		cout << "InputCloud, Size:" << InputCloud->points.size() << endl;
	
	if (UseCout)
		cout << "HullPointsIndex, Size:" << HullPointsIndex.size() << endl;

	//默认是不使用这个值的
	InitialPriorityIndex = -10;

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		P_HE_Edge Point_Edges;
		Point_Edges.DivideType = -1;
		Point_Edges.IsGuidPoint = false;
		
		Point_Edges.SumAnlge = 2 * M_PI;
		Point_HE_edges.push_back(Point_Edges);
	}	

	//2016.01.29 调整为取余的方式进行	
	int Hulln = HullPointsIndex.size();
	//凸包点不是360度

	//设置OMP的线程个数
	int CoreNumbers = omp_get_num_procs();
	//#pragma omp parallel for num_threads(2 * CoreNumbers - 1)
	for (int i = 0; i < Hulln; i++)
	{
		//SumAngle[HullPointsIndex[i]] = GeometryBase::AngleValueOfThreePoints(
		Point_HE_edges[HullPointsIndex[i]].SumAnlge = GeometryBase::AngleValueOfThreePoints(
			InputCloud->points[HullPointsIndex[(i + Hulln - 1) % Hulln]].x, 
			InputCloud->points[HullPointsIndex[(i + Hulln - 1) % Hulln]].y,
			InputCloud->points[HullPointsIndex[i]].x, InputCloud->points[HullPointsIndex[i]].y,
			InputCloud->points[HullPointsIndex[(i + 1) % Hulln]].x, 
			InputCloud->points[HullPointsIndex[(i + 1) % Hulln]].y);
	}
}

//2020.05.26 是否是边界边
bool CDelaunayGrowthWithHalfEdge::IsBorderEdge(HE_Edge Base_Edge)
{
	int FindIndex = FindEdgeinEdges(BorderEdges, Base_Edge);

	if (FindIndex == -1)
		return false;
	else
		return true;	
}

//2020.06.12 计算凸包
void CDelaunayGrowthWithHalfEdge::CalcConvexHull()
{
	//////开始计时
	time_t c_start, c_end;

	//c_start = clock();
	//获取凸包点 ///
	CContourAndConvexHull<pcl::PointXYZRGB> InPutCloudHull;
	InPutCloudHull.SetInputs(InputCloud);

	InPutCloudHull.GetPointsConvexHull(HullPointsIndex);	
	//InPutCloudHull.GetConvexHullOneByOne(HullPointsIndex);

	//c_end = clock();
	//cout << "凸包计算耗时: "<< " times: " << difftime(c_end, c_start) << " ms" << endl;

	HullPoints->points.clear();
	for (int i = 0; i < HullPointsIndex.size(); i++)
	{
		HullPoints->points.push_back(InputCloud->points[HullPointsIndex[i]]);
	}
	////PointBase::SavePCDToFileName(HullPoints, "i:\\PlanePoints\\ConvexHull.pcd");
}

//2020.06.12 计算边界边的端点的角度和
void CDelaunayGrowthWithHalfEdge::RefreshBorderEdgePointAngle()
{

}

//以该点为中心进行构建，直至无法扩展 2020.06.22
void CDelaunayGrowthWithHalfEdge::PerformWithOnePoint(int TempCurPointIndex)
{
	if (Point_HE_edges[TempCurPointIndex].V_HE_Edges.size() > 0)
	{
		NeedExpandEdges.clear();
		for (int i = 0; i < Point_HE_edges[TempCurPointIndex].V_HE_Edges.size(); i++)
		{
			PriorityIndex = TempCurPointIndex;
			//扩展从该点出发的边，
			HE_Edge TempNeedExpandEdge;
			TempNeedExpandEdge.StartIndex = Point_HE_edges[TempCurPointIndex].V_HE_Edges[i].EndIndex;
			TempNeedExpandEdge.EndIndex = Point_HE_edges[TempCurPointIndex].V_HE_Edges[i].StartIndex;
			TempNeedExpandEdge.OppositeEdgeIndex = i;
			//TempNeedExpandEdge.FaceIndex = -1;
			TempNeedExpandEdge.NextEdgeIndex = -1;

			ExpandEdge(TempNeedExpandEdge);

			//还有扩展该点是终点的边

			int NextNextEdgeIndex = Point_HE_edges[TempNeedExpandEdge.StartIndex].
				V_HE_Edges[Point_HE_edges[TempCurPointIndex].V_HE_Edges[i].NextEdgeIndex].NextEdgeIndex;
			int NextNextPointIndex = Point_HE_edges[TempNeedExpandEdge.StartIndex].
				V_HE_Edges[Point_HE_edges[TempCurPointIndex].V_HE_Edges[i].NextEdgeIndex].EndIndex;

			HE_Edge TempNextExpandEdge;
			TempNextExpandEdge.StartIndex = Point_HE_edges[NextNextPointIndex].V_HE_Edges[NextNextEdgeIndex].EndIndex;
			TempNextExpandEdge.EndIndex = Point_HE_edges[NextNextPointIndex].V_HE_Edges[NextNextEdgeIndex].StartIndex;
			TempNextExpandEdge.OppositeEdgeIndex = i;
			//TempNextExpandEdge.FaceIndex = -1;
			TempNextExpandEdge.NextEdgeIndex = -1;

			ExpandEdge(TempNextExpandEdge);

			while (NeedExpandEdges.size() > 0)
			{
				HE_Edge TempExpandEdge;
				//先进先出
				TempExpandEdge = NeedExpandEdges[0];
				NeedExpandEdges.erase(NeedExpandEdges.begin());

				if ((PriorityIndex == TempExpandEdge.StartIndex	|| PriorityIndex == TempExpandEdge.EndIndex))
				{					
					//ForwardMove();
					ExpandEdge(TempExpandEdge);
				}
				else
					break;				
			}
		}
	}
}

CDelaunayGrowthWithHalfEdge::CDelaunayGrowthWithHalfEdge()
{
	InputCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	HullPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	CopyIndexCloud.reset(new pcl::PointCloud<PointXYZRGBIndex>());
	Viewer = NULL;
	AllowAngle = 160;
	//AllowAngle = 170;
	UseCout = false;
}

CDelaunayGrowthWithHalfEdge::~CDelaunayGrowthWithHalfEdge()
{
	//if (Viewer != NULL)
	//	Viewer->removePointCloud("AnchorCloud");
}

void CDelaunayGrowthWithHalfEdge::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr)
{
	InputCloud = PointPtr;	
}


void CDelaunayGrowthWithHalfEdge::performReconstructionWithPriority()
{
	if (InputCloud->points.size() < 3) return;

	Initial();	
	
	HE_Edge TempEdge;
	int TempEdgeIndex = -1;
	int ThirdIndex = -1;
	PriorityIndex = -1;

	//如果边界边有，则从边界边开始拓展
	BorderEdgeExpanded = false;
	if (BorderEdges.size() > 0)
	{
		//处理边界点的角度和
		for (int i = 0; i < BorderEdges.size(); i++)
		{
			Point_HE_edges[BorderEdges[i].StartIndex].SumAnlge = BorderEdges[i].StartAngle;
			Point_HE_edges[BorderEdges[i].EndIndex].SumAnlge = BorderEdges[i].EndAngle;
		}

		//首先把边界边的拓展处理了
		for (int i = 0; i < BorderEdges.size(); i++)
		{
			int TempIndex = ExpandEdge(BorderEdges[i]);
			//cout<<"S:"<< BorderEdges[i].StartIndex<<", E:"<< BorderEdges[i].EndIndex 
			//	<<", T:"<< TempIndex << endl;
		}
		BorderEdgeExpanded = true;		
	}
	else//没有边界边，则从凸包点开始拓展
	{
		TempEdge.StartIndex = HullPointsIndex[0];
		TempEdge.EndIndex = HullPointsIndex[1];
		//TempEdge.FaceIndex = -1;
		TempEdge.NextEdgeIndex = -1;	
		TempEdge.OppositeEdgeIndex = -1;
		TempEdge.EdgeIndex = -1;

		ThirdIndex = FindNextPointByAngleMax(TempEdge);

		PriorityIndex = HullPointsIndex[0];
		AddFace(TempEdge, ThirdIndex);
	}	
	
	while (NeedExpandEdges.size() > 0)
	{
		HE_Edge TempExpandEdge;
		//先进先出
		TempExpandEdge = NeedExpandEdges[0];
		NeedExpandEdges.erase(NeedExpandEdges.begin());

		if (!(PriorityIndex == TempExpandEdge.StartIndex
			|| PriorityIndex == TempExpandEdge.EndIndex))
		{
			//RemovePointsFromCopyIndexCloud(PriorityIndex);
			PriorityIndex = TempExpandEdge.StartIndex;
			ForwardMove();
		}

		ExpandEdge(TempExpandEdge);
	}
		
	//cout<<"边的长度："<< Point_HE_edges.size()<<endl;

	//if (BorderEdges.size() > 0)
	//{
	//	cout << "SumAngle 94:" << Point_HE_edges[94].SumAnlge << endl;
	//	cout << "SumAngle 357:" << Point_HE_edges[357].SumAnlge << endl;
	//	cout << "SumAngle 384:" << Point_HE_edges[384].SumAnlge << endl;
	//	cout << "CopyIndexCloud Size:" << CopyIndexCloud->points.size() << endl;
	//}
}

int CDelaunayGrowthWithHalfEdge::ExpandEdge(HE_Edge Base_Edge, bool NeedFindEdge) //待扩展到两个顶点
{
	if (Point_HE_edges[Base_Edge.StartIndex].SumAnlge < EPSM6 ||
		Point_HE_edges[Base_Edge.EndIndex].SumAnlge < EPSM6)
		return -1;	

	//if (InputCloud->points.size() == 697)
	//{
	//	if ((Base_Edge.StartIndex == 38 && Base_Edge.EndIndex == 39) ||
	//		(Base_Edge.StartIndex == 39 && Base_Edge.EndIndex == 38))
	//	{
	//		Base_Edge.StartIndex = Base_Edge.StartIndex;
	//	}
	//}


	//需要扩展的半边已经生成了三角形
	//int TempIndex = FindEdgeinEdges(Point_HE_edges[Base_Edge.StartIndex].V_HE_Edges, Base_Edge);
	//if (TempIndex != -1)
	//	return -1;

	int ThirdIndex = FindNextPointByAngleMax(Base_Edge);

	if (ThirdIndex != -1)
		if (InitialPriorityIndex == PriorityIndex)
			AddFace(Base_Edge, ThirdIndex, -1, true);
		else 
			AddFace(Base_Edge, ThirdIndex);

	return ThirdIndex;	
}


int CDelaunayGrowthWithHalfEdge::FindEdgeinEdges(V_HE_Edge HE_Edges, int TempStartIndex, int TempEndIndex)
{
	int FindIndex = -1;
	for (int i = 0; i < HE_Edges.size(); i++)
	{
		if (HE_Edges[i].StartIndex == TempStartIndex && HE_Edges[i].EndIndex == TempEndIndex)
		{
			FindIndex = i;
			break;
		}
	}
	return FindIndex;
}

//2020.05.26 在带扩展列表中寻找是否
int CDelaunayGrowthWithHalfEdge::FindEdgeinEdges(V_HE_Edge HE_Edges, HE_Edge Base_Edge)
{
	int FindIndex = -1;
	for (int i = 0; i < HE_Edges.size(); i++)
	{
		if (HE_Edges[i].StartIndex == Base_Edge.StartIndex && HE_Edges[i].EndIndex == Base_Edge.EndIndex)
		{
			FindIndex = i;
			break;
		}
	}
	return FindIndex;
}

//将具有优先权的边移动到前面来
void CDelaunayGrowthWithHalfEdge::ForwardMove()
{
	int j = 0;

	for (int i = 1; i < NeedExpandEdges.size(); i++)
	{
		if (NeedExpandEdges[i].StartIndex == PriorityIndex 
			|| NeedExpandEdges[i].EndIndex == PriorityIndex)
		{
			swap(NeedExpandEdges[j], NeedExpandEdges[i]);
			j++;
		}
	}
}

//设置边界边 2020.05.26
void CDelaunayGrowthWithHalfEdge::SetOuterBorderEdges(V_HE_Edge BorderEdgesValue)
{
	if (BorderEdgesValue.size() == 0) return;

	BorderEdges.clear();
	BorderEdges.insert(BorderEdges.begin(), 
		BorderEdgesValue.begin(), BorderEdgesValue.end());
}

void CDelaunayGrowthWithHalfEdge::OutputMesh(pcl::PolygonMesh & MeshOut)
{
	MeshOut.polygons.clear();
	
	if (InputCloud->points.size() < 3) return;

	pcl::PCLPointCloud2 TempPointCloud2;
	pcl::toPCLPointCloud2(*InputCloud, TempPointCloud2);
	MeshOut.header = InputCloud->header;
	MeshOut.cloud = TempPointCloud2;

	for (int i = 0; i < T_Faces.size(); i++)
	{
		pcl::Vertices TempVertices;
		TempVertices.vertices.push_back(T_Faces[i].EdgeIndexs[0]);
		TempVertices.vertices.push_back(T_Faces[i].EdgeIndexs[1]);
		TempVertices.vertices.push_back(T_Faces[i].EdgeIndexs[2]);
		MeshOut.polygons.push_back(TempVertices);
	}	
}

//输出未闭合的点的索引及相对应的边界信息 2020.06.22
void CDelaunayGrowthWithHalfEdge::OutPutPointAngleAndEdges(
	Point_HE_Edge & CurEdges)
{	
	CurEdges.clear();
	CurEdges.insert(CurEdges.end(), Point_HE_edges.begin(), Point_HE_edges.end());
	   
	if (CurEdges.size() == 0) return;

	#pragma omp parallel for
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		CurEdges[i].SumAnlge = 2 * M_PI;		
	}	
	
	int Hulln = HullPointsIndex.size();
	
	#pragma omp parallel for 
	for (int i = 0; i < Hulln; i++)
	{
		//SumAngle[HullPointsIndex[i]] = GeometryBase::AngleValueOfThreePoints(
		CurEdges[HullPointsIndex[i]].SumAnlge = GeometryBase::AngleValueOfThreePoints(
			InputCloud->points[HullPointsIndex[(i + Hulln - 1) % Hulln]].x,
			InputCloud->points[HullPointsIndex[(i + Hulln - 1) % Hulln]].y,
			InputCloud->points[HullPointsIndex[i]].x, 
			InputCloud->points[HullPointsIndex[i]].y,
			InputCloud->points[HullPointsIndex[(i + 1) % Hulln]].x,
			InputCloud->points[HullPointsIndex[(i + 1) % Hulln]].y);
	}

	//根据剩余角度计算已使用角度，2020.06.22
	#pragma omp parallel for 
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		//如果有边，则更新，
		if (Point_HE_edges[i].V_HE_Edges.size() > 0)
			CurEdges[i].SumAnlge -= Point_HE_edges[i].SumAnlge;
		else
			CurEdges[i].SumAnlge = 0;
	}

}





