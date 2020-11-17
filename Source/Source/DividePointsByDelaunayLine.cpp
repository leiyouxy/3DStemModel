#include "DividePointsByDelaunayLine.h"

void CDividePointsByDelaunayLine::GetDirections()
{
	double a, b, c;
	Eigen::Matrix3f EigenVector;		//��������
	Eigen::Vector3f EigenValue(3);

	CenterPoint = GeometryBase::GetPointsTangentPlane(InputCloud, EigenVector, EigenValue);

	MaxDirection.x = EigenVector(6);
	MaxDirection.y = EigenVector(7);
	MaxDirection.z = EigenVector(8);

	SecondDirection.x = EigenVector(3);
	SecondDirection.y = EigenVector(4);
	SecondDirection.z = EigenVector(5);

	pcl::PointXYZRGB LineEnd = GeometryBase::GetPointAlongLine(SecondDirection,
		CenterPoint, 5000);
		
	GeometryBase::PointsProjectToLine(InputCloud, InputCloudInLine, CenterPoint, LineEnd);

	//PointBase::SetPointColor(HullPointsInLine, ColorBase::YellowColor);
	//PointBase::ShowPointXYZRGB(Viewer, HullPointsInLine, "HullPointsInLineSec", 5);

	//��X��Ϊ��׼ Ѱ��  MaxDirection �����ϵļ�ֵ��
	if (abs(SecondDirection.x) > abs(SecondDirection.y))
	{
		double XMin = InputCloudInLine->points[0].x, XMax = InputCloudInLine->points[0].x;
		int XMinIndex = 0, XMaxIndex = 0;

		for (int i = 1; i < InputCloudInLine->points.size(); i++)
		{
			if (InputCloudInLine->points[i].x > XMax)
			{
				XMax = InputCloudInLine->points[i].x;
				XMaxIndex = i;
			}

			if (InputCloudInLine->points[i].x < XMin)
			{
				XMin = InputCloudInLine->points[i].x;
				XMinIndex = i;
			}
		}
		SecDirectionStartPoint = InputCloudInLine->points[XMinIndex];
		SecDirectionEndPoint = InputCloudInLine->points[XMaxIndex];
	}
	else if (abs(SecondDirection.x) < abs(SecondDirection.y))
	{
		double YMin = InputCloudInLine->points[0].y, YMax = InputCloudInLine->points[0].y;
		int YMinIndex = 0, YMaxIndex = 0;

		for (int i = 1; i < InputCloudInLine->points.size(); i++)
		{
			if (InputCloudInLine->points[i].y > YMax)
			{
				YMax = InputCloudInLine->points[i].y;
				YMaxIndex = i;
			}

			if (InputCloudInLine->points[i].y < YMin)
			{
				YMin = InputCloudInLine->points[i].y;
				YMinIndex = i;
			}
		}
		SecDirectionStartPoint = InputCloudInLine->points[YMinIndex];
		SecDirectionEndPoint = InputCloudInLine->points[YMaxIndex];
	}
}

void CDividePointsByDelaunayLine::HalfDivide(
	vector<DiviedPointsAndBorderEdge> & OutPointsAndBorderEdges,
	pcl::PolygonMesh & GlobalMesh,	
	bool ShowGuidLineValue, bool ShowGuidPolyLineValue, bool ShowBorderEdgeValue)
{
	ShowGuidLine = ShowGuidLineValue;
	ShowGuidPolyLine = ShowGuidPolyLineValue;
	ShowBorderEdge = ShowBorderEdgeValue;
		
	//if (VectorBaseInt.FindIndexInVector(LocalSetIndexs, 7314) >= 0 &&
	//	VectorBaseInt.FindIndexInVector(LocalSetIndexs, 574) >= 0)
	//if (VectorBaseInt.FindIndexInVector(LocalSetIndexs, 9814) >= 0 &&
	//	VectorBaseInt.FindIndexInVector(LocalSetIndexs, 6973) >= 0)
	//if (VectorBaseInt.FindIndexInVector(LocalSetIndexs, 84) >= 0 &&
	//	VectorBaseInt.FindIndexInVector(LocalSetIndexs, 140) >= 0)
	//{
	//	cout << "��������������������Size:" << LocalSetIndexs.size() << endl;
	//	cout << "��������������������BorderEdges.size:" << BorderEdges.size() << endl;
	//}

	////if (LocalSetIndexs.size() == 115)
	//if (LocalSetIndexs.size() == 2394)
	//if (LocalSetIndexs.size() == 1247)
	//if (LocalSetIndexs.size() == 610 && BorderEdges.size() == 78) // "Rectangle100000.pcd" ʵ����10000����
	//if (LocalSetIndexs.size() == 292 && BorderEdges.size() == 66) // "Rectangle100000.pcd"
	//if (LocalSetIndexs.size() == 292 && BorderEdges.size() == 66) // "Rectangle100000.pcd"
	//if (LocalSetIndexs.size() == 116 && BorderEdges.size() == 37) // "Rectangle100000.pcd"
	//if (LocalSetIndexs.size() == 256 && BorderEdges.size() == 52) // "Rectangle100000.pcd"
	//if (LocalSetIndexs.size() == 554 && BorderEdges.size() == 86) // "Rectangle100000.pcd"
	//if ((LocalSetIndexs.size() == 107 && BorderEdges.size() == 49) 
	//	|| (LocalSetIndexs.size() == 283 && BorderEdges.size() == 78)) // "Rectangle100000.pcd"
	//if ((LocalSetIndexs.size() == 105 && BorderEdges.size() == 33)) // "Rectangle100000.pcd"
	//if ((LocalSetIndexs.size() == 143 && BorderEdges.size() == 59)) // "Rectangle100000.pcd"
	
	/*
	//if ((LocalSetIndexs.size() == 174 && BorderEdges.size() == 71)||
	//	(LocalSetIndexs.size() == 111 && BorderEdges.size() == 32)) // "Rectangle100000.pcd"

		//if ((LocalSetIndexs.size() == 214 && BorderEdges.size() == 49) ) // "PaperUsed.pcd"
	//if ((LocalSetIndexs.size() == 125 && BorderEdges.size() == 34)) // "PaperUsed.pcd"
	if ((LocalSetIndexs.size() == 214 && BorderEdges.size() == 49)) // "PaperUsed.pcd"
	{
		UseCout = true;
		ShowGuidPolyLine = true;
		ShowBorderEdge = true;		
		ShowGuidLine = true;
		ShowOuterGuidPoints = true;
		PointBase::SetPointColor(InputCloud, ColorBase::RedColor);
		PointBase::ShowPointXYZRGB(Viewer, InputCloud, "InputCloud" + StringBase::ClockValue(), 2);
		//PointBase::ShowPointXYZRGBText(Viewer, InputCloud, "", TextScale);
		//cout << "5350:" << VectorBaseInt.FindIndexInVector(LocalSetIndexs, 5350) <<endl;
		//cout << "9510:" << VectorBaseInt.FindIndexInVector(LocalSetIndexs, 9518) << endl;
	}
	else
	{
		UseCout = false;
		ShowGuidPolyLine = false;
		ShowBorderEdge = false;
		ShowGuidLine = false;
		ShowOuterGuidPoints = false;
	}

	//*/

	GetDirections();
	GetHalfGuidStartAndEndPoints();
	GuidPolyLineMesh();
	DividePointsByLine();	

	//�������
	if (LeftCloudIndexs.size() <= 3 && LeftCloudIndexs.size() > 0)
	{
		//cout <<"��������3��"<< endl;
		for (int i = 0; i < LeftCloudIndexs.size(); i++)
		{
			PerformWithOnePoint(LeftCloudIndexs[i]);
		//	Viewer->addText3D(StringBase::IntToStr(LeftCloudIndexs[i]),
		//		GlobalIndexCloud->points[LeftCloudIndexs[i]], TextScale, 0, 0, 255,
		//		"L" + StringBase::IntToStr(LeftCloudIndexs[i]) + StringBase::ClockValue());
		}
		LeftCloudIndexs.clear();
	}
	if (RightCloudIndexs.size() <= 3 && RightCloudIndexs.size() > 0)
	{
		//cout << "�Ҳ������3��" << endl;
		//for (int i = 0; i < RightCloudIndexs.size(); i++)
		//{
		//	Viewer->addText3D(StringBase::IntToStr(RightCloudIndexs[i]),
		//		GlobalIndexCloud->points[RightCloudIndexs[i]], TextScale, 0, 0, 255,
		//		"R" + StringBase::IntToStr(RightCloudIndexs[i]) + StringBase::ClockValue());
		//}
		for (int i = 0; i < RightCloudIndexs.size(); i++)
		{
			PerformWithOnePoint(RightCloudIndexs[i]);
		}
		RightCloudIndexs.clear();
	}

	GetBorderEdge();

	if (UseCout)
	{
		cout << endl;
		cout << "All:" << InputCloud->points.size() << ", No divide Count:" << InputCloud->points.size() -
			LeftCloudIndexs.size() - L_ClosedPointIndexs.size() -
			R_ClosedPointIndexs.size() - RightCloudIndexs.size() - G_GuidPoints[0].Path.size() << endl;
		cout<<"LeftCloud��Count:"<< LeftCloudIndexs.size()<<endl;
		cout << "Left Closure��Count:" << L_ClosedPointIndexs.size() << endl;
		cout << "RightCloud��Count:" << RightCloudIndexs.size() << endl;
		cout << "Right Closure��Count:" << R_ClosedPointIndexs.size() << endl;
		cout << "Path��Count:" << G_GuidPoints[0].Path.size() << endl;		
		cout << "Mesh��Count:" << T_Faces.size() << endl;
		cout << "BorderEdges��Count:" << BorderEdges.size() << endl;
		cout <<endl;
	}
	//�ֱ� G_GuidPoints[0].Path.size() �еĵ���뵽 LeftCloud �� RightCloud

	//for (int i = 0; i < LeftCloudIndexs.size(); i++)
	//{
	//	LeftCloud->points.push_back(GlobalIndexCloud->points[LeftCloudIndexs[i]]);
	//}

	//for (int i = 0; i < RightCloudIndexs.size(); i++)
	//{
	//	RightCloud->points.push_back(GlobalIndexCloud->points[RightCloudIndexs[i]]);
	//}

	OutPointsAndBorderEdges.clear();

	DiviedPointsAndBorderEdge TempDiviedPointsAndBorderEdgeRight;
	
	//TempDiviedPointsAndBorderEdge.OnePartIndexs.reset(new pcl::PointCloud<PointXYZRGBIndex>());
	TempDiviedPointsAndBorderEdgeRight.PartIndexs.clear();
	TempDiviedPointsAndBorderEdgeRight.PartIndexs.insert(TempDiviedPointsAndBorderEdgeRight.PartIndexs.begin(),
		RightCloudIndexs.begin(), RightCloudIndexs.end());
	TempDiviedPointsAndBorderEdgeRight.CurBorderEdges = RightBorderEdges;
	OutPointsAndBorderEdges.push_back(TempDiviedPointsAndBorderEdgeRight);	

	DiviedPointsAndBorderEdge TempDiviedPointsAndBorderEdgeLeft;
	TempDiviedPointsAndBorderEdgeLeft.PartIndexs.clear();
	TempDiviedPointsAndBorderEdgeLeft.PartIndexs.insert(TempDiviedPointsAndBorderEdgeLeft.PartIndexs.begin(),
		LeftCloudIndexs.begin(), LeftCloudIndexs.end());
	TempDiviedPointsAndBorderEdgeLeft.CurBorderEdges = LeftBorderEdges;

	//��Խ�����㼯�Ӽ��ı߽�ߣ�
	//TempDiviedPointsAndBorderEdgeLeft.BridgeEdges = TempBridgeEdges;

	OutPointsAndBorderEdges.push_back(TempDiviedPointsAndBorderEdgeLeft);

	OutputDivideMesh(GlobalMesh);
}

// ��Index��浽�ļ���   2020.06.24
void CDividePointsByDelaunayLine::SaveIndexsToFile(vector<int> Indexs, string FileName)
{
	for (int i = 0; i < Indexs.size(); i++)
	{
		string TempStr = StringBase::IntToStr(Indexs[i]);

		SaveStringToFile(FileName, TempStr);
	}
}

//�� InputPointsAndBorderEdges �����벢�м��� 2020.06.11
void CDividePointsByDelaunayLine::PerformParallel(
	vector<DiviedPointsAndBorderEdge> InputPointsAndBorderEdges, 
	pcl::PolygonMesh & GlobalMesh)
{
	GlobalMesh.polygons.clear();
		
	//#pragma omp parallel for
	for (int i = 0; i < InputPointsAndBorderEdges.size(); i++)	
	{
		//cout << "parallel Triangle "<< i << endl;
		//if (i == 10)
		//	i = i;
		CDelaunayGrowthWithHalfEdge TempDelaunayGrowthWithHalfEdge;
		//TempDelaunayGrowthWithHalfEdge.Viewer = Viewer;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPartPoints (new pcl::PointCloud<pcl::PointXYZRGB>());
				
		for (int j = 0; j < InputPointsAndBorderEdges[i].PartIndexs.size(); j++)
		{
			pcl::PointXYZRGB TempPoint;

			TempPoint.x = GlobalIndexCloud->points[InputPointsAndBorderEdges[i].PartIndexs[j]].x,
			TempPoint.y = GlobalIndexCloud->points[InputPointsAndBorderEdges[i].PartIndexs[j]].y,
			TempPoint.z = GlobalIndexCloud->points[InputPointsAndBorderEdges[i].PartIndexs[j]].z;

			TempPartPoints->points.push_back(TempPoint);
		}
		
		//bool ShowLine = false;

		//if (VectorBaseInt.FindIndexInVector(InputPointsAndBorderEdges[i].PartIndexs, 5350) >= 0 &&
		//	VectorBaseInt.FindIndexInVector(InputPointsAndBorderEdges[i].PartIndexs, 9518) >= 0)
		//{
		//	cout << "��������������������____Size:" << InputPointsAndBorderEdges[i].PartIndexs.size() << endl;
		//	cout << "��������������������____BorderEdges.size:" << InputPointsAndBorderEdges[i].CurBorderEdges.size() << endl;
		//	//ShowLine = true;
		//}	

		{
			PointBase::SetPointColor(TempPartPoints, ColorBaseS[(i+1) % 30]);
			PointBase::ShowPointXYZRGB(Viewer, TempPartPoints, StringBase::IntToStr(i) +
				StringBase::ClockValue(), 5);
			//SaveIndexsToFile(InputPointsAndBorderEdges[i].PartIndexs, 
			//	"I:\\PlanePoints\\" + StringBase::IntToStr(i) + ".txt");
			//PointBase::SavePCDToFileName(TempPartPoints, 
			//	"I:\\PlanePoints\\" + StringBase::IntToStr(i) + ".pcd");
			//PointBase::ShowPointXYZRGBText(Viewer, TempPartPoints, "", 100);
		}

		///*
		V_HE_Edge TempBorderEdges;		
		for (int j = 0; j < InputPointsAndBorderEdges[i].CurBorderEdges.size(); j++)
		{			
			HE_Edge TempEdge = InputPointsAndBorderEdges[i].CurBorderEdges[j];

			int TempStartInGlobal = InputPointsAndBorderEdges[i].CurBorderEdges[j].StartIndex;
			int TempEndInGlobal = InputPointsAndBorderEdges[i].CurBorderEdges[j].EndIndex;

			int TempStartIndexInLocal = VectorBaseInt.FindIndexInVector(InputPointsAndBorderEdges[i].PartIndexs, TempStartInGlobal);
			int TempEndIndexInLocal = VectorBaseInt.FindIndexInVector(InputPointsAndBorderEdges[i].PartIndexs, TempEndInGlobal);
			
			TempEdge.StartIndex = TempStartIndexInLocal;
			TempEdge.EndIndex = TempEndIndexInLocal;
			TempEdge.StartAngle = InputPointsAndBorderEdges[i].CurBorderEdges[j].StartAngle;
			TempEdge.EndAngle = InputPointsAndBorderEdges[i].CurBorderEdges[j].EndAngle;

			TempBorderEdges.push_back(TempEdge);
#ifdef ShowViewer
			//if (ShowLine)
				//Viewer->addLine(GlobalIndexCloud->points[TempStartInGlobal], 
				//	GlobalIndexCloud->points[TempEndInGlobal], 
				//	StringBase::ClockValue() + StringBase::IntToStr(TempStartInGlobal) 
				//	+ StringBase::IntToStr(TempEndInGlobal));
#endif
		}
		//*/

		//���ñ߽��
		
		TempDelaunayGrowthWithHalfEdge.SetOuterBorderEdges(TempBorderEdges);		
		//if (ShowLine)
			//PointBase::SavePCDToFileName(TempPartPoints, "i:\\PlanePoints\\TempPartPoints" + StringBase::IntToStr(i) + ".pcd");
		TempDelaunayGrowthWithHalfEdge.SetInputCloud(TempPartPoints);
		//if (ShowLine)
		TempDelaunayGrowthWithHalfEdge.performReconstructionWithPriority();
		
		pcl::PolygonMesh TempMesh;
		TempDelaunayGrowthWithHalfEdge.OutputMesh(TempMesh);

		TempDelaunayGrowthWithHalfEdge.OutPutPointAngleAndEdges(
			InputPointsAndBorderEdges[i].CurPointEdges);

		InputPointsAndBorderEdges[i].CurMesh.polygons.clear();
		for (int j = 0; j < TempMesh.polygons.size(); j++)
		{
			pcl::Vertices TempVertices;
			TempVertices.vertices.push_back(InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[0]]);
			TempVertices.vertices.push_back(InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[1]]);
			TempVertices.vertices.push_back(InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[2]]);

			InputPointsAndBorderEdges[i].CurMesh.polygons.push_back(TempVertices);			

			///if (ShowLine)
			//{
			//	Viewer->addLine(GlobalIndexCloud->points[InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[0]]],
			//		GlobalIndexCloud->points[InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[1]]], 0, 255, 0,
			//		"L0" + StringBase::ClockValue() + StringBase::IntToStr(InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[0]]));
			//	Viewer->addLine(GlobalIndexCloud->points[InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[1]]],
			//		GlobalIndexCloud->points[InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[2]]], 0, 255, 0,
			//		"L1" + StringBase::ClockValue() + StringBase::IntToStr(InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[1]]));
			//	Viewer->addLine(GlobalIndexCloud->points[InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[0]]],
			//		GlobalIndexCloud->points[InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[2]]], 0, 255, 0,
			//		"L2" + StringBase::ClockValue() + StringBase::IntToStr(InputPointsAndBorderEdges[i].PartIndexs[TempMesh.polygons[j].vertices[2]]));
			//}
		}
	}

	//�����ܽ��
	for (int i = 0; i < InputPointsAndBorderEdges.size(); i++)
	{
		//if ((LocalSetIndexs.size() == 50 && BorderEdges.size() == 31))
		GlobalMesh.polygons.insert(GlobalMesh.polygons.end(),
				InputPointsAndBorderEdges[i].CurMesh.polygons.begin(),
				InputPointsAndBorderEdges[i].CurMesh.polygons.end());

		RefreshPointAngleAndEdgesInGlobal(InputPointsAndBorderEdges[i].PartIndexs,
			InputPointsAndBorderEdges[i].CurPointEdges);
	}

	//���֮ǰ��Mesh��רע����������㼯��Mesh
	///*
	T_Faces.clear();
	for (int i = 0; i < Point_HE_edges.size(); i++)
	{
		if (Point_HE_edges[i].SumAnlge > EPSM6)
		{			
			//cout << "i "<<i<<", SumAnlge:" << Point_HE_edges[i].SumAnlge << endl;
			//Viewer->addText3D(StringBase::IntToStr(i),
			//	GlobalIndexCloud->points[i], TextScale, 255, 255, 0,
			//	StringBase::ClockValue() + StringBase::IntToStr(i));	
			int ThirdIndex = -1;
			for (int j = 0; j < Point_HE_edges[i].V_HE_Edges.size(); j++)
			{
				if (Point_HE_edges[i].V_HE_Edges[j].OppositeEdgeIndex == -1)
				{
					HE_Edge TempExpandEdge;
					TempExpandEdge.StartIndex = Point_HE_edges[i].V_HE_Edges[j].EndIndex;
					TempExpandEdge.EndIndex = Point_HE_edges[i].V_HE_Edges[j].StartIndex;
					TempExpandEdge.OppositeEdgeIndex = j;
					//TempExpandEdge.FaceIndex = -1;
					TempExpandEdge.NextEdgeIndex = -1;

					ThirdIndex = ExpandEdge(TempExpandEdge);
#ifdef ShowViewer
					cout << "UUU: " << i << ", UUUUUU: " << j << endl;
					Viewer->addLine(GlobalIndexCloud->points[Point_HE_edges[i].V_HE_Edges[j].StartIndex],
						GlobalIndexCloud->points[Point_HE_edges[i].V_HE_Edges[j].EndIndex], 255, 255, 0,
						"UUU" + StringBase::ClockValue() + 
						StringBase::IntToStr(Point_HE_edges[i].V_HE_Edges[j].StartIndex) + 
						StringBase::IntToStr(Point_HE_edges[i].V_HE_Edges[j].EndIndex));
#endif
				}
			}
				
		}
	}
	cout << "����������Ƭ����: " << T_Faces.size() << endl;
	OutputDivideMesh(GlobalMesh);
	//*/	
}

//��ȡ���ֵ�����������
void CDividePointsByDelaunayLine::GetHalfGuidStartAndEndPoints()
{
	PairPointIndexs Temp;
	//�ϲ��ڵ�
	double SecDirectionDisSpace = PointDis(SecDirectionStartPoint, SecDirectionEndPoint) * 3.0;

	//�ϲ��ڵ�
	Temp.StartPoint0 = PointBase::GetPointAlongLine(SecondDirection,
		CenterPoint, SecDirectionDisSpace);
	//Temp.StartPoint0.rgba = ColorBase::RedColor;

	//�²��ڵ�
	Temp.EndPoint0 = PointBase::GetPointAlongLine(SecondDirection,
		CenterPoint, SecDirectionDisSpace, false);	
	
	G_GuidPoints.push_back(Temp);
	
	//if ((LocalSetIndexs.size() == 214 && BorderEdges.size() == 49)) // "PaperUsed.pcd"
	//{
	//	PointBase::SetPointColor(InputCloudInLine, ColorBase::RedColor);
	//	PointBase::ShowPointXYZRGB(Viewer, InputCloudInLine, "InputCloudInLine", 10);
	//}

	///* �������㼯��ѡ����ʼ��
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		for (int j = 0; j < G_GuidPoints.size(); j++)
		{
			if (i == 0)
			{
				G_GuidPoints[j].SmallStartDis = 
					pow(PointDis(InputCloud->points[i],	G_GuidPoints[j].StartPoint0), 2)
					//+ pow(PointDis(InputCloudInLine->points[i],	G_GuidPoints[j].StartPoint0), 2);
					+ pow(PointDis(InputCloudInLine->points[i], InputCloud->points[i]), 2);
					+ 0;

				G_GuidPoints[j].StartEdge0 = i;

				G_GuidPoints[j].SmallEndDis = 
					pow(PointDis(InputCloud->points[i],	G_GuidPoints[j].EndPoint0), 2)
					//+ pow(PointDis(InputCloudInLine->points[i], G_GuidPoints[j].EndPoint0),2);
					+ pow(PointDis(InputCloudInLine->points[i], InputCloud->points[i]), 2);
					//+ 0;

				G_GuidPoints[j].EndEdge0 = i;
			}
			else
			{
				double TempAngle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
					G_GuidPoints[j].StartPoint0, InputCloud->points[i],
					G_GuidPoints[j].EndPoint0));

				if (TempAngle < AllowAngle)
					continue;

				//if ((LocalSetIndexs.size() == 214 && BorderEdges.size() == 49)) // "PaperUsed.pcd"
				//{
				//	cout<<i<<endl;
				//}

				//���Ƚ�
				double TempStartDis = 
					pow(PointDis(InputCloud->points[i],	G_GuidPoints[j].StartPoint0), 2)
					//+ pow(PointDis(InputCloudInLine->points[i],	G_GuidPoints[j].StartPoint0), 2);
					+ pow(PointDis(InputCloudInLine->points[i], InputCloud->points[i]), 2);
					//+ 0;

				if (TempStartDis < G_GuidPoints[j].SmallStartDis)
				{
					G_GuidPoints[j].SmallStartDis = TempStartDis;
					G_GuidPoints[j].StartEdge0 = i;
				}

				//�յ�Ƚ�
				double TempEndDis =
					pow(PointDis(InputCloud->points[i], G_GuidPoints[j].EndPoint0), 2)
					//+ pow(PointDis(InputCloudInLine->points[i],	G_GuidPoints[j].EndPoint0), 2);
					+ pow(PointDis(InputCloudInLine->points[i], InputCloud->points[i]), 2);
					//+ 0;

				if (TempEndDis < G_GuidPoints[j].SmallEndDis)
				{
					G_GuidPoints[j].SmallEndDis = TempEndDis;
					G_GuidPoints[j].EndEdge0 = i;
				}
			}
		}
	}

	//*/

	/* ����ʼλ����㼯͹�������̫Զ�������������µ�һ��͹����ϵ�� 2020.06.21
	//

	if (HullPointsIndex.size() == 0)
		CalcConvexHull();

	int StartInHullIndex = VectorBaseInt.FindIndexInVector(HullPointsIndex, G_GuidPoints[0].StartEdge0);

	if (StartInHullIndex == -1)//����͹���㼯��
	{
		double SmallDis = PointDis(InputCloud->points[G_GuidPoints[0].StartEdge0],
			InputCloud->points[HullPointsIndex[0]]);
		int SmallIndex = 0;

		for (int i = 1; i < HullPointsIndex.size(); i++)
		{
			double TempDis = PointDis(InputCloud->points[G_GuidPoints[0].StartEdge0],
				InputCloud->points[HullPointsIndex[i]]);
			if (TempDis < SmallDis)
			{
				SmallDis = TempDis;
				SmallIndex = i;
			}
		}

		int NextHullIndex = -1;
		if (PointDis(InputCloud->points[G_GuidPoints[0].StartEdge0],
			InputCloud->points[HullPointsIndex[(SmallIndex - 1 + HullPointsIndex.size()) %
			HullPointsIndex.size()]]) > PointDis(InputCloud->points[G_GuidPoints[0].StartEdge0],
				InputCloud->points[HullPointsIndex[(SmallIndex + 1) %
				HullPointsIndex.size()]]))
			NextHullIndex = (SmallIndex + 1) % HullPointsIndex.size();
		else
			NextHullIndex = (SmallIndex - 1 + HullPointsIndex.size()) % HullPointsIndex.size();

		double TempAngle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
			InputCloud->points[HullPointsIndex[NextHullIndex]],
			InputCloud->points[G_GuidPoints[0].StartEdge0],
			InputCloud->points[HullPointsIndex[SmallIndex]]));
		
		if (TempAngle > 10 && TempAngle < 170)
		{
			G_GuidPoints[0].StartEdge0 = HullPointsIndex[SmallIndex];
			cout << "Start Between Hull Angle:" << TempAngle << endl;
		}
	}

	int EndInHullIndex = VectorBaseInt.FindIndexInVector(HullPointsIndex, G_GuidPoints[0].EndEdge0);

	if (EndInHullIndex == -1)
	{
		double SmallDis = PointDis(InputCloud->points[G_GuidPoints[0].EndEdge0],
			InputCloud->points[HullPointsIndex[0]]);
		int SmallIndex = 0;

		for (int i = 1; i < HullPointsIndex.size(); i++)
		{
			double TempDis = PointDis(InputCloud->points[G_GuidPoints[0].EndEdge0],
				InputCloud->points[HullPointsIndex[i]]);
			if (TempDis < SmallDis)
			{
				SmallDis = TempDis;
				SmallIndex = i;
			}
		}

		int NextHullIndex = -1;
		if (PointDis(InputCloud->points[G_GuidPoints[0].EndEdge0],
			InputCloud->points[HullPointsIndex[(SmallIndex - 1 + HullPointsIndex.size()) %
			HullPointsIndex.size()]]) > PointDis(InputCloud->points[G_GuidPoints[0].EndEdge0],
				InputCloud->points[HullPointsIndex[(SmallIndex + 1) %
				HullPointsIndex.size()]]))
			NextHullIndex = (SmallIndex + 1) % HullPointsIndex.size();
		else
			NextHullIndex = (SmallIndex - 1 + HullPointsIndex.size()) % HullPointsIndex.size();

		double TempAngle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
			InputCloud->points[HullPointsIndex[NextHullIndex]],
			InputCloud->points[G_GuidPoints[0].EndEdge0],
			InputCloud->points[HullPointsIndex[SmallIndex]]));

		if (TempAngle > 10 && TempAngle < 170)
		{
			cout << "End Between Hull Angle:" << TempAngle << endl;
			G_GuidPoints[0].EndEdge0 = HullPointsIndex[SmallIndex];
		}
	}


	//*/
//#ifdef ShowViewer
	//����ʼλ�õ���
	if (Viewer != NULL && ShowGuidLine)
	{
		Viewer->addLine(InputCloud->points[G_GuidPoints[0].StartEdge0],
			InputCloud->points[G_GuidPoints[0].EndEdge0], 255, 255, 0, StringBase::ClockValue());	
	}

	if (Viewer != NULL && ShowOuterGuidPoints)
	{
		Viewer->addText3D("GS",	Temp.StartPoint0, TextScale * 2, 0, 255, 0,
			StringBase::ClockValue() + "GS");
		Viewer->addText3D("GE",	Temp.EndPoint0, TextScale * 2, 0, 255, 0,
			StringBase::ClockValue() + "GE");

		Viewer->addLine(Temp.StartPoint0,
			Temp.EndPoint0, 0, 255, 0, "GE" + StringBase::ClockValue());
	}
//#endif

	for (int j = 0; j < G_GuidPoints.size(); j++)
	{
		G_GuidPoints[j].SmallEndDis = 9.0e20;
		G_GuidPoints[j].EndEdge1 = -1;
		G_GuidPoints[j].StartEdge1 = -1;
	}

	//Ѱ���������յ����ĵ�
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		for (int j = 0; j < G_GuidPoints.size(); j++)
		{
			//���Ƚ�
			double TempStartDis = PointDis(InputCloud->points[i],
				InputCloud->points[G_GuidPoints[j].StartEdge0]);
			if (TempStartDis < G_GuidPoints[j].SmallStartDis && TempStartDis > 0)
			{
				G_GuidPoints[j].SmallStartDis = TempStartDis;
				G_GuidPoints[j].StartEdge1 = i;
			}

			//�յ�Ƚ� 
			double TempEndDis = PointDis(InputCloud->points[i],
				InputCloud->points[G_GuidPoints[j].EndEdge0]);

			if (TempEndDis < G_GuidPoints[j].SmallEndDis && TempEndDis > 0)
			{
				G_GuidPoints[j].SmallEndDis = TempEndDis;
				G_GuidPoints[j].EndEdge1 = i;
			}
		}
	}

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//for (int j = 0; j < G_GuidPoints.size(); j++)
	//{
	//	TempCloud->points.push_back(InputCloud->points[G_GuidPoints[j].StartEdge0]);
	//	TempCloud->points[TempCloud->points.size() - 1].rgba = ColorBase::RedColor;
	//	TempCloud->points.push_back(InputCloud->points[G_GuidPoints[j].EndEdge0]);
	//	TempCloud->points[TempCloud->points.size() - 1].rgba = ColorBase::RedColor;

	//	TempCloud->points.push_back(InputCloud->points[G_GuidPoints[j].StartEdge1]);
	//	TempCloud->points[TempCloud->points.size() - 1].rgba = ColorBase::YellowColor;
	//	//TempCloud->points.push_back(InputCloud->points[G_GuidPoints[j].EndEdge1]);
	//	//TempCloud->points[TempCloud->points.size() - 1].rgba = ColorBase::OrangeColor;
	//}

	//PointBase::SetPointColor(TempCloud, ColorBase::RedColor);
	//PointBase::ShowPointXYZRGB(Viewer, TempCloud, "AnchorCloud", 5);
}

//���������� 2020.06.07
void CDividePointsByDelaunayLine::GuidPolyLineMesh()
{
	Initial();	

	HE_Edge TempEdge;
	
	bool IsExpandFromBorderPointStart = false, IsExpandFromBorderPointEnd = false;
	int GuidPointIndex = 0;	
	PriorityIndex = -1;

	if (BorderEdges.size() > 0)
	{
		//����߽��ĽǶȺ�
		for (int i = 0; i < BorderEdges.size(); i++)
		{
			Point_HE_edges[BorderEdges[i].StartIndex].SumAnlge = BorderEdges[i].StartAngle;
			Point_HE_edges[BorderEdges[i].EndIndex].SumAnlge = BorderEdges[i].EndAngle;

			if (!IsExpandFromBorderPointStart &&
				BorderEdges[i].StartIndex == G_GuidPoints[GuidPointIndex].StartEdge0)
			{
				G_GuidPoints[GuidPointIndex].StartEdge1 = BorderEdges[i].EndIndex;
				IsExpandFromBorderPointStart = true;
			}
			//else if (!IsExpandFromBorderPointStart &&
			//	BorderEdges[i].EndIndex == G_GuidPoints[GuidPointIndex].StartEdge0)
			//{
			//	G_GuidPoints[GuidPointIndex].StartEdge1 = BorderEdges[i].StartIndex;
			//	IsExpandFromBorderPointStart = true;
			//}

			if (!IsExpandFromBorderPointEnd &&
				BorderEdges[i].StartIndex == G_GuidPoints[GuidPointIndex].EndEdge0)
			{
				G_GuidPoints[GuidPointIndex].EndEdge1 = BorderEdges[i].EndIndex;
				IsExpandFromBorderPointEnd = true;
			}
			//else if (!IsExpandFromBorderPointEnd &&
			//	BorderEdges[i].EndIndex == G_GuidPoints[GuidPointIndex].EndEdge0)
			//{
			//	G_GuidPoints[GuidPointIndex].EndEdge1 = BorderEdges[i].StartIndex;
			//	IsExpandFromBorderPointEnd = true;
			//}
		}
	}

	//˵��Ӧ�ô�β����ʼ����Ϊ����λ�ô��Ǳ߽�ߣ�Ϊȷ���ɹ���Ӧ��β��ͷ 2020.06.16
	if (!IsExpandFromBorderPointStart && IsExpandFromBorderPointEnd)
	{
		int TempStart0 = G_GuidPoints[GuidPointIndex].StartEdge0;
		int TempStart1 = G_GuidPoints[GuidPointIndex].StartEdge1;
		
		G_GuidPoints[GuidPointIndex].StartEdge0 = G_GuidPoints[GuidPointIndex].EndEdge0;
		G_GuidPoints[GuidPointIndex].StartEdge1 = G_GuidPoints[GuidPointIndex].EndEdge1;

		G_GuidPoints[GuidPointIndex].EndEdge0 = TempStart0;
		G_GuidPoints[GuidPointIndex].EndEdge1 = TempStart1;
	}
		
	//�ڸ���ָ���ߵ�·���ϣ���¼���Ƕ������Ƕȶ�Ӧ�ĵ������������	
	int NewPriorityIndex = -1;
	pcl::PointXYZRGB GuidStartPoint, GuidEndPoint;

	//if (LocalSetIndexs.size() == 146 && BorderEdges.size() == 41)
	//	cout << ", PriorityIndex" << PriorityIndex << endl;

	//�����ʼ�ߵķ����Ǳ߽�ߣ���ӱ߽�߿�ʼ���������ֲ��ɹ�������
	if (-1 != FindEdgeinEdges(BorderEdges, G_GuidPoints[GuidPointIndex].EndEdge0,
		G_GuidPoints[GuidPointIndex].StartEdge0))
	{
		int TempID = G_GuidPoints[GuidPointIndex].EndEdge0;
		G_GuidPoints[GuidPointIndex].EndEdge0 = G_GuidPoints[GuidPointIndex].StartEdge0;
		G_GuidPoints[GuidPointIndex].StartEdge0 = TempID;
	}
	
	GuidStartPoint = InputCloud->points[G_GuidPoints[GuidPointIndex].StartEdge0];
	GuidEndPoint = InputCloud->points[G_GuidPoints[GuidPointIndex].EndEdge0];
		
	TempEdge.StartIndex = G_GuidPoints[GuidPointIndex].StartEdge0;
	TempEdge.EndIndex = G_GuidPoints[GuidPointIndex].StartEdge1;

	G_GuidPoints[GuidPointIndex].Path.push_back(TempEdge.StartIndex);
	Point_HE_edges[TempEdge.StartIndex].IsGuidPoint = true;

	//TempEdge.FaceIndex = -1;
	TempEdge.NextEdgeIndex = -1;
	TempEdge.OppositeEdgeIndex = -1;
	TempEdge.EdgeIndex = -1;	

	//if (LocalSetIndexs.size() == 2394)
	//	cout <<", PriorityIndex" << PriorityIndex << endl;

	int ThirdIndex = FindNextPointByAngleMax(TempEdge);

	if (ThirdIndex == -1)
	{
		if (UseCout)
			cout<<"��չ��ʼ�߳���"<<endl;
#ifdef ShowViewer
		//Viewer->addLine(InputCloud->points[TempEdge.StartIndex], InputCloud->points[TempEdge.EndIndex], 0, 255, 0,
		//	"Arrow" + StringBase::ClockValue() + StringBase::IntToStr(TempEdge.EndIndex));

		//Viewer->addText3D(StringBase::IntToStr(TempEdge.StartIndex), 
		//	InputCloud->points[TempEdge.StartIndex], TextScale, 0, 255,0,
		//	StringBase::ClockValue() + StringBase::IntToStr(TempEdge.StartIndex));
		//Viewer->addText3D(StringBase::IntToStr(TempEdge.EndIndex),
		//	InputCloud->points[TempEdge.EndIndex], TextScale, 0, 255, 0,
		//	StringBase::ClockValue() + StringBase::IntToStr(TempEdge.EndIndex));
#endif
		return;
	}
	PriorityIndex = TempEdge.StartIndex;
	InitialPriorityIndex = PriorityIndex;	

	AddFace(TempEdge, ThirdIndex, -1, true);

#ifdef ShowViewer
	//if (LocalSetIndexs.size() == 115)
	//if (LocalSetIndexs.size() == 2394)
	//{
	//	Viewer->addText3D("S0",
	//		InputCloud->points[G_GuidPoints[0].StartEdge0],
	//		TextScale, 255, 0, 0, StringBase::IntToStr(0) + StringBase::ClockValue() + "T");
	//	Viewer->addText3D("S1",
	//		InputCloud->points[G_GuidPoints[0].TextScale],
	//		TextScale, 255, 0, 0, StringBase::IntToStr(1) + StringBase::ClockValue() + "T");
	//}
#endif
	//�����ȵ�ǰ��
	ForwardMove();

	bool ToEnd = false;

	while (NeedExpandEdges.size() > 0)
	{
		HE_Edge TempExpandEdge;
		//�Ƚ��ȳ�
		TempExpandEdge = NeedExpandEdges[0];

		//if (LocalSetIndexs.size() == 115)
		//	cout<<"PriorityIndex"<<endl;

		//���ȵ㷢���仯��֮ǰ�����ȵ��Ѿ�ѡ�����
		if (!(PriorityIndex == TempExpandEdge.StartIndex || PriorityIndex == TempExpandEdge.EndIndex))
		{
			//ѡ���µ� PriorityIndex
			NewPriorityIndex = FindPriorityPoint(PriorityIndex,
				G_GuidPoints[GuidPointIndex].StartEdge0, G_GuidPoints[GuidPointIndex].EndEdge0);			

			//��ʼ���Ǳ߽�ߣ����Һ������Ҳ�����һ�����ȵ㣬
			if (NewPriorityIndex == -1 && G_GuidPoints[GuidPointIndex].Path.size() <= 3)
			//if (NewPriorityIndex == -1)
			{
				//Ҳ���Կ��ǵ� PriorityIndex  �Ǳ߽��ʱ��ִ�д˲���
				NewPriorityIndex = FindPriorityPoint(PriorityIndex,
					G_GuidPoints[GuidPointIndex].StartEdge0, 
					G_GuidPoints[GuidPointIndex].EndEdge0, false);

				if (NewPriorityIndex != -1)		
				{
					int M = 0;
					for (int k = 0; k < BorderEdges.size(); k++)
					{
						if (BorderEdges[k].StartIndex == NewPriorityIndex 
							|| BorderEdges[k].EndIndex == NewPriorityIndex)
						{
							NeedExpandEdges.push_back(BorderEdges[k]);
							M++;

							if (M == 2)
								break;
						}
					}
				}				
			}

			//���ȵ��Ѿ��� G_GuidPoints[GuidPointIndex].EndEdge0 �л��� �����ڵ㣬�����ö��Ѿ�������
			if (ToEnd && NewPriorityIndex != G_GuidPoints[GuidPointIndex].EndEdge0)
			{
				//if (UseCout)
				//{
				//	cout << "�˶��Ѿ�����:" << GuidPointIndex << ", ��ʱ���ȵ�����Ϊ��" << NewPriorityIndex << endl;
				//	cout << "�������ߵ����" << G_GuidPoints[GuidPointIndex].Path.size() << endl;
				//	//Viewer->updateCamera();
				//	//cout << endl;
				//}
				
				GuidPointIndex++;

				NeedExpandEdges.clear();

				//���е������߶�������
				if (GuidPointIndex == G_GuidPoints.size())
				{
					break;
				}
				else//ת�� ��һ�������� ����Ѱ��
				{
					ToEnd = false;

					if (UseCout)
						cout << "�µķָ��߿�ʼ:" << GuidPointIndex << endl;

					GuidStartPoint = InputCloud->points[G_GuidPoints[GuidPointIndex].StartEdge0];
					GuidEndPoint = InputCloud->points[G_GuidPoints[GuidPointIndex].EndEdge0];

					HE_Edge TempNewExpandEdge;
					TempNewExpandEdge.StartIndex = G_GuidPoints[GuidPointIndex].StartEdge0;
					TempNewExpandEdge.EndIndex = G_GuidPoints[GuidPointIndex].StartEdge1;

					G_GuidPoints[GuidPointIndex].Path.push_back(TempNewExpandEdge.StartIndex);
					Point_HE_edges[TempNewExpandEdge.StartIndex].IsGuidPoint = true;

					//TempNewExpandEdge.FaceIndex = -1;
					TempNewExpandEdge.OppositeEdgeIndex = -1;
					TempNewExpandEdge.NextEdgeIndex = -1;

					NeedExpandEdges.insert(NeedExpandEdges.begin(), TempNewExpandEdge);
					PriorityIndex = TempNewExpandEdge.StartIndex;
				}
			}

			//˵���Ѿ���δ����
			if (NewPriorityIndex == G_GuidPoints[GuidPointIndex].EndEdge0 
				|| NewPriorityIndex == G_GuidPoints[GuidPointIndex].EndEdge1)
				ToEnd = true;

			if (NewPriorityIndex != -1)
			{
				G_GuidPoints[GuidPointIndex].Path.push_back(NewPriorityIndex);
				Point_HE_edges[NewPriorityIndex].IsGuidPoint = true;

				PriorityIndex = NewPriorityIndex;			
			}		

			ForwardMove();
			TempExpandEdge = NeedExpandEdges[0];
		}

		NeedExpandEdges.erase(NeedExpandEdges.begin());

		ThirdIndex = ExpandEdge(TempExpandEdge);		
	}
	   
	for (int i = 0; i < G_GuidPoints.size(); i++)
	{
		InputCloud->points[G_GuidPoints[i].StartEdge0].rgba = ColorBase::RedColor;
//#ifdef ShowViewer
		if (Viewer != NULL && ShowGuidPolyLine)
		{	
			for (int j = 0; j < G_GuidPoints[i].Path.size(); j++)
			{
				if (j > 0)
					Viewer->addLine(InputCloud->points[G_GuidPoints[i].Path[j - 1]],
						InputCloud->points[G_GuidPoints[i].Path[j]],
						255, 0, 0, StringBase::ClockValue() + StringBase::IntToStr(j));

				Viewer->addText3D("P" + StringBase::IntToStr(j),
					InputCloud->points[G_GuidPoints[i].Path[j]],
					TextScale, 255, 0, 0, StringBase::IntToStr(j) + StringBase::ClockValue() + "T");
			}
		}
//#endif
		//if (Viewer != NULL)
		//	Viewer->addLine(InputCloud->points[G_GuidPoints[i].EndEdge1],
		//		InputCloud->points[G_GuidPoints[i].Path[G_GuidPoints[i].Path.size() - 1]],
		//		255, 0, 0, StringBase::ClockValue());

		//cout << "," << G_GuidPoints[i].EndEdge1 << endl;
	}

	//c_end = clock();
	//cout<<"�ߵĳ��ȣ�"<< Point_HE_edges.size()<<endl;

	//cout << "The CDelaunayGrowthWithHalfEdge: "
	//	<< " times: " << difftime(c_end, c_start) << " ms" << endl;
}

//���ݵ㼯�����䶯���򣬼��㻮�ֵ���ʼ��  2020.06.05����ʱδ��
void CDividePointsByDelaunayLine::GetStartAndEndPointForDividing()
{
	MaxDirection = GeometryBase::GetDirectionVector(InputCloud, 2);
	SecondDirection = GeometryBase::GetDirectionVector(InputCloud, 1);

	pcl::PointXYZRGB LineEnd = GeometryBase::GetPointAlongLine(MaxDirection, InputCloud->points[0], 5000);

	if (HullPoints->points.size() > 0)
		GeometryBase::PointsProjectToLine(HullPoints, HullPointsInLine, InputCloud->points[0], LineEnd);
	else
		GeometryBase::PointsProjectToLine(InputCloud, HullPointsInLine, InputCloud->points[0], LineEnd);

	////��X��Ϊ��׼ Ѱ��  MaxDirection �����ϵļ�ֵ��
	//if (abs(MaxDirection.x) > abs(MaxDirection.y))
	//{
	//	double XMin = HullPointsInLine->points[0].x, XMax = HullPointsInLine->points[0].x;
	//	int XMinIndex = 0, XMaxIndex = 0;

	//	for (int i = 1; i < HullPointsInLine->points.size(); i++)
	//	{
	//		if (HullPointsInLine->points[i].x > XMax)
	//		{
	//			XMax = HullPointsInLine->points[i].x;
	//			XMaxIndex = i;
	//		}

	//		if (HullPointsInLine->points[i].x < XMin)
	//		{
	//			XMin = HullPointsInLine->points[i].x;
	//			XMinIndex = i;
	//		}
	//	}
	//	MaxDirectionStartPoint = HullPointsInLine->points[XMinIndex];
	//	MaxDirectionEndPoint = HullPointsInLine->points[XMaxIndex];
	//}
	//else if (abs(MaxDirection.x) < abs(MaxDirection.y))
	//{
	//	double YMin = HullPointsInLine->points[0].y, YMax = HullPointsInLine->points[0].y;
	//	int YMinIndex = 0, YMaxIndex = 0;

	//	for (int i = 1; i < HullPointsInLine->points.size(); i++)
	//	{
	//		if (HullPointsInLine->points[i].y > YMax)
	//		{
	//			YMax = HullPointsInLine->points[i].y;
	//			YMaxIndex = i;
	//		}

	//		if (HullPointsInLine->points[i].y < YMin)
	//		{
	//			YMin = HullPointsInLine->points[i].y;
	//			YMinIndex = i;
	//		}
	//	}
	//	MaxDirectionStartPoint = HullPointsInLine->points[YMinIndex];
	//	MaxDirectionEndPoint = HullPointsInLine->points[YMaxIndex];
	//}

	//Viewer->addLine(MaxDirectionStartPoint, MaxDirectionEndPoint, "MaxDirectionStartPoint");

	//PointBase::SetPointColor(HullPointsInLine, ColorBase::RedColor);
	//PointBase::ShowPointXYZRGB(Viewer, HullPointsInLine, "HullPointsInLineMax", 5);

	//Ѱ�ҵڶ��������ϵļ�ֵ��
	LineEnd = GeometryBase::GetPointAlongLine(SecondDirection, InputCloud->points[0], 5000);
	if (HullPoints->points.size() > 0)
		GeometryBase::PointsProjectToLine(HullPoints, HullPointsInLine, InputCloud->points[0], LineEnd);
	else
		GeometryBase::PointsProjectToLine(InputCloud, HullPointsInLine, InputCloud->points[0], LineEnd);

	//PointBase::SetPointColor(HullPointsInLine, ColorBase::YellowColor);
	//PointBase::ShowPointXYZRGB(Viewer, HullPointsInLine, "HullPointsInLineSec", 5);

	//��X��Ϊ��׼ Ѱ��  MaxDirection �����ϵļ�ֵ��
	/*if (abs(SecondDirection.x) > abs(SecondDirection.y))
	{
		double XMin = HullPointsInLine->points[0].x, XMax = HullPointsInLine->points[0].x;
		int XMinIndex = 0, XMaxIndex = 0;

		for (int i = 1; i < HullPointsInLine->points.size(); i++)
		{
			if (HullPointsInLine->points[i].x > XMax)
			{
				XMax = HullPointsInLine->points[i].x;
				XMaxIndex = i;
			}

			if (HullPointsInLine->points[i].x < XMin)
			{
				XMin = HullPointsInLine->points[i].x;
				XMinIndex = i;
			}
		}
		SecDirectionStartPoint = HullPointsInLine->points[XMinIndex];
		SecDirectionEndPoint = HullPointsInLine->points[XMaxIndex];
	}
	else if (abs(SecondDirection.x) < abs(SecondDirection.y))
	{
		double YMin = HullPointsInLine->points[0].y, YMax = HullPointsInLine->points[0].y;
		int YMinIndex = 0, YMaxIndex = 0;

		for (int i = 1; i < HullPointsInLine->points.size(); i++)
		{
			if (HullPointsInLine->points[i].y > YMax)
			{
				YMax = HullPointsInLine->points[i].y;
				YMaxIndex = i;
			}

			if (HullPointsInLine->points[i].y < YMin)
			{
				YMin = HullPointsInLine->points[i].y;
				YMinIndex = i;
			}
		}
		SecDirectionStartPoint = HullPointsInLine->points[YMinIndex];
		SecDirectionEndPoint = HullPointsInLine->points[YMaxIndex];
	}	*/
}

void CDividePointsByDelaunayLine::performDividByGuidPoints()
{
	if (InputCloud->points.size() < 3) return;

	//����������
	GuidPolyLineMesh();
}

//���������߽��㼯�еĵ�һ��Ϊ�� 2020.06.08
void CDividePointsByDelaunayLine::DividePointsByLine()
{
	//������������ ���������ε�λ�����ж�
	DividePointsByPolyLine();

	vector<int> UnDivided;
	
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		//////�����жϸõ��Ƿ��Ѿ��бߣ�����бߣ���δ������		

		if (Point_HE_edges[i].V_HE_Edges.size() > 0 && Point_HE_edges[i].DivideType == -1
			&& !Point_HE_edges[i].IsGuidPoint)
		{
			//int FindGuidPathIndex
			//cout<<"��û���ҵ���"<<i<<endl;
			UnDivided.push_back(i);
		}
		else if (Point_HE_edges[i].V_HE_Edges.size() == 0) //�õ�û�бߣ�ֱ���ж�
		{			
			if (i == G_GuidPoints[0].StartEdge0 || i == G_GuidPoints[0].EndEdge0)
				continue;

			double Area = GeometryBase::AreaOfThreePointsIn2D(
				InputCloud->points[G_GuidPoints[0].StartEdge0].x, InputCloud->points[G_GuidPoints[0].StartEdge0].y,
				InputCloud->points[G_GuidPoints[0].EndEdge0].x, InputCloud->points[G_GuidPoints[0].EndEdge0].y,
				InputCloud->points[i].x, InputCloud->points[i].y);
			if (Area > 0)
			{
				Point_HE_edges[i].DivideType = 0;
				//RightCloud->points.push_back(GlobalIndexCloud->points[i]);
				//���ص���ȫ������
				RightCloudIndexs.push_back(LocalSetIndexs[i]);
			}
			else if (Area < 0)
			{
				Point_HE_edges[i].DivideType = 1;
				//LeftCloud->points.push_back(GlobalIndexCloud->points[i]);
				//���ص���ȫ������
				LeftCloudIndexs.push_back(LocalSetIndexs[i]);

			}
			else if (Area == 0)
			{
				if (UseCout)
					cout<<"������ߣ����ʺܵͣ�"<<endl;
			}
		}
	}

	//�������Աߵ�EndIndex��ȷ���ߵ�StartIndex�Ĺ���		
	///*
	int CurIndex = 0;
	int LastSize = UnDivided.size();
	int Count = 0;
	while (UnDivided.size() > 0)
	{		
		Count++;
		//ѭ��50��Ҳû���ҵ�
		if (LastSize == UnDivided.size() && Count > 50)
		{	
#ifdef ShowViewer
			if (UseCout)
				cout << "�е�δ�ָ������ LocalSetIndexs��" << LocalSetIndexs.size() 
					<< ", BorderEdges��"<<BorderEdges.size()<< endl;
#endif
			//if (UseCout)
				for (int i = 0; i < UnDivided.size(); i++)
				{		
					PerformWithOnePoint(UnDivided[i]);
#ifdef ShowViewer
					if (UseCout)
						cout << "δ�ָ�ĵ㣺" << UnDivided[i] << endl;
					//Viewer->addText3D("U"+StringBase::IntToStr(LocalSetIndexs[UnDivided[i]]),
					//	GlobalIndexCloud->points[LocalSetIndexs[UnDivided[i]]], TextScale, 0, 255, 0,
					//	StringBase::IntToStr(LocalSetIndexs[UnDivided[i]]) + StringBase::ClockValue());
#endif
				}		
				break;
		}		

		int TempPointIndex = UnDivided[CurIndex];
		
		//cout << CurIndex <<":"<< Point_HE_edges[TempPointIndex].V_HE_Edges.size()<<":"<< TempPointIndex << endl;

		for (int i = 0; i < Point_HE_edges[TempPointIndex].V_HE_Edges.size(); i++)
		{			
			if (Point_HE_edges[TempPointIndex].DivideType != -1)
			{
				if (CurIndex == UnDivided.size() - 1)
					CurIndex = 0;

				UnDivided.erase(UnDivided.begin() + CurIndex);				
				break;
			}
			
			int TempEndIndex = Point_HE_edges[TempPointIndex].V_HE_Edges[i].EndIndex;

			/*
			//Ĭ��Ϊ�������ߴ�Start ָ�� End Ϊ���� 2020.06.19
			//�������ǳ�ʼ�������ߵ�
			if (TempEndIndex == G_GuidPoints[0].Path[0])
			{
				double TempArea = GeometryBase::AreaOfThreePointsIn2D(
					InputCloud->points[G_GuidPoints[0].Path[0]].x, InputCloud->points[G_GuidPoints[0].Path[0]].y,
					InputCloud->points[G_GuidPoints[0].Path[1]].x, InputCloud->points[G_GuidPoints[0].Path[1]].y,
					InputCloud->points[TempPointIndex].x, InputCloud->points[TempPointIndex].y);
				if (TempArea > 0)
				{
					Point_HE_edges[TempPointIndex].DivideType = 0;
					//RightCloud->points.push_back(InputCloud->points[TempPointIndex]);
					RightCloudIndexs.push_back(LocalSetIndexs[TempPointIndex]);

					//if (ShowGuidPolyLine)
						Viewer->addText3D("R_U_L", InputCloud->points[TempPointIndex], TextScale, 255,
							0, 0, "Line" + StringBase::ClockValue());
					UnDivided.erase(UnDivided.begin() + CurIndex);
					break;
				}
				else if (TempArea < 0)
				{
					Point_HE_edges[TempPointIndex].DivideType = 1;
					//LeftCloud->points.push_back(InputCloud->points[TempPointIndex]);
					LeftCloudIndexs.push_back(LocalSetIndexs[TempPointIndex]);

					//if (ShowGuidPolyLine)
						Viewer->addText3D("L_U_F", InputCloud->points[TempPointIndex], TextScale, 0,
							255, 0, "Line" + StringBase::ClockValue());
					UnDivided.erase(UnDivided.begin() + CurIndex);
					break;
				}

			}
			else if (TempEndIndex == G_GuidPoints[0].Path[G_GuidPoints[0].Path.size() - 1])
			{
				int LastPathIndex = G_GuidPoints[0].Path.size() - 1;

				double TempArea = GeometryBase::AreaOfThreePointsIn2D(
					InputCloud->points[G_GuidPoints[0].Path[LastPathIndex - 1 ]].x, 
					InputCloud->points[G_GuidPoints[0].Path[LastPathIndex - 1]].y,
					InputCloud->points[G_GuidPoints[0].Path[LastPathIndex]].x, 
					InputCloud->points[G_GuidPoints[0].Path[LastPathIndex]].y,
					InputCloud->points[TempPointIndex].x, InputCloud->points[TempPointIndex].y);
				if (TempArea > 0)
				{
					Point_HE_edges[TempPointIndex].DivideType = 0;
					
					RightCloudIndexs.push_back(LocalSetIndexs[TempPointIndex]);

					if (ShowGuidPolyLine)
						Viewer->addText3D("R_U_L", InputCloud->points[TempPointIndex], TextScale, 255,
							0, 0, "Line" + StringBase::ClockValue());
					UnDivided.erase(UnDivided.begin() + CurIndex);
					break;
				}
				else if (TempArea < 0)
				{
					Point_HE_edges[TempPointIndex].DivideType = 1;
					
					LeftCloudIndexs.push_back(LocalSetIndexs[TempPointIndex]);

					//if (ShowGuidPolyLine)
						Viewer->addText3D("L_U_F", InputCloud->points[TempPointIndex], TextScale, 0,
							255, 0, "Line" + StringBase::ClockValue());
					UnDivided.erase(UnDivided.begin() + CurIndex);
					break;
				}

			}
			//*/

			if (Point_HE_edges[TempEndIndex].IsGuidPoint)
				continue;			

			if (Point_HE_edges[TempEndIndex].DivideType == 0)
			{
				Point_HE_edges[TempPointIndex].DivideType = 0;
				//RightCloud->points.push_back(InputCloud->points[TempPointIndex]);
				RightCloudIndexs.push_back(LocalSetIndexs[TempPointIndex]);
#ifdef	ShowViewer			
				if (ShowGuidPolyLine)
					Viewer->addText3D("R", InputCloud->points[TempPointIndex], TextScale, 255,
						0, 0, "Line" + StringBase::ClockValue());
#endif
				UnDivided.erase(UnDivided.begin() + CurIndex);				
				break;
			}
			else if (Point_HE_edges[TempEndIndex].DivideType == 1)
			{
				Point_HE_edges[TempPointIndex].DivideType = 1;
				//LeftCloud->points.push_back(InputCloud->points[TempPointIndex]);
				LeftCloudIndexs.push_back(LocalSetIndexs[TempPointIndex]);
#ifdef ShowViewer
				if (ShowGuidPolyLine)
					Viewer->addText3D("L", InputCloud->points[TempPointIndex], TextScale, 0,
						255, 0, "Line" + StringBase::ClockValue());
#endif
				UnDivided.erase(UnDivided.begin() + CurIndex);				
				break;
			}
			//Viewer->updateCamera();
		}	
		CurIndex++;
		if (CurIndex >= UnDivided.size())
			CurIndex = 0;

		LastSize = UnDivided.size();
	}	

	//*/
}

//���� �������� �γɵ��������жϵ������㼯 2020.06.08
void CDividePointsByDelaunayLine::DividePointsByPolyLine()
{
	L_ClosedPointIndexs.clear();
	R_ClosedPointIndexs.clear();
			
	//����β�����Ķ˵㣬ֱ�Ӹ�����β�ָ����ߵ�λ������������

	//for (int i = 0; i < Point_HE_edges[G_GuidPoints[0].Path[0]].V_HE_Edges.size(); i++)
	//{
	//	int 
	//}


	//�˴�ʵ�ֵ��Ƕ��֣�����ֻ��Ҫѭ�� 0 ����
	for (int i = 1; i < G_GuidPoints[0].Path.size(); i++)
	{
		int GuidEdgeStartIndex = -1, GuidEdgeEndIndex = -1;
		
		GuidEdgeStartIndex = G_GuidPoints[0].Path[i - 1];		
		GuidEdgeEndIndex = G_GuidPoints[0].Path[i];
		
		//����һ���������ߵĵ�����������������������Ƚ���ȷ
		for (int j = 0; j < Point_HE_edges[GuidEdgeStartIndex].V_HE_Edges.size(); j++)
		{			
			if (Point_HE_edges[GuidEdgeStartIndex].V_HE_Edges[j].StartIndex == GuidEdgeStartIndex &&
				Point_HE_edges[GuidEdgeStartIndex].V_HE_Edges[j].EndIndex == GuidEdgeEndIndex) 
			{
				int TempEdgeIndex = Point_HE_edges[GuidEdgeStartIndex].V_HE_Edges[j].NextEdgeIndex;
				int ThirdPointIndex = Point_HE_edges[GuidEdgeEndIndex].V_HE_Edges[TempEdgeIndex].EndIndex;
				
				if (Point_HE_edges[ThirdPointIndex].DivideType != -1)
					continue;
								
				if (Point_HE_edges[ThirdPointIndex].IsGuidPoint)
					continue;

				//Ĭ��Ϊ�������ߴ�Start ָ�� End Ϊ���� 2020.06.19
				double TempArea = GeometryBase::AreaOfThreePointsIn2D(
					InputCloud->points[GuidEdgeStartIndex].x, InputCloud->points[GuidEdgeStartIndex].y,
					InputCloud->points[GuidEdgeEndIndex].x, InputCloud->points[GuidEdgeEndIndex].y,
					InputCloud->points[ThirdPointIndex].x, InputCloud->points[ThirdPointIndex].y);

				if (ThirdPointIndex == 133 )
					ThirdPointIndex = 133;
				
				if (ThirdPointIndex == 96)
					ThirdPointIndex = 96;

				if (TempArea > 0)
				{				
					if (Point_HE_edges[ThirdPointIndex].SumAnlge > EPSM6)
					{
						//RightCloudIndexsΪ ȫ�ֵ�����
						RightCloudIndexs.push_back(LocalSetIndexs[ThirdPointIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("R" + StringBase::IntToStr(ThirdPointIndex), 
								InputCloud->points[ThirdPointIndex], TextScale, 255,
								0, 0, "Line" + StringBase::ClockValue());
					}
					else
					{
						R_ClosedPointIndexs.push_back(LocalSetIndexs[ThirdPointIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("R0" + StringBase::IntToStr(ThirdPointIndex), 
								InputCloud->points[ThirdPointIndex], TextScale, 255,
							0, 0, "Line" + StringBase::ClockValue());
					}					
					
					Point_HE_edges[ThirdPointIndex].DivideType = 0; //Right
										
					DivideBroadcast(ThirdPointIndex, 0);
				}
				else if (TempArea < 0)
				{
					if (Point_HE_edges[ThirdPointIndex].SumAnlge > EPSM6)
					{
						//LeftCloudIndexsΪ ȫ�ֵ�����
						LeftCloudIndexs.push_back(LocalSetIndexs[ThirdPointIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("L" + StringBase::IntToStr(ThirdPointIndex), 
								InputCloud->points[ThirdPointIndex], TextScale, 0,
								255, 0, "Line" + StringBase::ClockValue());
					}
					else
					{
						L_ClosedPointIndexs.push_back(LocalSetIndexs[ThirdPointIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("L0" + StringBase::IntToStr(ThirdPointIndex), 
								InputCloud->points[ThirdPointIndex], TextScale, 0,
							255, 0, "Line" + StringBase::ClockValue());
					}

					Point_HE_edges[ThirdPointIndex].DivideType = 1; //Left

					DivideBroadcast(ThirdPointIndex, 1);
				}
			}
		}
				
		for (int j = 0; j < Point_HE_edges[GuidEdgeEndIndex].V_HE_Edges.size(); j++)
		{
			if (Point_HE_edges[GuidEdgeEndIndex].V_HE_Edges[j].StartIndex == GuidEdgeEndIndex &&
				Point_HE_edges[GuidEdgeEndIndex].V_HE_Edges[j].EndIndex == GuidEdgeStartIndex)
			{
				int TempEdgeIndex = Point_HE_edges[GuidEdgeEndIndex].V_HE_Edges[j].NextEdgeIndex;
				int ThirdPointIndex = Point_HE_edges[GuidEdgeStartIndex].V_HE_Edges[TempEdgeIndex].EndIndex;
				
				if (Point_HE_edges[ThirdPointIndex].DivideType != -1)
					continue;

				if (Point_HE_edges[ThirdPointIndex].IsGuidPoint)
					continue;	

				double TempArea = GeometryBase::AreaOfThreePointsIn2D(
					InputCloud->points[GuidEdgeStartIndex].x, InputCloud->points[GuidEdgeStartIndex].y,
					InputCloud->points[GuidEdgeEndIndex].x, InputCloud->points[GuidEdgeEndIndex].y,
					InputCloud->points[ThirdPointIndex].x, InputCloud->points[ThirdPointIndex].y);
				//cout << GuidEdgeStartIndex << "," << GuidEdgeEndIndex << ","<< ThirdPointIndex << ",SumAnlge:" << Point_HE_edges[ThirdPointIndex].SumAnlge << endl;
				if (TempArea > 0)
				{
					if (Point_HE_edges[ThirdPointIndex].SumAnlge > EPSM6)
					{
						RightCloudIndexs.push_back(LocalSetIndexs[ThirdPointIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("R" + StringBase::IntToStr(ThirdPointIndex), 
								InputCloud->points[ThirdPointIndex], TextScale, 255,
								0, 0, "Line" + StringBase::ClockValue());
					}
					else
					{
						R_ClosedPointIndexs.push_back(LocalSetIndexs[ThirdPointIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("R0" + StringBase::IntToStr(ThirdPointIndex), 
								InputCloud->points[ThirdPointIndex], TextScale, 255,
								0, 0, "Line" + StringBase::ClockValue());
					}

					Point_HE_edges[ThirdPointIndex].DivideType = 0; //Right

					DivideBroadcast(ThirdPointIndex, 0);

				}
				else if (TempArea < 0)
				{
					if (Point_HE_edges[ThirdPointIndex].SumAnlge > EPSM6)
					{
						LeftCloudIndexs.push_back(LocalSetIndexs[ThirdPointIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("L" + StringBase::IntToStr(ThirdPointIndex), 
								InputCloud->points[ThirdPointIndex], TextScale, 0,
								255, 0, "Line" + StringBase::ClockValue());
					}
					else
					{
						L_ClosedPointIndexs.push_back(LocalSetIndexs[ThirdPointIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("L0" + StringBase::IntToStr(ThirdPointIndex), 
								InputCloud->points[ThirdPointIndex], TextScale, 0,
								255, 0, "Line" + StringBase::ClockValue());
					}
					
					Point_HE_edges[ThirdPointIndex].DivideType = 1; //Left

					DivideBroadcast(ThirdPointIndex, 1);
				}
			}
		}		
	}	
}

//�ڹ�����Mesh��������ǰ TempPointIndex �� GuidPath�е� ��Щ���γ������Σ�����ֵ����1��
//������ֵ����1�������������У� 2020.06.09;
vector<int> CDividePointsByDelaunayLine::FindGuidPathPointIndex(int TempPointIndex)
{
	vector<int> Results;
	
	for (int i = 0; i < Point_HE_edges[TempPointIndex].V_HE_Edges.size(); i++)
	{
		int TempEndPointIndex = Point_HE_edges[TempPointIndex].V_HE_Edges[i].EndIndex;
		int TempFindIndex = VectorBaseInt.FindIndexInVector(G_GuidPoints[0].Path, TempEndPointIndex);
		if (TempFindIndex != -1)
		{
			Results.push_back(TempFindIndex);
		}	
	}

	VectorBaseInt.SortVector(Results);

	return Results;
}

//�����˵�ķָ��ϵ������õ����ڱߵ�EndIndex�����������ߣ�����Դ���  2020.06.09
//Type = 0 ����RightCloud��= 1 ����LeftCloud
//�ݹ鷽�������ã����ܻᵼ��ջ���
void CDividePointsByDelaunayLine::DivideBroadcast(int TempPointIndex, int TypeValue)
{	
	//if (Point_HE_edges[TempPointIndex].DivideType == 0 
	//	|| Point_HE_edges[TempPointIndex].DivideType == 1)
	//	return;

	for (int i = 0; i < Point_HE_edges[TempPointIndex].V_HE_Edges.size(); i++)
	{
		int TempEndIndex = Point_HE_edges[TempPointIndex].V_HE_Edges[i].EndIndex;

		if (Point_HE_edges[TempEndIndex].DivideType == -1)
		{
			//int FindIndex = VectorBaseInt.FindIndexInVector(G_GuidPoints[0].Path, TempEndIndex);
		
			if (!Point_HE_edges[TempEndIndex].IsGuidPoint)//�������������ϵĵ�
			{		
				///*// �˴����������õ�
				//����������������������ߵ���β�˵㣬�ҼнǴ���90�ȣ��򲻴���
				int TempNextEdgeIndex = Point_HE_edges[TempPointIndex].V_HE_Edges[i].NextEdgeIndex;
				int TempThirdIndex = Point_HE_edges[TempEndIndex].V_HE_Edges[TempNextEdgeIndex].EndIndex;
				if (TempThirdIndex == G_GuidPoints[0].Path[0]
					|| TempThirdIndex == G_GuidPoints[0].Path[G_GuidPoints[0].Path.size() - 1])
				{
					double TempAngle = GeometryBase::RadianToAngle( GeometryBase::AngleValueOfThreePoints(
						InputCloud->points[TempPointIndex], InputCloud->points[TempThirdIndex],
						InputCloud->points[TempEndIndex]));
					if (TempAngle > 90)
						continue;
				}
				//*/

				//if (TempEndIndex == 778)
				//	TempEndIndex = 778;


				//if (LocalSetIndexs.size() == 2394 && TempEndIndex == 2337)
				//{
				//	cout << "TempPointIndex:" << TempPointIndex << " Type," << TypeValue << endl;
				//}

				if (TypeValue == 0)
				{					
					//RightCloudIndexs.push_back(LocalSetIndexs[TempEndIndex]);

					if (Point_HE_edges[TempEndIndex].SumAnlge > EPSM6)
					{
						//RightCloudIndexsΪ ȫ�ֵ�����
						RightCloudIndexs.push_back(LocalSetIndexs[TempEndIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("R" + StringBase::IntToStr(TempEndIndex), 
								InputCloud->points[TempEndIndex], TextScale, 0,
								255, 0, "Line" + StringBase::ClockValue());
					}
					else
					{
						R_ClosedPointIndexs.push_back(LocalSetIndexs[TempEndIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("R0" + StringBase::IntToStr(TempEndIndex), 
								InputCloud->points[TempEndIndex], TextScale, 0,
								255, 0, "Line" + StringBase::ClockValue());
					}
				
					Point_HE_edges[TempEndIndex].DivideType = 0;
					
					DivideBroadcast(TempEndIndex, TypeValue);
				}
				else if (TypeValue == 1)
				{					
					//LeftCloudIndexs.push_back(LocalSetIndexs[TempEndIndex]);
					if (Point_HE_edges[TempEndIndex].SumAnlge > EPSM6)
					{
						//LeftCloudIndexsΪ ȫ�ֵ�����
						LeftCloudIndexs.push_back(LocalSetIndexs[TempEndIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("L" + StringBase::IntToStr(TempEndIndex), 
								InputCloud->points[TempEndIndex], TextScale, 0,
								255, 0, "Line" + StringBase::ClockValue());
					}
					else
					{
						L_ClosedPointIndexs.push_back(LocalSetIndexs[TempEndIndex]);
						if (ShowGuidPolyLine && UseCout)
							Viewer->addText3D("L0" + StringBase::IntToStr(TempEndIndex), 
								InputCloud->points[TempEndIndex], TextScale, 0,
								255, 0, "Line" + StringBase::ClockValue());
					}

					Point_HE_edges[TempEndIndex].DivideType = 1;

					DivideBroadcast(TempEndIndex, TypeValue);
				}
			}
		}
		//int TempOppositeEdgeIndex = Point_HE_edges[TempPointIndex].V_HE_Edges[i].OppositeEdgeIndex
	}
}

//Type = 0 ��ȡ�Ҳ�߽�ߣ�= 1 ���ȡ���߽�� 2020.06.10 ��ͬʱ���߽�ߵ������滻Ϊ�ֲ�����
//����ı߽���ǵ㼯�������б߽�ߵķ���ߣ�����һ��㼯���ǿ������������εıߣ�
void CDividePointsByDelaunayLine::GetBorderEdge()
{		
	//2020.06.16 ���Ǵ���ָ�������ı߽��
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		//////�����жϸõ��Ƿ��Ѿ��бߣ�����бߣ���δ������		
		if (Point_HE_edges[i].V_HE_Edges.size() > 0)
		{
			for(int j = 0; j < Point_HE_edges[i].V_HE_Edges.size(); j++)
			{				
				//��Ϊ�Ѿ����˶Աߣ����Ǳ߽�ߣ����Բ�������
				if (Point_HE_edges[i].V_HE_Edges[j].OppositeEdgeIndex != -1)
					continue;

				//��LocalSetIndexs�е�������
				int TempStartIndex = Point_HE_edges[i].V_HE_Edges[j].StartIndex;
				int TempEndIndex = Point_HE_edges[i].V_HE_Edges[j].EndIndex;
				
				//�˴��ı߽�����ⲿδ����Mesh�ı߽�ߣ������Ƿ����෴ 2020.06.14����Ū���������
				HE_Edge TempEdge;
				TempEdge.StartIndex = LocalSetIndexs[TempEndIndex];
				TempEdge.EndIndex = LocalSetIndexs[TempStartIndex];
				TempEdge.NextEdgeIndex = -1;
				//TempEdge.FaceIndex = -1;
				TempEdge.OppositeEdgeIndex = -1;
				TempEdge.EdgeIndex = -1;
				TempEdge.StartAngle = Point_HE_edges[TempEndIndex].SumAnlge;
				TempEdge.EndAngle = Point_HE_edges[TempStartIndex].SumAnlge;
				
				//�ж������Ǹ��ط�����
				int StartFindInRight = VectorBaseInt.FindIndexInVector(RightCloudIndexs, 
					LocalSetIndexs[TempStartIndex]);
				int EndFindInRight = VectorBaseInt.FindIndexInVector(RightCloudIndexs, 
					LocalSetIndexs[TempEndIndex]);
				
				if (StartFindInRight != -1 && EndFindInRight != -1)
				{					
					//�߽����Ҫ�滻Ϊȫ�ֱ߽�ߣ������ں��ڲ���					
					RightBorderEdges.push_back(TempEdge);					
					continue;
				}

				int StartFindInLeft = VectorBaseInt.FindIndexInVector(LeftCloudIndexs, 
					LocalSetIndexs[TempStartIndex]);
				int EndFindInLeft = VectorBaseInt.FindIndexInVector(LeftCloudIndexs, 
					LocalSetIndexs[TempEndIndex]);

				if (StartFindInLeft != -1 && EndFindInLeft != -1)
				{
					LeftBorderEdges.push_back(TempEdge);
				}
				//else
				//{
				//	//cout<<"BorderEdges Not Find Index In Global Indexs"<<endl;					
				//	
				//	//Viewer->addLine(GlobalIndexCloud->points[LocalSetIndexs[TempStartIndex]],
				//	//	InputCloud->points[LocalSetIndexs[TempEndIndex]], 255, 0, 0,
				//	//	StringBase::ClockValue());
				//}
			}
		}
	}
	//2020.06.17 ��ӶԱ߽�߻�·�Ĵ����߽���в�Ӧ�ô����Ż�·�������
	//��ʱ�Ѿ���ȫ������
	FindCircleInBorderEdges(LeftBorderEdges);
	FindCircleInBorderEdges(RightBorderEdges);

	///*
	//2020.06.16 ���Ǵ�����һ������������ı߽��
	for (int i = 0; i < BorderEdges.size(); i++)
	{
		int TempStartIndex = BorderEdges[i].StartIndex;
		int TempEndIndex = BorderEdges[i].EndIndex;

		//����ҵ�,˵����һ���Ѿ�����
		if (-1 != FindEdgeinEdges(Point_HE_edges[TempStartIndex].V_HE_Edges, BorderEdges[i]))
			continue;

		//��ʱ�������ߵķ���,����Ǹ������ݹ�����,
		HE_Edge TempEdge;
		TempEdge.StartIndex = LocalSetIndexs[TempStartIndex];
		TempEdge.EndIndex = LocalSetIndexs[TempEndIndex];
		TempEdge.NextEdgeIndex = -1;
		//TempEdge.FaceIndex = -1;
		TempEdge.OppositeEdgeIndex = -1;
		TempEdge.EdgeIndex = -1;
		TempEdge.StartAngle = Point_HE_edges[TempStartIndex].SumAnlge;
		TempEdge.EndAngle = Point_HE_edges[TempEndIndex].SumAnlge;

		//�ж�����Right or Left�ط�����
		int StartFindInRight = VectorBaseInt.FindIndexInVector(RightCloudIndexs,
			LocalSetIndexs[TempStartIndex]);
		int EndFindInRight = VectorBaseInt.FindIndexInVector(RightCloudIndexs,
			LocalSetIndexs[TempEndIndex]);

		int Temp = VectorBaseInt.FindIndexInVector(LocalSetIndexs,
			LocalSetIndexs[TempEndIndex]);

		if (StartFindInRight != -1 && EndFindInRight != -1)
		{
			//�߽����Ҫ�滻Ϊȫ�ֱ߽�ߣ������ں��ڲ���					
			//cout << "�ϴηָ�ı߽�߻����Ҳ�!" << endl;
			RightBorderEdges.push_back(TempEdge);
			continue;
		}

		int StartFindInLeft = VectorBaseInt.FindIndexInVector(LeftCloudIndexs,
			LocalSetIndexs[TempStartIndex]);
		int EndFindInLeft = VectorBaseInt.FindIndexInVector(LeftCloudIndexs,
			LocalSetIndexs[TempEndIndex]);

		if (StartFindInLeft != -1 && EndFindInLeft != -1)
		{
			//cout << "�ϴηָ�ı߽�߻������!" << endl;
			LeftBorderEdges.push_back(TempEdge);
		}
		else
		{
			int TempEnd = VectorBaseInt.FindIndexInVector(G_GuidPoints[0].Path,
				TempEndIndex);
			int TempStart = VectorBaseInt.FindIndexInVector(G_GuidPoints[0].Path,
				TempStartIndex);

			//TempBridgeEdges.push_back(TempEdge);

			if (UseCout)
			{
				cout << "���߶�������! Path Index Start:" << TempStart << ", TempEnd:" << TempEnd << endl;//�Ǿ����������Ǳ���
				
				///*
				Viewer->addText3D(StringBase::IntToStr(TempStartIndex),
					GlobalIndexCloud->points[LocalSetIndexs[TempStartIndex]], TextScale, 0,0, 255,
					StringBase::ClockValue() + StringBase::IntToStr(LocalSetIndexs[TempStartIndex]));

				Viewer->addText3D(StringBase::IntToStr(TempEndIndex),
					GlobalIndexCloud->points[LocalSetIndexs[TempEndIndex]], TextScale, 0, 0, 255,
					StringBase::ClockValue() + StringBase::IntToStr(LocalSetIndexs[TempEndIndex]));

				Viewer->addLine(GlobalIndexCloud->points[LocalSetIndexs[TempStartIndex]],
					GlobalIndexCloud->points[LocalSetIndexs[TempEndIndex]],
					StringBase::ClockValue() + StringBase::IntToStr(LocalSetIndexs[TempStartIndex]));
				//*/
			}
		}
	}
	//*/

	if (Viewer == NULL) return;

	///*
	if (ShowBorderEdge)
	{
		for (int i = 0; i < RightBorderEdges.size(); i++)
		{
			//Sleep(5);
			//cout <<"i:"<< i << ", " << GlobalIndexCloud->points.size()<<",R-S:"<< RightBorderEdges[i].StartIndex 
			//	<<",E:"<< RightBorderEdges[i].EndIndex << endl;
			Viewer->addLine(GlobalIndexCloud->points[RightBorderEdges[i].StartIndex],
				GlobalIndexCloud->points[RightBorderEdges[i].EndIndex], 0, 255, 0,
				StringBase::ClockValue() + StringBase::IntToStr(RightBorderEdges[i].StartIndex));
			//cout<<"St:"<< GlobalIndexCloud->points[RightBorderEdges[i].StartIndex] << endl;
			//cout << "En:" << GlobalIndexCloud->points[RightBorderEdges[i].EndIndex] << endl;
		}
		
		for (int i = 0; i < LeftBorderEdges.size(); i++)
		{
			Viewer->addLine(GlobalIndexCloud->points[LeftBorderEdges[i].StartIndex],
				GlobalIndexCloud->points[LeftBorderEdges[i].EndIndex], 255, 0, 0,
				StringBase::ClockValue() + StringBase::IntToStr(LeftBorderEdges[i].EndIndex));
			//cout << "St:" << GlobalIndexCloud->points[LeftBorderEdges[i].StartIndex] << endl;
			//cout << "En:" << GlobalIndexCloud->points[LeftBorderEdges[i].EndIndex] << endl;
		}	

		for (int i = 0; i < BorderEdges.size(); i++)
		{
			Viewer->addLine(InputCloud->points[BorderEdges[i].StartIndex],
				InputCloud->points[BorderEdges[i].EndIndex], 0, 0, 255,
				"Out" + StringBase::ClockValue() + StringBase::IntToStr(BorderEdges[i].EndIndex));
			//cout << "St:" << GlobalIndexCloud->points[LeftBorderEdges[i].StartIndex] << endl;
			//cout << "En:" << GlobalIndexCloud->points[LeftBorderEdges[i].EndIndex] << endl;
		}
	}	
	//*/
}

//2020.06.17 Ѱ����StartIndex��ʼ�ڵ�ı߽��
int CDividePointsByDelaunayLine::FindEdgeByStartIndex(V_HE_Edge CurEdge, int StartIndex)
{
	int TempIndex = -1;

	for (int i = 0; i < CurEdge.size(); i++)
	{
		if (CurEdge[i].StartIndex == StartIndex)
		{
			TempIndex = i;
			break;
		}
	}

	return TempIndex;
}

//���ݴ���㼯LocalIndexs�� �� ��߽ṹCurEdges ���� ȫ�ְ�߽ṹ Point_HE_edges 2020.06.22
void CDividePointsByDelaunayLine::RefreshPointAngleAndEdgesInGlobal(
	vector<int> TempLocalIndexs, Point_HE_Edge CurEdges)
{
	//���δ�պϵĵ�����������Ӧ�ı߽���Ϣ 2020.06.22	
	for (int j = 0; j < CurEdges.size(); j++)
	{
		int TempGlobalIndex = TempLocalIndexs[j];

		//�滻Ϊȫ�ֵ�����
		for (int k = 0; k < CurEdges[j].V_HE_Edges.size(); k++)
		{
			CurEdges[j].V_HE_Edges[k].StartIndex =
				TempLocalIndexs[CurEdges[j].V_HE_Edges[k].StartIndex];
			CurEdges[j].V_HE_Edges[k].EndIndex =
				TempLocalIndexs[CurEdges[j].V_HE_Edges[k].EndIndex];
		}

		if (CurEdges[j].V_HE_Edges.size() > 0)
		{
			Point_HE_edges[TempGlobalIndex].SumAnlge -= CurEdges[j].SumAnlge;

			Point_HE_edges[TempGlobalIndex].V_HE_Edges.insert(
				Point_HE_edges[TempGlobalIndex].V_HE_Edges.end(),
				CurEdges[j].V_HE_Edges.begin(),
				CurEdges[j].V_HE_Edges.end());
		}
	}
}

CDividePointsByDelaunayLine::CDividePointsByDelaunayLine()
{
	GlobalIndexCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	LeftCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	RightCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	InputCloudInLine.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

	ShowGuidPolyLine = false;
	ShowGuidLine = false;
	ShowBorderEdge = false;
	ShowOuterGuidPoints = false;

	TextScale = 0.2;
}

CDividePointsByDelaunayLine::~CDividePointsByDelaunayLine()
{
	G_GuidPoints[0].Path.clear();
	G_GuidPoints.clear();
}


void CDividePointsByDelaunayLine::SetInputIndexCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue,
	vector<int> & LocalSetInGlobalIndex,
	V_HE_Edge InputBorderEdgesValue)
{
	if (UseCout)
		cout << endl <<"*************************************************************" << endl;
	if (UseCout)
		cout << "LocalSetInGlobalIndex Size:"<< LocalSetInGlobalIndex.size() << endl;
	GlobalIndexCloud = InputCloudValue;
	InputCloud->points.clear();	
	LocalSetIndexs.clear();	

	if (LocalSetInGlobalIndex.size() == 0)
	{
		for (int i = 0; i < GlobalIndexCloud->points.size(); i++)
		{
			LocalSetIndexs.push_back(i);
			LocalSetInGlobalIndex.push_back(i);
		}
		//PointBase::PointXYZRGBIndexToPointXYZRGB(GlobalIndexCloud, InputCloud);
		PointBase::PointCopy(GlobalIndexCloud, InputCloud);		

		//��һ�ε���ʱ����Point_HE_edges��ʼ��
		for (int i = 0; i < InputCloud->points.size(); i++)
		{
			P_HE_Edge Point_Edges;

			//Point_Edges.SumAnlge = 2 * M_PI;
			//�˴��ǳ�ʼ��Ϊ�㣬�Լ������ʣ��ĽǶ� 2020.06.22
			//Point_Edges.SumAnlge = 0;
			//Point_HE_edges.push_back(Point_Edges);
		}

	}
	else
	{		
		LocalSetIndexs.insert(LocalSetIndexs.begin(), LocalSetInGlobalIndex.begin(), LocalSetInGlobalIndex.end());

		for (int i = 0; i < LocalSetIndexs.size(); i++)
		{
			pcl::PointXYZRGB TempPoint;

			TempPoint.x = GlobalIndexCloud->points[LocalSetIndexs[i]].x,
				TempPoint.y = GlobalIndexCloud->points[LocalSetIndexs[i]].y,
				TempPoint.z = GlobalIndexCloud->points[LocalSetIndexs[i]].z;

			InputCloud->points.push_back(TempPoint);			
		}
	}
	PointBase::PointXYZRGBToPointXYZRGBIndex(InputCloud, CopyIndexCloud);
	
	//��������ı߽��
	BorderEdges.clear();
	if (LocalSetInGlobalIndex.size() == 0) //���轫�߽�ߵĽ��������Ϊ�ֲ�������
	{
		BorderEdges.insert(BorderEdges.begin(),
				InputBorderEdgesValue.begin(), InputBorderEdgesValue.end());
	}
	else //�轫�߽�ߵĽ��������Ϊ�ֲ�������
	{
		BorderEdges.insert(BorderEdges.begin(),
			InputBorderEdgesValue.begin(), InputBorderEdgesValue.end());
				
		for (int i = 0; i < BorderEdges.size(); i++)
		{
			BorderEdges[i].StartIndex = VectorBaseInt.FindIndexInVector(LocalSetIndexs,
				BorderEdges[i].StartIndex);
			BorderEdges[i].EndIndex = VectorBaseInt.FindIndexInVector(LocalSetIndexs,
				BorderEdges[i].EndIndex);			

			//if (LocalSetInGlobalIndex.size() == 115 && UseCout)
			//{
			//	Viewer->addLine(InputCloud->points[BorderEdges[i].StartIndex],
			//		InputCloud->points[BorderEdges[i].EndIndex],
			//		StringBase::ClockValue() + StringBase::IntToStr(BorderEdges[i].StartIndex)
			//		+ StringBase::IntToStr(BorderEdges[i].EndIndex));
			//}
		}
	}	
	
	//PointBase::SavePCDToFileName(InputCloud, "I:\\PlanePoints\\Test.pcd");	
}

//������ֵ㼯ʱ������Mesh����MeshΪȫ��Mesh�����ⲿ���ú�����ʼ����2020.06.10
void CDividePointsByDelaunayLine::OutputDivideMesh(pcl::PolygonMesh & OutDivideMesh)
{
	for (int i = 0; i < T_Faces.size(); i++)
	{
		/*if (LocalSetIndexs[T_Faces[i].EdgeIndexs[0]] == 2963 ||
			LocalSetIndexs[T_Faces[i].EdgeIndexs[1]] == 2963 ||
			LocalSetIndexs[T_Faces[i].EdgeIndexs[2]] == 2963)
		{
			if (LocalSetIndexs[T_Faces[i].EdgeIndexs[0]] == 154 ||
				LocalSetIndexs[T_Faces[i].EdgeIndexs[1]] == 154 ||
				LocalSetIndexs[T_Faces[i].EdgeIndexs[2]] == 154)
			{
				cout<<"2963:" << LocalSetIndexs[T_Faces[i].EdgeIndexs[0]] << ","
					<< LocalSetIndexs[T_Faces[i].EdgeIndexs[1]] << ","
					<< LocalSetIndexs[T_Faces[i].EdgeIndexs[2]] << ","
					<<T_Faces[i].EdgeIndexs[0] <<","
					<< T_Faces[i].EdgeIndexs[1] <<","
					<< T_Faces[i].EdgeIndexs[2] <<endl;
			}
		}*/

		pcl::Vertices TempVertices;
		TempVertices.vertices.push_back(LocalSetIndexs[T_Faces[i].EdgeIndexs[0]]);
		TempVertices.vertices.push_back(LocalSetIndexs[T_Faces[i].EdgeIndexs[1]]);
		TempVertices.vertices.push_back(LocalSetIndexs[T_Faces[i].EdgeIndexs[2]]);
		OutDivideMesh.polygons.push_back(TempVertices);
	}
}

//2020.06.17 ���ֱ߽���еĻ�·����Ӹ����������Ƴ���·
void CDividePointsByDelaunayLine::FindCircleInBorderEdges(V_HE_Edge & CurEdge)
{
	vector<int> UsedIndexs;
	vector<int> DeleteIndexs;
	
	//�߽��·�еĵ������
	vector<int> InnerIndexs;	

	int SecondEdgeIndex = -1, ThirdEdgeIndex = -1;
	for (int i = 0; i < CurEdge.size(); i++)
	{
		if (-1 != VectorBaseInt.FindIndexInVector(DeleteIndexs, i))		
			continue;

		int TempStartIndex = CurEdge[i].StartIndex;
		int TempEndIndex = CurEdge[i].EndIndex;
		SecondEdgeIndex = FindEdgeByStartIndex(CurEdge, TempEndIndex);
		if (SecondEdgeIndex == -1)
		{
			UsedIndexs.push_back(i);
			continue;
		}

		ThirdEdgeIndex = FindEdgeByStartIndex(CurEdge, CurEdge[SecondEdgeIndex].EndIndex);
		if (ThirdEdgeIndex == -1)
		{
			UsedIndexs.push_back(i);
			continue;
		}

		//���ǻ�· �� ��ǰ�ڵ㲻����ɾ���б���
		if (CurEdge[ThirdEdgeIndex].EndIndex != TempStartIndex)
		{
			UsedIndexs.push_back(i);
		}
		else
		{
			DeleteIndexs.push_back(i);
			DeleteIndexs.push_back(SecondEdgeIndex);
			DeleteIndexs.push_back(ThirdEdgeIndex);

			if (UseCout)
			{
				cout << "!!!!!!!!!" << "Circle,1:" << CurEdge[i].StartIndex << ",2:" << CurEdge[i].EndIndex << ",3:" << CurEdge[SecondEdgeIndex].EndIndex << endl;
				cout << "!!!!!!!!!" << "Point Size:" << InputCloud->points.size() << ", Border Edge Size:" << BorderEdges.size() << endl;
			}

			HE_Edge TempEdge;

			TempEdge.StartIndex = VectorBaseInt.FindIndexInVector(LocalSetIndexs, CurEdge[i].StartIndex);
			TempEdge.EndIndex = VectorBaseInt.FindIndexInVector(LocalSetIndexs, CurEdge[i].EndIndex);
			TempEdge.OppositeEdgeIndex = -1;
			//TempEdge.FaceIndex = -1;
			TempEdge.NextEdgeIndex = -1;			
			TempEdge.EdgeIndex = -1;

			int ThirdPointIndex = VectorBaseInt.FindIndexInVector(LocalSetIndexs, CurEdge[SecondEdgeIndex].EndIndex);
			//ִ�������������Σ��п�����������ε���������һ��δ���������εĵ㣬�����Ļ���ֱ����������ξʹ�����
			//2020.06.19
			//AddFace(TempEdge, ThirdPointIndex);
			int TempThirdIndex = ExpandEdge(TempEdge);

			//�����չ�õ��ĵ�����պ�����Ҫ�ĵ������㲻��Ҫ���⴦��
			//������Ҫ���µõ��ı߽�����չ
			if (TempThirdIndex == ThirdPointIndex)
			{
				AddFace(TempEdge, TempThirdIndex);
				
				if (UseCout)
					cout<<"�պϻ�·�պ���������"<<endl;
			}
			else if (TempThirdIndex != -1)
			{
				AddFace(TempEdge, TempThirdIndex);
				
				
				if (UseCout)
					cout << "�պϻ�·�а������µĵ㣬����һ��������" << endl;

				InnerIndexs.push_back(TempThirdIndex);
				
				if (UseCout)
					cout<<"NeedExpandEdges Size:"<<NeedExpandEdges.size()<<endl;

				while (NeedExpandEdges.size() > 0)
				{
					HE_Edge TempExpandgeEdge = NeedExpandEdges[0];
					NeedExpandEdges.erase(NeedExpandEdges.begin());

					int TempIndex = ExpandEdge(TempExpandgeEdge);

					if (TempIndex == -1)
						continue;

					AddFace(TempExpandgeEdge, TempIndex);
					
					if (UseCout)
						cout << "�պϻ�·�а������µĵ㣬����һ��������" << endl;
					
					if (-1 == VectorBaseInt.FindIndexInVector(InnerIndexs, TempIndex))
					{
						InnerIndexs.push_back(TempIndex);
					}
				}				
			}	
		}
	}

	V_HE_Edge TempEdges;
	TempEdges.insert(TempEdges.begin(), CurEdge.begin(), CurEdge.end());
	CurEdge.clear();

	for(int i = 0; i < UsedIndexs.size(); i++)
	{
		//��ǰ���������ڴ�ɾ��������
		if(-1 == VectorBaseInt.FindIndexInVector(DeleteIndexs, UsedIndexs[i]))
			CurEdge.push_back(TempEdges[UsedIndexs[i]]);
	}

	//�����·�ڲ���Ĺ���
	for (int i = 0; i < InnerIndexs.size(); i++)
	{
		//�����㼯��
		int TempIndex = VectorBaseInt.FindIndexInVector(LeftCloudIndexs, LocalSetIndexs[InnerIndexs[i]]);
		if (-1 != TempIndex)
		{
			LeftCloudIndexs.erase(LeftCloudIndexs.begin() + TempIndex);
			L_ClosedPointIndexs.push_back(LocalSetIndexs[InnerIndexs[i]]);
			continue;
		}
		
		TempIndex = VectorBaseInt.FindIndexInVector(RightCloudIndexs, LocalSetIndexs[InnerIndexs[i]]);
		if (-1 != TempIndex)
		{
			RightCloudIndexs.erase(RightCloudIndexs.begin() + TempIndex);
			R_ClosedPointIndexs.push_back(LocalSetIndexs[InnerIndexs[i]]);
			continue;
		}

		cout<<"�����д���"<<endl;
	}
}

//���ݵ㵽��ʼ����ֹ���ͶӰ���µ����ȵ��ͶӰ�������ӽ����յ㣬���ҵ㵽ͶӰ��ľ����С
int CDividePointsByDelaunayLine::FindPriorityPoint(int CurrentPointIndex, int StartIndex, int EndIndex, bool Forward)
{
	pcl::PointXYZRGB ProjectPoint = GeometryBase::PointProjectToLine(InputCloud->points[CurrentPointIndex],
		InputCloud->points[StartIndex], InputCloud->points[EndIndex]);

	double SmallLineDis = 1.0e10;
	double ProjectEndDis = PointDis(ProjectPoint, InputCloud->points[EndIndex]);

	int SmallDisIndex = -1;
	int SmallToLineIndex = -1;

	//��֮ǰ���ȵ�õ������е���Ѱ����һ�����ȵ�
	for (int i = 0; i < Point_HE_edges[CurrentPointIndex].V_HE_Edges.size(); i++)
	{
		int TempIndex = -1;

		//ǰ��Ѱ��
		if (Forward)
		{
			//if (Point_HE_edges[CurrentPointIndex].V_HE_Edges[i].StartIndex != CurrentPointIndex)
			//	TempIndex = Point_HE_edges[CurrentPointIndex].V_HE_Edges[i].StartIndex;
			//else
				TempIndex = Point_HE_edges[CurrentPointIndex].V_HE_Edges[i].EndIndex;
		}
		//����Ѱ��
		else
		{
			int TempEndIndex = Point_HE_edges[CurrentPointIndex].V_HE_Edges[i].EndIndex;
			for (int j = 0; j < Point_HE_edges[TempEndIndex].V_HE_Edges.size(); j++)
			{
				int NextEndIndex = Point_HE_edges[TempEndIndex].V_HE_Edges[j].EndIndex;
				int TempFindIndex = FindEdgeinEdges(Point_HE_edges[NextEndIndex].V_HE_Edges, NextEndIndex, CurrentPointIndex);
				if (TempFindIndex != -1)
				{
					TempIndex = NextEndIndex;
					break;
				}
			}
		}

		//���ݵ���ֱ���ϵ�ͶӰ���յ�ľ�����Ѱ��
		ProjectPoint = GeometryBase::PointProjectToLine(InputCloud->points[TempIndex],
			InputCloud->points[StartIndex], InputCloud->points[EndIndex]);

		double TempEndDis = PointDis(ProjectPoint, InputCloud->points[EndIndex]);
		double TempLineDis = PointDis(ProjectPoint, InputCloud->points[TempIndex]);

		//���ӽ����յ㣬���ҵ�������յ�����ֱ�ߵľ������
		if (TempEndDis < ProjectEndDis && TempLineDis < SmallLineDis)
		{
			SmallLineDis = TempLineDis;
			SmallDisIndex = TempIndex;
		}
	}
	return SmallDisIndex;
}

//��ȡ Numbers �� GuidPoints���������㼯����Ϊ Numbers + 1�� 2020.06.01
//��ʱδʹ�ã�2020.06.17
void CDividePointsByDelaunayLine::GetGuidPoints(int Numbers)
{
	//��ȡ���͵ڶ����� �� �����ϵ� ������С ����Χֵ
	GetStartAndEndPointForDividing();

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	//double MaxDirectionDisSpace = PointDis(MaxDirectionStartPoint, MaxDirectionEndPoint) / Numbers;
	//double SecDirectionDisSpace = PointDis(SecDirectionStartPoint, SecDirectionEndPoint);
	//��������ϵĵ� ���� �ڶ����� �����ƶ� 1�� SecDirectionDis�ľ���

	////����һ�£������
	//if ((MaxDirectionEndPoint.x - MaxDirectionStartPoint.x) * MaxDirection.x < 0)
	//{
	//	MaxDirection.x = -MaxDirection.x, MaxDirection.y = -MaxDirection.y,
	//		MaxDirection.z = -MaxDirection.z;
	//}

	//if ((SecDirectionEndPoint.x - SecDirectionStartPoint.x) * SecondDirection.x < 0)
	//{
	//	SecondDirection.x = -SecondDirection.x, SecondDirection.y = -SecondDirection.y,
	//		SecondDirection.z = -SecondDirection.z;
	//}

	////�ȵõ� GuidPoints ����Զ�㣬Ȼ������һ���ڵ㼯��������������ĵ�
	//for (int i = 1; i < Numbers; i++)
	//{
	//	PairPointIndexs Temp;
	//	pcl::PointXYZRGB AnchorPointInLine = PointBase::GetPointAlongLine(MaxDirection,
	//		MaxDirectionStartPoint, i * MaxDirectionDisSpace);

	//	//�ϲ��ڵ�
	//	Temp.StartPoint0 = PointBase::GetPointAlongLine(SecondDirection,
	//		AnchorPointInLine, SecDirectionDisSpace);
	//	Temp.StartPoint0.rgba = ColorBase::RedColor;

	//	//�²��ڵ�
	//	Temp.EndPoint0 = PointBase::GetPointAlongLine(SecondDirection,
	//		AnchorPointInLine, SecDirectionDisSpace, false);
	//	Temp.EndPoint0.rgba = ColorBase::YellowColor;

	//	G_GuidPoints.push_back(Temp);

	//	//TempCloud->points.push_back(Temp.StartPoint0);
	//	//TempCloud->points.push_back(Temp.EndPoint0);
	//}
	//���±߽���Ѿ�����
	//PointBase::ShowPointXYZRGB(Viewer, TempCloud, "TempCloud", 3);	

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		for (int j = 0; j < G_GuidPoints.size(); j++)
		{
			if (i == 0)
			{
				G_GuidPoints[j].SmallEndDis = PointDis(InputCloud->points[i], G_GuidPoints[j].EndPoint0);
				G_GuidPoints[j].EndEdge0 = i;

				G_GuidPoints[j].SmallStartDis = PointDis(InputCloud->points[i], G_GuidPoints[j].StartPoint0);
				G_GuidPoints[j].StartEdge0 = i;
			}
			else
			{
				double TempAngle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
					G_GuidPoints[j].StartPoint0, InputCloud->points[i],
					G_GuidPoints[j].EndPoint0));

				if (TempAngle < AllowAngle)
					continue;

				//�յ�Ƚ�
				double TempEndDis = PointDis(InputCloud->points[i], G_GuidPoints[j].EndPoint0);
				if (TempEndDis < G_GuidPoints[j].SmallEndDis)
				{
					G_GuidPoints[j].SmallEndDis = TempEndDis;
					G_GuidPoints[j].EndEdge0 = i;
				}

				//���Ƚ�
				double TempStartDis = PointDis(InputCloud->points[i], G_GuidPoints[j].StartPoint0);
				if (TempStartDis < G_GuidPoints[j].SmallStartDis)
				{
					G_GuidPoints[j].SmallStartDis = TempStartDis;
					G_GuidPoints[j].StartEdge0 = i;
				}
			}
		}
	}

	for (int j = 0; j < G_GuidPoints.size(); j++)
	{
		G_GuidPoints[j].SmallEndDis = 9.0e20;
		G_GuidPoints[j].EndEdge1 = -1;
		G_GuidPoints[j].StartEdge1 = -1;
	}

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		for (int j = 0; j < G_GuidPoints.size(); j++)
		{
			//���Ƚϣ���㲻Ӧ�Ǿ�������ĵ㣬���ǽǶ���С�Ҿ�����С�ĵ㣬
			double TempStartDis = PointDis(InputCloud->points[i],
				InputCloud->points[G_GuidPoints[j].StartEdge0]);
			if (TempStartDis < G_GuidPoints[j].SmallStartDis && TempStartDis > 0)
			{
				G_GuidPoints[j].SmallStartDis = TempStartDis;
				G_GuidPoints[j].StartEdge1 = i;
			}

			//�յ�Ƚ� 
			double TempEndDis = PointDis(InputCloud->points[i],
				InputCloud->points[G_GuidPoints[j].EndEdge0]);

			if (TempEndDis < G_GuidPoints[j].SmallEndDis && TempEndDis > 0)
			{
				G_GuidPoints[j].SmallEndDis = TempEndDis;
				G_GuidPoints[j].EndEdge1 = i;
			}
		}
	}

	//for (int j = 0; j < G_GuidPoints.size(); j++)
	//{
	//	TempCloud->points.push_back(InputCloud->points[G_GuidPoints[j].StartEdge0]);
	//	TempCloud->points[TempCloud->points.size() - 1].rgba = ColorBase::RedColor;
	//	TempCloud->points.push_back(InputCloud->points[G_GuidPoints[j].EndEdge0]);
	//	TempCloud->points[TempCloud->points.size() - 1].rgba = ColorBase::RedColor;

	//	TempCloud->points.push_back(InputCloud->points[G_GuidPoints[j].StartEdge1]);
	//	TempCloud->points[TempCloud->points.size() - 1].rgba = ColorBase::OrangeColor;
	//	TempCloud->points.push_back(InputCloud->points[G_GuidPoints[j].EndEdge1]);
	//	TempCloud->points[TempCloud->points.size() - 1].rgba = ColorBase::OrangeColor;
	//}
	//PointBase::SetPointColor(TempCloud, ColorBase::RedColor);
	//PointBase::ShowPointXYZRGB(Viewer, TempCloud, "AnchorCloud", 3);
}