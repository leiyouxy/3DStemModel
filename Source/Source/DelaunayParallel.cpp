#include "DelaunayParallel.h"

void CDelaunayParallel::RefreshData()
{
	CTreeBase::RefreshData();
}

//并行计算Delaunay 2020.06.07
void CDelaunayParallel::DelaunayParallelGrowth()
{
	DelaunayParallelForm.pushButtonDelaunayGrowthParallel->setEnabled(false);
	emitUpdateStatusBar("Parallel DelaunayGrowth is working", 3000);
	pcl::PolygonMesh WithHalfEdgePolygonMesh;
	time_t c_start = clock();	
	double TimeHalfEdgeGrow = -1;
	
	bool ShowCompare;
	///*
	ShowCompare = false;
	ShowCompare = true;	

	if (ShowCompare)
	{
		CDelaunayGrowthWithHalfEdge DelaunayGrowthWithHalfEdge;
		DelaunayGrowthWithHalfEdge.SetInputCloud(InputCloud);
		DelaunayGrowthWithHalfEdge.Viewer = Viewer;
		DelaunayGrowthWithHalfEdge.performReconstructionWithPriority();
		DelaunayGrowthWithHalfEdge.OutputMesh(WithHalfEdgePolygonMesh);

		TimeHalfEdgeGrow = difftime(clock(), c_start);
		
		cout << endl << "DelaunayGrowthWithHalfEdge: "
			<< " times: " << TimeHalfEdgeGrow << " ms" << endl;
		cout << "点个数:" << InputCloud->points.size()
			<< " 三角形个数:" << WithHalfEdgePolygonMesh.polygons.size() << endl;
	}
	//*/	
	pcl::PolygonMesh ParaGlobalMeshs;
	V_HE_Edge GlobalBridgeEdges;

	if (InputCloud->points.size() < 3) return;
	pcl::PCLPointCloud2 TempPointCloud2;
	pcl::toPCLPointCloud2(*InputCloud, TempPointCloud2);
	ParaGlobalMeshs.header = InputCloud->header;
	ParaGlobalMeshs.cloud = TempPointCloud2;
	
	//*		
	Viewer->removeAllShapes();	
	Viewer->removeText3D();	
	
	CDividePointsByDelaunayLine DividePointsByDelaunayLine;
	DividePointsByDelaunayLine.Viewer = Viewer;
	V_HE_Edge InputBorderEdges;
	vector<int> LocalSets;
	
	//并行计算开始
	c_start = clock();
	DividePointsByDelaunayLine.SetInputIndexCloud(InputCloud, LocalSets, InputBorderEdges);	

	vector<DiviedPointsAndBorderEdge> TempPointsAndBorderEdges, 
		ResultPointsAndBorderEdges, UndoPointsAndBorderEdges;

	DividePointsByDelaunayLine.HalfDivide(TempPointsAndBorderEdges, ParaGlobalMeshs,
		DelaunayParallelForm.checkBoxShowGuidLine->checkState(),
		DelaunayParallelForm.checkBoxShowGuidPolyLine->checkState(),
		DelaunayParallelForm.checkBoxShowBorderEdge->checkState());	

	//SaveMeshToFile(ParaGlobalMeshs, "I:\\PlanePoints\\Mesh.txt");

	Point_HE_Edge CurTempEdges;

	//获取当前点集的角度与半边信息
	DividePointsByDelaunayLine.OutPutPointAngleAndEdges(CurTempEdges);
	DividePointsByDelaunayLine.RefreshPointAngleAndEdgesInGlobal(LocalSets, CurTempEdges);

	//GlobalMeshs.polygons.clear();	
	
	int const Number = DelaunayParallelForm.spinBoxSplitZonePoints->text().toInt();

	for (int i = 0; i < TempPointsAndBorderEdges.size(); i++)	
	{
		if (TempPointsAndBorderEdges[i].PartIndexs.size() > Number)
			UndoPointsAndBorderEdges.push_back(TempPointsAndBorderEdges[i]);
		else if (TempPointsAndBorderEdges[i].PartIndexs.size() > 0)
			ResultPointsAndBorderEdges.push_back(TempPointsAndBorderEdges[i]);
	}

	///*
	while (UndoPointsAndBorderEdges.size() > 0)
	{		
		//int UndoCount = UndoPointsAndBorderEdges.size();
		//cout<<"UndoCount:"<< UndoPointsAndBorderEdges.size() <<endl;

		vector<DiviedPointsAndBorderEdge> TemFor;

		//#pragma omp parallel for //如果显示资源，则不能使用并行模式
		for (int i = 0; i < UndoPointsAndBorderEdges.size(); i++)
		{			
			CDividePointsByDelaunayLine TempDividePointsByDelaunayLine;
			//TempDividePointsByDelaunayLine.Viewer = Viewer;
			pcl::PolygonMesh TempGlobalMeshs;

			TempDividePointsByDelaunayLine.SetInputIndexCloud(InputCloud,
				UndoPointsAndBorderEdges[i].PartIndexs, 
				UndoPointsAndBorderEdges[i].CurBorderEdges);			

			vector<DiviedPointsAndBorderEdge> TempTempPointsAndBorderEdges;

			TempDividePointsByDelaunayLine.HalfDivide(TempTempPointsAndBorderEdges, 
				TempGlobalMeshs,
				DelaunayParallelForm.checkBoxShowGuidLine->checkState(),
				DelaunayParallelForm.checkBoxShowGuidPolyLine->checkState(),
				DelaunayParallelForm.checkBoxShowBorderEdge->checkState());

			//获取当前点集的角度与半边信息
			TempDividePointsByDelaunayLine.OutPutPointAngleAndEdges(
				UndoPointsAndBorderEdges[i].CurPointEdges);

			//使用临时变量来更新当前点的角度与半边信息
			DividePointsByDelaunayLine.RefreshPointAngleAndEdgesInGlobal(
				UndoPointsAndBorderEdges[i].PartIndexs, 
				UndoPointsAndBorderEdges[i].CurPointEdges);

			#pragma omp critical
			{				
				//if (UndoPointsAndBorderEdges[i].PartIndexs.size() == 107 &&
				//	UndoPointsAndBorderEdges[i].CurBorderEdges.size() == 49)
				
					
				//if (UndoPointsAndBorderEdges[i].PartIndexs.size() == 174 &&
				//	UndoPointsAndBorderEdges[i].CurBorderEdges.size() == 71)	
				
					ParaGlobalMeshs.polygons.insert(ParaGlobalMeshs.polygons.end(),
						TempGlobalMeshs.polygons.begin(), TempGlobalMeshs.polygons.end());

				for (int j = 0; j < TempTempPointsAndBorderEdges.size(); j++)
				{
					if (TempTempPointsAndBorderEdges[j].PartIndexs.size() > Number)
						TemFor.push_back(TempTempPointsAndBorderEdges[j]);				
					else if (TempTempPointsAndBorderEdges[j].PartIndexs.size() > 0)
						ResultPointsAndBorderEdges.push_back(TempTempPointsAndBorderEdges[j]);				
				}
			}
		}

		UndoPointsAndBorderEdges.clear();
		UndoPointsAndBorderEdges.insert(UndoPointsAndBorderEdges.begin(),
			TemFor.begin(), TemFor.end());
	}
	
	//*/	

	double TimeParaDivideDis = difftime(clock(), c_start);
	
	cout << "dividing end" << endl;

	pcl::PolygonMesh TempParallelMesh;	
	
	DividePointsByDelaunayLine.PerformParallel(ResultPointsAndBorderEdges, TempParallelMesh);	

	double TimeParaGrowth = difftime(clock(), c_start);

	vector<int> Numbers;

	VectorBase<int> VectorBaseInt;

	for (int i = 0; i < ResultPointsAndBorderEdges.size(); i++)
	{
		Numbers.push_back(ResultPointsAndBorderEdges[i].PartIndexs.size());
	}

	MaxNum = VectorBaseInt.CalcMax(Numbers);
	MinNum = VectorBaseInt.CalcMin(Numbers);
	VarianceNum = VectorBaseInt.CalcVariances(Numbers, AvgNum);
	K = ResultPointsAndBorderEdges.size();

	cout << "划分用时: "
		<< " times: " << TimeParaDivideDis << " ms" << endl;
	cout << "划分后生成网格耗时: "
		<< " times: " << TimeParaGrowth << " ms" << endl;
	cout << "并行算法共耗时: "
		<< " times: " << TimeParaDivideDis + TimeParaGrowth << " ms" << endl;	
	cout << "划分子集点数最大: "<< MaxNum<<",最小: " << MinNum
		<< ", 平均: " << AvgNum << ", 方差：" << VarianceNum << endl;

	if (IsSave)
	{		
		SumTimeHalfEdgeGrow = SumTimeHalfEdgeGrow + TimeHalfEdgeGrow;
		SumTimeParaDivideDis = SumTimeParaDivideDis + TimeParaDivideDis;
		SumTimeParaGrowth = SumTimeParaGrowth + TimeParaGrowth;

		////cout<<"Ready for save:"<< SaveFileName <<endl;
		//SaveStr = OpenedFilePath + "," +
		//	GetDateTime() + "," +
		//	StringBase::IntToStr(Number) + "," +
		//	StringBase::FloatToStr(TimeHalfEdgeGrow) + "," +
		//	StringBase::FloatToStr(TimeParaDivideDis) + "," + 
		//	StringBase::FloatToStr(TimeParaGrowth) + "," + 
		//	StringBase::FloatToStr(TimeParaDivideDis + TimeParaGrowth);

		//cout << "SaveStr:" << SaveStr << endl;
		//if (SaveFileName.length() > 0)
		//	SaveStringToFile(SaveFileName, SaveStr);
	}

	//移除分割时产生的Mesh
	//GlobalMeshs.polygons.clear(); //

	ParaGlobalMeshs.polygons.insert(ParaGlobalMeshs.polygons.end(), TempParallelMesh.polygons.begin(), TempParallelMesh.polygons.end());
	
	cout << "点个数:" << InputCloud->points.size()
		<< " 三角形个数:" << ParaGlobalMeshs.polygons.size() << endl;
	
	/*
	double TextScale = 0.5;
	if (ShowCompare)		
	for (int i = 0; i < WithHalfEdgePolygonMesh.polygons.size(); i++)
	{
		bool Find = false;
		for (int j = 0; j < ParaGlobalMeshs.polygons.size(); j++)
		{
			if ((WithHalfEdgePolygonMesh.polygons[i].vertices[0] == ParaGlobalMeshs.polygons[j].vertices[0]
				|| WithHalfEdgePolygonMesh.polygons[i].vertices[0] == ParaGlobalMeshs.polygons[j].vertices[1]
				|| WithHalfEdgePolygonMesh.polygons[i].vertices[0] == ParaGlobalMeshs.polygons[j].vertices[2])

				&& (WithHalfEdgePolygonMesh.polygons[i].vertices[1] == ParaGlobalMeshs.polygons[j].vertices[0]
					|| WithHalfEdgePolygonMesh.polygons[i].vertices[1] == ParaGlobalMeshs.polygons[j].vertices[1]
					|| WithHalfEdgePolygonMesh.polygons[i].vertices[1] == ParaGlobalMeshs.polygons[j].vertices[2])

				&& (WithHalfEdgePolygonMesh.polygons[i].vertices[2] == ParaGlobalMeshs.polygons[j].vertices[0]
					|| WithHalfEdgePolygonMesh.polygons[i].vertices[2] == ParaGlobalMeshs.polygons[j].vertices[1]
					|| WithHalfEdgePolygonMesh.polygons[i].vertices[2] == ParaGlobalMeshs.polygons[j].vertices[2]))
			{
				Find = true;
				continue;
			}
		}
		if (!Find)
		{
			Viewer->addLine(InputCloud->points[WithHalfEdgePolygonMesh.polygons[i].vertices[0]],
				InputCloud->points[WithHalfEdgePolygonMesh.polygons[i].vertices[1]], 0, 255, 0,
				"L0" + StringBase::ClockValue() + StringBase::IntToStr(WithHalfEdgePolygonMesh.polygons[i].vertices[1]));
			Viewer->addLine(InputCloud->points[WithHalfEdgePolygonMesh.polygons[i].vertices[1]],
				InputCloud->points[WithHalfEdgePolygonMesh.polygons[i].vertices[2]], 0, 255, 0,
				StringBase::ClockValue() + "L1" + StringBase::IntToStr(WithHalfEdgePolygonMesh.polygons[i].vertices[2]));
			Viewer->addLine(InputCloud->points[WithHalfEdgePolygonMesh.polygons[i].vertices[0]],
				InputCloud->points[WithHalfEdgePolygonMesh.polygons[i].vertices[2]], 0, 255, 0,
				StringBase::ClockValue() + StringBase::IntToStr(WithHalfEdgePolygonMesh.polygons[i].vertices[0]) + "L2");

			
			Viewer->addText3D(StringBase::IntToStr(WithHalfEdgePolygonMesh.polygons[i].vertices[0]),
				InputCloud->points[WithHalfEdgePolygonMesh.polygons[i].vertices[0]],
				TextScale, 255, 0, 0, "T0" + StringBase::ClockValue() + StringBase::IntToStr(WithHalfEdgePolygonMesh.polygons[i].vertices[0]));
			Viewer->addText3D(StringBase::IntToStr(WithHalfEdgePolygonMesh.polygons[i].vertices[1]), 
				InputCloud->points[WithHalfEdgePolygonMesh.polygons[i].vertices[1]],
				TextScale, 255, 0, 0, "T1" + StringBase::ClockValue() + StringBase::IntToStr(WithHalfEdgePolygonMesh.polygons[i].vertices[1]));
			Viewer->addText3D(StringBase::IntToStr(WithHalfEdgePolygonMesh.polygons[i].vertices[2]),
				InputCloud->points[WithHalfEdgePolygonMesh.polygons[i].vertices[2]],
				TextScale, 255, 0, 0, "T2" + StringBase::ClockValue() + StringBase::IntToStr(WithHalfEdgePolygonMesh.polygons[i].vertices[2]));
				
		}
	}
	//ShowCompare = false;
	if (ShowCompare)
		for (int i = 0; i < ParaGlobalMeshs.polygons.size(); i++)
		{
			bool Find = false;
			for (int j = 0; j < WithHalfEdgePolygonMesh.polygons.size(); j++)
			{
				if ((ParaGlobalMeshs.polygons[i].vertices[0] == WithHalfEdgePolygonMesh.polygons[j].vertices[0]
					|| ParaGlobalMeshs.polygons[i].vertices[0] == WithHalfEdgePolygonMesh.polygons[j].vertices[1]
					|| ParaGlobalMeshs.polygons[i].vertices[0] == WithHalfEdgePolygonMesh.polygons[j].vertices[2])

					&& (ParaGlobalMeshs.polygons[i].vertices[1] == WithHalfEdgePolygonMesh.polygons[j].vertices[0]
						|| ParaGlobalMeshs.polygons[i].vertices[1] == WithHalfEdgePolygonMesh.polygons[j].vertices[1]
						|| ParaGlobalMeshs.polygons[i].vertices[1] == WithHalfEdgePolygonMesh.polygons[j].vertices[2])

					&& (ParaGlobalMeshs.polygons[i].vertices[2] == WithHalfEdgePolygonMesh.polygons[j].vertices[0]
						|| ParaGlobalMeshs.polygons[i].vertices[2] == WithHalfEdgePolygonMesh.polygons[j].vertices[1]
						|| ParaGlobalMeshs.polygons[i].vertices[2] == WithHalfEdgePolygonMesh.polygons[j].vertices[2]))
				{
					Find = true;
					continue;
				}
			}
			if (!Find)
			{
				Viewer->addLine(InputCloud->points[ParaGlobalMeshs.polygons[i].vertices[0]],
					InputCloud->points[ParaGlobalMeshs.polygons[i].vertices[1]], 255, 255, 0,
					"L0" + StringBase::ClockValue() + StringBase::IntToStr(ParaGlobalMeshs.polygons[i].vertices[1]));
				Viewer->addLine(InputCloud->points[ParaGlobalMeshs.polygons[i].vertices[1]],
					InputCloud->points[ParaGlobalMeshs.polygons[i].vertices[2]], 255, 255, 0,
					StringBase::ClockValue() + "L1" + StringBase::IntToStr(ParaGlobalMeshs.polygons[i].vertices[2]));
				Viewer->addLine(InputCloud->points[ParaGlobalMeshs.polygons[i].vertices[0]],
					InputCloud->points[ParaGlobalMeshs.polygons[i].vertices[2]], 255, 255, 0,
					StringBase::ClockValue() + StringBase::IntToStr(ParaGlobalMeshs.polygons[i].vertices[0]) + "L2");

				
				Viewer->addText3D(StringBase::IntToStr(ParaGlobalMeshs.polygons[i].vertices[0]),
					InputCloud->points[ParaGlobalMeshs.polygons[i].vertices[0]],
					TextScale, 255, 0, 0, "T0" + StringBase::ClockValue() + StringBase::IntToStr(ParaGlobalMeshs.polygons[i].vertices[0]));
				Viewer->addText3D(StringBase::IntToStr(ParaGlobalMeshs.polygons[i].vertices[1]),
					InputCloud->points[ParaGlobalMeshs.polygons[i].vertices[1]],
					TextScale, 255, 0, 0, "T1" + StringBase::ClockValue() + StringBase::IntToStr(ParaGlobalMeshs.polygons[i].vertices[1]));
				Viewer->addText3D(StringBase::IntToStr(ParaGlobalMeshs.polygons[i].vertices[2]),
					InputCloud->points[ParaGlobalMeshs.polygons[i].vertices[2]],
					TextScale, 255, 0, 0, "T2" + StringBase::ClockValue() + StringBase::IntToStr(ParaGlobalMeshs.polygons[i].vertices[2]));
					
			}
		}

	//*/


	/*
	for (int j = 0; j < ParaGlobalMeshs.polygons.size(); j++)
	//for (int j = 0; j < 1; j++)
	{
		Viewer->addText3D(StringBase::IntToStr(ParaGlobalMeshs.polygons[j].vertices[0]),
			InputCloud->points[ParaGlobalMeshs.polygons[j].vertices[0]], 100, 255, 0, 0, StringBase::ClockValue() + "0");
		Viewer->addText3D(StringBase::IntToStr(ParaGlobalMeshs.polygons[j].vertices[1]),
			InputCloud->points[ParaGlobalMeshs.polygons[j].vertices[1]], 100, 0, 255, 0, StringBase::ClockValue() + "1");
		Viewer->addText3D(StringBase::IntToStr(ParaGlobalMeshs.polygons[j].vertices[2]),
			InputCloud->points[ParaGlobalMeshs.polygons[j].vertices[2]], 100, 0, 0, 255, StringBase::ClockValue() + "2");
	}
	//*/

	//PointBase::SetPointColor(OutPointsAndBorderEdges[0].PartIndexs, ColorBase::RedColor);
	//PointBase::ShowPointXYZRGBIndex(Viewer, OutPointsAndBorderEdges[0].PartIndexs, "LeftCloud", 3);

	//PointBase::SetPointColor(OutPointsAndBorderEdges[1].PartIndexs, ColorBase::GreenColor);
	//PointBase::ShowPointXYZRGBIndex(Viewer, OutPointsAndBorderEdges[1].PartIndexs, "RightCloud", 3);

	//Viewer->addPolygonMesh(PolygonMesh, "DelaunayParallel");	
	
	if (DelaunayParallelForm.checkBoxShowDivideMesh->checkState())
	{
		//PointBase::MeshToTriangles(Viewer, InputCloud, GlobalMeshs, "DelaunayParallel");
		Viewer->removePolygonMesh("ParaGlobalMeshs");
		Viewer->addPolylineFromPolygonMesh(ParaGlobalMeshs, "ParaGlobalMeshs");
	}

	//PointBase::ShowPointXYZRGBText(Viewer, InputCloud, "Test", 1);	
	emitUpdateStatusBar("Parallel DelaunayGrowth has been done!", 3000);
	DelaunayParallelForm.pushButtonDelaunayGrowthParallel->setEnabled(true);
}

//批量处理以取得计算结果 2020.06.19
void CDelaunayParallel::BatRun()
{	
	DelaunayParallelForm.pushButtonBatRun->setEnabled(false);

	SaveFileName = "I:\\PlanePoints\\ParaResult.txt";

	vector<string> PFileNames;
	//PFileNames.push_back("I:\\PlanePoints\\Rectangle15000.pcd");
	PFileNames.push_back("I:\\PlanePoints\\Rectangle25000.pcd");

	/*PFileNames.push_back("I:\\PlanePoints\\Rectangle10000.pcd");
	PFileNames.push_back("I:\\PlanePoints\\Rectangle20000.pcd");
	PFileNames.push_back("I:\\PlanePoints\\Rectangle30000.pcd");*/
	//PFileNames.push_back("I:\\PlanePoints\\Rectangle40000.pcd");
	//PFileNames.push_back("I:\\PlanePoints\\Rectangle50000.pcd");
	//PFileNames.push_back("I:\\PlanePoints\\Rectangle70000.pcd");
	//PFileNames.push_back("I:\\PlanePoints\\Rectangle100000.pcd");

	//PFileNames.push_back("I:\\PlanePoints\\Test_10000.pcd");
	//PFileNames.push_back("I:\\PlanePoints\\Test_20000.pcd");
	//PFileNames.push_back("I:\\PlanePoints\\Test_30000.pcd");
	//PFileNames.push_back("I:\\PlanePoints\\Test_40000.pcd");
	//PFileNames.push_back("I:\\PlanePoints\\Test_50000.pcd");
	
	vector<int> PNums;
	//PNums.push_back(300);
	PNums.push_back(500);
	PNums.push_back(700);
	PNums.push_back(900);
	PNums.push_back(1100);
	//PNums.push_back(1300);
	//PNums.push_back(1500);
	//PNums.push_back(1700);
	//PNums.push_back(1900);

	//PNums.push_back(500);
	//PNums.push_back(1000);
	//PNums.push_back(1500);
	//PNums.push_back(2000);

	for (int III = 0; III < PFileNames.size(); III++)
	{
		OpenedFilePath = PFileNames[III];
		PointBase::OpenPCLFile(OpenedFilePath, InputCloud);		

		for (int II = 0; II < PNums.size(); II++)
		{
			DelaunayParallelForm.spinBoxSplitZonePoints->setValue(PNums[II]);

			IsSave = true;
			SumTimeHalfEdgeGrow = 0, SumTimeParaDivideDis = 0, SumTimeParaGrowth = 0;
			BatNum = 10;
			for (int i = 0; i < BatNum; i++)
			{				
				DelaunayParallelGrowth();
			}
			SumTimeHalfEdgeGrow = int(SumTimeHalfEdgeGrow / BatNum),
				SumTimeParaDivideDis = int(SumTimeParaDivideDis / BatNum),
				SumTimeParaGrowth = int(SumTimeParaGrowth / BatNum);

			SaveStr = GetDateTime() + "," +
				OpenedFilePath + "," +
				StringBase::FloatToStr(SumTimeHalfEdgeGrow) + "," +
				StringBase::IntToStr(DelaunayParallelForm.spinBoxSplitZonePoints->text().toInt()) + "," +
				StringBase::FloatToStr(K) + "," +				
				StringBase::FloatToStr(SumTimeParaDivideDis) + "," +
				StringBase::FloatToStr(SumTimeParaGrowth) + "," +
				StringBase::FloatToStr(SumTimeParaDivideDis + SumTimeParaGrowth) + "," +
				StringBase::FloatToStr(MaxNum) + "," +
				StringBase::FloatToStr(MinNum) + "," +
				StringBase::FloatToStr(AvgNum);
				
				cout << endl << "*******批量处理完成！" << endl;
			cout << "Bat Result:" << SaveStr << endl;
			if (SaveFileName.length() > 0)
				SaveStringToFile(SaveFileName, SaveStr);

			IsSave = false;
		}
	}	
	DelaunayParallelForm.pushButtonBatRun->setEnabled(true);
}

CDelaunayParallel::CDelaunayParallel(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	IsSave = false;
	DelaunayParallelForm.setupUi(widget);

	connect(DelaunayParallelForm.pushButtonConvexHull, SIGNAL(clicked()), this, SLOT(ConvexHullCom()));

	connect(DelaunayParallelForm.pushButtonDelaunayGrowth, SIGNAL(clicked()), this, SLOT(DelaunayGrowth()));

	connect(DelaunayParallelForm.pushButtonDelaunayGrowthParallel, SIGNAL(clicked()), 
		this, SLOT(DelaunayParallelGrowth()));

	connect(DelaunayParallelForm.pushButtonBatRun, SIGNAL(clicked()),
		this, SLOT(BatRun()));
	
	widget->show();

	SumTimeHalfEdgeGrow = SumTimeParaDivideDis = SumTimeParaGrowth = 0;
	K = AvgNum = MinNum = MaxNum = VarianceNum = 0;
}

//2020.05.23 计算凸包
void CDelaunayParallel::ConvexHullCom()
{
	emitUpdateStatusBar("Convex Hull Computation is working", 3000);

	if (!PointBase::PointsIs2D(InputCloud))
	{
		PointBase::Generate2DPoints(InputCloud, 1000);
		PointBase::SetPointColor(InputCloud, ColorBase::StemColor);
	}
	time_t c_start, c_end;
	c_start = clock();

	ConvexHull.SetInputs(InputCloud);
	ConvexhullIndexs.clear();
	//ConvexHull.GetPointsConvexHull(ConvexhullIndexs);
	ConvexHull.GetConvexHullOneByOne(ConvexhullIndexs);

	for (int j = 0; j < ConvexhullIndexs.size(); j++)
	{
		InputCloud->points[ConvexhullIndexs[j]].rgba = ColorBase::RedColor;
	}
	
	//cout << "凸包点个数：" << ConvexhullIndexs.size() << endl;
	for (int i = 0; i < ConvexhullIndexs.size(); i++)
	{
		cout << ConvexhullIndexs[i] << ",";
		Viewer->addLine(InputCloud->points[ConvexhullIndexs[i]],
			InputCloud->points[ConvexhullIndexs[(i + 1) % ConvexhullIndexs.size()]], 
			StringBase::IntToStr(i) + StringBase::ClockValue());
	}
	//cout << endl;

	//PointBase::ShowPointXYZRGBText(Viewer, InputCloud, "InputCloud", 1);
	c_end = clock();
	/*cout << "The time of ConvexHull Computation : "
		<< " times: " << difftime(c_end, c_start) << " ms" << endl;*/

	emitUpdateUI();
}

CDelaunayParallel::~CDelaunayParallel()
{
	if (Viewer != NULL)
	{		
		Viewer->removePointCloud("AnchorCloud");
		Viewer->removePointCloud("LeftCloud");
		Viewer->removePointCloud("RightCloud");
		Viewer->removePointCloud(DelaunayParallelStr);
		Viewer->removeAllShapes();		
	}

	emitUpdateUI();
}

//将Mesh另存为到文件中    2020.06.24
void CDelaunayParallel::SaveMeshToFile(pcl::PolygonMesh CurMesh, string FileName)
{
	
	for (int i = 0; i < CurMesh.polygons.size(); i++)
	{
		string TempStr = StringBase::IntToStr(CurMesh.polygons[i].vertices[0]+1)
			+ " " + StringBase::IntToStr(CurMesh.polygons[i].vertices[1] + 1)
			+ " " + StringBase::IntToStr(CurMesh.polygons[i].vertices[2] + 1);

		SaveStringToFile(FileName, TempStr);
	}
}



//2020.05.24 生长法计算Delaunay三角网
void CDelaunayParallel::DelaunayGrowth()
{
	emitUpdateStatusBar("DelaunayGrowth is working", 3000);
	pcl::PolygonMesh PolygonMesh;
	
	//////开始计时
	time_t c_start, c_end;
	c_start = clock();
	/*
	CDelaunayGrowthWithHalfEdge DelaunayGrowthWithHalfEdge;
	DelaunayGrowthWithHalfEdge.SetInputCloud(InputCloud);
	DelaunayGrowthWithHalfEdge.Viewer = Viewer;
	DelaunayGrowthWithHalfEdge.performReconstructionWithPriority();
	DelaunayGrowthWithHalfEdge.OutputMesh(PolygonMesh);
	//*/
	Viewer->removeAllShapes();
	Viewer->removePolygonMesh();
	///*
	CDelaunayGrowthAngleMaximizer DelaunayGrowthAngleMaximizer;
	DelaunayGrowthAngleMaximizer.SetInputs(InputCloud);
	DelaunayGrowthAngleMaximizer.performReconstruction(PolygonMesh);
	//*/

	cout << "DelaunayGrowthAngleMaximizer: "
		<< " times: " << difftime(clock(), c_start) << " ms" << endl;

	cout << "点个数:" << InputCloud->points.size() 
		<< " 三角形个数:" << PolygonMesh.polygons.size() << endl;
	//Viewer->addPolygonMesh(PolygonMesh);
	//Viewer->addPolylineFromPolygonMesh(PolygonMesh, "PolygonMesh");
	PointBase::MeshToTriangles(Viewer, InputCloud, PolygonMesh, "PolygonMesh");

	emitUpdateStatusBar("DelaunayGrowth has been done!", 3000);
}