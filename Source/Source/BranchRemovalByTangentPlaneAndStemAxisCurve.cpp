#include"BranchRemovalByTangentPlaneAndStemAxisCurve.h"

CBranchRemovalByTangentPlaneAndStemAxisCurve::
	CBranchRemovalByTangentPlaneAndStemAxisCurve(QGroupBox * ParentWin)
{
	TempGeometricalCenterPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	GlobalGeometricalCenterPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	PlanePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());	
	DownSamplePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	LowerCovexPologon.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	UpperCovexPologon.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

	Octree = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	VgDownSampleOctree = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	
	DoubleNormals.reset(new pcl::PointCloud<pcl::Normal>());
	HalfNormals.reset(new pcl::PointCloud<pcl::Normal>());
	CurvatureDirections.reset(new pcl::PointCloud<pcl::Normal>());

	QWidget *widget = new QWidget(ParentWin);
	BranchRemovalForm.setupUi(widget);

	connect(BranchRemovalForm.checkBoxSlices, SIGNAL(stateChanged(int)), this, SLOT(ShowSlicesPoints(int)));
	connect(BranchRemovalForm.checkBoxShowHeightPlane, SIGNAL(stateChanged(int)), this, SLOT(ShowHeightPlane(int)));

	connect(BranchRemovalForm.spinBoxThickNess, SIGNAL(valueChanged(int)), this, SLOT(ShowSlicesPoints(int)));
	connect(BranchRemovalForm.spinBoxStartHeight, SIGNAL(valueChanged(int)), this, SLOT(ShowHeightPlane(int)));

	connect(BranchRemovalForm.pushButtonBranchCheck, SIGNAL(clicked()), this, SLOT(CheckBranches()));
	connect(BranchRemovalForm.pushButtonRemoval, SIGNAL(clicked()), this, SLOT(DeleteBranches()));
	connect(BranchRemovalForm.pushButtonRedo, SIGNAL(clicked()), this, SLOT(Redo()));

	BranchRemovalForm.doubleSpinBox_RadiusForNormalEstimation->setEnabled(false);
	connect(BranchRemovalForm.radioButton_Radius, SIGNAL(toggled(bool)), this,
		SLOT(SetRadioRadius(bool)));	

	connect(BranchRemovalForm.pushButtonNormalCacluation,
		SIGNAL(clicked()), this, SLOT(NormalCalculation()));

	connect(BranchRemovalForm.checkBoxShowNormal,
		SIGNAL(stateChanged(int)), this, SLOT(ShowPointNormal(int)));

	widget->show();
	LastAvg = 0;
}

CBranchRemovalByTangentPlaneAndStemAxisCurve::~CBranchRemovalByTangentPlaneAndStemAxisCurve()
{
	TempGeometricalCenterPoints->points.clear();
	free(Octree);
	Octree = NULL;

	Viewer->removePointCloud(HeightPlanePointsStr);

	free(VgDownSampleOctree);
	VgDownSampleOctree = NULL;
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::RefreshParameters()
{
	SliceHeight = BranchRemovalForm.spinBoxThickNess->text().toDouble();
	NeighbourRadius = BranchRemovalForm.spinBoxRadius->text().toDouble();

	CalcSectionNumbers = floor(CalcUnitLength / SliceHeight);

	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetThickNess(SliceHeight);
	HorizontalPartition.PatitionSection();
	HorizontalPartition.SetViewer(Viewer);
	StemSkeleton.CentroidPoints->points.clear();
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::
	SetViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue)
{
	CTreeBase::SetViewer(ViewerValue);
	RefreshParameters();
	FindHeightPlane(BranchRemovalForm.spinBoxStartHeight->text().toDouble());
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::ShowDirection()
{	

	//PointGeometry.CalcDirectionRadius(BranchRemovalForm.spinBoxRadius->text().toDouble(), 
	//	PointGeometry.Cloud_Normals, false);
	//PointGeometry.CalcDirectionKNN(8, PointGeometry.Cloud_Normals, false);
	
	ShowNormals(InputCloud, PointGeometry.Cloud_Normals, "Cloud_Normals");
	//ShowPrincipalCurvatures(InputCloud, PointGeometry.Cloud_Normals, PointGeometry.Cloud_Curvatures, "PrincipalCurvatures");
	
	//for (int i = 0; i < InputCloud->points.size(); i++)
	//{
	//	vector<int> NeighbourIndexs;
	//	vector<float> NeighbourDis;

	//	GeometryBase::GetNeighbourInRadius(Octree, NeighbourRadius,
	//		i, &NeighbourIndexs, &NeighbourDis);

	//	int Count = 0;
	//	for (int j = 0; j < NeighbourIndexs.size(); j++)
	//	{
	//		double TempAngle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector((*PointGeometry.Cloud_Normals)[i],
	//			(*PointGeometry.Cloud_Normals)[NeighbourIndexs[j]]));
	//		if (TempAngle > 90) TempAngle = 180 - 90;
	//		//cout << "I:" << i << "J:"<< TempAngle <<endl;

	//		if (TempAngle > 20)
	//			Count++;
	//	}

	//	if (Count > NeighbourIndexs.size() * 3.0 / 4.0)
	//		InputCloud->points[i].rgba = ColorBase::GreenColor;
		//else if (Count == 2)
		//	InputCloud->points[i].rgba = ColorBase::DarkDoveColor;
		//else if (Count == 3)
		//	InputCloud->points[i].rgba = ColorBase::DarkGreenColor;
		//else if (Count == 4)
		//	InputCloud->points[i].rgba = ColorBase::DarkGreyColor;
		//else if (Count == 5)
		//	InputCloud->points[i].rgba = ColorBase::DarkPurpleColor;
		//else if (Count == 6)
		//	InputCloud->points[i].rgba = ColorBase::DoveColor;
	//}


	//vector<double> EigenValueRates;
	//PointGeometry.CalcEigenValueRateRadius(BranchRemovalForm.spinBoxRadius->text().toDouble(), 
	//	&EigenValueRates, 1, true);
	////PointGeometry.CalcEigenValueRateKNN(20, &EigenValueRates, 1, true);

	////PointBase::SetPointColor(InputCloud, ColorBase::RedColor);
	//for (int i = 0; i < EigenValueRates.size(); i++)
	//{
	//	//if (EigenValueRates[i] > 0.85)
	//	if (EigenValueRates[i] > 0.70)			
	//		InputCloud->points[i].rgba = ColorBase::RedColor;
	//	//InputCloud->points[i].r = 255.0 * EigenValueRates[i];
	//	//else //if ((*PointGeometry.Cloud_Curvatures)[i].pc1 > 0.1)
	//	//{
	//	//	InputCloud->points[i].rgba = ColorBase::GreenColor;
	//	//	InputCloud->points[i].g = 255.0 * PointGeometry.GetMeanCurvature(i) * 100;
	//	//	//cout <<"I: "<<i<< InputCloud->points[i].g << endl;
	//	//}
	//}
	emitUpdateUI();
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::ShowSlicesPoints(int CheckValue)
{
	if (InputCloud == NULL) return;

	QAction * Action = (QAction *)sender();
	if (Action == NULL) return;

	if (Action->objectName() == "spinBoxThickNess" || HorizontalPartition.SectionsCount == 0)
		RefreshParameters();

	CheckValue = BranchRemovalForm.checkBoxSlices->checkState();

	if (CheckValue == 2)
		HorizontalPartition.ShowSectionPoints(0, HorizontalPartition.SectionsCount);
	else if (CheckValue != 2)
		HorizontalPartition.UnShowSectionPoints();
	emitUpdateUI();
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::FindHeightPlane(int Value)
{
	int SectionIndex = HorizontalPartition.FindSectionIndex(Value);

	if (SectionIndex >= HorizontalPartition.SectionsCount - 2)
		SectionIndex = HorizontalPartition.SectionsCount - 2;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	HorizontalPartition.GetSectionPoints(SectionIndex + 1, SectionPoints);
	double TempR = GeometryBase::CircleFittingByLeastSquaresFitting(SectionPoints);
	//pcl::PointXYZRGB CenterPoint = GeometryBase::GetCentroidOfPoints(SectionPoints);
	pcl::PointXYZRGB CenterPoint = GeometryBase::GetCentroidOfPoints(SectionPoints);

	//2019.09.10 为统一法向方向而设定的参数
	NormalAnchorIndex = HorizontalPartition.SectionsVector[SectionIndex + 1].Indexs[0];
	NormalAnchorPoint = CenterPoint;

	if (BranchRemovalForm.checkBoxShowHeightPlane->checkState() == 2)
	{
		GeometryBase::GetCirclePoints(PlanePoints, CenterPoint,
			TempR * 2.0, true, 0.5);

		PointBase::SetPointsCoordinateValue(PlanePoints, "Z",
			HorizontalPartition.SectionsVector[SectionIndex + 1].ZMin);

		PointBase::SetPointColor(PlanePoints, ColorBase::BlueColor);

		ShowPoints(PlanePoints, HeightPlanePointsStr, PointSize);
	}
	else
	{
		Viewer->removePointCloud(HeightPlanePointsStr);
		emitUpdateUI();
	}
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::ShowHeightPlane(int Value)
{
	FindHeightPlane(BranchRemovalForm.spinBoxStartHeight->text().toDouble());
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::CalcGrowthDirectionAtStartHeight(double Height, 
	bool toUp)
{
	double SmallHeight = EPSP6;
	int PointIndex = -1;

	int SectionIndex = HorizontalPartition.FindSectionIndex(Height);
	int StartIndex = SectionIndex - CalcSectionNumbers;
	
	assert(SectionIndex);
	assert(StartIndex);

	StemSkeleton.SetInputs(InputCloud, SliceHeight, true, CalcSectionNumbers, StartIndex);
	
	CurrentGrowthDirection = StemSkeleton.GetInitialVector();
	cout<< StemSkeleton.CentroidPoints->points.size() <<endl;
	SliceCount = 5;
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::CheckBranchByGradientDescent()
{
	VgDownSample.setInputCloud(InputCloud);
	VgDownSample.setLeafSize(1, 1, 1);
	VgDownSample.filter(*DownSamplePoints);
	
	VgDownSampleOctree->setInputCloud(DownSamplePoints);
	VgDownSampleOctree->addPointsFromInputCloud();

	HorizontalPartition.SetInputCloud(DownSamplePoints);
	HorizontalPartition.SetThickNess(SliceHeight);
	HorizontalPartition.PatitionSection();
	
	AmendSkeletonCentriodPoints();

	StemSkeleton.ConstructStemSplineCurve(true);
	PointBase::SetPointColor(StemSkeleton.StemSkeletonSpline.CurvePoints, ColorBase::RedColor);
	ShowPoints(StemSkeleton.StemSkeletonSpline.CurvePoints, "CurvePoints", PointSize);

	int BlueNum = 0;
	for (int i = 0; i < StemSkeleton.CentroidPoints->points.size(); i++)
	{
		if (StemSkeleton.CentroidPoints->points[i].rgba == ColorBase::RedColor)		
			BlueNum = 0;		
		else
			BlueNum++;

		if (BlueNum >= 10) //有连续的10个
		{
			pcl::PointXYZRGB LowerPoint = StemSkeleton.CentroidPoints->points[i - 3];
			pcl::PointXYZRGB UpperPoint = StemSkeleton.CentroidPoints->points[i - 2];

			double U = StemSkeleton.StemSkeletonSpline.GetUValueBySplineHeight(LowerPoint.z);
			pcl::PointXYZRGB TempGrowthDirection = StemSkeleton.StemSkeletonSpline.GetSplineDerivativePoint(U, 1, true);			
			
			double dLower = -(LowerPoint.x * TempGrowthDirection.x
				+ LowerPoint.y * TempGrowthDirection.y
				+ LowerPoint.z * TempGrowthDirection.z);
			double dUpper = -(UpperPoint.x * TempGrowthDirection.x
				+ UpperPoint.y * TempGrowthDirection.y
				+ UpperPoint.z * TempGrowthDirection.z);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr BewteenPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
			vector<int> BewteenIndexs;

			GeometryBase::GetPointsBetweenTwoPlanes(DownSamplePoints, TempGrowthDirection.x,
				TempGrowthDirection.y, TempGrowthDirection.z, dLower, TempGrowthDirection.x,
				TempGrowthDirection.y, TempGrowthDirection.z, dUpper, BewteenPoints, 1, 
				true, false, false, &BewteenIndexs);

			for (int j = 0; j < BewteenIndexs.size(); j++)
			{
				//Seed Points;
				DownSamplePoints->points[BewteenIndexs[j]].rgba = ColorBase::YellowColor;
			}
		}
	}
	//ShowPoints(DownSamplePoints, "DownSamplePoints", PointSize);

	for (int i = 0; i < DownSamplePoints->points.size(); i++)
	{
		if (DownSamplePoints->points[i].rgba == ColorBase::YellowColor) // Seed Point
		{
			int Count = 0;
			//SeedBroadcast(i, Count);
		}
	}
	//ShowPoints(DownSamplePoints, "DownSamplePoints", PointSize);
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::AmendSkeletonCentriodPoints()
{
	int SuccessiveK = 2;
	int AllowAnlge = 170;

	PointBase::SetPointColor(StemSkeleton.CentroidPoints, ColorBase::RedColor);

	for (int i = SuccessiveK; i < StemSkeleton.CentroidPoints->points.size() - SuccessiveK; i++)
	{
		bool IsOk = true;
		for (int j = i - SuccessiveK + 1; j < i + SuccessiveK - 1; j++)
		{
			double Anlge = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
				StemSkeleton.CentroidPoints->points[j -1], StemSkeleton.CentroidPoints->points[j],
				StemSkeleton.CentroidPoints->points[j + 1]));
			if (Anlge < 170)
			{
				IsOk = false;
				break;
			}
		}
		if (IsOk)
		{
			#pragma omp parallel for
			for (int j = i - SuccessiveK; j < i + SuccessiveK; j++)
			{
				StemSkeleton.CentroidPoints->points[j].rgba = ColorBase::BlueColor;
			}
		}
	}

	ShowPoints(StemSkeleton.CentroidPoints, CentroidPointsStr, 5);

	for (int i = StemSkeleton.CentroidPoints->points.size() - 1; i >= 0; i--)
	{
		if (StemSkeleton.CentroidPoints->points[i].rgba == ColorBase::RedColor)
		{
			StemSkeleton.CentroidPoints->points.erase(StemSkeleton.CentroidPoints->points.begin() + i);
		}
	}	
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::SeedBroadcast(int PointIndex, int & Count)
{
	vector<int> NeighbourIndexs;
	vector<float> NeighbourDis;
	if (Count > 30) return;
	Count++;
	GeometryBase::GetNeighbourInRadius(VgDownSampleOctree, NeighbourRadius,
		PointIndex, &NeighbourIndexs, &NeighbourDis);

	double SkeletonU = StemSkeleton.StemSkeletonSpline.GetUValueBySplineHeight(DownSamplePoints->points[PointIndex].z);
	pcl::PointXYZRGB TempGrowthDirection = StemSkeleton.StemSkeletonSpline.GetSplineDerivativePoint(SkeletonU, 1, true);

	for (int j = 0; j < NeighbourIndexs.size(); j++)
	{
		if (DownSamplePoints->points[NeighbourIndexs[j]].rgba != ColorBase::YellowColor) // Not Seed Point
		{
			pcl::PointXYZRGB Direction;
			Direction.x = DownSamplePoints->points[NeighbourIndexs[j]].x - DownSamplePoints->points[PointIndex].x;
			Direction.y = DownSamplePoints->points[NeighbourIndexs[j]].y - DownSamplePoints->points[PointIndex].y;
			Direction.z = DownSamplePoints->points[NeighbourIndexs[j]].z - DownSamplePoints->points[PointIndex].z;

			double TempAngle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(Direction, TempGrowthDirection));			
			if (TempAngle > 80 && TempAngle < 100 )
			{
				cout << "TempAngle:" << TempAngle << endl;
				DownSamplePoints->points[NeighbourIndexs[j]].rgba = ColorBase::YellowColor;
				SeedBroadcast(NeighbourIndexs[j], Count);
			}
		}	
	}
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::CheckBranchByPriorBark(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SlicePoints,
	vector<int> SlicePointsIndexs, vector<AnglePartitionStruct> & PriorSectionAngle, double AnlgeValue)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Slice2DPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Slice2DXY(new pcl::PointCloud<pcl::PointXYZRGB>());

	SlicePoints->points.push_back(PriorCurrentCenterPoint);
	GeometryBase::ProjectPointsToPlane(SlicePoints, Slice2DPoints,
		CurrentGrowthDirection, PriorCurrentCenterPoint);
	GeometryBase::RotateNormalToVertical(Slice2DPoints, Slice2DXY, CurrentGrowthDirection);
	
	PointsMove(Slice2DXY, PriorCurrentCenterPoint.x - Slice2DXY->points[Slice2DXY->points.size() - 1].x,
		PriorCurrentCenterPoint.y - Slice2DXY->points[Slice2DXY->points.size() - 1].y,
		PriorCurrentCenterPoint.z - Slice2DXY->points[Slice2DXY->points.size() - 1].z);		

	pcl::PointXYZRGB CentroidPoint = Slice2DXY->points[Slice2DXY->points.size() - 1];

	CAnglePartition TempAnglePartition;
	vector<AnglePartitionStruct> SectionAngle;

	//PointBase::SetPointColor(Slice2DXY, ColorBase::BlueColor);
	//Slice2DXY->points[Slice2DXY->points.size() - 1].rgba = ColorBase::RedColor;
	//PointsMove(Slice2DXY, 0, 0, -1);
	//ShowPoints(Slice2DXY, "Slice2DXY" + StringBase::ClockValue(), 2);
	
	Slice2DXY->points.pop_back();
	TempAnglePartition.PartitionPoints(Slice2DXY, CentroidPoint, AnlgeValue, SectionAngle);
	   
	for (int i = 0; i < SectionAngle.size(); i++)
	{
		double MinDis = EPSP6;		
		for (int j = 0; j < SectionAngle[i].PointIndexs.size(); j++)
		{
			double Dis = PointDis(CentroidPoint, Slice2DXY->points[SectionAngle[i].PointIndexs[j]]);
			if (Dis < MinDis)
			{
				MinDis = MinDis < Dis ? MinDis : Dis;
			}
		}

		for (int j = 0; j < SectionAngle[i].PointIndexs.size(); j++)
		{
			double Dis = PointDis(CentroidPoint, Slice2DXY->points[SectionAngle[i].PointIndexs[j]]);
			//if (Dis > MinDis + CurBarkThickNess || Dis > PriorSectionAngle[i].MaxDis + CurBarkThickNess)
			if (Dis > MinDis + CurBarkThickNess)
			{
				InputCloud->points[SlicePointsIndexs[SectionAngle[i].PointIndexs[j]]].rgba = ColorBase::RedColor;
			}
			else if ( Dis > PriorSectionAngle[i].MaxDis + CurBarkThickNess && PriorSectionAngle[i].MaxDis > 1)
				InputCloud->points[SlicePointsIndexs[SectionAngle[i].PointIndexs[j]]].rgba = ColorBase::RedColor;
		}

	}
}

bool CBranchRemovalByTangentPlaneAndStemAxisCurve::UpdateCurSlice(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SlicePoints,
	vector<int> SlicePointsIndexs, pcl::PointXYZRGB PlaneAnchorPoint)
{
	SlicePoints->points.clear();
	for (int i = SlicePointsIndexs.size() - 1; i >= 0; i--)
	{
		int PointIndex = SlicePointsIndexs[i];
		pcl::PointXYZRGB CurrentPoint = InputCloud->points[PointIndex];

		if (CurrentPoint.rgba != ColorBase::RedColor)
			SlicePoints->points.push_back(CurrentPoint);
	}

	if (SlicePoints->points.size() == 0) return true;
	
	
	pcl::PointXYZRGB TempCentroidPoint =
		GeometryBase::GetCentroidOfPointsInSpaces(SlicePoints, CurrentGrowthDirection);

	//if (SliceCalced == 0)
	//	PointBase::SetPointColor(TempSlicePoints, ColorBase::RedColor);
	//else if (SliceCalced == 1)
	//	PointBase::SetPointColor(TempSlicePoints, ColorBase::BlueColor);
	//else if (SliceCalced == 2)
	//	PointBase::SetPointColor(TempSlicePoints, ColorBase::GreenColor);

	//ShowPoints(TempSlicePoints, "TempSlicePoints"+ StringBase::ClockValue(), 5);

	SlicePoints->points.push_back(TempCentroidPoint);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempOutPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	GeometryBase::ProjectPointsToPlane(SlicePoints, TempOutPoints, CurrentGrowthDirection, PlaneAnchorPoint);

	TempCentroidPoint = TempOutPoints->points[TempOutPoints->points.size() - 1];	
	TempCentroidPoint.rgba = ColorBase::BlueColor;

	/*
	//2019.06.21  如果TempCentroidPoint与前一个点形成的角度大于特定阈值，则修正方向依次 不可行 存在着异常点到正常点过度偏大的情况
	pcl::PointXYZRGB NewDirection;
	NewDirection.x = CurrentCenterPoint.x - TempCentroidPoint.x;
	NewDirection.y = CurrentCenterPoint.y - TempCentroidPoint.y;
	NewDirection.z = CurrentCenterPoint.z - TempCentroidPoint.z;

	double NewAngle = GeometryBase::AngleOfTwoVector(NewDirection, CurrentGrowthDirection);
	//if (NewAngle > M_PI / 12)
	if (NewAngle > GeometryBase::AngleToRadian(10))
		TempCentroidPoint.rgba = ColorBase::RedColor;
	//*/

	////cout << "NewCenter" << TempCentroidPoint << endl;
	StemSkeleton.CentroidPoints->points.push_back(TempCentroidPoint);

	//PointBase::SetPointColor(StemSkeleton.CentroidPoints, ColorBase::BlueColor);
	//cout << "Num:"<< StemSkeleton.CentroidPoints->points.size()<<endl;
	//ShowPoints(StemSkeleton.CentroidPoints, CentroidPointsStr, 5);	
}

double CBranchRemovalByTangentPlaneAndStemAxisCurve::CalcBarkThicknessofPriorSlice(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlicePoints, pcl::PointXYZRGB PriorCurrentCenterPoint,
	vector<AnglePartitionStruct> & SectionAngle, double Anlge)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlice2DPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlice2DXY(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	PriorSlicePoints->points.push_back(PriorCurrentCenterPoint);
	GeometryBase::ProjectPointsToPlane(PriorSlicePoints, PriorSlice2DPoints,
		CurrentGrowthDirection, PriorCurrentCenterPoint);
	//pcl::PointXYZRGB AnchorPoint = PriorSlice2DPoints->points[PriorSlice2DPoints->points.size() - 1];
/*	PointsMove(PriorSlice2DPoints, PriorCurrentCenterPoint.x - AnchorPoint.x,
		PriorCurrentCenterPoint.y - AnchorPoint.y, PriorCurrentCenterPoint.z - AnchorPoint.z);	*/	

	GeometryBase::RotateNormalToVertical(PriorSlice2DPoints, PriorSlice2DXY, CurrentGrowthDirection);
	pcl::PointXYZRGB CentroidPoint = PriorSlice2DXY->points[PriorSlice2DXY->points.size() - 1];

	PointsMove(PriorSlice2DXY, PriorCurrentCenterPoint.x - CentroidPoint.x,
		PriorCurrentCenterPoint.y - CentroidPoint.y, PriorCurrentCenterPoint.z - CentroidPoint.z);
	CentroidPoint = PriorSlice2DXY->points[PriorSlice2DXY->points.size() - 1];

	//PointBase::SetPointColor(PriorSlice2DXY, ColorBase::YellowColor);
	//PriorSlice2DXY->points[PriorSlice2DXY->points.size() - 1].rgba = ColorBase::RedColor;
	//ShowPoints(PriorSlice2DXY, "PriorSlice2DXY", 4);
	
	PriorSlice2DXY->points.pop_back();
	CAnglePartition TempAnglePartition;	
	TempAnglePartition.PartitionPoints(PriorSlice2DXY, CentroidPoint, Anlge, SectionAngle);

	CurBarkThickNess = 0;		
	
	int Num = 0;
	for (int i = 0; i < SectionAngle.size(); i++)
	{
		double MinDis = EPSP6, MaxDis = EPSM6;
		for (int j = 0; j < SectionAngle[i].PointIndexs.size(); j++)
		{
			double Dis = PointDis(CentroidPoint, PriorSlice2DXY->points[SectionAngle[i].PointIndexs[j]]);
			MinDis = MinDis < Dis ? MinDis : Dis;
			MaxDis = MaxDis > Dis ? MaxDis : Dis;
		}

		if (SectionAngle[i].PointIndexs.size() > 0)
		{			
			SectionAngle[i].MaxDis = MaxDis;
			SectionAngle[i].MinDis = MinDis;
			CurBarkThickNess += (MaxDis - MinDis);
			Num++;
		}
	}
	CurBarkThickNess = CurBarkThickNess / Num;
	
	return CurBarkThickNess;
}

bool CBranchRemovalByTangentPlaneAndStemAxisCurve::GetPriorSlice
	(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlicePoints, vector<int> & PriorSlicePointsIndexs,
		bool toUp, double PriorSliceThickNess, pcl::PointXYZRGB & PriorCurrentCenterPoint)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCentroidPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	if (SliceCalced == 0)
	{
		int TempStartIndex;
		if (toUp)
		{
			TempStartIndex = StemSkeleton.CentroidPoints->points.size() - CalcSectionNumbers + 1;
			for (int i = TempStartIndex; i < StemSkeleton.CentroidPoints->points.size(); i++)
			{
				TempCentroidPoints->points.push_back(StemSkeleton.CentroidPoints->points[i]);
			}
		}
		else
		{
			for (int i = 0; i < CalcSectionNumbers; i++)
			{
				TempCentroidPoints->points.push_back(StemSkeleton.CentroidPoints->points[i]);
			}
		}
		CurrentGrowthDirection = GeometryBase::GetMaxDirectionVector(TempCentroidPoints);
	}

	SliceCalced = (SliceCalced + 1) % CalcSectionNumbers;
	cout << " SliceCalced: " << SliceCalced << " CalcSectionNumbers: " << CalcSectionNumbers << endl;	
		
	StemSkeleton.GetPriorSlicePoints(CurrentGrowthDirection, PriorSlicePoints,
		PriorCurrentCenterPoint, toUp, &PriorSlicePointsIndexs, PriorSliceThickNess);	

	if (PriorSlicePoints->points.size() <= 0)
		return true;

	if (toUp)
	{
		PriorCurrentCenterPoint = StemSkeleton.CentroidPoints->points[StemSkeleton.CentroidPoints->points.size() - 2];
	}
	else
	{
		PriorCurrentCenterPoint = StemSkeleton.CentroidPoints->points[1];
	}
}

bool CBranchRemovalByTangentPlaneAndStemAxisCurve::CheckBranchForNextSliceByBarkThickness(	 	
	bool toUp)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlicePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	vector<int> PriorSlicePointsIndexs;

	GetPriorSlice(PriorSlicePoints, PriorSlicePointsIndexs, toUp, 
		SliceHeight * (CalcSectionNumbers / 2), PriorCurrentCenterPoint);
	
	for (int i = PriorSlicePoints->points.size() - 1; i >=0; i--)
	{
		if (PriorSlicePoints->points[i].rgba == ColorBase::RedColor)
		{
			PriorSlicePoints->points.erase(PriorSlicePoints->points.begin() + i);
			PriorSlicePointsIndexs.erase(PriorSlicePointsIndexs.begin() + i);
		}
	}

	if (PriorSlicePoints->points.size() < 0)
		return true;

	//if (SliceCalced % 3 == 1)
	//	PointBase::SetPointColor(PriorSlicePoints, ColorBase::GreyColor);
	//else if (SliceCalced % 3 == 2)
	//	PointBase::SetPointColor(PriorSlicePoints, ColorBase::GreenColor);
	//else if (SliceCalced % 3 == 0)
	//	PointBase::SetPointColor(PriorSlicePoints, ColorBase::BlueColor);
	//ShowPoints(PriorSlicePoints, "PriorSlicePoints" + StringBase::ClockValue(), 2);
	vector<AnglePartitionStruct> SectionAngle;

	CurBarkThickNess = CalcBarkThicknessofPriorSlice(PriorSlicePoints, PriorCurrentCenterPoint, SectionAngle, 2);
	
	vector<int> CurSlicePointsIndexs;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePoints(new pcl::PointCloud<pcl::PointXYZRGB>());	
	pcl::PointXYZRGB PlaneAnchorPoint = StemSkeleton.GetSlicePoints(CurrentGrowthDirection, CurSlicePoints,
		CurrentCenterPoint, toUp, &CurSlicePointsIndexs);	

	CheckBranchByPriorBark(CurSlicePoints, CurSlicePointsIndexs, SectionAngle, 5);
	cout<<"CurBarkThickNess: "<< CurBarkThickNess <<endl;
	UpdateCurSlice(CurSlicePoints, CurSlicePointsIndexs, PlaneAnchorPoint);

	if (SliceCalced % 3 == 1)
		PointBase::SetPointColor(CurSlicePoints, ColorBase::GreyColor);
	else if (SliceCalced % 3 == 2)
		PointBase::SetPointColor(CurSlicePoints, ColorBase::GreenColor);
	else if (SliceCalced % 3 == 0)
		PointBase::SetPointColor(CurSlicePoints, ColorBase::BlueColor);
	ShowPoints(CurSlicePoints, "CurSlicePoints" + StringBase::ClockValue(), 2);

	return false;
}

bool CBranchRemovalByTangentPlaneAndStemAxisCurve::CheckBranchForNextSliceUseConvexPolygon(bool toUp)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCentroidPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	if (SliceCalced == 0)
	{
		int TempStartIndex;
		if (toUp)
		{
			TempStartIndex = StemSkeleton.CentroidPoints->points.size() - CalcSectionNumbers + 1;
			for (int i = TempStartIndex; i < StemSkeleton.CentroidPoints->points.size(); i++)
			{				
				TempCentroidPoints->points.push_back(StemSkeleton.CentroidPoints->points[i]);
			}
		}
		else
		{			
			for (int i = 0; i < CalcSectionNumbers; i++)
			{
				TempCentroidPoints->points.push_back(StemSkeleton.CentroidPoints->points[i]);
			}
		}
		CurrentGrowthDirection = GeometryBase::GetMaxDirectionVector(TempCentroidPoints);
	}
	SliceCalced = (SliceCalced + 1) % CalcSectionNumbers;
	cout << " SliceCalced: " << SliceCalced << " CalcSectionNumbers: " << CalcSectionNumbers << endl;
	vector<int> PriorSlicePointsIndexs;
	pcl::PointXYZRGB PriorCurrentCenterPoint, CurrentCenterPoint;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlicePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	StemSkeleton.GetPriorSlicePoints(CurrentGrowthDirection, PriorSlicePoints,
		PriorCurrentCenterPoint, toUp, &PriorSlicePointsIndexs, CalcSectionNumbers / 2.0 * SliceHeight);

	if (PriorSlicePoints->points.size() < 0)
		return true;		

	if (toUp)
	{
		PriorCurrentCenterPoint = StemSkeleton.CentroidPoints->points[StemSkeleton.CentroidPoints->points.size() - 2];		
	}
	else
	{
		PriorCurrentCenterPoint = StemSkeleton.CentroidPoints->points[1];		
	}

	//pcl::PointXYZRGB AnchorPoint3D, AnchorPoint2D;
	//AnchorPoint3D = PriorCurrentCenterPoint;

	vector<int> HullIndexs;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlice2DPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlice2DConvexHullPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	GeometryBase::ProjectPointsToPlane(PriorSlicePoints, PriorSlice2DPoints, 
		CurrentGrowthDirection, PriorCurrentCenterPoint);	
	GeometryBase::GetCentroidAndConvexHullOfPointsInSpaces(PriorSlice2DPoints, 
		PriorSlice2DConvexHullPoints, CurrentGrowthDirection);
	
	//PointBase::SetPointColor(PriorSlice2DConvexHullPoints, ColorBase::RedColor);
	//ShowPoints(PriorSlice2DConvexHullPoints, "PriorSlice2DConvexHullPoints", 5);

	//PointBase::SetPointColor(PriorSlice2DPoints, ColorBase::RedColor);
	//ShowPoints(PriorSlice2DPoints, "PriorSlice2DPoints", 2);
	
	//*  根据凸包多边形 去除边界
	CBezierSubsectionG2 BezierSubsectionG2;	
	BezierSubsectionG2.SetInputs(PriorSlice2DConvexHullPoints);
	BezierSubsectionG2.ResolveControlPoints(BezierMiu);
	BezierSubsectionG2.DrawBezierCurve(LowerCovexPologon, 360);
	
	//LowerCovexPologon->points.clear();
	//PointBase::PointCopy(PriorSlice2DConvexHullPoints, LowerCovexPologon);

	PointBase::SetPointColor(LowerCovexPologon, ColorBase::YellowColor);
	ShowPoints(LowerCovexPologon, "LowerCovexPologon", 2, false, 0.1);

	pcl::PointXYZRGB MovePoint = GeometryBase::GetPointAlongLine(CurrentGrowthDirection, 
		LowerCovexPologon->points[0], 2 * SliceHeight);

	PointBase::PointCopy(LowerCovexPologon, UpperCovexPologon);
	cout <<"CurrentGrowthDirection:"<< CurrentGrowthDirection << endl;
	cout << "SliceHeight:" << SliceHeight << endl;
	PointsMove(UpperCovexPologon, MovePoint.x - LowerCovexPologon->points[0].x,
		MovePoint.y - LowerCovexPologon->points[0].y, 
		MovePoint.z - LowerCovexPologon->points[0].z);
	
	PointBase::SetPointColor(UpperCovexPologon, ColorBase::YellowColor);	
	ShowPoints(UpperCovexPologon, "UpperCovexPologon", 2, false, 0.1);

	//GeometryBase::GetCentroidAndConvexHullOfPointsInSpaces(BezierPoints, 
	//	PriorSlice2DConvexHullPoints, CurrentGrowthDirection);
	//PointBase::SetPointColor(PriorSlice2DConvexHullPoints, ColorBase::GreenColor);
	//ShowPoints(PriorSlice2DConvexHullPoints, "BezierPointsPriorSlice2DConvexHullPoints", 6);

	/*
	CSplineInterpolation SplineInterConvexCurve;
	SplineInterConvexCurve.SetInputs(PriorSlice2DConvexHullPoints, 3, true);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	vector<double> OutKnotValue;
	SplineInterConvexCurve.GetControlPointsAndKnotValue(ControlPoints, OutKnotValue);
	CSpline SplineConvexCurve;
	SplineConvexCurve.SetSplineInputs(ControlPoints, 3, OutKnotValue, true);
	SplineConvexCurve.FirstPointOfCloseCurve = PriorSlice2DConvexHullPoints->points[0];
	SplineConvexCurve.CreateSpline();

	PointBase::SetPointColor(SplineConvexCurve.CurvePoints, ColorBase::YellowColor);
	ShowPoints(SplineConvexCurve.CurvePoints, "SplineConvexCurve", 2);
	//*/

	vector<PlaneCoeff> CoeffValues;
	//#pragma omp parallel for
	for (int i = 0; i < UpperCovexPologon->points.size(); i++)
	{
		PlaneCoeff TempPlaneCoeff;
		
		int NextPoint = (i + 1) % UpperCovexPologon->points.size();
		double a, b, c, d;
		GeometryBase::GetPlaneByThreePoints(LowerCovexPologon->points[i],
			UpperCovexPologon->points[i],
			UpperCovexPologon->points[NextPoint], a, b, c, d);

		TempPlaneCoeff.a = a, TempPlaneCoeff.b = b, TempPlaneCoeff.c = c, TempPlaneCoeff.d = d;
		TempPlaneCoeff.PlaneValue = a * PriorCurrentCenterPoint.x + b * PriorCurrentCenterPoint.y 
			+ c * PriorCurrentCenterPoint.z	+ d;

		CoeffValues.push_back(TempPlaneCoeff);
	}

	vector<int> CurSlicePointsIndexs;
	
	//Current Slice
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	//cout<<"CurrentGrowthDirection："<< CurrentGrowthDirection <<endl;
	pcl::PointXYZRGB ThirdPoint = StemSkeleton.GetSlicePoints(CurrentGrowthDirection, CurSlicePoints,
		CurrentCenterPoint, toUp, &CurSlicePointsIndexs);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlice2DPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	GeometryBase::ProjectPointsToPlane(CurSlicePoints, CurSlice2DPoints,
		CurrentGrowthDirection, UpperCovexPologon->points[0]);

	PointBase::SetPointColor(CurSlice2DPoints, ColorBase::GreenColor);
	ShowPoints(CurSlice2DPoints, "CurSlice2DPoints", 2);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlice2DMovePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	//PointBase::PointCopy(CurSlice2DPoints, CurSlice2DMovePoints);
	
	PointBase::PointCopy(UpperCovexPologon, CurSlice2DMovePoints);
	//PointsMove(CurSlice2DMovePoints, 10, 5, -5);
	//
	pcl::PointXYZRGB FixCentroid =
		GeometryBase::GetCentroidOfPointsInSpaces(UpperCovexPologon, CurrentGrowthDirection);

	pcl::PointXYZRGB MoveCentroid =
		GeometryBase::GetCentroidOfPointsInSpaces(CurSlice2DMovePoints, CurrentGrowthDirection);
	PointsMove(CurSlice2DMovePoints, FixCentroid.x - MoveCentroid.x,
		FixCentroid.y - MoveCentroid.y, FixCentroid.z - MoveCentroid.z);

	CTwoPointsSetMatching TwoPointsSetMatching;
	TwoPointsSetMatching.SetInput(UpperCovexPologon, CurSlice2DMovePoints, CurrentGrowthDirection);
	vector<double> Move = TwoPointsSetMatching.Optimal();
	cout<<"X:"<< Move[0] << "Y:" << Move[1] << "Z:" << Move[2] <<endl;
	
	//PointBase::PointCopy(CurSlice2DPoints, CurSlice2DMovePoints);
	PointBase::PointCopy(UpperCovexPologon, CurSlice2DMovePoints);
	PointBase::SetPointColor(CurSlice2DMovePoints, ColorBase::RedColor);
	PointsMove(CurSlice2DMovePoints, -(FixCentroid.x - MoveCentroid.x),
		-(FixCentroid.y - MoveCentroid.y), -(FixCentroid.z - MoveCentroid.z));
	PointsMove(CurSlice2DMovePoints, Move[0], Move[1], Move[2]);
	//PointsMove(CurSlice2DMovePoints, -10, -5, 5);	
	ShowPoints(CurSlice2DMovePoints, "CurSlice2DMovePoints", 2);
	//*/

	///*
	for (int i = 0; i < CurSlicePoints->points.size(); i++)
	{
		for (int j = 0; j < UpperCovexPologon->points.size(); j++)
		{
			//int NextPoint = (i + 1) % UpperCovexPologon->points.size();
			//double a, b, c, d;
			//GeometryBase::GetPlaneByThreePoints(LowerCovexPologon->points[i],
			//	UpperCovexPologon->points[i],
			//	UpperCovexPologon->points[NextPoint], a, b, c, d);

			double TempValue = CoeffValues[j].a * InputCloud->points[CurSlicePointsIndexs[i]].x
				+ CoeffValues[j].b * InputCloud->points[CurSlicePointsIndexs[i]].y
				+ CoeffValues[j].c * InputCloud->points[CurSlicePointsIndexs[i]].z
				+ CoeffValues[j].d;

			//double TempValue1 = CoeffValues[j].a * PriorCurrentCenterPoint.x
			//	+ CoeffValues[j].b * PriorCurrentCenterPoint.y
			//	+ CoeffValues[j].c * PriorCurrentCenterPoint.z
			//	+ CoeffValues[j].d;

			if (TempValue * CoeffValues[j].PlaneValue < 0) //位于平面两侧
			{
				InputCloud->points[CurSlicePointsIndexs[i]].rgba = ColorBase::RedColor;
				break;
			}
		}		
	}

	//*/

	//pcl::PointXYZRGB GravityCenter;
	//GravityCenter.x = 0, GravityCenter.y = 0, GravityCenter.z = PriorCurrentCenterPoint.z;

	//for (int i = 0; i < PriorSlicePointsIndexs.size(); i++)
	//{
	//	int PointIndex = PriorSlicePointsIndexs[i];		

	//	GravityCenter.x += InputCloud->points[PointIndex].x / PriorSlicePointsIndexs.size();
	//	GravityCenter.y += InputCloud->points[PointIndex].y / PriorSlicePointsIndexs.size();
	//}
	//
	//// the distance of prior slice
	//// 首先根据重心移除 异常点
	//double MaxR = EPSM6, MinR = EPSP6, AvgR = 0;
	//for (int i = 0; i < PriorSlicePointsIndexs.size(); i++)
	//{
	//	int PointIndex = PriorSlicePointsIndexs[i];
	//	pcl::PointXYZRGB CurrentPoint = InputCloud->points[PointIndex];

	//	double Dis = PointDis(InputCloud->points[PointIndex], GravityCenter);

	//	MaxR = Dis > MaxR ? Dis : MaxR;
	//	MinR = Dis < MinR ? Dis : MinR;
	//	AvgR += Dis / PriorSlicePointsIndexs.size();
	//}

	//double Alpha = (MaxR - MinR) / AvgR;
	//Alpha = Alpha > 0.40 ? 0.40 : Alpha;

	//if (LastAvg == 0)
	//	LastAvg = AvgR;

	//for (int i = PriorSlicePointsIndexs.size() - 1; i >= 0; i--)
	//{
	//	int PointIndex = PriorSlicePointsIndexs[i];
	//	double Dis = PointDis(InputCloud->points[PointIndex], PriorCurrentCenterPoint);

	//	if (Dis > (1 + 1.2*Alpha)*AvgR)
	//	{
	//		InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
	//		PriorSlicePointsIndexs.erase(PriorSlicePointsIndexs.begin() + i);
	//		//cout<<"Removed PointIndex:"<<PointIndex <<endl;
	//		//continue;
	//	}
	//}

	return false;
}

//2019.08.30 在根据距离判断的基础上，投影至平面，根据与中心点的角度判断树枝点
void CBranchRemovalByTangentPlaneAndStemAxisCurve::CheckBranchForSliceByProjectionAngle(
	vector<int> SlicePointsIndexs,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePoints,
	pcl::PointXYZRGB PriorCenterPoint,
	double AnlgeOfPartition)
{
	AnlgeOfPartition = 2;
	SmoothBigAngle = 180 - AnlgeOfPartition;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePointsOfXY(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PointXYZRGB CurrentCenterPointOfXY;
	CurSlicePoints->points.push_back(PriorCenterPoint);
	GeometryBase::RotateNormalToVertical(CurSlicePoints, CurSlicePointsOfXY, CurrentGrowthDirection);

	CurrentCenterPointOfXY = CurSlicePointsOfXY->points[CurSlicePointsOfXY->points.size() - 1];
	CurSlicePoints->points.pop_back();
	CurSlicePointsOfXY->points.pop_back();

	if (CurSlicePoints->points.size() > 0 &&
		(CurSlicePoints->points[0].z - ZMin >= 980 && CurSlicePoints->points[0].z - ZMin <= 982))
		PointBase::SavePCDToFileName(CurSlicePointsOfXY, "i:\\CurSlicePointsOfXY.pcd");

	//计算每个点与中心点的角度值
	vector<int> AngleValues;
	vector<int> AngleSortIndexs;
	for (int i = 0; i < CurSlicePointsOfXY->points.size(); i++)
	{
		double TempAngle = GeometryBase::AngleOfTwoPointsInXY(CurrentCenterPointOfXY.x,
			CurrentCenterPointOfXY.y,
			CurSlicePointsOfXY->points[i].x,
			CurSlicePointsOfXY->points[i].y);

		AngleValues.push_back(TempAngle);
		AngleSortIndexs.push_back(SlicePointsIndexs[i]);
	}

	//先角度分区，
	vector<AnglePartitionStruct> CurSectionAnglePartition;
	CAnglePartition AnglePartition;

	//以 CurrentCenterPointOfXY 为中心点、2度 为角度分区角度值 对当前分段进行角度分区
	AnglePartition.PartitionPoints(CurSlicePointsOfXY, CurrentCenterPointOfXY, AnlgeOfPartition, 
		CurSectionAnglePartition);
	//首先计算每个角度分区的中心点

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AnglePartitionOfXY(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (int i = 0; i < CurSectionAnglePartition.size(); i++)
	{
		int Count = CurSectionAnglePartition[i].PointIndexs.size();
		if (Count > 0)
		{
			CurSectionAnglePartition[i].CenterPointofCurPartition.x = 0,
				CurSectionAnglePartition[i].CenterPointofCurPartition.y = 0,
				CurSectionAnglePartition[i].CenterPointofCurPartition.z = 0;
			//计算每个角度分区的重心点
			for (int j = 0; j < Count; j++)
			{
				CurSectionAnglePartition[i].CenterPointofCurPartition.x =
					CurSectionAnglePartition[i].CenterPointofCurPartition.x +
					CurSlicePointsOfXY->points[CurSectionAnglePartition[i].PointIndexs[j]].x / Count;

				CurSectionAnglePartition[i].CenterPointofCurPartition.y =
					CurSectionAnglePartition[i].CenterPointofCurPartition.y +
					CurSlicePointsOfXY->points[CurSectionAnglePartition[i].PointIndexs[j]].y / Count;

				//CurSectionAnglePartition[i].CenterPointofCurPartition.z = 
				//	CurSectionAnglePartition[i].CenterPointofCurPartition.z +
				//	CurSlicePointsOfXY->points[CurSectionAnglePartition[i].PointIndexs[j]].z / Count;

				CurSectionAnglePartition[i].CenterPointofCurPartition.z = 0;
			}
			AnglePartitionOfXY->points.push_back(CurSectionAnglePartition[i].CenterPointofCurPartition);
		}
	}
	//PointBase::SavePCDToFileName(AnglePartitionOfXY, "i:\\AnglePartitionOfXY.pcd");

	//根据角度值对 AngleSortIndexs 索引进行排序，冒泡排序
	//for (int i = 0; i < AngleValues.size(); i++)
	//{
	//	for (int j = i + 1; j < AngleValues.size(); j++)
	//	{
	//		if (AngleValues[i] > AngleValues[j])	//j的角度值小
	//		{
	//			double TempAngle = AngleValues[i];
	//			AngleValues[i] = AngleValues[j];
	//			AngleValues[j] = TempAngle;

	//			int TempIndex = AngleSortIndexs[i];
	//			AngleSortIndexs[i] = AngleSortIndexs[j];
	//			AngleSortIndexs[j] = TempIndex;
	//		}
	//	}
	//}
	//cout <<endl;
	//首先通过判断 AnlgeOfPartition 度角度分区后 是否光顺
	for (int i = 0; i < CurSectionAnglePartition.size(); i++)
	{
		int PriorOfi, NextOfi;
		PriorOfi = (i + CurSectionAnglePartition.size() - 1) % CurSectionAnglePartition.size();
		NextOfi = (i + 1) % CurSectionAnglePartition.size();	

		int Count = CurSectionAnglePartition[i].PointIndexs.size();

		if (Count > 0)
		{
			if (CurSectionAnglePartition[PriorOfi].PointIndexs.size() > 0 &&
				CurSectionAnglePartition[NextOfi].PointIndexs.size() > 0)
			{
				//cout << PriorOfi << "," << i << "," << NextOfi << endl;
				double Angle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
					CurSectionAnglePartition[PriorOfi].CenterPointofCurPartition,
					CurSectionAnglePartition[i].CenterPointofCurPartition,
					CurSectionAnglePartition[NextOfi].CenterPointofCurPartition));	
				
				//还要考虑距离
				//if (!(SmoothSmallAngle < Angle && Angle < SmoothBigAngle))//该角度分区存在着异常点

				double AnlgeError = 60;

				if ((Angle < AnlgeError))//该角度分区存在着异常点
				{
					cout << Angle <<"异常点"<<endl;
					while (Angle < AnlgeError)
					{
						cout << Angle << "循环删除异常点" << endl;
						
						RemoveFarestPoint(SlicePointsIndexs, 
							CurSlicePointsOfXY, 
							CurSectionAnglePartition[i],
							CurrentCenterPointOfXY);

						if (CurSectionAnglePartition[i].PointIndexs.size() == 0)
							break;

						Angle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
							CurSectionAnglePartition[PriorOfi].CenterPointofCurPartition,
							CurSectionAnglePartition[i].CenterPointofCurPartition,
							CurSectionAnglePartition[NextOfi].CenterPointofCurPartition));
					}
				}
				//else cout << Angle <<"角度正常"<< endl;
			}
		}
	}	
}

//2019.08.31 移除角度分区中距离中心点最远的点
void CBranchRemovalByTangentPlaneAndStemAxisCurve::RemoveFarestPoint(
	vector<int> SlicePointsIndexs,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePoints,
	AnglePartitionStruct & CurAnglePartition,
	pcl::PointXYZRGB CurCenter)
{
	double FarestDis = PointDis(CurCenter, CurSlicePoints->points[CurAnglePartition.PointIndexs[0]]);
	int FarestIndex = 0;

	for (int i = 1; i < CurAnglePartition.PointIndexs.size(); i++)
	{
		double TempFarestDis = PointDis(CurCenter, CurSlicePoints->points[CurAnglePartition.PointIndexs[i]], true);
		if (TempFarestDis > FarestDis)
		{
			FarestDis = TempFarestDis;
			FarestIndex = i;
		}
	}
	int RemoveIndex = CurAnglePartition.PointIndexs[FarestIndex];
	InputCloud->points[SlicePointsIndexs[RemoveIndex]].rgba = ColorBase::RedColor;
	if (CurAnglePartition.PointIndexs.size() > 1)
	{	
		CurAnglePartition.CenterPointofCurPartition.x =
			(CurAnglePartition.CenterPointofCurPartition.x * CurAnglePartition.PointIndexs.size() - InputCloud->points[RemoveIndex].x) /
			(CurAnglePartition.PointIndexs.size() - 1);

		CurAnglePartition.CenterPointofCurPartition.y =
			(CurAnglePartition.CenterPointofCurPartition.y * CurAnglePartition.PointIndexs.size() - InputCloud->points[RemoveIndex].z) /
			(CurAnglePartition.PointIndexs.size() - 1);

		CurAnglePartition.CenterPointofCurPartition.z = 0;
	}
	CurAnglePartition.PointIndexs.erase(CurAnglePartition.PointIndexs.begin() + FarestIndex);
}

//2019.08.19 根据垂直分段点在平面投影以及于中心点的角度，以及相邻点的角度判断是否是树干点云
//根据中心点与点的角度升序排列，剔除相同角度较远点，然后相邻角度的点相互连接，再判断角度是否合理，
//若角度分区度数为2度时，相邻角度的点构成的角度比较平坦，则是树干点。
void CBranchRemovalByTangentPlaneAndStemAxisCurve::CheckBranchForNextSliceByProjectionAngle(bool toUp)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePointsOfXY(new pcl::PointCloud<pcl::PointXYZRGB>());
	vector<int> SlicePointsIndexs;
	pcl::PointXYZRGB ThirdPoint = StemSkeleton.GetSlicePoints(CurrentGrowthDirection, CurSlicePoints,
		CurrentCenterPoint, toUp, &SlicePointsIndexs);
	pcl::PointXYZRGB CurrentCenterPointOfXY;
	CurSlicePoints->points.push_back(CurrentCenterPoint);
	GeometryBase::RotateNormalToVertical(CurSlicePoints, CurSlicePointsOfXY, CurrentGrowthDirection);	
		
	CurrentCenterPointOfXY = CurSlicePointsOfXY->points[CurSlicePointsOfXY->points.size() - 1];
	CurSlicePoints->points.pop_back();
	CurSlicePointsOfXY->points.pop_back();

	//计算每个点与中心点的角度值
	vector<int> AngleValues;
	vector<int> AngleSortIndexs;
	for (int i = 0; i < CurSlicePointsOfXY->points.size(); i++)
	{
		double TempAngle = GeometryBase::AngleOfTwoPointsInXY(CurrentCenterPointOfXY.x,
			CurrentCenterPointOfXY.y,
			CurSlicePointsOfXY->points[i].x,
			CurSlicePointsOfXY->points[i].y);

		AngleValues.push_back(TempAngle);
		AngleSortIndexs.push_back(SlicePointsIndexs[i]);
	}

	//先角度分区，
	vector<AnglePartitionStruct> CurSectionAnglePartition;
	CAnglePartition AnglePartition;

	//以 CurrentCenterPointOfXY 为中心点、2度 为角度分区角度值 对当前分段进行角度分区
	AnglePartition.PartitionPoints(CurSlicePointsOfXY, CurrentCenterPointOfXY, 2, CurSectionAnglePartition);
	//首先计算每个角度分区的中心点
	for (int i = 0; i < CurSectionAnglePartition.size(); i++)
	{
		int Count = CurSectionAnglePartition[i].PointIndexs.size();
		if (Count > 0)
		{
			CurSectionAnglePartition[i].CenterPointofCurPartition.x = 0,
				CurSectionAnglePartition[i].CenterPointofCurPartition.y = 0,
				CurSectionAnglePartition[i].CenterPointofCurPartition.z = 0;

			for (int j = 0; j < Count; j++)
			{
				CurSectionAnglePartition[i].CenterPointofCurPartition.x =
					CurSectionAnglePartition[i].CenterPointofCurPartition.x +
					CurSlicePointsOfXY->points[CurSectionAnglePartition[i].PointIndexs[j]].x / Count;

				CurSectionAnglePartition[i].CenterPointofCurPartition.y =
					CurSectionAnglePartition[i].CenterPointofCurPartition.y +
					CurSlicePointsOfXY->points[CurSectionAnglePartition[i].PointIndexs[j]].y / Count;

				CurSectionAnglePartition[i].CenterPointofCurPartition.z =
					CurSectionAnglePartition[i].CenterPointofCurPartition.z +
					CurSlicePointsOfXY->points[CurSectionAnglePartition[i].PointIndexs[j]].z / Count;
			}
		}
	}

	//根据角度值对 AngleSortIndexs 索引进行排序，冒泡排序
	for (int i = 0; i < AngleValues.size(); i++)
	{
		for (int j = i + 1; j < AngleValues.size(); j++)
		{
			if (AngleValues[i] > AngleValues[j])	//j的角度值小
			{
				double TempAngle = AngleValues[i];
				AngleValues[i] = AngleValues[j];
				AngleValues[j] = TempAngle;

				int TempIndex = AngleSortIndexs[i];
				AngleSortIndexs[i] = AngleSortIndexs[j];
				AngleSortIndexs[j] = TempIndex;
			}
		}
	}
	
	//首先通过判断 2 度角度分区后 是否光顺	
	for (int i = 0; i < CurSectionAnglePartition.size(); i++)
	{
		int PriorOfi, NextOfi;		
		PriorOfi = (i - 1) % CurSectionAnglePartition.size();
		NextOfi = (i + 1) % CurSectionAnglePartition.size();

		int Count = CurSectionAnglePartition[i].PointIndexs.size();		

		if (Count > 0)
		{
			if (CurSectionAnglePartition[PriorOfi].PointIndexs.size() > 0 &&
				CurSectionAnglePartition[NextOfi].PointIndexs.size() > 0)
			{
				double Angle = GeometryBase::RadianToAngle(GeometryBase::AngleValueOfThreePoints(
					CurSectionAnglePartition[PriorOfi].CenterPointofCurPartition,
					CurSectionAnglePartition[i].CenterPointofCurPartition,
					CurSectionAnglePartition[NextOfi].CenterPointofCurPartition));
				
				//还要考虑距离
				if (! (SmoothSmallAngle < Angle < SmoothBigAngle))//该角度分区存在着异常点
				{

				}
			}
		}
	}
}

//2019.08.30 根据法向量
//2020.02.11 修改只计算法向量
void CBranchRemovalByTangentPlaneAndStemAxisCurve::NormalCalculation()
{	
	//CDBScanCluster DBScanCluster;
	//DBScanCluster.SetInputCloud(InputCloud);
	//DBScanCluster.RunCluster(3, 8);
	//DBScanCluster.SetClusterColors();	

	BranchRemovalForm.pushButtonRedo->setEnabled(true);

	///*
	if (InputCloud->points.size() != PointGeometry.Cloud_Normals->points.size())
	{
		if (BranchRemovalForm.radioButton_Radius->isChecked() == 1)
		{
			PointGeometry.SetInputCloud(InputCloud);
			PointGeometry.CalcNormalAndCurvatureRadius(
				BranchRemovalForm.doubleSpinBox_RadiusForNormalEstimation->text().toDouble(), 
				BranchRemovalForm.checkBox_Weighted->isChecked() == 1);
			
			//cout << "法向一致化" << endl;
			//PointGeometry.SetNormalConsistency(NormalAnchorIndex, NormalAnchorPoint,
			//	BranchRemovalForm.doubleSpinBox_RadiusForNormalEstimation->text().toDouble());

			//cout<<"计算获取主曲率方向"<<endl;
			//PointGeometry.CalcOptimalCurvature();
			//PointGeometry.GetDirectionOfPrincipalCurvatures(CurvatureDirections);
		}
		else if (BranchRemovalForm.radioButton_OVR->isChecked() == 1)
		{
			PointGeometry.SetInputCloud(InputCloud);
			PointGeometry.CalcOptimalK();
			PointGeometry.CalcOptimalNormalWithMore(BranchRemovalForm.checkBox_Weighted->isChecked() == 1);
			
			////使法向指向一致
			//PointGeometry.SetNormalConsistency(NormalAnchorIndex, NormalAnchorPoint);	

			//PointGeometry.CalcOptimalCurvature();			
			//PointGeometry.GetDirectionOfPrincipalCurvatures(CurvatureDirections);
		}
	}

	//double CurvatureValue = BranchRemovalForm.doubleSpinBoxCurvature->text().toDouble();
	
/*	
	CDBScanClusterWithCurvatrue DBScanCluster;
	DBScanCluster.SetInputCloud(InputCloud);
	DBScanCluster.RunClusterWithCurvature(2, 8, &PointGeometry, CurvatureValue);
	DBScanCluster.SetClusterColors();	
*/	
		
	/*
	Octree->setInputCloud(InputCloud);
	Octree->addPointsFromInputCloud();

	VectorBase<double> VectorBaseDouble;
	vector<double> CurvatureVec;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsCurvature(new pcl::PointCloud<pcl::PointXYZRGB>());

	double Max = 0, Min = 99;
	int MaxIndex = -1, MinIndex = -1;
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		double TempCurvature = PointGeometry.GetGuassCurvature(i);
		CurvatureVec.push_back(TempCurvature);
		if (TempCurvature > Max)
		{
			Max = TempCurvature;
			MaxIndex = i;
		}
		if (TempCurvature < Min)
		{
			Min = TempCurvature;
			MinIndex = i;
		}		

		//pcl::PointXYZRGB CurvaturePoint;
		//CurvaturePoint.x = TempCurvature, CurvaturePoint.y = 0, CurvaturePoint.z = 0;
		//PointsCurvature->points.push_back(CurvaturePoint);
	}
	cout<<  "Global Max:" << Max << endl;
	cout << "Global Min:" << Min << endl;
	double Mean, Std;
	Std = sqrt(VectorBaseDouble.CalcVariances(CurvatureVec, Mean));
	cout << "Global Mean:" << Mean << endl;
	cout << "Global Std:" << Std << endl;

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		if (CurvatureVec[i] > Mean + 3 * Std)
		//if (CurvatureVec[i] > CurvatureValue)
			InputCloud->points[i].rgba = ColorBase::RedColor;
	}

	//*/

	//CK_MeansCluster K_MeansCluster_0;
	////PointBase::SavePCDToFileName(CurSlicePointsCurvature, "I:\\Error\\CurSlicePointsCurvature.pcd");
	//K_MeansCluster_0.SetInput(CurvatureVec);
	//K_MeansCluster_0.RunCluster(2);
	//vector<int> Indexs;
	//MeansCluster First, Second;
	//K_MeansCluster_0.GetClusterInfo(0, First);
	//K_MeansCluster_0.GetClusterInfo(1, Second);

	//if (First.ClusterCenter.x > Second.ClusterCenter.x)	//第一个类不包含树干点云
	//{
	//	for (int i = 0; i < First.PointIndexS.size(); i++)	//让曲率大的类显示为红色
	//	{
	//		InputCloud->points[First.PointIndexS[i]].rgba = ColorBase::RedColor;
	//	}
	//}
	//else
	//{
	//	for (int i = 0; i < Second.PointIndexS.size(); i++)	//让曲率大的类显示为红色
	//	{
	//		InputCloud->points[Second.PointIndexS[i]].rgba = ColorBase::RedColor;
	//	}		
	//}

	//PointBase::SavePCDToFileName(PointsCurvature, "I:\\Error\\PointsCurvature.pcd");
	//VectorBaseDouble.SaveToFile(CurvatureVec, "I:\\Error\\CurvatureS.pcd");

	/*vector<int> NeighbourIndexsMax;
	vector<float> NeighbourDisMax;

	GeometryBase::GetNeighbourInKNN(Octree, PointGeometry.OptimalK[MaxIndex],
		MaxIndex, &NeighbourIndexsMax, &NeighbourDisMax);

	for (int i = 0; i < NeighbourIndexsMax.size(); i++)
	{
		InputCloud->points[NeighbourIndexsMax[i]].rgba = ColorBase::RedColor;
	}


	vector<int> NeighbourIndexsMin;
	vector<float> NeighbourDisMin;
	GeometryBase::GetNeighbourInKNN(Octree, PointGeometry.OptimalK[MinIndex],
		MinIndex, &NeighbourIndexsMin, &NeighbourDisMin);

	for (int i = 0; i < NeighbourIndexsMin.size(); i++)
	{
		InputCloud->points[NeighbourIndexsMin[i]].rgba = ColorBase::BlueColor;
	}*/

	/*//根据曲率的阈值选定点
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		if (PointGeometry.GetGuassCurvature(i) < CurvatureValue / 10)
			InputCloud->points[i].rgba = ColorBase::YellowColor;

		if (PointGeometry.GetGuassCurvature(i) > CurvatureValue)
			InputCloud->points[i].rgba = ColorBase::RedColor;
	}

	BranchRemovalForm.pushButtonRedo->setEnabled(true);
	//*/

	/* 标识高斯曲率的最大值与最小值
	vector<int> MaxIndexs = PointGeometry.GetMaxGuassCurvature(1000);
	for (int i = 0; i < MaxIndexs.size(); i++)
	{
		//cout<<"Max GuassCurvature:"<< PointGeometry.GetGuassCurvature(MaxIndexs [i])<<endl;
		cout << "Max GuassCurvature:" << PointGeometry.GetCurvature(MaxIndexs[i]) << endl;
		cout << "Point Normal:" << PointGeometry.GetNormal(MaxIndexs[i]) << endl;
		cout << "Optimal K:" << PointGeometry.GetOptimalK(MaxIndexs[i]) << endl;
		InputCloud->points[MaxIndexs[i]].rgba = ColorBase::RedColor;
	}
	
	//vector<int> MinIndexs = PointGeometry.GetMinGuassCurvature(500);
	//for (int i = 0; i < MinIndexs.size(); i++)
	//{
	//	cout << "Min GuassCurvature:" << PointGeometry.GetCurvature(MinIndexs[i]) << endl;
	//	InputCloud->points[MinIndexs[i]].rgba = ColorBase::YellowColor;
	//}
	//*/

	ShowPointNormal(BranchRemovalForm.checkBoxShowNormal->checkState());	
	//ShowPrincipalCurvatures(InputCloud, PointGeometry.Cloud_Normals, PointGeometry.Cloud_Curvatures,
	//	"Cloud_Curvatures",1, 100);
	
	//*/

	//for (int i = 0; i < InputCloud->points.size(); i++)
	//{
	//	if (abs(PointGeometry.GetGuassCurvature(i) ) < EPSM6)
	//		InputCloud->points[i].rgba = ColorBase::RedColor;
	//}

	/*
	cout<<"计算2倍距离的法向"<<endl;
	PointGeometry.CalcScaleNormalByOptimalK(2, DoubleNormals);
	cout << "计算0.5倍距离的法向" << endl;
	PointGeometry.CalcScaleNormalByOptimalK(0.5, HalfNormals);

	#pragma omp parallel for
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		double Angle1 = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
			PointGeometry.Cloud_Normals->points[i], DoubleNormals->points[i]));
		if (Angle1 > 90) Angle1 = 180 - Angle1;

		double Angle2 = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
			PointGeometry.Cloud_Normals->points[i], HalfNormals->points[i]));
		if (Angle2 > 90) Angle2 = 180 - Angle2;
		if (Angle1 > 45 && Angle2 > 45)
			InputCloud->points[i].rgba = ColorBase::RedColor;
	}
	//通过缩放邻域尺寸来判断树干与树枝的分割点
	//*/

	
	//for (int i = 0; i < InputCloud->points.size(); i++)
	//{
	//	int PointIndex = i;		

		//pcl::Normal PointTangentNormal = PointGeometry.GetNormal(PointIndex);

		//double TempAngle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
		//	PointTangentNormal.x, PointTangentNormal.y, PointTangentNormal.z,
		//	PointDirectionNormal.x, PointDirectionNormal.y, PointDirectionNormal.z));			
		//if (TempAngle > 90) TempAngle = 180 - 90;

		//if (TempAngle < 45)
		//	InputCloud->points[PointIndex].rgba = ColorBase::YellowColor;

		//if ( i==0)
		//{
		//cout << "特征值：" << EigenValue(0) <<" "<< EigenValue(1) << " " << EigenValue(2) << " " << endl;
		//cout << "最大距离：" << PointGeometry.GetOptimalK_MaxDis(PointIndex) << " " <<endl;
		//cout << "最小距离：" << PointGeometry.GetOptimalK_MinDis(PointIndex) << " " << endl;
		//cout << "近邻个数：" << PointGeometry.GetOptimalK(PointIndex) << " " << endl;
		//}

		////前两大特征值未均匀分配, 不具有树干表面的特征
		//if (abs(PointGeometry.Optimal_EigenVectors[PointIndex](2) 
		//	/ PointGeometry.Optimal_EigenVectors[PointIndex](1) > 3))	
		//	InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
		//
		////最大特征值占比过高 比值控制在0.75即可
		//if (abs(PointGeometry.Optimal_EigenVectors[PointIndex](2)) / 
		//	(abs(PointGeometry.Optimal_EigenVectors[PointIndex](0)) 
		//		+ (abs(PointGeometry.Optimal_EigenVectors[PointIndex](2)
		//		+ abs(PointGeometry.Optimal_EigenVectors[PointIndex](1))))) > 0.75)
		//	InputCloud->points[PointIndex].rgba = ColorBase::RedColor;

		//计算与其相邻点最近的法向，则角度大于 15 度则认为是噪声点, 法向方向并未归一化，因此无法以此为依据处理	
		
		/*
		//此方法可以判断 多个扫描仪获取点的边界 2019.09.04
		double Angle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(PointGeometry.GetNormal(PointIndex),
			PointGeometry.GetNormal(PointGeometry.OptimalK_MinDisIndex[PointIndex])));	
		if (Angle > 90) Angle = 180 - Angle;
		if (Angle < 15)
		{
			//cout <<"Dis:"<< PointGeometry.OptimalK_MinDis[PointIndex];
			//cout << " Angle:" << Angle << endl;
			InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
			
			pcl::PointXYZRGB GrowthPoint = PointBase::PointsCrossProduct(PointGeometry.GetNormal(PointIndex),
				PointGeometry.GetNormal(PointGeometry.OptimalK_MinDisIndex[PointIndex]));
			PointBase::PointNormalized(GrowthPoint);
			cout<<"生长方向："<< GrowthPoint <<endl;
		}	
		//*/   

		//double Dis_p_i = ((1 + 1.5 * Alpha)*AvgR) / Dis;
		//double Dis_p_i = 1 -  abs((Dis - AvgR) / AvgR);
		//vector<int> NeighbourIndexs;
		//vector<float> NeighbourDis;
		//
		////GeometryBase::GetNeighbourInRadius(Octree, NeighbourRadius, 
		////	PointIndex, &NeighbourIndexs, &NeighbourDis);

		//GeometryBase::GetNeighbourInKNN(Octree, PointGeometry.GetOptimalK(PointIndex),
		//	PointIndex, &NeighbourIndexs, &NeighbourDis);

		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr NeighourPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
		//
		//for (int i = 0; i < NeighbourIndexs.size(); i++)
		//{
		//	NeighourPoints->points.push_back(InputCloud->points[NeighbourIndexs[i]]);
		//}

		////此时要加上当前点
		//NeighourPoints->points.push_back(InputCloud->points[PointIndex]);		
		//
		//Eigen::Matrix3f EigenVector;		//特征向量
		//Eigen::Vector3f EigenValue(3);
		//GeometryBase::GetPointsTangentPlane(NeighourPoints, EigenVector, EigenValue);
		//
		//pcl::PointXYZRGB PointTangentNormal, PointDirectionNormal;
		//
		//PointTangentNormal.x = EigenVector(0), PointTangentNormal.y = EigenVector(1),
		//	PointTangentNormal.z = EigenVector(2);
		//
		//PointDirectionNormal.x = CurrentPoint.x - CurrentPointOfProjectToGrowthDirection.x,
		//	PointDirectionNormal.y = CurrentPoint.y - CurrentPointOfProjectToGrowthDirection.y,
		//	PointDirectionNormal.z = CurrentPoint.z - CurrentPointOfProjectToGrowthDirection.z;

		//double TempAngle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
		//	PointTangentNormal.x, PointTangentNormal.y, PointTangentNormal.z,
		//	PointDirectionNormal.x, PointDirectionNormal.y, PointDirectionNormal.z));			
		//if (TempAngle > 90) TempAngle = 180 - 90;

		//if (TempAngle < 45)
		//	InputCloud->points[PointIndex].rgba = ColorBase::YellowColor;

		//if ( i==0)
		//{
		//cout << "特征值：" << EigenValue(0) <<" "<< EigenValue(1) << " " << EigenValue(2) << " " << endl;
		//cout << "最大距离：" << PointGeometry.GetOptimalK_MaxDis(PointIndex) << " " <<endl;
		//cout << "最小距离：" << PointGeometry.GetOptimalK_MinDis(PointIndex) << " " << endl;
		//cout << "近邻个数：" << PointGeometry.GetOptimalK(PointIndex) << " " << endl;
		//}

		//////根据特征值计算意义不是太大，只是小部分有作用　2019.08.30
		////前两大特征值未均匀分配, 不具有树干表面的特征
		//if (abs(EigenValue(2) / EigenValue(1) > 3))	
		//	InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
		//	
		////最大特征值占比过高 比值控制在0.75即可
		//if(abs(EigenValue(2)) / (abs(EigenValue(0)) + (abs(EigenValue(2) + abs(EigenValue(1))))) > 0.75)
		//	InputCloud->points[PointIndex].rgba = ColorBase::RedColor;

		//double AngleOfSmallNormal = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
		//	PointGeometry.Cloud_Normals->points[PointIndex].normal_x, 
		//	PointGeometry.Cloud_Normals->points[PointIndex].normal_y,
		//	PointGeometry.Cloud_Normals->points[PointIndex].normal_z,
		//	CurrentGrowthDirection.x, CurrentGrowthDirection.y, CurrentGrowthDirection.z));		
		//if (AngleOfSmallNormal > 90) AngleOfSmallNormal = 180 - AngleOfSmallNormal;

		//double AngleOfBiggerNormal = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
		//	BiggerPointGeometry.Cloud_Normals->points[PointIndex].normal_x,
		//	BiggerPointGeometry.Cloud_Normals->points[PointIndex].normal_y,
		//	BiggerPointGeometry.Cloud_Normals->points[PointIndex].normal_z,
		//	CurrentGrowthDirection.x, CurrentGrowthDirection.y, CurrentGrowthDirection.z));
		//if (AngleOfBiggerNormal > 90) AngleOfBiggerNormal = 180 - AngleOfBiggerNormal;
		////cout << "Angle between Normal and Growth Direction:" << PointIndex << "::" << TempAngleOfNormal << endl;
		////法向与生长方向应该是垂直关系，若法向与生长方向小于60度，则颜色设置为黄色
		//if (AngleOfSmallNormal < 75 && AngleOfBiggerNormal < 75)
		//		InputCloud->points[PointIndex].rgba = ColorBase::RedColor;

		//double AnlgeofNormals = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
		//	PointGeometry.Cloud_Normals->points[PointIndex].normal_x,
		//	PointGeometry.Cloud_Normals->points[PointIndex].normal_y,
		//	PointGeometry.Cloud_Normals->points[PointIndex].normal_z,
		//	BiggerPointGeometry.Cloud_Normals->points[PointIndex].normal_x,
		//	BiggerPointGeometry.Cloud_Normals->points[PointIndex].normal_y,
		//	BiggerPointGeometry.Cloud_Normals->points[PointIndex].normal_z));		

		//if (AnlgeofNormals > 90) AnlgeofNormals = 180 - AnlgeofNormals;
		//cout << "Angle between two Normals:" << PointIndex << "::" << NormalAnlge << endl;

		//if (AnlgeofNormals > 60)
		//	InputCloud->points[PointIndex].rgba = ColorBase::GreenColor;

		//if (PointGeometry.GetOptimalK_MaxDis(PointIndex) > 5) 
		//	InputCloud->points[PointIndex].rgba = ColorBase::BlueColor;

		//cout << PointGeometry.GetOptimalK(PointIndex) << endl;
		//cout << PointGeometry.GetOptimalK_MaxDis(PointIndex) << endl;
		//if (PointGeometry.GetOptimalK(PointIndex) < 10)
		//	InputCloud->points[PointIndex].rgba = ColorBase::BlueColor;
		//else if (PointGeometry.GetOptimalK(PointIndex) > 10 && PointGeometry.GetOptimalK(PointIndex) <= 30)
		//	InputCloud->points[PointIndex].rgba = ColorBase::DarkBlueColor;
		//else if (PointGeometry.GetOptimalK(PointIndex) > 30 && PointGeometry.GetOptimalK(PointIndex) <= 50)
		//	InputCloud->points[PointIndex].rgba = ColorBase::GreenColor;
		//else if (PointGeometry.GetOptimalK(PointIndex) > 50 && PointGeometry.GetOptimalK(PointIndex) <= 70)
		//	InputCloud->points[PointIndex].rgba = ColorBase::DarkGreenColor;
		//else if (PointGeometry.GetOptimalK(PointIndex) > 70 && PointGeometry.GetOptimalK(PointIndex) <= 100)
		//	InputCloud->points[PointIndex].rgba = ColorBase::DoveColor;

		//if(abs(EigenValue(0)) / (abs(EigenValue(0)) + (abs(EigenValue(2) + abs(EigenValue(1))))) > 0.25)
		//	InputCloud->points[PointIndex].rgba = ColorBase::YellowColor;

		//p_i *= Dis_p_i * SinValue * abs(EigenValue(1) / EigenValue(2));

		//if (SinValue < sin(M_PI / 3) || (abs(EigenValue(1) / EigenValue(2) < 0.5)))
		//if (SinValue < sin(M_PI / 3) )

		//if (p_i < 1 - p_i)
		//{
		//	cout << "PointIndex:" << PointIndex 
		//		<< " p_i:" << p_i << " Dis_p_i:"<< Dis_p_i <<" SinValue:" << SinValue
		//		<< " EigenValue: " << EigenValue(1) / EigenValue(2) << endl;
		//	InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
		//}	
//
////上一个分区的Center	
//	//JudgeBranchInSlices(SlicePointsIndexs, CurrentGrowthDirection, CurrentCenterPoint);	
//
//	//VectorBase<string> Features;
//	//vector<string> FeatureStrs;
//	//for (int i = 0; i < SlicePointsIndexs.size(); i++)
//	//{
//	//	string TempStr =
//	//		StringBase::IntToStr(PointGeometry.OptimalK[SlicePointsIndexs[i]]) + "," +
//	//		StringBase::FloatToStr(PointGeometry.OptimalK_MaxDis[SlicePointsIndexs[i]]) + "," +
//	//		StringBase::FloatToStr(PointGeometry.OptimalK_MinDis[SlicePointsIndexs[i]]) + "," +
//	//		StringBase::FloatToStr(PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](0)
//	//			/ (PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](0)
//	//				+ PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](1)
//	//				+ PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](2))) + "," +
//	//		StringBase::FloatToStr(PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](1)
//	//			/ (PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](0)
//	//				+ PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](1)
//	//				+ PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](2))) + "," +
//	//		StringBase::FloatToStr(PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](2) /
//	//		(PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](0)
//	//			+ PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](1)
//	//			+ PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](2))) + "," +
//	//		StringBase::FloatToStr(PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](1)
//	//			/ PointGeometry.Optimal_EigenVectors[SlicePointsIndexs[i]](2));
//	//	FeatureStrs.push_back(TempStr);
//	//}
//	//Features.SaveToFile(FeatureStrs, "I:\\FeatureStrs.txt");
//
	//}
	emitUpdateUI();
}

bool CBranchRemovalByTangentPlaneAndStemAxisCurve::CheckBranchForNextSlice(bool toUp)
{
	//获得当前的树干生长方向
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCentroidPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	if (SliceCalced == 0)
	{
		int TempStartIndex;
		if (toUp) 
			TempStartIndex = StemSkeleton.CentroidPoints->points.size() - 1;
		else
			TempStartIndex = CalcSectionNumbers - 1;

		for (int i = 0; i < CalcSectionNumbers; i++)
		{
			int TempIndex = TempStartIndex - i;
			TempCentroidPoints->points.push_back(StemSkeleton.CentroidPoints->points[TempIndex]);
		}

		pcl::PointXYZRGB TempCurrentGrowthDirection = GeometryBase::GetMaxDirectionVector(TempCentroidPoints);				
		double Angle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector
			(TempCurrentGrowthDirection, CurrentGrowthDirection));

		if (Angle < 20) CurrentGrowthDirection = TempCurrentGrowthDirection;
	}	

	SliceCalced = (SliceCalced + 1) % CalcSectionNumbers;
	cout <<" SliceCalced: "<< SliceCalced<<" CalcSectionNumbers: "<< CalcSectionNumbers<< endl;
	vector<int> PriorSlicePointsIndexs, SlicePointsIndexs;
	pcl::PointXYZRGB PriorCurrentCenterPoint, CurrentCenterPoint;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlicePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	StemSkeleton.GetPriorSlicePoints(CurrentGrowthDirection, PriorSlicePoints, 
		PriorCurrentCenterPoint, toUp, &PriorSlicePointsIndexs);
	
	if (PriorSlicePoints->points.size() < 0)
		return true;

	if (toUp)
		PriorCurrentCenterPoint = StemSkeleton.CentroidPoints->points[StemSkeleton.CentroidPoints->points.size() - 2];
	else
		PriorCurrentCenterPoint = StemSkeleton.CentroidPoints->points[1];

	// the distance of prior slice
	double MaxR = EPSM6, MinR = EPSP6, AvgR = 0;
	//double MaxGaussCurature = 0;

	vector<int> VectorOptimalK;

	for (int i = 0; i < PriorSlicePointsIndexs.size(); i++)
	{
		int PointIndex = PriorSlicePointsIndexs[i];
		pcl::PointXYZRGB CurrentPoint = InputCloud->points[PointIndex];
	
		double Dis = PointDis(InputCloud->points[PointIndex], PriorCurrentCenterPoint);

		MaxR = Dis > MaxR ? Dis: MaxR;
		MinR = Dis < MinR ? Dis: MinR;
		AvgR += Dis / PriorSlicePointsIndexs.size();

		//double TempCurvature = PointGeometry.GetGuassCurvature(i);
		//MaxGaussCurature = TempCurvature > MaxGaussCurature ? TempCurvature : MaxGaussCurature;
		//
		//VectorOptimalK.push_back(PointGeometry.OptimalK[PointIndex]);
			   
	}	

	//for (int kk = 0; kk < VectorOptimalK.size(); kk++)
	//{
	//	cout<< VectorOptimalK [kk]<<endl;
	//}
	//cin >> VectorOptimalK[0];

	double Alpha = (MaxR - MinR) / AvgR;
	Alpha = Alpha > 0.40 ? 0.40 : Alpha;
	double Mutilple = 1.2;

	if (LastAvg == 0)
		LastAvg = AvgR;

	if (toUp)
	{ 
		if (AvgR > LastAvg * Mutilple && LastAvg > 0)
		{
			AvgR = LastAvg;
			
			//先删除上一级的不规则的点
			for (int i = PriorSlicePointsIndexs.size() - 1; i >= 0; i--)
			{
				int PointIndex = PriorSlicePointsIndexs[i];
				double Dis = PointDis(InputCloud->points[PointIndex], PriorCurrentCenterPoint);

				if (Dis > (1 + Mutilple * Alpha) * AvgR)
				{
					InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
					//PriorSlicePointsIndexs.erase(PriorSlicePointsIndexs.begin() + i);
					//cout<<"Removed PointIndex:"<<PointIndex <<endl;
					continue;
				}
			}
		}
	}
			
	//cout<< "Alpha:" << Alpha <<" MaxR:"<< MaxR << " MinR:" << MinR << " AvgR:" << AvgR <<endl;
	//cout<< "Height:" << InputCloud->points[PriorSlicePointsIndexs[0]].z - ZMin <<endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePoints(new pcl::PointCloud<pcl::PointXYZRGB>());	
	pcl::PointXYZRGB ThirdPoint = StemSkeleton.GetSlicePoints(CurrentGrowthDirection, CurSlicePoints,
			CurrentCenterPoint, toUp, &SlicePointsIndexs);

	if (SlicePointsIndexs.size() < 5) 
		return true;
	
	SliceCount++;
	//double TempDis = PointDis(CurrentCenterPoint, PriorCurrentCenterPoint);
	//if (TempDis < SliceHeight / 3.0 || TempDis > 2 * SliceHeight) return true;

	//cout << "SlicePointsIndexs.size：" << SlicePointsIndexs.size() << endl;
	//cout << "TempDis：" << TempDis << endl;

	//* Show 
	//if (SliceCalced % 3 == 1)
		PointBase::SetPointColor(PriorSlicePoints, ColorBase::GreyColor);
	//else if (SliceCalced % 3 == 2)
	//	PointBase::SetPointColor(PriorSlicePoints, ColorBase::GreenColor);
	//else if (SliceCalced % 3 == 0)
	//	PointBase::SetPointColor(PriorSlicePoints, ColorBase::BlueColor);

	//ShowPoints(TempSlicePoints, "CurSlicePoints"+ StringBase::ClockValue(), 5);
	Viewer->removePointCloud("PriorSlicePoints");
	ShowPoints(PriorSlicePoints, "PriorSlicePoints", 5);
	//*/

	//* Show 
	//if (SliceCalced % 3 == 0)
	//	PointBase::SetPointColor(CurSlicePoints, ColorBase::GreyColor);
	//else if (SliceCalced % 3 == 1)
	//	PointBase::SetPointColor(CurSlicePoints, ColorBase::GreenColor);
	//else if (SliceCalced % 3 == 2)
	PointBase::SetPointColor(CurSlicePoints, ColorBase::BlueColor);

	//ShowPoints(TempSlicePoints, "CurSlicePoints"+ StringBase::ClockValue(), 5);
	Viewer->removePointCloud("CurSlicePoints");
	ShowPoints(CurSlicePoints, "CurSlicePoints", 5);
	//*/

	if (toUp)
		CurrentCenterPoint = StemSkeleton.CentroidPoints->points[StemSkeleton.CentroidPoints->points.size() - 1];
	else
		CurrentCenterPoint = StemSkeleton.CentroidPoints->points[0];

	//CheckSliceByPriorSlice(PriorSlicePoints, PriorCurrentCenterPoint, CurSlicePoints, 
	//	CurrentCenterPoint, SlicePointsIndexs, CurrentGrowthDirection);
///*


/*// 计算曲率最大点和最小点

	double Max = 0, Min = 99;
	int MaxIndex = -1, MinIndex = -1;
	for (int i = 0; i < SlicePointsIndexs.size(); i++)
	{
		double TempCurvature = PointGeometry.GetGuassCurvature(SlicePointsIndexs[i]);
		if (TempCurvature > Max)
		{
			Max = TempCurvature;
			MaxIndex = SlicePointsIndexs[i];
		}
		if (TempCurvature < Min)
		{
			Min = TempCurvature;
			MinIndex = SlicePointsIndexs[i];
		}
	}
	cout << "Max:" << Max << endl;
	cout << "Min:" << Min << endl;

	vector<int> NeighbourIndexsMax;
	vector<float> NeighbourDisMax;

	GeometryBase::GetNeighbourInKNN(Octree, PointGeometry.OptimalK[MaxIndex],
		MaxIndex, &NeighbourIndexsMax, &NeighbourDisMax);

	for (int i = 0; i < NeighbourIndexsMax.size(); i++)
	{
		InputCloud->points[NeighbourIndexsMax[i]].rgba = ColorBase::RedColor;
	}

	vector<int> NeighbourIndexsMin;
	vector<float> NeighbourDisMin;
	GeometryBase::GetNeighbourInKNN(Octree, PointGeometry.OptimalK[MinIndex],
		MinIndex, &NeighbourIndexsMin, &NeighbourDisMin);

	for (int i = 0; i < NeighbourIndexsMin.size(); i++)
	{
		InputCloud->points[NeighbourIndexsMin[i]].rgba = ColorBase::BlueColor;
	}
	//cin >> MinIndex;
	//// 计算曲率最大点和最小点  *///

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudForCurvatureCluster(new pcl::PointCloud<pcl::PointXYZRGB>());	
	
	for (int i = SlicePointsIndexs.size() - 1; i >=0; i--)
	{		
		int PointIndex = SlicePointsIndexs[i];
		pcl::PointXYZRGB CurrentPoint = InputCloud->points[PointIndex];
		
		//InputCloudForCurvatureCluster->points.push_back(CurrentPoint);


		if (CurrentPoint.rgba == ColorBase::RedColor)
		{
			SlicePointsIndexs.erase(SlicePointsIndexs.begin() + i);
			continue;
		}

		/***************************根据法向的最大变动方向 与 法向的叉乘 与 生长方向的夹角  差别比较大
		pcl::PointXYZRGB TempPrincipalCurvaturePoint;

		TempPrincipalCurvaturePoint.x = PointGeometry.GetDirectionOfPrincipalCurvature(i).normal_x;
		TempPrincipalCurvaturePoint.y = PointGeometry.GetDirectionOfPrincipalCurvature(i).normal_y;
		TempPrincipalCurvaturePoint.z = PointGeometry.GetDirectionOfPrincipalCurvature(i).normal_z;

		pcl::PointXYZRGB Direction = PointBase::PointsCrossProduct(
			CurrentGrowthDirection, TempPrincipalCurvaturePoint);

		pcl::PointXYZRGB TempPoint;
		TempPoint.x = PointGeometry.GetNormal(i).normal_x;
		TempPoint.y = PointGeometry.GetNormal(i).normal_y;
		TempPoint.z = PointGeometry.GetNormal(i).normal_z;

		double Angle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(Direction, TempPoint));

		if (Angle > 90) Angle = 180 - Angle;

		if (Angle > 30)
		{
			InputCloud->points[PointIndex].rgba = ColorBase::YellowColor;
			//cout<<"最大变动方向 与 法向的叉乘 与 法向的夹角:"<< Angle <<endl;
			cout << "最大变动方向 与 生长方向的叉乘 与 法向的夹角:" << Angle << endl;
			cout << "最大变动方向 与 生长方向的叉乘:" << Direction << endl;
			cout << "生长方向:" << CurrentGrowthDirection << endl;
		}

		////***************************/

		float PlaneD = -(CurrentPoint.x * CurrentGrowthDirection.x + CurrentPoint.y * CurrentGrowthDirection.y
			+ CurrentPoint.z * CurrentGrowthDirection.z);
		//获取当前点在中心线上的投影	

		pcl::PointXYZRGB CurrentPointOfProjectToGrowthDirection = 
			GeometryBase::LineCrossPlane(CurrentGrowthDirection.x,
			CurrentGrowthDirection.y, CurrentGrowthDirection.z, CurrentPoint, 
			CurrentGrowthDirection.x, CurrentGrowthDirection.y, CurrentGrowthDirection.z, PlaneD);

		double Dis = PointDis(InputCloud->points[PointIndex], CurrentCenterPoint);

		//首先根据距离移除一部分点
		if (Dis > (1 + Alpha) * AvgR)
		{
			InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
			SlicePointsIndexs.erase(SlicePointsIndexs.begin() + i);
			//cout<<"Removed PointIndex:"<<PointIndex <<endl;
			continue;
		}

		double AnlgeofMaxNormals = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
			PointGeometry.Cloud_NormalsOfLargestEigenValue->points[PointIndex].normal_x,
			PointGeometry.Cloud_NormalsOfLargestEigenValue->points[PointIndex].normal_y,
			PointGeometry.Cloud_NormalsOfLargestEigenValue->points[PointIndex].normal_z,
			CurrentGrowthDirection.x,
			CurrentGrowthDirection.y, CurrentGrowthDirection.z));		
		if (AnlgeofMaxNormals > 90) AnlgeofMaxNormals = 180 - AnlgeofMaxNormals;
		//cout << "树干生长方向和最大特征向量的夹角" << AnlgeofNormals << endl;

		double AnlgeofNormal = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
			PointGeometry.Cloud_Normals->points[PointIndex].normal_x,
			PointGeometry.Cloud_Normals->points[PointIndex].normal_y,
			PointGeometry.Cloud_Normals->points[PointIndex].normal_z,
			CurrentGrowthDirection.x,
			CurrentGrowthDirection.y, CurrentGrowthDirection.z));
		//cout << "生长方向和法向(最小特征向量)的夹角" << endl;
		if (AnlgeofNormal > 90) AnlgeofNormal = 180 - AnlgeofNormal;
		
		//树干点云的最大特征值对应的特征向量应于树干生长方向相近，
		//且树干点云的最小特征值对应的特征向量应于树干生长方向垂直，根据二者关系判断树干点云与树枝点云
		if (AnlgeofMaxNormals > 30 && (90 - AnlgeofNormal > 20))
		{		
			InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
			//cout<<"异常点的特征值比率："<< PointGeometry.Optimal_EigenVectors[i](1) 
			//	/ PointGeometry.Optimal_EigenVectors[i](2) <<endl;
			//cout << "异常点的最大特征值：" << PointGeometry.Optimal_EigenVectors[i](2)<< endl;
			//cout << "异常点的特征值比率：" << (PointGeometry.Optimal_EigenVectors[i](0)/(
			//	PointGeometry.Optimal_EigenVectors[i](0) +
			//	PointGeometry.Optimal_EigenVectors[i](1) + 
			//	PointGeometry.Optimal_EigenVectors[i](2))) << endl;
			//cout << endl;
		}
		else
		{
			//cout << "正常点的特征值比率：" << PointGeometry.Optimal_EigenVectors[i](1)
			//	/ PointGeometry.Optimal_EigenVectors[i](2) << endl;
			//cout << "正常点的最大特征值：" << PointGeometry.Optimal_EigenVectors[i](2) << endl;
			//cout << "异常点的特征值比率：" << (PointGeometry.Optimal_EigenVectors[i](0) / (
			//	PointGeometry.Optimal_EigenVectors[i](0) +
			//	PointGeometry.Optimal_EigenVectors[i](1) +
			//	PointGeometry.Optimal_EigenVectors[i](2))) << endl;
			//cout << endl;
		}

		//与生长方向的角度小于60度
		if((90 - AnlgeofNormal > 30))
		{
			InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
			cout<<"法向与生长方向夹角:"<< AnlgeofNormal <<endl;
		}

		//意义不大 2019.08.31
		//double DisRate = 0;
		//if (PointGeometry.DisOfPointToTangentPlane[PointIndex] != 0)
		//{ 
		//	DisRate = PointGeometry.DisOfPointToCentroid[PointIndex]
		//		/ PointGeometry.DisOfPointToTangentPlane[PointIndex];
		//}
		//double Value = 100;
		//if (DisRate > Value || DisRate < 1.0 / Value)
		//{
		//	cout<<"DisRate:"<< DisRate <<endl;
		//	InputCloud->points[PointIndex].rgba = ColorBase::YellowColor;
		//}
	}
	//*/

	emitUpdateUI();
	// refresh slice geometrical center
	CurSlicePoints->points.clear();

	/* //与其最近点的法向夹角大于 45 度
	for (int i = SlicePointsIndexs.size() - 1; i >= 0; i--)
	{
		int PointIndex = SlicePointsIndexs[i];
		pcl::PointXYZRGB CurrentPoint = InputCloud->points[PointIndex];

		if (CurrentPoint.rgba != ColorBase::RedColor)
		{
			CurSlicePoints->points.push_back(CurrentPoint);

			//与其最近点的法向夹角大于 45 度
			pcl::PointXYZRGB MinDisGrowthPoint = PointBase::PointsCrossProduct(PointGeometry.GetNormal(PointIndex),
				PointGeometry.GetNormal(PointGeometry.OptimalK_MinDisIndex[PointIndex]));
			PointBase::PointNormalized(MinDisGrowthPoint);
			//cout << "最近点法向计算的生长方向：" << MinDisGrowthPoint << endl;

			double MinDisAnlge = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(MinDisGrowthPoint, 
				CurrentGrowthDirection));
			if (MinDisAnlge > 90) MinDisAnlge = 180 - MinDisAnlge;
			if (MinDisAnlge > 45)
			{
				//cout << "MinDisGrowthPoint:" << MinDisGrowthPoint << endl;
				//cout << "CurrentGrowthDirection:" << CurrentGrowthDirection << endl;
				//cout<<"MinDisAnlge:"<< MinDisAnlge <<endl;
				InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
				continue;
			}

			//pcl::PointXYZRGB MaxDisGrowthPoint = PointBase::PointsCrossProduct(PointGeometry.GetNormal(PointIndex),
			//	PointGeometry.GetNormal(PointGeometry.OptimalK_MaxDisIndex[PointIndex]));
			//PointBase::PointNormalized(MaxDisGrowthPoint);
			////cout << "最远点法向计算的生长方向：" << MaxDisGrowthPoint << endl;

			//double MaxDisAnlge = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(MaxDisGrowthPoint, CurrentGrowthDirection));
			//if (MaxDisAnlge > 90) MaxDisAnlge = 180 - MaxDisAnlge;
			//if (MinDisAnlge <= 30 && MaxDisAnlge > 30)
			//{				
			//	cout << "MaxDisAnlge:" << MaxDisAnlge << endl;
			//	if (InputCloud->points[PointGeometry.OptimalK_MaxDisIndex[PointIndex]].rgba != ColorBase::RedColor)
			//		InputCloud->points[PointGeometry.OptimalK_MaxDisIndex[PointIndex]].rgba = ColorBase::YellowColor;
			//}
			//cout << "中心点计算的生长方向：" << CurrentGrowthDirection << endl;
		}

		//if (PointGeometry.GetGuassCurvature(PointIndex) > 2 * MaxGaussCurature)
		//{
		//	cout << "MaxGaussCurature：" << MaxGaussCurature << endl;
		//	cout<<"PointGeometry.GetGuassCurvature(PointIndex)："<< PointGeometry.GetGuassCurvature(PointIndex) <<endl;
		//	InputCloud->points[PointIndex].rgba = ColorBase::YellowColor;
		//}
	}
	//*/

	//距离修复后采用角度的形式移除树枝点 2019.08.30
	//CheckBranchForSliceByProjectionAngle(SlicePointsIndexs, CurSlicePoints, CurrentCenterPoint);
	//再次更新
	CurSlicePoints->points.clear();
	for (int i = SlicePointsIndexs.size() - 1; i >= 0; i--)
	{
		int PointIndex = SlicePointsIndexs[i];
		pcl::PointXYZRGB CurrentPoint = InputCloud->points[PointIndex];

		if (CurrentPoint.rgba != ColorBase::RedColor)
		{
			CurSlicePoints->points.push_back(CurrentPoint);
			//cout <<"最大距离："<< PointGeometry.OptimalK_MaxDis[PointIndex] << endl;;
		}
	}
	   
	if (CurSlicePoints->points.size() == 0) return true;	

	/* 使用 DBScan 对当前分段进行分类
	CDBScanCluster DBScanCluster;
	DBScanCluster.SetInputCloud(CurSlicePoints);
	DBScanCluster.RunCluster(1, 5);
	DBScanCluster.SetClusterColors();
	if (DBScanCluster.GetClusterNumbers() >= 2)
	{

		for (int k = 0; k < DBScanCluster.GetClusterNumbers(); k++)
		{
			vector<int> KIndexs;
			DBScanCluster.GetCluster(k, KIndexs);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

			for (int kk = 0; kk < KIndexs.size(); kk++)
			{
				TempPoints->points.push_back(CurSlicePoints->points[KIndexs[kk]]);
			}

			double a, b, c;
			pcl::PointXYZRGB TempNormalPoint;
			GeometryBase::GetPointsTangentPlane(TempPoints, TempNormalPoint);

			double Angle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(TempNormalPoint, CurrentGrowthDirection));

			if (Angle > 90) Angle = 180 - Angle;
			cout << "簇的法向量与生长方向的夹角："<< Angle << endl;
			if (Angle > 30)
			{
				//cin >> Angle;
				for (int kk = 0; kk < KIndexs.size(); kk++)
				{
					int PointIndex = SlicePointsIndexs[KIndexs[kk]];
					InputCloud->points[PointIndex].rgba = ColorBase::RedColor;
				}
			}
		}	

		//cout << "当前分段DBSCan分类个数大于2" << endl;
		//DBScanCluster.ShowCluster(Viewer, 2);

		//PointBase::SavePCDToFileName(CurSlicePoints, "I:\\Error\\Slice_" + StringBase::IntToStr(SliceCount) + ".pcd");
	}
	//*/

	//再次更新
	CurSlicePoints->points.clear();
	for (int i = SlicePointsIndexs.size() - 1; i >= 0; i--)
	{
		int PointIndex = SlicePointsIndexs[i];
		pcl::PointXYZRGB CurrentPoint = InputCloud->points[PointIndex];

		if (CurrentPoint.rgba != ColorBase::RedColor)
		{
			CurSlicePoints->points.push_back(CurrentPoint);
			//cout <<"最大距离："<< PointGeometry.OptimalK_MaxDis[PointIndex] << endl;;
		}
	}

	pcl::PointXYZRGB TempCentroidPoint =
		GeometryBase::GetCentroidOfPointsInSpaces(CurSlicePoints, CurrentGrowthDirection);

	//if (SliceCalced == 0)
	//	PointBase::SetPointColor(TempSlicePoints, ColorBase::RedColor);
	//else if (SliceCalced == 1)
	//	PointBase::SetPointColor(TempSlicePoints, ColorBase::BlueColor);
	//else if (SliceCalced == 2)
	//	PointBase::SetPointColor(TempSlicePoints, ColorBase::GreenColor);

	//ShowPoints(TempSlicePoints, "TempSlicePoints"+ StringBase::ClockValue(), 5);

	CurSlicePoints->points.push_back(TempCentroidPoint);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempOutPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	GeometryBase::ProjectPointsToPlane(CurSlicePoints, TempOutPoints, CurrentGrowthDirection, ThirdPoint);

	TempCentroidPoint.x = TempOutPoints->points[TempOutPoints->points.size() - 1].x;
	TempCentroidPoint.y = TempOutPoints->points[TempOutPoints->points.size() - 1].y;
	TempCentroidPoint.z = TempOutPoints->points[TempOutPoints->points.size() - 1].z;
	TempCentroidPoint.rgba = ColorBase::BlueColor;

	//*
	//2019.06.21  如果TempCentroidPoint与前一个点形成的角度大于特定阈值，则修正方向依次 
	//不可行 存在着异常点到正常点过度偏大的情况
	pcl::PointXYZRGB NewDirection;
	NewDirection.x = -(CurrentCenterPoint.x - TempCentroidPoint.x);
	NewDirection.y = -(CurrentCenterPoint.y - TempCentroidPoint.y);
	NewDirection.z = -(CurrentCenterPoint.z - TempCentroidPoint.z);
		
	double NewAngle = GeometryBase::RadianToAngle(
		GeometryBase::AngleOfTwoVector(NewDirection, CurrentGrowthDirection));
	cout<<"NewAngle:"<< NewAngle <<endl;
	while (NewAngle > 30) //要修正
	{
		pcl::PointXYZRGB TempPoint = GeometryBase::LineCrossPlane(
			CurrentGrowthDirection.x, CurrentGrowthDirection.y, CurrentGrowthDirection.z, CurrentCenterPoint, 
			CurrentGrowthDirection.x, CurrentGrowthDirection.y, CurrentGrowthDirection.z, 
			-(CurrentGrowthDirection.x * ThirdPoint.x + CurrentGrowthDirection.y * ThirdPoint.y +
				CurrentGrowthDirection.z * ThirdPoint.z));
		cout << "原节点坐标:" << TempCentroidPoint << endl;
		TempCentroidPoint.x = (TempPoint.x + TempCentroidPoint.x) / 2;
		TempCentroidPoint.y = (TempPoint.y + TempCentroidPoint.y) / 2;
		TempCentroidPoint.z = (TempPoint.z + TempCentroidPoint.z) / 2;
		cout << "新节点坐标:" << TempCentroidPoint << endl;

		NewDirection.x = -(CurrentCenterPoint.x - TempCentroidPoint.x);
		NewDirection.y = -(CurrentCenterPoint.y - TempCentroidPoint.y);
		NewDirection.z = -(CurrentCenterPoint.z - TempCentroidPoint.z);

		NewAngle = GeometryBase::RadianToAngle(
			GeometryBase::AngleOfTwoVector(NewDirection, CurrentGrowthDirection));
		cout << "NewAngle:" << NewAngle << endl;
	}

	//if (NewAngle > M_PI / 12)
	//if (NewAngle > GeometryBase::AngleToRadian(10))
	//	TempCentroidPoint.rgba = ColorBase::RedColor;
	if (NewAngle > 90)
		return true;
	//*/

	////cout << "NewCenter" << TempCentroidPoint << endl;
	StemSkeleton.CentroidPoints->points.push_back(TempCentroidPoint);

	//PointBase::SetPointColor(StemSkeleton.CentroidPoints, ColorBase::BlueColor);
	//cout << "Num:"<< StemSkeleton.CentroidPoints->points.size()<<endl;
	//ShowPoints(StemSkeleton.CentroidPoints, CentroidPointsStr, 5);

	return false;
}

//2019.09.20 待实现
bool CBranchRemovalByTangentPlaneAndStemAxisCurve::CheckBranchForNextSliceByCurvature(bool toUp)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCentroidPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	if (SliceCalced == 0)
	{
		int TempStartIndex;
		if (toUp)
			TempStartIndex = StemSkeleton.CentroidPoints->points.size() - 1;
		else
			TempStartIndex = CalcSectionNumbers - 1;

		for (int i = 0; i < CalcSectionNumbers; i++)
		{
			int TempIndex = TempStartIndex - i;
			TempCentroidPoints->points.push_back(StemSkeleton.CentroidPoints->points[TempIndex]);
		}

		pcl::PointXYZRGB TempCurrentGrowthDirection = GeometryBase::GetMaxDirectionVector(TempCentroidPoints);
		double Angle = GeometryBase::RadianToAngle(
			GeometryBase::AngleOfTwoVector(TempCurrentGrowthDirection, CurrentGrowthDirection));

		if (Angle < 20) CurrentGrowthDirection = TempCurrentGrowthDirection;
	}

	vector<int> CurSlicePointsIndexs;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	pcl::PointXYZRGB ThirdPoint = StemSkeleton.GetSlicePoints(CurrentGrowthDirection, CurSlicePoints,
		CurrentCenterPoint, toUp, &CurSlicePointsIndexs);
	//获取的点太少 就直接退出
	if (CurSlicePoints->points.size() <= 3)
		return true;
	
	// StemSkeleton.GetSlicePoints 获取的 CurrentCenterPoint 是新获取点云集的几何中心点
	if (toUp)
		CurrentCenterPoint = StemSkeleton.CentroidPoints->points[StemSkeleton.CentroidPoints->points.size() - 1];
	else
		CurrentCenterPoint = StemSkeleton.CentroidPoints->points[0];
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePointsCurvature(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	VectorBase<double> VectorBaseDouble;
	vector<double> CurvatureS;

	for (int i = 0; i < CurSlicePoints->points.size(); i++)
	{
		pcl::PointXYZRGB Point;
		Point.x = PointGeometry.GetGuassCurvature(CurSlicePointsIndexs[i]);
		CurvatureS.push_back(Point.x);
		Point.y = 0, Point.z = 0;

		CurSlicePointsCurvature->points.push_back(Point);
	}
	//VectorBaseDouble.SaveToFile(CurvatureS, "I:\\Error\\CurSlicePointsCurvature.pcd");

	if ((LastCurvatureMean == 0) && (LastCurvatureStd == 0))
	{
		LastCurvatureStd = sqrt(VectorBaseDouble.CalcVariances(CurvatureS, LastCurvatureMean));
	}

	for (int i = 0; i < CurSlicePoints->points.size(); i++)
	{
		if (CurvatureS[i] > LastCurvatureMean + 10 * LastCurvatureStd)
		{
			InputCloud->points[CurSlicePointsIndexs[i]].rgba = ColorBase::RedColor;
		}
	}

	/* //	//直接根据曲率值大小 聚类的效果并不好
	CK_MeansCluster K_MeansCluster_0;
	//PointBase::SavePCDToFileName(CurSlicePointsCurvature, "I:\\Error\\CurSlicePointsCurvature.pcd");
	K_MeansCluster_0.SetInput(CurSlicePointsCurvature);
	K_MeansCluster_0.RunCluster(2);
	vector<int> Indexs;
	MeansCluster First, Second;
	K_MeansCluster_0.GetClusterInfo(0, First);
	K_MeansCluster_0.GetClusterInfo(1, Second);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr LocalityStemPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	vector<int> LocalityStemPointIndexs;

	cout<<	"First.ClusterCenter.x:" << First.ClusterCenter.x <<endl;
	cout << "Second.ClusterCenter.x:" << Second.ClusterCenter.x << endl;
	double Mutiple = First.ClusterCenter.x / Second.ClusterCenter.x;


	//这个倍数很难调整
	if (Mutiple < 1) Mutiple = 1.0 / Mutiple;
	cout<<"Mutiple:"<< Mutiple <<endl;
	if (Mutiple > 10)
	{
		//PointBase::SavePCDToFileName(CurSlicePoints, "I:\\Error\\Mutiple.pcd");
		if (First.ClusterCenter.x > Second.ClusterCenter.x)	//第一个类不包含树干点云
		{
			for (int i = 0; i < First.PointIndexS.size(); i++)	//让曲率大的类显示为红色
			{
				InputCloud->points[SlicePointsIndexs[First.PointIndexS[i]]].rgba = ColorBase::RedColor;
			}

			for (int i = 0; i < Second.PointIndexS.size(); i++)	//让曲率小的类作为树干点云并计算中心点
			{
				LocalityStemPoints->points.push_back(InputCloud->points[SlicePointsIndexs[Second.PointIndexS[i]]]);
				LocalityStemPointIndexs.push_back(SlicePointsIndexs[Second.PointIndexS[i]]);
			}
		}
		else
		{
			for (int i = 0; i < Second.PointIndexS.size(); i++)	//让曲率大的类显示为红色
			{
				InputCloud->points[SlicePointsIndexs[Second.PointIndexS[i]]].rgba = ColorBase::RedColor;
			}

			for (int i = 0; i < First.PointIndexS.size(); i++)	//让曲率小的类作为树干点云并计算中心点
			{
				LocalityStemPoints->points.push_back(InputCloud->points[SlicePointsIndexs[First.PointIndexS[i]]]);
				LocalityStemPointIndexs.push_back(SlicePointsIndexs[First.PointIndexS[i]]);
			}
		}
	}
	else
	{
		LocalityStemPoints->points.insert(LocalityStemPoints->points.begin(), CurSlicePoints->points.begin(),
			CurSlicePoints->points.end());
	}	


	if (LocalityStemPoints->points.size() <= 3)
		return true;
	//*/

	//再次更新 点集 和 曲率均值和方差
	CurvatureS.clear();
	CurSlicePoints->points.clear();
	for (int i = CurSlicePointsIndexs.size() - 1; i >= 0; i--)
	{
		int PointIndex = CurSlicePointsIndexs[i];
		pcl::PointXYZRGB CurrentPoint = InputCloud->points[PointIndex];

		if (CurrentPoint.rgba != ColorBase::RedColor)
		{
			CurSlicePointsIndexs.erase(CurSlicePointsIndexs.begin() + i);
			CurSlicePoints->points.push_back(CurrentPoint);	

			CurvatureS.push_back(PointGeometry.GetGuassCurvature(PointIndex));
		}
	}
	LastCurvatureStd = sqrt(VectorBaseDouble.CalcVariances(CurvatureS, LastCurvatureMean));
	cout <<"曲率均值:" << LastCurvatureMean <<endl;
	cout << "曲率方差:" << LastCurvatureStd << endl;
	if (CurSlicePoints->points.size() == 0) return true;

	pcl::PointXYZRGB TempCentroidPoint =
		GeometryBase::GetCentroidOfPointsInSpaces(CurSlicePoints, CurrentGrowthDirection);

	//ShowPoints(CurSlicePoints, "LocalityStemPoints", 2);

	//cout << "CurSlicePoints Size:" << CurSlicePoints->points.size() << endl;
	//cout << "CurrentCenterPoint:" << CurrentCenterPoint << endl;
	//cout<<"TempCentroidPoint:"<< TempCentroidPoint <<endl;

	CurSlicePoints->points.push_back(TempCentroidPoint);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempOutPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	GeometryBase::ProjectPointsToPlane(CurSlicePoints, TempOutPoints, CurrentGrowthDirection, ThirdPoint);

	TempCentroidPoint.x = TempOutPoints->points[TempOutPoints->points.size() - 1].x;
	TempCentroidPoint.y = TempOutPoints->points[TempOutPoints->points.size() - 1].y;
	TempCentroidPoint.z = TempOutPoints->points[TempOutPoints->points.size() - 1].z;
	TempCentroidPoint.rgba = ColorBase::BlueColor;

	//*
	//2019.06.21  如果TempCentroidPoint与前一个点形成的角度大于特定阈值，则修正方向依次 
	//不可行 存在着异常点到正常点过度偏大的情况
	pcl::PointXYZRGB NewDirection;
	NewDirection.x = -(CurrentCenterPoint.x - TempCentroidPoint.x);
	NewDirection.y = -(CurrentCenterPoint.y - TempCentroidPoint.y);
	NewDirection.z = -(CurrentCenterPoint.z - TempCentroidPoint.z);
	//此处有CurrentCenterPoint 与 TempCentroidPoint 重合的情况

	double NewAngle = GeometryBase::RadianToAngle(
		GeometryBase::AngleOfTwoVector(NewDirection, CurrentGrowthDirection));
	//cout << "NewAngle:" << NewAngle <<"此轮迭代结束！"<< endl;
	//while (NewAngle > 30) //要修正
	//{
	//	pcl::PointXYZRGB TempPoint = GeometryBase::LineCrossPlane(
	//		CurrentGrowthDirection.x, CurrentGrowthDirection.y, CurrentGrowthDirection.z, CurrentCenterPoint,
	//		CurrentGrowthDirection.x, CurrentGrowthDirection.y, CurrentGrowthDirection.z,
	//		-(CurrentGrowthDirection.x * ThirdPoint.x + CurrentGrowthDirection.y * ThirdPoint.y +
	//			CurrentGrowthDirection.z * ThirdPoint.z));
	//	cout << "原节点坐标:" << TempCentroidPoint << endl;
	//	TempCentroidPoint.x = (TempPoint.x + TempCentroidPoint.x) / 2;
	//	TempCentroidPoint.y = (TempPoint.y + TempCentroidPoint.y) / 2;
	//	TempCentroidPoint.z = (TempPoint.z + TempCentroidPoint.z) / 2;
	//	cout << "新节点坐标:" << TempCentroidPoint << endl;

	//	NewDirection.x = -(CurrentCenterPoint.x - TempCentroidPoint.x);
	//	NewDirection.y = -(CurrentCenterPoint.y - TempCentroidPoint.y);
	//	NewDirection.z = -(CurrentCenterPoint.z - TempCentroidPoint.z);

	//	NewAngle = GeometryBase::RadianToAngle(
	//		GeometryBase::AngleOfTwoVector(NewDirection, CurrentGrowthDirection));
	//	cout << "NewAngle:" << NewAngle << endl;
	//}

	if (NewAngle > 90) return true;

	StemSkeleton.CentroidPoints->points.push_back(TempCentroidPoint);	

	return false;

}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::CheckSliceByPriorSlice(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlice,
	pcl::PointXYZRGB PriorCenter, pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlice,
	pcl::PointXYZRGB CurCenter, vector<int> CurSlicePointsIndexs, pcl::PointXYZRGB GrowthDirection)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorVerSlice(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurVerSlice(new pcl::PointCloud<pcl::PointXYZRGB>());

	PriorSlice->points.push_back(PriorCenter);
	GeometryBase::RotateNormalToVertical(PriorSlice, PriorVerSlice, GrowthDirection);

	CurSlice->points.push_back(CurCenter);
	GeometryBase::RotateNormalToVertical(CurSlice, CurVerSlice, GrowthDirection);
		
	//PointBase::SetPointColor(PriorVerSlice, ColorBase::RedColor);
	//PriorVerSlice->points[PriorVerSlice->points.size() - 1].rgba = ColorBase::BlueColor;
	//ShowPoints(PriorVerSlice, "PriorVerSlice", 5);

	//PointBase::SetPointColor(CurVerSlice, ColorBase::BlueColor);
	//CurVerSlice->points[CurVerSlice->points.size() - 1].rgba = ColorBase::RedColor;
	//ShowPoints(CurVerSlice, "CurVerSlice", 5);

	pcl::PointXYZRGB PriorCenterNew = PriorVerSlice->points[PriorVerSlice->points.size() - 1];
	PriorVerSlice->points.pop_back();

	pcl::PointXYZRGB CurCenterNew = CurVerSlice->points[CurVerSlice->points.size() - 1];
	CurVerSlice->points.pop_back();

	vector<AnglePartitionStruct> PriorSectionAngle;
	CAnglePartition AnglePartition;
	int theat = 2;

	AnglePartition.PartitionPoints(PriorVerSlice, theat, PriorSectionAngle);

	vector<AnglePartitionStruct> CurSectionAngle;
	AnglePartition.PartitionPoints(CurVerSlice, theat, CurSectionAngle);

	vector<double> DisThreshold;
	DisThreshold.resize(PriorSectionAngle.size());

	double AvgDisThreshold = 0;
	int RightNum = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPriorVerSliceAngle(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCurVerSliceAngle(new pcl::PointCloud<pcl::PointXYZRGB>());

	#pragma  omp parallel for ordered schedule(dynamic)
	for (int i = 0; i < CurSectionAngle.size(); i++)
	{
		double MaxDis = EPSM6, MinDis = EPSP6;
		int AngleIndex = i;

		for (int j = 0; j < PriorSectionAngle[AngleIndex].PointIndexs.size(); j++)
		{
			int TempPointIndex = PriorSectionAngle[AngleIndex].PointIndexs[j];
			double Dis = PointDis(PriorCenterNew, PriorVerSlice->points[TempPointIndex], true);
			MaxDis = MaxDis < Dis ? Dis : MaxDis;
			MinDis = MinDis > Dis ? Dis : MinDis;

			if (i == 0)
			{
				TempPriorVerSliceAngle->points.push_back(PriorVerSlice->points[TempPointIndex]);
			}
		}

		double TempDisThreshold = abs(MaxDis + (MaxDis - MinDis) / 2);
		DisThreshold[i] = TempDisThreshold;		
		
	#pragma omp ordered
		{			
			if (TempDisThreshold < 100)
			{
				AvgDisThreshold = AvgDisThreshold + TempDisThreshold;
				RightNum++;
			}
		}
	}

	AvgDisThreshold = AvgDisThreshold / RightNum;
	cout << " AvgDisThreshold: " << AvgDisThreshold << endl;
	
	#pragma omp parallel for
	for (int i = 0; i < CurSectionAngle.size(); i++)
	{		
		if (DisThreshold[i] > 100 && (DisThreshold[(i - 1) % CurSectionAngle.size()] < 100))
		{
			DisThreshold[i] = DisThreshold[(i - 1) % CurSectionAngle.size()];
		}
		else if (DisThreshold[i] > 100 && (DisThreshold[(i + 1) % CurSectionAngle.size()] < 100))
		{
			DisThreshold[i] = DisThreshold[(i + 1) % CurSectionAngle.size()];
		}
		else if (DisThreshold[i] > 100)
			DisThreshold[i] = AvgDisThreshold;
	}
	
	#pragma omp parallel for
	for (int i = 0; i < CurSectionAngle.size(); i++)
	{	
		//cout << "I: " << i << " DisThreshold: " << DisThreshold[i] << endl;
		int AngleIndex = i;
		for (int j = 0; j < CurSectionAngle[AngleIndex].PointIndexs.size(); j++)
		{
			int TempPointIndex = CurSectionAngle[AngleIndex].PointIndexs[j];
			int GlobalPointIndex = CurSlicePointsIndexs[TempPointIndex];
			double Dis = PointDis(CurCenterNew, CurVerSlice->points[TempPointIndex], true);

			if (i == 0)
			{
				TempCurVerSliceAngle->points.push_back(PriorVerSlice->points[TempPointIndex]);
			}

			if (Dis - DisThreshold[i] > DisThreshold[i] / 10)
				InputCloud->points[GlobalPointIndex].rgba = ColorBase::RedColor;
		}
	}
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::Redo()
{
	BranchRemovalForm.pushButtonBranchCheck->setEnabled(true);
	BranchRemovalForm.pushButtonRedo->setEnabled(false);
	BranchRemovalForm.pushButtonRemoval->setEnabled(false);
	CTreeBase::Redo();
	RefreshData();
	StemSkeleton.CentroidPoints->points.clear();
	if (Viewer != NULL)
	{
		Viewer->removePointCloud(CentroidPointsStr);
		Viewer->removeAllShapes();
	}
	emitUpdateUI();
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::ShowPointNormal(int CheckValue)
{
	if (InputCloud == NULL) return;

	QAction * Action = (QAction *)sender();
	if (Action == NULL) return;

	if (PointGeometry.Cloud_Normals->points.size() != InputCloud->points.size())
	{
		NormalCalculation();
	}

	if (CheckValue == 2)
		ShowNormals(InputCloud, PointGeometry.Cloud_Normals, "_Cloud_Normals", 1, 1);
		//ShowNormals(InputCloud, CurvatureDirections, "_Cloud_Normals", 1, 1);
		
		//ShowNormals(InputCloud, HalfNormals, "_Cloud_Normals", 1, 1);
	else
		Viewer->removePointCloud("_Cloud_Normals");

	emitUpdateUI();
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::SetRadioRadius(bool CheckValue)
{
	if (BranchRemovalForm.radioButton_Radius->isChecked() == 1)
	{
		BranchRemovalForm.doubleSpinBox_RadiusForNormalEstimation->setEnabled(true);
	}
	else
	{
		BranchRemovalForm.doubleSpinBox_RadiusForNormalEstimation->setEnabled(false);
	}
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::CheckBranches()
{
	RefreshParameters();

	Octree->setInputCloud(InputCloud);
	Octree->addPointsFromInputCloud();

	BranchRemovalForm.pushButtonBranchCheck->setEnabled(false);
	BranchRemovalForm.pushButtonRedo->setEnabled(true);
	BranchRemovalForm.pushButtonRemoval->setEnabled(true);	

	//此处获取起始位置处的
	SliceCount = 0;
	CalcGrowthDirectionAtStartHeight(BranchRemovalForm.spinBoxStartHeight->text().toDouble());
	SliceCalced = 0;
	bool IsContinue = true;	
	//PointGeometry.SetInputs(InputCloud);
	//BiggerPointGeometry.SetInputs(InputCloud, 2);
	//ShowDirection();

	if (PointGeometry.Cloud_Normals->points.size() != InputCloud->points.size())
	{
		NormalCalculation();
	}
	//计算法向量 Radius
	/*PointGeometry.CalcNormalAndCurvatureRadius(
		BranchRemovalForm.spinBoxRadius->text().toDouble(),
		PointGeometry.Cloud_Normals, true);	
	
	PointGeometry.CalcNormalAndCurvatureRadius(
		BranchRemovalForm.spinBoxRadius->text().toDouble() * 2,
		BiggerCloud_Normals, true);*/
	
	//CalcNormalAndCurvatureKNN
	//PointGeometry.CalcOptimalK();
	//PointGeometry.CalcOptimalNormalAndCurvature(false);
	//ShowNormals(InputCloud, PointGeometry.Cloud_Normals, "Cloud_Normals", 1, 2);

	//BiggerPointGeometry.CalcOptimalK();
	//BiggerPointGeometry.CalcOptimalNormalAndCurvature();
	//ShowNormals(InputCloud, BiggerPointGeometry.Cloud_Normals, "_Cloud_Normals", 1, 1);
	
	//int KnnK = 8;
	//PointGeometry.CalcNormalAndCurvatureKNN(
	//		//BranchRemovalForm.spinBoxRadius->text().toDouble(),
	//	KnnK,
	//		PointGeometry.Cloud_Normals, true);

	//BiggerPointGeometry.CalcNormalAndCurvatureKNN(
	//	//BranchRemovalForm.spinBoxRadius->text().toDouble() * 2,
	//	KnnK * 2,
	//	BiggerPointGeometry.Cloud_Normals, true);

	///*
	//IsContinue = !CheckBranchForNextSlice(true);	

	LastCurvatureMean = 0;
	LastCurvatureMean = 0;

	IsContinue = !CheckBranchForNextSliceByCurvature(true);
	int i = 0;
	int j = 0;
	while (IsContinue)
	{
		//cout<<"please input the number:"<<endl;
		//cin >> j;
		//IsContinue = !CheckBranchForNextSlice(true);
		IsContinue = !CheckBranchForNextSliceByCurvature(true);
		i++;
		
		//if (i > 200)
		//	break;
	}	

	AmendSkeletonCentriodPoints();

	StemSkeleton.ConstructStemSplineCurve(true);
	
	PointBase::SetPointColor(StemSkeleton.StemSkeletonSpline.CurvePoints, ColorBase::RedColor);
	ShowPoints(StemSkeleton.StemSkeletonSpline.CurvePoints, "CurvePoints", PointSize);
	//*/

	//IsContinue = !CheckBranchForNextSliceUseConvexPolygon(true);
	//
	////IsContinue = !CheckBranchForNextSliceByBarkThickness(true);
	//
	//int i = 0;
	//int j = 0;
	//while (IsContinue)
	//{
	//	//cin >> j;
	//	IsContinue = !CheckBranchForNextSliceUseConvexPolygon(true);
	//	i++;
	//	
	//	if (i > 5)
	//		break;
	//}

	//SliceCalced = 0;
	//CheckBranchForNextSlice(false);
	//DeleteBranches();
	//CheckBranchByGradientDescent();
}

void CBranchRemovalByTangentPlaneAndStemAxisCurve::DeleteBranches()
{
	BranchRemovalForm.pushButtonRedo->setEnabled(false);
	BranchRemovalForm.pushButtonBranchCheck->setEnabled(false);
	//BranchRemovalForm.pushButtonRemoval->setEnabled(false);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		if (!(InputCloud->points[i].r == 255 && InputCloud->points[i].g == 0 && InputCloud->points[i].b == 0))
		{
			TempPoints->points.push_back(InputCloud->points[i]);
		}
	}

	InputCloud->points.clear();
	InputCloud->points.insert(InputCloud->points.end(),
		TempPoints->points.begin(), TempPoints->points.end());
	TempPoints->points.clear();

	emitUpdateStatusBar("Branches points have been removed!", 5000);
	emitUpdateUI();
	RefreshData();

	BranchRemovalForm.pushButtonRedo->setEnabled(true);
	BranchRemovalForm.pushButtonBranchCheck->setEnabled(true);
}