#include "SimulateMeasure.h"

void CSimulateMeasure::RefreshData()
{
	CTreeBase::RefreshData();

	CylinderCenterPoint.x = 0, CylinderCenterPoint.y = 0, CylinderCenterPoint.z = 0;
	CircleCenter.x = 0, CircleCenter.y = 0, CircleCenter.z = 0;
	D3Center.x = 0, D3Center.y = 0, D3Center.z = 0;
	
	//if (InputCloud->points.size() > 0)
	//	CalculateDriection(InputCloud, GrownDirection);

	SimulatingForm.lineEdit_Height->setText("");
	SimulatingForm.lineEdit_NormalX->setText("");
	SimulatingForm.lineEdit_NormalY->setText("");
	SimulatingForm.lineEdit_NormalZ->setText("");

	SimulatingForm.lineEditD_Cylinder->setText("");
	SimulatingForm.lineEditD_Circle->setText("");
	SimulatingForm.lineEditD_ConvexHullLine->setText("");
	SimulatingForm.lineEditD_BSpline->setText("");
	SimulatingForm.lineEditD_BezierConvex->setText("");
	SimulatingForm.lineEditD_Caliper->setText("");

	SimulatingForm.lineEditD_Caliper->setText("");
	SimulatingForm.lineEditD_Caliper_Max->setText("");
	SimulatingForm.lineEditD_Caliper_Min->setText("");
	SimulatingForm.lineEditD_Caliper_Ovality->setText("");

	Viewer->removePointCloud(SimulatingPointsStr);
	Viewer->removePointCloud(PlanarPointsStr);
	Viewer->removePointCloud(CircleFittingPointsStr);
	Viewer->removePointCloud(CylinderFittingPointsStr);
	Viewer->removeAllShapes();
	Viewer->removePointCloud(SplineFittingStr);
	Viewer->removePointCloud(BezierFittingStr);
}

CSimulateMeasure::CSimulateMeasure(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	SimulatingForm.setupUi(widget);

	IsBat = false;

	D3PlanarPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	D2PlanarPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

	SimulatingPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	ConvexHullPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	D2ConvexHullPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	CylinderFittingPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	CircleFittingPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	BSplineFittingPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	BSplineProfilePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	BezierCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

	connect(SimulatingForm.pushButtonCalculate, SIGNAL(clicked()), this, SLOT(SimulateMeasure()));
	connect(SimulatingForm.pushButtonBatCalculate, SIGNAL(clicked()), this, SLOT(BatSimulateMeasure()));
	connect(SimulatingForm.pushButtonBatStemDividing, SIGNAL(clicked()), this, SLOT(BatStemDividing()));	

	connect(SimulatingForm.pushButtonStemDividing, SIGNAL(clicked()), this, SLOT(StemDividingByStemAxisCurve()));	

	connect(SimulatingForm.checkBoxShowMeasurePoints,
		SIGNAL(stateChanged(int)), this, SLOT(ShowModelPoints(int)));
	connect(SimulatingForm.checkBoxShowCylinder,
		SIGNAL(stateChanged(int)), this, SLOT(ShowModelPoints(int)));
	connect(SimulatingForm.checkBoxShowCircle,
		SIGNAL(stateChanged(int)), this, SLOT(ShowModelPoints(int)));
	connect(SimulatingForm.checkBoxShowConvexHullLine,
		SIGNAL(stateChanged(int)), this, SLOT(ShowModelPoints(int)));
	connect(SimulatingForm.checkBoxShowBSpline,
		SIGNAL(stateChanged(int)), this, SLOT(ShowModelPoints(int)));
	connect(SimulatingForm.checkBoxShowBezier,
		SIGNAL(stateChanged(int)), this, SLOT(ShowModelPoints(int)));
	connect(SimulatingForm.checkBoxShowConvexhullPoints,
		SIGNAL(stateChanged(int)), this, SLOT(ShowModelPoints(int)));
	connect(SimulatingForm.checkBoxShowCaliper,
		SIGNAL(stateChanged(int)), this, SLOT(ShowModelPoints(int)));
	widget->show();
}

CSimulateMeasure::~CSimulateMeasure()
{
	if (Viewer != NULL)
	{
		Viewer->removePointCloud(SimulatingPointsStr);
		Viewer->removePointCloud(PlanarPointsStr);
		Viewer->removePointCloud(CircleFittingPointsStr);
		Viewer->removePointCloud(CylinderFittingPointsStr);
		Viewer->removeAllShapes();
		Viewer->removePointCloud(SplineFittingStr);
		Viewer->removePointCloud(BezierFittingStr);
	}

	emitUpdateUI();
}

//单个模拟测量
void CSimulateMeasure::SimulateMeasure()
{		
	Viewer->removeAllShapes();

	//最后一个点是当前点云的法向
	//GrownDirection = InputCloud->points[InputCloud->points.size() -1];
	//InputCloud->points.pop_back();
	GrownDirection.x = 0, GrownDirection.y = 0, GrownDirection.z = 1;
	//if ( abs((pow(GrownDirection.x, 2) + pow(GrownDirection.y, 2) + pow(GrownDirection.z, 2)) - 1) >= EPSM3)
		
	//CalculateDriection(InputCloud, GrownDirection);
	
	cout<<"Grown Direction in Original:"<< GrownDirection <<endl;	

	AnchorPoint = PointBase::GetGravityPoint(InputCloud);
	
	//
	if (abs(GrownDirection.x) < EPSM9 && abs(GrownDirection.y) < EPSM9 && abs(GrownDirection.z) < EPSM9)
		return;
	
	Height = ZMax - ZMin;
	SimulatingForm.lineEdit_Height->setText(QString::number(Height));
	SimulatingForm.lineEdit_NormalX->setText(QString::number(GrownDirection.x));
	SimulatingForm.lineEdit_NormalY->setText(QString::number(GrownDirection.y));
	SimulatingForm.lineEdit_NormalZ->setText(QString::number(GrownDirection.z));

	//分别获取上下两个平面上的两个点 2020.08.07
	MeasureLength = SimulatingForm.spinBox_MeasureLength->text().toDouble();

	pcl::PointXYZRGB LowerPoint = GeometryBase::GetPointAlongLine(GrownDirection, 
		AnchorPoint, MeasureLength / 2, false);
	pcl::PointXYZRGB UpperPoint = GeometryBase::GetPointAlongLine(GrownDirection, 
		AnchorPoint, MeasureLength / 2, true);

	//获取计算的  三维点云
	GeometryBase::GetPointsBetweenTwoPlanes(InputCloud, GrownDirection.x, GrownDirection.y, GrownDirection.z,
		-(GrownDirection.x * LowerPoint.x + GrownDirection.y * LowerPoint.y + GrownDirection.z * LowerPoint.z),
		GrownDirection.x, GrownDirection.y, GrownDirection.z,
		-(GrownDirection.x * UpperPoint.x + GrownDirection.y * UpperPoint.y + GrownDirection.z * UpperPoint.z),
		SimulatingPoints);

	//获取计算的  二维点云
	GeometryBase::ProjectPointsToPlane(SimulatingPoints, D3PlanarPoints,
		GrownDirection, LowerPoint);
	D3Center = PointBase::GetGravityPoint(SimulatingPoints);
	
	//Circle  CircleFitting 是在二维空间上计算的结果
	GeometryBase::RotateNormalToVertical(D3PlanarPoints, D2PlanarPoints, GrownDirection);
	D2Center = PointBase::GetGravityPoint(D2PlanarPoints);
	//PointBase::SetPointColor(D2PlanarPoints, ColorBase::GreenColor);
	//ShowPoints(D2PlanarPoints, PlanarPointsStr);

	DCircle = GeometryBase::CircleFittingByLeastSquaresFitting(D2PlanarPoints, CircleCenter);
	SimulatingForm.lineEditD_Circle->setText(QString::number(DCircle * 2));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCirclePoints (new pcl::PointCloud<pcl::PointXYZRGB>());
	GeometryBase::GetCirclePoints(TempCirclePoints, CircleCenter, DCircle);
	GeometryBase::RotateToOriginal(TempCirclePoints, CircleFittingPoints, GrownDirection);
	cout << "CircleCenter in 2D:" << CircleCenter << endl;

	////Cricle Fitting Moving
	PointsMove(CircleFittingPoints, D3Center.x - CircleCenter.x,
		D3Center.y - CircleCenter.y, D3Center.z - CircleCenter.z);
	   
	//Cylinder	
	CCylinderFitting CylinderFitting;
	CylinderFitting.SetInput(SimulatingPoints);
	CylinderFitting.CylinderFitting(GrownDirection, CylinderCenterPoint, DCylinder, HCylinder);
	DCylinder = 2 * DCylinder;
	SimulatingForm.lineEditD_Cylinder->setText(QString::number(DCylinder));
	cout << "CylinderCenterPoint:" << CylinderCenterPoint << endl;
	cout << "CylinderH:" << HCylinder << endl;
	cout << "CylinderD:" << DCylinder << endl;

	//计算凸包
	ConvexHullPoints->points.clear();
	CContourAndConvexHull<pcl::PointXYZRGB> ContourAndConvexHull;
	ContourAndConvexHull.SetInputs(D2PlanarPoints);
	vector<int> ConvexHullIndexs;
	ContourAndConvexHull.GetPointsConvexHull(ConvexHullIndexs);

	//凸包折线计算
	double PerimeterHullLine = 0;
	for (int i = 0; i < ConvexHullIndexs.size(); i++)
	{
		ConvexHullPoints->points.push_back(D3PlanarPoints->points[ConvexHullIndexs[i]]);
		PerimeterHullLine = PerimeterHullLine + 
			PointDis(D3PlanarPoints->points[ConvexHullIndexs[i]],
				D3PlanarPoints->points[ConvexHullIndexs[(i + 1) % ConvexHullIndexs.size()]]);
	}

	DConvexLine = PerimeterHullLine / M_PI;
	SimulatingForm.lineEditD_ConvexHullLine->setText(QString::number(DConvexLine));

	GeometryBase::RotateNormalToVertical(ConvexHullPoints, D2ConvexHullPoints, GrownDirection);
		
	//B-Spline 计算
	CSplineInterpolation SplineInterpolation;
	SplineInterpolation.SetInputs(ConvexHullPoints, 3, true);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> KnotValues;
	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnotValues);

	//闭合曲线
	CSpline Spline;	
	Spline.SetSplineInputs(ControlPoints, 3, KnotValues, true);
	Spline.FirstPointOfCloseCurve = ConvexHullPoints->points[0];
	Spline.CreateSpline();

	double PerimeterSpline = Spline.GetSplineLengthBySimpson();
	DBSpline = PerimeterSpline / M_PI;
	SimulatingForm.lineEditD_BSpline->setText(QString::number(DBSpline));
	PointBase::PointCopy(Spline.CurvePoints, BSplineFittingPoints);

	//GeometryBase::RotateToOriginal(Spline.CurvePoints, BSplineFittingPoints, GrownDirection);
	//PointsMove(BSplineFittingPoints, D3Center.x - CircleCenter.x,
	//	D3Center.y - CircleCenter.y, D3Center.z - CircleCenter.z);

	//Bezier 计算
	CBezierSubsectionG2 BezierSubsectionG2;
	BezierSubsectionG2.SetInputs(ConvexHullPoints);
	//The tension parameter 
	BezierSubsectionG2.ResolveControlPoints(BezierMiu);
	BezierSubsectionG2.Viewer = Viewer;	

	double PerimeterBezier = BezierSubsectionG2.GetBezierLength();
	DBezierConvex = PerimeterBezier / M_PI;
	SimulatingForm.lineEditD_BezierConvex->setText(QString::number(DBezierConvex));
	BezierCurvePoints->points.clear();
	BezierSubsectionG2.DrawBezierCurve(BezierCurvePoints);

	///*
	string BezierFileName =
		"I:\\000Comparsion\\Temp\\Bezier_C_" + OpenedFileName + ".pcd";
	string ControlFileName =
		"I:\\000Comparsion\\Temp\\Spline_C_" + OpenedFileName + ".pcd";
	string KnotFileName =
		"I:\\000Comparsion\\Temp\\Knot_C_" + OpenedFileName + ".pcd";

	PointBase::SavePCDToFileName(BezierCurvePoints, BezierFileName);
	Spline.SaveControlPointsAndKnotValue(ControlPoints, KnotValues,
		ControlFileName, KnotFileName, Spline.StartU, Spline.EndU);
	PointBase::SavePCDToFileName(SimulatingPoints, "I:\\000Comparsion\\Temp\\3D_" + OpenedFileName + ".pcd");
	PointBase::SavePCDToFileName(D2PlanarPoints, "I:\\000Comparsion\\Temp\\2D_" + OpenedFileName + ".pcd");
	PointBase::SavePCDToFileName(D2ConvexHullPoints, "I:\\000Comparsion\\Temp\\2D_ConvexHull_" + OpenedFileName + ".pcd");
	PointBase::SavePCDToFileName(TempCirclePoints, "I:\\000Comparsion\\Temp\\2DCircle_" + OpenedFileName + ".pcd");
	//*/

	//GeometryBase::RotateToOriginal(TempCirclePoints, BezierCurvePoints, GrownDirection);
	//PointsMove(BSplineFittingPoints, D3Center.x - CircleCenter.x,
	//	D3Center.y - CircleCenter.y, D3Center.z - CircleCenter.z);

	//Caliper measurement
	CaliperMeasure();

	//计算面积

	CAnglePartition TempAnglePartition;
	vector<AnglePartitionStruct> SectionAngleS;

	//2021.02.05 以角度值5 计算每个Slice中 没有点云点的数据个数
	TempAnglePartition.PartitionPoints(D2PlanarPoints, 5, SectionAngleS, true);

	NoPointAngleCount = 0;
	Roughness = 0;

	double BarkThick = 0;
	for (int j = 0; j < SectionAngleS.size(); j++)
	{
		if (SectionAngleS[j].PointIndexs.size() == 0)
		{
			NoPointAngleCount++;
		}
		else
		{
			double MinDis = EPSP6, MaxDis = EPSM6;
			for (int kk = 0; kk < SectionAngleS[j].PointIndexs.size(); kk++)
			{
				double Dis = PointDis(D2PlanarPoints->points[SectionAngleS[j].PointIndexs[kk]], D2Center, true);
				if (Dis > MaxDis) MaxDis = Dis;
				if (Dis < MinDis) MinDis = Dis;
			}
			BarkThick = BarkThick + (MaxDis - MinDis);
		}
	}
	
	Roughness = BarkThick / (SectionAngleS.size() - NoPointAngleCount);

	Completeness = 100 * (SectionAngleS.size() - NoPointAngleCount) / SectionAngleS.size();

	CStemDiameterAndProfileCurveBasalArea StemDiameter;
	StemDiameter.SetInputCloud(D2PlanarPoints, D2Center, GrownDirection);
	ProfileArea = StemDiameter.GetProfileCurveAndArea(D2PlanarPoints, BSplineProfilePoints, 5);
	
	SimulatingForm.lineEditBA_BSplineProfileCurve->setText(QString::number(ProfileArea));

	SimulatingForm.lineEditBA_Cylinder->setText(QString::number(M_PI * DCylinder * DCylinder / 4.0));
	SimulatingForm.lineEditBA_Bezier->setText(QString::number(M_PI * DBezierConvex * DBezierConvex / 4.0));
	SimulatingForm.lineEditD_Roughness->setText(QString::number(Roughness));
	SimulatingForm.lineEditD_Completeness->setText(QString::number(Completeness));

	ShowModelPoints(0);
}

void CSimulateMeasure::ShowModelPoints(int CheckValue)
{
	Viewer->removePointCloud(SimulatingPointsStr);
	Viewer->removePointCloud(PlanarPointsStr);
	Viewer->removePointCloud(CircleFittingPointsStr);
	Viewer->removePointCloud(CylinderFittingPointsStr);
	//Viewer->removeAllShapes();
	Viewer->removePointCloud(SplineFittingStr);
	Viewer->removePointCloud(BezierFittingStr);

	if (SimulatingForm.checkBoxShowMeasurePoints->checkState())
	{
		PointBase::SetPointColor(SimulatingPoints, ColorBase::RedColor);
		ShowPoints(SimulatingPoints, SimulatingPointsStr);

		PointBase::SetPointColor(D3PlanarPoints, ColorBase::BlueColor);
		ShowPoints(D3PlanarPoints, PlanarPointsStr);
	}

	if (SimulatingForm.checkBoxShowCircle->checkState())
	{
		
		PointBase::SetPointColor(CircleFittingPoints, ColorBase::RedColor);
		ShowPoints(CircleFittingPoints, CircleFittingPointsStr);
	}

	if (SimulatingForm.checkBoxShowCylinder->checkState())
	{
		GeometryBase::GetCylinderPoints(CylinderFittingPoints, CylinderCenterPoint,
			GrownDirection, DCylinder / 2.0, HCylinder);
		PointBase::SetPointColor(CylinderFittingPoints, ColorBase::BlueColor);
		ShowPoints(CylinderFittingPoints, CylinderFittingPointsStr);
	}

	if (SimulatingForm.checkBoxShowConvexHullLine->checkState())
	{
		for(int i = 0 ; i < ConvexHullPoints->points.size(); i++)
		{
			Viewer->removeShape(ConvexHullLineStr + StringBase::IntToStr(i));
			Viewer->addLine(ConvexHullPoints->points[i],
				ConvexHullPoints->points[(i + 1) % ConvexHullPoints->points.size()],
				0, 255, 0, ConvexHullLineStr + StringBase::IntToStr(i));
		}
	}

	if (SimulatingForm.checkBoxShowBSpline->checkState())
	{
		PointBase::SetPointColor(BSplineFittingPoints, ColorBase::RedColor);
		ShowPoints(BSplineFittingPoints, SplineFittingStr);
	}

	if (SimulatingForm.checkBoxShowBezier->checkState())
	{		
		PointBase::SetPointColor(BezierCurvePoints, ColorBase::BlueColor);
		ShowPoints(BezierCurvePoints, BezierFittingStr);
	}

	if (SimulatingForm.checkBoxShowConvexhullPoints->checkState())
	{
		PointBase::SetPointColor(ConvexHullPoints, ColorBase::RedColor);
		ShowPoints(ConvexHullPoints, ConvexHullPointsStr, PointSize * 2);
	}

	//if (CaliperDs.size() == 0)
	//	CaliperMeasure();

	emitUpdateUI();
}

void CSimulateMeasure::StemDividingByStemAxisCurve()
{
	cout<<"Stem Dividing By Stem Axis Curve is Starting"<<endl;
	int pos = OpenedFilePath.find_last_of('/');
	if (pos == -1)
		pos = OpenedFilePath.find_last_of('\\');
	OpenedFileName = OpenedFilePath.substr(pos + 1);
	pos = OpenedFileName.find('.');
	OpenedFileName = OpenedFileName.substr(0, pos);

	CStemSkeleton StemSkeleton;
	StemSkeleton.SetInputs(InputCloud, 5, true, 5);
	StemSkeleton.ConstructStemSplineCurve();

	PointBase::SetPointColor(StemSkeleton.StemSkeletonSpline.CurvePoints, ColorBase::BlueColor);
	ShowPoints(StemSkeleton.StemSkeletonSpline.CurvePoints, "StemSkeleton", 2);

	double TempHeight = StemSkeleton.StemSkeletonSpline.GetSplineHeight();
	if (isnan(TempHeight)) 
	{
		cout << "Stem Height is nan, " << TempHeight << endl;
		return;
	}

	cout << "Stem Height is:" << TempHeight << endl;
	
	//从高度10开始，
	double CurHeight = 10;
	int i = 0;

	while (CurHeight < TempHeight)
	{
		double CurU =
			StemSkeleton.StemSkeletonSpline.GetUValueBySplineHeight(CurHeight);
		
		pcl::PointXYZRGB LowerPoint = StemSkeleton.StemSkeletonSpline.GetSplinePoint(CurU);

		pcl::PointXYZRGB CurStemGrowth =
			StemSkeleton.StemSkeletonSpline.GetSplineDerivativePoint(CurU, 1, true);

		if (CurStemGrowth.z < 0)
			CurStemGrowth.x = -CurStemGrowth.x, CurStemGrowth.y = -CurStemGrowth.y, 
			CurStemGrowth.z = -CurStemGrowth.z;

		pcl::PointXYZRGB UpperPoint = GeometryBase::GetPointAlongLine(CurStemGrowth, LowerPoint, //10);
			SimulatingForm.spinBox_SliceHeight->text().toDouble());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr D2SectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp2DSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		GeometryBase::GetPointsBetweenTwoPlanes(InputCloud, CurStemGrowth.x, CurStemGrowth.y, CurStemGrowth.z,
			-(CurStemGrowth.x * LowerPoint.x + CurStemGrowth.y * LowerPoint.y + CurStemGrowth.z * LowerPoint.z),
			CurStemGrowth.x, CurStemGrowth.y, CurStemGrowth.z, 
			-(CurStemGrowth.x * UpperPoint.x + CurStemGrowth.y * UpperPoint.y + CurStemGrowth.z * UpperPoint.z),
			TempSectionPoints);
		
		if (TempSectionPoints->points.size() == 0)
		{
			CurHeight += 10;
			i++;
			continue;
		}

		GeometryBase::ProjectPointsToPlane(TempSectionPoints, Temp2DSectionPoints,
			CurStemGrowth, TempSectionPoints->points[0]);
		GeometryBase::RotateNormalToVertical(Temp2DSectionPoints, D2SectionPoints, CurStemGrowth);

		CAnglePartition TempAnglePartition;
		vector<AnglePartitionStruct> SectionAngleS;

		//为 True时计算 切向量 与 CenterPointofCurPartition
		double AngleValue = 2;
		TempAnglePartition.PartitionPoints(D2SectionPoints, AngleValue, SectionAngleS, true);
		
		int AngleCount = 0;
		for (int j = 0; j < SectionAngleS.size(); j++)
		{
			if (SectionAngleS[j].PointIndexs.size() == 0)
			{				
				AngleCount++;
			}
		}

		cout << "CurHeight:" << CurHeight <<", AngleZone Count:"<< 
			SectionAngleS.size() <<", No Point AngleZone Count:"<< AngleCount << endl;

		//缺失数据的角度不超过30度
		if (!(AngleCount > 30 / AngleValue))
		{
			//将点云旋转至垂直方向
			CurStemGrowth.r = 255, CurStemGrowth.g = 0, CurStemGrowth.b = 0;			

			//TempSectionPoints->points.push_back(CurStemGrowth);
			//将三维空间中的点云旋转
			GeometryBase::RotateNormalToVertical(TempSectionPoints, 
				Temp2DSectionPoints, CurStemGrowth);			

			PointBase::SaveToVTX(Temp2DSectionPoints, "I:\\000Comparsion\\EvoTLS2014\\"
				+ OpenedFileName + "_Data_" + StringBase::IntToStr(i) + "_"
				+ StringBase::FloatToStr(CurHeight) + ".vtx");
		}

		CurHeight += 10;
		i++;
	}
	cout << "Stem Dividing By Stem Axis Curve has been done!" << endl << endl;
}

//批处理每一个树干得到若干树干段 2020.11.09
void CSimulateMeasure::BatStemDividing()
{
	IsBat = true;
	char Drive[_MAX_DRIVE];
	char FilePath[_MAX_DIR];
	char Fname[_MAX_FNAME];
	char Ext[_MAX_EXT];
	_splitpath(OpenedFilePath.c_str(), Drive, FilePath, Fname, Ext);
		
	string FilePathStr;
	FilePathStr = string(Drive) + string(FilePath);
	vector<string> BatFiles;
	GetFiles(FilePathStr, BatFiles);

	for each (string FileNameStr in BatFiles)
	{
		_splitpath(FileNameStr.c_str(), Drive, FilePath, Fname, Ext);
		if ((strcmp(Ext, ".pcd") == 0) || (strcmp(Ext, ".vtx") == 0)
			|| (strcmp(Ext, ".las") == 0))
		{
			OpenedFilePath = FilePathStr + FileNameStr;
			cout<<"Current Processing file name:"<< OpenedFilePath <<endl;

			InputCloud->points.clear();
			PointBase::OpenPCLFile(OpenedFilePath, InputCloud);
			CTreeBase::SetInputCloud(CTreeBase::InputCloud);

			RefreshData();
			emitUpdateAppTitle(OpenedFilePath);
			emitUpdateUI();

			StemDividingByStemAxisCurve();
		}
	}
}

//2020.11.09 废弃不用
//void CSimulateMeasure::StemDividingByHorizontalPartition()
//{
//	//pcl::PointXYZRGB 
//	CHorizontalPartition HorizontalPartition;
//	HorizontalPartition.SetInputCloud(InputCloud);
//	//高度为 
//	HorizontalPartition.SetThickNess(SimulatingForm.lineEdit_ThickNess->text().toDouble());
//	HorizontalPartition.PatitionSection();
//	
//	int pos = OpenedFilePath.find_last_of('/');
//	FileName = OpenedFilePath.substr(pos + 1);
//	pos = FileName.find('.');
//	FileName = FileName.substr(0, pos);
//	cout << "befroe SectionsCount:" << HorizontalPartition.SectionsCount << endl;
//	//cout<<"FileName"<< FileName <<endl;
//	HorizontalPartition.SaveSectionSToEachFile("I:\\000SimulateMeasureData\\" + FileName, 
//			0, HorizontalPartition.SectionsCount);
//	cout << "After SectionsCount:" << HorizontalPartition.SectionsCount << endl;
//		
//	#pragma omp parallel
//	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
//	//for (int i = 0; i < 10; i++)
//	{
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
//
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr D2SectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp2DSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr BasePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr SimulateStemPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
//		pcl::PointXYZRGB TempStemGrowth;
//		
//		#pragma omp critical
//		HorizontalPartition.GetSectionPoints(i, TempSectionPoints);
//		
//		//cout <<"SectionsCount:"<< HorizontalPartition.SectionsCount << ", i:" << i << endl;
//		
//		//if (i == 4)
//		//	FileName = FileName;
//
//		if (TempSectionPoints->points.size() == 0)
//			continue;
//
//		CalculateDriection(TempSectionPoints, TempStemGrowth);
//
//		GeometryBase::ProjectPointsToPlane(TempSectionPoints, Temp2DSectionPoints, 
//			TempStemGrowth, TempSectionPoints->points[0]);
//		GeometryBase::RotateNormalToVertical(Temp2DSectionPoints, D2SectionPoints, TempStemGrowth);
//		
//		CAnglePartition TempAnglePartition;
//		vector<AnglePartitionStruct> SectionAngleS;
//		
//		//为 True时计算 切向量 与 CenterPointofCurPartition
//		double AngleValue = 2;
//		TempAnglePartition.PartitionPoints(D2SectionPoints, AngleValue, SectionAngleS, true);
//		int AngleCount = 0;
//		for (int j = 0; j < SectionAngleS.size(); j++)
//		{
//			if (SectionAngleS[j].PointIndexs.size() > 0)
//			{
//				BasePoints->points.push_back(SectionAngleS[j].CenterPointofCurPartition);
//				AngleCount++;
//			}
//		}
//
//		if (AngleCount >= SectionAngleS.size() - 30 / AngleValue)
//		{
//			PointBase::SavePCDToFileName(TempSectionPoints, "I:\\000Comparsion\\"
//				+ FileName + "_Data_" + StringBase::IntToStr(i) + ".pcd");
//		}
//
//		/*  生成虚拟的树干点云，生成的虚拟点云并不是很像，因此暂不考虑，
//		是否可以自动生成与树木点云相似的点云，值得后面研究，可以考虑采用深度学习的方法
//		if (BasePoints->points.size() == 0)
//			continue;
//
//		PointBase::SetPointColor(BasePoints, ColorBase::RedColor);
//
//		//#pragma omp critical
//		PointBase::SavePCDToFileName(BasePoints, "I:\\000—Base—SimulateMeasureData\\"
//			+ FileName + "_Base_" + StringBase::IntToStr(i) + ".pcd");
//		
//		//ShowPoints(BasePoints, "ResultPoints" + StringBase::ClockValue(), 3);		
//
//		
//		GenerateSimulatePoints(BasePoints, SimulateStemPoints, TempStemGrowth,
//			SimulatingForm.lineEdit_ThickNess->text().toDouble(), 0.5, 1, true, 0.2, 1.0);
//
//		PointsMove(SimulateStemPoints, 50, 0, 0);
//
//		#pragma omp critical
//		ShowPoints(SimulateStemPoints, "SimulateStemPoints" + StringBase::ClockValue(), 3);
//		//*/
//	}
//	
//}

//计算点云所在树干的生长方向  从中间位置选取两个夹角接近90度的点计算切平面的法向量，法向量的差积为方向
void CSimulateMeasure::CalculateDriection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempStemPoints,
	pcl::PointXYZRGB & TempStemGrowth)
{
	//pcl::PointXYZRGB 
	CHorizontalPartition HorizontalPartition;
	HorizontalPartition.SetInputCloud(TempStemPoints);
	//高度为 2 
	HorizontalPartition.SetThickNess(10);
	HorizontalPartition.PatitionSection();
	//HorizontalPartition.SaveSectionSToEachFile("I:\\SimulateMeasureData\\Data_1", 0, 100);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempSectionPoints (new pcl::PointCloud<pcl::PointXYZRGB>());

	HorizontalPartition.GetSectionPoints(HorizontalPartition.SectionsCount / 2, TempSectionPoints);
		
	if (TempSectionPoints->points.size() == 0)
	{
		GrownDirection.x = 0, GrownDirection.y = 0, GrownDirection.z;
		return;
	}
	//AnchorPoint = PointBase::GetGravityPoint(TempSectionPoints);	

	CAnglePartition AnglePartition;
	vector<AnglePartitionStruct> SectionAngleS;
	AnglePartition.PartitionPoints(TempSectionPoints, 5, SectionAngleS, true);

	pcl::PointXYZRGB SurfaceNormal0;
	SurfaceNormal0.x = SectionAngleS[0].TangentPlaneCoefficient.a;
	SurfaceNormal0.y = SectionAngleS[0].TangentPlaneCoefficient.b;
	SurfaceNormal0.z = SectionAngleS[0].TangentPlaneCoefficient.c;

	pcl::PointXYZRGB SurfaceNormal8;
	SurfaceNormal8.x = SectionAngleS[8].TangentPlaneCoefficient.a;
	SurfaceNormal8.y = SectionAngleS[8].TangentPlaneCoefficient.b;
	SurfaceNormal8.z = SectionAngleS[8].TangentPlaneCoefficient.c;

	pcl::PointXYZRGB SurfaceNormal17;
	SurfaceNormal17.x = SectionAngleS[17].TangentPlaneCoefficient.a;
	SurfaceNormal17.y = SectionAngleS[17].TangentPlaneCoefficient.b;
	SurfaceNormal17.z = SectionAngleS[17].TangentPlaneCoefficient.c;

	pcl::PointXYZRGB SurfaceNormal26;
	SurfaceNormal26.x = SectionAngleS[26].TangentPlaneCoefficient.a;
	SurfaceNormal26.y = SectionAngleS[26].TangentPlaneCoefficient.b;
	SurfaceNormal26.z = SectionAngleS[26].TangentPlaneCoefficient.c;

	pcl::PointXYZRGB SurfaceNorma35;
	SurfaceNorma35.x = SectionAngleS[35].TangentPlaneCoefficient.a;
	SurfaceNorma35.y = SectionAngleS[35].TangentPlaneCoefficient.b;
	SurfaceNorma35.z = SectionAngleS[35].TangentPlaneCoefficient.c;

	pcl::PointXYZRGB SurfaceNormal44;
	SurfaceNormal44.x = SectionAngleS[44].TangentPlaneCoefficient.a;
	SurfaceNormal44.y = SectionAngleS[44].TangentPlaneCoefficient.b;
	SurfaceNormal44.z = SectionAngleS[44].TangentPlaneCoefficient.c;

	pcl::PointXYZRGB SurfaceNormal53;
	SurfaceNormal53.x = SectionAngleS[53].TangentPlaneCoefficient.a;
	SurfaceNormal53.y = SectionAngleS[53].TangentPlaneCoefficient.b;
	SurfaceNormal53.z = SectionAngleS[53].TangentPlaneCoefficient.c;

	pcl::PointXYZRGB SurfaceNormal62;
	SurfaceNormal53.x = SectionAngleS[62].TangentPlaneCoefficient.a;
	SurfaceNormal53.y = SectionAngleS[62].TangentPlaneCoefficient.b;
	SurfaceNormal53.z = SectionAngleS[62].TangentPlaneCoefficient.c;

	pcl::PointXYZRGB Driection1 = PointBase::PointsCrossProduct(SurfaceNormal0, SurfaceNormal17);
	pcl::PointXYZRGB Driection2 = PointBase::PointsCrossProduct(SurfaceNorma35, SurfaceNormal53);
	pcl::PointXYZRGB Driection3 = PointBase::PointsCrossProduct(SurfaceNormal8, SurfaceNormal26);
	pcl::PointXYZRGB Driection4 = PointBase::PointsCrossProduct(SurfaceNormal44, SurfaceNormal62);

	if (Driection1.z < 0)
	{
		Driection1.x = -Driection1.x;
		Driection1.y = -Driection1.y;
		Driection1.z = -Driection1.z;
	}

	if (Driection2.z < 0)
	{
		Driection2.x = -Driection2.x;
		Driection2.y = -Driection2.y;
		Driection2.z = -Driection2.z;
	}

	if (Driection3.z < 0)
	{
		Driection3.x = -Driection3.x;
		Driection3.y = -Driection3.y;
		Driection3.z = -Driection3.z;
	}

	if (Driection4.z < 0)
	{
		Driection4.x = -Driection4.x;
		Driection4.y = -Driection4.y;
		Driection4.z = -Driection4.z;
	}

	TempStemGrowth.x = Driection1.x + Driection2.x + Driection3.x + Driection4.x;
	TempStemGrowth.y = Driection1.y + Driection2.y + Driection3.y + Driection4.y;
	TempStemGrowth.z = Driection1.z + Driection2.z + Driection3.z + Driection4.z;
	
	PointBase::PointNormalized(TempStemGrowth);	
	cout << "The Grown Direction of the current point cloud is: " << GrownDirection << endl;
	//PointBase::ShowDirection(Viewer, InputCloud->points[0], GrownDirection, 10);
}

//根据基础断面轮廓点，采用 BSpline样条生成断面轮廓曲线，然后再生成一段树干点云 2020.08.11
void CSimulateMeasure::GenerateSimulatePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr BaseProfilePoints,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutStemPoints,
	pcl::PointXYZRGB AxisDirection, double Height, double HeightInterval,
	double AngleSpace, bool HaveGaussianNoise,
	float NoiseMean, float NoiseStddev)
{
	//B-Spline 计算
	CSplineInterpolation SplineInterpolation;
	SplineInterpolation.SetInputs(BaseProfilePoints, 3, true);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> KnoteVector;
	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnoteVector);

	CSpline Spline;
	//闭合曲线
	Spline.SetSplineInputs(ControlPoints, 3, KnoteVector, true);
	Spline.FirstPointOfCloseCurve = BaseProfilePoints->points[0];
	Spline.CreateSpline();	
	Spline.ZippedSplinePoints();

	OutStemPoints->points.clear();
	OutStemPoints->points.insert(OutStemPoints->points.begin(),
		Spline.CurvePoints->points.begin(), Spline.CurvePoints->points.end());

	double TempHeight = 0;
	while (TempHeight <= Height)
	{
		TempHeight = TempHeight + HeightInterval;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempHeightPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::PointXYZRGB TempAnchorPoint =
			GeometryBase::GetPointAlongLine(AxisDirection, 
				Spline.CurvePoints->points[0], TempHeight, true);

		PointBase::PointCopy(Spline.CurvePoints, TempHeightPoints);

		///*
			if (HaveGaussianNoise)
			{
				static std::default_random_engine generator(time(0));

				//static std::normal_distribution<double> distx(NoiseMean, NoiseStddev);
				static std::normal_distribution<double> disty(NoiseMean, NoiseStddev);

				for (auto & p : TempHeightPoints->points)
				{
					//p.x = p.x + distx(generator);
					p.y = p.y + disty(generator);
				}
			}
		//*/

		PointsMove(TempHeightPoints,
			TempAnchorPoint.x - Spline.CurvePoints->points[0].x,
			TempAnchorPoint.y - Spline.CurvePoints->points[0].y,
			TempAnchorPoint.z - Spline.CurvePoints->points[0].z);
		
		OutStemPoints->points.insert(OutStemPoints->points.begin(),
			TempHeightPoints->points.begin(), TempHeightPoints->points.end());
	}
}
//模拟卡尺测量 2020.08.13
void CSimulateMeasure::CaliperMeasure()
{
	Viewer->removeAllShapes();
	CaliperDs.clear();

	AngleSpace = SimulatingForm.spinBox_AngleSpace->text().toDouble();	

	if (D2ConvexHullPoints->points.size() == 0)
		SimulateMeasure();	

	string ResultStr;

	double CurAngle = AngleSpace / 2, a = 0, b = 0, c = 0;
	for (int i = 0; i < 180 / AngleSpace; i++)	
	{			
		a = sin(CurAngle * M_PI / 180);
		b = cos(CurAngle * M_PI / 180);
		
		//cout <<"I:"<< i << ",a:" << a << ", b:" << b << endl;

		for (int k = 0; k < D2ConvexHullPoints->points.size(); k++)
		{
			double dOne = -(D2ConvexHullPoints->points[k].x * a +
				D2ConvexHullPoints->points[k].y * b +
				D2ConvexHullPoints->points[k].z * c);

			bool FindOtherConvexHull = false;

			int FindIndex = -1;

			for (int j = 0; j < D2ConvexHullPoints->points.size(); j++)
			{
				if (j != k)
				{
					double dTwo = -(D2ConvexHullPoints->points[j].x * a +
						D2ConvexHullPoints->points[j].y * b +
						D2ConvexHullPoints->points[j].z * c);

					pcl::PointCloud<pcl::PointXYZRGB>::Ptr BewteenPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

					GeometryBase::GetPointsBetweenTwoPlanes(D2ConvexHullPoints,
						a, b, c, dOne, a, b, c,
						dTwo, BewteenPoints, 1, true, true);

					if (BewteenPoints->points.size() == D2ConvexHullPoints->points.size())
					{
						//PointBase::SetPointColor(BewteenPoints, ColorBase::RedColor);
						//ShowPoints(BewteenPoints, "BewteenPointsStr", 10);

						//cout<<"D2ConvexHullPoints Size:"<< D2ConvexHullPoints->points.size() << endl;
						FindOtherConvexHull = true;
						FindIndex = j;
						break;
					}
				}
			}

			if (FindOtherConvexHull)
			{				
				double PlaneDis = GeometryBase::GetDisFromPointToPlane(
					D2ConvexHullPoints->points[FindIndex].x,
					D2ConvexHullPoints->points[FindIndex].y,
					D2ConvexHullPoints->points[FindIndex].z,
					a, b, c, dOne);
				//cout << "CurAngle:"<< CurAngle <<", Find, K:" << k << ", Index:" << FindIndex <<", Dis:"<< PlaneDis << endl;

				ResultStr = StringBase::FloatToStr(a) + "," + StringBase::FloatToStr(b) + "," +
					StringBase::IntToStr(k) + "," + StringBase::IntToStr(FindIndex);
				
				SaveStringToFile("I:\\000Comparsion\\Final\\CaliperConvexIndexs.txt", ResultStr);

				CaliperDs.push_back(PlaneDis);

				if (SimulatingForm.checkBoxShowCaliper->checkState())
				{
					PointBase::ShowPlane(Viewer, a, b, c, 
						D2ConvexHullPoints->points[k], StringBase::ClockValue() + "One", 
						StringBase::IntToStr(i) + "_0", true, 3, 0.5, 0, 255);
					PointBase::ShowPlane(Viewer, a, b, c, 
						D2ConvexHullPoints->points[FindIndex], StringBase::ClockValue() + "Two",
						StringBase::IntToStr(i) + "_1", true, 3, 0.5);
				}
				break;
			}			
		}

		CurAngle += AngleSpace;
	}

	

	VectorBase<double> VectorBasedouble;

	DCaliperMax = CaliperDs[VectorBasedouble.GetMaxIndex(CaliperDs)];
	DCaliperMin = CaliperDs[VectorBasedouble.GetMinIndex(CaliperDs)];
	DCaliper = VectorBasedouble.CalcMeans(CaliperDs);

	//DCaliperOvality = 100.0 * (DCaliperMax - DCaliperMin) / DCaliper;
	DCaliperOvality = 100.0 * (DCaliperMax - DCaliperMin) / DCaliperMax;

	SimulatingForm.lineEditD_Caliper->setText(QString::number(DCaliper));
	SimulatingForm.lineEditD_Caliper_Max->setText(QString::number(DCaliperMax));
	SimulatingForm.lineEditD_Caliper_Min->setText(QString::number(DCaliperMin));
	SimulatingForm.lineEditD_Caliper_Ovality->setText(QString::number(DCaliperOvality));

	emitUpdateUI();
}

//寻找距离 CurPoint 点最近的一个凸包点 2020.08.13
int CSimulateMeasure::FindNearestConvexHull(pcl::PointXYZRGB CurPoint)
{
	double Dis = EPSP6;
	int TempIndex = -1;

	Dis = PointDis(CurPoint, ConvexHullPoints->points[0]);
	TempIndex = 0;

	for (int i = 1; i < ConvexHullPoints->points.size(); i++)
	{
		double TempDis = PointDis(CurPoint, ConvexHullPoints->points[i]);

		if (TempDis < Dis)
		{
			Dis = TempDis;
			TempIndex = i;
		}
	}

	return TempIndex;
}

//批量计算
void CSimulateMeasure::BatSimulateMeasure()
{
	IsBat = true;
	char Drive[_MAX_DRIVE];
	char FilePath[_MAX_DIR];
	char FName[_MAX_FNAME];
	char Ext[_MAX_EXT];
	_splitpath(OpenedFilePath.c_str(), Drive, FilePath, FName, Ext);

	ResultFileName = string(Drive) + string(FilePath) + StringBase::DateStr() + "_Result.txt";
	string FilePathStr;
	FilePathStr = string(Drive) + string(FilePath);
	vector<string> BatFiles;
	GetFiles(FilePathStr, BatFiles);

//#pragma omp parallel for
	//for each (string FileNameStr in BatFiles)
	for (int i = 0; i < BatFiles.size(); i++)
	{
		string FileNameStr = BatFiles[i];
		_splitpath(FileNameStr.c_str(), Drive, FilePath, FName, Ext);
		if ((strcmp(Ext, ".pcd") == 0) || (strcmp(Ext, ".vtx") == 0) 
			|| (strcmp(Ext, ".las") == 0))
		{
			OpenedFilePath = FilePathStr + FileNameStr;
			OpenedFileName = FName;

			InputCloud->points.clear();
			//因为最后一个点是当前点云的生长方向，所以不能移动到原点位置
			//PointBase::OpenPCLFile(OpenedFilePath, InputCloud, false);
			PointBase::OpenPCLFile(OpenedFilePath, InputCloud, true);
			CTreeBase::SetInputCloud(CTreeBase::InputCloud);

			RefreshData();
			emitUpdateAppTitle(OpenedFilePath);
			emitUpdateUI();

			SimulateMeasure();

			if (abs(GrownDirection.x) < EPSM9 && abs(GrownDirection.y) < EPSM9 
				&& abs(GrownDirection.z) < EPSM9)
				continue;

			//#pragma omp critical
			SaveValueToFile(FileNameStr);
		}
	}
}

void CSimulateMeasure::SaveValueToFile(string ProcFileName)
{	
	string ResultStr;
	ResultStr = GetDateTime() + "	" + ProcFileName;

	//read height value
	int TempIndexUnderLine = ProcFileName.find_last_of('_');
	int TempIndexDot = ProcFileName.find_last_of('.');
	string TempHeightStr = ProcFileName.substr(TempIndexUnderLine + 1, TempIndexDot - TempIndexUnderLine - 1);
	ResultStr = ResultStr + "	" + TempHeightStr;
	ResultStr = ResultStr + "	" + SimulatingForm.spinBox_AngleSpace->text().toStdString();
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditD_Caliper->text().toStdString();
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditD_Caliper_Max->text().toStdString();
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditD_Caliper_Min->text().toStdString();
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditD_Caliper_Ovality->text().toStdString();
		
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditD_Cylinder->text().toStdString();
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditD_Circle->text().toStdString();
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditD_ConvexHullLine->text().toStdString();
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditD_BSpline->text().toStdString();
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditD_BezierConvex->text().toStdString();
	//BA
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditBA_Cylinder->text().toStdString();
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditBA_BSplineProfileCurve->text().toStdString();
	ResultStr = ResultStr + "	" + SimulatingForm.lineEditBA_Bezier->text().toStdString();
	ResultStr = ResultStr + "	" + StringBase::IntToStr(NoPointAngleCount);
	ResultStr = ResultStr + "	" + StringBase::FloatToStr(Roughness);
	
	
	//ResultStr = ResultStr + "	" + SimulatingForm.lineEditVolumeBezierBefore->text().toStdString();
	////ResultStr = ResultStr + "	" + SimulatingForm.lineEditVolumeBezierAfter->text().toStdString();
	////ResultStr = ResultStr + "	" + SimulatingForm.lineEditVolumeProfileCurveBefore->text().toStdString();
	//ResultStr = ResultStr + "	" + SimulatingForm.lineEditVolumeProfileCurveAfter->text().toStdString();
	////ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditVolumeSlicesSurfaceAfter->text().toStdString();
	//ResultStr = ResultStr + "	" + (QString::number(CalcVolumeByStemSplineSurface(2), 10, 4)).toStdString();
	//ResultStr = ResultStr + "	" + (QString::number(CalcVolumeByStemSplineSurface(1), 10, 4)).toStdString();	////
	//ResultStr = ResultStr + "	" + (QString::number(CalcVolumeByStemSplineSurface(0.5), 10, 4)).toStdString();

	SaveStringToFile(ResultFileName, ResultStr);
}