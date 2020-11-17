#include "StemVolumeVerticalSlices.h"

void CStemVolumeVerticalSlices::FormReset()
{
	StemVolumeVerticalSlices.lineEditDiameterErrorCircleFitting->setText("");
	StemVolumeVerticalSlices.lineEditDiameterErrorBezier->setText("");
	StemVolumeVerticalSlices.lineEditVolumeCricleFittingAfter->setText("");
	StemVolumeVerticalSlices.lineEditVolumeCricleFittingBefore->setText("");
	StemVolumeVerticalSlices.lineEditVolumeCylinderFittingAfter->setText("");
	StemVolumeVerticalSlices.lineEditVolumeCylinderFittingBefore->setText("");
	StemVolumeVerticalSlices.lineEditVolumeBezierAfter->setText("");
	StemVolumeVerticalSlices.lineEditVolumeBezierBefore->setText("");
	StemVolumeVerticalSlices.lineEditVolumeProfileCurveAfter->setText("");
	StemVolumeVerticalSlices.lineEditVolumeProfileCurveBefore->setText("");	
	StemVolumeVerticalSlices.lineEditAverageDiameter->setText("");
	StemVolumeVerticalSlices.lineEditVolumeSlicesSurfaceAfter->setText("");
	//StemVolumeVerticalSlices.lineEditStemHeight->setText("");

	BeforeRefineProfileCurvePoints->points.clear();
	AfterRefineProfileCurvePoints->points.clear();
	BeforeRefineBezierCurvePoints->points.clear();
	AfterRefineBezierCurvePoints->points.clear();
	RefinedStemGridPoints->points.clear();
	AfterSplineSurfacePoints->points.clear();

	Viewer->removePointCloud(SlicesPointsStr);
	Viewer->removePointCloud(RefinedStemGridPointsStr);
	Viewer->removePointCloud(BeforeRefinedPointsStr);
	Viewer->removePointCloud(BeforeRefineProfileCurvePointsStr);
	Viewer->removePointCloud(AfterRefineProfileCurvePointsStr);
	Viewer->removePointCloud(AfterSplineSurfaceStr);
	
	emitUpdateUI();
}

void CStemVolumeVerticalSlices::OnShow()
{
	StemVolumeVerticalSlices.lineEditStemHeight->setText(QString::number(abs(ZMax - ZMin), 10, 4));
	StemVolumeVerticalSlices.spinBoxThickNess->setMaximum(abs(ZMax - ZMin));
	StemVolumeVerticalSlices.spinBoxThickNess->setValue(5);
}

void CStemVolumeVerticalSlices::RefreshData()
{
	//Invoking Base class RefreshData method
	CTreeBase::RefreshData();

	HorizontalPartition.SetInputCloud(InputCloud);
	AdjustThickNess(0);

	ShowSlicesPoints(StemVolumeVerticalSlices.checkBoxVerticalSlices->checkState());
	FormReset();	
	OnShow();
}

CStemVolumeVerticalSlices::CStemVolumeVerticalSlices(QGroupBox * ParentWin, string Type)
{
	QWidget *widget = new QWidget(ParentWin);
	StemVolumeVerticalSlices.setupUi(widget);

	connect(StemVolumeVerticalSlices.checkBoxVerticalSlices, SIGNAL(stateChanged(int)), this, SLOT(ShowSlicesPoints(int)));
	connect(StemVolumeVerticalSlices.spinBoxThickNess, SIGNAL(valueChanged(int)), this, SLOT(AdjustThickNess(int)));
	connect(StemVolumeVerticalSlices.spinBoxAngleValue, SIGNAL(valueChanged(int)), this, SLOT(AdjustAngle(int)));

	connect(StemVolumeVerticalSlices.pushButtonHoleFill, SIGNAL(clicked()), this, SLOT(FillRepairBySpline()));

	connect(StemVolumeVerticalSlices.checkBoxShowOriginalPoints, SIGNAL(stateChanged(int)), this,
		SLOT(ShowOriginalPoints(int)));
	connect(StemVolumeVerticalSlices.checkBoxShowRepairedPoints, SIGNAL(stateChanged(int)), this,
		SLOT(ShowRepairedPoints(int)));

	connect(StemVolumeVerticalSlices.pushButtonCalcStemVolume, SIGNAL(clicked()), this, SLOT(CalcStemByMethods()));
	
	connect(StemVolumeVerticalSlices.checkBoxShowProfileAfter, SIGNAL(stateChanged(int)), this,
		SLOT(ShowProfileAfter(int)));
	connect(StemVolumeVerticalSlices.checkBoxShowProfileBefore, SIGNAL(stateChanged(int)), this,
		SLOT(ShowProfileBefore(int)));
	connect(StemVolumeVerticalSlices.checkBoxShowBezierAfter, SIGNAL(stateChanged(int)), this,
		SLOT(ShowBezierAfter(int)));
	connect(StemVolumeVerticalSlices.checkBoxShowBezierBefore, SIGNAL(stateChanged(int)), this,
		SLOT(ShowBezierBefore(int)));

	connect(StemVolumeVerticalSlices.checkBoxShowSurface, SIGNAL(stateChanged(int)), this,
		SLOT(ShowSplineSurfacePoints(int)));

	connect(StemVolumeVerticalSlices.pushButtonBat, SIGNAL(clicked()), this, SLOT(Bat()));

	//StemVolumeVerticalSlices.pushButtonCalcStemVolume->setEnabled(false);
	//StemVolumeVerticalSlices.pushButtonShowCircle->setEnabled(false);
	//StemVolumeVerticalSlices.pushButtonShowCylinder->setEnabled(false);
	//StemVolumeVerticalSlices.pushButtonShowProfileCurve->setEnabled(false);
	//StemVolumeVerticalSlices.pushButtonShowSlicesSurface->setEnabled(false);

	AllRefinedStemGridPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	RefinedStemGridPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	BeforeRefineProfileCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	AfterRefineProfileCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	BeforeRefineBezierCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	AfterRefineBezierCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	AfterSplineSurfacePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	//MaxThreadNum = 10;
	IsRepaired = false;
	IsBat = false;
	widget->show();	
}
CStemVolumeVerticalSlices::~CStemVolumeVerticalSlices()
{		
	BeforeRefineProfileCurvePoints->points.clear();
	AfterRefineProfileCurvePoints->points.clear();
	BeforeRefineBezierCurvePoints->points.clear();
	AfterRefineBezierCurvePoints->points.clear();
	RefinedStemGridPoints->points.clear();

	Viewer->removePointCloud(SlicesPointsStr);
	Viewer->removePointCloud(RefinedStemGridPointsStr);
	Viewer->removePointCloud(BeforeRefinedPointsStr);
	Viewer->removePointCloud(BeforeRefineProfileCurvePointsStr);
	Viewer->removePointCloud(AfterRefineProfileCurvePointsStr);
	Viewer->removePointCloud(BeforeRefineBezierCurvePointsStr);
	Viewer->removePointCloud(AfterRefineBezierCurvePointsStr);	
	Viewer->removePointCloud(AfterSplineSurfaceStr);

	emitUpdateUI();
	emitUpdateUI();
}

void CStemVolumeVerticalSlices::AdjustAngle(int Value)
{
	IsRepaired = false;
	FormReset();
}

void CStemVolumeVerticalSlices::AdjustThickNess(int Value)
{
	IsRepaired = false;
	//StemVolumeVerticalSlices.doubleSpinBoxHeightStep->setMaximum(Value);

	FormReset();	
	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetThickNess(StemVolumeVerticalSlices.spinBoxThickNess->text().toDouble());
	HorizontalPartition.PatitionSection();	
	ShowSlicesPoints(StemVolumeVerticalSlices.checkBoxVerticalSlices->checkState());
}

void CStemVolumeVerticalSlices::ShowSlicesPoints(int CheckValue)
{
	if (InputCloud == NULL) return;

	QWidget *widget = (QWidget *)sender();
	if (widget == NULL) return;

	HorizontalPartition.SetViewer(Viewer);
	HorizontalPartition.p_TreePclQtGui = p_TreePclQtGui;
	   
	if (StemVolumeVerticalSlices.checkBoxVerticalSlices->checkState() == 2)
	{			
		if (widget->objectName() == "spinBoxThickNess" || HorizontalPartition.SectionsCount == 0)
		{
			AdjustThickNess(0);
		}
		HorizontalPartition.ShowSectionPoints(0, HorizontalPartition.SectionsCount);
	}
	else if (widget->objectName() == "checkBoxVerticalSlices" 
		&& StemVolumeVerticalSlices.checkBoxVerticalSlices->checkState() != 2)
	{
		HorizontalPartition.UnShowSectionPoints();
	}
}

void CStemVolumeVerticalSlices::ShowOriginalPoints(int CheckValue)
{
	if (CheckValue == 2)
	{		
		PointBase::SetPointColor(StemGridPointsRefine.BeforeRefinePoints, ColorBase::RedColor);
		PointBase::ShowPointXYZRGB(Viewer, StemGridPointsRefine.BeforeRefinePoints, BeforeRefinedPointsStr, PointSize);
	}
	else
	{
		Viewer->removePointCloud(BeforeRefinedPointsStr);
	}
	emitUpdateUI();
}

void CStemVolumeVerticalSlices::ShowRepairedPoints(int CheckValue)
{
	if (CheckValue == 2)
	{
		StemGridPointsRefine.GetRefinedPoints(RefinedStemGridPoints);
		
		PointsMove(RefinedStemGridPoints, -60, 0, 0);
		////PointBase::SetPointColor(RefinedStemGridPoints, ColorBase::BlueColor);
		PointBase::ShowPointXYZRGB(Viewer, RefinedStemGridPoints, RefinedStemGridPointsStr, PointSize);		
	}
	else
	{	
		Viewer->removePointCloud(RefinedStemGridPointsStr);
	}
	emitUpdateUI();
}

void CStemVolumeVerticalSlices::ShowProfileAfter(int CheckValue)
{
	Viewer->removePointCloud(AfterRefineProfileCurvePointsStr);
	if (CheckValue == 2)
	{
		PointBase::ShowPointXYZRGB(Viewer, AfterRefineProfileCurvePoints, AfterRefineProfileCurvePointsStr, PointSize);
		//PointBase::ShowPointXYZRGB(Viewer, BeforeRefineProfileCurvePoints, BeforeRefineProfileCurvePointsStr, 2);
	}
	emitUpdateUI();
}

void CStemVolumeVerticalSlices::ShowProfileBefore(int CheckValue)
{
	Viewer->removePointCloud(BeforeRefineProfileCurvePointsStr);
	if (CheckValue == 2)
	{
		//PointBase::ShowPointXYZRGB(Viewer, AfterRefineProfileCurvePoints, AfterRefineProfileCurvePointsStr, 2);
		PointBase::ShowPointXYZRGB(Viewer, BeforeRefineProfileCurvePoints, BeforeRefineProfileCurvePointsStr, PointSize);
	}
	emitUpdateUI();
}

void CStemVolumeVerticalSlices::ShowBezierAfter(int CheckValue)
{
	Viewer->removePointCloud(AfterRefineBezierCurvePointsStr);
	if (CheckValue == 2)
	{
		PointBase::ShowPointXYZRGB(Viewer, AfterRefineBezierCurvePoints, AfterRefineBezierCurvePointsStr, PointSize);
		//PointBase::ShowPointXYZRGB(Viewer, BeforeRefineProfileCurvePoints, BeforeRefineProfileCurvePointsStr, 2);
	}
	emitUpdateUI();
}

void CStemVolumeVerticalSlices::ShowBezierBefore(int CheckValue)
{
	Viewer->removePointCloud(BeforeRefineBezierCurvePointsStr);
	if (CheckValue == 2)
	{
		//PointBase::ShowPointXYZRGB(Viewer, AfterRefineProfileCurvePoints, AfterRefineProfileCurvePointsStr, 2);
		PointBase::ShowPointXYZRGB(Viewer, BeforeRefineBezierCurvePoints, BeforeRefineBezierCurvePointsStr, PointSize);
	}
	emitUpdateUI();
}

void CStemVolumeVerticalSlices::ShowSplineSurfacePoints(int CheckValue)
{
	Viewer->removePointCloud(AfterSplineSurfaceStr);
	if (CheckValue == 2)
	{
		//if (AfterSplineSurfacePoints->points.size() == 0)
			SurfaceInterpolation.GetSurfacePoints(AfterSplineSurfacePoints);
		//PointBase::ShowPointXYZRGB(Viewer, AfterRefineProfileCurvePoints, AfterRefineProfileCurvePointsStr, 2);

		PointBase::ShowPointXYZRGB(Viewer, AfterSplineSurfacePoints, AfterSplineSurfaceStr, PointSize);
	}
	emitUpdateUI();
}

//Repair holes of point cloud
void CStemVolumeVerticalSlices::FillRepairBySpline()
{		
	emitUpdateStatusBar("Repair is completing, please wait!", 5000);	
	StemVolumeVerticalSlices.pushButtonHoleFill->setEnabled(false);
	FormReset();
	double AngleValue = StemVolumeVerticalSlices.spinBoxAngleValue->text().toDouble();
	emitUpdateUI();
		
	if (StemVolumeVerticalSlices.checkBoxVerticalSlices->checkState() != 2)
	{
		if (abs(StemVolumeVerticalSlices.spinBoxThickNess->text().toDouble() < EPSM3))
			StemVolumeVerticalSlices.spinBoxThickNess->setValue(5);

		HorizontalPartition.SetInputCloud(InputCloud);
		HorizontalPartition.SetThickNess(StemVolumeVerticalSlices.spinBoxThickNess->text().toDouble());
		HorizontalPartition.PatitionSection();
	}

	StemGridPointsRefine.SetInput(InputCloud, HorizontalPartition.GeometryCenterPointsPtr, 
		&HorizontalPartition, AngleValue);
	StemGridPointsRefine.SetViewer(Viewer);
	StemGridPointsRefine.p_TreePclQtGui = this->p_TreePclQtGui;
	
	//两种修复方式，
	if (StemVolumeVerticalSlices.radioButtonBySplineCurve->isChecked())
		StemGridPointsRefine.RefineStemGridPointsBySplineCurve();
	else if (StemVolumeVerticalSlices.radioButtonByStemAxisCurve->isChecked())	
		StemGridPointsRefine.RefineStemGridPointsByStemAxisCurve();	

	ShowRepairedPoints(StemVolumeVerticalSlices.checkBoxShowRepairedPoints->checkState());
	
	//PointBase::SetPointColor(RefinedStemGridPoints, ColorBase::RedColor);

	//AllRefinedStemGridPoints->points.insert(AllRefinedStemGridPoints->points.end(),
	//	RefinedStemGridPoints->points.begin(), RefinedStemGridPoints->points.end());
	//
	//StemGridPointsRefine.SetInput(InputCloud, HorizontalPartition.GeometryCenterPointsPtr,
	//	&HorizontalPartition, 2);
	//StemGridPointsRefine.SetViewer(Viewer);
	//StemGridPointsRefine.p_TreePclQtGui = this->p_TreePclQtGui;

	////两种修复方式，
	//if (StemVolumeVerticalSlices.radioButtonBySplineCurve->isChecked())
	//	StemGridPointsRefine.RefineStemGridPointsBySplineCurve();
	//else if (StemVolumeVerticalSlices.radioButtonByStemAxisCurve->isChecked())
	//	StemGridPointsRefine.RefineStemGridPointsByStemAxisCurve();

	//ShowRepairedPoints(StemVolumeVerticalSlices.checkBoxShowRepairedPoints->checkState());

	//PointBase::SetPointColor(RefinedStemGridPoints, ColorBase::BlueColor);
	//AllRefinedStemGridPoints->points.insert(AllRefinedStemGridPoints->points.end(),
	//	RefinedStemGridPoints->points.begin(), RefinedStemGridPoints->points.end());

	//StemGridPointsRefine.SetInput(InputCloud, HorizontalPartition.GeometryCenterPointsPtr,
	//	&HorizontalPartition, 1);
	//StemGridPointsRefine.SetViewer(Viewer);
	//StemGridPointsRefine.p_TreePclQtGui = this->p_TreePclQtGui;

	////两种修复方式，
	//if (StemVolumeVerticalSlices.radioButtonBySplineCurve->isChecked())
	//	StemGridPointsRefine.RefineStemGridPointsBySplineCurve();
	//else if (StemVolumeVerticalSlices.radioButtonByStemAxisCurve->isChecked())
	//	StemGridPointsRefine.RefineStemGridPointsByStemAxisCurve();

	//ShowRepairedPoints(StemVolumeVerticalSlices.checkBoxShowRepairedPoints->checkState());

	//PointBase::SetPointColor(RefinedStemGridPoints, ColorBase::GreenColor);
	//AllRefinedStemGridPoints->points.insert(AllRefinedStemGridPoints->points.end(),
	//	RefinedStemGridPoints->points.begin(), RefinedStemGridPoints->points.end());

	//ShowPoints(AllRefinedStemGridPoints, "AllRefinedStemGridPoints", 2);

	//2019.07.16 暂不计算
	CalcStemDiameterErrorsAfterRefine();	

	emitUpdateUI();
	StemVolumeVerticalSlices.pushButtonCalcStemVolume->setEnabled(true);
	StemVolumeVerticalSlices.pushButtonHoleFill->setEnabled(true);
	IsRepaired = true;
	emitUpdateStatusBar("Repair is completed!", 5000);	
}

void CStemVolumeVerticalSlices::CalcStemDiameterErrorsAfterRefine()
{
	if (HorizontalPartition.SectionsCount == 0) return;
	double MeanErrorCircle = 0, MeanErrorBezier = 0, MeanBezier = 0;	

	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr OriginalSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		HorizontalPartition.GetSectionPoints(i, OriginalSectionPoints);
		CStemDiameter OriginalStemDiameter;

		OriginalStemDiameter.SetInputCloud(OriginalSectionPoints);
		double OriginalDCricle = OriginalStemDiameter.DRetrievalByCircleFittingIn2D();
		double OriginalDBezier = OriginalStemDiameter.DRetrievalByBezierCurve();		

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr RefineSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		StemGridPointsRefine.GetRefinedPointsBySliceIndex(RefineSectionPoints, i);
		CStemDiameter RefineStemDiameter;
		RefineStemDiameter.SetInputCloud(RefineSectionPoints);
		double RefineDCricle = RefineStemDiameter.DRetrievalByCircleFittingIn2D();
		double RefineDBezier = RefineStemDiameter.DRetrievalByBezierCurve();

		//if (isnan(RefineDBezier))
		//{
		//	PointBase::SavePCDToFileName(OriginalSectionPoints, "I:\\OriginalError.pcd");
		//	PointBase::SavePCDToFileName(OriginalStemDiameter.XYConvexhullPoints, "I:\\OriginalXYConvexhullPointsError.pcd");

		//	PointBase::SavePCDToFileName(RefineSectionPoints, "I:\\RefineError.pcd");
		//	PointBase::SavePCDToFileName(RefineStemDiameter.XYConvexhullPoints, "I:\\RefineXYConvexhullPointsError.pcd");
		//}

		MeanErrorCircle = MeanErrorCircle + abs(OriginalDCricle - RefineDCricle);
		MeanErrorBezier = MeanErrorBezier + abs(OriginalDBezier - RefineDBezier);
		MeanBezier = MeanBezier + OriginalDBezier;
		//if (i == 0)
		//{
		//	PointBase::SavePCDToFileName(OriginalSectionPoints, 
		//		"3DStemModelForMeasure\\Original" + QString::number(i, 10).toStdString() + ".pcd");
		//	PointBase::SavePCDToFileName(RefineSectionPoints, 
		//		"3DStemModelForMeasure\\Refine" + QString::number(i, 10).toStdString() + ".pcd");
		//	//RefineStemDiameter.
		//}
	}

	StemVolumeVerticalSlices.lineEditDiameterErrorBezier->setText(
		QString::number(MeanErrorBezier / HorizontalPartition.SectionsCount, 10, 4));
	StemVolumeVerticalSlices.lineEditDiameterErrorCircleFitting->setText(
		QString::number(MeanErrorCircle / HorizontalPartition.SectionsCount, 10, 4));

	StemVolumeVerticalSlices.lineEditAverageDiameter->setText(
		QString::number(MeanBezier / HorizontalPartition.SectionsCount, 10, 4));
}

double CStemVolumeVerticalSlices::DRetrievalByCircleFittingFromRefinedPoints(int SliceIndex)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoint(new pcl::PointCloud<pcl::PointXYZRGB>());
	StemGridPointsRefine.GetRefinedPointsBySliceIndex(TempPoint, SliceIndex);
	return GeometryBase::CircleFittingByLeastSquaresFitting(TempPoint) * 2.0;
}

double CStemVolumeVerticalSlices::DRetrievalByBezierFromRefinedPoints(int SliceIndex)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoint(new pcl::PointCloud<pcl::PointXYZRGB>());
	StemGridPointsRefine.GetRefinedPointsBySliceIndex(TempPoint, SliceIndex);

	CStemDiameter StemDiameter;
	StemDiameter.SetInputCloud(TempPoint);
	return StemDiameter.DRetrievalByBezierCurve();
}

double CStemVolumeVerticalSlices::DRetrievalByCircleFittingFromSlice(int SliceIndex)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BeforePoint(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (int i = 0; i < HorizontalPartition.SectionsVector[SliceIndex].Indexs.size(); i++)
	{
		int TempPointIndex = HorizontalPartition.SectionsVector[SliceIndex].Indexs[i];
		BeforePoint->points.push_back(InputCloud->points[TempPointIndex]);
	}
	return GeometryBase::CircleFittingByLeastSquaresFitting(BeforePoint) * 2.0;
}

void CStemVolumeVerticalSlices::CalcStemByMethods()
{	
	emitUpdateStatusBar("The stem volume calculation is completing, please wait!", 5000);
	StemVolumeVerticalSlices.pushButtonCalcStemVolume->setEnabled(false);
	StemVolumeVerticalSlices.pushButtonHoleFill->setEnabled(false);
	
	if (!IsRepaired)
	{
		FillRepairBySpline();
	}

	CircleStemVolumeBefore = 0, CylinderStemVolumeBefore = 0, 
	BezierStemVolumeBefore = 0,
	ProfileCurveStemVolumeBefore = 0, SliceSurfaceStemVolumeBefore = 0;
	
	CircleStemVolumeAfter = 0, CylinderStemVolumeAfter = 0,
	BezierStemVolumeAfter = 0;
	ProfileCurveStemVolumeAfter = 0, SliceSurfaceStemVolumeAfter = 0;

	BeforeRefineProfileCurvePoints->points.clear();
	AfterRefineProfileCurvePoints->points.clear();
	BeforeRefineBezierCurvePoints->points.clear();
	AfterRefineBezierCurvePoints->points.clear();
	VolumeThreads.clear();

	Viewer->removePointCloud(BeforeRefineProfileCurvePointsStr);
	Viewer->removePointCloud(AfterRefineProfileCurvePointsStr);
	Viewer->removePointCloud(BeforeRefineBezierCurvePointsStr);
	Viewer->removePointCloud(AfterRefineBezierCurvePointsStr);

	///*
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		VolumeThread SliceThread;
		SliceThread.SliceDone = false;
		SliceThread.BeforeProfileCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		SliceThread.AfterProfileCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		
		SliceThread.BeforeBezierCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		SliceThread.AfterBezierCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		
		//SliceThread.SliceVolume = 0;
		VolumeThreads.push_back(SliceThread);
	}

	#pragma omp parallel for
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		//cout<<"Slice Volume:"<<i<<endl;
		CalcStemByMethodsThread(i);
	}
	//int CurI = 0;
	////MaxThreadNum = 10;
	//ExecuteThreadNum = 0;

	//while (CurI < HorizontalPartition.SectionsCount)
	//{
	//	while (ExecuteThreadNum < MaxThreadNum)
	//	{
	//		if (CurI >= HorizontalPartition.SectionsCount)
	//		{
	//			break;
	//		}
	//		ExecuteThreadNum++;
	//		std::thread ThreadObj(std::bind(&CStemVolumeVerticalSlices::CalcStemByMethodsThread, this, CurI));			
	//		ThreadObj.join();
	//		CurI++;
	//	}
	//	Sleep(1000);
	//}

	//The height of the last slice may be less than SectionThickness	
	double ThickNessOfLastSlice = 0;
	double MaxZ = 0, MinZ = 0;
	int IndexOfLastSlice = HorizontalPartition.SectionsCount - 1;

	if (HorizontalPartition.SectionsVector[IndexOfLastSlice].Indexs.size() > 0)
	{
		MaxZ = MinZ = InputCloud->points[HorizontalPartition.SectionsVector[IndexOfLastSlice].Indexs[0]].z;

		for (int i = 1; i < HorizontalPartition.SectionsVector[IndexOfLastSlice].Indexs.size(); i++)
		{
			int TempIndex = HorizontalPartition.SectionsVector[IndexOfLastSlice].Indexs[i];
			double TempZ = InputCloud->points[HorizontalPartition.SectionsVector[IndexOfLastSlice].Indexs[i]].z;

			MaxZ = std::max(MaxZ, TempZ);
			MinZ = std::min(MinZ, TempZ);
		}
		ThickNessOfLastSlice = abs(MaxZ - MinZ);
	}

	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		if (i < IndexOfLastSlice)
		{
			CircleStemVolumeBefore = CircleStemVolumeBefore + VolumeThreads[i].SliceVolumeBeforeCircle * HorizontalPartition.SectionThickness;
			CylinderStemVolumeBefore = CylinderStemVolumeBefore + VolumeThreads[i].SliceVolumeBeforeCylinder * HorizontalPartition.SectionThickness;
			BezierStemVolumeBefore = BezierStemVolumeBefore + VolumeThreads[i].SliceVolumeBeforeBezier * HorizontalPartition.SectionThickness;
			ProfileCurveStemVolumeBefore = ProfileCurveStemVolumeBefore + VolumeThreads[i].SliceVolumeBeforeProfileCurve * HorizontalPartition.SectionThickness;

			CircleStemVolumeAfter = CircleStemVolumeAfter + VolumeThreads[i].SliceVolumeAfterCircle * HorizontalPartition.SectionThickness;
			CylinderStemVolumeAfter = CylinderStemVolumeAfter + VolumeThreads[i].SliceVolumeAfterCylinder * HorizontalPartition.SectionThickness;
			BezierStemVolumeAfter = BezierStemVolumeAfter + VolumeThreads[i].SliceVolumeAfterBezier * HorizontalPartition.SectionThickness;
			ProfileCurveStemVolumeAfter = ProfileCurveStemVolumeAfter + VolumeThreads[i].SliceVolumeAfterProfileCurve * HorizontalPartition.SectionThickness;
		}
		else if (i == IndexOfLastSlice)
		{
			CircleStemVolumeBefore = CircleStemVolumeBefore + VolumeThreads[i].SliceVolumeBeforeCircle * ThickNessOfLastSlice;
			CylinderStemVolumeBefore = CylinderStemVolumeBefore + VolumeThreads[i].SliceVolumeBeforeCylinder * ThickNessOfLastSlice;
			BezierStemVolumeBefore = BezierStemVolumeBefore + VolumeThreads[i].SliceVolumeBeforeBezier * ThickNessOfLastSlice;
			ProfileCurveStemVolumeBefore = ProfileCurveStemVolumeBefore + VolumeThreads[i].SliceVolumeBeforeProfileCurve * ThickNessOfLastSlice;

			CircleStemVolumeAfter = CircleStemVolumeAfter + VolumeThreads[i].SliceVolumeAfterCircle * ThickNessOfLastSlice;
			CylinderStemVolumeAfter = CylinderStemVolumeAfter + VolumeThreads[i].SliceVolumeAfterCylinder * ThickNessOfLastSlice;
			BezierStemVolumeAfter = BezierStemVolumeAfter + VolumeThreads[i].SliceVolumeAfterBezier * ThickNessOfLastSlice;
			ProfileCurveStemVolumeAfter = ProfileCurveStemVolumeAfter + VolumeThreads[i].SliceVolumeAfterProfileCurve * ThickNessOfLastSlice;
		}
		AfterRefineProfileCurvePoints->points.insert(
			AfterRefineProfileCurvePoints->points.end(),
			VolumeThreads[i].AfterProfileCurvePoints->points.begin(),
			VolumeThreads[i].AfterProfileCurvePoints->points.end());

		BeforeRefineProfileCurvePoints->points.insert(
			BeforeRefineProfileCurvePoints->points.end(),
			VolumeThreads[i].BeforeProfileCurvePoints->points.begin(),
			VolumeThreads[i].BeforeProfileCurvePoints->points.end());

		AfterRefineBezierCurvePoints->points.insert(
			AfterRefineBezierCurvePoints->points.end(),
			VolumeThreads[i].AfterBezierCurvePoints->points.begin(),
			VolumeThreads[i].AfterBezierCurvePoints->points.end());

		BeforeRefineBezierCurvePoints->points.insert(
			BeforeRefineBezierCurvePoints->points.end(),
			VolumeThreads[i].BeforeBezierCurvePoints->points.begin(),
			VolumeThreads[i].BeforeBezierCurvePoints->points.end());
	}
	
	
	CircleStemVolumeBefore = CircleStemVolumeBefore * M_PI;
	CylinderStemVolumeBefore = CylinderStemVolumeBefore * M_PI;
	BezierStemVolumeBefore = BezierStemVolumeBefore * M_PI;
	//ProfileCurveStemVolumeBefore = ProfileCurveStemVolumeBefore;
	
	StemVolumeVerticalSlices.lineEditVolumeCricleFittingBefore->setText(QString::number(CircleStemVolumeBefore, 10, 4));
	StemVolumeVerticalSlices.lineEditVolumeCylinderFittingBefore->setText(QString::number(CylinderStemVolumeBefore, 10, 4));
	StemVolumeVerticalSlices.lineEditVolumeBezierBefore->setText(QString::number(BezierStemVolumeBefore, 10, 4));
	StemVolumeVerticalSlices.lineEditVolumeProfileCurveBefore->setText(QString::number(ProfileCurveStemVolumeBefore, 10, 4));	
	
	CircleStemVolumeAfter = CircleStemVolumeAfter * M_PI;
	CylinderStemVolumeAfter = CylinderStemVolumeAfter * M_PI;
	BezierStemVolumeAfter = BezierStemVolumeAfter * M_PI;
	//ProfileCurveStemVolumeAfter = ProfileCurveStemVolumeAfter;

	StemVolumeVerticalSlices.lineEditVolumeCricleFittingAfter->setText(QString::number(CircleStemVolumeAfter, 10, 4));
	StemVolumeVerticalSlices.lineEditVolumeCylinderFittingAfter->setText(QString::number(CylinderStemVolumeAfter, 10, 4));
	StemVolumeVerticalSlices.lineEditVolumeBezierAfter->setText(QString::number(BezierStemVolumeAfter, 10, 4));
	StemVolumeVerticalSlices.lineEditVolumeProfileCurveAfter->setText(QString::number(ProfileCurveStemVolumeAfter, 10, 4));
	//*/

	//*
	emitUpdateStatusBar("Start Spline Surface Calculation", 0);	
	if (!IsBat)
	{
		double SplineSurfaceVolume = CalcVolumeByStemSplineSurface(
			StemVolumeVerticalSlices.doubleSpinBoxHeightStep->text().toDouble());

		StemVolumeVerticalSlices.lineEditVolumeSlicesSurfaceAfter->setText(
			QString::number(SplineSurfaceVolume, 10, 4));
	}
	//*/

	ShowProfileBefore(StemVolumeVerticalSlices.checkBoxShowProfileBefore->checkState());
	ShowProfileAfter(StemVolumeVerticalSlices.checkBoxShowProfileAfter->checkState());
	ShowBezierBefore(StemVolumeVerticalSlices.checkBoxShowBezierBefore->checkState());
	ShowBezierAfter(StemVolumeVerticalSlices.checkBoxShowBezierAfter->checkState());

	StemVolumeVerticalSlices.pushButtonCalcStemVolume->setEnabled(true);
	StemVolumeVerticalSlices.pushButtonHoleFill->setEnabled(true);
	emitUpdateStatusBar("The stem volume calculation has been completed!", 5000);
}


double CStemVolumeVerticalSlices::CalcVolumeByStemSplineSurface(double HStep, double VStep, double UStep)
{	
	//SurfaceInterpolation.SetInputStemPoints(InputCloud, 
	//	StemVolumeVerticalSlices.spinBoxThickNess->text().toDouble(), 10, 
	//	StemVolumeVerticalSlices.spinBoxAngleValue->text().toDouble(), ZMax, HStep, UStep, VStep);

	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> InterpolationPointsValue;	
	
	StemGridPointsRefine.GetRefinePointsForSurfaceInterpolation(InterpolationPointsValue);	
	//Procedure for the toppest slice
	///*
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSectionInterpolationPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	CurSectionInterpolationPoints->points.insert(CurSectionInterpolationPoints->points.begin(),
		InterpolationPointsValue[InterpolationPointsValue.size() - 1]->points.begin(), 
		InterpolationPointsValue[InterpolationPointsValue.size() - 1]->points.end());
	PointBase::SetPointsCoordinateValue(CurSectionInterpolationPoints, "Z", ZMax);
	InterpolationPointsValue.push_back(CurSectionInterpolationPoints);
	
	///*/
	//CSurfaceInterpolation SurfaceInterpolation;
	SurfaceInterpolation.SetInputInterpolationPoints(InterpolationPointsValue, HStep, UStep, VStep);
	SurfaceInterpolation.SetViewer(Viewer);
	//SurfaceInterpolation.ShowInterpolationPoints(Viewer);

	SurfaceInterpolation.p_TreePclQtGui = this->p_TreePclQtGui;
	SurfaceInterpolation.InterpolationSurface();	
	//SurfaceInterpolation.DrawSurfaceByDefinition(Viewer, 0.01, 0.001);
	

	if (StemVolumeVerticalSlices.checkBoxShowSurface->checkState() == 2)
	{
		ShowSplineSurfacePoints(2);
		//SurfaceInterpolation.GetSurfacePoints(AfterSplineSurfacePoints);		
	}


	//PointsMove(InputCloud, -60, 0, 0);
	//emitUpdateUI();

	//SurfaceInterpolation.ShowInterpolationPoints(Viewer);	
	return SurfaceInterpolation.CalcVolume(ZMin, ZMax);
	//*/
	//return 0;
}

void CStemVolumeVerticalSlices::CalcStemByMethodsThread(int SliceIndex)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BeforePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BeforeGridPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AfterPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BeforeBezierPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AfterBezierPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	//*
	{
		//lock_guard<mutex> lockGuard(g_lock);
		//p_this->g_lock.try_lock();		
		HorizontalPartition.GetSectionPoints(SliceIndex, BeforePoints);
		// The BeforePoint should be angle partitioned before using ProfileCurve.		
		StemGridPointsRefine.GetBeforeRefinedPointsBySliceIndex(BeforeGridPoints, SliceIndex);		
		//remove repreated points		
		StemGridPointsRefine.GetRefinedPointsBySliceIndex(AfterPoints, SliceIndex);
		//p_this->g_lock.unlock();
		
		/* Save each Slice
		string SaveFileNameSlicesPoints = OpenedFilePath;
		SaveFileNameSlicesPoints.insert(
			SaveFileNameSlicesPoints.length() - 4, "_" + StringBase::IntToStr(SliceIndex) + "_SlicesPoints");
		PointBase::SavePCDToFileName(BeforePoints, SaveFileNameSlicesPoints);

		string SaveFileNameBeforeGridPoints = OpenedFilePath;
		SaveFileNameBeforeGridPoints.insert(
			SaveFileNameBeforeGridPoints.length() - 4, "_" + StringBase::IntToStr(SliceIndex) + "_BeforeGridPoints");
		PointBase::SavePCDToFileName(BeforeGridPoints, SaveFileNameBeforeGridPoints);

		string SaveFileNameAfterPoints = OpenedFilePath;
		SaveFileNameAfterPoints.insert(
			SaveFileNameAfterPoints.length() - 4, "_" + StringBase::IntToStr(SliceIndex) + "_AfterPoints");
		PointBase::SavePCDToFileName(AfterPoints, SaveFileNameAfterPoints);
		//*/
	}
	//*/

	//Cricle Fitting
	pcl::PointXYZRGB DirectionPoint, CenterPoint;
	DirectionPoint.x = 0, DirectionPoint.y = 0, DirectionPoint.z = 1;

	CenterPoint = GeometryBase::GetCentroidOfPointsInSpaces(BeforePoints, DirectionPoint);

	CStemDiameter BeforeStemDiameter;
	BeforeStemDiameter.SetInputCloud(BeforePoints, CenterPoint, DirectionPoint);

	//if (SliceIndex == 2)
	//	PointBase::SavePCDToFileName(BeforePoints, "I:\\PCLQT\\TreePclQtGui\\TreePclQtGui\\pcdDemo\\Slices2.pcd");

	VolumeThreads[SliceIndex].SliceVolumeBeforeCircle
		= VolumeThreads[SliceIndex].SliceVolumeBeforeCircle +
		pow(BeforeStemDiameter.DRetrievalByCircleFittingIn2D(), 2) / 4.0;
	
	//Cylinder Fitting
	pcl::PointXYZRGB Direction;
	Direction.x = 0, Direction.y = 0, Direction.z = 1;
	VolumeThreads[SliceIndex].SliceVolumeBeforeCylinder
		= VolumeThreads[SliceIndex].SliceVolumeBeforeCylinder +
		pow(BeforeStemDiameter.DRetrievalByCylinerFittingIn3D(Direction), 2) / 4.0;

	//Bezier Fitting
	VolumeThreads[SliceIndex].SliceVolumeBeforeBezier
		= VolumeThreads[SliceIndex].SliceVolumeBeforeBezier +
		pow(BeforeStemDiameter.DRetrievalByBezierCurve(BeforeBezierPoints), 2) / 4.0;
	
	//each Bezier curve has 1000 points, rather than all points
	int k = BeforeBezierPoints->points.size() / 1000;
	for (int j = 0; j < BeforeBezierPoints->points.size(); j = j + k)
	{
		VolumeThreads[SliceIndex].BeforeBezierCurvePoints->points.
			push_back(BeforeBezierPoints->points[j]);
	}
	//VolumeThreads[SliceIndex].BeforeBezierCurvePoints

	//ProfileCurve Fitting
	CStemDiameterAndProfileCurveBasalArea BeforeProfileCurveBasalArea;
	VolumeThreads[SliceIndex].SliceVolumeBeforeProfileCurve
		= VolumeThreads[SliceIndex].SliceVolumeBeforeProfileCurve +
		BeforeProfileCurveBasalArea.GetProfileCurveAndArea(BeforeGridPoints);
	
	//each profile curve has 1000 points, rather than all points
	k = BeforeProfileCurveBasalArea.Spline.CurvePoints->points.size() / 1000;
	for (int j = 0; j < BeforeProfileCurveBasalArea.Spline.CurvePoints->points.size(); j = j + k)
	{
		VolumeThreads[SliceIndex].BeforeProfileCurvePoints->points.
			push_back(BeforeProfileCurveBasalArea.Spline.CurvePoints->points[j]);
	}	

	//surface method need supplemented
	//There are interpolation points from each slice for Surface method, 2019.06.03

	//CSurfaceInterpolation SurfaceInterpolation;
	//SurfaceInterpolation.SetInputPoints(VerticalSlicesInterpolationPoints, 0.001, 0.05);
	//SurfaceInterpolation.InterpolationSurface();
	//SurfaceInterpolation.DrawSurfaceByDefinition(Viewer);
	//return SurfaceInterpolation.CalcVolume();
	   
	//******above is before refine result and below is after refine results. 

	CStemDiameter AfterStemDiameter;
	CenterPoint = GeometryBase::GetCentroidOfPointsInSpaces(BeforePoints, DirectionPoint);
	AfterStemDiameter.SetInputCloud(AfterPoints, CenterPoint, DirectionPoint);
	
	//Circle Fitting
	VolumeThreads[SliceIndex].SliceVolumeAfterCircle
		= VolumeThreads[SliceIndex].SliceVolumeAfterCircle +
		pow(AfterStemDiameter.DRetrievalByCircleFittingIn2D(), 2) / 4.0;

	//Cylinder Fitting
	VolumeThreads[SliceIndex].SliceVolumeAfterCylinder
		= VolumeThreads[SliceIndex].SliceVolumeAfterCylinder +
		pow(AfterStemDiameter.DRetrievalByCylinerFittingIn3D(Direction), 2) / 4.0;

	//Bezier Fitting
	VolumeThreads[SliceIndex].SliceVolumeAfterBezier
		= VolumeThreads[SliceIndex].SliceVolumeAfterBezier +
		pow(AfterStemDiameter.DRetrievalByBezierCurve(AfterBezierPoints), 2) / 4.0;
	k = AfterBezierPoints->points.size() / 1000;
	for (int j = 0; j < AfterBezierPoints->points.size(); j = j + k)
	{
		VolumeThreads[SliceIndex].AfterBezierCurvePoints->points.
			push_back(AfterBezierPoints->points[j]);
	}

	//Profile Fitting
	CStemDiameterAndProfileCurveBasalArea AfterProfileCurveBasalArea;
	VolumeThreads[SliceIndex].SliceVolumeAfterProfileCurve
		= VolumeThreads[SliceIndex].SliceVolumeAfterProfileCurve +
		AfterProfileCurveBasalArea.GetProfileCurveAndArea(AfterPoints);

	//if (i > HorizontalPartition.SectionsCount - 4)
	//each profile curve has 1000 points, rather than all points
	k = AfterProfileCurveBasalArea.Spline.CurvePoints->points.size() / 1000;
	for (int j = 0; j < AfterProfileCurveBasalArea.Spline.CurvePoints->points.size(); j = j + k)
	{
		VolumeThreads[SliceIndex].AfterProfileCurvePoints->points.
			push_back(AfterProfileCurveBasalArea.Spline.CurvePoints->points[j]);
	}
	//if (SliceIndex % 3 == 0)
	{
		PointBase::SetPointColor(VolumeThreads[SliceIndex].AfterProfileCurvePoints, ColorBase::BlueColor);
		PointBase::SetPointColor(VolumeThreads[SliceIndex].BeforeProfileCurvePoints, ColorBase::RedColor);
		PointBase::SetPointColor(VolumeThreads[SliceIndex].AfterBezierCurvePoints, ColorBase::BlueColor);
		PointBase::SetPointColor(VolumeThreads[SliceIndex].BeforeBezierCurvePoints, ColorBase::RedColor);
	}
/*	else if (SliceIndex % 3 == 1)
	{
		PointBase::SetPointColor(VolumeThreads[SliceIndex].AfterProfileCurvePoints, ColorBase::GreenColor);
		PointBase::SetPointColor(VolumeThreads[SliceIndex].BeforeProfileCurvePoints, ColorBase::GreenColor);
		PointBase::SetPointColor(VolumeThreads[SliceIndex].AfterBezierCurvePoints, ColorBase::GreenColor);
		PointBase::SetPointColor(VolumeThreads[SliceIndex].BeforeBezierCurvePoints, ColorBase::GreenColor);
	}
	else if (SliceIndex % 3 == 2)
	{
		PointBase::SetPointColor(VolumeThreads[SliceIndex].AfterProfileCurvePoints, ColorBase::BlueColor);
		PointBase::SetPointColor(VolumeThreads[SliceIndex].BeforeProfileCurvePoints, ColorBase::BlueColor);
		PointBase::SetPointColor(VolumeThreads[SliceIndex].AfterBezierCurvePoints, ColorBase::BlueColor);
		PointBase::SetPointColor(VolumeThreads[SliceIndex].BeforeBezierCurvePoints, ColorBase::BlueColor);
	}*/				
	//ExecuteThreadNum--;
}

void CStemVolumeVerticalSlices::SaveToFile(string ProcFileName)
{
	//string SaveFileName;	
	//SaveFileName = OpenedFilePath.substr(0, OpenedFilePath.length() - 3) + "txt";//Need Modified
	string ResultStr;
	ResultStr = GetDateTime() + "	" + ProcFileName;	
	ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.spinBoxThickNess->text().toStdString();
	ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.spinBoxAngleValue->text().toStdString();

	ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditStemHeight->text().toStdString();	
	ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditAverageDiameter->text().toStdString();

	ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditDiameterErrorCircleFitting->text().toStdString();
	ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditDiameterErrorBezier->text().toStdString();

	ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditVolumeCricleFittingBefore->text().toStdString();
	//ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditVolumeCricleFittingAfter->text().toStdString();
	ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditVolumeCylinderFittingBefore->text().toStdString();
	//ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditVolumeCylinderFittingAfter->text().toStdString();
	ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditVolumeBezierBefore->text().toStdString();
	//ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditVolumeBezierAfter->text().toStdString();
	//ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditVolumeProfileCurveBefore->text().toStdString();
	ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditVolumeProfileCurveAfter->text().toStdString();	

	//ResultStr = ResultStr + "	" + StemVolumeVerticalSlices.lineEditVolumeSlicesSurfaceAfter->text().toStdString();

	ResultStr = ResultStr + "	" + (QString::number(CalcVolumeByStemSplineSurface(2), 10, 4)).toStdString();

	ResultStr = ResultStr + "	" + (QString::number(CalcVolumeByStemSplineSurface(1), 10, 4)).toStdString();
	//
	ResultStr = ResultStr + "	" + (QString::number(CalcVolumeByStemSplineSurface(0.5), 10, 4)).toStdString();

	SaveStringToFile(ResultFileName, ResultStr);
}

void CStemVolumeVerticalSlices::Bat()
{
	IsBat = true;
	char Drive[_MAX_DRIVE];
	char FilePath[_MAX_DIR];
	char Fname[_MAX_FNAME];
	char Ext[_MAX_EXT];
	_splitpath(OpenedFilePath.c_str(), Drive, FilePath, Fname, Ext);
		
	ResultFileName = string(Drive) + string(FilePath) + "Result.txt";
	string FilePathStr;
	FilePathStr = string(Drive) + string(FilePath);
	vector<string> BatFiles;	
	GetFiles(FilePath, BatFiles);

	for each (string FileNameStr in BatFiles)
	{
		_splitpath(FileNameStr.c_str(), Drive, FilePath, Fname, Ext);
		if ((strcmp(Ext, ".pcd") == 0) || (strcmp(Ext, ".vtx") == 0) || (strcmp(Ext, ".las") == 0))
		{			
			OpenedFilePath = FilePathStr + FileNameStr;			

			InputCloud->points.clear();
			PointBase::OpenPCLFile(OpenedFilePath, InputCloud);
			CTreeBase::SetInputCloud(CTreeBase::InputCloud);

			RefreshData();
			emitUpdateAppTitle(OpenedFilePath);
			emitUpdateUI();							

			//20190304 Too little to hard
			if (ZMax - ZMin < 30) continue;

			//StemVolumeVerticalSlices.spinBoxThickNess->setValue(10);
			//StemVolumeVerticalSlices.spinBoxAngleValue->setValue(5);
			//IsRepaired = false;
			//CalcStemByMethods();
			//SaveToFile(FileNameStr);
			//emitUpdateStatusBar("10, 5 has been done",0);

			/////*
			//StemVolumeVerticalSlices.spinBoxThickNess->setValue(10);
			//StemVolumeVerticalSlices.spinBoxAngleValue->setValue(2);
			//IsRepaired = false;
			//CalcStemByMethods();
			//SaveToFile(FileNameStr);
			//emitUpdateStatusBar("10, 2 has been done", 0);

			//StemVolumeVerticalSlices.spinBoxThickNess->setValue(10);
			//StemVolumeVerticalSlices.spinBoxAngleValue->setValue(1);
			//IsRepaired = false;
			//CalcStemByMethods();
			//SaveToFile(FileNameStr);
			//emitUpdateStatusBar("10, 1 has been done", 0);

			//StemVolumeVerticalSlices.spinBoxThickNess->setValue(5);
			//StemVolumeVerticalSlices.spinBoxAngleValue->setValue(5);
			//IsRepaired = false;
			//CalcStemByMethods();
			//SaveToFile(FileNameStr);
			//emitUpdateStatusBar("5, 5 has been done", 0);

			StemVolumeVerticalSlices.spinBoxThickNess->setValue(5);
			StemVolumeVerticalSlices.spinBoxAngleValue->setValue(2);
			IsRepaired = false;
			CalcStemByMethods();
			SaveToFile(FileNameStr);
			emitUpdateStatusBar("5, 2 has been done", 0);

			//StemVolumeVerticalSlices.spinBoxThickNess->setValue(5);
			//StemVolumeVerticalSlices.spinBoxAngleValue->setValue(1);
			//IsRepaired = false;
			//CalcStemByMethods();
			//SaveToFile(FileNameStr);
			//emitUpdateStatusBar("5, 1 has been done", 0);

			StemVolumeVerticalSlices.spinBoxThickNess->setValue(2);
			StemVolumeVerticalSlices.spinBoxAngleValue->setValue(5);
			IsRepaired = false;
			CalcStemByMethods();
			SaveToFile(FileNameStr);
			emitUpdateStatusBar("2, 5 has been done", 0);

			StemVolumeVerticalSlices.spinBoxThickNess->setValue(2);
			StemVolumeVerticalSlices.spinBoxAngleValue->setValue(2);
			IsRepaired = false;
			CalcStemByMethods();
			SaveToFile(FileNameStr);
			emitUpdateStatusBar("2, 2 has been done", 0);

/*			StemVolumeVerticalSlices.spinBoxThickNess->setValue(2);
			StemVolumeVerticalSlices.spinBoxAngleValue->setValue(1);
			IsRepaired = false;
			CalcStemByMethods();
			SaveToFile(FileNameStr);
			emitUpdateStatusBar("2, 1 has been done", 0);*/		

			//emitUpdateUI();
			//*/	
		}
	}
	IsBat = false;
}