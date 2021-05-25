#include "StemDiameterAndBasalRetrieval.h"

void CStemDiameterAndBasalRetrieval::RefreshData()
{
	CTreeBase::RefreshData();

	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetThickNess(
		StemDiameterAndBasalRetrievalForm.doubleSpinBoxThickNess->text().toDouble());
	HorizontalPartition.PatitionSection();

	ShowSlicesPoints(StemDiameterAndBasalRetrievalForm.checkBoxVerticalSlices->checkState());
	
	StemDiameterAndBasalRetrievalForm.lineEditStemAxisCurveHeight->setText("");
	StemDiameterAndBasalRetrievalForm.lineEditStemAxisCurveLength->setText("");
	ResetForm();
	StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->setValue(130);
	StemDiameterAndBasalRetrievalForm.spinBoxStemLength->setValue(130);	
}

void CStemDiameterAndBasalRetrieval::ResetForm()
{
	StemDiameterAndBasalRetrievalForm.lineEditBasalArea->setText("");
	StemDiameterAndBasalRetrievalForm.lineEditCurvature->setText("");
	StemDiameterAndBasalRetrievalForm.lineEditDBezier->setText("");
	StemDiameterAndBasalRetrievalForm.lineEditDCircleFitting->setText("");
	StemDiameterAndBasalRetrievalForm.lineEditDConvexHullLine->setText("");
	StemDiameterAndBasalRetrievalForm.lineEditDCylinderFitting->setText("");
	StemDiameterAndBasalRetrievalForm.lineEditDSpline->setText("");
	StemDiameterAndBasalRetrievalForm.lineEditInclination->setText("");
	StemDiameterAndBasalRetrievalForm.lineEditTorsion->setText("");
	StemDiameterAndBasalRetrievalForm.pushButtonSaveCalcingPoints->setEnabled(false);
	StemDiameterAndBasalRetrievalForm.pushButtonStemParameterRetrieval->setEnabled(true);
}

void CStemDiameterAndBasalRetrieval::ShowSlicesPoints(double thickNess)
{
	ShowSlicesPoints(StemDiameterAndBasalRetrievalForm.checkBoxVerticalSlices->checkState());
}

void CStemDiameterAndBasalRetrieval::ShowSlicesPoints(int CheckValue)
{
	if (InputCloud == NULL) return;

	QWidget *widget = (QWidget *)sender();

	if (widget == NULL) return;

	//if (widget->objectName() == "checkBoxVerticalSlices" && 
	//	StemDiameterAndBasalRetrievalForm.checkBoxVerticalSlices->checkState() == 2)
	if (StemDiameterAndBasalRetrievalForm.checkBoxVerticalSlices->checkState() == 2)
	{
		HorizontalPartition.SetViewer(Viewer);
		HorizontalPartition.p_TreePclQtGui = p_TreePclQtGui;
		HorizontalPartition.SetInputCloud(InputCloud);
		HorizontalPartition.SetThickNess(
			StemDiameterAndBasalRetrievalForm.doubleSpinBoxThickNess->text().toDouble());
		HorizontalPartition.PatitionSection();
		HorizontalPartition.ShowSectionPoints(0, HorizontalPartition.SectionsCount);
	}
	//else if (widget->objectName() == "checkBoxVerticalSlices"
	//	&& StemDiameterAndBasalRetrievalForm.checkBoxVerticalSlices->checkState() != 2)
	else if (StemDiameterAndBasalRetrievalForm.checkBoxVerticalSlices->checkState() != 2)
	{
		HorizontalPartition.UnShowSectionPoints();
	}
}

void CStemDiameterAndBasalRetrieval::ShowStemAxisCurvePoints(int CheckValue)
{
	if (StemSkeleton.StemSkeletonSpline.CurvePoints->points.size() == 0) return;

	if (StemDiameterAndBasalRetrievalForm.checkBoxShowAxisCurve->checkState() == 2)
	{
		PointBase::SetPointColor(StemSkeleton.StemSkeletonSpline.CurvePoints, ColorBase::RedColor);
		ShowPoints(StemSkeleton.StemSkeletonSpline.CurvePoints, StemAxisCurvePointsStr, PointSize + 1);
	}
	else
	{
		Viewer->removePointCloud(StemAxisCurvePointsStr);
		emitUpdateUI();
	}
}

//2021.02.02 Bat 方式获取Slice
void CStemDiameterAndBasalRetrieval::BatGetSlice()
{
	char Drive[_MAX_DRIVE];
	char FilePath[_MAX_DIR];
	char Fname[_MAX_FNAME];
	char Ext[_MAX_EXT];
	_splitpath(OpenedFilePath.c_str(), Drive, FilePath, Fname, Ext);
	IsBat = true;
	string FilePathStr;
	FilePathStr = string(Drive) + string(FilePath);
	string TxtFileName = FilePathStr + "DBH\\DBHHeight.txt";
		
	vector<string> Strs;
	ReadFileToStrings(TxtFileName, Strs);

	for (int i = 0; i < Strs.size(); i++)
	{
		int FindIndex = Strs[i].find(" ");
		OpenedFilePath = FilePathStr + Strs[i].substr(0, FindIndex) + ".pcd";
		string RightStr = Strs[i].substr(FindIndex + 1, Strs[i].length() - FindIndex);
		
		FindIndex = RightStr.find(" ");
		string HValue = RightStr.substr(0, FindIndex);				
		int HeightStep = -30;

		PointBase::OpenPCLFile(OpenedFilePath, CTreeBase::InputCloud, true);
		CTreeBase::SetInputCloud(CTreeBase::InputCloud);

		RefreshData();
		StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->setValue(
			QString::fromStdString(HValue).toDouble() + HeightStep);
		emitUpdateAppTitle(OpenedFilePath);
		emitUpdateUI();			

		cout << "OpenedFilePath:" << OpenedFilePath << endl;
		cout << "HValue:" << StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->text().toStdString() << endl;

		StemAxisCurveConstruct();
		StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->setValue(
			QString::fromStdString(HValue).toDouble() + HeightStep);
		ChangeCalcPosition();
		SaveCalcingStemPoints();
	}
	IsBat = false;
}

CStemDiameterAndBasalRetrieval::CStemDiameterAndBasalRetrieval()
{

}

CStemDiameterAndBasalRetrieval::CStemDiameterAndBasalRetrieval(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	StemDiameterAndBasalRetrievalForm.setupUi(widget);
	IsBat = false;
	connect(StemDiameterAndBasalRetrievalForm.checkBoxVerticalSlices, 
		SIGNAL(stateChanged(int)), this, SLOT(ShowSlicesPoints(int)));
	connect(StemDiameterAndBasalRetrievalForm.doubleSpinBoxThickNess,
		SIGNAL(valueChanged(double)), this, SLOT(ShowSlicesPoints(double)));
	connect(StemDiameterAndBasalRetrievalForm.checkBoxShowAxisCurve, 
		SIGNAL(stateChanged(int)), this, SLOT(ShowStemAxisCurvePoints(int)));
	connect(StemDiameterAndBasalRetrievalForm.checkBoxShowStemCrossSection, 
		SIGNAL(stateChanged(int)), this, SLOT(ShowCrossSectionalPoints(int)));
	connect(StemDiameterAndBasalRetrievalForm.checkBoxShowCalcingStemPoints, 
		SIGNAL(stateChanged(int)), this, SLOT(ShowCalcingStemPoints(int)));
	connect(StemDiameterAndBasalRetrievalForm.checkBoxShowBezierFitting,
		SIGNAL(stateChanged(int)), this, SLOT(ShowBezierFittingPoints(int)));
	connect(StemDiameterAndBasalRetrievalForm.checkBoxShowSplineFitting,
		SIGNAL(stateChanged(int)), this, SLOT(ShowSplineFittingPoints(int)));
	connect(StemDiameterAndBasalRetrievalForm.checkBoxShowPofileCurve,
		SIGNAL(stateChanged(int)), this, SLOT(ShowProfileCurvePoints(int)));

	connect(StemDiameterAndBasalRetrievalForm.pushButtonSaveCalcingPoints,
		SIGNAL(clicked()), this, SLOT(SaveCalcingStemPoints()));

	connect(StemDiameterAndBasalRetrievalForm.pushButtonStemAxisCurveConstruction,
		SIGNAL(clicked()), this, SLOT(StemAxisCurveConstruct()));

	connect(StemDiameterAndBasalRetrievalForm.pushButtonStemParameterRetrieval,
		SIGNAL(clicked()), this, SLOT(ChangeCalcPosition()));

	connect(StemDiameterAndBasalRetrievalForm.radioButtonStemHeight, 
		SIGNAL(clicked(bool)), this, SLOT(RadioPositionCheck(bool)));
	connect(StemDiameterAndBasalRetrievalForm.radioButtonStemLength, 
		SIGNAL(clicked(bool)), this, SLOT(RadioPositionCheck(bool)));

	connect(StemDiameterAndBasalRetrievalForm.spinBoxAngleForBasalArea,
		SIGNAL(valueChanged(int)), this, SLOT(CalcingBasalArea()));	

	connect(StemDiameterAndBasalRetrievalForm.spinBoxStemHeight, SIGNAL(valueChanged(double)),
		this, SLOT(ResetForm()));
	connect(StemDiameterAndBasalRetrievalForm.spinBoxStemLength, SIGNAL(valueChanged(double)),
		this, SLOT(ResetForm()));
	
	connect(StemDiameterAndBasalRetrievalForm.pushButtonBat,
		SIGNAL(clicked()), this, SLOT(BatGetSlice()));

	StemDiameterAndBasalRetrievalForm.groupBox_Parameter->setEnabled(false);
	StemDiameterAndBasalRetrievalForm.groupBox_ParameterRetrieval->setEnabled(false);
	StemDiameterAndBasalRetrievalForm.groupBox_BasalAreaRetrieval->setEnabled(false);
	
	//20190408 for Paper Use
	StemDiameterAndBasalRetrievalForm.radioButtonDiameterConvexHull->setVisible(false);
	StemDiameterAndBasalRetrievalForm.lineEditDConvexHullLine->setVisible(false);	

	StemDiameterAndBasalRetrievalForm.pushButtonBasalAreaComputation->setVisible(false);

	widget->show();

	CalcingStemPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	CopyCalcingStemPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	BezierFittingPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	SplineFittingPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	ProfileCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

CStemDiameterAndBasalRetrieval::~CStemDiameterAndBasalRetrieval()
{
	CalcingStemPoints->points.clear();
	CopyCalcingStemPoints->points.clear();
	BezierFittingPoints->points.clear();
	SplineFittingPoints->points.clear();
	ProfileCurvePoints->points.clear();

	Viewer->removePointCloud(SlicesPointsStr);
	Viewer->removePointCloud(StemAxisCurvePointsStr);
	Viewer->removePointCloud(StemCrossSectionPointsStr);
	Viewer->removePointCloud(CalcingStemPointsStr);

	Viewer->removePointCloud(SplineFittingPointsStr);
	Viewer->removePointCloud(BezierFittingPointsStr);
	Viewer->removePointCloud(ProfileCurvePointsStr);

	emitUpdateUI();
}

void CStemDiameterAndBasalRetrieval::StemAxisCurveConstruct()
{	
	StemDiameterAndBasalRetrievalForm.pushButtonStemAxisCurveConstruction->setEnabled(false);
	StemDiameterAndBasalRetrievalForm.groupBox_Parameter->setEnabled(false);
	StemDiameterAndBasalRetrievalForm.groupBox_ParameterRetrieval->setEnabled(false);
	StemDiameterAndBasalRetrievalForm.groupBox_BasalAreaRetrieval->setEnabled(false);

	CalcingStemPoints->points.clear();
	SplineFittingPoints->points.clear();
	BezierFittingPoints->points.clear();
	ProfileCurvePoints->points.clear();

	emitUpdateStatusBar("Stem axis curve is constructing, please wait!");
	//StemSkeleton;
	StemSkeleton.SetInputs(InputCloud,
		StemDiameterAndBasalRetrievalForm.doubleSpinBoxThickNess->text().toDouble(),
		StemDiameterAndBasalRetrievalForm.checkBoxSmoothing->checkState() == 2, 
		StemDiameterAndBasalRetrievalForm.spinBoxNumbersforStemSegment->text().toInt(),
		abs(ZMax-ZMin) / StemDiameterAndBasalRetrievalForm.doubleSpinBoxThickNess->text().toDouble() / 3.0,
		StemDiameterAndBasalRetrievalForm.doubleSpinBoxIncludeAnlgeValue->text().toDouble(),
		StemDiameterAndBasalRetrievalForm.doubleSpinBoxMaxHeight->text().toDouble());
	
	//emitUpdateUI();
	
	if (!StemSkeleton.IsSuitForConstruct())
	{
		string Str = "There are to many points in each slices, that will take too long time to calculate. Are you sure?";		
		int ID = QMessageBox::information(NULL, tr("Information"),
				tr(Str.c_str()), QMessageBox::Yes | QMessageBox::No);

		if (ID == QMessageBox::No)
		{
			StemDiameterAndBasalRetrievalForm.pushButtonStemAxisCurveConstruction->setEnabled(true);
			StemDiameterAndBasalRetrievalForm.groupBox_Parameter->setEnabled(true);
			StemDiameterAndBasalRetrievalForm.groupBox_ParameterRetrieval->setEnabled(true);
			StemDiameterAndBasalRetrievalForm.groupBox_BasalAreaRetrieval->setEnabled(true);
			emitUpdateStatusBar("Stem axis curve has been terminated! Please set the thickness value and try again!", 5000);
			return;
		}
	}

	StemSkeleton.ConstructStemSplineCurve(
		StemDiameterAndBasalRetrievalForm.doubleSpinBoxStartHeight->text().toDouble());

	//if (StemSkeleton.ResultStr != "Done")
	//	QMessageBox::information(NULL, tr("Information"),
	//		tr(StemSkeleton.ResultStr.c_str()));

	StemHeight = StemSkeleton.StemSkeletonSpline.GetSplineHeight();
	StemLength = StemSkeleton.StemSkeletonSpline.GetSplineLengthBySimpson();

	//PointsMove(StemSkeleton.StemSkeletonSpline.CurvePoints, 50, 0, 0);

	if (StemDiameterAndBasalRetrievalForm.checkBoxShowAxisCurve->checkState() == 2)
	{
		PointBase::SetPointColor(StemSkeleton.StemSkeletonSpline.CurvePoints, ColorBase::RedColor);
		ShowPoints(StemSkeleton.StemSkeletonSpline.CurvePoints, StemAxisCurvePointsStr, PointSize);
	}

	StemDiameterAndBasalRetrievalForm.lineEditStemAxisCurveLength->setText(
		QString::number(StemLength, 10, 4));
	StemDiameterAndBasalRetrievalForm.lineEditStemAxisCurveHeight->setText(
		QString::number(StemHeight, 10, 4));

	if (StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->text().toDouble() >= StemHeight)
		StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->setValue(StemHeight / 2.0);
	if (StemDiameterAndBasalRetrievalForm.spinBoxStemLength->text().toDouble() >= StemLength)
		StemDiameterAndBasalRetrievalForm.spinBoxStemLength->setValue(StemLength / 2.0);

	StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->setMaximum(StemHeight);
	StemDiameterAndBasalRetrievalForm.spinBoxStemLength->setMaximum(StemLength);

	ChangeCalcPosition();	

	StemDiameterAndBasalRetrievalForm.groupBox_Parameter->setEnabled(true);
	StemDiameterAndBasalRetrievalForm.groupBox_ParameterRetrieval->setEnabled(true);
	StemDiameterAndBasalRetrievalForm.groupBox_BasalAreaRetrieval->setEnabled(true);
	
	StemDiameterAndBasalRetrievalForm.pushButtonStemAxisCurveConstruction->setEnabled(true);
	emitUpdateStatusBar("Stem axis curve has been constructed!", 5000);
}

//U Value is determined by stem height or length. 2019.01.23 
void CStemDiameterAndBasalRetrieval::GetPointUValue()
{
	if (StemDiameterAndBasalRetrievalForm.radioButtonStemLength->isChecked())
	{
		if (StemDiameterAndBasalRetrievalForm.spinBoxStemLength->text().toDouble() > 
			StemDiameterAndBasalRetrievalForm.lineEditStemAxisCurveLength->text().toDouble())
		{
			StemDiameterAndBasalRetrievalForm.spinBoxStemLength->setValue(
				StemDiameterAndBasalRetrievalForm.lineEditStemAxisCurveLength->text().toDouble() / 2.0);
		}
		if (StemDiameterAndBasalRetrievalForm.spinBoxStemLength->text().toDouble() < StemLength)
			CurrentPointU = StemSkeleton.StemSkeletonSpline.GetUValueBySplineLength(
				StemDiameterAndBasalRetrievalForm.spinBoxStemLength->text().toDouble());
		else
			QMessageBox::information(NULL, tr("Information"), 
				"The chose stem length should be smaller than stem length, please reduce it!");
	}

	if (StemDiameterAndBasalRetrievalForm.radioButtonStemHeight->isChecked())
	{
		if (StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->text().toDouble() >
			StemDiameterAndBasalRetrievalForm.lineEditStemAxisCurveHeight->text().toDouble())
		{
			StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->setValue(
				StemDiameterAndBasalRetrievalForm.lineEditStemAxisCurveHeight->text().toDouble() / 2.0);
		}
		if (StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->text().toDouble() < StemHeight)
			CurrentPointU = StemSkeleton.StemSkeletonSpline.GetUValueBySplineHeight(
				StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->text().toDouble());
		else
			QMessageBox::information(NULL, tr("Information"), 
				"The chose stem height should be smaller than stem height, please reduce it!");
	}
	//QMessageBox::information(NULL, tr("Information"), QString::number(CurrentPointU));
}

void CStemDiameterAndBasalRetrieval::ShowCrossSectionalPoints(int CheckValue)
{	
	Viewer->removePointCloud(StemCrossSectionPointsStr);	

	if (StemDiameterAndBasalRetrievalForm.checkBoxShowStemCrossSection->checkState() != 2)
	{
		emitUpdateUI();
		return;
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OriginalStemCrossSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemCrossSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	GeometryBase::GetCirclePoints(OriginalStemCrossSectionPoints, CurrentPoint,
		StemDiameterAndBasalRetrievalForm.lineEditDCircleFitting->text().toDouble() * 1.5 / 2.0, true);
	OriginalStemCrossSectionPoints->points.insert(OriginalStemCrossSectionPoints->points.begin(), CurrentPoint);

	//PointBase::SetPointColor(OriginalStemCrossSectionPoints, ColorBase::BlueColor);
	////StemCrossSectionPoints->points.erase(StemCrossSectionPoints->points.begin());
	//ShowPoints(OriginalStemCrossSectionPoints, StemCrossSectionPointsStr, PointSize * 2);
	//QMessageBox::information(NULL, tr((QString::number(OriginalStemCrossSectionPoints->points.size())).toStdString().c_str()),
	//	tr((QString::number(StemDiameterAndBasalRetrievalForm.lineEditDCircleFitting->text().toDouble() * 1.5 / 2.0)).toStdString().c_str()));

	GeometryBase::RotateToOriginal(OriginalStemCrossSectionPoints, StemCrossSectionPoints, CurrentNormal);
	
	PointsMove(StemCrossSectionPoints, 
		CurrentPoint.x - StemCrossSectionPoints->points[0].x, 
		CurrentPoint.y - StemCrossSectionPoints->points[0].y, 
		CurrentPoint.z - StemCrossSectionPoints->points[0].z);
	PointBase::SetPointColor(StemCrossSectionPoints, ColorBase::BlueColor);
	StemCrossSectionPoints->points.erase(StemCrossSectionPoints->points.begin());
	ShowPoints(StemCrossSectionPoints, StemCrossSectionPointsStr, PointSize);	
}

void CStemDiameterAndBasalRetrieval::ShowCalcingStemPoints(int CheckValue)
{
	Viewer->removePointCloud(CalcingStemPointsStr);
	
	if (StemDiameterAndBasalRetrievalForm.checkBoxShowCalcingStemPoints->checkState() != 2)
	{
		emitUpdateUI();
		return;
	}

	ShowPoints(CalcingStemPoints, CalcingStemPointsStr, PointSize);	
}

void CStemDiameterAndBasalRetrieval::ShowSplineFittingPoints(int CheckValue)
{
	Viewer->removePointCloud(SplineFittingPointsStr);

	if (StemDiameterAndBasalRetrievalForm.checkBoxShowSplineFitting->checkState() != 2)
	{
		emitUpdateUI();
		return;
	}

	ShowPoints(SplineFittingPoints, SplineFittingPointsStr, PointSize);
}

void CStemDiameterAndBasalRetrieval::ShowProfileCurvePoints(int CheckValue)
{
	Viewer->removePointCloud(ProfileCurvePointsStr);

	if (StemDiameterAndBasalRetrievalForm.checkBoxShowPofileCurve->checkState() != 2)
	{
		emitUpdateUI();
		return;
	}

	ShowPoints(ProfileCurvePoints, ProfileCurvePointsStr, PointSize);
}

void CStemDiameterAndBasalRetrieval::ShowBezierFittingPoints(int CheckValue)
{
	Viewer->removePointCloud(BezierFittingPointsStr);

	if (StemDiameterAndBasalRetrievalForm.checkBoxShowBezierFitting->checkState() != 2)
	{
		emitUpdateUI();
		return;
	}

	ShowPoints(BezierFittingPoints, BezierFittingPointsStr, PointSize);
}

void CStemDiameterAndBasalRetrieval::SaveCalcingStemPoints()
{
	if (CalcingStemPoints->points.size() == 0) return;
	
	//QString FileName = QFileDialog::getSaveFileName(NULL,
	//	tr("Point Cloud Files"), "3DStemModelForMeasure", tr("Point Cloud Files (*.pcd)"));
	
	char Drive[_MAX_DRIVE];
	char FilePath[_MAX_DIR];
	char Fname[_MAX_FNAME];
	char Ext[_MAX_EXT];
	_splitpath(OpenedFilePath.c_str(), Drive, FilePath, Fname, Ext);
	OpenedFileName = Fname;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RotatedCalcingStemPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	GeometryBase::RotateNormalToVertical(CopyCalcingStemPoints, RotatedCalcingStemPoints, CurrentNormal);

	string FilePathStr;
	FilePathStr = string(Drive) + string(FilePath);

	string TempFileName = FilePathStr + "DBH\\" + OpenedFileName + ".pcd";
	cout << "TempFileName:" << TempFileName << endl;
	PointBase::SavePCDToFileName(RotatedCalcingStemPoints, TempFileName);

	string TempDBHHeight = OpenedFileName + " " 
		+ StemDiameterAndBasalRetrievalForm.spinBoxStemHeight->text().toStdString()
		+ " " + StringBase::DateStr();	
	if (!IsBat)
		SaveStringToFile(FilePathStr + "DBH\\DBHHeight.txt", TempDBHHeight);
	
	//if (FileName.length() > 0)
	//{
	//	emitUpdateStatusBar("File is being saving, please wait!", 0);
	//	PointBase::SavePCDToFileName(RotatedCalcingStemPoints, FileName.toStdString());
	//	emitUpdateStatusBar("File has been saved!", 3000);
	//}
}

void CStemDiameterAndBasalRetrieval::CalcingStemParameters()
{
	CurrentPoint = StemSkeleton.StemSkeletonSpline.GetSplinePoint(CurrentPointU);

	CurrentNormal = StemSkeleton.StemSkeletonSpline.GetSplineDerivativePoint(CurrentPointU, 1, true);
	
	StemDiameterAndBasalRetrievalForm.lineEditCurvature->setText(
		QString::number(StemSkeleton.StemSkeletonSpline.GetCurvature(CurrentPointU), 10, 4));
	StemDiameterAndBasalRetrievalForm.lineEditTorsion->setText(
		QString::number(StemSkeleton.StemSkeletonSpline.GetTorsion(CurrentPointU), 10, 4));
	StemDiameterAndBasalRetrievalForm.lineEditInclination->setText(
		QString::number(StemSkeleton.StemSkeletonSpline.GetInclination(CurrentPointU), 10, 4));

	pcl::PointXYZRGB TempUpPoint;
	if (CurrentPointU + EPSM3 < 1)
		TempUpPoint = StemSkeleton.StemSkeletonSpline.GetSplinePoint(CurrentPointU + EPSM3);
	else 
		TempUpPoint = StemSkeleton.StemSkeletonSpline.GetSplinePoint(1 - EPSM6*5);

	//Judge the growth direction of the CurrentNormal 
	double TempAngle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(TempUpPoint.x - CurrentPoint.x,
		TempUpPoint.y - CurrentPoint.y, TempUpPoint.z - CurrentPoint.z, 
		CurrentNormal.x, CurrentNormal.y, CurrentNormal.z));

	if (TempAngle > 90)	//shoule be redirection by max direction;
	{
		if ((abs(CurrentNormal.z) > abs(CurrentNormal.x)) && (abs(CurrentNormal.z) > abs(CurrentNormal.y)))
			CurrentNormal.z = -CurrentNormal.z;
		else if ((abs(CurrentNormal.x) > abs(CurrentNormal.y)) && (abs(CurrentNormal.x) > abs(CurrentNormal.z)))
			CurrentNormal.x = -CurrentNormal.x;
		else if ((abs(CurrentNormal.y) > abs(CurrentNormal.z)) && (abs(CurrentNormal.y) > abs(CurrentNormal.z)))
			CurrentNormal.y = -CurrentNormal.y;
	}

	CurrentUpperPoint = GeometryBase::GetPointAlongLine(CurrentNormal, CurrentPoint,
		StemDiameterAndBasalRetrievalForm.spinBoxThickNessForCalcing->text().toDouble());

	CalcingStemPoints->points.clear();

	double d1 = -(CurrentNormal.x * CurrentPoint.x + CurrentNormal.y * CurrentPoint.y + CurrentNormal.z * CurrentPoint.z);
	double d2 = -(CurrentNormal.x * CurrentUpperPoint.x + CurrentNormal.y * CurrentUpperPoint.y + CurrentNormal.z * CurrentUpperPoint.z);;

	GeometryBase::GetPointsBetweenTwoPlanes(InputCloud, CurrentNormal.x, CurrentNormal.y, CurrentNormal.z, d1,
		CurrentNormal.x, CurrentNormal.y, CurrentNormal.z, d2, CalcingStemPoints);

	PointBase::PointCopy(CalcingStemPoints, CopyCalcingStemPoints);
	PointBase::SetPointColor(CalcingStemPoints, ColorBase::RedColor);
	//ShowPoints(CalcingStemPoints, CalcingStemPointsStr, PointSize);	

	StemDiameter.SetInputCloud(CalcingStemPoints, CurrentPoint, CurrentNormal);
	StemDiameterAndBasalRetrievalForm.lineEditDCircleFitting->setText(
		QString::number(StemDiameter.DRetrievalByCircleFittingIn2D(), 10, 4));

	StemDiameterAndBasalRetrievalForm.lineEditDCylinderFitting->setText(
		QString::number(StemDiameter.DRetrievalByCylinerFittingIn3D(CurrentNormal), 10, 4));
	
	StemDiameterAndBasalRetrievalForm.lineEditDConvexHullLine->setText(
		QString::number(StemDiameter.DRetrievalByConvexHullLine(), 10, 4));

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempBezierFittingPoints (new pcl::PointCloud<pcl::PointXYZRGB>());
	StemDiameterAndBasalRetrievalForm.lineEditDBezier->setText(
		QString::number(StemDiameter.DRetrievalByBezierCurve(BezierFittingPoints), 10, 4));
	//GeometryBase::RotateToOriginal(TempBezierFittingPoints, BezierFittingPoints, CurrentNormal);
	PointBase::SetPointColor(BezierFittingPoints, ColorBase::BlueColor);
	//ShowPoints(BezierFittingPoints, SplineFittingPointsStr, PointSize);
	ShowBezierFittingPoints(StemDiameterAndBasalRetrievalForm.checkBoxShowBezierFitting->checkState());

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempSplineFittingPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	StemDiameterAndBasalRetrievalForm.lineEditDSpline->setText(
		QString::number(StemDiameter.DRetrievalBySpline(SplineFittingPoints), 10, 4));
	//GeometryBase::RotateToOriginal(TempSplineFittingPoints, SplineFittingPoints, CurrentNormal);
	PointBase::SetPointColor(SplineFittingPoints, ColorBase::BlueColor);
	ShowSplineFittingPoints(StemDiameterAndBasalRetrievalForm.checkBoxShowSplineFitting->checkState());
	//ShowPoints(SplineFittingPoints, SplineFittingPointsStr, PointSize);

	CalcingBasalArea();
}

void CStemDiameterAndBasalRetrieval::CalcingBasalArea()
{
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempProfileCurvePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	StemDiameterAndBasalRetrievalForm.lineEditBasalArea->setText(
		QString::number(StemDiameter.GetProfileCurveAndArea(
			StemDiameterAndBasalRetrievalForm.spinBoxAngleForBasalArea->text().toDouble(),
			ProfileCurvePoints), 10, 4));
	//GeometryBase::RotateToOriginal(TempProfileCurvePoints, ProfileCurvePoints, CurrentNormal);
	PointBase::SetPointColor(ProfileCurvePoints, ColorBase::BlueColor);
	ShowProfileCurvePoints(StemDiameterAndBasalRetrievalForm.checkBoxShowPofileCurve->checkState());
}

void CStemDiameterAndBasalRetrieval::ChangeCalcPosition()
{
	GetPointUValue();

	CalcingStemParameters();

	ShowCalcingStemPoints(StemDiameterAndBasalRetrievalForm.checkBoxShowCalcingStemPoints->checkState());

	//This step need the D result;
	ShowCrossSectionalPoints(StemDiameterAndBasalRetrievalForm.checkBoxShowStemCrossSection->checkState());
	StemDiameterAndBasalRetrievalForm.pushButtonSaveCalcingPoints->setEnabled(true);
	StemDiameterAndBasalRetrievalForm.pushButtonStemParameterRetrieval->setEnabled(false);
}

void CStemDiameterAndBasalRetrieval::RadioPositionCheck(bool Checked)
{
	ChangeCalcPosition();
}