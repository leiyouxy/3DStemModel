#include "FittingMethods.h"

CFittingMethods::CFittingMethods()
{

}

CFittingMethods::~CFittingMethods()
{
	Viewer->removePointCloud(CirclePointsStr);
	Viewer->removePointCloud(CylinderPointsStr);
	Viewer->removePointCloud(SplinePointsStr);
	Viewer->removePointCloud(BezierPointsStr);

	CirclePoints->points.clear();
	CylinderPoints->points.clear();
	SplinePoints->points.clear();
	BezierPoints->points.clear();	

	emitUpdateUI();
}

CFittingMethods::CFittingMethods(QGroupBox * ParentWin)
{	
	QWidget *widget = new QWidget(ParentWin);
	FittingMethodsForm.setupUi(widget);

	widget->show();

	CirclePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	CylinderPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	SplinePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	BezierPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	connect(FittingMethodsForm.pushButtonCircleFitting, SIGNAL(clicked()), this, SLOT(CircleFitting()));
	connect(FittingMethodsForm.checkBoxShowCircle, SIGNAL(stateChanged(int)), this,	SLOT(ShowCirclePoints(int)));

	connect(FittingMethodsForm.pushButtonCylinderFitting, SIGNAL(clicked()), this, SLOT(CylinderFitting()));
	connect(FittingMethodsForm.checkBoxShowCylinder, SIGNAL(stateChanged(int)), this, SLOT(ShowCylinderPoints(int)));

	connect(FittingMethodsForm.pushButtonSplineFitting, SIGNAL(clicked()), this, SLOT(SplineFitting()));
	connect(FittingMethodsForm.checkBoxSplineIsClosure, SIGNAL(stateChanged(int)), this, SLOT(SplineClosureChange(int)));
	connect(FittingMethodsForm.checkBoxShowSpline, SIGNAL(stateChanged(int)), this,	SLOT(ShowSplinePoints(int)));

	connect(FittingMethodsForm.pushButtonBezierFitting, SIGNAL(clicked()), this, SLOT(BeizerFitting()));
	connect(FittingMethodsForm.checkBoxBezierIsClosure, SIGNAL(stateChanged(int)), this, SLOT(BezierClosureChange(int)));
	connect(FittingMethodsForm.checkBoxShowBezier, SIGNAL(stateChanged(int)), this,	SLOT(ShowBezierPoints(int)));

	SplineIsClosure = false;
	BezierIsClosure = false;
}

void CFittingMethods::CircleFitting()
{
	double CenterX, CenterY, R = 0;
	PointBase::FittingCircleByLeastSquares(InputCloud, CenterX, CenterY, R);

	FittingMethodsForm.lineEditCircleCenterX->setText(QString::number(CenterX));
	FittingMethodsForm.lineEditCircleCenterY->setText(QString::number(CenterY));
	FittingMethodsForm.lineEditCircleCenterZ->setText(QString::number(ZMin));

	FittingMethodsForm.lineEditCricleRadius->setText(QString::number(R));
	
	ShowCirclePoints(FittingMethodsForm.checkBoxShowCircle->checkState());

	string FileName = OpenedFilePath;
	FileName.insert(FileName.length() - 4, "_CircleFiting_");
	PointBase::SavePCDToFileName(CirclePoints, FileName);
}

void CFittingMethods::ShowCirclePoints(int checkState)
{
	CirclePoints->points.clear();
	
	Viewer->removePointCloud(CirclePointsStr);
	Viewer->removePointCloud(CylinderPointsStr);
	Viewer->removePointCloud(SplinePointsStr);
	Viewer->removePointCloud(BezierPointsStr);

	if (checkState == 2)
	{
		pcl::PointXYZRGB CenterPoint;
		double R = 0;

		if (FittingMethodsForm.lineEditCircleCenterX->text() == "" || FittingMethodsForm.lineEditCircleCenterY->text() == ""
			|| FittingMethodsForm.lineEditCircleCenterZ->text() == "" || FittingMethodsForm.lineEditCricleRadius->text() == "")
			return;

		CenterPoint.x = FittingMethodsForm.lineEditCircleCenterX->text().toDouble();
		CenterPoint.y = FittingMethodsForm.lineEditCircleCenterY->text().toDouble();
		CenterPoint.z = FittingMethodsForm.lineEditCircleCenterZ->text().toDouble();
		
		R = FittingMethodsForm.lineEditCricleRadius->text().toDouble();		

		GeometryBase::GetCirclePoints(CirclePoints, CenterPoint, R, false, 0.2);
		PointBase::SetPointColor(CirclePoints, ColorBase::RedColor);
		ShowPoints(CirclePoints, CirclePointsStr, PointSize);
		//Viewer->addPointCloud(CirclePoints, CirclePointsStr);
	}
	emitUpdateUI();
}

void CFittingMethods::CylinderFitting()
{
	CylinderPoints->points.clear();
	Viewer->removePointCloud(CylinderPointsStr);	

	pcl::PointXYZRGB CenterPoint;
	pcl::PointXYZRGB AxisDirection;	

	//CenterPoint.x = 0, CenterPoint.y = 0, CenterPoint.z = 0;
	//AxisDirection.x = 0, AxisDirection.y = 0, AxisDirection.z = 1;
	//GeometryBase::GetCyinderPoints(CylinderPoints, CenterPoint, AxisDirection, 10, 10);
	//PointBase::SetPointColor(CylinderPoints, ColorBase::BlueColor);
	//PointBase::SavePCDToFileName(CylinderPoints, "I:\\PCLQT\\TreePclQtGui\\TreePclQtGui\\3DStemModelForMeasure\\Cylinder0.pcd");

	CCylinderFitting CylinderFitting;
	CylinderFitting.SetInput(InputCloud);
	AxisDirection = CylinderFitting.CalcAxisDirection();
	CylinderFitting.CylinderFitting(AxisDirection, CenterPoint, CylinderR, CylinderH);

	//AxisDirection = GeometryBase::GetMaxDirectionVector(InputCloud);
	//AxisDirection = GeometryBase::GetCylinderDirection(InputCloud);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ArrowPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
	//GeometryBase::GetArrowPoints(ArrowPoints, AxisDirection.x, AxisDirection.y, AxisDirection.z, CenterPoint,10);
	//PointBase::SetPointColor(ArrowPoints, ColorBase::RedColor);
	//ShowPoints(ArrowPoints, "ArrowPoints");	

	FittingMethodsForm.lineEditCylinderRadius->setText(QString::number(CylinderR, 10, 3));
	FittingMethodsForm.lineEditCylinderH->setText(QString::number(CylinderH, 10, 3));

	FittingMethodsForm.lineEditCylinderNormalX->setText(QString::number(AxisDirection.x, 10, 3));
	FittingMethodsForm.lineEditCylinderNormalY->setText(QString::number(AxisDirection.y, 10, 3));
	FittingMethodsForm.lineEditCylinderNormalZ->setText(QString::number(AxisDirection.z, 10, 3));
	
	FittingMethodsForm.lineEditCylinderCenterX->setText(QString::number(CenterPoint.x, 10, 3));
	FittingMethodsForm.lineEditCylinderCenterY->setText(QString::number(CenterPoint.y, 10, 3));
	FittingMethodsForm.lineEditCylinderCenterZ->setText(QString::number(CenterPoint.z, 10, 3));	

	GeometryBase::GetCylinderPoints(CylinderPoints, CenterPoint, AxisDirection, CylinderR, CylinderH, 0.2, 0.02);
	PointBase::SetPointColor(CylinderPoints, ColorBase::RedColor);
	
	if (FittingMethodsForm.checkBoxShowCylinder->checkState() == 2)
	{
		ShowPoints(CylinderPoints, CylinderPointsStr, PointSize);
	}

	vector<int> Indexs;
	for (int i = CylinderPoints->points.size() - 1; i >= 0; i--)
	{
		if (CylinderPoints->points[i].z > ZMax || CylinderPoints->points[i].z < ZMin)
		{
			//CylinderPoints->points.erase(CylinderPoints->points.begin() + i);
		}
		else
			Indexs.push_back(i);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCylinderPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < Indexs.size(); i++)
	{
		TempCylinderPoints->points.push_back(CylinderPoints->points[i]);
	}

	emitUpdateUI();

	//if (OpenedFilePath.size() > 0)
	//{
	//	string FileName = OpenedFilePath;
	//	FileName.insert(FileName.length() - 4, "_CylinderFiting_");
	//	PointBase::SavePCDToFileName(TempCylinderPoints, FileName);
	//}
}

void CFittingMethods::ShowCylinderPoints(int checkState)
{
	if (checkState != 2)
	{
		Viewer->removePointCloud(CylinderPointsStr);
	}
	else if (checkState == 2 && CylinderPoints->points.size() == 0)
	{
		//CylinderFitting();
	}
	else if (checkState == 2 && CylinderPoints->points.size() > 0)
	{
		Viewer->removePointCloud(CylinderPointsStr);
		PointBase::SetPointColor(CylinderPoints, ColorBase::RedColor);
		//Viewer->addPointCloud(CylinderPoints, CylinderPointsStr);
		ShowPoints(CylinderPoints, CylinderPointsStr, PointSize);
	}
	
	emitUpdateUI();
}

void CFittingMethods::SplineClosureChange(int checkState)
{
	Viewer->removePointCloud(CirclePointsStr);
	Viewer->removePointCloud(CylinderPointsStr);
	Viewer->removePointCloud(SplinePointsStr);
	Viewer->removePointCloud(BezierPointsStr);

	SplinePoints->points.clear();
	Viewer->removePointCloud(SplinePointsStr);
	Viewer->removePointCloud(SplinePointsStr);

	if (SplineIsClosure && checkState != 2)
	{
		SplineIsClosure = false;
	
	}
	else if (!SplineIsClosure && checkState == 2)
	{
		SplineIsClosure = true;
	}
	emitUpdateUI();
}

void CFittingMethods::RefreshData()
{
	Viewer->removePointCloud(CirclePointsStr);
	Viewer->removePointCloud(CylinderPointsStr);
	Viewer->removePointCloud(SplinePointsStr);
	Viewer->removePointCloud(BezierPointsStr);

	CirclePoints->points.clear();
	CylinderPoints->points.clear();
	SplinePoints->points.clear();
	BezierPoints->points.clear();
}

void CFittingMethods::SplineFitting()
{		
	Viewer->removePointCloud(CirclePointsStr);
	Viewer->removePointCloud(CylinderPointsStr);
	Viewer->removePointCloud(SplinePointsStr);
	Viewer->removePointCloud(BezierPointsStr);

	if (!PointBase::PointsIs2D(InputCloud))
	{
		QMessageBox::information(NULL, tr("Information"), 
			tr("Points are not in 2D spaces!"));
		return;
	}
	
	if (SplinePoints->points.size() > 0)
	{
		PointBase::SetPointColor(SplinePoints, ColorBase::RedColor);
		//Viewer->addPointCloud(SplinePoints, SplinePointsStr);
		ShowPoints(SplinePoints, SplinePointsStr, PointSize);
		return;
	}

	emitUpdateStatusBar("Spline Fitting is processing, please wait!");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> KnotValues;

	bool IsClosure = (FittingMethodsForm.checkBoxSplineIsClosure->checkState() == 2);

	CSplineInterpolation SplineInterpolation;
	CSpline Spline;
	if (IsClosure)
	{
		SplineInterpolation.SetInputs(InputCloud, 3, true);
		SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnotValues);
				
		Spline.SetSplineInputs(ControlPoints, 3, KnotValues, true);
		Spline.FirstPointOfCloseCurve = InputCloud->points[0];
	}
	else
	{
		SplineInterpolation.SetInputs(InputCloud, 3);
		SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnotValues);
		
		Spline.SetSplineInputs(ControlPoints, 3, KnotValues);
	}

	Spline.CreateSpline();

	if (FittingMethodsForm.checkBoxSaveSplineParameters->checkState() == 2)
	{
		string ControlFileName, KnotFileName;
		ControlFileName = OpenedFilePath;
		KnotFileName = OpenedFilePath;

		ControlFileName.insert(ControlFileName.length() - 4, "_Control_");
		KnotFileName.insert(KnotFileName.length() - 4, "_Knot_");
		
		//Save as PointMove method in openfile
		QMessageBox::information(NULL, tr("Information"),
			tr(OpenedFilePath.c_str()));
		
		PointBase::SavePCDToFileName(InputCloud, OpenedFilePath);
		Spline.SaveControlPointsAndKnotValue(ControlPoints, KnotValues, 
			ControlFileName, KnotFileName, Spline.StartU, Spline.EndU);
	}

	double TempLength = Spline.GetSplineLengthBySimpson();
	FittingMethodsForm.lineEditSplineLength->setText(QString::number(TempLength,10,4));
	FittingMethodsForm.lineEditSplineRadius->setText(QString::number(TempLength / M_PI / 2, 10, 4));	

	if (FittingMethodsForm.checkBoxSplineIsClosure->checkState() == 2)
	{
		FittingMethodsForm.lineEditSplineArea->setText(QString::number(Spline.GetSplineArea(),10,4));
	}

	if (FittingMethodsForm.checkBoxShowSpline->checkState() == 2)
	{		
		SplinePoints->points.insert(SplinePoints->points.begin(),
			Spline.CurvePoints->points.begin(),
			Spline.CurvePoints->points.end());
		
		PointBase::SetPointColor(SplinePoints, ColorBase::RedColor);
		//Viewer->addPointCloud(SplinePoints, SplinePointsStr);
		ShowPoints(SplinePoints, SplinePointsStr, PointSize);
	}

	emitUpdateStatusBar("Spline Fitting is done!", 5000);
}

void CFittingMethods::ShowSplinePoints(int checkState)
{
	Viewer->removePointCloud(CirclePointsStr);
	Viewer->removePointCloud(CylinderPointsStr);
	Viewer->removePointCloud(SplinePointsStr);
	Viewer->removePointCloud(BezierPointsStr);

	if (checkState == 2)
		SplineFitting();
	else
		Viewer->removePointCloud(SplinePointsStr);
	emitUpdateUI();
}

void CFittingMethods::BezierClosureChange(int checkState)
{
	if (BezierIsClosure == true && checkState != 2)
	{
		BezierIsClosure = false;
		BezierPoints->points.clear();
		Viewer->removePointCloud(BezierPointsStr);
	}
	else if (BezierIsClosure == false && checkState == 2)
	{
		BezierIsClosure = true;
		BezierPoints->points.clear();
		Viewer->removePointCloud(BezierPointsStr);
	}
	emitUpdateUI();
}

void CFittingMethods::BeizerFitting()
{
	Viewer->removePointCloud(CirclePointsStr);
	Viewer->removePointCloud(CylinderPointsStr);
	Viewer->removePointCloud(SplinePointsStr);
	Viewer->removePointCloud(BezierPointsStr);
	
	//PointBase::SetPointsCoordinateValue(InputCloud, "Z", InputCloud->points[0].z);

	//if (!PointBase::PointsIs2D(InputCloud))
	//{
	//	QMessageBox::information(NULL, tr("Information"),
	//		tr("Points are not in 2D spaces!"));
	//	return;
	//}

	//PointBase::GetDistinctPoints(BezierPoints);

	//if (BezierPoints->points.size() >= 0)
	//{
	//	PointBase::SetPointColor(BezierPoints, ColorBase::RedColor);
	//	//Viewer->addPointCloud(BezierPoints, BezierPointsStr);
	//	ShowPoints(BezierPoints, BezierPointsStr, PointSize);
	//	return;
	//}

	emitUpdateStatusBar("Bezier Fitting is processing, please wait!");

	bool IsClosure = (FittingMethodsForm.checkBoxBezierIsClosure->checkState() == 2);

	if (IsClosure)
	{
		CContourAndConvexHull<pcl::PointXYZRGB> ConvexHull;
		vector<int> ConvexhullIndexs;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYConvexhullPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		//pcl::PointXYZRGB NormalPoint = GeometryBase::GetMaxDirectionVector(InputCloud);
		//pcl::PointXYZRGB NormalPoint = GeometryBase::GetNormalOfTriangle(
		//	BezierPoints->points[0], BezierPoints->points[BezierPoints->points.size() / 3], 
		//	BezierPoints->points[BezierPoints->points.size() * 2/ 3]);
		pcl::PointXYZRGB NormalPoint = GeometryBase::GetMinDirectionVector(InputCloud);
		GeometryBase::RotateNormalToVertical(InputCloud, XYPoints, NormalPoint);
		PointBase::SetPointsCoordinateValue(XYPoints, "Z", XYPoints->points[0].z);

		//remove repeated points
		PointBase::GetDistinctPoints(XYPoints);

		ConvexHull.SetInputs(XYPoints);
		ConvexhullIndexs.clear();
		ConvexHull.GetPointsConvexHull(ConvexhullIndexs);

		XYConvexhullPoints->points.clear();
		for (int j = 0; j < ConvexhullIndexs.size(); j++)
		{
			XYConvexhullPoints->points.push_back(XYPoints->points[ConvexhullIndexs[j]]);			
		}		
		
		//PointBase::ShowPointXYZRGBText(Viewer, XYConvexhullPoints, "Text", 0.5);
		//PointBase::SetPointColor(XYConvexhullPoints, ColorBase::BlueColor);
		//PointBase::ShowPointXYZRGB(Viewer, XYConvexhullPoints, "XYConvexhullPoints", PointSize * 2);

		CBezierSubsectionG2 BezierSubsectionG2;
		BezierSubsectionG2.SetInputs(XYConvexhullPoints);
		//The tension parameter 
		BezierSubsectionG2.ResolveControlPoints(BezierMiu);
		BezierSubsectionG2.GetBezierLength();
		BezierPoints->points.clear();
		double Length = BezierSubsectionG2.DrawBezierCurve(BezierPoints);

		FittingMethodsForm.lineEditBezierLength->setText(QString::number(Length,10,4));
		FittingMethodsForm.lineEditBezierArea->setText(QString::number(BezierSubsectionG2.GetBezierArea(), 10, 4));		
		FittingMethodsForm.lineEditBezierRadius->setText(QString::number(Length / M_PI / 2, 10, 4));
	}
	else
	{
		QMessageBox::information(NULL, tr("Information"), tr("This function is need to be developed, please choose closure Bezier curve!"));
	}

	if (FittingMethodsForm.checkBoxShowBezier->checkState() == 2)
	{
		PointBase::SetPointColor(BezierPoints, ColorBase::RedColor);
		//Viewer->addPointCloud(BezierPoints, BezierPointsStr);
		ShowPoints(BezierPoints, BezierPointsStr, PointSize);
		emitUpdateUI();
	}

	string FileName = OpenedFilePath;
	FileName.insert(FileName.length() - 4, "_BezierFiting_");
	PointBase::SavePCDToFileName(BezierPoints, FileName);

	emitUpdateStatusBar("Bezier Fitting is done!", 5000);
}

void CFittingMethods::ShowBezierPoints(int checkState)
{
	Viewer->removePointCloud(CirclePointsStr);
	Viewer->removePointCloud(CylinderPointsStr);
	Viewer->removePointCloud(SplinePointsStr);
	Viewer->removePointCloud(BezierPointsStr);

	if (checkState == 2)
		BeizerFitting();
	else
		Viewer->removePointCloud(BezierPointsStr);
	emitUpdateUI();
}