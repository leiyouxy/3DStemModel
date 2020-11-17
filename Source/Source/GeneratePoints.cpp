#include "GeneratePoints.h"

//Need modify the app title name

void CGeneratePoints::RandomVec(vector<double> & RanValues, int Num, int Mutiple)
{
	srand((unsigned)time(NULL));
	vector<double> Vec;
	for (long long int i = 0; i < Num * Mutiple; i++)
	{
		Vec.push_back(i);
	}
	random_shuffle(Vec.begin(), Vec.end());
	
	int StartIndex = PointBase::GetRandom(0, Vec.size() - Num - 1);

	RanValues.clear();
	RanValues.insert(RanValues.begin(), Vec.begin() + StartIndex, Vec.begin() + StartIndex + Num);
}

CGeneratePoints::CGeneratePoints()
{

}

CGeneratePoints::CGeneratePoints(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	GeneratePointsForm.setupUi(widget);

	connect(GeneratePointsForm.pushButtonCirclePoints, SIGNAL(clicked()), this, SLOT(GenerateCirclePoints()));
	connect(GeneratePointsForm.pushButtonCylinderPoints, SIGNAL(clicked()), this, SLOT(GenerateCylinderPoints()));
	connect(GeneratePointsForm.pushButtonConePoints, SIGNAL(clicked()), this, SLOT(GenerateConePoints()));
	connect(GeneratePointsForm.pushButtonParabolaPoints, SIGNAL(clicked()), this, SLOT(GenerateParabolaPoints()));
	connect(GeneratePointsForm.pushButtonPolygonPoints, SIGNAL(clicked()), this, SLOT(GeneratePolygonPoints()));
	connect(GeneratePointsForm.pushButtonPlanePoints, SIGNAL(clicked()), this, SLOT(GeneratePlanePoints()));
	connect(GeneratePointsForm.spinBoxPolygonNumber, SIGNAL(valueChanged(int)), this, SLOT(PolygonNumberChange(int)));
	
	widget->show();
}

CGeneratePoints::~CGeneratePoints()
{

}

void CGeneratePoints::RefreshData()
{
	CTreeBase::RefreshData();
}

void CGeneratePoints::GenerateCirclePoints()
{
	emitUpdateStatusBar("Under generating", 3000);
	emitUpdateAppTitle("");

	pcl::PointXYZRGB CenterPoint, Direction;
	CenterPoint.x = GeneratePointsForm.lineEditCircleCenterX->text().toDouble();
	CenterPoint.y = GeneratePointsForm.lineEditCircleCenterY->text().toDouble();
	CenterPoint.z = GeneratePointsForm.lineEditCircleCenterZ->text().toDouble();

	pcl::PointXYZRGB TempCenterPoint;
	TempCenterPoint.x = 0, TempCenterPoint.y = 0, TempCenterPoint.z = 0;	
	InputCloud->points.clear();
	GeometryBase::GetCirclePoints(InputCloud, TempCenterPoint,
		GeneratePointsForm.lineEditCricleRadius->text().toDouble(), false,
		GeneratePointsForm.doubleSpinBoxAngleIntervalCircle->text().toDouble(),		
		GeneratePointsForm.checkBoxCircleGaussianNoise->checkState() == 2, 0,
		GeneratePointsForm.doubleSpinBoxNoiseCircleDev->text().toDouble());	

	if ((abs(CenterPoint.x - TempCenterPoint.x)) > EPSM6 ||
		(abs(CenterPoint.y - TempCenterPoint.y)) > EPSM6 || (abs(CenterPoint.z - TempCenterPoint.z) > EPSM6))
	{
		PointsMove(InputCloud, CenterPoint.x, CenterPoint.y, CenterPoint.z);
	}

	PointBase::SetPointColor(InputCloud, ColorBase::StemColor);
	emitUpdateStatusBar("Points has been generated", 3000);
	//emitUpdateUI();
	emitResetCamera();
}

void CGeneratePoints::GenerateCylinderPoints()
{
	emitUpdateStatusBar("Under generating", 3000);
	emitUpdateAppTitle("");

	pcl::PointXYZRGB CenterPoint, Direction;
	CenterPoint.x = GeneratePointsForm.lineEditCylinderCenterX->text().toDouble();
	CenterPoint.y = GeneratePointsForm.lineEditCylinderCenterY->text().toDouble();
	CenterPoint.z = GeneratePointsForm.lineEditCylinderCenterZ->text().toDouble();

	Direction.x = GeneratePointsForm.lineEditCylinderNormalX->text().toDouble();
	Direction.y = GeneratePointsForm.lineEditCylinderNormalY->text().toDouble();
	Direction.z = GeneratePointsForm.lineEditCylinderNormalZ->text().toDouble();

	pcl::PointXYZRGB TempCenterPoint, TempDirection;
	TempCenterPoint.x = 0, TempCenterPoint.y = 0, TempCenterPoint.z = 0;
	TempDirection.x = 0, TempDirection.y = 0, TempDirection.z = 1;	

	InputCloud->points.clear();
	GeometryBase::GetCylinderPoints(InputCloud, TempCenterPoint, TempDirection,
		GeneratePointsForm.lineEditCylinderRadius->text().toDouble(),
		GeneratePointsForm.lineEditCylinderH->text().toDouble(),
		GeneratePointsForm.doubleSpinBoxAngleIntervalCylinder->text().toDouble(),
		GeneratePointsForm.doubleSpinBoxHIntervalCylinder->text().toDouble(),
		GeneratePointsForm.checkBoxCylinderGaussianNoise->checkState() == 2, 0,
		GeneratePointsForm.doubleSpinBoxNoiseCylinderDev->text().toDouble());
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	if ((abs(Direction.x - TempDirection.x)) > EPSM6 ||
		(abs(Direction.y - TempDirection.y)) > EPSM6 ||
		(abs(Direction.z - TempDirection.z) > EPSM6))
	{
		GeometryBase::RotateToOriginal(InputCloud, TempCloud, Direction);
		InputCloud->points.clear();
		InputCloud->points.insert(InputCloud->points.end(), 
			TempCloud->points.begin(), TempCloud->points.end());
	}
	
	if ((abs(CenterPoint.x - TempCenterPoint.x)) > EPSM6 ||
		(abs(CenterPoint.y - TempCenterPoint.y)) > EPSM6 || (abs(CenterPoint.z - TempCenterPoint.z) > EPSM6))
	{
		PointsMove(InputCloud, CenterPoint.x, CenterPoint.y, CenterPoint.z);
	}	

	//BatGeneratePoints();

	PointBase::SetPointColor(InputCloud, ColorBase::StemColor);
	emitUpdateStatusBar("Cylinder Points has been generated", 3000);
	//emitUpdateUI();
	emitResetCamera();
}

void CGeneratePoints::GenerateParabolaPoints()
{
	emitUpdateStatusBar("Under generating", 3000);
	emitUpdateAppTitle("");

	pcl::PointXYZRGB CenterPoint, Direction;
	CenterPoint.x = GeneratePointsForm.lineEditConeCenterX->text().toDouble();
	CenterPoint.y = GeneratePointsForm.lineEditConeCenterY->text().toDouble();
	CenterPoint.z = GeneratePointsForm.lineEditConeCenterZ->text().toDouble();

	Direction.x = GeneratePointsForm.lineEditConeNormalX->text().toDouble();
	Direction.y = GeneratePointsForm.lineEditConeNormalY->text().toDouble();
	Direction.z = GeneratePointsForm.lineEditConeNormalZ->text().toDouble();

	pcl::PointXYZRGB TempCenterPoint, TempDirection;
	TempCenterPoint.x = 0, TempCenterPoint.y = 0, TempCenterPoint.z = 0;
	TempDirection.x = 0, TempDirection.y = 0, TempDirection.z = 1;
	InputCloud->points.clear();
	GeometryBase::GetParabolaBodyPoints(InputCloud, TempCenterPoint, TempDirection,
		GeneratePointsForm.lineEditConeLowerR->text().toDouble(),
		GeneratePointsForm.lineEditConeUpperR->text().toDouble(),
		GeneratePointsForm.lineEditConeH->text().toDouble(),
		GeneratePointsForm.doubleSpinBoxAngleIntervalCone->text().toDouble(),
		GeneratePointsForm.doubleSpinBoxHIntervalCone->text().toDouble(),
		GeneratePointsForm.checkBoxConeGaussianNoise->checkState() == 2, 0,
		GeneratePointsForm.doubleSpinBoxNoiseConeDev->text().toDouble());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	if ((abs(Direction.x - TempDirection.x)) > EPSM6 ||
		(abs(Direction.y - TempDirection.y)) > EPSM6 || (abs(Direction.z - TempDirection.z) > EPSM6))
	{
		GeometryBase::RotateToOriginal(InputCloud, TempCloud, Direction);
		InputCloud->points.clear();
		InputCloud->points.insert(InputCloud->points.end(),
			TempCloud->points.begin(), TempCloud->points.end());
	}

	if ((abs(CenterPoint.x - TempCenterPoint.x)) > EPSM6 ||
		(abs(CenterPoint.y - TempCenterPoint.y)) > EPSM6 || (abs(CenterPoint.z - TempCenterPoint.z) > EPSM6))
	{
		PointsMove(InputCloud, CenterPoint.x, CenterPoint.y, CenterPoint.z);
	}

	PointBase::SetPointColor(InputCloud, ColorBase::StemColor);
	emitUpdateStatusBar("Parabola Points has been generated", 3000);
	//emitUpdateUI();
	emitResetCamera();
}

void CGeneratePoints::GenerateConePoints()
{
	emitUpdateStatusBar("Under generating", 3000);
	emitUpdateAppTitle("");

	pcl::PointXYZRGB CenterPoint, Direction;
	CenterPoint.x = GeneratePointsForm.lineEditConeCenterX->text().toDouble();
	CenterPoint.y = GeneratePointsForm.lineEditConeCenterY->text().toDouble();
	CenterPoint.z = GeneratePointsForm.lineEditConeCenterZ->text().toDouble();

	Direction.x = GeneratePointsForm.lineEditConeNormalX ->text().toDouble();
	Direction.y = GeneratePointsForm.lineEditConeNormalY->text().toDouble();
	Direction.z = GeneratePointsForm.lineEditConeNormalZ->text().toDouble();

	pcl::PointXYZRGB TempCenterPoint, TempDirection;
	TempCenterPoint.x = 0, TempCenterPoint.y = 0, TempCenterPoint.z = 0;
	TempDirection.x = 0, TempDirection.y = 0, TempDirection.z = 1;

	InputCloud->points.clear();
	GeometryBase::GetConePoints(InputCloud, TempCenterPoint, TempDirection,
		GeneratePointsForm.lineEditConeLowerR->text().toDouble(),
		GeneratePointsForm.lineEditConeUpperR->text().toDouble(),
		GeneratePointsForm.lineEditConeH->text().toDouble(),
		GeneratePointsForm.doubleSpinBoxAngleIntervalCone->text().toDouble(),
		GeneratePointsForm.doubleSpinBoxHIntervalCone->text().toDouble(),
		GeneratePointsForm.checkBoxConeGaussianNoise->checkState() == 2, 0,
		GeneratePointsForm.doubleSpinBoxNoiseConeDev->text().toDouble());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	if ((abs(Direction.x - TempDirection.x)) > EPSM6 ||
		(abs(Direction.y - TempDirection.y)) > EPSM6 || (abs(Direction.z - TempDirection.z) > EPSM6))
	{
		GeometryBase::RotateToOriginal(InputCloud, TempCloud, Direction);
		InputCloud->points.clear();
		InputCloud->points.insert(InputCloud->points.end(),
			TempCloud->points.begin(), TempCloud->points.end());
	}

	if ((abs(CenterPoint.x - TempCenterPoint.x)) > EPSM6 ||
		(abs(CenterPoint.y - TempCenterPoint.y)) > EPSM6 || (abs(CenterPoint.z - TempCenterPoint.z) > EPSM6))
	{
		PointsMove(InputCloud, CenterPoint.x, CenterPoint.y, CenterPoint.z);
	}

	PointBase::SetPointColor(InputCloud, ColorBase::StemColor);
	emitUpdateStatusBar("Cone Points has been generated", 3000);
	//emitUpdateUI();
	emitResetCamera();
	//GeometryBase::GetConePoints(InputCloud, CenterPoint, Direction, 15, 30, 300, 1, 0.5, true, 0, 0.5);
}

void CGeneratePoints::PolygonNumberChange(int Number)
{
	if ((GeneratePointsForm.spinBoxPolygonNumber->text().toInt() % 2) == 1)
		GeneratePointsForm.spinBoxPolygonNumber->setValue(
			ceil(GeneratePointsForm.spinBoxPolygonNumber->text().toInt()));
}


 void CGeneratePoints::Generate2DPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud, int Count)
{
	Cloud->points.clear();

	vector<double> XValue, YValue;
	RandomVec(XValue, Count * 2);
	RandomVec(YValue, Count * 3, 20);

	for (int i = 0; i < Count; i++)
	{		
		pcl::PointXYZRGB Temp;
		Temp.x = XValue[i], Temp.y = YValue[i + PointBase::GetRandom(0, Count/2)], Temp.z = 0;

		Cloud->points.push_back(Temp);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	GeometryBase::RotateZ(Cloud, OutCloud, M_PI * (GeneratePointsForm.spinBoxZRotate->text().toInt() / 180.0));
	Cloud->points.clear();
	Cloud->points.insert(Cloud->points.begin(), OutCloud->points.begin(), OutCloud->points.end());

	//float Step = sqrt(Count);

	//for (int i = 0; i < Step; i++)
	//{
	//	//x 的基数
	//	float xValue = i * Step;
	//	for (int j = 0; j < Step; j++)
	//	{
	//		//y 的基数
	//		float yValue = j * Step;

	//		pcl::PointXYZRGB Temp;
	//		int rad = PointBase::GetRandom(1, clock()) % (int(Step)) * 1.0;
	//		//cout<<"rad"<< rad <<endl;
	//		Temp.x = xValue + PointBase::GetRandom(1, clock()) % (int(Step)) * 1.0;
	//		Temp.y = yValue + PointBase::GetRandom(1, Step) * 1.0;
	//		//cout << "xValue:"<< xValue <<",yValue:"<< yValue << endl;			
	//		Temp.z = 0;
	//		//cout << "Temp" << Temp << endl;

	//		Cloud->points.push_back(Temp);
	//	}
	//}
}

 //生成圆状 随机点 2020.05.28
 void CGeneratePoints::Generate2DCirclePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud, int Count)
 {
	 Cloud->points.clear();

	 vector<double> RValue, AngleValue;
	 RandomVec(RValue, Count * 2);
	 RandomVec(AngleValue, Count * 3, 20);

	 for (int i = 0; i < Count; i++)
	 {
		 pcl::PointXYZRGB Temp;

		 Temp.x = RValue[i] * sin(AngleValue[i]), 
			Temp.y = RValue[i] * cos(AngleValue[i]), Temp.z = 0;

		 Cloud->points.push_back(Temp);
	 }
 }

void CGeneratePoints::GeneratePlanePoints()
{
	emitUpdateStatusBar("Under generating", 3000);
	emitUpdateAppTitle("");

	if (GeneratePointsForm.radioButtonCircle->isChecked())
		Generate2DCirclePoints(InputCloud, GeneratePointsForm.spinBoxPlanarPointsNumber->text().toInt());
	else if (GeneratePointsForm.radioButtonRectangle->isChecked())
		Generate2DPoints(InputCloud, GeneratePointsForm.spinBoxPlanarPointsNumber->text().toInt());

	PointBase::SetPointColor(InputCloud, ColorBase::BlueColor);
	emitUpdateStatusBar("Plane Points has been generated", 3000);
	//emitUpdateUI();
	emitResetCamera();
}

void CGeneratePoints::GeneratePolygonPoints()
{
	emitUpdateStatusBar("Under generating", 3000);
	emitUpdateAppTitle("");

	pcl::PointXYZRGB TempCenterPoint, TempDirection;
	TempCenterPoint.x = 0, TempCenterPoint.y = 0, TempCenterPoint.z = 0;
	TempDirection.x = 0, TempDirection.y = 0, TempDirection.z = 1;

	double AngleSpace = 360 / GeneratePointsForm.spinBoxPolygonNumber->text().toInt();
	InputCloud->points.clear();
	GeometryBase::GetCylinderPoints(InputCloud, TempCenterPoint, TempDirection,
		GeneratePointsForm.lineEditPolygonRadius->text().toDouble(),
		GeneratePointsForm.lineEditPolygonH->text().toDouble(),
		AngleSpace,
		GeneratePointsForm.doubleSpinBoxHIntervalPolygon->text().toDouble(),
		false);	

	PointBase::SetPointColor(InputCloud, ColorBase::StemColor);	
	emitUpdateStatusBar("Polygon Points has been generated", 3000);
	//emitUpdateUI();
	emitResetCamera();
}

void CGeneratePoints::BatGeneratePoints()
{
	vector<int> RValues;
	
	RValues.push_back(1);

	for (int i = 1; i <= 40; i++)
	{
		RValues.push_back(RValues[i - 1] + 1);
	}

	for (int i = 0; i < RValues.size(); i++)
	{
		double LowerR = RValues[i], UpperR = RValues[i] + 1;
		
		double Length = 100;

		double AngleSpace = 1;

		//cylinder length1
		string FileName;		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points1 (new pcl::PointCloud<pcl::PointXYZRGB>);	
				
		pcl::PointXYZRGB TempCenterPoint, TempDirection;
		TempCenterPoint.x = 0, TempCenterPoint.y = 0, TempCenterPoint.z = 0;
		TempDirection.x = 0, TempDirection.y = 0, TempDirection.z = 1;
		
		for(int j = 0; j < 3; j++)
		{
			//cylinder AngleSpace
			GeometryBase::GetCylinderPoints(Points1, TempCenterPoint, TempDirection,
				LowerR, Length + j *100, AngleSpace, 0.5, false);
			FileName = StringBase::FloatToStr(LowerR*2) + "_" + StringBase::FloatToStr(Length + j * 100) + "_"
				+ StringBase::FloatToStr(AngleSpace) + "_0.5";
			PointBase::SavePCDToFileName(Points1, "I:\\3DStemVolumePoint\\20190305\\Cylinder1\\Cylinder" + FileName + ".pcd");

			//cylinder AngleSpace/2.0
			GeometryBase::GetCylinderPoints(Points1, TempCenterPoint, TempDirection,
				LowerR, Length + j * 100, AngleSpace / 2.0, 0.5, false);
			FileName = StringBase::FloatToStr(LowerR*2) + "_" + StringBase::FloatToStr(Length + j * 100) + "_"
				+ StringBase::FloatToStr(AngleSpace / 2.0) + "_0.5";
			PointBase::SavePCDToFileName(Points1, "I:\\3DStemVolumePoint\\20190305\\Cylinder2\\Cylinder" + FileName + ".pcd");

			//Cone AngleSpace
			GeometryBase::GetConePoints(Points1, TempCenterPoint, TempDirection,
				LowerR, UpperR, Length + j * 100, AngleSpace, 0.5, false);
			FileName = StringBase::FloatToStr(LowerR*2) + "_" + StringBase::FloatToStr(UpperR*2) + "_"
				+ StringBase::FloatToStr(Length + j * 100) + "_"
				+ StringBase::FloatToStr(AngleSpace) + "_0.5";
			PointBase::SavePCDToFileName(Points1, "I:\\3DStemVolumePoint\\20190305\\Cone1\\Cone" + FileName + ".pcd");

			//Cone AngleSpace/2.0
			GeometryBase::GetConePoints(Points1, TempCenterPoint, TempDirection,
				LowerR, UpperR, Length + j * 100, AngleSpace / 2.0, 0.5, false);
			FileName = StringBase::FloatToStr(LowerR*2) + "_" + StringBase::FloatToStr(UpperR*2) + "_"
				+ StringBase::FloatToStr(Length + j * 100) + "_"
				+ StringBase::FloatToStr(AngleSpace / 2.0) + "_0.5";
			PointBase::SavePCDToFileName(Points1, "I:\\3DStemVolumePoint\\20190305\\Cone2\\Cone" + FileName + ".pcd");
			
			//Parabola AngleSpace
			GeometryBase::GetParabolaBodyPoints(Points1, TempCenterPoint, TempDirection,
				LowerR, UpperR, Length + j * 100, AngleSpace, 0.5, false);
			FileName = StringBase::FloatToStr(LowerR*2) + "_" + StringBase::FloatToStr(UpperR*2) + "_"
				+ StringBase::FloatToStr(Length + j * 100) + "_"
				+ StringBase::FloatToStr(AngleSpace) + "_0.5";
			PointBase::SavePCDToFileName(Points1, "I:\\3DStemVolumePoint\\20190305\\Parabola1\\Parabola" + FileName + ".pcd");

			//Parabola AngleSpace/2.0
			GeometryBase::GetParabolaBodyPoints(Points1, TempCenterPoint, TempDirection,
				LowerR, UpperR, Length + j * 100, AngleSpace / 2.0, 0.5, false);
			FileName = StringBase::FloatToStr(LowerR*2) + "_" + StringBase::FloatToStr(UpperR*2) + "_"
				+ StringBase::FloatToStr(Length + j * 100) + "_"
				+ StringBase::FloatToStr(AngleSpace / 2.0) + "_0.5";
			PointBase::SavePCDToFileName(Points1, "I:\\3DStemVolumePoint\\20190305\\Parabola2\\Parabola" + FileName + ".pcd");
		}
	}
}