#include "StemPointsRemovalByPlane.h"

void CStemPointsRemovalByPlane::RefreshData()
{
	if (StemPointsRemoval.comboBoxRule->currentText()  == "Below")
		PointBase::PointMoveToOrigin(InputCloud);
	CTreeBase::RefreshData();		
	RefreshFormParameters();
	DrawPlanPoints(StemPointsRemoval.checkBoxPlane->checkState());
}

void CStemPointsRemovalByPlane::Redo()
{
	StemPointsRemoval.pushButtonRedo->setEnabled(false);
	CTreeBase::Redo();
	RefreshData();
}

CStemPointsRemovalByPlane::CStemPointsRemovalByPlane(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	StemPointsRemoval.setupUi(widget);

	//Some initial work
	StemPointsRemoval.comboBoxAxis->addItem("x");
	StemPointsRemoval.comboBoxAxis->addItem("y");
	StemPointsRemoval.comboBoxAxis->addItem("z");

	StemPointsRemoval.comboBoxAxis->setCurrentIndex(2);

	StemPointsRemoval.comboBoxRule->addItem("Above");
	StemPointsRemoval.comboBoxRule->addItem("Below");

	//Show the plane
	widget->show();
	
	connect(StemPointsRemoval.comboBoxAxis, SIGNAL(currentIndexChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
	connect(StemPointsRemoval.comboBoxAxisValue, SIGNAL(currentIndexChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));	
	connect(StemPointsRemoval.lineEditPlaneEquation, SIGNAL(textChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));

	connect(StemPointsRemoval.checkBoxPlane, SIGNAL(stateChanged(int)), this, SLOT(DrawPlanPoints(int)));
	connect(StemPointsRemoval.comboBoxRule, SIGNAL(currentIndexChanged(int)), this, SLOT(DrawArrow(int)));
		
	connect(StemPointsRemoval.pushButtonRemoval, SIGNAL(clicked()), this, SLOT(RemovalPoints()));

	connect(StemPointsRemoval.pushButtonRedo, SIGNAL(clicked()), this, SLOT(Redo()));

	connect(StemPointsRemoval.pushButtonParttition, SIGNAL(clicked()), this, SLOT(Parttition()));
	connect(StemPointsRemoval.pushButtonBat, SIGNAL(clicked()), this, SLOT(Bat()));

	StemPointsRemoval.pushButtonRedo->setEnabled(false);

	a = 0, b = 0, c = 0, d = 0;
	MaxDis = 0;
	PlaneCenterPoint.x = 0, PlaneCenterPoint.y = 0, PlaneCenterPoint.z = 0;
}

CStemPointsRemovalByPlane::~CStemPointsRemovalByPlane()
{
	Viewer->removePointCloud("PlanePoints");
	Viewer->removePointCloud("ArrowPoints");
	emitUpdateUI();	
}

void CStemPointsRemovalByPlane::RefreshFormParameters()
{		
	disconnect(StemPointsRemoval.comboBoxAxisValue, SIGNAL(currentIndexChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
	//disconnect(StemPointsRemoval.lineEditPlaneEquation, SIGNAL(textChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
	if (StemPointsRemoval.comboBoxAxis->currentText() == "x")
	{
		StemPointsRemoval.comboBoxAxisValue->clear();
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(XMax, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(7 * (XMax -XMin) / 8.0 + XMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(6 * (XMax - XMin) / 8.0 + XMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(5 * (XMax - XMin) / 8.0 + XMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(4 * (XMax - XMin) / 8.0 + XMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(3 * (XMax - XMin) / 8.0 + XMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(2 * (XMax - XMin) / 8.0 + XMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(1 * (XMax - XMin) / 8.0 + XMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->setCurrentIndex(1);
		a = 1, b = 0, c = 0;
	}
	else if (StemPointsRemoval.comboBoxAxis->currentText() == "y")
	{
		StemPointsRemoval.comboBoxAxisValue->clear();
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(YMax, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(7 * (YMax - YMin) / 8.0 + YMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(6 * (YMax - YMin) / 8.0 + YMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(5 * (YMax - YMin) / 8.0 + YMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(4 * (YMax - YMin) / 8.0 + YMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(3 * (YMax - YMin) / 8.0 + YMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(2 * (YMax - YMin) / 8.0 + YMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(1 * (YMax - YMin) / 8.0 + YMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->setCurrentIndex(1);
          		a = 0, b = 1, c = 0;
	}
	else if (StemPointsRemoval.comboBoxAxis->currentText() == "z")
	{
		StemPointsRemoval.comboBoxAxisValue->clear();
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(ZMax, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(7 * (ZMax - ZMin) / 8.0 + ZMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(6 * (ZMax - ZMin) / 8.0 + ZMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(5 * (ZMax - ZMin) / 8.0 + ZMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(4 * (ZMax - ZMin) / 8.0 + ZMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(3 * (ZMax - ZMin) / 8.0 + ZMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(2 * (ZMax - ZMin) / 8.0 + ZMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->addItem(QString::number(1 * (ZMax - ZMin) / 8.0 + ZMin, 10, 2));
		StemPointsRemoval.comboBoxAxisValue->setCurrentIndex(1);
		a = 0, b = 0, c = 1;
	}		

	StemPointsRemoval.lineEditPlaneEquation->setText(StemPointsRemoval.comboBoxAxis->currentText()
		+ " = " + StemPointsRemoval.comboBoxAxisValue->currentText());
	d = -1.0 * StemPointsRemoval.comboBoxAxisValue->currentText().toDouble();
	PlaneCenterPoint = FindPlaneCenterPointByPointsZone();
	connect(StemPointsRemoval.comboBoxAxisValue, SIGNAL(currentIndexChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
	//connect(StemPointsRemoval.lineEditPlaneEquation, SIGNAL(textChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
}

void CStemPointsRemovalByPlane::GeneratePlaneEquation(QString AxisStr)
{
	QWidget * Sender = (QWidget *)sender();
	if (Sender == NULL) return;

	QString SenderObjName = Sender->objectName();
	//QMessageBox::information(NULL, box->objectName(),
	//	tr("This example shows how the change layouts dynamically."));

	if (SenderObjName == "comboBoxAxis")
	{
		disconnect(StemPointsRemoval.comboBoxAxisValue, SIGNAL(currentIndexChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
		disconnect(StemPointsRemoval.lineEditPlaneEquation, SIGNAL(textChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
 		RefreshFormParameters();		
		connect(StemPointsRemoval.comboBoxAxisValue, SIGNAL(currentIndexChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
		connect(StemPointsRemoval.lineEditPlaneEquation, SIGNAL(textChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
		DrawPlanPoints(StemPointsRemoval.checkBoxPlane->checkState());
	}
	else if (SenderObjName == "lineEditPlaneEquation")
	{		
		if (StemPointsRemoval.lineEditPlaneEquation->text().toStdString().find("=") <= 0) return;
		DrawPlanPoints(StemPointsRemoval.checkBoxPlane->checkState());
	}
	else if (SenderObjName == "comboBoxAxisValue")
	{
		disconnect(StemPointsRemoval.lineEditPlaneEquation, SIGNAL(textChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
		StemPointsRemoval.lineEditPlaneEquation->setText(StemPointsRemoval.comboBoxAxis->currentText()
			+ " = " + StemPointsRemoval.comboBoxAxisValue->currentText());
		connect(StemPointsRemoval.lineEditPlaneEquation, SIGNAL(textChanged(QString)), this, SLOT(GeneratePlaneEquation(QString)));
		d = -1.0 * StemPointsRemoval.comboBoxAxisValue->currentText().toDouble();
		DrawPlanPoints(StemPointsRemoval.checkBoxPlane->checkState());
	}
}

void CStemPointsRemovalByPlane::DrawPlanPoints(int checkValue)
{
	Viewer->removePointCloud("PlanePoints");	
	if (checkValue == 2)
	{		
		if (XMax - XMin > YMax - YMin)
			MaxDis = XMax - XMin;
		else
			MaxDis = YMax - YMin;
				
		GetPlaneCoefficients(StemPointsRemoval.lineEditPlaneEquation->text().toStdString());
		
		//the procedure should be invoked after the procedure of GetPlaneCoefficients
		PlaneCenterPoint = FindPlaneCenterPointByPointsZone();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		GeometryBase::GetPlanePoints(PlanePoints, a, b, c, PlaneCenterPoint, MaxDis);
		PointBase::SetPointColor(PlanePoints, ColorBase::BlueColor);
		PointBase::ShowPointXYZRGB(Viewer, PlanePoints, "PlanePoints", 2);
		DrawArrow(StemPointsRemoval.comboBoxRule->currentIndex());
	}
	emitUpdateUI();
}

void CStemPointsRemovalByPlane::DrawArrow(int Index)
{
	if (abs(MaxDis) < EPSM6 || ((abs(PlaneCenterPoint.x) < EPSM6) && (abs(PlaneCenterPoint.y) < EPSM6)
		&& (abs(PlaneCenterPoint.z) < EPSM6)))
		return;

	pcl::PointXYZRGB Point2, Normal;
	Normal.x = a, Normal.y = b, Normal.z = c;
	
	Viewer->removePointCloud("ArrowPoints");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ArrowPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	if (Index == 0)
		GeometryBase::GetArrowPoints(ArrowPoints, Normal.x, Normal.y, Normal.z, PlaneCenterPoint, MaxDis);
	else
		GeometryBase::GetArrowPoints(ArrowPoints, Normal.x, Normal.y, Normal.z, PlaneCenterPoint, MaxDis, false);

	PointBase::SetPointColor(ArrowPoints, ColorBase::RedColor);
	//the ReferenceCenterPoint is equal to the tip of the arrow points
	ReferenceCenterPoint = ArrowPoints->points[100];
	PointBase::ShowPointXYZRGB(Viewer, ArrowPoints, "ArrowPoints", 2);
	
	QWidget * Sender = (QWidget *)sender();
	if (Sender == NULL) return;
	if (Sender->objectName() == "comboBoxRule")
		emitUpdateUI();
}
//get the coefficients of a plane by plane experssion
void CStemPointsRemovalByPlane::GetPlaneCoefficients(string EquationStr)
{
	int aIndex = 0, bIndex = 0, cIndex = 0, dIndex = 0;
	
	string LeftStr = "", RightStr = "";

	int TempIndex = 0;
	if ((TempIndex = EquationStr.find_first_of("X")) > 0)
		EquationStr.replace(TempIndex, 1, "x");
	if ((TempIndex = EquationStr.find_first_of("Y")) > 0)
		EquationStr.replace(TempIndex, 1, "y");
	if ((TempIndex = EquationStr.find_first_of("Z")) > 0)
		EquationStr.replace(TempIndex, 1, "z");
	   	
	aIndex = EquationStr.find_first_of("x");
	bIndex = EquationStr.find_first_of("y");
	cIndex = EquationStr.find_first_of("z");

	if (aIndex == -1) a = 0;
	if (bIndex == -1) b = 0;
	if (cIndex == -1) c = 0;
	
	while (aIndex >= 0 || bIndex >= 0 || cIndex >= 0)
	{
		//a parameter is exist
		if (aIndex >= 0 && (aIndex < bIndex || bIndex < 0) && (aIndex < cIndex || cIndex < 0))
		{ 
			LeftStr = EquationStr.substr(0, aIndex);
			if (LeftStr == "") a = 1;
			else a = atof(LeftStr.c_str());
			EquationStr = EquationStr.substr(aIndex + 1, EquationStr.length() - aIndex - 1);
		}

		//b parameter is exist
		if (bIndex >= 0 && (bIndex < aIndex || aIndex < 0) && (bIndex < cIndex || cIndex < 0))
		{
			LeftStr = EquationStr.substr(0, bIndex);
			if (LeftStr == "") b = 1;
			else b = atof(LeftStr.c_str());
			EquationStr = EquationStr.substr(bIndex + 1, EquationStr.length() - bIndex - 1);
		}

		//c parameter is exist
		if (cIndex >= 0 && (cIndex < aIndex || aIndex < 0) && (cIndex < bIndex || bIndex < 0))
		{
			LeftStr = EquationStr.substr(0, cIndex);
			if (LeftStr == "") c = 1;
			else c = atof(LeftStr.c_str());
			EquationStr = EquationStr.substr(cIndex + 1, EquationStr.length() - cIndex - 1);
		}
		aIndex = EquationStr.find_first_of("x");
		bIndex = EquationStr.find_first_of("y");
		cIndex = EquationStr.find_first_of("z");
	}

	dIndex = EquationStr.find_first_of("=");
	
	// not = expression
	if (dIndex < 0) 
	{
		d = 0;
		return;
	}
	
	LeftStr = EquationStr.substr(0, dIndex);
	RightStr = EquationStr.substr(dIndex + 1, EquationStr.length() - dIndex - 1);
	if (RightStr == "0")
	{
		d = 0;
		return;
	}

	LeftStr = LeftStr + "-" + RightStr;
	d = Arithmetic::expressionCalculate(LeftStr);
}

//find a suitable point for show plane by the domain of intput points
pcl::PointXYZRGB CStemPointsRemovalByPlane::FindPlaneCenterPointByPointsZone()
{
	pcl::PointXYZRGB ResultPoint;
	
	ResultPoint.x = (XMin + XMax) / 2.0;
	ResultPoint.y = (YMin + YMax) / 2.0;
	ResultPoint.z = (ZMin + ZMax) / 2.0;

	if (a != 0)
	{
		ResultPoint.x = (-d - b * ResultPoint.y - c * ResultPoint.z) / a;
	}
	else if (b != 0)
	{
		ResultPoint.y = (-d - a * ResultPoint.x - c * ResultPoint.z) / b;
	}
	else if (c != 0)
	{
		ResultPoint.z = (-d - a * ResultPoint.x - b * ResultPoint.y) / c;
	}

	return ResultPoint;
}

void CStemPointsRemovalByPlane::RemovalPoints()
{
	emitUpdateStatusBar("Points are being removed, please wait!", 3000);

	SaveToRedo();

	StemPointsRemoval.pushButtonRemoval->setEnabled(false);
	StemPointsRemoval.pushButtonRedo->setEnabled(false);
	//bool IsUp = (StemPointsRemoval.comboBoxRule->currentIndex() == 0);
	double ReferenceValue = a * ReferenceCenterPoint.x + b * ReferenceCenterPoint.y 
		+ c * ReferenceCenterPoint.z + d;
	
	vector<int> NoRemovedIndexs;
	for (int i = InputCloud->points.size() - 1; i >=0 ;i--)
	{
		double CurrentValue = a * InputCloud->points[i].x + b * InputCloud->points[i].y 
			+ c * InputCloud->points[i].z + d;
		//The current point is lies on the same side with ReferenceCenterPoint when the result is > 0 
		if (ReferenceValue * CurrentValue <= 0)	
		{
			NoRemovedIndexs.push_back(i);
			//InputCloud->points.erase(InputCloud->points.begin() + i);
		}		
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = 0; i < NoRemovedIndexs.size(); i++)
	{
		TempPoints->points.push_back(InputCloud->points[NoRemovedIndexs[i]]);
	}

	InputCloud->points.clear();
	InputCloud->points.insert(InputCloud->points.end(),
		TempPoints->points.begin(), TempPoints->points.end());
	TempPoints->points.clear();
		
	emitUpdateUI();
	RefreshData();
	StemPointsRemoval.pushButtonRedo->setEnabled(true);
	StemPointsRemoval.pushButtonRemoval->setEnabled(true);
	emitUpdateStatusBar("Points have been removed!", 3000);
}

void CStemPointsRemovalByPlane::Parttition()
{
	emitUpdateStatusBar("Parttition is being done!", 3000);
	double Dis = StemPointsRemoval.doubleSpinBoxPartDistance->text().toDouble();

	CHorizontalPartition HorizontalPartition;
	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetThickNess(Dis);
	HorizontalPartition.PatitionSection();
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		string FileName = OpenedFilePath;
				
		FileName.insert(FileName.length() - 4, "_" + StringBase::IntToStr(i*Dis) 
				+ "_" + StringBase::IntToStr((i+1)*Dis));
		
		HorizontalPartition.SaveSectionSToFile(FileName, i, i+1);
	}
	emitUpdateStatusBar("Parttition has been Done!", 3000);
}

void CStemPointsRemovalByPlane::Bat()
{
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
		if ((strcmp(Ext, ".pcd") == 0) || (strcmp(Ext, ".vtx") == 0) || (strcmp(Ext, ".las") == 0))
		{
			OpenedFilePath = FilePathStr + FileNameStr;
			InputCloud->points.clear();
			PointBase::OpenPCLFile(OpenedFilePath, InputCloud);
			CTreeBase::SetInputCloud(CTreeBase::InputCloud);

			RefreshData();
			emitUpdateAppTitle(OpenedFilePath);
			emitUpdateUI();

			Parttition();
		}
	}
	emitUpdateStatusBar("Bat has been Done!", 3000);
}