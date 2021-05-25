#include "OutliersRemovalByRadius.h"

COutliersRemovalByRadius::COutliersRemovalByRadius()
{
	
}

COutliersRemovalByRadius::COutliersRemovalByRadius(QGroupBox * ParentWin, string Type)
{
	//COutliersRemovalByRadius();
	Octree = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	GlobalCriclePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

	//Create New form
	QWidget *widget = new QWidget(ParentWin);
	SubuiOfOutliersRemoval.setupUi(widget);

	SubuiOfOutliersRemoval.comboBoxColor->addItem("Red", Red);
	SubuiOfOutliersRemoval.comboBoxColor->addItem("Green", Green);
	SubuiOfOutliersRemoval.comboBoxColor->addItem("Blue", Blue);
	SubuiOfOutliersRemoval.comboBoxColor->addItem("White", White);
	SubuiOfOutliersRemoval.comboBoxColor->addItem("Black", Black);

	SubuiOfOutliersRemoval.labelNeighbour_Number->setVisible(false);

	RemoveType = Type;
	OperationType = "";

	if (RemoveType != "Stat")
	{
		SubuiOfOutliersRemoval.labelThickNess_MutipleValue->setVisible(false);
		SubuiOfOutliersRemoval.spin_MutipleValue->setVisible(false);
	}

	if (RemoveType == "Number")
	{
		SubuiOfOutliersRemoval.labelNeighbour_Number->setVisible(true);
		SubuiOfOutliersRemoval.spin_MutipleValue->setVisible(true);
		SubuiOfOutliersRemoval.spin_MutipleValue->setDecimals(0);
	}

	if (RemoveType == "Irregularity")
	{
		SubuiOfOutliersRemoval.labelThickNess->setVisible(false);
		SubuiOfOutliersRemoval.spinBoxThickNess->setVisible(false);
		SubuiOfOutliersRemoval.checkBoxPlanes->setVisible(false);

		SubuiOfOutliersRemoval.spin_MutipleValue->setVisible(true);
		SubuiOfOutliersRemoval.spin_MutipleValue->setDecimals(1);
		SubuiOfOutliersRemoval.spin_MutipleValue->setValue(0.5 * 
			SubuiOfOutliersRemoval.doubleSpinBoxRadius->text().toDouble());

		SubuiOfOutliersRemoval.labelThickNess_MutipleValue->setVisible(true);
		SubuiOfOutliersRemoval.labelThickNess_MutipleValue->setText("Distance");		
	}
	
	connect(SubuiOfOutliersRemoval.spinBoxThickNess, SIGNAL(valueChanged(int)), this, SLOT(DrawSlicesPlane(int)));
	connect(SubuiOfOutliersRemoval.doubleSpinBoxRadius, SIGNAL(valueChanged(double)), this, SLOT(RadiusChange(double)));

	connect(SubuiOfOutliersRemoval.pushButtonCheckingByNeighbour, SIGNAL(clicked()), this, SLOT(CheckingOutliers()));	
	connect(SubuiOfOutliersRemoval.pushButtonRemovingByNeighbour, SIGNAL(clicked()), this, SLOT(RemoveOutliers()), Qt::UniqueConnection);
	connect(SubuiOfOutliersRemoval.checkBoxPlanes, SIGNAL(stateChanged(int)), this, SLOT(ShowSlicesPlane(int)));

	connect(SubuiOfOutliersRemoval.pushButtonRedo, SIGNAL(clicked()), this, SLOT(Redo()));
	connect(SubuiOfOutliersRemoval.pushButtonBat, SIGNAL(clicked()), this, SLOT(Bat()));

	OutliersNumber = 0;

	SubuiOfOutliersRemoval.pushButtonRedo->setEnabled(false);
	SubuiOfOutliersRemoval.pushButtonRemovingByNeighbour->setEnabled(false);
	SubuiOfOutliersRemoval.pushButtonCheckingByNeighbour->setEnabled(true);

	widget->show();	
}

void COutliersRemovalByRadius::RadiusChange(double Number)
{
	NeighbourNumberOfPoint.clear();
	if (RemoveType == "Irregularity")
	{
		SubuiOfOutliersRemoval.spin_MutipleValue->setValue(0.5 *
			SubuiOfOutliersRemoval.doubleSpinBoxRadius->text().toDouble());
	}
}

COutliersRemovalByRadius::~COutliersRemovalByRadius()
{
	Viewer->removePointCloud("CirclePointsForSlices");
	free(Octree);
	Octree = NULL;
	emitUpdateUI();
}

void COutliersRemovalByRadius::RefreshData()
{
	CTreeBase::RefreshData();
	NeighbourNumberOfPoint.clear();
	SubuiOfOutliersRemoval.pushButtonRedo->setEnabled(false);
	ShowSlicesPlane(SubuiOfOutliersRemoval.checkBoxPlanes->checkState());
}

void COutliersRemovalByRadius::Redo()
{
	SubuiOfOutliersRemoval.pushButtonRedo->setEnabled(false);
	CTreeBase::Redo();

	SubuiOfOutliersRemoval.pushButtonRedo->setEnabled(false);
	SubuiOfOutliersRemoval.pushButtonRemovingByNeighbour->setEnabled(false);
	SubuiOfOutliersRemoval.pushButtonCheckingByNeighbour->setEnabled(true);
}

void COutliersRemovalByRadius::GetParameters()
{
	Octree->deleteTree();
	Octree->setInputCloud(InputCloud);
	Octree->addPointsFromInputCloud();

	SectionThickness = SubuiOfOutliersRemoval.spinBoxThickNess->text().toDouble();
	SearchRadius = SubuiOfOutliersRemoval.doubleSpinBoxRadius->text().toDouble();

	if (SubuiOfOutliersRemoval.comboBoxColor->currentText() == "Red")
	{
		OlColor = Red;
		R = 255, G = 0, B = 0;
	}
	else if (SubuiOfOutliersRemoval.comboBoxColor->currentText() == "Green")
	{
		OlColor = Green;
		R = 0, G = 255, B = 0;
	}
	else if (SubuiOfOutliersRemoval.comboBoxColor->currentText() == "Blue")
	{
		OlColor = Blue;
		R = 0, G = 0, B = 255;
	}
	else if (SubuiOfOutliersRemoval.comboBoxColor->currentText() == "White")
	{
		OlColor = White;
		R = 255, G = 255, B = 255;
	}
	else if (SubuiOfOutliersRemoval.comboBoxColor->currentText() == "Black")
	{
		OlColor = Black;
		R = 0, G = 0, B = 0;
	}	
}

// calc the number of neighbour for each point
void COutliersRemovalByRadius::CalcNeighbourofPoint()
{
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		PointNeigbour TempPointNeigbour;
		TempPointNeigbour.PointIndex = i;
		
		Octree->radiusSearch(i, SearchRadius, TempPointNeigbour.NeigbourIndex, TempPointNeigbour.NeigbourDis);

		// Find the current index i is in itself's NeigbourIndex, if in, delete it.
		VectorBase<int> VectorBaseInt;
		int TempIndex = VectorBaseInt.FindIndexInVector(TempPointNeigbour.NeigbourIndex, i);
		if (TempIndex != -1)
		{
			//delete itself's index
			TempPointNeigbour.NeigbourIndex.erase(TempPointNeigbour.NeigbourIndex.begin() + TempIndex);
			//delete itself's dis value
			TempPointNeigbour.NeigbourDis.erase(TempPointNeigbour.NeigbourDis.begin() + TempIndex);		
		}

		NeighbourNumberOfPoint.push_back(TempPointNeigbour);
	}
}

//calc stat infor for each section
void COutliersRemovalByRadius::CalcSectionNeighbourStat()
{
	double MutipleValue = SubuiOfOutliersRemoval.spin_MutipleValue->text().toDouble();
	
	Vec_SectionNeighourStat.clear();
	OutliersNumber = 0;
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		// add section's neighbour info
		vector<int> Vec_CurSectionNeighbourCount;
		for (int j = 0; j < HorizontalPartition.SectionsVector[i].Indexs.size(); j++)
		{
			int CurPointIndex = HorizontalPartition.SectionsVector[i].Indexs[j];

			//Add the number of neighbours
			// May be the CurPointIndex is removed
			if (CurPointIndex == NeighbourNumberOfPoint[CurPointIndex].PointIndex)
				Vec_CurSectionNeighbourCount.push_back(NeighbourNumberOfPoint[CurPointIndex].NeigbourIndex.size());
		}

		SectionNeighourStat CurSectionNeighourStat;
		
		VectorBase<int> VectorBaseDouble;
		// calc the Neighour Statatics of the i-th section 
		CurSectionNeighourStat.SectionIndex = i;
		CurSectionNeighourStat.Std = sqrt(VectorBaseDouble.CalcVariances(Vec_CurSectionNeighbourCount, CurSectionNeighourStat.Mean));
		//Vec_SectionNeighourStat.push_back(CurSectionNeighourStat);

		//checkAndLabel  Outliers by Stat Information
		double Value = CurSectionNeighourStat.Mean - MutipleValue * CurSectionNeighourStat.Std;
		for (int j = 0; j < HorizontalPartition.SectionsVector[i].Indexs.size(); j++)		
		{			
			int CurPointIndex = HorizontalPartition.SectionsVector[i].Indexs[j];
			
			if (NeighbourNumberOfPoint[CurPointIndex].NeigbourIndex.size() < Value)
			{	
				InputCloud->points[CurPointIndex].r = R;
				InputCloud->points[CurPointIndex].g = G;
				InputCloud->points[CurPointIndex].b = B;
				OutliersNumber++;
			}
		}
	}	
	emitUpdateStatusBar(QString::number(OutliersNumber, 10) + " outliers were found and labled by " 
		+ SubuiOfOutliersRemoval.comboBoxColor->currentText() + " color!" );
	if (OutliersNumber > 0)
		SubuiOfOutliersRemoval.pushButtonRedo->setEnabled(true);
}

//calc Quartile infor for each section
void COutliersRemovalByRadius::CalcSectionNeighbourQuartile()
{
	Vec_SectionNeighourQuantile.clear();

	OutliersNumber = 0;
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		// add section's neighbour info
		vector<int> Vec_CurSectionNeighbourCount;
		for (int j = 0; j < HorizontalPartition.SectionsVector[i].Indexs.size(); j++)
		{
			int CurPointIndex = HorizontalPartition.SectionsVector[i].Indexs[j];

			//Add the number of neighbours
			if (CurPointIndex == NeighbourNumberOfPoint[CurPointIndex].PointIndex)
				Vec_CurSectionNeighbourCount.push_back(NeighbourNumberOfPoint[CurPointIndex].NeigbourIndex.size());
		}


		if (Vec_CurSectionNeighbourCount.size() < 3)
		{
			continue;
		}

		SectionNeighourQuantile CurSectionNeighourQuantile;

		VectorBase<int> VectorBaseDouble;
		// calc the Neighour Quartile of the i-th section 
		CurSectionNeighourQuantile.SectionIndex = i;
 		CurSectionNeighourQuantile.IQR =
			VectorBaseDouble.CalcQuartile(Vec_CurSectionNeighbourCount, 
				CurSectionNeighourQuantile.UpperLimit, CurSectionNeighourQuantile.LowerLimit);
			
		//Vec_SectionNeighourQuantile.push_back(CurSectionNeighourQuantile);

		//checkAndLabel  Outliers by Stat Information
		double Value = CurSectionNeighourQuantile.LowerLimit;
		for (int j = 0; j < HorizontalPartition.SectionsVector[i].Indexs.size(); j++)
		{
			int CurPointIndex = HorizontalPartition.SectionsVector[i].Indexs[j];

			if (NeighbourNumberOfPoint[CurPointIndex].NeigbourIndex.size() < Value)
			{
				InputCloud->points[CurPointIndex].r = R;
				InputCloud->points[CurPointIndex].g = G;
				InputCloud->points[CurPointIndex].b = B;
				OutliersNumber++;
			}
		}
	}
	emitUpdateStatusBar(QString::number(OutliersNumber, 10) + " outliers were found and labled by "
		+ SubuiOfOutliersRemoval.comboBoxColor->currentText() + " color!");
	if (OutliersNumber > 0)
		SubuiOfOutliersRemoval.pushButtonRedo->setEnabled(true);
}

//calc curvature for each point
void COutliersRemovalByRadius::CalcIrregularity()
{
	OutliersNumber = 0;
	PointGeometryFeatures.clear();

	double DistanceThreshold = SubuiOfOutliersRemoval.spin_MutipleValue->text().toDouble();

	PointCurvature.SetInputCloud(InputCloud);
	PointCurvature.CalcNormalAndCurvatureRadius(SearchRadius, false);

	//vector<int> Indexs = PointCurvature.GetMaxGuassCurvature(500);
	//for (int i = 0; i < Indexs.size(); i++)
	//{
	//	InputCloud->points[Indexs[i]].rgba = ColorBase::RedColor;
	//}
	
	//vector<int> Indexs;
	//vector<float> DisS;
	//vector<float> CurvatureS;
	//Octree->radiusSearch(21801, SearchRadius, Indexs, DisS);
	//
	//for (int i = 0; i < InputCloud->points.size(); i++)
	//{
	//	Viewer->removeText3D(StringBase::IntToStr(i));
	//}

	//for (int i = 0; i < Indexs.size(); i++)
	//{
	//	InputCloud->points[Indexs[i]].rgba = ColorBase::RedColor;
	//	CurvatureS.push_back(PointCurvature.GetMeanCurvature(Indexs[i]));
	//	Viewer->addText3D(StringBase::FloatToStr(CurvatureS[i]), InputCloud->points[Indexs[i]], 0.05, 255, 0, 0,
	//		StringBase::IntToStr(Indexs[i]));
	//}

	//InputCloud->points[21801].rgba = ColorBase::RedColor;

	//OutliersNumber = 10;

	//for (int i = 0; i < InputCloud->points.size(); i = i + 100)
	//{
	//	double Curvature = 0;

	//	//pcl::PrincipalCurvatures CurCurvatures = PointCurvature.GetCurvature(i);

	//	Curvature = PointCurvature.GetGuassCurvature(i);

	//	//Viewer->addText3D(StringBase::FloatToStr(Curvature), InputCloud->points[i],1,255,0,0, StringBase::IntToStr(i));
	//	Viewer->addText3D(StringBase::IntToStr(i), InputCloud->points[i], 1, 255, 0, 0, StringBase::IntToStr(i));
	//}

	//ShowNormals(InputCloud, PointCurvature.Cloud_Normals, "PointNormal", 1, 0.5);
	//PointsMove(InputCloud, -30, 0, 0);
	
	//ShowPrincipalCurvatures(InputCloud, PointCurvature.Cloud_Normals, PointCurvature.Cloud_Curvatures, 
	//	"PrincipalCurvatures", 10, 100);

	//以上为测试显示法向与曲率等参数

///*
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		PointGeometryFeature CurPointGF;
		CurPointGF.PointIndex = i;
		CurPointGF.PointNormal = PointCurvature.GetNormal(i);
		CurPointGF.PointCurvatures = PointCurvature.GetCurvature(i);

		Octree->radiusSearch(i, SearchRadius, CurPointGF.NeighourIndexs, CurPointGF.NeighourDis);

		// Find the current index i is in itself's NeigbourIndex, if in, delete it.
		VectorBase<int> VectorBaseInt;
		int TempIndex = VectorBaseInt.FindIndexInVector(CurPointGF.NeighourIndexs, i);
		if (TempIndex != -1)
		{
			//delete itself's index
			CurPointGF.NeighourIndexs.erase(CurPointGF.NeighourIndexs.begin() + TempIndex);
			//delete itself's dis value
			CurPointGF.NeighourDis.erase(CurPointGF.NeighourDis.begin() + TempIndex);
		}

		CalcPointGeometryFeature(CurPointGF);

		PointGeometryFeatures.push_back(CurPointGF);
	}

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		double Current = PointGeometryFeatures[i].DensityByDis
			+ PointGeometryFeatures[i].DisBetProjectAndCentriod
			+ PointGeometryFeatures[i].AvgGuassCurvatureChange;

		double NearestNeighbour =
			//PointGeometryFeatures[PointGeometryFeatures[i].MinDisNeighourIndex].DensityByDis 
			//+ PointGeometryFeatures[PointGeometryFeatures[i].MinDisNeighourIndex].DisBetProjectAndCentriod 
			//+ PointGeometryFeatures[PointGeometryFeatures[i].MinDisNeighourIndex].AvgGuassCurvatureChange;
			PointGeometryFeatures[PointGeometryFeatures[i].MaxDisNeighourIndex].DensityByDis
			+ PointGeometryFeatures[PointGeometryFeatures[i].MaxDisNeighourIndex].DisBetProjectAndCentriod
			+ PointGeometryFeatures[PointGeometryFeatures[i].MaxDisNeighourIndex].AvgGuassCurvatureChange;

		if (PointGeometryFeatures[i].NeighourIndexs.size() == 0
			|| PointGeometryFeatures[i].DisBetProjectAndCentriod > DistanceThreshold
			|| PointGeometryFeatures[i].DisBetProjectAndOriginal > DistanceThreshold)
		{			
			InputCloud->points[i].r = R;
			InputCloud->points[i].g = G;
			InputCloud->points[i].b = B;
			OutliersNumber++;
		}

		//if (PointGeometryFeatures[i].PointCurvatures.pc1 * PointGeometryFeatures[i].PointCurvatures.pc2 > 0)	//No Neighbour points
		//{
		//	InputCloud->points[i].rgba = ColorBase::RedColor;
		//	//PointGeometryFeatures[i].AvgGuassCurvatureChange
		//}

		//if (PointGeometryFeatures[i].NeighourIndexs.size() == 0)	//No Neighbour points
		//{
		//	InputCloud->points[i].rgba = ColorBase::RedColor;
		//}

		//if (PointGeometryFeatures[PointGeometryFeatures[i].MinDisNeighourIndex].DisBetProjectAndCentriod > Radius / 2)
		//{
		//	InputCloud->points[i].rgba = ColorBase::RedColor;
		//	//InputCloud->points[PointGeometryFeatures[i].MinDisNeighourIndex].rgba = ColorBase::RedColor;
		//}

		//if (Current > 2 * NearestNeighbour )
		//{
		//	InputCloud->points[i].rgba = ColorBase::RedColor;
		//}

		//if (InputCloud->points[PointGeometryFeatures[i].MinDisNeighourIndex].rgba == ColorBase::RedColor)
		//{
		//	InputCloud->points[i].rgba = ColorBase::RedColor;
		//}
	}
//*/
}

// calc GeometryFeature for current point
void COutliersRemovalByRadius::CalcPointGeometryFeature(PointGeometryFeature & PGF)
{
	int MinIndex = -1, MaxIndex = -1;
	float MinDis = EPSP6, MaxDis = EPSM6;

	PGF.CentriodPoint.x = 0, PGF.CentriodPoint.y = 0, PGF.CentriodPoint.z = 0;

	PGF.AvgDis = 0;
	PGF.AvgGuassCurvatureChange = 0;
	for (int i = 0; i < PGF.NeighourIndexs.size(); i++)
	{
		if (PGF.NeighourDis[i] > MaxDis)
		{
			MaxDis = PGF.NeighourDis[i];
			MaxIndex = PGF.NeighourIndexs[i];
		}

		if (PGF.NeighourDis[i] < MinDis)
		{
			MinDis = PGF.NeighourDis[i];
			MinIndex = PGF.NeighourIndexs[i];
		}

		PGF.CentriodPoint.x = PGF.CentriodPoint.x + InputCloud->points[PGF.NeighourIndexs[i]].x;
		PGF.CentriodPoint.y = PGF.CentriodPoint.y + InputCloud->points[PGF.NeighourIndexs[i]].y;
		PGF.CentriodPoint.z = PGF.CentriodPoint.z + InputCloud->points[PGF.NeighourIndexs[i]].z;

		PGF.AvgDis = PGF.AvgDis + PGF.NeighourDis[i];
		PGF.AvgGuassCurvatureChange = PGF.AvgGuassCurvatureChange +
			abs(PointCurvature.GetGuassCurvature(PGF.PointIndex)
				- PointCurvature.GetGuassCurvature(PGF.NeighourIndexs[i]));
	}

	if (PGF.NeighourIndexs.size() > 0)
	{
		PGF.MaxDisNeighourIndex = MaxIndex;
		PGF.MinDisNeighourIndex = MinIndex;

		PGF.CentriodPoint.x = PGF.CentriodPoint.x / PGF.NeighourIndexs.size();
		PGF.CentriodPoint.y = PGF.CentriodPoint.y / PGF.NeighourIndexs.size();
		PGF.CentriodPoint.z = PGF.CentriodPoint.z / PGF.NeighourIndexs.size();

		PGF.AvgDis = PGF.AvgDis / PGF.NeighourIndexs.size();
		PGF.DensityByDis =
			(abs(PGF.AvgDis -
				PointDis(InputCloud->points[PGF.PointIndex], InputCloud->points[PGF.MaxDisNeighourIndex]))
				+ abs(PGF.AvgDis -
					PointDis(InputCloud->points[PGF.PointIndex], InputCloud->points[PGF.MinDisNeighourIndex])));

		PGF.AvgGuassCurvatureChange = PGF.AvgGuassCurvatureChange / PGF.NeighourIndexs.size();

		Eigen::Vector4f PlaneCoefficients;
		PGF.ProjectPlaneCoefficients[0] = PGF.PointNormal.normal_x;
		PGF.ProjectPlaneCoefficients[1] = PGF.PointNormal.normal_y;
		PGF.ProjectPlaneCoefficients[2] = PGF.PointNormal.normal_z;
		PGF.ProjectPlaneCoefficients[3] = -(PGF.ProjectPlaneCoefficients[0] * PGF.CentriodPoint.x
			+ PGF.ProjectPlaneCoefficients[1] * PGF.CentriodPoint.y + PGF.ProjectPlaneCoefficients[2] * PGF.CentriodPoint.z);

		pcl::projectPoint(InputCloud->points[PGF.PointIndex], PGF.ProjectPlaneCoefficients, PGF.ProjectPoint);

		PGF.DisBetProjectAndCentriod = PointDis(PGF.ProjectPoint, PGF.CentriodPoint);
		PGF.DisBetProjectAndOriginal = PointDis(PGF.ProjectPoint, InputCloud->points[PGF.PointIndex]);
	}
}



//calc Number for each section
void COutliersRemovalByRadius::CalcSectionNeighbourNumber()
{	
	OutliersNumber = 0;
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		double Value = SubuiOfOutliersRemoval.spin_MutipleValue->text().toDouble();
		for (int j = 0; j < HorizontalPartition.SectionsVector[i].Indexs.size(); j++)
		{
			int CurPointIndex = HorizontalPartition.SectionsVector[i].Indexs[j];

			if (NeighbourNumberOfPoint[CurPointIndex].NeigbourIndex.size() < Value)
			{
				InputCloud->points[CurPointIndex].r = R;
				InputCloud->points[CurPointIndex].g = G;
				InputCloud->points[CurPointIndex].b = B;
				OutliersNumber++;
			}
		}
	}
	emitUpdateStatusBar(QString::number(OutliersNumber, 10) + " outliers were found and labled by "
		+ SubuiOfOutliersRemoval.comboBoxColor->currentText() + " color!");
}

void COutliersRemovalByRadius::CheckingOutliers()
{
	if (OutliersNumber > 0)
		SaveToRedo();
	
	OutliersNumber = 0;
	if (InputCloud->points.size() == 0) return;

	SubuiOfOutliersRemoval.pushButtonCheckingByNeighbour->setEnabled(false);
	SubuiOfOutliersRemoval.pushButtonBat->setEnabled(false);

	emitUpdateStatusBar("Outliers are being checked, please wait!");

	GetParameters();
	
	if (SectionThickness > 5)
	{
		int ID = QMessageBox::information(NULL, tr("Information"),
			tr("Slice thickness value is too large, that will take too long time to calculate! Are you sure?"),
			QMessageBox::Yes | QMessageBox::No);
		if (ID == QMessageBox::No)
			return;
	}

	//Partition tree points into several sections
	HorizontalPartition.SetInputCloud(this->InputCloud);
	HorizontalPartition.SetThickNess(SectionThickness);
	HorizontalPartition.PatitionSection();

	if (RemoveType != "Irregularity")	
		CalcNeighbourofPoint();

	if (RemoveType == "Stat")
		CalcSectionNeighbourStat();
	else if (RemoveType == "Quartile")
		CalcSectionNeighbourQuartile();
	else if (RemoveType == "Number")
		CalcSectionNeighbourNumber();
	else if (RemoveType == "Irregularity")
		CalcIrregularity();
	
	emitUpdateStatusBar(("Outliers have been checked! There are " 
			+ StringBase::IntToStr(OutliersNumber) + " Outliers").c_str(), 0);
	if (OperationType != "Bat") emitShowMsg("Outliers have been checked!");
	emitUpdateUI();

	if (OutliersNumber > 0)
	{
		SubuiOfOutliersRemoval.pushButtonRedo->setEnabled(true);
		SubuiOfOutliersRemoval.pushButtonRemovingByNeighbour->setEnabled(true);
		SubuiOfOutliersRemoval.pushButtonCheckingByNeighbour->setEnabled(false);
	}
	else
	{
		SubuiOfOutliersRemoval.pushButtonRedo->setEnabled(false);
		SubuiOfOutliersRemoval.pushButtonRemovingByNeighbour->setEnabled(false);
		SubuiOfOutliersRemoval.pushButtonCheckingByNeighbour->setEnabled(true);
	}
	SubuiOfOutliersRemoval.pushButtonBat->setEnabled(true);
}

void COutliersRemovalByRadius::RemoveOutliers()
{
	if (OperationType != "Bat")
	{
		int ButtonID =
			QMessageBox::question(NULL, "Warning", "OutliersChecking Starting.",
				QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);

		if (QMessageBox::Yes != ButtonID) return;	
	}

	SubuiOfOutliersRemoval.pushButtonRemovingByNeighbour->setEnabled(false);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	int Removed = 0;
	//for (int i = InputCloud->points.size() - 1; i >= 0; i--)
	for(int i = 0; i < InputCloud->points.size(); i++)
	{
		if (InputCloud->points[i].r == R && InputCloud->points[i].g == G 
			&& InputCloud->points[i].b == B)
		{
			//InputCloud->points.erase(InputCloud->points.begin() + i);
			
			if (NeighbourNumberOfPoint.size () > i + 1)
				NeighbourNumberOfPoint[i].PointIndex = -1;
			
			Removed++;
		}
		else
		{
			TempPoints->points.push_back(InputCloud->points[i]);
		}
	}	

	InputCloud->points.clear();
	InputCloud->points.insert(InputCloud->points.begin(),
		TempPoints->points.begin(), TempPoints->points.end());

	emitUpdateStatusBar(QString::number(Removed) + " outliers are removed!",5000);
	emitUpdateUI();

	SubuiOfOutliersRemoval.pushButtonRedo->setEnabled(false);
	SubuiOfOutliersRemoval.pushButtonRemovingByNeighbour->setEnabled(false);
	SubuiOfOutliersRemoval.pushButtonCheckingByNeighbour->setEnabled(true);
}

void COutliersRemovalByRadius::ShowSlicesPlane(int CheckValue)
{
	HorizontalPartition.UnShowSectionPoints();
	if (CheckValue == 2)
	{			
		DrawSlicesPlane(0);
	}
	emitUpdateUI();
}

void COutliersRemovalByRadius::DrawSlicesPlane(int ThickNess)
{
	//clear the calc result for 
	HorizontalPartition.UnShowSectionPoints();
	NeighbourNumberOfPoint.clear();
	if (SubuiOfOutliersRemoval.checkBoxPlanes->checkState() != 2) return;

	//Viewer->removePointCloud("CirclePointsForSlices");
	HorizontalPartition.SetInputCloud(this->InputCloud);
	HorizontalPartition.SetThickNess(SubuiOfOutliersRemoval.spinBoxThickNess->text().toDouble());
	HorizontalPartition.SetViewer(Viewer);
	HorizontalPartition.PatitionSection();
	HorizontalPartition.ShowSectionPoints(0, HorizontalPartition.SectionsCount);

	/*GlobalCriclePoints->points.clear();
	Viewer->removePointCloud("CirclePointsForSlices");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CriclePoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointXYZRGB CenterPoint;
	CenterPoint.x = 0, CenterPoint.y = 0;
	for (float ZValue = 0; ZValue < ZMax; ZValue = ZValue + ThickNess)
	{
		pcl::ModelCoefficients PlaneCoefficients;
		PlaneCoefficients.values.push_back(0.0);
		PlaneCoefficients.values.push_back(0.0);
		PlaneCoefficients.values.push_back(1.0);
		PlaneCoefficients.values.push_back(-ZValue);

		CenterPoint.z = ZValue;
		GeometryBase::GetCirclePoints(CriclePoints, CenterPoint, 20, 5);

		GlobalCriclePoints->points.insert(GlobalCriclePoints->points.end(),
			CriclePoints->points.begin(), CriclePoints->points.end());

	}
	PointBase::SetPointColor(GlobalCriclePoints, ColorBase::BlueColor);
	PointBase::ShowPointXYZRGB(Viewer, GlobalCriclePoints, "CirclePointsForSlices", 2);	
	//*/
}

void COutliersRemovalByRadius::Bat()
{
	SubuiOfOutliersRemoval.pushButtonBat->setEnabled(false);
	OperationType = "Bat";
	char Drive[_MAX_DRIVE];
	char FilePath[_MAX_DIR];
	char Fname[_MAX_FNAME];
	char Ext[_MAX_EXT];
	_splitpath(OpenedFilePath.c_str(), Drive, FilePath, Fname, Ext);

	string FilePathStr;
	FilePathStr = string(Drive) + string(FilePath);
	vector<string> BatFiles;
	//GetFiles(FilePath, BatFiles);
	GetFiles(FilePathStr, BatFiles);

	for each (string FileNameStr in BatFiles)
	{
		cout<<"Current File Name:" << FileNameStr <<endl;
		_splitpath(FileNameStr.c_str(), Drive, FilePath, Fname, Ext);
		if ((strcmp(Ext, ".pcd") == 0) || (strcmp(Ext, ".vtx") == 0) || (strcmp(Ext, ".las") == 0))
		{			
			OpenedFilePath = FilePathStr + FileNameStr;
			string SaveFileName = FilePathStr + "OutliersRemoved\\" + FileNameStr;

			InputCloud->points.clear();
			PointBase::OpenPCLFile(OpenedFilePath, InputCloud, false);
			CTreeBase::SetInputCloud(CTreeBase::InputCloud);

			RefreshData();
			emitUpdateAppTitle(OpenedFilePath);
			emitUpdateUI();

			CheckingOutliers();
			RemoveOutliers();

			//CheckingOutliers();
			//RemoveOutliers();

			//CheckingOutliers();
			//RemoveOutliers();
			
			//PointBase::SavePCDToFileName(InputCloud, OpenedFilePath);
			PointBase::SavePCDToFileName(InputCloud, SaveFileName);
		}
	}
	SubuiOfOutliersRemoval.pushButtonBat->setEnabled(true);
}


/* Old Class 
void COutliersRemovalByRadius::SetVoxelParameters(double LeafXValue, double LeafYValue, 
	double LeafZValue, double SearchRadiusValue, int MinNeighborsValue)
{
	LeafX = LeafXValue;	
	LeafY = LeafYValue;	
	LeafZ = LeafZValue;	
	SearchRadius = SearchRadiusValue;
	MinNeighbors = MinNeighborsValue;
}

void COutliersRemovalByRadius::RadiusOutlierRemoval()
{
	cout<<"正在根据近邻关系移除噪声点"<<endl;
	//InputCloud = PointBase::VoxelGridDownSample(InputCloud, 0.01f,0.01f,0.01f);
	//TempCloud = this->VoxelGridDownSample(TempCloud, 0.5f,0.5f,0.05f);

	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> OutlierRemoval;
	OutlierRemoval.setInputCloud(InputCloud);
	
	//设置搜索半径 在2CM 的范围内 必须有 5 个点
	//根部扫描密度比较高 故可以设置1Cm的半径范围内有10个点
	OutlierRemoval.setRadiusSearch(SearchRadius);	//0.5 * 0.5 * 0.5 立方厘米三维体素栅格
	OutlierRemoval.setMinNeighborsInRadius(MinNeighbors);

	// apply filter
	OutlierRemoval.setNegative(false);
	OutlierRemoval.filter (*OutputInners);	
	OutlierRemoval.setNegative(true);
	OutlierRemoval.filter (*Outliers);

	//for (int j = 0; j < cloud_Outliers->points.size(); j++)
	//{		
	//	OutliersCloud->points.push_back(cloud_Outliers->points[j]);			
	//}	

	////将树上半部的点加入到点云中
	//for (int i = Index; i < this->InterValNumber; i++)
	//{
	//	for(int j = 0; j < this->SectionPoint[i].SectionPtr->size(); j++)
	//	{
	//		cloud_Inners->points.push_back(this->SectionPoint[i].SectionPtr->points[j]);	
	//	}		
	//}
	//
	//StemCloud = cloud_Inners;	
}

//2015.06.15 新添加内容，根据每个分区的统计数据计算离散点
void COutliersRemovalByRadius::SetIntputParameters(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue,
		pcl::PointCloud<PointXYZRGBIndex>::Ptr InputIndexCloudValue,
		SectionVector & SectionsVectorValue, 
		double SearchRadiusValue)
{
	SectionsVector = SectionsVectorValue;
	SearchRadius = SearchRadiusValue;
	InputIndexCloud = InputIndexCloudValue;
	//MinNeighbors = MinNeighborsValue;
	InputCloud = InputCloudValue;

	Octree->deleteTree();
	Octree->setInputCloud(InputCloud);
	Octree->addPointsFromInputCloud();
}

//2015.06.15 计算每一个点的点云密度，即在 SearchRadius 半径内，有多少个点 MinNeighborsValue
void COutliersRemovalByRadius::CalcEachPointDensity()
{
	//
	vector<int> NearestPointIndex;	//最近邻点的索引
	vector<float> NearestPointDis;	//最近邻点的距离
	
	for(int i = 0; i < InputCloud->points.size(); i++)
	{
		
		NearestPointIndex.clear();
		NearestPointDis.clear();

		Octree->radiusSearch(i, SearchRadius, NearestPointIndex, NearestPointDis); //半径搜索

		//2015.01.15 因有时候邻域半径获取不到自己，所以添加此部分		
		if (PointBase::FindSelfInNegighbourIndex(NearestPointIndex, i))
			InputIndexCloud->points[i].Index = NearestPointIndex.size() - 1;	//个数是领域个数减1，还包括自己
		else
			InputIndexCloud->points[i].Index = NearestPointIndex.size();	//个数是领域个数减1，还包括自己
		
		
		//Octree->nearestKSearch(i, MinNeighbors * 4, 
		//	NearestPointIndex, NearestPointDis);

		//for (int j = 0; j < NearestPointDis.size(); j++)
		//{
		//	if (NearestPointDis[j] > SearchRadius)
		//	{
		//		if ( j > 0 ) InputIndexCloud->points[i].Index = j - 1;
		//		break;
		//	}
		//}
	}
}


//2015.06.15 计算每个分区的点云密度，求出平均值即可
void COutliersRemovalByRadius::CalcSectionsDensity(double StdMutiple, 
		int StartIndex, int EndIndex)
{
	CalcBase <double> floatCalcBase;
	double VarianceValue;
	double AvgValue;

	if (EndIndex == 0)
		EndIndex = SectionsVector.size() - 1;
	if (EndIndex >= SectionsVector.size())
		EndIndex = SectionsVector.size() - 1;

	vector<double> DisS;		
	for(int i = StartIndex; i < EndIndex;  i++)
	{			
		cout<<"计算第"<<i<<"个分区的统计信息"<<endl;
		for(int j = 0; j < SectionsVector[i].SectionPtr->points.size(); j++)
		{
			//根据分区点云的索引找到 所在位置 
			DisS.push_back(InputIndexCloud->points[SectionsVector[i].SectionPtr->points[j].Index].Index);
		}		
	}
	VarianceValue = floatCalcBase.CalcVariances(DisS, AvgValue);

	for(int i = StartIndex; i < EndIndex;  i++)
	{	
		cout<<"使用第"<<i<<"个分区的统计信息"<<endl;		
		for(int j = 0; j < SectionsVector[i].SectionPtr->points.size(); j++)
		{
			//根据分区点云的索引找到 所在位置 
			if (InputIndexCloud->points[SectionsVector[i].SectionPtr->points[j].Index].Index 
				< AvgValue - StdMutiple * VarianceValue)
			{	//是异常点
				InputIndexCloud->points[SectionsVector[i].SectionPtr->points[j].Index].Category = 0; 
			}
		}		
	}
}

//2015.05.15 对每个分区进行离群点划分，根据求出的个数平均值
// 出发点是根据 SearchRadiusValue 半径内 至少 有MinNeighborsValue个点，
void COutliersRemovalByRadius::OutlierRemovalBySectionsStat(int SuccessiveSectionCount,
	double StdMutiple)
{	
	CalcEachPointDensity();	//此处比较费时
	
	for(int i = 0; i < SectionsVector.size(); i = i + SuccessiveSectionCount)
	{
		CalcSectionsDensity(StdMutiple, i, i + SuccessiveSectionCount);
		if ( SectionsVector.size() - i < SuccessiveSectionCount)
		{
			CalcSectionsDensity(StdMutiple, 
				i + SuccessiveSectionCount, SectionsVector.size() - 1 );		
		}
	}
}

void COutliersRemovalByRadius::GetNormalAndOutliers(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr NormalCloud,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutliersCloud)
{
	for(int i = 0; i < InputIndexCloud->points.size(); i++)
	{				
		if (InputIndexCloud->points[i].Category == 0) //异常点
		{
			OutliersCloud->points.push_back(InputCloud->points[i]);
		}
		else
		{
			NormalCloud->points.push_back(InputCloud->points[i]);
		}
	}
}

//2016.01.15 直接根据给定半径内至少有多少个邻域点来移除噪声
///后修改为按照循环的方式实现
void COutliersRemovalByRadius::OutlierRemovalByMinNeighbour(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempInputCloud,
	double Radius, int MinNeighbourNum,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr NormalCloud,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutliersCloud, int LoopNum)	
{	
	OutliersCloud->points.clear();
	int LastOutliersNum = 1, CurrentOutliersNum = 0;

	while (LoopNum > 0)
	//while(LastOutliersNum != CurrentOutliersNum)
	{
		LastOutliersNum = CurrentOutliersNum;
		Octree->deleteTree();
		Octree->setInputCloud(TempInputCloud);
		Octree->addPointsFromInputCloud();

		NormalCloud->points.clear();

		for(int i = 0; i < TempInputCloud->points.size(); i++)
		{
			vector<int> NearestPointIndex;
			vector<float> NearestPointDis;
			NearestPointIndex.clear();
			NearestPointDis.clear();
			int NeighbourNum;

			Octree->radiusSearch(i, Radius, NearestPointIndex, NearestPointDis); //半径搜索

			//2015.01.15 因有时候邻域半径获取不到自己，所以添加此部分
			if (PointBase::FindSelfInNegighbourIndex(NearestPointIndex, i))
				NeighbourNum = NearestPointIndex.size() - 1;	//个数是邻域个数减1，还包括自己
			else
				NeighbourNum = NearestPointIndex.size();	//个数是邻域个数减1，还包括自己

			if (NeighbourNum >= MinNeighbourNum)
			{
				NormalCloud->points.push_back(TempInputCloud->points[i]);
			}
			else
			{
				OutliersCloud->points.push_back(TempInputCloud->points[i]);
			}
		}	

		CurrentOutliersNum = OutliersCloud->points.size();
		TempInputCloud->points.clear();
		TempInputCloud->points.insert(TempInputCloud->points.end(),
			NormalCloud->points.begin(), NormalCloud->points.end());
		LoopNum--;
	}
}
*/