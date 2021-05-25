#include "TreeStubDetection.h"

void CTreeStubDetection::GetParameters()
{
	HorizontalPartition.SetViewer(Viewer);
	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetThickNess(StemStubRemovalForm.spinBoxThickNess->text().toDouble());
	HorizontalPartition.PatitionSection();

	SearchStartIndex = ceil(StemStubRemovalForm.spinBoxStartHeight->text().toDouble() /
		StemStubRemovalForm.spinBoxThickNess->text().toDouble());
	
	if (SearchStartIndex >= HorizontalPartition.SectionsCount - 2)
		SearchStartIndex = HorizontalPartition.SectionsCount - 2;

	HorizontalPartition.CalcCenterPointsByGravity(0, SearchStartIndex + 1);
}

CTreeStubDetection::CTreeStubDetection(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	StemStubRemovalForm.setupUi(widget);

	connect(StemStubRemovalForm.checkBoxVerticalSlices, SIGNAL(stateChanged(int)), this, SLOT(ShowSlicesPoints(int)));
	connect(StemStubRemovalForm.spinBoxThickNess, SIGNAL(valueChanged(int)), this, SLOT(ShowSlicesPoints(int)));
	connect(StemStubRemovalForm.spinBoxStartHeight, SIGNAL(valueChanged(int)), this, SLOT(ShowHeightPlane(int)));

	connect(StemStubRemovalForm.pushButtonCheckStub, SIGNAL(clicked()), this, SLOT(CheckStub()));
	connect(StemStubRemovalForm.pushButtonRedo, SIGNAL(clicked()), this, SLOT(Redo()));
	connect(StemStubRemovalForm.pushButtonRemoval, SIGNAL(clicked()), this, SLOT(StubRemoval()));
	connect(StemStubRemovalForm.pushButtonRemoval_Bat, SIGNAL(clicked()), this, SLOT(StubRemovalUnderGivenHeightBat()));

	connect(StemStubRemovalForm.pushButton_CircleCutCheck, SIGNAL(clicked()), this, SLOT(CircleCuttingCheck()));
	connect(StemStubRemovalForm.pushButtonRedo_CircleCut, SIGNAL(clicked()), this, SLOT(CircleCuttingRedo()));
	connect(StemStubRemovalForm.pushButtonRemoval_CircleCut, SIGNAL(clicked()), this, SLOT(CircleCutting()));
	connect(StemStubRemovalForm.pushButtonRemoval_BatCircleCut, SIGNAL(clicked()), this, SLOT(CircleCuttingBat()));

	StemStubRemovalForm.pushButtonRedo->setEnabled(false);
	StemStubRemovalForm.pushButtonRemoval->setEnabled(false);

	PlanePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	widget->show();
}

CTreeStubDetection::~CTreeStubDetection()
{
	if (Viewer != NULL)
	{
		Viewer->removePointCloud(PlanePointsStr);
		emitUpdateUI();
	}
}

void CTreeStubDetection::ShowSlicesPoints(int CheckValue)
{
	if (InputCloud == NULL) return;

	QAction * Action = (QAction *)sender();
	if (Action == NULL) return;

	if (Action->objectName() == "spinBoxThickNess")
		GetParameters();

	CheckValue = StemStubRemovalForm.checkBoxVerticalSlices->checkState();

	if (CheckValue == 2)		
		HorizontalPartition.ShowSectionPoints(0, HorizontalPartition.SectionsCount);	
	else if (CheckValue != 2)
		HorizontalPartition.UnShowSectionPoints();
	emitUpdateUI();
}

void CTreeStubDetection::CheckStub()
{
	GetParameters();
	StemStartIndex = GetStemStartIndex(SearchStartIndex,
		StemStubRemovalForm.doubleSpinBoxVarianceError->text().toDouble());
		
	if (StemStartIndex > 0)
	{
		StemStubRemovalForm.pushButtonRemoval->setEnabled(true);
		StemStubRemovalForm.pushButtonRedo->setEnabled(true);
		HorizontalPartition.SetSectionColors(0, StemStartIndex, ColorBase::GreyColor);
		emitUpdateStatusBar("Stub index found: " + QString::number(StemStartIndex), 5000);
	}
	else
	{
		StemStubRemovalForm.pushButtonRemoval->setEnabled(false);
		StemStubRemovalForm.pushButtonRedo->setEnabled(false);
		emitUpdateStatusBar("Stub index not found!", 5000);
	}
	emitUpdateUI();
}

void CTreeStubDetection::Redo()
{
	StemStubRemovalForm.pushButtonRedo->setEnabled(false);
	StemStubRemovalForm.pushButtonRemoval->setEnabled(false);
	CTreeBase::Redo();
	RefreshData();
}

void CTreeStubDetection::ShowHeightPlane(int Value)
{
	SearchStartIndex = ceil(StemStubRemovalForm.spinBoxStartHeight->text().toDouble() /
		StemStubRemovalForm.spinBoxThickNess->text().toDouble());

	if (SearchStartIndex >= HorizontalPartition.SectionsCount - 2)
		SearchStartIndex = HorizontalPartition.SectionsCount - 2;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints (new pcl::PointCloud<pcl::PointXYZRGB>());
	HorizontalPartition.GetSectionPoints(SearchStartIndex + 1, SectionPoints);
	double TempR = GeometryBase::CircleFittingByLeastSquaresFitting(SectionPoints);

	pcl::PointXYZRGB CenterPoint = GeometryBase::GetCentroidOfPoints(SectionPoints);
	GeometryBase::GetCirclePoints(PlanePoints, CenterPoint, TempR * 2.0, true, 0.5);

	PointBase::SetPointsCoordinateValue(PlanePoints, "Z", HorizontalPartition.SectionsVector[SearchStartIndex + 1].ZMin);

	PointBase::SetPointColor(PlanePoints, ColorBase::BlueColor);

	ShowPoints(PlanePoints, PlanePointsStr, PointSize);
}

void CTreeStubDetection::CircleCuttingCheck()
{
}

void CTreeStubDetection::CircleCuttingRedo()
{
}

void CTreeStubDetection::CircleCutting()
{
}

void CTreeStubDetection::CircleCuttingBat()
{
}

void CTreeStubDetection::StubRemoval()
{
	StemStubRemovalForm.pushButtonRedo->setEnabled(false);
	StemStubRemovalForm.pushButtonCheckStub->setEnabled(false);
	StemStubRemovalForm.pushButtonRemoval->setEnabled(false);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = StemStartIndex; i < HorizontalPartition.SectionsCount; i++)
	{
		for(int j = 0; j < HorizontalPartition.SectionsVector[i].Indexs.size(); j++)
		{ 
			TempPoints->points.push_back(InputCloud->points[HorizontalPartition.SectionsVector[i].Indexs[j]]);
		}
	}

	InputCloud->points.clear();
	InputCloud->points.insert(InputCloud->points.end(),
		TempPoints->points.begin(), TempPoints->points.end());
	TempPoints->points.clear();

	emitUpdateStatusBar("Stub points have been removed!", 5000);
	emitUpdateUI();
	RefreshData();
	StemStubRemovalForm.pushButtonRedo->setEnabled(true);
	StemStubRemovalForm.pushButtonCheckStub->setEnabled(true);
}

//2021.01.30 批量移除树桩部位的点云
void CTreeStubDetection::StubRemovalUnderGivenHeight()
{
	int StartIndex = StemStubRemovalForm.doubleSpinBoxCount_Start->text().toInt();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ResultPoints (new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = StartIndex + 1; i < HorizontalPartition.SectionsCount; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
		HorizontalPartition.GetSectionPoints(i, SectionPoints);

		ResultPoints->points.insert(ResultPoints->points.end(), SectionPoints->points.begin(),
			SectionPoints->points.end());
	}
	InputCloud->points.clear();
	PointBase::PointCopy(ResultPoints, InputCloud);
}

void CTreeStubDetection::StubRemovalUnderGivenHeightBat()
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
		cout << "Current File Name:" << FileNameStr << endl;
		_splitpath(FileNameStr.c_str(), Drive, FilePath, Fname, Ext);
		if ((strcmp(Ext, ".pcd") == 0) || (strcmp(Ext, ".vtx") == 0) 
			|| (strcmp(Ext, ".las") == 0))
		{
			OpenedFilePath = FilePathStr + FileNameStr;
			string SaveFileName = FilePathStr + "StemAfterOutliersRemoved\\" + FileNameStr;

			InputCloud->points.clear();
			PointBase::OpenPCLFile(OpenedFilePath, InputCloud, false);
			CTreeBase::SetInputCloud(CTreeBase::InputCloud);

			RefreshData();
			emitUpdateAppTitle(OpenedFilePath);
			emitUpdateUI();

			StubRemovalUnderGivenHeight();
			//CheckingOutliers();
			//RemoveOutliers();

			//CheckingOutliers();
			//RemoveOutliers();

			//PointBase::SavePCDToFileName(InputCloud, OpenedFilePath);
			PointBase::SavePCDToFileName(InputCloud, SaveFileName);
		}
	}	
}

void CTreeStubDetection::SetViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue)
{
	CTreeBase::SetViewer(ViewerValue);

	if (StemStubRemovalForm.spinBoxStartHeight->text().toDouble() > ZMax)
	{
		disconnect(StemStubRemovalForm.spinBoxStartHeight, SIGNAL(valueChanged(int)), this, SLOT(ShowHeightPlane(int)));
		StemStubRemovalForm.spinBoxStartHeight->setValue(ZMax / 4);
		disconnect(StemStubRemovalForm.spinBoxStartHeight, SIGNAL(valueChanged(int)), this, SLOT(ShowHeightPlane(int)));
	}
	
	GetParameters();
	ShowHeightPlane(0);
}

int CTreeStubDetection::GetStemStartIndex(int StartSectionIndex, double AllowError)
{
	vector<double>  DisVec;
	//vector<double>  DisVecTemp;
	vector<double>  VariancesVec;
	int i = 0;	
	double Dis = 0;	
	
	StubIndex = -1;

	if (StartSectionIndex <= 0 ) return 0;
		
	VectorBase<double> BaseCalcInstance;	
	
	//Add distance uptodown 2019.03.25
	for ( i = StartSectionIndex; i > 0; i--)
	{		
		Dis = PointDis(HorizontalPartition.GeometryCenterPointsPtr->points[i],
			HorizontalPartition.GeometryCenterPointsPtr->points[i - 1]);

		DisVec.push_back(Dis); 
		
		//此处可以添加一些代码以处理噪声点影响的问题，2019.03.28
		VariancesVec.push_back(BaseCalcInstance.CalcVariances(DisVec));		
	}		
	
	for (i = 1 ; i < DisVec.size() - 1; i++)
	{
		if ((abs(VariancesVec[i] - VariancesVec[i + 1]) > AllowError))
		{
			StubIndex = StartSectionIndex - i;	
			break;
		}
	}		

	//if (StubIndex < 0) 	StubIndex = 1;
	if (StubIndex < 0) 	StubIndex = 0;
	
	return StubIndex;	
}

void CTreeStubDetection::RefreshData()
{
	//Invoking Base class RefreshData method
	CTreeBase::RefreshData();
	StemStubRemovalForm.spinBoxStartHeight->setValue(150);
	GetParameters();

	ShowSlicesPoints(StemStubRemovalForm.checkBoxVerticalSlices->checkState());
}