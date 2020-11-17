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
	StemStubRemovalForm.spinBoxStartHeight->setValue(50);
	GetParameters();

	ShowSlicesPoints(StemStubRemovalForm.checkBoxVerticalSlices->checkState());
}