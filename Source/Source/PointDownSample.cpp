
#include "PointDownSample.h"

void CPointDownSample::RefreshData()
{
	CTreeBase::RefreshData();
}

CPointDownSample::CPointDownSample(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	PointDownSampleForm.setupUi(widget);
	
	DownSampleCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

	connect(PointDownSampleForm.pushButtonDownSample, SIGNAL(clicked()), this, SLOT(DownSample()));
	connect(PointDownSampleForm.pushButtonRedo, SIGNAL(clicked()), this, SLOT(Redo()));

	widget->show();
}

CPointDownSample::~CPointDownSample()
{
	if (Viewer != NULL)
		Viewer->removePointCloud(DemoPointsStr);

	emitUpdateUI();
}

void CPointDownSample::Redo()
{
	CTreeBase::Redo();
}

void CPointDownSample::DownSample()
{
	float LeafX, LeafY, LeafZ;
	LeafX = PointDownSampleForm.doubleSpinBoxLeafSizeX->text().toDouble();
	LeafY = PointDownSampleForm.doubleSpinBoxLeafSizeY->text().toDouble();
	LeafZ = PointDownSampleForm.doubleSpinBoxLeafSizeZ->text().toDouble();

	int BeforeNum = InputCloud->points.size();

	DownSampleCloud->points.clear();
	DownSampleCloud->points.insert(DownSampleCloud->points.end(),
		InputCloud->points.begin(), InputCloud->points.end());
	
	InputCloud->points.clear();
	pcl::VoxelGrid<pcl::PointXYZRGB> VgDown;
	VgDown.setInputCloud(DownSampleCloud);
	VgDown.setLeafSize(LeafX, LeafY, LeafZ);
	VgDown.filter(*InputCloud);	

	string Hint = "Points Number before Downsample: " +
		StringBase::IntToStr(BeforeNum) + "; after: " +
		StringBase::IntToStr(InputCloud->points.size());

	emitUpdateStatusBar(Hint.c_str(), 5000);
	emitUpdateUI();
}