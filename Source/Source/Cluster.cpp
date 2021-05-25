#include "Cluster.h"

void CCluster::RefreshData()
{
	CTreeBase::RefreshData();	
}

void CCluster::RunK_MeansCluster()
{
	emitUpdateStatusBar("K_Means Cluster is working", 3000);	
	K_MeansCluster.SetInput(InputCloud);
	K_MeansCluster.RunCluster(ClusterForm.spinBoxClusterNumber->text().toDouble(),
		ClusterForm.doubleSpinBoxThresold->text().toDouble());
	K_MeansCluster.SetClusterColors();

	for (int i = 0; i < K_MeansCluster.GetClusterNumbers(); i++)
	{
		cout<<"第"<<i<<"个聚类的中心点："<<K_MeansCluster.GetClusterInfo(i).ClusterCenter<<endl;
	}

	if (ClusterForm.checkBox_ShowK_MenasCluster->checkState() == 2)
	{
		K_MeansCluster.ShowCluster(this->Viewer, PointSize);
		emitUpdateUI();
	}
	emitUpdateStatusBar("K_Means Cluster has been finished", 3000);
}

void CCluster::DBScan()
{
	emitUpdateStatusBar("DBScan Cluster is working", 3000);
	
	DBScanCluster.SetInputCloud(InputCloud);
	DBScanCluster.RunCluster(ClusterForm.spinBoxRadius->text().toDouble(), 
		ClusterForm.spinBoxMinialPoints->text().toDouble());
	DBScanCluster.SetClusterColors();
	if (ClusterForm.checkBox_ShowK_DBSCluster->checkState() == 2)
	{
		DBScanCluster.ShowCluster(this->Viewer, PointSize);
		emitUpdateUI();
	}
	emitUpdateStatusBar("DBScan Cluster has been finished", 3000);
}

void CCluster::SHowClusterPoints(int CheckStatu)
{
	QAction * Action = (QAction *)sender();
	if (Action == NULL) return;

	if (Action->objectName() == "checkBox_ShowK_DBSCluster")
	{
		DBScanCluster.ShowCluster(this->Viewer, PointSize);
	}
	else if (Action->objectName() == "checkBox_ShowK_MenasCluster")
	{
		K_MeansCluster.ShowCluster(this->Viewer, PointSize);
	}
	emitUpdateUI();
}

CCluster::CCluster(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	ClusterForm.setupUi(widget);

	widget->show();

	connect(ClusterForm.pushButtonK_Means, SIGNAL(clicked()), this, SLOT(RunK_MeansCluster()));
	connect(ClusterForm.pushButtonDBScan, SIGNAL(clicked()), this, SLOT(DBScan()));
	connect(ClusterForm.pushButton_Redo, SIGNAL(clicked()), this, SLOT(Redo()));

	connect(ClusterForm.checkBox_ShowK_DBSCluster, SIGNAL(stateChanged(int)), this, SLOT(SHowClusterPoints(int)));
	connect(ClusterForm.checkBox_ShowK_MenasCluster, SIGNAL(stateChanged(int)), this, SLOT(SHowClusterPoints(int)));
}

CCluster::~CCluster()
{
	if (Viewer != NULL)
		Viewer->removePointCloud(DemoPointsStr);

	emitUpdateUI();
}