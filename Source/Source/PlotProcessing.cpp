#include "PlotProcessing.h"

void CPlotProcessing::RefreshData()
{
	CTreeBase::RefreshData();
}

//将整个Plot点云垂直分段，并逐段保存，然后选择一个较好的段来寻找树木 2021.01.24
void CPlotProcessing::PlotVecSlice()
{
	CHorizontalPartition HorizontalPartition;
	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SectionThickness = PlotProcessingForm.doubleSpinBoxPartDistance->text().toDouble();
	HorizontalPartition.PatitionSection();

	HorizontalPartition.SaveSectionSToEachFile("J:\\TempOut\\" + OpenedFileName + "_", 
		0, HorizontalPartition.SectionsCount);
	cout<<"处理完毕！"<<endl;
}

//对某一个分段进行DBScan聚类，并输出其的中心位置 2020.01.25
void CPlotProcessing::ClusterForASlice()
{
	CDBScanCluster DBScanCluster;
	PointBase::SetPointsCoordinateValue(InputCloud, "Z", InputCloud->points[0].z);
	DBScanCluster.SetInputCloud(InputCloud);	
	DBScanCluster.RunCluster(
		PlotProcessingForm.doubleSpinBoxNeighbourDis->text().toDouble(),
		PlotProcessingForm.doubleSpinBoxNeighbourCount->text().toDouble(),
		true);
	DBScanCluster.CalcClusterParameters(true);	
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeometryCenters(new pcl::PointCloud<pcl::PointXYZRGB>());

	DBScanCluster.SetClusterColorsByDisVariance(
		PlotProcessingForm.doubleSpinBoxPartMinMean->text().toDouble(),
		PlotProcessingForm.doubleSpinBoxPartMaxMean->text().toDouble(),
		PlotProcessingForm.doubleSpinBoxPartVariance->text().toDouble(), GeometryCenters);

	if (GeometryCenters->points.size() > 0)
	{
		PointBase::SavePCDToFileName(GeometryCenters, "J:\\"+ OpenedFileName + "_PlotTreeCoordinates.pcd");
	}

	DBScanCluster.ShowCluster(Viewer, 2);
	emitUpdateUI();
}

CPlotProcessing::CPlotProcessing(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	PlotProcessingForm.setupUi(widget);

	TreeCoordinatePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	TreeConvexHullPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	PlotTreePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

	connect(PlotProcessingForm.pushButtonOpenCoordinates, 
		SIGNAL(clicked()), this, SLOT(OpenCoordinateFile()));

	connect(PlotProcessingForm.pushButtonVerSlice,
		SIGNAL(clicked()), this, SLOT(PlotVecSlice()));
	
	connect(PlotProcessingForm.pushButtonTreeCluster,
		SIGNAL(clicked()), this, SLOT(ClusterForASlice()));
	

	widget->show();
}

CPlotProcessing::~CPlotProcessing()
{
	if (Viewer != NULL)
		Viewer->removePointCloud(PlotProcessingPointsStr);

	emitUpdateUI();
}


void CPlotProcessing::OpenCoordinateFile()
{
	cout<<"FileName:"<< OpenedFileName <<endl;
	QString CurFileName;
	string TempOpenedFilePath;

	if (CurFileName.size() == 0)
		CurFileName = QFileDialog::getOpenFileName(NULL,
			tr("Point Cloud Files"), 
			TempOpenedFilePath.c_str(), tr("Point Cloud Files (*.pcd *.las *.xyz *.vtx *.ply)"));

	if (CurFileName.length() <= 0)
		return;

	TempOpenedFilePath = string((const char *)CurFileName.toLocal8Bit());
	PointBase::OpenPCLFile(TempOpenedFilePath, TreeCoordinatePoints, false);
	//PointBase::PointZoom(TreeCoordinatePoints, 100);
	
	cout << "TreeCoordinatePoints:" << TempOpenedFilePath << endl;

	for (int i = 0; i < TreeCoordinatePoints->points.size(); i++)
	{
		TreeStruct TempTree;
		TempTree.TreeID = TreeCoordinatePoints->points[i].z;		
		TreeCoordinatePoints->points[i].z = 0;
		TempTree.CenterPoint = TreeCoordinatePoints->points[i]; 
		TempTree.TreeClouds.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

		TreeStructs.push_back(TempTree);
	}
	ShowPoints(TreeCoordinatePoints, PlotProcessingPointsStr);
	cout<<"树木位置初始化完毕！"<<endl;
	/*
	CContourAndConvexHull<pcl::PointXYZRGB> ConvexHull;

	ConvexHull.SetInputs(TreeCoordinatePoints);
	ConvexHullIndexs.clear();
	ConvexHull.GetPointsConvexHull(ConvexHullIndexs);

	TreeConvexHullPoints->points.clear();
	ConvexHullCentroidPoint.x = 0, ConvexHullCentroidPoint.y = 0, ConvexHullCentroidPoint.z = 0;
	//if (PlotProcessingForm.checkShowConvexHull->checkState())
	{
		for (int i = 0; i < ConvexHullIndexs.size(); i++)
		{
			pcl::PointXYZRGB Point1, Point2;

			Point1.x = TreeCoordinatePoints->points[ConvexHullIndexs[i]].x;
			Point1.y = TreeCoordinatePoints->points[ConvexHullIndexs[i]].y;
			Point1.z = 0;
			
			Point2.x = TreeCoordinatePoints->points[ConvexHullIndexs[(i + 1) % ConvexHullIndexs.size()]].x;
			Point2.y = TreeCoordinatePoints->points[ConvexHullIndexs[(i + 1) % ConvexHullIndexs.size()]].y;
			Point2.z = 0;

			if (Point1.x >= 0) 
				Point1.x += 300;
			else 
				Point1.x -= 300;

			if (Point1.y >= 0)
				Point1.y += 300;
			else
				Point1.y -= 300;

			if (Point2.x >= 0)
				Point2.x += 300;
			else
				Point2.x -= 300;

			if (Point2.y >= 0)
				Point2.y += 300;
			else
				Point2.y -= 300;

			//计算质心点
			ConvexHullCentroidPoint.x += Point1.x / ConvexHullIndexs.size(), 
				ConvexHullCentroidPoint.y += Point1.y / ConvexHullIndexs.size(), 
				ConvexHullCentroidPoint.z += Point1.z / ConvexHullIndexs.size();
			TreeConvexHullPoints->points.push_back(Point1);

			if (PlotProcessingForm.checkShowConvexHull->checkState())
				Viewer->addLine(Point1, Point2, 255, 0, 0, StringBase::IntToStr(i));
		}
	}
	emitUpdateUI();
	//*/

	PlotTreePoints->points.clear();	
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		#pragma omp parallel for
		for (int j = 0; j < TreeStructs.size(); j++)
		{
			double TempDis = PointDis(InputCloud->points[i], TreeStructs[j].CenterPoint, true);

			if (TempDis <= PlotProcessingForm.doubleSpinBoxDistance->text().toDouble() * 100)
			{
				TreeStructs[j].TreeClouds->points.push_back(InputCloud->points[i]);
			}
		}

		if (i % 1000000 == 0)
		{
			cout<< "Processing data after " + StringBase::IntToStr(i) <<endl;
		}
	}

	#pragma omp parallel for
	for (int j = 0; j < TreeStructs.size(); j++)
	{
		if (TreeStructs[j].TreeClouds->points.size() > 0)
		{
			string TempOutFileName = "J:\\TempOut\\" + OpenedFileName + "_" + StringBase::IntToStr(TreeStructs[j].TreeID) + "_" +
				StringBase::FloatToStr(PlotProcessingForm.doubleSpinBoxDistance->text().toDouble() * 100) + ".pcd";
			PointBase::SavePCDToFileName(TreeStructs[j].TreeClouds, TempOutFileName);
			cout<<"OutPutFileName: J"<<j <<","<< TempOutFileName <<endl;
		}
		else
			cout << "当前没有找到合适的点: J" << j << endl;

		TreeStructs[j].TreeClouds->points.clear();
		TreeStructs[j].TreeClouds.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	}

	cout << "Processing has done" <<endl;
	emitUpdateUI();
}

//点是否在多边形内部，
bool CPlotProcessing::CurrentPointIsInConvexHull(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreeCoordinatePoints, 
	pcl::PointXYZRGB CurPoint)
{
	bool IsIn = false;

	for (int i = 0; i < TreeCoordinatePoints->points.size(); i++)
	{
		pcl::PointXYZRGB FirstPoint = TreeCoordinatePoints->points[i];

		pcl::PointXYZRGB SecondPoint = TreeCoordinatePoints->points[(i + 1) % 
			TreeCoordinatePoints->points.size()];

		int InValue = PointBase::PointIsInTriange2D(FirstPoint, SecondPoint,
			ConvexHullCentroidPoint, CurPoint);

		if (InValue != -1) //只有该点位于一个三角形的外部，则视为需要增加凸包
		{
			IsIn = true;
			break;
		}
	}

	return IsIn;
}