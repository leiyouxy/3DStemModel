#include "BranchesClassficationByDBScan.h"

void CBranchesClassficationByDBScan::RefreshData()
{
	CTreeBase::RefreshData();	
	DBScanTreeSkeleton.ResetSliceDBScanClusters();
}

void CBranchesClassficationByDBScan::ShowSkeleton(int State)
{	
	DBScanTreeSkeleton.ShowSkeleton(Form.checkBox_ShowSkeletonPoints->checkState() == 2,
		Form.checkBox_ShowTreeSkeleton->checkState() == 2);

	emitUpdateUI();
}

void CBranchesClassficationByDBScan::ShowShowNormal(int State)
{
	if (Form.checkBox_ShowNormal->checkState() == 2)
	{
		ShowNormals(InputCloud, DBScanTreeSkeleton.PointGeometry.Cloud_Normals, "NormalPointsStr");
	}
	else
		Viewer->removePointCloud("NormalPointsStr");

	emitUpdateUI();
}

//改变Slice Num时 SliceThickNess跟着调整
void CBranchesClassficationByDBScan::SliceNumChange(int Num)
{
	double TempThickNess;

	TempThickNess = abs(ZMax - ZMin) / Form.spinBoxSliceNum->text().toInt();
	Form.doubleSpinBoxSliceHeight->setValue(TempThickNess);
}

CBranchesClassficationByDBScan::
CBranchesClassficationByDBScan(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	Form.setupUi(widget);

	connect(Form.pushButton_MainTreeBranchesDetection, SIGNAL(clicked()), this, SLOT(MainTreeBranchesDetection()));

	connect(Form.pushButton_SliceDBScanCluster, SIGNAL(clicked()), this, SLOT(GetSlicesClusters()));

	connect(Form.pushButton_ClusterMerge, SIGNAL(clicked()), this, SLOT(SkeletonGenerate()));

	connect(Form.pushButton_Redo, SIGNAL(clicked()), this, SLOT(Redo()));

	connect(Form.pushButton_MainTreeBranchesRetrieval, SIGNAL(clicked()), this, SLOT(RetrievalMainTree()));

	connect(Form.checkBox_ShowBranchesPoints, SIGNAL(stateChanged(int)), this, SLOT(ShowBranchesPoints(int)));

	connect(Form.doubleSpinBoxSliceHeight, SIGNAL(valueChanged(double)), this, SLOT(ReSlice(double)));

	connect(Form.checkBox_ShowSlicePoint, SIGNAL(stateChanged(int)), this, SLOT(ShowSlicePoint(int)));

	connect(Form.checkBox_ShowSliceClusters, SIGNAL(stateChanged(int)), this, SLOT(ShowSliceClusters(int)));

	connect(Form.checkBox_ShowSkeletonPoints, SIGNAL(stateChanged(int)), this, SLOT(ShowSkeleton(int)));	

	connect(Form.checkBox_ShowTreeSkeleton, SIGNAL(stateChanged(int)), this, SLOT(ShowSkeleton(int)));

	connect(Form.checkBox_ShowNormal, SIGNAL(stateChanged(int)), this, SLOT(ShowShowNormal(int)));
	
	connect(Form.spinBoxSliceNum, SIGNAL(valueChanged(int)), this, SLOT(SliceNumChange(int)));
	  
	connect(Form.spinBoxSuccessivePointNum, SIGNAL(valueChanged(int)), this, SLOT(SuccessiveNumChange(int)));
	
	connect(Form.doubleSpinBoxSuccessiveAngle, 
		SIGNAL(valueChanged(double)), this, SLOT(SuccessiveAngleChange(double)));	

	connect(Form.pushButton_OutliersDetection, SIGNAL(clicked()), this, SLOT(OutliersDetection()));

	connect(Form.pushButton_OutliersRemoval, SIGNAL(clicked()), this, SLOT(OutliersRemoval()));

	connect(Form.pushButton_SkeletonPointRepair, SIGNAL(clicked()), this, SLOT(SkeletonPointRepair()));
	
	connect(Form.pushButtonNormalCacluation, SIGNAL(clicked()), this, SLOT(NormalComputation()));
	
	NodePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());	
	OutliersPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	OctreeCloudSearch = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));

	widget->show();
}

void CBranchesClassficationByDBScan::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreePointsValue)
{
	CTreeBase::SetInputCloud(TreePointsValue);

	CalcSliceNumAndShow();
}

CBranchesClassficationByDBScan::~CBranchesClassficationByDBScan()
{
	if (Viewer != NULL)
	{
		Viewer->removePointCloud(MainBranchesPointsStr);
		Viewer->removePointCloud(SliceClusterPointsStr);
		Viewer->removePointCloud(NodePointsStr);
	}

	ResetSliceDBScanClusters();

	OctreeCloudSearch->deleteTree();
	delete OctreeCloudSearch;
	OctreeCloudSearch = NULL;

	NodePoints->points.clear();
	OutliersPoints->points.clear();
	emitUpdateUI();
}

void CBranchesClassficationByDBScan::MainTreeBranchesDetection()
{
	emitUpdateStatusBar("DBScan Cluster for Main Tree Branches Detection is working", 3000);	

	DBScanTreeSkeleton.SetInputCloud(InputCloud, Viewer);

	DBScanTreeSkeleton.SlicesClusters(
		Form.doubleSpinBoxSliceHeightForBranches->text().toDouble(),
		Form.doublespinBoxRadius->text().toDouble(),
		Form.spinBoxMinialPoints->text().toDouble(), false);
		
	DBScanTreeSkeleton.MainTreeBranchesDetection(		
		Form.doublespinBoxRadius->text().toDouble(),
		Form.spinBoxMinialPoints->text().toDouble());
	//cout << "CalculateMainTree end" << endl;

	//DBScanTreeSkeleton.ShowClusters(Form.checkBox_ShowBranchesPoints->checkState() == 2);

	/*if (Viewer != NULL)
	{
		Viewer->removePointCloud(MainBranchesPointsStr);
	}
		
	DBScanCluster.SetInputCloud(InputCloud);
	DBScanCluster.RunCluster(Form.doublespinBoxRadius->text().toDouble(),
		Form.spinBoxMinialPoints->text().toDouble());

	if (DBScanCluster.GetClusterNumbers() > 0 &&
		Form.checkBox_Show_MainCluster->checkState() == 2)
	{
		DBScanCluster.SetClusterColors();
		DBScanCluster.ShowCluster(Viewer, PointSize, MainBranchesPointsStr);
	}*/

	//DBScanCluster.SetInputCloud(InputCloud);
	//DBScanCluster.RunClusterParallel(Form.doublespinBoxRadius->text().toDouble(),
	//		Form.spinBoxMinialPoints->text().toDouble());
	//DBScanCluster.ShowCluster(Viewer, PointSize, MainBranchesPointsStr);

	emitUpdateUI();
	emitUpdateStatusBar("DBScan Cluster for Main Tree Branches Detection has been finished!", 3000);
}

void CBranchesClassficationByDBScan::Redo()
{
	DBScanTreeSkeleton.ResetSliceDBScanClusters();
	DBScanTreeSkeleton.ShowSkeleton(false, false);
	
	emitUpdateUI();
}

//Retrie Top N Main branches points
void CBranchesClassficationByDBScan::RetrievalMainTree()
{
	emitUpdateStatusBar("Main Tree Branches Retrieval is working!", 0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreePoints(new pcl::PointCloud<pcl::PointXYZRGB>());	

	DBScanTreeSkeleton.MainTreeBranchesRetrieval(TreePoints, Form.spinBoxMinialPoints_TopClusterN->text().toInt());

	InputCloud->points.clear();

	Form.checkBox_ShowBranchesPoints->setChecked(false);
	DBScanTreeSkeleton.ShowSliceClusters(false);

	InputCloud->points.insert(InputCloud->points.begin(), 
		TreePoints->points.begin(), TreePoints->points.end());



	//有可能

	/*if (Viewer != NULL)
		Viewer->removePointCloud(MainBranchesPointsStr);

	vector<int> IndexS;
	DBScanCluster.GetMainCluster(IndexS);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreePoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = 0; i < IndexS.size(); i++)
	{
		TreePoints->points.push_back(InputCloud->points[IndexS[i]]);
	}

	InputCloud->points.clear();
	InputCloud->points.insert(InputCloud->points.begin(), TreePoints->points.begin(), TreePoints->points.end());
	*/

	emitUpdateUI();
	cout << "当前点数:" << InputCloud->points.size() << endl;
	emitUpdateStatusBar("Main Tree Branches Retrieval has been finished!", 3000);
}

void CBranchesClassficationByDBScan::ShowBranchesPoints(int State)
{
	DBScanTreeSkeleton.ShowSliceClusters(true, Form.checkBox_ShowBranchesPoints->checkState() == 2);
	emitUpdateUI();
}
//
//void CBranchesClassficationByDBScan::ShowMainClusterPoints(int State)
//{
//	if (DBScanCluster.GetClusterNumbers() > 0 &&
//		Form.checkBox_Show_MainBranches->checkState() == 2)
//	{
//		DBScanCluster.SetClusterColors();
//		DBScanCluster.ShowCluster(Viewer, PointSize, MainBranchesPointsStr);
//	}
//	else if (Form.checkBox_Show_MainBranches->checkState() != 2)
//	{
//		Viewer->removePointCloud(MainBranchesPointsStr);
//	}
//}

void CBranchesClassficationByDBScan::ShowSlicePoint(int State)
{
	if (Form.checkBox_ShowSlicePoint->checkState() == 2)
	{
		HorizontalPartition.SetViewer(Viewer);
		HorizontalPartition.p_TreePclQtGui = p_TreePclQtGui;
		HorizontalPartition.SetInputCloud(InputCloud);
		HorizontalPartition.SetThickNess(
			Form.doubleSpinBoxSliceHeight->text().toDouble());
		HorizontalPartition.PatitionSection();
		HorizontalPartition.ShowSectionPoints(0, HorizontalPartition.SectionsCount);		
	}
	else if (Form.checkBox_ShowSlicePoint->checkState() != 2)
	{
		HorizontalPartition.UnShowSectionPoints();
	}
}

void CBranchesClassficationByDBScan::ResetSliceDBScanClusters()
{
	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		delete SliceDBScanClusters[i].pCluster;
		SliceDBScanClusters[i].pCluster = NULL;
		SliceDBScanClusters[i].SectionPoints->points.clear();
	}
	SliceDBScanClusters.clear();
}

//Calc SliceNum  by thickness and show in form
void CBranchesClassficationByDBScan::CalcSliceNumAndShow()
{
	double SliceNum = 0;

	if (abs(ZMax - ZMin) < Form.doubleSpinBoxSliceHeight->text().toDouble())
		SliceNum = 1;
	else
		SliceNum = abs(ZMax - ZMin) / Form.doubleSpinBoxSliceHeight->text().toDouble();

	disconnect(Form.spinBoxSliceNum, SIGNAL(valueChanged(int)), this, SLOT(SliceNumChange(int)));

	Form.spinBoxSliceNum->setValue(ceil(SliceNum));

	connect(Form.spinBoxSliceNum, SIGNAL(valueChanged(int)), this, SLOT(SliceNumChange(int)));
}

void CBranchesClassficationByDBScan::SlicesCluster()
{
	//原始数据划分为Slice后的聚类
	HorizontalPartition.SetInputCloud(InputCloud);
	SliceHeight =
		Form.doubleSpinBoxSliceHeight->text().toDouble();
	HorizontalPartition.SetThickNess(SliceHeight);
	HorizontalPartition.PatitionSection();
	cout << "asdfasdf" << endl;
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		SliceCluster TempSliceCluster;
		TempSliceCluster.pCluster = new CDBScanCluster();
		TempSliceCluster.SectionPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
		HorizontalPartition.GetSectionPoints(i, TempSliceCluster.SectionPoints);
		TempSliceCluster.pCluster->SetInputCloud(TempSliceCluster.SectionPoints);
		SliceDBScanClusters.push_back(TempSliceCluster);
	}
	
	//此处并行计算 2020.02.18
	//#pragma omp parallel for
	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		SliceDBScanClusters[i].pCluster->RunCluster(
			Form.doublespinBoxRadiusForSlice->text().toDouble(),
			Form.spinBoxMinialPointsForSlice->text().toDouble());
		SliceDBScanClusters[i].pCluster->SetClusterColors();
		//计算每个簇的质心点和最大变动方向，
		SliceDBScanClusters[i].pCluster->CalcClusterParameters();
		
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.clear();
			for (int k = 0; k < SliceDBScanClusters[i].pCluster->Clusters[j].Indexs.size(); k++)
			{
				int TempSectionIndex = SliceDBScanClusters[i].pCluster->Clusters[j].Indexs[k];
				int TempGlobalIndex = HorizontalPartition.SectionsVector[i].Indexs[TempSectionIndex];
								
				SliceDBScanClusters[i].pCluster->Clusters[j].GlobalIndexs.push_back(TempGlobalIndex);
			}
		}
	}
}

//获取每一个Slice的聚类信息
void CBranchesClassficationByDBScan::GetSlicesClusters()
{
	emitUpdateStatusBar("Slice Cluster is working!", 3000);
	
	DBScanTreeSkeleton.SetInputCloud(InputCloud, Viewer);
	DBScanTreeSkeleton.SlicesClusters(
		Form.doubleSpinBoxSliceHeight->text().toDouble(),
		Form.doublespinBoxRadius->text().toDouble(),
		Form.spinBoxMinialPointsForSlice->text().toInt(), Form.checkBox_ShowSkeletonPoints->checkState() == 2);
	
	emitUpdateUI();
	emitUpdateStatusBar("Slice Cluster has been finished!", 3000);
}

void CBranchesClassficationByDBScan::ShowSliceClusters(int State)
{
	if (DBScanTreeSkeleton.GetSlicesClustersCount() == 0) return;

	DBScanTreeSkeleton.ShowSliceClusters(Form.checkBox_ShowSliceClusters->checkState() == 2, false, PointSize);
	
	emitUpdateUI();
}

//将多个Slice的结果相连接得到树木的骨架
void CBranchesClassficationByDBScan::SkeletonGenerate()
{
	emitUpdateStatusBar("Skeleton Generate is working! Please waiting!", 0);
	
	if (DBScanTreeSkeleton.GetSlicesClustersCount() == 0)
		GetSlicesClusters();

	DBScanTreeSkeleton.SkeletonGenerate(
		Form.doubleSpinBoxSliceHeight->text().toDouble(),
		Form.doublespinBoxRadius->text().toDouble(),
		Form.spinBoxMinialPointsForSlice->text().toInt(),
		Form.radioButtonConvexCentroid->isChecked());

	DBScanTreeSkeleton.ShowSkeleton(Form.checkBox_ShowSkeletonPoints->checkState() == 2,
		Form.checkBox_ShowTreeSkeleton->checkState() == 2);
	
	emitUpdateUI();
	emitUpdateStatusBar("Skeleton Generate has been finished!", 3000);
}

void CBranchesClassficationByDBScan::ReSlice(double Height)
{
	CalcSliceNumAndShow();

	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetThickNess(Form.doubleSpinBoxSliceHeight->text().toDouble());
	HorizontalPartition.PatitionSection();

	ResetSliceDBScanClusters();	
}

void CBranchesClassficationByDBScan::SuccessiveNumChange(int NumValue)
{
	//DBScanTreeSkeleton.SkeletonIsGenerate = false;
}

void CBranchesClassficationByDBScan::SuccessiveAngleChange(double AngleValue)
{
	//DBScanTreeSkeleton.SkeletonIsGenerate = false;
}

void CBranchesClassficationByDBScan::OutliersDetection()
{
	emitUpdateStatusBar("Outliers Detection is working! Please waiting!", 0);
	
	DBScanTreeSkeleton.SetInputCloud(InputCloud, Viewer);
	DBScanTreeSkeleton.OutliersDetection(Form.doubleSpinBoxSliceHeightForBranches->text().toDouble(),
		Form.doublespinBoxRadius->text().toDouble(),
		Form.spinBoxMinialPoints->text().toDouble());

	OutliersPoints->points.clear();
	for (int i = 0; i < DBScanTreeSkeleton.OutliersIndexs.size(); i++)
	{
		OutliersPoints->points.push_back(InputCloud->points[DBScanTreeSkeleton.OutliersIndexs[i]]);
	}	

	PointBase::SetPointColor(OutliersPoints, ColorBase::RedColor);
	ShowPoints(OutliersPoints, OutliersPointsStr, PointSize);
	
	emitUpdateUI();
	emitUpdateStatusBar("Outliers Detection has been finished!", 3000);
}

void CBranchesClassficationByDBScan::OutliersRemoval()
{
	emitUpdateStatusBar("Outliers Removal is working! Please waiting!", 0);

	Viewer->removePointCloud(OutliersPointsStr);
	DBScanTreeSkeleton.OutliersRemoval();	

	emitUpdateUI();
	emitUpdateStatusBar("Outliers Removal has been finished!", 3000);
}

//根据树干的连续性修改树干骨架中的点，并明确每个点所在树干的层次关系
void CBranchesClassficationByDBScan::SkeletonPointRepair()
{
	emitUpdateStatusBar("Skeleton Point Repair is working! Please waiting!", 0);
	
	if (!DBScanTreeSkeleton.SkeletonIsGenerate)
		SkeletonGenerate();

	DBScanTreeSkeleton.SkeletonPointRepair(
		Form.doubleSpinBoxSliceHeight->text().toDouble(),
		Form.spinBoxSuccessivePointNum->text().toInt(), 
		Form.doubleSpinBoxSuccessiveAngle->text().toDouble(),
		Form.doubleSpinBoxAllowDis->text().toDouble());

	ShowSkeleton(2);

	emitUpdateUI();
	emitUpdateStatusBar("Skeleton Point Repair has been finished!", 3000);
}

void CBranchesClassficationByDBScan::NormalComputation()
{	
	emitUpdateStatusBar("Normal Computation is working! Please waiting!", 0);
	if (DBScanTreeSkeleton.GetSlicesClustersCount() == 0)
		DBScanTreeSkeleton.SetInputCloud(InputCloud, Viewer);
	DBScanTreeSkeleton.NormalComputation();

	ShowShowNormal(0);

	emitUpdateUI();
	emitUpdateStatusBar("Normal Computation has been finished!", 3000);
}

////如果连续的 SuccessiveSliceNum Slice的聚类个数都为1，则这些被认为是树干点云
//int CBranchesClassficationByDBScan::FindStemSlice(int SuccessiveSliceNum)
//{
//	int FindIndex = -1;
//	int Count = 0;
//	for (int i = 0; i < SliceDBScanClusters.size(); i++)
//	{
//		if (SliceDBScanClusters[i].pCluster->Clusters.size() == 1)
//		{
//			Count++;
//		}
//		else Count = 0;
//
//		if (Count == SuccessiveSliceNum + 1)
//		{
//			FindIndex = i;
//			break;
//		}
//	}
//
//	for (int i = FindIndex; i >= FindIndex - SuccessiveSliceNum; i--)
//	{
//		SliceDBScanClusters[i].pCluster->Clusters[0].Label = "000";
//	}
//
//	return FindIndex - SuccessiveSliceNum;
//}

//根据Label的值Value获取最大的MaxID
int CBranchesClassficationByDBScan::FindMaxID(string Value)
{
	int MaxValue = 0;
	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			string TempLabel = SliceDBScanClusters[i].pCluster->Clusters[j].Label;

			if (TempLabel == "")
				continue;
			
			if ((Value.length() + 3 == TempLabel.length()) &&
				((TempLabel.substr(0, Value.length()) == Value)))
			{
				string TempStr = TempLabel.substr(Value.length(), 3);
				int TempInt = atoi(TempStr.c_str());

/*				cout << Value << ", Length:" << Value.length() + 3 <<
					", TempLabel:" << TempLabel << 
					", SubStr:" << TempLabel.substr(0, Value.length()) <<
					", TempStr:"<< TempStr<<endl;*/					
				
				if (TempInt > MaxValue)
				{
					MaxValue = TempInt;
				}				
			}
		}
	}

	return MaxValue;
}

void CBranchesClassficationByDBScan::SetColors()
{
	for (int i = 0; i < SliceDBScanClusters.size(); i++)
	{
		for (int j = 0; j < SliceDBScanClusters[i].pCluster->Clusters.size(); j++)
		{
			//cout << "Label:" << SliceDBScanClusters[i].pCluster->Clusters[j].Label << endl;
			//if (SliceDBScanClusters[i].pCluster->Clusters[j].Label == "000")
			if (SliceDBScanClusters[i].pCluster->Clusters[j].NodeRoles.ChildNodes.size() > 0)
			{
				for (int k = 0; k < SliceDBScanClusters[i].pCluster->Clusters[j].Indexs.size(); k++)
				{
					//HorizontalPartition.SectionsVector[i].Indexs
					//cout<<"Index"<< SliceDBScanClusters[i].pCluster->Clusters[j].Indexs[k] <<endl;
					InputCloud->points[
						HorizontalPartition.SectionsVector[i].Indexs[SliceDBScanClusters[i].pCluster->Clusters[j].Indexs[k]]].rgba =
						ColorBase::RedColor;
				}
			}
		}
	}
}