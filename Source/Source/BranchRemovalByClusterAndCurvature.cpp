#include "BranchRemovalByClusterAndCurvature.h"

CBranchRemovalByClusterAndCurvature::CBranchRemovalByClusterAndCurvature(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	Form.setupUi(widget);

	connect(Form.pushButton_DBScanforPointDistance, SIGNAL(clicked()), this, SLOT(DBClusterforPointDis()));
	connect(Form.checkBox_ShowK_DBSCluster, SIGNAL(stateChanged(int)), this, SLOT(ShowClusterPoints(int)));

	connect(Form.pushButton_MainTreeRetrieval, SIGNAL(clicked()), this, SLOT(RetrievalMainTree()));

	connect(Form.pushButtonNormalCacluation, SIGNAL(clicked()), this, SLOT(NormalComputation()));
	connect(Form.pushButtonRedo, SIGNAL(clicked()), this, SLOT(Redo()));

	connect(Form.checkBoxShowNormal, SIGNAL(stateChanged(int)), this, SLOT(ShowNormal(int)));
	
	connect(Form.pushButtonBranchCheck, SIGNAL(clicked()), this, SLOT(ButtonBranchCheck()));
	connect(Form.pushButtonRemoval, SIGNAL(clicked()), this, SLOT(RemoveBranches()));

	connect(Form.pushButtonNormalCheck, SIGNAL(clicked()), this, SLOT(NormalCheck()));

	widget->show();
}

CBranchRemovalByClusterAndCurvature::~CBranchRemovalByClusterAndCurvature()
{
	if (Viewer != NULL)
		Viewer->removePointCloud(PointClusterPointsStr);

	emitUpdateUI();
}

void CBranchRemovalByClusterAndCurvature::DBClusterforPointDis()
{
	emitUpdateStatusBar("DBScan Cluster is working", 3000);

	NormalComputation();

	if (Viewer != NULL)
		Viewer->removePointCloud(PointClusterPointsStr);

	//DBScanCluster.SetInputCloud(InputCloud);
	//DBScanCluster.RunCluster(Form.spinBoxRadius->text().toDouble(),
	//	Form.spinBoxMinialPoints->text().toDouble());
	
	/*
	CHorizontalPartition HorizontalPartition;
	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetParameters(Form.doubleSpinBoSliceHeight->text().toDouble());
	HorizontalPartition.PatitionSection();

	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		vector<int>	RedIndexs, BlueIndexS;
		PointGeometry.SearchCurvaturePathPoints(HorizontalPartition.SectionsVector[i].Indexs,
			RedIndexs, BlueIndexS);

		for (int i = 0; i < RedIndexs.size(); i++)
		{
			InputCloud->points[RedIndexs[i]].rgba = ColorBase::RedColor;
		}

		for (int i = 0; i < BlueIndexS.size(); i++)
		{
			InputCloud->points[BlueIndexS[i]].rgba = ColorBase::BlueColor;
		}
	}
	//*/

	CDBScanClusterWithCurvatrue TempDBScanCluster;
	TempDBScanCluster.SetInputCloud(InputCloud);
	TempDBScanCluster.RunClusterWithSliceCurvature(
		Form.doubleSpinBoSliceHeight->text().toDouble(),
		Form.doublespinBoxRadius->text().toDouble(),
		Form.spinBoxMinialPoints->text().toDouble(), &PointGeometry);
	TempDBScanCluster.SetClusterColors();
	TempDBScanCluster.ShowCluster(Viewer, PointSize, PointClusterPointsStr);

	//
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempInputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	////对每一个进行先DBSCan距离聚类，然后根据DBSCan曲率聚类
	//for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	//{
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionDisMainPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionCurvatureMainPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	//	HorizontalPartition.GetSectionPoints(i, SectionPoints);

	//	CDBScanClusterWithCurvatrue TempDBScanCluster;

	//	TempDBScanCluster.SetInputCloud(SectionPoints);
	//	TempDBScanCluster.RunCluster(Form.spinBoxRadius->text().toDouble(),
	//		Form.spinBoxMinialPoints->text().toDouble());
	//	
	//	TempDBScanCluster.GetClusterPoints(TempDBScanCluster.GetMainClusterIndex(), SectionDisMainPoints);

	//	if (i == 0)
	//	{
	//		TempDBScanCluster.GetClusterPoints(TempDBScanCluster.GetMainClusterIndex(), SectionDisMainPoints);

	//		TempInputCloud->points.insert(TempInputCloud->points.begin(),
	//			SectionDisMainPoints->points.begin(),
	//			SectionDisMainPoints->points.end());
	//	}
	//}

	//DBScanClusterWithSlice.SetInputCloud(InputCloud);
	//DBScanClusterWithSlice.RunSliceCluster(Form.doubleSpinBoSliceHeight->text().toDouble(),
	//	Form.doublespinBoxRadius->text().toDouble(), Form.spinBoxMinialPoints->text().toDouble());	

	//DBScanClusterWithSlice.SetColorsByID();
	//DBScanClusterWithSlice.ShowCluster(Viewer, PointSize, PointClusterPointsStr);

	//ShowClusterPoints(Form.checkBox_ShowK_DBSCluster->checkState());
	emitUpdateUI();
	emitUpdateStatusBar("DBScan Cluster has been finished!", 3000);
}

void CBranchRemovalByClusterAndCurvature::ShowClusterPoints(int State)
{
	//if (DBScanClusterWithSlice.GetClusterNumbers() > 0 
	//  && Form.checkBox_ShowK_DBSCluster->checkState() == 2)
	//{
	//	DBScanClusterWithSlice.SetColorsByID();
	//	DBScanClusterWithSlice.ShowCluster(Viewer, PointSize, PointClusterPointsStr);

	//	//DBScanCluster.SetClusterColors();
	//	//DBScanCluster.ShowCluster(this->Viewer, PointSize, PointClusterPointsStr);		
	//}
	//else if (Form.checkBox_ShowK_DBSCluster->checkState() != 2)
	//{
	//	Viewer->removePointCloud(PointClusterPointsStr);
	//}
	emitUpdateUI();
}

void CBranchRemovalByClusterAndCurvature::RefreshData()
{
	CTreeBase::RefreshData();
	DBScanCluster.SetInputCloud(InputCloud);
}

void CBranchRemovalByClusterAndCurvature::RetrievalMainTree()
{
	emitUpdateStatusBar("Retrieval is working!", 3000);
	if (Viewer != NULL)
		Viewer->removePointCloud(PointClusterPointsStr);

	vector<int> IndexS;
	DBScanCluster.GetMainCluster(IndexS);	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreePoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = 0; i < IndexS.size(); i++)
	{
		TreePoints->points.push_back(InputCloud->points[IndexS[i]]);
	}

	InputCloud->points.clear();
	InputCloud->points.insert(InputCloud->points.begin(), TreePoints->points.begin(), TreePoints->points.end());
	emitUpdateUI();
	cout<<"当前点数:"<< InputCloud->points.size()<<endl;
	emitUpdateStatusBar("Retrieval has been finished!", 3000);
}

void CBranchRemovalByClusterAndCurvature::NormalComputation()
{
	emitUpdateStatusBar("Normal Computation is working!", 3000);
	//if (InputCloud->points.size() != PointGeometry.Cloud_Curvatures->points.size())
	if (InputCloud->points.size() != PointGeometry.Cloud_Normals->points.size())
	{
		if (Form.radioButton_Radius->isChecked() == 1)
		{
			PointGeometry.SetInputCloud(InputCloud);
			PointGeometry.CalcNormalAndCurvatureRadius(
				Form.doubleSpinBox_RadiusForNormalEstimation->text().toDouble(),
				Form.checkBox_Weighted->isChecked() == 1);
			//cout << "法向一致化" << endl;
			//PointGeometry.SetNormalConsistency(NormalAnchorIndex, NormalAnchorPoint,
			//	BranchRemovalForm.doubleSpinBox_RadiusForNormalEstimation->text().toDouble());
			//cout << "计算获取主曲率方向" << endl;
			//PointGeometry.GetDirectionOfPrincipalCurvatures(CurvatureDirections);
		}
		else if (Form.radioButton_OVR->isChecked() == 1)
		{
			PointGeometry.SetInputCloud(InputCloud);
			PointGeometry.CalcOptimalK();
			PointGeometry.CalcOptimalNormalWithMore(Form.checkBox_Weighted->isChecked() == 1);
			
			//使法向指向一致
			//PointGeometry.SetNormalConsistency(NormalAnchorIndex, NormalAnchorPoint);
			//PointGeometry.CalcOptimalCurvature();
			//PointGeometry.GetDirectionOfPrincipalCurvatures(CurvatureDirections);
		}
	}
	
	//vector<int> IndexS;
	//PointGeometry.GetCurvatureVariationSignificantly(IndexS);
	//cout << "曲率值大的点的个数：" << IndexS.size()<<endl;
	//for (int i = 0; i < IndexS.size(); i++)
	//{
	//	InputCloud->points[IndexS[i]].rgba = ColorBase::RedColor;
	//}

	//VectorBase<double> VectorBaseDouble;
	//vector<double> CurvatureVec;
	//for (int i = 0; i < InputCloud->points.size(); i++)
	//{
	//	CurvatureVec.push_back(PointGeometry.GetGuassCurvature(i));
	//}
	//double Mean, Std;
	//Std = sqrt(VectorBaseDouble.CalcVariances(CurvatureVec, Mean));

	//cout << "Global Mean:" << Mean << endl;
	//cout << "Global Std:" << Std << endl;
	//
	//MeanThresold = Mean + 3 * Std;

	double CurvatureValue = Form.doubleSpinBoxCurvature->text().toDouble();

	//vector<int> CurvatureBigIndexs;

	//for (int i = 0; i < InputCloud->points.size(); i++)
	//{			
	//	//平均法向与法向的夹角
	//	double Angle = GeometryBase::RadianToAngle(
	//		GeometryBase::AngleOfTwoVector(
	//			PointGeometry.AvgNormalOfNeighbour->points[i],
	//			PointGeometry.Cloud_Normals->points[i]));
	//	if (Angle > 90) Angle = 180 - Angle;		

	//	//if (Angle > Form.doubleSpinBoxAngle->text().toDouble())
	//	//	InputCloud->points[i].rgba = ColorBase::BlueColor;

	//	if (CurvatureVec[i] > Mean + 3 * Std)
	//	{
	//		InputCloud->points[i].rgba = ColorBase::RedColor;
	//		cout << "法向与平均法向夹角：" << Angle << endl;

	//		CurvatureBigIndexs.push_back(i);
	//	}
	//}
	/*
	vector<int> SliceIndexs, RedIndexs, BlueIndexS;
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		SliceIndexs.push_back(i);
	}

	PointGeometry.SearchCurvaturePathPoints(SliceIndexs, RedIndexs, BlueIndexS);
	
	for (int i = 0; i < RedIndexs.size(); i++)
	{
		InputCloud->points[RedIndexs[i]].rgba = ColorBase::RedColor;
	}
	
	for (int i = 0; i < BlueIndexS.size(); i++)
	{
		InputCloud->points[BlueIndexS[i]].rgba = ColorBase::BlueColor;
	}
	*/
	ShowNormal(0);
	emitUpdateUI();
	emitUpdateStatusBar("Normal Computation has been finished!", 3000);
}

void CBranchRemovalByClusterAndCurvature::ShowNormal(int State)
{
	if (Form.checkBoxShowNormal->checkState() == 2 && 
		InputCloud->points.size() != PointGeometry.Cloud_Normals->points.size())
	{
		NormalComputation();
	}

	if (Form.checkBoxShowNormal->checkState() == 2)
		ShowNormals(InputCloud, PointGeometry.Cloud_Normals, "_Cloud_Normals", 1, 1);
	else
		Viewer->removePointCloud("_Cloud_Normals");	
}

void CBranchRemovalByClusterAndCurvature::ButtonBranchCheck()
{
	NormalComputation();
	emitUpdateStatusBar("Branch Checking is working!", 3000);	
	
	/* 
	//CDBScanClusterWithCurvatrue DBScanClusterWithCurvatrue;
	DBScanCluster.SetInputCloud(InputCloud);
	DBScanCluster.RunClusterWithCurvature(2, 2, &PointGeometry, MeanThresold);
	
	//DBScanCluster.ClusterMerge(Form.doubleSpinBoxAngle->text().toDouble());

	DBScanCluster.SetClusterColors();

	DBScanCluster.ShowCluster(Viewer, PointSize, PointClusterPointsStr);
	*/

	emitUpdateStatusBar("Branch Checking has been finished!", 3000);
}

void CBranchRemovalByClusterAndCurvature::RemoveBranches()
{
	emitUpdateStatusBar("Branch Removing is working!", 3000);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		if (InputCloud->points[i].rgba != ColorBase::RedColor)
		{
			TempPoints->points.push_back(InputCloud->points[i]);
		}
	}

	InputCloud->points.clear();
	InputCloud->points.insert(InputCloud->points.end(),
		TempPoints->points.begin(), TempPoints->points.end());
	TempPoints->points.clear();

	emitUpdateStatusBar("Branches points have been removed!", 5000);
	emitUpdateUI();
	RefreshData();
}

void CBranchRemovalByClusterAndCurvature::NormalCheck()
{
	NormalComputation();

	emitUpdateStatusBar("Normal Angle is checking!", 3000);
	double Value = Form.dsbNormalAngle->text().toDouble();	//角度
	Value = abs(cos(M_PI * Value / 180.0));	//到弧度，到COS
	cout << "Value" << Value << endl;
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		double Cos0 = GeometryBase::CosOfTwoVector(PointGeometry.Cloud_Normals->points[i],
			PointGeometry.DoubleCloud_Normals->points[i]);

		double Cos1 = GeometryBase::CosOfTwoVector(PointGeometry.Cloud_Normals->points[i],
			PointGeometry.HalfCloud_Normals->points[i]);

		//if (abs(Cos0 * Cos1) < Value)
		if (abs(Cos0) < Value || abs(Cos1) < Value)
		{
			//cout << "Cos0:" << Cos0 << ",Cos1:" << Cos1 << ", abs(Cos0 * Cos1):" << abs(Cos0 * Cos1) << endl;
			InputCloud->points[i].rgba = ColorBase::RedColor;
		}
	}
	int BranchCount = 0;
	//根据邻域点是否是树干点 投票判定当前点是不是 树干点
	for (int i = 0; i < InputCloud->points.size(); i++)
	{		
		if (InputCloud->points[i].rgba != ColorBase::RedColor)
		{
			int Count = 0;
			vector<int> NeighbourIndexs;
			PointGeometry.GetOptimalNeighbourIndex(i, NeighbourIndexs);
			//cout << "NeighbourIndexs i:" << NeighbourIndexs.size() << endl;

			for (int j = 0; j < NeighbourIndexs.size(); j++)
			{
				if (InputCloud->points[NeighbourIndexs[j]].rgba == ColorBase::RedColor)
				{
					Count++;
				}
			}
			double TempValue = Count * 1.0 / NeighbourIndexs.size();
			//如果周围1半的点都是 树杈点，则也认为是树杈点
			if (TempValue >= 0.5)
			{
				cout<<"TempValue: "<< TempValue <<", Branches i:"<<i<<endl;
				InputCloud->points[i].rgba = ColorBase::YellowColor;
				BranchCount++;
			}
		}		
	}
	cout << "BranchCount: " << BranchCount << endl;

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		if (InputCloud->points[i].rgba == ColorBase::RedColor)
		{
			int Count = 0;
			vector<int> NeighbourIndexs;
			PointGeometry.GetOptimalNeighbourIndex(i, NeighbourIndexs);

			for (int j = 0; j < NeighbourIndexs.size(); j++)
			{
				if (InputCloud->points[NeighbourIndexs[j]].rgba != ColorBase::RedColor
					&& InputCloud->points[NeighbourIndexs[j]].rgba != ColorBase::YellowColor)
				{
					Count++;
				}
			}
			//如果周围1半的点都是 Stem，则也认为是 Stem
			if (Count * 1.0 / NeighbourIndexs.size() > 0.5)
			{
				cout << "Stem i:" << i << endl;
				InputCloud->points[i].rgba = ColorBase::BlueColor;
			}
		}
	}

	emitUpdateStatusBar("Normal Angle Check has been Finished!", 3000);
	emitUpdateUI();
	RefreshData();
}


