#include "BranchRemovalByTangentPlane.h"

CBranchRemovalByTangentPlane::CBranchRemovalByTangentPlane(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	BranchRemovalForm.setupUi(widget);

	connect(BranchRemovalForm.checkBoxVerticalSlices, SIGNAL(stateChanged(int)), this, SLOT(ShowSlicesPoints(int)));
	connect(BranchRemovalForm.spinBoxThickNess, SIGNAL(valueChanged(int)), this, SLOT(ShowSlicesPoints(int)));
	connect(BranchRemovalForm.spinBoxStartHeight, SIGNAL(valueChanged(int)), this, SLOT(ShowHeightPlane(int)));

	connect(BranchRemovalForm.pushButtonBranchCheck, SIGNAL(clicked()), this, SLOT(CheckBranches()));
	connect(BranchRemovalForm.pushButtonRedo, SIGNAL(clicked()), this, SLOT(Redo()));
	connect(BranchRemovalForm.pushButtonRemoval, SIGNAL(clicked()), this, SLOT(RemoveBranches()));

	BranchRemovalForm.pushButtonRedo->setEnabled(false);
	BranchRemovalForm.pushButtonRemoval->setEnabled(false);

	PlanePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	CenterPointsPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	TargentPointsPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

	OutliersColor = ColorBase::RedColor;

	widget->show();
}

CBranchRemovalByTangentPlane::~CBranchRemovalByTangentPlane()
{
	PlanePoints->points.clear();
	if (Viewer != NULL)
	{
		Viewer->removePointCloud(BranchPointsStr);
		Viewer->removePointCloud(HeightPlanePointsStr);
		Viewer->removePointCloud(CenterPointsStr);
		Viewer->removePointCloud(TargentPointsStr);
		Viewer->removePointCloud(CenterPointsPointsStr);
		
		Viewer->removeAllShapes();
	}

	emitUpdateUI();
}

void CBranchRemovalByTangentPlane::ShowSlicesPoints(int CheckValue)
{
	if (InputCloud == NULL) return;

	QAction * Action = (QAction *)sender();
	if (Action == NULL) return;

	if (Action->objectName() == "spinBoxThickNess")
		GetParameters();

	CheckValue = BranchRemovalForm.checkBoxVerticalSlices->checkState();	

	if (CheckValue == 2)
		HorizontalPartition.ShowSectionPoints(0, HorizontalPartition.SectionsCount);
	else if (CheckValue != 2)
		HorizontalPartition.UnShowSectionPoints();
	emitUpdateUI();
}

void CBranchRemovalByTangentPlane::GetParameters()
{
	Thickness = BranchRemovalForm.spinBoxThickNess->text().toDouble();	
	SearchStartIndex = ceil(BranchRemovalForm.spinBoxStartHeight->text().toDouble() /
		BranchRemovalForm.spinBoxThickNess->text().toDouble());
	SuccessiveSectionNumber = BranchRemovalForm.spinBoxSectionNumber->text().toInt();	
	Angle = BranchRemovalForm.spinBoxAngleValue->text().toDouble();

	AlongDistance = BranchRemovalForm.doubleSpinBoxAlongDistance->text().toDouble();

	HorizontalPartition.SetViewer(Viewer);
	HorizontalPartition.SetInputCloud(InputCloud);
	HorizontalPartition.SetThickNess(Thickness);
	HorizontalPartition.PatitionSection();

	HorizontalPartition.CalcCenterPointsByGravity();
}

void CBranchRemovalByTangentPlane::Initial()
{
	GetParameters();

	//Octree = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));	//毫米级
	//Octree->setInputCloud(InputCloud);
	//Octree->addPointsFromInputCloud();

	AnglePartition.SetInputs(InputCloud, HorizontalPartition.SectionsVector,
		HorizontalPartition.GeometryCenterPointsPtr, Angle);

	//Initiation Tangent parameters for each anlge of each section
	for (int i = 0; i < HorizontalPartition.SectionsVector.size(); i++)
	{
		SectionTargentInfo TempSectionTargentInfo;
		TempSectionTargentInfo.SectionIndex = i;
		TempSectionTargentInfo.AvgDis = 0;
		for (int j = 0; j < AnglePartition.PartitionsCount; j++)
		{
			AnglePartitionTargentInfo TempAnglePartitionTargentInfo;
			TempAnglePartitionTargentInfo.AngleIndex = j;
			TempAnglePartitionTargentInfo.Calculated = false;
			TempSectionTargentInfo.AngleSTangentInfo.push_back(TempAnglePartitionTargentInfo);
		}
		SectionSTangentInfo.push_back(TempSectionTargentInfo);
	}
}

void CBranchRemovalByTangentPlane::ShowAngularCenterPoints()
{
	CenterPointsPtr->points.clear();
	TargentPointsPtr->points.clear();

	for (int i = 0; i < HorizontalPartition.SectionsCount; i++)
	{
		for (int j = 0; j < AnglePartition.PartitionsCount; j++)
		{
			if (SectionSTangentInfo[i].AngleSTangentInfo[j].Calculated)
			{
				CenterPointsPtr->points.push_back(SectionSTangentInfo[i].AngleSTangentInfo[j].CenterPoint);
				TargentPointsPtr->points.push_back(SectionSTangentInfo[i].AngleSTangentInfo[j].TargentPoint);
			}
		}
	}

	PointBase::SetPointColor(CenterPointsPtr, ColorBase::RedColor);
	PointBase::SetPointColor(TargentPointsPtr, ColorBase::BlueColor);

	//ShowPoints(CenterPointsPtr, CenterPointsStr, PointSize * 2);
	ShowPoints(TargentPointsPtr, TargentPointsStr, PointSize * 2);

	emitUpdateUI();
}

void CBranchRemovalByTangentPlane::ShowHeightPlane(int Value)
{
	SearchStartIndex = ceil(BranchRemovalForm.spinBoxStartHeight->text().toDouble() /
		BranchRemovalForm.spinBoxThickNess->text().toDouble());

	if (SearchStartIndex >= HorizontalPartition.SectionsCount - 2)
		SearchStartIndex = HorizontalPartition.SectionsCount - 2;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	HorizontalPartition.GetSectionPoints(SearchStartIndex + 1, SectionPoints);

	if (SectionPoints->points.size() == 0) return;

	double TempR = GeometryBase::CircleFittingByLeastSquaresFitting(SectionPoints);
	pcl::PointXYZRGB CenterPoint = GeometryBase::GetCentroidOfPoints(SectionPoints);

	GeometryBase::GetCirclePoints(PlanePoints,	CenterPoint,
		TempR * 2.0, true, 0.5);

	PointBase::SetPointsCoordinateValue(PlanePoints, "Z",
		HorizontalPartition.SectionsVector[SearchStartIndex + 1].ZMin);

	PointBase::SetPointColor(PlanePoints, ColorBase::BlueColor);

	ShowPoints(PlanePoints, HeightPlanePointsStr, PointSize);
}

void CBranchRemovalByTangentPlane::SetViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue)
{
	CTreeBase::SetViewer(ViewerValue);
		
	if (BranchRemovalForm.spinBoxStartHeight->text().toDouble() > ZMax)
	{
		disconnect(BranchRemovalForm.spinBoxStartHeight, SIGNAL(valueChanged(int)), this, SLOT(ShowHeightPlane(int)));
		BranchRemovalForm.spinBoxStartHeight->setValue(ZMax / 4);
		connect(BranchRemovalForm.spinBoxStartHeight, SIGNAL(valueChanged(int)), this, SLOT(ShowHeightPlane(int)));
	}
	
	GetParameters();
	ShowHeightPlane(0);
}

void CBranchRemovalByTangentPlane::CheckBranches()
{
	BranchRemovalForm.pushButtonBranchCheck->setEnabled(false);
	Initial();

	HorizontalPartition.CalcCenterPointsByConvexPolygon(SearchStartIndex,
		SearchStartIndex + SuccessiveSectionNumber + 1);
	for (int i = SearchStartIndex; i < SearchStartIndex + SuccessiveSectionNumber + 1; i++)
	{
		AnglePartition.PartitionSection(i);		
	}


	
	//IsUp = true;
	//CalcSectionSTangent(SearchStartIndex + SuccessiveSectionNumber + 1);

	//HorizontalPartition.ShowSectionPoints(SearchStartIndex - 2, SearchStartIndex);
	
	///*
	//for (int i = SearchStartIndex - 1; i >= SearchStartIndex - 2; i--)
	for (int i = SearchStartIndex - 1; i >= 0; i--)
	{
		IsUp = false;
		QString Hint = "Removing branches of section " + QString::number(i) + " ";
		cout << Hint.toStdString() << endl;
		//emitUpdateStatusBar(Hint, 3000);
		//使用前面分区的点云计算当前角度分区的切平面数据

		CalcSectionSTangent(i);

		//标记树杈点云
		MarkSectionsBranches(i);

		//重新计算几何中心 并对当前分区重新分区
		HorizontalPartition.CalcCenterPointsByConvexPolygon(i);
		
		AnglePartition.PartitionSection(i);
	}
	//*/
	///*
	for (int i = SearchStartIndex; i < HorizontalPartition.SectionsCount; i++)
	{
		IsUp = true;
		QString Hint = "Removing branches of section " + QString::number(i) + " ";
		cout << Hint.toStdString() << endl;
		//emitUpdateStatusBar(Hint, 3000);
		//使用前面分区的点云计算当前角度分区的切平面数据

		CalcSectionSTangent(i);

		//标记树杈点云
		MarkSectionsBranches(i);

		if (i == 72)
		{
			i = 72;
		}

		//重新计算几何中心 并对当前分区重新分区
		HorizontalPartition.CalcCenterPointsByConvexPolygon(i);
		AnglePartition.PartitionSection(i);
	}
	//*/

	PointBase::SetPointColor(HorizontalPartition.GeometryCenterPointsPtr, ColorBase::BlueColor);
	ShowPoints(HorizontalPartition.GeometryCenterPointsPtr, CenterPointsPointsStr, PointSize * 2);

	if (BranchRemovalForm.checkBoxShowTangentPoint->checkState() == 2)
	{
		ShowAngularCenterPoints();
	}

	BranchRemovalForm.pushButtonRedo->setEnabled(true);
	BranchRemovalForm.pushButtonRemoval->setEnabled(true);
	emitUpdateUI();
}

//void CBranchRemovalByTangentPlane::UpdateGeometricalCenterAndReAnglePartition(int SectionIndex)
//{
//	HorizontalPartition.CalcCenterPointsByConvexPolygon(SectionIndex);
//	AnglePartition.PartitionSection(SectionIndex);
//}

void CBranchRemovalByTangentPlane::Redo()
{
	BranchRemovalForm.pushButtonBranchCheck->setEnabled(true);
	BranchRemovalForm.pushButtonRedo->setEnabled(false);
	BranchRemovalForm.pushButtonRemoval->setEnabled(false);
	CTreeBase::Redo();
	RefreshData();
	if (Viewer != NULL)
	{
		Viewer->removeAllShapes();
	}
}

void CBranchRemovalByTangentPlane::RemoveBranches()
{
	BranchRemovalForm.pushButtonRedo->setEnabled(false);
	BranchRemovalForm.pushButtonBranchCheck->setEnabled(false);
	BranchRemovalForm.pushButtonRemoval->setEnabled(false);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		if (!(InputCloud->points[i].r == 255 && InputCloud->points[i].g == 0 && InputCloud->points[i].b == 0))
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

	BranchRemovalForm.pushButtonRedo->setEnabled(true);
	BranchRemovalForm.pushButtonBranchCheck->setEnabled(true);
}

void CBranchRemovalByTangentPlane::RefreshData()
{	
	CTreeBase::RefreshData();
	BranchRemovalForm.spinBoxStartHeight->setValue(80);
	GetParameters();	
}

void CBranchRemovalByTangentPlane::MarkSectionsBranches(int SectionIndex)
{
	for (int i = 0; i < AnglePartition.PartitionsCount; i++)
	{
		double a = SectionSTangentInfo[SectionIndex].AngleSTangentInfo[i].a;
		double b = SectionSTangentInfo[SectionIndex].AngleSTangentInfo[i].b;
		double c = SectionSTangentInfo[SectionIndex].AngleSTangentInfo[i].c;
		double d = SectionSTangentInfo[SectionIndex].AngleSTangentInfo[i].d;
		double Symbol = SectionSTangentInfo[SectionIndex].AngleSTangentInfo[i].Symbol;

		//当前角度分区中的点云
		///*
		//for(int n = 0; n < AnglePartition.PartitionsNumer; n++)
		for (int n = 0; n < 3; n++)
		{
			//if (n == 0)		//2015.10.21 当 n == 0 的时候 
			// 当 n == 0 时 或大于0 时 移除 i 后 第 n 个分区的树杈点
			//{
			int k = (i + n) % AnglePartition.PartitionsCount;
			for (int j = 0; j < AnglePartition.SectionAnglePartitionS[SectionIndex].AnglePartition[k].PointIndexs.size(); j++)
			{
				//在当前分区的索引

				int TempIndex = AnglePartition.SectionAnglePartitionS[SectionIndex].AnglePartition[k].PointIndexs[j];

				if (TempIndex > InputCloud->points.size())
				{
					QMessageBox::information(NULL, tr("Error"),
						tr("The point index is exceed!"));
					break;
				}

				pcl::PointXYZRGB TempPoint = InputCloud->points[TempIndex];
				//// 此处需要调整	容易导致多次加入 树杈点
				double TempValue = TempPoint.x * a + TempPoint.y * b + TempPoint.z * c + d;				
				if (TempValue * Symbol < 0)
				{
					InputCloud->points[TempIndex].rgba = ColorBase::RedColor;
				}
			}			
		}
	}
}

void CBranchRemovalByTangentPlane::CalcSectionSAnglePartitionTargent(int SectionIndex,
	int AnglePartitionIndex)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurrentAnglePointPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	//Get stem points from adjacent section and adjacent angleindex.
	GetTargentPoint(PointPtr, CurrentAnglePointPtr, 
		SectionIndex, AnglePartitionIndex, 3, SuccessiveSectionNumber, 1);

	//获取切平面参数 
	pcl::PointXYZRGB CenterPoint;

	//用于定位平面参数的延长点
	pcl::PointXYZRGB TargentPoint;

	double a, b, c, d;
	//double Symbol;
	CalcBase<double> CalcBaseFloat;

	if (PointPtr->points.size() == 0)	//有可能出现 PointPtr 点集为空的情况
	{
		////2015.08.14 如果为空就沿用上个分区的切平面信息，
		if (SectionIndex > SearchStartIndex) //大于
		{
			a = SectionSTangentInfo[SectionIndex - 1].AngleSTangentInfo[AnglePartitionIndex].a;
			b = SectionSTangentInfo[SectionIndex - 1].AngleSTangentInfo[AnglePartitionIndex].b;
			c = SectionSTangentInfo[SectionIndex - 1].AngleSTangentInfo[AnglePartitionIndex].c;
			CenterPoint = SectionSTangentInfo[SectionIndex - 1].AngleSTangentInfo[AnglePartitionIndex].CenterPoint;
			CenterPoint.z = CenterPoint.z + HorizontalPartition.SectionThickness;
		}
		else
		{
			a = SectionSTangentInfo[SectionIndex + 1].AngleSTangentInfo[AnglePartitionIndex].a;
			b = SectionSTangentInfo[SectionIndex + 1].AngleSTangentInfo[AnglePartitionIndex].b;
			c = SectionSTangentInfo[SectionIndex + 1].AngleSTangentInfo[AnglePartitionIndex].c;
			CenterPoint = SectionSTangentInfo[SectionIndex + 1].AngleSTangentInfo[AnglePartitionIndex].CenterPoint;
			CenterPoint.z - CenterPoint.z + HorizontalPartition.SectionThickness;
		}
	}
	else
	{
		CenterPoint = GeometryBase::GetPointsTangentPlane(PointPtr, a, b, c);
	}

	////2015.08.17 注释掉看看效果
	//使用全局模式
	if (SectionIndex == SearchStartIndex + SuccessiveSectionNumber + 1) //在计算初始分区的时候
	{
		SectionSTangentInfo[SearchStartIndex + SuccessiveSectionNumber + 1].AngleSTangentInfo[AnglePartitionIndex].a = a;
		SectionSTangentInfo[SearchStartIndex + SuccessiveSectionNumber + 1].AngleSTangentInfo[AnglePartitionIndex].b = b;
		SectionSTangentInfo[SearchStartIndex + SuccessiveSectionNumber + 1].AngleSTangentInfo[AnglePartitionIndex].c = c;
	}
	//else	//取之前计算的结果
	//{
	//	a = SectionSTangentInfo[SearchStartIndex + SuccessiveSectionNumber].AngleSTangentInfo[AnglePartitionIndex].a;
	//	b = SectionSTangentInfo[SearchStartIndex + SuccessiveSectionNumber].AngleSTangentInfo[AnglePartitionIndex].b;
	//	c = SectionSTangentInfo[SearchStartIndex + SuccessiveSectionNumber].AngleSTangentInfo[AnglePartitionIndex].c;
	//}
	//使用全局模式

	//将重心点高度设置为当前分区高度
	//2015.11.17 此步对于树干倾斜的时候 不适用 ×××××××××××××××
	//2016.03.17 注意此处的问题，切平面的位置对于移除效果有较大影响。 2016.03.17
	//CenterPoint.z = (HorizontalPartition.SectionsVector[SectionIndex].ZMax + 
	//				HorizontalPartition.SectionsVector[SectionIndex].ZMin) / 2;

	///2015.08.15 计算 重心点 与 几何中心点的平均距离	
	SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].DisFromCenterToGeoCenter =
		PointDis(CenterPoint, HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex]);

	//用于计算每一个角度分区切平面法向量的重心点

	SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].a = a;
	SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].b = b;
	SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].c = c;

	SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].CenterPoint = CenterPoint;
	
	pcl::PointXYZRGB RerferencePoint;
	RerferencePoint.x = 0, RerferencePoint.y = 0, RerferencePoint.z = 0;

	if (SectionIndex > SearchStartIndex)
	{
		//int K = 0;
		int PointIndex = SectionIndex - 1;
		for (int i = 0; i < SuccessiveSectionNumber; i++)
		{
			
			//if (PointIndex )
			RerferencePoint.x = RerferencePoint.x + HorizontalPartition.GeometryCenterPointsPtr->points[PointIndex].x / SuccessiveSectionNumber;
			RerferencePoint.y = RerferencePoint.y + HorizontalPartition.GeometryCenterPointsPtr->points[PointIndex].y / SuccessiveSectionNumber;
			RerferencePoint.z = RerferencePoint.z + HorizontalPartition.GeometryCenterPointsPtr->points[PointIndex].z / SuccessiveSectionNumber;
			PointIndex = PointIndex - 1;
			//K++;
		}
	}
	else
	{
		int PointIndex = SectionIndex + 1;
		for (int i = 0; i < SuccessiveSectionNumber; i++)
		{

			//if (PointIndex )
			RerferencePoint.x = RerferencePoint.x + HorizontalPartition.GeometryCenterPointsPtr->points[PointIndex].x / SuccessiveSectionNumber;
			RerferencePoint.y = RerferencePoint.y + HorizontalPartition.GeometryCenterPointsPtr->points[PointIndex].y / SuccessiveSectionNumber;
			RerferencePoint.z = RerferencePoint.z + HorizontalPartition.GeometryCenterPointsPtr->points[PointIndex].z / SuccessiveSectionNumber;
			PointIndex = PointIndex + 1;
			//K++;
		}
	}

	SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].TargentPoint =
		GeometryBase::GetPointAlongAToB(RerferencePoint, CenterPoint, AlongDistance, true);

	SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].TargentPoint.z =
		HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex].z;

	//计算角度分区的直线参数及延长点 在切平面法向参数已求的情况下
	//CalcAnglePartitionLineParameters(SectionIndex, AnglePartitionIndex, PointPtr, CurrentAnglePointPtr);

	//2016.03.17 改为使用凸包点
	//CalcAnglePartitionTangentPlanePoint(SectionIndex, AnglePartitionIndex);

	//取计算后的延长点
	TargentPoint = SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].TargentPoint;	
	SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].Calculated = true;

	//获取切平面的d值
	SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].d = -
		(TargentPoint.x * SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].a
			+ TargentPoint.y * SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].b
			+ TargentPoint.z * SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].c);

	d = SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].d;
	//根据前一个(或后一个)分区的几何中心点 计算方向
	//if (IsUp)
	if (SectionIndex > SearchStartIndex) 
	{
		SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].Symbol =
			HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex - 1].x * SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].a
			+ HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex - 1].y * SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].b
			+ HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex - 1].z * SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].c
			+ SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].d;
	}
	else
	{
		SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].Symbol =
			HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex + 1].x * SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].a
			+ HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex + 1].y * SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].b
			+ HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex + 1].z * SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].c
			+ SectionSTangentInfo[SectionIndex].AngleSTangentInfo[AnglePartitionIndex].d;
	}	
}

void CBranchRemovalByTangentPlane::CalcSectionSTangent(int SectionIndex)
{
	if (SectionIndex > SearchStartIndex)	//form down to up
	{
		HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex].x 
			= HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex - 1].x;
		HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex].y 
			= HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex - 1].y;
	}
	else									//Up to down
	{
		HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex].x 
			= HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex + 1].x;
		HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex].y 
			= HorizontalPartition.GeometryCenterPointsPtr->points[SectionIndex + 1].y;
	}

	AnglePartition.PartitionSection(SectionIndex);

	for (int i = 0; i < AnglePartition.SectionAnglePartitionS[SectionIndex].AnglePartition.size(); i++)
	{		
		CalcSectionSAnglePartitionTargent(SectionIndex, i);	
		/*
		//if (SectionIndex > 440 && SectionIndex < 460 && i == 0)
		if (SectionIndex % 5 == 0 && i == 0)
		{
			PointBase::ShowPlane(Viewer,
				SectionSTangentInfo[SectionIndex].AngleSTangentInfo[i].a,
				SectionSTangentInfo[SectionIndex].AngleSTangentInfo[i].b,
				SectionSTangentInfo[SectionIndex].AngleSTangentInfo[i].c,
				SectionSTangentInfo[SectionIndex].AngleSTangentInfo[i].TargentPoint,
				StringBase::ClockValue());
		}
		//*/
	}
}

void CBranchRemovalByTangentPlane::GetPoint(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr,
	int SectionIndex,
	int AnglePartitionIndex)
{
	//超过
	if (SectionIndex > AnglePartition.SectionAnglePartitionS.size() - 1) return;
	if (SectionIndex < 0) return;
			
	vector<int> PointsIndex =
		AnglePartition.SectionAnglePartitionS[SectionIndex].AnglePartition[AnglePartitionIndex].PointIndexs;

	for (int i = 0; i < PointsIndex.size(); i++)
	{
		pcl::PointXYZRGB TempPoint;				

		TempPoint = InputCloud->points[PointsIndex[i]];

		if (!(TempPoint.r == 255 && TempPoint.g == 0 && TempPoint.b == 0))	//The color of the stem point is not red.
		{
			PointPtr->points.push_back(TempPoint);
		}
	}
}

void CBranchRemovalByTangentPlane::GetTargentPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AnglePartitionIndexPointPtr,	//当前角度分区的点云 Not used
	int SectionIndex,
	int AnglePartitionIndex,
	int AnglePartitions,	//使用角度分区的个数，此参数应该为奇数，
	int Sections,			//使用垂直分区的个数，
	int SectionSpace)
{
	int k = 1;

	if (SectionIndex >= SearchStartIndex)	//Is Upward or Downward
	{
		k = 1;
		SectionIndex = SectionIndex - SectionSpace;
	}
	else
	{
		k = -1;
		SectionIndex = SectionIndex + SectionSpace;
	}

	for (int i = 1; i <= Sections; i++)
	{
		//Add points from different section but in same angla index		
		GetPoint(PointPtr, SectionIndex - i * k, AnglePartitionIndex);		

		for (int j = 1; j <= AnglePartitions / 2; j++)
		{
			GetPoint(PointPtr, SectionIndex - i * k, //第前j个角度分区
				(AnglePartitionIndex - j + AnglePartition.PartitionsCount) % AnglePartition.PartitionsCount);
			GetPoint(PointPtr, SectionIndex - i * k, //第后j个角度分区
				(AnglePartitionIndex + j) % AnglePartition.PartitionsCount);
		}
	}
}