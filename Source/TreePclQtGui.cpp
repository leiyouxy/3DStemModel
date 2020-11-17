#include "TreePclQtGui.h"

TreePclQtGui * TreePclQtGui::p_Self;

//it does not work;
void TreePclQtGui::mousePointPickingOccurred(const pcl::visualization::PointPickingEvent &event,
	void* Viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer =
		*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (Viewer_void);
	p_Self->SelectedPointIndex = event.getPointIndex();		
	if (p_Self->SelectedPointIndex != -1)
	{
		float x, y, z;
		event.getPoint(x, y, z);		
		QString StrValue = "Selected Index:" + QString::number(p_Self->SelectedPointIndex, 10)
			+ "; x:" + QString::number(x, 10, 6)
			+ ", y:" + QString::number(y, 10, 6)
			+ ", z:" + QString::number(z, 10, 6);
		p_Self->SetStatusText(StrValue, 5000);
	}
}

void TreePclQtGui::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* Viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer = 
		*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (Viewer_void);
	if (event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease ||
		event.getButton() == pcl::visualization::MouseEvent::MouseScrollDown || 
		event.getButton() == pcl::visualization::MouseEvent::MouseScrollUp)
	{
		if (p_Self->p_TreeBase->ClassName == "CPointRenderingSetting")
		{
			vector<pcl::visualization::Camera> TempCameras;
			p_Self->Viewer->getCameras(TempCameras);
			//指定子类对象和子类Ui的指针
			//p_Self->CRenderingSetting = ((CPointRenderingSetting*)(p_Self->p_TreeBase.get()))->RenderingSetting;
			CPointRenderingSetting * TempRenderingSetting = ((CPointRenderingSetting*)(p_Self->p_TreeBase.get()));

			//断开原始信号槽
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_posX, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_posY, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_posZ, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));

			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_viewA, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_viewB, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_viewC, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));

			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_Focal_X, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_Focal_Y, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_Focal_Z, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_Fov, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));

			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_clipA, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_clipB, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));

			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_winsize_Height, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_winsize_Width, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_win_posX, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->disconnect(TempRenderingSetting->RenderingSetting.doubleSpinBox_win_posY, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			//修改子类Ui::RenderingSettingFrom 的控件属性
			TempRenderingSetting->RenderingSetting.doubleSpinBox_posX->setValue(TempCameras[0].pos[0]);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_posY->setValue(TempCameras[0].pos[1]);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_posZ->setValue(TempCameras[0].pos[2]);

			TempRenderingSetting->RenderingSetting.doubleSpinBox_viewA->setValue(TempCameras[0].view[0]);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_viewB->setValue(TempCameras[0].view[1]);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_viewC->setValue(TempCameras[0].view[2]);

			TempRenderingSetting->RenderingSetting.doubleSpinBox_clipA->setValue(TempCameras[0].clip[0]);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_clipB->setValue(TempCameras[0].clip[0]);

			TempRenderingSetting->RenderingSetting.doubleSpinBox_Fov->setValue(TempCameras[0].fovy);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_Focal_X->setValue(TempCameras[0].focal[0]);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_Focal_Y->setValue(TempCameras[0].focal[1]);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_Focal_Z->setValue(TempCameras[0].focal[2]);

			TempRenderingSetting->RenderingSetting.doubleSpinBox_winsize_Height->setValue(TempCameras[0].window_size[0]);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_winsize_Width->setValue(TempCameras[0].window_size[1]);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_win_posX->setValue(TempCameras[0].window_pos[0]);
			TempRenderingSetting->RenderingSetting.doubleSpinBox_win_posY->setValue(TempCameras[0].window_pos[1]);

			//连接原始信号槽
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_posX, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_posY, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_posZ, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));

			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_viewA, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_viewB, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_viewC, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));

			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_Focal_X, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_Focal_Y, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_Focal_Z, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_Fov, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));

			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_clipA, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_clipB, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));

			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_winsize_Height, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_winsize_Width, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_win_posX, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));
			TempRenderingSetting->connect(TempRenderingSetting->RenderingSetting.doubleSpinBox_win_posY, SIGNAL(valueChanged(double)), TempRenderingSetting, SLOT(doubleSpinBox_ValueChanged(double)));

			//SetStatusText about Camera_Properities
			/*QString StrPosition = "Camera pos ("
				+ QString::number(TempCameras[0].pos[0])
				+ "," + QString::number(TempCameras[0].pos[1])
				+ "," + QString::number(TempCameras[0].pos[2]) + ")";
			QString StrView = ", View("
				+ QString::number(TempCameras[0].view[0])
				+ "," + QString::number(TempCameras[0].view[1])
				+ "," + QString::number(TempCameras[0].view[2]) + ")";
			QString StrClip = ", Clip("
				+ QString::number(TempCameras[0].clip[0])
				+ "," + QString::number(TempCameras[0].clip[1]) + ")";
			QString StrFovyandFocal = ", Fovy = "
				+ QString::number(TempCameras[0].fovy)
				+ ",focal(" + QString::number(TempCameras[0].focal[0])
				+ "," + QString::number(TempCameras[0].focal[1])
				+ "," + QString::number(TempCameras[0].focal[2]) + ")";
			QString StrWin_size = ", win_sizeh = "
				+ QString::number(TempCameras[0].window_size[0])
				+ ",win_sizew=" + QString::number(TempCameras[0].window_size[1]);
			QString StrWin_position = ", win_pos("
				+ QString::number(TempCameras[0].window_pos[0])
				+ "," + QString::number(TempCameras[0].window_pos[1]) + ")";

			p_Self->SetStatusText(StrPosition + StrView + StrClip
				+ StrFovyandFocal + StrWin_size + StrWin_position, 10000);*/
		}
		//std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;

		//char str[512];
		//sprintf(str, "text#%03d", text_id++);
		//Viewer->addText ("clicked here", event.getX (), event.getY (), str);
	}
}

void TreePclQtGui::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* Viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (Viewer_void);
	if (event.getKeySym() == "d" && event.keyDown())
	{
		//std::cout << "d was pressed => removing all text" << std::endl;
		QString StrValue = "d was pressed => removing all text";
		p_Self->SetStatusText(StrValue, 5000);

		//char str[512];
		//for (unsigned int i = 0; i < text_id; ++i)
		//{
		//	sprintf(str, "text#%03d", i);
		//	Viewer->removeShape(str);
		//}
		//text_id = 0;
	}
}

void TreePclQtGui::parseArguments()
{
	// 获取命令行参数
	//QStringList arguments = QCoreApplication::arguments();
	//if (arguments.count() < 2)
	//	return;

	//QString strJson = arguments.at(1);

	//// 解析Json对象
	//QJsonParseError jsonError;
	//QJsonDocument doucment = QJsonDocument::fromJson(strJson.toLocal8Bit(), &jsonError);
	//if (jsonError.error != QJsonParseError::NoError)
	//	return;

	//if (doucment.isObject())
	//{
	//	QJsonObject obj = doucment.object();
	//	QJsonValue value;
	//	if (obj.contains("FileName"))
	//	{
	//		value = obj.take("UserName");
	//		if (value.isString())
	//			OpenedFilePath = value.toString().toStdString();
	//	}
	//}
}

TreePclQtGui::TreePclQtGui(QWidget *parent)
	: QMainWindow(parent)
{	
	p_Self = this;
	Mainui.setupUi(this);
	
	AppPath = getcwd(NULL, 0);
	parseArguments();
	//this->setWindowTitle("3D Stem Modelling and Measurement for a Single Tree");

	//int screenwidth = GetSystemMetrics(SM_CXFULLSCREEN);
	//int screenheight = GetSystemMetrics(SM_CYFULLSCREEN);
	//Mainui.groupBox->resize(350, screenheight);
	//Mainui.qvtkWidget->resize(screenwidth - 350, screenheight);

	////20190408 for Paper Use
	//Mainui.menuTreePointPrc->setTitle("");	
	//Mainui.menuStemPointPrc->setTitle("");
	//Mainui.actionStemVolumeByVerticalSlices->setVisible(false);
	//Mainui.actionStemVolumeByTetrahedron->setVisible(false);

	//Status Labels initialize	
	FirstLabelofStatus = new QLabel(this);
	//FirstLabelofStatus->setMinimumWidth(350);
	//FirstLabelofStatus->setText("Third");
	Mainui.statusBar->addPermanentWidget(FirstLabelofStatus);

	SecondLabelofStatus = new QLabel(this);
	Mainui.statusBar->addPermanentWidget(SecondLabelofStatus);

	VersionLabel = new QLabel(this);
	//VersionLabel->setText("The Program of 3D Stem Modelling and Measurement for a Single Tree using Point Cloud Data, Version 1.0. ");
	VersionLabel->setText("3D Stem Modelling and Measurement of Tree Plus Plus, Version 1.0. ");	//2019.03.25		
	Mainui.statusBar->addPermanentWidget(VersionLabel);

	// Setup the cloud pointer
	TreeStemCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	//InitialpcdFileName = "3DStemModelForMeasure\\A2015111401_4Stem_0_100_Tip.pcd";	
	//InitialpcdFileName = "3DStemModelForMeasure\\A2015111401_4Stem_0_350_New.pcd";
	//InitialpcdFileName = "pcdDemo\\A2015111401_4Stem_0_250.pcd";
	//InitialpcdFileName = "pcdDemo\\Refine0.pcd";
	//InitialpcdFileName = "pcdDemo\\StemCone.pcd";
	//OpenedFilePath = AppPath + "\\" + "pcdDemo\\Demo.pcd";	
	//OpenedFilePath =  "I:\\8_BranchesTest.xyz";
	
	OpenedFilePath = "I:\\RemoveBranches\\8_RemoveStub.xyz";
	OpenedFilePath = "I:\\RemoveBranches\\8_MainTree_Bigger.pcd";
	//OpenedFilePath = "I:\\RemoveBranches\\A032701001AllStem.pcd";
	OpenedFilePath = "I:\\RemoveBranches\\8_MainTree.pcd";
	OpenedFilePath = "I:\\RemoveBranches\\8_MainTree_RemoveOutliers.pcd";
	OpenedFilePath = "I:\\PlanePoints\\PlanePoints20200525.pcd";
	OpenedFilePath = "I:\\PlanePoints\\Circle1000.pcd";
	OpenedFilePath = "I:\\PlanePoints\\Circle3000.pcd";
	OpenedFilePath = "I:\\PlanePoints\\Rectangle10000.pcd";

	OpenedFilePath = "I:\\PlanePoints\\PaperUsed700.pcd";
	OpenedFilePath = "I:\\PlanePoints\\Rectangle25000.pcd";
	OpenedFilePath = "I:\\PlanePoints\\Rectangle30000.pcd";
	OpenedFilePath = "I:\\3DStemModel\\1740_001Stem.vtx";
	OpenedFilePath = "I:\\000Comparsion\\Final\\1740_001Stem_Data_3.vtx";
	OpenedFilePath = "I:\\000Comparsion\\Final\\A2015111403_NewStem_0_1200_New_Data_0.vtx";	

	OpenedFilePath = "E:\\20180623-20180704DaGuJiaScans\\PCLOutPut\\20201027_1776_Tree41.vtx";
	OpenedFilePath = "I:\\000Comparsion\\FinalSlice20201109\\1728_002_Stem_Data_0_10.vtx";

	//OpenedFilePath = "I:\\PlanePoints\\Circle1000.pcd";
	//OpenedFilePath = "I:\\PlanePoints\\PaperUsed700.pcd";
	//OpenedFilePath = "I:\\PlanePoints\\Rectangle1000.pcd";
	
	//OpenedFilePath = "I:\\PlanePoints\\TempPoints.pcd";
		
	//OpenedFilePath = "I:\\RemoveBranches\\8_RemoveStub_AfterPreprocessing.xyz";	
	
	//OpenedFilePath = "I:\\3DStemVolumePoint\\20190414Add\\A03270511ProcessedStemCloud_100_200.pcd";
	//OpenedFilePath = "I:\\3DStemModelForMeasure\\A2015111401_4Stem_0_360.pcd";

	//OpenedFilePath = "pcdDemo\\StemCone.pcd";

	this->setWindowTitle(QObject::tr((AppTitle + string("--") + OpenedFilePath).c_str()));

	PointBase::OpenPCLFile(OpenedFilePath, TreeStemCloud, true);	

	// Set up the QVTK window	
	Viewer.reset(new pcl::visualization::PCLVisualizer("Viewer", false));
	Mainui.qvtkWidget->SetRenderWindow(Viewer->getRenderWindow());	
	Viewer->setupInteractor(Mainui.qvtkWidget->GetInteractor(), Mainui.qvtkWidget->GetRenderWindow());
	Mainui.qvtkWidget->update();
		
	//设置背景颜色为白色
	Viewer->setBackgroundColor(255, 255, 255);
	//PointBase::SetPointColor(TreeStemCloud, ColorBaseS[0]);

	Viewer->registerPointPickingCallback(TreePclQtGui::mousePointPickingOccurred, (void*)&Viewer);
	Viewer->registerMouseCallback(TreePclQtGui::mouseEventOccurred, (void*)&Viewer);
	Viewer->registerKeyboardCallback(TreePclQtGui::keyboardEventOccurred, (void*)&Viewer);
	//Viewer->registerAreaPickingCallback()

	this->setMouseTracking(true);//设置鼠标跟踪;

	connect(this, SIGNAL(UpdateUI()), this, SLOT(UpdateCloudAndUI()));
	connect(this, SIGNAL(UpdateStatus(QString, int)), this, SLOT(SetStatusText(QString, int)));
	connect(this, SIGNAL(UpdateTitle(string)), this, SLOT(UpdateAppTitle(string)));
	connect(this, SIGNAL(ShowMsg(string)), this, SLOT(ShowMessage(string)));
	connect(this, SIGNAL(UpdateResetCamera()), this, SLOT(ResetCamera()));

	//Action
	connect(Mainui.actionOpen_File, SIGNAL(triggered()), this, SLOT(OpenCloudByFile()));
	connect(Mainui.actionReload, SIGNAL(triggered()), this, SLOT(Reload()));

	connect(Mainui.actionSave_Point_Cloud, SIGNAL(triggered()), this, SLOT(SaveCloudtoFile()));
	connect(Mainui.actionSave_as, SIGNAL(triggered()), this, SLOT(SaveAsCloudtoFile()));
	
	connect(Mainui.actionInsert_And_Move, SIGNAL(triggered()), this, SLOT(OpenAndInsert_And_Move()));

	connect(Mainui.actionSetting_For_Rending, SIGNAL(triggered()), this, SLOT(ShowPointRenderingSetting()));

	connect(Mainui.actionReset_Camera, SIGNAL(triggered()), this, SLOT(Reset_Camera()));
	connect(Mainui.actionSave_Picture, SIGNAL(triggered()), this, SLOT(SaveViewerAsPicture()));	

	connect(Mainui.actionSimulate_Measure, SIGNAL(triggered()), this, SLOT(ShowSimulateMeasureForm()));
	
	//Tools
	connect(Mainui.actionGenerate_Points, SIGNAL(triggered()), this, SLOT(GeneratePoints()));
	connect(Mainui.actionFittingMethods, SIGNAL(triggered()), this, SLOT(ShowFittingMethodsForm()));
	connect(Mainui.actionDownSample, SIGNAL(triggered()), this, SLOT(DownSample()));
	
	connect(Mainui.actionPoints_Cluster, SIGNAL(triggered()), this, SLOT(ShowClusterForm()));

	connect(Mainui.actionDemo_Form_and_Class, SIGNAL(triggered()), this, SLOT(ShowDemoForm()));	

	connect(Mainui.actionDelaunay_Parallel, SIGNAL(triggered()), this, SLOT(ShowDelaunayParallelForm()));

	connect(Mainui.actionTreeStubRemove, SIGNAL(triggered()), this, SLOT(TreeStubRemoval()));
	connect(Mainui.actionBranchesRemovalByTangentPlane, SIGNAL(triggered()), this, SLOT(BranchRemovalByTangentPlane()));
	connect(Mainui.actionTangent_Plane_And_Stem_Axis_Curve, SIGNAL(triggered()), this, SLOT(BranchRemovalByTangentPlaneAndStemAxisCurve()));

	connect(Mainui.action_BranchesRemovalClusterAndCurvature, SIGNAL(triggered()), this, SLOT(BranchesRemovalClusterAndCurvature()));
	
	//outliers removal
	connect(Mainui.actionRemoval_By_Neighbourhood_Number, SIGNAL(triggered()), this, SLOT(OutliersRemoval()));
	connect(Mainui.actionOutliersRemovalByQuantile, SIGNAL(triggered()), this, SLOT(OutliersRemoval()));
	connect(Mainui.actionOutliersRemovalByNeighbour, SIGNAL(triggered()), this, SLOT(OutliersRemoval()));
	connect(Mainui.actionRemove_By_Irregularity, SIGNAL(triggered()), this, SLOT(OutliersRemoval()));

	connect(Mainui.action_StemDiameterAndBasalArea, SIGNAL(triggered()), this, SLOT(StemDiameterAndBasalRetrieval()));
	connect(Mainui.actionSlice_DBScan, SIGNAL(triggered()), this, SLOT(BranchesClassficationSliceDBScan()));
	
	//Status Bar

	//stem points removal by plane
	connect(Mainui.actionStemPoints_Removal, SIGNAL(triggered()), this, SLOT(StemPointsRemovalByPlaneEquation()));

	connect(Mainui.actionStemVolumeByVerticalSlices, SIGNAL(triggered()), this, SLOT(StemVolumeCalculationByVerticalSlices()));
	//connect(Mainui.actionProfile_Curve_of_Vertical_slices_calculation, SIGNAL(triggered()), this, SLOT(StemVolumeCalculationByVerticalSlices()));			
	if (TreeStemCloud->points.size())
		Viewer->addPointCloud(TreeStemCloud, "TreeStemCloud");
	Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "TreeStemCloud");
	Viewer->resetCamera();

	//pcl::PointXYZRGB Point1, Point2;
	//Point1.x = 23.9539, Point1.y = -5.7755, Point1.z = 136.01;
	//Point2.x = 23.9371, Point2.y = -5.77218, Point2.z = 135.363;

	//double NewAngle = GeometryBase::RadianToAngle(
	//	GeometryBase::AngleOfTwoVector(Point1, Point2));
	//cout << "NewAngle:" << NewAngle << endl;

	Mainui.qvtkWidget->update();	
	PointSize = 2;
	//ShowPointRenderingSetting();

	//ShowDelaunayParallelForm();

	ShowSimulateMeasureForm();
}

void TreePclQtGui::ShowPointRenderingSetting()
{
	//if (TreeStemCloud->points.size() == 0)
	//{
	//	QMessageBox::information(this, tr("Information"),
	//		tr("No points to opeate, please open a point cloud file first!"));
	//	return;
	//}

	RemoveGroupBoxItems();

	p_TreeBase.reset(new CPointRenderingSetting(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;
	p_TreeBase->ClassName = "CPointRenderingSetting";
	((CPointRenderingSetting *)(p_TreeBase.get()))->CameraValueRead();
	//此处设置

	Mainui.groupBox->setTitle(tr("Point Viewer Setting"));
}

void TreePclQtGui::ShowDelaunayParallelForm()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();
	p_TreeBase.reset(new CDelaunayParallel(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	Mainui.groupBox->setTitle(tr("Delaunay Parallel Form"));
}

void TreePclQtGui::ShowSimulateMeasureForm()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();
	p_TreeBase.reset(new CSimulateMeasure(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	Mainui.groupBox->setTitle(tr("Simulating Measure Form"));
}

void TreePclQtGui::ShowDemoForm()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();
	p_TreeBase.reset(new CDemo(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;
	Mainui.groupBox->setTitle(tr("Demo Class and Form"));
}

void TreePclQtGui::ShowClusterForm()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();
	p_TreeBase.reset(new CCluster(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;
	Mainui.groupBox->setTitle(tr("Cluster For Points"));
}

void TreePclQtGui::GeneratePoints()
{
	RemoveGroupBoxItems();
	p_TreeBase.reset(new CGeneratePoints(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	Mainui.groupBox->setTitle(tr("Generate points"));
}

void TreePclQtGui::DownSample()
{
	RemoveGroupBoxItems();
	p_TreeBase.reset(new CPointDownSample(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	Mainui.groupBox->setTitle(tr("Generate points"));
}

void TreePclQtGui::ShowFittingMethodsForm()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();
	p_TreeBase.reset(new CFittingMethods(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	Mainui.groupBox->setTitle(tr("Fitting Methods Effect"));
}

TreePclQtGui::~TreePclQtGui()
{		
	if (TreeStemCloud->points.size() > 0)
	{
		TreeStemCloud->points.clear();
	}	
}

void TreePclQtGui::Reload()
{
	OpenCloudByFile(QString::fromLocal8Bit(OpenedFilePath.c_str()));
}

//2018.12.12 Open a new point file
void TreePclQtGui::OpenCloudByFile(QString FileName)
{		
	if (FileName.size() == 0)
		FileName = QFileDialog::getOpenFileName(this,
		tr("Point Cloud Files"), OpenedFilePath.c_str(), tr("Point Cloud Files (*.pcd *.las *.xyz *.vtx *.ply)"));	

	if (FileName.length() <= 0)
		return;

	TreeStemCloud->points.clear();
	SetStatusText("File is opening, please wait!", 0);
	Viewer->removeAllPointClouds();
	Viewer->removeAllShapes();

	OpenedFilePath = string((const char *)FileName.toLocal8Bit());
	PointBase::OpenPCLFile(OpenedFilePath, TreeStemCloud);

	if(TreeStemCloud->points.size() > 0)
	{
		CPointUnit PointUnit(this);
		PointUnit.SetInputCloud(TreeStemCloud);
		PointUnit.exec();
	}
	this->setWindowTitle(QObject::tr((AppTitle + string("--") + OpenedFilePath).c_str()));	
	SetStatusText("File has been opened! Points number:" + QString::number(TreeStemCloud->points.size()), 3000);

	//New point cloud, p_TreeBase should refresh data. Invoking RefreshData function of sub class.
	if (p_TreeBase != NULL)
	{
		p_TreeBase->SetInputCloud(TreeStemCloud);
		p_TreeBase->SetViewer(Viewer);
		p_TreeBase->OpenedFilePath = OpenedFilePath;
		p_TreeBase->ShowPoints(TreeStemCloud, TreeStemCloudStr);
		Viewer->resetCamera();
		p_TreeBase->RefreshData();
	}
	else
	{
		Viewer->removePointCloud(TreeStemCloudStr);
		Viewer->addPointCloud(TreeStemCloud, TreeStemCloudStr);
		Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
			PointSize, TreeStemCloudStr);
		Viewer->resetCamera();		
	}
	UpdateCloudAndUI();
}

void TreePclQtGui::SaveCloudtoFile()
{
	if (OpenedFilePath.length() > 0)
	{
		SetStatusText("File is saving, please wait!", 0);
		PointBase::SavePCDToFileName(TreeStemCloud, OpenedFilePath);
		SetStatusText("File is saved!", 3000);
	}
	else
		SaveAsCloudtoFile();
}

void TreePclQtGui::SaveAsCloudtoFile()
{
	QString FileName = QFileDialog::getSaveFileName(this,
		tr("Point Cloud Files"), OpenedFilePath.c_str(), 
		//tr("Point Cloud Files (*.pcd);; XYZ Files(*.xyz);; VTX Files(*.vtx);;  LAS Files(*.las)")); //las is not workable
		tr("Point Cloud Files (*.pcd);; XYZ Files(*.xyz);; VTX Files(*.vtx);; PLY Files(*.ply)"));
	
	if (FileName.length() > 0)
	{
		SetStatusText("File is saving, please wait!", 0);
		OpenedFilePath = string((const char *)FileName.toLocal8Bit());
		PointBase::SavePCDToFileName(TreeStemCloud, OpenedFilePath);
		
		//OpenedFilePath = string((const char *)FileName.toLocal8Bit());
		UpdateAppTitle(OpenedFilePath);		
		
		if (p_TreeBase != NULL)
			p_TreeBase->OpenedFilePath = OpenedFilePath;

		SetStatusText("File is saved!", 3000);
	}
} 

void TreePclQtGui::Reset_Camera()
{
	Viewer->updatePointCloud(TreeStemCloud, "TreeStemCloud");
	Viewer->resetCamera();
	Mainui.qvtkWidget->update();
}

//将PCLVisualizer中的视图存为图片
void TreePclQtGui::SaveViewerAsPicture()
{	 
	string FileName = "PCL_ScreenShot_" + StringBase::ClockValue();
	Viewer->saveScreenshot(FileName + ".png");
	Viewer->saveCameraParameters(FileName + ".cam");
}

//2019.03.17
void TreePclQtGui::TreeStubRemoval()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();

	Mainui.groupBox->setTitle(tr("Tree Stub Removal by Variance of Distances between Center Points"));
		
	p_TreeBase.reset(new CTreeStubDetection(Mainui.groupBox));	
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);	
	p_TreeBase->PointSize = PointSize;	
	p_TreeBase->OpenedFilePath = OpenedFilePath;	
	p_TreeBase->SetViewer(Viewer);
}

//2019.03.26
void TreePclQtGui::BranchRemovalByTangentPlane()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();

	Mainui.groupBox->setTitle(tr("Branches Removal by Tangent Plane"));

	p_TreeBase.reset(new CBranchRemovalByTangentPlane(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->PointSize = PointSize;
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	p_TreeBase->SetViewer(Viewer);
}

void TreePclQtGui::BranchesRemovalClusterAndCurvature()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();

	Mainui.groupBox->setTitle(tr("Branches Removal using DBScan for Point Distance and Curvature"));

	p_TreeBase.reset(new CBranchRemovalByClusterAndCurvature(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->PointSize = PointSize;
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	p_TreeBase->SetViewer(Viewer);
}

//2019.06.13
void TreePclQtGui::BranchRemovalByTangentPlaneAndStemAxisCurve()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();

	Mainui.groupBox->setTitle(tr("Branches Removal using hybrid geometric characteristic probability"));

	p_TreeBase.reset(new CBranchRemovalByTangentPlaneAndStemAxisCurve(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->PointSize = PointSize;
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	p_TreeBase->SetViewer(Viewer);
}



//移除异常点根据邻域点个数的
void TreePclQtGui::OutliersRemoval()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();

	QAction * Action = (QAction *)sender();
	if (Action == NULL) return;
	string Type = "";

   	if (Action->objectName() == "actionOutliersRemovalByNeighbour")
	{
		Type = "Stat";
		Mainui.groupBox->setTitle(tr("Outliers Removal By Neighbour"));
	}
	else if (Action->objectName() == "actionOutliersRemovalByQuantile")
	{
		Type = "Quartile";
		Mainui.groupBox->setTitle(tr("Outliers Removal By Quartile"));
	}
	else if (Action->objectName() == "actionRemoval_By_Neighbourhood_Number")
	{
		Type = "Number";
		Mainui.groupBox->setTitle(tr("Outliers Removal By Neighbourhood Number"));
	}
	else if (Action->objectName() == "actionRemove_By_Irregularity")
	{
		Type = "Irregularity";
		Mainui.groupBox->setTitle(tr("Outliers Removal By Point Irregularity"));
	}
	
	p_TreeBase.reset(new COutliersRemovalByRadius(Mainui.groupBox, Type));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);	
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	p_TreeBase->PointSize = PointSize;
}

void TreePclQtGui::SetStatusText(QString StrValue, int TimeOut)
{	
	Mainui.statusBar->showMessage(StrValue, TimeOut);
	//ThirdLabelofStatus->setText(StrValue);
}

//移除 groupBox 中的所有组件
void TreePclQtGui::RemoveGroupBoxItems()
{	
	QObjectList list = Mainui.groupBox->children();
	Q_FOREACH(QObject* obj, list)
	{		
		obj->deleteLater();
		delete obj;
	}
}



void TreePclQtGui::StemPointsRemovalByPlaneEquation()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();
	Mainui.groupBox->setTitle(tr("Stem Points Removal by Plane Equation"));

	//p_TreeBase = new CStemPointsRemovalByPlane(Mainui.groupBox);
	p_TreeBase.reset(new CStemPointsRemovalByPlane(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);	
	p_TreeBase->PointSize = PointSize;
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	((CStemPointsRemovalByPlane *)(p_TreeBase.get()))->RefreshFormParameters();
}

//2020.02.12 Slice DBScan 
void TreePclQtGui::BranchesClassficationSliceDBScan()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();
	Mainui.groupBox->setTitle(tr("Branch Classfication By Slice DBScan"));
		
	p_TreeBase.reset(new CBranchesClassficationByDBScan(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;

	((CBranchesClassficationByDBScan *)(p_TreeBase.get()))->SetInputCloud(TreeStemCloud);
	//p_TreeBase->SetInputCloud(TreeStemCloud);
	
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;
}

//2019.01.21 Retrieval stem Form Parameters
void TreePclQtGui::StemDiameterAndBasalRetrieval()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();
	Mainui.groupBox->setTitle(tr("Stem Diameter and Basal Area Retrieval"));

	p_TreeBase.reset(new CStemDiameterAndBasalRetrieval(Mainui.groupBox));
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;
}

void TreePclQtGui::StemVolumeCalculationByVerticalSlices()
{
	if (TreeStemCloud->points.size() == 0)
	{
		QMessageBox::information(this, tr("Information"),
			tr("No points to opeate, please open a point cloud file first!"));
		return;
	}

	RemoveGroupBoxItems();

	QAction * Action = (QAction *)sender();
	if (Action == NULL) return;
	string Type = "";

	if (Action->objectName() == "actionStemVolumeByVerticalSlices")
	{
		Mainui.groupBox->setTitle(tr("Volume Calculation"));		
	}
	//else if (Action->objectName() ==  "actionProfile_Curve_of_Vertical_slices_calculation")
	//{
	//	Mainui.groupBox->setTitle(tr("Volume Calculation by Profile Curve of Vertical Slices"));
	//	Type = "Profile";
	//}
	//else if (Action->objectName() == "actionProfile_Curve_of_Vertical_slices_calculation")
	//{
	//	Mainui.groupBox->setTitle(tr("Volume Calculation by Profile Curve of Vertical Slices"));
	//	Type = "Profile";
	//}

	//p_TreeBase = new CStemVolumeVerticalSlices(Mainui.groupBox, Type);	
	p_TreeBase.reset(new CStemVolumeVerticalSlices(Mainui.groupBox, Type));	
	p_TreeBase->p_TreePclQtGui = this;
	p_TreeBase->SetInputCloud(TreeStemCloud);	
	p_TreeBase->SetViewer(Viewer);
	p_TreeBase->PointSize = PointSize;		
	p_TreeBase->OpenedFilePath = OpenedFilePath;
	((CStemVolumeVerticalSlices *)(p_TreeBase.get()))->OnShow();
}

void TreePclQtGui::TestMsg()
{
	QMessageBox::information(this, tr("Test emit signal"),
		tr("This example shows how the change layouts dynamically."));	
}

void TreePclQtGui::UpdateCloudAndUI()
{
	Viewer->updatePointCloud(TreeStemCloud, TreeStemCloudStr);
	Viewer->updateCamera();			
	cout << endl;
	Mainui.qvtkWidget->update();
}

void TreePclQtGui::ResetCamera()
{
	Viewer->updatePointCloud(TreeStemCloud, TreeStemCloudStr);
	Viewer->resetCamera();
	cout << endl;
	Mainui.qvtkWidget->update();
}

void TreePclQtGui::ShowMessage(string Msg)
{
	QMessageBox::information(this, tr("Information"),
		tr(Msg.c_str()));
}

void TreePclQtGui::UpdateAppTitle(string FileName)
{
	OpenedFilePath = FileName;
	this->setWindowTitle(QObject::tr((AppTitle + string("--") + OpenedFilePath).c_str()));
	QApplication::processEvents();
}

void TreePclQtGui::OpenAndInsert_And_Move()
{
	QString FileName = QFileDialog::getOpenFileName(this,
			tr("Point Cloud Files"), OpenedFilePath.c_str(), tr("Point Cloud Files (*.pcd *.las *.xyz *.vtx)"));

	if (FileName.length() <= 0)
		return;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	string TempOpenedFilePath = string((const char *)FileName.toLocal8Bit());
	PointBase::OpenPCLFile(TempOpenedFilePath, TempPoints, false);
	
	if (TempPoints->points.size() > 0)
	{
		CPointUnit PointUnit(this);
		PointUnit.SetInputCloud(TempPoints);
		PointUnit.exec();
	}
	
	PointsMove(TreeStemCloud, -30, 0, 0);

	TreeStemCloud->points.insert(TreeStemCloud->points.end(),
		TempPoints->points.begin(), TempPoints->points.end());
	UpdateCloudAndUI();
}
