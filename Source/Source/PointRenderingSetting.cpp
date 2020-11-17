#include "PointRenderingSetting.h"

void CPointRenderingSetting::randomButtonPressed()
{
	for (size_t i = 0; i < InputCloud->size(); i++)
	{
		InputCloud->points[i].r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
		InputCloud->points[i].g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
		InputCloud->points[i].b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
	}
	emitUpdateUI();
}

void CPointRenderingSetting::RGBsliderReleased()
{
	for (size_t i = 0; i < InputCloud->size(); i++)
	{
		InputCloud->points[i].r = red;
		InputCloud->points[i].g = green;
		InputCloud->points[i].b = blue;
	}

	emitUpdateUI();
}

void CPointRenderingSetting::pSliderValueChanged(int value)
{
	if(p_TreePclQtGui != NULL)
		p_TreePclQtGui->PointSize = value;
	Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "TreeStemCloud");
	RenderingSetting.lcdNumber_p->display(int(value));
	emitUpdateUI();
}

void CPointRenderingSetting::redSliderValueChanged(int value)
{
	red = value;	
	RenderingSetting.lcdNumber_R->display(int(red));
}

void CPointRenderingSetting::greenSliderValueChanged(int value)
{
	green = value;
	RenderingSetting.lcdNumber_G->display(int(green));
}

void CPointRenderingSetting::blueSliderValueChanged(int value)
{
	blue = value;
	RenderingSetting.lcdNumber_B->display(int(blue));
}

void CPointRenderingSetting::Redo()
{

}

CPointRenderingSetting::CPointRenderingSetting()
{

}

CPointRenderingSetting::CPointRenderingSetting(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	RenderingSetting.setupUi(widget);	

	connect(RenderingSetting.pushButton_random, SIGNAL(clicked()), this, SLOT(randomButtonPressed()));
	connect(RenderingSetting.pushButtonHeightColor, SIGNAL(clicked()), this, SLOT(SetHeightColor()));
	connect(RenderingSetting.pushButtonStemColor, SIGNAL(clicked()), this, SLOT(SetStemColor()));

	connect(RenderingSetting.horizontalSlider_R, SIGNAL(valueChanged(int)), this, SLOT(redSliderValueChanged(int)));
	connect(RenderingSetting.horizontalSlider_G, SIGNAL(valueChanged(int)), this, SLOT(greenSliderValueChanged(int)));
	connect(RenderingSetting.horizontalSlider_B, SIGNAL(valueChanged(int)), this, SLOT(blueSliderValueChanged(int)));
	connect(RenderingSetting.horizontalSlider_R, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	connect(RenderingSetting.horizontalSlider_G, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	connect(RenderingSetting.horizontalSlider_B, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	connect(RenderingSetting.horizontalSlider_p, SIGNAL(valueChanged(int)), this, SLOT(pSliderValueChanged(int)));	

	//connect(RenderingSetting.horizontalSlider_R, SIGNAL(valueChanged(int)), this, SLOT(RenderingSetting.lcdNumber_R->display(int)));
	//connect(RenderingSetting.horizontalSlider_G, SIGNAL(valueChanged(int)), this, SLOT(RenderingSetting.lcdNumber_G->display(int)));
	//connect(RenderingSetting.horizontalSlider_B, SIGNAL(valueChanged(int)), this, SLOT(RenderingSetting.lcdNumber_B->display(int)));
	
	//RenderingSetting.horizontalSlider_R->valueChanged.connect(RenderingSetting.lcdNumber_R->display);
	//RenderingSetting.horizontalSlider_G->valueChanged.connect(RenderingSetting.lcdNumber_G->display);
	//RenderingSetting.horizontalSlider_B->valueChanged.connect(RenderingSetting.lcdNumber_B->display);
	
	connect(RenderingSetting.checkBoxShowAxis, SIGNAL(stateChanged(int)), this, SLOT(ShowCoordinateAxis(int)));
	connect(RenderingSetting.checkBoxShowIndex, SIGNAL(stateChanged(int)), this, SLOT(ShowPointIndex(int)));	

	//Luo Jian Man
	connect(RenderingSetting.doubleSpinBox_posX, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_posY, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_posZ, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	connect(RenderingSetting.doubleSpinBox_viewA, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_viewB, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_viewC, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	connect(RenderingSetting.doubleSpinBox_Focal_X, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_Focal_Y, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_Focal_Z, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_Fov, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	connect(RenderingSetting.doubleSpinBox_clipA, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_clipB, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	connect(RenderingSetting.doubleSpinBox_winsize_Height, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_winsize_Width, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_win_posX, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_win_posY, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	   
	widget->show();
}

CPointRenderingSetting::~CPointRenderingSetting()
{

}

void CPointRenderingSetting::ShowCoordinateAxis(int CheckValue)
{
	if (CheckValue == 2)
	{
		Viewer->addCoordinateSystem(10.0, 0);
	}
	else if (CheckValue == 0)
	{
		Viewer->removeCoordinateSystem(0);
	}
	emitUpdateUI();
}

void CPointRenderingSetting::SetHeightColor()
{
	PointBase::SetPointColorByHeight(InputCloud);
	emitUpdateUI();
}

void CPointRenderingSetting::SetStemColor()
{	
	RenderingSetting.horizontalSlider_R->setValue(79);
	RenderingSetting.horizontalSlider_G->setValue(63);
	RenderingSetting.horizontalSlider_B->setValue(59);

	PointBase::SetPointColor(InputCloud, ColorBase::StemColor);
	emitUpdateUI();
}

void CPointRenderingSetting::ShowPointIndex(int CheckValue)
{
	if (InputCloud->points.size() > 500)
	{
		QMessageBox::information(NULL, tr("Information"),
			tr("There are two many points, much time need to be cost for point index showing! So, it is been canceled!"));
		return;
	}

	if (CheckValue == 2)
		PointBase::ShowPointXYZRGBText(Viewer, InputCloud, "Text", 0.05);
	else
		Viewer->removeText3D();
		
	emitUpdateUI();
}


void CPointRenderingSetting::CameraValueRead()
{
	disconnect(RenderingSetting.doubleSpinBox_posX, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_posY, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_posZ, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	disconnect(RenderingSetting.doubleSpinBox_viewA, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_viewB, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_viewC, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	disconnect(RenderingSetting.doubleSpinBox_Focal_X, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_Focal_Y, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_Focal_Z, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_Fov, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	disconnect(RenderingSetting.doubleSpinBox_clipA, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_clipB, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	disconnect(RenderingSetting.doubleSpinBox_winsize_Height, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_winsize_Width, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_win_posX, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	disconnect(RenderingSetting.doubleSpinBox_win_posY, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	vector<pcl::visualization::Camera> TempCameras;
	Viewer->getCameras(TempCameras);

	RenderingSetting.doubleSpinBox_posX->setValue(TempCameras[0].pos[0]);	//相机坐标
	RenderingSetting.doubleSpinBox_posY->setValue(TempCameras[0].pos[1]);
	RenderingSetting.doubleSpinBox_posZ->setValue(TempCameras[0].pos[2]);

	RenderingSetting.doubleSpinBox_viewA->setValue(TempCameras[0].view[0]);
	RenderingSetting.doubleSpinBox_viewB->setValue(TempCameras[0].view[1]);
	RenderingSetting.doubleSpinBox_viewC->setValue(TempCameras[0].view[2]);

	RenderingSetting.doubleSpinBox_Focal_X->setValue(TempCameras[0].focal[0]);
	RenderingSetting.doubleSpinBox_Focal_Y->setValue(TempCameras[0].focal[1]);
	RenderingSetting.doubleSpinBox_Focal_Z->setValue(TempCameras[0].focal[2]);
	RenderingSetting.doubleSpinBox_Fov->setValue(TempCameras[0].fovy);

	RenderingSetting.doubleSpinBox_clipA->setValue(TempCameras[0].clip[0]);
	RenderingSetting.doubleSpinBox_clipB->setValue(TempCameras[0].clip[1]);

	RenderingSetting.doubleSpinBox_winsize_Height->setValue(TempCameras[0].window_size[0]);
	RenderingSetting.doubleSpinBox_winsize_Width->setValue(TempCameras[0].window_size[1]);
	RenderingSetting.doubleSpinBox_win_posX->setValue(TempCameras[0].window_pos[0]);
	RenderingSetting.doubleSpinBox_win_posY->setValue(TempCameras[0].window_pos[1]);

	connect(RenderingSetting.doubleSpinBox_posX, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_posY, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_posZ, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	connect(RenderingSetting.doubleSpinBox_viewA, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_viewB, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_viewC, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	connect(RenderingSetting.doubleSpinBox_Focal_X, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_Focal_Y, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_Focal_Z, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_Fov, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	connect(RenderingSetting.doubleSpinBox_clipA, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_clipB, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));

	connect(RenderingSetting.doubleSpinBox_winsize_Height, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_winsize_Width, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_win_posX, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
	connect(RenderingSetting.doubleSpinBox_win_posY, SIGNAL(valueChanged(double)), this, SLOT(doubleSpinBox_ValueChanged(double)));
}

void CPointRenderingSetting::CameraValueChanged()
{
	//Viewer->getCameras(TempCameras);

	pcl::visualization::Camera TempCamera;

	TempCamera.pos[0] = posX;	//相机坐标
	TempCamera.pos[1] = posY;
	TempCamera.pos[2] = posZ;

	TempCamera.view[0] = viewA;	//相机视角
	TempCamera.view[1] = viewB;
	TempCamera.view[2] = viewC;

	TempCamera.focal[0] = focalX;
	TempCamera.focal[1] = focalY;
	TempCamera.focal[2] = focalZ;

	TempCamera.fovy = Fov;		//相机焦距
	TempCamera.clip[0] = clipA;	//相机裁切面
	TempCamera.clip[1] = clipB;

	TempCamera.window_size[0] = windows_size_height;	//窗口属性
	TempCamera.window_size[1] = windows_size_width;
	TempCamera.window_pos[0] = windows_posX;
	TempCamera.window_pos[1] = windows_posY;

	Viewer->setCameraParameters(TempCamera);
	Viewer->resetCamera();	
	Viewer->updateCamera();
}

void CPointRenderingSetting::doubleSpinBox_ValueChanged(double value) 
{
	posX = RenderingSetting.doubleSpinBox_posX->text().toDouble();
	posY = RenderingSetting.doubleSpinBox_posY->text().toDouble();
	posZ = RenderingSetting.doubleSpinBox_posZ->text().toDouble();

	viewA = RenderingSetting.doubleSpinBox_viewA->text().toDouble();
	viewB = RenderingSetting.doubleSpinBox_viewB->text().toDouble();
	viewC = RenderingSetting.doubleSpinBox_viewC->text().toDouble();

	focalX = RenderingSetting.doubleSpinBox_Focal_X->text().toDouble();
	focalY = RenderingSetting.doubleSpinBox_Focal_Y->text().toDouble();
	focalZ = RenderingSetting.doubleSpinBox_Focal_Z->text().toDouble();

	Fov = RenderingSetting.doubleSpinBox_Fov->text().toDouble();

	clipA = RenderingSetting.doubleSpinBox_clipA->text().toDouble();
	clipB = RenderingSetting.doubleSpinBox_clipB->text().toDouble();

	windows_size_height = RenderingSetting.doubleSpinBox_winsize_Height->text().toDouble();
	windows_size_width = RenderingSetting.doubleSpinBox_winsize_Width->text().toDouble();

	windows_posX = RenderingSetting.doubleSpinBox_win_posX->text().toDouble();
	windows_posX = RenderingSetting.doubleSpinBox_win_posY->text().toDouble();

	CameraValueChanged();
}