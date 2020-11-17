#pragma once

#ifndef PointRenderingSetting_H
#define PointRenderingSetting_H

#include "TreeBase.h"

#include "GeneratedFiles/ui_RenderingSetting.h"
#include "../TreePclQtGui.h"

//extern class TreePclQtGui;

class CPointRenderingSetting : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:	

	unsigned int red;
	unsigned int green;
	unsigned int blue;

	//vector<pcl::visualization::Camera> TempCameras;	
	double posX, posY, posZ;	//���λ��
	double viewA, viewB, viewC;	//�ӽ�
	double clipA, clipB;	//ǰ�������
	double Fov;		//�ӳ���
	double focalX, focalY, focalZ;	//�������
	double windows_size_height, windows_size_width;	//���ڴ�С
	double windows_posX, windows_posY;		//��������

signals:
	void UpdateUI();
	void UpdateStatus(QString, int);

public Q_SLOTS:
	void randomButtonPressed();

	void RGBsliderReleased();

	void pSliderValueChanged(int value);

	void redSliderValueChanged(int value);

	void greenSliderValueChanged(int value);

	void blueSliderValueChanged(int value);

	void Redo();

	void ShowCoordinateAxis(int CheckValue);

	void SetHeightColor();

	void SetStemColor();

	void ShowPointIndex(int CheckValue);

	//Camera Properties Setting
	void CameraValueChanged();
	//void pushButton_GetPositionPressed();
	void doubleSpinBox_ValueChanged(double value);	
public:
	Ui::RenderingSettingForm RenderingSetting;

	void CameraValueRead();
	CPointRenderingSetting();
	CPointRenderingSetting(QGroupBox * ParentWin);
	~CPointRenderingSetting();
};

#endif