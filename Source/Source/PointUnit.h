#pragma once

#include <QDialog>

#include "CommPointBase.h"
#include "GeneratedFiles/ui_PointUnit.h"

class CPointUnit : public QDialog
{
	Q_OBJECT
private:
	Ui::PointUnitDialog PointUnitDialog;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;
	double Mutiple;
public:
	CPointUnit(QWidget *parent = Q_NULLPTR);
	~CPointUnit();
	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue);
public Q_SLOTS:
	void SetUnit(QString UnitStr);
	void Ok();
};