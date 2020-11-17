#pragma once

/*
2019.02.22

Generate Points, Circle, Cylinder, Cone, etc,.

Leiyou
*/

#include "TreeBase.h"
#include "CommGeometry.h"

#include "GeneratedFiles/ui_GeneratePoints.h"

class CGeneratePoints : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::GeneratePointsForm GeneratePointsForm;

	void BatGeneratePoints();
	//生成方形 随机点 2020.05.28
	void Generate2DPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud, int Count);

	//生成圆状 随机点 2020.05.28
	void Generate2DCirclePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud, int Count);
signals:

public Q_SLOTS:
	void GenerateCirclePoints();
	void GenerateCylinderPoints();
	void GenerateConePoints();
	void GenerateParabolaPoints();	
	void GeneratePolygonPoints();
	void GeneratePlanePoints();
	
	void PolygonNumberChange(int Number);

	//随机生成 0 ~  Num * Mutiple 间不重复的 Num 个数的容器  2020.05.28
	void RandomVec(vector<double> & RanValues, int Num, int Mutiple = 10);
public:
	CGeneratePoints();
	CGeneratePoints(QGroupBox * ParentWin);
	~CGeneratePoints();

	void RefreshData();
};