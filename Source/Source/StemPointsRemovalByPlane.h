#pragma once
#ifndef StemPointsRemovalByPlane_H
#define StemPointsRemovalByPlane_H

/*
stem points removal by plane equation 
by leiyou 2018.12.25
*/

#include "Commdefinitions.h"
#include "CommPointBase.h"
#include "CommVector.h"
#include "CommGeometry.h"

#include <QMainWindow>
#include <QMessageBox>
#include <QGroupBox>

#include "TreeBase.h"
#include "HorizontalPartition.h"

#include "GeneratedFiles/ui_StemPointsRemoval.h"

class CStemPointsRemovalByPlane	: public CTreeBase
{
	Q_OBJECT
public Q_SLOTS:	
	//the function for get axis str, such as x, y, z, the value of comboBoxAxis
	void GeneratePlaneEquation(QString AxisStr);

	void DrawPlanPoints(int checkValue);

	//find a suitable point for show plane by the domain of intput points
	pcl::PointXYZRGB FindPlaneCenterPointByPointsZone();

	//draw arrow points as the addrow function of Viewer is error when used
	void DrawArrow(int Index);

	void RemovalPoints();

	void Redo();

	void Parttition();

	void Bat();
public:
	CStemPointsRemovalByPlane(QGroupBox * ParentWin);
	~CStemPointsRemovalByPlane();
	void RefreshFormParameters();

	void RefreshData();
private:
	Ui::StemPointsRemovalForm StemPointsRemoval;

	//the coefficient for plane equation
	double a;
	double b;
	double c;
	double d;

	double MaxDis;

	//the point for genetater plane points and arrow points
	pcl::PointXYZRGB PlaneCenterPoint;

	//the point for removal points
	pcl::PointXYZRGB ReferenceCenterPoint;

	//get the coefficients of a plane by plane experssion
	void GetPlaneCoefficients(string EquationStr);

};

#endif