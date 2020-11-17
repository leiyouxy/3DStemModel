#pragma once

#include "TreeBase.h"

#include "GeneratedFiles/ui_DelaunayParallel.h"

#include "ContourAndConvexHull.h"

#include "DelaunayGrowthAngleMaximizer.h"
//#include "DelaunayGrowthWithHalfEdge.h"
#include "DividePointsByDelaunayLine.h"

class CDelaunayParallel : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::FormDelaunayParallel DelaunayParallelForm;
	const string DelaunayParallelStr = "DelaunayParallelPoints";

	vector<int> ConvexhullIndexs;
	CContourAndConvexHull<pcl::PointXYZRGB> ConvexHull;
	bool IsSave;
	int BatNum;
	string SaveStr, SaveFileName;

	double SumTimeHalfEdgeGrow, SumTimeParaDivideDis, SumTimeParaGrowth;
	double K, AvgNum, MinNum, MaxNum, VarianceNum;

	//将Mesh另存为到文件中    2020.06.24
	void SaveMeshToFile(pcl::PolygonMesh CurMesh, string FileName);

signals:

public Q_SLOTS:

	//2020.05.23 计算凸包
	void ConvexHullCom();
	void DelaunayGrowth();

	//并行计算Delaunay 2020.06.07
	void DelaunayParallelGrowth();

	//批量处理以取得计算结果 2020.06.19
	void BatRun();
public:	

	CDelaunayParallel(QGroupBox * ParentWin);
	~CDelaunayParallel();
	void RefreshData();
};