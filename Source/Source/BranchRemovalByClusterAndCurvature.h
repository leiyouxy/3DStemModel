#pragma once

#include "TreeBase.h"

#include "GeneratedFiles/ui_BranchRemovalByClusterAndCurvature.h"

#include "DBScanCluster.h"
#include "PointGeometry.h"
#include "HorizontalPartition.h"

class CBranchRemovalByClusterAndCurvature : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::FormBranchRemovalByClusterAndCurvature Form;
	const string PointClusterPointsStr = "PointClusterPoints";

	CDBScanClusterWithCurvatrue DBScanCluster;
	//CDBScanClusterWithSlice DBScanClusterWithSlice;
	CPointGeometry PointGeometry;

	void RefreshData();
	double MeanThresold; 


signals:

public Q_SLOTS:
	void DBClusterforPointDis();

	void RetrievalMainTree();

	void NormalComputation();

	void ShowNormal(int State);

	void ShowClusterPoints(int State);

	void ButtonBranchCheck();

	void RemoveBranches();

	void NormalCheck();
public:	
	CBranchRemovalByClusterAndCurvature(QGroupBox * ParentWin);
	~CBranchRemovalByClusterAndCurvature();

	//2019.09.24 获取高斯曲率变化比较大的点，该店的曲率值显著高于邻域点的曲率值
	//void GetCurvatureVariationSignificantly(vector<int> IndexS);
};