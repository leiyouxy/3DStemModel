#pragma once

#include "TreeBase.h"

#include "../GeneratedFiles/ui_Cluster.h"

#include "DBScanCluster.h"
#include "K_MeansCluster.h"

class CCluster : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::FormCluster ClusterForm;
	const string DemoPointsStr = "ClusterPoints";
	CDBScanCluster DBScanCluster;
	CK_MeansCluster K_MeansCluster;

signals:

public Q_SLOTS:
	void SHowClusterPoints(int CheckStatu);
	void RunK_MeansCluster();
	void DBScan();	
public:	
	CCluster(QGroupBox * ParentWin);
	~CCluster();
	void RefreshData();
};