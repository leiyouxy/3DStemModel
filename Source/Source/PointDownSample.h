#pragma once

#include "TreeBase.h"
#include "CommPointBase.h"
#include <pcl/filters/voxel_grid.h>

#include "GeneratedFiles/ui_PointDownSample.h"

class CPointDownSample : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::PointDownSampleForm PointDownSampleForm;
	const string DemoPointsStr = "DemoPoints";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr DownSampleCloud;
signals:

public Q_SLOTS:
	void DownSample();
	void Redo();
public:
	CPointDownSample(QGroupBox * ParentWin);
	~CPointDownSample();
	void RefreshData();
};
