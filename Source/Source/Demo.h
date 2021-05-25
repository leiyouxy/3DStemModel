#pragma once

#include "TreeBase.h"

#include "GeneratedFiles/ui_Demo.h"
#include "GeneratedFiles/ui_PlotProcessing.h"

class CDemo : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::FormDemo DemoForm;
	const string DemoPointsStr = "DemoPoints";

signals:

public Q_SLOTS:


public:	
	CDemo(QGroupBox * ParentWin);
	~CDemo();
	void RefreshData();
};