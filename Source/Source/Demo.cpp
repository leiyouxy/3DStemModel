#include "Demo.h"

void CDemo::RefreshData()
{
	CTreeBase::RefreshData();
}

CDemo::CDemo(QGroupBox * ParentWin)
{
	QWidget *widget = new QWidget(ParentWin);
	DemoForm.setupUi(widget);

	widget->show();
}

CDemo::~CDemo()
{
	if (Viewer != NULL)
		Viewer->removePointCloud(DemoPointsStr);

	emitUpdateUI();
}