#include "PointUnit.h"

CPointUnit::CPointUnit(QWidget *parent)
	:QDialog(parent, Qt::WindowTitleHint | Qt::CustomizeWindowHint)
{
	PointUnitDialog.setupUi(this);

	//this->show();
	PointUnitDialog.comboBoxUnit->addItem("meter");
	PointUnitDialog.comboBoxUnit->addItem("decimetre");
	PointUnitDialog.comboBoxUnit->addItem("centimeter");
	PointUnitDialog.comboBoxUnit->addItem("millimeter");
	QObject::connect(PointUnitDialog.comboBoxUnit, SIGNAL(currentIndexChanged(QString)),
		this, SLOT(SetUnit(QString)));
	QObject::connect(PointUnitDialog.pushButtonOk, SIGNAL(clicked()),
		this, SLOT(Ok()));
	PointUnitDialog.comboBoxUnit->setCurrentText("meter");
	Mutiple = 100.0;
}

CPointUnit::~CPointUnit()
{

}

void CPointUnit::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue)
{
	InputCloud = InputCloudValue;
	double MaxX, MaxY, MaxZ, MinX, MinY, MinZ;
	PointBase::GetPointsMaxAndMin(InputCloudValue, MaxX, MinX, MaxY, MinY, MaxZ, MinZ);
	double Value = abs(MaxZ - MinZ);
	if (Value <= 50) //meter
	{
		PointUnitDialog.comboBoxUnit->setCurrentIndex(0);
		Mutiple = 100.0;
	}
	else if (Value >= 80)//centimeter
	{
		PointUnitDialog.comboBoxUnit->setCurrentIndex(2);
		Mutiple = 1.0;
	}
}

void CPointUnit::SetUnit(QString UnitStr)
{
	if (UnitStr == "meter")
	{		
		Mutiple = 100.0;
	}
	else if (UnitStr == "decimetre")
	{	
		Mutiple = 10.0;
	}
	else if (UnitStr == "centimeter")
	{	
		Mutiple = 1.0;
	}
	else if (UnitStr == "millimeter")
	{		
		Mutiple = 0.1;
	}		
}

void CPointUnit::Ok()
{
	if (abs(Mutiple) <= EPSM6)
	{
		SetUnit(PointUnitDialog.comboBoxUnit->currentText());
	}

	if (abs(Mutiple - 1) > EPSM6)
	{
		//cout << "PointZoom(InputCloud, Mutiple), Mutiple:" << Mutiple << endl;
		PointBase::PointZoom(InputCloud, Mutiple);
	}

	this->close();
}