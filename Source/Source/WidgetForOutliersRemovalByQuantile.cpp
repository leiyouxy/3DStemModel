#include "WidgetForOutliersRemovalByQuantile.h"

void RemovalByQuantileWidgetManage::SetParentBoxHandle(QGroupBox * ParentBoxValue)
{
	ParentBox = ParentBoxValue;
	
	mainLayout = new QGridLayout;

	optionsGroupBox = new QGroupBox();
	optionsGroupBox->setTitle("Parameters or Options");
	mainLayout->addWidget(optionsGroupBox, 0, 0);

	buttonBox = new QDialogButtonBox;

	CloseButton = buttonBox->addButton(QDialogButtonBox::Close);
	
	OutliersCheckButton = buttonBox->addButton(("Outliers Checking"),
		QDialogButtonBox::ActionRole);
	OutliersRemoveButton = buttonBox->addButton(("Outliers Points Removing"),
		QDialogButtonBox::ActionRole);

	//connect(rotateWidgetsButton, SIGNAL(clicked()), this, SLOT(rotateWidgets()));
	connect((QObject *)CloseButton, SIGNAL(clicked()), this, SLOT(close()));
	connect((QObject *)OutliersRemoveButton, SIGNAL(clicked()), this, SLOT(OutliersRemoval()));

	mainLayout->addWidget(buttonBox, 1, 0);	
	ParentBox->setLayout(mainLayout);	
}

void RemovalByQuantileWidgetManage::OutliersChecking()
{

}

void RemovalByQuantileWidgetManage::OutliersRemoval()
{		
	//QMessageBox::information(this, tr("Dynamic Layouts Help"),
	//	tr("This example shows how the change layouts dynamically."));
}

/*

RemovalByQuantileWidget::RemovalByQuantileWidget(QWidget *parent) :
	QWidget(parent)
{
	createRotableGroupBox();
	createOptionsGroupBox();
	createButtonBox();

	mainLayout = new QGridLayout;
	//�����������3����Ͽ�ؼ�
	mainLayout->addWidget(rotableGroupBox, 0, 0);
	mainLayout->addWidget(optionsGroupBox, 1, 0);
	mainLayout->addWidget(buttonBox, 2, 0);
	setLayout(mainLayout);

	//���ֵĴ�С����(Constraint)ģʽ��������С��С
	mainLayout->setSizeConstraint(QLayout::SetMinimumSize);

	setWindowTitle(tr("Dynamic layouts"));
}


void RemovalByQuantileWidget::createRotableGroupBox()
{
	rotableGroupBox = new QGroupBox(tr("Rotable Widgets"));

	//rotableWidgets.enqueue(new QSpinBox);   //΢�������
	//rotableWidgets.enqueue(new QSlider);    //����
	//rotableWidgets.enqueue(new QDial);  //�̶���
	//rotableWidgets.enqueue(new QProgressBar);   //������

	//�ӿؼ�����
	int n = rotableWidgets.count();

	//�ĸ��ؼ���β��������ϵ�γ�һ�����Σ�һ���ؼ�����ֵ�ı���֮����Ŀؼ�Ҳ�ı�
	for (int i = 0; i < n; ++i)
	{
		connect(rotableWidgets[i], SIGNAL(valueChanged(int)),
			rotableWidgets[(i + 1) % n], SLOT(setValue(int)));
	}

	rotableLayout = new QGridLayout;

	//QGroupBox��Ͽ�ؼ�����Ͽ�ؼ��м��������ӿؼ���
	rotableGroupBox->setLayout(rotableLayout);

	//����rotableGroupBox��Ͽ��е��ĸ��ӿؼ��Ķ�̬����
	rotateWidgets();
}


void RemovalByQuantileWidget::rotateWidgets()
{
	//����
	Q_ASSERT(rotableWidgets.count() % 2 == 0);

	//��˳����������еĶ��󣬱��������еĿؼ�
	foreach(QWidget *widget, rotableWidgets)
		rotableLayout->removeWidget(widget);    //ɾ�������е��ӿؼ�����̬����

	//�ؼ������е�ͷ���ؼ����Ӻ����β
	rotableWidgets.enqueue(rotableWidgets.dequeue());

	const int n = rotableWidgets.count();   //4
	for (int i = 0; i < n / 2; ++i)
	{
		//��ӿؼ����������У���̬���֣������ؼ���ʾ��λ��
		rotableLayout->addWidget(rotableWidgets[n - i - 1], 0, i);  //3,(0,0)   2,(0,1)
		rotableLayout->addWidget(rotableWidgets[i], 1, i);  //0,(1,0)   1,(1,1)
	}
}


void RemovalByQuantileWidget::createOptionsGroupBox()
{
	optionsGroupBox = new QGroupBox(tr("Options"));

	buttonsOrientationLabel = new QLabel(tr("Orientation of buttons:"));

	buttonsOrientationComboBox = new QComboBox;
	buttonsOrientationComboBox->addItem(tr("Horizontal"), Qt::Horizontal);
	buttonsOrientationComboBox->addItem(tr("Vertical"), Qt::Vertical);

	//QComboBox�е�����ֵ(index)�ı���ź����ӵ��ۺ���
	connect(buttonsOrientationComboBox, SIGNAL(currentIndexChanged(int)),
		this, SLOT(buttonsOrientationChanged(int)));

	optionsLayout = new QGridLayout;
	optionsLayout->addWidget(buttonsOrientationLabel, 0, 0);
	optionsLayout->addWidget(buttonsOrientationComboBox, 0, 1);
	//���õڶ��е���������(strech factor)
	optionsLayout->setColumnStretch(2, 1);
	optionsGroupBox->setLayout(optionsLayout);
}


void RemovalByQuantileWidget::buttonsOrientationChanged(int index)
{
	mainLayout->setSizeConstraint(QLayout::SetNoConstraint);
	setMinimumSize(0, 0);

	//��ȡQComboBox�еķ�����Ϣ
	Qt::Orientation orientation = Qt::Orientation(
		buttonsOrientationComboBox->itemData(index).toInt());

	//�жϷ����Ƿ�ı�
	if (orientation == buttonBox->orientation())
		return;

	//����buttonBox�Ĳ���
	mainLayout->removeWidget(buttonBox);

	int spacing = mainLayout->spacing();

	QSize oldSizeHint = buttonBox->sizeHint() + QSize(spacing, spacing);
	//���ð�ť��Ĳ��ַ���
	buttonBox->setOrientation(orientation);
	QSize newSizeHint = buttonBox->sizeHint() + QSize(spacing, spacing);

	if (orientation == Qt::Horizontal)
	{
		mainLayout->addWidget(buttonBox, 2, 0);
		resize(size() + QSize(-oldSizeHint.width(), newSizeHint.height()));
	}
	else
	{
		mainLayout->addWidget(buttonBox, 0, 3, 2, 1);
		resize(size() + QSize(newSizeHint.width(), -oldSizeHint.height()));
	}

	mainLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
}


void RemovalByQuantileWidget::createButtonBox()
{
	buttonBox = new QDialogButtonBox;

	closeButton = buttonBox->addButton(QDialogButtonBox::Close);
	helpButton = buttonBox->addButton(QDialogButtonBox::Help);
	rotateWidgetsButton = buttonBox->addButton(tr("Rotate &Widgets"),
		QDialogButtonBox::ActionRole);

	connect(rotateWidgetsButton, SIGNAL(clicked()), this, SLOT(rotateWidgets()));
	connect(closeButton, SIGNAL(clicked()), this, SLOT(close()));
	connect(helpButton, SIGNAL(clicked()), this, SLOT(help()));
}


void RemovalByQuantileWidget::help()
{
	QMessageBox::information(this, tr("Dynamic Layouts Help"),
		tr("This example shows how the change layouts dynamically."));
}

*/