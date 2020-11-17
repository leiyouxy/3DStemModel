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
	//主布局中添加3个组合框控件
	mainLayout->addWidget(rotableGroupBox, 0, 0);
	mainLayout->addWidget(optionsGroupBox, 1, 0);
	mainLayout->addWidget(buttonBox, 2, 0);
	setLayout(mainLayout);

	//布局的大小限制(Constraint)模式：设置最小大小
	mainLayout->setSizeConstraint(QLayout::SetMinimumSize);

	setWindowTitle(tr("Dynamic layouts"));
}


void RemovalByQuantileWidget::createRotableGroupBox()
{
	rotableGroupBox = new QGroupBox(tr("Rotable Widgets"));

	//rotableWidgets.enqueue(new QSpinBox);   //微调器入队
	//rotableWidgets.enqueue(new QSlider);    //滑块
	//rotableWidgets.enqueue(new QDial);  //刻度盘
	//rotableWidgets.enqueue(new QProgressBar);   //进度条

	//子控件个数
	int n = rotableWidgets.count();

	//四个控件首尾相连，关系形成一个环形，一个控件的数值改变随之后面的控件也改变
	for (int i = 0; i < n; ++i)
	{
		connect(rotableWidgets[i], SIGNAL(valueChanged(int)),
			rotableWidgets[(i + 1) % n], SLOT(setValue(int)));
	}

	rotableLayout = new QGridLayout;

	//QGroupBox组合框控件，组合框控件中加入其它子控件，
	rotableGroupBox->setLayout(rotableLayout);

	//构建rotableGroupBox组合框中的四个子控件的动态布局
	rotateWidgets();
}


void RemovalByQuantileWidget::rotateWidgets()
{
	//断言
	Q_ASSERT(rotableWidgets.count() % 2 == 0);

	//按顺序遍历容器中的对象，遍历队列中的控件
	foreach(QWidget *widget, rotableWidgets)
		rotableLayout->removeWidget(widget);    //删除布局中的子控件，动态布局

	//控件队列中的头部控件出队后，入队尾
	rotableWidgets.enqueue(rotableWidgets.dequeue());

	const int n = rotableWidgets.count();   //4
	for (int i = 0; i < n / 2; ++i)
	{
		//添加控件到布局类中，动态布局，交换控件显示的位置
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

	//QComboBox中的索引值(index)改变的信号连接到槽函数
	connect(buttonsOrientationComboBox, SIGNAL(currentIndexChanged(int)),
		this, SLOT(buttonsOrientationChanged(int)));

	optionsLayout = new QGridLayout;
	optionsLayout->addWidget(buttonsOrientationLabel, 0, 0);
	optionsLayout->addWidget(buttonsOrientationComboBox, 0, 1);
	//设置第二列的延伸属性(strech factor)
	optionsLayout->setColumnStretch(2, 1);
	optionsGroupBox->setLayout(optionsLayout);
}


void RemovalByQuantileWidget::buttonsOrientationChanged(int index)
{
	mainLayout->setSizeConstraint(QLayout::SetNoConstraint);
	setMinimumSize(0, 0);

	//获取QComboBox中的方向信息
	Qt::Orientation orientation = Qt::Orientation(
		buttonsOrientationComboBox->itemData(index).toInt());

	//判断方向是否改变
	if (orientation == buttonBox->orientation())
		return;

	//控制buttonBox的布局
	mainLayout->removeWidget(buttonBox);

	int spacing = mainLayout->spacing();

	QSize oldSizeHint = buttonBox->sizeHint() + QSize(spacing, spacing);
	//设置按钮框的布局方向
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