#pragma once

#ifndef WidgetForOutliersRemovalByQuantile_H
#define WidgetForOutliersRemovalByQuantile_H


#include <QtWidgets\QWidget>
#include <QtWidgets\QDialogButtonBox>
#include <QtWidgets\QGroupBox>
#include <QtWidgets\QLabel>
#include <QtWidgets\QTextEdit>
#include <QtWidgets\QComboBox>
#include <QtWidgets\QGridLayout>
#include <QtWidgets\QMessageBox>

class RemovalByQuantileWidgetManage :public QWidget
{
	Q_OBJECT
private:
	//创建第一部分界面：选项组合框
	//void createOptionsGroupBox();
	////创建第二部分界面：按钮组合框
	//void createButtonBox();

	//第一部分：QGroupBox控件
	QGroupBox * optionsGroupBox;
	
	//Slice Thickness
	QLabel * SliceThicknessbuttonsLabel;
	QTextEdit * SliceThicknessbuttonsEdit;
	
	//Outliers Color
	QLabel * OutliersColorLabel;
	QComboBox * OutliersColorComboBox;

	//第二部分：QDialogButtonBox控件
	QDialogButtonBox * buttonBox;
	
	QPushButton * CloseButton;
	QPushButton * OutliersCheckButton;
	QPushButton * OutliersRemoveButton;	

	//布局信息
	//总体布局
	QGridLayout * mainLayout;
	//第一部分中的控件布局
	QGridLayout * optionLayout;
	//第二部分中的控件布局
	QGridLayout * buttonLayout;

	//ParentBox for components
	QGroupBox * ParentBox;

signals:

private slots:
	void OutliersChecking();
	void OutliersRemoval();

public:
	void SetParentBoxHandle(QGroupBox * ParentBoxValue);
};

/*
class RemovalByQuantileWidget : public QWidget
{
	Q_OBJECT
public:
	explicit RemovalByQuantileWidget(QWidget *parent = 0);

private:
	//函数部分
	//创建第一部分界面：旋转控件组合框
	void createRotableGroupBox();
	//创建第二部分界面：方向组合框
	void createOptionsGroupBox();
	//创建第三部分界面：按钮组合框
	void createButtonBox();

	//控件部分
	//第一部分：QGroupBox控件
	QGroupBox *rotableGroupBox;
	QQueue<QWidget *> rotableWidgets;   //控件队列

	//第二部分：QGroupBox控件
	QGroupBox *optionsGroupBox;
	QLabel *buttonsOrientationLabel;
	QComboBox *buttonsOrientationComboBox;

	//第三部分：QDialogButtonBox控件
	QDialogButtonBox *buttonBox;
	QPushButton *closeButton;
	QPushButton *helpButton;
	QPushButton *rotateWidgetsButton;

	//布局
	//总体布局
	QGridLayout * mainLayout;
	//第一部分中的控件布局
	QGridLayout * rotableLayout;
	//第二部分中的控件布局
	QGridLayout *optionsLayout;

signals:

public slots:
	//QComboBox中index改变产生currentIndexChanged()信号对应的槽函数
	void buttonsOrientationChanged(int index);
	//构建rotableGroupBox组合框中的四个子控件的动态布局
	void rotateWidgets();
	void help();

};
*/
#endif