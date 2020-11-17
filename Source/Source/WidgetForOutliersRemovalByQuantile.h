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
	//������һ���ֽ��棺ѡ����Ͽ�
	//void createOptionsGroupBox();
	////�����ڶ����ֽ��棺��ť��Ͽ�
	//void createButtonBox();

	//��һ���֣�QGroupBox�ؼ�
	QGroupBox * optionsGroupBox;
	
	//Slice Thickness
	QLabel * SliceThicknessbuttonsLabel;
	QTextEdit * SliceThicknessbuttonsEdit;
	
	//Outliers Color
	QLabel * OutliersColorLabel;
	QComboBox * OutliersColorComboBox;

	//�ڶ����֣�QDialogButtonBox�ؼ�
	QDialogButtonBox * buttonBox;
	
	QPushButton * CloseButton;
	QPushButton * OutliersCheckButton;
	QPushButton * OutliersRemoveButton;	

	//������Ϣ
	//���岼��
	QGridLayout * mainLayout;
	//��һ�����еĿؼ�����
	QGridLayout * optionLayout;
	//�ڶ������еĿؼ�����
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
	//��������
	//������һ���ֽ��棺��ת�ؼ���Ͽ�
	void createRotableGroupBox();
	//�����ڶ����ֽ��棺������Ͽ�
	void createOptionsGroupBox();
	//�����������ֽ��棺��ť��Ͽ�
	void createButtonBox();

	//�ؼ�����
	//��һ���֣�QGroupBox�ؼ�
	QGroupBox *rotableGroupBox;
	QQueue<QWidget *> rotableWidgets;   //�ؼ�����

	//�ڶ����֣�QGroupBox�ؼ�
	QGroupBox *optionsGroupBox;
	QLabel *buttonsOrientationLabel;
	QComboBox *buttonsOrientationComboBox;

	//�������֣�QDialogButtonBox�ؼ�
	QDialogButtonBox *buttonBox;
	QPushButton *closeButton;
	QPushButton *helpButton;
	QPushButton *rotateWidgetsButton;

	//����
	//���岼��
	QGridLayout * mainLayout;
	//��һ�����еĿؼ�����
	QGridLayout * rotableLayout;
	//�ڶ������еĿؼ�����
	QGridLayout *optionsLayout;

signals:

public slots:
	//QComboBox��index�ı����currentIndexChanged()�źŶ�Ӧ�Ĳۺ���
	void buttonsOrientationChanged(int index);
	//����rotableGroupBox��Ͽ��е��ĸ��ӿؼ��Ķ�̬����
	void rotateWidgets();
	void help();

};
*/
#endif