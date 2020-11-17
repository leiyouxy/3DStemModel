#ifndef BezierSubsectionG2_H
#define BezierSubsectionG2_H

///2015.12.11�ֶι������ι⻬�ıպ�Bezier����

#include "CommClass.h"
#include "BezierCurve.h"
//#include "AnglePartition.h"
//#include "FormulaAnalytic.h"
#include "CommVector.h"

class CBezierSubsectionG2
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPoints;

	vector<long double> MaxCoefficientValue;

	vector<long double> CoefficientValue;

	vector<long double> CoefficientRate0fOneAndFour;

	//����
	void CalcCoefficientRateAndMaxCoefficientValue();
	long double Size;

	//�����PointIndex����ĵ�Order�׵��� ����������
	pcl::PointXYZRGB CalcPointDerivative(int PointIndex, int Order, int Position);
	
	//���ص�������ֵ��
	double CalcPointDerivativeValue(int PointIndex, int Order, int Position);

	//�������ϵ�� ���ݷ�����
	void ResolveCoefficientByEquations();

	///������������������Ϊ֮ǰ�ϵĳ��� ��Ϊ2015.12.31��д���� ����ʦ���뷨

	long double U;

	vector<long double> UU;	//��Ӧ�� minu
	vector<long double> AA;	//��Ӧ��
	vector<long double> BB;	//��Ӧ�� beita
	vector<long double> EE;	//��Ӧ��e
	vector<long double> FF;	//��Ӧ��f
	vector<long double> KK;	//��Ӧ��k
	vector<long double> GG;	//��Ӧ��g
	vector<long double> RR;	//��Ӧ�� ���Բ�ʹ��

	//��֮�乹�ɵ����� VectorofQPoints[i] = p(i+1) - p(i)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr VectorofQPoints;
	
	//�����ڵ��������  �������α���	 2015.12.31 
	//2015.01.26 �޸�Ϊ ��̬�ж� EE ֵ�Ƿ�Ϊ0 ���Ϊ0 ��ɾ����Ӧ�� i+1 �Ľڵ�
	bool GetVectorofQPointsAndOthers();

	//��������������������Ƶ� 2015.12.31
	void ResolveControlPoints();	

	//���е��Ӧ��Uֵ //2016.01.05
	vector<double> AllPointUValues;

	//ÿ�ο�ʼ�Ľڵ����� //2016.01.05
	vector<int> SectionStartPointIndex;

	////�жϴ˵��Ƿ���͹�����ϵĵ� 	//2016.01.05
	//bool IsConvexHullPoint(int AngleIndex, pcl::PointXYZRGB TangentLinePoint,
	//	vector<AnglePartitionStruct> & SectionAngle);

	//2016.01.05
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalBezierCurvePoints;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr NoConvexhullPoints;

	//�����������ߵ����� 	//2016.01.05
	pcl::PointXYZRGB ResolveTangentLine(int PointIndex);

	//����(Li) X (Li-1)����0������
	void FilterPoints();
public:
	CBezierSubsectionG2()
	{
		QPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		ControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		VectorofQPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		//NoConvexhullPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints;

	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, double SizeValue = 0.8);

	//������Ƶ�
	void ResolveControlPoints(double UValue = BezierMiu);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr DrawBroderPoints();

	double DrawBezierCurve(pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierCurvePoints, int PointNum = -1);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;

	void ShowPointDerivative();

	////����㹹�����ߵ�͹��
	//void ComputeConvexPropertyOfCurve();

	//��ȡ�ض��������
	double GetCurvature(int PointIndex);

	double GetBezierLength();

	double GetBezierArea();
};

#endif