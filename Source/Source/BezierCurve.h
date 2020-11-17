//////�� BezierCurve 
#ifndef BezierCurve_H
#define BezierCurve_H

#include "CommClass.h"
#include "CommGeometry.h"
#include "FormulaAnalytic.h"

class CBezierCurve
{		//��ֵ��
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPoints;

	double UStep;

	double BezierPolynomial(int n, int i);

	//���Ƶ�ĸ���
	int kCurve;

	//����K�� Bernstein����ʽ��U����ֵ 
	double GetBernsteinValue(int K, int i, double U);

	//����K�� Bernstein����ʽ��U����ֵ �ĵ���ֵ
	double GetBernsteinDerivativeValue(int K, int i, double U, int Order = 1);

	vector<string> PointSStr;

	vector<double> PointUValues;
public:
	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue);

	//��ȡbezier���� ���õ����߳���
	double GetBezierCurve(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePointsValue, 
		double UStepValue = EPSM3);

	// ���� U λ�ô��ĵ�
	pcl::PointXYZRGB GetBezierPoint(double u);

	//���� U λ�ô���һ�׵���
	pcl::PointXYZRGB GetBezierDerivativePoint(double u, int Order = 1);

	//����Bezier���߳��ȣ�����Simpson��ʽ
	double GetBezierLengthBySimpson();

	//2016.04.17 ����Bezier������Χ�������
	double GetBezierArea();

	//��ȡ���������ϵĵ��Ӧ������
	vector<string> GetUFormula();

	//��ȡ������Ӧ�ļ����ַ���
	vector<string> GetDerivativeStr(int Order = 1);

	vector<double> GetPointsUValues();

	double GetCurvature(double U, bool Relative = true);
};

#endif