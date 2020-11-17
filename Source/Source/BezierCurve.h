//////画 BezierCurve 
#ifndef BezierCurve_H
#define BezierCurve_H

#include "CommClass.h"
#include "CommGeometry.h"
#include "FormulaAnalytic.h"

class CBezierCurve
{		//插值点
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPoints;

	double UStep;

	double BezierPolynomial(int n, int i);

	//控制点的个数
	int kCurve;

	//计算K次 Bernstein多项式在U处的值 
	double GetBernsteinValue(int K, int i, double U);

	//计算K次 Bernstein多项式在U处的值 的导数值
	double GetBernsteinDerivativeValue(int K, int i, double U, int Order = 1);

	vector<string> PointSStr;

	vector<double> PointUValues;
public:
	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue);

	//获取bezier曲线 并得到曲线长度
	double GetBezierCurve(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePointsValue, 
		double UStepValue = EPSM3);

	// 计算 U 位置处的点
	pcl::PointXYZRGB GetBezierPoint(double u);

	//计算 U 位置处的一阶导数
	pcl::PointXYZRGB GetBezierDerivativePoint(double u, int Order = 1);

	//计算Bezier曲线长度，根据Simpson公式
	double GetBezierLengthBySimpson();

	//2016.04.17 计算Bezier曲线所围区域面积
	double GetBezierArea();

	//获取计算曲线上的点对应的曲线
	vector<string> GetUFormula();

	//获取导数对应的计算字符串
	vector<string> GetDerivativeStr(int Order = 1);

	vector<double> GetPointsUValues();

	double GetCurvature(double U, bool Relative = true);
};

#endif