#pragma once

#include <thread>
#include <future>
#include <omp.h>

#include "CommClass.h"
#include "Spline.h"
#include "SplineInterpolation.h"
#include "StemSkeleton.h"
#include "AnglePartition.h"
#include "TreeBase.h"

/*

add by yleiou Construction of stem surface and and stem volume

*/

//�����ֵ����
class CSurfaceInterpolation : public CTreeBase
{
private:
	//InterpolationPoints �д洢���ǣ��������������ݵ�
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> InterpolationPoints;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> RowControlPoints;

	//�պ����߽ڵ� ���ݲ�ֵ��ڵ㼯�ϵĵ�һ���㹹������������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ClosedReferenceCurvePoint;

	//��� һ��ȫ�ֿ��Ƶ㣬
	bool ResolveOneGlobalControlPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue,
		vector<double> ParameterValues, vector<double> KnotValues,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints);

	int k, l; //k�в�ֵ�� and //l�в�ֵ��

	//���� RowIndex�� ColIndex ���� �� �ڵ����� RowU �� ColV �õ������ϵĵ�
	pcl::PointXYZRGB GetSurfacePoint(int RowIndex, int ColIndex, double RowU, double ColV);

	double RowStep, ColStep, HStep; //Step show surface points and calculation precision

	int dSpline;

	bool HorizontalIsClosed;

	double DisError;
	double eps;

	std::mutex g_lock;
	const int MaxThreadNum = 60;
	int ExecuteThreadNum;
	double StemVolume;

	//double ReferenceHeight;

	//�����ڵ�����ֵ���ڵ�����
	int FindUIndex(double u, vector<double> KnotValues, int StartIndex, int EndIndex);

	//���ݲ���ֵ �� �����ռ��� Ѱ�� ���� ���ַ�����׼ȷ
	void FindUVIndex(double RowU, double ColV, int & RowIndex, int & ColIndex);

	//���ݲ�ֵ�㼯�ϵĵ�һ���ڵ��� ColStep �����ıպ����ߵĿ��Ƶ�
	void ConstructClosedCurvePoint();

	//�ڹ����ıպ����ߵĿ��Ƶ����� ���� ColU �����ƽڵ�߶�λ�ô��� �����
	pcl::PointXYZRGB FindHNearestPoint(double RowU, double ColU);

	//�����еĽڵ�������Ѱ���ڸ���λ�ô� �����պ����ߵ� ��ʼ StartU �� EndU��
	void FindStartUandEndU(int RowIndex, double ColU,
		double & StartRowU, double & EndRowU, double DisError = 0.001, double UStep = 0.000001);

	//���ݸ߶�Zֵ��ȡ��Ӧλ�ô��� ������������߶�Ӧ�����
	double CalcAreaByHeight(double Z, int RowIndex, double DisError = EPSM3);
	
//	double CalcAreaByHeightByThread(double Z, int RowIndex, double DisError = 0.001);

	//���ݸ߶�Zֵ��ȡ��Ӧλ�ô��� �������������
	void GetCrossSectionalCurveByHeight(double Z, pcl::PointCloud<pcl::PointXYZRGB>::Ptr HeightPoints,
		double DisError = 0.001, double UStep = 0.001);

	//����uֵ���� u ���ַ�����ʽ���ʽ 2016.12.13 ���ݾ����Uֵ �� VValue ֵ�����������ض�VValueֵ�µı��ʽ
	virtual vector<string> GetUVFormula(double RowU, double ColV);

	double CalcZoneValueByThread(vector<string> FormulaS, double TempUStart, double TempUEnd);
public:
	CSurfaceInterpolation()
	{
		//GlobalControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		//RowControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);	
		ClosedReferenceCurvePoint.reset(new pcl::PointCloud<pcl::PointXYZRGB>);		
		dSpline = 3;
		HorizontalIsClosed = true;
		DisError = 0.001;
		eps = 1.0e-8;
	}
	~CSurfaceInterpolation();

	//���ݸ����߶�Ѱ�� ColU ��ֵ 2019.07.10
	double FindColUByHeight(double Z);

	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> SurfacePoints;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SplineSurfacePoints;

	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> GlobalControlPoints;

	vector<vector<double>> KnotValuesCol;
	//�ڵ����ֵ
	vector<vector<double>> ParametersCol;	

	vector<vector<double>> KnotValuesRow;
	//�ڵ����ֵ
	vector<vector<double>> ParametersRow;	

	//����ȫ�������ֵ
	void InterpolationSurface();

	//���ݶ��廭��ֵ����
	void DrawSurfaceByDefinition(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		double UStep = 0.001, double VStep = 0.001);

	void GetSurfacePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutSurfacePoints, 
		double UStep = 0.001, double VStep = 0.001);

	double CalcVolume(double StartZ, double EndZ);

	//The input is the formatted Interpolation points 
	void SetInputInterpolationPoints(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> InterpolationPointsValue,
		double HSetpValue = 0.05, double RowStepValue = 0.05, double ColStepValue = 0.05);
	
	////The input is the stem points that should be formatted to Interpolation points
	void SetInputStemPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemPointPtr, 
		float SectionThickNessValue, int CalcSectionNumbersValue, double AnlgeValue,
		double ZMax,
		double HSetpValue = 0.05, double RowStepValue = 0.05, double ColStepValue = 0.05);

	void ShowInterpolationPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer);

};
