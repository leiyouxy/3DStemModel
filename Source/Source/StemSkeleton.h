#ifndef StemSkeleton_H
#define StemSkeleton_H

//2016.04.05  ������ȡ��������������ֱ�ķֶμ��ֶε����ĵ㣬

#include "CommClass.h"
#include "HorizontalPartition.h"
#include "AnglePartition.h"
#include "Commdefinitions.h"
#include "SplineInterpolation.h"
#include "Spline.h"
#include "StemDiameter.h"

using namespace std;

class CStemSkeleton
{
private:	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemCloud;		//���ɵ���
	float SectionThickNess;									//�ֶκ��
	//float MiniError;										//����������С�����
	int CalcSectionNumbers;									//���ٸ��ֶ�һ�����
	int StartIndex;
	int TheIncludedAngle;
	bool IsOptimized;

	double CurrentHeightShow;								//2018.10.31 Ϊ����������ʾ�ĺ��������ļ����趨�ĸ߶�
	
	//�������ֵ����Сֵ
	void GetMaxAndMinValueFromPointsAndNormal(pcl::PointCloud<PointXYZRGBIndex>::Ptr TempCloud,
		pcl::PointXYZRGB InitialNormal, float & MinValue, float & MaxValue);
public:
	//�����ʼ��������������
	pcl::PointXYZRGB GetInitialVector();

	//������һ��ƽ���ϵ�һ����
	pcl::PointXYZRGB GetSlicePoints(pcl::PointXYZRGB NormalPoint, pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints,
		pcl::PointXYZRGB & SliceCentroidPoint, bool IsUp = true, vector<int> * Indexs = NULL);

	//2019.06.15 ��ǰλ�õ�ǰһ��Slice
	void GetPriorSlicePoints(pcl::PointXYZRGB NormalPoint, pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints,
		pcl::PointXYZRGB & SliceCentroidPoint, bool IsUp = true, vector<int> * Indexs = NULL, int SliceNum = 1);

	//��ȡ�㼯�����ĵ�
	pcl::PointXYZRGB GetCentroidOfPointsInSpaces(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
		pcl::PointXYZRGB NormalPoint, bool & IsCircle);
private:
	//�������ĵ��жϵ�ǰƽ��㼯�Ƿ���Բ��
	bool PlanePointsIsCircle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints, 
		pcl::PointXYZRGB Centriod);

//����ʽ��ȡ���ɹǼ�
	void GetSkeletonPointS();

	void UseSlicesSkeleton();

	//void AngleSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints, double Angle = 1);

	//double GetCrossSectionalProfileAndArea(
	//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CrossSectionalProfilePtr,
	//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurve, bool IsArea = false);	//��ȡ������������� �� ��������

	//void ShowCrossSectionalPlaneByCircle(pcl::PointXYZRGB CenterPoint, pcl::PointXYZRGB NormalPoint, 
	//	double Radius, int PointSize = 1,
	//	int r = 255, int g = 0, int b = 0 );

public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CentroidPoints;		//���ɵĹǼܵ�
private:
	double MaximumHeight;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OptimizedCentroidPoints;		//���ɵĹǼܵ�
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemSkeletonPoints;	//���ɹǼܵ�������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AllProfilecurvePoints;	//���ɹǼܵ�������
	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;
	
	//SectionVector SectionsVector;

	//is the input cloud has points from stem tip, if not, 2019.1.23
	bool IsHaveTip;
	void IsHaveTipPoints();
public:	
	CStemSkeleton()
	{
		IsHaveTip = true;
		StemCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		CentroidPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		OptimizedCentroidPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		StemSkeletonPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		AllProfilecurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		//UnPartitedCloud.reset(new pcl::PointCloud<PointXYZRGBIndex>);
	}
	string ResultStr;
	CSpline StemSkeletonSpline;
	CHorizontalPartition HorizontalPartition;

	// IsOptimizedValue ��ʾ �Ƿ�Լ������ĵ�����Ż���
	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemCloudValue,
		float SectionThickNessValue = 0.5, bool IsOptimizedValue = true, int CalcSectionNumbersValue = 10,
		int StartIndexValue = 0, int TheIncludedAngleValue = 60, double MaximumHeightValue = 0);
	
	//�����Ǽܵ���������
	void ConstructStemSplineCurve(double StartHeight = 100, bool IsDirect = false);

	////��ָ����λ�ô��������  �������Ӧ�����ɹǼ��ƶ�ǰ���
	//void DrawCrossSectionalPlaneByHeight(float Height, int LineWidth = 10, float PointSize = 3, 
	//	bool ShowProfileCurve = false, bool DrawUpper = false);

////�ӿ�ʼλ��StartHeight�� �� EndHeight�� �� PlaneNumbers ������� 2017.05.27
//	void DrawCrossSectionalPlane(float StartHeight, float EndHeight, float PlaneNumbers = 20, 
//		float PointSize = 3,
//		bool ShowProfileCurve = false, bool DrawUpper = false);

	bool IsSuitForConstruct();

	//�ڸ���Heightֵ����ʾ�����
	void DrawCrossSectionalCurve(float Height, float PointSize = 3);

	//�ڸ���UValueֵ����ʾ�����
	void DrawCrossSectionalCurveAtU(float UValue, float PointSize = 3);

	void SaveInclinationCurvatureTorsionToFile(double StartLength, double EndLength, double Interval);
};

#endif