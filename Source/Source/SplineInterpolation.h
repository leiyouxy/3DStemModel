#ifndef SplineInterpolation_H
#define SplineInterpolation_H

//������ֵ  ���������ϵĵ� �������Ŀ��Ƶ���ڵ����� 2015.09.26
#include "CommClass.h"
#include "Spline.h"

class CSplineInterpolation
{
private:
	//�����ϵĵ�
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPoints;
	//����õ��Ŀ��Ƶ�
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints;
	
	//���Ƶ��γɵ����������Ƿ���������
	bool IsClosed;

	//�ڵ����ҳ�ֵ
	double ChordLength;

	//��������ʽ����
	int dSpline;

	//�������Ƶ���������Ƶ�����������ϵĵ�ĸ�����ͬ
	int nSpline;
	
	//�Ƿ�������ķ��������
	bool IsCentripetal;

	vector<double> ChordLengthS;
	////�ڵ����ֵ
	//vector<double> UParameters;

	//����ڵ����ҳ�ֵ
	double GetPointsChordLength();

	//����ڵ�Ľڵ����ֵ �ҳ������� ��The Nurbs books��P257
	void GetUParametesByChordLength();

	//����ڵ�Ľڵ����ֵ ���Ĳ����� ��The Nurbs books��P257 2015.12.24
	void GetUParametesByCentripetalParameter();

	//����ڵ�Ľڵ����ֵ ���������� ��������������������Ǿ���B������P45 2015.12.24
	void GetUParametesByFoleyParameter();

	//�������߼нǵ����	��������������������Ǿ���B������P48 2015.12.24	
	void CalcChordAngle(vector<double> & Angle);

	//����ڵ�����
	void GetKnotValuesByAvg();

	//����ڵ����ֵ
	void CalcKnotParameters();

	//��ȡ���Ƶ���ڵ�����
	bool ResolveControlPoints();

	//�������Է����鲢��� ����������ص� ControlPoints �㼯��
	bool ResolveEquations();

	//���Ȼ������Ĳ�ֵ��
	void HomogenizationSplineInterPolatingPoints();

	//��λ���ڲ�ֵ������䷶Χ
	int FindPointInQPoints(pcl::PointXYZRGB  Point);

	//2015.12.24 ��͹����Uֵ����ʼ��
	vector<double> NoConvexHullStartU;
	vector<double> NoConvexHullEndU;

	//2015.12.24 ������������������ӷ�͹�����U����ʼ����
	void InsertNoConvexHullU(double StartU, double EndU);

	//��ȡ��͹�������ʼ��Χ 2015.12.22  2015.12.22 ֻ�����������
	//bool GetNoConvexHullZone(int i, vector<double> KnotVector,		
	//	double & NoConvexHullUStart, double & NoConvexHullUEnd);

	//��С�ڵ��Ĳ���ֵ ���ַ��������ã�����Ť������
	void ShrinkKnotParameters(int i, double Multiple = 0.1);

	////���� U ֵ ��ȡU
	//int FindUBetweenParameters(double U);
public:
	CSplineInterpolation()
	{
		ControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		QPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		//dSpline = 3; //���β�ֵ
	}
	~CSplineInterpolation()
	{
		ControlPoints->points.clear();
		QPoints->points.clear();
	}
	//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPoints;

	//��������
	//CRationalSpline RationalSpline;
	CSpline RationalSpline;
	
	//����õ��Ľڵ�����
	vector<double> KnotValues;	
	//�ڵ����ֵ	Ϊ���ڹ����պ�������������˴�
	vector<double> UParameters;

	//���ó�ʼ����  IsClosed �պ�  IsSuccessiveValue ����
	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, 
		int dValue = 3, bool IsClosed = false, bool IsSuccessiveValue = true);

	//��ȡ��ֵ�Ľ��
	bool GetControlPointsAndKnotValue(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & ControlPointsValue,
		vector<double> & OutKnotValue, bool CentripetalValue = false);

	//��ȡ͹�Ապϵ������������ߵĿ��Ƶ���ڵ����� ���ص�һ���ڵ�*****���ַ��� �в�ͨ*****
	//pcl::PointXYZRGB GetConvexHullAndClosedControlPoints(
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, 
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr & ControlPointsValue,
	//	vector<double> & OutKnotValue);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;
};

#endif


////2015.09.17 ����(2015.10.11����)************************************** 	
//	PointBase::OpenPCLFile("ProcessedPoints\\AStemGeometryCenterPoints.pcd", StemPointsIndexPtr);
//	StemPointsIndexPtr->points.erase(StemPointsIndexPtr->points.begin(), 
//				StemPointsIndexPtr->points.begin() + 30);
//
//	CenterX = -1 * StemPointsIndexPtr->points[0].x;
//	CenterY = -1 * StemPointsIndexPtr->points[0].y;
//	CenterZ = -1 * StemPointsIndexPtr->points[0].z;
//	PointBase::PointCoordinateTransform(StemPointsIndexPtr, CenterX + 1, CenterY + 1, CenterZ + 1);	
//
//	PointBase::PointXYZRGBIndexToPointXYZRGB(StemPointsIndexPtr, StemPointsPtr);
//	PointBase::SetPointColor(StemPointsPtr, ColorBase::WhiteColor);
//	//StemPointsPtr->points.push_back(StemPointsPtr->points[0]);
//	while (StemPointsPtr->points.size() > 100)
//	{
//		StemPointsPtr->points.pop_back();
//	}
//
//	////����Sin��������
//	//StemPointsPtr->points.clear();
//	//for(int i = 0; i <= 10; i++)
//	//{
//	//	pcl::PointXYZRGB Point;
//	//	Point.x = i;
//	//	Point.y = sin(i*1.0);
//	//	Point.z = 0;
//	//	Point.rgba = ColorBase::WhiteColor;
//	//	StemPointsPtr->points.push_back(Point);
//	//}
//	//����Sin��������
//
//	//�������ε����
//	StemPointsPtr->points.clear();
//	pcl::PointXYZRGB Point1;
//	Point1.x = -1;
//	Point1.y = -2;
//	Point1.z = 1;
//	Point1.rgba = ColorBase::WhiteColor;
//	StemPointsPtr->points.push_back(Point1);
//	
//	pcl::PointXYZRGB Point2;
//	Point2.x = -3;
//	Point2.y = 0;
//	Point2.z = 1;
//	Point2.rgba = ColorBase::WhiteColor;
//	StemPointsPtr->points.push_back(Point2);
//
//	pcl::PointXYZRGB Point3;
//	Point3.x = -1;
//	Point3.y = 2;
//	Point3.z = 1;
//	Point3.rgba = ColorBase::WhiteColor;
//	StemPointsPtr->points.push_back(Point3);
//
//	pcl::PointXYZRGB Point4;
//	Point4.x = 1;
//	Point4.y = 2;
//	Point4.z = 1;
//	Point4.rgba = ColorBase::WhiteColor;
//	StemPointsPtr->points.push_back(Point4);
//
//	pcl::PointXYZRGB Point5;
//	Point5.x = 3;
//	Point5.y = 0;
//	Point5.z = 1;
//	Point5.rgba = ColorBase::WhiteColor;
//	StemPointsPtr->points.push_back(Point5);
//
//	pcl::PointXYZRGB Point6;
//	Point6.x = 1;
//	Point6.y = -2;
//	Point6.z = 1;
//	Point6.rgba = ColorBase::WhiteColor;
//	StemPointsPtr->points.push_back(Point6);
//
//	//StemPointsPtr->points.push_back(Point4);
//	StemPointsPtr->points.push_back(Point1);
//	//StemPointsPtr->points.push_back(Point2);
//	//StemPointsPtr->points.push_back(Point3);
//
//	//�ĸ����ε����
//	
//	vector<double> KnoteVector;
////
//	PointBase::OpenPCLFile("ProcessedPoints\\MeshConvexhullPoints.pcd", StemPointsIndexPtr);
//	//PointBase::OpenPCLFile("ProcessedPoints\\OriginalConvexhullPoints.pcd", StemPointsIndexPtr);
//	PointBase::PointXYZRGBIndexToPointXYZRGB(StemPointsIndexPtr, StemPointsPtr);
//	////������B����
//	CSpline Spline;
//	Spline.SetSplineInputs(StemPointsPtr, 3, KnoteVector, PointsNumber);
//	
//	Spline.CreateUniformKnoteVector();
//	//Spline.CreateOpenAndUniformKnoteVector();
//	Spline.CreateSpline();
//	//
//	////Spline.GetCurvePointsByXValue(0.5);
//	////Spline.DrawSplineDerivative(Viewer, Number, Number + 100, 1, "Spline");
//	////Spline.DrawSplineDerivative(Viewer, Number, Number + 100, 2, "Spline", 0, 0, 255);
//	////Spline.DrawSplineDerivative(Viewer, Number, Number + 100, 3, "Spline",255, 0, 0);
//	////Spline.DrawSplineDerivative(Viewer, Number, Number + 100, 4, "Spline",255, 255, 0);
//	//
//	//PointBase::SetPointColor(StemPointsPtr, ColorBase::WhiteColor);
//	//PointBase::ShowPointXYZRGB(Viewer, StemPointsPtr, "StemPointsPtr", 2);
//	PointBase::SetPointColor(Spline.CurvePoints, ColorBase::RedColor);
//	PointBase::ShowPointXYZRGB(Viewer, Spline.CurvePoints, "CurvePoints", 1);
//	//PointBase::SetPointColor(Spline.KnotValuePoints, ColorBase::BlueColor);
//	//PointBase::ShowPointXYZRGB(Viewer, Spline.KnotValuePoints, "KnotValuePoints", 2);
//
//	//vector<double> U = Spline.GetCurvePointsByValue(0.5, "Y", 1.0e-3);
//
//	////////NURBS
//	PointBase::PointCoordinateTransform(StemPointsPtr, 0, 5, 0);
////	PointBase::SetPointColor(StemPointsPtr, ColorBase::RedColor);
////	for(int i = 0; i < StemPointsPtr->points.size(); i++)
////	{
////		Viewer->addText3D("Stem_" + CalcBaseInt.ConvertToString(i), 
////			StemPointsPtr->points[i],0.03);
////	}
////	//StemPointsPtr->points[0].rgba = ColorBase::BlueColor;
////	//StemPointsPtr->points[StemPointsPtr->points.size() -1].rgba = ColorBase::BlueColor;
////	//PointBase::ShowPointXYZRGB(Viewer, StemPointsPtr, "StemPointsPtr", 5);
//////
//////	//������ֵ
////	CSplineInterpolation SplineInterpolation;
////	SplineInterpolation.SetInputs(StemPointsPtr, 3, true);
////
////	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints;
////	ControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
////		
////	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnoteVector);
////
////	CRationalSpline RationalSpline;
////	RationalSpline.SetSplineInputs(ControlPoints, 3, KnoteVector, true, PointsNumber);
//////	//RationalSpline.CreateUniformKnoteVector();	
//////	//RationalSpline.CreateOpenAndUniformKnoteVector();
////	RationalSpline.CreatePointWeights();
////	cout<<"��������Ϊ:"<<RationalSpline.CreateSpline()/M_PI/2<<endl;
//////	//RationalSpline.DrawSplineDerivative(Viewer, Number, Number + 100, 1, "RationalSpline");
//////	//RationalSpline.DrawSplineDerivative(Viewer, Number, Number + 100, 2, "RationalSpline", 0, 0, 255);	
//////	//RationalSpline.DrawSplineDerivative(Viewer, Number, Number + 100, 3, "RationalSpline", 255, 0, 0);
//////	//RationalSpline.DrawSplineDerivative(Viewer, Number, Number + 100, 4, "RationalSpline", 255, 255, 0);
//////	//RationalSpline.DrawSplineDerivative(Viewer, Number, Number + 100, 5, "RationalSpline", 255, 0, 255);
//////	
////	PointBase::SetPointColor(RationalSpline.CurvePoints, ColorBase::GreenColor);
////	PointBase::ShowPointXYZRGB(Viewer, RationalSpline.CurvePoints, "RationalSplineCurvePoints", 1);	
//////	
////	PointBase::SetPointColor(ControlPoints, ColorBase::DoveColor);
////	PointBase::ShowPointXYZRGB(Viewer, ControlPoints, "ControlPoints", 2);	
////		
////	for(int i = 0; i < ControlPoints->points.size(); i++)
////	{
////		Viewer->addText3D("C_" + CalcBaseInt.ConvertToString(i), 
////			ControlPoints->points[i],0.03);
////	}
////		
////	PointBase::ShowPointXYZRGB(Viewer, StemPointsPtr, "StemPointsPtr", 5);
////
////	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points = 
////	//	RationalSpline.GetCurvePointsByValue(6.5, "Y", 1.0e-3);
////	//PointBase::SetPointColor(Points, ColorBase::DoveColor);
////	//PointBase::ShowPointXYZRGB(Viewer, Points, "Points", 3);
////	cout<<"��������Ϊ:"<<RationalSpline.GetSplineLength()/M_PI/2<<endl;
////	//cout<<"��������Ϊ:"<<RationalSpline.GetSplineLength(0.000001)<<endl;
////	//cout<<"��������Ϊ:"<<RationalSpline.GetSplineLength(0.0000001)<<endl;
////2015.09.17 ����(2015.10.11����)************************************** 	