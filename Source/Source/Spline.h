//2015.09.17 ���ݿ��Ƶ���ڵ�������ȡ��������
//���Ƶ�ĸ���Ϊn+1, �ײ���Ϊd ����ʽ����Ϊd-1,�ڵ���������Ϊn+1+d

////2015.09.21 ���������(��09.28)�����(1)������֪���ߵ㣬��λ���Ƶ���������� ������ֵ����
////2015.09.21 ���������(��09.28)�����(2)�����ض����꣬�����ض�������ֵ  ȷ���ڵ�������Χ��Ȼ���Uֵ���ֲ��ҵõ�


#ifndef Spline_H
#define Spline_H

#include "CommClass.h"
#include "AnglePartition.h"
#include "FormulaAnalytic.h"
#include "CommGeometry.h"

typedef vector<pcl::PointXYZRGB> PointAndDerivative;
typedef vector<PointAndDerivative> PointSAndDerivativeS;

//�������ʽ���ַ���
typedef vector<string> FormulaStr;

//�������ʽ�� X��Y��Z��� �ַ���
typedef vector<FormulaStr> FormulaVec;

class CSpline
{
protected:
	//���Ƶ����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints;
	
	FormulaVec FormulaVecValue;

	//���ȿ��� 
	double eps;

	//����Ŀ��Ƶ㼰�ڵ������Ƿ����ظ����������Ƿ���������
	bool IsClosed;
	
	bool IsConvexhullControl;

	//����ʹ�õĶ���ʽ���� Ĭ����3
	int dSpline;
	
	//���Ƶ����
	int nSpline;
		
	//�ڵ�����
	vector<double> KnotVector;

	//�ڵ�������Ӧλ�ô��ĳ��� 2018.09.25
	vector<double> SplineLengthOfKnotVector;

	//�ڵ�������Ӧλ�ô��ĸ߶� 2018.09.25
	vector<double> SplineHeightOfKnotVector;

	//�洢ÿһ���ڵ��Uֵ ��������ѯʹ��
	vector<double> PointUValues;

	////��Ȼ����������ÿ�ε��׽ڵ������
	//vector<int> NaturalTripleSplinePointSectionIndexS;

	////�ڵ����������ֵ
	//double KnotVectorMaxValue;
	////�ڵ���������Сֵ
	//double KnotVectorMinValue;

	//��ʶ�Ƿ��������������� Ĭ����true
	bool CanCreateCurve;

	//���������Ni,p(u)��k�׵���  i �ǽڵ�����������d����������ʽ�Ĵ���, Order���󵼵Ĵ���
	double DerivativeOfCoxdeBoor(int i, int p, double u, int Order);
	
	//��ȡ������Step
	//void GetKnoteVectorStep();

	//��ȡ�ڵ������ж�Ӧ�����������ϵĵ�
	void GetKnotValuePoints();

	//��UMinֵ �� UMin ֵ�� ʹ�ö��ַ���λ ��λ Axis ���� Value ��Ӧ�� Uֵ
	double FindUValueBetweenUParamters(double Value, std::string Axis, 
		double UMin, double UMax, double LocateEps = EPSM6);

	//��ȡһ�׵���������Ϣ ���ݽڵ��Ĺ�ϵ �� �ڵ����� ��Ĺ�ϵ
	void GetOneDerivativeData();

	//2015.12.24 ��͹����Uֵ����ʼ��
	vector<double> NoConvexHullStartU;
	vector<double> NoConvexHullEndU;

	//2015.12.24 ������������������ӷ�͹�����U����ʼ����
	void InsertNoConvexHullU(double StartU, double EndU);
public:
	CSpline()
	{
		ControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		CurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		ShowCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		ShowedCurvePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		KnotValuePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		NoConvexhullPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		OneDerivativeControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

		dSpline = 3;	//Ĭ����3������
		//KnotVectorMinValue = FLT_MAX;	//��Сֵ��ʼ��Ϊ���ֵ
		//KnotVectorMaxValue = FLT_MIN;	//���ֵ��ʼ��Ϊ��Сֵ
		CanCreateCurve = true;
		eps = 1.0e-8;
		UStep = 1.0e-6;
		FirstPointOfCloseCurve.z = -100000;
	}
	~CSpline()
	{
		ControlPoints->points.clear();
		CurvePoints->points.clear();
		ShowedCurvePoints->points.clear();
		KnotValuePoints->points.clear();
		NoConvexhullPoints->points.clear();
		OneDerivativeControlPoints->points.clear();	
	}
	pcl::PointXYZRGB FirstPointOfCloseCurve;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePoints;	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ShowCurvePoints;

	//���е�����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ShowedCurvePoints;	

	//��͹��ĵ㼯
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr NoConvexhullPoints;	

	//�ڵ������ڵ�ֵ ��Ӧ�� �����ϵĵ�
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr KnotValuePoints;

	PointSAndDerivativeS CurvePointsAndDerivativeS;

	//һ�׵���Ӧ�Ŀ��Ƶ���ڵ���������ⷽ�����ա�The Nurbs Book��P68��P69
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OneDerivativeControlPoints;
	vector<double> OneDerivativeKnotVector;

	//�����ڵ�����ֵ���ڵ�����
	int FindUIndex(double u);

	//�����Ϻ��� ע�� d������ʹ�õĶ���ʽ�Ĵ����������ǽ�����
	double CoxdeBoor(int i, int d, double u);

	//�����Ϻ��� ע�� d������ʹ�õĶ���ʽ�Ĵ����������ǽ����� ���ع�ʽ
	//  ���ع�ʽ 2015.10.17
	string CoxdeBoorFormula(int i, int d, double u);

	//������������
	void SetSplineInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPointsPtr,
		int dValue, vector<double> KnotVectorValue,
		bool IsClosedCurve = false, bool IsConvexhullControlValue = false);

	//�������������ĵ� ���������ĳ��� �ڵ��������Ϊ0.000001
	void CreateSpline(int DerivativeOrder = 0);

	//��ȡ������SplinePoint 2020.08.11
	void ZippedSplinePoints(int PointNum = 10000);

	//����������
	void DrawSpline(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer, string SplineName = "Spline");

	//���������ߵ� Order ������ Ĭ�� Order = 1
	void DrawSplineDerivative(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		int StartIndex = 0, int EndIndex = 0,
		int Order = 1,	string SplineName = "SplineDerivative",
		int r = 0, int g = 255, int b = 0, 
		double LineLength = 1);

	//������������ U ���� Order ������ Ĭ�� Order = 1
	void DrawSplinePointDerivative(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		double U, int Order = 1,	string SplineName = "PointDerivative",
		int r = 0, int g = 255, int b = 0, 
		double LineLength = 1);

	//���ɿ��ž��������Ľڵ�����
	void CreateOpenAndUniformKnoteVector();

	//���ɾ��������Ľڵ�����
	void CreateUniformKnoteVector();

	//���ýڵ�����
	void SetKnoteVector(vector<double> KnotVectorValue);

	//��ȡ�ض�������� Relative = trueʱ����������� ������ʵ�Դ����û���ҵ�
	virtual double GetCurvature(double U, bool Relative = false);

	//��ȡ�ض��������
	virtual double GetTorsion(double U);

	//��ȡ�ض���λ�ô�������ˮƽ��ļн�
	virtual double GetInclination(double U);

	//���ݽڵ�������ֵ��ȡ�ڵ� 2015.09.24
	virtual pcl::PointXYZRGB GetSplinePoint(double U);
	
	//����uֵ���� u ���ַ�����ʽ���ʽ 2015.10.17
	virtual vector<string> GetUFormula(double U);

	//���ݽڵ������ĵ� k �׵���GetSplineDerivativePoint  2015.09.24
	virtual pcl::PointXYZRGB GetSplineDerivativePoint(double u, int Order, bool Normalized = false);

	//����x(y��z)�����ȡ ���������ϵĽڵ�Uֵ  �ݹ�ķ�ʽʵ�֣����׳����������
	vector<double> GetCurvePointsUByValue(double Value, std::string Axis, 
		double LocateEps = 1.0e-3);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCurvePointsByValue(double Value, std::string Axis, 
		double LocateEps = 1.0e-3);

	//��ȡ����(����)����λ�ô���Uֵ
	virtual double GetUValueBySplineLength(double SplineLength);

	//��ȡ����(����)�߶�λ�ô���Uֵ
	virtual double GetUValueBySplineHeight(double SplineHeight);

	//virtual vector<double> GetUValuesBySplineLengths(vector<double> SplineLengths);
	
	//������Ȼ��������
	//double CreateNaturalTripleSpline(bool IsChord =false, bool IsHomogenized = false);

	//���Ȼ������Ĳ�ֵ��
	void HomogenizationSplineInterPolatingPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints);

	////////�������������㷨�д���
	////����Ȼ������������ֻ�ȡ����
	//double GetNaturalTripleSplineLength(double bx, double cx, double dx,
	//	double by, double cy, double dy,
	//	double bz, double cz, double dz,
	//	double UStart, double UEnd);

	////��ȡһ�׵��� x(u)= a + b*u + c*u^2 + d*u^3 ��Uֵ
	//double GetUIDerivative(double b, double c, double d, double u);
	//
	////��ȡ���׵��� x(u)= a + b*u + c*u^2 + d*u^3 ��Uֵ
	//double GetUIIDerivative(double c, double d, double u);
	////////�������������㷨�д���

	//����㹹�����ߵ�͹��
	void ComputeConvexPropertyOfCurve(string Type = "BSpline");
	
	//�����������ߵ�����
	pcl::PointXYZRGB ResolveTangentLine(pcl::PointXYZRGB CurPoint, int PointIndex, string Type);

	//�жϴ˵��Ƿ���͹�����ϵĵ�
	bool IsConvexHullPoint(int AngleIndex, pcl::PointXYZRGB TangentLinePoint,
		vector<AnglePartitionStruct> & SectionAngle);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;

	//int ClosedCurveStartKnoteIndex;
	//int ClosedCurveEndKnoteIndex;

	//����Simpson�����ʽ�������߳��� KValue �ǷֵĶ���
	double SplineLength;
	double GetSplineLengthBySimpson();

	//2019.01.23 only used for 3D curve
	double GetSplineHeight();

	//����Simpson�����ʽ�������� �� ���� u λ�ô��ĳ��� KValue �ǷֵĶ���, 
	double GetSplineLengthBySimpson(double u);

	//��ȡ�պ�����������Χ��������
	double GetSplineArea();

	//�����͹�����λ�� 2015.12.22  2015.12.22 ֻ�����������
	void JudgeNoConvexhullKnoteValue(int i, string XFormula, string YFormula, double UStart, double UEnd);

	//��ȡ��͹�������ʼ��Χ 2015.12.22  2015.12.22 ֻ�����������
	bool GetNoConvexHullZone(int i, double & NoConvexHullUStart, double & NoConvexHullUEnd);

	////���� һ�׵����Ŀ��Ƶ���ڵ����� ����u����һ�׵��� 
	pcl::PointXYZRGB GetDerivativePointByPoints(double u);

	float StartU;
	float EndU;
	//���������а�������ܸ���
	//int CurvePointsNumber; //Ĭ��1000����
	double UStep; //��Ĳ�����(UMax-Umin)/CurvePointsNumber 

	//������Ƶ� �� �ڵ�����
	void SaveControlPointsAndKnotValue(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPointsValue,
		vector<double> KnotValue,
		string FileNameControlPoints, string FileNameKnotValue,
		float StartKnot = 0, float EndKnot = 0);
};

/////****2015.09.21 ����B������
// PointWeights ��Ҫʹ�����ʱ��������
class CRationalSpline : public CSpline	
{
private:
	//���Ƶ�Ȩ��
	vector<double> PointWeights;

	//���㷽�������Ǿ�������B������ ��91ҳ ��ʽ 4.8 �е� A �� k �׵���
	pcl::PointXYZRGB DerivativeOfRationalCoxdeBoor_A(int d, double u, int k);

	//���㷽�������Ǿ�������B������ ��91ҳ ��ʽ 4.8 �е� W �� k �׵���
	double DerivativeOfRationalCoxdeBoor_W(int d, double u, int k);

	//���� ��ʽ 4.8 �ķ�ĸ
	double RationalCoxdeBoor_W(int d, double u);	

public:
	//���ɿ��Ƶ�Ȩ��  �ֽ׶�Ȩ�ض�����Ϊ1
	void CreatePointWeights();

	//���ÿ��Ƶ�Ȩ��
	//void SetPointWeights(vector<double> PointWeightsValue);

	////��������B�����ĵ�
	//void CreateRationalSpline(int DerivativeOrder = 0);

	//���ݽڵ�������ֵ��ȡ�ڵ� 2015.09.24
	virtual pcl::PointXYZRGB GetSplinePoint(double u);

	//����uֵ���� u ���ַ�����ʽ���ʽ 2015.10.17
	virtual vector<string> GetUFormula(double U);

	//��������B�����Ļ�����Ni,p(u)��k�׵���ʸ��   i �ǽڵ�����������d����������ʽ�Ĵ���, k���󵼵Ĵ���
	//���㷽�������Ǿ�������B������ ��91ҳ ��ʽ 4.8 
	virtual pcl::PointXYZRGB GetSplineDerivativePoint(double u, int k, bool Normalized = false);
};

#endif



////�������� 2015.10.13������������������������������������������
//	vector<double> KnoteVector;
//	PointBase::OpenPCLFile("ProcessedPoints\\MeshConvexhullPoints.pcd", StemPointsIndexPtr);
//	
//	//PointBase::OpenPCLFile("ProcessedPoints\\OriginalConvexhullPoints.pcd", StemPointsIndexPtr);
//	PointBase::PointXYZRGBIndexToPointXYZRGB(StemPointsIndexPtr, StemPointsPtr);
//	PointBase::SimplifyConvexHull(StemPointsPtr, 0.5);
//////	//������ֵ
//	CSplineInterpolation SplineInterpolation;
//	SplineInterpolation.SetInputs(StemPointsPtr, 5, true);
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints;
//	ControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
//		
//	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnoteVector);
//
//	CRationalSpline RationalSpline;
//	RationalSpline.SetSplineInputs(ControlPoints, 5, KnoteVector);
////	//RationalSpline.CreateUniformKnoteVector();	
////	//RationalSpline.CreateOpenAndUniformKnoteVector();
//	RationalSpline.CreatePointWeights();
//	cout<<"��������Ϊ:"<<RationalSpline.CreateSpline()/M_PI<<endl;
//	PointBase::ShowPointXYZRGB(Viewer, RationalSpline.CurvePoints, "RationalSplineCurvePoints", 1);	
//
//	PointBase::SetPointColor(StemPointsPtr, ColorBase::GreenColor);
//	PointBase::ShowPointXYZRGB(Viewer, StemPointsPtr, "StemPointsPtr", 5);
//	for(int i = 0; i < StemPointsPtr->points.size(); i++)
//	{
//		Viewer->addText3D(CalcBaseInt.ConvertToString(i), 	
//			StemPointsPtr->points[i],0.1);
//	}
////�������� 2015.10.13������������������������������������������