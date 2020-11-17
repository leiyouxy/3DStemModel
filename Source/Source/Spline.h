//2015.09.17 根据控制点与节点向量获取样条曲线
//控制点的个数为n+1, 阶参数为d 多项式次数为d-1,节点向量个数为n+1+d

////2015.09.21 两个问题待(已09.28)解决，(1)根据已知曲线点，定位控制点与控制向量 样条插值问题
////2015.09.21 两个问题待(已09.28)解决，(2)根据特定坐标，返回特定的坐标值  确定节点向量范围，然后对U值二分查找得到


#ifndef Spline_H
#define Spline_H

#include "CommClass.h"
#include "AnglePartition.h"
#include "FormulaAnalytic.h"
#include "CommGeometry.h"

typedef vector<pcl::PointXYZRGB> PointAndDerivative;
typedef vector<PointAndDerivative> PointSAndDerivativeS;

//参数表达式的字符串
typedef vector<string> FormulaStr;

//参数表达式的 X，Y和Z轴的 字符串
typedef vector<FormulaStr> FormulaVec;

class CSpline
{
protected:
	//控制点点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints;
	
	FormulaVec FormulaVecValue;

	//精度控制 
	double eps;

	//传入的控制点及节点向量是否有重复样条曲线是否是连续的
	bool IsClosed;
	
	bool IsConvexhullControl;

	//样条使用的多项式次数 默认是3
	int dSpline;
	
	//控制点个数
	int nSpline;
		
	//节点向量
	vector<double> KnotVector;

	//节点向量对应位置处的长度 2018.09.25
	vector<double> SplineLengthOfKnotVector;

	//节点向量对应位置处的高度 2018.09.25
	vector<double> SplineHeightOfKnotVector;

	//存储每一个节点的U值 供后续查询使用
	vector<double> PointUValues;

	////自然三次样条，每段的首节点的索引
	//vector<int> NaturalTripleSplinePointSectionIndexS;

	////节点向量的最大值
	//double KnotVectorMaxValue;
	////节点向量的最小值
	//double KnotVectorMinValue;

	//标识是否能生成样条曲线 默认是true
	bool CanCreateCurve;

	//计算基函数Ni,p(u)的k阶导数  i 是节点向量索引，d是样条多项式的次数, Order是求导的次数
	double DerivativeOfCoxdeBoor(int i, int p, double u, int Order);
	
	//获取样条的Step
	//void GetKnoteVectorStep();

	//获取节点向量中对应的样条曲线上的点
	void GetKnotValuePoints();

	//在UMin值 与 UMin 值间 使用二分法定位 定位 Axis 轴中 Value 对应的 U值
	double FindUValueBetweenUParamters(double Value, std::string Axis, 
		double UMin, double UMax, double LocateEps = EPSM6);

	//获取一阶导的数据信息 根据节点间的关系 与 节点向量 间的关系
	void GetOneDerivativeData();

	//2015.12.24 非凸区域U值的起始点
	vector<double> NoConvexHullStartU;
	vector<double> NoConvexHullEndU;

	//2015.12.24 向上面两组容器中添加非凸区域的U的起始区域
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

		dSpline = 3;	//默认是3次样条
		//KnotVectorMinValue = FLT_MAX;	//最小值初始化为最大值
		//KnotVectorMaxValue = FLT_MIN;	//最大值初始化为最小值
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

	//所有的曲线
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ShowedCurvePoints;	

	//非凸点的点集
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr NoConvexhullPoints;	

	//节点向量节点值 对应的 曲线上的点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr KnotValuePoints;

	PointSAndDerivativeS CurvePointsAndDerivativeS;

	//一阶导对应的控制点与节点向量，求解方法参照《The Nurbs Book》P68、P69
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OneDerivativeControlPoints;
	vector<double> OneDerivativeKnotVector;

	//搜索节点向量值所在的索引
	int FindUIndex(double u);

	//计算混合函数 注意 d是样条使用的多项式的次数，而不是阶数，
	double CoxdeBoor(int i, int d, double u);

	//计算混合函数 注意 d是样条使用的多项式的次数，而不是阶数， 返回公式
	//  返回公式 2015.10.17
	string CoxdeBoorFormula(int i, int d, double u);

	//设置样条参数
	void SetSplineInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPointsPtr,
		int dValue, vector<double> KnotVectorValue,
		bool IsClosedCurve = false, bool IsConvexhullControlValue = false);

	//生成样条函数的点 返回样条的长度 节点向量间隔为0.000001
	void CreateSpline(int DerivativeOrder = 0);

	//获取少量的SplinePoint 2020.08.11
	void ZippedSplinePoints(int PointNum = 10000);

	//画样条曲线
	void DrawSpline(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer, string SplineName = "Spline");

	//画样条曲线的 Order 级导数 默认 Order = 1
	void DrawSplineDerivative(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		int StartIndex = 0, int EndIndex = 0,
		int Order = 1,	string SplineName = "SplineDerivative",
		int r = 0, int g = 255, int b = 0, 
		double LineLength = 1);

	//画样条曲线在 U 处的 Order 级导数 默认 Order = 1
	void DrawSplinePointDerivative(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		double U, int Order = 1,	string SplineName = "PointDerivative",
		int r = 0, int g = 255, int b = 0, 
		double LineLength = 1);

	//生成开放均匀样条的节点向量
	void CreateOpenAndUniformKnoteVector();

	//生成均匀样条的节点向量
	void CreateUniformKnoteVector();

	//设置节点向量
	void SetKnoteVector(vector<double> KnotVectorValue);

	//获取特定点的曲率 Relative = true时，是相对曲率 相对曲率的源处还没有找到
	virtual double GetCurvature(double U, bool Relative = false);

	//获取特定点的挠率
	virtual double GetTorsion(double U);

	//获取特定点位置处切线与水平面的夹角
	virtual double GetInclination(double U);

	//根据节点向量的值获取节点 2015.09.24
	virtual pcl::PointXYZRGB GetSplinePoint(double U);
	
	//根据u值返回 u 的字符串公式表达式 2015.10.17
	virtual vector<string> GetUFormula(double U);

	//根据节点向量的第 k 阶导数GetSplineDerivativePoint  2015.09.24
	virtual pcl::PointXYZRGB GetSplineDerivativePoint(double u, int Order, bool Normalized = false);

	//根据x(y或z)坐标获取 样条曲线上的节点U值  递归的方式实现，容易出现深度问题
	vector<double> GetCurvePointsUByValue(double Value, std::string Axis, 
		double LocateEps = 1.0e-3);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCurvePointsByValue(double Value, std::string Axis, 
		double LocateEps = 1.0e-3);

	//获取样条(绝对)长度位置处的U值
	virtual double GetUValueBySplineLength(double SplineLength);

	//获取样条(绝对)高度位置处的U值
	virtual double GetUValueBySplineHeight(double SplineHeight);

	//virtual vector<double> GetUValuesBySplineLengths(vector<double> SplineLengths);
	
	//构建自然三次样条
	//double CreateNaturalTripleSpline(bool IsChord =false, bool IsHomogenized = false);

	//均匀化样条的插值点
	void HomogenizationSplineInterPolatingPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints);

	////////如下三个函数算法有错误
	////对自然三次样条求积分获取长度
	//double GetNaturalTripleSplineLength(double bx, double cx, double dx,
	//	double by, double cy, double dy,
	//	double bz, double cz, double dz,
	//	double UStart, double UEnd);

	////获取一阶导数 x(u)= a + b*u + c*u^2 + d*u^3 的U值
	//double GetUIDerivative(double b, double c, double d, double u);
	//
	////获取二阶导数 x(u)= a + b*u + c*u^2 + d*u^3 的U值
	//double GetUIIDerivative(double c, double d, double u);
	////////如下三个函数算法有错误

	//计算点构成曲线的凸性
	void ComputeConvexPropertyOfCurve(string Type = "BSpline");
	
	//计算样条曲线的切线
	pcl::PointXYZRGB ResolveTangentLine(pcl::PointXYZRGB CurPoint, int PointIndex, string Type);

	//判断此点是否是凸曲线上的点
	bool IsConvexHullPoint(int AngleIndex, pcl::PointXYZRGB TangentLinePoint,
		vector<AnglePartitionStruct> & SectionAngle);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;

	//int ClosedCurveStartKnoteIndex;
	//int ClosedCurveEndKnoteIndex;

	//根据Simpson求积公式计算曲线长度 KValue 是分的段数
	double SplineLength;
	double GetSplineLengthBySimpson();

	//2019.01.23 only used for 3D curve
	double GetSplineHeight();

	//根据Simpson求积公式计算曲线 在 参数 u 位置处的长度 KValue 是分的段数, 
	double GetSplineLengthBySimpson(double u);

	//获取闭合样条曲线所围区域的面积
	double GetSplineArea();

	//计算非凸区域的位置 2015.12.22  2015.12.22 只针对三次曲线
	void JudgeNoConvexhullKnoteValue(int i, string XFormula, string YFormula, double UStart, double UEnd);

	//获取非凸区域的起始范围 2015.12.22  2015.12.22 只针对三次曲线
	bool GetNoConvexHullZone(int i, double & NoConvexHullUStart, double & NoConvexHullUEnd);

	////根据 一阶导数的控制点与节点向量 计算u处的一阶导数 
	pcl::PointXYZRGB GetDerivativePointByPoints(double u);

	float StartU;
	float EndU;
	//样条曲线中包含点的总个数
	//int CurvePointsNumber; //默认1000个点
	double UStep; //点的步长，(UMax-Umin)/CurvePointsNumber 

	//保存控制点 与 节点向量
	void SaveControlPointsAndKnotValue(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPointsValue,
		vector<double> KnotValue,
		string FileNameControlPoints, string FileNameKnotValue,
		float StartKnot = 0, float EndKnot = 0);
};

/////****2015.09.21 有理B样条类
// PointWeights 需要使用这个时才有意义
class CRationalSpline : public CSpline	
{
private:
	//控制点权重
	vector<double> PointWeights;

	//计算方法见《非均匀有理B样条》 第91页 公式 4.8 中的 A 的 k 阶导数
	pcl::PointXYZRGB DerivativeOfRationalCoxdeBoor_A(int d, double u, int k);

	//计算方法见《非均匀有理B样条》 第91页 公式 4.8 中的 W 的 k 阶导数
	double DerivativeOfRationalCoxdeBoor_W(int d, double u, int k);

	//计算 公式 4.8 的分母
	double RationalCoxdeBoor_W(int d, double u);	

public:
	//生成控制点权重  现阶段权重都设置为1
	void CreatePointWeights();

	//设置控制点权重
	//void SetPointWeights(vector<double> PointWeightsValue);

	////生成有理B样条的点
	//void CreateRationalSpline(int DerivativeOrder = 0);

	//根据节点向量的值获取节点 2015.09.24
	virtual pcl::PointXYZRGB GetSplinePoint(double u);

	//根据u值返回 u 的字符串公式表达式 2015.10.17
	virtual vector<string> GetUFormula(double U);

	//计算有理B样条的基函数Ni,p(u)的k阶导数矢量   i 是节点向量索引，d是样条多项式的次数, k是求导的次数
	//计算方法见《非均匀有理B样条》 第91页 公式 4.8 
	virtual pcl::PointXYZRGB GetSplineDerivativePoint(double u, int k, bool Normalized = false);
};

#endif



////样条最新 2015.10.13×××××××××××××××××××××
//	vector<double> KnoteVector;
//	PointBase::OpenPCLFile("ProcessedPoints\\MeshConvexhullPoints.pcd", StemPointsIndexPtr);
//	
//	//PointBase::OpenPCLFile("ProcessedPoints\\OriginalConvexhullPoints.pcd", StemPointsIndexPtr);
//	PointBase::PointXYZRGBIndexToPointXYZRGB(StemPointsIndexPtr, StemPointsPtr);
//	PointBase::SimplifyConvexHull(StemPointsPtr, 0.5);
//////	//样条插值
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
//	cout<<"样条长度为:"<<RationalSpline.CreateSpline()/M_PI<<endl;
//	PointBase::ShowPointXYZRGB(Viewer, RationalSpline.CurvePoints, "RationalSplineCurvePoints", 1);	
//
//	PointBase::SetPointColor(StemPointsPtr, ColorBase::GreenColor);
//	PointBase::ShowPointXYZRGB(Viewer, StemPointsPtr, "StemPointsPtr", 5);
//	for(int i = 0; i < StemPointsPtr->points.size(); i++)
//	{
//		Viewer->addText3D(CalcBaseInt.ConvertToString(i), 	
//			StemPointsPtr->points[i],0.1);
//	}
////样条最新 2015.10.13×××××××××××××××××××××