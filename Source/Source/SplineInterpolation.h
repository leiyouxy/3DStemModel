#ifndef SplineInterpolation_H
#define SplineInterpolation_H

//样条插值  根据样条上的点 求样条的控制点与节点向量 2015.09.26
#include "CommClass.h"
#include "Spline.h"

class CSplineInterpolation
{
private:
	//样条上的点
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPoints;
	//计算得到的控制点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints;
	
	//控制点形成的样条曲线是否是连续的
	bool IsClosed;

	//节点间的弦长值
	double ChordLength;

	//样条多项式次数
	int dSpline;

	//样条控制点个数，控制点个数与样条上的点的个数相同
	int nSpline;
	
	//是否采用向心法计算参数
	bool IsCentripetal;

	vector<double> ChordLengthS;
	////节点参数值
	//vector<double> UParameters;

	//计算节点间的弦长值
	double GetPointsChordLength();

	//计算节点的节点参数值 弦长参数法 《The Nurbs books》P257
	void GetUParametesByChordLength();

	//计算节点的节点参数值 向心参数法 《The Nurbs books》P257 2015.12.24
	void GetUParametesByCentripetalParameter();

	//计算节点的节点参数值 福利参数法 《计算机辅助几何设计与非均匀B样条》P45 2015.12.24
	void GetUParametesByFoleyParameter();

	//计算弦线夹角的外角	《计算机辅助几何设计与非均匀B样条》P48 2015.12.24	
	void CalcChordAngle(vector<double> & Angle);

	//计算节点向量
	void GetKnotValuesByAvg();

	//计算节点参数值
	void CalcKnotParameters();

	//获取控制点与节点向量
	bool ResolveControlPoints();

	//构建线性方程组并求解 将求解结果返回到 ControlPoints 点集中
	bool ResolveEquations();

	//均匀化样条的插值点
	void HomogenizationSplineInterPolatingPoints();

	//定位点在插值点的区间范围
	int FindPointInQPoints(pcl::PointXYZRGB  Point);

	//2015.12.24 非凸区域U值的起始点
	vector<double> NoConvexHullStartU;
	vector<double> NoConvexHullEndU;

	//2015.12.24 向上面两组容器中添加非凸区域的U的起始区域
	void InsertNoConvexHullU(double StartU, double EndU);

	//获取非凸区域的起始范围 2015.12.22  2015.12.22 只针对三次曲线
	//bool GetNoConvexHullZone(int i, vector<double> KnotVector,		
	//	double & NoConvexHullUStart, double & NoConvexHullUEnd);

	//缩小节点间的参数值 此种方法不适用，产生扭曲曲线
	void ShrinkKnotParameters(int i, double Multiple = 0.1);

	////根据 U 值 获取U
	//int FindUBetweenParameters(double U);
public:
	CSplineInterpolation()
	{
		ControlPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		QPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		//dSpline = 3; //三次插值
	}
	~CSplineInterpolation()
	{
		ControlPoints->points.clear();
		QPoints->points.clear();
	}
	//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPoints;

	//有理样条
	//CRationalSpline RationalSpline;
	CSpline RationalSpline;
	
	//计算得到的节点向量
	vector<double> KnotValues;	
	//节点参数值	为便于构建闭合曲面而调整到此处
	vector<double> UParameters;

	//设置初始参数  IsClosed 闭合  IsSuccessiveValue 连续
	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, 
		int dValue = 3, bool IsClosed = false, bool IsSuccessiveValue = true);

	//获取插值的结果
	bool GetControlPointsAndKnotValue(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & ControlPointsValue,
		vector<double> & OutKnotValue, bool CentripetalValue = false);

	//获取凸性闭合的三次样条曲线的控制点与节点向量 返回第一个节点*****此种方法 行不通*****
	//pcl::PointXYZRGB GetConvexHullAndClosedControlPoints(
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, 
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr & ControlPointsValue,
	//	vector<double> & OutKnotValue);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;
};

#endif


////2015.09.17 样条(2015.10.11修正)************************************** 	
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
//	////三角Sin函数测试
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
//	//三角Sin函数测试
//
//	//六个矩形点测试
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
//	//四个矩形点测试
//	
//	vector<double> KnoteVector;
////
//	PointBase::OpenPCLFile("ProcessedPoints\\MeshConvexhullPoints.pcd", StemPointsIndexPtr);
//	//PointBase::OpenPCLFile("ProcessedPoints\\OriginalConvexhullPoints.pcd", StemPointsIndexPtr);
//	PointBase::PointXYZRGBIndexToPointXYZRGB(StemPointsIndexPtr, StemPointsPtr);
//	////非有理B样条
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
//////	//样条插值
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
////	cout<<"样条长度为:"<<RationalSpline.CreateSpline()/M_PI/2<<endl;
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
////	cout<<"样条长度为:"<<RationalSpline.GetSplineLength()/M_PI/2<<endl;
////	//cout<<"样条长度为:"<<RationalSpline.GetSplineLength(0.000001)<<endl;
////	//cout<<"样条长度为:"<<RationalSpline.GetSplineLength(0.0000001)<<endl;
////2015.09.17 样条(2015.10.11修正)************************************** 	