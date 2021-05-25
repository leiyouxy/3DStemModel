#ifndef CommGeometry_H
#define CommGeometry_H

//#include "stdafx.h"
#include <sstream>
#include <strstream>
#include <string>
#include <iomanip>
#include <limits>
#include <math.h>
#include <iostream>
#include <vector>
#include <stack>
#include <map>
#include <random>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/search/search.h>

#include "Commdefinitions.h"
#include "pcl/console/parse.h"
#include "Eigen/src/Eigenvalues/EigenSolver.h"
#include "pcl/filters/project_inliers.h"

#include "CommClass.h"
#include "ContourAndConvexHull.h"
#include "CommPointBase.h"

using namespace std;

typedef struct
{
	double a;
	double b;
	double c;
	double d;
	double PlaneValue;
} PlaneCoeff;

static class GeometryBase
{
public:

	//返回 Radius 近邻， NeighbourDis是升序排序的距离
static void GetNeighbourInRadius(
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree,
		double Radius, int PointIndex, vector<int> * NeighbourIndexs, vector<float> * NeighbourDis, 
	bool IncludeSelf = false) //IncludeSelf = false 表示不包括自己
{
	NeighbourIndexs->clear();	
	if (NeighbourDis != NULL)
		NeighbourDis->clear();

	Octree->radiusSearch(PointIndex, Radius, *NeighbourIndexs, *NeighbourDis);	

	// Find the current index i is in itself's NeigbourIndex, if in, delete it.
	VectorBase<int> VectorBaseInt;
	int TempIndex = VectorBaseInt.FindIndexInVector(*NeighbourIndexs, PointIndex);
	
	if (TempIndex != -1 && !IncludeSelf)	//不包括自己
	{
		//delete itself's index
		(*NeighbourIndexs).erase((*NeighbourIndexs).begin() + TempIndex);
		//delete itself's dis value
		if (NeighbourDis != NULL)
			(*NeighbourDis).erase((*NeighbourDis).begin() + TempIndex);
		return;
	}

	if (TempIndex == -1 && IncludeSelf)	//包括自己
	{
		NeighbourIndexs->push_back(PointIndex);
		NeighbourDis->push_back(0);
	}
}

//////返回查询点QueryPoint的K近邻，QueryPoint并不在给定的点集中
//static void GetNeighbourInRadius(
//	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree,
//	double Radius, pcl::PointXYZRGB QueryPoint, vector<int> * NeighbourIndexs, vector<float> * NeighbourDis,
//	bool IncludeSelf = false) //IncludeSelf = false 表示不包括自己
//{
//	NeighbourIndexs->clear();
//	if (NeighbourDis != NULL)
//		NeighbourDis->clear();
//
//	Octree->radiusSearch(QueryPoint, Radius, *NeighbourIndexs, *NeighbourDis);	
//}

//返回K近邻， NeighbourDis是升序排序的距离
static void GetNeighbourInKNN(
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree,
	int K, int PointIndex, vector<int> * NeighbourIndexs, vector<float> * NeighbourDis,
	bool IncludeSelf = false) //IncludeSelf = false 表示不包括自己
{
	NeighbourIndexs->clear();
	Octree->nearestKSearch(PointIndex, K, *NeighbourIndexs, *NeighbourDis);

	// Find the current index i is in itself's NeigbourIndex, if in, delete it.
	VectorBase<int> VectorBaseInt;
	int TempIndex = VectorBaseInt.FindIndexInVector(*NeighbourIndexs, PointIndex);
	
	if (TempIndex != -1 && !IncludeSelf)	//不包括自己
	{
		//delete itself's index
		(*NeighbourIndexs).erase((*NeighbourIndexs).begin() + TempIndex);
		//delete itself's dis value
		if (NeighbourDis != NULL)
			(*NeighbourDis).erase((*NeighbourDis).begin() + TempIndex);
	}

	if (TempIndex == -1 && IncludeSelf)	//包括自己
	{
		NeighbourIndexs->push_back(PointIndex);
		NeighbourDis->push_back(0);
	}
}

////返回K近邻， NeighbourDis是升序排序的距离
//static void GetNeighbourInKNN(
//	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree,
//	int K, pcl::PointXYZRGB QueryPoint, vector<int> * NeighbourIndexs, vector<float> * NeighbourDis,
//	bool IncludeSelf = false) //IncludeSelf = false 表示不包括自己
//{
//	NeighbourIndexs->clear();
//	Octree->nearestKSearch(QueryPoint, K, *NeighbourIndexs, *NeighbourDis);	
//}

//根据中心点的CenterX与CenterY 与 分区点的PointX与PointY 计算角度值 返回角度值
static double AngleOfTwoPointsInXY(double CenterX, double CenterY, double PointX, double PointY)
{
	double Angle;
	if ((PointY == CenterY) && (PointX > CenterX))	
	{	//在X轴右侧
		Angle = 0;
	}
	else if ((PointX == CenterX) && (PointY > CenterY))	
	{	//在Y轴上侧
		Angle = M_PI / 2;
	}
	else if ((PointY == CenterY) && (PointX < CenterX))	
	{	//在X轴左侧
		Angle = M_PI;
	}
	else if ((PointX == CenterX) && (PointY < CenterY))	
	{	//在Y轴下侧
		Angle = M_PI * 3 / 2;	
	}
	else if ((PointX > CenterX) && (PointY > CenterY))
	{	//第一象限
		// asin 返回的是弧度值	
		Angle = atan((PointY - CenterY)/(PointX - CenterX));
	}
	else if ((PointX < CenterX) && (PointY > CenterY))
	{	//第二象限
		Angle = atan((PointY - CenterY)/(PointX - CenterX)) + M_PI;
	}
	else if ((PointX < CenterX) && (PointY < CenterY))
	{	//第三象限
		Angle = atan((PointY - CenterY)/(PointX - CenterX)) + M_PI;
	}
	else if ((PointX > CenterX) && (PointY < CenterY))
	{	//第四象限
		Angle = atan((PointY - CenterY)/(PointX - CenterX)) + 2 * M_PI;
	}
	else
	{
		return 0;
	}
	return 180 * Angle / M_PI;
}

//将三个三维点转到二维平面，X1,Y1,X2,Y2,X3,Y3,即是传入值又是传出值
static bool ThreeDimensionToTwoDimension(
			double & X1, double & Y1, double Z1,
			double & X2, double & Y2, double Z2,
			double & X3, double & Y3, double Z3)
{
	double TempArea = GeometryBase::AreaOfThreePointsIn3D(X1, Y1, Z1,
				X2, Y2, Z2,
				X3, Y3, Z3);
	if (TempArea == 0)
	{
		cout<<"Three points collinear!"<<endl;
		return false;
	}

	double u1;	double v1;	double w1;
	double u2;  double v2;  double w2;

	u1 = X2 - X1;
	v1=  Y2 - Y1;
	w1=  Z2 - Z1;
	double OneTwoLength = sqrt(pow(u1,2) + pow(v1,2) + pow(w1,2));
	u1 = u1/OneTwoLength;
	v1 = v1/OneTwoLength;
	w1 = w1/OneTwoLength;

	double OnThreeProjectionLength = (u1 * (X3 - X1) 
		+ v1 * (Y3 - Y1) + w1 * (Z3 - Z1));

	u2 = (X3 - X1) - u1 * OnThreeProjectionLength;
	v2 = (Y3 - Y1) - v1 * OnThreeProjectionLength;
	w2 = (Z3 - Z1) - w1 * OnThreeProjectionLength;

	double Length = sqrt(pow(u2,2) + pow(v2,2) + pow(w2,2));
	u2 = u2/Length;
	v2 = v2/Length;
	w2 = w2/Length;	

	X1 = X1 * u1 + Y1 * v1 + Z1 * w1;
	Y1 = X1 * u2 + Y1 * v2 + Z1 * w2;
	
	X2 = X2 * u1 + Y2 * v1 + Z2 * w1;
	Y2 = X2 * u2 + Y2 * v2 + Z2 * w2;
	
	X3 = X3 * u1 + Y3 * v1 + Z3 * w1;
	Y3 = X3 * u2 + Y3 * v2 + Z3 * w2;	
}

static void GetPointsTangentPlane(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr,
	pcl::PointXYZRGB & NormalPoint)
{
	double a, b, c;
	GetPointsTangentPlane(PointXYZRGBPtr, a,b,c);
	NormalPoint.x = a; NormalPoint.y = b; NormalPoint.z = c;
}

static pcl::PointXYZRGB GetPointsTangentPlane(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr,
	double & a, double & b, double & c)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr PointPtr (new pcl::PointCloud<pcl::PointXYZ>);
	PointBase::PointXYZRGBToXYZ(PointXYZRGBPtr, PointPtr);	
	return GetPointsTangentPlane(PointPtr,a,b,c); 
}

static pcl::PointXYZRGB GetPointsTangentPlane(
	pcl::PointCloud<pcl::PointXYZ>::Ptr PointPtr,
	double & a, double & b, double & c)
{
	Eigen::Matrix3f EigenVector;		//特征向量
	Eigen::Vector3f EigenValue(3);

	pcl::PointXYZRGB TempCenterPoint;
	TempCenterPoint = GetPointsTangentPlane(PointPtr, EigenVector, EigenValue);
	a = EigenVector(0);
	b = EigenVector(1);
	c = EigenVector(2);
	return TempCenterPoint;
}

static int Points3DTo2D(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Point3D,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Point2D)
{
	if (Point3D->points.size() < 3) return 0;
	pcl::PointXYZRGB NormalPoint = GetNormalOfTriangle(Point3D->points[0],
		Point3D->points[1], Point3D->points[2]);
	if (NormalPoint.x == 0 && NormalPoint.y ==0 && NormalPoint.z ==0)
		return 0;

	GeometryBase::RotateNormalToVertical(Point3D, Point2D, NormalPoint);
		return 1;
}

static double AreaOfThreePoints(double X1, double Y1, double Z1,
		double X2, double Y2, double Z2,
		double X3, double Y3, double Z3)
{

	if (ThreeDimensionToTwoDimension(X1, Y1, Z1, X2, Y2, Z2, X3, Y3, Z3))
	{
		return AreaOfThreePointsIn2D(X1, Y1, X2, Y2, X3, Y3);
	}
	else
		return 0;
}

static long double AreaOfThreePointsIn2D(double Point1x, double Point1y,
	double Point2x, double Point2y, double Point3x, double Point3y)
{
//返回 三角形面积，如果 > 0 是逆时针排列，
 //  Dim Area As Double //二维平面
 //Area = p1(1) * P2(2) + P2(1) * P3(2) + P3(1) * p1(2)
 //Area = Area - p1(1) * P3(2) - P2(1) * p1(2) - P3(1) * P2(2)  
////2015.07.22 此处的计算精度与matlab有差异 matlab 结果是极低的负值，而此处还是正值。

  	long double Area;	
	Area = Point1x * Point2y + Point2x * Point3y + Point3x * Point1y 
		- Point1x * Point3y - Point2x * Point1y - Point3x * Point2y;	
	return Area / 2;
}

static long double AreaOfThreePointsIn2D(pcl::PointXYZRGB A, pcl::PointXYZRGB B, 
	pcl::PointXYZRGB C)
{
	return AreaOfThreePointsIn2D(A.x, A.y, B.x, B.y, C.x, C.y);
}

static double AreaOfThreePointsIn3D(double Point1x, double Point1y, double Point1z,
	double Point2x, double Point2y, double Point2z,
	double Point3x, double Point3y, double Point3z)
{
	long double a1 = Point2x - Point1x;
	long double a2 = Point2y - Point1y;
	long double a3 = Point2z - Point1z;

	long double b1 = Point3x - Point1x;
	long double b2 = Point3y - Point1y;
	long double b3 = Point3z - Point1z;
		
	return sqrt(pow(a2*b3-a3*b2,2) + pow(a1*b3- a3*b1,2) + pow(a1*b2-b1*a2,2)) / 2;	
}

static pcl::PointXYZRGB GetNormalOfTriangle(pcl::PointXYZRGB A, pcl::PointXYZRGB B, pcl::PointXYZRGB C)
{
	pcl::PointXYZRGB AB, AC;
	AB.x = A.x - B.x, AB.y = A.y - B.y, AB.z = A.z - B.z;
	AC.x = A.x - C.x, AC.y = A.y - C.y, AC.z = A.z - C.z;

	return PointBase::PointNormalized(PointBase::PointsCrossProduct(AB, AC));
}

//面积公式有错误，待测试(2018.12.31)
static double AreaOfThreePointsIn3D(pcl::PointXYZRGB A, pcl::PointXYZRGB B, pcl::PointXYZRGB C)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Original (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Out (new pcl::PointCloud<pcl::PointXYZRGB>);

	Original->points.push_back(A);
	Original->points.push_back(B);
	Original->points.push_back(C);

	pcl::PointXYZRGB NormalPoint = GetNormalOfTriangle(A, B, C);
	GeometryBase::RotateNormalToVertical(Original, Out, NormalPoint);

	return GeometryBase::AreaOfThreePointsIn2D(
		Out->points[0].x, Out->points[0].y,
		Out->points[1].x, Out->points[1].y,
		Out->points[2].x, Out->points[2].y);
}

//获取方向向量, Order = 0, 最小， Order = 1, 中间， Order = 2, 最大，
static pcl::PointXYZRGB GetDirectionVector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr, int Order)
{
	double a, b, c;
	Eigen::Matrix3f EigenVector;		//特征向量
	Eigen::Vector3f EigenValue(3);

	pcl::PointXYZRGB TempCenterPoint;

	TempCenterPoint = GetPointsTangentPlane(PointPtr, EigenVector, EigenValue);

	if (Order == 2)
	{
		a = EigenVector(6);
		b = EigenVector(7);
		c = EigenVector(8);
	}
	else if (Order == 1)
	{
		a = EigenVector(3);
		b = EigenVector(4);
		c = EigenVector(5);
	}
	else if (Order == 0)
	{
		a = EigenVector(0);
		b = EigenVector(1);
		c = EigenVector(2);
	}

	//法向量标准化
	double SqrtValue = sqrt(a*a + b * b + c * c);
	a = a / SqrtValue; b = b / SqrtValue; c = c / SqrtValue;
	pcl::PointXYZRGB NormalPoint;
	NormalPoint.x = a;
	NormalPoint.y = b;
	NormalPoint.z = c;

	//2018.09.28 更改为按照点的次序来确定方向
	pcl::PointXYZRGB TempDirection;
	TempDirection.x = PointPtr->points[PointPtr->points.size() - 1].x - PointPtr->points[PointPtr->points.size() - 2].x;
	TempDirection.y = PointPtr->points[PointPtr->points.size() - 1].y - PointPtr->points[PointPtr->points.size() - 2].y;
	TempDirection.z = PointPtr->points[PointPtr->points.size() - 1].z - PointPtr->points[PointPtr->points.size() - 2].z;
	double VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(TempDirection.x, TempDirection.y, TempDirection.z,
		NormalPoint.x, NormalPoint.y, NormalPoint.z) / M_PI;

	if (VectorAnlge > 90)
	{
		NormalPoint.x = -NormalPoint.x, NormalPoint.y = -NormalPoint.y, NormalPoint.z = -NormalPoint.z;
	}

	return NormalPoint;
}

//获取点集的最大方向向量 其方向向量的方向与点集的排列次序有关系
static pcl::PointXYZRGB GetMaxDirectionVector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr)
{
	double a, b, c;
	Eigen::Matrix3f EigenVector;		//特征向量
	Eigen::Vector3f EigenValue(3);

	pcl::PointXYZRGB TempCenterPoint;
		
	TempCenterPoint = GetPointsTangentPlane(PointPtr, EigenVector, EigenValue);
	
	a = EigenVector(6);
	b = EigenVector(7);
	c = EigenVector(8);
	//法向量标准化
	double SqrtValue = sqrt(a*a + b*b + c*c);
	a = a/SqrtValue; b = b/SqrtValue; c = c/SqrtValue;	
	pcl::PointXYZRGB NormalPoint;
	NormalPoint.x = a;
	NormalPoint.y = b;
	NormalPoint.z = c;

	//2018.09.28 更改为按照点的次序来确定方向
	pcl::PointXYZRGB TempDirection;	
	TempDirection.x = PointPtr->points[PointPtr->points.size() - 1].x - PointPtr->points[PointPtr->points.size() - 2].x;
	TempDirection.y = PointPtr->points[PointPtr->points.size() - 1].y - PointPtr->points[PointPtr->points.size() - 2].y;
	TempDirection.z = PointPtr->points[PointPtr->points.size() - 1].z - PointPtr->points[PointPtr->points.size() - 2].z;
	double VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(TempDirection.x, TempDirection.y, TempDirection.z, 
		NormalPoint.x, NormalPoint.y, NormalPoint.z) / M_PI;
	
	if (VectorAnlge > 90) 
	{
		NormalPoint.x = -NormalPoint.x, NormalPoint.y = -NormalPoint.y, NormalPoint.z = -NormalPoint.z;
	}

	//if (NormalPoint.z < 0) //
	//{
	//	NormalPoint.x = -NormalPoint.x; NormalPoint.y = -NormalPoint.y; NormalPoint.z = -NormalPoint.z;
	//}

	return NormalPoint;
}

static pcl::PointXYZRGB GetMinDirectionVector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr)
{
	double a, b, c;
	Eigen::Matrix3f EigenVector;		//特征向量
	Eigen::Vector3f EigenValue(3);

	pcl::PointXYZRGB TempCenterPoint;

	TempCenterPoint = GetPointsTangentPlane(PointPtr, EigenVector, EigenValue);

	a = EigenVector(0);
	b = EigenVector(1);
	c = EigenVector(2);
	//法向量标准化
	double SqrtValue = sqrt(a*a + b * b + c * c);
	a = a / SqrtValue; b = b / SqrtValue; c = c / SqrtValue;
	pcl::PointXYZRGB NormalPoint;
	NormalPoint.x = a;
	NormalPoint.y = b;
	NormalPoint.z = c;

	//2018.09.28 更改为按照点的次序来确定方向
	pcl::PointXYZRGB TempDirection;
	TempDirection.x = PointPtr->points[PointPtr->points.size() - 1].x - PointPtr->points[PointPtr->points.size() - 2].x;
	TempDirection.y = PointPtr->points[PointPtr->points.size() - 1].y - PointPtr->points[PointPtr->points.size() - 2].y;
	TempDirection.z = PointPtr->points[PointPtr->points.size() - 1].z - PointPtr->points[PointPtr->points.size() - 2].z;
	double VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(TempDirection.x, TempDirection.y, TempDirection.z,
		NormalPoint.x, NormalPoint.y, NormalPoint.z) / M_PI;

	if (VectorAnlge > 90)
	{
		NormalPoint.x = -NormalPoint.x, NormalPoint.y = -NormalPoint.y, NormalPoint.z = -NormalPoint.z;
	}

	//if (NormalPoint.z < 0) //
	//{
	//	NormalPoint.x = -NormalPoint.x; NormalPoint.y = -NormalPoint.y; NormalPoint.z = -NormalPoint.z;
	//}

	return NormalPoint;
}
//返回值为重心点
static pcl::PointXYZRGB GetPointsTangentPlane(
	pcl::PointCloud<PointXYZRGBIndex>::Ptr PointPtr,
	double & a, double & b, double & c)
{
	Eigen::Matrix3f EigenVector;		//特征向量
	Eigen::Vector3f EigenValue(3);

	pcl::PointXYZRGB TempCenterPoint;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPtr (new pcl::PointCloud<pcl::PointXYZRGB>);;
	PointBase::PointXYZRGBIndexToPointXYZRGB(PointPtr, TempPtr);
	TempCenterPoint = GetPointsTangentPlane(TempPtr, EigenVector, EigenValue);
	
	a = EigenVector(0);
	b = EigenVector(1);
	c = EigenVector(2);
	//法向量标准化
	double SqrtValue = sqrt(a*a + b*b + c*c);
	a = a/SqrtValue; b = b/SqrtValue; c = c/SqrtValue;
	if (c < 0)
	{
		a = -a; b = -b; c = -c;
	}
	return TempCenterPoint;
}

//计算给定点集的切平面法向量 2016.01.11 Weight 是否加权，以当前点为中心
///返回的是法向量 与 之前的函数不同
static pcl::PointXYZRGB GetPointsTangentPlaneBySelfAndWighted(
	int CurrentIndex,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr, bool Weight = false)			//特征值
{
	int N = PointPtr->points.size();
	Eigen::MatrixXf MatrixA = Eigen::MatrixXf::Zero(N - 1,3); //原始矩阵	 

	//构建原始矩阵
	int j = 0; //标准矩阵的行号
	for(int i = 0; i < N; i++)
	{
		if (i != CurrentIndex)
		{
			double Dis = 1;
			if (Weight)	//如果是加权矩阵
			{
				Dis = (PointDis(PointPtr->points[i], PointPtr->points[CurrentIndex]))/(N-1);

				Dis = pow((1-Dis), 4) * (1 + 4*Dis);
			}
			
			MatrixA(j, 0) = (PointPtr->points[i].x - PointPtr->points[CurrentIndex].x) * Dis;
			MatrixA(j, 1) = (PointPtr->points[i].y - PointPtr->points[CurrentIndex].y) * Dis;
			MatrixA(j, 2) = (PointPtr->points[i].z - PointPtr->points[CurrentIndex].z) * Dis;

			j++;
		}
	}
	//转置矩阵
	Eigen::MatrixXf B = MatrixA.transpose();
	//协方差矩阵
	Eigen::Matrix3f Covariance_Matrix = B * MatrixA;

	Eigen::Matrix3f EigenVector;		//特征向量
	Eigen::Vector3f EigenValue(3);
	pcl::eigen33(Covariance_Matrix, EigenVector, EigenValue); 

	pcl::PointXYZRGB NormalPoint;
	NormalPoint.x = EigenVector(0);
	NormalPoint.y = EigenVector(1);
	NormalPoint.z = EigenVector(2);
	PointBase::PointNormalized(NormalPoint);

	return NormalPoint;
}


//计算给定点集的切平面法向量 返回值为重心点
static pcl::PointXYZRGB GetPointsTangentPlane(
	pcl::PointCloud<pcl::PointXYZ>::Ptr PointPtr, 	
	Eigen::Matrix3f & EigenVector,		//特征向量
	Eigen::Vector3f & EigenValue)		//特征值
	//double & D)	//平面方程ax+by+cz+d = 0 中的 d 值
{
	Eigen::Matrix3f Covariance_Matrix;	//协方差矩阵
    Eigen::Vector4f CentroidPoint;		//中心点
	pcl::PointXYZRGB TempCenterPoint;	//质心点

	pcl::compute3DCentroid(*PointPtr, CentroidPoint);
	TempCenterPoint.x = CentroidPoint(0);
	TempCenterPoint.y = CentroidPoint(1);
	TempCenterPoint.z = CentroidPoint(2);
	TempCenterPoint.rgba = ColorBase::RedColor;
	
	//PointBase::SavePCDToFileName(PointPtr, "ProcessedPoints\\Test.pcd");
	
	//pcl::computeCovarianceMatrix(CloudPtr, CentroidPoint, Covariance_matrix);
	pcl::computeCovarianceMatrixNormalized(*PointPtr, CentroidPoint, Covariance_Matrix);	

	//Eigen::EigenSolver<Eigen::Matrix3f> es(Covariance_Matrix);
	//cout << "The eigenvalues of A are:" << endl << es.eigenvalues() << endl;
	//cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;

	//这个才是正确的参数调用方式，备注给的有错误
	pcl::eigen33(Covariance_Matrix, EigenVector, EigenValue); 
	//cout<<"协方差矩阵矩阵:"<<endl<<Covariance_Matrix<<endl;	
	////cout<<"最小特征值:"<<endl<<EigenValue(0)<<endl;
	////cout<<"最小特征值对应的特征向量:"<<endl<<EigenVector(0)<<endl;
	////cout<<EigenVector(1)<<endl;
	////cout<<EigenVector(2)<<endl;

	//cout<<"特征值:"<<endl<<EigenValue<<endl;
	//cout<<"特征向量:"<<endl<<EigenVector<<endl;

	//D = (CentroidPoint(0) * EigenVector(0) + CentroidPoint(1) * EigenVector(1)
	//	+ CentroidPoint(2) * EigenVector(2));
	return TempCenterPoint;
}

//返回值为重心点
static pcl::PointXYZRGB GetPointsTangentPlane(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr, 	
	Eigen::Matrix3f & EigenVector,		//特征向量
	Eigen::Vector3f & EigenValue)		//特征值
	//double & D)	//平面方程ax+by+cz+d = 0 中的 d 值
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr PointPtr (new pcl::PointCloud<pcl::PointXYZ>);
	PointBase::PointXYZRGBToXYZ(PointXYZRGBPtr, PointPtr);
	//GetPointsTangentPlane(PointPtr, TempCenterPoint,EigenVector, EigenValue, D); 
	return GetPointsTangentPlane(PointPtr, EigenVector, EigenValue); 
}

// Get the axis direction of the cylinder points
static pcl::PointXYZRGB GetCylinderDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CylinderPoints)
{
	double a, b, c;
	Eigen::Matrix3f EigenVector;		
	Eigen::Vector3f EigenValue(3);

	pcl::PointXYZRGB ResultPoint(0,0,0);

	GetPointsTangentPlane(CylinderPoints, EigenVector, EigenValue);

	if ((EigenValue(0) - EigenValue(1) == 0))
	{
		ResultPoint.x = EigenVector(6);
		ResultPoint.y = EigenVector(7);
		ResultPoint.z = EigenVector(8);
	}
	else if ((EigenValue(2) - EigenValue(1) == 0))
	{
		ResultPoint.x = EigenVector(0);
		ResultPoint.y = EigenVector(1);
		ResultPoint.z = EigenVector(2);
	}
	else
	{
		double Mutiple = abs((EigenValue(2) - EigenValue(1)) / (EigenValue(0) - EigenValue(1)));

		if (Mutiple > 10)
		{
			ResultPoint.x = EigenVector(6);
			ResultPoint.y = EigenVector(7);
			ResultPoint.z = EigenVector(8);
		}
		else if(Mutiple < 0.1)
		{
			ResultPoint.x = EigenVector(0);
			ResultPoint.y = EigenVector(1);
			ResultPoint.z = EigenVector(2);
		}
	}

	return ResultPoint;
}

//根据空间三点坐标 获取 法向量 2015.07.30
static void GetNormalByThreePoints(double X1, double Y1, double Z1,
	double X2, double Y2, double Z2,
	double X3, double Y3, double Z3, double & a, double & b, double & c)
{
	a = ((Y2 - Y1)*(Z3 - Z1) - (Z2 - Z1)*(Y3 - Y1));
	b = ((Z2 - Z1)*(X3 - X1) - (X2 - X1)*(Z3 - Z1));  
	c = ((X2 - X1)*(Y3 - Y1) - (Y2 - Y1)*(X3 - X1));
}
template<typename PointT>
static void GetPlaneByThreePoints(PointT A, PointT B, PointT C, double & a, double & b, double & c, double & d)
{
	GetNormalByThreePoints(A.x, A.y, A.z, B.x, B.y, B.z, C.x, C.y, C.z, a, b, c);
	d = -(a * A.x + b * A.y + c * A.z);
}

//根据三点获取平面方程	2015.07.30
static void GetPlaneByThreePoints(double X1, double Y1, double Z1,
	double X2, double Y2, double Z2,
	double X3, double Y3, double Z3, double & a, double & b, double & c, double & d)
{
	GetNormalByThreePoints(X1,Y1,Z1,X2,Y2,Z2,X3,Y3,Z3,a,b,c);
	d = -(a * X1 + b * Y1 + c * Z1);  	
}

//点到平面的距离 2015.07.30
static double GetDisFromPointToPlane(double X1, double Y1, double Z1,
	double a, double b, double c, double d)
{
	return fabs(a * X1 + b * Y1 + c * Z1 + d)/sqrt(a*a+b*b+c*c);  
}

//点到线段的距离 2020.05.28
static double GetDisFromPointToLineSegment(double X1, double Y1, double Z1,
	double AX, double AY, double AZ, double BX, double BY, double BZ)
{
	double Area = fabs(AreaOfThreePointsIn3D(X1, Y1, Z1, AX, AY, AZ, BX, BY, BZ));
	
	return  2 * Area / (PointDis(AX, AY, AZ, BX, BY, BZ));
}

static double AngleToRadian(double Angle)
{
	return Angle * M_PI / 180.0;
}

static double RadianToAngle(double Radian)
{
	return Radian * 180.0 / M_PI;
}

//返回两个向量之间的夹角  弧度值
static double AngleOfTwoVector(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double Value = (x1 * x2 + y1 * y2 + z1 * z2)
			/(sqrt(pow(x1, 2) + pow(y1, 2) + pow(z1, 2)))
			/(sqrt(pow(x2, 2) + pow(y2, 2) + pow(z2, 2)));

	//cout << "x1 Value：" << x1 ;
	//cout << "y1 Value：" << y1;
	//cout << "z1 Value：" << z1;

	//cout << "x2 Value：" << x2;
	//cout << "y2 Value：" << y2;
	//cout << "z2 Value：" << z2 << endl;

	//cout<<"AngleOfTwoVector Value："<< Value <<endl;

	if (Value > 1.0) Value = 1.0;

	return acos(Value);	// return value is in [0, M_PI]
	//return acos(abs(Value)); //2019.06.21 	// return value is in [0, M_PI / 2]
}

//返回两个向量之间的夹角  弧度值
static double AngleOfTwoVector(pcl::PointXYZRGB A, pcl::PointXYZRGB B)
{
	return AngleOfTwoVector(A.x, A.y, A.z, B.x, B.y, B.z);
}

static double AngleOfTwoVector(pcl::Normal A, pcl::Normal B)
{
	return AngleOfTwoVector(A.normal_x, A.normal_y, A.normal_z, B.normal_x, B.normal_y, B.normal_z);
}

static double CosOfTwoVector(double x1, double y1, double z1, 
	double x2, double y2, double z2)
{
	double Value = (x1 * x2 + y1 * y2 + z1 * z2) 
		/ (sqrt(pow(x1, 2) + pow(y1, 2) + pow(z1, 2)))
		/ (sqrt(pow(x2, 2) + pow(y2, 2) + pow(z2, 2)));

	if (Value > 1.0) Value = 1.0;	
	else if (Value < -1.0) Value = -1.0;
	return Value;
}

static double CosOfTwoVector(pcl::PointXYZRGB A, pcl::PointXYZRGB B)
{
	double Value = (A.x * B.x + A.y * B.y + A.z * B.z) 
		/ (sqrt(pow(A.x, 2) + pow(A.y, 2) + pow(A.z, 2)))
		/ (sqrt(pow(B.x, 2) + pow(B.y, 2) + pow(B.z, 2)));

	if (Value > 1.0) Value = 1.0;
	return Value;
}

static double CosOfTwoVector(pcl::Normal A, pcl::Normal B)
{
	double Value = (A.normal_x * B.normal_x + A.normal_y * B.normal_y + A.normal_z * B.normal_z)
		/ (sqrt(pow(A.normal_x, 2) + pow(A.normal_y, 2) + pow(A.normal_z, 2)))
		/ (sqrt(pow(B.normal_x, 2) + pow(B.normal_y, 2) + pow(B.normal_z, 2)));

	if (Value > 1.0) Value = 1.0;
	return Value;
}

//根据两点和与平面平行的向量Normal(NormalA, NormalB, NormalC)获取平面方程 2015.07.30
//平面的法向量为P1P2×Normal
static void GetPlaneByTwoPointsAndParalleledNormal(double X1, double Y1, double Z1,
	double X2, double Y2, double Z2,
	double NormalA, double NormalB, double NormalC, 
	double & a, double & b, double & c, double & d)
{
	a = (Y2-Y1)*NormalC - (Z2-Z1)*NormalB;
	b = (Z2-Z1)*NormalA - (X2-X1)*NormalC;	//注意符号
	c = (X2-X1)*NormalB - (Y2-Y1)*NormalA;
	d = -(a * X1 + b * Y1 + c * Z1);  
}

static void ProjectPointsToPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints,
	pcl::PointXYZRGB PanelNormal,
	pcl::PointXYZRGB PanelPoint)
{
	double d;
	d = - (PanelPoint.x * PanelNormal.x + PanelPoint.y * PanelNormal.y 
			+ PanelPoint.z * PanelNormal.z);
	ProjectPointsToPlane(InPoints, OutPoints, PanelNormal.x, PanelNormal.y, PanelNormal.z, d);
}

//将点集中的点 投影到 一个平面上
static void ProjectPointsToPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints,
	double a, double b, double c, double d)
{
	pcl::ModelCoefficients::Ptr PlaneCoefficients (new pcl::ModelCoefficients);	
	PlaneCoefficients->values.push_back(a);
	PlaneCoefficients->values.push_back(b);
	PlaneCoefficients->values.push_back(c);
	PlaneCoefficients->values.push_back(d);
	
	//投影	
	OutPoints->points.clear();
	pcl::ProjectInliers<pcl::PointXYZRGB> proj; //点云投影滤波对象
	proj.setModelType (pcl::SACMODEL_PLANE);//设置投影模型为SACMODEL_PLANE。
	proj.setInputCloud (InPoints);     //设置输入点云为滤波后的点云cloud_filtered。
	proj.setModelCoefficients(PlaneCoefficients);//将估计得到的平面coefficients参数设置为投影平面模型系数
	proj.filter(*OutPoints);          //将滤波后的点云投影到平面模型中得到投影后的点云
}


////点云绕X轴旋转 弧度
static void RotateX(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints,
	double Angle)
{
	Eigen::Matrix3f RotateMatrix = Eigen::Matrix3f::Zero(3, 3);

	RotateMatrix(0,0)= 1;
	RotateMatrix(1,1)= cos(Angle);
	RotateMatrix(1,2)= -sin(Angle);
	RotateMatrix(2,1)= sin(Angle);
	RotateMatrix(2,2)= cos(Angle);

	//RotateMatrix(1,1)= cos( M_PI* Angle/180);
	//RotateMatrix(1,2)= -sin( M_PI* Angle/180);
	//RotateMatrix(2,1)= sin( M_PI* Angle/180);
	//RotateMatrix(2,2)= cos( M_PI* Angle/180);
	OutPoints->points.clear();
	for(int i = 0; i < Points->points.size(); i++)
	{
		Eigen::Vector3f OldVector, NewVector;
		
		OldVector(0) = Points->points[i].x;
		OldVector(1) = Points->points[i].y;
		OldVector(2) = Points->points[i].z;

		NewVector = RotateMatrix * OldVector;
		
		pcl::PointXYZRGB TempPoint;
		TempPoint.x = NewVector(0);
		TempPoint.y = NewVector(1);
		TempPoint.z = NewVector(2);
		TempPoint.r = Points->points[i].r;
		TempPoint.g = Points->points[i].g;
		TempPoint.b = Points->points[i].b;
		OutPoints->points.push_back(TempPoint);
	}
}

//点云绕Y轴旋转
static void RotateY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints, double Angle)
{
	Eigen::Matrix3f RotateMatrix = Eigen::Matrix3f::Zero(3, 3);

	RotateMatrix(0,0)= cos(Angle);
	RotateMatrix(0,2)= -sin(Angle);
	RotateMatrix(1,1)= 1;
	RotateMatrix(2,0)= sin(Angle);
	RotateMatrix(2,2)= cos(Angle);

	//RotateMatrix(1,1)= cos( M_PI* Angle/180);
	//RotateMatrix(1,2)= -sin( M_PI* Angle/180);
	//RotateMatrix(2,1)= sin( M_PI* Angle/180);
	//RotateMatrix(2,2)= cos( M_PI* Angle/180);
	OutPoints->points.clear();
	for(int i = 0; i < Points->points.size(); i++)
	{
		Eigen::Vector3f OldVector, NewVector;
		
		OldVector(0) = Points->points[i].x;
		OldVector(1) = Points->points[i].y;
		OldVector(2) = Points->points[i].z;

		NewVector = RotateMatrix * OldVector;
		
		pcl::PointXYZRGB TempPoint;
		TempPoint.x = NewVector(0);
		TempPoint.y = NewVector(1);
		TempPoint.z = NewVector(2);
		TempPoint.r = Points->points[i].r;
		TempPoint.g = Points->points[i].g;
		TempPoint.b = Points->points[i].b;
		OutPoints->points.push_back(TempPoint);
	}
}

//点云绕Z轴旋转
static void RotateZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints, double Angle)
{
	Eigen::Matrix3f RotateMatrix = Eigen::Matrix3f::Zero(3, 3);
	RotateMatrix(0,0)= cos(Angle);
	RotateMatrix(0,1)= -sin(Angle);
	RotateMatrix(1,0)= sin(Angle);
	RotateMatrix(1,1)= cos(Angle);
	RotateMatrix(2,2)= 1;

	//RotateMatrix(1,1)= cos( M_PI* Angle/180);
	//RotateMatrix(1,2)= -sin( M_PI* Angle/180);
	//RotateMatrix(2,1)= sin( M_PI* Angle/180);
	//RotateMatrix(2,2)= cos( M_PI* Angle/180);
	OutPoints->points.clear();
	for(int i = 0; i < Points->points.size(); i++)
	{
		Eigen::Vector3f OldVector, NewVector;
		
		OldVector(0) = Points->points[i].x;
		OldVector(1) = Points->points[i].y;
		OldVector(2) = Points->points[i].z;

		NewVector = RotateMatrix * OldVector;
		
		pcl::PointXYZRGB TempPoint;
		TempPoint.x = NewVector(0);
		TempPoint.y = NewVector(1);
		TempPoint.z = NewVector(2);
		TempPoint.r = Points->points[i].r;
		TempPoint.g = Points->points[i].g;
		TempPoint.b = Points->points[i].b;
		OutPoints->points.push_back(TempPoint);
	}
}

//将平面点集旋转到与向量StemCrossSectionNormal垂直的平面上去
static void RotateToOriginal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints,
	pcl::PointXYZRGB StemCrossSectionNormal, bool IsVertical = true)
{	
	PointBase::PointNormalized(StemCrossSectionNormal);
	if (StemCrossSectionNormal.z < 0)
	{
		StemCrossSectionNormal.x = -StemCrossSectionNormal.x;
		StemCrossSectionNormal.y = -StemCrossSectionNormal.y;
		StemCrossSectionNormal.z = -StemCrossSectionNormal.z;
	}

	//It can not work when StemCrossSectionNormal is (0,1,0), So add the this step 2018.12.27
	if (abs(StemCrossSectionNormal.x) < EPSM6 &&
		abs(StemCrossSectionNormal.y - 1) < EPSM6 && abs(StemCrossSectionNormal.z) < EPSM6)
		IsVertical = false;

	double XAnlge, YAnlge; 
	if (IsVertical)
	{	
		if (StemCrossSectionNormal.y * StemCrossSectionNormal.z != 0)
			XAnlge = asin(StemCrossSectionNormal.y/sqrt(
				StemCrossSectionNormal.y * StemCrossSectionNormal.y +
				StemCrossSectionNormal.z * StemCrossSectionNormal.z));
		else
			XAnlge = 0;	
		
		YAnlge = asin(StemCrossSectionNormal.x/sqrt(
				StemCrossSectionNormal.x * StemCrossSectionNormal.x +
				StemCrossSectionNormal.y * StemCrossSectionNormal.y + 
				StemCrossSectionNormal.z * StemCrossSectionNormal.z));
	}
	else
	{
		if (StemCrossSectionNormal.y * StemCrossSectionNormal.z != 0)
			XAnlge = asin(StemCrossSectionNormal.y/sqrt(
				StemCrossSectionNormal.y * StemCrossSectionNormal.y +
				StemCrossSectionNormal.z * StemCrossSectionNormal.z));
		else
			XAnlge = 0;
	
		YAnlge = -asin(sqrt(StemCrossSectionNormal.y*StemCrossSectionNormal.y +
				StemCrossSectionNormal.z*StemCrossSectionNormal.z)/
				sqrt(StemCrossSectionNormal.x*StemCrossSectionNormal.x +
				StemCrossSectionNormal.y*StemCrossSectionNormal.y + 
				StemCrossSectionNormal.z*StemCrossSectionNormal.z));	
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints (new pcl::PointCloud<pcl::PointXYZRGB>);

	//正向旋转 的 逆向操作
	RotateY(Points, TempPoints, -YAnlge);
	RotateX(TempPoints, OutPoints, -XAnlge);	
}

//将点集的法向 旋转到与XY平行 即将点集旋转到 XY 垂直的平面上
static void RotateNormalToHorizontal(pcl::PointCloud<PointXYZRGBIndex>::Ptr Points,
	pcl::PointCloud<PointXYZRGBIndex>::Ptr OutPoints,
	pcl::PointXYZRGB StemCrossSectionNormal)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPointsRGB (new pcl::PointCloud<pcl::PointXYZRGB>);

	PointBase::PointXYZRGBIndexToPointXYZRGB(Points, PointsRGB);
	RotateNormalToHorizontal(PointsRGB, OutPointsRGB, StemCrossSectionNormal);
	PointBase::PointXYZRGBToPointXYZRGBIndex(OutPointsRGB, OutPoints);
}

//将点集的法向 旋转到与XY平行 即将点集旋转到 XY 垂直的平面上
static void RotateNormalToHorizontal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints,
	pcl::PointXYZRGB StemCrossSectionNormal)
{
	if (StemCrossSectionNormal.z < 0)
	{
		StemCrossSectionNormal.x = -StemCrossSectionNormal.x;
		StemCrossSectionNormal.y = -StemCrossSectionNormal.y;
		StemCrossSectionNormal.z = -StemCrossSectionNormal.z;	
	}

	double XAnlge;
	if (StemCrossSectionNormal.y * StemCrossSectionNormal.z != 0)
		XAnlge = asin(StemCrossSectionNormal.y/sqrt(
			StemCrossSectionNormal.y * StemCrossSectionNormal.y +
			StemCrossSectionNormal.z * StemCrossSectionNormal.z));
	else
		XAnlge = 0;

	//double XAnlge = asin(StemCrossSectionNormal.y/sqrt(StemCrossSectionNormal.y*StemCrossSectionNormal.y +
	//	StemCrossSectionNormal.z*StemCrossSectionNormal.z));
	
	double YAnlge = -asin(sqrt(StemCrossSectionNormal.y*StemCrossSectionNormal.y +
		StemCrossSectionNormal.z*StemCrossSectionNormal.z)/
		sqrt(StemCrossSectionNormal.x*StemCrossSectionNormal.x +
		StemCrossSectionNormal.y*StemCrossSectionNormal.y + 
		StemCrossSectionNormal.z*StemCrossSectionNormal.z));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints (new pcl::PointCloud<pcl::PointXYZRGB>);

	RotateX(Points, TempPoints, XAnlge);
	RotateY(TempPoints, OutPoints, YAnlge);
}

//将点集的法向 旋转到与XY平面垂直 即将点集旋转到 XY 平行的平面上
static void RotateNormalToVertical(pcl::PointCloud<PointXYZRGBIndex>::Ptr Points,
	pcl::PointCloud<PointXYZRGBIndex>::Ptr OutPoints,
	pcl::PointXYZRGB StemCrossSectionNormal)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPointsRGB (new pcl::PointCloud<pcl::PointXYZRGB>);

	PointBase::PointXYZRGBIndexToPointXYZRGB(Points, PointsRGB);
	RotateNormalToVertical(PointsRGB, OutPointsRGB, StemCrossSectionNormal);
	PointBase::PointXYZRGBToPointXYZRGBIndex(OutPointsRGB, OutPoints);
}

//将点集的法向 旋转到与XY平面垂直 即将点集旋转到 XY 平行的平面上
static void RotateNormalToVertical(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints,
	pcl::PointXYZRGB StemCrossSectionNormal)
{
	if (StemCrossSectionNormal.z < 0)
	{
		StemCrossSectionNormal.x = -StemCrossSectionNormal.x;
		StemCrossSectionNormal.y = -StemCrossSectionNormal.y;
		StemCrossSectionNormal.z = -StemCrossSectionNormal.z;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	double XAnlge;
	if (StemCrossSectionNormal.y * StemCrossSectionNormal.y +
			StemCrossSectionNormal.z * StemCrossSectionNormal.z > 0)
		XAnlge = asin(StemCrossSectionNormal.y/sqrt(
			StemCrossSectionNormal.y * StemCrossSectionNormal.y +
			StemCrossSectionNormal.z * StemCrossSectionNormal.z));
	else
		XAnlge = 0;
	
	double YAnlge = asin(StemCrossSectionNormal.x/sqrt(
		StemCrossSectionNormal.x * StemCrossSectionNormal.x +
		StemCrossSectionNormal.y * StemCrossSectionNormal.y + 
		StemCrossSectionNormal.z * StemCrossSectionNormal.z));

	RotateX(Points, TempPoints, XAnlge);

	RotateY(TempPoints, OutPoints, YAnlge);

	//double a,b,c;
	//GeometryBase::GetPointsTangentPlane(OutPoints,a,b,c);
}

//将点集的旋转到与XY平面平行 即将点集旋转到 XY 平行的平面上
//static void RotatePointsToParallelOfXY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints)
//{
//	double a,b,c;
//	GeometryBase::GetPointsTangentPlane(Points,a,b,c); //这种方法有时也是有错误的
//	pcl::PointXYZRGB PointsNormal;
//	PointsNormal.x = a, PointsNormal.y = b, PointsNormal.z =c;
//
//	RotateNormalToVertical(Points, OutPoints, PointsNormal);
//}
//
////将点集的旋转到与XY平面垂直 即将点集旋转到 XY 垂直的平面上
//static void RotatePointsToVerticalOfXY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints)
//{
//	double a,b,c;
//	GeometryBase::GetPointsTangentPlane(Points,a,b,c);
//	pcl::PointXYZRGB PointsNormal;
//	PointsNormal.x = a, PointsNormal.y = b, PointsNormal.z =c;
//
//	RotateNormalToHorizontal(Points, OutPoints, PointsNormal);
//}

//PointB 绕 PointA 旋转 theta 弧度
static pcl::PointXYZRGB RotatePointBAroundPointA(pcl::PointXYZRGB PointA, pcl::PointXYZRGB PointB, double Angle)
{
	pcl::PointXYZRGB Result;

	Result.x = (PointB.x - PointA.x) * cos(Angle) - (PointB.y - PointA.y)* sin(Angle) + PointA.x;
	Result.y = (PointB.x - PointA.x) * sin(Angle) + (PointB.y - PointA.y)* cos(Angle) + PointA.y;
	Result.z = PointA.z;

	return Result;
}

//点云绕向量(x,y,z)轴旋转
static void RotateVector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints,
	double x, double y, double z,
	double Angle)
{
	Eigen::Matrix3f RotateMatrix = Eigen::Matrix3f::Zero(3, 3);
	double c = cos(Angle), s = sin(Angle);
	RotateMatrix(0,0)= x*x*(1-c) + c;
	RotateMatrix(0,1)= x*y*(1-c) - z * s;
	RotateMatrix(0,2)= x*z*(1-c) + y * s;

	RotateMatrix(1,0)= y*x*(1-c) + z * s;
	RotateMatrix(1,1)= y*y*(1-c) + c;
	RotateMatrix(1,2)= y*z*(1-c) - x * s;

	RotateMatrix(2,0)= x*z*(1-c) - y*s;
	RotateMatrix(2,1)= y*z*(1-c) + x*s;
	RotateMatrix(2,2)= z*z*(1-c) + c;

	//RotateMatrix(1,1)= cos( M_PI* Angle/180);
	//RotateMatrix(1,2)= -sin( M_PI* Angle/180);
	//RotateMatrix(2,1)= sin( M_PI* Angle/180);
	//RotateMatrix(2,2)= cos( M_PI* Angle/180);
	OutPoints->points.clear();
	for(int i = 0; i < Points->points.size(); i++)
	{
		Eigen::Vector3f OldVector, NewVector;
		
		OldVector(0) = Points->points[i].x;
		OldVector(1) = Points->points[i].y;
		OldVector(2) = Points->points[i].z;

		NewVector = RotateMatrix * OldVector;
		
		pcl::PointXYZRGB TempPoint;
		TempPoint.x = NewVector(0);
		TempPoint.y = NewVector(1);
		TempPoint.z = NewVector(2);
		OutPoints->points.push_back(TempPoint);
	}
}
//获取直线y=ax+b与线段PointA->PointB的交点 如果有则返回，无则返回原点值
static pcl::PointXYZRGB CalcIntersectionOfLineAndSegment(
	double a, double b, pcl::PointXYZRGB PointA, pcl::PointXYZRGB PointB, bool & Have)
{
	pcl::PointXYZRGB ResultPoint;
	ResultPoint.x = 0; ResultPoint.y = 0; ResultPoint.z = 0;
	double a2 = (PointB.y - PointA.y) / (PointB.x - PointA.x);
	double b2 = PointB.y - a2 * PointB.x;
	ResultPoint = CalcIntersectionOfTwoLine(a, -1, b, a2, -1, b2, Have);
	if (Have)
	{
		if ((ResultPoint.x - PointA.x) * (ResultPoint.x - PointB.x) <= 0)
			Have = true;
		else
			Have = false;
	}
	return ResultPoint;
}


//在二维平面中计算两条直线的交点 a1*x+b1*y+d1 = 0 与 a2*x+b2*y+d2 = 0
static pcl::PointXYZRGB CalcIntersectionOfTwoLine(
	double a1, double b1, double d1, double a2, double b2, double d2, bool & Have)
{
	pcl::PointXYZRGB IntersectionPoint;

	Have = true;
	if ((a1*b2 - a2 * b1) == 0)
	{
		cout<<"两条直线平行，无交点"<<endl;
		Have = false;		
	}
	else if (b1 == 0)
	{
		IntersectionPoint.x = - d1/a1;
		IntersectionPoint.y = -(a2 * IntersectionPoint.x + d2)/b2;
	}
	else if (b2 == 0)
	{
		IntersectionPoint.x = - d2/a2;
		IntersectionPoint.y = -(a1 * IntersectionPoint.x + d1)/b1;	
	}
	else 
	{
		IntersectionPoint.x = (b1 * d2 - b2 * d1)/(a1*b2 - a2 * b1);		
		IntersectionPoint.y = -(a1 * IntersectionPoint.x + d1)/b1;	
	}	

	return IntersectionPoint;
}

//计算点 Point 在直线上(LineA-LineB)的投影 
static pcl::PointXYZRGB CalcProjectOFPointInLine(pcl::PointXYZRGB Point,
	pcl::PointXYZRGB LineA, pcl::PointXYZRGB LineB)
{
	pcl::PointXYZRGB ProjectPoint = LineA;
	if (LineA.x - LineB.x == 0)
	{
		ProjectPoint = Point;		
		ProjectPoint.x = LineA.x;			
	}
	else if (LineA.y - LineB.y == 0)
	{
		ProjectPoint = Point;		
		ProjectPoint.y = LineA.y;		
	}
	else
	{
		double Line_a, Point_a, Point_b;
		//凸包的直线段
		Line_a = (LineA.y - LineB.y)/(LineA.x - LineB.x);	

		//过点的直线
		Point_a = 1 / Line_a;
		Point_b = Point.y - Point_a * Point.x;
		
		bool Have; //两个直线的交点
		ProjectPoint = GeometryBase::
			CalcIntersectionOfLineAndSegment(Point_a, Point_b, LineA, LineB, Have);	
	}

	return ProjectPoint;
}

//2016.04.30 获取线段上的点
static void GetPointsInLineSegement(pcl::PointXYZRGB LineA, pcl::PointXYZRGB LineB, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints)
{

}

//计算线段(LineA->CircleCenter)与圆(CircleCenter, Radius)的交点  假设圆心在原点，然后再平移得到交点
static pcl::PointXYZRGB CalcIntersectionOfByLinesegmentAndCircle(pcl::PointXYZRGB LineA,
	pcl::PointXYZRGB CircleCenter, double Radius)
{
	pcl::PointXYZRGB CrossA, CrossB;
	double a;
	if (LineA.x == CircleCenter.x)
	{
		CrossA.x = CircleCenter.x, CrossA.y = CircleCenter.y - Radius;
		CrossB.x = CircleCenter.x, CrossB.y = CircleCenter.y + Radius;
	}
	else
	{
		a = (LineA.y - CircleCenter.y)/(LineA.x - CircleCenter.x);
		CrossA.x = sqrt(Radius*Radius/(1+a*a)), CrossA.y = a*CrossA.x;
		CrossB.x = -CrossA.x, CrossB.y = a*CrossB.x;

		CrossA.x += CircleCenter.x, CrossA.y += CircleCenter.y;
		CrossB.x += CircleCenter.x, CrossB.y += CircleCenter.y;
	}
	//根据向量的居中性 判断返回那个点
	float A = CrossA.x - LineA.x;
	float B = CrossA.x - CircleCenter.x;
	if (A != 0 && B != 0)
	{
		if (A*B < 0) return CrossA;
		else return CrossB;
	}
	else
	{
		A = CrossA.y - LineA.y;
		B = CrossA.y - CircleCenter.y;
		if (A != 0 && B != 0)
		{
			if (A*B < 0) return CrossA;
			else return CrossB;			
		}
	}

}

//将点集中的点 投影到 一个平面上
static void ProjectPointsToPlane(pcl::PointCloud<PointXYZRGBIndex>::Ptr InPoints,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints,
	double a, double b, double c, double d)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempInPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
	PointBase::PointXYZRGBIndexToPointXYZRGB(InPoints, TempInPoints);
	ProjectPointsToPlane(TempInPoints, OutPoints, a, b, c, d);
}

static void GetPointsBetweenTwoPlanes(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	double a1, double b1, double c1, double d1,
	double a2, double b2, double c2, double d2,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BetweenPoints, int Category = 1,
	bool IncludePlane1 = true, bool IncludePlane2 = false, bool Deleted = false, vector<int> * Indexs = NULL)
{
	//根据两个平面的关系获取参考点，参考点应该位于两个平面之外
	pcl::PointXYZRGB ReferencePoint;

	ReferencePoint.x = 0; 
	ReferencePoint.y = 0; 
	ReferencePoint.z = 0;

	if (abs(c1) > EPSM3 && abs(c2) > EPSM3)
	{
		double Z1 = -d1 / c1;
		double Z2 = -d2 / c2;

		if (Z1 > Z2)
			ReferencePoint.z = Z1 + 1;
		else if (Z2 > Z1)
			ReferencePoint.z = Z2 + 1;
	}
	else if (abs(b1) > EPSM3 && abs(b2) > EPSM3)
	{
		double B1 = -d1 / b1;
		double B2 = -d2 / b2;

		if (B1 > B2)
			ReferencePoint.y = B1 + 1;
		else if (B2 > B1)
			ReferencePoint.y = B2 + 1;
	}
	else if (abs(a1) > EPSM3 && abs(a2) > EPSM3)
	{
		double A1 = -d1 / a1;
		double A2 = -d2 / a2;

		if (A1 > A2)
			ReferencePoint.x = A1 + 1;
		else if (A2 > A1)
			ReferencePoint.x = A2 + 1;
	}

	GetPointsBetweenTwoPlanesByOutSidePoint(Points,
		a1, b1, c1, d1,
		a2, b2, c2, d2,
		ReferencePoint,
		BetweenPoints, Category, IncludePlane1, IncludePlane2, Deleted, Indexs);
}


//取点云 Points 夹在两个平面中的点
// 根据参考点，参考点是位于两个平面之外的一个点
static void GetPointsBetweenTwoPlanesByOutSidePoint(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	double a1, double b1, double c1, double d1,
	double a2, double b2, double c2, double d2,
	pcl::PointXYZRGB ReferencePoint,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BetweenPoints, int Category = 1,
	bool IncludePlane1 = true, bool IncludePlane2 = false, bool Deleted = false, vector<int> * Indexs = NULL)
{
	BetweenPoints->points.clear();
	double SymbolPlane1 = ReferencePoint.x * a1 + ReferencePoint.y * b1 + ReferencePoint.z * c1 + d1;
	double SymbolPlane2 = ReferencePoint.x * a2 + ReferencePoint.y * b2 + ReferencePoint.z * c2 + d2;
	int i = 0;
	while(i < Points->points.size())
	//for(int i = 0; i < Points->points.size(); i++)
	{		
		pcl::PointXYZRGB TempPoint = Points->points[i];

		//if (TempPoint.Category != 1)	//
		//{
		//	if (Deleted)
		//		Points->points.erase(Points->points.begin() + i);
		//	else
		//		i++;
		//	continue;			
		//}

		double TempSymbolPlane1 = TempPoint.x * a1 + TempPoint.y * b1 + TempPoint.z * c1 + d1;
		double TempSymbolPlane2 = TempPoint.x * a2 + TempPoint.y * b2 + TempPoint.z * c2 + d2;
		
		if (SymbolPlane1 * SymbolPlane2 > 0)	//两个平面之外的点乘积大于 0  平面之内的点之乘积小于 0
		{
			//包括两个面上的点		
			if ((IncludePlane1 && IncludePlane2) 
				&& TempSymbolPlane1 * TempSymbolPlane2 <= 0)
			{
				BetweenPoints->points.push_back(Points->points[i]);	

				if (Indexs != NULL)
					Indexs->push_back(i);

				if (Deleted)
					Points->points.erase(Points->points.begin() + i);
				else
					i++;
			}
			//只包括面2上的点		
			else if ((!IncludePlane1 && IncludePlane2) 
				&& TempSymbolPlane1 * TempSymbolPlane2 <= 0 && (TempSymbolPlane1 != 0))
			{
				BetweenPoints->points.push_back(Points->points[i]);	
				if (Indexs != NULL)
					Indexs->push_back(i);

				if (Deleted)
					Points->points.erase(Points->points.begin() + i);
				else
					i++;
			}
			//只包括面1上的点		
			else if ((IncludePlane1 && !IncludePlane2) 
				&& TempSymbolPlane1 * TempSymbolPlane2 <= 0 && (TempSymbolPlane2 != 0))
			{
				BetweenPoints->points.push_back(Points->points[i]);	
				if (Indexs != NULL)
					Indexs->push_back(i);

				if (Deleted)
					Points->points.erase(Points->points.begin() + i);
				else
					i++;
			}
			//不包括两个面上的点		
			else if ((!IncludePlane1 && !IncludePlane2) 
				&& TempSymbolPlane1 * TempSymbolPlane2 < 0)
			{
				BetweenPoints->points.push_back(Points->points[i]);
				if (Indexs != NULL)
					Indexs->push_back(i);

				if (Deleted)
					Points->points.erase(Points->points.begin() + i);
				else
					i++;
			}
			else
				i++;
		}
		else
		{
			//包括两个面上的点		
			if ((IncludePlane1 && IncludePlane2) 
				&& TempSymbolPlane1 * TempSymbolPlane2 >= 0)
			{
				BetweenPoints->points.push_back(Points->points[i]);	
				if (Indexs != NULL)
					Indexs->push_back(i);

				if (Deleted)
					Points->points.erase(Points->points.begin() + i);
				else
					i++;
			}
			//只包括面2上的点		
			else if ((!IncludePlane1 && IncludePlane2) 
				&& TempSymbolPlane1 * TempSymbolPlane2 >= 0 && (TempSymbolPlane1 != 0))
			{
				BetweenPoints->points.push_back(Points->points[i]);
				if (Indexs != NULL)
					Indexs->push_back(i);

				if (Deleted)
					Points->points.erase(Points->points.begin() + i);
				else
					i++;
			}
			//只包括面1上的点		
			else if ((IncludePlane1 && !IncludePlane2) 
				&& TempSymbolPlane1 * TempSymbolPlane2 >= 0 && (TempSymbolPlane2 != 0))
			{
				BetweenPoints->points.push_back(Points->points[i]);	
				if (Indexs != NULL)
					Indexs->push_back(i);

				if (Deleted)
					Points->points.erase(Points->points.begin() + i);
				else
					i++;
			}
			//不包括两个面上的点		
			else if ((!IncludePlane1 && !IncludePlane2) 
				&& TempSymbolPlane1 * TempSymbolPlane2 > 0)
			{
				BetweenPoints->points.push_back(Points->points[i]);	
				if (Indexs != NULL)
					Indexs->push_back(i);

				if (Deleted)
					Points->points.erase(Points->points.begin() + i);
				else
					i++;
			}
			else
				i++;
		}		
	}
}


//Line direction is A to B, the result is the point along the direction and the dis is the dis between the result point and B
template<typename PointNT>
static PointNT GetPointAlongAToB(
	PointNT APoint, PointNT BPoint, double Dis, bool Is2D = false)
{
	pcl::PointXYZRGB Normal;
	if (Is2D)
		Normal.x = BPoint.x - APoint.x, Normal.y = BPoint.y - APoint.y, Normal.z = 0;
	else
		Normal.x = BPoint.x - APoint.x, Normal.y = BPoint.y - APoint.y, Normal.z = BPoint.z - APoint.z;
	
	return GetPointAlongLine(Normal, BPoint, Dis, Dis >= 0);
}

////根据公式计算很繁琐，需要求解一元四次方程组，实际可使用 0.618法计算
////输入时，PointA，DirectionA，PointB，DirectionB 四点呈逆时针次序排列
////在线段(DirectionA，DirectionB)上寻找一点ResultPoint，使得角(PointA, ResultPoint, PointB)为Angle(弧度)，且三点呈逆时针次序排列
//template<typename PointNT> 
//static PointNT GetMiddlePointAlongLineBetweenTwoPoints(PointNT DirectionA, PointNT DirectionB, 
//	PointNT PointA, PointNT PointB, double Angle)
//{
//	PointNT ResultPoint, Direction;
//	Direction.x = DirectionB.x - DirectionA.x;
//	Direction.y = DirectionB.y - DirectionA.y;
//	Direction.z = DirectionB.z - DirectionA.z;
//	//PointBase::PointNormalized(Direction);
//	double DisAB = PointDis(PointA, PointB);
//	double Lamda = 0;
//
//	double Lower = 0, Upper = 1;
//	double Range = Upper - Lower;
//	double Start_Try, End_Try;
//	Start_Try = Lower + 0.312 * Range;
//	End_Try = Lower + 0.618 * Range;
//
//	PointNT StartPoint, EndPoint;
//	StartPoint.x = DirectionA.x + Start_Try * Direction.x;
//	StartPoint.y = DirectionA.y + Start_Try * Direction.y;
//	StartPoint.z = DirectionA.z + Start_Try * Direction.z;
//
//	EndPoint.x = DirectionA.x + End_Try * Direction.x;
//	EndPoint.y = DirectionA.y + End_Try * Direction.y;
//	EndPoint.z = DirectionA.z + End_Try * Direction.z;
//
//	double StartAngle = GeometryBase::AngleValueOfThreePoints(PointA, StartPoint, PointB);
//	double EndAngle = GeometryBase::AngleValueOfThreePoints(PointA, EndPoint, PointB);	
//
//	while (abs(StartAngle - Angle) > EPSM3 || abs(EndAngle - Angle) > EPSM3)
//	{
//		double StartAngle = GeometryBase::AngleValueOfThreePoints(PointA, StartPoint, PointB);
//		double EndAngle = GeometryBase::AngleValueOfThreePoints(PointA, EndPoint, PointB);
//
//		if (abs(StartAngle - Angle) <= EPSM3)
//		{
//			Lamda = Start_Try;
//			break;
//		}
//		else if (abs(EndAngle - Angle) <= EPSM3)
//		{
//			Lamda = End_Try;
//			break;
//		}
//
//		if (EndAngle > Angle || (StartAngle + EndAngle) / 2.0 > Angle)
//		{			
//			Lower = Start_Try;
//			Range = Upper - Lower;
//			Start_Try = Lower + 0.312 * Range;
//			End_Try = Lower + 0.618 * Range;
//		}
//		else
//		{			
//			Upper = End_Try;
//			Range = Upper - Lower;
//			Start_Try = Lower + 0.312 * Range;
//			End_Try = Lower + 0.618 * Range;
//		}
//		cout << "StartAngle:" << StartAngle << endl;
//		cout << "EndAngle:" << EndAngle << endl;
//		cout << "Start_Try:" << Start_Try << endl;
//		cout << "End_Try:"<< End_Try << endl;
//
//		StartPoint.x = DirectionA.x + Start_Try * Direction.x;
//		StartPoint.y = DirectionA.y + Start_Try * Direction.y;
//		StartPoint.z = DirectionA.z + Start_Try * Direction.z;
//
//		EndPoint.x = DirectionA.x + End_Try * Direction.x;
//		EndPoint.y = DirectionA.y + End_Try * Direction.y;
//		EndPoint.z = DirectionA.z + End_Try * Direction.z;
//
//		if (abs(End_Try - Start_Try) < EPSM3) break;
//	}
//	cout << "Lamda:" << Lamda << endl;
//	ResultPoint.x = DirectionA.x + Lamda * Direction.x;
//	ResultPoint.y = DirectionA.y + Lamda * Direction.y;
//	ResultPoint.z = DirectionA.z + Lamda * Direction.z;
//	cin >> Lamda;
//	return ResultPoint;
//}

template<typename PointNT>
static PointNT GetPointAlongLine(
	pcl::PointXYZRGB Normal, PointNT APoint, double Dis, bool Alonged = true)
{
	PointBase::PointNormalized(Normal);

	PointNT ResultPoint0, ResultPoint1;
	double t = Dis / sqrt(pow(Normal.x, 2) + pow(Normal.y, 2) + pow(Normal.z, 2));

	ResultPoint0.x = APoint.x + Normal.x*t;
	ResultPoint0.y = APoint.y + Normal.y*t;
	ResultPoint0.z = APoint.z + Normal.z*t;

	ResultPoint1.x = APoint.x - Normal.x*t;
	ResultPoint1.y = APoint.y - Normal.y*t;
	ResultPoint1.z = APoint.z - Normal.z*t;

	if (Alonged)
	{
		if ((ResultPoint0.z - APoint.z) * Normal.z > 0
			|| (ResultPoint0.x - APoint.x) * Normal.x > 0 || (ResultPoint0.y - APoint.y) * Normal.y > 0)
			return ResultPoint0;
		else
			return ResultPoint1;
	}
	else
	{
		if ((ResultPoint0.z - APoint.z) * Normal.z > 0
			|| (ResultPoint0.x - APoint.x) * Normal.x > 0 || (ResultPoint0.y - APoint.y) * Normal.y > 0)
			return ResultPoint1;
		else
			return ResultPoint0;
	}
}

//根据直线的方向向量，获取距离PointA 点距离 为Dis的两个点
template<typename PointNT>
static void GetPointAlongLine(PointNT PointA, double a, double b,
	double c, double Dis, PointNT & OnePoint, PointNT & AnotherPoint)
{
	double t = sqrt(Dis*Dis/(a*a+b*b+c*c));
	
	OnePoint.x = PointA.x + a*t;
	OnePoint.y = PointA.y + b*t;
	OnePoint.z = PointA.z + c*t;

	AnotherPoint.x = PointA.x - a*t;
	AnotherPoint.y = PointA.y - b*t;
	AnotherPoint.z = PointA.z - c*t;
}

//返回角 点（X1,Y1）,点（X3,Y3）,点（X2,Y2）角度的Cos值
static long double CosOfThreePoints(double X1, double Y1, double X3, double Y3, double X2, double Y2)
{
	return ((X1 - X3) * (X2 - X3) + (Y1 - Y3) * (Y2 - Y3))/sqrt(pow(X1 - X3, 2) 
		+ pow(Y1 - Y3, 2))/sqrt(pow(X2 - X3, 2) + pow(Y2 - Y3, 2)); 
}

//返回角 点（X1,Y1,Z1）,点（X3,Y3,Z3）,点（X2,Y2,Z2）角度的Cos值
static long double CosOfThreePoints(double X1, double Y1, double Z1, 
	double X3, double Y3,  double Z3, 
	double X2, double Y2,  double Z2)
{	
	return ((X1 - X3) * (X2 - X3) + (Y1 - Y3) * (Y2 - Y3) + (Z1 - Z3) * (Z2 - Z3))
		/sqrt(pow(X1 - X3, 2) + pow(Y1 - Y3, 2) + pow(Z1 - Z3, 2))
		/sqrt(pow(X2 - X3, 2) + pow(Y2 - Y3, 2) + pow(Z2 - Z3, 2)); 
}

//返回两个向量间的夹角的弧度值
template<typename PointNT>
static long double AngleValueOfThreePoints(PointNT A, PointNT Center, PointNT B)
{
	return acos(CosOfThreePoints(A.x, A.y, A.z, Center.x, Center.y, Center.z, B.x, B.y, B.z));
}

//返回两个向量间的夹角的弧度值
static long double AngleValueOfThreePoints(double X1, double Y1, double X3, double Y3, double X2, double Y2)
{
	return acos(CosOfThreePoints(X1,Y1,X3,Y3,X2,Y2));
}

//获取点集中点间的平均距离 IsClosed 是首尾闭合封闭的距离
static double GetAvgDisOfPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, bool IsClosed = true)
{
	double Dis = 0;
	for(int i = 0; i < Points->points.size() - 1; i++)
	{
		Dis = Dis + PointDis(Points->points[i], Points->points[i+1]);
	}

	if (IsClosed)
	{
		Dis = Dis + PointDis(Points->points[0], Points->points[Points->points.size() - 1]);
		return Dis /(Points->points.size()+1);
	}
	else
		return Dis /(Points->points.size());
}

////使用最小二乘法拟合平面， z=ax+by+c 或者 x=az+by+c 或者 y=ax+bz+c
////方程因变量为DependentVariant 默认是z 2015.12.28
//返回 ax+by+cz+d = 0的平面方程	2015.12.28
static void FittingPlaneByLeastSquare(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, 
	double & a, double & b,  double & c, double & d, string DependentVariant = "z")
{
	double SumX = 0, SumY = 0, SumZ = 0, SumXY = 0, SumXZ = 0, SumYZ = 0, SumXX = 0, SumYY = 0, SumZZ = 0;
	
	for(int i = 0; i < Points->points.size(); i++)
	{
		SumX = SumX + Points->points[i].x;
		SumY = SumY + Points->points[i].y;
		SumZ = SumZ + Points->points[i].z;

		SumXY = SumXY + Points->points[i].x * Points->points[i].y;
		SumXZ = SumXZ + Points->points[i].x * Points->points[i].z;
		SumYZ = SumYZ + Points->points[i].y * Points->points[i].z;

		SumXX = SumXX + Points->points[i].x * Points->points[i].x;
		SumYY = SumYY + Points->points[i].y * Points->points[i].y;
		SumZZ = SumZZ + Points->points[i].z * Points->points[i].z;
	}
	int n = Points->points.size();

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3); //默认是0矩阵
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 1);	// 默认是0矩阵

	//求解矩阵AX=B
	if (DependentVariant == "z")//z=ax+by+c
	{
		A(0, 0)=SumXX;	A(0, 1)=SumXY;	A(0, 2)=SumX;	B(0,0)=SumXZ;
		A(1, 0)=SumXY;	A(1, 1)=SumYY;	A(1, 2)=SumY;	B(1,0)=SumYZ;
		A(2, 0)=SumX;	A(2, 1)=SumY;	A(2, 2)=n;		B(2,0)=SumZ;
		Eigen::MatrixXd X = Math::ResloveLinearEquationsByLUDecomposition(A, B);
		a = X(0,0);	b = X(1,0);	c = -1; d = X(2,0);
	}
	else if (DependentVariant == "x")//x=az+by+c	//是第一种情况的 x 与 z 互换
	{
		A(0, 0)=SumZZ;	A(0, 1)=SumYZ;	A(0, 2)=SumZ;	B(0,0)=SumXZ;
		A(1, 0)=SumYZ;	A(1, 1)=SumYY;	A(1, 2)=SumY;	B(1,0)=SumXY;
		A(2, 0)=SumZ;	A(2, 1)=SumY;	A(2, 2)=n;		B(2,0)=SumX;	
		Eigen::MatrixXd X = Math::ResloveLinearEquationsByLUDecomposition(A, B);
		a = 1;	b = X(1,0);	c = X(0,0); d = X(2,0);
	}
	else if (DependentVariant == "y")//y=ax+bz+c	//是第一种情况的 y 与 z 互换
	{
		A(0, 0)=SumXX;	A(0, 1)=SumXZ;	A(0, 2)=SumX;	B(0,0)=SumXY;
		A(1, 0)=SumXZ;	A(1, 1)=SumZZ;	A(1, 2)=SumZ;	B(1,0)=SumYZ;
		A(2, 0)=SumX;	A(2, 1)=SumZ;	A(2, 2)=n;		B(2,0)=SumY;		
		Eigen::MatrixXd X = Math::ResloveLinearEquationsByLUDecomposition(A, B);
		a =  X(0,0);	b = 1;	c =X(1,0); d = X(2,0);
	}
}

//获取空间直线的方程通过使用最小二乘法拟合两个平面，平面的交线即是直线方程
//返回直线的方向向量 与 直线上的一点	2015.12.28
//此种方法的精确较 GetMaxDirectionVector 差很多 2015.12.29 不建议使用
static pcl::PointXYZRGB FittingLineByCrossPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, 
	double & a, double & b,  double & c, bool IsLinePoint = false)
{
	double a1, b1, c1, d1;
	double a2, b2, c2, d2;
	FittingPlaneByLeastSquare(Points, a1, b1, c1, d1, "x");
	FittingPlaneByLeastSquare(Points, a2, b2, c2, d2, "y");
	
	pcl::PointXYZRGB PlaneOne, PlaneTwo;
	PlaneOne.x = a1;
	PlaneOne.y = b1;
	PlaneOne.z = c1;

	PlaneTwo.x = a2;
	PlaneTwo.y = b2;
	PlaneTwo.z = c2;

	//直线的方向向量
	pcl::PointXYZRGB CrossPoint = PointBase::PointsCrossProduct(PlaneOne, PlaneTwo);
	a = CrossPoint.x;	b = CrossPoint.y;	c = CrossPoint.z;
	
	pcl::PointXYZRGB ResultPoint;

	//直线上的一点
	if (IsLinePoint)	
	{
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);	//默认是0矩阵
		Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2, 1);	//默认是0矩阵

		A(0,0) = b1; A(0,1) = c1; B(0,0) = d1;
		A(1,0) = b2; A(1,1) = c2; B(1,0) = d2;

		Eigen::MatrixXd X = Math::ResloveLinearEquationsByLUDecomposition(A, B);
		ResultPoint.x = 0; ResultPoint.y = X(0,0);	ResultPoint.z = X(1,0);
	};

	return ResultPoint;
}

static int PointInLinesegment(pcl::PointXYZRGB  LinesegmentA,
	pcl::PointXYZRGB  LinesegmentB, pcl::PointXYZRGB  Point)
{
	float a1,b1,c1,a2,b2,c2;
	a1 = Point.x - LinesegmentA.x;
	b1 = Point.y - LinesegmentA.y;
	c1 = Point.z - LinesegmentA.z;

	a2 = Point.x - LinesegmentB.x;
	b2 = Point.y - LinesegmentB.y;
	c2 = Point.z - LinesegmentB.z;
	if (LinesegmentA.x == Point.x && LinesegmentA.y == Point.y && LinesegmentA.z == Point.z)
		return 1;
	else if (LinesegmentB.x == Point.x && LinesegmentB.y == Point.y && LinesegmentB.z == Point.z)
		return 1;
	//else if (a1*a2 <= 0 || b1*b2 <= 0 || c1*c2 <= 0)
	else if (a1*a2 <= 0 && b1*b2 <= 0)
		return 1;	// Point 在 LinesegmentA， LinesegmentB 内	
	else
		return 0;	// Point 在 LinesegmentA， LinesegmentB 外		
}
//2016.01.15 直线与三角形是否有交点，如果有则返回 交点 
////HaveCross = 0 平行且不在面上 1，相交，2，直线在面上,×××××× 
///HaveCross = 10 与三角形的一条边相交，且交点落在线段LineA-LineB内
///HaveCross = 11 与三角形的一条边相交，且交点落在线段LineA-LineB外
///HaveCross = 12 与三角形相交在三角形内，且交点落在线段LineA-LineB内
///HaveCross = 13 与三角形相交在三角形内，且交点落在线段LineA-LineB外
///HaveCross = 14 与三角形相交在三角形外,
static pcl::PointXYZRGB LineCrossTiangle(pcl::PointXYZRGB TriangleA, 
	pcl::PointXYZRGB TriangleB, pcl::PointXYZRGB TriangleC,
	pcl::PointXYZRGB LineA, pcl::PointXYZRGB LineB, int & HaveCross)
{
	pcl::PointXYZRGB CrossPoint;

	//三点确定一个平面方程
	double a,b,c,d;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TrianglePoints (new pcl::PointCloud<pcl::PointXYZRGB>);
	TrianglePoints->points.push_back(TriangleA);
	TrianglePoints->points.push_back(TriangleB);
	TrianglePoints->points.push_back(TriangleC);
	GeometryBase::GetPointsTangentPlane(TrianglePoints,a,b,c);
	d = -(TriangleA.x * a + TriangleA.y * b + TriangleA.z * c);	

	if (-(LineA.x * a + LineA.y * b + LineA.z * c) == d 
		&& -(LineB.x * a + LineB.y * b + LineB.z * c) == d)
	{
		HaveCross = 2; //直线在面上，交点无穷多
		return CrossPoint;
	}

	double Linea,Lineb,Linec;
	
	Linea = LineA.x - LineB.x;
	Lineb = LineA.y - LineB.y;
	Linec = LineA.z - LineB.z;
	
	pcl::PointXYZRGB PlaneNormal;
	pcl::PointXYZRGB LineDirection;
	PlaneNormal.x = a; PlaneNormal.y = b; PlaneNormal.z = c;
	LineDirection.x = Linea; LineDirection.y = Lineb; LineDirection.z = Linec;

	// 直线与平面的法线 垂直，也即直线平行于平面 无交点
	if(PointBase::PointsDotProduct(LineDirection, PlaneNormal) == 0)	
	{
		HaveCross = 0;
		return CrossPoint;
	}

	//剩下的即是一个交点的问题 但要考虑是否在三角形内部的问题  上一步已经保证 分母不会为0
	double Lint = (-d-(a*LineA.x + b*LineA.y + c*LineA.z))/(a*Linea + b*Lineb + c*Linec);
	CrossPoint.x = LineA.x + Lint*Linea;
	CrossPoint.y = LineA.y + Lint*Lineb;
	CrossPoint.z = LineA.z + Lint*Linec;

	int Position = PointIsInTriange(TriangleA, TriangleB, TriangleC, CrossPoint);//=0 是在边上，
	if (Position == 0)	//=0 是在边上，
	{			
		if (PointInLinesegment(LineA, LineB, CrossPoint) == 1)
			HaveCross = 10;	//与三角形的一条边相交，且交点落在线段LineA-LineB内
		else
			HaveCross = 11;	//与三角形的一条边相交，且交点落在线段LineA-LineB外
	}
	else if(Position == 1)	// Position=1在内部 			
	{
		if (PointInLinesegment(LineA, LineB, CrossPoint) == 1)
			HaveCross = 12;	//与三角形相交在三角形内，且交点落在线段LineA-LineB内
		else
			HaveCross = 13;	//与三角形相交在三角形内，且交点落在线段LineA-LineB外		
	}
	else HaveCross = 14;	//Position=-1是在外部  与三角形相交在三角形外,
	
	return CrossPoint;
}

static void PointsProjectToLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints,
	pcl::PointXYZRGB LinePointA, pcl::PointXYZRGB LinePointB)
{
	OutPoints->points.clear();
	for (int i = 0; i < InPoints->points.size(); i++)
	{
		pcl::PointXYZRGB TempPoint = PointProjectToLine(InPoints->points[i], LinePointA, LinePointB);
		OutPoints->points.push_back(TempPoint);
	}
}

//计算点在直线(LinePointA至LinePointB)上的投影点 2020.05.30
static pcl::PointXYZRGB PointProjectToLine(pcl::PointXYZRGB Point,
	pcl::PointXYZRGB LinePointA, pcl::PointXYZRGB LinePointB)
{
	pcl::PointXYZRGB LineDirection;

	LineDirection.x = LinePointA.x - LinePointB.x;
	LineDirection.y = LinePointA.y - LinePointB.y;
	LineDirection.z = LinePointA.z - LinePointB.z;

	return LineCrossPlane(LineDirection.x, LineDirection.y, LineDirection.z, LinePointA,
		LineDirection.x, LineDirection.y, LineDirection.z,
		-(LineDirection.x * Point.x + LineDirection.y * Point.y + LineDirection.z * Point.z));
}

//计算三维空间中直线与平面的交点
static pcl::PointXYZRGB LineCrossPlane(float LineA, float LineB, float LineC, pcl::PointXYZRGB LinePoint, 
	float PlaneA, float PlaneB, float PlaneC, float PlaneD)
{
	pcl::PointXYZRGB CrossPoint;

	float t;

	t = (-PlaneD - PlaneA * LinePoint.x - PlaneB * LinePoint.y - PlaneC * LinePoint.z)/(PlaneA * LineA + PlaneB * LineB + PlaneC * LineC);

	CrossPoint.x = LineA * t + LinePoint.x;
	CrossPoint.y = LineB * t + LinePoint.y;
	CrossPoint.z = LineC * t + LinePoint.z;

	return CrossPoint;
}

///2016.01.15 判断一个点是否在三角形的内部 或者在边上 
//=0 是在边上，=-1是在外部  =1在内部 包括 三维空间中 点A在三角形所在的平面的投影是否会落在三角形中
static int PointIsInTriange(pcl::PointXYZRGB TriangleA, 
	pcl::PointXYZRGB TriangleB, 
	pcl::PointXYZRGB TriangleC,
	pcl::PointXYZRGB A)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Original (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Out (new pcl::PointCloud<pcl::PointXYZRGB>);

	Original->points.push_back(TriangleA);
	Original->points.push_back(TriangleB);
	Original->points.push_back(TriangleC);	
	pcl::PointXYZRGB PointNormal;

	GeometryBase::GetPointsTangentPlane(Original, PointNormal);

	Original->points.push_back(A);	//因A可能与前面不在一个平面，旋转后好处理
	GeometryBase::RotateNormalToVertical(Original, Out, PointNormal);

	TriangleA = Out->points[0];
	TriangleB = Out->points[1];
	TriangleC = Out->points[2];
	A = Out->points[3];

	double Area1 = GeometryBase::AreaOfThreePointsIn2D(A, TriangleA, TriangleB);
	double Area2 = GeometryBase::AreaOfThreePointsIn2D(A, TriangleB, TriangleC);
	double Area3 = GeometryBase::AreaOfThreePointsIn2D(A, TriangleC, TriangleA);
	if (Area1 == 0 || Area2 == 0 ||Area3 == 0)
		return 0;
	else if (Area1 * Area2 < 0 ||Area1 * Area3 < 0 ||Area2 * Area3 < 0)
		return -1;
	else
		return 1;
}

///2016.01.16 判断点C是否在点AB的正下方 暂时未用
static bool PointCIsDownInLineOFAB(pcl::PointXYZRGB A, 
	pcl::PointXYZRGB B, 
	pcl::PointXYZRGB C, bool IsUpper = true)
{		
	pcl::PointXYZRGB D, Other;
	
	if (IsUpper) //上界
	{
		if (A.z > B.z)	
			D.x = B.x, D.y = B.y, D.z = A.z, Other = A;	
		else	
			D.x = A.x, D.y = A.y, D.z = B.z, Other = B;		
	}
	else	//下界
	{
		if (A.z > B.z)	
			D.x = A.x, D.y = A.y, D.z = B.z, Other = B;	
		else	
			D.x = B.x, D.y = B.y, D.z = A.z, Other = A;	
	}

	int Position = PointIsInTriange(A, B, D, C);
	if (Position == 1 || Position == 0)
	{		
		return true;
	}
	else
	{
		//计算 点Other,D,C 的夹角
		double Angle1 = GeometryBase::AngleOfTwoVector(
			Other.x - D.x, Other.y - D.y, Other.z - D.z,
			C.x - D.x, C.y - D.y, C.z - D.z);

		//计算 点C,Other,D 的夹角
		double Angle2 = GeometryBase::AngleOfTwoVector(
			C.x - Other.x, C.y - Other.y, C.z - Other.z,
			D.x - Other.x, D.y - Other.y, D.z - Other.z);

		if (Angle1 < M_PI/2 && Angle2 < M_PI/2)
			return true;
		else 
			return false;
	}
}

//2016.01.18 
static double GetConvexHullArea(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexHullPoints)
{
	double Area = 0;
	for(int i = 0; i < ConvexHullPoints->points.size(); i++)
	{
		pcl::PointXYZRGB Current, Next;
		Current = ConvexHullPoints->points[i];
		Next = ConvexHullPoints->points[(i+1)%ConvexHullPoints->points.size()];
		Area = Area + Current.x * Next.y - Current.y * Next.x; 
	}
	return Area/2;
}

//2020.02.14 获取XY空间上的凸包多边形的质心
static pcl::PointXYZRGB GetConvexHullCentroidOfPointsInXY(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexHullPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	GetConvexHullOfPoints(Points, ConvexHullPoints);
	return GetConvexHullCentroid(ConvexHullPoints);
}

//2016.01.18 
static pcl::PointXYZRGB GetConvexHullCentroid(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexHullPoints)
{
	pcl::PointXYZRGB Centroid;
	Centroid.x = 0; Centroid.y = 0;
	Centroid.z = ConvexHullPoints->points[0].z;

	//pcl::PointXYZRGB Centroid1, Centroid2;
	//Centroid1.x = 0; Centroid1.y = 0;

	double Area = GetConvexHullArea(ConvexHullPoints);

	if (Area == 0)	//如果Area等于0的处理 就当中心为质心
	{
		for(int i = 0; i < ConvexHullPoints->points.size(); i++)
		{
			Centroid.x = Centroid.x + ConvexHullPoints->points[i].x;
			Centroid.y = Centroid.y + ConvexHullPoints->points[i].y;
		}
		Centroid.x = Centroid.x / ConvexHullPoints->points.size();
		Centroid.y = Centroid.y / ConvexHullPoints->points.size();

		return Centroid;
	}
	
	double X1, X2, Y1, Y2, TempX1 = 0, TempY1 = 0;
	for(int i = 0; i < ConvexHullPoints->points.size(); i++)
	{
		pcl::PointXYZRGB Current, Next;
		Current = ConvexHullPoints->points[i];
		Next = ConvexHullPoints->points[(i+1)%ConvexHullPoints->points.size()];		
		
		X1 = Current.x, Y1 = Current.y; X2 = Next.x, Y2 = Next.y;

		TempX1 = TempX1 + 2*((X1+X2)*(X1*Y2-X2*Y1));
		TempY1 = TempY1 + 2*((Y1+Y2)*(X1*Y2-X2*Y1));

		//TempX2 = TempX2 + 2*(pow(X1, 2)*Y2 + X1*X2*Y2
		//	-X1*X2*Y1-pow(X2, 2)*Y1)
		//	- pow(X2, 2)*Y2 + pow(X1, 2)*Y1;

		//TempY2 = TempY2 - 2*(pow(Y1, 2)*X2 + Y1*Y2*X2
		//	-Y1*Y2*X1-pow(Y2, 2)*X1)
		//	+ pow(Y2, 2)*X2 - pow(Y1, 2)*X1;

		//TempX3 = TempX3 + 2*(Y2 - Y1)*(X1 * X1 +X1*X2+X2*X2);

		//TempY3 = TempY3 - 2*(X2 - X1)*(Y1 * Y1 +Y1*Y2+Y2*Y2);
	}
	Centroid.x = TempX1/Area/12.0;
	Centroid.y = TempY1/Area/12.0;

	//Centroid2.x = TempX2/Area/12.0;
	//Centroid2.y = TempY2/Area/12.0;

	//Centroid1.x = TempX3/Area/12.0;
	//Centroid1.y = TempY3/Area/12.0;

	return Centroid;
}

//2016.01.18 计算点集的质心
static pcl::PointXYZRGB GetCentroidOfPoints(
	pcl::PointCloud<PointXYZRGBIndex>::Ptr IndexPoints)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
		Points (new pcl::PointCloud<pcl::PointXYZRGB>);		
	PointBase::PointXYZRGBIndexToPointXYZRGB(IndexPoints, Points);
	return GetCentroidOfPoints(Points);
}

//2016.01.18 
static pcl::PointXYZRGB GetCentroidOfPointsInSpaces(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints)
{
	pcl::PointXYZRGB NormalPoint;
	GetPointsTangentPlane(InPoints, NormalPoint);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints (new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints (new pcl::PointCloud<pcl::PointXYZRGB>);	

	RotateNormalToVertical(InPoints, PlanePoints, NormalPoint);
	
	PlanePoints->points.clear();
	PlanePoints->points.push_back(GetCentroidOfPoints(PlanePoints));
	RotateToOriginal(PlanePoints, OutPoints, NormalPoint);
	return OutPoints->points[OutPoints->points.size() - 1];
}

//static pcl::PointXYZRGB GetCentroidOfPointsInSpaces(
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints, pcl::PointXYZRGB NormalPoint)
//{
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//	RotateNormalToVertical(InPoints, PlanePoints, NormalPoint);
//
//	PlanePoints->points.clear();
//	PlanePoints->points.push_back(GetCentroidOfPoints(PlanePoints));
//	RotateToOriginal(PlanePoints, OutPoints, NormalPoint);
//	return OutPoints->points[0];
//}

static pcl::PointXYZRGB GetGravityOfPoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points)
{
	pcl::PointXYZRGB GravityPoint;
	GravityPoint.x = 0, GravityPoint.y = 0, GravityPoint.z = 0;
	for (int i = 0; i < Points->points.size(); i++)
	{
		GravityPoint.x = GravityPoint.x + Points->points[i].x / Points->points.size();
		GravityPoint.y = GravityPoint.y + Points->points[i].y / Points->points.size();
		GravityPoint.z = GravityPoint.z + Points->points[i].z / Points->points.size();
	}
	return GravityPoint;
}

//2016.01.18 计算点集的质心
static pcl::PointXYZRGB GetCentroidOfPoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points)
{
	CContourAndConvexHull<pcl::PointXYZRGB> ContourAndConvexHull;	
	ContourAndConvexHull.SetInputs(Points);

	vector<int> HullIndexs;
	ContourAndConvexHull.GetPointsConvexHull(HullIndexs);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
		HullPoints (new pcl::PointCloud<pcl::PointXYZRGB>);	

	for(int i = 0; i < HullIndexs.size(); i++)
	{
		HullPoints->points.push_back(Points->points[HullIndexs[i]]);
	}

	return GeometryBase::GetConvexHullCentroid(HullPoints);	
}

//获取点集 Points 的凸包点 HullPoints 并返回凸包点集构成凸包多边形的质心点
//2016.09.24 
static pcl::PointXYZRGB GetConvexHullOfPoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr HullPoints)
{
	CContourAndConvexHull<pcl::PointXYZRGB> ContourAndConvexHull;	
	ContourAndConvexHull.SetInputs(Points);

	vector<int> HullIndexs;
	ContourAndConvexHull.GetPointsConvexHull(HullIndexs);

	HullPoints->points.clear();
	for(int i = 0; i < HullIndexs.size(); i++)
	{
		HullPoints->points.push_back(Points->points[HullIndexs[i]]);
	}
	return GeometryBase::GetConvexHullCentroid(HullPoints);	
}


//获取距离特定点最近的点 2016.01.31
static int GetNearestPointFromPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointXYZRGB Point, bool Is2D = false)
{	
	int ReturnIndex = 0; 
	double SmallRadius = PointDis(Points->points[0], Point, Is2D);
	double Radius = SmallRadius;
	for(int i = 1; i < Points->points.size(); i++)
	{
		SmallRadius = PointDis(Points->points[i], Point, Is2D);
		if (Radius > SmallRadius && SmallRadius > 0)
		{
			Radius = SmallRadius;
			ReturnIndex = i;
		}
	}	
	return ReturnIndex;
}

static pcl::PointXYZRGB GetAvgRadiusOfPoints(
	pcl::PointCloud<PointXYZRGBIndex>::Ptr IndexPoints, double & Radius)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
		Points (new pcl::PointCloud<pcl::PointXYZRGB>);		
	PointBase::PointXYZRGBIndexToPointXYZRGB(IndexPoints, Points);
	return GetAvgRadiusOfPoints(Points, Radius);	
}

static pcl::PointXYZRGB GetAvgRadiusOfPoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, double & Radius)
{
	pcl::PointXYZRGB Centroid = GetCentroidOfPoints(Points);
	Radius = 0;
	for(int i = 0; i < Points->points.size(); i++)
	{
		Radius = Radius + PointDis(Centroid, Points->points[i], true);
	}
	Radius = Radius / Points->points.size();
	return Centroid;
}

//2020.05.20  该方法要求点集是逆时针或顺时针次序排列
static double GetAreaPolygon(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PolygonPoints)
{
	double Area = 0;

	int n = PolygonPoints->points.size();
	for(int i = 0; i < PolygonPoints->points.size(); i++)
	{
		Area = 	Area + 
			PolygonPoints->points[i].x * PolygonPoints->points[(i+1)%n].y -   
			PolygonPoints->points[i].y * PolygonPoints->points[(i+1)%n].x;
	}

	return abs(Area/2);
}

//获取以CenterPoint为圆心，以Radius为半径, 以AngleSpace为角度间隔的画圆的点云
static void GetCirclePoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CriclePoints,
	pcl::PointXYZRGB CenterPoint, float Radius,
	bool DoubleCircle = false, float AngleSpace = 1,
	bool HaveGaussianNoise = false, float NoiseMean = 0, float NoiseStddev = 0)	//have the gaussiannoise,
{
	CriclePoints->points.clear();
	float Angle = 0, Radian;
	while (Angle < 360)
	{
		Radian = M_PI * Angle / 180;
		pcl::PointXYZRGB Point;
		Point.z = CenterPoint.z;
		Point.x = CenterPoint.x + Radius * cos(Radian);
		Point.y = CenterPoint.y + Radius * sin(Radian);
		CriclePoints->points.push_back(Point);

		if (DoubleCircle)
		{
			pcl::PointXYZRGB DoublePoint;
			DoublePoint.z = CenterPoint.z;
			DoublePoint.x = CenterPoint.x + Radius * 0.8 * cos(Radian);
			DoublePoint.y = CenterPoint.y + Radius * 0.8 * sin(Radian);
			CriclePoints->points.push_back(DoublePoint);
		}

		Angle = Angle + AngleSpace;
	}
	PointBase::SetPointColor(CriclePoints, ColorBase::StemColor);
	
	if (HaveGaussianNoise)
	{
		static std::default_random_engine generator(time(0));
		
		//static std::normal_distribution<double> distx(NoiseMean, NoiseStddev);
		static std::normal_distribution<double> disty(NoiseMean, NoiseStddev);

		for (auto& p : CriclePoints->points)
		{
			//p.x = p.x + distx(generator);
			p.y = p.y + disty(generator);
		}
	}
}

//Generate Parabola body points
static void GetParabolaBodyPoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ParabolaBodyPoints,
	pcl::PointXYZRGB CenterPoint, pcl::PointXYZRGB AxisDirection,
	float LowerRadius, float UpperRadius, double Height, float AngleSpace = 1, float HeightInterval = 1,
	bool HaveGaussianNoise = false, float NoiseMean = 0, float NoiseStddev = 0)
{
	if (LowerRadius < UpperRadius)	//exchange 
	{
		double Temp = LowerRadius;
		LowerRadius = UpperRadius;
		UpperRadius = Temp;
	}

	double p = (UpperRadius * UpperRadius - LowerRadius * LowerRadius) / 2.0 / Height;

	ParabolaBodyPoints->points.clear();

	const double StartH = LowerRadius * LowerRadius / 2.0 / p;
	//double DRate = UpperRadius / LowerRadius;
	//double HUpper = Height * DRate / (1 - DRate);

	double CurHeight = 0, CurRadius = LowerRadius;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CirclePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointXYZRGB CurrentCenterPoints;

	while (CurHeight < Height)
	{
		CurrentCenterPoints = GeometryBase::GetPointAlongLine(AxisDirection, CenterPoint, CurHeight);
		GetCirclePoints(CirclePoints, CurrentCenterPoints, CurRadius,
			false, AngleSpace, HaveGaussianNoise, NoiseMean, NoiseStddev);
		ParabolaBodyPoints->points.insert(ParabolaBodyPoints->points.end(),
			CirclePoints->points.begin(), CirclePoints->points.end());

		CurHeight = CurHeight + HeightInterval;
		CurRadius = sqrt(2 * p * (CurHeight + StartH));
	}

	//generating Points at hightest position
	CurrentCenterPoints = GeometryBase::GetPointAlongLine(AxisDirection, CenterPoint, Height);
	GetCirclePoints(CirclePoints, CurrentCenterPoints, UpperRadius,
		false, AngleSpace, HaveGaussianNoise, NoiseMean, NoiseStddev);
	ParabolaBodyPoints->points.insert(ParabolaBodyPoints->points.end(),
		CirclePoints->points.begin(), CirclePoints->points.end());
}

static void GetConePoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConePoints,
	pcl::PointXYZRGB CenterPoint, pcl::PointXYZRGB AxisDirection,
	float LowerRadius, float UpperRadius, double Height, 
	float AngleSpace = 1, float HeightInterval = 1,
	bool HaveGaussianNoise = false, float NoiseMean = 0, float NoiseStddev = 0)	//have the gaussiannoise,
{
	if (LowerRadius < UpperRadius)	//exchange 
	{
		double Temp = LowerRadius;
		LowerRadius = UpperRadius;
		UpperRadius = Temp;
	}

	ConePoints->points.clear();

	double DRate = UpperRadius / LowerRadius;
	double HUpper = Height * DRate / (1 - DRate);	

	double CurHeight = 0, CurRadius = LowerRadius;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CirclePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointXYZRGB CurrentCenterPoints;

	while (CurHeight < Height)
	{
		CurrentCenterPoints = GeometryBase::GetPointAlongLine(AxisDirection, CenterPoint, CurHeight);
		GetCirclePoints(CirclePoints, CurrentCenterPoints, CurRadius, 
			false, AngleSpace, HaveGaussianNoise, NoiseMean, NoiseStddev);
		ConePoints->points.insert(ConePoints->points.end(),
			CirclePoints->points.begin(), CirclePoints->points.end());

		CurHeight = CurHeight + HeightInterval;
		CurRadius = (HUpper + Height - CurHeight) * LowerRadius / (HUpper + Height);
	}

	//generating Points at hightest position
	CurrentCenterPoints = GeometryBase::GetPointAlongLine(AxisDirection, CenterPoint, Height);
	GetCirclePoints(CirclePoints, CurrentCenterPoints, UpperRadius,
		false, AngleSpace, HaveGaussianNoise, NoiseMean, NoiseStddev);
	ConePoints->points.insert(ConePoints->points.end(),
		CirclePoints->points.begin(), CirclePoints->points.end());
}

static void GetCylinderPoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CylinderPoints,
	pcl::PointXYZRGB CenterPoint, pcl::PointXYZRGB AxisDirection, 
	float Radius, double Height, float AngleSpace = 1, float HeightInterval = 1,
	bool HaveGaussianNoise = false, float NoiseMean = 0, float NoiseStddev = 0)	//have the gaussiannoise,
{
	CylinderPoints->points.clear();

	double CurHeight = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CirclePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointXYZRGB CurrentCenterPoints;

	while(CurHeight < Height)
	{
		CurrentCenterPoints = GeometryBase::GetPointAlongLine(AxisDirection, CenterPoint, CurHeight);	
		CurHeight = CurHeight + HeightInterval;

		GetCirclePoints(CirclePoints, CurrentCenterPoints, Radius, false, AngleSpace,
			HaveGaussianNoise, NoiseMean, NoiseStddev);
		//GetCirclePoints(CirclePoints, CurrentCenterPoints, Radius, false, AngleSpace);
		CylinderPoints->points.insert(CylinderPoints->points.end(),
			CirclePoints->points.begin(), CirclePoints->points.end());
	}

	//generating Points at Hight 
	CurrentCenterPoints = GeometryBase::GetPointAlongLine(AxisDirection, CenterPoint, Height);
	GetCirclePoints(CirclePoints, CurrentCenterPoints, Radius, false, AngleSpace,
		HaveGaussianNoise, NoiseMean, NoiseStddev);
	//GetCirclePoints(CirclePoints, CurrentCenterPoints, Radius, false, AngleSpace);
	CylinderPoints->points.insert(CylinderPoints->points.end(),
		CirclePoints->points.begin(), CirclePoints->points.end());
}

//Get plane points by plane equations (or normal vector of the plane) and a given point on the plane, 
//MinSquareEdgeLength is the distance bewteen the CenterPoint to the nearest edge,
////SamplePointDis is the distance the points on a common edge.
static void GetPlanePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints,
	float a, float b, float c,
	pcl::PointXYZRGB CenterPoint, float MinSquareEdgeLength)
{
	PlanePoints->points.clear();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPlanePoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	pcl::PointXYZRGB OriginPoint;
	OriginPoint.x = 0, OriginPoint.y = 0, OriginPoint.z = 0;

	TempPlanePoints->points.push_back(OriginPoint);
	for (int i = 0; i < 4; i++)
	{
		//The four point in each quadrant.
		pcl::PointXYZRGB FirstPoint, SecondPoint, ThirdPoint, FourthPoint;

		FirstPoint.x = MinSquareEdgeLength * (i + 1) * sqrt(2);
		FirstPoint.y = MinSquareEdgeLength * (i + 1) * sqrt(2);
		FirstPoint.z = 0;
		TempPlanePoints->points.push_back(FirstPoint);

		SecondPoint.x = -1.0 * MinSquareEdgeLength * (i + 1) * sqrt(2);
		SecondPoint.y = MinSquareEdgeLength * (i + 1) * sqrt(2);
		SecondPoint.z = 0;
		TempPlanePoints->points.push_back(SecondPoint);

		ThirdPoint.x = -1.0 * MinSquareEdgeLength * (i + 1) * sqrt(2);
		ThirdPoint.y = -1.0 * MinSquareEdgeLength * (i + 1) * sqrt(2);
		ThirdPoint.z = 0;
		TempPlanePoints->points.push_back(ThirdPoint);

		FourthPoint.x = MinSquareEdgeLength * (i + 1) * sqrt(2);
		FourthPoint.y = -1.0 * MinSquareEdgeLength * (i + 1) * sqrt(2);
		FourthPoint.z = 0;
		TempPlanePoints->points.push_back(FourthPoint);

		double SampleDistance = PointDis(FirstPoint, SecondPoint) / 10.0;

		for (int j = 0; j < 9; j++)
		{
			pcl::PointXYZRGB TempFirstPoint(FirstPoint);
			TempFirstPoint.x = FirstPoint.x - SampleDistance * (j + 1);
			TempPlanePoints->points.push_back(TempFirstPoint);

			pcl::PointXYZRGB TempSecondPoint(FirstPoint);
			TempSecondPoint.y = FirstPoint.y - SampleDistance * (j + 1);
			TempPlanePoints->points.push_back(TempSecondPoint);

			pcl::PointXYZRGB TempThirdPoint(ThirdPoint);
			TempThirdPoint.x = ThirdPoint.x + SampleDistance * (j + 1);
			TempPlanePoints->points.push_back(TempThirdPoint);

			pcl::PointXYZRGB TempFourthPoint(ThirdPoint);
			TempFourthPoint.y = ThirdPoint.y + SampleDistance * (j + 1);
			TempPlanePoints->points.push_back(TempFourthPoint);
		}
	}

	pcl::PointXYZRGB NormalPoint;
	NormalPoint.x = a, NormalPoint.y = b, NormalPoint.z = c;
	GeometryBase::RotateToOriginal(TempPlanePoints, PlanePoints, NormalPoint);
	//Should be move after Rotating
	PointsMove(PlanePoints, CenterPoint.x - PlanePoints->points[0].x,
		CenterPoint.y - PlanePoints->points[0].y,
		CenterPoint.z - PlanePoints->points[0].z);
}

//2018.12.27 Get Arrow points , 
static void GetArrowPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ArrowPoints,
	float a, float b, float c,
	pcl::PointXYZRGB BasePoint, float ArrowLength, bool isUp = true)
{
	ArrowPoints->points.clear();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempArrowPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PointXYZRGB OriginPoint;
	OriginPoint.x = 0, OriginPoint.y = 0, OriginPoint.z = 0;

	TempArrowPoints->points.push_back(OriginPoint);
	
	//Generate points of arrow line. 100 points
	for (int i = 0; i < 100; i++)
	{
		pcl::PointXYZRGB TempPoint;
		TempPoint.x = 0, TempPoint.y = 0;
		if (isUp)
			TempPoint.z = ArrowLength * (i + 1) / 100.0;
		else
			TempPoint.z = -1 * ArrowLength * (i + 1) / 100.0;
		TempArrowPoints->points.push_back(TempPoint);
	}

	int N = 10;
	//Generate points of arrow hat.
	double ArrowHatLength = ArrowLength * 0.25 / N;

	for (int i = 0; i < N; i++)	//N slices
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempArrowHatPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointXYZRGB CircleCenterPoint;
		CircleCenterPoint.x = 0, CircleCenterPoint.y = 0;
		if (isUp)
		{
			CircleCenterPoint.z = ArrowLength - ((i + 1)*ArrowHatLength);
			GetCirclePoints(TempArrowHatPoints, CircleCenterPoint, (ArrowLength - CircleCenterPoint.z) * tan(M_PI / 6));
		}
		else
		{
			CircleCenterPoint.z = -1 * (ArrowLength - ((i + 1)*ArrowHatLength));
			GetCirclePoints(TempArrowHatPoints, CircleCenterPoint, (ArrowLength + CircleCenterPoint.z) * tan(M_PI / 6));
		}
		
		TempArrowPoints->points.insert(TempArrowPoints->points.end(), TempArrowHatPoints->points.begin(),
			TempArrowHatPoints->points.end());
	}

	PointsMove(TempArrowPoints, BasePoint.x, BasePoint.y, BasePoint.z);
	pcl::PointXYZRGB NormalPoint;
	NormalPoint.x = a, NormalPoint.y = b, NormalPoint.z = c;
	GeometryBase::RotateToOriginal(TempArrowPoints, ArrowPoints, NormalPoint);
	//Should be move after Rotating
	PointsMove(ArrowPoints, BasePoint.x - ArrowPoints->points[0].x,
		BasePoint.y - ArrowPoints->points[0].y,
		BasePoint.z - ArrowPoints->points[0].z);
}

//2016.04.16 根据 InversionP 点对 InPoints做反演变换
static void InversionPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints, pcl::PointXYZRGB InversionP, 
	double InversionR) //GetPointAlongLine
{
	OutPoints->points.clear();
	double Dis = 0;
	double TempDis;
	for(int i = 0; i < InPoints->points.size(); i++)
	{
		pcl::PointXYZRGB CurrentPoint, One, LineNormal;
		
		CurrentPoint = InPoints->points[i];

		TempDis = PointDis(CurrentPoint, InversionP);
		Dis = InversionR / TempDis;

		LineNormal.x = CurrentPoint.x - InversionP.x;
		LineNormal.y = CurrentPoint.y - InversionP.y;
		LineNormal.z = CurrentPoint.z - InversionP.z;

		One = GeometryBase::GetPointAlongLine(LineNormal, InversionP, Dis);
		One.rgba = CurrentPoint.rgba;
		OutPoints->points.push_back(One);
	}
}

static pcl::PointXYZRGB GetCentroidOfPointsInSpaces(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints, pcl::PointXYZRGB NormalPoint)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints (new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints (new pcl::PointCloud<pcl::PointXYZRGB>);	

	GeometryBase::RotateNormalToVertical(InPoints, PlanePoints, NormalPoint);
	
	pcl::PointXYZRGB TempCentriod = GeometryBase::GetCentroidOfPoints(PlanePoints);	
	
	PlanePoints->points.push_back(TempCentriod);		
	GeometryBase::RotateToOriginal(PlanePoints, OutPoints, NormalPoint, true);

	return OutPoints->points[OutPoints->points.size()-1];	
}

//在三维空间中计算点集的质心点 与 凸包点集, WithDis 是放大或缩小的距离，2020.04.09
static pcl::PointXYZRGB GetCentroidAndConvexHullOfPointsInSpaces(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr HullPoints,
	pcl::PointXYZRGB NormalPoint, double WithDis = 0.0)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints (new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints (new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempHullPoints (new pcl::PointCloud<pcl::PointXYZRGB>);	

	GeometryBase::RotateNormalToVertical(InPoints, PlanePoints, NormalPoint);
	
	pcl::PointXYZRGB TempCentriod = GeometryBase::GetConvexHullOfPoints(PlanePoints, TempHullPoints);	
	
	//需要放大或者缩小
	if (abs(WithDis) > EPSM6)
	{
		for (int i = 0; i < TempHullPoints->points.size(); i++)
		{			
			pcl::PointXYZRGB TempPoint = GeometryBase::GetPointAlongAToB(TempCentriod,
				TempHullPoints->points[i], WithDis, true);
			TempHullPoints->points[i] = TempPoint;
		}
	}

	//凸包点
	GeometryBase::RotateToOriginal(TempHullPoints, HullPoints, NormalPoint, true);

	//几何中心点
	PlanePoints->points.push_back(TempCentriod);
	GeometryBase::RotateToOriginal(PlanePoints, OutPoints, NormalPoint, true);
	return OutPoints->points[OutPoints->points.size() - 1];	
}

//三点定一个平面
static void PlaneResolveByThreePoints(pcl::PointXYZRGB A, pcl::PointXYZRGB B, pcl::PointXYZRGB C,
	double & a, double & b, double & c, double & d)
{
	pcl::PointXYZRGB PlaneNormal, AB, AC;
	
	AB.x = A.x - B.x, AB.y = A.y - B.y, AB.z = A.z - B.z;
	AC.x = A.x - C.x, AC.y = A.y - C.y, AC.z = A.z - C.z;

	PlaneNormal = PointBase::PointsCrossProduct(AB, AC);
	PointBase::PointNormalized(PlaneNormal);
	a = PlaneNormal.x, b = PlaneNormal.y, c = PlaneNormal.z;

	d = -(A.x * a + A.y * b + A.z * c);
}

static void SameSideOfPlane(pcl::PointCloud<PointXYZRGBIndex>::Ptr Points, 
	pcl::PointXYZRGB MarkPoint, int InteriorValue, int ExteriorValue,
	float a, float b, float c, float d)
{
	double MarkPointSymbol = (MarkPoint.x * a + MarkPoint.y * b + MarkPoint.z * c + d);
	double Symbol;

	for(int i = 0; i < Points->points.size(); i++)
	{
		Symbol = (Points->points[i].x * a + Points->points[i].y * b + Points->points[i].z * c + d);	
		if (MarkPointSymbol * Symbol >= 0)
			Points->points[i].Category = InteriorValue;			
		else
			Points->points[i].Category = ExteriorValue;
	}
}

//return value is radius value, not diameter value
static double CircleFittingByLeastSquaresFitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points)
{
	pcl::PointXYZRGB  CircleCenter;
	return CircleFittingByLeastSquaresFitting(Points, CircleCenter);
}

//return value is radius value, not diameter value
static double CircleFittingByLeastSquaresFitting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointXYZRGB & CircleCenter)
{
	if (Points->points.size() < 3) return 0;

	double X1 = 0;
	double Y1 = 0;
	double X2 = 0;
	double Y2 = 0;
	double X3 = 0;
	double Y3 = 0;
	double X1Y1 = 0;
	double X1Y2 = 0;
	double X2Y1 = 0;

	for (int i = 0; i < Points->points.size(); i++)
	{
		X1 = X1 + Points->points[i].x;
		Y1 = Y1 + Points->points[i].y;
		X2 = X2 + Points->points[i].x * Points->points[i].x;
		Y2 = Y2 + Points->points[i].y * Points->points[i].y;
		X3 = X3 + Points->points[i].x * Points->points[i].x * Points->points[i].x;
		Y3 = Y3 + Points->points[i].y * Points->points[i].y * Points->points[i].y;
		X1Y1 = X1Y1 + Points->points[i].x * Points->points[i].y;
		X1Y2 = X1Y2 + Points->points[i].x * Points->points[i].y * Points->points[i].y;
		X2Y1 = X2Y1 + Points->points[i].x * Points->points[i].x * Points->points[i].y;
	}

	double C, D, E, G, H, N;
	double a, b, c;
	N = Points->points.size();
	C = N * X2 - X1 * X1;
	D = N * X1Y1 - X1 * Y1;
	E = N * X3 + N * X1Y2 - (X2 + Y2)*X1;
	G = N * Y2 - Y1 * Y1;
	H = N * X2Y1 + N * Y3 - (X2 + Y2)*Y1;
	a = (H*D - E * G) / (C*G - D * D);
	b = (H*C - E * D) / (D*D - G * C);
	c = -(a*X1 + b * Y1 + X2 + Y2) / N;

	CircleCenter.x = a / (-2);
	CircleCenter.y = b / (-2);
	return sqrt(a * a + b * b - 4 * c) / 2;	 //
}

};

#endif