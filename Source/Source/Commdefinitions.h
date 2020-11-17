/* 2015.06.05 
程序公共定义部分，

*/
#ifndef Commdefinitions_H
#define Commdefinitions_H

#include <sstream>
#include <string>
#include <iomanip>
#include <limits>
#include <math.h>
#include <iostream>
#include <vector>
#include <stdio.h>

#define PCL_NO_PRECOMPILE
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/octree/octree_search.h>

using namespace std;

#define EPSM9                 1.0e-9
#define EPSM6                 1.0e-6
#define EPSM3                 1.0e-3
#define EPSP3                 1.0e+3
#define EPSP6                 1.0e+6
#define EPSP9                 1.0e+9
#define BezierMiu             0.3
//2015.06.05 定义两种点云类型

typedef enum 
{
	Unknow = 0,
	TreePoint = 1,
	StemPoint = 2,
	BranchePoint = 3,
	OutlierPoint = 4,
	StemAxisCurvePoint = 5
}PointCategory;

typedef enum
{
	Red = 0,
	Green = 1,
	Blue = 2,
	White = 3,
	Black = 4
}OutliersColor;

/*
//定义用于保存分区信息的点云结构，中心点，以及分区对应的半径值
struct SectionInfoPoint
{
	PCL_ADD_POINT4D;			// 该点类型有4个元素
	PCL_ADD_RGB;                // 该点类型有rgb

	int SectionIndex;				//分区索引	4
	double rValue;					//当前分区的半径值 5
	double Variance;					//当前分区中心点得到时的方差值 6
	double BasalArea;				//当前在二维平面的表面积 7
	double V;						//当前分区体积 8
	int SectionPointsNumber;		//分区点个数 9
	double HValue;					//当前分区的高度 10
	double H1Value;					//树梢到当前分区的高度 11
	double rValueToH1Value;			//当前分区的半径值与 树梢到当前分区的高度 之比
	double rDisToUpper;				//当前分区半径与上分区半径之差
	double CenterPointDisToUpper;	//当前分区中心点与上分区中心点的距离
	double ZValueDisToUpper;			//当前分区Z值与上分区Z值之差

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW	// 确保new操作符对齐操作}
}EIGEN_ALIGN16;					// 强制SSE对齐

	POINT_CLOUD_REGISTER_POINT_STRUCT(SectionInfoPoint,// 注册点类型宏
		(float, x, x)
		(float, y, y)
		(float, z, z)
		(int, SectionIndex, SectionIndex)
		(float, rValue, rValue)
		(float, Variance, Variance)
		(float, BasalArea, BasalArea)
		(float, V, V)
		(int, SectionPointsNumber, SectionPointsNumber)
		(float, HValue, HValue)
		(float, H1Value, H1Value)
		(float, rValueToH1Value, rValueToH1Value)
		(float, rDisToUpper, rDisToUpper)
		(float, CenterPointDisToUpper, CenterPointDisToUpper)
		(float, ZValueDisToUpper, ZValueDisToUpper)

	)
*/

//2015.06.05 带有原索引类型的点云
struct PointXYZRGBIndex : pcl::PointXYZRGB		//定义带有ID的点云
{
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;                // 该点类型有4个元素

	int Index;							//Index	属于总记录集的索引	
	int Category;
	//Unknow = 0,
	//TreePoint = 1,
	//StemPoint = 2,
	//BranchePoint = 3,
	//OutlierPoint = 4,
	//StemAxisCurvePoint = 5

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW	// 确保new操作符对齐操作}
}EIGEN_ALIGN16;					// 强制SSE对齐 

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBIndex,// 注册点类型宏
(float, x, x)
(float, y, y)
(float, z, z)
(int, Index, Index)
(int, Category, Category)
)

//2018.08.03 带有色彩经纬度坐标的点云

struct PointXYZRGBWithLongiAndLati					//定义带有ID的点云
{
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;                // 该点类型有4个元素

	double X;
	double Y;
	double Z;

	double Longitude;	//经度
	double Latitude;	//纬度
	double Altitude;	//海拔

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW	// 确保new操作符对齐操作}
}EIGEN_ALIGN16;					// 强制SSE对齐 

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBWithLongiAndLati,// 注册点类型宏
(float, x, x)
(float, y, y)
(float, z, z)
(double, X, X)
(double, Y, Y)
(double, Z, Z)
(double, Longitude, Longitude)
(double, Latitude, Latitude)
(double, Altitude, Altitude)
)


//平面系数
struct PlaneCoefficient
{
	double a;
	double b;
	double c;
	double d;
	// ax+by+cz+d = 0 确定的平面
};

//按照角度对每个水平分区再次分区 在知道几何中心点的前提下
////2015.06.22 
struct AnglePartitionStruct
{
	int AngleIndex;			// the index for anlge Partition
	bool NeedRefined;			// Identifies whether the column has been repaired 
	bool Refined;			// Identifies whether the column has been repaired 
	double AngleStart;		// >= radian value，not degree value
	double AngleEnd;		// <  radian value，not degree value
	double AvgDis;			// average distantce from center to each point
	double MaxDis;
	double MinDis;
	vector<int> PointIndexs;		//the point index for a partition
	pcl::PointXYZRGB BeforeRefineCenterPointofCurPartition;	//center point for the current angle partition
	pcl::PointXYZRGB CenterPointofCurPartition;	//center point for  the current angle partition
	PlaneCoefficient TangentPlaneCoefficient;	//tagent palne parameters for  the current angle partition
};

//一个分区的多个角度下的分区数据
struct SectionAnglePartition
{
	int SectionIndex;
	double AvgDis;			// average distantce from center to each Angle  CenterPointofCurPartition
	vector<AnglePartitionStruct> AnglePartition;
};

//struct SectionTangentPlaneCoefficient	//一个分区的多个角度下的分区切平面的数据
//{
//	int SectionIndex;
//	vector<PlaneCoefficient> TangentPlaneCoefficients;
//};

struct SectionStruct	//分区点云的结构体
{
	//pcl::PointCloud<PointXYZRGBIndex>::Ptr SectionPtr;			//指向分区点云的指针
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPtr;			//指向分区点云的指针	
	vector<int> Indexs;  //the Indexs of current section 2018.12.20 modified by yl
	double ZMax;	//该区域Z的最大值	
	double ZMin;	//该区域Z的最小值	
	double xCenter;	//x坐标的中心点 质心	
	double yCenter;	//y坐标的中心点	质心	
	double zCenter;	//z坐标的中心点	质心	

	//double DValue;	//直径	
};

typedef vector<SectionStruct> SectionVector;
//指向点云数据集的指针容器

struct CrossSectionStruct	//分区点云的结构体
{
	pcl::PointCloud<PointXYZRGBIndex>::Ptr SectionPtr;		//指向分区点云的指针
	bool  IsCircle;	//此区域是否呈圆形分布
	double ZMax;	//该区域Z的最大值	
	double ZMin;	//该区域Z的最小值	
	double DValue;	//直径	

	pcl::PointXYZRGB CenterPoint;	//几何中心点

	//2015.11.01 add 只为垂直树干横断面分割而使用
	PlaneCoefficient PlaneLower;	//下部平面
	PointXYZRGBIndex PointLower;	//下部平面的一个固定点
	PlaneCoefficient PlaneUpper;	//上部平面
	PointXYZRGBIndex PointUpper;	//上部平面的一个固定点	
};

typedef vector<CrossSectionStruct> CrossSectionVector;
//指向点云数据集的指针容器

typedef struct
{
	//int Index;	//加入的次序；
	int ConstantIndex;	//已经固定的一个点
	int EdgeIndexOne;	//需要拓展的第一个节点
	int EdgeIndexTwo;	//需要拓展的第二个节点
} EdgeNodes;

typedef struct
{
	int Count;
	int IndexOne;
	int IndexTwo;
} EdgeCount;

//一个运算公式的栈
typedef vector<string> OperationItem;

//多个运算公式的多个栈
typedef vector<OperationItem> OperationItemS;

//2019.05.16 
typedef struct PointWithCurvature{		
	int index;				//the point index for a point cloud 
	float Curvature;		//the Curvature for the index-th point 
} PointWithCurvature;


typedef struct
{
	int PointIndex;
	int MaxDisNeighourIndex;			//the point index of Max distance in NeighourIndexs
	int MinDisNeighourIndex;			//the point index of Min distance in NeighourIndexs

	double AvgDis;						//点与邻域点的平均距离
	double DensityByDis;				//平均距离与最大距离之差 与  平均距离与最小距离之差，值为0 则邻域点集分布比较均匀
	double DisBetProjectAndCentriod;	//点在切平面投影点到重心点的距离
	double DisBetProjectAndOriginal;	//点在切平面投影点到点的距离
	double AvgGuassCurvatureChange;		//当前点与邻域点高斯曲率变化的平均值

	vector<int> NeighourIndexs;
	vector<float> NeighourDis;

	pcl::Normal PointNormal;
	pcl::PrincipalCurvatures PointCurvatures;
	pcl::PointXYZRGB CentriodPoint;		// the centriod point of the Nearest Points
	pcl::PointXYZRGB ProjectPoint;		// The project point of the current point project onto the plane passed through the centriod point 	

	Eigen::Vector4f ProjectPlaneCoefficients;	//The coefficients of the projection plane equation
} PointGeometryFeature;

class ColorBase
{
public:
	static const int BlackColor = ((int)0) << 16 | ((int)0) << 8 | ((int)0);  //不再数组中
	static const int WhiteColor = ((int)255) << 16 | ((int)255) << 8 | ((int)255);
	static const int GreyColor = ((int)192) << 16 | ((int)192) << 8 | ((int)192);	//灰色 不是太显眼
	static const int DarkGreyColor = ((int)128) << 16 | ((int)128) << 8 | ((int)128);

	static const int RedColor = ((int)255) << 16 | ((int)0) << 8 | ((int)0);
	static const int DarkRedColor = ((int)128) << 16 | ((int)0) << 8 | ((int)0);
	static const int GreenColor = ((int)0) << 16 | ((int)255) << 8 | ((int)0);
	static const int DarkGreenColor = ((int)0) << 16 | ((int)128) << 8 | ((int)0);

	static const int BlueColor = ((int)0) << 16 | ((int)0) << 8 | ((int)255);
	static const int DarkBlueColor = ((int)0) << 16 | ((int)0) << 8 | ((int)128);
	static const int LightBlueColor = ((int)0) << 16 | ((int)255) << 8 | ((int)255);
	static const int DoveColor = ((int)255) << 16 | ((int)0) << 8 | ((int)255);
	static const int DarkDoveColor = ((int)128) << 16 | ((int)0) << 8 | ((int)128);

	static const int PurpleColor = ((int)0) << 16 | ((int)255) << 8 | ((int)255);
	static const int DarkPurpleColor = ((int)0) << 16 | ((int)128) << 8 | ((int)128);
	static const int YellowColor = ((int)255) << 16 | ((int)255) << 8 | ((int)0);
	static const int DarkYellowColor = ((int)128) << 16 | ((int)128) << 8 | ((int)0);

	static const int PinkColor = ((int)255) << 16 | ((int)153) << 8 | ((int)255);
	static const int OrangeColor = ((int)255) << 16 | ((int)128) << 8 | ((int)0);
	static const int MagentaColor = ((int)255) << 16 | ((int)0) << 8 | ((int)255);

	static const int StemColor = ((int)79) << 16 | ((int)63) << 8 | ((int)59);

	static const int ColorIndex22 = ((int)51) << 16 | ((int)153) << 8 | ((int)255);
	static const int ColorIndex23 = ((int)153) << 16 | ((int)51) << 8 | ((int)153);
	static const int ColorIndex24 = ((int)153) << 16 | ((int)153) << 8 | ((int)51);
	static const int ColorIndex25 = ((int)163) << 16 | ((int)38) << 8 | ((int)51);
	static const int ColorIndex26 = ((int)204) << 16 | ((int)153) << 8 | ((int)102);
	static const int ColorIndex27 = ((int)204) << 16 | ((int)224) << 8 | ((int)255);
	static const int ColorIndex28 = ((int)128) << 16 | ((int)179) << 8 | ((int)255);
	static const int ColorIndex29 = ((int)206) << 16 | ((int)255) << 8 | ((int)0);
	static const int ColorIndex30 = ((int)255) << 16 | ((int)204) << 8 | ((int)204);
	static const int ColorIndex31 = ((int)204) << 16 | ((int)255) << 8 | ((int)153);
};

static int ColorBaseS[30] = 
{
	ColorBase::RedColor,
	ColorBase::GreenColor,
	ColorBase::BlueColor,
	ColorBase::DarkPurpleColor,
	ColorBase::GreyColor, ColorBase::MagentaColor,
	ColorBase::DarkGreyColor,  ColorBase::PinkColor,
	ColorBase::LightBlueColor, ColorBase::DoveColor, ColorBase::PurpleColor,
	ColorBase::YellowColor, ColorBase::DarkYellowColor,
	ColorBase::OrangeColor, ColorBase::StemColor, ColorBase::DarkRedColor,
	ColorBase::ColorIndex22, ColorBase::ColorIndex23, ColorBase::DarkGreenColor,
	ColorBase::ColorIndex24, ColorBase::ColorIndex25, ColorBase::DarkDoveColor,
	ColorBase::ColorIndex26, ColorBase::ColorIndex27,
	ColorBase::ColorIndex28, ColorBase::ColorIndex29, ColorBase::ColorIndex30, ColorBase::ColorIndex31,
	ColorBase::DarkBlueColor, ColorBase::WhiteColor
};

#endif