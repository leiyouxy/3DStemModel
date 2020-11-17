/* 2015.06.05 
���򹫹����岿�֣�

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
//2015.06.05 �������ֵ�������

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
//�������ڱ��������Ϣ�ĵ��ƽṹ�����ĵ㣬�Լ�������Ӧ�İ뾶ֵ
struct SectionInfoPoint
{
	PCL_ADD_POINT4D;			// �õ�������4��Ԫ��
	PCL_ADD_RGB;                // �õ�������rgb

	int SectionIndex;				//��������	4
	double rValue;					//��ǰ�����İ뾶ֵ 5
	double Variance;					//��ǰ�������ĵ�õ�ʱ�ķ���ֵ 6
	double BasalArea;				//��ǰ�ڶ�άƽ��ı���� 7
	double V;						//��ǰ������� 8
	int SectionPointsNumber;		//��������� 9
	double HValue;					//��ǰ�����ĸ߶� 10
	double H1Value;					//���ҵ���ǰ�����ĸ߶� 11
	double rValueToH1Value;			//��ǰ�����İ뾶ֵ�� ���ҵ���ǰ�����ĸ߶� ֮��
	double rDisToUpper;				//��ǰ�����뾶���Ϸ����뾶֮��
	double CenterPointDisToUpper;	//��ǰ�������ĵ����Ϸ������ĵ�ľ���
	double ZValueDisToUpper;			//��ǰ����Zֵ���Ϸ���Zֵ֮��

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW	// ȷ��new�������������}
}EIGEN_ALIGN16;					// ǿ��SSE����

	POINT_CLOUD_REGISTER_POINT_STRUCT(SectionInfoPoint,// ע������ͺ�
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

//2015.06.05 ����ԭ�������͵ĵ���
struct PointXYZRGBIndex : pcl::PointXYZRGB		//�������ID�ĵ���
{
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;                // �õ�������4��Ԫ��

	int Index;							//Index	�����ܼ�¼��������	
	int Category;
	//Unknow = 0,
	//TreePoint = 1,
	//StemPoint = 2,
	//BranchePoint = 3,
	//OutlierPoint = 4,
	//StemAxisCurvePoint = 5

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW	// ȷ��new�������������}
}EIGEN_ALIGN16;					// ǿ��SSE���� 

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBIndex,// ע������ͺ�
(float, x, x)
(float, y, y)
(float, z, z)
(int, Index, Index)
(int, Category, Category)
)

//2018.08.03 ����ɫ�ʾ�γ������ĵ���

struct PointXYZRGBWithLongiAndLati					//�������ID�ĵ���
{
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;                // �õ�������4��Ԫ��

	double X;
	double Y;
	double Z;

	double Longitude;	//����
	double Latitude;	//γ��
	double Altitude;	//����

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW	// ȷ��new�������������}
}EIGEN_ALIGN16;					// ǿ��SSE���� 

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBWithLongiAndLati,// ע������ͺ�
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


//ƽ��ϵ��
struct PlaneCoefficient
{
	double a;
	double b;
	double c;
	double d;
	// ax+by+cz+d = 0 ȷ����ƽ��
};

//���սǶȶ�ÿ��ˮƽ�����ٴη��� ��֪���������ĵ��ǰ����
////2015.06.22 
struct AnglePartitionStruct
{
	int AngleIndex;			// the index for anlge Partition
	bool NeedRefined;			// Identifies whether the column has been repaired 
	bool Refined;			// Identifies whether the column has been repaired 
	double AngleStart;		// >= radian value��not degree value
	double AngleEnd;		// <  radian value��not degree value
	double AvgDis;			// average distantce from center to each point
	double MaxDis;
	double MinDis;
	vector<int> PointIndexs;		//the point index for a partition
	pcl::PointXYZRGB BeforeRefineCenterPointofCurPartition;	//center point for the current angle partition
	pcl::PointXYZRGB CenterPointofCurPartition;	//center point for  the current angle partition
	PlaneCoefficient TangentPlaneCoefficient;	//tagent palne parameters for  the current angle partition
};

//һ�������Ķ���Ƕ��µķ�������
struct SectionAnglePartition
{
	int SectionIndex;
	double AvgDis;			// average distantce from center to each Angle  CenterPointofCurPartition
	vector<AnglePartitionStruct> AnglePartition;
};

//struct SectionTangentPlaneCoefficient	//һ�������Ķ���Ƕ��µķ�����ƽ�������
//{
//	int SectionIndex;
//	vector<PlaneCoefficient> TangentPlaneCoefficients;
//};

struct SectionStruct	//�������ƵĽṹ��
{
	//pcl::PointCloud<PointXYZRGBIndex>::Ptr SectionPtr;			//ָ��������Ƶ�ָ��
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPtr;			//ָ��������Ƶ�ָ��	
	vector<int> Indexs;  //the Indexs of current section 2018.12.20 modified by yl
	double ZMax;	//������Z�����ֵ	
	double ZMin;	//������Z����Сֵ	
	double xCenter;	//x��������ĵ� ����	
	double yCenter;	//y��������ĵ�	����	
	double zCenter;	//z��������ĵ�	����	

	//double DValue;	//ֱ��	
};

typedef vector<SectionStruct> SectionVector;
//ָ��������ݼ���ָ������

struct CrossSectionStruct	//�������ƵĽṹ��
{
	pcl::PointCloud<PointXYZRGBIndex>::Ptr SectionPtr;		//ָ��������Ƶ�ָ��
	bool  IsCircle;	//�������Ƿ��Բ�ηֲ�
	double ZMax;	//������Z�����ֵ	
	double ZMin;	//������Z����Сֵ	
	double DValue;	//ֱ��	

	pcl::PointXYZRGB CenterPoint;	//�������ĵ�

	//2015.11.01 add ֻΪ��ֱ���ɺ����ָ��ʹ��
	PlaneCoefficient PlaneLower;	//�²�ƽ��
	PointXYZRGBIndex PointLower;	//�²�ƽ���һ���̶���
	PlaneCoefficient PlaneUpper;	//�ϲ�ƽ��
	PointXYZRGBIndex PointUpper;	//�ϲ�ƽ���һ���̶���	
};

typedef vector<CrossSectionStruct> CrossSectionVector;
//ָ��������ݼ���ָ������

typedef struct
{
	//int Index;	//����Ĵ���
	int ConstantIndex;	//�Ѿ��̶���һ����
	int EdgeIndexOne;	//��Ҫ��չ�ĵ�һ���ڵ�
	int EdgeIndexTwo;	//��Ҫ��չ�ĵڶ����ڵ�
} EdgeNodes;

typedef struct
{
	int Count;
	int IndexOne;
	int IndexTwo;
} EdgeCount;

//һ�����㹫ʽ��ջ
typedef vector<string> OperationItem;

//������㹫ʽ�Ķ��ջ
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

	double AvgDis;						//����������ƽ������
	double DensityByDis;				//ƽ��������������֮�� ��  ƽ����������С����֮�ֵΪ0 ������㼯�ֲ��ȽϾ���
	double DisBetProjectAndCentriod;	//������ƽ��ͶӰ�㵽���ĵ�ľ���
	double DisBetProjectAndOriginal;	//������ƽ��ͶӰ�㵽��ľ���
	double AvgGuassCurvatureChange;		//��ǰ����������˹���ʱ仯��ƽ��ֵ

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
	static const int BlackColor = ((int)0) << 16 | ((int)0) << 8 | ((int)0);  //����������
	static const int WhiteColor = ((int)255) << 16 | ((int)255) << 8 | ((int)255);
	static const int GreyColor = ((int)192) << 16 | ((int)192) << 8 | ((int)192);	//��ɫ ����̫����
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