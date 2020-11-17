#ifndef HorizontalPartition_H
#define HorizontalPartition_H

#include <vector>
#include <pcl/point_types.h>
#include "Commdefinitions.h"
#include "CommVector.h"
#include "CommGeometry.h"
#include "TreeBase.h"

using namespace std;


/*
2018.12.19
The unit for point cloud is m, not mm. 
*/
class CHorizontalPartition : public CTreeBase
{
private:	

	void Initialization();

	const string SlicesPointsStr = "SlicesPoints";

	//2019.04.18 The color for exclusion points when calculating
	int ExclusionColor;
public:
	CHorizontalPartition()
	{
		SectionsCount = 0;
		ZMax = 0, ZMin = 0;
		MassCenterPointsPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
		GeometryCenterPointsPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
		Viewer = NULL;
		ExclusionColor = ColorBase::RedColor;
	}
	~CHorizontalPartition();

	double XMax, XMin, YMax, YMin, ZMax, ZMin;

	int SectionsCount;

	double SectionThickness;

	// Vector for Partition points;
	SectionVector SectionsVector;

	//The mass point of sections
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MassCenterPointsPtr;
	
	//The geometrical center points of sections
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeometryCenterPointsPtr;
		
	void SetThickNess(double ThickNessValue);

	//Patition Patitions for inputclouds
	void PatitionSection();
		
	//Find Section Index By ZValue. The condition is that The ZValue is equal or greater than the ZMin of the Section, meanwhile, the ZValue is smaller than the  ZMax of the Section 
	int FindSectionIndex(double ZValue);

	// Save points from several sections to file
	void SaveSectionSToFile(string FileName, int StartIndex, int EndIndex);

	//将点集分别存储到各自的分区中 2020.08.06
	void SaveSectionSToEachFile(string FileNamePrefix, int StartIndex, int EndIndex);

	//Show section points uses R G B colors
	void ShowSectionPoints(int StartIndex, int EndIndex);

	//UnShow section points
	void UnShowSectionPoints();

	//calc Geometry Center Points for each slice using the center gravity of convex polygon  2018.12.31
	void CalcCenterPointsByConvexPolygon(int StartIndex = 0, int EndIndex = 0);

	//calc Geometry Center Points for each slice using the Least Squares method 2018.12.31
	void CalcCenterPointsByLeastSquares();

	//calc Geometry Center Points for each slice using gravity 2018.12.31
	void CalcCenterPointsByGravity(int StartIndex = 0, int EndIndex = 0);

	//get points of section by SectionIndex 2018.12.31
	void GetSectionPoints(int SectionIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints);

	void SetSectionColors(int StartIndex = 0, int EndIndex = 0, const int ColorValue = ColorBase::RedColor);

	void SetExclusionColor(int ColorValue = ColorBase::RedColor);
};

/*

class CHorizontalPartition: CTreeBase //partition points into serveral slices
{
private:

	pcl::PointCloud<PointXYZRGBIndex>::Ptr UnPatitionPointsPtr;

	//待分区的点云数据指针	
	
	//double CalcSectionDValue(SectionStruct SectionValue);
	//计算每个分区的直径信息

	//void SaveCenterPCDToDB(SectionStruct Section,
	//	string ConnStr, string TableName, int Index, bool ExistsCreate = true);	
	//将分区得到的数据保存到数据库中

	//2014.09.22 在xy平面探索当前分区的中心位置，Step 为步长，选定某一个坐标轴一直前进
	void ExploreSectionCenter(SectionStruct & SectionValue, 
		double & CenterX, double & CenterY, double & AvgRadius,
		double Step, double & LastVariance,
		int & MaxExecutions, string Axis = "");

	int SectionsNumber;
	//int SectionsNumber;
	//点云的分区个数，如果指定此值,就按照此值来确定划分分区，与上面的 SectionThickness 只选之一。

	//在点集中获取下一个边界点，上界或者下界 2016.01.16
	//vector<int> GetNextBoundPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	//	int BoundIndex, vector<int> NeighbourIndex, bool IsUpper = true);
public:		

	CHorizontalPartition()
	{		
		MassCenterPointsPtr.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
		GeometryCenterPointsPtr.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
	}
	SectionVector SectionsVector;
	//分区后的点云分区容器

	double XMax, XMin, YMax, YMin, ZMax, ZMin;	
			
	//每个分区的厚度
	double SectionThickness;	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MassCenterPointsPtr;
	//每个分区的重心点指针

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeometryCenterPointsPtr;
	//每个分区的几何中心点指针
		
	//int SectionsNumber;
	//点云的分区个数，如果指定此值,就按照此值来确定划分分区，
	//与上面的 SectionThickness 只选之一。
	void SetInputS(
		pcl::PointCloud<PointXYZRGBIndex>::Ptr pUnPatitionPoints, 
		double SectionThicknessValue);
	//设置待分区的数据信息

	void PatitionSection();
	//根据信息对点云进行分区处理

	//根据 ZValue 的值 获取 ZValue 所在的 SectionPoint 的Index 是大于等于最小值，小于最大值
	int FindSectionIndex(double ZValue);

	//2016.01.21 调整
	//计算每个分区的几何中心，Sampled==true是根据抽样规则计算的 默认 每个角度值15度，抽样的点个数是5个
	void CalcGeometryCenterPoints(int StartIndex = 0, int EndIndex = 0, 
		bool Sampled = false, double AngleValue = 15, int AngleNumer = 5);

	//计算每个分区的质心
	void CalcCentroidPoints(int StartIndex = 0, int EndIndex = 0, 
		bool Sampled = false, double AngleValue = 15, int AngleNumer = 5);
	
	//将当前分区Index的中心点CurrentCenter 移动到 BaseCenter， 
	//同时移动本分区及后面分区的点云及中心点。 2015.07.08
	// SourcePointMoved 标志 原始点云是否移动
	void MovePointsToBaseCenter(pcl::PointXYZRGB NewCenter,
		pcl::PointXYZRGB CurrentCenter, int Index, 
		bool SourcePointMoved = true, bool Up = true);

	//显示点云
	void ShowSectionsPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		int StartIndex, int EndIndex, bool ShowLabel = false, string PointIDStr = "",
		double Size = 1, bool PointColorChanged = false,
		int Colorr = 255, int Colorg = 0, int Colorb = 0);
	
	//获取特定分区的点云数据
	void GetSectionsPoint(pcl::PointCloud<PointXYZRGBIndex>::Ptr SectionSPoints,int StartIndex, int EndIndex);

	//根据分区重心点点云初始化几何中心点云
	void InitialGeometryPoint();

	//计算某个分区的几何中心
	void CalcGeometryCenterPoint(int SectionIndex, double Step = 0.1);
	
	//保存部分区域的点云到文件中
	void SaveSectionSToFile(string FileName, int StartIndex, int EndIndex);

	//根据树干长度所在位置的索引 2015.10.22 
	//使用这个的前提是 中心点 获取比较正确才可以 暂时不用
	int GetStemLengthIndexBySectionCenter(double StemLength);

	////2016.01.16 获取分区的边界点   2016.01.16 算法未完全实现，
	//void GetBoundPoints(int SectionIndex, 
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BoundPoints,
	//	bool IsUpper = true);	
	//
	////边A B 是否是平面点集的边界
	//bool LineIsBorder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints,
	//	int LinePointA, int LinePointB, int LinePointD, bool IsUpper = true);


	//2016.01.21 移动到合适的显示位置
	void GetShowPosition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsPtr);
	void GetShowPosition(pcl::PointCloud<PointXYZRGBIndex>::Ptr PointsIndexPtr);
};

*/

#endif

	//////保存多少个分区的点云
	////for(int i = 0; i < 400; i++)
	////{
	////	BranchesPointsPtr->points.insert(BranchesPointsPtr->points.end(),
	////		HorizontalPartition.SectionsVector[i].SectionPtr->points.begin(),
	////		HorizontalPartition.SectionsVector[i].SectionPtr->points.end());
	////}
	////PointBase::SavePCDToFileName(BranchesPointsPtr, "ProcessedPoints\\NormalStemPointsPtr400.pcd");



	////HorizontalPartition.SetInputS(StemPointsIndexPtr, SectionThick);
	////HorizontalPartition.PatitionSection();
	////HorizontalPartition.CalcCentroidPoints(false);
	////		
	////PointBase::ShowPointXYZRGB(Viewer, StemPointsIndexPtr, "StemPointsIndexPtr", 1);
	//////HorizontalPartition.ShowSectionsPoints(Viewer, 0, HorizontalPartition.SectionsVector.size(), true);
	////PointBase::SetPointColor(HorizontalPartition.GeometryCenterPointsPtr, ColorBase::RedColor);
	////PointBase::ShowPointXYZRGB(Viewer, HorizontalPartition.GeometryCenterPointsPtr, "GeometryCenterPointsPtr", 5);

	//////再次分区
	////PointsMove(StemPointsIndexPtr,  0, 40, 0);	
	////HorizontalPartition.SetInputS(StemPointsIndexPtr, SectionThick);
	////HorizontalPartition.PatitionSection();
	////HorizontalPartition.CalcCentroidPoints(true);
	////
	////PointBase::ShowPointXYZRGB(Viewer, StemPointsIndexPtr, "SampleStemPointsIndexPtr", 1);
	//////HorizontalPartition.ShowSectionsPoints(Viewer, 0, HorizontalPartition.SectionsVector.size(), true, "Sample");
	////PointBase::SetPointColor(HorizontalPartition.GeometryCenterPointsPtr, ColorBase::BlueColor);
	////PointBase::ShowPointXYZRGB(Viewer, HorizontalPartition.GeometryCenterPointsPtr, "SampleGeometryCenterPointsPtr", 5);
	////

	//PointBase::OpenPCLFile(FileName, StemPointsIndexPtr);
	//HorizontalPartition.SetInputS(StemPointsIndexPtr, SectionThick);
	//HorizontalPartition.PatitionSection();
	//HorizontalPartition.CalcCentroidPoints();
	//PointBase::ShowPointXYZRGB(Viewer, HorizontalPartition.GeometryCenterPointsPtr, "GeometryCenterPointsPtr", 1);
	//HorizontalPartition.ShowSectionsPoints(Viewer, 0, HorizontalPartition.SectionsVector.size() - 1,false);
