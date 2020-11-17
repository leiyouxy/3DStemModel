#ifndef AnglePartition_H
#define AnglePartition_H

#include <math.h>
#include "Commdefinitions.h"
#include "CommClass.h"
#include "CalcGeometryCenter.h"
#include "CommGeometry.h"

using namespace std;

//Angle partition for vertical slices based on the geometrical center points for each vertical slice.
//modified by lei you at 2018.12.29
class CAnglePartition
{
private:
	double Angle;						//radian value for angle Partition
	//pcl::PointCloud<PointXYZRGBIndex>::Ptr InputCloud;	//point cloud for unpartition points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;	//point cloud for unpartition points
	SectionVector SectionsVector;		//the vertical slices for InputCloud, must be done before using angle partition
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CenterPoints; //the geometrical center points for each vertical slice.

	//每个分区角度划分的初始化操作
	void SectionPartitionInitiation(vector<AnglePartitionStruct> & SectionAngle);

	//计算每一个角度分区的切平面参数  默认情况下 d 参数有该角度分区的点云重心点云确定
	void CalcAnglePartitionTangentPlane(int SectionIndex,
		AnglePartitionStruct & AnglePartitionValue);

	//计算每一个角度分区的切平面参数 2015.07.10
	void CalcAnglePartitionTangentPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		AnglePartitionStruct & AnglePartitionValue);

public:
	int PartitionsCount;		//PartitionsNumer for anlge partition
	
	//the anlge partition result for serveral slices  
	vector<SectionAnglePartition> SectionAnglePartitionS;

	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemCloudValue,
		SectionVector & SectionsVector,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CenterPoints,
		double AngleValue = 15);

	//对 SectionIndex 这个分区进行角度划分
	void PartitionSection(int SectionIndex,	bool UseTangentPlane = false);

	//对所有分区进行角度划分 如果对所有分区进行角度划分 就涉及到如何保存这些数据的问题
	vector<SectionAnglePartition> PartitionAllSection(int StartIndex = -1, int EndIndex = -1,
		bool UseTangentPlane = false);

	////根据 分区起始范围及 角度范围获取点云数据
	void GetPointsByAngleZone(pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutCloud,
		int StartSection, int EndSection,
		double StartAngle, double EndAngle);

	////2015.06.30 根据给定的点云集及中心点  按照 AngleValue 做角度划分  外部调用方便
	void PartitionPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		pcl::PointXYZRGB CenterPoint,  double AngleValue,
		vector<AnglePartitionStruct> & SectionAngle, bool UseTangentPlane = false);

	////2015.06.30 根据给定的点云集及中心点  按照 AngleValue 做角度划分  外部调用方便
	void PartitionPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		double AngleValue, vector<AnglePartitionStruct> & SectionAngle, bool UseTangentPlane = false);

	//AngleSample, one point for each angle section
	void AngleSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints, double Angle = 1);
};

#endif


	///////每个水平分区的角度分区
	//AnglePartition.SetInputs(StemPointsPtr,
	//	HorizontalPartition.SectionsVector,
	//	HorizontalPartition.GeometryCenterPointsPtr, 15);
	////每个分区的角度分区是一个容器，
	//AnglePartition.PartitionAllSection(300,310,true);


	//////模拟画切平面
	//pcl::PointXYZRGB CenterPoint;	
	//CenterPoint.x = 0;
	//CenterPoint.y = 0;
	//CenterPoint.z = 0;
	//PointBase::DrawPlane(Viewer, 2, 1, 0.1, CenterPoint);
	//////模拟画切平面


	////Test 切平面
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TestPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
	//
	//AnglePartition.GetPointsByAngleZone(TestPtr,300,310,0,45);
	//AnglePartition.GetPointsByAngleZone(TestPtr,300,310,330,360);

	//Eigen::Matrix3f EigenVector;
	//Eigen::Vector3f EigenValue(3);
	//pcl::PointXYZRGB CenterPoint;
	//
	//GeometryBase::GetPointsTangentPlane(TestPtr, CenterPoint, EigenVector, EigenValue);
	//PointBase::DrawPlane(Viewer,EigenVector(0),EigenVector(1),EigenVector(2),CenterPoint);

	//PointBase::ShowPointXYZRGB(Viewer, TestPtr, "TestPtr", 1);
	////Test 切平面