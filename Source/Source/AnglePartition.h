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

	//ÿ�������ǶȻ��ֵĳ�ʼ������
	void SectionPartitionInitiation(vector<AnglePartitionStruct> & SectionAngle);

	//����ÿһ���Ƕȷ�������ƽ�����  Ĭ������� d �����иýǶȷ����ĵ������ĵ���ȷ��
	void CalcAnglePartitionTangentPlane(int SectionIndex,
		AnglePartitionStruct & AnglePartitionValue);

	//����ÿһ���Ƕȷ�������ƽ����� 2015.07.10
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

	//�� SectionIndex ����������нǶȻ���
	void PartitionSection(int SectionIndex,	bool UseTangentPlane = false);

	//�����з������нǶȻ��� ��������з������нǶȻ��� ���漰����α�����Щ���ݵ�����
	vector<SectionAnglePartition> PartitionAllSection(int StartIndex = -1, int EndIndex = -1,
		bool UseTangentPlane = false);

	////���� ������ʼ��Χ�� �Ƕȷ�Χ��ȡ��������
	void GetPointsByAngleZone(pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutCloud,
		int StartSection, int EndSection,
		double StartAngle, double EndAngle);

	////2015.06.30 ���ݸ����ĵ��Ƽ������ĵ�  ���� AngleValue ���ǶȻ���  �ⲿ���÷���
	void PartitionPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		pcl::PointXYZRGB CenterPoint,  double AngleValue,
		vector<AnglePartitionStruct> & SectionAngle, bool UseTangentPlane = false);

	////2015.06.30 ���ݸ����ĵ��Ƽ������ĵ�  ���� AngleValue ���ǶȻ���  �ⲿ���÷���
	void PartitionPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		double AngleValue, vector<AnglePartitionStruct> & SectionAngle, bool UseTangentPlane = false);

	//AngleSample, one point for each angle section
	void AngleSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints, double Angle = 1);
};

#endif


	///////ÿ��ˮƽ�����ĽǶȷ���
	//AnglePartition.SetInputs(StemPointsPtr,
	//	HorizontalPartition.SectionsVector,
	//	HorizontalPartition.GeometryCenterPointsPtr, 15);
	////ÿ�������ĽǶȷ�����һ��������
	//AnglePartition.PartitionAllSection(300,310,true);


	//////ģ�⻭��ƽ��
	//pcl::PointXYZRGB CenterPoint;	
	//CenterPoint.x = 0;
	//CenterPoint.y = 0;
	//CenterPoint.z = 0;
	//PointBase::DrawPlane(Viewer, 2, 1, 0.1, CenterPoint);
	//////ģ�⻭��ƽ��


	////Test ��ƽ��
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
	////Test ��ƽ��