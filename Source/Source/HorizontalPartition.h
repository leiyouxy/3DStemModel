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

	//���㼯�ֱ�洢�����Եķ����� 2020.08.06
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

	//�������ĵ�������ָ��	
	
	//double CalcSectionDValue(SectionStruct SectionValue);
	//����ÿ��������ֱ����Ϣ

	//void SaveCenterPCDToDB(SectionStruct Section,
	//	string ConnStr, string TableName, int Index, bool ExistsCreate = true);	
	//�������õ������ݱ��浽���ݿ���

	//2014.09.22 ��xyƽ��̽����ǰ����������λ�ã�Step Ϊ������ѡ��ĳһ��������һֱǰ��
	void ExploreSectionCenter(SectionStruct & SectionValue, 
		double & CenterX, double & CenterY, double & AvgRadius,
		double Step, double & LastVariance,
		int & MaxExecutions, string Axis = "");

	int SectionsNumber;
	//int SectionsNumber;
	//���Ƶķ������������ָ����ֵ,�Ͱ��մ�ֵ��ȷ�����ַ������������ SectionThickness ֻѡ֮һ��

	//�ڵ㼯�л�ȡ��һ���߽�㣬�Ͻ�����½� 2016.01.16
	//vector<int> GetNextBoundPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	//	int BoundIndex, vector<int> NeighbourIndex, bool IsUpper = true);
public:		

	CHorizontalPartition()
	{		
		MassCenterPointsPtr.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
		GeometryCenterPointsPtr.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
	}
	SectionVector SectionsVector;
	//������ĵ��Ʒ�������

	double XMax, XMin, YMax, YMin, ZMax, ZMin;	
			
	//ÿ�������ĺ��
	double SectionThickness;	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MassCenterPointsPtr;
	//ÿ�����������ĵ�ָ��

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeometryCenterPointsPtr;
	//ÿ�������ļ������ĵ�ָ��
		
	//int SectionsNumber;
	//���Ƶķ������������ָ����ֵ,�Ͱ��մ�ֵ��ȷ�����ַ�����
	//������� SectionThickness ֻѡ֮һ��
	void SetInputS(
		pcl::PointCloud<PointXYZRGBIndex>::Ptr pUnPatitionPoints, 
		double SectionThicknessValue);
	//���ô�������������Ϣ

	void PatitionSection();
	//������Ϣ�Ե��ƽ��з�������

	//���� ZValue ��ֵ ��ȡ ZValue ���ڵ� SectionPoint ��Index �Ǵ��ڵ�����Сֵ��С�����ֵ
	int FindSectionIndex(double ZValue);

	//2016.01.21 ����
	//����ÿ�������ļ������ģ�Sampled==true�Ǹ��ݳ����������� Ĭ�� ÿ���Ƕ�ֵ15�ȣ������ĵ������5��
	void CalcGeometryCenterPoints(int StartIndex = 0, int EndIndex = 0, 
		bool Sampled = false, double AngleValue = 15, int AngleNumer = 5);

	//����ÿ������������
	void CalcCentroidPoints(int StartIndex = 0, int EndIndex = 0, 
		bool Sampled = false, double AngleValue = 15, int AngleNumer = 5);
	
	//����ǰ����Index�����ĵ�CurrentCenter �ƶ��� BaseCenter�� 
	//ͬʱ�ƶ�����������������ĵ��Ƽ����ĵ㡣 2015.07.08
	// SourcePointMoved ��־ ԭʼ�����Ƿ��ƶ�
	void MovePointsToBaseCenter(pcl::PointXYZRGB NewCenter,
		pcl::PointXYZRGB CurrentCenter, int Index, 
		bool SourcePointMoved = true, bool Up = true);

	//��ʾ����
	void ShowSectionsPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		int StartIndex, int EndIndex, bool ShowLabel = false, string PointIDStr = "",
		double Size = 1, bool PointColorChanged = false,
		int Colorr = 255, int Colorg = 0, int Colorb = 0);
	
	//��ȡ�ض������ĵ�������
	void GetSectionsPoint(pcl::PointCloud<PointXYZRGBIndex>::Ptr SectionSPoints,int StartIndex, int EndIndex);

	//���ݷ������ĵ���Ƴ�ʼ���������ĵ���
	void InitialGeometryPoint();

	//����ĳ�������ļ�������
	void CalcGeometryCenterPoint(int SectionIndex, double Step = 0.1);
	
	//���沿������ĵ��Ƶ��ļ���
	void SaveSectionSToFile(string FileName, int StartIndex, int EndIndex);

	//�������ɳ�������λ�õ����� 2015.10.22 
	//ʹ�������ǰ���� ���ĵ� ��ȡ�Ƚ���ȷ�ſ��� ��ʱ����
	int GetStemLengthIndexBySectionCenter(double StemLength);

	////2016.01.16 ��ȡ�����ı߽��   2016.01.16 �㷨δ��ȫʵ�֣�
	//void GetBoundPoints(int SectionIndex, 
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BoundPoints,
	//	bool IsUpper = true);	
	//
	////��A B �Ƿ���ƽ��㼯�ı߽�
	//bool LineIsBorder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints,
	//	int LinePointA, int LinePointB, int LinePointD, bool IsUpper = true);


	//2016.01.21 �ƶ������ʵ���ʾλ��
	void GetShowPosition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsPtr);
	void GetShowPosition(pcl::PointCloud<PointXYZRGBIndex>::Ptr PointsIndexPtr);
};

*/

#endif

	//////������ٸ������ĵ���
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

	//////�ٴη���
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
