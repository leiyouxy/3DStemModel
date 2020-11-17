#ifndef OutliersRemovalByRadius_H
#define OutliersRemovalByRadius_H

//���ݸ�������뾶�ڵ�ĸ����Ĳ����Ƴ�������

#include <pcl/filters/radius_outlier_removal.h>

#include "Commdefinitions.h"
#include "CommPointBase.h"
#include "CommVector.h"

#include <QMainWindow>
#include <QMessageBox>
#include <QGroupBox>

#include "TreeBase.h"

#include "GeneratedFiles/ui_OutliersRemovalByNeighbour.h"

#include "HorizontalPartition.h"

#include "PointGeometry.h"
//#include "TreeOutliersRemoval.h"

using namespace std;

struct PointNeigbour
{
	int PointIndex;
	vector<int> NeigbourIndex;
	vector<float> NeigbourDis;
};

struct SectionNeighourStat {
	int SectionIndex;
	double Mean;
	double Std;
} ;

struct SectionNeighourQuantile {
	int SectionIndex;
	double IQR;
	double UpperLimit;
	double LowerLimit;
};

class COutliersRemovalByRadius : public CTreeBase
{
	Q_OBJECT
public Q_SLOTS:

	void CheckingOutliers();
	void RemoveOutliers();
	
	void ShowSlicesPlane(int CheckValue);
	void DrawSlicesPlane(int ThickNess);

	void RadiusChange(double Number);

	void Redo();

	void Bat();
private:
	//double LeafX, LeafY, LeafZ;
	double SectionThickness;
	double SearchRadius;	

	int OutliersNumber;
	OutliersColor OlColor;
	int R;
	int G;
	int B;

	Ui::OutliersRemovalByNeighbourForm SubuiOfOutliersRemoval;	
	
	//The work type (Stat or Quartile) for outliers removing
	string RemoveType; 
	
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree;
	
	//sections
	CHorizontalPartition HorizontalPartition;

	//The number of Neighbour for each point, the index of vector is equal to the 
	vector<PointNeigbour> NeighbourNumberOfPoint;

	//SectionNeighourStat info for sections
	vector<SectionNeighourStat> Vec_SectionNeighourStat;
	
	//SectionNeighourQuantile info for sections
	vector<SectionNeighourQuantile> Vec_SectionNeighourQuantile;

	//Circle points for show slices
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalCriclePoints;

	// calc the number of neighbour for each point
	void CalcNeighbourofPoint();

	//calc stat infor for each section
	void CalcSectionNeighbourStat();

	//calc Quartile infor for each section
	void CalcSectionNeighbourQuartile();

	//calc Number for each section
	void CalcSectionNeighbourNumber();	

	//calc Irregularity for each point
	void CalcIrregularity();

	string OperationType;
	
	CPointGeometry PointCurvature;

	vector<PointGeometryFeature> PointGeometryFeatures;

	void GetParameters();
public:
	COutliersRemovalByRadius();
	COutliersRemovalByRadius(QGroupBox * ParentWin, string Type);
	~COutliersRemovalByRadius();

	void CalcPointGeometryFeature(PointGeometryFeature & PGF);
	void RefreshData();
};

/*  Old Class 
//�����Ƴ����������
class COutliersRemovalByRadius : public CTreeBase
{
private:
	//��ά���ؿռ�Ĵ�С
	double LeafX, LeafY, LeafZ;
	double SearchRadius;
	int MinNeighbors;

	//�ܵĵ�������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;
	//���������ĵ������� Index�ڴ˽���Ϊ�������
	pcl::PointCloud<PointXYZRGBIndex>::Ptr InputIndexCloud;

	//��������
	SectionVector SectionsVector;	

	//�����Octree�� 
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree;

public:

	//�����������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutputInners;
	
	//����쳣����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Outliers;	

	//�������ؿռ估�������
	void SetVoxelParameters(double LeafXValue, double LeafYValue, 
		double LeafZValue, double SearchRadiusValue, int MinNeighborsValue);

	//�����ܶ�ɾ���쳣��(�Լ��²���_���²��� Ҳ����ɾ����ɢ������) 2015.06.15 
	void RadiusOutlierRemoval();

	//2015.06.15 ��д
	COutliersRemovalByRadius()
	{	//�˲������������׼���
		Octree = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));	
		MinNeighbors = 20;
		OutputInners.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
		Outliers.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
	}

	//2015.06.15
	void SetIntputParameters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue,
		pcl::PointCloud<PointXYZRGBIndex>::Ptr InputIndexCloudValue,
		SectionVector & SectionsVectorValue, 
		double SearchRadiusValue);
private:
	//2015.06.15 ����ÿһ����ĵ����ܶȣ����� SearchRadius �뾶�ڣ��ж��ٸ��� MinNeighborsValue
	void CalcEachPointDensity();

	//2015.06.15 ����ÿ�������ĵ����ܶȣ����ƽ��ֵ����
	void CalcSectionsDensity(double StdMutiple, 
		int StartIndex = 0, int EndIndex = 0);
public:
	//2015.05.15 ��ÿ������������Ⱥ�㻮�֣���������ĸ���ƽ��ֵ
	// �������Ǹ��� SearchRadiusValue �뾶�� ���� ��MinNeighborsValue���㣬
	void OutlierRemovalBySectionsStat(int SuccessiveSectionCount, double StdMutiple);

	//2016.01.15 ֱ�Ӹ��ݸ����뾶�������ж��ٸ���������Ƴ�����
	void OutlierRemovalByMinNeighbour(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempInputCloud,
		double Radius, int MinNeighbourNum,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr NormalCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutliersCloud, int LoopNum = 1);

	void GetNormalAndOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr NormalCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutliersCloud);
};
*/


#endif

//���� �ض������� ���ض���ĸ��� �ķ������Ƴ���ɢ������
	//CRadiusOutlierRemoval RadiusOutlierRemoval;
	////RadiusOutlierRemoval.SetIntputParameters(StemPointsPtr, StemPointsIndexPtr,
	////		HorizontalPartition.SectionsVector, 1);
	//RadiusOutlierRemoval.SetIntputParameters(StemPointsPtr, StemPointsIndexPtr,
	//		HorizontalPartition.SectionsVector, 0.5);
	//RadiusOutlierRemoval.RadiusOutlierRemovalForSections(5, 1);	
	//RadiusOutlierRemoval.GetNormalAndOutliers(NormalStemPointsPtr, OutliersStemPointsPtr);
	//if (NormalStemPointsPtr->points.size() > 0)
	//	PointBase::SavePCDToFileName(NormalStemPointsPtr,"ProcessedPoints\\NormalStemPointsPtr.pcd");
	//if (OutliersStemPointsPtr->points.size() > 0)
	//	PointBase::SavePCDToFileName(OutliersStemPointsPtr,"ProcessedPoints\\OutliersStemPointsPtr.pcd");


	/////���ݸ�������뾶������������ͳ����Ϣ�Ƴ�������
	//COutliersRemovalByRadius OutliersRemovalByRadius;
	////OutliersRemovalByRadius.SetIntputParameters(StemPointsPtr, StemPointsIndexPtr,
	////		HorizontalPartition.SectionsVector, 1);
	//OutliersRemovalByRadius.SetIntputParameters(StemPointsPtr, StemPointsIndexPtr,
	//		HorizontalPartition.SectionsVector, 0.5);
	//OutliersRemovalByRadius.RadiusOutlierRemovalForSections(5, 1);	
	//OutliersRemovalByRadius.GetNormalAndOutliers(NormalStemPointsPtr, OutliersStemPointsPtr);
	//if (NormalStemPointsPtr->points.size() > 0)
	//	PointBase::SavePCDToFileName(NormalStemPointsPtr,"ProcessedPoints\\NormalStemPointsPtr.pcd");
	//if (OutliersStemPointsPtr->points.size() > 0)
	//	PointBase::SavePCDToFileName(OutliersStemPointsPtr,"ProcessedPoints\\OutliersStemPointsPtr.pcd");
