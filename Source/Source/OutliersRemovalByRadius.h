#ifndef OutliersRemovalByRadius_H
#define OutliersRemovalByRadius_H

//根据给定领域半径内点的个数的差异移除噪声点

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
//定义移除噪声点的类
class COutliersRemovalByRadius : public CTreeBase
{
private:
	//三维体素空间的大小
	double LeafX, LeafY, LeafZ;
	double SearchRadius;
	int MinNeighbors;

	//总的点云数据
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;
	//带有索引的点云数据 Index在此借用为邻域个数
	pcl::PointCloud<PointXYZRGBIndex>::Ptr InputIndexCloud;

	//分区数据
	SectionVector SectionsVector;	

	//构造的Octree树 
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree;

public:

	//输出正常点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutputInners;
	
	//输出异常点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Outliers;	

	//设置体素空间及邻域参数
	void SetVoxelParameters(double LeafXValue, double LeafYValue, 
		double LeafZValue, double SearchRadiusValue, int MinNeighborsValue);

	//根据密度删除异常点(以及下采样_不下采样 也可以删除离散噪声点) 2015.06.15 
	void RadiusOutlierRemoval();

	//2015.06.15 后写
	COutliersRemovalByRadius()
	{	//八叉树建立到毫米级别
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
	//2015.06.15 计算每一个点的点云密度，即在 SearchRadius 半径内，有多少个点 MinNeighborsValue
	void CalcEachPointDensity();

	//2015.06.15 计算每个分区的点云密度，求出平均值即可
	void CalcSectionsDensity(double StdMutiple, 
		int StartIndex = 0, int EndIndex = 0);
public:
	//2015.05.15 对每个分区进行离群点划分，根据求出的个数平均值
	// 出发点是根据 SearchRadiusValue 半径内 至少 有MinNeighborsValue个点，
	void OutlierRemovalBySectionsStat(int SuccessiveSectionCount, double StdMutiple);

	//2016.01.15 直接根据给定半径内至少有多少个邻域点来移除噪声
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

//采用 特定距离内 到特定点的个数 的方法来移除离散噪声点
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


	/////根据给定邻域半径内邻域点个数的统计信息移除噪声点
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
