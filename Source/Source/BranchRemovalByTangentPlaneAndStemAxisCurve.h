#pragma once

/*

根据局部迭代准确计算树干的生长方向，定位树干的切平面
2019.06.13
概率 abs(q_i-c) / avgD携程
每个点的切平面法向量 与 中心点 到该点形成方向向量的夹角
协方差矩阵的最大两个特征根的比例，比例值越接近1，越近似为圆形
*/

#include "GeneratedFiles/ui_BranchRemovalByTangentPlaneAndStemAxisCurve.h"

#include "CommGeometry.h"
#include <omp.h>

#include "TreeBase.h"
#include "SplineInterpolation.h"
#include "HorizontalPartition.h"
#include "StemSkeleton.h"
#include "AnglePartition.h"
#include "BezierSubsectionG2.h"
#include "CommOptimal.h"
#include "PointGeometry.h"
#include "DBScanCluster.h"
#include "K_MeansCluster.h"

class CBranchRemovalByTangentPlaneAndStemAxisCurve : public CTreeBase
{
	Q_OBJECT
private:
	Ui::BranchRemovalByTangentPlaneFormAndStemAxisCurve BranchRemovalForm;
	//The two Anchor points for branch removing branches.
	pcl::PointXYZRGB UpperAnchorPoint, LowerAnchorPoint; 	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempGeometricalCenterPoints;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalGeometricalCenterPoints;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr LowerCovexPologon;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr UpperCovexPologon;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *VgDownSampleOctree;

	int SliceCount;

	int NormalAnchorIndex;
	pcl::PointXYZRGB NormalAnchorPoint;

	//CSpline Spline;
	//CSplineInterpolation SplineSkeleton;
	CHorizontalPartition HorizontalPartition;
	CStemSkeleton StemSkeleton;
	pcl::PointXYZRGB CurrentGrowthDirection;
	pcl::PointXYZRGB PriorCurrentCenterPoint;
	pcl::PointXYZRGB CurrentCenterPoint;

	pcl::VoxelGrid<pcl::PointXYZRGB> VgDownSample;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr DownSamplePoints;

	bool IsUp;	
	double LastAvg;

	//the number of slices for stem growth calculation	
	double SliceHeight;
	double NeighbourRadius;
	int CalcSectionNumbers;
	const double CalcUnitLength = 10;
	double SmoothSmallAngle = 150;		//2019.08.30 设置光顺最小角
	double SmoothBigAngle = 180;		//2019.08.30 设置光顺最大角
	double CurBarkThickNess;
	int SliceCalced;

	CPointGeometry PointGeometry;
	//CPointGeometry BiggerPointGeometry;

	pcl::PointCloud<pcl::Normal>::Ptr DoubleNormals;
	pcl::PointCloud<pcl::Normal>::Ptr HalfNormals;
	pcl::PointCloud<pcl::Normal>::Ptr CurvatureDirections = NULL;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints;
	const string HeightPlanePointsStr = "HeightPlanePoints";

	const string CentroidPointsStr = "CentroidPointsStr";
		
	void CalcGrowthDirectionAtStartHeight(double Height, bool toUp = true);

	bool CheckBranchForNextSlice(bool toUp = true);	

	double LastCurvatureMean, LastCurvatureStd;
	bool CheckBranchForNextSliceByCurvature(bool toUp = true);
	
	//2018.08.19 根据垂直分段点在平面投影以及于中心点的角度，以及相邻点的角度判断是否是树干点云
	void CheckBranchForNextSliceByProjectionAngle(bool toUp = true);

	//2019.08.30 在根据距离判断的基础上，投影至平面，根据与中心点的角度判断树枝点
	void CheckBranchForSliceByProjectionAngle(
		vector<int> SlicePointsIndexs,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePoints,
		pcl::PointXYZRGB PriorCenterPoint,
		double AnlgeOfPartition = 2);

	//Not Use 
	bool CheckBranchForNextSliceUseConvexPolygon(bool toUp = true);	
	
	//2019.06.27
	bool CheckBranchForNextSliceByBarkThickness(bool toUp = true);
	bool GetPriorSlice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlicePoints, vector<int> & PriorSlicePointsIndexs,
		bool toUp, double PriorSliceThickNess, pcl::PointXYZRGB & PriorCurrentCenterPoint);
	double CalcBarkThicknessofPriorSlice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlicePoints, 
		pcl::PointXYZRGB PriorCurrentCenterPoint, vector<AnglePartitionStruct> & SectionAngle, double Anlge = 2);

	void CheckBranchByPriorBark(pcl::PointCloud<pcl::PointXYZRGB>::Ptr SlicePoints, 
			vector<int> SlicePointsIndexs, vector<AnglePartitionStruct> & PriorSectionAngle, double AnlgeValue = 2);

	bool UpdateCurSlice(pcl::PointCloud<pcl::PointXYZRGB>::Ptr SlicePoints,
		vector<int> SlicePointsIndexs, pcl::PointXYZRGB PlaneAnchorPoint);
	
	//2019.06.27
	void CheckBranchByGradientDescent();
	void RefreshParameters();
	void AmendSkeletonCentriodPoints();
	void SeedBroadcast(int PointIndex, int & Count);

	void CheckSliceByPriorSlice(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PriorSlice,
		pcl::PointXYZRGB PriorCenter, pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlice,
		pcl::PointXYZRGB CurCenter, vector<int> CurSlicePointsIndexs, pcl::PointXYZRGB GrowthDirection);

	pcl::PointCloud<pcl::Normal>::Ptr MaxDirections;
	void ShowDirection();

	//2019.08.31 移除角度分区中距离中心点最远的点
	void RemoveFarestPoint(
		vector<int> SlicePointsIndexs,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSlicePoints,
		AnglePartitionStruct & CurAnglePartition,
		pcl::PointXYZRGB CurCenter);
public:	
	CBranchRemovalByTangentPlaneAndStemAxisCurve(QGroupBox * ParentWin);
	~CBranchRemovalByTangentPlaneAndStemAxisCurve();
	void SetViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue);

public Q_SLOTS:
	void CheckBranches();
	void DeleteBranches();
	void Redo();

	void ShowSlicesPoints(int CheckValue);
	void FindHeightPlane(int Value);
	void ShowHeightPlane(int Value);

	//2019.08.30 根据法向量预处理噪声点，效果并不是很明显 2019.08.30
	void NormalCalculation();
	void ShowPointNormal(int CheckValue);

	//2019.09.18
	void SetRadioRadius(bool CheckValue);
};
	 