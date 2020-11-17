#pragma once

/*

���ݾֲ�����׼ȷ�������ɵ��������򣬶�λ���ɵ���ƽ��
2019.06.13
���� abs(q_i-c) / avgDЯ��
ÿ�������ƽ�淨���� �� ���ĵ� ���õ��γɷ��������ļн�
Э����������������������ı���������ֵԽ�ӽ�1��Խ����ΪԲ��
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
	double SmoothSmallAngle = 150;		//2019.08.30 ���ù�˳��С��
	double SmoothBigAngle = 180;		//2019.08.30 ���ù�˳����
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
	
	//2018.08.19 ���ݴ�ֱ�ֶε���ƽ��ͶӰ�Լ������ĵ�ĽǶȣ��Լ����ڵ�ĽǶ��ж��Ƿ������ɵ���
	void CheckBranchForNextSliceByProjectionAngle(bool toUp = true);

	//2019.08.30 �ڸ��ݾ����жϵĻ����ϣ�ͶӰ��ƽ�棬���������ĵ�ĽǶ��ж���֦��
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

	//2019.08.31 �Ƴ��Ƕȷ����о������ĵ���Զ�ĵ�
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

	//2019.08.30 ���ݷ�����Ԥ���������㣬Ч�������Ǻ����� 2019.08.30
	void NormalCalculation();
	void ShowPointNormal(int CheckValue);

	//2019.09.18
	void SetRadioRadius(bool CheckValue);
};
	 