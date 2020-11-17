#pragma once

/*
2019.05.16 Curvature computation for each point in point cloud

*/

#include "Commdefinitions.h"
#include "CommPointBase.h"
#include "CommGeometry.h"
#include <vector>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/search/search.h>

#include "third_party_includes/PCL_normEst.h"

class CPointGeometry
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;
	
//	float Radius;
	pcl::search::Octree<pcl::PointXYZRGB>::Ptr OCtree;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *OctreeCloudSearch;
	int OptimalK_a, OptimalK_b;	 

	//2019.10.3 计算每个点的几何特征
	void CalcPointGeometryFeature(PointGeometryFeature & PGF);
public:
	//vector<PointWithCurvature> PointGuassCurvatures;		//Guass Curvature for each point k1*k2
	//vector<PointWithCurvature> PointMeanCurvatures;		//Mean Curvature for each point (k1+k2)/2	
	vector<int> OptimalK;							//每个点的最佳最近邻个数
	vector<float> OptimalK_MaxDis;					//每个点的最佳最近邻个数下的最大距离
	vector<int> OptimalK_MaxDisIndex;				//每个点的最佳最近邻个数下的最大距离
	vector<float> OptimalK_MinDis;					//每个点的最佳最近邻个数下的最大距离
	vector<int> OptimalK_MinDisIndex;				//每个点的最佳最近邻个数下的最大距离	
	vector<Eigen::Vector3f> Optimal_EigenVectors;	//每个点的最佳最近邻个数下得到的特征值
	
	vector<double> DisOfPointToCentroid;		//每个点到其邻域点集中心点的距离	2019.08.31
	vector<double> DisOfPointToTangentPlane;	//每个点到其邻域点集中心点构成切平面的距离	2019.08.31

	//2019.10.03  每个点的几何特征
	vector< PointGeometryFeature> PointGeometryFeatures;

	CPointGeometry();
	~CPointGeometry();

	double CurvatureMean, CurvatureStdError;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr EigenValues;

	pcl::PointCloud<pcl::Normal>::Ptr Cloud_Normals;	// Normal vector for each point	//每个点的最佳最近邻个数下得到的法向量
	pcl::PointCloud<pcl::Normal>::Ptr DoubleCloud_Normals;	// Normal vector for each point	//每个点的最佳最近邻个数 * 2 下得到的法向量
	pcl::PointCloud<pcl::Normal>::Ptr HalfCloud_Normals;	// Normal vector for each point	//每个点的最佳最近邻个数 / 2 下得到的法向量

	//2019.09.26 最佳近邻下的平均法向
	pcl::PointCloud<pcl::Normal>::Ptr AvgNormalOfNeighbour;

	pcl::PointCloud<pcl::Normal>::Ptr Cloud_NormalsOfLargestEigenValue;	// Normal vector for each point	//每个点的最佳最近邻个数下得到的法向量

	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr Cloud_Curvatures;		//Principal Curvatures for each point		

	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPointPtr);
		
	void CalcNormalAndCurvatureRadius(double RadiusValue,bool IsWeighted = false);

	void CalcNormalAndCurvatureKNN(int k, pcl::PointCloud<pcl::Normal>::Ptr NormalsValue, bool IsWeighted = false);

	void CalcDirectionRadius(double RadiusValue, pcl::PointCloud<pcl::Normal>::Ptr NormalsValue, bool Max = true);	

	void CalcDirectionKNN(int k, pcl::PointCloud<pcl::Normal>::Ptr NormalsValue, bool Max = true);

	void CalcEigenValueRateRadius(double RadiusValue, vector<double> * Rates, int Type = 1, bool IsWeighted = false);

	void CalcEigenValueRateKNN(int k, vector<double> * Rates, int Type = 1, bool IsWeighted = false);

	pcl::PrincipalCurvatures GetCurvature(int PointIndex);

	pcl::Normal GetNormal(int PointIndex);
	pcl::Normal GetDirectionOfPrincipalCurvature(int PointIndex);	

	void GetDirectionOfPrincipalCurvatures(pcl::PointCloud<pcl::Normal>::Ptr CurvatureDirections);

	float GetGuassCurvature(int PointIndex);
	float GetMeanCurvature(int PointIndex);
	
	//get Maxest GuassCurvature
	vector<int> GetMaxGuassCurvature(int MaxNum);

	vector<int> GetMinGuassCurvature(int MaxNum);

	void SetOptimalK(int K);

	//计算每个点的最佳近邻个数
	void CalcOptimalK();	

	//只计算每个点在最近邻域个数下的法向量 2020.02.26
	void CalcOptimalNormal();

	//计算每个点 最佳近邻个数下的法向 和曲率 以及其它各种法向量(半邻域及二倍邻域等环境下的法向量)
	void CalcOptimalNormalWithMore(bool IsWeighted = false);

	//根据给定点的索引及该点给定的内部点 设置点云对象的法向方向。
	void SetNormalConsistency(int PointIndex, pcl::PointXYZRGB InteriorPoint, double RadiusValue = 0);

	//根据法向计算每个点的曲率
	void CalcOptimalCurvature();	

	//根据 OptimalK 计算的结果，得到最远点的距离MaxDis，然后以 Scale*MaxDis 为半径计算法向 2019.09.04
	void CalcScaleNormalByOptimalK(double Scale, pcl::PointCloud<pcl::Normal>::Ptr Normals);

	//Normal estimator based on the publication "Fast and Robust Normal Estimator for Point Clouds"
	void CalcNormalByFRNE();

	//计算某一个点的最佳近邻
	int CalcOptimalKForPoint(int PointIndex, int StartK, int EndK);

	int GetOptimalK(int PointIndex);

	void GetOptimalNeighbourIndex(int PointIndex, vector<int> & Indexs);

	float GetOptimalK_MaxDis(int PointIndex);

	float GetOptimalK_MinDis(int PointIndex);

	Eigen::Vector3f CalcEigenValueForPoint(int PointIndex, int k);

	//2019.09.24 获取高斯曲率变化比较大的点，该点的曲率值显著高于邻域点的曲率值
	void GetCurvatureVariationSignificantly(vector<int> & IndexS);

	//2019.09.29 根据曲率的均值与标准差 计算每一个曲率异常点到均值点的路径		
	void SearchCurvaturePathPoints(vector<int> SliceIndex, vector<int> & RedIndexs, 
		vector<int> & BlueIndexs);
	   
	//2019.10.03 计算最优邻域下每个点的几何特征
	void CalcOptimalPointGeometryFeatures();
};