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

	//2019.10.3 ����ÿ����ļ�������
	void CalcPointGeometryFeature(PointGeometryFeature & PGF);
public:
	//vector<PointWithCurvature> PointGuassCurvatures;		//Guass Curvature for each point k1*k2
	//vector<PointWithCurvature> PointMeanCurvatures;		//Mean Curvature for each point (k1+k2)/2	
	vector<int> OptimalK;							//ÿ������������ڸ���
	vector<float> OptimalK_MaxDis;					//ÿ������������ڸ����µ�������
	vector<int> OptimalK_MaxDisIndex;				//ÿ������������ڸ����µ�������
	vector<float> OptimalK_MinDis;					//ÿ������������ڸ����µ�������
	vector<int> OptimalK_MinDisIndex;				//ÿ������������ڸ����µ�������	
	vector<Eigen::Vector3f> Optimal_EigenVectors;	//ÿ������������ڸ����µõ�������ֵ
	
	vector<double> DisOfPointToCentroid;		//ÿ���㵽������㼯���ĵ�ľ���	2019.08.31
	vector<double> DisOfPointToTangentPlane;	//ÿ���㵽������㼯���ĵ㹹����ƽ��ľ���	2019.08.31

	//2019.10.03  ÿ����ļ�������
	vector< PointGeometryFeature> PointGeometryFeatures;

	CPointGeometry();
	~CPointGeometry();

	double CurvatureMean, CurvatureStdError;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr EigenValues;

	pcl::PointCloud<pcl::Normal>::Ptr Cloud_Normals;	// Normal vector for each point	//ÿ������������ڸ����µõ��ķ�����
	pcl::PointCloud<pcl::Normal>::Ptr DoubleCloud_Normals;	// Normal vector for each point	//ÿ������������ڸ��� * 2 �µõ��ķ�����
	pcl::PointCloud<pcl::Normal>::Ptr HalfCloud_Normals;	// Normal vector for each point	//ÿ������������ڸ��� / 2 �µõ��ķ�����

	//2019.09.26 ��ѽ����µ�ƽ������
	pcl::PointCloud<pcl::Normal>::Ptr AvgNormalOfNeighbour;

	pcl::PointCloud<pcl::Normal>::Ptr Cloud_NormalsOfLargestEigenValue;	// Normal vector for each point	//ÿ������������ڸ����µõ��ķ�����

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

	//����ÿ�������ѽ��ڸ���
	void CalcOptimalK();	

	//ֻ����ÿ�����������������µķ����� 2020.02.26
	void CalcOptimalNormal();

	//����ÿ���� ��ѽ��ڸ����µķ��� ������ �Լ��������ַ�����(�����򼰶�������Ȼ����µķ�����)
	void CalcOptimalNormalWithMore(bool IsWeighted = false);

	//���ݸ�������������õ�������ڲ��� ���õ��ƶ���ķ�����
	void SetNormalConsistency(int PointIndex, pcl::PointXYZRGB InteriorPoint, double RadiusValue = 0);

	//���ݷ������ÿ���������
	void CalcOptimalCurvature();	

	//���� OptimalK ����Ľ�����õ���Զ��ľ���MaxDis��Ȼ���� Scale*MaxDis Ϊ�뾶���㷨�� 2019.09.04
	void CalcScaleNormalByOptimalK(double Scale, pcl::PointCloud<pcl::Normal>::Ptr Normals);

	//Normal estimator based on the publication "Fast and Robust Normal Estimator for Point Clouds"
	void CalcNormalByFRNE();

	//����ĳһ�������ѽ���
	int CalcOptimalKForPoint(int PointIndex, int StartK, int EndK);

	int GetOptimalK(int PointIndex);

	void GetOptimalNeighbourIndex(int PointIndex, vector<int> & Indexs);

	float GetOptimalK_MaxDis(int PointIndex);

	float GetOptimalK_MinDis(int PointIndex);

	Eigen::Vector3f CalcEigenValueForPoint(int PointIndex, int k);

	//2019.09.24 ��ȡ��˹���ʱ仯�Ƚϴ�ĵ㣬�õ������ֵ������������������ֵ
	void GetCurvatureVariationSignificantly(vector<int> & IndexS);

	//2019.09.29 �������ʵľ�ֵ���׼�� ����ÿһ�������쳣�㵽��ֵ���·��		
	void SearchCurvaturePathPoints(vector<int> SliceIndex, vector<int> & RedIndexs, 
		vector<int> & BlueIndexs);
	   
	//2019.10.03 ��������������ÿ����ļ�������
	void CalcOptimalPointGeometryFeatures();
};