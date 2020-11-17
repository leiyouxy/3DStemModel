#pragma once

/*

2019.05.22 

Remove Tree outliers by point Geometrical Feature by neighour points.

�ܶȾ����ԣ�(abs(max-avg)+abs(min-avg)),ԽСԽ��
����ƫ���ԣ�dis(p_projection, p_centroid),ԽСԽ��
����ͻ���ԣ����ʼ�Ĳ��,ԽСԽ��

*/

#include "Commdefinitions.h"
#include "CommPointBase.h"
#include "Source/TreeBase.h"
#include <vector>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/search/search.h>
#include "PointCurvature.h"

class  CTreeOutliersRemoval : public CTreeBase
{
public:
	CTreeOutliersRemoval();
	~ CTreeOutliersRemoval();

	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPointPtr, int RadiusValue);

	// calc GeometryFeature for points one by one
	void CalcGeometryFeature();
private:
	int Radius;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree;

	CPointCurvature PointCurvature;

	vector<PointGeometryFeature> PointGeometryFeatures;

	// calc GeometryFeature for current point
	void CalcPointGeometryFeature(PointGeometryFeature & PGF);
};

