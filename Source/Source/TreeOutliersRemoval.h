#pragma once

/*

2019.05.22 

Remove Tree outliers by point Geometrical Feature by neighour points.

密度均匀性：(abs(max-avg)+abs(min-avg)),越小越好
重心偏向性：dis(p_projection, p_centroid),越小越好
几何突变性：曲率间的差距,越小越好

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

