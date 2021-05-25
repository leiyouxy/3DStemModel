#pragma once

/*

根据树干垂直分段的轮廓点采用基于切平面的表面重建算法先重建树干粗粒度的树干表面模型，
然后根据原始树干点与粗粒度树干表面间的几何位置关系重建树干细粒度的树干表面模型，此步具体做法为：
（原始树干点向距其最近的粗粒度的树干表面模型投影，
根据投影关系将粗粒度的树干表面的三角形剖分得到细粒度的树干表面模型，
其中有些树干点云可能要被舍弃，具体要看实验时遇到的具体问题）

2020.12.31

*/
#include "AnglePartition.h"
#include "DelaunayGrowthWithHalfEdge.h"

class CSurfaceReconstructionByTangentPlane
{
private:
	//树干点云的若干垂直分段的角度分区 2020.12.31
	CAnglePartition * p_AnglePartition;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OriginalStemPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfilePoints;

	V_HE_Face CoarseSurface; //粗粒度的树干表面模型，根据 ProfilePoints 采用切平面表面重建算法得到
	V_HE_Face FineSurface;  //细粒度的树干表面模型 由 OriginalStemPoints 中的部分点 确定的表面模型，
	
	CSurfaceReconstructionByTangentPlane();
	~CSurfaceReconstructionByTangentPlane();
public:
	void SetInput(CAnglePartition * p_AnglePartitionValue, 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputStemPoints);

};