#pragma once
#include "nlopt.hpp"
#include "Commdefinitions.h"
#include "CommPointBase.h"
#include "CommGeometry.h"

//MovePoints move result point to get the smallest distance sum for each 

double vfunc(const std::vector<double> &x, std::vector<double> &grad, void* f_data);

class CTwoPointsSetMatching
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr FixPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MovePoints;

	pcl::PointXYZRGB MovePoint;
	pcl::PointXYZRGB NormalPoint;	

public:
	//nlopt::opt opt();
	//nlopt::opt opt(nlopt::, 3);
	vector<double> Optimal();
	void SetInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr FixPointsValue,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr MovePointsValue, pcl::PointXYZRGB NormalPointValue);

	double objf(const std::vector<double >  &x, std::vector<double> &grad, void* f_data);
};

//pcl::PointXYZRGB GetTwoPointsSetMatching(pcl::PointCloud<pcl::PointXYZRGB>::Ptr FixPoints, 
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MovePoints, pcl::PointXYZRGB NormalPoint)
//{
//	nlopt::opt opt(nlopt::LD_MMA, 3);
//	std::vector<double> lb(3);
//	std::vector<double> ub(3);
//
//	double XMax, XMin, YMax, YMin, ZMax, ZMin;
//
//	PointBase::GetPointsMaxAndMin(FixPoints, XMax, XMin, YMax, YMin, ZMax, ZMin);
//
//	pcl::PointXYZRGB MoveCentroid = GeometryBase::GetCentroidOfPointsInSpaces(MovePoints, NormalPoint);
//	lb[0] = XMin, ub[0] = XMax, lb[1] = YMin, ub[1] = YMax, lb[2] = ZMin, ub[2] = ZMax;
//
//	opt.set_lower_bounds(lb);
//	opt.set_upper_bounds(ub);
//
//	opt.set_min_objective();
//
//
//	opt.set_ftol_rel(ReconstructorPara::MMA_FTOL);
//}