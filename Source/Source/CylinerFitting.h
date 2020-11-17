/*


Cylinder Fitting For 3D points

The method is origned from ApprCylinder3.h in Geometric Tools Engine (GTEngine)
(https://www.geometrictools.com)
(https://www.geometrictools.com/Source/Approximation.html)
(https://www.geometrictools.com/GTE/Mathematics/ApprCylinder3.h)
*/

#include "CommGeometry.h"
#include "Eigen/src/Core/EigenBase.h"

#pragma once

class CCylinderFitting
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingPoints;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MeanPoints;

// Preprocessed information that depends only on the sample points.
// This allows precomputed summations so that G(...) can be evaluated
// extremely fast.
	Eigen::Matrix<double, 1, 6> mMu;
	Eigen::Matrix<double, 3, 3> mF0;
	Eigen::Matrix<double, 3, 6> mF1;
	Eigen::Matrix<double, 6, 6> mF2;

	double numPoints;
	double mInvNumPoints;

	pcl::PointXYZRGB Average;	

	void ProProcess();

	double G(Eigen::Matrix<double, 3, 1> const& W, Eigen::Matrix<double, 3, 1> & PC, double & rsqr);

	pcl::PointXYZRGB CalcAxisDirectionBySurfaceNormal();
public:
	CCylinderFitting();

	~CCylinderFitting();

	pcl::PointXYZRGB CalcAxisDirection();

	void SetInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingPointsValue);		

	double CylinderFitting(pcl::PointXYZRGB Direction, pcl::PointXYZRGB & CenterPoint, double & R, double & H);

};