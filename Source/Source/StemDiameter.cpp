#include "StemDiameter.h"

CStemDiameter::CStemDiameter()
{
	CenterPoint.x = 0, CenterPoint.y = 0, CenterPoint.z = 0;
	XYCenterPoint.x = 0, XYCenterPoint.y = 0, XYCenterPoint.z = 0;
	XYPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	XYConvexhullPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

CStemDiameter::~CStemDiameter()
{
	XYPoints->points.clear();
	XYConvexhullPoints->points.clear();	
}

void CStemDiameter::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr MeasurePointsValue,	
	pcl::PointXYZRGB CenterPointValue,
	pcl::PointXYZRGB NormalPointValue)
{
	InputCloud = MeasurePointsValue;
	CenterPoint = CenterPointValue;
	NormalPoint = NormalPointValue;

	//if (CenterPoint.x != 0 && CenterPoint.y != 0 && CenterPoint.z != 0)	
	//	InputCloud->points.push_back(CenterPoint);	

	//rotate points to XYPoints paralled to XY planes	
	//pcl::PointXYZRGB NormalPoint = GeometryBase::GetMaxDirectionVector(InputCloud);

	//此处需细细琢磨
	//bool Is2D = PointBase::PointsIs2D(InputCloud);
	//if (!Is2D)
	//{
	//	if (abs(NormalPoint.x) < EPSM6 && abs(NormalPoint.y) < EPSM6 && abs(NormalPoint.z - 1) < EPSM6)			
	//	{
	//		//GeometryBase::RotateNormalToVertical(InputCloud, XYPoints, NormalPoint);
	//		PointBase::PointCopy(InputCloud, XYPoints);
	//		PointBase::SetPointsCoordinateValue(XYPoints, "Z", InputCloud->points[0].z);
	//	}
	//	else if (abs(NormalPoint.x) < EPSM6 && abs(NormalPoint.y) < EPSM6 && abs(NormalPoint.z) < EPSM6)
	//	{
	//		pcl::PointXYZRGB TempNormalPoint = GeometryBase::GetMinDirectionVector(InputCloud);
	//		//GeometryBase::RotatePointsToParallelOfXY(InputCloud, XYPoints);
	//		GeometryBase::RotateNormalToVertical(InputCloud, XYPoints, TempNormalPoint);
	//	}
	//	else
	//		GeometryBase::RotateNormalToVertical(InputCloud, XYPoints, NormalPoint);
	//}
	//else
	//{
	//	XYPoints->points.clear();
	//	XYPoints->points.insert(XYPoints->points.end(), InputCloud->points.begin(), InputCloud->points.end());
	//}

	//如果点集法向是垂直于水平面的情况下
	PointBase::PointCopy(InputCloud, XYPoints);
	PointBase::SetPointsCoordinateValue(XYPoints, "Z", XYPoints->points[0].z);
	PointBase::GetDistinctPoints(XYPoints);
	XYCenterPoint = CenterPoint;

	if (NormalPoint.x == 0 && NormalPoint.y == 0 && (NormalPoint.z == 1 || NormalPoint.z == 0))
	{
		XYCenterPoint.z = XYPoints->points[0].z;
	}
	else//需后续完善 2019.06.06
	{
		XYCenterPoint = CenterPoint;
	}
	//由于去除重复点后会导致 XYPoints 点集变小，所以下面语句会出错

	//if (CenterPoint.x != 0 && CenterPoint.y != 0 && CenterPoint.z != 0)
	//{
	//	XYCenterPoint = XYPoints->points[InputCloud->points.size() - 1];
	//	InputCloud->points.pop_back();
	//	XYPoints->points.pop_back();
	//}

	CContourAndConvexHull<pcl::PointXYZRGB> ConvexHull;
	vector<int> ConvexhullIndexs;
	ConvexHull.SetInputs(XYPoints);
	ConvexhullIndexs.clear();
	ConvexHull.GetPointsConvexHull(ConvexhullIndexs);

	XYConvexhullPoints->points.clear();
	for (int j = 0; j < ConvexhullIndexs.size(); j++)
	{
		XYConvexhullPoints->points.push_back(XYPoints->points[ConvexhullIndexs[j]]);
	}
}

//Return value is D value, not radius value
double CStemDiameter::DRetrievalByCircleFittingIn2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePoints)
{
	return GeometryBase::CircleFittingByLeastSquaresFitting(XYPoints) * 2.0;
}

////Return value is D value, not radius value
//double CStemDiameter::DRetrievalByBezierCurve()
//{
//	CBezierSubsectionG2 BezierSubsectionG2;
//	//BezierSubsectionG2.Viewer = Viewer;
//	BezierSubsectionG2.SetInputs(XYConvexhullPoints);
//	//The tension parameter 
//	BezierSubsectionG2.ResolveControlPoints(BezierMiu);
//	double SplineLength = BezierSubsectionG2.GetBezierLength();
//	return SplineLength / M_PI;
//}

//Return value is D value, not radius value and get CircleCenter coordinate
double CStemDiameter::DRetrievalByCircleFittingIn2D(pcl::PointXYZRGB & CircleCenter)
{
	return GeometryBase::CircleFittingByLeastSquaresFitting(XYPoints, CircleCenter);
}

//
double CStemDiameter::DRetrievalByConvexHullLine()
{
	double perimeter = 0;
	int n = XYConvexhullPoints->points.size();
	for (int i = 0; i < n; i++)
	{
		perimeter = perimeter + PointDis(XYConvexhullPoints->points[i], XYConvexhullPoints->points[(i + 1) % n]);
	}
	return perimeter / M_PI;
}

//Return value is D value, not radius value and get Bezier curve points
double CStemDiameter::DRetrievalByBezierCurve(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePoints)
{
	if (XYConvexhullPoints->points.size() < 3) return 0;

	CBezierSubsectionG2 BezierSubsectionG2;
	//BezierSubsectionG2.Viewer = Viewer;
	BezierSubsectionG2.SetInputs(XYConvexhullPoints); 

	//The tension parameter 
	BezierSubsectionG2.ResolveControlPoints(BezierMiu);
	double SplineLength = BezierSubsectionG2.GetBezierLength();
	if (CurvePoints != NULL)
	{
		CurvePoints->points.clear();

		if (CenterPoint.x != 0 && CenterPoint.y != 0 && CenterPoint.z != 0
			&& NormalPoint.x != 0 && NormalPoint.y != 0 && NormalPoint.z != 0)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
			BezierSubsectionG2.DrawBezierCurve(TempPoints);

			TempPoints->points.push_back(XYCenterPoint);
			GeometryBase::RotateToOriginal(TempPoints, CurvePoints, NormalPoint);
			PointsMove(CurvePoints, CenterPoint.x - CurvePoints->points[CurvePoints->points.size() - 1].x,
				CenterPoint.y - CurvePoints->points[CurvePoints->points.size() - 1].y,
				CenterPoint.z - CurvePoints->points[CurvePoints->points.size() - 1].z);
			CurvePoints->points.pop_back();
		}
		else
		{
			BezierSubsectionG2.DrawBezierCurve(CurvePoints);
		}		
	}
	return SplineLength / M_PI;
}

//Return value is D value, not radius value
double CStemDiameter::DRetrievalBySpline(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePoints)
{
	if (XYConvexhullPoints->points.size() < 3) return 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> KnotValues;
	CSplineInterpolation SplineInterpolation;
	SplineInterpolation.SetInputs(XYConvexhullPoints, 3, true);		
	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnotValues);

	CSpline Spline;	
	Spline.SetSplineInputs(ControlPoints, 3, KnotValues, true);
	Spline.FirstPointOfCloseCurve = XYConvexhullPoints->points[0];
	Spline.CreateSpline();

	if (CurvePoints != NULL)
	{		
		CurvePoints->points.clear();

		if (CenterPoint.x != 0 && CenterPoint.y != 0 && CenterPoint.z != 0
			&& NormalPoint.x != 0 && NormalPoint.y != 0 && NormalPoint.z != 0)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

			TempPoints->points.insert(TempPoints->points.end(),
				Spline.CurvePoints->points.begin(),
				Spline.CurvePoints->points.end());
			TempPoints->points.push_back(XYCenterPoint);
			GeometryBase::RotateToOriginal(TempPoints, CurvePoints, NormalPoint);
			PointsMove(CurvePoints, CenterPoint.x - CurvePoints->points[CurvePoints->points.size()-1].x,
				CenterPoint.y - CurvePoints->points[CurvePoints->points.size() - 1].y,
				CenterPoint.z - CurvePoints->points[CurvePoints->points.size() - 1].z);
			CurvePoints->points.pop_back();
		}
		else 
		{
			CurvePoints->points.insert(CurvePoints->points.end(),
				Spline.CurvePoints->points.begin(),
				Spline.CurvePoints->points.end());
		}
	}
	return Spline.GetSplineLengthBySimpson() / M_PI;
}

//Return value is D value, not radius value
double CStemDiameter::DRetrievalByCylinerFittingIn3D(double & R, double & H, pcl::PointXYZRGB Direction)
{
	pcl::PointXYZRGB CenterPoint;	
	
	if (abs(Direction.x) < EPSM6 && abs(Direction.y) < EPSM6 && abs(Direction.z) < EPSM6)
		Direction = GeometryBase::GetCylinderDirection(InputCloud);

	CCylinderFitting CylinderFitting;
	CylinderFitting.SetInput(InputCloud);
	CylinderFitting.CylinderFitting(Direction, CenterPoint, R, H);

	return 2 * R;
}

double CStemDiameter::DRetrievalByCylinerFittingIn3D(pcl::PointXYZRGB Direction, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePoints)
{
	if (abs(Direction.x) < EPSM6 && abs(Direction.y) < EPSM6 && abs(Direction.z) < EPSM6)
		Direction = GeometryBase::GetCylinderDirection(InputCloud);

	pcl::PointXYZRGB CenterPoint;
	double R, H;
	CCylinderFitting CylinderFitting;
	CylinderFitting.SetInput(InputCloud);
	CylinderFitting.CylinderFitting(Direction, CenterPoint, R, H);

	if (CurvePoints != NULL)
	{
		GeometryBase::GetCylinderPoints(CurvePoints, CenterPoint, Direction, R, H);
		PointBase::SetPointColor(CurvePoints, ColorBase::RedColor);
	}

	return 2 * R;
}

/*

Class for calc Basal area by profile curve
leiyou
2019.01.17

*/

CStemDiameterAndProfileCurveBasalArea::CStemDiameterAndProfileCurveBasalArea()
{
	//CStemDiameter::CStemDiameter();
	ProfilePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

CStemDiameterAndProfileCurveBasalArea::~CStemDiameterAndProfileCurveBasalArea()
{
	ProfilePoints->points.clear();	
}

//Input is Profile curve points pointer, and return value is basal area
double CStemDiameterAndProfileCurveBasalArea::GetProfileCurveAndArea(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempXYPoints,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurvePoints, 
	double Angle)
{
	CAnglePartition AnglePartition;
	AnglePartition.AngleSample(TempXYPoints, ProfilePoints, Angle);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	CSplineInterpolation SplineInterpolation;
	vector<double> KnotValues;
	SplineInterpolation.SetInputs(ProfilePoints, 3, true);
	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnotValues);
		
	Spline.SetSplineInputs(ControlPoints, 3, KnotValues, true);
	Spline.FirstPointOfCloseCurve = ProfilePoints->points[0];
	Spline.CreateSpline();
	ProfileCurvePoints->points.clear();
	ProfileCurvePoints->points.insert(ProfileCurvePoints->points.end(),
		Spline.CurvePoints->points.begin(), Spline.CurvePoints->points.end());
	return Spline.GetSplineArea();
}

//return value is basal area
double CStemDiameterAndProfileCurveBasalArea::GetProfileCurveAndArea(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfilePointsValue)
{
	//ProfilePoints = ProfilePointsValue;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> KnotValues;

	CSplineInterpolation SplineInterpolation;
	SplineInterpolation.SetInputs(ProfilePointsValue, 3, true);
	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnotValues);
		
	Spline.SetSplineInputs(ControlPoints, 3, KnotValues, true);
	Spline.FirstPointOfCloseCurve = ProfilePointsValue->points[0];
	Spline.CreateSpline();
	return Spline.GetSplineArea();
}

double CStemDiameterAndProfileCurveBasalArea::GetProfileCurveAndArea(double Angle,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurvePoints)
{
	CAnglePartition AnglePartition;
	AnglePartition.AngleSample(XYPoints, ProfilePoints, Angle);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	CSplineInterpolation SplineInterpolation;
	vector<double> KnotValues;
	SplineInterpolation.SetInputs(ProfilePoints, 3, true);
	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnotValues);
		
	Spline.SetSplineInputs(ControlPoints, 3, KnotValues, true);
	Spline.FirstPointOfCloseCurve = ProfilePoints->points[0];
	Spline.CreateSpline();
	if (ProfileCurvePoints != NULL)
	{	
		ProfileCurvePoints->points.clear();
		if (CenterPoint.x != 0 && CenterPoint.y != 0 && CenterPoint.z != 0
			&& NormalPoint.x != 0 && NormalPoint.y != 0 && NormalPoint.z != 0)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

			TempPoints->points.insert(TempPoints->points.end(),
				Spline.CurvePoints->points.begin(),
				Spline.CurvePoints->points.end());
			TempPoints->points.push_back(XYCenterPoint);
			GeometryBase::RotateToOriginal(TempPoints, ProfileCurvePoints, NormalPoint);
			PointsMove(ProfileCurvePoints, CenterPoint.x - ProfileCurvePoints->points[ProfileCurvePoints->points.size() - 1].x,
				CenterPoint.y - ProfileCurvePoints->points[ProfileCurvePoints->points.size() - 1].y,
				CenterPoint.z - ProfileCurvePoints->points[ProfileCurvePoints->points.size() - 1].z);
			ProfileCurvePoints->points.pop_back();
		}
		else
		{
			ProfileCurvePoints->points.insert(ProfileCurvePoints->points.end(),
				Spline.CurvePoints->points.begin(), Spline.CurvePoints->points.end());
		}
	}
	return Spline.GetSplineArea();
}