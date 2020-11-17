#include "CommOptimal.h"

double vfunc(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
	CTwoPointsSetMatching * p_TwoPointsSetMatching = (CTwoPointsSetMatching *)f_data;

	double fx = p_TwoPointsSetMatching->objf(x, grad, p_TwoPointsSetMatching);

	return  fx;
}

vector<double> CTwoPointsSetMatching::Optimal()
{
	//nlopt::opt opt(nlopt::GN_ESCH, 3);
	//nlopt::opt opt(nlopt::GN_ISRES, 3);
	
	nlopt::opt opt(nlopt::GN_DIRECT_L_RAND, 3); //可用 但效果不理想
	
	std::vector<double> lb(3);
	std::vector<double> ub(3);	

	double XMax, XMin, YMax, YMin, ZMax, ZMin;
	
	PointBase::GetPointsMaxAndMin(FixPoints, XMax, XMin, YMax, YMin, ZMax, ZMin);
	//
	//pcl::PointXYZRGB FixCentroid =
	//	GeometryBase::GetCentroidOfPointsInSpaces(FixPoints, NormalPoint);

	pcl::PointXYZRGB MoveCentroid = 
		GeometryBase::GetCentroidOfPointsInSpaces(MovePoints, NormalPoint);
	//PointsMove(MovePoints, MoveCentroid.x - FixCentroid.x, 
	//	MoveCentroid.y - FixCentroid.y, MoveCentroid.z - FixCentroid.z);

	vector<double> x{ 0,0,0 };	

	//lb[0] = (MoveCentroid.x - XMax) / 2.0, ub[0] = (MoveCentroid.x - XMin) / 2.0;
	//lb[1] = (MoveCentroid.y - YMax) / 2.0, ub[1] = (MoveCentroid.y - YMin) / 2.0;
	//lb[2] = (MoveCentroid.z - ZMax) / 2.0, ub[2] = (MoveCentroid.z - ZMin) / 2.0;

	lb[0] = (XMin - MoveCentroid.x) / 2.0, ub[0] = (XMax - MoveCentroid.x) / 2.0;
	lb[1] = (YMin - MoveCentroid.y) / 2.0, ub[0] = (YMax - MoveCentroid.y) / 2.0;
	lb[2] = (ZMin - MoveCentroid.z) / 2.0, ub[0] = (ZMax - MoveCentroid.z) / 2.0;

	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	opt.set_min_objective(vfunc, this);
	opt.set_xtol_rel(0.1);
	opt.set_force_stop(0.1);

	double f_Min;
	cout << endl << "Start optimize" << endl;
	opt.optimize(x, f_Min);	
	cout << "f_Min:" << f_Min << endl;
	return x;
}

void CTwoPointsSetMatching::SetInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr FixPointsValue,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MovePointsValue, pcl::PointXYZRGB NormalPointValue)
{
	FixPoints = FixPointsValue;
	MovePoints = MovePointsValue;
	NormalPoint = NormalPointValue;
}

double CTwoPointsSetMatching::objf(const std::vector<double> &x, 
	std::vector<double> &grad, void* f_data)
{
	//pcl::PointXYZRGB MovePoint;
	//MovePoint.x = x[0], MovePoint.x = x[1], MovePoint.x = x[2];		
	PointsMove(MovePoints, x[0], x[1], x[2]);

	double DisOfTwoPointsSet = 0;
	for (int i = 0; i < FixPoints->points.size(); i++)
	{
		double MinValue = 1000;		
		int SmallIndexJ = -1;
		for (int j = 0; j < MovePoints->points.size(); j++)
		{
			double Dis = PointDis(FixPoints->points[i], MovePoints->points[j]);
			
			MinValue = Dis < MinValue ? Dis: MinValue;
			SmallIndexJ = j;
		}
		DisOfTwoPointsSet += pow(MinValue,2);
	}

	return DisOfTwoPointsSet;
}