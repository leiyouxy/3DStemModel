#include "CylinerFitting.h"

CCylinderFitting::CCylinderFitting()
{
	MeanPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
}

CCylinderFitting::~CCylinderFitting()
{
	MeanPoints->points.clear();
}

void CCylinderFitting::ProProcess()
{
	numPoints = FittingPoints->points.size();
	MeanPoints->points.resize(FittingPoints->points.size());
	mInvNumPoints = 1.0 / numPoints;

	Average.x = 0, Average.y = 0, Average.z = 0;
	// Copy the points and translate by the average for numerical robustness.
	for (unsigned int i = 0; i < numPoints; ++i)
	{
		Average.x += FittingPoints->points[i].x * mInvNumPoints;
		Average.y += FittingPoints->points[i].y * mInvNumPoints;
		Average.z += FittingPoints->points[i].z * mInvNumPoints;
	}

	for (unsigned int i = 0; i < numPoints; ++i)
	{
		MeanPoints->points[i].x = FittingPoints->points[i].x - Average.x;
		MeanPoints->points[i].y = FittingPoints->points[i].y - Average.y;
		MeanPoints->points[i].z = FittingPoints->points[i].z - Average.z;
	}

	Eigen::Matrix<double, 1, 6> zero;
	zero.setZero();
	//std::vector<Eigen::Matrix<double, 1, 6>> products(numPoints, zero);
	//Eigen::MatrixXd products
	Eigen::MatrixXd products = Eigen::MatrixXd::Zero(numPoints, 6); //默认是0矩阵

	mMu = zero;
	for (size_t i = 0; i < numPoints; ++i)
	{
		products(i,0) = double(MeanPoints->points[i].x) * double(MeanPoints->points[i].x);
		products(i,1) = MeanPoints->points[i].x * MeanPoints->points[i].y;
		products(i,2) = MeanPoints->points[i].x * MeanPoints->points[i].z;
		products(i,3) = MeanPoints->points[i].y * MeanPoints->points[i].y;
		products(i,4) = MeanPoints->points[i].y * MeanPoints->points[i].z;
		products(i,5) = MeanPoints->points[i].z * MeanPoints->points[i].z;
		mMu[0] += products(i, 0);
		mMu[1] += 2 * products(i, 1);
		mMu[2] += 2 * products(i, 2);
		mMu[3] += products(i, 3);
		mMu[4] += 2 * products(i, 4);
		mMu[5] += products(i, 5);
	}
	mMu *= mInvNumPoints;

	mF0.setZero();
	mF1.setZero();
	mF2.setZero();

	for (size_t i = 0; i < numPoints; ++i)
	{
		Eigen::Matrix<double, 1, 6>  delta;
		delta(0, 0) = products(i, 0) - mMu[0];
		delta(0, 1) = 2 * products(i, 1) - mMu[1];
		delta(0, 2) = 2 * products(i, 2) - mMu[2];
		delta(0, 3) = products(i, 3) - mMu[3];
		delta(0, 4) = 2 * products(i, 4) - mMu[4];
		delta(0, 5) = products(i, 5) - mMu[5];

		mF0(0, 0) += products(i, 0);
		mF0(0, 1) += products(i, 1);
		mF0(0, 2) += products(i, 2);
		mF0(1, 1) += products(i, 3);
		mF0(1, 2) += products(i, 4);
		mF0(2, 2) += products(i, 5);

		Eigen::Matrix<double, 3, 1> temp(MeanPoints->points[i].x, MeanPoints->points[i].y, MeanPoints->points[i].z);

		mF1 += temp * delta;
		mF2 += delta.transpose() * delta;
	}
	mF0 *= mInvNumPoints;
	//mF0 = mF0.transpose();
	mF0(1, 0) = mF0(0, 1);
	mF0(2, 0) = mF0(0, 2);
	mF0(2, 1) = mF0(1, 2);
	mF1 *= mInvNumPoints;
	mF2 *= mInvNumPoints;
}

// U*V^T, U is NumRows-by-1, V is Num-Cols-by-1, result is NumRows-by-NumCols.
//template <int NumRows, int NumCols, typename Real>
//Matrix<NumRows, NumCols, Real>
//OuterProduct(Vector<NumRows, Real> const& U, Vector<NumCols, Real> const& V);

//double CCylinderFitting::ComputeUsingCovariance(Eigen::Matrix<double, 3, 1> & minPC,
//	Eigen::Matrix<double, 3, 1> & minW, double & minRSqr)
//{
//	//Computing Axis Direction;
//
//
//	//return G(minW, minPC, minRSqr);
//	return 0;
//}

double CCylinderFitting::G(Eigen::Matrix<double, 3, 1> const& W, 
		Eigen::Matrix<double, 3, 1> & PC, double & rsqr)
{
	Eigen::Matrix<double, 3, 3> P;// = Eigen::Matrix<double, 3, 3>::setIdentity() - W * W.transpose();
	P.setIdentity();
	P = P - W * W.transpose();
	Eigen::Matrix<double, 3, 3> S;
	S.setZero();
	S(0, 1) = -W[2];
	S(0, 2) = W[1];

	S(1, 0) = W[2];
	S(1, 2) = -W[0];

	S(2, 0) = -W[1];
	S(2, 1) = W[0];
	//{
	//	0, -W[2], W[1],
	//	W[2], 0, -W[0],
	//	-W[1], W[0], 0
	//};

	Eigen::Matrix<double, 3, 3> A = P * mF0 * P;
	Eigen::Matrix<double, 3, 3> hatA = -(S * A * S);
	Eigen::Matrix<double, 3, 3> hatAA = hatA * A;
	double trace = hatAA.trace();
	Eigen::Matrix<double, 3, 3> Q = hatA / trace;
	Eigen::Matrix<double, 6, 1> pVec;
	//{ P(0, 0), P(0, 1), P(0, 2), P(1, 1), P(1, 2), P(2, 2) };
	pVec(0, 0) = P(0, 0);
	pVec(1, 0) = P(0, 1);
	pVec(2, 0) = P(0, 2);
	pVec(3, 0) = P(1, 1);
	pVec(4, 0) = P(1, 2);
	pVec(5, 0) = P(2, 2);

	Eigen::Matrix<double, 3, 1> alpha = mF1 * pVec;
	Eigen::Matrix<double, 3, 1> beta = Q * alpha;
	//double GValue = (Dot(pVec, mF2 * pVec) - 4 * Dot(alpha, beta) + 4 * Dot(beta, mF0 * beta)) / numPoints;
	double GValue = (pVec.dot( mF2 * pVec) - 4 * alpha.dot(beta) + 4 * beta.dot(mF0 * beta)) / numPoints;
	PC = beta;
	rsqr = pVec.dot(mMu) + PC.dot(PC);
	return GValue;
}

void CCylinderFitting::SetInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingPointsValue)
{
	FittingPoints = FittingPointsValue;
}

pcl::PointXYZRGB CCylinderFitting::CalcAxisDirectionBySurfaceNormal()
{
	pcl::PointXYZRGB Direction;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> *Octree;
	Octree = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	Octree->setInputCloud(FittingPoints);
	Octree->addPointsFromInputCloud();

	vector<int> NeigbourIndex0, NeigbourIndexHalf, NeigbourIndexEnd;
	vector<float> NeigbourDis0, NeigbourDisHalf, NeigbourDisEnd;

	Octree->radiusSearch(FittingPoints->points.size() / 4, 1, NeigbourIndex0, NeigbourDis0);
	Octree->radiusSearch(FittingPoints->points.size() / 2, 1, NeigbourIndexHalf, NeigbourDisHalf);
	Octree->radiusSearch(FittingPoints->points.size() * 3 / 4, 1, NeigbourIndexEnd, NeigbourDisEnd);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points0(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsHalf(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsEnd(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < NeigbourIndex0.size(); i++)
	{
		Points0->points.push_back(FittingPoints->points[NeigbourIndex0[i]]);
	}

	for (int i = 0; i < NeigbourIndexHalf.size(); i++)
	{
		PointsHalf->points.push_back(FittingPoints->points[NeigbourIndexHalf[i]]);
	}

	for (int i = 0; i < NeigbourIndexEnd.size(); i++)
	{
		PointsEnd->points.push_back(FittingPoints->points[NeigbourIndexEnd[i]]);
	}

	pcl::PointXYZRGB Direction0, DirectionHalf, DirectionEnd;

	GeometryBase::GetPointsTangentPlane(Points0, Direction0);
	GeometryBase::GetPointsTangentPlane(PointsHalf, DirectionHalf);
	GeometryBase::GetPointsTangentPlane(PointsEnd, DirectionEnd);

	pcl::PointXYZRGB Result1 = PointBase::PointsCrossProduct(Direction0, DirectionHalf);
	pcl::PointXYZRGB Result2 = PointBase::PointsCrossProduct(Direction0, DirectionEnd);
	pcl::PointXYZRGB Result3 = PointBase::PointsCrossProduct(DirectionHalf, DirectionEnd);

	PointBase::PointNormalized(Result1);
	PointBase::PointNormalized(Result2);
	PointBase::PointNormalized(Result3);

	Direction.x = (Result1.x + Result2.x + Result3.x) / 3.0;
	Direction.y = (Result1.y + Result2.y + Result3.y) / 3.0;
	Direction.z = (Result1.z + Result2.z + Result3.z) / 3.0;

	//Result1 = GeometryBase::GetMaxDirectionVector(FittingPoints);

	return Direction;
}

//这种计算方法不准确，更准确的其实，各个角度分区的切平面法向量的平均值
pcl::PointXYZRGB CCylinderFitting::CalcAxisDirection()
{
	pcl::PointXYZRGB Direction = GeometryBase::GetCylinderDirection(FittingPoints);

	if (Direction.x == Direction.y && Direction.x ==  Direction.z & Direction.x == 0)
		Direction = CalcAxisDirectionBySurfaceNormal();
	return Direction;
}

double CCylinderFitting::CylinderFitting(pcl::PointXYZRGB Direction,
	pcl::PointXYZRGB & CenterPoint,
	double & R, double & H)
{
	ProProcess();

	Eigen::Matrix<double, 3, 1> W(Direction.x, Direction.y, Direction.z);
	Eigen::Matrix<double, 3, 1> PC;
	PC.setZero();
	double Error = G(W, PC, R);
	R = sqrt(R);

	CenterPoint.x = PC(0, 0) + Average.x;
	CenterPoint.y = PC(1, 0) + Average.y;
	CenterPoint.z = PC(2, 0) + Average.z;

	double tmin = EPSP9, tmax = 0;
	for (unsigned int i = 0; i < numPoints; ++i)
	{
		pcl::PointXYZRGB TempPoint;
		
		TempPoint.x = FittingPoints->points[i].x - Direction.x;
		TempPoint.y = FittingPoints->points[i].y - Direction.y;
		TempPoint.z = FittingPoints->points[i].z - Direction.z;
		
		double t = PointBase::PointsDotProduct(Direction, TempPoint);			
		tmin = std::min(t, tmin);
		tmax = std::max(t, tmax);
	}

	//cylinder.axis.origin += ((tmin + tmax) * 0.5) * cylinder.axis.direction;
	H = tmax - tmin;

	CenterPoint.x = CenterPoint.x - H / 2 * Direction.x;
	CenterPoint.y = CenterPoint.y - H / 2 * Direction.y;
	CenterPoint.z = CenterPoint.z - H / 2 * Direction.z;

	return Error;
}