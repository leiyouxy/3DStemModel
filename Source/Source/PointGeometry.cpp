#include "PointGeometry.h"

void CPointGeometry::CalcPointGeometryFeature(PointGeometryFeature & PGF)
{
	int MinIndex = -1, MaxIndex = -1;
	float MinDis = EPSP6, MaxDis = EPSM6;

	PGF.CentriodPoint.x = 0, PGF.CentriodPoint.y = 0, PGF.CentriodPoint.z = 0;

	PGF.AvgDis = 0;
	PGF.AvgGuassCurvatureChange = 0;
	for (int i = 0; i < PGF.NeighourIndexs.size(); i++)
	{
		if (PGF.NeighourDis[i] > MaxDis)
		{
			MaxDis = PGF.NeighourDis[i];
			MaxIndex = PGF.NeighourIndexs[i];
		}

		if (PGF.NeighourDis[i] < MinDis)
		{
			MinDis = PGF.NeighourDis[i];
			MinIndex = PGF.NeighourIndexs[i];
		}

		PGF.CentriodPoint.x = PGF.CentriodPoint.x + InputCloud->points[PGF.NeighourIndexs[i]].x;
		PGF.CentriodPoint.y = PGF.CentriodPoint.y + InputCloud->points[PGF.NeighourIndexs[i]].y;
		PGF.CentriodPoint.z = PGF.CentriodPoint.z + InputCloud->points[PGF.NeighourIndexs[i]].z;

		PGF.AvgDis = PGF.AvgDis + PGF.NeighourDis[i];
		PGF.AvgGuassCurvatureChange = PGF.AvgGuassCurvatureChange +
			abs(GetGuassCurvature(PGF.PointIndex)
				- GetGuassCurvature(PGF.NeighourIndexs[i]));
	}

	if (PGF.NeighourIndexs.size() > 0)
	{
		PGF.MaxDisNeighourIndex = MaxIndex;
		PGF.MinDisNeighourIndex = MinIndex;

		PGF.CentriodPoint.x = PGF.CentriodPoint.x / PGF.NeighourIndexs.size();
		PGF.CentriodPoint.y = PGF.CentriodPoint.y / PGF.NeighourIndexs.size();
		PGF.CentriodPoint.z = PGF.CentriodPoint.z / PGF.NeighourIndexs.size();

		PGF.AvgDis = PGF.AvgDis / PGF.NeighourIndexs.size();
		
		//PGF.DensityByDis =
		//	(abs(PGF.AvgDis -
		//		PointDis(InputCloud->points[PGF.PointIndex], InputCloud->points[PGF.MaxDisNeighourIndex]))
		//	+abs(PGF.AvgDis -
		//		PointDis(InputCloud->points[PGF.PointIndex], InputCloud->points[PGF.MinDisNeighourIndex])));
		PGF.DensityByDis =
			((PGF.AvgDis -
				PointDis(InputCloud->points[PGF.PointIndex], InputCloud->points[PGF.MaxDisNeighourIndex]))
				+ (PGF.AvgDis -
					PointDis(InputCloud->points[PGF.PointIndex], InputCloud->points[PGF.MinDisNeighourIndex])));

		PGF.AvgGuassCurvatureChange = PGF.AvgGuassCurvatureChange / PGF.NeighourIndexs.size();

		Eigen::Vector4f PlaneCoefficients;
		PGF.ProjectPlaneCoefficients[0] = PGF.PointNormal.normal_x;
		PGF.ProjectPlaneCoefficients[1] = PGF.PointNormal.normal_y;
		PGF.ProjectPlaneCoefficients[2] = PGF.PointNormal.normal_z;
		PGF.ProjectPlaneCoefficients[3] = -(PGF.ProjectPlaneCoefficients[0] * PGF.CentriodPoint.x
			+ PGF.ProjectPlaneCoefficients[1] * PGF.CentriodPoint.y + PGF.ProjectPlaneCoefficients[2] * PGF.CentriodPoint.z);

		pcl::projectPoint(InputCloud->points[PGF.PointIndex], 
			PGF.ProjectPlaneCoefficients, PGF.ProjectPoint);

		//点在切平面投影点到重心点的距离
		PGF.DisBetProjectAndCentriod = PointDis(PGF.ProjectPoint, PGF.CentriodPoint);

		//点在切平面投影点到点的距离
		PGF.DisBetProjectAndOriginal = PointDis(PGF.ProjectPoint, InputCloud->points[PGF.PointIndex]);
	}
}

CPointGeometry::CPointGeometry()
{
	Cloud_Curvatures.reset(new pcl::PointCloud<pcl::PrincipalCurvatures>());
	
	Cloud_Normals.reset(new pcl::PointCloud<pcl::Normal>());
	DoubleCloud_Normals.reset(new pcl::PointCloud<pcl::Normal>());
	HalfCloud_Normals.reset(new pcl::PointCloud<pcl::Normal>());

	AvgNormalOfNeighbour.reset(new pcl::PointCloud<pcl::Normal>());
	Cloud_NormalsOfLargestEigenValue.reset(new pcl::PointCloud<pcl::Normal>());
	EigenValues.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	OptimalK_a = 3;
	OptimalK_b = 100;

	
}

CPointGeometry::~CPointGeometry()
{
	if (Cloud_Curvatures != NULL)
		Cloud_Curvatures->points.clear();	
	
	if (Cloud_Normals != NULL)
		Cloud_Normals->points.clear();
	
	if (Cloud_NormalsOfLargestEigenValue != NULL)
		Cloud_NormalsOfLargestEigenValue->points.clear();

	//if (EigenValues != NULL)
	//	EigenValues->points.clear();
	//PointGuassCurvatures.clear();
	//PointMeanCurvatures.clear();
	OCtree = NULL;
	OctreeCloudSearch = NULL;
}

void CPointGeometry::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPointPtr)
{
	InputCloud = TempPointPtr;
	//Radius = RadiusValue;

	Cloud_Curvatures->points.clear();
	//PointGuassCurvatures.clear();
	//PointMeanCurvatures.clear();

	OCtree.reset(new pcl::search::Octree<pcl::PointXYZRGB>(0.1f));
	OCtree->setInputCloud(InputCloud);

	OctreeCloudSearch = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	OctreeCloudSearch->setInputCloud(InputCloud);
	OctreeCloudSearch->addPointsFromInputCloud();
}

void CPointGeometry::CalcDirectionRadius(double RadiusValue, pcl::PointCloud<pcl::Normal>::Ptr NormalsValue, bool Max)
{	
	NormalsValue->points.clear();

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		pcl::PointXYZRGB ResultDirection;		

		vector<int> NeighbourIndexs;
		vector<float> NeighbourDis;

		GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue, i, &NeighbourIndexs, &NeighbourDis);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp (new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int j = 0; j < NeighbourIndexs.size(); j++)
			Temp->points.push_back(InputCloud->points[NeighbourIndexs[j]]);

		if (Temp->points.size())
		{
			if (Max)
				ResultDirection = GeometryBase::GetMaxDirectionVector(Temp);
			else 
				ResultDirection = GeometryBase::GetMinDirectionVector(Temp);
		}

		pcl::Normal TempNormal(ResultDirection.x, ResultDirection.y, ResultDirection.z);

		NormalsValue->points.push_back(TempNormal);
	}
}

void CPointGeometry::CalcEigenValueRateRadius(double RadiusValue, vector<double> * Rates, int Type, bool IsWeighted)
{
	Rates->clear();
	//OctreeCloudSearch = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	//OctreeCloudSearch->setInputCloud(InputCloud);
	//OctreeCloudSearch->addPointsFromInputCloud();

	vector<double> WeightedRates;
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		Rates->push_back(0);
		EigenValues->points.push_back(pcl::PointXYZRGB());
	}

	#pragma omp parallel for
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		double Rate = 0;

		vector<int> NeighbourIndexs;
		vector<float> NeighbourDis;

		GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue, i, &NeighbourIndexs, &NeighbourDis);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int j = 0; j < NeighbourIndexs.size(); j++)
			Temp->points.push_back(InputCloud->points[NeighbourIndexs[j]]);

		if (Temp->points.size())
		{	
			Eigen::Matrix3f EigenVector;		//特征向量
			Eigen::Vector3f EigenValue;

			GeometryBase::GetPointsTangentPlane(Temp, EigenVector, EigenValue);

			EigenValues->points[i].x = EigenValue[0];
			EigenValues->points[i].y = EigenValue[1];
			EigenValues->points[i].z = EigenValue[2];

			if (Type == 1)
				Rate = EigenValue(2) / (EigenValue(0) + EigenValue(1) + EigenValue(2)); // 最大特质值越大 越近似于直线 
			else if (Type == 2)
				Rate = EigenValue(0) / (EigenValue(0) + EigenValue(1) + EigenValue(2)); // 最小特质值越小 越近似于平面 
			else if (Type == 3)
				Rate = EigenValue(1) / (EigenValue(2)); // 最大两个特征值越接近，越接近于圆或球 
		}

		(*Rates)[i] = Rate;
	}

	if (IsWeighted)
	{
		
		vector<double> WeightedRates;
		for (int i = 0; i < InputCloud->points.size(); i++)
			WeightedRates.push_back(0);

		#pragma omp parallel for
		for (int i = 0; i < InputCloud->points.size(); i++)
		{
			double WeightedRate = 0;
			vector<int> NeighbourIndexs;
			vector<float> NeighbourDis;
			GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue, i, &NeighbourIndexs, &NeighbourDis);
			
			if (NeighbourIndexs.size() > 0)
			{
				double TempRate = 0;
				for (int j = 0; j < NeighbourIndexs.size(); j++)
				{					
					TempRate += (*Rates)[NeighbourIndexs[j]];
				}
				TempRate += (*Rates)[i];

				WeightedRate = TempRate / (NeighbourIndexs.size() + 1);
			}
			WeightedRates[i] = WeightedRate;
		}

		Rates->clear();
		Rates->insert(Rates->begin(), WeightedRates.begin(), WeightedRates.end());
	}
}

void CPointGeometry::CalcDirectionKNN(int k, pcl::PointCloud<pcl::Normal>::Ptr NormalsValue, 
	bool Max)
{
	NormalsValue->points.clear();
	
	//OctreeCloudSearch = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	//OctreeCloudSearch->setInputCloud(InputCloud);
	//OctreeCloudSearch->addPointsFromInputCloud();

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		pcl::PointXYZRGB ResultDirection;

		vector<int> NeighbourIndexs;
		vector<float> NeighbourDis;

		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, k, i, &NeighbourIndexs, &NeighbourDis);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int j = 0; j < NeighbourIndexs.size(); j++)
			Temp->points.push_back(InputCloud->points[NeighbourIndexs[j]]);

		if (Temp->points.size())
		{
			if (Max)
				ResultDirection = GeometryBase::GetMaxDirectionVector(Temp);
			else
				ResultDirection = GeometryBase::GetMinDirectionVector(Temp);
		}

		pcl::Normal TempNormal(ResultDirection.x, ResultDirection.y, ResultDirection.z);

		NormalsValue->points.push_back(TempNormal);
	}
}


void CPointGeometry::CalcEigenValueRateKNN(int k, vector<double> * Rates, int Type, bool IsWeighted)
{
	Rates->clear();
	//OctreeCloudSearch = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	//OctreeCloudSearch->setInputCloud(InputCloud);
	//OctreeCloudSearch->addPointsFromInputCloud();

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		Rates->push_back(0);
		EigenValues->points.push_back(pcl::PointXYZRGB());
	}

	#pragma omp parallel for
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		double Rate = 0;

		vector<int> NeighbourIndexs;
		vector<float> NeighbourDis;

		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, k, i, &NeighbourIndexs, &NeighbourDis);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int j = 0; j < NeighbourIndexs.size(); j++)
			Temp->points.push_back(InputCloud->points[NeighbourIndexs[j]]);

		if (Temp->points.size())
		{
			Eigen::Matrix3f EigenVector;		//特征向量
			Eigen::Vector3f EigenValue;

			GeometryBase::GetPointsTangentPlane(Temp, EigenVector, EigenValue);

			EigenValues->points[i].x = EigenValue[0];
			EigenValues->points[i].y = EigenValue[1];
			EigenValues->points[i].z = EigenValue[2];

			if (Type == 1)
				Rate = EigenValue(2) / (EigenValue(0) + EigenValue(1) + EigenValue(2)); // 最大特质值越大 越近似于直线 
			else if (Type == 2)
				Rate = EigenValue(0) / (EigenValue(0) + EigenValue(1) + EigenValue(2)); // 最小特质值越小 越近似于平面 
			else if (Type == 3)
				Rate = EigenValue(1) / (EigenValue(2)); // 最大两个特征值越接近，越接近于圆或球 
		}
		(*Rates)[i] = Rate;
	}

	if (IsWeighted)
	{
		vector<double> WeightedRates;
		for (int i = 0; i < InputCloud->points.size(); i++)
			WeightedRates.push_back(0);

		#pragma omp parallel for
		for (int i = 0; i < InputCloud->points.size(); i++)
		{
			double WeightedRate = 0;
			vector<int> NeighbourIndexs;
			vector<float> NeighbourDis;
			GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, k, i, &NeighbourIndexs, &NeighbourDis);

			if (NeighbourIndexs.size() > 0)
			{
				double TempRate = 0;
				for (int j = 0; j < NeighbourIndexs.size(); j++)
				{
					TempRate += (*Rates)[NeighbourIndexs[j]];
				}
				TempRate += (*Rates)[i];

				WeightedRate = TempRate / (NeighbourIndexs.size() + 1);
			}
			WeightedRates[i] = WeightedRate;
		}

		Rates->clear();
		Rates->insert(Rates->begin(), WeightedRates.begin(), WeightedRates.end());
	}
}

//Normal estimator based on the publication "Fast and Robust Normal Estimator for Point Clouds"
void CPointGeometry::CalcNormalByFRNE()
{
	PCL_Normal_Estimator<pcl::PointXYZRGB, pcl::Normal> NE(InputCloud, Cloud_Normals);
	NE.number_of_planes() = 100;	
	///*!< Type of selection of normals (1: best, 2: cluster, default: mean)*/
	//NE.normal_selection_mode() = NORMAL_ESTIMATOR::MEAN;
	NE.estimate();
}

//根据 OptimalK 计算的结果，得到最远点的距离MaxDis，然后以 Scale*MaxDis 为半径计算法向 2019.09.04
void CPointGeometry::CalcScaleNormalByOptimalK(double Scale, pcl::PointCloud<pcl::Normal>::Ptr Normals)
{
	if (OptimalK.size() != InputCloud->points.size())
	{
		CalcOptimalK();
		CalcOptimalNormalWithMore(true);
	}
	
	Normals->points.clear();

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		Normals->points.push_back(pcl::Normal());
	}

	#pragma omp parallel for
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		double Radius = OptimalK_MaxDis[i] * Scale;

		vector<int> NeighbourIndexs;
		vector<float> NeighbourDiss;
		GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, Radius, i, &NeighbourIndexs, &NeighbourDiss);

		//没有自己则加入自己
		VectorBase<int> VectorBaseInt;
		int TempIndex = VectorBaseInt.FindIndexInVector(NeighbourIndexs, i);
		if (TempIndex == -1)		
			NeighbourIndexs.push_back(i);		

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr NeighourPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int j = 0; j < NeighbourIndexs.size(); j++)
			NeighourPoints->points.push_back(InputCloud->points[NeighbourIndexs[j]]);

		Eigen::Matrix3f EigenVector;		//特征向量
		Eigen::Vector3f EigenValue(3);
		pcl::PointXYZRGB CentroidPoint =
			GeometryBase::GetPointsTangentPlane(NeighourPoints, EigenVector, EigenValue);

		Normals->points[i].normal_x = EigenVector(0);
		Normals->points[i].normal_y = EigenVector(1);
		Normals->points[i].normal_z = EigenVector(2);
	}
}

//计算每个点 最佳近邻个数下的法向 和 曲率
void CPointGeometry::CalcOptimalNormalWithMore(bool IsWeighted)
{
	Cloud_Normals->points.clear();
	DoubleCloud_Normals->points.clear();
	HalfCloud_Normals->points.clear();

	Cloud_NormalsOfLargestEigenValue->points.clear();
	OptimalK_MaxDis.clear();
	OptimalK_MinDis.clear();
	Optimal_EigenVectors.clear();

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		Cloud_Normals->points.push_back(pcl::Normal());
		DoubleCloud_Normals->points.push_back(pcl::Normal());
		HalfCloud_Normals->points.push_back(pcl::Normal());

		AvgNormalOfNeighbour->points.push_back(pcl::Normal());
		Cloud_NormalsOfLargestEigenValue->points.push_back(pcl::Normal());
		
		OptimalK_MaxDis.push_back(0);		
		OptimalK_MaxDisIndex.push_back(-1);

		OptimalK_MinDis.push_back(0);
		OptimalK_MinDisIndex.push_back(-1);

		Optimal_EigenVectors.push_back(Eigen::Vector3f());

		//DisOfPointToCentroid.push_back(0);
		//DisOfPointToTangentPlane.push_back(0);
	}

	if (OptimalK.size() != InputCloud->points.size())
		CalcOptimalK();
	
	#pragma omp parallel for ordered schedule(dynamic)
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		vector<int> NeighbourIndexs;
		vector<float> NeighbourDiss;

		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[i], i, 
			&NeighbourIndexs, &NeighbourDiss);

		//如果自己的索引不在其中，则加入自己的索引，不影响后续最大距离和最小距离的计算
		VectorBase<int> VectorBaseInt;
		int TempIndex = VectorBaseInt.FindIndexInVector(NeighbourIndexs, i);
		if (TempIndex == -1)
		{
			NeighbourIndexs.push_back(i);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr NeighourPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int j = 0; j < NeighbourIndexs.size(); j++)
			NeighourPoints->points.push_back(InputCloud->points[NeighbourIndexs[j]]);

		NeighourPoints->points.push_back(InputCloud->points[i]);

		Eigen::Matrix3f EigenVector;		//特征向量
		Eigen::Vector3f EigenValue(3);				

		pcl::PointXYZRGB CentroidPoint = 
			GeometryBase::GetPointsTangentPlane(NeighourPoints, EigenVector, EigenValue);

		Cloud_Normals->points[i].normal_x = EigenVector(0);
		Cloud_Normals->points[i].normal_y = EigenVector(1);
		Cloud_Normals->points[i].normal_z = EigenVector(2);
		
		//意义不是很大
		//DisOfPointToCentroid[i] = PointDis(CentroidPoint, InputCloud->points[i]);
		//DisOfPointToTangentPlane[i] = GeometryBase::GetDisFromPointToPlane(
		//	InputCloud->points[i].x, InputCloud->points[i].y, InputCloud->points[i].z,
		//	EigenVector(0), EigenVector(1), EigenVector(2), -(EigenVector(0) * CentroidPoint.x
		//		+ EigenVector(1) * CentroidPoint.y + EigenVector(2) * CentroidPoint.z));

		Cloud_NormalsOfLargestEigenValue->points[i].normal_x = EigenVector(6);
		Cloud_NormalsOfLargestEigenValue->points[i].normal_y = EigenVector(7);
		Cloud_NormalsOfLargestEigenValue->points[i].normal_z = EigenVector(8);

		Optimal_EigenVectors[i] = EigenValue;

		//#pragma omp ordered
		{
			VectorBase<float> DisVector;
			int MaxIndex = DisVector.GetMaxIndex(NeighbourDiss);
			if (MaxIndex != -1)
			{
				OptimalK_MaxDis[i] = NeighbourDiss[MaxIndex];
				OptimalK_MaxDisIndex[i] = MaxIndex;
			}

			int MinIndex = DisVector.GetMinIndex(NeighbourDiss);
			if (MinIndex != -1)
			{
				OptimalK_MinDis[i] = NeighbourDiss[MinIndex];
				OptimalK_MinDisIndex[i] = MinIndex;
			}
		}

		vector<int> DoubleNeighbourIndexs;
		vector<float> DoubleNeighbourDiss;

		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[i] * 2, i,
			&DoubleNeighbourIndexs, &DoubleNeighbourDiss);
		
		NeighourPoints->points.clear();
		for (int j = 0; j < DoubleNeighbourIndexs.size(); j++)
			NeighourPoints->points.push_back(InputCloud->points[DoubleNeighbourIndexs[j]]);

		NeighourPoints->points.push_back(InputCloud->points[i]);
		
		CentroidPoint =
			GeometryBase::GetPointsTangentPlane(NeighourPoints, EigenVector, EigenValue);

		DoubleCloud_Normals->points[i].normal_x = EigenVector(0);
		DoubleCloud_Normals->points[i].normal_y = EigenVector(1);
		DoubleCloud_Normals->points[i].normal_z = EigenVector(2);

		vector<int> HalfNeighbourIndexs;
		vector<float> HalfNeighbourDiss;

		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[i] / 2.0, i,
			&HalfNeighbourIndexs, &HalfNeighbourDiss);
				
		NeighourPoints->points.clear();
		for (int j = 0; j < HalfNeighbourIndexs.size(); j++)
			NeighourPoints->points.push_back(InputCloud->points[HalfNeighbourIndexs[j]]);

		NeighourPoints->points.push_back(InputCloud->points[i]);
		
		CentroidPoint =
			GeometryBase::GetPointsTangentPlane(NeighourPoints, EigenVector, EigenValue);

		HalfCloud_Normals->points[i].normal_x = EigenVector(0);
		HalfCloud_Normals->points[i].normal_y = EigenVector(1);
		HalfCloud_Normals->points[i].normal_z = EigenVector(2);
	}	

	//计算平均法向
	#pragma omp parallel for ordered schedule(dynamic)
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		vector<int> NeighbourIndexs;
		vector<float> NeighbourDiss;

		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[i], i, &NeighbourIndexs, &NeighbourDiss);
		
		//如果不包括自己，则加入自己
		VectorBase<int> VectorBaseInt;
		int TempIndex = VectorBaseInt.FindIndexInVector(NeighbourIndexs, i);
		if (TempIndex == -1)
		{
			NeighbourIndexs.push_back(i);
		}

		if (NeighbourIndexs.size() > 0)
		{
			pcl::Normal TempNormal;
			TempNormal.normal_x = 0, TempNormal.normal_y = 0, TempNormal.normal_z = 0;

			for (int j = 0; j < NeighbourIndexs.size(); j++)
			{				
				TempNormal.normal_x += Cloud_Normals->points[NeighbourIndexs[j]].normal_x;
				TempNormal.normal_y += Cloud_Normals->points[NeighbourIndexs[j]].normal_y;
				TempNormal.normal_z += Cloud_Normals->points[NeighbourIndexs[j]].normal_z;
			}

			TempNormal.normal_x = TempNormal.normal_x / NeighbourIndexs.size();
			TempNormal.normal_y = TempNormal.normal_y / NeighbourIndexs.size();
			TempNormal.normal_z = TempNormal.normal_z / NeighbourIndexs.size();

			PointBase::PointNormalized(TempNormal);

			AvgNormalOfNeighbour->points[i] = TempNormal;
		}
	}

	//应该根据距离加权 而不是根据直接平均加权
	//exp(-( Dis * Dis / (MaxDis/2 * MaxDis/2)))
	if (IsWeighted)	
	{
		pcl::PointCloud<pcl::Normal>::Ptr TempNormals(new pcl::PointCloud<pcl::Normal>());
		//pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr Cloud_Curvatures

		for (int i = 0; i < InputCloud->points.size(); i++)
		{
			TempNormals->points.push_back(pcl::Normal());
		}

		#pragma omp parallel for
		for (int i = 0; i < InputCloud->points.size(); i++)
		{			
			vector<int> NeighbourIndexs;
			vector<float> NeighbourDis;
			GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[i],
				i, &NeighbourIndexs, &NeighbourDis);

			if (NeighbourIndexs.size() > 0)
			{
				pcl::Normal TempNormal;

				double H = (OptimalK_MinDis[i] + OptimalK_MaxDis[i]) / 2;
				double WeightSum = 0;

				TempNormal.normal_x = Cloud_Normals->points[i].normal_x;
				TempNormal.normal_y = Cloud_Normals->points[i].normal_y;
				TempNormal.normal_z = Cloud_Normals->points[i].normal_z;
				WeightSum = WeightSum + 1;

				for (int j = 0; j < NeighbourIndexs.size(); j++)
				{
					double Dis = PointDis(InputCloud->points[i], InputCloud->points[NeighbourIndexs[j]]);
					
					//快速衰减的平滑权重函数
					double Weight = 0;
					Weight = exp(-1 * pow(Dis, 2)/pow(H, 2));
					//cout <<"Weight:"<< Weight << endl;
					//Weight = 1;
					WeightSum = WeightSum + Weight;
					
					TempNormal.normal_x = Cloud_Normals->points[NeighbourIndexs[j]].normal_x 
						+ Weight * TempNormal.normal_x;
					TempNormal.normal_y = Cloud_Normals->points[NeighbourIndexs[j]].normal_y 
						+ Weight * TempNormal.normal_y;
					TempNormal.normal_z = Cloud_Normals->points[NeighbourIndexs[j]].normal_z 
						+ Weight * TempNormal.normal_z;
				}
				
				TempNormal.normal_x = TempNormal.normal_x / WeightSum;
				TempNormal.normal_y = TempNormal.normal_y / WeightSum;
				TempNormal.normal_z = TempNormal.normal_z / WeightSum;

				PointBase::PointNormalized(TempNormal);				
				TempNormals->points[i] = TempNormal;
			}			
		}

		Cloud_Normals->points.clear();
		Cloud_Normals->points.insert(Cloud_Normals->points.begin(),
			TempNormals->points.begin(), TempNormals->points.end());		
	}	
	
}

//根据给定点的索引及该点给定的内部点 设置点云对象的法向方向。
void CPointGeometry::SetNormalConsistency(int PointIndex, pcl::PointXYZRGB InteriorPoint, double RadiusValue)
{
	vector<bool> IsVisited(InputCloud->points.size(), false);
	if (PointIndex < 0 || PointIndex >= InputCloud->points.size()) return;	

	pcl::Normal TempNormal(InputCloud->points[PointIndex].x - InteriorPoint.x, 
		InputCloud->points[PointIndex].y - InteriorPoint.y,
		InputCloud->points[PointIndex].z - InteriorPoint.z);

	IsVisited[PointIndex] = true;
	double Angle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(TempNormal, Cloud_Normals->points[PointIndex]));
	if (Angle > 90) 
	{ 
		Cloud_Normals->points[PointIndex].normal_x = -Cloud_Normals->points[PointIndex].normal_x,
		Cloud_Normals->points[PointIndex].normal_y = -Cloud_Normals->points[PointIndex].normal_y,
		Cloud_Normals->points[PointIndex].normal_z = -Cloud_Normals->points[PointIndex].normal_z;
	}
	
	//to Rear
	for (int i = PointIndex + 1; i < InputCloud->points.size(); i++)
	{
		//cout << "法向一致化，to Rear" << i<<endl;
		vector<int> NeighbourIndexS;
		vector<int> AnchorIndexS;
		vector<float> NeighbourDisS;

		if (RadiusValue == 0)
			GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[i], i,
				&NeighbourIndexS, &NeighbourDisS);
		else
			GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue, i,
				&NeighbourIndexS, &NeighbourDisS);

		if (NeighbourIndexS.size() > 3)
			NeighbourIndexS.erase(NeighbourIndexS.begin() + 3, NeighbourIndexS.end());

		AnchorIndexS.push_back(i);
		for (int j = 0; j < NeighbourIndexS.size(); j++)
		{
			//计算3个后就切换 基准Index， 因为后面是每1个基准Index 加入3个近邻点
			if (j > 0 && j % 3 == 0)	
				AnchorIndexS.erase(AnchorIndexS.begin());

			int CurPointIndex = NeighbourIndexS[j];

			if (IsVisited[CurPointIndex]) continue;	//已经访问过

			IsVisited[CurPointIndex] = true;
			int TempAnchorIndex = AnchorIndexS[0];
									
			double Angle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
				Cloud_Normals->points[TempAnchorIndex], 
				Cloud_Normals->points[CurPointIndex]));

			if (Angle > 90)
			{
				Cloud_Normals->points[CurPointIndex].normal_x = -Cloud_Normals->points[CurPointIndex].normal_x,
					Cloud_Normals->points[CurPointIndex].normal_y = -Cloud_Normals->points[CurPointIndex].normal_y,
					Cloud_Normals->points[CurPointIndex].normal_z = -Cloud_Normals->points[CurPointIndex].normal_z;
			}

			vector<int> SubNeighbourIndexS;
			vector<float> SubNeighbourDisS;

			if (RadiusValue == 0)
				GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[CurPointIndex], i,
					&SubNeighbourIndexS, &SubNeighbourDisS);
			else
				GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue, CurPointIndex,
					&SubNeighbourIndexS, &SubNeighbourDisS);

			//只增加3个最近邻
			if (SubNeighbourIndexS.size() > 3)
				SubNeighbourIndexS.erase(SubNeighbourIndexS.begin() + 3, SubNeighbourIndexS.end());

			AnchorIndexS.push_back(CurPointIndex);
			NeighbourIndexS.insert(NeighbourIndexS.end(), SubNeighbourIndexS.begin(), SubNeighbourIndexS.end());
		}
	}
	
	//to head;
	for (int i = PointIndex - 1; i >= 0; i--)
	{
		//cout << "法向一致化，to head" << i << endl;
		vector<int> NeighbourIndexS;
		vector<int> AnchorIndexS;
		vector<float> NeighbourDisS;

		if (RadiusValue == 0)
			GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[i], i,
				&NeighbourIndexS, &NeighbourDisS);
		else
			GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue, i,
				&NeighbourIndexS, &NeighbourDisS);

		if (NeighbourIndexS.size() > 3)
			NeighbourIndexS.erase(NeighbourIndexS.begin() + 3, NeighbourIndexS.end());

		AnchorIndexS.push_back(i);
		for (int j = 0; j < NeighbourIndexS.size(); j++)
		{
			//计算3个后就切换 基准Index， 因为后面是每1个基准Index 加入3个近邻点
			if (j > 0 && j % 3 == 0)
				AnchorIndexS.erase(AnchorIndexS.begin());

			int CurPointIndex = NeighbourIndexS[j];

			if (IsVisited[CurPointIndex]) continue;	//已经访问过

			IsVisited[CurPointIndex] = true;
			int TempAnchorIndex = AnchorIndexS[0];

			double Angle = GeometryBase::RadianToAngle(GeometryBase::AngleOfTwoVector(
				Cloud_Normals->points[TempAnchorIndex],
				Cloud_Normals->points[CurPointIndex]));

			if (Angle > 90)
			{
				Cloud_Normals->points[CurPointIndex].normal_x = -Cloud_Normals->points[CurPointIndex].normal_x,
					Cloud_Normals->points[CurPointIndex].normal_y = -Cloud_Normals->points[CurPointIndex].normal_y,
					Cloud_Normals->points[CurPointIndex].normal_z = -Cloud_Normals->points[CurPointIndex].normal_z;
			}

			vector<int> SubNeighbourIndexS;
			vector<float> SubNeighbourDisS;

			if (RadiusValue == 0)
				GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[CurPointIndex], i,
					&SubNeighbourIndexS, &SubNeighbourDisS);
			else
				GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue, CurPointIndex,
					&SubNeighbourIndexS, &SubNeighbourDisS);

			//只增加3个最近邻
			if (SubNeighbourIndexS.size() > 3)
				SubNeighbourIndexS.erase(SubNeighbourIndexS.begin() + 3, SubNeighbourIndexS.end());

			AnchorIndexS.push_back(CurPointIndex);
			NeighbourIndexS.insert(NeighbourIndexS.end(), SubNeighbourIndexS.begin(), SubNeighbourIndexS.end());
		}
	}
}

void CPointGeometry::CalcOptimalCurvature()
{
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, 
		pcl::PrincipalCurvatures> PrincipalCurvatures;	

	//根据法向量计算曲率	

	//PrincipalCurvatures.setInputCloud(InputCloud);
	//PrincipalCurvatures.setInputNormals(Cloud_Normals);
	//PrincipalCurvatures.setSearchMethod(OCtree);
	//PrincipalCurvatures.setRadiusSearch(5);
	////PrincipalCurvatures.setKSearch(5);
	//PrincipalCurvatures.compute(*Cloud_Curvatures);

	///*
	PrincipalCurvatures.setInputCloud(InputCloud);
	PrincipalCurvatures.setInputNormals(Cloud_Normals);		

	Cloud_Curvatures->points.clear();
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		pcl::PrincipalCurvatures TempCurvatures;
		Cloud_Curvatures->points.push_back(TempCurvatures);
		Cloud_Curvatures->points[i].principal_curvature[0] = Cloud_Curvatures->points[i].principal_curvature[1]
			= Cloud_Curvatures->points[i].principal_curvature[2] = Cloud_Curvatures->points[i].pc1
			= Cloud_Curvatures->points[i].pc2 = 0;
	}

	//#pragma omp parallel for 此处不能并行计算，否则结果是错误的
	for(int i =0; i < InputCloud->points.size(); i++)
	{
		vector<int> NeighbourIndexs;
		vector<float> NeighbourDis;
		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[i],
			i, &NeighbourIndexs, &NeighbourDis);

		if (NeighbourIndexs.size() == 0)
		{
			Cloud_Curvatures->points[i].principal_curvature[0] = Cloud_Curvatures->points[i].principal_curvature[1] 
				= Cloud_Curvatures->points[i].principal_curvature[2] = Cloud_Curvatures->points[i].pc1 
				= Cloud_Curvatures->points[i].pc2 = std::numeric_limits<float>::quiet_NaN();
		}
		else
		{
			PrincipalCurvatures.computePointPrincipalCurvatures(*(Cloud_Normals.get()), i, NeighbourIndexs,
				Cloud_Curvatures->points[i].principal_curvature_x, Cloud_Curvatures->points[i].principal_curvature_y,
				Cloud_Curvatures->points[i].principal_curvature_z, Cloud_Curvatures->points[i].pc1,
				Cloud_Curvatures->points[i].pc2);
		}
	}
	//*/
}

void CPointGeometry::CalcNormalAndCurvatureKNN(int k, pcl::PointCloud<pcl::Normal>::Ptr NormalsValue,
	bool IsWeighted)
{
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> NormalEstimation;

	if (NormalsValue != NULL)
		NormalsValue->points.clear();

	if (InputCloud->points.size() <= 0) return;

	NormalEstimation.setInputCloud(InputCloud);
	NormalEstimation.setSearchMethod(OCtree);
	//NormalEstimation.setKSearch();
	NormalEstimation.setKSearch(k);
	NormalEstimation.compute(*NormalsValue);

	if (IsWeighted)
	{
		//OctreeCloudSearch = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
		//OctreeCloudSearch->setInputCloud(InputCloud);
		//OctreeCloudSearch->addPointsFromInputCloud();

		pcl::PointCloud<pcl::Normal>::Ptr TempNormals(new pcl::PointCloud<pcl::Normal>());
		//pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr Cloud_Curvatures

		for (int i = 0; i < InputCloud->points.size(); i++)
		{
			TempNormals->points.push_back(pcl::Normal());
		}

		#pragma omp parallel for
		for (int i = 0; i < InputCloud->points.size(); i++)
		{
			pcl::Normal TempNormal;
			vector<int> NeighbourIndexs;
			vector<float> NeighbourDis;
			GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, k,
				i, &NeighbourIndexs, &NeighbourDis);

			if (NeighbourIndexs.size() > 0)
			{
				TempNormal.normal_x = NormalsValue->points[i].normal_x + TempNormal.normal_x;
				TempNormal.normal_y = NormalsValue->points[i].normal_y + TempNormal.normal_y;
				TempNormal.normal_z = NormalsValue->points[i].normal_z + TempNormal.normal_z;

				for (int j = 0; j < NeighbourIndexs.size(); j++)
				{
					TempNormal.normal_x = NormalsValue->points[NeighbourIndexs[j]].normal_x + TempNormal.normal_x;
					TempNormal.normal_y = NormalsValue->points[NeighbourIndexs[j]].normal_y + TempNormal.normal_y;
					TempNormal.normal_z = NormalsValue->points[NeighbourIndexs[j]].normal_z + TempNormal.normal_z;
				}

				TempNormal.normal_x = TempNormal.normal_x / (NeighbourIndexs.size() + 1);
				TempNormal.normal_y = TempNormal.normal_y / (NeighbourIndexs.size() + 1);
				TempNormal.normal_z = TempNormal.normal_z / (NeighbourIndexs.size() + 1);
			}
			TempNormals->points[i] = TempNormal;
		}

		NormalsValue->points.clear();
		NormalsValue->points.insert(NormalsValue->points.begin(),
			TempNormals->points.begin(), TempNormals->points.end());
	}
}


void CPointGeometry::CalcNormalAndCurvatureRadius(double RadiusValue, bool IsWeighted)
{
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> NormalEstimation;

/*	OCtree.reset(new pcl::search::Octree<pcl::PointXYZRGB>(0.1f));	
	OCtree->setInputCloud(InputCloud);	*/	

	if (Cloud_Normals != NULL)
		Cloud_Normals->points.clear();

	if (InputCloud->points.size() <= 0) return;
	
	NormalEstimation.setInputCloud(InputCloud);
	NormalEstimation.setSearchMethod(OCtree);
	//NormalEstimation.setKSearch();
	NormalEstimation.setRadiusSearch(RadiusValue);
	NormalEstimation.compute(*Cloud_Normals);

	if (IsWeighted)
	{
		pcl::PointCloud<pcl::Normal>::Ptr TempNormals(new pcl::PointCloud<pcl::Normal>());
		//pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr Cloud_Curvatures

		for (int i = 0; i < InputCloud->points.size(); i++)
		{
			TempNormals->points.push_back(pcl::Normal());
		}

		//#pragma omp parallel for


		for (int i = 0; i < InputCloud->points.size(); i++)
		{
			pcl::Normal TempNormal;
			vector<int> NeighbourIndexs;
			vector<float> NeighbourDis;
			GeometryBase::GetNeighbourInRadius(OctreeCloudSearch, RadiusValue,
				i, &NeighbourIndexs, &NeighbourDis);

			if (NeighbourIndexs.size() > 0)
			{
				double H = (NeighbourDis[0] + NeighbourDis[NeighbourDis.size() - 1]) / 2;
				double WeightSum = 0;

				TempNormal.normal_x = Cloud_Normals->points[i].normal_x;
				TempNormal.normal_y = Cloud_Normals->points[i].normal_y;
				TempNormal.normal_z = Cloud_Normals->points[i].normal_z;
				WeightSum = WeightSum + 1;

				for (int j = 0; j < NeighbourIndexs.size(); j++)
				{
					double Dis = PointDis(InputCloud->points[i], InputCloud->points[NeighbourIndexs[j]]);

					//快速衰减的平滑权重函数
					double Weight = 0;
					Weight = exp(-1 * pow(Dis, 2) / pow(H, 2));
					//cout <<"Weight:"<< Weight << endl;
					//Weight = 1;
					//WeightSum = WeightSum + Weight;

					TempNormal.normal_x = Cloud_Normals->points[NeighbourIndexs[j]].normal_x
						+ Weight * TempNormal.normal_x;
					TempNormal.normal_y = Cloud_Normals->points[NeighbourIndexs[j]].normal_y
						+ Weight * TempNormal.normal_y;
					TempNormal.normal_z = Cloud_Normals->points[NeighbourIndexs[j]].normal_z
						+ Weight * TempNormal.normal_z;
				}

				TempNormal.normal_x = TempNormal.normal_x / WeightSum;
				TempNormal.normal_y = TempNormal.normal_y / WeightSum;
				TempNormal.normal_z = TempNormal.normal_z / WeightSum;

				PointBase::PointNormalized(TempNormal);
			}
			TempNormals->points[i] = TempNormal;
		}

		Cloud_Normals->points.clear();
		Cloud_Normals->points.insert(Cloud_Normals->points.begin(),
			TempNormals->points.begin(), TempNormals->points.end());
	}
	//根据加权后的法向量计算曲率
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> PrincipalCurvatures;
	PrincipalCurvatures.setInputCloud(InputCloud);
	PrincipalCurvatures.setInputNormals(Cloud_Normals);
	PrincipalCurvatures.setSearchMethod(OCtree);
	PrincipalCurvatures.setRadiusSearch(RadiusValue);
	//PrincipalCurvatures.setKSearch(5);
	PrincipalCurvatures.compute(*Cloud_Curvatures);
}

pcl::PrincipalCurvatures CPointGeometry::GetCurvature(int PointIndex)
{
	if (Cloud_Curvatures->points.size() > PointIndex) 
		return (*Cloud_Curvatures)[PointIndex];
}

pcl::Normal CPointGeometry::GetNormal(int PointIndex)
{
	if (Cloud_Normals->points.size() > PointIndex)
		return(Cloud_Normals->points[PointIndex]);
}

pcl::Normal CPointGeometry::GetDirectionOfPrincipalCurvature(int PointIndex)
{
	pcl::Normal TempNormal;
	TempNormal.normal_x = Cloud_Curvatures->points[PointIndex].principal_curvature_x;
	TempNormal.normal_y = Cloud_Curvatures->points[PointIndex].principal_curvature_y;
	TempNormal.normal_z = Cloud_Curvatures->points[PointIndex].principal_curvature_z;
	return TempNormal;
}

void CPointGeometry::GetDirectionOfPrincipalCurvatures(pcl::PointCloud<pcl::Normal>::Ptr CurvatureDirections)
{
	if (CurvatureDirections == NULL)
		CurvatureDirections.reset(new pcl::PointCloud<pcl::Normal>());

	CurvatureDirections->points.clear();

	for (int i = 0; i < Cloud_Curvatures->points.size(); i++)
	{
		pcl::Normal TempNormal;
		TempNormal.normal_x = Cloud_Curvatures->points[i].principal_curvature_x;
		TempNormal.normal_y = Cloud_Curvatures->points[i].principal_curvature_y;
		TempNormal.normal_z = Cloud_Curvatures->points[i].principal_curvature_z;
		CurvatureDirections->points.push_back(TempNormal);
	}
}

float CPointGeometry::GetGuassCurvature(int PointIndex)
{
	if (Cloud_Curvatures->points.size() > PointIndex)
		return ((*Cloud_Curvatures)[PointIndex].pc1 * (*Cloud_Curvatures)[PointIndex].pc2);
}

float CPointGeometry::GetMeanCurvature(int PointIndex)
{
	if (Cloud_Curvatures->points.size() > PointIndex)
		return ((*Cloud_Curvatures)[PointIndex].pc1 + (*Cloud_Curvatures)[PointIndex].pc2) / 2.0;
}

//get Maxest GuassCurvature
vector<int> CPointGeometry::GetMaxGuassCurvature(int MaxNum)
{
	vector<int> IndexS;

	for (int i = 0; i < Cloud_Curvatures->points.size(); i++)
	{
		IndexS.push_back(i);
	}

	float MaxGuass;
	int MaxIndex;
	int Count = 0;

	for (int i = 0; i < IndexS.size(); i++)
	{
		MaxGuass = abs(GetGuassCurvature(IndexS[i]));
		MaxIndex = IndexS[i];
		for (int j = i + 1; j < IndexS.size(); j++)
		{
			if (abs(GetGuassCurvature(IndexS[j])) > MaxGuass)
			{
				MaxGuass = abs(GetGuassCurvature(IndexS[j]));
				MaxIndex = IndexS[j];
			}
		}

		if (IndexS[i] != MaxIndex)
		{
			int TempIndex = IndexS[i];
			IndexS[i] = IndexS[MaxIndex];
			IndexS[MaxIndex] = TempIndex;
			Count++;

			if (Count > MaxNum)
				break;
		}
	}

	for (int i = IndexS.size() - 1; i >= MaxNum; i--)
	{
		IndexS.pop_back();
	}

	return IndexS;
}

vector<int> CPointGeometry::GetMinGuassCurvature(int MaxNum)
{
	vector<int> IndexS;

	for (int i = 0; i < Cloud_Curvatures->points.size(); i++)
	{
		IndexS.push_back(i);
	}

	float MinGuass;
	int MinIndex;
	int Count = 0;

	for (int i = 0; i < IndexS.size(); i++)
	{
		MinGuass = abs(GetGuassCurvature(IndexS[i]));
		MinIndex = IndexS[i];
		for (int j = i + 1; j < IndexS.size(); j++)
		{
			if (abs(GetGuassCurvature(IndexS[j])) < MinGuass)
			{
				MinGuass = abs(GetGuassCurvature(IndexS[j]));
				MinIndex = IndexS[j];
			}
		}

		if (IndexS[i] != MinIndex)
		{
			int TempIndex = IndexS[i];
			IndexS[i] = IndexS[MinIndex];
			IndexS[MinIndex] = TempIndex;
			Count++;

			if (Count > MaxNum)
				break;
		}
	}

	for (int i = IndexS.size() - 1; i >= MaxNum; i--)
	{
		IndexS.pop_back();
	}

	return IndexS;
}


void CPointGeometry::SetOptimalK(int K)
{
	OptimalK.clear();
	
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		OptimalK.push_back(K);
	}
}

//计算每个点的最佳近邻个数
void CPointGeometry::CalcOptimalK()
{
	OptimalK.clear();

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		OptimalK.push_back(0);
	}
		
	#pragma omp parallel for
	for (int i = 0; i < InputCloud->points.size(); i++)	
	{
		OptimalK[i] = CalcOptimalKForPoint(i, OptimalK_a, OptimalK_b);	
	}
}

//只计算每个点在最近邻域个数下的法向量 2020.02.26
void CPointGeometry::CalcOptimalNormal()
{
	Cloud_Normals->points.clear();

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		Cloud_Normals->points.push_back(pcl::Normal());
	}

	if (OptimalK.size() != InputCloud->points.size())
		CalcOptimalK();

	#pragma omp parallel for ordered schedule(dynamic)
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		vector<int> NeighbourIndexs;
		vector<float> NeighbourDiss;

		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[i], i,
			&NeighbourIndexs, &NeighbourDiss, true);
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr NeighourPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int j = 0; j < NeighbourIndexs.size(); j++)
			NeighourPoints->points.push_back(InputCloud->points[NeighbourIndexs[j]]);

		NeighourPoints->points.push_back(InputCloud->points[i]);

		Eigen::Matrix3f EigenVector;		//特征向量
		Eigen::Vector3f EigenValue(3);

		pcl::PointXYZRGB CentroidPoint =
			GeometryBase::GetPointsTangentPlane(NeighourPoints, EigenVector, EigenValue);

		Cloud_Normals->points[i].normal_x = EigenVector(0);
		Cloud_Normals->points[i].normal_y = EigenVector(1);
		Cloud_Normals->points[i].normal_z = EigenVector(2);		
	}
}

Eigen::Vector3f CPointGeometry::CalcEigenValueForPoint(int PointIndex, int k)
{
	vector<int> NeighbourIndexs;
	vector<float> NeighbourDiss;

	GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, k, PointIndex, &NeighbourIndexs, &NeighbourDiss);

	VectorBase<float> VectorBasefloat;
	//int MaxIndex = VectorBasefloat.GetMaxIndex(NeighbourDiss);
	//int MinIndex = VectorBasefloat.GetMinIndex(NeighbourDiss);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr NeighourPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int j = 0; j < NeighbourIndexs.size(); j++)
		NeighourPoints->points.push_back(InputCloud->points[NeighbourIndexs[j]]);

	NeighourPoints->points.push_back(InputCloud->points[PointIndex]);

	Eigen::Matrix3f EigenVector;		//特征向量
	Eigen::Vector3f EigenValue(3);
	GeometryBase::GetPointsTangentPlane(NeighourPoints, EigenVector, EigenValue);

	return EigenValue;
}

//2019.09.24 获取高斯曲率变化比较大的点，该点的曲率值显著高于邻域点的曲率值
void CPointGeometry::GetCurvatureVariationSignificantly(vector<int> & IndexS)
{
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		int K = OptimalK[i];

		vector<int> NeighbourIndexS;
		vector<float> NeighbourDisS;
		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, K, i, &NeighbourIndexS, &NeighbourDisS);
		
		double TempCurvature = GetGuassCurvature(i);

		int Count = 0;
		for (int j = 0; j < NeighbourIndexS.size(); j++)
		{
			if (TempCurvature > GetGuassCurvature(NeighbourIndexS[j]) * 5)
			{
				Count++;
			}
		}

		//半数曲率小于当前曲率值的 1 /2；
		if (Count > NeighbourIndexS.size() / 2)
		{
			IndexS.push_back(i);
		}
	}
}

//2019.09.29 根据曲率的均值与标准差 计算每一个曲率异常点到均值点的路径
void CPointGeometry::SearchCurvaturePathPoints(vector<int> SliceIndex, 
	vector<int> & RedIndexs, vector<int> & BlueIndexs)
{
	VectorBase<double> VectorBaseDouble;
	
	RedIndexs.clear();
	BlueIndexs.clear();

	vector<double> CurvatureVec;
	for (int i = 0; i < SliceIndex.size(); i++)
	{
		CurvatureVec.push_back(GetGuassCurvature(SliceIndex[i]));
	}
	
	CurvatureStdError = sqrt(VectorBaseDouble.CalcVariances(CurvatureVec, CurvatureMean));

	vector<int> CurvatureOutliersPointsIndexS;
	for (int i = 0; i < SliceIndex.size(); i++)
	{
		if (CurvatureVec[i] > CurvatureMean + 3 * CurvatureStdError)
		{
			InputCloud->points[i].rgba = ColorBase::RedColor;
			CurvatureOutliersPointsIndexS.push_back(SliceIndex[i]);
			RedIndexs.push_back(SliceIndex[i]);
		}
	}	

	//cout << "Global Mean:" << CurvatureMean << endl;
	//cout << "Global Std:" << CurvatureStdError << endl;

	for (int i = 0; i < CurvatureOutliersPointsIndexS.size(); i++)
	{
		vector<int> NeighbourIndexS;
		vector<float> NeighbourDisS;

		//表明是红色异常点
		//if (InputCloud->points[CurvatureOutliersPointsIndexS[i]].rgba == ColorBase::RedColor)
		//{

		//}
		//cout<<"Curvature Point Index:"<< CurvatureOutliersPointsIndexS[i] <<endl;
		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, OptimalK[CurvatureOutliersPointsIndexS[i]],
			CurvatureOutliersPointsIndexS[i], &NeighbourIndexS, &NeighbourDisS);

		//在邻域内寻找曲率最大的点
		double MaxCurvature = 0; 
		int MaxCurvatureIndex = -1;
		VectorBase<int> VectorBaseInt;

		for (int j = 0; j < NeighbourIndexS.size(); j++)
		{
			int TempPointIndex = NeighbourIndexS[j];

			//不再已知的曲率大的索引中
			if (VectorBaseInt.FindIndexInVector(CurvatureOutliersPointsIndexS, TempPointIndex) == -1)
			{
				if (GetGuassCurvature(TempPointIndex) > CurvatureMean + 1 * CurvatureStdError)
				{
					InputCloud->points[TempPointIndex].rgba = ColorBase::BlueColor;
					CurvatureOutliersPointsIndexS.push_back(TempPointIndex);
					BlueIndexs.push_back(TempPointIndex);
				}
			}
		}
	}
}

//2019.10.03 计算最优邻域下每个点的几何特征
void CPointGeometry::CalcOptimalPointGeometryFeatures()
{	
	if (InputCloud->points.size() != OptimalK.size())
	{
		CalcOptimalK();
		CalcOptimalNormalWithMore();
		CalcOptimalCurvature();
	}

	PointGeometryFeatures.clear();
	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		PointGeometryFeature CurPointGF;
		
		CurPointGF.PointIndex = i;
		CurPointGF.PointNormal = Cloud_Normals->points[i];
		CurPointGF.PointCurvatures = Cloud_Curvatures->points[i];

		GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, i, OptimalK[i],
			&CurPointGF.NeighourIndexs, &CurPointGF.NeighourDis);

		// Find the current index i is in itself's NeigbourIndex, if in, delete it.
		VectorBase<int> VectorBaseInt;
		int TempIndex = VectorBaseInt.FindIndexInVector(CurPointGF.NeighourIndexs, i);
		if (TempIndex != -1)
		{
			//delete itself's index
			CurPointGF.NeighourIndexs.erase(CurPointGF.NeighourIndexs.begin() + TempIndex);
			//delete itself's dis value
			CurPointGF.NeighourDis.erase(CurPointGF.NeighourDis.begin() + TempIndex);
		}

		//计算每个点的几何特征
		CalcPointGeometryFeature(CurPointGF);

		PointGeometryFeatures.push_back(CurPointGF);
	}
}

//使用黄金分割法求解点的最优邻域尺度
int CPointGeometry::CalcOptimalKForPoint(int PointIndex, int StartK, int EndK)
{
	if (StartK == EndK) return EndK;
		
	double Range = EndK - StartK;

	int Start_Try, End_Try;

	Start_Try = StartK + 0.312 * Range;
	End_Try = StartK + 0.618 * Range;	

	while (Start_Try != End_Try)
	{
		double ValueOfStart, ValueOfEnd;

		Eigen::Vector3f EigenValuesOfStart = CalcEigenValueForPoint(PointIndex, Start_Try);
		Eigen::Vector3f EigenValuesOfEnd = CalcEigenValueForPoint(PointIndex, End_Try);
	
		ValueOfStart = abs(EigenValuesOfStart(1) / EigenValuesOfStart(2));
		ValueOfEnd = abs(EigenValuesOfEnd(1) / EigenValuesOfEnd(2));
		if (Start_Try + 1 == End_Try)
		{
			if (ValueOfStart > ValueOfEnd) return Start_Try;
			else return End_Try;
		}

		if (ValueOfStart < ValueOfEnd)
		{
			StartK = Start_Try;
			Range = EndK - StartK;
			Start_Try = StartK + 0.312 * Range;
			End_Try = StartK + 0.618 * Range;
		}
		else
		{
			EndK = End_Try;
			Range = EndK - StartK;
			Start_Try = StartK + 0.312 * Range;
			End_Try = StartK + 0.618 * Range;
		}

		if (StartK == EndK) return EndK;
	}
}


int CPointGeometry::GetOptimalK(int PointIndex)
{
	return OptimalK[PointIndex];
}

void CPointGeometry::GetOptimalNeighbourIndex(int PointIndex, vector<int>& Indexs)
{
	Indexs.clear();
	vector<float> NeighbourDiss;

	GeometryBase::GetNeighbourInKNN(OctreeCloudSearch, 
		OptimalK[PointIndex], PointIndex, &Indexs, &NeighbourDiss);
}

float CPointGeometry::GetOptimalK_MaxDis(int PointIndex)
{
	return OptimalK_MaxDis[PointIndex];
}

float CPointGeometry::GetOptimalK_MinDis(int PointIndex)
{
	return OptimalK_MinDis[PointIndex];
}