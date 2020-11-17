#include "TreeOutliersRemoval.h"

CTreeOutliersRemoval::CTreeOutliersRemoval()
{
	Octree = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));
	if (Viewer != NULL)
	{
		Viewer->removePointCloud("PointNormal");
	}
}

CTreeOutliersRemoval::~CTreeOutliersRemoval()
{
	PointGeometryFeatures.clear();

	Octree->deleteTree();
	free(Octree);
}

void CTreeOutliersRemoval::SetInputs(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPointPtr, int RadiusValue)
{
	InputCloud = TempPointPtr;
	Radius = RadiusValue;
	
	Octree->deleteTree();
	Octree->setInputCloud(InputCloud);
	Octree->addPointsFromInputCloud();	
}

//calc Geometry Feature for each point
void CTreeOutliersRemoval::CalcGeometryFeature()
{
	PointGeometryFeatures.clear();

	PointCurvature.SetInputs(InputCloud, Radius);
	PointCurvature.CalcCurvature();
	
	//ShowNormals(InputCloud, PointCurvature.Cloud_Normals, "PointNormal", 10, 10);
	//ShowPrincipalCurvatures(InputCloud, PointCurvature.Cloud_Normals, PointCurvature.Cloud_Curvatures, 
	//	"PrincipalCurvatures", 10, 100);
	//

	for (int i = 0; i < InputCloud->points.size(); i++)
	{
		PointGeometryFeature CurPointGF;
		CurPointGF.PointIndex = i;
		CurPointGF.PointNormal = PointCurvature.GetNormal(i);
		CurPointGF.PointCurvatures = PointCurvature.GetCurvature(i);

		Octree->radiusSearch(i, Radius, CurPointGF.NeighourIndexs, CurPointGF.NeighourDis);

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
		
		CalcPointGeometryFeature(CurPointGF);

		PointGeometryFeatures.push_back(CurPointGF);
	}

	for (int i = 0; i < InputCloud->points.size(); i++)
	{		
		double Current = PointGeometryFeatures[i].DensityByDis 
			+ PointGeometryFeatures[i].DisBetProjectAndCentriod 
			+ PointGeometryFeatures[i].AvgGuassCurvatureChange;
				
		double NearestNeighbour = 
			//PointGeometryFeatures[PointGeometryFeatures[i].MinDisNeighourIndex].DensityByDis 
			//+ PointGeometryFeatures[PointGeometryFeatures[i].MinDisNeighourIndex].DisBetProjectAndCentriod 
			//+ PointGeometryFeatures[PointGeometryFeatures[i].MinDisNeighourIndex].AvgGuassCurvatureChange;
			PointGeometryFeatures[PointGeometryFeatures[i].MaxDisNeighourIndex].DensityByDis
			+ PointGeometryFeatures[PointGeometryFeatures[i].MaxDisNeighourIndex].DisBetProjectAndCentriod
			+ PointGeometryFeatures[PointGeometryFeatures[i].MaxDisNeighourIndex].AvgGuassCurvatureChange;

		if (PointGeometryFeatures[i].NeighourIndexs.size() == 0
			|| PointGeometryFeatures[i].DisBetProjectAndCentriod > Radius / 2
			|| PointGeometryFeatures[i].DisBetProjectAndOriginal > Radius / 2)
		{
			InputCloud->points[i].rgba = ColorBase::RedColor;
		}

		//if (PointGeometryFeatures[i].PointCurvatures.pc1 * PointGeometryFeatures[i].PointCurvatures.pc2 > 0)	//No Neighbour points
		//{
		//	InputCloud->points[i].rgba = ColorBase::RedColor;
		//	//PointGeometryFeatures[i].AvgGuassCurvatureChange
		//}

		//if (PointGeometryFeatures[i].NeighourIndexs.size() == 0)	//No Neighbour points
		//{
		//	InputCloud->points[i].rgba = ColorBase::RedColor;
		//}

		//if (PointGeometryFeatures[PointGeometryFeatures[i].MinDisNeighourIndex].DisBetProjectAndCentriod > Radius / 2)
		//{
		//	InputCloud->points[i].rgba = ColorBase::RedColor;
		//	//InputCloud->points[PointGeometryFeatures[i].MinDisNeighourIndex].rgba = ColorBase::RedColor;
		//}
			   
		//if (Current > 2 * NearestNeighbour )
		//{
		//	InputCloud->points[i].rgba = ColorBase::RedColor;
		//}

		//if (InputCloud->points[PointGeometryFeatures[i].MinDisNeighourIndex].rgba == ColorBase::RedColor)
		//{
		//	InputCloud->points[i].rgba = ColorBase::RedColor;
		//}
	}
}

// calc GeometryFeature for current point
void CTreeOutliersRemoval::CalcPointGeometryFeature(PointGeometryFeature & PGF)
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
			abs(PointCurvature.GetGuassCurvature(PGF.PointIndex)
				- PointCurvature.GetGuassCurvature(PGF.NeighourIndexs[i]));
	}

	if (PGF.NeighourIndexs.size() > 0)
	{
		PGF.MaxDisNeighourIndex = MaxIndex;
		PGF.MinDisNeighourIndex = MinIndex;

		PGF.CentriodPoint.x = PGF.CentriodPoint.x / PGF.NeighourIndexs.size();
		PGF.CentriodPoint.y = PGF.CentriodPoint.y / PGF.NeighourIndexs.size();
		PGF.CentriodPoint.z = PGF.CentriodPoint.z / PGF.NeighourIndexs.size();	

		PGF.AvgDis = PGF.AvgDis / PGF.NeighourIndexs.size();
		PGF.DensityByDis = 
			(abs(PGF.AvgDis - 
				PointDis(InputCloud->points[PGF.PointIndex], InputCloud->points[PGF.MaxDisNeighourIndex]))
			+ abs(PGF.AvgDis - 
				PointDis(InputCloud->points[PGF.PointIndex], InputCloud->points[PGF.MinDisNeighourIndex])));
		
		PGF.AvgGuassCurvatureChange = PGF.AvgGuassCurvatureChange / PGF.NeighourIndexs.size();

		Eigen::Vector4f PlaneCoefficients;
		PGF.ProjectPlaneCoefficients[0] = PGF.PointNormal.normal_x;
		PGF.ProjectPlaneCoefficients[1] = PGF.PointNormal.normal_y;
		PGF.ProjectPlaneCoefficients[2] = PGF.PointNormal.normal_z;
		PGF.ProjectPlaneCoefficients[3] = -(PGF.ProjectPlaneCoefficients[0] * PGF.CentriodPoint.x
			+ PGF.ProjectPlaneCoefficients[1] * PGF.CentriodPoint.y + PGF.ProjectPlaneCoefficients[2] * PGF.CentriodPoint.z);

		pcl::projectPoint(InputCloud->points[PGF.PointIndex], PGF.ProjectPlaneCoefficients, PGF.ProjectPoint);

		PGF.DisBetProjectAndCentriod = PointDis(PGF.ProjectPoint, PGF.CentriodPoint);
		PGF.DisBetProjectAndOriginal = PointDis(PGF.ProjectPoint, InputCloud->points[PGF.PointIndex]);
	}
}