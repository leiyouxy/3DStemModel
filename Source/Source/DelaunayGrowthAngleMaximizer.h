//2015.07.01 根据基边对角最大化构建Delaunay三角形
////平面上
#ifndef DelaunayGrowthAngleMaximizer_H
#define DelaunayGrowthAngleMaximizer_H

#include "CommClass.h"
#include "ContourAndConvexHull.h"
#include "CommGeometry.h"

using namespace std;


class CDelaunayGrowthAngleMaximizer
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPtr;
	pcl::PointCloud<PointXYZRGBIndex>::Ptr CopyIndexCloud;
	
	////test
	//bool test;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TestPtr;

	bool UsePointSumAngle;

	bool FindTriangle;
	bool FindEdge;

	vector<double> SumAngle;
	//bool UseNearestNeighbour;
	//构造的Octree树 不一定使用
	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> * Octree;

	//需要拓展的边
	vector<EdgeNodes> NeedExpandEdges;
	
	//通过第三点角度最大的方式找下一个点 返回的第三个点总是 OneIndex->TwoIndex 方向的正向
	//即 OneIndex->TwoIndex TwoIndex->ThreeIndex ThreeIndex->OneIndex 是逆时针方向
	int FindNextPointByAngleMax(int OneIndex, int TwoIndex);	

	//添加三角形
	void AddTriangle(pcl::PolygonMesh & output,
		int OneIndex, int TwoIndex, int ThreeIndex, bool NeedFindEdge = true);

	void ExpandEdge(pcl::PolygonMesh & output, 
		int ConstantIndex, //已经固定的一个顶点
		int OneIndex, int TwoIndex, bool NeedFindEdge = true); //待扩展到两个顶点

	//是否是全局的边界边 //
	bool IsBorderEdge(int OneIndex, int TwoIndex);

	//判断边是否在需要扩展的边中
	bool EdgeIsInNeedExpaned(int OneIndex, int TwoIndex);
	
	//从点集中寻找距离OneIndex最近的点
	int FindNearestPoint(int OneIndex);

	//2015.07.15 在CopyIndexCloud点云中寻找Index为Index的点云
	int GetTempIndexFromCopyIndexCloud(int Index);

	//2015.07.15 优先拓展的顶点
	int PriorityIndex;	//加入这个后有问题

	//2015.07.15 将具有优先权的边移动到前面来
	void ForwardMove();

	//2015.07.15 从CopyIndexCloud删除Index为Index的节点
	void RemovePointsFromCopyIndexCloud(int Index);

	//2015.07.23 判断三角形是否重复
	bool FindTriangleIsExists(pcl::PolygonMesh & output, int OneIndex, int TwoIndex, int ThreeIndex);

	//2015.07.23 判断边是否还需要扩展
	bool FindEdgeNeedExpand(pcl::PolygonMesh & output, int OneIndex, int TwoIndex);

	//2015.07.23 点的角度和初始化
	void SumAngleInitial();
public:
	CDelaunayGrowthAngleMaximizer()
	{
		TheSpecifiedPoint = -1;	
	}

	//点集凸包集合，在设置输入参数的时候初始化
	vector<int> HullPoints;

	//设置输入数据
	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPtrValue);

	void performReconstruction(pcl::PolygonMesh & output, 
		//是否使用角度最大准则  剩下的三个变量主要体现在选择第三个点上
		bool UsePointSumAngleValue = true); //是否使用单点角度和为360

	//2015.07.23
	//使用经典算法构建 以点集中一点 及 最近点做 基边 且在三角形中判断边是否需要扩展
	void performReconstructionRandomBaseEdge(pcl::PolygonMesh & output,
		bool UsePointSumAngleValue = false, bool FindTriangleValue = true);

	////2016.01.11 指定一个点，以这个点优先计算，计算后即返回重建结果。并判断这个点是否已经封闭
	int TheSpecifiedPoint;

	//返回 Index 这个点 是否已经是封闭点
	bool GetPointIsClosed(int PointIndex, vector<int> & NeighborIndexs);
};

#endif


	//文章使用的测试

	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints5000.pcd", StemPointsPtr);	//9964 个三角形
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints10000.pcd", StemPointsPtr);	//19961 个三角形
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints15000.pcd", StemPointsPtr);		//29958 个三角形
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints20000.pcd", StemPointsPtr);	//39952 个三角形
	
	//////pcl::PolygonMesh PolygonMeshClassic;	
	//////CDelaunayGrowthAngleMaximizer RandomBaseEdge;
	//////RandomBaseEdge.SetInputs(StemPointsPtr);
	////////RandomBaseEdge.performReconstructionRandomBaseEdge(PolygonMeshClassic, false, false);	//检索边是否需要扩展
	////////cout<<"点个数:"<<StemPointsPtr->points.size()<<" 三角形个数:"<<PolygonMeshClassic.polygons.size()<<endl;

	//////RandomBaseEdge.performReconstructionRandomBaseEdge(PolygonMeshClassic, false, true);
	//////cout<<"点个数:"<<StemPointsPtr->points.size()<<" 三角形个数:"<<PolygonMeshClassic.polygons.size()<<endl;

	//////Viewer->addPolygonMesh(PolygonMeshClassic, "PolygonMeshClassic");

	//////PointBase::PointCoordinateTransform(StemPointsPtr, 0, 10, 0);
	//////pcl::PolygonMesh PolygonMesh;
	//CDelaunayGrowthAngleMaximizer DelaunayGrowthAngleMaximizer;
	//DelaunayGrowthAngleMaximizer.SetInputs(StemPointsPtr);
	//DelaunayGrowthAngleMaximizer.performReconstruction(PolygonMesh);
	//cout<<"点个数:"<<StemPointsPtr->points.size()<<" 三角形个数:"<<PolygonMesh.polygons.size()<<endl;
	//Viewer->addPolygonMesh(PolygonMesh, "DelaunayGrowthAngleMaximizer");

	//////PointBase::PointCoordinateTransform(StemPointsPtr, 0, 20, 0);
	//////pcl::PolygonMesh PolygonMeshAngleSum;	
	//////CDelaunayGrowthAngleMaximizer DelaunayGrowthAngleMaximizerAngleSum;
	//////DelaunayGrowthAngleMaximizerAngleSum.SetInputs(StemPointsPtr);
	//////DelaunayGrowthAngleMaximizerAngleSum.performReconstruction(PolygonMeshAngleSum, true);
	//////cout<<"点个数:"<<StemPointsPtr->points.size()<<" 三角形个数:"<<PolygonMeshAngleSum.polygons.size()<<endl;
	//////Viewer->addPolygonMesh(PolygonMeshAngleSum, "DelaunayGrowthAngleMaximizerAngleSum");	


	////////////////测试三角网生长 I:\\ProcessedPoints\\PlanePointsRGBPtr.pcd
	
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\PlanePointsRGBPtr6977.pcd", StemPointsPtr);	
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints5000.pcd", StemPointsPtr);	//9964 个三角形
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints10000.pcd", StemPointsPtr);	//19961 个三角形
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints15000.pcd", StemPointsPtr);		//29958 个三角形
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints20000.pcd", StemPointsPtr);	//39952 个三角形
	
	/*CenterX = -1 * StemPointsPtr->points[0].x;
	CenterY = -1 * StemPointsPtr->points[0].y;
	CenterZ = -1 * StemPointsPtr->points[0].z;
	PointBase::PointCoordinateTransform(StemPointsPtr, CenterX, CenterY, CenterZ);	
	PointBase::ShowPointXYZRGB(Viewer, StemPointsPtr, "20000", 1);
	PointBase::SavePCDToFileName(StemPointsPtr, "I:\\ProcessedPoints\\120PlanePoints5000.pcd");
	*/	
	/*pcl::PolygonMesh PointPolygonMesh;
	CDelaunayGrowth DelaunayGrowth;
	DelaunayGrowth.SetInputs(StemPointsPtr, 2);
	DelaunayGrowth.performReconstruction(PointPolygonMesh);
	cout<<"三角形个数:"<<PointPolygonMesh.polygons.size()<<endl;
	Viewer->addPolygonMesh(PointPolygonMesh, "DelaunayGrowth");
	*///PointBase::PointCoordinateTransform(StemPointsPtr, 0, 3, 0);
	//PointBase::MeshToTriangles(Viewer, StemPointsPtr, PointPolygonMesh, "DelaunayGrowth");

	//////pcl::PolygonMesh PolygonMeshClassic;	
	//////CDelaunayGrowthAngleMaximizer RandomBaseEdge;
	//////RandomBaseEdge.SetInputs(StemPointsPtr);
	////////RandomBaseEdge.performReconstructionRandomBaseEdge(PolygonMeshClassic, false, false);	//检索边是否需要扩展
	////////cout<<"点个数:"<<StemPointsPtr->points.size()<<" 三角形个数:"<<PolygonMeshClassic.polygons.size()<<endl;

	//////RandomBaseEdge.performReconstructionRandomBaseEdge(PolygonMeshClassic, false, true);
	//////cout<<"点个数:"<<StemPointsPtr->points.size()<<" 三角形个数:"<<PolygonMeshClassic.polygons.size()<<endl;

	//////Viewer->addPolygonMesh(PolygonMeshClassic, "PolygonMeshClassic");

	//////PointBase::PointCoordinateTransform(StemPointsPtr, 0, 10, 0);
	//////pcl::PolygonMesh PolygonMesh;
	//////CDelaunayGrowthAngleMaximizer DelaunayGrowthAngleMaximizer;
	//////DelaunayGrowthAngleMaximizer.SetInputs(StemPointsPtr);
	//////DelaunayGrowthAngleMaximizer.performReconstruction(PolygonMesh);
	//////cout<<"点个数:"<<StemPointsPtr->points.size()<<" 三角形个数:"<<PolygonMesh.polygons.size()<<endl;
	//////Viewer->addPolygonMesh(PolygonMesh, "DelaunayGrowthAngleMaximizer");

	//////PointBase::PointCoordinateTransform(StemPointsPtr, 0, 20, 0);
	//////pcl::PolygonMesh PolygonMeshAngleSum;	
	//////CDelaunayGrowthAngleMaximizer DelaunayGrowthAngleMaximizerAngleSum;
	//////DelaunayGrowthAngleMaximizerAngleSum.SetInputs(StemPointsPtr);
	//////DelaunayGrowthAngleMaximizerAngleSum.performReconstruction(PolygonMeshAngleSum, true);
	//////cout<<"点个数:"<<StemPointsPtr->points.size()<<" 三角形个数:"<<PolygonMeshAngleSum.polygons.size()<<endl;
	//////Viewer->addPolygonMesh(PolygonMeshAngleSum, "DelaunayGrowthAngleMaximizerAngleSum");
	//////

	////PointBase::MeshToTriangles(Viewer, StemPointsPtr, PolygonMesh, "DelaunayGrowthAngleMaximizer");

	////////////////测试三角网生长