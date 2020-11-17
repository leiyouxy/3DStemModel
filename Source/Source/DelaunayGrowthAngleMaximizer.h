//2015.07.01 ���ݻ��߶Խ���󻯹���Delaunay������
////ƽ����
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
	//�����Octree�� ��һ��ʹ��
	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> * Octree;

	//��Ҫ��չ�ı�
	vector<EdgeNodes> NeedExpandEdges;
	
	//ͨ��������Ƕ����ķ�ʽ����һ���� ���صĵ����������� OneIndex->TwoIndex ���������
	//�� OneIndex->TwoIndex TwoIndex->ThreeIndex ThreeIndex->OneIndex ����ʱ�뷽��
	int FindNextPointByAngleMax(int OneIndex, int TwoIndex);	

	//���������
	void AddTriangle(pcl::PolygonMesh & output,
		int OneIndex, int TwoIndex, int ThreeIndex, bool NeedFindEdge = true);

	void ExpandEdge(pcl::PolygonMesh & output, 
		int ConstantIndex, //�Ѿ��̶���һ������
		int OneIndex, int TwoIndex, bool NeedFindEdge = true); //����չ����������

	//�Ƿ���ȫ�ֵı߽�� //
	bool IsBorderEdge(int OneIndex, int TwoIndex);

	//�жϱ��Ƿ�����Ҫ��չ�ı���
	bool EdgeIsInNeedExpaned(int OneIndex, int TwoIndex);
	
	//�ӵ㼯��Ѱ�Ҿ���OneIndex����ĵ�
	int FindNearestPoint(int OneIndex);

	//2015.07.15 ��CopyIndexCloud������Ѱ��IndexΪIndex�ĵ���
	int GetTempIndexFromCopyIndexCloud(int Index);

	//2015.07.15 ������չ�Ķ���
	int PriorityIndex;	//���������������

	//2015.07.15 ����������Ȩ�ı��ƶ���ǰ����
	void ForwardMove();

	//2015.07.15 ��CopyIndexCloudɾ��IndexΪIndex�Ľڵ�
	void RemovePointsFromCopyIndexCloud(int Index);

	//2015.07.23 �ж��������Ƿ��ظ�
	bool FindTriangleIsExists(pcl::PolygonMesh & output, int OneIndex, int TwoIndex, int ThreeIndex);

	//2015.07.23 �жϱ��Ƿ���Ҫ��չ
	bool FindEdgeNeedExpand(pcl::PolygonMesh & output, int OneIndex, int TwoIndex);

	//2015.07.23 ��ĽǶȺͳ�ʼ��
	void SumAngleInitial();
public:
	CDelaunayGrowthAngleMaximizer()
	{
		TheSpecifiedPoint = -1;	
	}

	//�㼯͹�����ϣ����������������ʱ���ʼ��
	vector<int> HullPoints;

	//������������
	void SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPtrValue);

	void performReconstruction(pcl::PolygonMesh & output, 
		//�Ƿ�ʹ�ýǶ����׼��  ʣ�µ�����������Ҫ������ѡ�����������
		bool UsePointSumAngleValue = true); //�Ƿ�ʹ�õ���ǶȺ�Ϊ360

	//2015.07.23
	//ʹ�þ����㷨���� �Ե㼯��һ�� �� ������� ���� �������������жϱ��Ƿ���Ҫ��չ
	void performReconstructionRandomBaseEdge(pcl::PolygonMesh & output,
		bool UsePointSumAngleValue = false, bool FindTriangleValue = true);

	////2016.01.11 ָ��һ���㣬����������ȼ��㣬����󼴷����ؽ���������ж�������Ƿ��Ѿ����
	int TheSpecifiedPoint;

	//���� Index ����� �Ƿ��Ѿ��Ƿ�յ�
	bool GetPointIsClosed(int PointIndex, vector<int> & NeighborIndexs);
};

#endif


	//����ʹ�õĲ���

	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints5000.pcd", StemPointsPtr);	//9964 ��������
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints10000.pcd", StemPointsPtr);	//19961 ��������
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints15000.pcd", StemPointsPtr);		//29958 ��������
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints20000.pcd", StemPointsPtr);	//39952 ��������
	
	//////pcl::PolygonMesh PolygonMeshClassic;	
	//////CDelaunayGrowthAngleMaximizer RandomBaseEdge;
	//////RandomBaseEdge.SetInputs(StemPointsPtr);
	////////RandomBaseEdge.performReconstructionRandomBaseEdge(PolygonMeshClassic, false, false);	//�������Ƿ���Ҫ��չ
	////////cout<<"�����:"<<StemPointsPtr->points.size()<<" �����θ���:"<<PolygonMeshClassic.polygons.size()<<endl;

	//////RandomBaseEdge.performReconstructionRandomBaseEdge(PolygonMeshClassic, false, true);
	//////cout<<"�����:"<<StemPointsPtr->points.size()<<" �����θ���:"<<PolygonMeshClassic.polygons.size()<<endl;

	//////Viewer->addPolygonMesh(PolygonMeshClassic, "PolygonMeshClassic");

	//////PointBase::PointCoordinateTransform(StemPointsPtr, 0, 10, 0);
	//////pcl::PolygonMesh PolygonMesh;
	//CDelaunayGrowthAngleMaximizer DelaunayGrowthAngleMaximizer;
	//DelaunayGrowthAngleMaximizer.SetInputs(StemPointsPtr);
	//DelaunayGrowthAngleMaximizer.performReconstruction(PolygonMesh);
	//cout<<"�����:"<<StemPointsPtr->points.size()<<" �����θ���:"<<PolygonMesh.polygons.size()<<endl;
	//Viewer->addPolygonMesh(PolygonMesh, "DelaunayGrowthAngleMaximizer");

	//////PointBase::PointCoordinateTransform(StemPointsPtr, 0, 20, 0);
	//////pcl::PolygonMesh PolygonMeshAngleSum;	
	//////CDelaunayGrowthAngleMaximizer DelaunayGrowthAngleMaximizerAngleSum;
	//////DelaunayGrowthAngleMaximizerAngleSum.SetInputs(StemPointsPtr);
	//////DelaunayGrowthAngleMaximizerAngleSum.performReconstruction(PolygonMeshAngleSum, true);
	//////cout<<"�����:"<<StemPointsPtr->points.size()<<" �����θ���:"<<PolygonMeshAngleSum.polygons.size()<<endl;
	//////Viewer->addPolygonMesh(PolygonMeshAngleSum, "DelaunayGrowthAngleMaximizerAngleSum");	


	////////////////�������������� I:\\ProcessedPoints\\PlanePointsRGBPtr.pcd
	
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\PlanePointsRGBPtr6977.pcd", StemPointsPtr);	
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints5000.pcd", StemPointsPtr);	//9964 ��������
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints10000.pcd", StemPointsPtr);	//19961 ��������
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints15000.pcd", StemPointsPtr);		//29958 ��������
	//PointBase::OpenPCLFile("I:\\ProcessedPoints\\120PlanePoints20000.pcd", StemPointsPtr);	//39952 ��������
	
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
	cout<<"�����θ���:"<<PointPolygonMesh.polygons.size()<<endl;
	Viewer->addPolygonMesh(PointPolygonMesh, "DelaunayGrowth");
	*///PointBase::PointCoordinateTransform(StemPointsPtr, 0, 3, 0);
	//PointBase::MeshToTriangles(Viewer, StemPointsPtr, PointPolygonMesh, "DelaunayGrowth");

	//////pcl::PolygonMesh PolygonMeshClassic;	
	//////CDelaunayGrowthAngleMaximizer RandomBaseEdge;
	//////RandomBaseEdge.SetInputs(StemPointsPtr);
	////////RandomBaseEdge.performReconstructionRandomBaseEdge(PolygonMeshClassic, false, false);	//�������Ƿ���Ҫ��չ
	////////cout<<"�����:"<<StemPointsPtr->points.size()<<" �����θ���:"<<PolygonMeshClassic.polygons.size()<<endl;

	//////RandomBaseEdge.performReconstructionRandomBaseEdge(PolygonMeshClassic, false, true);
	//////cout<<"�����:"<<StemPointsPtr->points.size()<<" �����θ���:"<<PolygonMeshClassic.polygons.size()<<endl;

	//////Viewer->addPolygonMesh(PolygonMeshClassic, "PolygonMeshClassic");

	//////PointBase::PointCoordinateTransform(StemPointsPtr, 0, 10, 0);
	//////pcl::PolygonMesh PolygonMesh;
	//////CDelaunayGrowthAngleMaximizer DelaunayGrowthAngleMaximizer;
	//////DelaunayGrowthAngleMaximizer.SetInputs(StemPointsPtr);
	//////DelaunayGrowthAngleMaximizer.performReconstruction(PolygonMesh);
	//////cout<<"�����:"<<StemPointsPtr->points.size()<<" �����θ���:"<<PolygonMesh.polygons.size()<<endl;
	//////Viewer->addPolygonMesh(PolygonMesh, "DelaunayGrowthAngleMaximizer");

	//////PointBase::PointCoordinateTransform(StemPointsPtr, 0, 20, 0);
	//////pcl::PolygonMesh PolygonMeshAngleSum;	
	//////CDelaunayGrowthAngleMaximizer DelaunayGrowthAngleMaximizerAngleSum;
	//////DelaunayGrowthAngleMaximizerAngleSum.SetInputs(StemPointsPtr);
	//////DelaunayGrowthAngleMaximizerAngleSum.performReconstruction(PolygonMeshAngleSum, true);
	//////cout<<"�����:"<<StemPointsPtr->points.size()<<" �����θ���:"<<PolygonMeshAngleSum.polygons.size()<<endl;
	//////Viewer->addPolygonMesh(PolygonMeshAngleSum, "DelaunayGrowthAngleMaximizerAngleSum");
	//////

	////PointBase::MeshToTriangles(Viewer, StemPointsPtr, PolygonMesh, "DelaunayGrowthAngleMaximizer");

	////////////////��������������