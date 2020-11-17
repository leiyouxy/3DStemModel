/*
2020.06.08 ���� ���ȵ�Ϊ���ĵ�Delaunay�����������㷨 ����������
���㼯һ��Ϊ��

*/

#pragma once
#include "DelaunayGrowthWithHalfEdge.h"

typedef struct DiviedPointsAndBorderEdge
{
	vector<int> PartIndexs;
	V_HE_Edge CurBorderEdges;	//��㼯����ƥ��ı߽��
	pcl::PolygonMesh CurMesh;	//��ǰ��ȡ��Mesh
	//V_HE_Edge BridgeEdges;
	Point_HE_Edge CurPointEdges;	//�����˵�ǰ�ĵ㼯�ĽǶȺͰ����Ϣ2020.06.22

}DiviedPointsAndBorderEdge;

class CDividePointsByDelaunayLine : public CDelaunayGrowthWithHalfEdge
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudInLine;

	double TextScale;

	//��Index��浽�ļ���   2020.06.24
	void SaveIndexsToFile(vector<int> Indexs, string FileName);

	//δ�պϵĵ� �� ��Ӧ�İ����Ϣ 2020.06.22
	vector<int> UnClosedIndexs;
	Point_HE_Edge UnClosedEdges;
	//δ�պϵĵ� �� ��Ӧ�İ����Ϣ 2020.06.22

	VectorBase<int> VectorBaseInt;

	pcl::PointXYZRGB CenterPoint;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalIndexCloud;	//ȫ�ֵ��Ƽ�
	vector<int> LocalSetIndexs;									//��ǰ��Ҫ����ĵ����ȫ�ֵ��Ƽ��е�����
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr HullPointsInLine;

	//�ָ����Ҳ�ĵ㣬�ָ����Ϸ��㣬�·��㣬�Ҳ�㹹����ʱ��Ĵ���
	vector<int> LeftCloudIndexs;
	vector<int> RightCloudIndexs;
	//2020.06.13 ��¼�����������ϵĵ㣬�����ܱ��������Ѿ��������
	vector<int> L_ClosedPointIndexs;
	vector<int> R_ClosedPointIndexs;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr LeftCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RightCloud;
	V_HE_Edge RightBorderEdges;	//�Ҳ�߽��
	V_HE_Edge LeftBorderEdges;	//���߽��

	V_HE_Edge TempBridgeEdges;	//���߽��
	//V_HE_Edge InputBorderEdges;	//������ı߽�� ���� BorderEdges

	bool ShowGuidLine;			//�Ƿ���ʾ����ֱ��
	bool ShowGuidPolyLine;		//�Ƿ���ʾ��������
	bool ShowBorderEdge;		//�Ƿ���ʾ�߽��
	bool ShowOuterGuidPoints;	//�Ƿ���ʾ�ⲿ��������

	//�������͵ڶ���������
	void GetDirections();

	//��ȡ���ֵ�����������
	void GetHalfGuidStartAndEndPoints();

	//���������� 2020.06.07
	void GuidPolyLineMesh();

	//ָ���ָ���
	V_GuidPoints G_GuidPoints;

	pcl::PointXYZRGB MaxDirectionStartPoint, MaxDirectionEndPoint,
		SecDirectionStartPoint, SecDirectionEndPoint;
	
	//���ݵ㼯�����䶯���򣬼��㻮�ֵ���ʼ��  2020.06.05
	void GetStartAndEndPointForDividing();

	//���ݵ㵽��ʼ����ֹ���ͶӰ���µ����ȵ��ͶӰ�������ӽ����յ㣬���ҵ㵽ͶӰ��ľ����С 2020.05.29 
	//Ĭ��ǰ��Ѱ��Forward=true�����������Ҳ���ܺ���Ѱ��Forward=false��
	int FindPriorityPoint(int CurrentPointIndex, int StartIndex, int EndIndex, bool Forward = true);
	
	//����ָ���ߵĻ���
	void performDividByGuidPoints();

	//��ȡ Numbers �� GuidPoints���������㼯����Ϊ Numbers + 1�� 2020.06.01
	//��ʱδʹ�� 2020.06.17
	void GetGuidPoints(int Numbers);

	//������β����ֱ�߽��㼯�еĵ�һ��Ϊ�� 2020.06.08
	void DividePointsByLine();

	//�������������γɵ��������жϵ������㼯 2020.06.08
	void DividePointsByPolyLine();

	//�ڹ�����Mesh��������ǰ TempPointIndex �� GuidPath�е� ��Щ���γ������Σ�����ֵ����1��
	//������ֵ����1�������������У� 2020.06.09;
	vector<int> FindGuidPathPointIndex(int TempPointIndex);

	//�����˵�ķָ��ϵ������õ����ڱߵ�EndIndex�����������ߣ�����Դ���  2020.06.09��Ŀǰ�����������Դ�Ϊ���ı�
	//Type = 0 ����RightCloud��= 1 ����LeftCloud
	void DivideBroadcast(int TempPointIndex, int Type);
	
	//Type = 0 ��ȡ�Ҳ�߽�ߣ�= 1 ���ȡ���߽�� 2020.06.10 
	void GetBorderEdge();

	//������ֵ㼯ʱ������Mesh����MeshΪȫ��Mesh�����ⲿ���� �� OutDivideMesh ���㼯��ʼ����2020.06.10
	void OutputDivideMesh(pcl::PolygonMesh & OutDivideMesh);	

	//2020.06.17 ���ֱ߽���еĻ�·����Ӹ����������Ƴ���·
	void FindCircleInBorderEdges(V_HE_Edge & CurEdge);

	//2020.06.17 Ѱ����StartIndex��ʼ�ڵ�ı߽��
	int FindEdgeByStartIndex(V_HE_Edge CurEdge, int StartIndex);

public:
	CDividePointsByDelaunayLine();
	~CDividePointsByDelaunayLine();

	void SetInputIndexCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue,
		vector<int> & LocalSetInGlobalIndex,
		V_HE_Edge InputBorderEdgesValue);

	//2020.06.08
	//void HalfDivide(vector<pcl::PointCloud<PointXYZRGBIndex>::Ptr> & OutPoints,
	void HalfDivide(vector<DiviedPointsAndBorderEdge> & OutPointsAndBorderEdges,
		pcl::PolygonMesh & GlobalMesh, 		
		bool ShowGuidLine = false, bool ShowGuidPolyLine = false,
		bool ShowBorderEdge = false);	

	//�� InputPointsAndBorderEdges �����벢�м��� 2020.06.11
	void PerformParallel(vector<DiviedPointsAndBorderEdge> InputPointsAndBorderEdges,
		pcl::PolygonMesh & GlobalMesh);

	//���ݴ���㼯LocalIndexs�� �� ��߽ṹCurEdges ���� ȫ�ְ�߽ṹ Point_HE_edges 2020.06.22
	void RefreshPointAngleAndEdgesInGlobal(vector<int> LocalIndexs, Point_HE_Edge CurEdges);


};
