#pragma once

#include <vector>
#include "CommClass.h"
#include "ContourAndConvexHull.h"
#include "CommGeometry.h"

using namespace std;

// 2020.5.25 ��߽ṹ�����ݽṹ
typedef struct HE_Edge
{
	int EdgeIndex;			//����Edges�е� ����
	int StartIndex;			//��ߵ���� ����	
	int EndIndex;			//��ߵ��յ� ����	
	int OppositeEdgeIndex;	//��Ӧ��ߵ� ����
	//int FaceIndex;			//�����������
	int NextEdgeIndex;		//��һ���� �� ���� �ɴ˿�֪��������
	double StartAngle;		//���ĽǶȺ�
	double EndAngle;		//�յ�ĽǶȺ�
}HE_Edge;

//�ߵļ���
typedef vector<HE_Edge> V_HE_Edge;

typedef struct P_HE_Edge
{	
	bool IsGuidPoint;	//�Ƿ������������ϵĵ�
	int DivideType;		//�Ƿ��Ѿ�����Ϊ���ҵ㼯��-1δ���֣�0��ʾRight 1��ʾLeft
	long double SumAnlge;	//�ǶȺ�	
	V_HE_Edge V_HE_Edges; //�Ե�ǰ��Ϊ������İ��	
} P_HE_Edge;

//����Ϊ��ʼ��ıߵļ���,2020.05.27 ʹ�ú�ɼ��ٱߵļ���Ч�ʣ���߼����ٶ�
typedef vector<P_HE_Edge> Point_HE_Edge;

typedef struct
{
	int EdgeIndexs[3];
}HE_Face;

//��������ļ���
typedef vector<HE_Face> V_HE_Face;

typedef struct
{
	int StartEdge0;			//��ʼ͹���ߵ���ʼ����	
	int StartEdge1;			//��ʼ͹���ߵĽ�������

	int EndEdge0;			//����͹���ߵ���ʼ����
	int EndEdge1;			//����͹���ߵĽ�������

	double SmallStartDis;	//����Զ�˵�1	���������
	double SmallEndDis;		//����Զ�˵�2	���������

	pcl::PointXYZRGB StartPoint0;	//�������������Զ�˵�1	
	pcl::PointXYZRGB EndPoint0;		//�������������Զ�˵�2

	vector<int> Path;			//�ָ���������Ҳ����·�����ȵ㣬������ʼ�˵�	
} PairPointIndexs;

typedef vector<PairPointIndexs> V_GuidPoints;

class CDelaunayGrowthWithHalfEdge
{
private:
	
protected:
	bool UseCout;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;

	//����Ѱ�ҵ�����ĵ��Ƽ���ÿ���һ�����ȵ㣬ɾ�����е�һ���㣬��˵㼯�𽥼���
	pcl::PointCloud<PointXYZRGBIndex>::Ptr CopyIndexCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr HullPoints;
	
	//2020.06.01
	pcl::PointXYZRGB MaxDirection, SecondDirection;

	bool BorderEdgeExpanded;

	//���а�ߵļ���
	Point_HE_Edge Point_HE_edges;

	//�߽�ߣ��˴���Ҫ�ѱ߽�ߵķ�������� 2020.06.15
	V_HE_Edge BorderEdges;
	
	//��Ҫ��չ�İ�߼���
	V_HE_Edge NeedExpandEdges;
	
	//��ļ���
	V_HE_Face T_Faces;

	//�㼯͹�����ϣ����������������ʱ���ʼ��
	vector<int> HullPointsIndex;

	//��¼�ǶȺ�
	vector<double> SumAngle;
	
	//2015.07.23 ��ĽǶȺͳ�ʼ��
	void Initial();

	//2015.07.15 ������չ�Ķ���
	int PriorityIndex;	
	int InitialPriorityIndex;

	double AllowAngle;

	//ͨ��������Ƕ����ķ�ʽ����һ���� ���صĵ����������� OneIndex->TwoIndex ���������
	//�� OneIndex->TwoIndex TwoIndex->ThreeIndex ThreeIndex->OneIndex ����ʱ�뷽��
	int FindNextPointByAngleMax(HE_Edge Base_Edge);

	//��ӵ�Face�У�����Ӵ���չ�б�  2020.05.25
	void AddFace(HE_Edge Base_Edge, int ThirdPointIndex, 
		int GuidLineIndex = -1, bool IsExpandReverseEdge = false);

	//2015.07.15 ����������Ȩ�ı��ƶ���ǰ����
	void ForwardMove();

	//2020.05.26 ��չһ���ߣ����ص��ǵ������������
	int ExpandEdge(HE_Edge Base_Edge, bool NeedFindEdge = false);

	//2020.05.26 �ڴ���չ�б���Ѱ���Ƿ�
	int FindEdgeinEdges(V_HE_Edge HE_Edges, HE_Edge Base_Edge);	



	int GetTempIndexFromCopyIndexCloud(int Index);

	//2020.05.26 �Ƿ��Ǳ߽�ߣ�����Ǳ߽�ߣ�����չ�䷴���
	bool IsBorderEdge(HE_Edge Base_Edge);

	//2020.06.12 ����͹��
	void CalcConvexHull();

	//2020.06.12 ����߽�ߵĶ˵�ĽǶȺ�
	void RefreshBorderEdgePointAngle();

	//2020.06.12 �ڱ߽����Ѱ����һ���߽��
	//int FindNexeBorderEdge(HE_Edge Cur_Edge);

	//�Ըõ�Ϊ���Ľ��й�����ֱ���޷���չ 2020.06.22
	void PerformWithOnePoint(int TempCurPointIndex);
public:
	CDelaunayGrowthWithHalfEdge();
	~CDelaunayGrowthWithHalfEdge();

	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr);	

	//2020.05.26 �ߣ�TempStartIndex��TempEndIndex���Ƿ���HE_Edges��
	int FindEdgeinEdges(V_HE_Edge HE_Edges, int TempStartIndex, int TempEndIndex);

	//�������ȵ�Ĺ���
	void performReconstructionWithPriority();
		
	//���ñ߽�� 2020.05.26 �߽�ߵķ����������������ķ���������߲���չ
	void SetOuterBorderEdges(V_HE_Edge BorderEdgesValue);

	//���
	void OutputMesh(pcl::PolygonMesh & MeshOut);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;

	//���δ�պϵĵ�����������Ӧ�ı߽���Ϣ 2020.06.22
	void OutPutPointAngleAndEdges(Point_HE_Edge & CurEdges);
};