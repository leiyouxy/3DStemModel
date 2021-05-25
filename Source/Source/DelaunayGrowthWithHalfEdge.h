#pragma once

#include <vector>
#include "CommClass.h"
#include "ContourAndConvexHull.h"
#include "CommGeometry.h"

using namespace std;

// 2020.5.25 半边结构的数据结构
typedef struct HE_Edge
{
	int EdgeIndex;			//所在Edges中的 索引
	int StartIndex;			//半边的起点 索引	
	int EndIndex;			//半边的终点 索引	
	int OppositeEdgeIndex;	//对应半边的 索引
	//int FaceIndex;			//所在面的索引
	int NextEdgeIndex;		//下一条边 的 索引 由此可知第三个点
	double StartAngle;		//起点的角度和
	double EndAngle;		//终点的角度和
}HE_Edge;

//边的集合
typedef vector<HE_Edge> V_HE_Edge;

typedef struct P_HE_Edge
{	
	bool IsGuidPoint;	//是否是引导折线上的点
	int DivideType;		//是否已经划分为左右点集，-1未划分，0表示Right 1表示Left
	long double SumAnlge;	//角度和	
	V_HE_Edge V_HE_Edges; //以当前点为出发点的半边	
} P_HE_Edge;

//点作为初始点的边的集合,2020.05.27 使用后可减少边的检索效率，提高计算速度
typedef vector<P_HE_Edge> Point_HE_Edge;

typedef struct
{
	int EdgeIndexs[3];
}HE_Face;

//三角形面的集合
typedef vector<HE_Face> V_HE_Face;

typedef struct
{
	int StartEdge0;			//起始凸包边的起始索引	
	int StartEdge1;			//起始凸包边的结束索引

	int EndEdge0;			//结束凸包边的起始索引
	int EndEdge1;			//结束凸包边的结束索引

	double SmallStartDis;	//距离远端点1	的最近距离
	double SmallEndDis;		//距离远端点2	的最近距离

	pcl::PointXYZRGB StartPoint0;	//根据最大方向计算的远端点1	
	pcl::PointXYZRGB EndPoint0;		//根据最大方向计算的远端点2

	vector<int> Path;			//分割点的索引，也是沿路的优先点，包括起始端点	
} PairPointIndexs;

typedef vector<PairPointIndexs> V_GuidPoints;

class CDelaunayGrowthWithHalfEdge
{
private:
	
protected:
	bool UseCout;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;

	//用以寻找第三点的点云集，每完成一个优先点，删除其中的一个点，因此点集逐渐减少
	pcl::PointCloud<PointXYZRGBIndex>::Ptr CopyIndexCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr HullPoints;
	
	//2020.06.01
	pcl::PointXYZRGB MaxDirection, SecondDirection;

	bool BorderEdgeExpanded;

	//所有半边的集合
	Point_HE_Edge Point_HE_edges;

	//边界边，此处需要把边界边的方向理清楚 2020.06.15
	V_HE_Edge BorderEdges;
	
	//需要扩展的半边集合
	V_HE_Edge NeedExpandEdges;
	
	//面的集合
	V_HE_Face T_Faces;

	//点集凸包集合，在设置输入参数的时候初始化
	vector<int> HullPointsIndex;

	//记录角度和
	vector<double> SumAngle;
	
	//2015.07.23 点的角度和初始化
	void Initial();

	//2015.07.15 优先拓展的顶点
	int PriorityIndex;	
	int InitialPriorityIndex;

	double AllowAngle;

	//通过第三点角度最大的方式找下一个点 返回的第三个点总是 OneIndex->TwoIndex 方向的正向
	//即 OneIndex->TwoIndex TwoIndex->ThreeIndex ThreeIndex->OneIndex 是逆时针方向
	int FindNextPointByAngleMax(HE_Edge Base_Edge);

	//添加到Face中，并添加待扩展列表  2020.05.25
	void AddFace(HE_Edge Base_Edge, int ThirdPointIndex, 
		int GuidLineIndex = -1, bool IsExpandReverseEdge = false);

	//2015.07.15 将具有优先权的边移动到前面来
	void ForwardMove();

	//2020.05.26 扩展一条边，返回的是第三个点的索引
	int ExpandEdge(HE_Edge Base_Edge, bool NeedFindEdge = false);

	//2020.05.26 在带扩展列表中寻找是否
	int FindEdgeinEdges(V_HE_Edge HE_Edges, HE_Edge Base_Edge);	



	int GetTempIndexFromCopyIndexCloud(int Index);

	//2020.05.26 是否是边界边，如果是边界边，则不扩展其反向边
	bool IsBorderEdge(HE_Edge Base_Edge);

	//2020.06.12 计算凸包
	void CalcConvexHull();

	//2020.06.12 计算边界边的端点的角度和
	void RefreshBorderEdgePointAngle();

	//2020.06.12 在边界边中寻找下一条边界边
	//int FindNexeBorderEdge(HE_Edge Cur_Edge);

	//以该点为中心进行构建，直至无法扩展 2020.06.22
	void PerformWithOnePoint(int TempCurPointIndex);
public:
	CDelaunayGrowthWithHalfEdge();
	~CDelaunayGrowthWithHalfEdge();

	void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr);	

	//2020.05.26 边（TempStartIndex至TempEndIndex）是否在HE_Edges中
	int FindEdgeinEdges(V_HE_Edge HE_Edges, int TempStartIndex, int TempEndIndex);

	//基于优先点的构网
	void performReconstructionWithPriority();
		
	//设置边界边 2020.05.26 边界边的方向是所在三角网的方向，其逆向边不扩展
	void SetOuterBorderEdges(V_HE_Edge BorderEdgesValue);

	//输出
	void OutputMesh(pcl::PolygonMesh & MeshOut);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer;

	//输出未闭合的点的索引及相对应的边界信息 2020.06.22
	void OutPutPointAngleAndEdges(Point_HE_Edge & CurEdges);
};