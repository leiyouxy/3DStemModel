/*
2020.06.08 采用 优先点为中心的Delaunay三角网构网算法 沿着引导线
将点集一份为二

*/

#pragma once
#include "DelaunayGrowthWithHalfEdge.h"

typedef struct DiviedPointsAndBorderEdge
{
	vector<int> PartIndexs;
	V_HE_Edge CurBorderEdges;	//与点集索引匹配的边界边
	pcl::PolygonMesh CurMesh;	//当前获取的Mesh
	//V_HE_Edge BridgeEdges;
	Point_HE_Edge CurPointEdges;	//保存了当前的点集的角度和半边信息2020.06.22

}DiviedPointsAndBorderEdge;

class CDividePointsByDelaunayLine : public CDelaunayGrowthWithHalfEdge
{
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudInLine;

	double TextScale;

	//将Index另存到文件中   2020.06.24
	void SaveIndexsToFile(vector<int> Indexs, string FileName);

	//未闭合的点 与 对应的半边信息 2020.06.22
	vector<int> UnClosedIndexs;
	Point_HE_Edge UnClosedEdges;
	//未闭合的点 与 对应的半边信息 2020.06.22

	VectorBase<int> VectorBaseInt;

	pcl::PointXYZRGB CenterPoint;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalIndexCloud;	//全局点云集
	vector<int> LocalSetIndexs;									//当前类要处理的点击在全局点云集中的索引
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr HullPointsInLine;

	//分割线右侧的点，分割线上方点，下方点，右侧点构成逆时针的次序
	vector<int> LeftCloudIndexs;
	vector<int> RightCloudIndexs;
	//2020.06.13 记录非引导折线上的点，但其周边三角形已经计算完毕
	vector<int> L_ClosedPointIndexs;
	vector<int> R_ClosedPointIndexs;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr LeftCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RightCloud;
	V_HE_Edge RightBorderEdges;	//右侧边界边
	V_HE_Edge LeftBorderEdges;	//左侧边界边

	V_HE_Edge TempBridgeEdges;	//左侧边界边
	//V_HE_Edge InputBorderEdges;	//新输入的边界边 并入 BorderEdges

	bool ShowGuidLine;			//是否显示引导直线
	bool ShowGuidPolyLine;		//是否显示引导折线
	bool ShowBorderEdge;		//是否显示边界边
	bool ShowOuterGuidPoints;	//是否显示外部的引导点

	//计算最大和第二方向向量
	void GetDirections();

	//获取二分的上下引导点
	void GetHalfGuidStartAndEndPoints();

	//生成引导线 2020.06.07
	void GuidPolyLineMesh();

	//指导分割点对
	V_GuidPoints G_GuidPoints;

	pcl::PointXYZRGB MaxDirectionStartPoint, MaxDirectionEndPoint,
		SecDirectionStartPoint, SecDirectionEndPoint;
	
	//根据点集的最大变动方向，计算划分的起始点  2020.06.05
	void GetStartAndEndPointForDividing();

	//根据点到起始和终止点的投影，新的优先点的投影点必须更接近于终点，而且点到投影点的距离更小 2020.05.29 
	//默认前向寻找Forward=true，特殊情况下也可能后向寻找Forward=false；
	int FindPriorityPoint(int CurrentPointIndex, int StartIndex, int EndIndex, bool Forward = true);
	
	//基于指导线的划分
	void performDividByGuidPoints();

	//获取 Numbers 组 GuidPoints，将整个点集划分为 Numbers + 1份 2020.06.01
	//暂时未使用 2020.06.17
	void GetGuidPoints(int Numbers);

	//根据首尾引导直线将点集中的点一份为二 2020.06.08
	void DividePointsByLine();

	//根据引导折现形成的三角形判断点所处点集 2020.06.08
	void DividePointsByPolyLine();

	//在构建的Mesh中搜索当前 TempPointIndex 与 GuidPath中的 哪些点形成三角形，返回值大于1个
	//若返回值大于1个，则按升序排列， 2020.06.09;
	vector<int> FindGuidPathPointIndex(int TempPointIndex);

	//传播此点的分割关系，如果该点所在边的EndIndex不是引导折线，则可以传播  2020.06.09，目前传播仅限于以此为起点的边
	//Type = 0 属于RightCloud，= 1 则是LeftCloud
	void DivideBroadcast(int TempPointIndex, int Type);
	
	//Type = 0 获取右侧边界边，= 1 则获取左侧边界边 2020.06.10 
	void GetBorderEdge();

	//输出划分点集时产生的Mesh，此Mesh为全局Mesh，由外部函数 对 OutDivideMesh 做点集初始化，2020.06.10
	void OutputDivideMesh(pcl::PolygonMesh & OutDivideMesh);	

	//2020.06.17 发现边界边中的回路并添加该三角形以移除回路
	void FindCircleInBorderEdges(V_HE_Edge & CurEdge);

	//2020.06.17 寻找以StartIndex开始节点的边界边
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

	//对 InputPointsAndBorderEdges 的输入并行计算 2020.06.11
	void PerformParallel(vector<DiviedPointsAndBorderEdge> InputPointsAndBorderEdges,
		pcl::PolygonMesh & GlobalMesh);

	//根据传入点集LocalIndexs的 与 半边结构CurEdges 更新 全局半边结构 Point_HE_edges 2020.06.22
	void RefreshPointAngleAndEdgesInGlobal(vector<int> LocalIndexs, Point_HE_Edge CurEdges);


};
