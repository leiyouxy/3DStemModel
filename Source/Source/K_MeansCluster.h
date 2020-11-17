#pragma once

/*

2019.09.20

Ϊ���ڴ����Ե����ʽ��Ϊ��������

K-Means Cluster ����


*/

#include "CommPointBase.h"
#include "CommGeometry.h"

struct MeansCluster				//��������ݽṹ
{
	int ClusterIndex;				//������
	double CurSquareErrorOfDis;		//��ǰ�����е㵽���ĵ�ľ���
	pcl::PointXYZRGB ClusterCenter;	//�������ģ���������ɫ��ʾ
	vector<int> PointIndexS;		//��������ĵ�����
};

struct DisOfPointToCluster
{
	int PointIndex;			//�������		
	int ClusterIndex;		//�ص�����
	double Dis;
};

class CK_MeansCluster 
{
public:
	CK_MeansCluster();
	~CK_MeansCluster();

private:	
	int ClusterNumber;						//��ĸ���
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud;
	vector<MeansCluster> ClusterResults;	//������������

	double DisDIMENSION;					//���٣�Ϊͳһ��ͬ��ֵ��λ�µ�����
	double GetDisDIMENSION();				//�д��ڸĽ��������Ǻܺ� 2019.09.20
	
	void PreProcessingForCluster();			//����ǰ��Ԥ������ѡ����ʵľ��࿪ʼ��

	//Ѱ���� �� ClusterIndex �������������ĵ� ��Ҫ���ھ������ĵĳ�ʼ������
	DisOfPointToCluster GetMaxDisBetweenPointAndCluster(int ClusterIndex);

	//Ѱ�ҵ��������ľ�����С�ĵ�
	int GetMinDis(vector<DisOfPointToCluster> DisSOfPointToCluster);

	double ClusterIteration();				//ÿһ�ξ������
	
	double RefreshClusterData(int ClusterIndex);	//���� ClusterIndex ������ݣ��������¾����ĵ�ľ���
public:
	void SetInput(vector <double> Values);
	void SetInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloudValue);
	void RunCluster(int Number, double Threshold = 0.01);			//�������Ϊ Number ��
	void SetClusterColors();
	void ShowCluster(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer, int PointSize,
		string PointsStr = "");
	//����ĳһ����ĵ�����
	void GetClusterPoints(int Index, vector<int> & Indexs);
	void GetClusterInfo(int Index, MeansCluster & Cluster);
	MeansCluster GetClusterInfo(int Index);
	int GetClusterNumbers();
};
