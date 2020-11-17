#pragma once
#ifndef BranchRemoval_H
#define BranchRemoval_H

#include "TreeBase.h"
#include "HorizontalPartition.h"
#include "AnglePartition.h"

#include "GeneratedFiles/ui_BranchRemovalByTangentPlane.h"

//ÿ���Ƕȷ�������ƽ���������
typedef struct AnglePartitionTargentInfo
{
	bool Calculated;	// is calculated 
	int AngleIndex;		//�Ƕȷ���������	
	double Symbol; //������ţ����ŵ���������ƽ��Ĳ������������ĵ�ȷ����Ȼ�����෴�ķ��Ų��Ƴ���辵�
	double a;
	double b;
	double c;
	double d;	
			
	double LineParameterSlope;			//ֱ�߲������̵Ĳ�������ֱ���ǹ��������ĵ���Ƕȷ������ĵ��ֱ�߷���
	double DisFromCenterToGeoCenter;	//2015.08.15 �������ĵ���Ƕȷ������ĵ�ľ���	
	double ExtendValue;					//2016.03.17 ������ƽ��λ��ʱʹ�õ��ӳ���ľ���ֵ
	pcl::PointXYZRGB CenterPoint;		//������ƽ��ʱ�õ������ĵ�
	pcl::PointXYZRGB TargentPoint;		//��λ��ƽ��ʱ�õ��ĵ㣨�ӳ��㣩���˵������ĵ���ͬһ��ֱ���ϣ��Ҿ������ĵ����ΪL�������趨��
			//��ֱ�������ĵ��������ߵ�xy�����ĵ��zֵȷ��	
};

//ÿ�������ĽǶ���ƽ������
typedef struct SectionTargentInfo
{
	int SectionIndex;
	double AvgDis;					//2015.08.15 ͳ�� ������ƽ��ʱ�õ������ĵ� �� �������ĵ� ��ƽ������
	double DisVariance;				//2015.08.15 ��������ķ���
	vector<AnglePartitionTargentInfo> AngleSTangentInfo;
};


class CBranchRemovalByTangentPlane : public CTreeBase
{
	Q_OBJECT //Must be used, otherwise the slot and signal do not work
private:
	Ui::BranchRemovalByTangentPlaneForm BranchRemovalForm;
	const string BranchPointsStr = " BranchPoints";
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints;
	const string HeightPlanePointsStr = "HeightPlanePoints";

	const string CenterPointsPointsStr = "CenterPointsPoints";

	//��������ĽǶȻ���
	CAnglePartition AnglePartition;
	CHorizontalPartition HorizontalPartition;

	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> * Octree;

	//�����ֱ�����ĽǶȷ�������ƽ������
	vector<SectionTargentInfo> SectionSTangentInfo;
	int SearchStartIndex;
	int SuccessiveSectionNumber;
	double Thickness;
	double Angle;	
	double AlongDistance;
	bool IsUp;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CenterPointsPtr;
	const string CenterPointsStr = "CenterPointsStr";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TargentPointsPtr;
	const string TargentPointsStr = "TargentPointsStr";

	//Except for the stem points, all other points are outliers points
	int OutliersColor;

	void GetParameters();

	void Initial();

	//2015.07.30 �Ƴ���ǰ�����������Ϣ��	
	void MarkSectionsBranches(int SectionIndex);

	//2015.07.31 ���� SectionIndex ���� ����ƽ�����
	void CalcSectionSTangent(int SectionIndex);

	void CalcSectionSAnglePartitionTargent(int SectionIndex,
		int AnglePartitionIndex);

	void GetPoint(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr,
		int SectionIndex,
		int AnglePartitionIndex);

	//��ȡ������ƽ��ĵ������ݼ�
	void GetTargentPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr AnglePartitionIndexPointPtr,	//��ǰ�Ƕȷ����ĵ���
		int SectionIndex,
		int AnglePartitionIndex,
		int AnglePartitions = 3,	//ʹ�ýǶȷ����ĸ������˲���Ӧ��Ϊ������
		int Sections = 10,			//ʹ�ô�ֱ�����ĸ�����
		int SectionSpace = 10);		//���ַ����ļ����
	
	//void UpdateGeometricalCenterAndReAnglePartition(int SectionIndex);
	void ShowAngularCenterPoints();

signals:

public Q_SLOTS:
	void ShowSlicesPoints(int CheckValue);

	void ShowHeightPlane(int Value);

	void CheckBranches();

	void Redo();

	void RemoveBranches();
public:	
	CBranchRemovalByTangentPlane(QGroupBox * ParentWin);
	~CBranchRemovalByTangentPlane();

	void SetViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue);
	void RefreshData();
};

#endif