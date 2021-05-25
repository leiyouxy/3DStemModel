#pragma once

/*

�������ɴ�ֱ�ֶε���������û�����ƽ��ı����ؽ��㷨���ؽ����ɴ����ȵ����ɱ���ģ�ͣ�
Ȼ�����ԭʼ���ɵ�����������ɱ����ļ���λ�ù�ϵ�ؽ�����ϸ���ȵ����ɱ���ģ�ͣ��˲���������Ϊ��
��ԭʼ���ɵ����������Ĵ����ȵ����ɱ���ģ��ͶӰ��
����ͶӰ��ϵ�������ȵ����ɱ�����������ʷֵõ�ϸ���ȵ����ɱ���ģ�ͣ�
������Щ���ɵ��ƿ���Ҫ������������Ҫ��ʵ��ʱ�����ľ������⣩

2020.12.31

*/
#include "AnglePartition.h"
#include "DelaunayGrowthWithHalfEdge.h"

class CSurfaceReconstructionByTangentPlane
{
private:
	//���ɵ��Ƶ����ɴ�ֱ�ֶεĽǶȷ��� 2020.12.31
	CAnglePartition * p_AnglePartition;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OriginalStemPoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfilePoints;

	V_HE_Face CoarseSurface; //�����ȵ����ɱ���ģ�ͣ����� ProfilePoints ������ƽ������ؽ��㷨�õ�
	V_HE_Face FineSurface;  //ϸ���ȵ����ɱ���ģ�� �� OriginalStemPoints �еĲ��ֵ� ȷ���ı���ģ�ͣ�
	
	CSurfaceReconstructionByTangentPlane();
	~CSurfaceReconstructionByTangentPlane();
public:
	void SetInput(CAnglePartition * p_AnglePartitionValue, 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputStemPoints);

};