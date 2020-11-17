#ifndef ContourAndConvexHull_H
#define ContourAndConvexHull_H

//2015.06.17 ����㼯��͹�����ߵ㼯��������

//���㷨��Ҫ���������㼯�еĵ����ظ�ʱ ��δ���


#include <pcl/common/common.h>
#include <boost/function.hpp>
#include <math.h>
#include "CommClass.h"
#include "CommPointBase.h"

using namespace std;

template<typename PointNT>
class CContourAndConvexHull
{
private:
	typedef typename pcl::PointCloud<PointNT>::Ptr PointCloudPtr;
	PointCloudPtr CloudPtr;			//��Ҫ����ĵ㼯 //

	//���ǵ��㼯�����ظ���ʱ�ļ�������
	vector<int> PointsIndex;
	//�� PointsIndex ��ɾ�� ����Indexֵ
	void RemoveIndex(int Index);

	//double XMin, XMax, YMin, YMax, ZMin, ZMax;	//��Ӧ�����ֵ����Сֵ

	void Check2DDimension();		//ȷ����ά���ڵ�ƽ�棬�������xyƽ�棬�������XYƽ��
	void CovertZToY();				//��Zֵת����Y
	void CovertZToX();				//��Zֵת����X
	//vector<int> MaxMinNo;					//��ֵ�����ڵ�λ��

	int YMinIndex;
	//Yֵ��С�ĵ㣬����ж��Yֵ��С�ĵ㣬�����ȡXҲ��С�ĵ�
	int GetYMinIndex();

	// PointA ָ�� PointB ������X���γɵļн�
	double AngleOfPoints(PointNT PointA, PointNT PointB);
	//������PointA�ļн���С �Ҿ�������ĵ�
	vector<int> GetMinAngleAndDis(int Index, double & LastAngle);

public:
	CContourAndConvexHull<PointNT>()
	{
		CloudPtr.reset(new pcl::PointCloud<PointNT>);
	}

	//��ȡ�ֶε����ֵ����Сֵ
/*	void GetMaxAndMin(PointCloudPtr CloudValue, double & XMaxValue, double & XMinValue,
		double & YMaxValue, double & YMinValue,
		double & ZMaxValue, double & ZMinValue, vector<int> & MaxMinVector);	*/

	void SetInputs(PointCloudPtr CloudPtrValue);
	void GetPointsContour(vector<int> & ContourIndex, bool Is2D = true);		//��ȡ������ δʵ��
	void GetPointsConvexHull(vector<int> & HullIndex, bool Is2D = true);	//��ȡ͹��

	//2020.05.20 �����뷨����͹��
	void GetConvexHullOneByOne(vector<int> & HullIndex, bool Is2D = true);

private:

	//2020.05.21 ͹������ε�����
	pcl::PointXYZRGB CentroidPoint;

	//2020.05.20 ���ѡ����������Ϊ��ʼ͹��
	void RandomStart(vector<int> & HullIndex, bool Is2D = true);

	//2020.05.21 ����͹������ε�����
	void RefreshCentroidPoint(const vector<int> HullIndex);

	//2020.05.21 �жϵ�ǰ�� CurIndex �Ƿ�λ��͹����
	////��͹��������з��� True�����򷵻�False
	bool CurrentIsInConvexHull(const vector<int> HullIndex, int CurIndex);

	//2020.05.22 Ѱ�Ҳ��滻͹��������HullIndex �� ��Ҫ������ ͹��������
	int ReplaceConvexIndex(vector<int> & HullIndex, int CurIndex);

	//2020.05.31 �жϵ�ǰ͹�����Ƿ���͹���㣬����true���ǣ������ǣ� ��δʹ��
	bool IsConvexHull(vector<int> HullIndex, int CurConvexIndex);

	//2012.05.24 Ѱ��͹����ĺ��ʲ���λ�ã� ��δʹ��
	int FindProperPos(vector<int> & HullIndex, int CurIndex);
};


//Yֵ��С�ĵ㣬����ж��Yֵ��С�ĵ㣬�����ȡXҲ��С�ĵ�
template<typename PointNT>
int CContourAndConvexHull<PointNT>::GetYMinIndex()
{
	int Index;
	vector<int> YMinIndexVec;
	double YMinValue = CloudPtr->points[0].y;
	YMinIndexVec.push_back(0);

	for (int i = 1; i < PointsIndex.size(); i++)
	{
		if (CloudPtr->points[PointsIndex[i]].y < YMinValue)
		{
			YMinIndexVec.clear();
			YMinValue = CloudPtr->points[PointsIndex[i]].y;
			YMinIndexVec.push_back(PointsIndex[i]);
		}
		else if (CloudPtr->points[PointsIndex[i]].y == YMinValue)
		{
			YMinIndexVec.push_back(PointsIndex[i]);
		}
	}

	if (YMinIndexVec.size() == 1)
		Index = YMinIndexVec[0];
	else if (YMinIndexVec.size() > 1)
	{
		double XMinValue;
		XMinValue = CloudPtr->points[YMinIndexVec[0]].x;
		Index = YMinIndexVec[0];
		for (int i = 1; i < YMinIndexVec.size(); i++)
		{
			if (CloudPtr->points[YMinIndexVec[i]].x < XMinValue)
			{
				XMinValue = CloudPtr->points[YMinIndexVec[i]].x;
				Index = YMinIndexVec[i];
			}
		}
	}
	return Index;
}

//������PointA�ļн���С �Ҿ�������ĵ�
template<typename PointNT>
vector<int> CContourAndConvexHull<PointNT>::GetMinAngleAndDis(int Index, double & LastAngle)
{
	vector<int> MinAngleIndex;
	double AngI = 0;
	double TempAngle = 0;
	double MinAngle = 3 * M_PI;
	for (int i = 0; i < PointsIndex.size(); i++)
	{
		if (PointsIndex[i] != Index)
		{
			TempAngle = AngleOfPoints(CloudPtr->points[Index],
				CloudPtr->points[PointsIndex[i]]);

			if (TempAngle - LastAngle < 0)
				TempAngle = TempAngle + 2 * M_PI;	//�˴���Ҫ��⣬������ʱ�뷽����

			if (TempAngle - LastAngle < MinAngle)
			{
				MinAngle = TempAngle - LastAngle;
				AngI = TempAngle;
				MinAngleIndex.clear();
				MinAngleIndex.push_back(PointsIndex[i]);
			}
			else if (TempAngle - LastAngle == MinAngle) //������ʱ��ֱ���ۼ�
			{
				MinAngleIndex.push_back(PointsIndex[i]);
			}
		}
	}

	if (AngI > 2 * M_PI)
		AngI = AngI - 2 * M_PI;

	LastAngle = AngI;

	//�����������
	if (MinAngleIndex.size() > 0)
	{
		vector<double> Dis;
		for (int i = 0; i < MinAngleIndex.size(); i++)	//������룬
		{
			Dis.push_back(PointDis(CloudPtr->points[Index],
				CloudPtr->points[MinAngleIndex[i]]));
		}

		//�Ծ������� 
		for (int i = 0; i < MinAngleIndex.size() - 1; i++)
		{
			for (int j = i + 1; j < MinAngleIndex.size(); j++)
			{
				if (Dis[i] > Dis[j])
				{
					swap(Dis[i], Dis[j]);
					swap(MinAngleIndex[i], MinAngleIndex[j]);
				}
			}
		}
	}
	return MinAngleIndex;
}

// PointA ָ�� PointB ������X���γɵļн�
template<typename PointNT>
double CContourAndConvexHull<PointNT>::AngleOfPoints(PointNT PointA, PointNT PointB)
{
	double Angle;

	double YDis = PointB.y - PointA.y;
	double XDis = PointB.x - PointA.x;

	if (XDis > 0)	//�ڷ����к����Ķ������� 
	{
		if (YDis > 0)	//�ڵ�һ����
		{
			Angle = atan(YDis / XDis);
		}
		else if (YDis < 0)	//�ڵ�������
		{
			Angle = atan(YDis / XDis) + 2 * M_PI;
		}
		else	//��X��������
		{
			Angle = 0;
		}
	}
	else if (XDis < 0) //�ڷ����к����Ķ������� ����������
	{
		if (YDis != 0)	//�ڵڶ�����
		{
			Angle = atan(YDis / XDis) + M_PI;
		}
		else	//��X��������
		{
			Angle = M_PI;
		}
	}
	else  //XDis ���� 0 �����
	{
		if (YDis > 0)	//��Y��������
		{
			Angle = M_PI_2;
		}
		else if (YDis < 0)	//��Y��������
		{
			Angle = M_PI_2 * 3;
		}
		else
			Angle = 4 * M_PI;	//����λ����ͬ ��˵㲻��ҪѰ�ҵ�͹����
	}
	return Angle;
}

template<typename PointNT>
void CContourAndConvexHull<PointNT>::Check2DDimension()		//ȷ����ά���ڵ�ƽ�棬�������xyƽ�棬�������XYƽ��
{
	bool XEquality = true;
	bool YEquality = true;
	bool ZEquality = true;

	for (int i = 0; i < CloudPtr->points.size() - 1; i++)
	{
		if ((XEquality) && (CloudPtr->points[i].x != CloudPtr->points[i + 1].x)) XEquality = false;
		if ((YEquality) && (CloudPtr->points[i].y != CloudPtr->points[i + 1].y)) YEquality = false;
		if ((ZEquality) && (CloudPtr->points[i].z != CloudPtr->points[i + 1].z)) ZEquality = false;
	}
	if (XEquality)
		CovertZToX();
	else if (YEquality)
		CovertZToY();
}

template<typename PointNT>
void CContourAndConvexHull<PointNT>::CovertZToY()				//��Zֵת����Y
{
	double TempY = 0;
	for (int i = 0; i < CloudPtr->points.size(); i++)
	{
		TempY = CloudPtr->points[i].y;
		CloudPtr->points[i].y = CloudPtr->points[i].z;
		CloudPtr->points[i].z = TempY;
	}
}

template<typename PointNT>
void CContourAndConvexHull<PointNT>::CovertZToX()				//��Zֵת����X
{
	double TempX = 0;
	for (int i = 0; i < CloudPtr->points.size(); i++)
	{
		TempX = CloudPtr->points[i].x;
		CloudPtr->points[i].x = CloudPtr->points[i].z;
		CloudPtr->points[i].z = TempX;
	}
}


template<typename PointNT>
void CContourAndConvexHull<PointNT>::SetInputs(PointCloudPtr CloudPtrValue)
{
	bool FindSamePoint = false;
	CloudPtr->points.clear();
	PointsIndex.clear();

	for (int i = 0; i < CloudPtrValue->points.size(); i++)
	{
		CloudPtr->points.push_back(CloudPtrValue->points[i]);
		PointsIndex.push_back(i);
	}
	Check2DDimension();

	//��Ҫȥ���ظ��� ͨ����������
	int i = 0;
	while (i < PointsIndex.size())
	{
		int j = i + 1;
		while (j < PointsIndex.size())
		{
			//if ((CloudPtr->points[PointsIndex[i]].x == CloudPtr->points[PointsIndex[j]].x)
			//	&& (CloudPtr->points[PointsIndex[i]].y == CloudPtr->points[PointsIndex[j]].y))
			if ((abs(CloudPtr->points[PointsIndex[i]].x - CloudPtr->points[PointsIndex[j]].x)) < EPSM6
				&& (abs(CloudPtr->points[PointsIndex[i]].y - CloudPtr->points[PointsIndex[j]].y)) < EPSM6
				)
			{
				RemoveIndex(PointsIndex[j]);
				FindSamePoint = true;
				//break; ���滹�п�����ȵ�
			}
			else
				j++;
		}
		i++;
	}

	YMinIndex = GetYMinIndex();
	//GetMaxAndMin(CloudPtr, XMax, XMin, YMax, YMin, ZMax, ZMin, MaxMinNo);
}

//�� PointsIndex ��ɾ�� ����Indexֵ
template<typename PointNT>
void CContourAndConvexHull<PointNT>::RemoveIndex(int Index)
{
	for (int i = 0; i < PointsIndex.size(); i++)
	{
		if (PointsIndex[i] == Index)
		{
			PointsIndex.erase(PointsIndex.begin() + i);
			break;
		}
	}
}

template<typename PointNT>	//��ȡ������ �������岻����������ʵ��
void CContourAndConvexHull<PointNT>::GetPointsContour(vector<int> & ContourIndex, bool Is2D = true)
{

}


template<typename PointNT>	//��ȡ͹��
void CContourAndConvexHull<PointNT>::GetPointsConvexHull(vector<int> & HullIndex, bool Is2D = true)
{
	HullIndex.clear();
	HullIndex.push_back(YMinIndex);	//��Y����Сֵ��ʼ����

	if (CloudPtr->points.size() <= 1)	//���ֻ��һ���ڵ�ʱ��������� 2015.10.25
	{
		return;
	}

	vector<int> TempHullIndex;
	double MinAngle = 0;
	double CurrentIndex = YMinIndex;
	bool FindStartPoint = false;

	while (!FindStartPoint)
	{
		TempHullIndex = GetMinAngleAndDis(CurrentIndex, MinAngle);

		if (TempHullIndex.size() > 0)
		{
			for (int i = 0; i < TempHullIndex.size(); i++)
			{
				HullIndex.push_back(TempHullIndex[i]);
				if (TempHullIndex[i] == YMinIndex)
				{
					FindStartPoint = true;
					break;
				}
			}
		}
		else
		{
			return;
		}
		CurrentIndex = HullIndex[HullIndex.size() - 1]; //���һ���ڵ�Ҫ��
	}
	HullIndex.pop_back();//���һ��͹���ڵ����һ���ظ�			
}

//2020.05.20 �����뷨����͹��  ͹���������㷽�����Ѿ���ʵ��
template<typename PointNT>
inline void CContourAndConvexHull<PointNT>::GetConvexHullOneByOne(vector<int>& HullIndex, bool Is2D)
{
	RandomStart(HullIndex, Is2D);

	VectorBase<int> VectorBaseInt;

	for (int i = 0; i < CloudPtr->points.size(); i++)
	{
		bool IsIn = CurrentIsInConvexHull(HullIndex, i);

		if (!IsIn)	//�õ㲻��͹��������У�����Ҫ����͹���������
		{
			//˵����Ҫ�����µ�͹����
			ReplaceConvexIndex(HullIndex, i);
			
			/*// ֱ�Ӽ���֮ǰ͹�����¼�����͹�� 2020.05.31 ���ַ�����㣬�׶�
			CContourAndConvexHull<PointXYZRGBIndex> Self;
			pcl::PointCloud<PointXYZRGBIndex>::Ptr NewPointCloudPtr(new pcl::PointCloud<PointXYZRGBIndex>);

			for (int j = 0; j < HullIndex.size(); j++)
			{
				PointXYZRGBIndex Temp;
				Temp.x = CloudPtr->points[HullIndex[j]].x;
				Temp.y = CloudPtr->points[HullIndex[j]].y;
				Temp.z = CloudPtr->points[HullIndex[j]].z;
				Temp.Index = HullIndex[j];

				NewPointCloudPtr->points.push_back(Temp);
			}
			PointXYZRGBIndex Temp;

			Temp.x = CloudPtr->points[i].x;
			Temp.y = CloudPtr->points[i].y;
			Temp.z = CloudPtr->points[i].z;
			Temp.Index = i;

			NewPointCloudPtr->points.push_back(Temp);

			vector<int> TempHullIndex;
			Self.SetInputs(NewPointCloudPtr);
			Self.GetPointsConvexHull(TempHullIndex);
			
			HullIndex.clear();
			for (int j = 0; j < TempHullIndex.size(); j++)
			{
				HullIndex.push_back(NewPointCloudPtr->points[TempHullIndex[j]].Index);
			}
			//*/
		}
	}
}

//2020.05.20 ���ѡ����������Ϊ��ʼ͹��
template<typename PointNT>
inline void CContourAndConvexHull<PointNT>::RandomStart(vector<int>& HullIndex, bool Is2D)
{
	HullIndex.push_back(0);
	HullIndex.push_back(1);

	for (int i = 2; i < CloudPtr->points.size(); i++)
	{
		double Area = PointBase::AreaOfThreePointsIn2D(
			CloudPtr->points[0].x, CloudPtr->points[0].y,
			CloudPtr->points[1].x, CloudPtr->points[1].y,
			CloudPtr->points[i].x, CloudPtr->points[i].y);

		if (abs(Area) > 0.001)
		{
			HullIndex.push_back(i);
			break;
		}
	}

	if (HullIndex.size() == 2)
		return;

	//2020.05.22 ����������������ʱ���������
	bool AntiClockWise;
	double area = PointBase::AreaOfThreePointsIn2D(
		CloudPtr->points[HullIndex[0]].x, CloudPtr->points[HullIndex[0]].y,
		CloudPtr->points[HullIndex[1]].x, CloudPtr->points[HullIndex[1]].y,
		CloudPtr->points[HullIndex[2]].x, CloudPtr->points[HullIndex[2]].y);

	if (area < 0)
		swap(HullIndex[1], HullIndex[2]);

	RefreshCentroidPoint(HullIndex);
}

//2020.05.21 ����͹������ε�����
template<typename PointNT>
inline void CContourAndConvexHull<PointNT>::RefreshCentroidPoint(const vector<int> HullIndex)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexHullPointsPtr;

	CentroidPoint.x = 0;
	CentroidPoint.y = 0;
	CentroidPoint.z = 0;

	int TempIndex = HullIndex.size() / 2;

	CentroidPoint.x = CloudPtr->points[HullIndex[0]].x + CloudPtr->points[HullIndex[TempIndex]].x;
	CentroidPoint.y = CloudPtr->points[HullIndex[0]].y + CloudPtr->points[HullIndex[TempIndex]].y;
	CentroidPoint.z = CloudPtr->points[HullIndex[0]].z + CloudPtr->points[HullIndex[TempIndex]].z;

	//for (int i = 0; i < HullIndex.size(); i++)
	//{
	//	pcl::PointXYZRGB TempPoint;
	//	CentroidPoint.x += CloudPtr->points[HullIndex[i]].x / HullIndex.size();
	//	CentroidPoint.y += CloudPtr->points[HullIndex[i]].y / HullIndex.size();
	//	CentroidPoint.z += CloudPtr->points[HullIndex[i]].z / HullIndex.size();
	//}
}

//2020.05.21 �жϵ�ǰ�� CurIndex �Ƿ�λ��͹����
//��͹��������з��� True�����򷵻�False
template<typename PointNT>
inline bool CContourAndConvexHull<PointNT>::CurrentIsInConvexHull(const vector<int> HullIndex, int CurIndex)
{
	bool IsIn = false;

	pcl::PointXYZRGB CurPoint;
	CurPoint.x = CloudPtr->points[CurIndex].x;
	CurPoint.y = CloudPtr->points[CurIndex].y;
	CurPoint.z = CloudPtr->points[CurIndex].z;

	for (int i = 0; i < HullIndex.size(); i++)
	{
		pcl::PointXYZRGB FirstPoint;
		FirstPoint.x = CloudPtr->points[HullIndex[i]].x;
		FirstPoint.y = CloudPtr->points[HullIndex[i]].y;
		FirstPoint.z = CloudPtr->points[HullIndex[i]].z;

		pcl::PointXYZRGB SecondPoint;
		SecondPoint.x = CloudPtr->points[HullIndex[(i + 1) % HullIndex.size()]].x;
		SecondPoint.y = CloudPtr->points[HullIndex[(i + 1) % HullIndex.size()]].y;
		SecondPoint.z = CloudPtr->points[HullIndex[(i + 1) % HullIndex.size()]].z;

		int InValue = PointBase::PointIsInTriange2D(FirstPoint, SecondPoint,
			CentroidPoint, CurPoint);

		if (InValue != -1) //ֻ�иõ�λ��һ�������ε��ⲿ������Ϊ��Ҫ����͹��
		{
			IsIn = true;
			break;
		}
	}

	return IsIn;
}

template<typename PointNT>
inline int CContourAndConvexHull<PointNT>::ReplaceConvexIndex(vector<int>& HullIndex, int CurIndex)
{
	int FindIndex = -1;

	pcl::PointXYZRGB LatentPoint;
	LatentPoint.x = CloudPtr->points[CurIndex].x;
	LatentPoint.y = CloudPtr->points[CurIndex].y;
	LatentPoint.z = CloudPtr->points[CurIndex].z;

	int SearchStartIndex = -1;

	//�ҵ�һ�� ��������ʼ�㣬�Ա���������������״̬
	for (int i = 0; i < HullIndex.size(); i++)
	{
		int NextIndex = (i + 1) % (HullIndex.size());

		pcl::PointXYZRGB CurHullPoint;
		CurHullPoint.x = CloudPtr->points[HullIndex[i]].x;
		CurHullPoint.y = CloudPtr->points[HullIndex[i]].y;
		CurHullPoint.z = CloudPtr->points[HullIndex[i]].z;

		pcl::PointXYZRGB NextPoint;
		NextPoint.x = CloudPtr->points[HullIndex[NextIndex]].x;
		NextPoint.y = CloudPtr->points[HullIndex[NextIndex]].y;
		NextPoint.z = CloudPtr->points[HullIndex[NextIndex]].z;

		double TempArea = GeometryBase::AreaOfThreePointsIn2D(CurHullPoint, LatentPoint, NextPoint);

		if (TempArea < 0)
		{
			SearchStartIndex = i;
			break;
		}
	}

	int StartIndex = -1, EndIndex = -1;
	//Ѱ��һ�������ķ�Χ���÷�Χ�ڵ�͹����Ҫ���µ�͹�����滻��
	for (int i = SearchStartIndex; i < (SearchStartIndex + HullIndex.size()); i++)
	{
		pcl::PointXYZRGB CurHullPoint;
		CurHullPoint.x = CloudPtr->points[HullIndex[i % (HullIndex.size())]].x;
		CurHullPoint.y = CloudPtr->points[HullIndex[i % (HullIndex.size())]].y;
		CurHullPoint.z = CloudPtr->points[HullIndex[i % (HullIndex.size())]].z;

		pcl::PointXYZRGB NextPoint;
		NextPoint.x = CloudPtr->points[HullIndex[(i + 1) % (HullIndex.size())]].x;
		NextPoint.y = CloudPtr->points[HullIndex[(i + 1) % (HullIndex.size())]].y;
		NextPoint.z = CloudPtr->points[HullIndex[(i + 1) % (HullIndex.size())]].z;

		double TempArea = GeometryBase::AreaOfThreePointsIn2D(CurHullPoint, LatentPoint, NextPoint);
		if (TempArea > 0 && StartIndex == -1)
			StartIndex = i;
		else if (TempArea > 0 && StartIndex != -1)
			EndIndex = i;
		else if (TempArea < 0 && StartIndex != -1 && EndIndex != -1)
			break;
	}

	//��Ҫ����һ��͹����
	if (EndIndex == -1)
	{
		HullIndex.insert(HullIndex.begin() + (StartIndex % (HullIndex.size())) + 1, CurIndex);
	}
	else
	{
		//��Ҫɾ�����������͹���������һ��͹����
		vector<int> TempHullIndex;

		TempHullIndex.push_back(HullIndex[StartIndex % (HullIndex.size())]);

		for (int i = StartIndex + 1; i < (StartIndex + HullIndex.size()); i++)
		{
			if (i == EndIndex)
				TempHullIndex.push_back(CurIndex);
			else if(i > EndIndex)
				TempHullIndex.push_back(HullIndex[i % HullIndex.size()]);
		}
	
		HullIndex.clear();
		HullIndex.insert(HullIndex.begin(), TempHullIndex.begin(), TempHullIndex.end());
	}

	return FindIndex;
}

//2020.05.31 �жϵ�ǰ͹�����Ƿ���͹���㣬����true���ǣ������� �� ��δʹ��
template<typename PointNT>
inline bool CContourAndConvexHull<PointNT>::IsConvexHull(vector<int> HullIndex, int CurConvexIndex)
{
	if (HullIndex.size() <= 3)
		return true;

	int PriorOne = (CurConvexIndex - 1 + HullIndex.size()) % HullIndex.size();
	int PriorTwo = (CurConvexIndex - 2 + HullIndex.size()) % HullIndex.size();
	int Next = (CurConvexIndex + 1) % HullIndex.size();

	int IsIn = GeometryBase::PointIsInTriange(CloudPtr->points[HullIndex[PriorOne]],
		CloudPtr->points[HullIndex[PriorTwo]],
		CloudPtr->points[HullIndex[Next]],
		CloudPtr->points[HullIndex[CurConvexIndex]]);

	if (IsIn == -1)
		return true;
	else
		return false;
}

//2012.05.24 Ѱ��͹����ĺ��ʲ���λ�ã� ��δʹ��
template<typename PointNT>
inline int CContourAndConvexHull<PointNT>::FindProperPos(vector<int>& HullIndex, int CurIndex)
{
	int FindIndex = -1;
	for (int i = 0; i < HullIndex.size(); i++)
	{
		int PriorOneIndex = (i - 1 + HullIndex.size()) % (HullIndex.size());
		int PriorTwoIndex = (i - 2 + HullIndex.size()) % (HullIndex.size());

		int NextOneIndex = (i) % (HullIndex.size());
		int NextTwoIndex = (i + 1) % (HullIndex.size());

		pcl::PointXYZRGB PriorOnePoint;
		PriorOnePoint.x = CloudPtr->points[HullIndex[PriorOneIndex]].x;
		PriorOnePoint.y = CloudPtr->points[HullIndex[PriorOneIndex]].y;
		PriorOnePoint.z = CloudPtr->points[HullIndex[PriorOneIndex]].z;
		pcl::PointXYZRGB PriorTwoPoint;
		PriorTwoPoint.x = CloudPtr->points[HullIndex[PriorTwoIndex]].x;
		PriorTwoPoint.y = CloudPtr->points[HullIndex[PriorTwoIndex]].y;
		PriorTwoPoint.z = CloudPtr->points[HullIndex[PriorTwoIndex]].z;

		pcl::PointXYZRGB NextOnePoint;
		NextOnePoint.x = CloudPtr->points[HullIndex[NextOneIndex]].x;
		NextOnePoint.y = CloudPtr->points[HullIndex[NextOneIndex]].y;
		NextOnePoint.z = CloudPtr->points[HullIndex[NextOneIndex]].z;
		pcl::PointXYZRGB NextTwoPoint;
		NextTwoPoint.x = CloudPtr->points[HullIndex[NextTwoIndex]].x;
		NextTwoPoint.y = CloudPtr->points[HullIndex[NextTwoIndex]].y;
		NextTwoPoint.z = CloudPtr->points[HullIndex[NextTwoIndex]].z;

		pcl::PointXYZRGB CurPoint;
		CurPoint.x = CloudPtr->points[CurIndex].x;
		CurPoint.y = CloudPtr->points[CurIndex].y;
		CurPoint.z = CloudPtr->points[CurIndex].z;

		double Area0 = PointBase::AreaOfThreePointsIn2D(PriorTwoPoint.x, PriorTwoPoint.y,
			PriorOnePoint.x, PriorOnePoint.y, CurPoint.x, CurPoint.y);
		double Area1 = PointBase::AreaOfThreePointsIn2D(PriorOnePoint.x, PriorOnePoint.y,
			CurPoint.x, CurPoint.y, NextOnePoint.x, NextOnePoint.y);
		double Area2 = PointBase::AreaOfThreePointsIn2D(CurPoint.x, CurPoint.y,
			NextOnePoint.x, NextOnePoint.y, NextTwoPoint.x, NextTwoPoint.y);

		if (Area0 > 0 && Area1 > 0 && Area2 > 0)
		{
			FindIndex = i;
		}
	}

	return FindIndex;
}

//�ඨ�����


//template<typename PointNT>
//void CCContourAndConvexHull<PointNT>::GetMaxAndMin(PointCloudPtr CloudValue,
//		double & XMaxValue, double & XMinValue, 
//		double & YMaxValue, double & YMinValue, 
//		double & ZMaxValue, double & ZMinValue, vector<int> & MaxMinVector)			
//		//��ȡ�ֶε����ֵ����Сֵ ���ն�Ӧ����ֱ���뵽MaxMinVector
//{
//	if (CloudValue->points.size() == 0)  return;
//
//	XMaxValue = CloudValue->points[0].x;	//�����ֵ�����ֶ����͵���Сֵ
//	YMaxValue = CloudValue->points[0].y;
//	ZMaxValue = CloudValue->points[0].z;
//
//	XMinValue = CloudValue->points[0].x;	//����Сֵ�����ֶ����͵����ֵ
//	YMinValue = CloudValue->points[0].y;
//	ZMinValue = CloudValue->points[0].z;
//
//	if (MaxMinVector.size() != 6)
//	{
//		MaxMinVector.clear();
//		for(int i = 0 ; i < 6; i++)
//		{
//			MaxMinVector.push_back(0);
//		}
//	}
//
//	for(int i = 0 ; i < CloudValue->points.size(); i++)
//	{
//		if (CloudValue->points[i].x > XMaxValue) 
//		{
//			XMaxValue = CloudValue->points[i].x;
//			MaxMinVector[0] = i;
//		}
//		if (CloudValue->points[i].y > YMaxValue)
//		{
//			YMaxValue = CloudValue->points[i].y;
//			MaxMinVector[2] = i;
//		}
//		if (CloudValue->points[i].z > ZMaxValue)
//		{
//			ZMaxValue = CloudValue->points[i].z;
//			MaxMinVector[4] = i;
//		}
//
//		if (CloudValue->points[i].x < XMinValue) 
//		{
//			XMinValue = CloudValue->points[i].x;
//			MaxMinVector[1] = i;
//		}
//		if (CloudValue->points[i].y < YMinValue)
//		{
//			YMinValue = CloudValue->points[i].y;
//			MaxMinVector[3] = i;
//		}
//		if (CloudValue->points[i].z < ZMinValue)
//		{
//			ZMinValue = CloudValue->points[i].z;			
//			MaxMinVector[5] = i;
//		}
//	}
//}


#endif