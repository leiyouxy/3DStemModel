#ifndef CalcGeometryCenter_H
#define CalcGeometryCenter_H

//2015.06.30 ���ݸ�����һ�����������ĵ㣬���㾫ȷ�����ĵ�
///2016.01.18 ����ʹ�ô���
#include "CommClass.h"
//#include "AnglePartition.h"

using namespace std;

template<typename PointNT>

class CCalcGeometryCenter
{
private:
	typedef typename pcl::PointCloud<PointNT>::Ptr PointCloudPtr;
	PointCloudPtr CloudPtr;	

	//vector<AnglePartitionStruct> SectionAngleS; //���ƵĽǶȷ�������

	int EachAnglePointNumber;	//�������ĵ㣬ÿ���Ƕȷ�Χʹ�õĵ����

	//���Żس���  ��������ŵ�SampleCloud������
	void NoSameSample(vector<int> CloudIndexs, PointCloudPtr SampleCloud);

	//�Żس��� ��������ŵ�SampleCloud������
	void SameSample(vector<int> CloudIndexs, PointCloudPtr SampleCloud);

	//���㼸������
	bool CalcGeometryCenterPoint(PointCloudPtr CalcCloudValue, PointNT & CenterPoint, 
		double & AvgRadius, double & RadiusVariances,
		double Step = 0.1);

	bool ExploreSectionCenter(PointCloudPtr CalcCloudValue,
		PointNT & TempCenterPoint, 
		double & AvgRadius, double Step, double & LastVariance, 
		int & MaxExecutions, string Axis = "");
public:
	CCalcGeometryCenter()
	{
		
	}
	//������Ҫ�������ĵ�ĵ���ָ���Լ�ÿ���Ƕȷ���ʹ�õĵ�ĸ���
	void SetInputs(PointCloudPtr CloudValue, int EachAnglePointNumberValue);

	//�������XYƽ��ļ������ĵ�  �������false ˵��û����ȷ���ҵ��������ģ���Ҫ����
	bool CalcGeometryCenter(PointNT & CenterPoint, double AngleValue, 
		double & AvgRadius, double & RadiusVariances);

	//�������XYƽ��ļ������ĵ�(ʹ�����е����ݵ�)  �������false ˵��û����ȷ���ҵ��������ģ���Ҫ����
	//// ������������ ���ô˹��̣���ʼ���ı��븳ֵ  �����֪�� ���ĵ� Ϊ���ѡ�� 2015.10.16
	bool CalcGeometryCenterByAllPoints(PointNT & CenterPoint, double & AvgRadius, 
		double & RadiusVariances, bool IsHaveReferencedPoint = false);

	//��ȡ������Ƽ������ĵ� 2015.10.16
	PointNT GetGravityPointByAllPoints();

	////�������XYƽ��ļ������ĵ�  �������false ˵��û����ȷ���ҵ��������ģ���Ҫ����	
	//bool CalcGeometryCenter(PointNT & CenterPoint, double AngleValue, 
	//	double & AvgRadius, double & RadiusVariances, vector<AnglePartitionStruct> & SectionAngleS);

	void FittingCircleByLeastSquares(PointCloudPtr Cloud,	
	double & CenterX, double & CenterY, double & AvgRadius);

};


//���Żس��� 
template<typename PointNT>
void CCalcGeometryCenter<PointNT>::NoSameSample(vector<int> CloudIndexs, 
	PointCloudPtr SampleCloud)
{	
	SampleCloud->points.clear();
	while (SampleCloud->points.size() < EachAnglePointNumber)
	{
		int RandomIndex = PointBase::GetRandom(0, CloudIndexs.size() - 1);
		SampleCloud->points.push_back(CloudPtr->points[CloudIndexs[RandomIndex]]);
		CloudIndexs.erase(CloudIndexs.begin() + RandomIndex);
		/*bool Find = false;
		for(int i = 0; i < CloudIndexs.size(); i++)
		{
			if (CloudIndexs[i] == RandomIndex)
			{
				Find = true;
				break;
			}
		}
		if (Find == false)
		{*/
			
		//}
	}
}


//�Żس���   
template<typename PointNT>
void CCalcGeometryCenter<PointNT>::SameSample(vector<int> CloudIndexs, 
	PointCloudPtr SampleCloud)
{
	SampleCloud->points.clear();
	while (SampleCloud->points.size() < EachAnglePointNumber)
	{
		int RandomIndex = PointBase::GetRandom(0, CloudIndexs.size() - 1);
		SampleCloud->points.push_back(CloudPtr->points[CloudIndexs[RandomIndex]]);
		////if (SampleCloud->points.size() != 1)
		//	CloudIndexs.push_back(PointBase::GetRandom(0, SampleCloud->points.size()- 1));		
		//else
		//	CloudIndexs.push_back(0);
	}

}

	//���㼸�����ĵ�  CenterPoint �Ǵ��봫������
template<typename PointNT>
bool CCalcGeometryCenter<PointNT>::CalcGeometryCenter(PointNT & CenterPoint, double AngleValue, 
		double & AvgRadius, double & RadiusVariances)
{
	PointCloudPtr CalcCloud (new pcl::PointCloud<PointNT>);
		
	vector<AnglePartitionStruct> SectionAngleS; //���ƵĽǶȷ�������

	////���ݽǶȷ���������Ƶ�����λ��
	//CAnglePartition AnglePartitionInstance;  2018.12.15 ע���� �����÷�������ʽ��
	//AnglePartitionInstance.PartitionPoints(CloudPtr, CenterPoint, AngleValue, SectionAngleS);

	for (int i = 0; i < SectionAngleS.size(); i++)
	{
		PointCloudPtr EachAngleCloudPtr (new pcl::PointCloud<PointNT>);

		if (SectionAngleS[i].Points.size() > EachAnglePointNumber)
		{
			NoSameSample(SectionAngleS[i].Points, EachAngleCloudPtr); 
		}
		else if (SectionAngleS[i].Points.size() > 0)
		{
			SameSample(SectionAngleS[i].Points, EachAngleCloudPtr);
		}
		CalcCloud->points.insert(CalcCloud->points.end(),
			EachAngleCloudPtr->points.begin(), EachAngleCloudPtr->points.end());
	}

	return CalcGeometryCenterPoint(CloudPtr, CenterPoint, AvgRadius, RadiusVariances);
	//return CalcGeometryCenterPoint(CalcCloud, CenterPoint, AvgRadius, RadiusVariances); 2018.12.15 ע���� �����÷�������ʽ��
}

//�������XYƽ��ļ������ĵ�(ʹ�����е����ݵ�)  
//�������false ˵��û����ȷ���ҵ��������ģ���Ҫ���� 
// ������������ ���ô˹��̣���ʼ���ı��븳ֵ  �����֪�� ���ĵ� Ϊ���ѡ��  2015.10.16
template<typename PointNT>
bool CCalcGeometryCenter<PointNT>::CalcGeometryCenterByAllPoints(PointNT & CenterPoint,
	double & AvgRadius, double & RadiusVariances, bool IsHaveReferencedPoint)//����Ƿ��Բ�ηֲ�, 2015.12.21, ������3/4 3/4�ĽǶȷ����е㼴��Բ�ηֲ�
{
	bool IsFind = true;
	//����С�������Բ��λ�� 
	if (!IsHaveReferencedPoint)
	{
		double x = CenterPoint.x;
		double y = CenterPoint.y;
		FittingCircleByLeastSquares(CloudPtr, x, y, AvgRadius);
		CenterPoint.x = x;
		CenterPoint.y = y;
		CenterPoint.z = CloudPtr->points[0].z;
	}
	
	PointNT CopyPoint = CenterPoint;

	//CenterPoint = GetGravityPointByAllPoints();
	IsFind = CalcGeometryCenterPoint(CloudPtr, CenterPoint, AvgRadius, RadiusVariances);	
	if (!IsFind)	//���û���ҵ� ������С���˷���ϵĽ��
	{
		CenterPoint = CopyPoint;	
	}

	return IsFind;
}

	//��ȡ������Ƽ������ĵ� 2015.10.16
template<typename PointNT>
PointNT CCalcGeometryCenter<PointNT>::GetGravityPointByAllPoints()
{
	PointNT GravityPoint;
	GravityPoint.x = 0;
	GravityPoint.y = 0;
	GravityPoint.z = 0;

	for(int i = 0; i < CloudPtr->points.size(); i++)
	{
		GravityPoint.x = GravityPoint.x + CloudPtr->points[i].x;
		GravityPoint.y = GravityPoint.y + CloudPtr->points[i].y;
		GravityPoint.z = GravityPoint.z + CloudPtr->points[i].z;
	}
	if (CloudPtr->points.size() > 0)
	{
		GravityPoint.x = GravityPoint.x / CloudPtr->points.size();
		GravityPoint.y = GravityPoint.y / CloudPtr->points.size();
		GravityPoint.z = GravityPoint.z / CloudPtr->points.size();
	}
	return GravityPoint;
}


//���㼸������
template<typename PointNT>
bool CCalcGeometryCenter<PointNT>::CalcGeometryCenterPoint(PointCloudPtr CalcCloudValue, 
	PointNT & CenterPoint, double & AvgRadius, double & RadiusVariances,
	double Step)
{
	AvgRadius = 0;
	RadiusVariances = 0;
	int MaxExecutions = 0;

	//double TempX = CenterPoint.x;
	//double TempY = CenterPoint.y;

	//if (!ExploreSectionCenter(CalcCloudValue, 
	//	CenterPoint, AvgRadius, Step, LastVariance, MaxExecutions))	
	//{	//û����ȷ���ؽ��
	//	CenterPoint.x = TempX;
	//	CenterPoint.y = TempY;
	//}
	return ExploreSectionCenter(CalcCloudValue, 
		CenterPoint, AvgRadius, Step, RadiusVariances, MaxExecutions);
}

template<typename PointNT>
void CCalcGeometryCenter<PointNT>::SetInputs(PointCloudPtr CloudValue, int EachAnglePointNumberValue)
{
	CloudPtr = CloudValue;
	EachAnglePointNumber = EachAnglePointNumberValue;
}


//2014.09.22 ��xyƽ��̽����ǰ����������λ�ã�Step Ϊ������ѡ��ĳһ��������һֱǰ��
//Ӧ�������ּ��㷽ʽ����ʱ������2018.12.15
template<typename PointNT>
bool CCalcGeometryCenter<PointNT>::	ExploreSectionCenter(PointCloudPtr CalcCloudValue, PointNT & TempCenterPoint, 
		double & AvgRadius, double Step, double & LastVariance, int & MaxExecutions, string Axis)
{
	//�ȴ�X�����Ὺʼ 
	CalcBase<double> CalcBasefloat;
	vector<double> vectorfloat;
	double MeanValue;	
	
	double CenterX = TempCenterPoint.x;
	double CenterY = TempCenterPoint.y;

	//if (LastVariance > 10)	//˵������̫̫����������
	//{
	//	Step = LastVariance / 5;	
	//}
	//else if (LastVariance > 5)	//˵������̫̫����������
	//{
	//	Step = 1;	
	//}
	//else if (LastVariance > 1)	//˵������̫̫����������
	//{
	//	Step = 0.5;	
	//}
	//else if (LastVariance < 1 && Step > 0.1)
	//{
	//	Step = 0.1;
	//}

	if (MaxExecutions == 300)	//�ݹ���������ƣ�������Կ���
	{
		return false; //�˴�����Ҫ��Ҫ��������λ��
	}	
	MaxExecutions++;

	//̽���ķ�ʽ��ȡ����������λ�ã�
	if ("" == Axis)	//��һ��ִ�� ������x���y����Ǹ������ƶ�
	{
		vector<double> vectorfloatXPlus;
		vector<double> vectorfloatXSub;
		vector<double> vectorfloatYPlus;
		vector<double> vectorfloatYSub;

		double MeanValueXPlus;
		double MeanValueXSub;
		double MeanValueYPlus;
		double MeanValueYSub;

		double VarianceValueXPlus;
		double VarianceValueXSub;
		double VarianceValueYPlus;
		double VarianceValueYSub;

		double VarianceValue = 0;
		vectorfloat.clear();
		for (int i = 0; i < CalcCloudValue->points.size(); i++)
		{
			vectorfloat.push_back(PointDis(CenterX, CenterY,
				CalcCloudValue->points[i].x,
				CalcCloudValue->points[i].y));

			vectorfloatXPlus.push_back(PointDis(CenterX + Step, CenterY,
				CalcCloudValue->points[i].x,
				CalcCloudValue->points[i].y));

			vectorfloatXSub.push_back(PointDis(CenterX - Step, CenterY,
				CalcCloudValue->points[i].x,
				CalcCloudValue->points[i].y));

			vectorfloatYPlus.push_back(PointDis(CenterX , CenterY + Step,
				CalcCloudValue->points[i].x,
				CalcCloudValue->points[i].y));

			vectorfloatYSub.push_back(PointDis(CenterX , CenterY - Step,
				CalcCloudValue->points[i].x,
				CalcCloudValue->points[i].y));
		}
		VarianceValue = CalcBasefloat.CalcVariances(vectorfloat, MeanValue);
		AvgRadius = MeanValue;
		LastVariance = VarianceValue;

		VarianceValueXPlus = CalcBasefloat.CalcVariances(vectorfloatXPlus, MeanValueXPlus);
		VarianceValueXSub = CalcBasefloat.CalcVariances(vectorfloatXSub, MeanValueXSub);
		
		VarianceValueYPlus = CalcBasefloat.CalcVariances(vectorfloatYPlus, MeanValueYPlus);
		VarianceValueYSub = CalcBasefloat.CalcVariances(vectorfloatYSub, MeanValueYSub);
		
		if (VarianceValue > VarianceValueXPlus)			//��X�������ƶ�		
		{
			TempCenterPoint.x = CenterX + Step;
			AvgRadius = MeanValueXPlus;			
			LastVariance = VarianceValueXPlus;

			ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueXPlus,
					Step, LastVariance, MaxExecutions, "X+"); //X����� �ݹ� ��ǰ�ƶ� 
			
			if (VarianceValue > VarianceValueYPlus)		//��Y�������ƶ�
			{
				TempCenterPoint.y = CenterY + Step;
				AvgRadius = MeanValueYPlus;				
				LastVariance = VarianceValueYPlus;

				return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueYPlus,
					Step, LastVariance, MaxExecutions, "Y+");	
			}
			else if (VarianceValue > VarianceValueYSub) //��Y�Ḻ���ƶ�
			{
				TempCenterPoint.y = CenterY - Step;
				AvgRadius = MeanValueYSub;	
				LastVariance = VarianceValueYSub;

				return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueYSub,
					Step, LastVariance, MaxExecutions, "Y-");			
			}
		}
		else if (VarianceValue > VarianceValueXSub)		//��X�Ḻ���ƶ�		
		{
			TempCenterPoint.x = CenterX - Step;
			AvgRadius = MeanValueXSub;			
			LastVariance = VarianceValueXSub;

			ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueXSub, 
				Step, LastVariance, MaxExecutions, "X-");		
			
			if (VarianceValue > VarianceValueYPlus)		//��Y�������ƶ�
			{
				TempCenterPoint.y = CenterY + Step;
				AvgRadius = MeanValueYPlus;	
				LastVariance = VarianceValueYPlus;
				
				return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueYPlus,
					Step, LastVariance, MaxExecutions, "Y+");	
			}
			else if (VarianceValue > VarianceValueYSub) //��Y�Ḻ���ƶ�
			{
				TempCenterPoint.y = CenterY - Step;
				AvgRadius = MeanValueYSub;	
				LastVariance = VarianceValueYSub;

				return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueYSub,
					Step, LastVariance, MaxExecutions, "Y-");			
			}
		}	
	}
	else if ("X+" == Axis)
	{
		double VarianceValue = 0;		

		vectorfloat.clear();
		for (int i = 0; i < CalcCloudValue->points.size(); i++)
		{
			vectorfloat.push_back(PointDis(CenterX + Step, CenterY,
				CalcCloudValue->points[i].x,
				CalcCloudValue->points[i].y));
		}
		VarianceValue = CalcBasefloat.CalcVariances(vectorfloat, MeanValue);
		if (VarianceValue >= LastVariance)	//���ĵ� �Ѿ� ���� x�������λ��
		{
			return true;
		}
		else	//���ĵ� �Ѿ� ���� x�������λ��
		{
			TempCenterPoint.x = CenterX + Step;
			AvgRadius = MeanValue;		
			LastVariance = VarianceValue;

			return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValue,
				Step, LastVariance, MaxExecutions, "X+");
		}
	}
	else if ("X-" == Axis)
	{
		double VarianceValue = 0;

		vectorfloat.clear();
		for (int i = 0; i < CalcCloudValue->points.size(); i++)
		{
			vectorfloat.push_back(PointDis(CenterX - Step, CenterY,
				CalcCloudValue->points[i].x,
				CalcCloudValue->points[i].y));
		}
		VarianceValue = CalcBasefloat.CalcVariances(vectorfloat, MeanValue);
		if (VarianceValue >= LastVariance)	//���ĵ� �Ѿ� ���� x�������λ��
		{
			return true;
		}
		else	//���ĵ� �Ѿ� ���� x�������λ��
		{
			TempCenterPoint.x = CenterX - Step;
			AvgRadius = MeanValue;			
			LastVariance = VarianceValue;

			return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValue,
				Step, LastVariance, MaxExecutions, "X-");
		}
	}	 
	//������Y������
	else if ("Y+" == Axis)
	{
		double VarianceValue = 0;
		vectorfloat.clear();
		for (int i = 0; i < CalcCloudValue->points.size(); i++)
		{
			vectorfloat.push_back(PointDis(CenterX , CenterY + Step,
				CalcCloudValue->points[i].x,
				CalcCloudValue->points[i].y));
		}
		VarianceValue = CalcBasefloat.CalcVariances(vectorfloat, MeanValue);
		if (VarianceValue >= LastVariance)	//���ĵ� �Ѿ� ���� x�������λ��
		{
			return true;
		}
		else	//���ĵ� �Ѿ� ���� x�������λ��
		{
			TempCenterPoint.y = CenterY + Step;
			AvgRadius = MeanValue;	
			LastVariance = VarianceValue;

			return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValue,
				Step, LastVariance, MaxExecutions, "Y+");
		}
	}
	else if ("Y-" == Axis)
	{
		double VarianceValue = 0;
		vectorfloat.clear();
		for (int i = 0; i < CalcCloudValue->points.size(); i++)
		{
			vectorfloat.push_back(PointDis(CenterX, CenterY - Step,
				CalcCloudValue->points[i].x,
				CalcCloudValue->points[i].y));
		}
		VarianceValue = CalcBasefloat.CalcVariances(vectorfloat, MeanValue);
		
		if (VarianceValue >= LastVariance)	//���ĵ� �Ѿ� ���� x�������λ��
		{			
			return true;
		}
		else	//���ĵ� �Ѿ� ���� x�������λ��
		{
			TempCenterPoint.y = CenterY - Step;
			AvgRadius = MeanValue;	
			LastVariance = VarianceValue;

			return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValue,
				Step, VarianceValue, MaxExecutions, "Y-");
		}
	}	
	
}

template<typename PointNT>
void CCalcGeometryCenter<PointNT>::FittingCircleByLeastSquares(
	PointCloudPtr Cloud,	
	double & CenterX, double & CenterY, double & AvgRadius)
{
	if (Cloud->points.size() < 3)
	{
		return;
	}

	double X1=0;
	double Y1=0;
	double X2=0;
	double Y2=0;
	double X3=0;
	double Y3=0;
	double X1Y1=0;
	double X1Y2=0;
	double X2Y1=0;

	for (int i = 0; i < Cloud->points.size(); i++)
	{
		X1 = X1 + Cloud->points[i].x;
		Y1 = Y1 + Cloud->points[i].y;
		X2 = X2 + Cloud->points[i].x * Cloud->points[i].x;
		Y2 = Y2 + Cloud->points[i].y * Cloud->points[i].y;
		X3 = X3 + Cloud->points[i].x * Cloud->points[i].x * Cloud->points[i].x;
		Y3 = Y3 + Cloud->points[i].y * Cloud->points[i].y * Cloud->points[i].y;
		X1Y1 = X1Y1 + Cloud->points[i].x * Cloud->points[i].y;
		X1Y2 = X1Y2 + Cloud->points[i].x * Cloud->points[i].y * Cloud->points[i].y;
		X2Y1 = X2Y1 + Cloud->points[i].x * Cloud->points[i].x * Cloud->points[i].y;
	}

	double C,D,E,G,H,N;
	double a,b,c;
	N = Cloud->points.size();
	C = N * X2 - X1 * X1;
	D = N * X1Y1 - X1 * Y1;
	E = N*X3 + N *X1Y2 - (X2+Y2)*X1;
	G = N*Y2 - Y1*Y1;
	H = N*X2Y1 + N*Y3 - (X2+Y2)*Y1;
	a = (H*D-E*G)/(C*G-D*D);
	b = (H*C-E*D)/(D*D-G*C);
	c = -(a*X1 + b*Y1 + X2 + Y2)/N;
		
	CenterX = a/(-2);
	CenterY = b/(-2);
	AvgRadius = sqrt(a*a+b*b-4*c)/2;	 //���Ǳ�׼��С���˷�����İ뾶	
}

#endif

////��ȡ����
	///*FileName = "ProcessedPoints\\A2015111507_XYConvexhullPoints_34.6.pcd";
	//PointBase::OpenPCLFile(FileName, StemPointsPtr);
	//PointBase::PointMoveToOrigin(StemPointsPtr);

	//CContourAndConvexHull<pcl::PointXYZRGB> ContourAndConvexHull;
	//PointBase::SetPointColor(StemPointsPtr, ColorBase::RedColor);
	//ContourAndConvexHull.SetInputs(StemPointsPtr);

	//vector<int> HullIndexs;
	//ContourAndConvexHull.GetPointsConvexHull(HullIndexs);

	//for(int i = 0; i < HullIndexs.size(); i++)
	//{
	//	GeometryCentersPtr->points.push_back(StemPointsPtr->points[HullIndexs[i]]);
	//}

	//pcl::PointXYZRGB Center = GeometryBase::GetConvexHullCentroid(GeometryCentersPtr);
	//Center.z = StemPointsPtr->points[0].z;
	//Center.rgba = ColorBase::YellowColor;	
	//

	//CCalcGeometryCenter<pcl::PointXYZRGB> CalcGeometryCenter;
	//CalcGeometryCenter.SetInputs(StemPointsPtr, 5);

	//pcl::PointXYZRGB CenterPoint1;
	//double R,V;
	//CalcGeometryCenter.CalcGeometryCenterByAllPoints(CenterPoint1, R, V);

	//CenterPoint1.z = StemPointsPtr->points[0].z;
	//CenterPoint1.rgba = ColorBase::BlueColor;	

	//StemPointsPtr->points.push_back(Center);
	//StemPointsPtr->points.push_back(CenterPoint1);
	//PointBase::ShowPointXYZRGB(Viewer, StemPointsPtr, "StemPointsPtr", 5);*/