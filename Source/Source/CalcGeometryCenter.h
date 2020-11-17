#ifndef CalcGeometryCenter_H
#define CalcGeometryCenter_H

//2015.06.30 根据给定的一个待估的中心点，计算精确的中心点
///2016.01.18 不在使用此类
#include "CommClass.h"
//#include "AnglePartition.h"

using namespace std;

template<typename PointNT>

class CCalcGeometryCenter
{
private:
	typedef typename pcl::PointCloud<PointNT>::Ptr PointCloudPtr;
	PointCloudPtr CloudPtr;	

	//vector<AnglePartitionStruct> SectionAngleS; //点云的角度分区容器

	int EachAnglePointNumber;	//计算中心点，每个角度范围使用的点个数

	//不放回抽样  抽样结果放到SampleCloud点云中
	void NoSameSample(vector<int> CloudIndexs, PointCloudPtr SampleCloud);

	//放回抽样 抽样结果放到SampleCloud点云中
	void SameSample(vector<int> CloudIndexs, PointCloudPtr SampleCloud);

	//计算几何中心
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
	//设置需要计算中心点的点云指针以及每个角度分区使用的点的个数
	void SetInputs(PointCloudPtr CloudValue, int EachAnglePointNumberValue);

	//计算点云XY平面的几何中心点  如果返回false 说明没有正确地找到几何中心，需要调整
	bool CalcGeometryCenter(PointNT & CenterPoint, double AngleValue, 
		double & AvgRadius, double & RadiusVariances);

	//计算点云XY平面的几何中心点(使用所有的数据点)  如果返回false 说明没有正确地找到几何中心，需要调整
	//// ×××××× 调用此过程，初始中心必须赋值  如果不知道 重心点 为最佳选择 2015.10.16
	bool CalcGeometryCenterByAllPoints(PointNT & CenterPoint, double & AvgRadius, 
		double & RadiusVariances, bool IsHaveReferencedPoint = false);

	//获取输入点云集的重心点 2015.10.16
	PointNT GetGravityPointByAllPoints();

	////计算点云XY平面的几何中心点  如果返回false 说明没有正确地找到几何中心，需要调整	
	//bool CalcGeometryCenter(PointNT & CenterPoint, double AngleValue, 
	//	double & AvgRadius, double & RadiusVariances, vector<AnglePartitionStruct> & SectionAngleS);

	void FittingCircleByLeastSquares(PointCloudPtr Cloud,	
	double & CenterX, double & CenterY, double & AvgRadius);

};


//不放回抽样 
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


//放回抽样   
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

	//计算几何中心点  CenterPoint 是传入传出变量
template<typename PointNT>
bool CCalcGeometryCenter<PointNT>::CalcGeometryCenter(PointNT & CenterPoint, double AngleValue, 
		double & AvgRadius, double & RadiusVariances)
{
	PointCloudPtr CalcCloud (new pcl::PointCloud<PointNT>);
		
	vector<AnglePartitionStruct> SectionAngleS; //点云的角度分区容器

	////根据角度分区计算点云的中心位置
	//CAnglePartition AnglePartitionInstance;  2018.12.15 注销掉 不采用分区的形式做
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
	//return CalcGeometryCenterPoint(CalcCloud, CenterPoint, AvgRadius, RadiusVariances); 2018.12.15 注销掉 不采用分区的形式做
}

//计算点云XY平面的几何中心点(使用所有的数据点)  
//如果返回false 说明没有正确地找到几何中心，需要调整 
// ×××××× 调用此过程，初始中心必须赋值  如果不知道 重心点 为最佳选择  2015.10.16
template<typename PointNT>
bool CCalcGeometryCenter<PointNT>::CalcGeometryCenterByAllPoints(PointNT & CenterPoint,
	double & AvgRadius, double & RadiusVariances, bool IsHaveReferencedPoint)//检查是否呈圆形分布, 2015.12.21, 比率是3/4 3/4的角度分区有点即是圆形分布
{
	bool IsFind = true;
	//先最小二乘拟合圆心位置 
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
	if (!IsFind)	//如果没有找到 就用最小二乘法拟合的结果
	{
		CenterPoint = CopyPoint;	
	}

	return IsFind;
}

	//获取输入点云集的重心点 2015.10.16
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


//计算几何中心
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
	//{	//没有正确返回结果
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


//2014.09.22 在xy平面探索当前分区的中心位置，Step 为步长，选定某一个坐标轴一直前进
//应抛弃这种计算方式，耗时费力！2018.12.15
template<typename PointNT>
bool CCalcGeometryCenter<PointNT>::	ExploreSectionCenter(PointCloudPtr CalcCloudValue, PointNT & TempCenterPoint, 
		double & AvgRadius, double Step, double & LastVariance, int & MaxExecutions, string Axis)
{
	//先从X坐标轴开始 
	CalcBase<double> CalcBasefloat;
	vector<double> vectorfloat;
	double MeanValue;	
	
	double CenterX = TempCenterPoint.x;
	double CenterY = TempCenterPoint.y;

	//if (LastVariance > 10)	//说明方差太太，调整步长
	//{
	//	Step = LastVariance / 5;	
	//}
	//else if (LastVariance > 5)	//说明方差太太，调整步长
	//{
	//	Step = 1;	
	//}
	//else if (LastVariance > 1)	//说明方差太太，调整步长
	//{
	//	Step = 0.5;	
	//}
	//else if (LastVariance < 1 && Step > 0.1)
	//{
	//	Step = 0.1;
	//}

	if (MaxExecutions == 300)	//递归深度有限制，必须加以控制
	{
		return false; //此处返回要，要处理坐标位置
	}	
	MaxExecutions++;

	//探索的方式获取分区的中心位置，
	if ("" == Axis)	//第一次执行 决定向x轴和y轴的那个方向移动
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
		
		if (VarianceValue > VarianceValueXPlus)			//向X轴正向移动		
		{
			TempCenterPoint.x = CenterX + Step;
			AvgRadius = MeanValueXPlus;			
			LastVariance = VarianceValueXPlus;

			ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueXPlus,
					Step, LastVariance, MaxExecutions, "X+"); //X轴继续 递归 向前移动 
			
			if (VarianceValue > VarianceValueYPlus)		//向Y轴正向移动
			{
				TempCenterPoint.y = CenterY + Step;
				AvgRadius = MeanValueYPlus;				
				LastVariance = VarianceValueYPlus;

				return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueYPlus,
					Step, LastVariance, MaxExecutions, "Y+");	
			}
			else if (VarianceValue > VarianceValueYSub) //向Y轴负向移动
			{
				TempCenterPoint.y = CenterY - Step;
				AvgRadius = MeanValueYSub;	
				LastVariance = VarianceValueYSub;

				return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueYSub,
					Step, LastVariance, MaxExecutions, "Y-");			
			}
		}
		else if (VarianceValue > VarianceValueXSub)		//向X轴负向移动		
		{
			TempCenterPoint.x = CenterX - Step;
			AvgRadius = MeanValueXSub;			
			LastVariance = VarianceValueXSub;

			ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueXSub, 
				Step, LastVariance, MaxExecutions, "X-");		
			
			if (VarianceValue > VarianceValueYPlus)		//向Y轴正向移动
			{
				TempCenterPoint.y = CenterY + Step;
				AvgRadius = MeanValueYPlus;	
				LastVariance = VarianceValueYPlus;
				
				return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValueYPlus,
					Step, LastVariance, MaxExecutions, "Y+");	
			}
			else if (VarianceValue > VarianceValueYSub) //向Y轴负向移动
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
		if (VarianceValue >= LastVariance)	//中心点 已经 到了 x轴的中心位置
		{
			return true;
		}
		else	//中心点 已经 到了 x轴的中心位置
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
		if (VarianceValue >= LastVariance)	//中心点 已经 到了 x轴的中心位置
		{
			return true;
		}
		else	//中心点 已经 到了 x轴的中心位置
		{
			TempCenterPoint.x = CenterX - Step;
			AvgRadius = MeanValue;			
			LastVariance = VarianceValue;

			return ExploreSectionCenter(CalcCloudValue, TempCenterPoint, MeanValue,
				Step, LastVariance, MaxExecutions, "X-");
		}
	}	 
	//下面是Y轴的情况
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
		if (VarianceValue >= LastVariance)	//中心点 已经 到了 x轴的中心位置
		{
			return true;
		}
		else	//中心点 已经 到了 x轴的中心位置
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
		
		if (VarianceValue >= LastVariance)	//中心点 已经 到了 x轴的中心位置
		{			
			return true;
		}
		else	//中心点 已经 到了 x轴的中心位置
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
	AvgRadius = sqrt(a*a+b*b-4*c)/2;	 //这是标准最小二乘法定义的半径	
}

#endif

////获取重心
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