#include "AnglePartition.h"

void CAnglePartition::SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemCloudValue,
	SectionVector & SectionsVectorValue,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CenterPointsValue,
	double AngleValue) // AngleValue 默认15度
{
	InputCloud = StemCloudValue;
	SectionsVector = SectionsVectorValue;
	CenterPoints = CenterPointsValue;
	Angle = AngleValue;

	PartitionsCount = ceil(360.0 / AngleValue);

	if (SectionsVector.size() != CenterPoints->points.size())
	{
		cout<<"分区个数与中心点个数不等，请检查数据是否正确！"<<endl;
		return;
	}

	//角度分区容器初始化
	SectionAnglePartitionS.clear();
	for(int i = 0; i < SectionsVector.size(); i++)
	{
		SectionAnglePartition TempSectionAnglePartition;		
		vector<AnglePartitionStruct> SectionTempAngle;
		
		TempSectionAnglePartition.AnglePartition = SectionTempAngle;	
		TempSectionAnglePartition.SectionIndex = i;		
		
		SectionAnglePartitionS.push_back(TempSectionAnglePartition);	
	}
}

//对 SectionIndex 这个分区进行角度划分
void CAnglePartition::PartitionSection(int SectionIndex, bool UseTangentPlane)
{
	//先对这个分区的角度分区初始化操作，然后添加数据点
	SectionPartitionInitiation(SectionAnglePartitionS[SectionIndex].AnglePartition);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int j = 0; j < SectionsVector[SectionIndex].Indexs.size(); j++)
	{
		int CurPointIndex = SectionsVector[SectionIndex].Indexs[j];
		double AngleValue = GeometryBase::AngleOfTwoPointsInXY(CenterPoints->points[SectionIndex].x,
				CenterPoints->points[SectionIndex].y,
				InputCloud->points[CurPointIndex].x,
				InputCloud->points[CurPointIndex].y);
	
		int AngleIndex = AngleValue / Angle;
		SectionAnglePartitionS[SectionIndex].AnglePartition[AngleIndex].PointIndexs.push_back(CurPointIndex);
	}	
	
	//Calc the CenterPointofCurPartition for the section SectionIndex 2019.01.01
	//if uses the TangentPlane for the anlge section, the CenterPointofCurPartition point is modified by the centroid of the anlge points
	for (int i = 0; i < SectionAnglePartitionS[SectionIndex].AnglePartition.size(); i++)
	{
		int CurAnglePointCount = SectionAnglePartitionS[SectionIndex].AnglePartition[i].PointIndexs.size();

		SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.x = 0;
		SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.y = 0;
		SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.z = 0;
		SectionAnglePartitionS[SectionIndex].AnglePartition[i].Refined = false;

		for (int j = 0; j < CurAnglePointCount; j++)
		{
			int PointIndex = SectionAnglePartitionS[SectionIndex].AnglePartition[i].PointIndexs[j];
						
			SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.x =
				SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.x 
				+ InputCloud->points[PointIndex].x / CurAnglePointCount;
			
			SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.y =
				SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.y 
				+ InputCloud->points[PointIndex].y / CurAnglePointCount;
			
			SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.z =
				SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.z 
				+ InputCloud->points[PointIndex].z / CurAnglePointCount;

			//Color Information
			SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.r =
				SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.r 
				+ InputCloud->points[PointIndex].r / CurAnglePointCount;
			
			SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.g =
				SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.g 
				+ InputCloud->points[PointIndex].g / CurAnglePointCount;
			
			SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.b =
				SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition.b 
				+ InputCloud->points[PointIndex].b / CurAnglePointCount;
		}

		SectionAnglePartitionS[SectionIndex].AnglePartition[i].BeforeRefineCenterPointofCurPartition =
			SectionAnglePartitionS[SectionIndex].AnglePartition[i].CenterPointofCurPartition;
	}

	//// 数据点在分区容器后再计算切平面数据
	if (UseTangentPlane)	//如果需要计算切平面
	{
		for(int i = 0; i < PartitionsCount; i++)
		{
			CalcAnglePartitionTangentPlane(SectionIndex, 
				SectionAnglePartitionS[SectionIndex].AnglePartition[i]);
		}		
	}
}

void CAnglePartition::PartitionPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
	double AngleValue, vector<AnglePartitionStruct> & SectionAngle, bool UseTangentPlane)
{
	pcl::PointXYZRGB CenterPoint = GeometryBase::GetCentroidOfPoints(Cloud);

	PartitionPoints(Cloud, CenterPoint, AngleValue, SectionAngle, UseTangentPlane);
}

////2015.06.30 根据给定的点云集及中心点 按照 AngleValue 做角度划分 
void CAnglePartition::PartitionPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
	pcl::PointXYZRGB CenterPoint, double AngleValue,
	vector<AnglePartitionStruct> & SectionAngle, bool UseTangentPlane)
{
	Angle = AngleValue;	

	PartitionsCount = ceil(360.0 / AngleValue);

	SectionPartitionInitiation(SectionAngle);
	for(int i = 0; i < Cloud->points.size(); i++)
	{
		double PointAngleValue = GeometryBase::AngleOfTwoPointsInXY(CenterPoint.x,
			CenterPoint.y, Cloud->points[i].x, Cloud->points[i].y);
	
		int AngleIndex = floor(PointAngleValue / AngleValue);
		SectionAngle[AngleIndex].PointIndexs.push_back(i);
	}

	if (UseTangentPlane)	//如果需要计算切平面 计算每个分区的切平面数据
	{
		for(int i = 0; i < PartitionsCount; i++)
		{
			if (SectionAngle[i].PointIndexs.size() > 0)
				CalcAnglePartitionTangentPlane(Cloud, SectionAngle[i]);
		}		
	}
}

//计算每一个角度分区的切平面参数 2015.07.10
void CAnglePartition::CalcAnglePartitionTangentPlane(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
	AnglePartitionStruct & AnglePartitionValue)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
	//获取每一个角度分区的点
	for (int i = 0; i < AnglePartitionValue.PointIndexs.size(); i++)
	{		
		PointPtr->points.push_back(Cloud->points[AnglePartitionValue.PointIndexs[i]]);
	}		

	Eigen::Matrix3f EigenVector;
	Eigen::Vector3f EigenValue(3);

	AnglePartitionValue.CenterPointofCurPartition =
		GeometryBase::GetPointsTangentPlane(PointPtr, EigenVector, EigenValue);
		
	AnglePartitionValue.TangentPlaneCoefficient.a = EigenVector(0);
	AnglePartitionValue.TangentPlaneCoefficient.b = EigenVector(1);
	AnglePartitionValue.TangentPlaneCoefficient.c = EigenVector(2);		
}

//计算每一个角度分区的切平面参数   默认情况下 d 参数有该角度分区的点云重心点云确定
void CAnglePartition::CalcAnglePartitionTangentPlane(int SectionIndex,
	AnglePartitionStruct & AnglePartitionValue)
{	
	pcl::PointCloud<pcl::PointXYZ>::Ptr PointPtr (new pcl::PointCloud<pcl::PointXYZ>);

	//获取分区的点 只使用分区的点计算切平面存在较大误差
	for (int i = 0; i < AnglePartitionValue.PointIndexs.size(); i++)
	{
		pcl::PointXYZ TempPoint;
		TempPoint.x = InputCloud->points[AnglePartitionValue.PointIndexs[i]].x;
		TempPoint.y = InputCloud->points[AnglePartitionValue.PointIndexs[i]].y;
		TempPoint.z = InputCloud->points[AnglePartitionValue.PointIndexs[i]].z;
		
		PointPtr->points.push_back(TempPoint);
	}	

	Eigen::Matrix3f EigenVector;
	Eigen::Vector3f EigenValue(3);

	AnglePartitionValue.CenterPointofCurPartition = 
		GeometryBase::GetPointsTangentPlane(PointPtr, EigenVector, EigenValue);
	
	AnglePartitionValue.TangentPlaneCoefficient.a = EigenVector(0);
	AnglePartitionValue.TangentPlaneCoefficient.b = EigenVector(1);
	AnglePartitionValue.TangentPlaneCoefficient.c = EigenVector(2);		
	AnglePartitionValue.TangentPlaneCoefficient.d = - 
		(AnglePartitionValue.CenterPointofCurPartition.x * EigenVector(0)
		+ AnglePartitionValue.CenterPointofCurPartition.y * EigenVector(1)
		+ AnglePartitionValue.CenterPointofCurPartition.z * EigenVector(2));
}

//对所有分区进行角度划分
vector<SectionAnglePartition> CAnglePartition::PartitionAllSection(
	int StartIndex, int EndIndex, bool UseTangentPlane)
{
	if (StartIndex == -1) StartIndex = 0;
	if (EndIndex == -1) EndIndex = SectionsVector.size();
	
	for(int i = StartIndex; i < EndIndex; i++)
	{		
		PartitionSection(i, UseTangentPlane);	
	}	
	return SectionAnglePartitionS;
}

//每个分区角度划分的初始化操作
void CAnglePartition::SectionPartitionInitiation(vector<AnglePartitionStruct> & SectionAngle)
{
	SectionAngle.clear();
	for (int i = 0; i < PartitionsCount; i++)
	{
		AnglePartitionStruct TempAnglePartition;
		TempAnglePartition.AngleStart = i * Angle ;	//转换为弧度
		TempAnglePartition.AngleEnd = (i + 1) * Angle; //转换为弧度
		TempAnglePartition.Refined = false;
		TempAnglePartition.NeedRefined = false;		
		TempAnglePartition.MaxDis = TempAnglePartition.MinDis = 0;
		SectionAngle.push_back(TempAnglePartition);	
	}
	if (SectionAngle[PartitionsCount - 1].AngleEnd > 360 )
		SectionAngle[PartitionsCount - 1].AngleEnd = 360;
}

	////根据 分区起始范围及 角度范围获取点云数据
void CAnglePartition::GetPointsByAngleZone(pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutCloud,
	int StartSection, int EndSection,
	double StartAngle, double EndAngle)
{	
	for(int i = 0; i < SectionAnglePartitionS.size(); i++)
	{
		if ((SectionAnglePartitionS[i].SectionIndex >= StartSection) && 
			(SectionAnglePartitionS[i].SectionIndex < EndSection))	//如果在指定的分区范围内
		{
			for(int j = 0; j < SectionAnglePartitionS[i].AnglePartition.size(); j++)
			{	//角度在范围内
				if (((SectionAnglePartitionS[i].AnglePartition[j].AngleStart <= StartAngle)  &&
					(SectionAnglePartitionS[i].AnglePartition[j].AngleEnd > StartAngle))	//偏左位置 
					|| ((SectionAnglePartitionS[i].AnglePartition[j].AngleStart >= StartAngle) &&
					(SectionAnglePartitionS[i].AnglePartition[j].AngleEnd <= EndAngle))		//中间位置
					|| ((SectionAnglePartitionS[i].AnglePartition[j].AngleStart < EndAngle) &&
					(SectionAnglePartitionS[i].AnglePartition[j].AngleEnd > EndAngle)))		//偏右位置

				{
					for(int k = 0; k < SectionAnglePartitionS[i].AnglePartition[j].PointIndexs.size(); k++)
					{
						//The index in InputCloud
						int SectionPointIndex = SectionAnglePartitionS[i].AnglePartition[j].PointIndexs[k];

						OutCloud->points.push_back(InputCloud->points[SectionPointIndex]);

					}
				}
			}
		}
	}
}

//AngleSample, one point for each angle section
void CAnglePartition::AngleSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints, double Angle)
{
	pcl::PointXYZRGB Centriod = GeometryBase::GetCentroidOfPoints(InPoints);

	vector<AnglePartitionStruct> EachPartitions;

	PartitionPoints(InPoints, Centriod, Angle, EachPartitions);

	OutPoints->points.clear();

	for (int i = 0; i < EachPartitions.size(); i++)
	{
		int n = EachPartitions[i].PointIndexs.size();
		if (n > 0)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = 0, TempPoint.y = 0, TempPoint.z = 0;
			for (int j = 0; j < EachPartitions[i].PointIndexs.size(); j++)
			{
				TempPoint.x = TempPoint.x + InPoints->points[EachPartitions[i].PointIndexs[j]].x / n;
				TempPoint.y = TempPoint.y + InPoints->points[EachPartitions[i].PointIndexs[j]].y / n;
				TempPoint.z = TempPoint.z + InPoints->points[EachPartitions[i].PointIndexs[j]].z / n;
			}
			OutPoints->points.push_back(TempPoint);
		}
	}
}

