#include "StemSkeleton.h"

void CStemSkeleton::SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemCloudValue,
	float SectionThickNessValue, bool IsOptimizedValue, int CalcSectionNumbersValue, int StartIndexValue,
	int TheIncludedAngleValue, double MaximumHeightValue)
{
	StemCloud = StemCloudValue;	
	SectionThickNess = SectionThickNessValue;
	CalcSectionNumbers = CalcSectionNumbersValue;
	StartIndex = StartIndexValue;
	IsOptimized = IsOptimizedValue;
	TheIncludedAngle = TheIncludedAngleValue;
	
	//PointBase::PointMoveToOrigin(StemCloud);

	if (MaximumHeightValue > 0)
		MaximumHeight = MaximumHeightValue;
	else
		MaximumHeight = EPSP6;

	HorizontalPartition.SetInputCloud(StemCloud);
	HorizontalPartition.SetThickNess(SectionThickNess);
	HorizontalPartition.PatitionSection();

	if (StartIndex == 0)
		StartIndex = HorizontalPartition.SectionsCount;
}

bool CStemSkeleton::IsSuitForConstruct()
{
	if (HorizontalPartition.SectionsVector[HorizontalPartition.SectionsCount - 1].Indexs.size() > 500)	
		return false;	
	else return true;
}

//因树干有倾斜，垂直分段时，距离地面的分段不会存在残缺，
//但按照树干生长方向分段则有可能出现残缺
pcl::PointXYZRGB CStemSkeleton::GetInitialVector()
{
	//2018.09.27 更新为只在一个区域进行迭代计算生长方向的方法
	CentroidPoints->points.clear();
	pcl::PointXYZRGB LastNormal, CurrentNormal;

	if (HorizontalPartition.GeometryCenterPointsPtr->points.size() == 0)
		HorizontalPartition.CalcCenterPointsByGravity();

	for (int i = StartIndex - 2; i < StartIndex + CalcSectionNumbers; i++) //前面2个质心点保留，以更好地连续获取质心点
	{
		if (!(i >= 0 && i < HorizontalPartition.SectionsVector.size()))
			break;

		CentroidPoints->points.push_back(HorizontalPartition.GeometryCenterPointsPtr->points[i]);		
	}

	LastNormal = GeometryBase::GetMaxDirectionVector(CentroidPoints);
	if (2 + CalcSectionNumbers < CentroidPoints->points.size())
		CentroidPoints->points.erase(CentroidPoints->points.begin() + 2, CentroidPoints->points.begin() + 2 + CalcSectionNumbers);
	else
		CentroidPoints->points.erase(CentroidPoints->points.begin() + 2, CentroidPoints->points.end());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempSlicesPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempVectorPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < CalcSectionNumbers; i++)
	{
		if (!(i >= 0 && i < HorizontalPartition.SectionsVector.size()))
			break;

		pcl::PointXYZRGB Centroid;
		GetSlicePoints(LastNormal, TempSlicesPoints, Centroid);
		CentroidPoints->points.push_back(Centroid);
		TempVectorPoints->points.push_back(Centroid);		
	}
	CurrentNormal = GeometryBase::GetMaxDirectionVector(TempVectorPoints);	

	float Anlge = 180 * GeometryBase::AngleOfTwoVector(LastNormal.x, LastNormal.y, LastNormal.z, CurrentNormal.x, CurrentNormal.y, CurrentNormal.z) / M_PI;
	float LastAngle = 180;

	int Number = 1;

	while (Anlge > 0.5)
	{
		//cout<<"正在进行第"<< Number++<<"次迭代计算初始生长方向"<<endl;
		LastAngle = Anlge;
		LastNormal = CurrentNormal;
		pcl::PointXYZRGB Centroid;

		TempVectorPoints->points.clear();

		if (2 + CalcSectionNumbers < CentroidPoints->points.size())
			CentroidPoints->points.erase(CentroidPoints->points.begin() + 2, CentroidPoints->points.begin() + 2 + CalcSectionNumbers);
		else
			CentroidPoints->points.erase(CentroidPoints->points.begin() + 2, CentroidPoints->points.end());		

		for (int i = 0; i < CalcSectionNumbers; i++)		{
			
			if (!(i >= 0 && i < HorizontalPartition.SectionsVector.size()))
				break;
			
			GetSlicePoints(LastNormal, TempSlicesPoints, Centroid);
			//PointBase::SavePCDToFileName(TempSlicesPoints, "I:\\TempSlicesPoints.pcd");
			CentroidPoints->points.push_back(Centroid);
			TempVectorPoints->points.push_back(Centroid);
		}
		CurrentNormal = GeometryBase::GetMaxDirectionVector(TempVectorPoints);
		//PointBase::SavePCDToFileName(TempVectorPoints, "I:\\TempVectorPoints.pcd");

		Anlge = 180 * GeometryBase::AngleOfTwoVector(LastNormal.x, LastNormal.y, LastNormal.z, CurrentNormal.x, CurrentNormal.y, CurrentNormal.z) / M_PI;

		if (abs(LastAngle - Anlge) < 0.5)
		{
			break;
		}
	}

	CentroidPoints->points.clear();
	CentroidPoints->points.insert(CentroidPoints->points.begin(), 
			TempVectorPoints->points.begin(), TempVectorPoints->points.end());
		
	return LastNormal;
}

void CStemSkeleton::IsHaveTipPoints()
{
	if (HorizontalPartition.SectionsVector.size() == 0) return;
		
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	HorizontalPartition.GetSectionPoints(HorizontalPartition.SectionsVector.size()-2, TempSectionPoints);

	if (TempSectionPoints->points.size() == 0) return;

	CStemDiameter StemDiameter;
	StemDiameter.SetInputCloud(TempSectionPoints);
	double TempValue = StemDiameter.DRetrievalByCircleFittingIn2D();
	if (TempValue > 5) IsHaveTip = false;
}

//计算最大值与最小值
void CStemSkeleton::GetMaxAndMinValueFromPointsAndNormal(
	pcl::PointCloud<PointXYZRGBIndex>::Ptr TempCloud,
	pcl::PointXYZRGB InitialNormal, float & MinValue, float & MaxValue)
{
	if (TempCloud->points.size() <= 0) return;
	PointXYZRGBIndex TempPoint = TempCloud->points[0];
	
	MinValue = -(TempPoint.x * InitialNormal.x + TempPoint.y * InitialNormal.y
		+ TempPoint.z * InitialNormal.z);
	MaxValue = MinValue;

	for(int i = 1; i < TempCloud->points.size(); i++)
	{
		TempPoint = TempCloud->points[i];
		float TempValue = -(TempPoint.x * InitialNormal.x 
			+ TempPoint.y * InitialNormal.y	+ TempPoint.z * InitialNormal.z);
		MaxValue = std::max(TempValue, MaxValue);
		MinValue = std::min(TempValue, MinValue);
	}	
}

void CStemSkeleton::UseSlicesSkeleton()
{
	//CentroidPoints->points.clear();
	//HorizontalPartition.CalcCenterPointsByConvexPolygon();
	CentroidPoints->points.insert(CentroidPoints->points.end(),
		HorizontalPartition.GeometryCenterPointsPtr->points.begin(),
		HorizontalPartition.GeometryCenterPointsPtr->points.end());
}

//渐进式获取树干骨架
void CStemSkeleton::GetSkeletonPointS()
{
	CentroidPoints->points.clear();

	pcl::PointXYZRGB InitialNormalPoint = GetInitialVector();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints (new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::PointXYZRGB TempNomral = InitialNormalPoint;
	int i = 0;
	bool IsCircle = true;

	//Judege the point set is have stem tip points
	IsHaveTipPoints();

	pcl::PointXYZRGB SliceCentroidPoint;

	//向上找质心
	GetSlicePoints(TempNomral, TempPoints, SliceCentroidPoint);

	//cout << "正在向上搜索，当前中心点个数:" << CentroidPoints->points.size() << endl;

	while(TempPoints->points.size() > 0)
	{	
		//检查新获取的质心点构成的方向是否和预期的一致
		pcl::PointXYZRGB NewDirectionNormal;
		NewDirectionNormal.x = SliceCentroidPoint.x - CentroidPoints->points[CentroidPoints->points.size() - 1].x;
		NewDirectionNormal.y = SliceCentroidPoint.y - CentroidPoints->points[CentroidPoints->points.size() - 1].y;
		NewDirectionNormal.z = SliceCentroidPoint.z - CentroidPoints->points[CentroidPoints->points.size() - 1].z;
		double VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(
			TempNomral.x, TempNomral.y, TempNomral.z, NewDirectionNormal.x, 
			NewDirectionNormal.y, NewDirectionNormal.z) / M_PI;
		//cout << "向上与树干生长方向夹角" << VectorAnlge << endl;
		if (VectorAnlge > TheIncludedAngle) break;	//如果获取的		

		if (!IsHaveTip && VectorAnlge > 45) break; //2019.01.23
			
		VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(
			NewDirectionNormal.x, NewDirectionNormal.y, NewDirectionNormal.z, 0, 0, 1) / M_PI;
		//cout << "向上与Z轴夹角" << VectorAnlge << endl;
		if (IsHaveTip)
		{
			if (VectorAnlge > 90) break;	//水平生长的情况出现
		}
		else
		{
			if (VectorAnlge > 45) break;
		}

		CentroidPoints->points.push_back(SliceCentroidPoint);		

		if ((i == CalcSectionNumbers || CalcSectionNumbers > HorizontalPartition.SectionsVector.size()) && (CentroidPoints->points.size() > 5))
		{
			TempPoints->points.clear();
			TempPoints->points.insert(TempPoints->points.begin(), 
				CentroidPoints->points.end() - CalcSectionNumbers, CentroidPoints->points.end());	
			TempNomral = GeometryBase::GetMaxDirectionVector(TempPoints);
			i = 0;
		}

		GetSlicePoints(TempNomral, TempPoints, SliceCentroidPoint);
		i++;

		//达到指定高度则退出
		if (abs(SliceCentroidPoint.z - CentroidPoints->points[0].z) > MaximumHeight)
		{
			cout<<"到达指定高度，退出树干骨架点的计算"<<endl;
			break;
		}
	}

	TempNomral = InitialNormalPoint;
	GetSlicePoints(TempNomral, TempPoints, SliceCentroidPoint, false);
	//cout << "正在向下搜索，当前中心点个数:" << CentroidPoints->points.size() << endl;
	i = 0; //向下寻找时可以不迭代更新生长方向
	while(TempPoints->points.size() > 0)
	{
		GetCentroidOfPointsInSpaces(TempPoints, TempNomral, IsCircle);

		//检查新获取的质心点构成的方向是否和预期的一致
		pcl::PointXYZRGB NewDirectionNormal;
		NewDirectionNormal.x = CentroidPoints->points[0].x - SliceCentroidPoint.x;
		NewDirectionNormal.y = CentroidPoints->points[0].y - SliceCentroidPoint.y;
		NewDirectionNormal.z = CentroidPoints->points[0].z - SliceCentroidPoint.z;
		
		double VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(TempNomral.x, TempNomral.y, TempNomral.z, 
			NewDirectionNormal.x, NewDirectionNormal.y, NewDirectionNormal.z) / M_PI;
		//cout << "向下与树干生长方向夹角" << VectorAnlge << endl;
		if (VectorAnlge > TheIncludedAngle) break;		

		//VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(TempNomral.x, TempNomral.y, TempNomral.z, 0, 0, 1) / M_PI;		
		VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(NewDirectionNormal.x, NewDirectionNormal.y, NewDirectionNormal.z, 0, 0, 1) / M_PI;
		//cout<< "向下与Z轴夹角" << VectorAnlge <<endl;
		if (VectorAnlge > 45) break;	//树根位置处一般与Z轴夹角不会太大, 处理不一样

		CentroidPoints->points.insert(CentroidPoints->points.begin(), SliceCentroidPoint);
		//cout<<"正在向下搜索，当前中心点个数:"<<CentroidPoints->points.size()<<endl;
		
		if (i == CalcSectionNumbers)
		{
			TempPoints->points.clear();
			TempPoints->points.insert(TempPoints->points.begin(),
				CentroidPoints->points.begin(), CentroidPoints->points.begin() + CalcSectionNumbers);
			TempNomral = GeometryBase::GetMaxDirectionVector(TempPoints);
			i = 0;
		}

		GetSlicePoints(TempNomral, TempPoints, SliceCentroidPoint, false);
		i++;

		//达到指定高度则退出
		if (abs(SliceCentroidPoint.z - CentroidPoints->points[0].z) > MaximumHeight)
		{
			cout << "到达指定高度，退出树干骨架点的计算" << endl;
			break;
		}
	}
	//*/
}

//2019.06.15 当前位置的前一个Slice
void CStemSkeleton::GetPriorSlicePoints(pcl::PointXYZRGB NormalPoint,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints,
	pcl::PointXYZRGB & SliceCentroidPoint, bool IsUp, 
	vector<int> * Indexs, int SliceNum)
{
	double a1, b1, c1, d1, d2;
	double Tempd1, Tempd2;
	//int OneIndex, TwoIndex;

	a1 = NormalPoint.x, b1 = NormalPoint.y, c1 = NormalPoint.z;
	pcl::PointXYZRGB ThirdPoint;	

	double MiddleD, UpperD, LowerD;
	int MiddlePointIndex = -1, UpperPointIndex = -1, LowerPointIndex = -1;

	TempPoints->points.clear();

	if (IsUp)
	{
		UpperPointIndex = CentroidPoints->points.size() - 1;
		MiddlePointIndex = UpperPointIndex - 1;

		pcl::PointXYZRGB UpperPlanePoint = GeometryBase::LineCrossPlane(a1, b1, c1,
			CentroidPoints->points[UpperPointIndex],
			a1, b1, c1, -(a1 * CentroidPoints->points[UpperPointIndex].x + b1 *
				CentroidPoints->points[UpperPointIndex].y
				+ c1 * CentroidPoints->points[UpperPointIndex].z));

		pcl::PointXYZRGB TempNormalPoint;

		TempNormalPoint.x = CentroidPoints->points[MiddlePointIndex].x - UpperPlanePoint.x;
		TempNormalPoint.y = CentroidPoints->points[MiddlePointIndex].y - UpperPlanePoint.y;
		TempNormalPoint.z = CentroidPoints->points[MiddlePointIndex].z - UpperPlanePoint.z;

		ThirdPoint = GeometryBase::GetPointAlongLine(TempNormalPoint,
			UpperPlanePoint, SectionThickNess * SliceNum);

		d1 = -(a1 * CentroidPoints->points[UpperPointIndex].x + b1 * CentroidPoints->points[UpperPointIndex].y
			+ c1 * CentroidPoints->points[UpperPointIndex].z);
	}
	else
	{
		//MiddlePointIndex = 1;
		//UpperPointIndex = 2;

		//pcl::PointXYZRGB UpperPlanePoint = GeometryBase::LineCrossPlane(a1, b1, c1, CentroidPoints->points[MiddlePointIndex],
		//	a1, b1, c1, -(a1 * CentroidPoints->points[UpperPointIndex].x + b1 * CentroidPoints->points[UpperPointIndex].y
		//		+ c1 * CentroidPoints->points[UpperPointIndex].z));

		//pcl::PointXYZRGB TempNormalPoint;

		//TempNormalPoint.x = CentroidPoints->points[MiddlePointIndex].x - UpperPlanePoint.x;
		//TempNormalPoint.y = CentroidPoints->points[MiddlePointIndex].y - UpperPlanePoint.y;
		//TempNormalPoint.z = CentroidPoints->points[MiddlePointIndex].z - UpperPlanePoint.z;

		//ThirdPoint = GeometryBase::GetPointAlongLine(TempNormalPoint,
		//	CentroidPoints->points[MiddlePointIndex], SectionThickNess * SliceNum);
	}	

	d2 = -(a1 * ThirdPoint.x + b1 * ThirdPoint.y + c1 * ThirdPoint.z);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempIndexPoints (new pcl::PointCloud<PointXYZRGBIndex>);	

	GeometryBase::GetPointsBetweenTwoPlanes(
		StemCloud, a1, b1, c1, d1, a1, b1, c1, d2, TempPoints, 1, true, false, false, Indexs);
	//PointBase::PointXYZRGBIndexToPointXYZRGB(TempIndexPoints, TempPoints);

	if (TempPoints->points.size() > 0)
	{
		//2018.09.27 获取在截面上的质心投影点
		bool IsCircle;
		pcl::PointXYZRGB TempCentroidPoint = GetCentroidOfPointsInSpaces(TempPoints, NormalPoint, IsCircle);

		TempPoints->points.push_back(TempCentroidPoint);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempOutPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		GeometryBase::ProjectPointsToPlane(TempPoints, TempOutPoints, NormalPoint, ThirdPoint);

		SliceCentroidPoint.x = TempOutPoints->points[TempOutPoints->points.size() - 1].x;
		SliceCentroidPoint.y = TempOutPoints->points[TempOutPoints->points.size() - 1].y;
		SliceCentroidPoint.z = TempOutPoints->points[TempOutPoints->points.size() - 1].z;
		TempPoints->points.pop_back();
	}
	else
	{
		cout << "没有获取到Slice上的点" << endl;
	}
}

//获取 以NormalPoint为方向，IsUp决定上下，参考点为CentroidPoints中的点，厚度为SectionThickNess的 
//两个平面中间的点集
//返回值是另一个平面上的一个点
pcl::PointXYZRGB CStemSkeleton::GetSlicePoints(pcl::PointXYZRGB NormalPoint,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints,
	pcl::PointXYZRGB & SliceCentroidPoint, bool IsUp, vector<int> * Indexs)
{
	double a1, b1, c1, d1, d2;
	double Tempd1, Tempd2;
	//int OneIndex, TwoIndex;
		
	a1 = NormalPoint.x, b1 = NormalPoint.y, c1 = NormalPoint.z;
	pcl::PointXYZRGB ThirdPoint;
	//2018.09.27 修改为新方法

	double MiddleD, UpperD, LowerD;
	int MiddlePointIndex = -1, UpperPointIndex = -1, LowerPointIndex = -1;

	TempPoints->points.clear();
	//cout << "SectionThickNess:" << SectionThickNess << endl;
	if (IsUp)
	{
		MiddlePointIndex = CentroidPoints->points.size() - 1;
		LowerPointIndex = MiddlePointIndex - 1;

		pcl::PointXYZRGB LowerPlanePoint = GeometryBase::LineCrossPlane(a1, b1, c1, 
			CentroidPoints->points[MiddlePointIndex],
			a1, b1, c1, -(a1 * CentroidPoints->points[LowerPointIndex].x + b1 * 
				CentroidPoints->points[LowerPointIndex].y 
						+ c1 * CentroidPoints->points[LowerPointIndex].z));

		pcl::PointXYZRGB TempNormalPoint;

		TempNormalPoint.x = CentroidPoints->points[MiddlePointIndex].x - LowerPlanePoint.x;
		TempNormalPoint.y = CentroidPoints->points[MiddlePointIndex].y - LowerPlanePoint.y;
		TempNormalPoint.z = CentroidPoints->points[MiddlePointIndex].z - LowerPlanePoint.z;

		ThirdPoint = GeometryBase::GetPointAlongLine(TempNormalPoint, 
			CentroidPoints->points[MiddlePointIndex], SectionThickNess);
	}
	else
	{
		MiddlePointIndex = 0;
		UpperPointIndex = 1;

		pcl::PointXYZRGB UpperPlanePoint = GeometryBase::LineCrossPlane(a1, b1, c1, CentroidPoints->points[MiddlePointIndex],
			a1, b1, c1, -(a1 * CentroidPoints->points[UpperPointIndex].x + b1 * CentroidPoints->points[UpperPointIndex].y
				+ c1 * CentroidPoints->points[UpperPointIndex].z));

		pcl::PointXYZRGB TempNormalPoint;

		TempNormalPoint.x = CentroidPoints->points[MiddlePointIndex].x - UpperPlanePoint.x;
		TempNormalPoint.y = CentroidPoints->points[MiddlePointIndex].y - UpperPlanePoint.y;
		TempNormalPoint.z = CentroidPoints->points[MiddlePointIndex].z - UpperPlanePoint.z;

		ThirdPoint = GeometryBase::GetPointAlongLine(TempNormalPoint, 
			CentroidPoints->points[MiddlePointIndex], SectionThickNess);
	}

	d1 = -(a1 * CentroidPoints->points[MiddlePointIndex].x + b1 * CentroidPoints->points[MiddlePointIndex].y
		+ c1 * CentroidPoints->points[MiddlePointIndex].z);

	d2 = -(a1 * ThirdPoint.x + b1 * ThirdPoint.y + c1 * ThirdPoint.z);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempIndexPoints (new pcl::PointCloud<PointXYZRGBIndex>);	
	
	GeometryBase::GetPointsBetweenTwoPlanes(
		StemCloud, a1, b1, c1, d1, a1, b1, c1, d2, TempPoints, 1, true, false, false, Indexs);
	//PointBase::PointXYZRGBIndexToPointXYZRGB(TempIndexPoints, TempPoints);

	//PointBase::SavePCDToFileName(TempPoints, "I:\\Test.pcd");

	if (TempPoints->points.size() > 0)
	{		
		//2018.09.27 获取在截面上的质心投影点
		bool IsCircle;
		pcl::PointXYZRGB TempCentroidPoint = GetCentroidOfPointsInSpaces(TempPoints, NormalPoint, IsCircle);

		TempPoints->points.push_back(TempCentroidPoint);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempOutPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		GeometryBase::ProjectPointsToPlane(TempPoints, TempOutPoints, NormalPoint, ThirdPoint);

		SliceCentroidPoint.x = TempOutPoints->points[TempOutPoints->points.size() - 1].x;
		SliceCentroidPoint.y = TempOutPoints->points[TempOutPoints->points.size() - 1].y;
		SliceCentroidPoint.z = TempOutPoints->points[TempOutPoints->points.size() - 1].z;
		TempPoints->points.pop_back();
	}
	else
	{
		cout <<"没有获取到Slice上的点"<< endl;
	}

	return ThirdPoint;
}

//获取点集的质心点
pcl::PointXYZRGB CStemSkeleton::GetCentroidOfPointsInSpaces(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
	pcl::PointXYZRGB NormalPoint, bool & IsCircle)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints (new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints (new pcl::PointCloud<pcl::PointXYZRGB>);	

	GeometryBase::RotateNormalToVertical(InPoints, PlanePoints, NormalPoint);
	//PointBase::SavePCDToFileName(InPoints, "I:\\InPlanePoints.pcd");
	
	pcl::PointXYZRGB TempCentriod = GeometryBase::GetCentroidOfPoints(PlanePoints);	
	
	IsCircle = PlanePointsIsCircle(PlanePoints, TempCentriod);

	PlanePoints->points.push_back(TempCentriod);		
	//PointBase::SavePCDToFileName(PlanePoints, "I:\\PlanePoints.pcd");
	GeometryBase::RotateToOriginal(PlanePoints, OutPoints, NormalPoint, true);

	//PointBase::SavePCDToFileName(OutPoints, "I:\\OutPoints_PlanePoints.pcd");

	return OutPoints->points[OutPoints->points.size()-1];	
}

bool CStemSkeleton::PlanePointsIsCircle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints, 
	pcl::PointXYZRGB Centriod)
{
	bool IsCircle = true;
	CAnglePartition AnglePartitionInstance;
	vector<AnglePartitionStruct> SectionAngle;
	AnglePartitionInstance.PartitionPoints(InPoints, 15, SectionAngle);
	
	for(int i = 0; i < SectionAngle.size(); i++)
	{
		if (SectionAngle[i].PointIndexs.size() == 0)
		{
			IsCircle = false;
			break;
		}		
	}
	return IsCircle;
}

//构建树干中心点的样条曲线  2016.04.06
void CStemSkeleton::ConstructStemSplineCurve(double StartHeight, bool IsDirect)
{
	if (StemCloud->points.size() == 0) return;

	CentroidPoints->points.clear();
	OptimizedCentroidPoints->points.clear();
	StemSkeletonPoints->points.clear();
	AllProfilecurvePoints->points.clear();

	CSplineInterpolation SplineInterpolation;
	
	if (!IsDirect)	//重新获取树干骨架点
	{
		//2018.10.24 如果树高不足StartHeightCM时,就从中间位置开始
		StartIndex = StartHeight / HorizontalPartition.SectionThickness - 1;

		if (StartIndex > HorizontalPartition.SectionsVector.size() - CalcSectionNumbers)
			StartIndex = HorizontalPartition.SectionsVector.size() / 2;

		if (StartIndex + CalcSectionNumbers >= HorizontalPartition.SectionsVector.size())
			CalcSectionNumbers = CalcSectionNumbers / 2;

		if (StartIndex + CalcSectionNumbers >= HorizontalPartition.SectionsVector.size())	
			StartIndex = 2;	

		HorizontalPartition.CalcCenterPointsByGravity();
		if ((CalcSectionNumbers >= HorizontalPartition.SectionsVector.size())
			|| (StartIndex + CalcSectionNumbers >= HorizontalPartition.SectionsVector.size()))
		{
			CentroidPoints->points.clear();
			CentroidPoints->points.insert(CentroidPoints->points.end(),
				HorizontalPartition.GeometryCenterPointsPtr->points.begin(),
				HorizontalPartition.GeometryCenterPointsPtr->points.end());		
		}
		else
		{		
			GetSkeletonPointS();	
		}
	}

	cout<<"轮廓点已经选取"<<endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints (new pcl::PointCloud<pcl::PointXYZRGB>); 

	//对质心点优化 //数量减少后效果好 2018.09.25
	if (IsOptimized)
	{		
		OptimizedCentroidPoints->points.clear();
		//for(int i = 0; i < CentroidPoints->points.size(); i = i + 3)
		OptimizedCentroidPoints->points.push_back(CentroidPoints->points[0]);
		for (int i = 1; i < CentroidPoints->points.size() - 1; i = i + 3)	//确保其实节点不会消失
		{
			pcl::PointXYZRGB OptimizedPoint;
		
			if (i + 2 < CentroidPoints->points.size())
			{
				OptimizedPoint.x = (CentroidPoints->points[i].x + CentroidPoints->points[i+1].x + CentroidPoints->points[i+2].x)/3; 
				OptimizedPoint.y = (CentroidPoints->points[i].y + CentroidPoints->points[i+1].y + CentroidPoints->points[i+2].y)/3; 
				OptimizedPoint.z = (CentroidPoints->points[i].z + CentroidPoints->points[i+1].z + CentroidPoints->points[i+2].z)/3; 
				OptimizedCentroidPoints->points.push_back(OptimizedPoint);
			}
			else
			{   int k = CentroidPoints->points.size() - i;
				OptimizedPoint.x = 0, OptimizedPoint.y = 0, OptimizedPoint.z = 0;
				for(int j = i; j < CentroidPoints->points.size(); j++)
				{
					OptimizedPoint.x = OptimizedPoint.x + CentroidPoints->points[j].x/k;
					OptimizedPoint.y = OptimizedPoint.y + CentroidPoints->points[j].y/k;
					OptimizedPoint.z = OptimizedPoint.z + CentroidPoints->points[j].z/k;					
				}				
				OptimizedCentroidPoints->points.push_back(OptimizedPoint);
			}		
		}		
		OptimizedCentroidPoints->points.push_back(CentroidPoints->points[CentroidPoints->points.size() - 1]);

		SplineInterpolation.SetInputs(OptimizedCentroidPoints, 3, false, false);	
	}
	else	
		SplineInterpolation.SetInputs(CentroidPoints, 3, false, false);	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> KnoteVector;				
	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnoteVector);

	cout << "控制点已经计算完成！" << endl;

	StemSkeletonSpline.SetSplineInputs(ControlPoints, 3, KnoteVector, false);	
	StemSkeletonSpline.CreateSpline();
	StemSkeletonPoints = StemSkeletonSpline.CurvePoints;	
	ResultStr = "Done";
}

////在指定的位置处画横断面与横断面轮廓曲线
//void CStemSkeleton::DrawCrossSectionalPlaneByHeight(float Height, int LineWidth, float PointSize, bool ShowProfileCurve,
//	bool DrawUpper)
//{
//	//获取测量位置
//	pcl::PointXYZRGB StemCrossPlanePoint, StemCrossSectionNormal;
//	
//	CurrentHeightShow = Height;
//
//	double UValue = StemSkeletonSpline.GetUValueBySplineHeight(Height);
//
//	if (abs(UValue - 0) <= 0.001 || abs(UValue - 1) <= 0.001) return;
//
//	StemCrossPlanePoint = StemSkeletonSpline.GetSplinePoint(UValue);	
//
//	StemCrossSectionNormal = StemSkeletonSpline.GetSplineDerivativePoint(UValue, 1);
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//	
//	StemCrossPlanePoint.rgba = ColorBase::RedColor;
//	ControlPoints->points.push_back(StemCrossPlanePoint);
//		
//	{
//		//PointBase::ShowPointXYZRGB(Viewer, ControlPoints, "ControlPoints"+ StringBase::ClockValue(), 5 );
//		//PointBase::ShowPlane(Viewer, StemCrossSectionNormal.x, StemCrossSectionNormal.y, StemCrossSectionNormal.z,
//		//	StemCrossPlanePoint, "SplineCross_Section"+StringBase::ClockValue(),"",true, LineWidth);
//		
//		ShowCrossSectionalPlaneByCircle(StemCrossPlanePoint, StemCrossSectionNormal, LineWidth, PointSize);
//	
//		if (DrawUpper)
//		{
//			pcl::PointXYZRGB AnotherPoint = GeometryBase::GetPointAlongLine(
//				StemCrossSectionNormal, StemCrossPlanePoint, 1);
//
//			//PointBase::ShowPlane(Viewer, StemCrossSectionNormal.x, StemCrossSectionNormal.y, StemCrossSectionNormal.z,
//			//	AnotherPoint, "SplineCross_Section_Upper"+StringBase::ClockValue(),"",true, LineWidth, 1, 0, 255, 0);
//			
//			ShowCrossSectionalPlaneByCircle(AnotherPoint, StemCrossSectionNormal, LineWidth, PointSize, 0, 255, 0);
//		}
//	}
//		
//	DrawCrossSectionalCurveAtU(UValue, PointSize);	//在此处显示横断面轮廓曲线	
//}

////从开始位置StartHeight处 至 EndHeight处 画 PlaneNumbers 个横断面 2017.05.27
//void CStemSkeleton::DrawCrossSectionalPlane(float StartHeight, float EndHeight, float PlaneNumbers, 
//	float PointSize,
//	bool ShowProfileCurve, bool DrawUpper)
//{
//	double StartUValue = StemSkeletonSpline.GetUValueBySplineLength(StartHeight);
//
//	if (abs(StartUValue - 1) <= 0.01) return;
//
//	double EndUValue = StemSkeletonSpline.GetUValueBySplineLength(EndHeight);
//
//	if (abs(EndUValue - 1) <= 0.01) EndUValue = 0.99;
//
//	float UStep = (EndUValue - StartUValue) / PlaneNumbers;
//
//	double UValue = StartUValue;
//
//	int i = 0;
//
//	while (UValue < EndUValue)
//	{
//		cout<<"正在画第 "<<++i<<" 个横断面,当前U值："<<UValue<<"，结束U值："<<EndUValue<<endl;
//
//		pcl::PointXYZRGB StemCrossPlanePoint, StemCrossSectionNormal;
//
//		StemCrossPlanePoint = StemSkeletonSpline.GetSplinePoint(UValue);	
//
//		StemCrossSectionNormal = StemSkeletonSpline.GetSplineDerivativePoint(UValue, 1);
//
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//	
//		StemCrossPlanePoint.rgba = ColorBase::RedColor;
//		ControlPoints->points.push_back(StemCrossPlanePoint);
//
//		if (!ShowProfileCurve)
//		{
//			//PointBase::ShowPointXYZRGB(Viewer, ControlPoints, "ControlPoints"+ StringBase::ClockValue(), 5 );
//			PointBase::ShowPlane(Viewer, StemCrossSectionNormal.x,
//				StemCrossSectionNormal.y, StemCrossSectionNormal.z,
//				StemCrossPlanePoint, 
//				"SplineCross_Section"+StringBase::ClockValue(),"",true, 8);
//	
//			if (DrawUpper)
//			{
//				pcl::PointXYZRGB AnotherPoint = PointBase::GetPointAlongLineToAPoint(
//					StemCrossSectionNormal, StemCrossPlanePoint, 1);
//				PointBase::ShowPlane(Viewer, StemCrossSectionNormal.x,
//					StemCrossSectionNormal.y, StemCrossSectionNormal.z,
//					AnotherPoint,
//					"SplineCross_Section_Upper"+StringBase::ClockValue(),"",true, 8, 1, 0, 255, 0);
//			}
//		}
//		else		
//			DrawCrossSectionalCurveAtU(UValue, PointSize);	//在此处显示横断面轮廓曲线
//
//		UValue = UValue + UStep;
//	}
//}

////角度抽样， 一角度只取一个点
// void CStemSkeleton::AngleSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr InPoints,
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutPoints, double Angle)
//{
//	pcl::PointXYZRGB Centriod = GeometryBase::GetCentroidOfPoints(InPoints);
//
//	vector<AnglePartitionStruct> EachPartitions;
//
//	CAnglePartition TempAnglePartition;
//	TempAnglePartition.PartitionPoints(InPoints,
//		Centriod, Angle, EachPartitions);
//
//	OutPoints->points.clear();
//
//	for(int i = 0; i < EachPartitions.size(); i++)
//	{
//		int n = EachPartitions[i].PointIndexs.size();
//		if (n > 0)
//		{
//			pcl::PointXYZRGB TempPoint;
//			TempPoint.x = 0, TempPoint.y = 0, TempPoint.z = 0;
//			for(int j = 0; j < EachPartitions[i].PointIndexs.size(); j++)
//			{
//				TempPoint.x = TempPoint.x + InPoints->points[EachPartitions[i].PointIndexs[j]].x/n;
//				TempPoint.y = TempPoint.y + InPoints->points[EachPartitions[i].PointIndexs[j]].y/n;
//				TempPoint.z = TempPoint.z + InPoints->points[EachPartitions[i].PointIndexs[j]].z/n;
//			}
//			OutPoints->points.push_back(TempPoint);
//		}
//	}
//}

// //2016.04.15  计算横断面的真实面积
//double CStemSkeleton::GetCrossSectionalProfileAndArea(
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CrossSectionalProfilePtr,
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurve, bool IsArea)
//{
//	double Area = 0;
//
//	//三次B样条横断面轮廓
//	ProfileCurve->points.clear();
//
//	CSplineInterpolation SplineInterpolation;
//	SplineInterpolation.SetInputs(CrossSectionalProfilePtr, 3, true);	
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//	vector<double> KnoteVector;		
//	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnoteVector, true);
//
//	//无理样条
//	//CRationalSpline RationalSpline;
//	CSpline RationalSpline;
//	RationalSpline.Viewer = Viewer;
//	RationalSpline.FirstPointOfCloseCurve = CrossSectionalProfilePtr->points[0];
//	//需要显示曲线的非凸区域
//	RationalSpline.SetSplineInputs(ControlPoints, 3, KnoteVector, true, false);
//	//double SplineLength = RationalSpline.CreateSpline();	
//	RationalSpline.CreateSpline();	
//	if (IsArea)
//		Area = RationalSpline.GetSplineArea();
//	
//	ProfileCurve->points.insert(ProfileCurve->points.begin(),
//		RationalSpline.CurvePoints->points.begin(),
//		RationalSpline.CurvePoints->points.end());
//
//	return Area;
//}

//void CStemSkeleton::ShowCrossSectionalPlaneByCircle(pcl::PointXYZRGB CenterPoint, pcl::PointXYZRGB NormalPoint, double Radius, int PointSize,
//	int r, int g, int b)
//{
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CriclePoints(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutCriclePoints(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//	PointBase::PointNormalized(NormalPoint);	
//	GeometryBase::GetCirclePoints(CriclePoints, CenterPoint, Radius, M_PI / 1000);
//	CriclePoints->points.insert(CriclePoints->points.begin(), CenterPoint);
//	int TempColor = ((int)r) << 16 | ((int)g) << 8 | ((int)b);			
//	PointBase::SetPointColor(CriclePoints, TempColor);	
//	GeometryBase::RotateToOriginal(CriclePoints, OutCriclePoints, NormalPoint);
//
//	PointsMove(OutCriclePoints, CenterPoint.x - OutCriclePoints->points[0].x,
//		CenterPoint.y - OutCriclePoints->points[0].y, CenterPoint.z - OutCriclePoints->points[0].z);
//	OutCriclePoints->points.erase(OutCriclePoints->points.begin());
//
//	PointBase::ShowPointXYZRGB(Viewer, OutCriclePoints, "Circle"+StringBase::ClockValue(), PointSize);
//}

void CStemSkeleton::SaveInclinationCurvatureTorsionToFile(double StartLength, double EndLength, double Interval)
{
	double CurEndLength = StartLength;
	
	vector<double> LengthValues;
	vector<double> InclinationValues;
	vector<double> CurvatureValues;
	vector<double> TorsionValues;

	while (CurEndLength <= EndLength)
	{
		double UValue = StemSkeletonSpline.GetUValueBySplineLength(CurEndLength);

		LengthValues.push_back(CurEndLength);
		InclinationValues.push_back(StemSkeletonSpline.GetInclination(UValue));
		CurvatureValues.push_back(StemSkeletonSpline.GetCurvature(UValue));
		TorsionValues.push_back(StemSkeletonSpline.GetTorsion(UValue));

		CurEndLength = CurEndLength + Interval;
	}	

	//ofstream outfile("E:\\AAPapers\\ISPRS Of Stem Model\\Matlab\\StemShape.txt", ios::trunc);
	//	
	//for (int i = 0; i < InclinationValues.size(); i++)	//
	//{
	//	outfile << LengthValues[i] << "," << InclinationValues[i] << ","
	//			<< CurvatureValues[i] << "," << TorsionValues[i] << endl;
	//}
	//outfile.close();
}

void CStemSkeleton::DrawCrossSectionalCurve(float Height, float PointSize)
{
//获取测量位置
	pcl::PointXYZRGB StemCrossPlanePoint, StemCrossSectionNormal;
	
	double UValue = StemSkeletonSpline.GetUValueBySplineLength(Height);

	DrawCrossSectionalCurveAtU(UValue, PointSize);
}


//在给定u值处显示横断面
void CStemSkeleton::DrawCrossSectionalCurveAtU(float UValue, float PointSize)
{
	pcl::PointXYZRGB StemCrossPlanePoint, StemCrossSectionNormal;	
	StemCrossPlanePoint = StemSkeletonSpline.GetSplinePoint(UValue);	

	//获取树干的生长方向，默认取标准化向量
	StemCrossSectionNormal = StemSkeletonSpline.GetSplineDerivativePoint(UValue, 1, true);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	StemCrossPlanePoint.rgba = ColorBase::RedColor;
	ControlPoints->points.push_back(StemCrossPlanePoint);

	//PointBase::ShowPointXYZRGB(Viewer, ControlPoints, "ControlPoints"+ StringBase::ClockValue(), 5);
	//PointBase::ShowPlane(Viewer, StemCrossSectionNormal.x,
	//	StemCrossSectionNormal.y, StemCrossSectionNormal.z,
	//	StemCrossPlanePoint, 
	//	"SplineCross_Section"+StringBase::ClockValue(),"",true, 8);
	
	pcl::PointXYZRGB AnotherPoint = GeometryBase::GetPointAlongLine(
		StemCrossSectionNormal, StemCrossPlanePoint, 1);
	//PointBase::ShowPlane(Viewer, StemCrossSectionNormal.x,
	//		StemCrossSectionNormal.y, StemCrossSectionNormal.z,
	//		AnotherPoint,
	//		"SplineCross_Section_Upper"+StringBase::ClockValue(),"",true, 8,1,0,255,0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CalcPoints (new pcl::PointCloud<pcl::PointXYZRGB>);					//三维空间中的点集
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CalcPlanePoints (new pcl::PointCloud<pcl::PointXYZRGB>);				//平面点集
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CrossSectionalProfilePtr (new pcl::PointCloud<pcl::PointXYZRGB>);	//轮廓点集
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurve (new pcl::PointCloud<pcl::PointXYZRGB>);	//轮廓点集
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurve3D (new pcl::PointCloud<pcl::PointXYZRGB>);	//轮廓点集

	GeometryBase::GetPointsBetweenTwoPlanes(
		StemCloud,
		StemCrossSectionNormal.x,StemCrossSectionNormal.y,StemCrossSectionNormal.z,
		-(StemCrossPlanePoint.x * StemCrossSectionNormal.x + StemCrossPlanePoint.y * StemCrossSectionNormal.y + StemCrossPlanePoint.z * StemCrossSectionNormal.z),
		StemCrossSectionNormal.x,StemCrossSectionNormal.y,StemCrossSectionNormal.z,
		-(AnotherPoint.x * StemCrossSectionNormal.x + AnotherPoint.y * StemCrossSectionNormal.y + AnotherPoint.z * StemCrossSectionNormal.z),
		CalcPoints, 1, true, false, false);

		if (CalcPoints->points.size() == 0)
		{
			cout << "未得到横断面的轮廓点" << endl;
			return;
		}

	//PointBase::PointXYZRGBIndexToPointXYZRGB(TempIndexPoints, CalcPoints);
	//2017.05.28 注释的内容
	//PointBase::SetPointColor(TempIndexPoints, ColorBase::BlueColor);
	//PointBase::ShowPointXYZRGB(Viewer, TempIndexPoints, "TempIndexPoints"+ StringBase::ClockValue(), 3);

	//将获取的点集转换为平面点集
	CalcBase<double> CalcBaseDouble;

	GeometryBase::RotateNormalToVertical(CalcPoints, CalcPlanePoints, StemCrossSectionNormal);
	
	//旋转后才能设置	
	//这种设置方式更好 2017.05.28
	PointBase::SetPointsCoordinateValue(CalcPlanePoints, "Z", CalcPlanePoints->points[0].z);

	//PointBase::SavePCDToFileName(CalcPlanePoints,
	//	"I:\\3DStemModel\\CalcProfileCurve3D_" + CalcBaseDouble.ConvertToString(CurrentHeightShow) + "_" + StringBase::ClockValue() + ".pcd");

	//2017.05.28 注释的内容
	//PointBase::SetPointColor(CalcPlanePoints, ColorBase::GreenColor);
	//PointBase::ShowPointXYZRGB(Viewer, CalcPlanePoints, "CalcPlanePoints"+ StringBase::ClockValue(), 3);

	//2016.04.16 角度抽样得到轮廓点
	//AngleSample(CalcPlanePoints, CrossSectionalProfilePtr);
	//	
	////此处只显示横断面轮廓曲线
	//GetCrossSectionalProfileAndArea(CrossSectionalProfilePtr, ProfileCurve, false);	
	//PointBase::SavePCDToFileName(ProfileCurve, 
	//	"I:\\3DStemModel\\ProfileCurve3D_" + CalcBaseDouble.ConvertToString(CurrentHeightShow) +  "_"+ StringBase::ClockValue() + ".pcd");

	GeometryBase::RotateToOriginal(ProfileCurve, ProfileCurve3D, StemCrossSectionNormal);
	//cout<<"当前位置处的树干生长方向："<<StemCrossSectionNormal<<endl;

	PointBase::SetPointColor(ProfileCurve3D, ColorBase::RedColor);
	PointBase::ShowPointXYZRGB(Viewer, ProfileCurve3D, "ProfileCurve3D"+ StringBase::ClockValue(), 2);	
	AllProfilecurvePoints->points.insert(AllProfilecurvePoints->points.begin(),
		ProfileCurve3D->points.begin(), ProfileCurve3D->points.end());
}