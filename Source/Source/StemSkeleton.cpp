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

//����������б����ֱ�ֶ�ʱ���������ķֶβ�����ڲ�ȱ��
//������������������ֶ����п��ܳ��ֲ�ȱ
pcl::PointXYZRGB CStemSkeleton::GetInitialVector()
{
	//2018.09.27 ����Ϊֻ��һ��������е���������������ķ���
	CentroidPoints->points.clear();
	pcl::PointXYZRGB LastNormal, CurrentNormal;

	if (HorizontalPartition.GeometryCenterPointsPtr->points.size() == 0)
		HorizontalPartition.CalcCenterPointsByGravity();

	for (int i = StartIndex - 2; i < StartIndex + CalcSectionNumbers; i++) //ǰ��2�����ĵ㱣�����Ը��õ�������ȡ���ĵ�
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
		//cout<<"���ڽ��е�"<< Number++<<"�ε��������ʼ��������"<<endl;
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

//�������ֵ����Сֵ
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

//����ʽ��ȡ���ɹǼ�
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

	//����������
	GetSlicePoints(TempNomral, TempPoints, SliceCentroidPoint);

	//cout << "����������������ǰ���ĵ����:" << CentroidPoints->points.size() << endl;

	while(TempPoints->points.size() > 0)
	{	
		//����»�ȡ�����ĵ㹹�ɵķ����Ƿ��Ԥ�ڵ�һ��
		pcl::PointXYZRGB NewDirectionNormal;
		NewDirectionNormal.x = SliceCentroidPoint.x - CentroidPoints->points[CentroidPoints->points.size() - 1].x;
		NewDirectionNormal.y = SliceCentroidPoint.y - CentroidPoints->points[CentroidPoints->points.size() - 1].y;
		NewDirectionNormal.z = SliceCentroidPoint.z - CentroidPoints->points[CentroidPoints->points.size() - 1].z;
		double VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(
			TempNomral.x, TempNomral.y, TempNomral.z, NewDirectionNormal.x, 
			NewDirectionNormal.y, NewDirectionNormal.z) / M_PI;
		//cout << "������������������н�" << VectorAnlge << endl;
		if (VectorAnlge > TheIncludedAngle) break;	//�����ȡ��		

		if (!IsHaveTip && VectorAnlge > 45) break; //2019.01.23
			
		VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(
			NewDirectionNormal.x, NewDirectionNormal.y, NewDirectionNormal.z, 0, 0, 1) / M_PI;
		//cout << "������Z��н�" << VectorAnlge << endl;
		if (IsHaveTip)
		{
			if (VectorAnlge > 90) break;	//ˮƽ�������������
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

		//�ﵽָ���߶����˳�
		if (abs(SliceCentroidPoint.z - CentroidPoints->points[0].z) > MaximumHeight)
		{
			cout<<"����ָ���߶ȣ��˳����ɹǼܵ�ļ���"<<endl;
			break;
		}
	}

	TempNomral = InitialNormalPoint;
	GetSlicePoints(TempNomral, TempPoints, SliceCentroidPoint, false);
	//cout << "����������������ǰ���ĵ����:" << CentroidPoints->points.size() << endl;
	i = 0; //����Ѱ��ʱ���Բ�����������������
	while(TempPoints->points.size() > 0)
	{
		GetCentroidOfPointsInSpaces(TempPoints, TempNomral, IsCircle);

		//����»�ȡ�����ĵ㹹�ɵķ����Ƿ��Ԥ�ڵ�һ��
		pcl::PointXYZRGB NewDirectionNormal;
		NewDirectionNormal.x = CentroidPoints->points[0].x - SliceCentroidPoint.x;
		NewDirectionNormal.y = CentroidPoints->points[0].y - SliceCentroidPoint.y;
		NewDirectionNormal.z = CentroidPoints->points[0].z - SliceCentroidPoint.z;
		
		double VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(TempNomral.x, TempNomral.y, TempNomral.z, 
			NewDirectionNormal.x, NewDirectionNormal.y, NewDirectionNormal.z) / M_PI;
		//cout << "������������������н�" << VectorAnlge << endl;
		if (VectorAnlge > TheIncludedAngle) break;		

		//VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(TempNomral.x, TempNomral.y, TempNomral.z, 0, 0, 1) / M_PI;		
		VectorAnlge = 180.0 * GeometryBase::AngleOfTwoVector(NewDirectionNormal.x, NewDirectionNormal.y, NewDirectionNormal.z, 0, 0, 1) / M_PI;
		//cout<< "������Z��н�" << VectorAnlge <<endl;
		if (VectorAnlge > 45) break;	//����λ�ô�һ����Z��нǲ���̫��, ����һ��

		CentroidPoints->points.insert(CentroidPoints->points.begin(), SliceCentroidPoint);
		//cout<<"����������������ǰ���ĵ����:"<<CentroidPoints->points.size()<<endl;
		
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

		//�ﵽָ���߶����˳�
		if (abs(SliceCentroidPoint.z - CentroidPoints->points[0].z) > MaximumHeight)
		{
			cout << "����ָ���߶ȣ��˳����ɹǼܵ�ļ���" << endl;
			break;
		}
	}
	//*/
}

//2019.06.15 ��ǰλ�õ�ǰһ��Slice
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
		//2018.09.27 ��ȡ�ڽ����ϵ�����ͶӰ��
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
		cout << "û�л�ȡ��Slice�ϵĵ�" << endl;
	}
}

//��ȡ ��NormalPointΪ����IsUp�������£��ο���ΪCentroidPoints�еĵ㣬���ΪSectionThickNess�� 
//����ƽ���м�ĵ㼯
//����ֵ����һ��ƽ���ϵ�һ����
pcl::PointXYZRGB CStemSkeleton::GetSlicePoints(pcl::PointXYZRGB NormalPoint,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints,
	pcl::PointXYZRGB & SliceCentroidPoint, bool IsUp, vector<int> * Indexs)
{
	double a1, b1, c1, d1, d2;
	double Tempd1, Tempd2;
	//int OneIndex, TwoIndex;
		
	a1 = NormalPoint.x, b1 = NormalPoint.y, c1 = NormalPoint.z;
	pcl::PointXYZRGB ThirdPoint;
	//2018.09.27 �޸�Ϊ�·���

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
		//2018.09.27 ��ȡ�ڽ����ϵ�����ͶӰ��
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
		cout <<"û�л�ȡ��Slice�ϵĵ�"<< endl;
	}

	return ThirdPoint;
}

//��ȡ�㼯�����ĵ�
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

//�����������ĵ����������  2016.04.06
void CStemSkeleton::ConstructStemSplineCurve(double StartHeight, bool IsDirect)
{
	if (StemCloud->points.size() == 0) return;

	CentroidPoints->points.clear();
	OptimizedCentroidPoints->points.clear();
	StemSkeletonPoints->points.clear();
	AllProfilecurvePoints->points.clear();

	CSplineInterpolation SplineInterpolation;
	
	if (!IsDirect)	//���»�ȡ���ɹǼܵ�
	{
		//2018.10.24 ������߲���StartHeightCMʱ,�ʹ��м�λ�ÿ�ʼ
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

	cout<<"�������Ѿ�ѡȡ"<<endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints (new pcl::PointCloud<pcl::PointXYZRGB>); 

	//�����ĵ��Ż� //�������ٺ�Ч���� 2018.09.25
	if (IsOptimized)
	{		
		OptimizedCentroidPoints->points.clear();
		//for(int i = 0; i < CentroidPoints->points.size(); i = i + 3)
		OptimizedCentroidPoints->points.push_back(CentroidPoints->points[0]);
		for (int i = 1; i < CentroidPoints->points.size() - 1; i = i + 3)	//ȷ����ʵ�ڵ㲻����ʧ
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

	cout << "���Ƶ��Ѿ�������ɣ�" << endl;

	StemSkeletonSpline.SetSplineInputs(ControlPoints, 3, KnoteVector, false);	
	StemSkeletonSpline.CreateSpline();
	StemSkeletonPoints = StemSkeletonSpline.CurvePoints;	
	ResultStr = "Done";
}

////��ָ����λ�ô����������������������
//void CStemSkeleton::DrawCrossSectionalPlaneByHeight(float Height, int LineWidth, float PointSize, bool ShowProfileCurve,
//	bool DrawUpper)
//{
//	//��ȡ����λ��
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
//	DrawCrossSectionalCurveAtU(UValue, PointSize);	//�ڴ˴���ʾ�������������	
//}

////�ӿ�ʼλ��StartHeight�� �� EndHeight�� �� PlaneNumbers ������� 2017.05.27
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
//		cout<<"���ڻ��� "<<++i<<" �������,��ǰUֵ��"<<UValue<<"������Uֵ��"<<EndUValue<<endl;
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
//			DrawCrossSectionalCurveAtU(UValue, PointSize);	//�ڴ˴���ʾ�������������
//
//		UValue = UValue + UStep;
//	}
//}

////�Ƕȳ����� һ�Ƕ�ֻȡһ����
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

// //2016.04.15  �����������ʵ���
//double CStemSkeleton::GetCrossSectionalProfileAndArea(
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CrossSectionalProfilePtr,
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurve, bool IsArea)
//{
//	double Area = 0;
//
//	//����B�������������
//	ProfileCurve->points.clear();
//
//	CSplineInterpolation SplineInterpolation;
//	SplineInterpolation.SetInputs(CrossSectionalProfilePtr, 3, true);	
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//	vector<double> KnoteVector;		
//	SplineInterpolation.GetControlPointsAndKnotValue(ControlPoints, KnoteVector, true);
//
//	//��������
//	//CRationalSpline RationalSpline;
//	CSpline RationalSpline;
//	RationalSpline.Viewer = Viewer;
//	RationalSpline.FirstPointOfCloseCurve = CrossSectionalProfilePtr->points[0];
//	//��Ҫ��ʾ���ߵķ�͹����
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
//��ȡ����λ��
	pcl::PointXYZRGB StemCrossPlanePoint, StemCrossSectionNormal;
	
	double UValue = StemSkeletonSpline.GetUValueBySplineLength(Height);

	DrawCrossSectionalCurveAtU(UValue, PointSize);
}


//�ڸ���uֵ����ʾ�����
void CStemSkeleton::DrawCrossSectionalCurveAtU(float UValue, float PointSize)
{
	pcl::PointXYZRGB StemCrossPlanePoint, StemCrossSectionNormal;	
	StemCrossPlanePoint = StemSkeletonSpline.GetSplinePoint(UValue);	

	//��ȡ���ɵ���������Ĭ��ȡ��׼������
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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CalcPoints (new pcl::PointCloud<pcl::PointXYZRGB>);					//��ά�ռ��еĵ㼯
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CalcPlanePoints (new pcl::PointCloud<pcl::PointXYZRGB>);				//ƽ��㼯
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CrossSectionalProfilePtr (new pcl::PointCloud<pcl::PointXYZRGB>);	//�����㼯
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurve (new pcl::PointCloud<pcl::PointXYZRGB>);	//�����㼯
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProfileCurve3D (new pcl::PointCloud<pcl::PointXYZRGB>);	//�����㼯

	GeometryBase::GetPointsBetweenTwoPlanes(
		StemCloud,
		StemCrossSectionNormal.x,StemCrossSectionNormal.y,StemCrossSectionNormal.z,
		-(StemCrossPlanePoint.x * StemCrossSectionNormal.x + StemCrossPlanePoint.y * StemCrossSectionNormal.y + StemCrossPlanePoint.z * StemCrossSectionNormal.z),
		StemCrossSectionNormal.x,StemCrossSectionNormal.y,StemCrossSectionNormal.z,
		-(AnotherPoint.x * StemCrossSectionNormal.x + AnotherPoint.y * StemCrossSectionNormal.y + AnotherPoint.z * StemCrossSectionNormal.z),
		CalcPoints, 1, true, false, false);

		if (CalcPoints->points.size() == 0)
		{
			cout << "δ�õ�������������" << endl;
			return;
		}

	//PointBase::PointXYZRGBIndexToPointXYZRGB(TempIndexPoints, CalcPoints);
	//2017.05.28 ע�͵�����
	//PointBase::SetPointColor(TempIndexPoints, ColorBase::BlueColor);
	//PointBase::ShowPointXYZRGB(Viewer, TempIndexPoints, "TempIndexPoints"+ StringBase::ClockValue(), 3);

	//����ȡ�ĵ㼯ת��Ϊƽ��㼯
	CalcBase<double> CalcBaseDouble;

	GeometryBase::RotateNormalToVertical(CalcPoints, CalcPlanePoints, StemCrossSectionNormal);
	
	//��ת���������	
	//�������÷�ʽ���� 2017.05.28
	PointBase::SetPointsCoordinateValue(CalcPlanePoints, "Z", CalcPlanePoints->points[0].z);

	//PointBase::SavePCDToFileName(CalcPlanePoints,
	//	"I:\\3DStemModel\\CalcProfileCurve3D_" + CalcBaseDouble.ConvertToString(CurrentHeightShow) + "_" + StringBase::ClockValue() + ".pcd");

	//2017.05.28 ע�͵�����
	//PointBase::SetPointColor(CalcPlanePoints, ColorBase::GreenColor);
	//PointBase::ShowPointXYZRGB(Viewer, CalcPlanePoints, "CalcPlanePoints"+ StringBase::ClockValue(), 3);

	//2016.04.16 �Ƕȳ����õ�������
	//AngleSample(CalcPlanePoints, CrossSectionalProfilePtr);
	//	
	////�˴�ֻ��ʾ�������������
	//GetCrossSectionalProfileAndArea(CrossSectionalProfilePtr, ProfileCurve, false);	
	//PointBase::SavePCDToFileName(ProfileCurve, 
	//	"I:\\3DStemModel\\ProfileCurve3D_" + CalcBaseDouble.ConvertToString(CurrentHeightShow) +  "_"+ StringBase::ClockValue() + ".pcd");

	GeometryBase::RotateToOriginal(ProfileCurve, ProfileCurve3D, StemCrossSectionNormal);
	//cout<<"��ǰλ�ô���������������"<<StemCrossSectionNormal<<endl;

	PointBase::SetPointColor(ProfileCurve3D, ColorBase::RedColor);
	PointBase::ShowPointXYZRGB(Viewer, ProfileCurve3D, "ProfileCurve3D"+ StringBase::ClockValue(), 2);	
	AllProfilecurvePoints->points.insert(AllProfilecurvePoints->points.begin(),
		ProfileCurve3D->points.begin(), ProfileCurve3D->points.end());
}