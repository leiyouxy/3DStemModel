
#include "HorizontalPartition.h"
#include <limits>
#include "CommClass.h"
#include "CalcGeometryCenter.h"

void CHorizontalPartition::SetThickNess(double ThickNessValue)
{
	SectionThickness = ThickNessValue;
}

void CHorizontalPartition::Initialization()
{
	SectionsVector.clear();

	PointBase::GetPointsMaxAndMin(InputCloud,
		XMax, XMin, YMax, YMin, ZMax, ZMin);

	double ZZoneValue = ZMax - ZMin;

	double ZNumber = 0;
	
	if (abs(ZZoneValue) < SectionThickness)
		ZNumber = 1;
	else
		ZNumber = ZZoneValue / SectionThickness;
	
	SectionsCount = ceil(ZNumber);
}

void CHorizontalPartition::PatitionSection()
{
	Initialization();
	if (SectionThickness <= EPSM6) return;
	int I = 0;
	int Index = -1;
	double TempHeight = 0;

	//cout << "���ڶԵ��ƽ��д�ֱ�ֶ�" << endl;
	MassCenterPointsPtr->points.clear();
	GeometryCenterPointsPtr->points.clear();

	TempHeight = ZMin;

	//�����Ƶ�ÿ��������Ϣ����
	for (I = 0; I < SectionsCount; I++)
	{//���Ƚ���ŷ������Ƶ�ָ���ʼ��
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempSectionPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
		//��ʼ��ÿһ��������Ԫ
		SectionStruct TempSection;
		//cout<<"�ѳ�ʼ��"<<I<<"������"<<endl;
		//TempSection.SectionPtr = TempSectionPtr;
		TempSection.ZMax = TempHeight + SectionThickness;
		TempSection.ZMin = TempHeight;
		TempSection.xCenter = 0;
		TempSection.yCenter = 0;
		TempSection.zCenter = 0;

		SectionsVector.push_back(TempSection);

		TempHeight = TempHeight + SectionThickness;
	}
	//���һ��������ZMaxҪ�ʵ��Ŵ�
	SectionsVector[SectionsVector.size() - 1].ZMax = SectionsVector[SectionsVector.size() - 1].ZMax + 1;

	//find section for each point by its height value
	for (I = 0; I < InputCloud->points.size(); I++)
	{
		Index = FindSectionIndex(InputCloud->points[I].z);

		if (-1 != Index && Index < SectionsCount)
		{				
			//�ҵ�����λ�ú�����뵽��Ӧ�ķ�����
			//SectionsVector[Index].SectionPtr->push_back(TreePoints->points[I]);
			SectionsVector[Index].Indexs.push_back(I);
		}
		else if (Index == SectionsCount)	//Z���ֵ�Ĵ��� 2015.10.16
		{
			//TreePoints->points[I].data[3] = I;
			//SectionsVector[Index - 1].SectionPtr->push_back(TreePoints->points[I]);
			SectionsVector[Index - 1].Indexs.push_back(I);
		}
		else if (Index > SectionsCount || Index < 0)
		{
			cout << "The index of Z value is not find" << InputCloud->points[I].z << endl;
		}
	}


	//2019.07.17 ��������ĵ�̫�٣����Ƴ������ķֶ�
	if (SectionsCount - 2 > 0)
	{
		//�������ƻ�û�����²���Ƹ�����1/5
		if (SectionsVector[SectionsCount - 1].Indexs.size() 
			< SectionsVector[SectionsCount - 2].Indexs.size() / 5)
		{
			SectionsVector.pop_back();
			SectionsCount = SectionsVector.size();
		}
	}

	//����ÿ�����������ĵ�����(�˴�������)
	for (I = 0; I < SectionsCount; I++)
	{
		//cout<<"���ڼ����"<<I<<"������������"<<endl;
		//int SectionPointsCount = SectionsVector[I].SectionPtr->points.size();
		int SectionPointsCount = SectionsVector[I].Indexs.size();

		pcl::PointXYZRGB CenterPoint;

		if (SectionPointsCount > 0)	//�����ǰ�����ĸ�������0����ִ�д˲���
		{
			for (int j = 0; j < SectionPointsCount; j++)
			{
				int CurrentIndex = SectionsVector[I].Indexs[j];
				SectionsVector[I].xCenter = SectionsVector[I].xCenter
					//+ SectionsVector[I].SectionPtr->points[j].x / SectionPointsCount;
					+ InputCloud->points[CurrentIndex].x / SectionPointsCount;
				SectionsVector[I].yCenter = SectionsVector[I].yCenter
					//+ SectionsVector[I].SectionPtr->points[j].y / SectionPointsCount;
					+ InputCloud->points[CurrentIndex].y / SectionPointsCount;
				SectionsVector[I].zCenter = SectionsVector[I].zCenter
					//+ SectionsVector[I].SectionPtr->points[j].z / SectionPointsCount;
					+ InputCloud->points[CurrentIndex].z / SectionPointsCount;
			}

			CenterPoint.x = SectionsVector[I].xCenter;
			CenterPoint.y = SectionsVector[I].yCenter;
			CenterPoint.z = SectionsVector[I].zCenter;
			CenterPoint.r = 255;
			CenterPoint.g = 0;
			CenterPoint.b = 0;
		}
		else	//�����ǰ������ û������  ��ʹ����һ������������
		{
			CenterPoint = MassCenterPointsPtr->points[I - 1];
			//x��y�����겻�䣬��zֵӦ�仯
			CenterPoint.z = SectionsVector[I - 1].zCenter + SectionThickness;
		}
	
		MassCenterPointsPtr->points.push_back(CenterPoint);
		//if (CalcGeometryCenter)
		//{
		//	this->GeometryCenterPointsPtr->points.push_back(CenterPoint);
		//}
	}	
}

//Find Section Index By ZValue. The condition is that The ZValue is equal or greater than the ZMin of the Section, meanwhile, the ZValue is smaller than the  ZMax of the Section 
int CHorizontalPartition::FindSectionIndex(double ZValue)
{
	int Index = -1;
	int TempIndex = (ZValue - ZMin) / SectionThickness;	
	return TempIndex;
}

// Save points from several sections to file
void CHorizontalPartition::SaveSectionSToFile(string FileName, int StartIndex, int EndIndex)
{
	if (StartIndex > SectionsVector.size() || EndIndex < StartIndex || EndIndex < 0)
	{
		//cout << "������ʼ������ڷ������ܸ�����" << endl;
		return;
	}

	if (EndIndex >= SectionsVector.size())
		EndIndex = SectionsVector.size();

	if (StartIndex < 0)
		StartIndex = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = StartIndex; i < EndIndex; i++)
	{
		for (int j = 0; j < SectionsVector[i].Indexs.size(); j++)
		{		
			Temp->points.insert(Temp->points.end(),
				InputCloud->points[SectionsVector[i].Indexs[j]]);
		}
	}
	PointBase::SavePCDToFileName(Temp, FileName);
}

void CHorizontalPartition::SaveSectionSToEachFile(string FileNamePrefix, int StartIndex, int EndIndex)
{
	if (StartIndex > SectionsVector.size() || EndIndex < StartIndex || EndIndex < 0)
	{
		//cout << "������ʼ������ڷ������ܸ�����" << endl;
		return;
	}

	if (EndIndex >= SectionsVector.size())
		EndIndex = SectionsVector.size();

	if (StartIndex < 0)
		StartIndex = 0;		

	#pragma omp parallel for
	for (int i = StartIndex; i < EndIndex; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int j = 0; j < SectionsVector[i].Indexs.size(); j++)
		{
			Temp->points.insert(Temp->points.end(),
				InputCloud->points[SectionsVector[i].Indexs[j]]);
		}
		string TempFileName = FileNamePrefix + "_" + StringBase::FloatToStr(SectionThickness) 
			+ "_" + StringBase::IntToStr(i * SectionThickness) + ".pcd";
		//cout <<"TempFileName"<< TempFileName << endl;
		PointBase::SavePCDToFileName(Temp, TempFileName);
	}	
}

void CHorizontalPartition::ShowSectionPoints(int StartIndex, int EndIndex)
{
	if (StartIndex > SectionsVector.size() || EndIndex < StartIndex || EndIndex < 0)
	{		
		return;
	}

	if (StartIndex == 0 && EndIndex == 0)
	{
		StartIndex = 0, EndIndex = SectionsCount;
	}

	if (EndIndex >= SectionsVector.size())
		EndIndex = SectionsVector.size();

	if (StartIndex < 0)
		StartIndex = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ShowSlicesPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	Viewer->removePointCloud(SlicesPointsStr);

	for (int i = StartIndex; i < EndIndex; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurrentSlicesPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int j = 0; j < SectionsVector[i].Indexs.size(); j++)
		{
			CurrentSlicesPoints->points.insert(CurrentSlicesPoints->points.end(),
				InputCloud->points[SectionsVector[i].Indexs[j]]);
		}
		int Value = i % 3;
		if (Value == 0)
			PointBase::SetPointColor(CurrentSlicesPoints, ColorBase::RedColor);
		else if (Value == 1)
			PointBase::SetPointColor(CurrentSlicesPoints, ColorBase::GreenColor);
		else if (Value == 2)
			PointBase::SetPointColor(CurrentSlicesPoints, ColorBase::BlueColor);

		ShowSlicesPoints->points.insert(ShowSlicesPoints->points.end(),
			CurrentSlicesPoints->points.begin(),
			CurrentSlicesPoints->points.end());
	}	

	if (Viewer != NULL)
		PointBase::ShowPointXYZRGB(Viewer, ShowSlicesPoints, SlicesPointsStr, 2);

	if (p_TreePclQtGui != NULL)
		emitUpdateUI();
}

void CHorizontalPartition::UnShowSectionPoints()
{
	if (Viewer != NULL)
		Viewer->removePointCloud(SlicesPointsStr);
	if (p_TreePclQtGui != NULL)
		emitUpdateUI();
}

CHorizontalPartition::~CHorizontalPartition()
{
	UnShowSectionPoints();
}

//calc Geometry Center Points for each slice using the center gravity of convex polygon  2018.12.31
void CHorizontalPartition::CalcCenterPointsByConvexPolygon(int StartIndex, int EndIndex)
{
	//If the Number is not equal, then regenrate it.
	if (GeometryCenterPointsPtr->points.size() != SectionsCount)
	{
		GeometryCenterPointsPtr->points.clear();
		for (int i = 0; i < SectionsCount; i++)
			GeometryCenterPointsPtr->points.push_back(pcl::PointXYZRGB());
	}

	if (StartIndex == 0 && EndIndex == 0)
	{
		StartIndex = 0, EndIndex = SectionsCount;
	}

	if (StartIndex < 0 || StartIndex >= SectionsCount)
		StartIndex = 0;	

	if (EndIndex == 0)	//only caculate the StartIndex-th section
		EndIndex = StartIndex + 1;

	if (EndIndex < 0 || EndIndex >= SectionsCount)
		EndIndex = SectionsCount;

	for (int i = StartIndex; i < EndIndex; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		GetSectionPoints(i, SectionPoints);

		if (SectionPoints->points.size() > 0)		
			GeometryCenterPointsPtr->points[i] = GeometryBase::GetCentroidOfPoints(SectionPoints);
		else
		{
			if (i - 1 >= 0)
			{
				GeometryCenterPointsPtr->points[i] = GeometryCenterPointsPtr->points[i - 1];
				GeometryCenterPointsPtr->points[i].z = GeometryCenterPointsPtr->points[i].z + SectionThickness;
			}
		}
	}
}

//calc Geometry Center Points for each slice using the Least Squares method 2018.12.31
void CHorizontalPartition::CalcCenterPointsByLeastSquares()
{
	GeometryCenterPointsPtr->points.clear();
	for (int i = 0; i < SectionsCount; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		GetSectionPoints(i, SectionPoints);

		pcl::PointXYZRGB GeometryCenterPoint;
		GeometryBase::CircleFittingByLeastSquaresFitting(SectionPoints, GeometryCenterPoint);

		GeometryCenterPointsPtr->points.push_back(GeometryCenterPoint);
	}
}

void CHorizontalPartition::SetExclusionColor(int ColorValue)
{
	ExclusionColor = ColorValue;
}

//get points of section by SectionIndex 2018.12.31
void CHorizontalPartition::GetSectionPoints(int SectionIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints)
{	
	SectionPoints->points.clear();

	if (SectionIndex < 0 || SectionIndex >= SectionsVector.size()) return;

	for (int j = 0; j < SectionsVector[SectionIndex].Indexs.size(); j++)
	{
		pcl::PointXYZRGB TempPoint = InputCloud->points[SectionsVector[SectionIndex].Indexs[j]];

		//if (TempPoint.rgba == ExclusionColor)
		//�˴���Ӧ�ø�����ɫ�ų��㣬��������Ϊ������
		//if (TempPoint.r == 255 && TempPoint.g == 0 && TempPoint.b == 0)
		//	continue;

		if (abs(TempPoint.x) < EPSM6 && abs(TempPoint.y) < EPSM6  && abs(TempPoint.z) < EPSM6)
			continue;

		SectionPoints->points.push_back(TempPoint);
	}
}

void CHorizontalPartition::SetSectionColors(int StartIndex, int EndIndex, const int ColorValue)
{
	if (StartIndex > SectionsVector.size() || EndIndex < StartIndex || EndIndex < 0)
	{	
		return;
	}

	if (EndIndex >= SectionsVector.size())
		EndIndex = SectionsVector.size();

	if (StartIndex < 0)	StartIndex = 0;	

	for (int i = StartIndex; i < EndIndex; i++)
	{
		for (int j = 0; j < SectionsVector[i].Indexs.size(); j++)
		{
			InputCloud->points[SectionsVector[i].Indexs[j]].rgba = ColorValue;
		}
	}
}

//calc Geometry Center Points for each slice using gravity 2018.12.31
void CHorizontalPartition::CalcCenterPointsByGravity(int StartIndex, int EndIndex)
{
	if (StartIndex < 0 || StartIndex >= SectionsCount)
		StartIndex = 0;		

	if (EndIndex <= 0 || EndIndex >= SectionsCount) 
		EndIndex = SectionsCount;

	//If the Number is not equal, then regenrate it.
	if (GeometryCenterPointsPtr->points.size() != SectionsCount)
	{
		GeometryCenterPointsPtr->points.clear();
		for (int i = 0; i < SectionsCount; i++)
			GeometryCenterPointsPtr->points.push_back(pcl::PointXYZRGB());
	}

	for (int i = StartIndex; i < EndIndex; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		GetSectionPoints(i, SectionPoints);

		pcl::PointXYZRGB GravityCenterPoint;
		
		GravityCenterPoint = GeometryBase::GetGravityOfPoints(SectionPoints);

		//GeometryCenterPointsPtr->points.push_back(GravityCenterPoint);
		GeometryCenterPointsPtr->points[i].x = GravityCenterPoint.x;
		GeometryCenterPointsPtr->points[i].y = GravityCenterPoint.y;
		GeometryCenterPointsPtr->points[i].z = GravityCenterPoint.z;
	}
}


/*
void CHorizontalPartition::SetInputS(
	pcl::PointCloud<PointXYZRGBIndex>::Ptr pUnPatitionPoints,
	double SectionThicknessValue)
{	//���ô�������������Ϣ ���ݷ����ĸ��� �趨��Ϣ	

	UnPatitionPointsPtr = pUnPatitionPoints;

	//��Ҫ�����һ�εķ�����Ϣ���������ĵ�������Ϣ
	SectionsVector.clear();	
	MassCenterPointsPtr->points.clear();
	GeometryCenterPointsPtr->points.clear();

	PointBase::GetPointsMaxAndMin(pUnPatitionPoints, 
			XMax, XMin, YMax, YMin, ZMax, ZMin);

	double ZZoneValue = ZMax - ZMin;
	SectionThickness = SectionThicknessValue;
	
	double ZNumber = ZZoneValue / SectionThickness;	
	
	int ZNumberInt = int(ZNumber);
	if ((ZNumber - ZNumberInt) > 0.01 )	//2014.14.24	ȡ���󻹴���1
		SectionsNumber = ZNumberInt + 1;
	else
		SectionsNumber = ZNumberInt;	
}

void CHorizontalPartition::PatitionSection()	
{	//������Ϣ�Ե��ƽ��з�������
	int I = 0;
	int Index = -1;
	double TempValue = 0;		

	//���ĵ��Ƶ���ʱָ��
	cout<<"���ڶԵ��ƽ��д�ֱ�ֶ�"<<endl;
	MassCenterPointsPtr->points.clear();
	
	TempValue = ZMin;

	//�����Ƶ�ÿ��������Ϣ����
	for ( I = 0; I < SectionsNumber; I++)
	{//���Ƚ���ŷ������Ƶ�ָ���ʼ��
		pcl::PointCloud<PointXYZRGBIndex>::Ptr 
			TempSectionPtr (new pcl::PointCloud<PointXYZRGBIndex>);
		//��ʼ��ÿһ��������Ԫ
		SectionStruct TempSection;		
		//cout<<"�ѳ�ʼ��"<<I<<"������"<<endl;
		TempSection.SectionPtr = TempSectionPtr;	
		TempSection.ZMax = TempValue + SectionThickness;
		TempSection.ZMin = TempValue;
		TempSection.DValue = 0;
		TempSection.xCenter = 0;
		TempSection.yCenter = 0;
		TempSection.zCenter = 0;

		SectionsVector.push_back(TempSection);

		TempValue = TempValue + SectionThickness;
	}	
	//���һ��������ZMaxҪ�ʵ��Ŵ�
	SectionsVector[SectionsVector.size() - 1].ZMax = 
		SectionsVector[SectionsVector.size() - 1].ZMax + 1;

	//���ݵ��Ƶ�������Ϣ������Ӧ�ķ�����Ϣ Ѱ�ҵ������ڵķ��������������ݷŵ�������
	for (I = 0; I < UnPatitionPointsPtr->points.size(); I++)
	{
		Index = FindSectionIndex(UnPatitionPointsPtr->points[I].z);
		
		if (-1 != Index && Index < SectionsNumber)
		{	//�ҵ�����λ�ú�����뵽��Ӧ�ķ�����
			SectionsVector[Index].SectionPtr->push_back(UnPatitionPointsPtr->points[I]);
		}
		else if (Index == SectionsNumber)	//Z���ֵ�Ĵ��� 2015.10.16
		{
			SectionsVector[Index - 1].SectionPtr->push_back(UnPatitionPointsPtr->points[I]);
		}
		else if (Index > SectionsNumber || Index < 0)
		{			
			cout<<"Not Find Z Value "<<UnPatitionPointsPtr->points[I].z <<endl;
		}
	}	
	
	//����ÿ�����������ĵ�����(�˴�������)
	for ( I = 0; I < SectionsNumber; I++)
	{
		//cout<<"���ڼ����"<<I<<"������������"<<endl;
		int SectionPointsCount = SectionsVector[I].SectionPtr->points.size();

		pcl::PointXYZRGB CenterPoint;

		if (SectionPointsCount > 0)	//�����ǰ�����ĸ�������0����ִ�д˲���
		{			
			for(int j = 0; j < SectionPointsCount; j++)
			{
				SectionsVector[I].xCenter = SectionsVector[I].xCenter 
						+ SectionsVector[I].SectionPtr->points[j].x / SectionPointsCount;
				SectionsVector[I].yCenter = SectionsVector[I].yCenter 
						+ SectionsVector[I].SectionPtr->points[j].y / SectionPointsCount;
				SectionsVector[I].zCenter = SectionsVector[I].zCenter 
						+ SectionsVector[I].SectionPtr->points[j].z / SectionPointsCount;			
			}
			
			//SectionsVector[I].DValue = CalcSectionDValue(SectionsVector[I]);
			//SectionsVector[I].PointToMassCenterVariance = 0;

			CenterPoint.x = SectionsVector[I].xCenter;
			CenterPoint.y = SectionsVector[I].yCenter;
			CenterPoint.z = SectionsVector[I].zCenter;
			CenterPoint.r = 255;
			CenterPoint.g = 0;
			CenterPoint.b = 0;
		}
		else	//�����ǰ������ û������  ��ʹ����һ������������
		{
			CenterPoint = MassCenterPointsPtr->points[I - 1];
			//x��y�����겻�䣬��zֵӦ�仯
			CenterPoint.z = SectionsVector[I - 1].zCenter + this->SectionThickness;
		}
			
		//�����ĵ����
		MassCenterPointsPtr->points.push_back(CenterPoint);	
		//if (CalcGeometryCenter)
		//{
		//	this->GeometryCenterPointsPtr->points.push_back(CenterPoint);
		//}
	}		
	//if (CalcGeometryCenter)
	//{
	//	this->CalcCentroidPoints();
	//}
}

int CHorizontalPartition::FindSectionIndex(double ZValue)
{//���� ZValue ��ֵ ��ȡ ZValue ���ڵ� SectionPoint ��Index �Ǵ��ڵ�����Сֵ��С�����ֵ
	int Index = -1;

	int TempIndex = (ZValue - ZMin) / SectionThickness;
	//int TempIndex = ZValue / SectionThickness;
	return TempIndex;
}

//���ݷ������ĵ���Ƴ�ʼ���������ĵ���
void CHorizontalPartition::InitialGeometryPoint()
{
	GeometryCenterPointsPtr->points.clear();
	for(int i = 0; i < MassCenterPointsPtr->points.size(); i++)
	{
		pcl::PointXYZRGB TempPoint;
		TempPoint = MassCenterPointsPtr->points[i];
		GeometryCenterPointsPtr->points.push_back(TempPoint);
	}
}

//double CHorizontalPartition::CalcSectionDValue(SectionStruct SectionValue)
//{
//	double DValue = 0;
//	double X = 0;
//	double Y = 0;
//	double Z = 0;
//	int Count = SectionValue.SectionPtr->points.size();
//
//	CalcBase<double> BaseCalcInstance;
//
//	for  (int i = 0; i < Count; i++)
//	{
//		DValue 	= DValue + PointDis(SectionValue.SectionPtr->points[i].x,
//			SectionValue.SectionPtr->points[i].y,SectionValue.SectionPtr->points[i].z,
//			SectionValue.xCenter,SectionValue.yCenter,SectionValue.zCenter) / Count;
//	}
//	return DValue * 2;
//}

//void CHorizontalPartition::SaveCenterPCDToDB(SectionStruct Section,
//		string ConnStr, string TableName, int Index, bool ExistsCreate)
//{	
//	string SQLStr;
//	
//	CalcBase<int> BaseCalcIntInstance;
//	CalcBase<double> BaseCalcFloatInstance;
//	
//	PointDB::SavePCDToDB(Section.SectionPtr, ConnStr,
//			TableName + "_Section_" + BaseCalcIntInstance.ConvertToString(Index), ExistsCreate);
//
//	SQLStr = "Insert Into Tree_Section(TreeName, SectionIndex, SectionZMax, SectionZMin, ";
//	SQLStr = SQLStr + "SectionDiameter, Used, xCenter, yCenter, zCenter)Values('";
//	SQLStr = SQLStr +  TableName + "'," + BaseCalcIntInstance.ConvertToString(Index) + ","; 
//	SQLStr = SQLStr + BaseCalcFloatInstance.ConvertToString(Section.ZMax) + ",";
//	SQLStr = SQLStr + BaseCalcFloatInstance.ConvertToString(Section.ZMin ) + ",";
//	SQLStr = SQLStr + BaseCalcFloatInstance.ConvertToString(Section.DValue) + ",";
//	SQLStr = SQLStr + "1,";
//	SQLStr = SQLStr + BaseCalcFloatInstance.ConvertToString(Section.xCenter ) + ",";
//	SQLStr = SQLStr + BaseCalcFloatInstance.ConvertToString(Section.yCenter) + ",";
//	SQLStr = SQLStr + BaseCalcFloatInstance.ConvertToString(Section.zCenter) + ")";
//
//	CAccessDBHelper AccessDBHelper = CAccessDBHelper(ConnStr); 
//
//	AccessDBHelper.ExecuteSQL(SQLStr);
//	
//}


//����ÿ�������ļ������ģ�Sampled==true�Ǹ��ݳ����������� Ĭ�� ÿ���Ƕ�ֵ15�ȣ������ĵ������5��
void CHorizontalPartition::CalcGeometryCenterPoints(int StartIndex, int EndIndex, bool Sampled,
	double AngleValue, int AngleNumer)
{
	if (StartIndex < 0 || StartIndex > SectionsVector.size())  
	{
		cout<<"����������ĵ�Ŀ�ʼ��������ֵ����"<<endl;
		return;
	}
	if (EndIndex < 0 || EndIndex > SectionsVector.size())  
	{
		cout<<"����������ĵ�Ľ�����������ֵ����"<<endl;
		return;
	}
	
	if (StartIndex == EndIndex == 0)
	{
		StartIndex = 0;
		EndIndex = SectionsVector.size();
	}

	//���û���ȳ�ʼ��
	if (GeometryCenterPointsPtr->points.size() == 0 
		|| GeometryCenterPointsPtr->points.size() < MassCenterPointsPtr->points.size())
	{
		GeometryCenterPointsPtr->points.clear();
		for (int i = 0; i < SectionsVector.size(); i++)
		{
			GeometryCenterPointsPtr->points.push_back(this->MassCenterPointsPtr->points[i]);
		}
	}	

	if (!Sampled)
	{	
		cout<<"���ڲ������е�ķ�ʽ���㼸������"<<endl;
		for (int i = StartIndex; i < EndIndex; i++)
		{		
			CalcGeometryCenterPoint(i);
			
			//ƽ��
			//MovePointsToBaseCenter(GeometryCenterPointsPtr->points[0],
			//	GeometryCenterPointsPtr->points[i], i);
		}	
	}
	else
	{
		cout<<"���ڲ��ó�����ʽ���㼸������"<<endl;
		CCalcGeometryCenter<pcl::PointXYZRGB>  CalcGeometryCenter;
		for(int i = StartIndex; i < EndIndex; i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp (new pcl::PointCloud<pcl::PointXYZRGB>);
			PointBase::PointXYZRGBIndexToPointXYZRGB(SectionsVector[i].SectionPtr, Temp);
			CalcGeometryCenter.SetInputs(Temp, AngleNumer);

			double Radius, RadiusVariance;
			pcl::PointXYZRGB TempPoint;
			if ( i == 0)
				TempPoint = GeometryCenterPointsPtr->points[0];
			else
				TempPoint = GeometryCenterPointsPtr->points[i - 1];
					
			if (!CalcGeometryCenter.CalcGeometryCenter(TempPoint, AngleValue, Radius, RadiusVariance))
			{	//����������ʽû���ҵ����ĵ�����
				TempPoint.x = SectionsVector[i].xCenter;
				TempPoint.y = SectionsVector[i].yCenter;
			}

			GeometryCenterPointsPtr->points[i].z = (SectionsVector[i].ZMax
				+ SectionsVector[i].ZMin) / 2;			
			GeometryCenterPointsPtr->points[i].x = TempPoint.x;
			GeometryCenterPointsPtr->points[i].y = TempPoint.y;	

			//MovePointsToBaseCenter(GeometryCenterPointsPtr->points[0],
			//	GeometryCenterPointsPtr->points[i], i);
		}
	}	
}

void CHorizontalPartition::CalcCentroidPoints(int StartIndex, int EndIndex, bool Sampled,
	double AngleValue, int AngleNumer)
{	//����ÿ������������	
	if (StartIndex < 0 || StartIndex > SectionsVector.size())  
	{
		cout<<"����������ĵ�Ŀ�ʼ��������ֵ����"<<endl;
		return;
	}
	if (EndIndex < 0 || EndIndex > SectionsVector.size())  
	{
		cout<<"����������ĵ�Ľ�����������ֵ����"<<endl;
		return;
	}
	
	if (StartIndex == 0)	
		StartIndex = 0;	
	if (EndIndex == 0)
		EndIndex = SectionsVector.size();

	//���û���ȳ�ʼ��
	if (GeometryCenterPointsPtr->points.size() == 0 
		|| GeometryCenterPointsPtr->points.size() < MassCenterPointsPtr->points.size())
	{
		GeometryCenterPointsPtr->points.clear();
		for (int i = 0; i < SectionsVector.size(); i++)
		{
			GeometryCenterPointsPtr->points.push_back(this->MassCenterPointsPtr->points[i]);
		}
	}	

	cout<<"���ڲ������е�ķ�ʽ��������"<<endl;
	for (int i = StartIndex; i < EndIndex; i++)
	{	
		//if (i / 10 == 0)
		//cout<<"���ڴ����"<<i<<"�����������ĵ�"<<endl;

		if (SectionsVector[i].SectionPtr->points.size() < 2) 
		{			
			if (i-1 >= 0)	//2016.07.17 ����
			{
				GeometryCenterPointsPtr->points[i].x = GeometryCenterPointsPtr->points[i-1].x;
				GeometryCenterPointsPtr->points[i].y = GeometryCenterPointsPtr->points[i-1].y;
				GeometryCenterPointsPtr->points[i].z = (SectionsVector[i].ZMax + SectionsVector[i].ZMin)/2;
			}
			else
			{
				
			}
		}
		else
		{
			//2016.01.18 ����
			pcl::PointXYZRGB Centroid =	GeometryBase::GetCentroidOfPoints(SectionsVector[i].SectionPtr);
		
			GeometryCenterPointsPtr->points[i].x = Centroid.x;
			GeometryCenterPointsPtr->points[i].y = Centroid.y;
			GeometryCenterPointsPtr->points[i].z = (SectionsVector[i].ZMax + SectionsVector[i].ZMin)/2;
		}
	}	
}
	
///*
//����ĳ�������ļ�������
//2014.09.23 ��xyƽ��̽����ǰ����������λ�ã�Step Ϊ������
/////�˴���Ҫ���������  2015.10.22
void CHorizontalPartition::CalcGeometryCenterPoint(int Index, double Step)
{
	//���Ȼ�ȡ��ǰ�����ģ�Ȼ����㵱ǰ�ļ������ģ�����ֵ 
	//�����ǰ�������̫�٣��ᵼ�³����޷����㼸�����ĵ��������ʱʹ����һ�����XY�����ĵ�
	//2014.11.24 ͬʱ��ȡ��������󷽲�ֵ
	//������һ�������ļ������Ŀ�ʼ	

	//cout<<"���ڼ����"<<Index<<"�������ļ�������"<<endl;

	//if (Index == 3855) 
	//{
	//	cout<<endl;
	//}

	//2016.01.18 ��Ϊ��������
	double CenterX = 0;
	double CenterY = 0;
	if (Index - 1 > 0)
	{	//�Ѿ���ȡ��һ�������ļ�������
		CenterX = this->GeometryCenterPointsPtr->points[Index - 1].x;
		CenterY = this->GeometryCenterPointsPtr->points[Index - 1].y;
	}
	else
	{	//��û����һ�������ļ������� �������ĵ����
		CenterX = this->MassCenterPointsPtr->points[Index].x;
		CenterY = this->MassCenterPointsPtr->points[Index].y;
	}
	
	double PriorAvgRadius = 0;
	double LastVariance = 0;		
	{	
		int MaxExecutions = 0;

		//2014.10.23 �������ƽ������С
		cout << "���ڼ��㵱ǰ���������ĵ㣺" << Index << endl;
		ExploreSectionCenter(this->SectionsVector[Index], CenterX,
			CenterY, PriorAvgRadius, Step, LastVariance, MaxExecutions);	

		////2015.01.18 ������С���˷����Բ 		
		//if (Index == 213)	//����ʹ�����������������Ϊ�����ϸ��ȡԲ�������㷨����������ʵ������Ǻܷ���
		//{			cout<<Index<<endl;	}
		//2015.05.14 ������Ч�� ���ַ�ʽ�ǲ��е� 
		//2015.05.17 ���� ��С���˷���ϵķ�ʽҲ���ԣ�����Ҫ�Ѳ��ɹ��������׼����
		//ExploreSectionCenterByLeastSquaresFitting(
		//	TempSectionStruct, CenterX, CenterY, PriorAvgRadius, LastVariance);

		if (PriorAvgRadius == 0)
		{
			LastVariance = 0;
			//if (Index > 0)
			//{				
			//	PriorAvgRadius = SectionsVector[Index - 1].DValue / 2.0;					
			//}
			//else
			//	PriorAvgRadius = SectionsVector[Index].DValue / 2.0;
		}

		//if (200 == MaxExecutions)	
		if (200 == MaxExecutions)	
			//ִ����200�Σ�˵���Ѿ��Ҳ������������ĵ㣬ֻ������һ�������ļ���ִ��
			//2014.10.21
		{
			LastVariance = 0;
			if (Index > 0)
			{				
				CenterX = GeometryCenterPointsPtr->points[Index-1].x;
				CenterY = GeometryCenterPointsPtr->points[Index-1].y;
				//PriorAvgRadius = SectionsVector[Index - 1].DValue / 2.0;
			}
			else
			{
				CenterX = GeometryCenterPointsPtr->points[Index].x;
				CenterY = GeometryCenterPointsPtr->points[Index].y;
				//PriorAvgRadius = SectionsVector[Index].DValue / 2.0;			
			}
		}

		this->GeometryCenterPointsPtr->points[Index].x = CenterX;
		this->GeometryCenterPointsPtr->points[Index].y = CenterY;
		//SectionsVector[Index].DValue = PriorAvgRadius * 2;		
	}			
}
//*/

/*

//2014.09.22 ��xyƽ��̽����ǰ����������λ�ã�Step Ϊ������ѡ��ĳһ��������һֱǰ��
void CHorizontalPartition::ExploreSectionCenter(SectionStruct & SectionValue, 
	double & CenterX, double & CenterY, double & AvgRadius,
	double Step, double & LastVariance,
	int & MaxExecutions, string Axis)
{
	//�ȴ�X�����Ὺʼ 
	CalcBase<double> CalcBasefloat;
	vector<double> vectorfloat;
	double MeanValue;	

	if (MaxExecutions == 200)
	{
		return;
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
		for (int i = 0; i < SectionValue.SectionPtr->points.size(); i++)
		{
			vectorfloat.push_back(PointDis(CenterX, CenterY,
				SectionValue.SectionPtr->points[i].x,
				SectionValue.SectionPtr->points[i].y));

			vectorfloatXPlus.push_back(PointDis(CenterX + Step, CenterY,
				SectionValue.SectionPtr->points[i].x,
				SectionValue.SectionPtr->points[i].y));

			vectorfloatXSub.push_back(PointDis(CenterX - Step, CenterY,
				SectionValue.SectionPtr->points[i].x,
				SectionValue.SectionPtr->points[i].y));

			vectorfloatYPlus.push_back(CalcBasefloat.CalcDis2D(CenterX , CenterY + Step,
				SectionValue.SectionPtr->points[i].x,
				SectionValue.SectionPtr->points[i].y));

			vectorfloatYSub.push_back(CalcBasefloat.CalcDis2D(CenterX , CenterY - Step,
				SectionValue.SectionPtr->points[i].x,
				SectionValue.SectionPtr->points[i].y));
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
			CenterX = CenterX + Step;
			AvgRadius = MeanValueXPlus;			
			LastVariance = VarianceValueXPlus;

			ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueXPlus,
					Step, LastVariance, 
					MaxExecutions, "X+"); //X����� �ݹ� ��ǰ�ƶ� 
			
			if (VarianceValue > VarianceValueYPlus)		//��Y�������ƶ�
			{
				CenterY = CenterY + Step;
				AvgRadius = MeanValueYPlus;				
				LastVariance = VarianceValueYPlus;

				ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueYPlus,
					Step, LastVariance, MaxExecutions, "Y+");	
			}
			else if (VarianceValue > VarianceValueYSub) //��Y�Ḻ���ƶ�
			{
				CenterY = CenterY - Step;
				AvgRadius = MeanValueYSub;	
				LastVariance = VarianceValueYSub;

				ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueYSub,
					Step, LastVariance, MaxExecutions, "Y-");			
			}
		}
		else if (VarianceValue > VarianceValueXSub)		//��X�Ḻ���ƶ�		
		{
			CenterX = CenterX - Step;
			AvgRadius = MeanValueXSub;			
			LastVariance = VarianceValueXSub;

			ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueXSub, 
				Step, LastVariance, MaxExecutions, "X-");		
			
			if (VarianceValue > VarianceValueYPlus)		//��Y�������ƶ�
			{
				CenterY = CenterY + Step;
				AvgRadius = MeanValueYPlus;	
				LastVariance = VarianceValueYPlus;
				
				ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueYPlus,
					Step, LastVariance, MaxExecutions, "Y+");	
			}
			else if (VarianceValue > VarianceValueYSub) //��Y�Ḻ���ƶ�
			{
				CenterY = CenterY - Step;
				AvgRadius = MeanValueYSub;	
				LastVariance = VarianceValueYSub;

				ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueYSub,
					Step, LastVariance, MaxExecutions, "Y-");			
			}
		}	
	}
	else if ("X+" == Axis)
	{
		double VarianceValue = 0;		

		vectorfloat.clear();
		for (int i = 0; i < SectionValue.SectionPtr->points.size(); i++)
		{
			vectorfloat.push_back(CalcBasefloat.CalcDis2D(CenterX + Step, CenterY,
				SectionValue.SectionPtr->points[i].x,
				SectionValue.SectionPtr->points[i].y));
		}
		VarianceValue = CalcBasefloat.CalcVariances(vectorfloat, MeanValue);
		if (VarianceValue >= LastVariance)	//���ĵ� �Ѿ� ���� x�������λ��
		{
			return;
		}
		else	//���ĵ� �Ѿ� ���� x�������λ��
		{
			CenterX = CenterX + Step;
			AvgRadius = MeanValue;		
			LastVariance = VarianceValue;

			ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValue,
				Step, LastVariance, MaxExecutions, "X+");
		}
	}
	else if ("X-" == Axis)
	{
		double VarianceValue = 0;

		vectorfloat.clear();
		for (int i = 0; i < SectionValue.SectionPtr->points.size(); i++)
		{
			vectorfloat.push_back(CalcBasefloat.CalcDis2D(CenterX - Step, CenterY,
				SectionValue.SectionPtr->points[i].x,
				SectionValue.SectionPtr->points[i].y));
		}
		VarianceValue = CalcBasefloat.CalcVariances(vectorfloat, MeanValue);
		if (VarianceValue >= LastVariance)	//���ĵ� �Ѿ� ���� x�������λ��
		{
			return;
		}
		else	//���ĵ� �Ѿ� ���� x�������λ��
		{
			CenterX = CenterX - Step;
			AvgRadius = MeanValue;			
			LastVariance = VarianceValue;

			ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValue,
				Step, LastVariance, MaxExecutions, "X-");
		}
	}	 
	//������Y������
	else if ("Y+" == Axis)
	{
		double VarianceValue = 0;
		vectorfloat.clear();
		for (int i = 0; i < SectionValue.SectionPtr->points.size(); i++)
		{
			vectorfloat.push_back(CalcBasefloat.CalcDis2D(CenterX , CenterY + Step,
				SectionValue.SectionPtr->points[i].x,
				SectionValue.SectionPtr->points[i].y));
		}
		VarianceValue = CalcBasefloat.CalcVariances(vectorfloat, MeanValue);
		if (VarianceValue >= LastVariance)	//���ĵ� �Ѿ� ���� x�������λ��
		{
			return;
		}
		else	//���ĵ� �Ѿ� ���� x�������λ��
		{
			CenterY = CenterY + Step;
			AvgRadius = MeanValue;	
			LastVariance = VarianceValue;

			ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValue,
				Step, LastVariance, MaxExecutions, "Y+");
		}
	}
	else if ("Y-" == Axis)
	{
		double VarianceValue = 0;
		vectorfloat.clear();
		for (int i = 0; i < SectionValue.SectionPtr->points.size(); i++)
		{
			vectorfloat.push_back(CalcBasefloat.CalcDis2D(CenterX, CenterY - Step,
				SectionValue.SectionPtr->points[i].x,
				SectionValue.SectionPtr->points[i].y));
		}
		VarianceValue = CalcBasefloat.CalcVariances(vectorfloat, MeanValue);
		
		if (VarianceValue >= LastVariance)	//���ĵ� �Ѿ� ���� x�������λ��
		{			
			return;
		}
		else	//���ĵ� �Ѿ� ���� x�������λ��
		{
			CenterY = CenterY - Step;
			AvgRadius = MeanValue;	
			LastVariance = VarianceValue;

			ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValue,
				Step, VarianceValue, MaxExecutions, "Y-");
		}
	}	 
}

//����ǰ����Index�����ĵ� CurrentCenter �ƶ��� BaseCenter�� 
//ͬʱ�ƶ�����������������ĵ��Ƽ����ĵ㡣
void CHorizontalPartition::MovePointsToBaseCenter
	(pcl::PointXYZRGB NewCenter, pcl::PointXYZRGB CurrentCenter, int Index,
	bool SourcePointMoved, bool Up)
{
	if (Up)	//�ϲ��ĵ����ƶ���
	{
		for(int i = Index; i < SectionsVector.size(); i++)
		{			
			//cout<<"�����ƶ���"<<i<<"�����������ɵ���"<<endl; //�ƶ���������
			PointsMove(SectionsVector[i].SectionPtr,
				NewCenter.x - CurrentCenter.x,
				NewCenter.y - CurrentCenter.y, 0);

			//////�ƶ��������ƶ�Ӧ�����ɵ���		
			//if (SourcePointMoved)
			//{
			//	for(int j = 0; j < SectionsVector[i].SectionPtr->points.size(); j++)
			//	{
			//		PointsMove(
			//			UnPatitionPointsPtr->points[SectionsVector[i].SectionPtr->points[j].Index],
			//			NewCenter.x - CurrentCenter.x,
			//			NewCenter.y - CurrentCenter.y, 0);
			//	}
			//}

			////cout<<"�����ƶ���"<<i<<"�����ĵ���"<<endl;
			////�ƶ����ĵ���Ƽ����ĵ����	//˵�����ĵ㳤�ȴ��ڵ�ǰ��������
			//if (MassCenterPointsPtr->points.size() > i)
			//	PointsMove(MassCenterPointsPtr->points,
			//		NewCenter.x - CurrentCenter.x,
			//		NewCenter.y - CurrentCenter.y, 0);
			//else
			//	cout<<"����"<<i<<endl;

			//if (GeometryCenterPointsPtr->points.size() > i)
			//	PointsMove(GeometryCenterPointsPtr->points[i],
			//		NewCenter.x - CurrentCenter.x,
			//		NewCenter.y - CurrentCenter.y, 0);
		}
	}
	else	//�²��ĵ����ƶ���		
	{
		for(int i = Index; i >= 0; i--)
		{			
			//cout<<"�����ƶ���"<<i<<"�����������ɵ���"<<endl; //�ƶ���������
			PointsMove(SectionsVector[i].SectionPtr,
				NewCenter.x - CurrentCenter.x,
				NewCenter.y - CurrentCenter.y, 0);

			////�ƶ��������ƶ�Ӧ�����ɵ���		
			//if (SourcePointMoved)
			//{
			//	for(int j = Index; j >= 0; j--)
			//	{
			//		PointsMove(
			//			UnPatitionPointsPtr->points[SectionsVector[i].SectionPtr->points[j].Index],
			//			NewCenter.x - CurrentCenter.x,
			//			NewCenter.y - CurrentCenter.y, 0);
			//	}
			//}

			//cout<<"�����ƶ���"<<i<<"�����ĵ���"<<endl;
			//�ƶ����ĵ���Ƽ����ĵ����	
			//PointsMove(MassCenterPointsPtr->points[i],
			//		NewCenter.x - CurrentCenter.x,
			//		NewCenter.y - CurrentCenter.y, 0);			

			//PointsMove(GeometryCenterPointsPtr->points[i],
			//		NewCenter.x - CurrentCenter.x,
			//		NewCenter.y - CurrentCenter.y, 0);
		}	
	}
}


	//��ʾ����
void CHorizontalPartition::ShowSectionsPoints(
	boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
	int StartIndex, int EndIndex, bool ShowLabel, string PointIDStr, 
	double Size, bool PointColorChanged, 
	int Colorr, int Colorg, int Colorb)
{
	if (EndIndex == 0)
	{
		EndIndex = SectionsVector.size() - 1;
	}

	pcl::PointCloud<PointXYZRGBIndex>::Ptr 
			TempShowPoints (new pcl::PointCloud<PointXYZRGBIndex>);
	if ( StartIndex > SectionsVector.size() ||  EndIndex > SectionsVector.size() )
	{
		cout<<"������ʼ������ڷ������ܸ�����"<<endl;

		EndIndex = SectionsVector.size() - 1;
	}

	for(int i = StartIndex; i < EndIndex; i++)
	{
		TempShowPoints->points.insert(TempShowPoints->points.end(),
			SectionsVector[i].SectionPtr->points.begin(),
			SectionsVector[i].SectionPtr->points.end());
		CalcBase<int> CalcBaseInt;

		if (GeometryCenterPointsPtr != NULL)
			if (ShowLabel && (GeometryCenterPointsPtr->points.size() > i))
			{
				Viewer->addText3D(CalcBaseInt.ConvertToString(i),
					GeometryCenterPointsPtr->points[i], Size, Colorr, Colorg, Colorb,
					"Lbl" + PointIDStr + CalcBaseInt.ConvertToString(i));
			}
	}
	
	if (PointColorChanged)
		PointBase::SetPointColor(TempShowPoints, Colorr, Colorg, Colorb);
	PointBase::ShowPointXYZRGB(Viewer, TempShowPoints, PointIDStr + "ShowSectionsPoints", 1); 
	cout<<"��ʾ������ɣ�"<<endl;
}

//��ȡ�ض������ĵ�������
void CHorizontalPartition::GetSectionsPoint(pcl::PointCloud<PointXYZRGBIndex>::Ptr 
	SectionSPoints,int StartIndex, int EndIndex)
{
	SectionSPoints->points.clear();
	for(int i = StartIndex; i < EndIndex; i++)
	{
		SectionSPoints->points.insert(SectionSPoints->points.end(),
			SectionsVector[i].SectionPtr->points.begin(),
			SectionsVector[i].SectionPtr->points.end());		
	}
}

//���沿������ĵ��Ƶ��ļ���
void CHorizontalPartition::SaveSectionSToFile(string FileName, int StartIndex, int EndIndex)
{
	if (StartIndex > SectionsVector.size() || EndIndex < StartIndex || EndIndex < 0)
	{
		cout<<"������ʼ������ڷ������ܸ�����"<<endl;
		return;
	}
	
	if (EndIndex >= SectionsVector.size())
		EndIndex = SectionsVector.size();

	if (StartIndex < 0 )
		StartIndex = 0;

	pcl::PointCloud<PointXYZRGBIndex>::Ptr Temp (new pcl::PointCloud<PointXYZRGBIndex>);

	for(int i = StartIndex; i < EndIndex; i++)
	{
		Temp->points.insert(Temp->points.end(),
			SectionsVector[i].SectionPtr->points.begin(),
			SectionsVector[i].SectionPtr->points.end());
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	PointBase::PointXYZRGBIndexToPointXYZRGB(Temp, TempRGB);
	PointBase::SavePCDToFileName(TempRGB, FileName);
}

//�������ɳ�������λ�õ����� 2015.10.22 
int CHorizontalPartition::GetStemLengthIndexBySectionCenter(double StemLength)
{
	if (GeometryCenterPointsPtr->points.size() == 0)
		CalcCentroidPoints(0, SectionsVector.size() - 1);
	
	double CurrentLength = 0;
	int ReturnIndex = -1;

	CalcBase<double> CalcBaseFloat;

	for(int i = 1; i < SectionsVector.size(); i++)
	{
		CurrentLength = CurrentLength + CalcDis(
			GeometryCenterPointsPtr->points[i].x,
			GeometryCenterPointsPtr->points[i].y,
			GeometryCenterPointsPtr->points[i].z,
			GeometryCenterPointsPtr->points[i-1].x,
			GeometryCenterPointsPtr->points[i-1].y,
			GeometryCenterPointsPtr->points[i-1].z);
		if (CurrentLength >= StemLength)
		{
			ReturnIndex = i;
			break;
		}
	}

	return ReturnIndex;
}
//*/

////2016.01.16 ��ȡ�����ı߽��   �߽��Ķ��岻�ö��壬 ����׼ȷ������
//void CHorizontalPartition::GetBoundPoints(int SectionIndex,
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BoundPoints,
//	bool IsUpper)
//{
//	BoundPoints->points.clear();
//	if (SectionIndex < 0 && SectionIndex > SectionsVector.size())
//		return;
//	vector<int> BoundPointIndexs;
//	//���Ȼ�ȡZֵ����Сֵ����Сֵ��Ӧ���������߽����Դ˴���ʼѰ��
//	float ZMin, ZMax;
//	int ZMinIndex, ZMaxIndex;
//	if (SectionsVector[SectionIndex].SectionPtr->points.size() > 1)
//	{
//		ZMin = SectionsVector[SectionIndex].SectionPtr->points[0].z;
//		ZMax = ZMin;
//		ZMinIndex = 0;
//		ZMaxIndex = 0;
//	}
//
//	for(int i = 1; i < SectionsVector[SectionIndex].SectionPtr->points.size(); i++)
//	{
//		if(ZMin > SectionsVector[SectionIndex].SectionPtr->points[i].z)
//		{
//			ZMin = SectionsVector[SectionIndex].SectionPtr->points[i].z;
//			ZMinIndex = i;
//		}
//		if(ZMax < SectionsVector[SectionIndex].SectionPtr->points[i].z)
//		{
//			ZMax = SectionsVector[SectionIndex].SectionPtr->points[i].z;
//			ZMaxIndex = i;
//		}
//	}
//	//�����㼯�İ˲���
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreePoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//	PointBase::PointXYZRGBIndexToPointXYZRGB(SectionsVector[SectionIndex].SectionPtr,
//		TreePoints);
//	
//	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> * Octree;	
//	Octree = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));	//���׼�
//	Octree->setInputCloud(TreePoints);
//	Octree->addPointsFromInputCloud();	
//
//	vector<int> NeighbourIndexS;
//	vector<float> NeighbourDisS;
//
//	int PriorPointIndex; 
//	int NextPointIndex = -1;
//	int EndIndex;	
//
//	if (IsUpper) //��ȡ�Ͻ��	
//	{	
//		PriorPointIndex = ZMaxIndex;
//		EndIndex = ZMaxIndex;
//	}
//	else
//	{	
//		PriorPointIndex = ZMinIndex;
//		EndIndex = ZMinIndex;
//	}
//
//	BoundPointIndexs.push_back(PriorPointIndex);
//	//BoundPoints->points.push_back(TreePoints->points[PriorPointIndex]);
//
//	int LastSize, Size, Count = 0;
//	LastSize = 0, Size = 1; 
//	while(LastSize != Size)	//��û�е���β�غϵ�ʱ��
//	{
//		LastSize = Size;
//		NeighbourIndexS.clear();
//		Octree->radiusSearch(PriorPointIndex, 2, NeighbourIndexS, NeighbourDisS); //��ȡ��һ����Ľ��ڵ�
//		
//		//2015.01.15 ����ʱ������뾶��ȡ�����Լ���������Ӵ˲���		
//		if (VectorBase::FindIndexInVector(NeighbourIndexS, PriorPointIndex) == -1)
//		{
//			NeighbourIndexS.push_back(PriorPointIndex);	
//			NeighbourDisS.push_back(0);	
//		}		 
//
//		//Ҫ���վ����NeighbourIndexS��������
//		for(int i = 0; i < NeighbourDisS.size(); i++)
//		{
//			for(int j = i+1; j < NeighbourDisS.size(); j++)
//			{
//				if (NeighbourDisS[i] > NeighbourDisS[j])
//				{
//					swap(NeighbourDisS[i], NeighbourDisS[j]);
//					swap(NeighbourIndexS[i], NeighbourIndexS[j]);
//				}
//			}
//		}
//
//		//for(int i = 0; i < NeighbourIndexS.size(); i++)
//		//{
//		//	if (PriorPointIndex != NeighbourIndexS[i])
//		//	{
//		vector<int> NextIndexs;
//		NextIndexs = GetNextBoundPoint(TreePoints, PriorPointIndex, NeighbourIndexS, IsUpper); 
//		
//		for(int i = 0 ; i < NextIndexs.size(); i++)
//		{
//			VectorBase::InsertIndexToVector(BoundPointIndexs, NextIndexs[i]);
//		}
//		
//		//��һ��������Զ�ĵ�
//		if (NextIndexs.size() > 0) NextPointIndex = NextIndexs[NextIndexs.size() - 1];
//		else break;
//			
//		PriorPointIndex = NextPointIndex;
//		Size = BoundPointIndexs.size();
//		Count++;
//		if (Count == 3)
//			cout<<endl;
//	}
//	for(int i = 0; i < NeighbourIndexS.size(); i++)
//	{
//		BoundPoints->points.push_back(TreePoints->points[NeighbourIndexS[i]]);	
//	}
//}

////�ڵ㼯�л�ȡ��һ���߽��(�п�����������)���Ͻ�����½�  2016.01.16 �㷨δ��ȫʵ�֣� ��Ҫ���巽������һ������ǰ��
//vector<int> CHorizontalPartition::GetNextBoundPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
//	int BoundIndex, vector<int> NeighbourIndex, bool IsUpper)
//{	
//	vector<int> NextBoundS; 
//	int NextBoundIndex = -1;
//	for(int i = 0; i < NeighbourIndex.size(); i++)
//	{
//		//cout<<"I��"<<i<<endl;
//		bool Find = true;
//		int CurrentIndex = NeighbourIndex[i];
//		if (BoundIndex == CurrentIndex)
//			continue;
//		//�ж� ��BoundIndex �� CurrentIndex �Ƿ��Ǳ߽��
//		for(int j = 0 ; j < NeighbourIndex.size(); j++)
//		{
//			//cout<<"J"<<j<<endl;
//			if (BoundIndex != NeighbourIndex[j] && CurrentIndex != NeighbourIndex[j])	
//			{
//				//��ʱ��Ҫѡ��������� ȷ��һ��ƽ�棬Ȼ��������ͶӰ�����ƽ���ϣ���ȷ����ϵ
//				pcl::PointXYZRGB A = Points->points[BoundIndex];
//				pcl::PointXYZRGB B = Points->points[CurrentIndex];
//
//				if (A.z == B.z)	//��������ͬһ���߶ȾͲ�����
//					continue;
//
//				pcl::PointXYZRGB D, Other;
//				int AIndex, BIndex;
//				if (IsUpper) //�Ͻ� D��λ�� �� AIndex λ��һ���� ����BIndex��
//				{
//					if (A.z > B.z)	
//						D.x = B.x, D.y = B.y, D.z = A.z, AIndex = BoundIndex, BIndex = CurrentIndex;	
//					else	
//						D.x = A.x, D.y = A.y, D.z = B.z, AIndex = CurrentIndex, BIndex = BoundIndex;		
//				}
//				else	//�½� D��λ�� �� AIndex λ��һ���� ����BIndex��
//				{
//					if (A.z > B.z)	
//						D.x = A.x, D.y = A.y, D.z = B.z, AIndex = CurrentIndex, BIndex = BoundIndex;	
//					else	
//						D.x = B.x, D.y = B.y, D.z = A.z, AIndex = BoundIndex, BIndex = CurrentIndex;	
//				}
//				
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr NoramlPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//				NoramlPoints->points.push_back(A);
//				NoramlPoints->points.push_back(B);
//				NoramlPoints->points.push_back(D);
//				pcl::PointXYZRGB NormalPoint;
//
//				//����㼯
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr NeighbourPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//				for(int k = 0; k < NeighbourIndex.size(); k++)
//				{					
//					NeighbourPoints->points.push_back(Points->points[NeighbourIndex[k]]);
//				}
//				//������D				
//				NeighbourPoints->points.push_back(D);
//				int IndexA = VectorBase::FindIndexInVector(NeighbourIndex, AIndex);
//				int IndexB = VectorBase::FindIndexInVector(NeighbourIndex, BIndex);
//				int IndexD = NeighbourPoints->points.size() - 1;
//
//				//������㼯 ͶӰ�� ABDȷ����ƽ���ϣ�
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProjectPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//				GeometryBase::GetPointsTangentPlane(NoramlPoints, NormalPoint);
//				GeometryBase::ProjectPointsToPlane(NeighbourPoints, ProjectPoints,
//					NormalPoint.x, NormalPoint.y,NormalPoint.z,0);
//				//�ٽ���ת����XYƽ��
//				GeometryBase::RotateNormalToVertical(ProjectPoints, NeighbourPoints, NormalPoint); 				
//
//				//��ת����ƽ�漯��ʱ�� ��ת�����棬�Ӷ�����λ��Ҳ�õ���
//				// A��D��ͬһ�߶� ��B��
//				if ((NeighbourPoints->points[IndexD].x == NeighbourPoints->points[IndexB].x
//					&& NeighbourPoints->points[IndexD].y > NeighbourPoints->points[IndexB].y
//					&& NeighbourPoints->points[IndexD].y == NeighbourPoints->points[IndexA].y)
//
//					||(NeighbourPoints->points[IndexD].y == NeighbourPoints->points[IndexB].y
//					&& NeighbourPoints->points[IndexD].x > NeighbourPoints->points[IndexB].x
//					&& NeighbourPoints->points[IndexD].x == NeighbourPoints->points[IndexA].x))
//				{
//					//���½��ʱ�� B��λ�� Ӧ�ñ�A��D��
//					if (!IsUpper) IsUpper = true;
//				}// A��D��ͬһ�߶� ��B��
//				else if((NeighbourPoints->points[IndexD].x == NeighbourPoints->points[IndexB].x
//					&& NeighbourPoints->points[IndexD].y < NeighbourPoints->points[IndexB].y
//					&& NeighbourPoints->points[IndexD].y == NeighbourPoints->points[IndexA].y)
//
//					||(NeighbourPoints->points[IndexD].y == NeighbourPoints->points[IndexB].y
//					&& NeighbourPoints->points[IndexD].x < NeighbourPoints->points[IndexB].x
//					&& NeighbourPoints->points[IndexD].x == NeighbourPoints->points[IndexA].x))
//				{
//					if (IsUpper) IsUpper = false;	
//				}
//
//				//������Ǳ�Ե��
//				if (!LineIsBorder(NeighbourPoints, IndexA, IndexB, IndexD, IsUpper))
//				{
//					Find = false;
//					break;
//				}
//			}
//		}
//		if (Find)
//		{
//			NextBoundS.push_back(CurrentIndex);
//			//NextBoundIndex = CurrentIndex;
//			//break;
//		}
//	}
//	return NextBoundS;
//}
//
////��A B �Ƿ���ƽ��㼯�ı߽�
//
/////IsUpper = true ���߶�LinePointA LinePointD �����Ϸ��Ƿ��е�
/////IsUpper = false ���߶�LinePointA LinePointD �����·��Ƿ��е�
//bool CHorizontalPartition::LineIsBorder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints,
//	int LinePointA, int LinePointB, int LinePointD, bool IsUpper)
//{//
//	bool IsBounder = true;
//	pcl::PointXYZRGB A = PlanePoints->points[LinePointA];
//	pcl::PointXYZRGB B = PlanePoints->points[LinePointB];
//	pcl::PointXYZRGB D = PlanePoints->points[LinePointD];
//	
//	for(int i = 0; i < PlanePoints->points.size(); i++)
//	{
//			
//		if (i == LinePointA || i == LinePointB || i == LinePointD)
//			continue;
//		//��������������ڲ�����
//		pcl::PointXYZRGB Current = PlanePoints->points[i];
//		if (GeometryBase::PointIsInTriange(A, B, D, Current) == 1)
//		{
//			IsBounder = false;
//			break;
//		}
//		if (IsUpper)
//		{
//			if (D.y == A.y)		//�����Y�����
//			{
//				if (PlanePoints->points[i].y > D.y)
//				{
//					//���� ��Current,D,A �ļн�
//					double Angle1 = GeometryBase::AngleOfTwoVector(
//						Current.x - D.x, Current.y - D.y, Current.z - D.z,
//						A.x - D.x, A.y - D.y, A.z - D.z);
//
//					//���� ��Current,A,D�ļн�
//					double Angle2 = GeometryBase::AngleOfTwoVector(
//						Current.x - A.x, Current.y - A.y, Current.z - A.z,
//						D.x - A.x, D.y - A.y, D.z - A.z);
//
//					if (Angle1 < M_PI/2 && Angle2 < M_PI/2)
//					{
//						IsBounder = false;
//						break;					
//					}
//				}
//			}
//			else if(D.x == A.x)
//			{
//				if (PlanePoints->points[i].x > D.x)
//				{
//					//���� ��Current,D,A �ļн�
//					double Angle1 = GeometryBase::AngleOfTwoVector(
//						Current.x - D.x, Current.y - D.y, Current.z - D.z,
//						A.x - D.x, A.y - D.y, A.z - D.z);
//
//					//���� ��Current,A,D�ļн�
//					double Angle2 = GeometryBase::AngleOfTwoVector(
//						Current.x - A.x, Current.y - A.y, Current.z - A.z,
//						D.x - A.x, D.y - A.y, D.z - A.z);
//
//					if (Angle1 < M_PI/2 && Angle2 < M_PI/2)
//					{
//						IsBounder = false;
//						break;					
//					}
//				}
//			}
//		}
//		else	//�½�
//		{
//			if (D.y == A.y)		//�����Y�����
//			{
//				if (PlanePoints->points[i].y < D.y)
//				{
//					//���� ��Current,D,A �ļн�
//					double Angle1 = GeometryBase::AngleOfTwoVector(
//						Current.x - D.x, Current.y - D.y, Current.z - D.z,
//						A.x - D.x, A.y - D.y, A.z - D.z);
//
//					//���� ��Current,A,D�ļн�
//					double Angle2 = GeometryBase::AngleOfTwoVector(
//						Current.x - A.x, Current.y - A.y, Current.z - A.z,
//						D.x - A.x, D.y - A.y, D.z - A.z);
//
//					if (Angle1 < M_PI/2 && Angle2 < M_PI/2)
//					{
//						IsBounder = false;
//						break;					
//					}
//				}
//			}
//			else if(D.x == A.x)
//			{
//				if (PlanePoints->points[i].x < D.x)
//				{
//					//���� ��Current,D,A �ļн�
//					double Angle1 = GeometryBase::AngleOfTwoVector(
//						Current.x - D.x, Current.y - D.y, Current.z - D.z,
//						A.x - D.x, A.y - D.y, A.z - D.z);
//
//					//���� ��Current,A,D�ļн�
//					double Angle2 = GeometryBase::AngleOfTwoVector(
//						Current.x - A.x, Current.y - A.y, Current.z - A.z,
//						D.x - A.x, D.y - A.y, D.z - A.z);
//
//					if (Angle1 < M_PI/2 && Angle2 < M_PI/2)
//					{
//						IsBounder = false;
//						break;					
//					}
//				}
//			}			
//			
//		}
//	}	
//	//if (IsBounder)
//	//	PointBase::SavePCDToFileName(PlanePoints, "ProcessedPoints\\BounderTrue.pcd");
//	//else
//	//	PointBase::SavePCDToFileName(PlanePoints, "ProcessedPoints\\BounderFalse.pcd");
//	return IsBounder;
//}
/*
//2016.01.21 �ƶ������ʵ���ʾλ��
void CHorizontalPartition::GetShowPosition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointsPtr)
{
	pcl::PointCloud<PointXYZRGBIndex>::Ptr 
		PointsIndexPtr (new pcl::PointCloud<PointXYZRGBIndex>);	
	PointBase::PointMoveToOrigin(PointsPtr);
	PointBase::PointXYZRGBToPointXYZRGBIndex(PointsPtr, PointsIndexPtr);
	SetInputS(PointsIndexPtr, 0.5);
	PatitionSection();
	double XMax,XMin,YMax,YMin,ZMax,ZMin;
	PointBase::GetPointsMaxAndMin(SectionsVector[0].SectionPtr, 
		XMax,XMin,YMax,YMin,ZMax,ZMin);  
	//HorizontalPartition.CalcCentroidPoints();
	PointsMove(PointsPtr, 
		PointsPtr->points[0].x - XMin, PointsPtr->points[0].y - YMin, -ZMin);	
}

//2016.01.21 �ƶ������ʵ���ʾλ��
void CHorizontalPartition::GetShowPosition(pcl::PointCloud<PointXYZRGBIndex>::Ptr PointsIndexPtr)
{
	PointBase::PointMoveToOrigin(PointsIndexPtr);
	SetInputS(PointsIndexPtr, 0.5);
	PatitionSection();
	double XMax,XMin,YMax,YMin,ZMax,ZMin;
	PointBase::GetPointsMaxAndMin(SectionsVector[0].SectionPtr, 
		XMax,XMin,YMax,YMin,ZMax,ZMin);  
	//HorizontalPartition.CalcCentroidPoints();
	PointsMove(PointsIndexPtr, 
		PointsIndexPtr->points[0].x - XMin, PointsIndexPtr->points[0].y - YMin, -ZMin);	
}
*/