
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

	//cout << "正在对点云进行垂直分段" << endl;
	MassCenterPointsPtr->points.clear();
	GeometryCenterPointsPtr->points.clear();

	TempHeight = ZMin;

	//将点云的每个分区信息存入
	for (I = 0; I < SectionsCount; I++)
	{//首先将存放分区点云的指针初始化
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempSectionPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
		//初始化每一个分区单元
		SectionStruct TempSection;
		//cout<<"已初始化"<<I<<"个分区"<<endl;
		//TempSection.SectionPtr = TempSectionPtr;
		TempSection.ZMax = TempHeight + SectionThickness;
		TempSection.ZMin = TempHeight;
		TempSection.xCenter = 0;
		TempSection.yCenter = 0;
		TempSection.zCenter = 0;

		SectionsVector.push_back(TempSection);

		TempHeight = TempHeight + SectionThickness;
	}
	//最后一个分区的ZMax要适当放大
	SectionsVector[SectionsVector.size() - 1].ZMax = SectionsVector[SectionsVector.size() - 1].ZMax + 1;

	//find section for each point by its height value
	for (I = 0; I < InputCloud->points.size(); I++)
	{
		Index = FindSectionIndex(InputCloud->points[I].z);

		if (-1 != Index && Index < SectionsCount)
		{				
			//找到索引位置后将其放入到对应的分区中
			//SectionsVector[Index].SectionPtr->push_back(TreePoints->points[I]);
			SectionsVector[Index].Indexs.push_back(I);
		}
		else if (Index == SectionsCount)	//Z最大值的处理 2015.10.16
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


	//2019.07.17 如果顶部的点太少，就移除顶部的分段
	if (SectionsCount - 2 > 0)
	{
		//顶部点云还没有其下层点云个数的1/5
		if (SectionsVector[SectionsCount - 1].Indexs.size() 
			< SectionsVector[SectionsCount - 2].Indexs.size() / 5)
		{
			SectionsVector.pop_back();
			SectionsCount = SectionsVector.size();
		}
	}

	//计算每个分区的中心点数据(此处是质心)
	for (I = 0; I < SectionsCount; I++)
	{
		//cout<<"正在计算第"<<I<<"个分区的质心"<<endl;
		//int SectionPointsCount = SectionsVector[I].SectionPtr->points.size();
		int SectionPointsCount = SectionsVector[I].Indexs.size();

		pcl::PointXYZRGB CenterPoint;

		if (SectionPointsCount > 0)	//如果当前分区的个数大于0，则执行此操作
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
		else	//如果当前点云区 没有数据  则使用上一个分区的数据
		{
			CenterPoint = MassCenterPointsPtr->points[I - 1];
			//x与y的坐标不变，但z值应变化
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
		//cout << "分区起始区间大于分区的总个数！" << endl;
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
		//cout << "分区起始区间大于分区的总个数！" << endl;
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
		//此处不应该根据颜色排除点，否则不能作为公共类
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
{	//设置待分区的数据信息 根据分区的个数 设定信息	

	UnPatitionPointsPtr = pUnPatitionPoints;

	//需要清空上一次的分区信息及分区中心点数据信息
	SectionsVector.clear();	
	MassCenterPointsPtr->points.clear();
	GeometryCenterPointsPtr->points.clear();

	PointBase::GetPointsMaxAndMin(pUnPatitionPoints, 
			XMax, XMin, YMax, YMin, ZMax, ZMin);

	double ZZoneValue = ZMax - ZMin;
	SectionThickness = SectionThicknessValue;
	
	double ZNumber = ZZoneValue / SectionThickness;	
	
	int ZNumberInt = int(ZNumber);
	if ((ZNumber - ZNumberInt) > 0.01 )	//2014.14.24	取整后还大于1
		SectionsNumber = ZNumberInt + 1;
	else
		SectionsNumber = ZNumberInt;	
}

void CHorizontalPartition::PatitionSection()	
{	//根据信息对点云进行分区处理
	int I = 0;
	int Index = -1;
	double TempValue = 0;		

	//中心点云的临时指针
	cout<<"正在对点云进行垂直分段"<<endl;
	MassCenterPointsPtr->points.clear();
	
	TempValue = ZMin;

	//将点云的每个分区信息存入
	for ( I = 0; I < SectionsNumber; I++)
	{//首先将存放分区点云的指针初始化
		pcl::PointCloud<PointXYZRGBIndex>::Ptr 
			TempSectionPtr (new pcl::PointCloud<PointXYZRGBIndex>);
		//初始化每一个分区单元
		SectionStruct TempSection;		
		//cout<<"已初始化"<<I<<"个分区"<<endl;
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
	//最后一个分区的ZMax要适当放大
	SectionsVector[SectionsVector.size() - 1].ZMax = 
		SectionsVector[SectionsVector.size() - 1].ZMax + 1;

	//根据点云的数据信息，及对应的分区信息 寻找点云所在的分区，将点云数据放到分区中
	for (I = 0; I < UnPatitionPointsPtr->points.size(); I++)
	{
		Index = FindSectionIndex(UnPatitionPointsPtr->points[I].z);
		
		if (-1 != Index && Index < SectionsNumber)
		{	//找到索引位置后将其放入到对应的分区中
			SectionsVector[Index].SectionPtr->push_back(UnPatitionPointsPtr->points[I]);
		}
		else if (Index == SectionsNumber)	//Z最大值的处理 2015.10.16
		{
			SectionsVector[Index - 1].SectionPtr->push_back(UnPatitionPointsPtr->points[I]);
		}
		else if (Index > SectionsNumber || Index < 0)
		{			
			cout<<"Not Find Z Value "<<UnPatitionPointsPtr->points[I].z <<endl;
		}
	}	
	
	//计算每个分区的中心点数据(此处是质心)
	for ( I = 0; I < SectionsNumber; I++)
	{
		//cout<<"正在计算第"<<I<<"个分区的质心"<<endl;
		int SectionPointsCount = SectionsVector[I].SectionPtr->points.size();

		pcl::PointXYZRGB CenterPoint;

		if (SectionPointsCount > 0)	//如果当前分区的个数大于0，则执行此操作
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
		else	//如果当前点云区 没有数据  则使用上一个分区的数据
		{
			CenterPoint = MassCenterPointsPtr->points[I - 1];
			//x与y的坐标不变，但z值应变化
			CenterPoint.z = SectionsVector[I - 1].zCenter + this->SectionThickness;
		}
			
		//将中心点存入
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
{//根据 ZValue 的值 获取 ZValue 所在的 SectionPoint 的Index 是大于等于最小值，小于最大值
	int Index = -1;

	int TempIndex = (ZValue - ZMin) / SectionThickness;
	//int TempIndex = ZValue / SectionThickness;
	return TempIndex;
}

//根据分区重心点点云初始化几何中心点云
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


//计算每个分区的几何中心，Sampled==true是根据抽样规则计算的 默认 每个角度值15度，抽样的点个数是5个
void CHorizontalPartition::CalcGeometryCenterPoints(int StartIndex, int EndIndex, bool Sampled,
	double AngleValue, int AngleNumer)
{
	if (StartIndex < 0 || StartIndex > SectionsVector.size())  
	{
		cout<<"计算分区中心点的开始分区索引值有误！"<<endl;
		return;
	}
	if (EndIndex < 0 || EndIndex > SectionsVector.size())  
	{
		cout<<"计算分区中心点的结束分区索引值有误！"<<endl;
		return;
	}
	
	if (StartIndex == EndIndex == 0)
	{
		StartIndex = 0;
		EndIndex = SectionsVector.size();
	}

	//如果没有先初始化
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
		cout<<"正在采用所有点的方式计算几何中心"<<endl;
		for (int i = StartIndex; i < EndIndex; i++)
		{		
			CalcGeometryCenterPoint(i);
			
			//平移
			//MovePointsToBaseCenter(GeometryCenterPointsPtr->points[0],
			//	GeometryCenterPointsPtr->points[i], i);
		}	
	}
	else
	{
		cout<<"正在采用抽样方式计算几何中心"<<endl;
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
			{	//根据正常方式没有找到中心点数据
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
{	//计算每个分区的质心	
	if (StartIndex < 0 || StartIndex > SectionsVector.size())  
	{
		cout<<"计算分区中心点的开始分区索引值有误！"<<endl;
		return;
	}
	if (EndIndex < 0 || EndIndex > SectionsVector.size())  
	{
		cout<<"计算分区中心点的结束分区索引值有误！"<<endl;
		return;
	}
	
	if (StartIndex == 0)	
		StartIndex = 0;	
	if (EndIndex == 0)
		EndIndex = SectionsVector.size();

	//如果没有先初始化
	if (GeometryCenterPointsPtr->points.size() == 0 
		|| GeometryCenterPointsPtr->points.size() < MassCenterPointsPtr->points.size())
	{
		GeometryCenterPointsPtr->points.clear();
		for (int i = 0; i < SectionsVector.size(); i++)
		{
			GeometryCenterPointsPtr->points.push_back(this->MassCenterPointsPtr->points[i]);
		}
	}	

	cout<<"正在采用所有点的方式计算质心"<<endl;
	for (int i = StartIndex; i < EndIndex; i++)
	{	
		//if (i / 10 == 0)
		//cout<<"正在处理第"<<i<<"个分区的质心点"<<endl;

		if (SectionsVector[i].SectionPtr->points.size() < 2) 
		{			
			if (i-1 >= 0)	//2016.07.17 调整
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
			//2016.01.18 调整
			pcl::PointXYZRGB Centroid =	GeometryBase::GetCentroidOfPoints(SectionsVector[i].SectionPtr);
		
			GeometryCenterPointsPtr->points[i].x = Centroid.x;
			GeometryCenterPointsPtr->points[i].y = Centroid.y;
			GeometryCenterPointsPtr->points[i].z = (SectionsVector[i].ZMax + SectionsVector[i].ZMin)/2;
		}
	}	
}
	
///*
//计算某个分区的几何中心
//2014.09.23 在xy平面探索当前分区的中心位置，Step 为步长，
/////此处需要调整下最好  2015.10.22
void CHorizontalPartition::CalcGeometryCenterPoint(int Index, double Step)
{
	//首先获取当前的质心，然后计算当前的几何中心，并赋值 
	//如果当前区域点数太少，会导致出现无法计算几何中心的情况，此时使用上一个层的XY的中心点
	//2014.11.24 同时获取分区的最大方差值
	//采样上一个分区的几何中心开始	

	//cout<<"正在计算第"<<Index<<"个分区的几何中心"<<endl;

	//if (Index == 3855) 
	//{
	//	cout<<endl;
	//}

	//2016.01.18 改为计算质心
	double CenterX = 0;
	double CenterY = 0;
	if (Index - 1 > 0)
	{	//已经获取上一个分区的几何中心
		CenterX = this->GeometryCenterPointsPtr->points[Index - 1].x;
		CenterY = this->GeometryCenterPointsPtr->points[Index - 1].y;
	}
	else
	{	//还没有上一个分区的几何中心 就用质心点迭代
		CenterX = this->MassCenterPointsPtr->points[Index].x;
		CenterY = this->MassCenterPointsPtr->points[Index].y;
	}
	
	double PriorAvgRadius = 0;
	double LastVariance = 0;		
	{	
		int MaxExecutions = 0;

		//2014.10.23 采用离差平方和最小
		cout << "正在计算当前分区的中心点：" << Index << endl;
		ExploreSectionCenter(this->SectionsVector[Index], CenterX,
			CenterY, PriorAvgRadius, Step, LastVariance, MaxExecutions);	

		////2015.01.18 采用最小二乘法拟合圆 		
		//if (Index == 213)	//不能使用这个方法来做，因为它是严格获取圆参数的算法，对树的真实情况不是很符合
		//{			cout<<Index<<endl;	}
		//2015.05.14 测试下效果 这种方式是不行的 
		//2015.05.17 修正 最小二乘法拟合的方式也可以，但需要把不成功的区域标准出来
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
			//执行了200次，说明已经找不到分区的中心点，只好以上一个分区的继续执行
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

//2014.09.22 在xy平面探索当前分区的中心位置，Step 为步长，选定某一个坐标轴一直前进
void CHorizontalPartition::ExploreSectionCenter(SectionStruct & SectionValue, 
	double & CenterX, double & CenterY, double & AvgRadius,
	double Step, double & LastVariance,
	int & MaxExecutions, string Axis)
{
	//先从X坐标轴开始 
	CalcBase<double> CalcBasefloat;
	vector<double> vectorfloat;
	double MeanValue;	

	if (MaxExecutions == 200)
	{
		return;
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
		
		if (VarianceValue > VarianceValueXPlus)			//向X轴正向移动		
		{
			CenterX = CenterX + Step;
			AvgRadius = MeanValueXPlus;			
			LastVariance = VarianceValueXPlus;

			ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueXPlus,
					Step, LastVariance, 
					MaxExecutions, "X+"); //X轴继续 递归 向前移动 
			
			if (VarianceValue > VarianceValueYPlus)		//向Y轴正向移动
			{
				CenterY = CenterY + Step;
				AvgRadius = MeanValueYPlus;				
				LastVariance = VarianceValueYPlus;

				ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueYPlus,
					Step, LastVariance, MaxExecutions, "Y+");	
			}
			else if (VarianceValue > VarianceValueYSub) //向Y轴负向移动
			{
				CenterY = CenterY - Step;
				AvgRadius = MeanValueYSub;	
				LastVariance = VarianceValueYSub;

				ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueYSub,
					Step, LastVariance, MaxExecutions, "Y-");			
			}
		}
		else if (VarianceValue > VarianceValueXSub)		//向X轴负向移动		
		{
			CenterX = CenterX - Step;
			AvgRadius = MeanValueXSub;			
			LastVariance = VarianceValueXSub;

			ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueXSub, 
				Step, LastVariance, MaxExecutions, "X-");		
			
			if (VarianceValue > VarianceValueYPlus)		//向Y轴正向移动
			{
				CenterY = CenterY + Step;
				AvgRadius = MeanValueYPlus;	
				LastVariance = VarianceValueYPlus;
				
				ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValueYPlus,
					Step, LastVariance, MaxExecutions, "Y+");	
			}
			else if (VarianceValue > VarianceValueYSub) //向Y轴负向移动
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
		if (VarianceValue >= LastVariance)	//中心点 已经 到了 x轴的中心位置
		{
			return;
		}
		else	//中心点 已经 到了 x轴的中心位置
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
		if (VarianceValue >= LastVariance)	//中心点 已经 到了 x轴的中心位置
		{
			return;
		}
		else	//中心点 已经 到了 x轴的中心位置
		{
			CenterX = CenterX - Step;
			AvgRadius = MeanValue;			
			LastVariance = VarianceValue;

			ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValue,
				Step, LastVariance, MaxExecutions, "X-");
		}
	}	 
	//下面是Y轴的情况
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
		if (VarianceValue >= LastVariance)	//中心点 已经 到了 x轴的中心位置
		{
			return;
		}
		else	//中心点 已经 到了 x轴的中心位置
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
		
		if (VarianceValue >= LastVariance)	//中心点 已经 到了 x轴的中心位置
		{			
			return;
		}
		else	//中心点 已经 到了 x轴的中心位置
		{
			CenterY = CenterY - Step;
			AvgRadius = MeanValue;	
			LastVariance = VarianceValue;

			ExploreSectionCenter(SectionValue,CenterX, CenterY, MeanValue,
				Step, VarianceValue, MaxExecutions, "Y-");
		}
	}	 
}

//将当前分区Index的中心点 CurrentCenter 移动到 BaseCenter， 
//同时移动本分区及后面分区的点云及中心点。
void CHorizontalPartition::MovePointsToBaseCenter
	(pcl::PointXYZRGB NewCenter, pcl::PointXYZRGB CurrentCenter, int Index,
	bool SourcePointMoved, bool Up)
{
	if (Up)	//上部的点云移动，
	{
		for(int i = Index; i < SectionsVector.size(); i++)
		{			
			//cout<<"正在移动第"<<i<<"个分区的树干点云"<<endl; //移动分区点云
			PointsMove(SectionsVector[i].SectionPtr,
				NewCenter.x - CurrentCenter.x,
				NewCenter.y - CurrentCenter.y, 0);

			//////移动分区点云对应的树干点云		
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

			////cout<<"正在移动第"<<i<<"个重心点云"<<endl;
			////移动中心点点云及重心点点云	//说明中心点长度大于当前分区长度
			//if (MassCenterPointsPtr->points.size() > i)
			//	PointsMove(MassCenterPointsPtr->points,
			//		NewCenter.x - CurrentCenter.x,
			//		NewCenter.y - CurrentCenter.y, 0);
			//else
			//	cout<<"出错"<<i<<endl;

			//if (GeometryCenterPointsPtr->points.size() > i)
			//	PointsMove(GeometryCenterPointsPtr->points[i],
			//		NewCenter.x - CurrentCenter.x,
			//		NewCenter.y - CurrentCenter.y, 0);
		}
	}
	else	//下部的点云移动，		
	{
		for(int i = Index; i >= 0; i--)
		{			
			//cout<<"正在移动第"<<i<<"个分区的树干点云"<<endl; //移动分区点云
			PointsMove(SectionsVector[i].SectionPtr,
				NewCenter.x - CurrentCenter.x,
				NewCenter.y - CurrentCenter.y, 0);

			////移动分区点云对应的树干点云		
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

			//cout<<"正在移动第"<<i<<"个重心点云"<<endl;
			//移动中心点点云及重心点点云	
			//PointsMove(MassCenterPointsPtr->points[i],
			//		NewCenter.x - CurrentCenter.x,
			//		NewCenter.y - CurrentCenter.y, 0);			

			//PointsMove(GeometryCenterPointsPtr->points[i],
			//		NewCenter.x - CurrentCenter.x,
			//		NewCenter.y - CurrentCenter.y, 0);
		}	
	}
}


	//显示点云
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
		cout<<"分区起始区间大于分区的总个数！"<<endl;

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
	cout<<"显示点云完成！"<<endl;
}

//获取特定分区的点云数据
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

//保存部分区域的点云到文件中
void CHorizontalPartition::SaveSectionSToFile(string FileName, int StartIndex, int EndIndex)
{
	if (StartIndex > SectionsVector.size() || EndIndex < StartIndex || EndIndex < 0)
	{
		cout<<"分区起始区间大于分区的总个数！"<<endl;
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

//根据树干长度所在位置的索引 2015.10.22 
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

////2016.01.16 获取分区的边界点   边界点的定义不好定义， 难以准确描述，
//void CHorizontalPartition::GetBoundPoints(int SectionIndex,
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BoundPoints,
//	bool IsUpper)
//{
//	BoundPoints->points.clear();
//	if (SectionIndex < 0 && SectionIndex > SectionsVector.size())
//		return;
//	vector<int> BoundPointIndexs;
//	//首先获取Z值的最小值与最小值对应的索引，边界点可以此处开始寻找
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
//	//构建点集的八叉树
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreePoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//	PointBase::PointXYZRGBIndexToPointXYZRGB(SectionsVector[SectionIndex].SectionPtr,
//		TreePoints);
//	
//	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> * Octree;	
//	Octree = (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(0.1f));	//毫米级
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
//	if (IsUpper) //获取上界点	
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
//	while(LastSize != Size)	//还没有到首尾重合的时候
//	{
//		LastSize = Size;
//		NeighbourIndexS.clear();
//		Octree->radiusSearch(PriorPointIndex, 2, NeighbourIndexS, NeighbourDisS); //获取上一个点的近邻点
//		
//		//2015.01.15 因有时候邻域半径获取不到自己，所以添加此部分		
//		if (VectorBase::FindIndexInVector(NeighbourIndexS, PriorPointIndex) == -1)
//		{
//			NeighbourIndexS.push_back(PriorPointIndex);	
//			NeighbourDisS.push_back(0);	
//		}		 
//
//		//要按照距离对NeighbourIndexS升序排序，
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
//		//下一个点是最远的点
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

////在点集中获取下一个边界点(有可能是两个点)，上界或者下界  2016.01.16 算法未完全实现， 需要定义方向，沿着一个方向前进
//vector<int> CHorizontalPartition::GetNextBoundPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
//	int BoundIndex, vector<int> NeighbourIndex, bool IsUpper)
//{	
//	vector<int> NextBoundS; 
//	int NextBoundIndex = -1;
//	for(int i = 0; i < NeighbourIndex.size(); i++)
//	{
//		//cout<<"I："<<i<<endl;
//		bool Find = true;
//		int CurrentIndex = NeighbourIndex[i];
//		if (BoundIndex == CurrentIndex)
//			continue;
//		//判断 点BoundIndex 与 CurrentIndex 是否是边界边
//		for(int j = 0 ; j < NeighbourIndex.size(); j++)
//		{
//			//cout<<"J"<<j<<endl;
//			if (BoundIndex != NeighbourIndex[j] && CurrentIndex != NeighbourIndex[j])	
//			{
//				//此时需要选择第三个点 确定一个平面，然后将其它点投影到这个平面上，再确定关系
//				pcl::PointXYZRGB A = Points->points[BoundIndex];
//				pcl::PointXYZRGB B = Points->points[CurrentIndex];
//
//				if (A.z == B.z)	//两个点在同一个高度就不考虑
//					continue;
//
//				pcl::PointXYZRGB D, Other;
//				int AIndex, BIndex;
//				if (IsUpper) //上界 D的位置 与 AIndex 位置一样高 都比BIndex高
//				{
//					if (A.z > B.z)	
//						D.x = B.x, D.y = B.y, D.z = A.z, AIndex = BoundIndex, BIndex = CurrentIndex;	
//					else	
//						D.x = A.x, D.y = A.y, D.z = B.z, AIndex = CurrentIndex, BIndex = BoundIndex;		
//				}
//				else	//下界 D的位置 与 AIndex 位置一样高 都比BIndex低
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
//				//邻域点集
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr NeighbourPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//				for(int k = 0; k < NeighbourIndex.size(); k++)
//				{					
//					NeighbourPoints->points.push_back(Points->points[NeighbourIndex[k]]);
//				}
//				//最后加入D				
//				NeighbourPoints->points.push_back(D);
//				int IndexA = VectorBase::FindIndexInVector(NeighbourIndex, AIndex);
//				int IndexB = VectorBase::FindIndexInVector(NeighbourIndex, BIndex);
//				int IndexD = NeighbourPoints->points.size() - 1;
//
//				//将邻域点集 投影到 ABD确定的平面上，
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProjectPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
//				GeometryBase::GetPointsTangentPlane(NoramlPoints, NormalPoint);
//				GeometryBase::ProjectPointsToPlane(NeighbourPoints, ProjectPoints,
//					NormalPoint.x, NormalPoint.y,NormalPoint.z,0);
//				//再将其转换到XY平面
//				GeometryBase::RotateNormalToVertical(ProjectPoints, NeighbourPoints, NormalPoint); 				
//
//				//因转换到平面集的时候 会转到反面，从而上下位置也得调整
//				// A与D在同一高度 比B高
//				if ((NeighbourPoints->points[IndexD].x == NeighbourPoints->points[IndexB].x
//					&& NeighbourPoints->points[IndexD].y > NeighbourPoints->points[IndexB].y
//					&& NeighbourPoints->points[IndexD].y == NeighbourPoints->points[IndexA].y)
//
//					||(NeighbourPoints->points[IndexD].y == NeighbourPoints->points[IndexB].y
//					&& NeighbourPoints->points[IndexD].x > NeighbourPoints->points[IndexB].x
//					&& NeighbourPoints->points[IndexD].x == NeighbourPoints->points[IndexA].x))
//				{
//					//求下界的时候 B的位置 应该比A与D高
//					if (!IsUpper) IsUpper = true;
//				}// A与D在同一高度 比B低
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
//				//如果不是边缘边
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
////边A B 是否是平面点集的边界
//
/////IsUpper = true 即线段LinePointA LinePointD 的正上方是否有点
/////IsUpper = false 即线段LinePointA LinePointD 的正下方是否有点
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
//		//如果落在三角形内部，则
//		pcl::PointXYZRGB Current = PlanePoints->points[i];
//		if (GeometryBase::PointIsInTriange(A, B, D, Current) == 1)
//		{
//			IsBounder = false;
//			break;
//		}
//		if (IsUpper)
//		{
//			if (D.y == A.y)		//如果在Y上相等
//			{
//				if (PlanePoints->points[i].y > D.y)
//				{
//					//计算 点Current,D,A 的夹角
//					double Angle1 = GeometryBase::AngleOfTwoVector(
//						Current.x - D.x, Current.y - D.y, Current.z - D.z,
//						A.x - D.x, A.y - D.y, A.z - D.z);
//
//					//计算 点Current,A,D的夹角
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
//					//计算 点Current,D,A 的夹角
//					double Angle1 = GeometryBase::AngleOfTwoVector(
//						Current.x - D.x, Current.y - D.y, Current.z - D.z,
//						A.x - D.x, A.y - D.y, A.z - D.z);
//
//					//计算 点Current,A,D的夹角
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
//		else	//下界
//		{
//			if (D.y == A.y)		//如果在Y上相等
//			{
//				if (PlanePoints->points[i].y < D.y)
//				{
//					//计算 点Current,D,A 的夹角
//					double Angle1 = GeometryBase::AngleOfTwoVector(
//						Current.x - D.x, Current.y - D.y, Current.z - D.z,
//						A.x - D.x, A.y - D.y, A.z - D.z);
//
//					//计算 点Current,A,D的夹角
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
//					//计算 点Current,D,A 的夹角
//					double Angle1 = GeometryBase::AngleOfTwoVector(
//						Current.x - D.x, Current.y - D.y, Current.z - D.z,
//						A.x - D.x, A.y - D.y, A.z - D.z);
//
//					//计算 点Current,A,D的夹角
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
//2016.01.21 移动到合适的显示位置
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

//2016.01.21 移动到合适的显示位置
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