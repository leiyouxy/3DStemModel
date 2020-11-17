#include "StemGridPointsRefine.h"

CStemGridPointsRefine::CStemGridPointsRefine()
{
	BeforeRefinePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	p_HorizontalPartition = NULL;
	Viewer = NULL;
}

CStemGridPointsRefine::~CStemGridPointsRefine()
{
	if (Viewer != NULL)
		Viewer->removePointCloud("RefineRow0");

	if (p_HorizontalPartition != NULL)
		Viewer->removePointCloud("RefineRow" + QString::number(p_HorizontalPartition->SectionsCount - 1).toStdString());
	
	if (AnglePartitionInstance.SectionAnglePartitionS.size() > 0)
	{ 
		for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS[0].AnglePartition.size(); i++)
		{
			Viewer->removePointCloud("RefineCol" + QString::number(i).toStdString());
		}
	}
}

void CStemGridPointsRefine::SetInput(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Input,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SectionsCenterPointsValue,
	CHorizontalPartition * p_HorizontalPartitionValue, double AngleValue)
{
	InputCloud = Input;
	SectionsCenterPoints = SectionsCenterPointsValue;
	p_HorizontalPartition = p_HorizontalPartitionValue;
	Angle = AngleValue;

	p_HorizontalPartition->CalcCenterPointsByConvexPolygon();
	//p_HorizontalPartition->CalcCenterPointsByGravity();
	//p_HorizontalPartition->CalcCenterPointsByLeastSquares();

	AnglePartitionInstance.SetInputs(InputCloud, p_HorizontalPartition->SectionsVector,
		p_HorizontalPartition->GeometryCenterPointsPtr, Angle);
	AnglePartitionInstance.PartitionAllSection();

	GetOriginalGirdPoints();
}

// filling the hole anlge by using the same anlge partition in the upper section 
void CStemGridPointsRefine::RefineDownToUpByDirectlyFilling()
{
	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS.size() - 1; i++)
	{
		for (int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition.size(); j++)
		{
			if (AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].PointIndexs.size() == 0)
			{
				AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition
					= AnglePartitionInstance.SectionAnglePartitionS[i + 1].AnglePartition[j].CenterPointofCurPartition;
			}
		}
	}
}

//Refine 
void CStemGridPointsRefine::RefineStemGridPointsBySplineCurve(bool ShowFittingPoints)
{
	RefineDownToUpByDirectlyFilling();	
	//RefineRow(0);
	//RefineRow(AnglePartitionInstance.SectionAnglePartitionS.size() - 1);
	//ExecuteThreadNum = 0;

	#pragma omp parallel for
	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS.size(); i++)
		RefineRowByThread(i, ShowFittingPoints);

	//*
	//Row Refine
	/*int CurRowI = 0;
	while (CurRowI < AnglePartitionInstance.SectionAnglePartitionS.size())
	{
		while (ExecuteThreadNum < MaxThreadNum)
		{
			if (CurRowI >= AnglePartitionInstance.SectionAnglePartitionS.size())
			{
				break;
			}

			std::thread ThreadObj(std::bind(&CStemGridPointsRefine::RefineRowByThread,
				this, CurRowI, ShowFittingPoints));
			ExecuteThreadNum++;
			ThreadObj.join();
			CurRowI++;
		}
		Sleep(1000);
	}*/
	//*/

	/*
	//Col Refine
	int CurI = 0;
	while (CurI < AnglePartitionInstance.SectionAnglePartitionS[0].AnglePartition.size())
	{
		while (ExecuteThreadNum < MaxThreadNum)
		{
			if (CurI >= AnglePartitionInstance.SectionAnglePartitionS[0].AnglePartition.size())
			{
				break;
			}			
			std::thread ThreadObj(std::bind(&CStemGridPointsRefine::RefineColByThread,
				this, CurI, ShowFittingPoints));
			ExecuteThreadNum++;
			ThreadObj.join();
			CurI++;
		}
		Sleep(1000);
	}
	//*/
	   
	emitUpdateStatusBar("Repairation using Spline Curve is completed!", 5000);
	//for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS[0].AnglePartition.size(); i++)
	//{		
	//	RefineCol(i, ShowFittingPoints);		
	//}	
}

bool CStemGridPointsRefine::CheckRowIsNeedRefine(int RowIndex)
{
	for (int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition.size(); j++)
	{
		if (AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition[j].PointIndexs.size() == 0)
		{
			return true;
		}
	}
	return false;
}

pcl::PointXYZRGB CStemGridPointsRefine::FindAngleCenterPointInSplineCurve(CSpline & SplineCurve,
	pcl::PointXYZRGB CenterPoint, int AngleIndex)
{
	vector<AnglePartitionStruct> EachPartitions;

	CAnglePartition TempAnglePartition;
	TempAnglePartition.PartitionPoints(SplineCurve.CurvePoints,	CenterPoint, Angle, EachPartitions);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints (new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = 0; i < EachPartitions[AngleIndex].PointIndexs.size(); i++)
	{
		TempPoints->points.push_back(SplineCurve.CurvePoints->points[EachPartitions[AngleIndex].PointIndexs[i]]);
	}

	return GeometryBase::GetGravityOfPoints(TempPoints);
}

void CStemGridPointsRefine::RefineRowByThread(int RowIndex, bool DrawCurve)
{
	vector<int> HoleSectionIndexs;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RowInterpolationPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	{
		lock_guard<mutex> lockGuard(g_lock);
		//g_lock.lock();
		//emitUpdateStatusBar("Repair " + QString::number(RowIndex, 10) + " row, please wait!");

		for (int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition.size(); j++)
		{
			if (AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition[j].PointIndexs.size() == 0)
			{
				HoleSectionIndexs.push_back(j);
			}
			else
			{
				RowInterpolationPoints->points.push_back(
					AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition[j].CenterPointofCurPartition);
				RowInterpolationPoints->points[RowInterpolationPoints->points.size() - 1].z
					= p_HorizontalPartition->SectionsVector[RowIndex].ZMin;
			}
		}
		//g_lock.unlock();
	}

	if (HoleSectionIndexs.size() < 1)
	{
		//ExecuteThreadNum--;
		return;
	}

	if (RowInterpolationPoints->points.size() < 4)
	{
		cout << "该行可用插值点小于4个，无法通过B样条进行插值来修复该列缺失的节点！" << endl;
		//ExecuteThreadNum--;
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RowControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> KnotValues;

	//此处需要构建闭合曲线
	CSplineInterpolation SplineInterpolation;
	SplineInterpolation.SetInputs(RowInterpolationPoints, 3, true);
	SplineInterpolation.GetControlPointsAndKnotValue(RowControlPoints, KnotValues);

	//CRationalSpline Spline;
	CSpline Spline;
	Spline.SetSplineInputs(RowControlPoints, 3, KnotValues, true);
	Spline.FirstPointOfCloseCurve = RowInterpolationPoints->points[0];
	Spline.CreateSpline();	//生成曲线上的点\	

	{
		lock_guard<mutex> lockGuard(g_lock);
		//g_lock.lock();
		if (DrawCurve)
		{
			if (RowIndex % 3 == 0)
				PointBase::SetPointColor(Spline.CurvePoints, ColorBase::RedColor);
			else if (RowIndex % 3 == 1)
				PointBase::SetPointColor(Spline.CurvePoints, ColorBase::GreenColor);
			else if (RowIndex % 3 == 2)
				PointBase::SetPointColor(Spline.CurvePoints, ColorBase::BlueColor);

			PointBase::ShowPointXYZRGB(Viewer, Spline.CurvePoints, "RefineRow" + QString::number(RowIndex).toStdString(), 2);
		}

		for (int i = 0; i < HoleSectionIndexs.size(); i++)	//MissIndex 中 存储的是对应的 列号
		{
			pcl::PointXYZRGB TempPoint = FindAngleCenterPointInSplineCurve(Spline, SectionsCenterPoints->points[RowIndex], HoleSectionIndexs[i]);

			TempPoint.z = p_HorizontalPartition->SectionsVector[RowIndex].ZMin;	//插值点的高度 与 该行的高度值 一致
			AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition[HoleSectionIndexs[i]].CenterPointofCurPartition
				= TempPoint;
			AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition[HoleSectionIndexs[i]].Refined = true;
		}
		//g_lock.unlock();
	}
	//ExecuteThreadNum--;
}

void CStemGridPointsRefine::RefineRow(int RowIndex, bool DrawCurve)
{
	vector<int> HoleSectionIndexs;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RowInterpolationPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	emitUpdateStatusBar("Repair " + QString::number(RowIndex, 10) + " row, please wait!");

	for (int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition.size(); j++)
	{
		if (AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition[j].PointIndexs.size() == 0
			|| !AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition[j].Refined)
		{
			HoleSectionIndexs.push_back(j);
		}
		else
		{
			RowInterpolationPoints->points.push_back(
				AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition[j].CenterPointofCurPartition);
			RowInterpolationPoints->points[RowInterpolationPoints->points.size() - 1].z 
				= p_HorizontalPartition->SectionsVector[RowIndex].ZMin;			
		}
	}

	if (HoleSectionIndexs.size() < 1) return;

	if (RowInterpolationPoints->points.size() < 4)
	{
		//cout << "该行可用插值点小于4个，无法通过B样条进行插值来修复该列缺失的节点！" << endl;
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RowControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> KnotValues;

	//此处需要构建闭合曲线
	CSplineInterpolation SplineInterpolation;
	SplineInterpolation.SetInputs(RowInterpolationPoints, 3, true);
	SplineInterpolation.GetControlPointsAndKnotValue(RowControlPoints, KnotValues);

	//CRationalSpline Spline;
	CSpline Spline;
	Spline.SetSplineInputs(RowControlPoints, 3, KnotValues, true);
	Spline.FirstPointOfCloseCurve = RowInterpolationPoints->points[0];
	Spline.CreateSpline();	//生成曲线上的点\	

	if (DrawCurve)
	{
		if (RowIndex % 3 == 0)
			PointBase::SetPointColor(Spline.CurvePoints, ColorBase::RedColor);
		else if (RowIndex % 3 == 1)
			PointBase::SetPointColor(Spline.CurvePoints, ColorBase::GreenColor);
		else if (RowIndex % 3 == 2)
			PointBase::SetPointColor(Spline.CurvePoints, ColorBase::BlueColor);

		PointBase::ShowPointXYZRGB(Viewer, Spline.CurvePoints, "RefineRow" + QString::number(RowIndex).toStdString(), 2);
	}

	for (int i = 0; i < HoleSectionIndexs.size(); i++)	//MissIndex 中 存储的是对应的 列号
	{
		pcl::PointXYZRGB TempPoint = FindAngleCenterPointInSplineCurve(Spline, SectionsCenterPoints->points[RowIndex], HoleSectionIndexs[i]);

		TempPoint.z = p_HorizontalPartition->SectionsVector[RowIndex].ZMin;	//插值点的高度 与 该行的高度值 一致
		AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition[HoleSectionIndexs[i]].CenterPointofCurPartition
			= TempPoint;
		AnglePartitionInstance.SectionAnglePartitionS[RowIndex].AnglePartition[HoleSectionIndexs[i]].Refined = true;
	}
}

bool CStemGridPointsRefine::CheckColIsNeedRefine(int ColIndex)
{
	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS.size() - 1; i++)
	{
		if (AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[ColIndex].PointIndexs.size() == 0)
		{
			return true;
		}
	}
	return false;
}

void CStemGridPointsRefine::RefineCol(int ColIndex, bool DrawCurve)
{
	emitUpdateStatusBar("Repair " + QString::number(ColIndex, 10) + " column, please wait!");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColInterpolationPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<int> HoleSectionIndexs;
		
	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS.size(); i++)
	{
		if (AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[ColIndex].PointIndexs.size() > 0)
		{
			ColInterpolationPoints->points.push_back(
				(AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[ColIndex].CenterPointofCurPartition));
			
			ColInterpolationPoints->points[ColInterpolationPoints->points.size() - 1].z = 
				p_HorizontalPartition->SectionsVector[i].ZMin;			
		}
		else
			HoleSectionIndexs.push_back(i);
	}

	if (HoleSectionIndexs.size() < 1) return;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MissPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> KnotValues;

	if (ColInterpolationPoints->points.size() < 4)
	{
		cout << "该列可用插值点小于4个，无法通过B样条进行插值来修复该列缺失的节点！" << endl;
		return;
	}

	CSplineInterpolation SplineInterpolation;
	SplineInterpolation.SetInputs(ColInterpolationPoints, 3);
	SplineInterpolation.GetControlPointsAndKnotValue(ColControlPoints, KnotValues);

	//CRationalSpline Spline;
	CSpline Spline;
	Spline.SetSplineInputs(ColControlPoints, 3, KnotValues);
	Spline.CreateSpline();	

	if (DrawCurve)
	{
		if (ColIndex % 3 == 0)
			PointBase::SetPointColor(Spline.CurvePoints, ColorBase::RedColor);
		else if (ColIndex % 3 == 1)
			PointBase::SetPointColor(Spline.CurvePoints, ColorBase::GreenColor);
		else if (ColIndex % 3 == 2)
			PointBase::SetPointColor(Spline.CurvePoints, ColorBase::BlueColor);

		PointBase::ShowPointXYZRGB(Viewer, Spline.CurvePoints, "RefineCol" + QString::number(ColIndex).toStdString(), 2);
	}

	//PointBase::SavePCDToFileName(Spline.CurvePoints, "3DStemModelForMeasure\\RefineCol" + QString::number(ColIndex).toStdString() + ".pcd");

	for (int i = 0; i < HoleSectionIndexs.size(); i++)
	{
		int SectionIndex = HoleSectionIndexs[i];
		double HeightValue = p_HorizontalPartition->SectionsVector[SectionIndex].ZMin;

		MissPoints->points.clear();
		MissPoints = Spline.GetCurvePointsByValue(HeightValue, "Z", 0.000001);
		if (MissPoints->points.size() > 0)
		{
			AnglePartitionInstance.SectionAnglePartitionS[SectionIndex].
				AnglePartition[ColIndex].CenterPointofCurPartition = MissPoints->points[0];
			AnglePartitionInstance.SectionAnglePartitionS[SectionIndex].AnglePartition[ColIndex].Refined = true;
		}
	}
}

void CStemGridPointsRefine::RefineColByThread(int ColIndex, bool DrawCurve)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColInterpolationPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<int> HoleSectionIndexs;

	{
		lock_guard<mutex> lockGuard(g_lock);		
		//g_lock.lock();
		//emitUpdateStatusBar("Repair " + QString::number(ColIndex, 10) + " column, please wait!");
		for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS.size(); i++)
		{
			if (AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[ColIndex].PointIndexs.size() > 0)
			{
				ColInterpolationPoints->points.push_back(
					(AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[ColIndex].CenterPointofCurPartition));

				ColInterpolationPoints->points[ColInterpolationPoints->points.size() - 1].z =
					p_HorizontalPartition->SectionsVector[i].ZMin;
			}
			else
				HoleSectionIndexs.push_back(i);
		}
		//g_lock.unlock();
	}

	if (HoleSectionIndexs.size() < 1)
	{
		//ExecuteThreadNum--;
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MissPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<double> KnotValues;

	if (ColInterpolationPoints->points.size() < 4)
	{
		//ExecuteThreadNum--;
		cout << "该列可用插值点小于4个，无法通过B样条进行插值来修复该列缺失的节点！" << endl;
		return;
	}

	CSplineInterpolation SplineInterpolation;
	SplineInterpolation.SetInputs(ColInterpolationPoints, 3);
	SplineInterpolation.GetControlPointsAndKnotValue(ColControlPoints, KnotValues);

	//CRationalSpline Spline;
	CSpline Spline;
	Spline.SetSplineInputs(ColControlPoints, 3, KnotValues);
	Spline.CreateSpline();

	{
		lock_guard<mutex> lockGuard(g_lock);
		//g_lock.lock();
		if (DrawCurve)
		{
			if (ColIndex % 3 == 0)
				PointBase::SetPointColor(Spline.CurvePoints, ColorBase::RedColor);
			else if (ColIndex % 3 == 1)
				PointBase::SetPointColor(Spline.CurvePoints, ColorBase::GreenColor);
			else if (ColIndex % 3 == 2)
				PointBase::SetPointColor(Spline.CurvePoints, ColorBase::BlueColor);

			PointBase::ShowPointXYZRGB(Viewer, Spline.CurvePoints, "RefineCol" + QString::number(ColIndex).toStdString(), 2);
		}

		for (int i = 0; i < HoleSectionIndexs.size(); i++)
		{
			int SectionIndex = HoleSectionIndexs[i];
			double HeightValue = p_HorizontalPartition->SectionsVector[SectionIndex].ZMin;

			MissPoints->points.clear();
			MissPoints = Spline.GetCurvePointsByValue(HeightValue, "Z", 0.000001);
			if (MissPoints->points.size() > 0)
			{
				AnglePartitionInstance.SectionAnglePartitionS[SectionIndex].
					AnglePartition[ColIndex].CenterPointofCurPartition = MissPoints->points[0];
				AnglePartitionInstance.SectionAnglePartitionS[SectionIndex].AnglePartition[ColIndex].Refined = true;
			}
		}
		//g_lock.unlock();
	}
	//ExecuteThreadNum--;
}

void CStemGridPointsRefine::GetOriginalGirdPoints()
{
	BeforeRefinePoints->points.clear();

	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS.size(); i++)
		//for (int i = AnglePartitionInstance.SectionAnglePartitionS.size() - 4; i < AnglePartitionInstance.SectionAnglePartitionS.size(); i++)
	{
		for (int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition.size(); j++)
		{
			//It doesn't need to set the value of Refined as the function can be invoked after refining
			//AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].Refined = false;

			if (AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].PointIndexs.size() == 0)
				continue;

			BeforeRefinePoints->points.push_back(
				AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].BeforeRefineCenterPointofCurPartition);
			
			BeforeRefinePoints->points[BeforeRefinePoints->points.size() - 1].z = p_HorizontalPartition->SectionsVector[i].ZMin;

			//if (i % 3 == 0)
			//	BeforeRefinePoints->points[BeforeRefinePoints->points.size() - 1].rgba = ColorBase::RedColor;
			//else if (i % 3 == 1)
			//	BeforeRefinePoints->points[BeforeRefinePoints->points.size() - 1].rgba = ColorBase::GreenColor;
			//else if (i % 3 == 2)
			//	BeforeRefinePoints->points[BeforeRefinePoints->points.size() - 1].rgba = ColorBase::BlueColor;
		}
	}
}

//output
void CStemGridPointsRefine::GetRefinedPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Output)
{
	Output->points.clear();

	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS.size(); i++)
	//for (int i = AnglePartitionInstance.SectionAnglePartitionS.size() - 4; i < AnglePartitionInstance.SectionAnglePartitionS.size(); i++)
	{
		for (int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition.size(); j++)
		{
			if (AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].PointIndexs.size() == 0
				&& !AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].Refined)
			{
				//bool Value = AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].Refined;
				continue;
			}

			Output->points.push_back(AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition);
			//Output->points[Output->points.size() - 1].z = p_HorizontalPartition->SectionsVector[i].ZMin;

			//Output->points[Output->points.size() - 1].rgba = ColorBase::RedColor;

	/*		if (j % 3 == 0)
				Output->points[Output->points.size() - 1].rgba = ColorBase::RedColor;
			else if (j % 3 == 1)
				Output->points[Output->points.size() - 1].rgba = ColorBase::GreenColor;
			else if (j % 3 == 2)
				Output->points[Output->points.size() - 1].rgba = ColorBase::BlueColor;
				*/
		}
	}
}

void CStemGridPointsRefine::GetRefinedPointsBySliceIndex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Output, int SliceIndex)
{
	Output->points.clear();
	if (SliceIndex < 0 || SliceIndex >= AnglePartitionInstance.SectionAnglePartitionS.size()) return;

	for (int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[SliceIndex].AnglePartition.size(); j++)
	{
		pcl::PointXYZRGB TempPoint = AnglePartitionInstance.SectionAnglePartitionS[SliceIndex].AnglePartition[j].CenterPointofCurPartition;
		
		if (abs(TempPoint.x) < EPSM6 && abs(TempPoint.y) < EPSM6  && abs(TempPoint.z) < EPSM6)
			continue;

		TempPoint.z = p_HorizontalPartition->SectionsVector[SliceIndex].ZMin;		
		Output->points.push_back(TempPoint);		
	}
}

void CStemGridPointsRefine::GetBeforeRefinedPointsBySliceIndex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Output, int SliceIndex)
{
	Output->points.clear();
	if (SliceIndex < 0 || SliceIndex >= AnglePartitionInstance.SectionAnglePartitionS.size()) return;

	for (int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[SliceIndex].AnglePartition.size(); j++)
	{
		pcl::PointXYZRGB TempPoint = AnglePartitionInstance.SectionAnglePartitionS
							[SliceIndex].AnglePartition[j].BeforeRefineCenterPointofCurPartition;
		if (abs(TempPoint.x) < EPSM6 && abs(TempPoint.y) < EPSM6) continue;

		TempPoint.z = p_HorizontalPartition->SectionsVector[SliceIndex].ZMin;
		Output->points.push_back(TempPoint);		 
	}
	PointBase::GetDistinctPoints(Output);
}


void CStemGridPointsRefine::RefineStemGridPointsByStemAxisCurve(	 
	bool ShowFittingPoints)
{
	//Use StemSkeleton to get suitable interpolations
	CStemSkeleton StemSkeleton;
	StemSkeleton.SetInputs(InputCloud, p_HorizontalPartition->SectionThickness, true);
	StemSkeleton.ConstructStemSplineCurve();		

	PointBase::SetPointColor(StemSkeleton.StemSkeletonSpline.CurvePoints, ColorBase::RedColor);
	ShowPoints(StemSkeleton.StemSkeletonSpline.CurvePoints, "StemSkeleton");	
	
	///*	
	for (int i = 0; i < StemSkeleton.HorizontalPartition.SectionsCount; i++)
	{
		cout<<"RefineStemGridPointsByStemAxisCurve, Row: "<<i<<endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSectionInterpolationPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		StemSkeleton.HorizontalPartition.GetSectionPoints(i, CurSectionPoints);

		//StemSkeleton.HorizontalPartition.SaveSectionSToFile("I:\\ErrorPoint.pcd", 19, 20);

		//2019.07.21
		//vector<double> CurU = StemSkeleton.StemSkeletonSpline.GetCurvePointsUByValue(
			//StemSkeleton.HorizontalPartition.SectionsVector[i].ZMin, "Z");
		
		//此处有计算不合理的地方，由于StemSkeleton 并没有包括最低点与最高点,从而导致出现偏差。2019.08.18 
		double CurU = StemSkeleton.StemSkeletonSpline.GetUValueBySplineHeight(
			StemSkeleton.HorizontalPartition.SectionsVector[i].ZMin);

		pcl::PointXYZRGB CurSectionGeometricalPoint 
			//= StemSkeleton.StemSkeletonSpline.GetSplinePoint(CurU[0]);		
			= StemSkeleton.StemSkeletonSpline.GetSplinePoint(CurU);
		
		//cout<<"Z:"<< StemSkeleton.HorizontalPartition.SectionsVector[i].ZMin <<endl;
		//cout << "CurSectionGeometricalPoint:" << CurSectionGeometricalPoint << endl;

		p_HorizontalPartition->GeometryCenterPointsPtr->points[i] = CurSectionGeometricalPoint;		

		AnglePartitionInstance.PartitionPoints(CurSectionPoints, CurSectionGeometricalPoint, Angle, 
			AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition);

		double AngleAvgDis = 0;
		int AngleNum = 0;
		
		#pragma  omp parallel for ordered schedule(dynamic)
		for(int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition.size(); j++)		
		{		
			//cout << "RefineStemGridPointsByStemAxisCurve, Row: " << i << ", Col: " << j << endl;
			//emitUpdateStatusBar(("Repair " + StringBase::IntToStr(i) + " " + StringBase::IntToStr(j)).c_str(), 0);
			AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition.x = 0, 
			AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition.y = 0, 
			AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition.z = 0;
			AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition.rgba = ColorBase::StemColor;

			int PointNums = AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].PointIndexs.size();
			if (PointNums > 0)
			{
				AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].NeedRefined = false;
				double AvgDis = 0;
				for each (int PointIndex in AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].PointIndexs)
				{
					AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition.x += 						
						CurSectionPoints->points[PointIndex].x / PointNums;
					
					AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition.y += 						
						CurSectionPoints->points[PointIndex].y / PointNums;

					///*
					AvgDis += PointDis(CurSectionGeometricalPoint.x, CurSectionGeometricalPoint.y,
						CurSectionPoints->points[PointIndex].x, CurSectionPoints->points[PointIndex].y) /
						(1.0 * AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].PointIndexs.size());
						//*/
				}
				AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition.z =
					StemSkeleton.HorizontalPartition.SectionsVector[i].ZMin;

				//AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition.rgba = ColorBase::RedColor;

				AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].BeforeRefineCenterPointofCurPartition =
					AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition;
				
				//for each (int PointIndex in AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].PointIndexs)
				//{
				//	float TempDis = PointDis(CurSectionGeometricalPoint.x, CurSectionGeometricalPoint.y,
				//		CurSectionPoints->points[PointIndex].x, CurSectionPoints->points[PointIndex].y);
				//	if (TempDis < 7.5 * AvgDis / 10.0)
				//	{
				//		cout << "i:" << i << ", j:" << j << endl;
				//		cout <<"TempDis:"<< TempDis <<", AvgDis:"<< AvgDis << endl;
				//	}
				//}

				#pragma omp ordered
				{
					AngleNum++;
					AngleAvgDis += PointDis(AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition,
						CurSectionGeometricalPoint);
					//cout<<"AngleNum: "<<AngleNum << endl;
				}
			}
			else
			{
				AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].NeedRefined = true;
				AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].Refined = false;
			}
		}

		if (AngleNum > 0)
			AngleAvgDis = AngleAvgDis / AngleNum;
		AnglePartitionInstance.SectionAnglePartitionS[i].AvgDis = AngleAvgDis;
				
		RefineSlicesByLinear(i);
		//RefineAngleRule(i);
	}

	////need to be refine the empty angle partition
	//RefineHeadAndRearByStemAxisCurve(0);
	//RefineHeadAndRearByStemAxisCurve(AnglePartitionInstance.SectionAnglePartitionS.size() - 1);

	//#pragma omp parallel for 
	//for (int i = 1; i < AnglePartitionInstance.SectionAnglePartitionS.size() - 1; i++)
	//{
	//	//emitUpdateStatusBar(("Repair Middle " + StringBase::IntToStr(i) ).c_str(), 0);
	//	RefineMiddleByStemAxisCurve(i);		
	//}
	//*/

	//cout<<"Size:"<< p_HorizontalPartition->GeometryCenterPointsPtr->points.size() <<endl;
	PointBase::SetPointColor(p_HorizontalPartition->GeometryCenterPointsPtr, ColorBase::BlueColor);
	ShowPoints(p_HorizontalPartition->GeometryCenterPointsPtr, "GeometryCenterPointsPtr", 5);	

	emitUpdateStatusBar("Repairation using Stem Axis Curve is completed!", 5000);
}

//2019.07.16 根据角度与距离的线性关系对多个连续缺失区域的修复
void CStemGridPointsRefine::RefineSlicesByLinear(int Index)
{	
	pcl::PointXYZRGB SectionCenter = p_HorizontalPartition->GeometryCenterPointsPtr->points[Index];

	if (Index > 0 && (abs(p_HorizontalPartition->GeometryCenterPointsPtr->points[Index].z
		- p_HorizontalPartition->GeometryCenterPointsPtr->points[Index - 1].z) > p_HorizontalPartition->SectionThickness))
		return;

	//先上下线性关系修复
	#pragma omp parallel for
	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size(); i++)
	{
		if ((AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].NeedRefined)
			&& (!AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].Refined))
		{
			int UpperIndex = Index + 1;
			int LowerIndex = Index - 1;

			if (UpperIndex >= p_HorizontalPartition->SectionsCount) continue;
			if (LowerIndex < 0) continue;

			if (AnglePartitionInstance.SectionAnglePartitionS[UpperIndex].AnglePartition[i].PointIndexs.size() > 0
				&& AnglePartitionInstance.SectionAnglePartitionS[LowerIndex].AnglePartition[i].PointIndexs.size() > 0)
			{
				pcl::PointXYZRGB RefinedPoint;
				RefinedPoint.x = (AnglePartitionInstance.SectionAnglePartitionS[UpperIndex].AnglePartition[i].CenterPointofCurPartition.x
					+ AnglePartitionInstance.SectionAnglePartitionS[LowerIndex].AnglePartition[i].CenterPointofCurPartition.x)/2.0;

				RefinedPoint.y = (AnglePartitionInstance.SectionAnglePartitionS[UpperIndex].AnglePartition[i].CenterPointofCurPartition.y
					+ AnglePartitionInstance.SectionAnglePartitionS[LowerIndex].AnglePartition[i].CenterPointofCurPartition.y)/2.0;

				/*RefinedPoint.z = (AnglePartitionInstance.SectionAnglePartitionS[UpperIndex].AnglePartition[i].CenterPointofCurPartition.z
					+ AnglePartitionInstance.SectionAnglePartitionS[LowerIndex].AnglePartition[i].CenterPointofCurPartition.z) / 2;*/

				RefinedPoint.z = p_HorizontalPartition->SectionsVector[Index].ZMin;

				RefinedPoint.rgba = ColorBase::BlueColor;

				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition = RefinedPoint;

				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].Refined = true;
			}

		}
	}
	
	///*
	bool Continued = false;
	int K = 0;
	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size(); i++)
	{		
		if ((AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].NeedRefined)
			&& (!AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].Refined))
		{							
			Continued = true;	
			//cout<<"Missed:"<< AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i] .CenterPointofCurPartition <<endl;
			K++;
		}
		else if (!(AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].NeedRefined) 
			|| (AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].Refined))
		{
			if (Continued)
			{
				//cout << endl;
				//cout << "RefineSlicesByLinear:" << Index << ", i" << i << ", K:" << K << endl;
				int StartIndex = (i - K - 1) % AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size();
				int EndIndex = i;
				pcl::PointXYZRGB StartPoint = AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[StartIndex].CenterPointofCurPartition;
				pcl::PointXYZRGB EndPoint = AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[EndIndex].CenterPointofCurPartition;						
				//SectionCenter.z = StartPoint.z;

				double avgAngle = (GeometryBase::AngleValueOfThreePoints(StartPoint, SectionCenter, EndPoint)) / (K + 1);
				double StartDis = PointDis(StartPoint, SectionCenter);
				double EndDis = PointDis(EndPoint, SectionCenter);
				double AvgDis = (EndDis - StartDis) / (K + 1);

				//cout << "StartPoint:" << StartPoint << ", EndPoint:" << EndPoint << endl;
				//cout << "StartDis:" << StartDis << ", EndDis:" << EndDis << ", AvgDis:" << AvgDis << endl;								
				//cout << "StartIndex:" << StartIndex << ", EndIndex:" << EndIndex << endl;
				#pragma  omp parallel for
				for (int j = (i - K - 1) + 1; j < EndIndex; j++)
				{
					int jIndex = j % AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size();

					//cout << "StartIndex:" << StartIndex << ", EndIndex:" << EndIndex << ", RefinedIndex:" << jIndex << endl;
					double CurAngle = avgAngle * (j - (i - K - 1));
					double Dis = AvgDis * (j - (i - K - 1));
					
					pcl::PointXYZRGB DirectionEndPoint = GeometryBase::RotatePointBAroundPointA(SectionCenter, StartPoint, CurAngle);
					pcl::PointXYZRGB DirectionPoint;
					DirectionPoint.x = DirectionEndPoint.x - SectionCenter.x;
					DirectionPoint.y = DirectionEndPoint.y - SectionCenter.y;
					DirectionPoint.z = DirectionEndPoint.z - SectionCenter.z;

					//cout << "StartDis:" << StartDis << ", EndDis:" << EndDis << ", StartDis + Dis:" << StartDis + Dis << endl;
					pcl::PointXYZRGB CurAnglePoint = GeometryBase::GetPointAlongLine(DirectionPoint, SectionCenter, StartDis + Dis);
					//cout << "Refined Dis:" <<PointDis(SectionCenter, CurAnglePoint)<<endl;

					CurAnglePoint.rgba = ColorBase::RedColor;
					CurAnglePoint.z = StartPoint.z;				

					//cout<<"Missed:"<< AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[jIndex] .CenterPointofCurPartition <<endl;
					AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[jIndex].CenterPointofCurPartition = CurAnglePoint;
					//cout << "Refined:" << AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[jIndex].CenterPointofCurPartition << endl;

					AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[jIndex].Refined = true;
				}
			}

			Continued = false;
			K = 0;
		}
	}
	//*/
}

//2019.07.21 对当前垂直分段中的轮廓点进行优化，优化准则是当前轮廓点到当前角度分区中的点的距离最小，且相邻角度分区间的角度也越均衡，
//且满足相邻角度分区的轮廓点到中心点距离偏差不是太大.
void CStemGridPointsRefine::RefineSlincesByOptimal(int Index)
{

}


//根据公式计算很繁琐，需要求解一元四次方程组，实际可使用 0.618法计算
//输入时，PointA，DirectionA，PointB，DirectionB 四点呈逆时针次序排列
//在线段(DirectionA，DirectionB)上寻找一点ResultPoint，使得角(PointA, ResultPoint, PointB)为Angle(弧度)，且三点呈逆时针次序排列
pcl::PointXYZRGB CStemGridPointsRefine::GetMiddlePointAlongLineBetweenTwoPoints(
	pcl::PointXYZRGB DirectionA, pcl::PointXYZRGB DirectionB,
	pcl::PointXYZRGB PointA, pcl::PointXYZRGB PointB, double Angle)
{	
	pcl::PointXYZRGB ResultPoint, Direction;
	Direction.x = DirectionB.x - DirectionA.x;
	Direction.y = DirectionB.y - DirectionA.y;
	Direction.z = DirectionB.z - DirectionA.z;
	//PointBase::PointNormalized(Direction);
	double DisAB = PointDis(PointA, PointB);
	double Lamda = 0;
	
	double Lower = 0, Upper = 1;
	double Range = Upper - Lower;
	double Start_Try = 0, End_Try = 1;
	//cin >> Range;
	//Start_Try = Lower + 0.312 * Range;
	//End_Try = Lower + 0.618 * Range;
	cout << "DirectionB:" << DirectionB << endl;
	cout << "DirectionA:" << DirectionA << endl;
	pcl::PointXYZRGB StartPoint, EndPoint;
	StartPoint.x = DirectionA.x + Start_Try * Direction.x;
	StartPoint.y = DirectionA.y + Start_Try * Direction.y;
	StartPoint.z = DirectionA.z + Start_Try * Direction.z;
	cout<<"StartPoint:"<< StartPoint <<endl;
	EndPoint.x = DirectionA.x + End_Try * Direction.x;
	EndPoint.y = DirectionA.y + End_Try * Direction.y;
	EndPoint.z = DirectionA.z + End_Try * Direction.z;
	cout << "EndPoint:" << EndPoint << endl;

	double InitialAngle = GeometryBase::AngleValueOfThreePoints(PointA, DirectionA, PointB);
	cout << "InitialAngle:" << GeometryBase::RadianToAngle(InitialAngle) << endl;

	double StartAngle = GeometryBase::AngleValueOfThreePoints(PointA, StartPoint, PointB);
	double EndAngle = GeometryBase::AngleValueOfThreePoints(PointA, EndPoint, PointB);

	cout << "Initial StartAngle:" << GeometryBase::RadianToAngle(StartAngle) << endl;
	cout << "Initial EndAngle:" << GeometryBase::RadianToAngle(EndAngle) << endl;
	cout << "Result Angle:" << GeometryBase::RadianToAngle(Angle) << endl;
	cout << "Start_Try:" << Start_Try << endl;
	cout << "End_Try:" << End_Try << endl << endl;

	while (abs(StartAngle - Angle) > EPSM3 || abs(EndAngle - Angle) > EPSM3)
	{
		double StartAngle = GeometryBase::AngleValueOfThreePoints(PointA, StartPoint, PointB);
		double EndAngle = GeometryBase::AngleValueOfThreePoints(PointA, EndPoint, PointB);

		if (abs(StartAngle - Angle) <= EPSM3)
		{
			Lamda = Start_Try;
			break;
		}
		else if (abs(EndAngle - Angle) <= EPSM3)
		{
			Lamda = End_Try;
			break;
		}

		//if (abs(EndAngle - Angle) < abs(StartAngle - Angle))
		if ((EndAngle - Angle) < (Angle - StartAngle))
		{
			//cout << "Lower:" << endl;
			Lower = Start_Try;
			Range = Upper - Lower;
			Start_Try = Lower + 0.312 * Range;
			End_Try = Lower + 0.618 * Range;
		}
		else
		{			
			Upper = End_Try;
			Range = Upper - Lower;
			Start_Try = Lower + 0.312 * Range;
			End_Try = Lower + 0.618 * Range;
		}
		cout << "StartAngle:" << GeometryBase::RadianToAngle(StartAngle) << endl;
		cout << "EndAngle:" << GeometryBase::RadianToAngle(EndAngle) << endl;
		cout << "Start_Try:" << Start_Try << endl;
		cout << "End_Try:" << End_Try << endl << endl;

		StartPoint.x = DirectionA.x + Start_Try * Direction.x;
		StartPoint.y = DirectionA.y + Start_Try * Direction.y;
		StartPoint.z = DirectionA.z + Start_Try * Direction.z;

		EndPoint.x = DirectionA.x + End_Try * Direction.x;
		EndPoint.y = DirectionA.y + End_Try * Direction.y;
		EndPoint.z = DirectionA.z + End_Try * Direction.z;

		if (abs(End_Try - Start_Try) < EPSM6) 
		{
			if (abs(StartAngle - Angle) < abs(EndAngle - Angle))
				Lamda = Start_Try;
			else 
				Lamda = End_Try;
			break;
		}
	}
	cout << "Lamda:" << Lamda << endl;
	ResultPoint.x = DirectionA.x + Lamda * Direction.x;
	ResultPoint.y = DirectionA.y + Lamda * Direction.y;
	ResultPoint.z = DirectionA.z + Lamda * Direction.z;	
	return ResultPoint;
}

//2019.07.18 确保相邻三个点的距离不小于90度，如果有，则向中心点移动
void CStemGridPointsRefine::RefineAngleRule(int Index)
{
	//if (Index == 0)
	//{
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points(new pcl::PointCloud<pcl::PointXYZRGB>);

	//	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size(); i++)
	//	{		
	//		Points->points.push_back(AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition);
	//	}
	//	PointBase::SavePCDToFileName(Points, "I://TTTesttCenterPoint.pcd");
	//}

	//#pragma omp parallel for
	pcl::PointXYZRGB SectionCenter = p_HorizontalPartition->GeometryCenterPointsPtr->points[Index];
	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size(); i++)
	{
		int Prior = (i - 1) % AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size();
		int Last = (i + 1) % AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size();

		double LeftAngle = GeometryBase::AngleValueOfThreePoints(
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition,
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Prior].CenterPointofCurPartition,
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Last].CenterPointofCurPartition);

		double RightAngle = GeometryBase::AngleValueOfThreePoints(
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition,
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Last].CenterPointofCurPartition,
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Prior].CenterPointofCurPartition);

		/*if (LeftAngle >= M_PI_2 || RightAngle >= M_PI_2)  //此处会出现错误
		{
			cout<<"LeftAngle:"<< LeftAngle <<endl;
			cout << "RightAngle:" << RightAngle << endl;
			double Dis = PointDis(AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition, SectionCenter);
			pcl::PointXYZRGB Direction;
			
			Direction.x = (AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Prior].CenterPointofCurPartition.x
				+ AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Last].CenterPointofCurPartition.x) / 2 - SectionCenter.x;

			Direction.y = (AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Prior].CenterPointofCurPartition.y
				+ AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Last].CenterPointofCurPartition.y) / 2 - SectionCenter.y;

			Direction.z = (AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Prior].CenterPointofCurPartition.z
				+ AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Last].CenterPointofCurPartition.z) / 2 - SectionCenter.z;

			pcl::PointXYZRGB RefinedPoint = GeometryBase::GetPointAlongLine(Direction, SectionCenter, Dis);
			
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition = RefinedPoint;
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition.rgba = ColorBase::DoveColor;
		}*/

		/*
		double MiddleAngle = GeometryBase::AngleValueOfThreePoints(
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Prior].CenterPointofCurPartition,
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition,
			AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Last].CenterPointofCurPartition);

		if (MiddleAngle < M_PI_2 * 10.0 / 90.0)
		{
			cout<<"MiddleAngle:"<< GeometryBase::RadianToAngle(MiddleAngle) <<endl;
			pcl::PointXYZRGB PointA = AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Prior].CenterPointofCurPartition;
			pcl::PointXYZRGB PointB = AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition;
			pcl::PointXYZRGB PointC = AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[Last].CenterPointofCurPartition;

			double Line1A, Line1B, Line1D, Line2A, Line2B, Line2D;
			
			if (PointA.x - PointC.x != 0)
			{
				Line1A = -(PointA.y - PointC.y) / (PointA.x - PointC.x);
				Line1B = 1;
				Line1D = -PointA.y - Line1A * PointA.x;
			}
			else
			{
				Line1A = 0; Line1B = 1; Line1D = -PointA.y;
			}

			if (PointB.x - SectionCenter.x != 0)
			{
				Line2A = -(PointB.y - SectionCenter.y) / (PointB.x - SectionCenter.x);
				Line2B = 1;
				Line2D = -PointB.y - Line1A * PointB.x;
			}
			else
			{
				Line2A = 0; Line2B = 1; Line2D = -PointB.y;
			}

			bool IsHave;
			pcl::PointXYZRGB LineCorssPoint = GeometryBase::CalcIntersectionOfTwoLine(Line1A, Line1B, Line1D, Line2A, Line2B, Line2D, IsHave);

			if (IsHave)
			{	
				//pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points(new pcl::PointCloud<pcl::PointXYZRGB>);

				//Points->points.push_back(PointA);
				//Points->points.push_back(PointB);
				//Points->points.push_back(PointC);
				//Points->points.push_back(SectionCenter);
				//Points->points.push_back(LineCorssPoint);

				//PointBase::SavePCDToFileName(Points, "I://TTTestt.pcd");

				pcl::PointXYZRGB RefinedPoint = GetMiddlePointAlongLineBetweenTwoPoints(
					PointB, LineCorssPoint, PointA, PointC, M_PI_2 * 10.0 / 90.0);
				
				cout << "PointB:" << PointB << endl;
				cout << "RefinedPoint:" << RefinedPoint << endl;

				RefinedPoint.rgba = ColorBase::BlueColor;
				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition = RefinedPoint;

			}		
			
		}
		//*/
	}
//	if (Index == 0)
//{
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size(); i++)
//	{		
//		Points->points.push_back(AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition);
//	}
//	PointBase::SavePCDToFileName(Points, "I://TTTesttCenterPoint.pcd");
//}
//	cin >> Index;
}


void CStemGridPointsRefine::RefineHeadAndRearByStemAxisCurve(int Index)
{
	//emitUpdateStatusBar(("Repair HeadAndRear " + StringBase::IntToStr(Index)).c_str(), 0);
	int SectionNum = round(RepairLength / p_HorizontalPartition->SectionThickness);
	int Coefficient = 0;

	if (Index == 0)
		Coefficient = 1;
	else if (Index == AnglePartitionInstance.SectionAnglePartitionS.size() - 1)
		Coefficient = -1;

	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size(); i++)
	{
		if (AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].PointIndexs.size() == 0)
		{
			//First use the opposite point to repair. if not then use the Upper or Lower slice to repair.

			pcl::PointXYZRGB TempPoint;
			TempPoint.x = 0, TempPoint.y = 0, TempPoint.z = 0;

			int OppositeAngleIndex =
				(i + AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size())
				% AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size();
			if (AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[OppositeAngleIndex].PointIndexs.size() > 0 ||
				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[OppositeAngleIndex].Refined)
			{
				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition.x
					= 2 * p_HorizontalPartition->GeometryCenterPointsPtr->points[i].x - 
					AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[OppositeAngleIndex].CenterPointofCurPartition.x;

				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition.y
					= 2 * p_HorizontalPartition->GeometryCenterPointsPtr->points[i].y -
					AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[OppositeAngleIndex].CenterPointofCurPartition.y;

				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition.z
					= AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[OppositeAngleIndex].CenterPointofCurPartition.z;

				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].Refined = true;
			
				//continue;
			}
			else
			{
				int TempNum = 0;
				for (int j = SectionNum; j > 0; j--)
				{
					int TempSectionIndex = Index + Coefficient * j;
				
					if (TempSectionIndex < 0 || TempSectionIndex >= AnglePartitionInstance.SectionAnglePartitionS.size())
						continue;

					if (AnglePartitionInstance.SectionAnglePartitionS[TempSectionIndex].AnglePartition[i].PointIndexs.size() > 0)
					{
						TempPoint.x = TempPoint.x + AnglePartitionInstance.SectionAnglePartitionS[TempSectionIndex].AnglePartition[i].CenterPointofCurPartition.x;
						TempPoint.y = TempPoint.y + AnglePartitionInstance.SectionAnglePartitionS[TempSectionIndex].AnglePartition[i].CenterPointofCurPartition.y;

						TempNum++;
					}
				}

				if (TempNum > 0)
				{
					TempPoint.x = TempPoint.x / TempNum;
					TempPoint.y = TempPoint.y / TempNum;
					TempPoint.z = p_HorizontalPartition->SectionsVector[Index].ZMin;

					AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition = TempPoint;
					AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].Refined = true;
				}
			}
		}
	}
	RefineRow(Index);
}

void CStemGridPointsRefine::RefineMiddleByStemAxisCurve(int Index)
{
	int SectionNum = round(RepairLength / p_HorizontalPartition->SectionThickness / 2.0);

	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size(); i++)
	{
		if (AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].PointIndexs.size() == 0)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = 0, TempPoint.y = 0, TempPoint.z = 0;

			int OppositeAngleIndex =
				(i + AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size())
				% AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition.size();
			
			if (AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[OppositeAngleIndex].PointIndexs.size() > 0 ||
				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[OppositeAngleIndex].Refined)
			{
				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition.x
					= 2 * p_HorizontalPartition->GeometryCenterPointsPtr->points[i].x -
					AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[OppositeAngleIndex].CenterPointofCurPartition.x;

				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition.y
					= 2 * p_HorizontalPartition->GeometryCenterPointsPtr->points[i].y -
					AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[OppositeAngleIndex].CenterPointofCurPartition.y;

				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition.z
					= AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[OppositeAngleIndex].CenterPointofCurPartition.z;

				AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].Refined = true;

				//continue;
			}
			else
			{
				int TempNum = 0;
				for (int j = SectionNum; j > 0; j--)
				{
					int TempSectionIndex = Index + j;
					if (TempSectionIndex < AnglePartitionInstance.SectionAnglePartitionS.size())
					{
						if (AnglePartitionInstance.SectionAnglePartitionS[TempSectionIndex].AnglePartition[i].PointIndexs.size() > 0)
						{
							TempPoint.x = TempPoint.x + AnglePartitionInstance.SectionAnglePartitionS[TempSectionIndex].AnglePartition[i].CenterPointofCurPartition.x;
							TempPoint.y = TempPoint.y + AnglePartitionInstance.SectionAnglePartitionS[TempSectionIndex].AnglePartition[i].CenterPointofCurPartition.y;

							TempNum++;
						}
					}

					TempSectionIndex = Index - j;
					if (TempSectionIndex < AnglePartitionInstance.SectionAnglePartitionS.size() && TempSectionIndex >= 0)
					{
						if (AnglePartitionInstance.SectionAnglePartitionS[TempSectionIndex].AnglePartition[i].PointIndexs.size() > 0)
						{
							TempPoint.x = TempPoint.x + AnglePartitionInstance.SectionAnglePartitionS[TempSectionIndex].AnglePartition[i].CenterPointofCurPartition.x;
							TempPoint.y = TempPoint.y + AnglePartitionInstance.SectionAnglePartitionS[TempSectionIndex].AnglePartition[i].CenterPointofCurPartition.y;

							TempNum++;
						}
					}
				}

				if (TempNum > 0)
				{
					TempPoint.x = TempPoint.x / TempNum;
					TempPoint.y = TempPoint.y / TempNum;
					TempPoint.z = p_HorizontalPartition->SectionsVector[Index].ZMin;

					AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].CenterPointofCurPartition = TempPoint;
					AnglePartitionInstance.SectionAnglePartitionS[Index].AnglePartition[i].Refined = true;
				}
			}
		}
	}
	// if the empty angle partition can not be repaired by the above procedure, then uese the spline procedure.
	RefineRow(Index);
}

void CStemGridPointsRefine::GetRefinePointsForSurfaceInterpolation(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & Output)
{
	for (int i = 0; i < Output.size(); i++)
	{
		Output[i]->points.clear();
	}
	Output.clear();

	for (int i = 0; i < AnglePartitionInstance.SectionAnglePartitionS.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
		for (int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition.size(); j++)
		{
			if (AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].PointIndexs.size() > 0 ||
				AnglePartitionInstance.SectionAnglePartitionS[i].AnglePartition[j].Refined)
			{
				TempPoints->points.push_back(AnglePartitionInstance.
					SectionAnglePartitionS[i].AnglePartition[j].CenterPointofCurPartition);
			}
		}
		Output.push_back(TempPoints);
	}

	//Procedure for the toppest slice
	/*
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSectionInterpolationPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	CurSectionInterpolationPoints->points.insert(CurSectionInterpolationPoints->points.begin(),
		Output[Output.size() - 1]->points.begin(),	Output[Output.size() - 1]->points.end());
	PointBase::SetPointsCoordinateValue(CurSectionInterpolationPoints, "Z", ZMax);
	Output.push_back(CurSectionInterpolationPoints);
	//*/

}

void CStemGridPointsRefine::GetSliceRefinePoints(int SliceIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr SliceRefinedPoints)
{
	SliceRefinedPoints->points.clear();
	for (int j = 0; j < AnglePartitionInstance.SectionAnglePartitionS[SliceIndex].AnglePartition.size(); j++)
	{
		if (AnglePartitionInstance.SectionAnglePartitionS[SliceIndex].AnglePartition[j].PointIndexs.size() > 0 ||
			AnglePartitionInstance.SectionAnglePartitionS[SliceIndex].AnglePartition[j].Refined)
		{
			SliceRefinedPoints->points.push_back(AnglePartitionInstance.
				SectionAnglePartitionS[SliceIndex].AnglePartition[j].CenterPointofCurPartition);
		}
	}
}
