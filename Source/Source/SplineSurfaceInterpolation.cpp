#include "SplineSurfaceInterpolation.h"

////The input is the stem points that should be formatted to Interpolation points

CSurfaceInterpolation::~CSurfaceInterpolation()
{
	for each (pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp in InterpolationPoints)
	{
		Temp->points.clear();
	}
	InterpolationPoints.clear();

	for each (pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp in RowControlPoints)
	{
		Temp->points.clear();
	}
	RowControlPoints.clear();

	for each (pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp in SurfacePoints)
	{
		Temp->points.clear();
	}
	SurfacePoints.clear();

	for each (pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp in GlobalControlPoints)
	{
		Temp->points.clear();
	}
	GlobalControlPoints.clear();

	KnotValuesCol.clear();
	ParametersCol.clear();
	KnotValuesRow.clear();
	ParametersRow.clear();
}

void CSurfaceInterpolation::SetInputStemPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr StemPointPtr,
	float SectionThickNessValue, int CalcSectionNumbersValue, double AnlgeValue, 
	double ZMax,
	double HStepValue, double ColStepValue, double RowStepValue)
{
	//Use StemSkeleton to get suitable interpolations
	CStemSkeleton StemSkeleton;
	StemSkeleton.SetInputs(StemPointPtr, SectionThickNessValue, true, CalcSectionNumbersValue);
	StemSkeleton.ConstructStemSplineCurve();

	CAnglePartition AnglePartitionInstance;

	for (int i = 0; i < StemSkeleton.HorizontalPartition.SectionsCount; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSectionPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSectionInterpolationPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		StemSkeleton.HorizontalPartition.GetSectionPoints(i, CurSectionPoints);
		double CurU = StemSkeleton.StemSkeletonSpline.GetUValueBySplineHeight(StemSkeleton.HorizontalPartition.SectionsVector[i].ZMin);
		pcl::PointXYZRGB CurSectionGeometricalPoint = StemSkeleton.StemSkeletonSpline.GetSplinePoint(CurU);

		vector<AnglePartitionStruct> SectionAngle;
		AnglePartitionInstance.PartitionPoints(CurSectionPoints, CurSectionGeometricalPoint, AnlgeValue, SectionAngle);

		for each (AnglePartitionStruct Struct in SectionAngle)
		{			
			Struct.CenterPointofCurPartition.x = 0, Struct.CenterPointofCurPartition.y = 0, Struct.CenterPointofCurPartition.z = 0;
			if (Struct.PointIndexs.size() > 0)
			{
				for each (int PointIndex in Struct.PointIndexs)
				{
					Struct.CenterPointofCurPartition.x = Struct.CenterPointofCurPartition.x +
						CurSectionPoints->points[PointIndex].x / (1.0 * Struct.PointIndexs.size());
					Struct.CenterPointofCurPartition.y = Struct.CenterPointofCurPartition.y +
						CurSectionPoints->points[PointIndex].y / (1.0 * Struct.PointIndexs.size());
					
					Struct.CenterPointofCurPartition.z = StemSkeleton.HorizontalPartition.SectionsVector[i].ZMin;
				}
			}

			CurSectionInterpolationPoints->points.push_back(Struct.CenterPointofCurPartition);
		}	

		InterpolationPoints.push_back(CurSectionInterpolationPoints);
	}
	
	//Procedure for the toppest slice
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurSectionInterpolationPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	CurSectionInterpolationPoints->points.insert(CurSectionInterpolationPoints->points.begin(),
		InterpolationPoints[InterpolationPoints.size() - 1]->points.begin(),
		InterpolationPoints[InterpolationPoints.size() - 1]->points.end());
	
	PointBase::SetPointsCoordinateValue(CurSectionInterpolationPoints, "Z", ZMax);
	InterpolationPoints.push_back(CurSectionInterpolationPoints);

	//The missing points should be repaired here.

	k = InterpolationPoints.size();				//k行插值点
	l = InterpolationPoints[0]->points.size();	//l列插值点

	RowStep = RowStepValue;
	ColStep = ColStepValue;
	HStep = HStepValue;

	ConstructClosedCurvePoint();
}

void CSurfaceInterpolation::SetInputInterpolationPoints(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
	InterpolationPointsValue, double HStepValue, double RowStepValue, double ColStepValue)
{
	// clear first
	for (int i = 0; i < InterpolationPoints.size(); i++)
	{
		InterpolationPoints[i]->points.clear();
	}
	InterpolationPoints.clear();

	for (int i = 0; i < InterpolationPointsValue.size(); i++)
	{
		InterpolationPoints.push_back(InterpolationPointsValue[i]);
	}
	k = InterpolationPoints.size();				//k行插值点
	l = InterpolationPoints[0]->points.size();	//l列插值点

	RowStep = RowStepValue;
	ColStep = ColStepValue;
	HStep = HStepValue;

	ConstructClosedCurvePoint();
}

//求解 一组全局控制点， 无需闭合曲线
bool CSurfaceInterpolation::ResolveOneGlobalControlPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue,
	vector<double> ParameterValues, vector<double> KnotValues,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPoints)
{
	CSpline RationalSpline;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	QPoints->points.insert(QPoints->points.begin(),
		QPointsValue->points.begin(), QPointsValue->points.end());

	int nSpline = QPoints->points.size();
	RationalSpline.SetSplineInputs(QPoints, dSpline, KnotValues);
	// AX=B
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nSpline, nSpline); //默认是0矩阵
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nSpline, 3);		// 默认是0矩阵

	//对A赋值  需要补充
	for (int i = 0; i < nSpline; i++)	//行
	{	//获取参数所在的位置
		int KnotVectorIndex = RationalSpline.FindUIndex(ParameterValues[i]);
		for (int j = 0; j < nSpline; j++)	//列
		{
			int K = KnotVectorIndex - dSpline + j;
			if (K >= nSpline) continue;

			A(i, K) = RationalSpline.CoxdeBoor(K, dSpline, ParameterValues[i]);
			//cout<<"i: "<<i<<" k:"<<K<<" d:"<<dSpline<<" u:"<<ParameterValues[i]<<" CoxdeBoor:"<<A(i,K)<<endl;
		}
	}
	A(nSpline - 1, nSpline - 1) = 1;

	for (int i = 0; i < nSpline; i++)
	{
		B(i, 0) = QPoints->points[i].x;
		B(i, 1) = QPoints->points[i].y;
		B(i, 2) = QPoints->points[i].z;
	}	

	Eigen::MatrixXd X = Math::ResloveLinearEquationsByLUDecomposition(A, B);

	//将求解结果转为控制点
	bool IsRight = true;
	ControlPoints->points.clear();
	for (int i = 0; i < nSpline; i++)
	{
		pcl::PointXYZRGB TempPoint;		
		TempPoint.x = X(i, 0);
		TempPoint.y = X(i, 1);
		TempPoint.z = X(i, 2);
		
		if (isnan(TempPoint.x) || isnan(TempPoint.y) || isnan(TempPoint.z))
		{
			IsRight = false;
		}

		TempPoint.rgba = ColorBase::RedColor;
		ControlPoints->points.push_back(TempPoint);
	}
	return IsRight;
}

//根据插值点集合的第一个节点与 ColStep 创建的闭合曲线
void CSurfaceInterpolation::ConstructClosedCurvePoint()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloseInterpolationPoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloseControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

	vector<double> KnotValues;

	for (int i = 0; i < InterpolationPoints.size(); i++)
	{
		CloseInterpolationPoints->points.push_back(InterpolationPoints[i]->points[0]);
	}

	CSplineInterpolation SplineInterpolation;
	SplineInterpolation.SetInputs(CloseInterpolationPoints, 3);
	SplineInterpolation.GetControlPointsAndKnotValue(CloseControlPoints, KnotValues);

	CSpline Spline;
	Spline.SetSplineInputs(CloseControlPoints, 3, KnotValues);
	Spline.UStep = ColStep / 2;
	Spline.CreateSpline();	

	ClosedReferenceCurvePoint->points.clear();
	ClosedReferenceCurvePoint->points.insert(ClosedReferenceCurvePoint->points.end(),
		Spline.CurvePoints->points.begin(),
		Spline.CurvePoints->points.end());
	
	PointBase::SetPointColor(ClosedReferenceCurvePoint, ColorBase::RedColor);
}

void CSurfaceInterpolation::InterpolationSurface()
{
	for each (pcl::PointCloud<pcl::PointXYZRGB>::Ptr var in RowControlPoints)
	{
		var->points.clear();		
	}

	RowControlPoints.clear();
	KnotValuesRow.clear();
	ParametersRow.clear();	

	//对每一行的插值点进行处理 计算得到每一行的控制点及节点向量  横向插值
	for (int i = 0; i < InterpolationPoints.size(); i++)
	{
		vector<double> TempKnotValuesV;
		KnotValuesRow.push_back(TempKnotValuesV);
		ParametersRow.push_back(TempKnotValuesV);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp(new pcl::PointCloud<pcl::PointXYZRGB>());
		RowControlPoints.push_back(Temp);
	}

	#pragma omp parallel for
	for (int i = 0; i < InterpolationPoints.size(); i++)
	{
		cout << "正在计算第" << i << "条横向曲线的节点参数与节点向量" << endl;
		//对当前行进行处理，得到参数值ParametersV 与节点向量 KnotValuesV
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		vector<double> TempKnotValuesV;

		CSplineInterpolation SplineInterpolation;
		//SplineInterpolation.SetInputs(InterpolationPoints[i], 3, true, true);		

		if (HorizontalIsClosed)	//横向闭合曲线
			SplineInterpolation.SetInputs(InterpolationPoints[i], 3, true, true);
		else					//横向非闭合曲线
			SplineInterpolation.SetInputs(InterpolationPoints[i], 3, false, true);

		//最后一个参数为 false 表明 不使用向心参数法
		bool IsRowRight = SplineInterpolation.GetControlPointsAndKnotValue(TempControlPoints, TempKnotValuesV, false);

		if (!IsRowRight)
		{
			cout << "计算第" << i << "条横向曲线的控制点时出现错误！" << endl;
			//PointBase::SavePCDToFileName(InterpolationPoints[i], "I:\\Error\\RowControl_" + StringBase::IntToStr(i) + ".pcd");
		}

		KnotValuesRow.push_back(TempKnotValuesV);
		ParametersRow.push_back(SplineInterpolation.UParameters);

		KnotValuesRow[i].clear();
		KnotValuesRow[i].insert(KnotValuesRow[i].begin(), TempKnotValuesV.begin(), TempKnotValuesV.end());
		ParametersRow[i].clear();
		ParametersRow[i].insert(ParametersRow[i].begin(), SplineInterpolation.UParameters.begin(), SplineInterpolation.UParameters.end());

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp (new pcl::PointCloud<pcl::PointXYZRGB>());
		Temp->points.insert(Temp->points.end(),
			TempControlPoints->points.begin(), TempControlPoints->points.end());

		//RowControlPoints.push_back(Temp);
		RowControlPoints[i]->points.clear();		
		RowControlPoints[i]->points.insert(RowControlPoints[i]->points.begin(),	Temp->points.begin(), Temp->points.end());

		if (HorizontalIsClosed)	//横向是闭合曲线
		{
			InterpolationPoints[i]->points.clear();
			InterpolationPoints[i]->points.insert(InterpolationPoints[i]->points.begin(),
				SplineInterpolation.QPoints->points.begin(),
				SplineInterpolation.QPoints->points.end());
		}
	}

	//对每一列的插值点进行处理	计算得到每一列的节点参数 及 节点向量，每一列的控制点将由下一步求解得到
	//此处每一列的插值不需要做	 

	//****** RowControlPoints 的内容会发生变化
	for each (pcl::PointCloud<pcl::PointXYZRGB>::Ptr var in GlobalControlPoints)
	{
		var->points.clear();
	}

	GlobalControlPoints.clear();
	KnotValuesCol.clear();
	ParametersCol.clear();

	for (int j = 0; j < InterpolationPoints[0]->points.size(); j++)
	{
		vector<double> TempKnotValuesU;
		KnotValuesCol.push_back(TempKnotValuesU);
		ParametersCol.push_back(TempKnotValuesU);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);		
		GlobalControlPoints.push_back(TempControlPoints);
	}

	#pragma omp parallel for
	for (int j = 0; j < InterpolationPoints[0]->points.size(); j++)
	{
		cout << "正在计算第" << j << "条纵向曲线的节点参数与节点向量" << endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (int i = 0; i < InterpolationPoints.size(); i++)	//添加一行 对应列的元素
		{
			TempPoints->points.push_back(InterpolationPoints[i]->points[j]);
		}

		//对当前列进行处理，得到参数值ParametersV 与节点向量 KnotValuesV
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempControlPointsA(new pcl::PointCloud<pcl::PointXYZRGB>);
		vector<double> TempKnotValuesU;

		CSplineInterpolation SplineInterpolation;
		//不构建闭合曲线
		SplineInterpolation.SetInputs(TempPoints, 3, false, false);

		//最后一个参数为 false 表明 不使用向心参数法
		bool IsColRight = SplineInterpolation.GetControlPointsAndKnotValue(TempControlPointsA, TempKnotValuesU, false);
		if (!IsColRight)
		{
			cout << "计算第" << j << "条纵向曲线的控制点时出现错误！" << endl;
			//PointBase::SavePCDToFileName(TempPoints, "I:\\Error\\ColControl_" + StringBase::IntToStr(j) + ".pcd");
		}

		KnotValuesCol[j].clear();
		KnotValuesCol[j].insert(KnotValuesCol[j].begin(), TempKnotValuesU.begin(), TempKnotValuesU.end());
		//KnotValuesCol.push_back(TempKnotValuesU);
		//ParametersCol.push_back(SplineInterpolation.UParameters);
		ParametersCol[j].clear();
		ParametersCol[j].insert(ParametersCol[j].begin(), SplineInterpolation.UParameters.begin(), SplineInterpolation.UParameters.end());
	}

	//下一步是根据已得到的节点向量 KnotValuesU 与 节点参数 ParametersU 
	// 与 插值点 RowControlPoints 计算得到全局控制点GlobalControlPoints

	#pragma omp parallel for
	for (int j = 0; j < InterpolationPoints[0]->points.size(); j++)
	{	//原始控制点的列数，
		//cout << "正在计算第" << j << "条纵向曲线的控制点" << endl;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempQPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (int i = 0; i < RowControlPoints.size(); i++)	//第i行，第j列(j值固定)
		{
			//TempQPoints->points.push_back(RowControlPoints[i]->points[j + 3]); //是闭曲线 有多余的3个节点
			TempQPoints->points.push_back(RowControlPoints[i]->points[j]); //是闭曲线 有多余的3个节点
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		bool IsSurfaceRight = ResolveOneGlobalControlPoints(TempQPoints, ParametersCol[j], KnotValuesCol[j], TempControlPoints);
		
		if (!IsSurfaceRight)
		{
			cout << "计算Spline曲面第" << j << "列纵向控制点时出现错误！" << endl;
			//PointBase::SavePCDToFileName(TempQPoints, "I:\\Error\\SurfaceControl_" + StringBase::IntToStr(j) + ".pcd");
		}

		//GlobalControlPoints.push_back(TempControlPoints);
		GlobalControlPoints[j]->points.clear();
		GlobalControlPoints[j]->points.insert(GlobalControlPoints[j]->points.begin(), TempControlPoints->points.begin(), TempControlPoints->points.end());
	}
}

//根据u值返回 u 的字符串公式表达式 2016.12.13 根据具体的U值 与 VValue 值计算曲面在特定VValue值下的表达式
vector<string> CSurfaceInterpolation::GetUVFormula(double RowU, double ColV)
{
	vector<string> ResultStrS;

	vector<string> ResultStrU;

	ResultStrS.push_back("");
	ResultStrS.push_back("");
	ResultStrS.push_back("");
	//OutPutPoints->points.clear();

	CalcBase<double> CalcBaseFloat;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RowControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	CSpline RowSpline, ColSpline;

	//全局插值的结果 是 列
	ColControlPoints->points.insert(ColControlPoints->points.begin(),
		GlobalControlPoints[0]->points.begin(), GlobalControlPoints[0]->points.end());

	for (int i = 0; i < GlobalControlPoints.size(); i++)
	{
		RowControlPoints->points.push_back(GlobalControlPoints[i]->points[0]);
		//RowControlPoints->points.push_back(GlobalControlPoints[i]->points[RowIndex]);
	}

	//if ((RowIndex % 3) == 0)
	//	PointBase::SetPointColor(RowControlPoints, ColorBase::RedColor);
	//else if ((RowIndex % 3) == 1)
	//	PointBase::SetPointColor(RowControlPoints, ColorBase::BlueColor);
	//else if ((RowIndex % 3) == 2)
	//	PointBase::SetPointColor(RowControlPoints, ColorBase::GreenColor);

	//ShowPoints(RowControlPoints, "RowControlPoints" + StringBase::ClockValue(), 5);

	RowSpline.SetSplineInputs(RowControlPoints, dSpline, KnotValuesRow[0]);
	//RowSpline.SetSplineInputs(RowControlPoints, dSpline, KnotValuesRow[RowIndex]);
	ColSpline.SetSplineInputs(ColControlPoints, dSpline, KnotValuesCol[0]);

	int RowKnotVectorIndex = RowSpline.FindUIndex(RowU);
	int ColKnotVectorIndex = ColSpline.FindUIndex(ColV);

	for (int Rowj = 0; Rowj < dSpline + 1; Rowj++) //样条上的点使用前 dSpline + 1 个控制点
	{
		//int K = KnotVectorIndex - d + j + 1;
		int RowK = RowKnotVectorIndex - dSpline + Rowj;

		double RowB = RowSpline.CoxdeBoor(RowK, dSpline, RowU);
		string FromulaRowU = RowSpline.CoxdeBoorFormula(RowK, dSpline, RowU);
		//cout<<"k:"<<K<<" d:"<<dSpline<<" u:"<<u<<" B:"<<B<<endl;

		for (int Colj = 0; Colj < dSpline + 1; Colj++) //样条上的点使用前 dSpline + 1 个控制点
		{
			//int K = KnotVectorIndex - d + j + 1;
			int ColK = ColKnotVectorIndex - dSpline + Colj;

			double ColB = ColSpline.CoxdeBoor(ColK, dSpline, ColV);

			if (ResultStrS[0] == "")
			{
				ResultStrS[0] = CalcBaseFloat.ConvertToString(GlobalControlPoints[RowK]->points[ColK].x * ColB)
					+ "*(" + FromulaRowU + ")";

				ResultStrS[1] = CalcBaseFloat.ConvertToString(GlobalControlPoints[RowK]->points[ColK].y * ColB)
					+ "*(" + FromulaRowU + ")";

				ResultStrS[2] = CalcBaseFloat.ConvertToString(GlobalControlPoints[RowK]->points[ColK].z * ColB)
					+ "*(" + FromulaRowU + ")";
			}
			else
			{
				if (GlobalControlPoints[RowK]->points[ColK].x > 0)
					ResultStrS[0] = ResultStrS[0] + " + "
					+ CalcBaseFloat.ConvertToString(GlobalControlPoints[RowK]->points[ColK].x * ColB)
					+ "*(" + FromulaRowU + ")";
				else
					ResultStrS[0] = ResultStrS[0]
					+ CalcBaseFloat.ConvertToString(GlobalControlPoints[RowK]->points[ColK].x * ColB)
					+ "*(" + FromulaRowU + ")";

				if (GlobalControlPoints[RowK]->points[ColK].y > 0)
					ResultStrS[1] = ResultStrS[1] + " + "
					+ CalcBaseFloat.ConvertToString(GlobalControlPoints[RowK]->points[ColK].y * ColB)
					+ "*(" + FromulaRowU + ")";
				else
					ResultStrS[1] = ResultStrS[1]
					+ CalcBaseFloat.ConvertToString(GlobalControlPoints[RowK]->points[ColK].y * ColB)
					+ "*(" + FromulaRowU + ")";

				if (GlobalControlPoints[RowK]->points[ColK].z > 0)
					ResultStrS[2] = ResultStrS[2] + " + "
					+ CalcBaseFloat.ConvertToString(GlobalControlPoints[RowK]->points[ColK].z * ColB)
					+ "*(" + FromulaRowU + ")";
				else
					ResultStrS[2] = ResultStrS[2]
					+ CalcBaseFloat.ConvertToString(GlobalControlPoints[RowK]->points[ColK].z * ColB)
					+ "*(" + FromulaRowU + ")";
			}
			
			/*if (OutPutPoints != NULL)
			{
				pcl::PointXYZRGB TempPoint;
				TempPoint.x = 0;
				TempPoint.y = 0;
				TempPoint.z = 0;
				TempPoint.rgba = ColorBase::RedColor;

				TempPoint.x = TempPoint.x + GlobalControlPoints[RowK]->points[ColK].x * RowB * ColB;
				TempPoint.y = TempPoint.y + GlobalControlPoints[RowK]->points[ColK].y * RowB * ColB;
				TempPoint.z = TempPoint.z + GlobalControlPoints[RowK]->points[ColK].z * RowB * ColB;

				OutPutPoints->points.push_back(TempPoint);
			}*/
		}
	}

	//cout<<"当前点坐标为"<<TempPoint<<endl;

	return ResultStrS;
}


//根据 RowIndex和 ColIndex 索引 与 节点向量 RowU 和 ColV 得到曲面上的点
pcl::PointXYZRGB CSurfaceInterpolation::GetSurfacePoint(int RowIndex, int ColIndex, double RowU, double ColV)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RowControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	CSpline RowSpline, ColSpline;

	//全局插值的结果 是 列
	ColControlPoints->points.insert(ColControlPoints->points.begin(),
		GlobalControlPoints[ColIndex]->points.begin(), GlobalControlPoints[ColIndex]->points.end());

	for (int i = 0; i < GlobalControlPoints.size(); i++)
	{
		RowControlPoints->points.push_back(GlobalControlPoints[i]->points[RowIndex]);
	}

	RowSpline.SetSplineInputs(RowControlPoints, dSpline, KnotValuesRow[RowIndex]);
	ColSpline.SetSplineInputs(ColControlPoints, dSpline, KnotValuesCol[ColIndex]);

	int RowKnotVectorIndex = RowSpline.FindUIndex(RowU);
	int ColKnotVectorIndex = ColSpline.FindUIndex(ColV);

	pcl::PointXYZRGB TempPoint;
	TempPoint.x = 0;
	TempPoint.y = 0;
	TempPoint.z = 0;
	TempPoint.rgba = ColorBase::RedColor;

	for (int Rowj = 0; Rowj < dSpline + 1; Rowj++) //样条上的点使用前 dSpline + 1 个控制点
	{
		//int K = KnotVectorIndex - d + j + 1;
		int RowK = RowKnotVectorIndex - dSpline + Rowj;

		double RowB = RowSpline.CoxdeBoor(RowK, dSpline, RowU);
		//cout<<"k:"<<K<<" d:"<<dSpline<<" u:"<<u<<" B:"<<B<<endl;

		for (int Colj = 0; Colj < dSpline + 1; Colj++) //样条上的点使用前 dSpline + 1 个控制点
		{
			//int K = KnotVectorIndex - d + j + 1;
			int ColK = ColKnotVectorIndex - dSpline + Colj;

			double ColB = ColSpline.CoxdeBoor(ColK, dSpline, ColV);

			TempPoint.x = TempPoint.x + GlobalControlPoints[RowK]->points[ColK].x * RowB * ColB;
			TempPoint.y = TempPoint.y + GlobalControlPoints[RowK]->points[ColK].y * RowB * ColB;
			TempPoint.z = TempPoint.z + GlobalControlPoints[RowK]->points[ColK].z * RowB * ColB;
		}
	}

	return TempPoint;
}

int CSurfaceInterpolation::FindUIndex(double u, vector<double> KnotValues, int StartIndex, int EndIndex)
{
	if (EndIndex >= KnotValues.size())
		cout << endl;
	double Minus = abs(u - KnotValues[EndIndex]);
	if (Minus <= eps) return EndIndex - 1;
	int low = StartIndex;
	int high = EndIndex;

	int Index = (low + high) / 2;
	while ((u < KnotValues[Index] || u >= KnotValues[Index + 1]) && (low != high))
	{
		if (u < KnotValues[Index]) high = Index;
		else low = Index;
		Index = (low + high) / 2;
	}
	return Index;
}


//在构建的闭合曲线的控制点上找 距离 ColU 所控制节点高度位置处的 最近点	
pcl::PointXYZRGB CSurfaceInterpolation::FindHNearestPoint(double RowU, double ColU)
{
	pcl::PointXYZRGB NearestPoint;

	pcl::PointXYZRGB SurfacePoint = GetSurfacePoint(0, 0, RowU, ColU);

	double ZMinus = abs(ClosedReferenceCurvePoint->points[0].z - SurfacePoint.z);
	NearestPoint = ClosedReferenceCurvePoint->points[0];

	for (int i = 1; i < ClosedReferenceCurvePoint->points.size(); i++)
	{
		if (abs(ClosedReferenceCurvePoint->points[i].z - SurfacePoint.z) < ZMinus)
		{
			ZMinus = abs(ClosedReferenceCurvePoint->points[i].z - SurfacePoint.z);
			NearestPoint = ClosedReferenceCurvePoint->points[i];
		}
	}

	return NearestPoint;
}

//根据参数值 在 参数空间中 寻找 索引  此种方法不准确
void CSurfaceInterpolation::FindUVIndex(double RowU, double ColV, int & RowIndex, int & ColIndex)
{
	//列的索引 根据行的节点参数向量来寻找
	ColIndex = FindUIndex(RowU, KnotValuesRow[0], dSpline, InterpolationPoints[0]->points.size()) - dSpline;

	RowIndex = FindUIndex(ColV, KnotValuesCol[ColIndex], dSpline, InterpolationPoints.size()) - dSpline;
}

//给定列的节点向量，寻找在该列位置处 构建闭合曲线的 起始 StartU 与 EndU；
void CSurfaceInterpolation::FindStartUandEndU(int RowIndex, double ColU,
	double & StartRowU, double & EndRowU, double DisError, double UStep)
{
	//// ColU = 1 时会出错

	double RowU = 0;
	bool FindFirstNode = false;
	pcl::PointXYZRGB CloseNearestPoint = FindHNearestPoint(RowU, ColU);
	StartRowU = 0; EndRowU = 1;
	CalcBase<float> CalcBaseFloat;
	double LastDis = 100, LastU;

	while (RowU < KnotValuesRow[RowIndex][dSpline + dSpline])
	{
		pcl::PointXYZRGB NextPoint = GetSurfacePoint(0, 0, RowU, ColU);
		double TempDis = PointDis(NextPoint.x, NextPoint.y, NextPoint.z,
			CloseNearestPoint.x, CloseNearestPoint.y, CloseNearestPoint.z);

		if (TempDis < DisError)
		{
			StartRowU = RowU;
			break;
		}
		else if (TempDis < LastDis)
		{
			LastDis = TempDis;
			LastU = RowU;
		}
		else if (TempDis > LastDis)
		{			
			StartRowU = RowU;
			break;
		}

		RowU = RowU + UStep;
	}

	int RowKnotEndIndex = GlobalControlPoints.size();
	RowU = KnotValuesRow[RowIndex][RowKnotEndIndex - dSpline];

	LastDis = 100;
	while (RowU >= KnotValuesRow[RowIndex][RowKnotEndIndex - dSpline] && RowU < 1)
	{
		pcl::PointXYZRGB NextPoint = GetSurfacePoint(0, 0, RowU, ColU);
		double TempDis = PointDis(NextPoint.x, NextPoint.y, NextPoint.z,
			CloseNearestPoint.x, CloseNearestPoint.y, CloseNearestPoint.z);

		if (TempDis < DisError)
		{
			EndRowU = RowU;
			break;
		}
		else if (TempDis < LastDis)
		{
			LastDis = TempDis;
			LastU = RowU;
		}
		else if (TempDis > LastDis)
		{
			EndRowU = RowU;
			break;
		}

		RowU = RowU + UStep;
	}
}

//根据高度Z值获取对应位置处的 横断面轮廓曲线
void CSurfaceInterpolation::GetCrossSectionalCurveByHeight(double Z, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr HeightPoints,
	double DisError, double UStep)
{
	int ColKnotStartIndex, ColKnotEndIndex;
	ColKnotStartIndex = dSpline;
	ColKnotEndIndex = GlobalControlPoints[0]->points.size();

	double StartRowU = 0, ColU = 0, FindColU = 0;
	ColU = KnotValuesCol[0][ColKnotStartIndex];
	while (ColU < 1)
	{
		pcl::PointXYZRGB NextPoint = GetSurfacePoint(0, 0, 0, ColU);
		if (abs(NextPoint.z - Z) < DisError)
		{
			FindColU = ColU;
			break;
		}
		ColU = ColU + UStep;
	}

	double EndRowU = 1;
	//FindColU = 1;
	FindStartUandEndU(0, FindColU, StartRowU, EndRowU, 0.1, 0.001);	//找到闭合区域的

	HeightPoints->points.clear();
	while (StartRowU < EndRowU)
	{
		HeightPoints->points.push_back(this->GetSurfacePoint(0, 0, StartRowU, FindColU));
		StartRowU = StartRowU + UStep;
	}
}

double CSurfaceInterpolation::CalcZoneValueByThread(vector<string> FormulaS, double TempUStart, double TempUEnd)
{
	CFormulaAnalytic FormulaAnalytic;

	FormulaAnalytic.SetInputs(FormulaS[0]);
	string FormulaStrX = FormulaAnalytic.GetFormula();

	FormulaAnalytic.SetInputs(FormulaS[1]);
	string FormulaStrY = FormulaAnalytic.GetFormula();

	string DFormulaStrX = FormulaAnalytic.DerivativeStr(FormulaStrX);
	string DFormulaStrY = FormulaAnalytic.DerivativeStr(FormulaStrY);

	string Formula;
	//此段是根据格式公式P = -y, Q = x
	Formula = "((" + FormulaStrX + ")*(" + DFormulaStrY + "))-(("
		+ FormulaStrY + ")*(" + DFormulaStrX + "))";

	////此段是根据格式公式P = 0, Q = x
	Formula = "2*((" + FormulaStrX + ")*(" + DFormulaStrY + "))";

	////此段是根据格式公式P = y, Q = 0
	Formula = "-2*((" + FormulaStrY + ")*(" + DFormulaStrX + "))";

	FormulaAnalytic.SetInputs(Formula);
	Formula = FormulaAnalytic.GetFormula();

	Formula = FormulaAnalytic.PolynomialMerge(Formula);
	return FormulaAnalytic.ResolveIntegralValue(Formula, TempUStart, TempUEnd);
}

//根据给定高度寻找 ColU 的值 2019.07.10
double CSurfaceInterpolation::FindColUByHeight(double Z)
{
	double Lower = KnotValuesCol[0][dSpline];
	double Upper = 1;	
	
	///*
	double Height = ZMax - ZMin;
	Lower = Z - Height / 20.0;
	Upper = Z + Height / 20;

	if (Lower <= 0) Lower = KnotValuesCol[0][dSpline];
	else Lower = Lower / Height;

	if (Upper >= ZMax) Upper = 1;
	else Upper = Upper / Height;

	///* 0.618法
	double Range = Upper - Lower;
	double Start_Try, End_Try;
	Start_Try = Lower + 0.312 * Range;
	End_Try = Lower + 0.618 * Range;	

	pcl::PointXYZRGB StartPoint = GetSurfacePoint(0, 0, 0, Start_Try);
	pcl::PointXYZRGB EndPoint = GetSurfacePoint(0, 0, 0, End_Try);

	while (abs(StartPoint.z - Z) > EPSM3 || abs(EndPoint.z - Z) > EPSM3)
	{
		StartPoint = GetSurfacePoint(0, 0, 0, Start_Try);
		EndPoint   = GetSurfacePoint(0, 0, 0, End_Try);
		//cout <<"Z:" << Z << " StartPoint" << StartPoint << endl;
		//cout <<"Z:" << Z << " EndPoint"   << EndPoint   << endl;
		
		if (abs(StartPoint.z - Z) <= EPSM3)
			return Start_Try;
		else if (abs(EndPoint.z - Z) <= EPSM3)
			return End_Try;
				
		if (EndPoint.z > Z || (StartPoint.z + EndPoint.z) / 2.0 > Z )
		{
			Upper = End_Try;
			Range = Upper - Lower;
			Start_Try = Lower + 0.312 * Range;
			End_Try = Lower + 0.618 * Range;
		}
		else
		{
			Lower = Start_Try;
			Range = Upper - Lower;
			Start_Try = Lower + 0.312 * Range;
			End_Try = Lower + 0.618 * Range;
		}

		if (abs(End_Try - Start_Try) < EPSM3) break;
	}

	StartPoint = GetSurfacePoint(0, 0, 0, Start_Try);
	EndPoint = GetSurfacePoint(0, 0, 0, End_Try);

	if (abs(StartPoint.z - Z) > abs(EndPoint.z - Z))
		return End_Try;
	else return Start_Try;

	// 0.618法*//
}

//根据高度Z值获取对应位置处的 横断面轮廓曲线 及其对应的面积
double CSurfaceInterpolation::CalcAreaByHeight(double Z, int RowIndex, double DisError)
{
	int ColKnotStartIndex, ColKnotEndIndex;
	ColKnotStartIndex = dSpline;
	ColKnotEndIndex = GlobalControlPoints[0]->points.size();

	double StartRowU = 0, ColU = 0, UStep = EPSM6, FindColU = 0;
	ColU = KnotValuesCol[0][ColKnotStartIndex];

	FindColU = FindColUByHeight(Z);

	//cout<< "Height Z:"<<Z <<", FindColU:"<< FindColU <<", Point:"<<GetSurfacePoint(0,0,0, FindColU)<<endl;

	double EndRowU = 1;
	//FindColU = 1;
	//cout<< "RowIndex:"<<RowIndex <<", FindColU:"<< FindColU <<endl;
	FindStartUandEndU(0, FindColU, StartRowU, EndRowU, 0.1, 0.001);	//找到闭合区域的
	//FindStartUandEndU(RowIndex, FindColU, StartRowU, EndRowU, 0.1, 0.001);	//找到闭合区域的

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RowControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	CSpline RowSpline;

	for (int i = 0; i < GlobalControlPoints.size(); i++)
	{
		RowControlPoints->points.push_back(GlobalControlPoints[i]->points[RowIndex]);
	}

	//if ((RowIndex % 3) == 0)
	//	PointBase::SetPointColor(RowControlPoints, ColorBase::RedColor);
	//else if ((RowIndex % 3) == 1)
	//	PointBase::SetPointColor(RowControlPoints, ColorBase::BlueColor);
	//else if ((RowIndex % 3) == 2)
	//	PointBase::SetPointColor(RowControlPoints, ColorBase::GreenColor);

	//ShowPoints(RowControlPoints, "RowControlPoints" + StringBase::ClockValue(), 5);
	
	RowSpline.SetSplineInputs(RowControlPoints, dSpline, KnotValuesRow[RowIndex]);

	int StartUZoneIndex = RowSpline.FindUIndex(StartRowU);
	int EndUZoneIndex = RowSpline.FindUIndex(EndRowU);

	double Area = 0, TempUStart = 0, TempUEnd = 1;

	vector<double> SumVolumes;
	for (int i = StartUZoneIndex; i <= EndUZoneIndex; i++) //每一段	
	{
		SumVolumes.push_back(0);
	}
	#pragma omp parallel for
	for (int i = StartUZoneIndex; i <= EndUZoneIndex; i++) //每一段	
	{
		if (i == StartUZoneIndex)
		{
			TempUStart = StartRowU;
			TempUEnd = KnotValuesRow[RowIndex][i + 1];
		}
		else if (i == EndUZoneIndex)
		{
			TempUStart = KnotValuesRow[RowIndex][EndUZoneIndex];
			TempUEnd = EndRowU;
		}
		else
		{
			TempUStart = KnotValuesRow[RowIndex][i];
			TempUEnd = KnotValuesRow[RowIndex][i + 1];
		}

		double UMiddle = (TempUStart + TempUEnd) / 2;

		vector<string> FormulaS = GetUVFormula(UMiddle, FindColU);		

		CFormulaAnalytic FormulaAnalytic;

		FormulaAnalytic.SetInputs(FormulaS[0]);
		string FormulaStrX = FormulaAnalytic.GetFormula();

		FormulaAnalytic.SetInputs(FormulaS[1]);
		string FormulaStrY = FormulaAnalytic.GetFormula();

		string DFormulaStrX = FormulaAnalytic.DerivativeStr(FormulaStrX);
		string DFormulaStrY = FormulaAnalytic.DerivativeStr(FormulaStrY);			

		string Formula;
		//此段是根据格式公式P = -y, Q = x
		Formula = "((" + FormulaStrX + ")*(" + DFormulaStrY + "))-(("
			+ FormulaStrY + ")*(" + DFormulaStrX + "))";

		////此段是根据格式公式P = 0, Q = x
		Formula = "2*((" + FormulaStrX + ")*(" + DFormulaStrY + "))";

		////此段是根据格式公式P = y, Q = 0
		Formula = "-2*((" + FormulaStrY + ")*(" + DFormulaStrX + "))";

		FormulaAnalytic.SetInputs(Formula);
		Formula = FormulaAnalytic.GetFormula();	

		Formula = FormulaAnalytic.PolynomialMerge(Formula);
		double TempArea = FormulaAnalytic.ResolveIntegralValue(Formula, TempUStart, TempUEnd);
		//*/
		//double TempArea = CalcZoneValueByThread(FormulaS, TempUStart, TempUEnd);
		//Area = Area + TempArea;
		SumVolumes[i - StartUZoneIndex] = TempArea;
	}

	for (int i = StartUZoneIndex; i <= EndUZoneIndex; i++) //每一段	
	{
		Area += SumVolumes[i - StartUZoneIndex];
	}

	return Area / 2;
}

void CSurfaceInterpolation::GetSurfacePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutSurfacePoints, 
	double UStep, double VStep)
{
	int RowKnotStartIndex, RowKnotEndIndex;
	int ColKnotStartIndex, ColKnotEndIndex;

	OutSurfacePoints->points.clear();
	for each (pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp in SurfacePoints)
	{
		Temp->points.clear();
	}
	SurfacePoints.clear();

	RowKnotStartIndex = dSpline;
	RowKnotEndIndex = GlobalControlPoints.size();

	if (RowKnotEndIndex == 0) return;

	ColKnotStartIndex = dSpline;
	ColKnotEndIndex = GlobalControlPoints[0]->points.size();

	double RowU = 0, ColU = 0;

	CalcBase<float> CalcBaseFloat;
	int i = 0, j = 0;
	int RowIndex = 0, ColIndex = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr RowCurvePoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	ColU = KnotValuesCol[j][ColKnotStartIndex];
	double StartRowU = 0, EndRowU = 1;
	bool FindFirstPoint = false, FindEndPoint = false;
	int OldRowIndex = -1;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr NearestPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	while (ColU < 1)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr SubRowCurvePoints(new pcl::PointCloud<pcl::PointXYZRGB>);

		RowU = KnotValuesRow[RowIndex][RowKnotStartIndex];

		EndRowU = 1;		

		FindStartUandEndU(RowIndex, ColU, RowU, EndRowU, 0.1, 0.001);
		
		while (RowU < EndRowU)
		{
			pcl::PointXYZRGB NextPoint = GetSurfacePoint(0, 0, RowU, ColU);

			SubRowCurvePoints->points.push_back(NextPoint);
					
			RowU = RowU + UStep;
		}

		SurfacePoints.push_back(SubRowCurvePoints);
				
		ColU = ColU + VStep;
	}	

	for (int i = 0; i < SurfacePoints.size(); i++)
	{
		if (i % 3 == 0)
			PointBase::SetPointColor(SurfacePoints[i], ColorBase::RedColor);
		else if (i % 3 == 1)
			PointBase::SetPointColor(SurfacePoints[i], ColorBase::GreenColor);
		else if (i % 3 == 2)
			PointBase::SetPointColor(SurfacePoints[i], ColorBase::BlueColor);
		
		OutSurfacePoints->points.insert(OutSurfacePoints->points.end(),
			SurfacePoints[i]->points.begin(), SurfacePoints[i]->points.end());		
	}
}

//根据定义画插值曲面
void CSurfaceInterpolation::DrawSurfaceByDefinition(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
	double UStep, double VStep)
{	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SurfaceAllPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		
	GetSurfacePoints(SurfaceAllPoints, UStep, VStep);

	cout<<"Num:"<< SurfaceAllPoints->points.size()<<endl;
	PointBase::ShowPointXYZRGB(Viewer, SurfaceAllPoints, StringBase::ClockValue() + "SurfacePoints", 3);

}

double CSurfaceInterpolation::CalcVolume(double StartZ, double EndZ)
{
	//cout << "开始采用曲面模型计算材积" << endl;
	//double LowZ = this->InterpolationPoints[0]->points[0].z;
	//double UpperZ = this->InterpolationPoints[InterpolationPoints.size() - 1]->points[0].z;
	StemVolume = 0;
	if (SplineSurfacePoints != NULL)
		SplineSurfacePoints->points.clear();
	SplineSurfacePoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

	ZMin = StartZ;
	ZMax = EndZ;

	double Z = StartZ;
	int RowIndex = 0;

	int CurI = 0;	
	ExecuteThreadNum = 0;
	
	//int UpperNumber = floor(abs(MaxZ - LowZ) / HStep);
	int UpperNumber = floor(abs(EndZ - StartZ) / HStep);
	//cout<<"UpperNumber:"<< UpperNumber <<endl;
	double ThreadMaxZ = UpperNumber * HStep;
	//cout << "ThreadMaxZ:" << ThreadMaxZ << endl;
	///*
	//while (Z <= ThreadMaxZ)	//插值点是来自投影至平面上的点，因此最高值应加入其中
	vector<double> TempVolume;
	for (int i = 0; i < UpperNumber; i++)
		TempVolume.push_back(0);

	#pragma omp parallel for
	for(int i = 0; i < UpperNumber; i++)
	{
		Z = StartZ + i * HStep;
		RowIndex = (Z - StartZ) / abs(InterpolationPoints[0]->points[0].z - InterpolationPoints[1]->points[0].z);
		TempVolume[i] = CalcAreaByHeight(Z, RowIndex);
		//StemVolume = StemVolume + CalcAreaByHeight(Z, RowIndex);
		//cout <<"RowIndex:"<< RowIndex << ",高度" << Z << "处的体积为:" << TempVolume[i] << endl;
		//cout << ",高度" << Z << "处的体积为:" << TempVolume[i] << endl;
		//emitUpdateStatusBar(("Z:" + StringBase::FloatToStr(Z)).c_str(), 0);
	}	

	StemVolume = 0;
	for (int i = 0; i < UpperNumber; i++)
	{
		StemVolume += TempVolume[i];
		cout << "i:" <<i<<",Volume:"<< TempVolume[i] << endl;
		if (TempVolume[i] < 0)
		{
			cout << "InterpolationPoints Size:" << InterpolationPoints.size()<<endl;
			cout << "InterpolationPoints[i] Size:" << InterpolationPoints[i]->points.size() << endl;
			//PointBase::SavePCDToFileName(InterpolationPoints[i], "I:\\TestMinusVolume" + StringBase::IntToStr(i)+".pcd");
		}
	}
	//*/

	/*
	while (Z < ThreadMaxZ)
	{
		while (ExecuteThreadNum < MaxThreadNum * 10)
		{
			if (Z >= ThreadMaxZ)
				break;
			
			emitUpdateStatusBar(("Z:" + StringBase::FloatToStr(Z)).c_str(), 0);
			ExecuteThreadNum++;
			RowIndex = (Z - LowZ) / abs(InterpolationPoints[0]->points[0].z - InterpolationPoints[1]->points[0].z);
			std::thread ThreadObj(std::bind(&CSurfaceInterpolation::CalcAreaByHeightByThread, this, Z, RowIndex, 0.001));			
			ThreadObj.join();

			Z = Z + HStep;
		}
		Sleep(1000);
	}
	//*/

	//RowIndex = (ThreadMaxZ - LowZ) / abs(InterpolationPoints[0]->points[0].z - InterpolationPoints[1]->points[0].z);	

	double TopHeight = (abs(EndZ - StartZ) - ThreadMaxZ);
	double TopVolume = 0;
	if (TopHeight > abs(InterpolationPoints[0]->points[0].z - InterpolationPoints[1]->points[0].z) / 10.0)
	{ 
		TopVolume = CalcAreaByHeight(ThreadMaxZ, RowIndex);
		cout << "TopVolume:" << TopVolume <<",TopHeight:"<< TopHeight << endl;
		TopVolume *= TopHeight;
	}

	PointBase::SetPointColor(SplineSurfacePoints, ColorBase::RedColor);
	ShowPoints(SplineSurfacePoints, "SplineSurfacePoints", 2);

	return StemVolume * HStep + TopVolume;
}


void CSurfaceInterpolation::ShowInterpolationPoints(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer)
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr SurfaceAllPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < InterpolationPoints.size(); i++)
	{
		//PointsMove(InterpolationPoints[i], 60, 0, 0);
		for(int j = 0; j < InterpolationPoints[i]->points.size(); j++)
		{
			if (j % 3 == 0)
				InterpolationPoints[i]->points[j].rgba = ColorBase::RedColor;
			else if (j % 3 == 1)
				InterpolationPoints[i]->points[j].rgba = ColorBase::GreenColor;
			else if (j % 3 == 2)
				InterpolationPoints[i]->points[j].rgba = ColorBase::BlueColor;
		}		

		SurfaceAllPoints->points.insert(SurfaceAllPoints->points.end(),
			InterpolationPoints[i]->points.begin(), InterpolationPoints[i]->points.end());

		//PointBase::ShowPointXYZRGB(Viewer, SurfacePoints[i], StringBase::ClockValue() + "SurfacePoints", 3);
	}
	PointsMove(SurfaceAllPoints, 60, 0, 0);
	PointBase::ShowPointXYZRGB(Viewer, SurfaceAllPoints, StringBase::ClockValue() + "InterpolationPoints", 3);
}