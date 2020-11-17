//样条插值  根据样条上的点 求样条的控制点与节点向量 2015.09.26

#include "SplineInterpolation.h"


//设置初始参数  IsClosed 闭合  IsSuccessiveValue 连续
void CSplineInterpolation::SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, 
	int dValue, bool IsClosedValue, bool IsSuccessiveValue)
{
	dSpline = dValue;
	IsClosed = IsClosedValue;

	QPoints->points.clear();	
	QPoints->points.insert(QPoints->points.end(), 
		QPointsValue->points.begin(), QPointsValue->points.end());
	
	//先执行弦长均匀化 其实不保形
	//if (ConvexityPreserving)
	//{
	//	HomogenizationSplineInterPolatingPoints();
	//}

	//需要构建连续的样条插值
	if (IsClosed)
	{		
		if (IsSuccessiveValue)
		{
			for(int i = 0; i < dSpline; i++)	
			{
				//尾部加入首部节点
				if (QPointsValue->points.size() > i)	
					QPoints->points.push_back(QPointsValue->points[i]);

				 //首部加入尾部节点
				if (QPointsValue->points.size() > QPointsValue->points.size() - 1 - i)
					QPoints->points.insert(QPoints->points.begin(), 
						QPointsValue->points[QPointsValue->points.size() - 1 - i]);
			}
		}
		else
		{	// //首部加入尾部节点
			if (QPointsValue->points.size() > QPointsValue->points.size() - 1)
				QPoints->points.insert(QPoints->points.end(), 
					QPointsValue->points[0]);
		}
	}

	nSpline = QPoints->points.size();	
}

////获取非凸区域的起始范围 2015.12.22 只针对三次曲线
//bool CSplineInterpolation::GetNoConvexHullZone(int i, vector<double> KnotVector,		
//		double & NoConvexHullUStart, double & NoConvexHullUEnd)
//{
//	if(dSpline != 3){cout<<"此方法只针对于三次曲线"<<endl;	return true;}
//	
//	if (i >= nSpline || i < dSpline) 
//	{	cout<<"超出了节点向量的取值范围"<<endl; return true;}
//	
//	double UStart, UEnd;
//	bool ReturnValue = false;
//	UStart = KnotVector[i];
//	UEnd = KnotVector[i+1];
//
//	vector<string> FormulaS = RationalSpline.GetUFormula((UStart+UEnd)/2);
//
//	string FormulaStrX = FormulaS[0];
//	string FormulaStrY = FormulaS[1];
//
//	CFormulaAnalytic FormulaAnalytic;
//
//	FormulaAnalytic.SetInputs(FormulaStrX);
//	FormulaStrX = FormulaAnalytic.GetFormula();	
//
//	FormulaAnalytic.SetInputs(FormulaStrY);
//	FormulaStrY = FormulaAnalytic.GetFormula();
//
//	vector<double> XCoefficient, YCoefficient;
//	vector<int> XOrder, YOrder;
//	FormulaAnalytic.ExtractCofficientAndVairantOrder(FormulaStrX, XCoefficient, XOrder);
//	FormulaAnalytic.ExtractCofficientAndVairantOrder(FormulaStrY, YCoefficient, YOrder);
//
//	if (XCoefficient.size() > dSpline + 1)
//		cout<<"多项式混合计算错误，X表达式的阶数"<<XCoefficient.size()
//			<<"大于样条指定的阶数"<<dSpline<<endl;
//	else if(XCoefficient.size() < dSpline + 1)
//	{
//		int i = 0; 
//		while(i < XOrder.size())
//		{
//			if (XOrder[i] != (dSpline - i))
//			{
//				XOrder.insert(XOrder.begin() + i, (dSpline - i));
//				XCoefficient.insert(XCoefficient.begin() + i, 0);
//			}
//			i++;
//		}
//	}
//
//	if (YCoefficient.size() > dSpline + 1)
//		cout<<"多项式混合计算错误，Y表达式的阶数"<<XCoefficient.size()
//			<<"大于样条指定的阶数"<<dSpline<<endl;
//	else if(YCoefficient.size() < dSpline + 1)
//	{
//		int i = 0; 
//		while(i < YOrder.size())
//		{
//			if (YOrder[i] != (dSpline - i))
//			{
//				YOrder.insert(YOrder.begin() + i, (dSpline - i));
//				YCoefficient.insert(YCoefficient.begin() + i, 0);
//			}
//			i++;
//		}
//	}
//	
//	double a = 6*(YCoefficient[0]*XCoefficient[1] - XCoefficient[0]*YCoefficient[1]);	
//	double b = 6*(YCoefficient[0]*XCoefficient[2] - XCoefficient[0]*YCoefficient[2]);
//	double c = 2*(YCoefficient[1]*XCoefficient[2] - XCoefficient[1]*YCoefficient[2]);
//
//	double d = (b*b-4*a*c);
//	if (d < 0 && a > 0)
//		ReturnValue = true;
//		//cout<<"此区间无凹性区域"<<endl;
//	else if(d > 0 && a > 0)
//	{
//		double RootLeft, RootRight;
//		RootLeft = (-b-sqrt(d))/2/a;
//		RootRight = (-b+sqrt(d))/2/a;
//		if (UEnd < RootLeft || UStart > RootRight)
//		{	
//			cout<<"此区间无凹性区域"<<endl;
//			ReturnValue = true;
//		}
//		else if( UStart > RootLeft &&  UEnd < RootRight)
//		{
//			ReturnValue = false;
//			NoConvexHullUStart = UStart;
//			NoConvexHullUEnd = UEnd;
//			cout<<"此区间全部是凹性区域"<<endl;
//		}
//		else
//		{
//			if( UStart > RootLeft &&  UStart < RootRight)			
//			{
//				cout<<"第"<<i<<"个区间凹性区域为 "<< UStart<<" 至 "<<RootRight<<endl;
//				ReturnValue = false;
//				NoConvexHullUStart = UStart;
//				NoConvexHullUEnd = RootRight;
//			}
//			else if( UEnd > RootLeft &&  UEnd < RootRight)
//			{
//				cout<<"第"<<i<<"个区间凹性区域为 "<< RootLeft<<" 至 "<<UEnd<<endl;
//				ReturnValue = false;
//				NoConvexHullUStart = RootLeft;
//				NoConvexHullUEnd = UEnd;
//			}
//		}
//	}
//	else if (d > 0 && a < 0)
//	{
//		double RootLeft, RootRight;
//		RootLeft = (-b+sqrt(d))/2/a;
//		RootRight = (-b-sqrt(d))/2/a;
//		if (UStart > RootLeft && UEnd < RootRight)
//		{
//			cout<<"第"<<i<<"个区间无凹性区域"<<endl;
//			ReturnValue = true;
//		}
//		else if (UStart < RootLeft && UEnd > RootLeft)
//		{
//			cout<<"第"<<i<<"个区间凹性区域为 "<<UStart<<" 至 "<<RootLeft<<endl;
//			ReturnValue = false;
//			NoConvexHullUStart = UStart;
//			NoConvexHullUEnd = RootLeft;
//		}
//		else if (UStart < RootLeft && UEnd > RootRight)	//分两步进行
//		{
//			cout<<"第"<<i<<"个区间凹性区域为 "<<UStart<<" 至 "<<RootLeft <<" 且 包括"<<RootRight<<" 至 "<<UEnd<<endl;
//			ReturnValue = false;
//			NoConvexHullUStart = UStart;
//			NoConvexHullUEnd = RootLeft;
//		}
//		else if (UStart < RootRight && UEnd > RootRight)
//		{
//			cout<<"第"<<i<<"个区间凹性区域为 "<<RootRight<<" 至 "<<UEnd<<endl;
//			ReturnValue = false;
//			NoConvexHullUStart = RootRight;
//			NoConvexHullUEnd = UEnd;
//		}	
//		else if (RootRight < UStart ||  UEnd < RootLeft)
//		{
//			ReturnValue = false;
//			NoConvexHullUStart = UStart;
//			NoConvexHullUEnd = UEnd;
//			cout<<"此区间全部是凹性区域"<<endl;
//		}
//	}
//	return ReturnValue;
//}



//获取凸性闭合的三次样条曲线的控制点与节点向量//  *****方法(缩小U的值，插入非凸中间点) 行不通*****
//2015.12.26 调整为迭代缩小参数值的方法
//pcl::PointXYZRGB CSplineInterpolation::GetConvexHullAndClosedControlPoints(
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, 
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr & ControlPointsValue,
//	vector<double> & OutKnotValue)
//{
//	QPoints->points.clear();	
//	QPoints->points.insert(QPoints->points.end(), 
//		QPointsValue->points.begin(), QPointsValue->points.end());	
//
//	//最后才首尾相接 首尾相接形成闭合曲线
//	for(int i = 0; i < dSpline; i++)	
//	{
//		//尾部加入首部节点
//		if (QPoints->points.size() > i)	
//			QPoints->points.push_back(QPoints->points[i]);
//
//			//首部加入尾部节点
//		if (QPointsValue->points.size() > QPoints->points.size() - 1 - i)
//			QPoints->points.insert(QPoints->points.begin(), 
//				QPoints->points[QPoints->points.size() - 1 - i]);
//	}
//
//	dSpline = 3;
//
//	GetControlPointsAndKnotValue(ControlPointsValue, OutKnotValue, true);
//
//	RationalSpline.SetSplineInputs(ControlPointsValue, dSpline, OutKnotValue, true);
//	
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPointsValue (new pcl::PointCloud<pcl::PointXYZRGB>);
//	double UStart, UEnd;
//	//获取插值点的平均距离
//	//double AvgDis = GeometryBase::GetAvgDisOfPoints(QPoints);
//	int InsertNode = 0;
//	
//	//获取非凸区域的U值取值范围
//	for(int i = dSpline; i < ControlPointsValue->points.size(); i++)
//	{				
//		double NoConvexhullUStart = 0; double NoConvexhullUEnd = 0;
//		if (!RationalSpline.GetNoConvexHullZone(i, NoConvexhullUStart, NoConvexhullUEnd))
//		{				
//			InsertNoConvexHullU(NoConvexhullUStart, NoConvexhullUEnd);	
//		}
//	}
//	int FirstNoConveHullNumber = NoConvexHullStartU.size();
//
//	int IterationNumber = 0;
//	while(NoConvexHullStartU.size() > 0)
//	{
//		cout<<endl;
//		cout<<"正在进行第"<<++IterationNumber<<"次迭代,当前非凸区间共"
//			<<NoConvexHullStartU.size()<<"个"<<endl<<endl;
//		vector<int> InsertPointIndex;
//		//计算二次切矢量与两点之间直接的交点
//		for(int i = 0; i < NoConvexHullStartU.size(); i++)
//		{
//			//double U = (NoConvexHullStartU[i] + NoConvexHullEndU[i]) / 2;
//			double U = NoConvexHullStartU[i];
//			pcl::PointXYZRGB CurrentPoint = RationalSpline.GetSplinePoint(U);
//			pcl::PointXYZRGB TwoDerivativePoint = RationalSpline.GetSplineDerivativePoint(U, 2);
//		
//			double a1 = TwoDerivativePoint.y/TwoDerivativePoint.x;
//			double b1 = CurrentPoint.y - a1*CurrentPoint.x;
//
//			int UIndex = RationalSpline.FindUIndex(U);
//			int PointStartIndex = UIndex-dSpline;
//			int PointEndIndex = UIndex + 1;
//		
//			if (PointStartIndex < 0) PointStartIndex = 0;
//			if (PointEndIndex >= QPoints->points.size()) PointEndIndex = QPoints->points.size() - 1;
//			bool HaveCross = false;
//			for(int j = PointStartIndex; j < PointEndIndex; j++)
//			{				
//				pcl::PointXYZRGB CrossPoint = GeometryBase::CalcIntersectionOfLineAndSegment
//					(a1, b1, QPoints->points[j], QPoints->points[j+1], HaveCross);
//				if (HaveCross)
//				{
//					//2015.12.26 如果有交点，说明此区间的参数范围需要缩小
//					////ShrinkKnotParameters(i);
//					////HaveCross = true;
//					////break;
//
//					//如下2015.12.26注释
//					Viewer->addLine(QPoints->points[j], QPoints->points[j+1], StringBase::ClockValue());
//					PointBase::DrawExtentedLineByBasePointAndDirection(Viewer,CurrentPoint, TwoDerivativePoint, 5);
//					CrossPoint.z = QPoints->points[j].z;
//					CrossPoint.rgba = ColorBase::YellowColor;
//					//if ((GeometryBase::GetDisOfTwoPoints(CrossPoint, QPoints->points[j]) > AvgDis) && 
//					//	(GeometryBase::GetDisOfTwoPoints(CrossPoint, QPoints->points[j+1]) > AvgDis))
//					{
//						TempPointsValue->points.push_back(CrossPoint);
//						InsertPointIndex.push_back(j);
//					}
//					break;
//				}
//			}
//			if (HaveCross) break;
//		}
//
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempQPointsValue (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//		for(int i = 0; i < QPoints->points.size(); i++)
//		{		
//			TempQPointsValue->points.push_back(QPoints->points[i]);
//			if (InsertPointIndex.size() > 0 && InsertPointIndex[0] == i)
//			{
//				TempQPointsValue->points.push_back(TempPointsValue->points[0]);
//				InsertPointIndex.erase(InsertPointIndex.begin());
//				TempPointsValue->points.erase(TempPointsValue->points.begin());
//			}
//		}
//
//		QPoints->points.clear();
//		QPoints->points.insert(QPoints->points.begin(), 
//			TempQPointsValue->points.begin(), TempQPointsValue->points.end());
//
//		ResolveEquations();
//
//		OutKnotValue.clear();	
//		OutKnotValue.insert(OutKnotValue.end(), KnotValues.begin(), KnotValues.end());	
//
//		RationalSpline.SetSplineInputs(ControlPointsValue, dSpline, OutKnotValue, true);
//		
//		NoConvexHullStartU.clear();
//		NoConvexHullEndU.clear();
//		for(int i = dSpline; i < ControlPointsValue->points.size(); i++)
//		{				
//			double NoConvexhullUStart = 0; double NoConvexhullUEnd = 0;
//			if (!RationalSpline.GetNoConvexHullZone(i, NoConvexhullUStart, NoConvexhullUEnd))
//			{				
//				InsertNoConvexHullU(NoConvexhullUStart, NoConvexhullUEnd);	
//			}
//		}
//		//break;
//		if (NoConvexHullStartU.size() < FirstNoConveHullNumber)
//		{
//			cout<<"经过"<<IterationNumber<<"次迭代，已经成功减少了一个区间的凹性"<<endl;
//			break;
//		}
//		if (IterationNumber == 2)
//			break;
//
//	}

	//for(int i = dSpline; i < ControlPointsValue->points.size(); i++)
	//{		
	//	if (i == dSpline)
	//	{
	//		TempPointsValue->points.insert(TempPointsValue->points.end(),
	//			ControlPointsValue->points.begin(), 
	//			ControlPointsValue->points.begin() + i);	
	//	}
	//	else
	//	{
	//		TempPointsValue->points.push_back(ControlPointsValue->points[i]);
	//	}

	//	if (!RationalSpline.GetNoConvexHullZone(i, UStart, UEnd))
	//	{
	//		double U = (UStart + UEnd)/2;

	//		pcl::PointXYZRGB TempPoint = RationalSpline.GetSplinePoint(U);
	//		
	//		//int Index = FindPointInQPoints(TempPoint) - InsertNode;
	//		int Index = FindPointInQPoints(TempPoint);
	//		cout<<i<<"对应的Index"<<Index<<endl;
	//		if (i == 54)
	//			cout<<endl;

	//		if (Index > 0)
	//		{
	//			double a = (U - UParameters[Index])/(UParameters[Index+1] - UParameters[Index]);
	//
	//			pcl::PointXYZRGB Point0 = QPoints->points[Index];
	//			pcl::PointXYZRGB Point1 = QPoints->points[Index + 1];

	//			TempPoint = PointBase::PointsAdd(PointBase::PointsMutile(Point0, 1-a),
	//				PointBase::PointsMutile(Point1, a));
	//			
	//			Viewer->addText3D("-", TempPoint, 0.5, 255,0,0, StringBase::ClockValue());

	//			//TempPointsValue->points.insert(
	//			//	TempPointsValue->points.begin() + Index + 1 + InsertNode, TempPoint);

	//			TempPointsValue->points.push_back(TempPoint);

	//			InsertNode++;
	//			//if ( InsertNode == 1)
	//			//	break;
	//		}	
	//	}
	//}
	//添加后再获取一次
	//QPoints = TempQPointsValue;
	//GetControlPointsAndKnotValue(ControlPointsValue, OutKnotValue);
	
//	pcl::PointXYZRGB ResultPoint = QPoints->points[0];
//		
//	//ControlPointsValue = TempQPointsValue;	
//	return ResultPoint;
//}

//2015.12.24 向上面两组容器中添加非凸区域的U的起始区域
void CSplineInterpolation::InsertNoConvexHullU(double StartU, double EndU)
{
	if (NoConvexHullStartU.size() == 0)
	{
		NoConvexHullStartU.push_back(StartU);
		NoConvexHullEndU.push_back(EndU);
		return;
	}
	bool Find = false;
	for(int i = NoConvexHullStartU.size() - 1; i >= 0; i--) //逆序开始查找
	{
		if (NoConvexHullEndU[i] == StartU)
		{
			Find = true;
			NoConvexHullEndU[i] = EndU;
			break;
		}		
	}
	if (!Find)
	{
		NoConvexHullStartU.push_back(StartU);
		NoConvexHullEndU.push_back(EndU);		
	}
}


//定位点在插值点的区间范围
int CSplineInterpolation::FindPointInQPoints(pcl::PointXYZRGB Point)
{
	int Index = -1;
	for(int i = 0; i < QPoints->points.size()-1; i++)
	{		
		pcl::PointXYZRGB Point0 = PointBase::PointsMinus(Point, QPoints->points[i]);
		pcl::PointXYZRGB Point1 = PointBase::PointsMinus(Point, QPoints->points[i+1]);
		if (Point0.x * Point1.x < 0 && Point0.y * Point1.y < 0)
		{
			Index = i;
			break;
		}
	}
	return Index;
}

//均匀化样条的插值点
void CSplineInterpolation::HomogenizationSplineInterPolatingPoints()
{
	double AvgDis = 0;
	CalcBase<double> CalcBaseFloat;
	vector<double> VecDis;

	//2015.12.22 调整为按照最小距离均匀化，看看效果
	double MinDis = 0;

	if (IsClosed)
	{
		QPoints->points.push_back(QPoints->points[0]);
	}

	for(int i = 0; i < QPoints->points.size() - 1; i++)
	{
		double TempDis = PointDis(QPoints->points[i], QPoints->points[i+1]);
		VecDis.push_back(TempDis);
		
		//取最小距离
		if (MinDis == 0)
			MinDis = TempDis;
		else
		{
			if (TempDis < MinDis)
				MinDis = TempDis;
		}
		//取最小距离
		AvgDis = AvgDis +  TempDis / QPoints->points.size();
	}

	int AddNumber = 0;
	for(int i = 0; i < VecDis.size(); i++)
	{
		int K = VecDis[i] / MinDis;
		
		if (K > 2)
		{
			double XStep = (QPoints->points[i + AddNumber + 1].x - QPoints->points[i + AddNumber].x) / K;
			double YStep = (QPoints->points[i + AddNumber + 1].y - QPoints->points[i + AddNumber].y) / K;
			double ZStep = (QPoints->points[i + AddNumber + 1].z - QPoints->points[i + AddNumber].z) / K;
		
			for(int j = 1; j < K; j++)
			{
				pcl::PointXYZRGB NewPoint;
				NewPoint.x = QPoints->points[i + AddNumber].x + XStep;
				NewPoint.y = QPoints->points[i + AddNumber].y + YStep;
				NewPoint.z = QPoints->points[i + AddNumber].z + ZStep;
				QPoints->insert(QPoints->begin() + i + AddNumber + 1, NewPoint);			
				AddNumber = AddNumber + 1;
			}			
		}
	}	



	//for(int i = 0; i < QPoints->points.size() - 1; i++)
	//{
	//	double TempDis = PointDis(QPoints->points[i].x, QPoints->points[i].y, QPoints->points[i].z,
	//		QPoints->points[i+1].x, QPoints->points[i+1].y, QPoints->points[i+1].z);
	//	VecDis.push_back(TempDis);
	//	AvgDis = AvgDis +  TempDis / QPoints->points.size();
	//}

	//int AddNumber = 0;
	//for(int i = 0; i < VecDis.size(); i++)
	//{
	//	int K = VecDis[i] / AvgDis;
	//	
	//	if (K > 2)
	//	{
	//		double XStep = (QPoints->points[i + AddNumber + 1].x - QPoints->points[i + AddNumber].x) / K;
	//		double YStep = (QPoints->points[i + AddNumber + 1].y - QPoints->points[i + AddNumber].y) / K;
	//		double ZStep = (QPoints->points[i + AddNumber + 1].z - QPoints->points[i + AddNumber].z) / K;
	//	
	//		for(int j = 1; j < K; j++)
	//		{
	//			pcl::PointXYZRGB NewPoint;
	//			NewPoint.x = QPoints->points[i + AddNumber].x + XStep;
	//			NewPoint.y = QPoints->points[i + AddNumber].y + YStep;
	//			NewPoint.z = QPoints->points[i + AddNumber].z + ZStep;
	//			QPoints->insert(QPoints->begin() + i + AddNumber + 1, NewPoint);			
	//			AddNumber = AddNumber + 1;
	//		}			
	//	}
	//}	

	if (IsClosed)
	{
		QPoints->points.pop_back();
	}
}

//计算节点间的弦长值
double CSplineInterpolation::GetPointsChordLength()
{
	double Length = 0;
	CalcBase<double> CalcBaseFloat;

	ChordLengthS.clear();
	if (QPoints->points.size() < 2) 
	{	
		return 0;
	}

	ChordLengthS.push_back(0);
	for(int i = 0; i < QPoints->points.size() - 1; i++)
	{
		double TempLength = PointDis(QPoints->points[i],	QPoints->points[i+1]);
		ChordLengthS.push_back(TempLength);
		Length = Length + TempLength;		
	}	
	//ChordLengthS.push_back(0);
	return Length;
}


//计算节点的节点参数值 采用弦长法
void CSplineInterpolation::GetUParametesByChordLength()
{
	ChordLength = GetPointsChordLength();

	UParameters.clear();	
	double Value = 0;
	for(int i = 0; i < QPoints->points.size(); i++)
	{
		Value = Value + ChordLengthS[i];
		UParameters.push_back(Value/ChordLength);
	}
}


//计算节点的节点参数值 向心参数法 《The Nurbs books》P257 2015.12.24
void CSplineInterpolation::GetUParametesByCentripetalParameter()
{
	ChordLength = GetPointsChordLength();
	UParameters.clear();	
	
	//vector<double> Angle;
	//CalcChordAngle(Angle);	//计算角度

	double RootSum = 0;
	for(int i = 0; i < ChordLengthS.size(); i++)
	{
		RootSum = RootSum + sqrt(ChordLengthS[i]);		
		//RootSum = RootSum + pow(ChordLengthS[i], 1.0/3.0);
	}

	double Value = 0;
	for(int i = 0; i < QPoints->points.size(); i++)
	{
		Value = Value + sqrt(ChordLengthS[i]);		
		//Value = Value + pow(ChordLengthS[i], 1.0/3.0);
		UParameters.push_back(Value/RootSum);
	}	
}

//计算节点的节点参数值 福利参数法 《计算机辅助几何设计与非均匀B样条》P45 2015.12.24
void CSplineInterpolation::GetUParametesByFoleyParameter()
{
	ChordLength = GetPointsChordLength();
	vector<double> Angle;
	CalcChordAngle(Angle);

	UParameters.clear();	

	double Value = 0;
	UParameters.push_back(0);
	for(int i = 1; i < QPoints->points.size(); i++)
	{		
		cout<<i<<endl;
		double Dis1, Dis2, Dis3, k;
		if (i == 1) 		
			Dis1 = 0;			
		else
			Dis1 = ChordLengthS[i-1];
		if ( i == QPoints->points.size()-1)		
			Dis3 = 0;
		else
			Dis3 = ChordLengthS[i+1];
		
		Dis2 = ChordLengthS[i];		

		k = 1 + 1.5 * (Dis1 * Angle[i-1]/(Dis1+Dis2) + Dis3*Angle[i]/(Dis2+Dis3));
		Value = Value + k*Dis2;
		UParameters.push_back(Value);
	}		
	for(int i = 0; i < UParameters.size(); i++)
	{
		UParameters[i] = UParameters[i]/ UParameters[UParameters.size()-1];
	}
}

	//计算弦线夹角的外角	《计算机辅助几何设计与非均匀B样条》P48 2015.12.24	
void CSplineInterpolation::CalcChordAngle(vector<double> & Angle)
{
	Angle.clear();
	Angle.push_back(0);
	for(int i = 1; i < QPoints->points.size() - 1; i++)
	{
		double Dis1 = ChordLengthS[i];
		double Dis2 = ChordLengthS[i+1];
		double Dis3 = PointDis(QPoints->points[i-1], QPoints->points[i+1]);
		double tempAngle = acos((Dis1*Dis1+Dis2*Dis2-Dis3*Dis3)/(2.0*Dis1*Dis2));

		if ((M_PI - tempAngle) < M_PI/2.0) Angle.push_back(M_PI - tempAngle);
		else Angle.push_back(M_PI/2.0);
	}
	Angle.push_back(0);

	//for(int i = 0; i < QPoints->points.size() ; i++)
	//{
	//	Viewer->addText3D(StringBase::FloatToStr(Angle[i]),QPoints->points[i] ,0.05);
	//}
}


//计算节点参数值
void CSplineInterpolation::CalcKnotParameters()
{
	nSpline = QPoints->points.size();
	KnotValues.clear();
	
	if (IsCentripetal)
		GetUParametesByCentripetalParameter();
	else
		GetUParametesByChordLength();

	//GetUParametesByFoleyParameter();	
}

//计算节点向量 采用取平均值法
void CSplineInterpolation::GetKnotValuesByAvg()
{
	KnotValues.clear();
	for(int i =0 ; i < nSpline + dSpline + 1; i++)
	{
		if (i <= dSpline) KnotValues.push_back(0);
		else if (i > dSpline && i < nSpline) 
		{
			double Sum = 0;
			for(int j = 1; j <= dSpline; j++)
			{
				Sum = Sum + UParameters[i-j];
			}
			Sum = Sum / dSpline;
			KnotValues.push_back(Sum);
		}
		else if ( i >= nSpline)
		{
			KnotValues.push_back(1);	
		}
	}
}

//构建线性方程组并求解 将求解结果返回到 ControlPoints 点集中
bool CSplineInterpolation::ResolveEquations()
{
	//计算节点向量
	GetKnotValuesByAvg();

	//设置有理样条的参数信息，主要用于计算 在节点向量获取参数的位置 	
	RationalSpline.SetSplineInputs(QPoints, dSpline, KnotValues);
	// AX=B
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nSpline, nSpline); //默认是0矩阵
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nSpline, 3);	// 默认是0矩阵
	
	//对A赋值  需要补充
	for(int i = 0; i < nSpline; i++)	//行
	{	//获取参数所在的位置
		int KnotVectorIndex = RationalSpline.FindUIndex(UParameters[i]);
		for(int j = 0; j < nSpline; j++)	//列
		{				
			int K = KnotVectorIndex - dSpline + j;
			if (K >= nSpline) continue;
			
			A(i,K) = RationalSpline.CoxdeBoor(K, dSpline, UParameters[i]);
			//cout<<"i: "<<i<<" k:"<<K<<" d:"<<dSpline<<" u:"<<UParameters[i]<<" CoxdeBoor:"<<A(i,K)<<endl;
		}
	}
	A(nSpline - 1, nSpline - 1) = 1;
	//cout<<"矩阵A："<<endl;
	//cout<<A<<endl;
	//cout<<A(0,0)<<endl;
	//对B赋值
	for(int i = 0; i < nSpline; i++)
	{
		B(i,0) = QPoints->points[i].x;
		B(i,1) = QPoints->points[i].y;
		B(i,2) = QPoints->points[i].z;
	}
	//cout<<"矩阵B："<<endl;
	//cout<<B<<endl;
	//求解方程组
	
	Eigen::MatrixXd X = Math::ResloveLinearEquationsByLUDecomposition(A, B);
		
	//cout<<"矩阵X："<<endl;
	//cout<<X<<endl;

	//将求解结果转为控制点
	bool IsRight = true;
	ControlPoints->points.clear();
	for(int i = 0; i < nSpline; i++)
	{
		pcl::PointXYZRGB TempPoint;
		TempPoint.x = X(i,0);
		TempPoint.y = X(i,1);
		TempPoint.z = X(i,2);

		if (isnan(TempPoint.x) || isnan(TempPoint.y) || isnan(TempPoint.z))
		{
			IsRight = false;
		}

		TempPoint.rgba = ColorBase::RedColor;
		ControlPoints->points.push_back(TempPoint);
	}	
	return IsRight;
}

//获取控制点与节点向量
bool CSplineInterpolation::ResolveControlPoints()
{
	//计算节点参数值	
	CalcKnotParameters();

	//求解方程组 获取控制点坐标
	return ResolveEquations();
}

bool CSplineInterpolation::GetControlPointsAndKnotValue(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  &
	ControlPointsValue,	vector<double> & OutKnotValue, bool CentripetalValue)
{
	IsCentripetal = CentripetalValue;
	bool IsRight = ResolveControlPoints();
	ControlPointsValue = ControlPoints;

	OutKnotValue.clear();	
	OutKnotValue.insert(OutKnotValue.end(), KnotValues.begin(), KnotValues.end());	
	return IsRight;
}


//缩小节点间的参数值
void CSplineInterpolation::ShrinkKnotParameters(int Index, double Multiple)
{
	if (Index < 0 || Index >= UParameters.size() - 2) return;

	double MinusU = (UParameters[Index+1] - UParameters[Index]) * Multiple;
	
	//此种方法总配比发生变化
	for(int i = Index+1; i < UParameters.size(); i++)
	{
		UParameters[i] = UParameters[i] - MinusU;
	}
	//只有相邻的两个配比发生变化，其它配比不变
	//UParameters[Index+1] = UParameters[Index+1] - MinusU;
	//UParameters[Index+2] = UParameters[Index+2] + MinusU;
}