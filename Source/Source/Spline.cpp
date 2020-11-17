#include "Spline.h"

//设置样条参数
void CSpline::SetSplineInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPointsPtr,
	int dValue, vector<double> KnotVectorValue, 
	bool IsClosedCurve, bool IsConvexhullControlValue)
{
	ControlPoints->points.clear();
	CurvePoints->points.clear();
	ControlPoints->points.insert(ControlPoints->points.end(),
		ControlPointsPtr->points.begin(),
		ControlPointsPtr->points.end());	
	nSpline = ControlPoints->points.size();	//控制点个数	
	//d = dValue + 1;								//dValue样条次数 
	dSpline = dValue;								//dValue样条次数 
	KnotVector = KnotVectorValue;
	IsClosed = IsClosedCurve;
	KnotValuePoints->points.clear();	
	IsConvexhullControl = IsConvexhullControlValue;	
	
	FirstPointOfCloseCurve.x = FirstPointOfCloseCurve.y = FirstPointOfCloseCurve.z = 0;
}

//计算混合函数 注意 d 是样条使用的多项式的次数，而不是阶数，
double CSpline::CoxdeBoor(int i, int d, double u)
{
	//d是次数时的计算方法 *****************************	
	if (d == 0)  
	{
		if (KnotVector[i] <= u && u < KnotVector[i+1])		
			return 1;		
		else
			return 0;
	}
	else
	{		
		if (abs(KnotVector[i + d] - KnotVector[i]) < eps)
		{
			if (abs(KnotVector[i + d + 1] - KnotVector[i + 1]) < eps)
				return 0;
			else
				return CoxdeBoor(i + 1, d - 1, u) * (KnotVector[i + d + 1] - u)/(KnotVector[i + d + 1] - KnotVector[i + 1]);
		}
		else
		{
			if (abs(KnotVector[i + d + 1] - KnotVector[i + 1]) < eps)
				return CoxdeBoor(i, d - 1, u) * (u - KnotVector[i])/(KnotVector[i + d] - KnotVector[i]);
			else
				return CoxdeBoor(i, d - 1, u) * (u - KnotVector[i])/(KnotVector[i + d] - KnotVector[i])
						+ CoxdeBoor(i + 1, d - 1, u) * (KnotVector[i + d + 1] - u)/(KnotVector[i + d + 1] - KnotVector[i + 1]);
		}
		//cout<<"k:"<<k<<" d:"<<d<<" u:"<<u<<" Result:"<<Result<<endl;		
	}
	//*/		//d是次数时的计算方法 *****************************

	////d是阶数时的计算方法 *****************************	
	//if (d == 1)  
	//{
	//	if (KnotVector[k] <= u && u < KnotVector[k+1])		
	//		return 1;		
	//	else
	//		return 0;
	//}
	//else
	//{
	//	double Result = 0;
	//	if ((KnotVector[k+d-1] - KnotVector[k]) == 0)
	//	{
	//		if ((KnotVector[k+d] - KnotVector[k+1]) == 0)
	//			Result = 0;
	//		else
	//			Result = CoxdeBoor(k+1, d-1, u) * (KnotVector[k+d] - u)/(KnotVector[k+d] - KnotVector[k+1]);
	//	}
	//	else
	//	{
	//		if ((KnotVector[k+d] - KnotVector[k+1]) == 0)
	//			Result = CoxdeBoor(k, d-1, u) * (u - KnotVector[k])/(KnotVector[k+d-1] - KnotVector[k]);
	//		else
	//			Result = CoxdeBoor(k, d-1, u) * (u - KnotVector[k])/(KnotVector[k+d-1] - KnotVector[k])
	//					+ CoxdeBoor(k+1, d-1, u) * (KnotVector[k+d] - u)/(KnotVector[k+d] - KnotVector[k+1]);
	//	}
	//	//cout<<"k:"<<k<<" d:"<<d<<" u:"<<u<<" Result:"<<Result<<endl;
	//	return Result;
	//}
	//*/		//d是阶数时的计算方法 *****************************
}

//计算混合函数 注意 d是样条使用的多项式的次数，而不是阶数， 返回公式
string CSpline::CoxdeBoorFormula(int i, int d, double u)
{
	if (d == 0)  
	{
		if (KnotVector[i] <= u && u < KnotVector[i+1])		
			return "1";		
		else
			return "0";
	}
	else
	{		
		if (abs(KnotVector[i+d] - KnotVector[i]) < eps)
		{
			if (abs(KnotVector[i+d+1] - KnotVector[i+1]) < eps)
				return "0";
			else
			{
				string CoxdeBoorValue = CoxdeBoorFormula(i+1, d-1, u);
				CalcBase<double> CalcBaseFloat;
				double DenominatorValue = (KnotVector[i+d+1] - KnotVector[i+1]);

				if (CoxdeBoorValue == "0")
				{
					return "0";	
				}
				else if(CoxdeBoorValue == "1")
				{
					//return "(" + CalcBaseFloat.ConvertToString(KnotVector[i+d+1]) + "-u) /" 
					//	+ CalcBaseFloat.ConvertToString(DenominatorValue);
					return "(" + CalcBaseFloat.ConvertToString(KnotVector[i+d+1]/DenominatorValue) 
						+ "-u*" + CalcBaseFloat.ConvertToString(1/DenominatorValue) + ") ";
				}
				else
				{									
/*					return CoxdeBoorValue + " * (" 
						+ CalcBaseFloat.ConvertToString(KnotVector[i+d+1]) + "-u) /" 
						+ CalcBaseFloat.ConvertToString(DenominatorValue);	*/	
					return CoxdeBoorValue + " * (" 
						+ CalcBaseFloat.ConvertToString(KnotVector[i+d+1]/DenominatorValue) 
						+ "-u*"+ CalcBaseFloat.ConvertToString(1/DenominatorValue) + ")" ;						
				}
				//return CoxdeBoor(i+1, d-1, u) * (KnotVector[i+d+1] - u)/(KnotVector[i+d+1] - KnotVector[i+1]);
			}
		}
		else
		{
			if (abs(KnotVector[i+d+1] - KnotVector[i+1]) < eps)
			{
				string CoxdeBoorValue =  CoxdeBoorFormula(i, d-1, u);
				CalcBase<double> CalcBaseFloat;
				double DenominatorValue = (KnotVector[i+d] - KnotVector[i]);
				if (CoxdeBoorValue == "0")
				{
					return "0";
				}
				else if (CoxdeBoorValue == "1")
				{
					//return  "(u - " + CalcBaseFloat.ConvertToString(KnotVector[i]) + ") /" 
					//	+ CalcBaseFloat.ConvertToString(DenominatorValue);
					return  "(u*" + CalcBaseFloat.ConvertToString(1/DenominatorValue) + " - " 
						+ CalcBaseFloat.ConvertToString(KnotVector[i]/DenominatorValue) + ")" ;						
				}
				else
				{
					//return "(" + CoxdeBoorValue + ") * (u - " + CalcBaseFloat.ConvertToString(KnotVector[i]) + ") /" 
					//	+ CalcBaseFloat.ConvertToString(DenominatorValue); 
					return "(" + CoxdeBoorValue + ") * (u*" 
						+ CalcBaseFloat.ConvertToString(1/DenominatorValue) + " - " 
						+ CalcBaseFloat.ConvertToString(KnotVector[i]/DenominatorValue) + ")"; 
						 
				}
				//return CoxdeBoor(i, d-1, u) * (u - KnotVector[i])/(KnotVector[i+d] - KnotVector[i]);
			}
			else
			{
				CalcBase<double> CalcBaseFloat;
				string CoxdeBoorValueOne =  CoxdeBoorFormula(i, d-1, u);
				string CoxdeBoorValueTwo =  CoxdeBoorFormula(i+1, d-1, u);

				double DenominatorValueOne = (KnotVector[i+d] - KnotVector[i]);
				double DenominatorValueTwo = (KnotVector[i+d+1] - KnotVector[i+1]);

				string ResultOne;
				string ResultTwo;

				if (CoxdeBoorValueOne == "0")
				{
					ResultOne = "0";
				}
				else if (CoxdeBoorValueOne == "1")
				{
					ResultOne =  "(u*" + CalcBaseFloat.ConvertToString(1/DenominatorValueOne) + " - " 
						+ CalcBaseFloat.ConvertToString(KnotVector[i]/DenominatorValueOne) + ")"; 						
				}
				else
				{
					//ResultOne = "(" + CoxdeBoorValueOne + ") * (u - " + CalcBaseFloat.ConvertToString(KnotVector[i]) + ") /" 
					//	+ CalcBaseFloat.ConvertToString(DenominatorValueOne); 
					ResultOne = "(" + CoxdeBoorValueOne + ") * (u*" +  CalcBaseFloat.ConvertToString(1/DenominatorValueOne) 					
						+ " - " + CalcBaseFloat.ConvertToString(KnotVector[i]/DenominatorValueOne) + ")" ;
						
				}

				if (CoxdeBoorValueTwo == "0")
				{
					ResultTwo = "0";	
				}
				else if(CoxdeBoorValueTwo == "1")
				{
					//ResultTwo =  "(" + CalcBaseFloat.ConvertToString(KnotVector[i+d+1]) + "-u) /" 
					//	+ CalcBaseFloat.ConvertToString(DenominatorValueTwo);
					ResultTwo =  "(" + CalcBaseFloat.ConvertToString(KnotVector[i+d+1]/DenominatorValueTwo) 
						+ "-u*" + CalcBaseFloat.ConvertToString(1/DenominatorValueTwo) + ")" ;
				}
				else
				{									
/*					ResultTwo = "(" + CoxdeBoorValueTwo + ") * (" 
						+ CalcBaseFloat.ConvertToString(KnotVector[i+d+1]) + "-u) /" 
						+ CalcBaseFloat.ConvertToString(DenominatorValueTwo);	*/		
					ResultTwo = "(" + CoxdeBoorValueTwo + ") * (" 					
						+ CalcBaseFloat.ConvertToString(KnotVector[i+d+1]/DenominatorValueTwo) 
						+ "-u*" + CalcBaseFloat.ConvertToString(1/DenominatorValueTwo) + ")" ;						
				}

				if (ResultOne == "0") 
					return ResultTwo;
				else if (ResultTwo == "0") return ResultOne;
				else 
					return ResultOne + " + " + ResultTwo;
			}
			//	return CoxdeBoor(i, d-1, u) * (u - KnotVector[i])/(KnotVector[i+d] - KnotVector[i])
			//			+ CoxdeBoor(i+1, d-1, u) * (KnotVector[i+d+1] - u)/(KnotVector[i+d+1] - KnotVector[i+1]);
		}
		//cout<<"k:"<<k<<" d:"<<d<<" u:"<<u<<" Result:"<<Result<<endl;		
	}	
}

//计算基函数Ni,p(u)的k阶导数
double CSpline::DerivativeOfCoxdeBoor(int i, int p, double u, int Order)
{
	if (Order == 1)	//一阶导数 计算方式见《非均匀有理B样条》第42页 公式 2.7
	{
		if (abs(KnotVector[i+p] - KnotVector[i]) < eps)  //==0
		{
			if (abs(KnotVector[i+p+1] - KnotVector[i+1]) < eps)	//==0
				return 0;
			else
				return -p * CoxdeBoor(i+1, p-1, u) / (KnotVector[i+p+1] - KnotVector[i+1]);			
		}
		else
		{
			if (abs(KnotVector[i+p+1] - KnotVector[i+1]) < eps)	//==0
				return p * CoxdeBoor(i, p-1, u) / (KnotVector[i+p] - KnotVector[i]);
			else
				return p * CoxdeBoor(i, p-1, u) / (KnotVector[i+p] - KnotVector[i]) -
					p * CoxdeBoor(i+1, p-1, u) / (KnotVector[i+p+1] - KnotVector[i+1]);
		}
	}
	else	//高阶导	计算方式见《非均匀有理B样条》第44页 公式 2.9
	{
		if (abs(KnotVector[i+p] - KnotVector[i]) < eps)  //==0
		{
			if (abs(KnotVector[i+p+1] - KnotVector[i+1]) < eps)	//==0
				return 0;
			else
				return p * (-1) * DerivativeOfCoxdeBoor(i+1, p-1, u, Order-1)/(KnotVector[i+p+1] - KnotVector[i+1]);
		}
		else
		{
			if (abs(KnotVector[i+p+1] - KnotVector[i+1]) < eps)	//==0
				return p * DerivativeOfCoxdeBoor(i, p-1, u, Order-1)/ (KnotVector[i+p] - KnotVector[i]);
			else
				return p * DerivativeOfCoxdeBoor(i, p-1, u, Order-1)/ (KnotVector[i+p] - KnotVector[i]) - 
					p * DerivativeOfCoxdeBoor(i+1, p-1, u, Order-1)/ (KnotVector[i+p+1] - KnotVector[i+1]);				
		}
	}
}

//2015.12.24 向上面两组容器中添加非凸区域的U的起始区域
void CSpline::InsertNoConvexHullU(double StartU, double EndU)
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


//生长样条函数的点 //生成样条函数的点 返回样条的长度 节点向量间隔为0.000001
void CSpline::CreateSpline(int DerivativeOrder)
{	
	SplineLengthOfKnotVector.clear();
	SplineHeightOfKnotVector.clear();

	if (IsClosed && FirstPointOfCloseCurve.x == 0 
		&& FirstPointOfCloseCurve.y == 0 && FirstPointOfCloseCurve.z == 0)
	{
		cout << "the closure curve can not be constructed as the first node is not assigned" << endl;
		return;
	}

	StartU = 0;
	EndU = 0;
	//构建节点向量
	if (KnotVector.size() == 0) 
	{
		CanCreateCurve = false;
		cout<<"节点向量为空，无法构建样条！"<<endl;
		return;
	}

	if (KnotVector.size() != nSpline + dSpline + 1) 
	{
		cout<<"节点向量个数不等于控制点的个数(n)与阶参数的个数(d)的和不等，请核实！"<<endl;
		CanCreateCurve = false;
		return ;
	}

	if (!CanCreateCurve)
	{
		cout<<"控制点个数与节点向量个数不符合要求，无法构建样条！"<<endl;
		return;
	}

	if (IsClosed)
	{
		if (FirstPointOfCloseCurve.z == -100000)
		{
			cout<<"闭合曲线的首节点未赋值，无法构建样条！"<<endl;
			return ;			
		}
	}

	//if ( DerivativeOrder > dSpline) DerivativeOrder = dSpline;	//求导最大次数不超过 样条次数

	//cout<<"正在构建样条曲线上的点"<<endl;

	//计算一阶导数的控制点与节点向量 根据节点间与节点向量间的关系
	//GetOneDerivativeData();

	int KnotStartIndex, KnotEndIndex;
	int ClosedZone = 1;
	
	if (IsClosed) ClosedZone = dSpline;

	////针对是否有循环的开始索引
	//if (IsClosed) KnotStartIndex = dSpline * 2 + 1;
	//else KnotStartIndex = dSpline;
	//
	//if (IsClosed) KnotStartIndex = dSpline + dSpline - 1;
	//else KnotStartIndex = dSpline;

	//////针对是否有连续的结束索引
	//if (IsClosed) KnotEndIndex = nSpline - dSpline + 1;
	//else KnotEndIndex = nSpline;

	KnotStartIndex = dSpline;
	KnotEndIndex = nSpline;

	PointUValues.clear();
	CurvePoints->points.clear();

	double UValue = KnotVector[KnotStartIndex];
	double Dis = 0, Error = 0.001;	//如果精度再小的话，容易识别第一个点失败 0.001
	CalcBase<double> CalcBaseFloat;

	pcl::PointXYZRGB PriorPoint = GetSplinePoint(UValue);
	
	pcl::PointXYZRGB NextPoint;

	bool FindFirstNode = false;
	//如果不是闭合样条 就要从开始位置计算
	if (!IsClosed) 
	{
		CurvePoints->points.push_back(PriorPoint);	//如果非闭合就要加上第一个节点 2016.12.12
		PointUValues.push_back(UValue);
		FindFirstNode = true;
	}
	//FindFirstNode = true;

	UValue = UValue + UStep;

	int k = 0;
	
	int CurrentKnotIndex, OldKnotIndex = -1;
	double NoConvexhullUStart, NoConvexhullUEnd;
	bool FindNoConvexhull = false; 
	while(UValue < KnotVector[KnotEndIndex])
	{			
		NextPoint = GetSplinePoint(UValue);

		//按照节点向量区间控制 显示
		if (UValue < KnotVector[KnotStartIndex + 1])
			NextPoint.rgba = ColorBase::RedColor;
		else if (UValue > KnotVector[KnotEndIndex - 1])
			NextPoint.rgba = ColorBase::GreenColor;
		else
			NextPoint.rgba = ColorBase::BlueColor;

		if (!FindFirstNode)
			NextPoint.rgba = ColorBase::RedColor;
		else
			NextPoint.rgba = ColorBase::BlueColor;

		ShowedCurvePoints->points.push_back(NextPoint);
			
		//寻找此区间的非凸区域
			//在找到闭合曲线的首尾节点时 有效
		CurrentKnotIndex = FindUIndex(UValue);
		if (CurrentKnotIndex != OldKnotIndex)	
		{
			OldKnotIndex = CurrentKnotIndex;
			if (IsConvexhullControl)
			{
				NoConvexhullUStart = 0; NoConvexhullUEnd = 0;
				if (!GetNoConvexHullZone(CurrentKnotIndex, NoConvexhullUStart, NoConvexhullUEnd))
				{	
					FindNoConvexhull = true;
					InsertNoConvexHullU(NoConvexhullUStart, NoConvexhullUEnd);	
				}
				else
					FindNoConvexhull = false;
			}
		}
		else
			//颜色控制
			NextPoint.rgba = ColorBase::BlueColor;

		//PointAndDerivative CurrentPoint;	
		
		if (UValue < KnotVector[KnotStartIndex + ClosedZone])
		{	//开始区域
			//NextPoint.rgba = ColorBase::RedColor;
			if (!FindFirstNode)
			{
				double TempDis = PointDis(NextPoint.x, NextPoint.y, NextPoint.z,
					FirstPointOfCloseCurve.x, FirstPointOfCloseCurve.y, FirstPointOfCloseCurve.z);
				if (TempDis < Error)
				{					
					StartU = UValue;
					FindFirstNode = true;
				}
			}
		}
		else if (UValue > KnotVector[KnotEndIndex - ClosedZone])
		{
			//结束区域
			//NextPoint.rgba = ColorBase::GreenColor;
			//NextPoint.z = NextPoint.z + 0.1;
			double TempDis = PointDis(NextPoint, FirstPointOfCloseCurve);
			if (TempDis < Error)
			{
				EndU = UValue;
				FindFirstNode = false;
				//break;
			}
		}
		
		if (FindFirstNode)
		{	
			//if (FindNoConvexhull && IsConvexhullControl
			//	&& NoConvexhullUStart < UValue && UValue < NoConvexhullUEnd)
			//{
			//	NextPoint.rgba = ColorBase::RedColor;
			//}
			PointUValues.push_back(UValue);
			CurvePoints->points.push_back(NextPoint);
			
			//if (CurvePoints->points.size() > 0)
			//	Dis = Dis + CalcBaseFloat.PointDis(PriorPoint.x, PriorPoint.y, PriorPoint.z,
			//			NextPoint.x, NextPoint.y, NextPoint.z);
		}
			//求各阶导数
		//if (DerivativeOrder > 0)
		//{
		//	CurrentPoint.push_back(NextPoint);
		//	for (int Order = 1; Order <= DerivativeOrder; Order++)	//以此求各阶导数
		//	{
		//		//获取第 Order 阶 导数
		//		pcl::PointXYZRGB DerivativePoint = GetSplineDerivativePoint(UValue, Order);
		//		CurrentPoint.push_back(DerivativePoint);
		//	}	
		//	CurvePointsAndDerivativeS.push_back(CurrentPoint);
		//}
		//求各阶导数
		//cout<<"当前 u 值为 " <<U<<" 此处的曲率是 "<<GetCurvature(U)<<" 此处的挠率是 "<<GetTorsion(U)<<endl; 
						
		PriorPoint = NextPoint;
		UValue = UValue + UStep;		
	}
	//cout<<"样条曲线构建完毕"<<endl;
	if (IsConvexhullControl)
	{
		for(int i = 0 ; i < NoConvexHullStartU.size(); i++)
		{
			double UTempMiddle = (NoConvexHullStartU[i] + NoConvexHullEndU[i]) / 2.0;
			pcl::PointXYZRGB BasePoint = GetSplinePoint(UTempMiddle);
			//pcl::PointXYZRGB TempPoint = GetSplineDerivativePoint(UTempMiddle, 2);
			//2016.03.26  不显示带符号的曲率值
			//Viewer->addText3D(StringBase::IntToStr(i) + ":"	+ StringBase::FloatToStr(GetCurvature(UTempMiddle)), BasePoint, 0.5);
			//PointBase::DrawExtentedLineByBasePointAndDirection(Viewer, 
			//	BasePoint, TempPoint, 5);
			//NextPoint.rgba = ColorBase::PurpleColor;

			if (i == 0)
			{			
				pcl::PointXYZRGB Point0 = GetSplinePoint(NoConvexHullStartU[i]);
				pcl::PointXYZRGB Point01 = GetSplineDerivativePoint(NoConvexHullStartU[i], 1, false);
				pcl::PointXYZRGB Point3 = GetSplinePoint(NoConvexHullEndU[i]);
				pcl::PointXYZRGB Point31 = GetSplineDerivativePoint(NoConvexHullEndU[i], 1, false);;

				pcl::PointXYZRGB Point1 = PointBase::PointsMutile(
					PointBase::PointsAdd(Point01, PointBase::PointsMutile(Point0, 3)),1/3.0);

				pcl::PointXYZRGB Point2 = PointBase::PointsMutile(
					PointBase::PointsMinus(PointBase::PointsMutile(Point3, 3), Point31),1/3.0);
				//2016.03.26  不显示带符号的曲率值
				//Viewer->addText3D(StringBase::IntToStr(i) + "-1", Point1, 0.5, 255,0,0,StringBase::ClockValue());
				//Viewer->addText3D(StringBase::IntToStr(i) + "-2", Point2, 0.5, 0, 255,0,StringBase::ClockValue());
			}
		}
	}

	return;
	
	////获取节点向量的步长
	////GetKnoteVectorStep();

	////从KnotVector[d] 开始 至 KnotVector[n] 结束 此处的n为节点个数加1 d 是样条的次数
	//double U = KnotVector[dSpline];		
	////注意节点向量的起始位置
	//while((U - KnotVector[nSpline]) < eps)
	//{					
	//	PointAndDerivative CurrentPoint;

	//	pcl::PointXYZRGB TempPoint = GetSplinePoint(U);
	//	CurvePoints->points.push_back(TempPoint);

	//	//求各阶导数
	//	if (DerivativeOrder > 0)
	//	{
	//		CurrentPoint.push_back(TempPoint);
	//		for (int Order = 1; Order <= DerivativeOrder; Order++)	//以此求各阶导数
	//		{
	//			//获取第 Order 阶 导数
	//			pcl::PointXYZRGB DerivativePoint = GetSplineDerivativePoint(U, Order);
	//			CurrentPoint.push_back(DerivativePoint);
	//		}	
	//		CurvePointsAndDerivativeS.push_back(CurrentPoint);
	//	}
	//	//求各阶导数
	//	//cout<<"当前 u 值为 " <<U<<" 此处的曲率是 "<<GetCurvature(U)<<" 此处的挠率是 "<<GetTorsion(U)<<endl; 
	//	U = U + UStep;
	//}
}

//获取少量的SplinePoint 2020.08.11
void CSpline::ZippedSplinePoints(int PointNum)
{
	if (PointNum != -1)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		double Space = 1.0 * CurvePoints->points.size() / PointNum;
		//for (int i = 0; i < 360; i++) 
		for (int i = 0; i < PointNum; i++)
		{
			TempPoints->points.push_back(CurvePoints->points[floor(i * Space)]);
		}
		CurvePoints->points.clear();
		PointBase::PointCopy(TempPoints, CurvePoints);
	}
}

void CSpline::SaveControlPointsAndKnotValue(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPointsValue,
	vector<double> KnotValue,
	string FileNameControlPoints, string FileNameKnotValue,
	float StartKnot, float EndKnot)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr KnotPoints (new pcl::PointCloud<pcl::PointXYZRGB>);

	for(int i = 0; i < KnotValue.size(); i++)
	{
		pcl::PointXYZRGB TempPoint;
		TempPoint.x = KnotValue[i];
		TempPoint.y = KnotValue[i];
		TempPoint.z = KnotValue[i];

		KnotPoints->points.push_back(TempPoint);
	}
	
	if (StartKnot > 0) 
	{
		pcl::PointXYZRGB TempPoint;
		TempPoint.x = StartKnot;
		TempPoint.y = EndKnot;
		TempPoint.z = -1;

		KnotPoints->points.push_back(TempPoint);	
	}

	PointBase::SavePCDToFileName(ControlPointsValue, FileNameControlPoints);
	PointBase::SavePCDToFileName(KnotPoints, FileNameKnotValue);
}

//根据u值返回 u 的字符串公式表达式 2015.10.17
vector<string> CSpline::GetUFormula(double U)
{
	vector<string> ResultStrS;

	ResultStrS.push_back("");
	ResultStrS.push_back("");
	ResultStrS.push_back("");

	int KnotVectorIndex = FindUIndex(U);
	CalcBase<double> CalcBaseFloat;

	//for(int j = 0; j < d; j++) //d是阶数时的使用情况
	for(int j = 0; j < dSpline + 1; j++) //样条上的点使用前 dSpline + 1 个控制点
	{			
		//int K = KnotVectorIndex - d + j + 1;
		int K = KnotVectorIndex - dSpline + j;	

		double CoxdeBoorValue = CoxdeBoor(K, dSpline, U);
		string CoxdeBoorStr = CoxdeBoorFormula(K, dSpline, U);
		//cout<<"k:"<<K<<" d:"<<dSpline<<" u:"<<u<<" B:"<<B<<endl;
		
		if (ResultStrS[0] == "")
		{
			ResultStrS[0] = CalcBaseFloat.ConvertToString(ControlPoints->points[K].x) 
				+ "*(" + CoxdeBoorStr + ")";

			ResultStrS[1] = CalcBaseFloat.ConvertToString(ControlPoints->points[K].y) 
				+ "*(" + CoxdeBoorStr + ")";

			ResultStrS[2] = CalcBaseFloat.ConvertToString(ControlPoints->points[K].z) 
				+ "*(" + CoxdeBoorStr + ")";
		}
		else
		{
			if (ControlPoints->points[K].x > 0)
				ResultStrS[0] = ResultStrS[0] + " + " 
					+ CalcBaseFloat.ConvertToString(ControlPoints->points[K].x) 
					+ "*(" + CoxdeBoorStr + ")";
			else if (ControlPoints->points[K].x < 0)
				ResultStrS[0] = ResultStrS[0] 
				+ CalcBaseFloat.ConvertToString(ControlPoints->points[K].x) 
					+ "*(" + CoxdeBoorStr + ")";

			if (ControlPoints->points[K].y > 0)
				ResultStrS[1] = ResultStrS[1] + " + " 
					+ CalcBaseFloat.ConvertToString(ControlPoints->points[K].y) 
					+ "*(" + CoxdeBoorStr + ")";
			else if (ControlPoints->points[K].y < 0)
				ResultStrS[1] = ResultStrS[1] 
					+ CalcBaseFloat.ConvertToString(ControlPoints->points[K].y) 
					+ "*(" + CoxdeBoorStr + ")";
			
			if (ControlPoints->points[K].z > 0)
				ResultStrS[2] = ResultStrS[2] + " + " 
					+ CalcBaseFloat.ConvertToString(ControlPoints->points[K].z) 
					+ "*(" + CoxdeBoorStr + ")";	
			else if (ControlPoints->points[K].z < 0)
				ResultStrS[2] = ResultStrS[2]
					+ CalcBaseFloat.ConvertToString(ControlPoints->points[K].z) 
					+ "*(" + CoxdeBoorStr + ")";
		}
	}
	return ResultStrS;
}

//根据节点向量的值获取节点 2015.09.24
pcl::PointXYZRGB CSpline::GetSplinePoint(double U)
{
	pcl::PointXYZRGB TempPoint;
	TempPoint.x = 0;
	TempPoint.y = 0;
	TempPoint.z = 0;
	TempPoint.rgba = ColorBase::RedColor;
	
	int KnotVectorIndex = FindUIndex(U);

	//for(int j = 0; j < d; j++) //d是阶数时的使用情况
	for(int j = 0; j < dSpline + 1; j++) //样条上的点使用前 dSpline + 1 个控制点
	{			
		//int K = KnotVectorIndex - d + j + 1;
		int K = KnotVectorIndex - dSpline + j;	

		double B = CoxdeBoor(K, dSpline, U);
		//cout<<"k:"<<K<<" d:"<<dSpline<<" u:"<<u<<" B:"<<B<<endl;
		
		//if (abs(U - 1) <= eps)	B = 1;

		TempPoint.x = TempPoint.x + ControlPoints->points[K].x * B;
		TempPoint.y = TempPoint.y + ControlPoints->points[K].y * B;
		TempPoint.z = TempPoint.z + ControlPoints->points[K].z * B;
	}
	//CurvePoints->points.push_back(TempPoint);
	
	return TempPoint;
}

////根据 一阶导数的控制点与节点向量 计算u处的一阶导数 
pcl::PointXYZRGB CSpline::GetDerivativePointByPoints(double u)
{
	pcl::PointXYZRGB TempPoint;
	TempPoint.x = 0;
	TempPoint.y = 0;
	TempPoint.z = 0;
	TempPoint.rgba = ColorBase::RedColor;

	int KnotVectorIndex = FindUIndex(u) - 1;

	int TempdSpline = dSpline - 1;
	//for(int j = 0; j < d; j++) //d是阶数时的使用情况
	for (int j = 0; j < TempdSpline + 1; j++) //样条上的点使用前 dSpline + 1 个控制点
	{
		//int K = KnotVectorIndex - d + j + 1;
		int K = KnotVectorIndex - TempdSpline + j;

		double B = CoxdeBoor(K, TempdSpline, u);
		//cout<<"k:"<<K<<" d:"<<dSpline<<" u:"<<u<<" B:"<<B<<endl;

		TempPoint.x = TempPoint.x + OneDerivativeControlPoints->points[K].x * B;
		TempPoint.y = TempPoint.y + OneDerivativeControlPoints->points[K].y * B;
		TempPoint.z = TempPoint.z + OneDerivativeControlPoints->points[K].z * B;
	}
	//CurvePoints->points.push_back(TempPoint);
	return TempPoint;
}

//根据节点向量的第 k 阶导数矢量 2015.09.24
pcl::PointXYZRGB CSpline::GetSplineDerivativePoint(double u, int Order, bool Normalized)
{
	//if (Order == 1)
	//	return GetDerivativePointByPoints(u);

	int KnotVectorIndex = FindUIndex(u);
	pcl::PointXYZRGB DerivativePoint;
	DerivativePoint.x = 0;
	DerivativePoint.y = 0;
	DerivativePoint.z = 0;	

	for(int j = 0; j < dSpline + 1; j++) 
	//for(int j = 0; j < n; j++) 
	{
		int K = KnotVectorIndex - dSpline + j;
		//int K = j;
		double  DerivativeValue = DerivativeOfCoxdeBoor(K, dSpline, u, Order);

		DerivativePoint.x = DerivativePoint.x + ControlPoints->points[K].x * DerivativeValue;
		DerivativePoint.y = DerivativePoint.y + ControlPoints->points[K].y * DerivativeValue;
		DerivativePoint.z = DerivativePoint.z + ControlPoints->points[K].z * DerivativeValue;
	}	

	if (Normalized)
	{
		double SqrtValue = sqrt(DerivativePoint.x * DerivativePoint.x
			+ DerivativePoint.y * DerivativePoint.y + DerivativePoint.z * DerivativePoint.z);

		DerivativePoint.x = DerivativePoint.x / SqrtValue;
		DerivativePoint.y = DerivativePoint.y / SqrtValue;
		DerivativePoint.z = DerivativePoint.z / SqrtValue;
	}

	return DerivativePoint;
}

////生成均匀的样条节点向量，0为初值, 1为间距
//vector<double> CSpline::CreatePeriodicKnoteVector(int Number)
//{
//	vector<double> KnotVectorValue;
//	for(int i = 0; i < Number; i++)
//	{
//		KnotVectorValue.push_back(i);	
//	}
//	return KnotVectorValue;
//}

//画样条曲线
void CSpline::DrawSpline(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer, string SplineName)
{
	if (!CanCreateCurve) return;
	CalcBase<int> CalcBaseInt;
	for(int i = 0; i < CurvePoints->points.size() - 1; i++)
	{
		Viewer->addLine(CurvePoints->points[i],CurvePoints->points[i+1],
			SplineName + CalcBaseInt.ConvertToString(i));
	}
}

//设置节点向量
void CSpline::SetKnoteVector(vector<double> KnotVectorValue)
{
	KnotVector = KnotVectorValue;	
}

//搜索节点向量值所在的索引
int CSpline::FindUIndex(double u)
{		
	if ((u > 1 ) || (abs(u - KnotVector[nSpline]) <= eps)) return nSpline - 1;	

	int low = dSpline;
	int high = nSpline;

	int Index = (low + high)/2;
	while((u < KnotVector[Index] || u >= KnotVector[Index + 1]) && (low != high))
	{
		if (u < KnotVector[Index]) high = Index;
		else low = Index;
		Index = (low+high)/2;
	}
	return Index;
}

//生成开放均匀样条的节点向量
void CSpline::CreateOpenAndUniformKnoteVector()
{
	KnotVector.clear();
	for(int i = 0; i < nSpline + dSpline + 1; i++)
	{
		if (i <= dSpline) KnotVector.push_back(0);
		else if ( i > dSpline && i <= nSpline) KnotVector.push_back((i - dSpline));
		else if ( i > nSpline) KnotVector.push_back((nSpline - dSpline));
	}
}

//生成均匀样条的节点向量
void CSpline::CreateUniformKnoteVector()
{
	KnotVector.clear();
	for(int i = 0; i < nSpline + dSpline + 1; i++)
	{
		KnotVector.push_back(i*1.0);		
	}	
}

////获取样条的Step
//void CSpline::GetKnoteVectorStep()
//{
//	//获取节点向量的最大值与最小值
//	for(int i = dSpline; i <= nSpline; i++)
//	{
//		if (KnotVectorMinValue > KnotVector[i])
//			KnotVectorMinValue = KnotVector[i];
//		if (KnotVectorMaxValue < KnotVector[i])
//			KnotVectorMaxValue = KnotVector[i];
//	}
//	//UStep = (KnotVectorMaxValue - KnotVectorMinValue) / CurvePointsNumber;	
//}

//画样条曲线的 Order 级导数 默认 Order = 1
void CSpline::DrawSplineDerivative(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
	int StartIndex, int EndIndex, int Order, string SplineName,
	int r, int g, int b, double LineLength)
{	
	if (CurvePointsAndDerivativeS.size() == 0)
	{
		cout<<"还未获取节点的各阶导数数据"<<endl;
		return ;
	}

	if (CurvePointsAndDerivativeS[0].size() < Order + 1)
	{
		cout<<"还未获取节点的第 "<<Order<<" 阶的导数数据"<<endl;
		return ;
	}

	if (EndIndex == 0) EndIndex = CurvePointsAndDerivativeS.size();
	CalcBase<int> CalcBaseInt;
	
	for(int i = StartIndex; i < EndIndex; i++)
	{
		pcl::PointXYZRGB BasePoint = CurvePointsAndDerivativeS[i][0];
		pcl::PointXYZRGB DerivativePoint = CurvePointsAndDerivativeS[i][Order];
				
		double L = sqrt(LineLength*LineLength/
				(DerivativePoint.x * DerivativePoint.x 
				+ DerivativePoint.y * DerivativePoint.y + 
				DerivativePoint.z * DerivativePoint.z));
		DerivativePoint.x = BasePoint.x + L * DerivativePoint.x;
		DerivativePoint.y = BasePoint.y + L * DerivativePoint.y;
		DerivativePoint.z = BasePoint.z + L * DerivativePoint.z;				

		Viewer->addLine(BasePoint, DerivativePoint, r, g, b,
			SplineName + "_" + CalcBaseInt.ConvertToString(i) + "_"
			+ CalcBaseInt.ConvertToString(Order));
	}
}

//画样条曲线在 U 处的 Order 级导数 默认 Order = 1
void CSpline::DrawSplinePointDerivative(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
	double U, int Order, string SplineName, int r, int g, int b, double LineLength)
{
	pcl::PointXYZRGB BasePoint = GetSplinePoint(U);
	CalcBase<int> CalcBaseInt;
	for(int i = 1; i <= Order; i++)	
	{
		//pcl::PointXYZRGB DerivativePoint = GetSplineDerivativePoint(U, i);

		pcl::PointXYZRGB DerivativePoint = GetDerivativePointByPoints(U);
		double L = sqrt(LineLength*LineLength/
				(DerivativePoint.x * DerivativePoint.x 
				+ DerivativePoint.y * DerivativePoint.y + 
				DerivativePoint.z * DerivativePoint.z));
		DerivativePoint.x = BasePoint.x + L * DerivativePoint.x;
		DerivativePoint.y = BasePoint.y + L * DerivativePoint.y;
		DerivativePoint.z = BasePoint.z + L * DerivativePoint.z;				

		if (i == 1)
		{
			r = 0; g = 255; b =0;
		}
		else if (i == 2)
		{
			r = 0; g = 0; b =255;
		}
		else if (i == 3)
		{
			r = 255; g = 0; b =255;
		}

		Viewer->addText3D(CalcBaseInt.ConvertToString(i) + "阶导", DerivativePoint, 0.5, 1,1,1,
			SplineName+ "_Text" + CalcBaseInt.ConvertToString(i) + "_" + CalcBaseInt.ConvertToString(Order));

		Viewer->addLine(BasePoint, DerivativePoint, r, g, b,
			SplineName + "_" + CalcBaseInt.ConvertToString(i) + "_"
			+ CalcBaseInt.ConvertToString(Order));
	}
}


//计算点构成曲线的凸性 2015.12.04
void CSpline::ComputeConvexPropertyOfCurve(string Type)
{
	CAnglePartition AnglePartitionInstance;

	vector<AnglePartitionStruct> SectionAngle;
	AnglePartitionInstance.PartitionPoints(CurvePoints, 1, SectionAngle);

	CalcBase<int> CalcBaseInt;
	CalcBase<double> CalcBaseFloat;
	int NoConvexhull = 0;

	NoConvexhullPoints->points.clear();

	for(int i = 0; i < SectionAngle.size(); i++)
	{
		pcl::PointXYZRGB TempPoint;
		if (SectionAngle[i].PointIndexs.size() > 0)
		{
			//cout<<"正在处理第"<<i<<"个角度分区"<<endl;
			TempPoint = CurvePoints->points[SectionAngle[i].PointIndexs[0]];
			pcl::PointXYZRGB TangentLinePoint = 
				ResolveTangentLine(TempPoint, SectionAngle[i].PointIndexs[0], Type);
			
			bool IsConvexhull = IsConvexHullPoint(i, TangentLinePoint, SectionAngle);
			if (!IsConvexhull) 
			{
				NoConvexhullPoints->points.push_back(TempPoint);
				NoConvexhull++;		

				Viewer->addText3D(CalcBaseInt.ConvertToString(i), TempPoint,0.05,
					255,0,0,CalcBaseFloat.ConvertToString(clock()) 
				+  CalcBaseInt.ConvertToString(i));

			}
			/*
			if (!IsConvexhull)
			{

			/////////如下是画切线
				double LineLength = 5.0;
				pcl::PointXYZRGB BasePoint = TempPoint;
				pcl::PointXYZRGB DerivativePoint;
				
				double L = sqrt(LineLength*LineLength/
						(TangentLinePoint.x * TangentLinePoint.x 
						+ TangentLinePoint.y * TangentLinePoint.y + 
						TangentLinePoint.z * TangentLinePoint.z));
				DerivativePoint.x = BasePoint.x + L * TangentLinePoint.x;
				DerivativePoint.y = BasePoint.y + L * TangentLinePoint.y;
				DerivativePoint.z = BasePoint.z + L * TangentLinePoint.z;				

				BasePoint.x = BasePoint.x - L * TangentLinePoint.x;
				BasePoint.y = BasePoint.y - L * TangentLinePoint.y;
				BasePoint.z = BasePoint.z - L * TangentLinePoint.z;	

				Viewer->addLine(BasePoint, DerivativePoint, 255, 0, 0,
					"SplineName_" + CalcBaseFloat.ConvertToString(clock()) 
					+  CalcBaseInt.ConvertToString(i) + "_"
					+ CalcBaseInt.ConvertToString(1));				
			}
			//*/
		}
	}
	cout<<"共有 "<<NoConvexhull<<" 个非凸点, 非凸点比率是"<<100.0*NoConvexhull/360.0<<"%"<<endl;
}

//判断此点是否是凸曲线上的点
bool CSpline::IsConvexHullPoint(int AngleIndex, pcl::PointXYZRGB TangentLinePoint,
	vector<AnglePartitionStruct> & SectionAngle)
{
	pcl::PointXYZRGB BasePoint = CurvePoints->points[SectionAngle[AngleIndex].PointIndexs[0]];
	
	//方程 y=a*x+b
	double a = TangentLinePoint.y / TangentLinePoint.x;
	double b = BasePoint.y-a * BasePoint.x;

	if (AngleIndex == 191) 
		cout<<endl;

	int Flag = 0;
	bool Return = true;

	//选定的另外一个参考点是其对面的一个点
	int j = 180;		

	while (SectionAngle[(j + AngleIndex) % 360].PointIndexs.size() == 0)
	{
		j++;
	}
	
	int SelectIndex = (j + AngleIndex) % 360;

	pcl::PointXYZRGB SelectedPoint = CurvePoints->points[SectionAngle[SelectIndex].PointIndexs[0]];
	double Symbol = SelectedPoint.y - a*SelectedPoint.x - b;

	for(int i = 0; i < SectionAngle.size(); i++)
	{
		if (i != AngleIndex && i != SelectIndex)
		{
			if (SectionAngle[i].PointIndexs.size() > 0)
			{
				pcl::PointXYZRGB TempPoint = CurvePoints->points[SectionAngle[i].PointIndexs[0]];
				double TempSymbol = TempPoint.y - a*TempPoint.x - b;
				//if (Symbol * TempSymbol < 0 && abs(TempSymbol) > 0.001) 
				if (Symbol * TempSymbol < 0) 
				{
					Return = false;
					break;
				}
			}
		}
	}
	return Return;
}


pcl::PointXYZRGB CSpline::ResolveTangentLine(pcl::PointXYZRGB CurPoint, int PointIndex, string Type)
{
	if (Type == "BSpline")	//非均匀B样条
	{		
		return GetSplineDerivativePoint(PointUValues[PointIndex], 1);
	}
	else	//自然三次样条
	{
		
	}
}

//根据Simpson求积公式计算曲线 在 参数 u 位置处的长度 KValue 是分的段数, 
double CSpline::GetSplineLengthBySimpson(double u)
{
	int UStartIndex, UEndIndex;

	UStartIndex = FindUIndex(u);

	double TempUStart = KnotVector[UStartIndex - 1];

	double Length = SplineLengthOfKnotVector[UStartIndex - 1];

	pcl::PointXYZRGB StartPoint, EndPoint;

	StartPoint = this->GetSplinePoint(TempUStart);
	EndPoint = this->GetSplinePoint(u);

	int KValue = PointDis(StartPoint, EndPoint) / 0.001;

	double U;

	if (KValue > 0)
	{
		pcl::PointXYZRGB TempPoint;
		TempPoint = GetSplineDerivativePoint(TempUStart, 1, false);
		double aValue = sqrt(pow(TempPoint.x, 2) + pow(TempPoint.y, 2) + pow(TempPoint.z, 2));

		TempPoint = GetSplineDerivativePoint(u, 1, false);
		double bValue = sqrt(pow(TempPoint.x, 2) + pow(TempPoint.y, 2) + pow(TempPoint.z, 2));

		double XMiddleValue = 0, XkMiddleValue = 0;

		double hStep = (u - TempUStart) / KValue;

		U = TempUStart;
		double UMiddle = U + hStep / 2;

		TempPoint = GetSplineDerivativePoint(UMiddle, 1, false);
		//XMiddleValue = sqrt(1+pow(TempPoint.y/TempPoint.x,2));
		XMiddleValue = sqrt(pow(TempPoint.x, 2) + pow(TempPoint.y, 2) + pow(TempPoint.z, 2));

		U = U + hStep;//从第1个点开始执行循环，起始点是第0个点
		for (int j = 1; j < KValue; j++)
		{
			UMiddle = U + hStep / 2;
			TempPoint = GetSplineDerivativePoint(UMiddle, 1, false);
			XMiddleValue = XMiddleValue + sqrt(pow(TempPoint.x, 2) + pow(TempPoint.y, 2) + pow(TempPoint.z, 2));
			//XMiddleValue = XMiddleValue + sqrt(1 + pow(TempPoint.y/TempPoint.x,2));

			TempPoint = GetSplineDerivativePoint(U, 1, false);
			XkMiddleValue = XkMiddleValue + sqrt(pow(TempPoint.x, 2) + pow(TempPoint.y, 2) + pow(TempPoint.z, 2));
			//XkMiddleValue = XkMiddleValue + sqrt(1 + pow(TempPoint.y/TempPoint.x,2));
			U = U + hStep;
		}
		Length = Length + hStep * (aValue + 4 * XMiddleValue + 2 * XkMiddleValue + bValue) / 6;
	}

	return Length;
}

//2019.01.23 only used for 3D curve
double CSpline::GetSplineHeight()
{
	if (CurvePoints->points.size() == 0)
		return abs(GetSplinePoint(KnotVector[nSpline] - EPSM6).z - GetSplinePoint(KnotVector[dSpline]).z);
	else
		return abs(CurvePoints->points[CurvePoints->points.size() - 1].z - CurvePoints->points[0].z);
}

//根据Simpson求积公式计算曲线长度
double CSpline::GetSplineLengthBySimpson()
{
	double Length = 0;
	double UStart, UEnd, U;
	double aValue, bValue;
	//cout << "正在计算样条曲线长度！" << endl;

	//Simpson公式 分的段数		
	if (IsClosed && PointUValues.size() == 0)
	{
		//cout<<"记录节点有误！无法计算长度！"<<endl;
		return 0;
	}

	//计算各个节点向量对应区间的样条的面积

	for (int i = 0; i < KnotVector.size() - dSpline; i++)
	{
		SplineLengthOfKnotVector.push_back(0);
		SplineHeightOfKnotVector.push_back(0);
	}

	int StartUZoneIndex = FindUIndex(PointUValues[0]);
	int EndUZoneIndex = FindUIndex(PointUValues[PointUValues.size() - 1]);
	UStart = PointUValues[0];
	UEnd = PointUValues[PointUValues.size() - 1];

	pcl::PointXYZRGB SplineStartPoint = this->GetSplinePoint(UStart);

	double TempUStart, TempUEnd;
	aValue = 0; bValue = 0;
	for(int i = StartUZoneIndex; i <= EndUZoneIndex; i++) //每一段
	{		
		if (i == StartUZoneIndex)
		{
			TempUStart = PointUValues[0];
			TempUEnd = KnotVector[i+1];			
		}
		else if(i == EndUZoneIndex)
		{
			TempUStart = KnotVector[EndUZoneIndex];
			TempUEnd = PointUValues[PointUValues.size() - 1];				
		}
		else
		{
			TempUStart = KnotVector[i];
			TempUEnd = KnotVector[i+1];				
		}					
		
		pcl::PointXYZRGB StartPoint, EndPoint;

		StartPoint = this->GetSplinePoint(TempUStart);
		EndPoint = this->GetSplinePoint(TempUEnd);
		
		int KValue = PointDis(StartPoint, EndPoint) / 0.001;

		if ( KValue > 0)	
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint = GetSplineDerivativePoint(TempUStart, 1, false);			
			aValue = sqrt(pow(TempPoint.x,2)+pow(TempPoint.y,2)+pow(TempPoint.z,2));
			
			TempPoint = GetSplineDerivativePoint(TempUEnd, 1, false);			
			bValue = sqrt(pow(TempPoint.x,2)+pow(TempPoint.y,2)+pow(TempPoint.z,2));

			double XMiddleValue = 0, XkMiddleValue = 0;
			
			double hStep = (TempUEnd-TempUStart)/KValue;

			U = TempUStart;	
			double UMiddle = U + hStep/2;

			TempPoint = GetSplineDerivativePoint(UMiddle, 1, false);
			//XMiddleValue = sqrt(1+pow(TempPoint.y/TempPoint.x,2));
			XMiddleValue = sqrt(pow(TempPoint.x,2)+pow(TempPoint.y,2)+pow(TempPoint.z,2));

			U = U + hStep;//从第1个点开始执行循环，起始点是第0个点
			for(int j = 1; j < KValue; j++)
			{
				UMiddle = U + hStep/2;
				TempPoint = GetSplineDerivativePoint(UMiddle, 1, false);
				XMiddleValue = XMiddleValue + sqrt(pow(TempPoint.x,2)+pow(TempPoint.y,2)+pow(TempPoint.z,2));
				//XMiddleValue = XMiddleValue + sqrt(1 + pow(TempPoint.y/TempPoint.x,2));
				
				TempPoint = GetSplineDerivativePoint(U, 1, false);
				XkMiddleValue = XkMiddleValue + sqrt(pow(TempPoint.x,2)+pow(TempPoint.y,2)+pow(TempPoint.z,2));
				//XkMiddleValue = XkMiddleValue + sqrt(1 + pow(TempPoint.y/TempPoint.x,2));
				U = U + hStep;
			}
			Length = Length + hStep*(aValue+ 4*XMiddleValue+ 2*XkMiddleValue + bValue)/6;
		}

		//给当前节点向量的区间赋值
		//长度区间
		SplineLengthOfKnotVector[i + 1] = Length;

		//高度区间
		SplineHeightOfKnotVector[i + 1] = EndPoint.z - SplineStartPoint.z;
		
	}

	return Length;
}

	//获取闭合样条曲线所围区域的面积
double CSpline::GetSplineArea()
{
	double Area = 0;
	//cout<<"正在计算面积，请稍候"<<endl;
	if (PointUValues.size() == 0)
	{
		//cout<<"记录节点有误！无法计算面积"<<endl;
		return 0;
	}
	//起始的节点向量区间
	int StartUZoneIndex = FindUIndex(PointUValues[0]);	
	int EndUZoneIndex = FindUIndex(PointUValues[PointUValues.size() - 1]); 

	//起始的节点向量
	double UStart = PointUValues[0];
	double UEnd = PointUValues[PointUValues.size() - 1];

	double TempUStart, TempUEnd; 
	string LastDFormulaStrX = "", LastDFormulaStrY ="";
	//double LastEnd;

	for(int i = StartUZoneIndex; i <= EndUZoneIndex; i++) //每一段
	{		
		if (i == StartUZoneIndex)
		{
			TempUStart = PointUValues[0];
			TempUEnd = KnotVector[i+1];
		}
		else if(i == EndUZoneIndex)
		{
			TempUStart = KnotVector[EndUZoneIndex];
			TempUEnd = PointUValues[PointUValues.size() - 1];			
		}
		else
		{
			TempUStart = KnotVector[i];
			TempUEnd = KnotVector[i+1];				
		}

		double UMiddle = (TempUStart + TempUEnd)/2;

		vector<string> FormulaS = GetUFormula(UMiddle);
		
		pcl::PointXYZRGB TempPoint = this->GetSplinePoint(UMiddle);

		CFormulaAnalytic FormulaAnalytic;

		FormulaAnalytic.SetInputs(FormulaS[0]);
		string FormulaStrX = FormulaAnalytic.GetFormula();	

		FormulaAnalytic.SetInputs(FormulaS[1]);
		string FormulaStrY = FormulaAnalytic.GetFormula();		
		
		string DFormulaStrX = FormulaAnalytic.DerivativeStr(FormulaStrX);
		string DFormulaStrY = FormulaAnalytic.DerivativeStr(FormulaStrY);
				
		//JudgeNoConvexhullKnoteValue(i, FormulaStrX, FormulaStrY, TempUStart, TempUEnd);

		//cout<<"正在计算第 "<<i<<" 段的面积"<<endl;
		//
		//cout<<"参数对应点: "<<TempPoint<<endl;
		//cout<<"表达式计算的X坐标: "<<FormulaAnalytic.ResolveValue(FormulaStrX, UMiddle)<<endl;
		//cout<<"表达式计算的Y坐标: "<<FormulaAnalytic.ResolveValue(FormulaStrY, UMiddle)<<endl;
		//
		//cout<<"切向量:"<<endl;
		//pcl::PointXYZRGB CurrentDeriPoint = this->GetSplineDerivativePoint(UMiddle,1);
		//cout<<"参数对应点的切向量: "<<CurrentDeriPoint<<endl;
		//cout<<"表达式计算的切向量X坐标: "<<FormulaAnalytic.ResolveValue(DFormulaStrX, UMiddle)<<endl;
		//cout<<"表达式计算的切向量Y坐标: "<<FormulaAnalytic.ResolveValue(DFormulaStrY, UMiddle)<<endl;


		//if (LastDFormulaStrX != "")
		//{
		//	cout<<"上一个结束点处的导数X: "<<FormulaAnalytic.ResolveValue(LastDFormulaStrX,LastEnd)<<endl;
		//	cout<<"上一个结束点处的导数Y: "<<FormulaAnalytic.ResolveValue(LastDFormulaStrY,LastEnd)<<endl;	

		//	cout<<"当前起始点处的导数X: "<<FormulaAnalytic.ResolveValue(DFormulaStrX,	TempUStart)<<endl;
		//	cout<<"当前起始点处的导数Y: "<<FormulaAnalytic.ResolveValue(DFormulaStrY,	TempUStart)<<endl;				
		//}
		//cout<<endl;
		//LastDFormulaStrX = DFormulaStrX;
		//LastDFormulaStrY = DFormulaStrY;
		//LastEnd = TempUEnd; 
		
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

	/*	FormulaStrX = "(" + FormulaStrX + ")*(" + DFormulaStrY + ")"; 
		FormulaAnalytic.SetInputs(FormulaStrX);
		FormulaStrX = FormulaAnalytic.GetFormula();

		FormulaStrY = "(" + FormulaStrY + ")*(" + DFormulaStrX + ")"; 
		FormulaAnalytic.SetInputs(FormulaStrY);
		FormulaStrY = FormulaAnalytic.GetFormula();
		///需要有个减法运算
		string TempFormulaStrY = FormulaAnalytic.OppositeFormula(FormulaStrY);
		if (TempFormulaStrY.substr(0,1) == "+" || TempFormulaStrY.substr(0,1) == "-")
			Formula = FormulaStrX + TempFormulaStrY;
		else
			Formula = FormulaStrX + "+" + TempFormulaStrY;	*/

		Formula = FormulaAnalytic.PolynomialMerge(Formula);
		double TempArea = FormulaAnalytic.ResolveIntegralValue(Formula, TempUStart, TempUEnd);
		//cout<<"当前区域面积:"<<TempArea<<endl;
		
		Area = Area + TempArea;
	}
	//cout<<"面积计算已完成！"<<endl;
	return Area /2.0;
}

//获取非凸区域的起始范围 2015.12.22 只针对三次曲线
bool CSpline::GetNoConvexHullZone(int i, double & NoConvexHullUStart, double & NoConvexHullUEnd)
{
	if(dSpline != 3)
	{
	//	cout<<"此方法只针对于三次曲线"<<endl;	
		return true;
	}
	
	if (i >= nSpline || i < dSpline) 
	{	cout<<"超出了节点向量的取值范围"<<endl; return true;}
	
	double UStart, UEnd;
	bool ReturnValue = false;
	UStart = KnotVector[i];
	UEnd = KnotVector[i+1];

	vector<string> FormulaS = GetUFormula((UStart+UEnd)/2);

	string FormulaStrX = FormulaS[0];
	string FormulaStrY = FormulaS[1];

	CFormulaAnalytic FormulaAnalytic;

	FormulaAnalytic.SetInputs(FormulaStrX);
	FormulaStrX = FormulaAnalytic.GetFormula();	

	FormulaAnalytic.SetInputs(FormulaStrY);
	FormulaStrY = FormulaAnalytic.GetFormula();

	vector<double> XCoefficient, YCoefficient;
	vector<int> XOrder, YOrder;
	FormulaAnalytic.ExtractCofficientAndVairantOrder(FormulaStrX, XCoefficient, XOrder);
	FormulaAnalytic.ExtractCofficientAndVairantOrder(FormulaStrY, YCoefficient, YOrder);

	if (XCoefficient.size() > dSpline + 1)
		cout<<"多项式混合计算错误，X表达式的阶数"<<XCoefficient.size()
			<<"大于样条指定的阶数"<<dSpline<<endl;
	else if(XCoefficient.size() < dSpline + 1)
	{
		int i = 0; 
		while(i < XOrder.size())
		{
			if (XOrder[i] != (dSpline - i))
			{
				XOrder.insert(XOrder.begin() + i, (dSpline - i));
				XCoefficient.insert(XCoefficient.begin() + i, 0);
			}
			i++;
		}
	}

	if (YCoefficient.size() > dSpline + 1)
		cout<<"多项式混合计算错误，Y表达式的阶数"<<XCoefficient.size()
			<<"大于样条指定的阶数"<<dSpline<<endl;
	else if(YCoefficient.size() < dSpline + 1)
	{
		int i = 0; 
		while(i < YOrder.size())
		{
			if (YOrder[i] != (dSpline - i))
			{
				YOrder.insert(YOrder.begin() + i, (dSpline - i));
				YCoefficient.insert(YCoefficient.begin() + i, 0);
			}
			i++;
		}
	}
	
	double a = 6*(YCoefficient[0]*XCoefficient[1] - XCoefficient[0]*YCoefficient[1]);	
	double b = 6*(YCoefficient[0]*XCoefficient[2] - XCoefficient[0]*YCoefficient[2]);
	double c = 2*(YCoefficient[1]*XCoefficient[2] - XCoefficient[1]*YCoefficient[2]);

	double d = (b*b-4*a*c);
	if (d < 0 && a > 0)
		ReturnValue = true;
		//cout<<"此区间无凹性区域"<<endl;
	else if(d > 0 && a > 0)
	{
		double RootLeft, RootRight;
		RootLeft = (-b-sqrt(d))/2/a;
		RootRight = (-b+sqrt(d))/2/a;
		if (UEnd < RootLeft || UStart > RootRight)
		{	
			cout<<"此区间无凹性区域"<<endl;
			ReturnValue = true;
		}
		else if( UStart > RootLeft &&  UEnd < RootRight)
		{
			ReturnValue = false;
			NoConvexHullUStart = UStart;
			NoConvexHullUEnd = UEnd;
			cout<<"此区间全部是凹性区域"<<endl;
		}
		else
		{
			if( UStart > RootLeft &&  UStart < RootRight)			
			{
				cout<<"第"<<i<<"个区间凹性区域为 "<< UStart<<" 至 "<<RootRight<<endl;
				ReturnValue = false;
				NoConvexHullUStart = UStart;
				NoConvexHullUEnd = RootRight;
			}
			else if( UEnd > RootLeft &&  UEnd < RootRight)
			{
				cout<<"第"<<i<<"个区间凹性区域为 "<< RootLeft<<" 至 "<<UEnd<<endl;
				ReturnValue = false;
				NoConvexHullUStart = RootLeft;
				NoConvexHullUEnd = UEnd;
			}
		}
	}
	else if (d > 0 && a < 0)
	{
		double RootLeft, RootRight;
		RootLeft = (-b+sqrt(d))/2/a;
		RootRight = (-b-sqrt(d))/2/a;
		if (UStart > RootLeft && UEnd < RootRight)
		{
			cout<<"第"<<i<<"个区间无凹性区域"<<endl;
			ReturnValue = true;
		}
		else if (UStart < RootLeft && UEnd > RootLeft)
		{
			cout<<"第"<<i<<"个区间凹性区域为 "<<UStart<<" 至 "<<RootLeft<<endl;
			ReturnValue = false;
			NoConvexHullUStart = UStart;
			NoConvexHullUEnd = RootLeft;
		}
		else if (UStart < RootLeft && UEnd > RootRight)	//分两步进行
		{
			cout<<"第"<<i<<"个区间凹性区域为 "<<UStart<<" 至 "<<RootLeft <<" 且 包括"<<RootRight<<" 至 "<<UEnd<<endl;
			ReturnValue = false;
			NoConvexHullUStart = UStart;
			NoConvexHullUEnd = RootLeft;
		}
		else if (UStart < RootRight && UEnd > RootRight)
		{
			cout<<"第"<<i<<"个区间凹性区域为 "<<RootRight<<" 至 "<<UEnd<<endl;
			ReturnValue = false;
			NoConvexHullUStart = RootRight;
			NoConvexHullUEnd = UEnd;
		}	
		else if (RootRight < UStart ||  UEnd < RootLeft)
		{
			ReturnValue = false;
			NoConvexHullUStart = UStart;
			NoConvexHullUEnd = UEnd;
			cout<<"此区间全部是凹性区域"<<endl;
		}
	}
	return ReturnValue;
}

//计算非凸区域的位置 2015.12.22 此处只针对三次样条
void CSpline::JudgeNoConvexhullKnoteValue(int i, string XFormula, string YFormula, 
		double UStart, double UEnd)
{
	if(dSpline != 3){cout<<"此方法只针对于三次曲线"<<endl;	return;}
	vector<double> XCoefficient, YCoefficient;
	vector<int> XOrder, YOrder;
	CFormulaAnalytic FormulaAnalytic;
	FormulaAnalytic.ExtractCofficientAndVairantOrder(XFormula, XCoefficient, XOrder);
	FormulaAnalytic.ExtractCofficientAndVairantOrder(YFormula, YCoefficient, YOrder);
	
	cout<<"第"<<i<<"个区间 "<<endl;
	//if (i == 6)
	//	cout<<endl;
	if (XCoefficient.size() > dSpline + 1)
		cout<<"多项式混合计算错误，X表达式的阶数"<<XCoefficient.size()
			<<"大于样条指定的阶数"<<dSpline<<endl;
	else if(XCoefficient.size() < dSpline + 1)
	{
		int i = 0; 
		while(i < XOrder.size())
		{
			if (XOrder[i] != (dSpline - i))
			{
				XOrder.insert(XOrder.begin() + i, (dSpline - i));
				XCoefficient.insert(XCoefficient.begin() + i, 0);
			}
			i++;
		}
	}

	if (YCoefficient.size() > dSpline + 1)
		cout<<"多项式混合计算错误，Y表达式的阶数"<<XCoefficient.size()
			<<"大于样条指定的阶数"<<dSpline<<endl;
	else if(YCoefficient.size() < dSpline + 1)
	{
		int i = 0; 
		while(i < YOrder.size())
		{
			if (YOrder[i] != (dSpline - i))
			{
				YOrder.insert(YOrder.begin() + i, (dSpline - i));
				YCoefficient.insert(YCoefficient.begin() + i, 0);
			}
			i++;
		}
	}
	
	double a = 6*(YCoefficient[0]*XCoefficient[1] - XCoefficient[0]*YCoefficient[1]);	
	double b = 6*(YCoefficient[0]*XCoefficient[2] - XCoefficient[0]*YCoefficient[2]);
	double c = 2*(YCoefficient[1]*XCoefficient[2] - XCoefficient[1]*YCoefficient[2]);

	double d = (b*b-4*a*c);
	if (d < 0 && a > 0)
		cout<<"此区间无凹性区域"<<endl;
	else if(d > 0 && a > 0)
	{
		double RootLeft, RootRight;
		RootLeft = (-b-sqrt(d))/2/a;
		RootRight = (-b+sqrt(d))/2/a;
		if (UEnd < RootLeft || UStart > RootRight)
		{	//cout<<"此区间无凹性区域"<<endl;
		}
		else if( UStart > RootLeft &&  UEnd < RootRight)
			cout<<"此区间全部是凹性区域"<<endl;
		else
		{
			if( UStart > RootLeft &&  UStart < RootRight)			
				cout<<"此区间凹性区域为 "<< UStart<<" 至 "<<RootRight<<endl;
			else if( UEnd > RootLeft &&  UEnd < RootRight)
				cout<<"此区间凹性区域为 "<< RootLeft<<" 至 "<<UEnd<<endl;
		}
	}
	else if (d > 0 && a < 0)
	{
		double RootLeft, RootRight;
		RootLeft = (-b+sqrt(d))/2/a;
		RootRight = (-b-sqrt(d))/2/a;
		if (UStart > RootLeft && UEnd < RootRight)
		{
			//cout<<"此区间无凹性区域"<<endl;
		}
		else if (UStart < RootLeft && UEnd > RootLeft)
			cout<<"此区间凹性区域为 "<<UStart<<" 至 "<<RootLeft<<endl;
		else if (UStart < RootLeft && UEnd > RootRight)
			cout<<"此区间凹性区域为 "<<UStart<<" 至 "<<RootLeft <<" 且 包括"<<RootRight<<" 至 "<<UEnd<<endl;
		else if (UStart < RootRight && UEnd > RootRight)
			cout<<"此区间凹性区域为 "<<RootRight<<" 至 "<<UEnd<<endl;
		
	}
}



void CSpline::HomogenizationSplineInterPolatingPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints)
{
	double AvgDis = 0;
	CalcBase<double> CalcBaseFloat;
	vector<double> VecDis;

	for(int i = 0; i < TempPoints->points.size() - 1; i++)
	{
		double TempDis = PointDis(TempPoints->points[i], TempPoints->points[i+1]);
		VecDis.push_back(TempDis);
		AvgDis = AvgDis +  TempDis / TempPoints->points.size();
	}

	int AddNumber = 0;
	for(int i = 0; i < VecDis.size(); i++)
	{
		int K = VecDis[i] / AvgDis;
		
		if (K > 2)
		{
			double XStep = (TempPoints->points[i + AddNumber + 1].x - TempPoints->points[i + AddNumber].x) / K;
			double YStep = (TempPoints->points[i + AddNumber + 1].y - TempPoints->points[i + AddNumber].y) / K;
			double ZStep = (TempPoints->points[i + AddNumber + 1].z - TempPoints->points[i + AddNumber].z) / K;
		
			for(int j = 1; j < K; j++)
			{
				pcl::PointXYZRGB NewPoint;
				NewPoint.x = TempPoints->points[i + AddNumber].x + XStep;
				NewPoint.y = TempPoints->points[i + AddNumber].y + YStep;
				NewPoint.z = TempPoints->points[i + AddNumber].z + ZStep;
				TempPoints->insert(TempPoints->begin() + i + AddNumber + 1, NewPoint);			
				AddNumber = AddNumber + 1;
			}			
		}
	}	
}

//获取特定点的曲率 Relative = true时，是相对曲率
double CSpline::GetCurvature(double U, bool Relative)
{
	pcl::PointXYZRGB OneDerivative = GetSplineDerivativePoint(U, 1, false);
	pcl::PointXYZRGB TwoDerivative = GetSplineDerivativePoint(U, 2, false);	
	
	pcl::PointXYZRGB OneCrossTwo = PointBase::PointsCrossProduct(OneDerivative, TwoDerivative);
	pcl::PointXYZRGB UnitPoint;
	UnitPoint.x = 0; UnitPoint.y = 0; UnitPoint.z = 1;

	double Denominator = pow(sqrt(pow(OneDerivative.x, 2) + pow(OneDerivative.y, 2) + pow(OneDerivative.z, 2)),3);
	
	if (Denominator < eps) 
		return 0;
	else
	{
		if (Relative)
			return PointBase::PointsDotProduct(OneCrossTwo, UnitPoint) / Denominator;
		else
			return sqrt(pow(OneCrossTwo.x,2) + pow(OneCrossTwo.y, 2) + pow(OneCrossTwo.z,2)) /Denominator;
	}
}

//获取特定点的挠率
double CSpline::GetTorsion(double U)
{
	pcl::PointXYZRGB OneDerivative = GetSplineDerivativePoint(U, 1, false);
	pcl::PointXYZRGB TwoDerivative = GetSplineDerivativePoint(U, 2, false);
	pcl::PointXYZRGB ThreeDerivative = GetSplineDerivativePoint(U, 3, false);

	pcl::PointXYZRGB OneCrossTwo = PointBase::PointsCrossProduct(OneDerivative, TwoDerivative);
	double Denominator = pow(OneCrossTwo.x,2) + pow(OneCrossTwo.y, 2) + pow(OneCrossTwo.z,2);
	if (Denominator < eps) 
		return 0;
	else
		return PointBase::PointsDotProduct(OneCrossTwo, ThreeDerivative) / Denominator;		
}

//获取特定点位置处切线与水平面的夹角
double CSpline::GetInclination(double U)
{
	pcl::PointXYZRGB OneDerivative = GetSplineDerivativePoint(U, 1, false);
	//2019.02.14 
	return 90.0 - 180.0 * GeometryBase::AngleOfTwoVector(
		OneDerivative.x, OneDerivative.y, OneDerivative.z, 0, 0, 1) / M_PI;	
}

	//获取节点向量中对应的样条曲线上的点
void CSpline::GetKnotValuePoints()
{
	KnotValuePoints->points.clear();

	for(int i = dSpline; i <= nSpline; i++)	//在这些节点之间 注意索引关系
	{
		pcl::PointXYZRGB TempPoint;
		if ( i == nSpline)
			TempPoint = GetSplinePoint(0.999);	//	插入最后一个节点,非常接近于最后一个插值点
		else
			TempPoint = GetSplinePoint(KnotVector[i]);
		KnotValuePoints->points.push_back(TempPoint);
	}
}

//根据x(y或z)坐标获取 样条曲线上的节点U值
//采取逼近法实现
vector<double> CSpline::GetCurvePointsUByValue(double Value, std::string Axis, double LocateEps)
{
	if (KnotValuePoints->points.size() == 0)
		GetKnotValuePoints();

	vector<double> ResultU;

	for(int i = 1; i < KnotValuePoints->points.size(); i++)
	{
		if (Axis == "X")
		{
			if (KnotValuePoints->points[i-1].x < Value && KnotValuePoints->points[i].x > Value)
			{
				double UValue = FindUValueBetweenUParamters(Value, "X", 
					KnotVector[dSpline + i - 1], KnotVector[dSpline + i], LocateEps);
				ResultU.push_back(UValue);
				//pcl::PointXYZRGB TempPoint = GetSplinePoint(UValue);
				//ResultPoints->points.push_back(TempPoint);
			}
		}
		else if (Axis == "Y")
		{
			if (KnotValuePoints->points[i-1].y < Value && KnotValuePoints->points[i].y > Value)
			{
				double UValue = FindUValueBetweenUParamters(Value, "Y", 
					KnotVector[dSpline + i - 1], KnotVector[dSpline + i], LocateEps);
				ResultU.push_back(UValue);
				//pcl::PointXYZRGB TempPoint = GetSplinePoint(UValue);
				//ResultPoints->points.push_back(TempPoint);
			}
		}
		else if (Axis == "Z")
		{
			if (KnotValuePoints->points[i-1].z < Value && KnotValuePoints->points[i].z > Value)
			{
				double UValue = FindUValueBetweenUParamters(Value, "Z", 
					KnotVector[dSpline + i -1], KnotVector[dSpline + i], LocateEps);
				ResultU.push_back(UValue);
				//pcl::PointXYZRGB TempPoint = GetSplinePoint(UValue);
				//ResultPoints->points.push_back(TempPoint);
			}
		}
	}
	return ResultU;
}

//根据给定的坐标值计算该坐标值下对应的节点
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CSpline::GetCurvePointsByValue(
	double Value, std::string Axis, double LocateEps)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Result;
	Result.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	vector<double> U = GetCurvePointsUByValue(Value, Axis, LocateEps);

	for(int i = 0; i < U.size(); i++)
	{
		Result->points.push_back(GetSplinePoint(U[i]));
	}

	return Result;
}

	//在UMin值 与 UMin 值间 使用二分法定位 定位 Axis 轴中 Value 对应的 U值
double CSpline::FindUValueBetweenUParamters(double Value, 
	std::string Axis, double UMin, double UMax, double LocateEps )
{	
	if (UMax == 1)	UMax = 0.9999999; 

	if (abs(UMax - UMin) < EPSM6) 
		return (UMin + UMax) / 2;

	double UMiddle = (UMin+UMax)/2;
	pcl::PointXYZRGB MinPoint = GetSplinePoint(UMin);
	pcl::PointXYZRGB MaxPoint = GetSplinePoint(UMax);
	pcl::PointXYZRGB MiddlePoint = GetSplinePoint(UMiddle);
	
	if (Axis == "X")
	{
		if (abs(MinPoint.x - Value) <= LocateEps)
			return UMin;
		else if (abs(MaxPoint.x - Value) <= LocateEps)
			return UMax;
		else if (abs(MiddlePoint.x - Value) <= LocateEps)
			return UMiddle;
		else if (MinPoint.x < Value && Value < MiddlePoint.x)
			return FindUValueBetweenUParamters(Value, "X", UMin, UMiddle);
		else if (MiddlePoint.x < Value && Value < MaxPoint.x)
			return FindUValueBetweenUParamters(Value, "X", UMiddle, UMax);
	}
	else if(Axis == "Y")
	{
		if (abs(MinPoint.y - Value) <= LocateEps)
			return UMin;
		else if (abs(MaxPoint.y - Value) <= LocateEps)
			return UMax;
		else if (abs(MiddlePoint.y - Value) <= LocateEps)
			return UMiddle;
		else if (MinPoint.y < Value && Value < MiddlePoint.y)
			return FindUValueBetweenUParamters(Value, "Y", UMin, UMiddle);
		else if (MiddlePoint.y < Value && Value < MaxPoint.y)
			return FindUValueBetweenUParamters(Value, "Y", UMiddle, UMax);	
	}
	else if(Axis == "Z")
	{
		if (abs(MinPoint.z - Value) <= LocateEps)
			return UMin;
		else if (abs(MaxPoint.z - Value) <= LocateEps)
			return UMax;
		else if (abs(MiddlePoint.z - Value) <= LocateEps)
			return UMiddle;
		else if (MinPoint.z < Value && Value < MiddlePoint.z)
			return FindUValueBetweenUParamters(Value, "Z", UMin, UMiddle);
		else if (MiddlePoint.z < Value && Value < MaxPoint.z)
			return FindUValueBetweenUParamters(Value, "Z", UMiddle, UMax);
	}	
}

//获取样条长度位置处的U值 此处需要改进，需要使用Simpson公式计算(2016.01.21)
double CSpline::GetUValueBySplineLength(double TempLength)
{
	//2018.09.25 modified by yl 
	VectorBase<double> DoubleCalcBase;
	
	if (SplineLengthOfKnotVector.size() == 0)	//先计算样条曲线长度
		SplineLength = GetSplineLengthBySimpson();

	int Index = DoubleCalcBase.FindValueByBinarySearch(SplineLengthOfKnotVector, TempLength);

	if (Index == -1) return 0;

	double StartU = KnotVector[Index];
	double EndU = KnotVector[Index + 1];

	double StartULength = SplineLengthOfKnotVector[Index];
	double EndULength = SplineLengthOfKnotVector[Index + 1];
	
	if (abs(TempLength - StartULength) <= eps) return StartU;
	if (abs(TempLength - EndULength) <= eps) return EndU;

	double TempU = (StartU + EndU) / 2;
	double TempULength = GetSplineLengthBySimpson(TempU);

	while ((abs(TempLength - TempULength) > eps ) && (abs(StartU - EndU) > eps))
	{
		if (TempLength < TempULength) EndU = TempU;
		else StartU = TempU;
		TempU = (StartU + EndU) / 2;
		TempULength = GetSplineLengthBySimpson(TempU);
	}
	return TempU;	
}

//获取样条高度位置处的U值
double CSpline::GetUValueBySplineHeight(double TempHeight)
{
	//2018.09.25 modified by yl 
	VectorBase<double> DoubleCalcBase;

	if (SplineLengthOfKnotVector.size() == 0)	//先计算样条曲线长度
		SplineLength = GetSplineLengthBySimpson();

	int Index = DoubleCalcBase.FindValueByBinarySearch(SplineHeightOfKnotVector, TempHeight);

	if (Index == -1) return 0;

	double StartU = KnotVector[Index];
	double EndU = KnotVector[Index + 1];

	double StartULength = SplineHeightOfKnotVector[Index];
	double EndULength = SplineHeightOfKnotVector[Index + 1];

	if (abs(TempHeight - StartULength) <= eps) return StartU;
	if (abs(TempHeight - EndULength) <= eps) return EndU;
	
	double TempU = (StartU + EndU) / 2;

	pcl::_PointXYZRGB SplineStartPoint = this->GetSplinePoint(PointUValues[0]);
	pcl::_PointXYZRGB CurrentPoint = this->GetSplinePoint(TempU);
	double TempUHeight = CurrentPoint.z - SplineStartPoint.z;	
		
	while ((abs(TempHeight - TempUHeight) > eps) && (abs(StartU - EndU) > eps))
	{
		if (TempHeight < TempUHeight) EndU = TempU;
		else StartU = TempU;
		TempU = (StartU + EndU) / 2;

		CurrentPoint = this->GetSplinePoint(TempU);
		TempUHeight = CurrentPoint.z - SplineStartPoint.z;		
	}
	return TempU;
}

//获取一阶导的数据信息 根据节点间的关系 与 节点向量 间的关系
void CSpline::GetOneDerivativeData()
{
	OneDerivativeKnotVector.clear();
	OneDerivativeControlPoints->points.clear();

	OneDerivativeKnotVector.insert(OneDerivativeKnotVector.begin(),
		KnotVector.begin()+1,KnotVector.end()-1);

	for(int i = 0; i < ControlPoints->points.size() - 1; i++)
	{
		double k = dSpline;
		pcl::PointXYZRGB TempPoint = PointBase::PointsMinus(ControlPoints->points[i+1],
			ControlPoints->points[i]);
		k = k / (OneDerivativeKnotVector[i+dSpline] - OneDerivativeKnotVector[i]);
		TempPoint = PointBase::PointsMutile(TempPoint, k);
		OneDerivativeControlPoints->points.push_back(TempPoint);
	}
}



/////*******************************************
/////*******************************************
/////*******************************************
/////*******************************************以下为有理B样条的实现类



//生成控制点权重 现阶段权重都设置为1
void CRationalSpline::CreatePointWeights()
{
	PointWeights.clear();
	for(int i = 0; i < ControlPoints->points.size(); i++)
	{
		//PointWeights.push_back(i*1.0); 权重对线条的影响
		PointWeights.push_back(1);	
	}
}

////设置控制点权重
//void CRationalSpline::SetPointWeights(vector<double> PointWeightsValue)
//{
//	if (PointWeightsValue.size() != ControlPoints->points.size())
//	{
//		cout<<"控制点权限与控制点的个数不相同，请检查后再试"<<endl;
//		this->CanCreateCurve = false;
//		return;
//	}
//
//	PointWeights.clear();	
//	PointWeights.insert(PointWeightsValue.begin(), PointWeightsValue.end());
//}

////生成有理B样条的点
//void CRationalSpline::CreateRationalSpline(int DerivativeOrder)
//{
//	if (KnotVector.size() == 0) 
//	{
//		CanCreateCurve = false;
//		cout<<"节点向量为空，无法构建样条！"<<endl;
//		return; 		
//	}
//
//	if (KnotVector.size() != nSpline + dSpline + 1) 
//	{
//		cout<<"节点向量个数不等于控制点的个数(n)与阶参数的个数(d)的和不等，请核实！"<<endl;
//		CanCreateCurve = false;
//		return;
//	}
//
//	if (PointWeights.size() != ControlPoints->points.size())
//	{
//		cout<<"控制点权限与控制点的个数不相同，请检查后再试"<<endl;
//		CanCreateCurve = false;
//		return;
//	}
//
//	//if ( DerivativeOrder > d) DerivativeOrder = d;	//求导最大次数不超过 样条次数
//
//	cout<<"正在构建 NURBS 样条曲线上的点"<<endl;
//	
//	//获取节点向量的步长
//	GetKnoteVectorStep();
//
//	//从KnotVector[d] 开始 至 KnotVector[n] 结束 此处的n为节点个数加1 d 是样条的次数
//	double U = KnotVector[dSpline];		
//	//注意节点向量的起始位置
//	while((U - KnotVector[nSpline]) < eps)
//	{		
//		PointAndDerivative CurrentPoint;
//		
//		pcl::PointXYZRGB TempPoint = GetSplinePoint(U);		
//		
//		//记录当前的节点
//		CurvePoints->points.push_back(TempPoint);
//		
//		//if (abs(U - 0.997499)  < eps)
//		//{
//		//	cout<<FindUIndex(U)<<endl;
//		//}
//		//求各阶导数  有理样条的求导需要调整
//		if (DerivativeOrder > 0)
//		{
//			CurrentPoint.push_back(TempPoint);
//			for (int Order = 1; Order <= DerivativeOrder; Order++)	//依次求各阶导数
//			{
//				pcl::PointXYZRGB DerivativePoint = 
//					GetSplineDerivativePoint(U, Order);
//
//				CurrentPoint.push_back(DerivativePoint);
//			}	
//			CurvePointsAndDerivativeS.push_back(CurrentPoint);
//		}
//		//求各阶导数
//		//cout<<"当前 u 值为 " <<U<<" 此处的曲率是 "<<GetCurvature(U)<<" 此处的挠率是 "<<GetTorsion(U)<<endl; 
//
//		U = U + UStep;
//	}
//	cout<<"有理样条曲线构建完毕"<<endl;
//}

//计算有理B样条的基函数Ni,p(u)的k阶导数   i 是节点向量索引，d是样条多项式的次数, k是求导的次数 
// 计算方法见《非均匀有理B样条》 第91页 公式 4.8
pcl::PointXYZRGB CRationalSpline::GetSplineDerivativePoint(double u, int k, bool Normalized)
{
	pcl::PointXYZRGB ResultPoint;
	ResultPoint.x = 0;
	ResultPoint.y = 0;
	ResultPoint.z = 0;

	pcl::PointXYZRGB DerivativePointA;
	pcl::PointXYZRGB DerivativePointWC;

	DerivativePointA = DerivativeOfRationalCoxdeBoor_A(dSpline, u, k);
	
	DerivativePointWC.x = 0;
	DerivativePointWC.y = 0;
	DerivativePointWC.z = 0;
	
	if (k == 1)	//求一阶导数时的处理
	{
		double W = DerivativeOfRationalCoxdeBoor_W(dSpline, u, 1);
		DerivativePointWC = GetSplinePoint(u);

		DerivativePointWC.x = DerivativePointWC.x * W;
		DerivativePointWC.y = DerivativePointWC.y * W;
		DerivativePointWC.z = DerivativePointWC.z * W;		
	}
	else	//二阶及其以上导数
	{
		//计算二次项系数与 w 和 C 的对应的导数
		for(int i = 1; i <= k; i++)
		{
			double TempCombinatorialNumber = Math::CombinatorialNumber(k, i);

			double W = DerivativeOfRationalCoxdeBoor_W(dSpline, u, i);
		
			pcl::PointXYZRGB TempDerivative = GetSplineDerivativePoint(u, k - i);

			DerivativePointWC.x = DerivativePointWC.x + TempCombinatorialNumber * TempDerivative.x * W;
			DerivativePointWC.y = DerivativePointWC.y + TempCombinatorialNumber * TempDerivative.y * W;
			DerivativePointWC.z = DerivativePointWC.z + TempCombinatorialNumber * TempDerivative.z * W;
		}
	}
	//分母
	double w = RationalCoxdeBoor_W(dSpline, u);

	ResultPoint.x = (DerivativePointA.x - DerivativePointWC.x)/w;
	ResultPoint.y = (DerivativePointA.y - DerivativePointWC.y)/w;
	ResultPoint.z = (DerivativePointA.z - DerivativePointWC.z)/w;

	if (Normalized)
	{
		double SqrtValue = sqrt(ResultPoint.x * ResultPoint.x
			+ ResultPoint.y * ResultPoint.y + ResultPoint.z * ResultPoint.z);

		ResultPoint.x = ResultPoint.x / SqrtValue;
		ResultPoint.y = ResultPoint.y / SqrtValue;
		ResultPoint.z = ResultPoint.z / SqrtValue;
	}

	return ResultPoint;
}

//计算方法见《非均匀有理B样条》 第91页 公式 4.8 中的 A 的 k 阶导数
pcl::PointXYZRGB CRationalSpline::DerivativeOfRationalCoxdeBoor_A(int d, double u, int k)
{
	int KnotVectorIndex = FindUIndex(u);

	pcl::PointXYZRGB DerivativePoint;
	DerivativePoint.x = 0;
	DerivativePoint.y = 0;
	DerivativePoint.z = 0;	
			
	for(int j = 0; j < d + 1; j++) 
	{
		int K = KnotVectorIndex - d + j;
		double  DerivativeValue = DerivativeOfCoxdeBoor(K, d, u, k);

		DerivativePoint.x = DerivativePoint.x + 
			ControlPoints->points[K].x * DerivativeValue * PointWeights[K];
		DerivativePoint.y = DerivativePoint.y + 
			ControlPoints->points[K].y * DerivativeValue * PointWeights[K];
		DerivativePoint.z = DerivativePoint.z + 
			ControlPoints->points[K].z * DerivativeValue * PointWeights[K];
	}		
	return DerivativePoint;	
}

//计算方法见《非均匀有理B样条》 第91页 公式 4.8 中的 W 的 k 阶导数
double CRationalSpline::DerivativeOfRationalCoxdeBoor_W(int d, double u, int k)
{
	int KnotVectorIndex = FindUIndex(u);

	double Result = 0;

	for(int j = 0; j < d + 1; j++) 
	{
		int K = KnotVectorIndex - d + j;
		double  DerivativeValue = DerivativeOfCoxdeBoor(K, d, u, k);

		Result = Result + DerivativeValue * PointWeights[K];				
	}
	return Result;	
}


//计算 公式 4.8 的分母
double CRationalSpline::RationalCoxdeBoor_W(int d, double u)
{
	int KnotVectorIndex = FindUIndex(u);	
	double Result = 0;
	for(int j = 0; j < d + 1; j++) 
	{
		int K = KnotVectorIndex - d + j;		

		Result = Result + CoxdeBoor(K, d, u) * PointWeights[K];
	}
	return Result;
}

//根据u值返回 u 的字符串公式表达式 2015.10.17
vector<string> CRationalSpline::GetUFormula(double U)
{
	vector<string> ResultStrS;

	ResultStrS.push_back("");
	ResultStrS.push_back("");
	ResultStrS.push_back("");

	int KnotVectorIndex = FindUIndex(U);
	CalcBase<double> CalcBaseFloat;
	string DenominatorStr = "";

	//for(int j = 0; j < d; j++) //d是阶数时的使用情况
	for(int j = 0; j < dSpline + 1; j++) //样条上的点使用前 dSpline + 1 个控制点
	{			
		//int K = KnotVectorIndex - d + j + 1;
		int K = KnotVectorIndex - dSpline + j;	

		double CoxdeBoorValue = CoxdeBoor(K, dSpline, U);
		string CoxdeBoorStr = CoxdeBoorFormula(K, dSpline, U);
		//cout<<"k:"<<K<<" d:"<<dSpline<<" u:"<<u<<" B:"<<B<<endl;
		

		if (ResultStrS[0] == "")
		{
			ResultStrS[0] = CalcBaseFloat.ConvertToString(ControlPoints->points[K].x) 
				+ " * (" + CoxdeBoorStr + ") * " + CalcBaseFloat.ConvertToString(PointWeights[K]);
			ResultStrS[1] = CalcBaseFloat.ConvertToString(ControlPoints->points[K].y) 
				+ " * (" + CoxdeBoorStr + ") * " + CalcBaseFloat.ConvertToString(PointWeights[K]);
			ResultStrS[2] = CalcBaseFloat.ConvertToString(ControlPoints->points[K].z) 
				+ " * (" + CoxdeBoorStr + ") * " + CalcBaseFloat.ConvertToString(PointWeights[K]);

			//Denominator = Denominator + B * PointWeights[K];
			DenominatorStr = "(" + CoxdeBoorStr + ") * " + CalcBaseFloat.ConvertToString(PointWeights[K]);
		}
		else
		{
			ResultStrS[0] = ResultStrS[0] + " + " 
				+ CalcBaseFloat.ConvertToString(ControlPoints->points[K].x) 
				+ " * (" + CoxdeBoorStr + ") * " + CalcBaseFloat.ConvertToString(PointWeights[K]);
			ResultStrS[1] = ResultStrS[1] + " + " 
				+ CalcBaseFloat.ConvertToString(ControlPoints->points[K].y) 
				+ " * (" + CoxdeBoorStr + ") * " + CalcBaseFloat.ConvertToString(PointWeights[K]);
			ResultStrS[2] = ResultStrS[2] + " + " 
				+ CalcBaseFloat.ConvertToString(ControlPoints->points[K].z) 
				+ " * (" + CoxdeBoorStr + ") * " + CalcBaseFloat.ConvertToString(PointWeights[K]);		

			DenominatorStr = DenominatorStr + " + " + "(" + CoxdeBoorStr + ") * " + CalcBaseFloat.ConvertToString(PointWeights[K]);
		}
	}

	ResultStrS[0] = ResultStrS[0] + "/(" + DenominatorStr + ")";
	ResultStrS[1] = ResultStrS[1] + "/(" + DenominatorStr + ")";
	ResultStrS[2] = ResultStrS[2] + "/(" + DenominatorStr + ")";

	return ResultStrS;	
}


//根据节点向量的值获取节点 2015.09.24
pcl::PointXYZRGB CRationalSpline::GetSplinePoint(double U)
{
	// if this procedure has error, please ensure that the value of PointWeights is assigned.
	if (PointWeights.size() == 0) 
		CreatePointWeights();

	pcl::PointXYZRGB TempPoint;
	TempPoint.x = 0;
	TempPoint.y = 0;
	TempPoint.z = 0;
	TempPoint.rgba = ColorBase::RedColor;	

	int KnotVectorIndex = FindUIndex(U);
		
	double Denominator = 0;
	//for(int j = 0; j < d; j++) //d是阶数时的使用情况
	for(int j = 0; j < dSpline + 1; j++) 
	{			
		//int K = KnotVectorIndex - d + j + 1;
		int K = KnotVectorIndex - dSpline + j;

		double B = CoxdeBoor(K, dSpline, U);
		//cout<<"k:"<<K<<" d:"<<dSpline<<" u:"<<U<<" B:"<<B<<endl;
								
		TempPoint.x = TempPoint.x + ControlPoints->points[K].x * B * PointWeights[K];
		TempPoint.y = TempPoint.y + ControlPoints->points[K].y * B * PointWeights[K];
		TempPoint.z = TempPoint.z + ControlPoints->points[K].z * B * PointWeights[K];
		Denominator = Denominator + B * PointWeights[K];

	}
	TempPoint.x = TempPoint.x / Denominator;
	TempPoint.y = TempPoint.y / Denominator;
	TempPoint.z = TempPoint.z / Denominator;

	return TempPoint;
}
