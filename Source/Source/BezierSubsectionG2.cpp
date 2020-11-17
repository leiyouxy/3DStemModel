#include "BezierSubsectionG2.h"

void CBezierSubsectionG2::CalcCoefficientRateAndMaxCoefficientValue()
{
	int n = QPoints->points.size();
	double ValueT, Value0;
	for(int i = 0; i < QPoints->points.size(); i++)
	{
		pcl::PointXYZRGB PriorPoint = QPoints->points[(i-1+n)%n];
		pcl::PointXYZRGB CurPoint = QPoints->points[i];
		pcl::PointXYZRGB NextOnePoint = QPoints->points[(i+1+n)%n];
		pcl::PointXYZRGB NextTwoPoint = QPoints->points[(i+2+n)%n];
		pcl::PointXYZRGB NextThreePoint = QPoints->points[(i+3+n)%n];
		
		Value0 =  ((NextOnePoint.x - PriorPoint.x)*(CurPoint.y-NextTwoPoint.y)-(NextOnePoint.y-PriorPoint.y)*(CurPoint.x - NextTwoPoint.x));

		ValueT = ((NextOnePoint.x - CurPoint.x)*(CurPoint.y - NextTwoPoint.y)-(NextOnePoint.y - CurPoint.y)*(CurPoint.x - NextTwoPoint.x));		
		ValueT = Size*ValueT / Value0;
		MaxCoefficientValue.push_back(ValueT);

		ValueT = ((NextOnePoint.x - PriorPoint.x)*(NextOnePoint.y - CurPoint.y)-(NextOnePoint.y - PriorPoint.y)*(NextOnePoint.x - CurPoint.x));
		ValueT = Size*ValueT / Value0;
		MaxCoefficientValue.push_back(ValueT);

		pcl::PointXYZRGB PointA = PointBase::PointsCrossProduct(
			PointBase::PointsMinus(NextTwoPoint,CurPoint), PointBase::PointsMinus(NextThreePoint,NextOnePoint)); 

		pcl::PointXYZRGB PointB = PointBase::PointsCrossProduct(
			PointBase::PointsMinus(NextTwoPoint,CurPoint), PointBase::PointsMinus(NextOnePoint,PriorPoint)); 

		ValueT = PointA.z / PointB.z;
		CoefficientRate0fOneAndFour.push_back(ValueT);
	}
}

void CBezierSubsectionG2::SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, double SizeValue )
{
	QPoints->points.clear();
	QPoints->points.insert(QPoints->points.begin(),
		QPointsValue->points.begin(),QPointsValue->points.end());
	Size = SizeValue;
	//形成闭合体
	//QPoints->points.push_back(QPoints->points[0]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CBezierSubsectionG2::DrawBroderPoints()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ResultPoints (new pcl::PointCloud<pcl::PointXYZRGB>); 
	int n = QPoints->points.size();
	CalcBase<double> CalcBasefloat;
	for(int i = 0; i < QPoints->points.size(); i++)
	{			
		pcl::PointXYZRGB PriorPoint =  QPoints->points[(i-1+n)%n];
		pcl::PointXYZRGB CurPoint =  QPoints->points[i];
		pcl::PointXYZRGB NextOnePoint =  QPoints->points[(i+1+n)%n];
		pcl::PointXYZRGB NextTwoPoint =  QPoints->points[(i+2+n)%n];
/*		
		pcl::PointXYZRGB PointA = PointBase::PointsAdd(CurPoint, 
			PointBase::PointsMutile(PointBase::PointsMinus(NextOnePoint, PriorPoint),MaxCoefficientValue[2*i]));
		ResultPoints->points.push_back(PointA);

		if (ResultPoints->points.size() > 1)
			Viewer->addLine(ResultPoints->points[ResultPoints->points.size() - 1],
				ResultPoints->points[ResultPoints->points.size() - 2], CalcBasefloat.ConvertToString(clock()));
		//*/

		pcl::PointXYZRGB PointB = PointBase::PointsAdd(NextOnePoint, 
			PointBase::PointsMutile(PointBase::PointsMinus(NextTwoPoint, CurPoint),MaxCoefficientValue[2*i+1]));
		ResultPoints->points.push_back(PointB);		

		if (ResultPoints->points.size() > 1)
			Viewer->addLine(ResultPoints->points[ResultPoints->points.size() - 1],
				ResultPoints->points[ResultPoints->points.size() - 2], CalcBasefloat.ConvertToString(clock()));
	}

	Viewer->addLine(ResultPoints->points[ResultPoints->points.size() - 1],
			ResultPoints->points[0], CalcBasefloat.ConvertToString(clock()));

	return ResultPoints;
}

double CBezierSubsectionG2::GetBezierLength()
{
	double Length = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr LocalityCurvePoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	int n = QPoints->points.size();

	AllPointUValues.clear();

	for (int i = 0; i < QPoints->points.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierCurveControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

		BezierCurveControlPoints->points.push_back(ControlPoints->points[3 * i]);
		BezierCurveControlPoints->points.push_back(ControlPoints->points[(3 * i + 1) % (3 * n)]);
		BezierCurveControlPoints->points.push_back(ControlPoints->points[(3 * i + 2) % (3 * n)]);
		BezierCurveControlPoints->points.push_back(ControlPoints->points[(3 * i + 3) % (3 * n)]);

		CBezierCurve BezierCurve;
		BezierCurve.SetInputs(BezierCurveControlPoints);

		Length = Length + BezierCurve.GetBezierLengthBySimpson();
	}
	return Length;
}

double CBezierSubsectionG2::GetBezierArea()
{
	double Area = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr LocalityCurvePoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	int n = QPoints->points.size();

	AllPointUValues.clear();

	for (int i = 0; i < QPoints->points.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierCurveControlPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

		BezierCurveControlPoints->points.push_back(ControlPoints->points[3 * i]);
		BezierCurveControlPoints->points.push_back(ControlPoints->points[(3 * i + 1) % (3 * n)]);
		BezierCurveControlPoints->points.push_back(ControlPoints->points[(3 * i + 2) % (3 * n)]);
		BezierCurveControlPoints->points.push_back(ControlPoints->points[(3 * i + 3) % (3 * n)]);

		CBezierCurve BezierCurve;
		BezierCurve.SetInputs(BezierCurveControlPoints);

		if (i == 22)
			i = i;
		Area = Area + BezierCurve.GetBezierArea();
	}
	return Area;
}


double CBezierSubsectionG2::DrawBezierCurve(pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierCurvePoints, int PointNum)
{
	double Length = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr LocalityCurvePoints (new pcl::PointCloud<pcl::PointXYZRGB>);
	int n = QPoints->points.size();

	AllPointUValues.clear();

	for(int i = 0; i < QPoints->points.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierCurveControlPoints (new pcl::PointCloud<pcl::PointXYZRGB>);	
		
		BezierCurveControlPoints->points.push_back(ControlPoints->points[3*i]);
		BezierCurveControlPoints->points.push_back(ControlPoints->points[(3*i+1)%(3*n)]);
		BezierCurveControlPoints->points.push_back(ControlPoints->points[(3*i+2)%(3*n)]);
		BezierCurveControlPoints->points.push_back(ControlPoints->points[(3*i+3)%(3*n)]);

		CBezierCurve BezierCurve;
		BezierCurve.SetInputs(BezierCurveControlPoints);
		Length = Length + BezierCurve.GetBezierCurve(LocalityCurvePoints);
		
		//Length = Length + BezierCurve.GetBezierLengthBySimpson();
		//Area = Area + BezierCurve.GetBezierArea();

		BezierCurvePoints->points.insert(BezierCurvePoints->points.end(),
			LocalityCurvePoints->points.begin(), LocalityCurvePoints->points.end());

		vector<double> CurrentSectionU = BezierCurve.GetPointsUValues();
		
		AllPointUValues.insert(AllPointUValues.end(), CurrentSectionU.begin(), CurrentSectionU.end());
		//保存每段的索引值
		SectionStartPointIndex.push_back(AllPointUValues.size());

		/*
		vector<string> DerivativeStrOne = BezierCurve.GetDerivativeStr(1);
		vector<string> DerivativeStrTwo = BezierCurve.GetDerivativeStr(2);

		CFormulaAnalytic FormulaAnalytic;

		////显示 u=0.5 处的导数
		pcl::PointXYZRGB BasePoint = BezierCurve.GetBezierPoint(0.5);
		pcl::PointXYZRGB OneDerivativePoint = BezierCurve.GetBezierDerivative(0.5, 1);
		cout<<"一阶导数"<<OneDerivativePoint<<endl;
		cout<<"一阶导数 X:"<<FormulaAnalytic.ResolveValue(DerivativeStrOne[0], 0.5);
		cout<<" Y:"<<FormulaAnalytic.ResolveValue(DerivativeStrOne[1], 0.5)<<endl;
		//cout<<"Z:"<<FormulaAnalytic.ResolveValue(DerivativeStrOne[2], 0.5)<<endl;

		pcl::PointXYZRGB TwoDerivativePoint = BezierCurve.GetBezierDerivative(0.5, 2);
		cout<<"二阶导数"<<TwoDerivativePoint<<endl;
		cout<<"二阶导数 X:"<<FormulaAnalytic.ResolveValue(DerivativeStrTwo[0], 0.5);
		cout<<" Y:"<<FormulaAnalytic.ResolveValue(DerivativeStrTwo[1], 0.5)<<endl;
		//cout<<"Z:"<<FormulaAnalytic.ResolveValue(DerivativeStrTwo[2], 0.5)<<endl;
		//*/
		//PointBase::DrawExtentedLineByBasePointAndDirection(Viewer, BasePoint, OneDerivativePoint, 2);
		//PointBase::DrawExtentedLineByBasePointAndDirection(Viewer, BasePoint, TwoDerivativePoint, 3);
	}	
	//GlobalBezierCurvePoints = BezierCurvePoints;

	if (PointNum != -1)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		double Space = 1.0 * BezierCurvePoints->points.size() / PointNum;
		//for (int i = 0; i < 360; i++) 
		for (int i = 0; i < PointNum; i++)
		{
			TempPoints->points.push_back(BezierCurvePoints->points[floor(i * Space)]);
		}
		BezierCurvePoints->points.clear();
		PointBase::PointCopy(TempPoints, BezierCurvePoints);
	}

	return Length;	
}

//计算第PointIndex个点的第Order阶导数
pcl::PointXYZRGB CBezierSubsectionG2::CalcPointDerivative(int PointIndex, int Order, int Position)
{
	if (Order == 1)
	{
		if (Position == 0)
		{
			int ControlPointIndex = 3 * PointIndex + 1;
			return PointBase::PointsMutile(PointBase::PointsMinus(
				ControlPoints->points[ControlPointIndex], QPoints->points[PointIndex]), 3);
		}
		else if (Position == 1)
		{
			int ControlPointIndex = 3 * PointIndex + 2;
			return PointBase::PointsMutile(PointBase::PointsMinus(
				QPoints->points[(PointIndex + 1) % QPoints->points.size()], ControlPoints->points[ControlPointIndex]), 3);
		}
	}
	else if (Order == 2)
	{
		if (Position == 0)
		{
			int ControlPointIndex = 3 * PointIndex + 1;
			return PointBase::PointsMutile(PointBase::PointsAdd(PointBase::PointsMinus(
				ControlPoints->points[ControlPointIndex + 1],
				PointBase::PointsMutile(ControlPoints->points[ControlPointIndex], 2)),
				QPoints->points[PointIndex]), 6);
		}
		else if (Position == 1)
		{
			int ControlPointIndex = 3 * PointIndex + 1;
			return PointBase::PointsMutile(PointBase::PointsAdd(PointBase::PointsMinus(
				QPoints->points[(PointIndex + 1) % QPoints->points.size()],
				PointBase::PointsMutile(ControlPoints->points[ControlPointIndex + 1], 2)),
				ControlPoints->points[ControlPointIndex]), 6);
		}
	}
	//else
	//	return 0;
}

	
//返回数值
double CBezierSubsectionG2::CalcPointDerivativeValue(int PointIndex, int Order, int Position)
{
	pcl::PointXYZRGB Point = CalcPointDerivative(PointIndex,Order,Position);
	if (Order == 1)
		if (Point.x!=0)
			return Point.y/Point.x;
		else return -0;
	else if(Order == 2)
	{
		pcl::PointXYZRGB PointOne = CalcPointDerivative(PointIndex,1,Position);	
		if (PointOne.x!=0)
			return (Point.y*PointOne.x - Point.x*PointOne.y)/pow(PointOne.x,3);
		else return -0;
	}
	//else
	//	return 0;
}


void CBezierSubsectionG2::ShowPointDerivative()
{
	cout<<"一阶导****"<<endl;
	for(int i = 0; i < QPoints->points.size();i++)
	{			
		cout<<"第"<<i<<"个节点0位置的一阶导"<<CalcPointDerivativeValue(i,1,0)<<endl;			
		cout<<"第"<<i<<"个节点1位置的一阶导"<<CalcPointDerivativeValue(i,1,1)<<endl;	

	}
	cout<<"二阶导****"<<endl;
	for(int i = 0; i < QPoints->points.size();i++)
	{		
		cout<<"第"<<i<<"个节点0位置的二阶导"<<CalcPointDerivativeValue(i,2,0)<<endl;	
		cout<<"第"<<i<<"个节点1位置的二阶导"<<CalcPointDerivativeValue(i,2,1)<<endl;
	}
}

//计算控制系数 根据方程组 //此方程无解
void CBezierSubsectionG2::ResolveCoefficientByEquations()
{
	CoefficientValue.clear();
	int n = QPoints->points.size();

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2*n, 2*n); //默认是0矩阵
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2*n, 1);	// 默认是0矩阵
	for(int i = 0; i < n; i++)
	{
		A(2*i,2*i)=1;	A(2*i,(2*i+3)%(2*n))= -CoefficientRate0fOneAndFour[i];
		A(2*i+1,2*i+1)=1;	A(2*i+1,(2*i+2)%(2*n))=1;
	}

	cout<<"矩阵A："<<endl<<A<<endl;
	cout<<"矩阵B："<<endl<<B<<endl;

	Eigen::MatrixXd X = Math::ResloveLinearEquationsByLUDecomposition(A, B);

	cout<<"系数解"<<endl<<X<<endl;
	for(int i = 0; i < 2*n; i++)
	{
		CoefficientValue.push_back(X(i,0));	
	}

}

//避免(Li) * (Li-1)等于0的问题  2015.01.26 后期不再使用
void CBezierSubsectionG2::FilterPoints()
{
	int n = QPoints->points.size();
	for(int i = 0; i < n; i++)
	{
		VectorofQPoints->points.push_back(PointBase::PointsMinus(
			QPoints->points[(i+1) % n], QPoints->points[(i) % n]));
	}	

	vector<int> NeedDelete;
	int Index = 0;
	for(int i = 0; i < n; i++)
	{
		//|L(i), L(i+1)| 删除		

		pcl::PointXYZRGB PointC = PointBase::PointsCrossProduct(
			VectorofQPoints->points[i%n], VectorofQPoints->points[(i+1)%n]);

		if (abs(PointC.z) <= (10e-5))//解决|L(i), L(i+1)|值过小 ，导致EE(i)的值为零的问题2016.01.25
		{
			NeedDelete.push_back((i+1)%n);
/*			QPoints->points.erase(QPoints->points.begin() + i + 1 - Index);
			Index++;	*/	
		}
	}

}

//构建节点的弦向量 及其它参变量	 2015.12.31
//2016.01.26 修改为 动态判断 EE 值是否为0 如果为0 则删除对应的 i+1 的节点
bool CBezierSubsectionG2::GetVectorofQPointsAndOthers()
{
	//FilterPoints();
	VectorofQPoints->points.clear();
	int n = QPoints->points.size();
	for(int i = 0; i < n; i++)
	{
		VectorofQPoints->points.push_back(PointBase::PointsMinus(
			QPoints->points[(i+1) % n], QPoints->points[(i) % n]));
	}
	
	UU.clear();
	AA.clear();
	BB.clear();
	EE.clear();
	double TempU;

	int SmallUIndex = 0;
	double SmallUValue = 10e-8;

	for(int i = 0; i < n; i++)
	{
		//double a = pow(PointBase::PointModel(VectorofQPoints->points[i]),2);
		//double b = PointBase::PointModel(VectorofQPoints->points[(i+1)%n]);
		//double c = PointBase::PointModel(VectorofQPoints->points[(i-1+n)%n]);

		//|L(i-1), L(i+1)|
		pcl::PointXYZRGB PointA = PointBase::PointsCrossProduct(
			VectorofQPoints->points[(i-1+n)%n], VectorofQPoints->points[(i+1)%n]);
		//|L(i-1), L(i)|
		pcl::PointXYZRGB PointB = PointBase::PointsCrossProduct(
			VectorofQPoints->points[(i-1+n)%n], VectorofQPoints->points[i%n]);
		//|L(i), L(i+1)|
		pcl::PointXYZRGB PointC = PointBase::PointsCrossProduct(
			VectorofQPoints->points[i%n], VectorofQPoints->points[(i+1)%n]);

		AA.push_back(PointA.z/PointC.z);
		BB.push_back(PointA.z/PointB.z);

		//获取最小的U值
		//double TempUValue = AA[AA.size()-1] / BB[BB.size()-1];
		//double TempUValue = BB[BB.size()-1] / AA[AA.size()-1];		
		//if (TempUValue < SmallUValue)

		//寻找最大的 |L(i-1), L(i+1)| 
		if (SmallUValue < PointA.z)
		{
			SmallUValue = PointA.z;
			SmallUIndex = i;
		}
		
		UU.push_back(1);
		EE.push_back(1);		
	}
	
	//计算U值
	for(int i = SmallUIndex; i < SmallUIndex + n; i++)
	{	
		if ( i == SmallUIndex)
			//UU[i%n] = U*pow(BB[i%n]/AA[i%n], 2);
			UU[i%n] = U;
		else
			//UU[i%n] = UU[(i-1+n)%n]*pow(BB[(i-1+n)%n]/AA[(i-1+n)%n], 2);
			UU[i%n] = UU[(i-1+n)%n] * pow(BB[(i-1+n)%n]/AA[(i-1+n)%n], 2);			

		double TempValue = AA[i%n]/BB[i%n]+AA[i%n]+1;

		EE[i%n] = (-TempValue + sqrt(pow(TempValue,2)+4*UU[i%n]))/2/UU[i%n];

		if ((UU[i%n] == 0) || (EE[i%n] == 0))	//有时候会出现这样的错误
		{			
			//cout<<"因凸包点太密集，删除导致e值为0 的点，其索引为 "<<(i+1)%n<<" 的凸包点"<<endl;
			QPoints->points.erase(QPoints->points.begin() + (i+1)%n);
			return false;
		}
	}

	FF.clear();
	KK.clear();
	GG.clear();
	RR.clear();
	for(int i = 0; i < n; i++)
	{
		FF.push_back(EE[i] * PointBase::PointModel(PointBase::PointsAdd(
			VectorofQPoints->points[(i-1+n)%n], VectorofQPoints->points[(i)%n])));		

		KK.push_back(EE[(i-1+n)%n] * AA[(i-1+n)%n]/BB[(i-1+n)%n]*
			PointBase::PointModel(PointBase::PointsAdd(
			VectorofQPoints->points[(i-1+n)%n], VectorofQPoints->points[(i)%n])));

		GG.push_back(PointBase::PointModel(VectorofQPoints->points[(i)%n])*
			(1 - EE[i]*(AA[i]/BB[i]+AA[i]+1)));

	}
	
	//如下是获得控制点 2016.01.26
	ControlPoints->points.clear();
	for(int i = 0; i < QPoints->points.size(); i++)
	{
		pcl::PointXYZRGB PointA, PointB;
		pcl::PointXYZRGB PointU1, PointU2;

		PointU1 = PointBase::PointsMinus(QPoints->points[(i+1)%n], QPoints->points[(i-1+n)%n]);
		PointBase::PointNormalized(PointU1);
		PointU2 = PointBase::PointsMinus(QPoints->points[(i+2)%n], QPoints->points[(i)%n]);
		PointBase::PointNormalized(PointU2);

		PointA = PointBase::PointsAdd(QPoints->points[i], PointBase::PointsMutile(PointU1, FF[i]));
		PointB = PointBase::PointsMinus(QPoints->points[(i+1)%n], 
				PointBase::PointsMutile(PointU2, KK[(i+1)%n]));
	
		ControlPoints->points.push_back(QPoints->points[i]);
		ControlPoints->points.push_back(PointA);
		ControlPoints->points.push_back(PointB);

		//if(PointA.x == PointB.x && PointA.y == PointB.y && PointA.z == PointB.z) //中间两点相同
		if(PointA.x == PointB.x || PointA.y == PointB.y) //中间两点的XY坐标相同，只要有一个相同就不行
		{
			//cout<<"因凸包点太密集，导致Bezier插值中间两个点相同，删除索引为 "<<(i+1)%n<<" 的凸包点"<<endl;
			QPoints->points.erase(QPoints->points.begin() + (i+1)%n); 
			return false;
		}

		if(QPoints->points[i].x == PointB.x && QPoints->points[i].y == PointB.y) //中间两点的XY坐标相同，
		{
			//cout<<"因凸包点太密集，导致Bezier插值的第一个点与第三个点相同，删除索引为 "<<(i+1)%n<<" 的凸包点"<<endl;
			QPoints->points.erase(QPoints->points.begin() + (i+1)%n); 
			return false;
		}

		if(QPoints->points[(i+1)%n].x == PointA.x && QPoints->points[(i+1)%n].y == PointA.y) //中间两点的XY坐标相同，
		{
			//cout<<"因凸包点太密集，导致Bezier插值的第二个点与第四个点相同，删除索引为 "<<(i+1)%n<<" 的凸包点"<<endl;
			QPoints->points.erase(QPoints->points.begin() + (i+1)%n); 
			return false;
		}
				
		if(PointDis(PointA, PointB) < 1.0e-7)
			//中间两点距离太近
		{
			//cout<<"因凸包点太密集，导致Bezier插值中间点距离太近(小于1.0e-7)，删除索引为 "<<(i+1)%n<<" 的凸包点"<<endl;
			QPoints->points.erase(QPoints->points.begin() + (i+1)%n); 
			return false;
		}

	}	

	return true;
}

	//计算控制点
void CBezierSubsectionG2::ResolveControlPoints(double UValue)
{
	U = UValue;

	bool IsTrue = GetVectorofQPointsAndOthers();
	while (!IsTrue)
		IsTrue = GetVectorofQPointsAndOthers();	
}

//void CBezierSubsectionG2::ComputeConvexPropertyOfCurve()
//{
//	CAnglePartition AnglePartitionInstance;
//
//	vector<AnglePartitionStruct> SectionAngle;
//	AnglePartitionInstance.PartitionPoints(GlobalBezierCurvePoints, 1, SectionAngle);
//
//	CalcBase<int> CalcBaseInt;
//	CalcBase<double> CalcBaseFloat;
//	int NoConvexhull = 0;
//
//	NoConvexhullPoints->points.clear();
//
//	for(int i = 0; i < SectionAngle.size(); i++)
//	{
//		pcl::PointXYZRGB TempPoint;
//		if (SectionAngle[i].PointIndexs.size() > 0)
//		{
//			//cout<<"正在处理第"<<i<<"个角度分区"<<endl;
//			TempPoint = GlobalBezierCurvePoints->points[SectionAngle[i].PointIndexs[0]];
//			pcl::PointXYZRGB TangentLinePoint = 
//				ResolveTangentLine(SectionAngle[i].PointIndexs[0]);
//			
//			bool IsConvexhull = IsConvexHullPoint(i, TangentLinePoint, SectionAngle);
//			if (!IsConvexhull) 
//			{
//				NoConvexhullPoints->points.push_back(TempPoint);
//				NoConvexhull++;		
//
//				//Viewer->addText3D(CalcBaseInt.ConvertToString(i), TempPoint,0.05,
//				//	255,0,0,CalcBaseFloat.ConvertToString(clock()) 
//				//+  CalcBaseInt.ConvertToString(i));
//
//				//PointBase::DrawExtentedLineByBasePointAndDirection(Viewer,
//				//	TempPoint, TangentLinePoint, 6);
//				//cout<<"当前非凸点曲率为"<<GetCurvature(AllPointUValues[SectionAngle[i].Points[0]])<<endl;
//			}			
//		}
//	}
//	cout<<"共有 "<<NoConvexhull<<" 个非凸点, 非凸点比率是"<<100.0 * NoConvexhull/360.0<<"%"<<endl;
//}

//获取特定点的曲率
double CBezierSubsectionG2::GetCurvature(int PointIndex)
{
	int SectionIndex = 0;
	for(int i = 0; i < SectionStartPointIndex.size(); i++)
	{
		if (PointIndex < SectionStartPointIndex[i])
		{
			SectionIndex = i;	
			break;
		}
	}

	int n = QPoints->points.size();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierCurveControlPoints (new pcl::PointCloud<pcl::PointXYZRGB>);	
		
	BezierCurveControlPoints->points.push_back(ControlPoints->points[3*SectionIndex]);
	BezierCurveControlPoints->points.push_back(ControlPoints->points[(3*SectionIndex+1)%(3*n)]);
	BezierCurveControlPoints->points.push_back(ControlPoints->points[(3*SectionIndex+2)%(3*n)]);
	BezierCurveControlPoints->points.push_back(ControlPoints->points[(3*SectionIndex+3)%(3*n)]);

	CBezierCurve BezierCurve;
	BezierCurve.SetInputs(BezierCurveControlPoints);
	
	return BezierCurve.GetCurvature(AllPointUValues[PointIndex]);	
}

////判断此点是否是凸曲线上的点
//bool CBezierSubsectionG2::IsConvexHullPoint(int AngleIndex, pcl::PointXYZRGB TangentLinePoint,
//	vector<AnglePartitionStruct> & SectionAngle)
//{
//	pcl::PointXYZRGB BasePoint = GlobalBezierCurvePoints->points[SectionAngle[AngleIndex].PointIndexs[0]];
//	
//	//方程 y=a*x+b
//	double a = TangentLinePoint.y / TangentLinePoint.x;
//	double b = BasePoint.y - a * BasePoint.x;
//
//	int Flag = 0;
//	bool Return = true;
//
//	//选定的另外一个参考点是其对面的一个点
//	int j = 180;		
//
//	while (SectionAngle[(j + AngleIndex) % 360].PointIndexs.size() == 0)
//	{
//		j++;
//	}
//	
//	int SelectIndex = (j + AngleIndex) % 360;
//
//	pcl::PointXYZRGB SelectedPoint = GlobalBezierCurvePoints->points[SectionAngle[SelectIndex].PointIndexs[0]];
//	double Symbol = SelectedPoint.y - a*SelectedPoint.x - b;
//
//	for(int i = 0; i < SectionAngle.size(); i++)
//	{
//		if (i != AngleIndex && i != SelectIndex)
//		{
//			if (SectionAngle[i].PointIndexs.size() > 0)
//			{
//				pcl::PointXYZRGB TempPoint = GlobalBezierCurvePoints->points[SectionAngle[i].PointIndexs[0]];
//				double TempSymbol = TempPoint.y - a*TempPoint.x - b;
//				//if (Symbol * TempSymbol < 0 && abs(TempSymbol) > 0.001) 
//				if (Symbol * TempSymbol < 0) 
//				{
//					Return = false;
//					break;
//				}
//			}
//		}
//	}
//	return Return;
//}

//获取当前点的切线的方向向量 其实就是此点的一阶导数
pcl::PointXYZRGB CBezierSubsectionG2::ResolveTangentLine(int PointIndex)
{
	int SectionIndex = 0;
	for(int i = 0; i < SectionStartPointIndex.size(); i++)
	{
		if (PointIndex < SectionStartPointIndex[i])
		{
			SectionIndex = i;	
			break;
		}
	}

	int n = QPoints->points.size();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr BezierCurveControlPoints (new pcl::PointCloud<pcl::PointXYZRGB>);	
		
	BezierCurveControlPoints->points.push_back(ControlPoints->points[3*SectionIndex]);
	BezierCurveControlPoints->points.push_back(ControlPoints->points[(3*SectionIndex+1)%(3*n)]);
	BezierCurveControlPoints->points.push_back(ControlPoints->points[(3*SectionIndex+2)%(3*n)]);
	BezierCurveControlPoints->points.push_back(ControlPoints->points[(3*SectionIndex+3)%(3*n)]);

	CBezierCurve BezierCurve;
	BezierCurve.SetInputs(BezierCurveControlPoints);
	//cout<<"当前点"<<CurPoint<<endl;
	//cout<<"计算得到的当前点"<<BezierCurve.GetBezierPoint(AllPointUValues[PointIndex])<<endl;
	return BezierCurve.GetBezierDerivativePoint(AllPointUValues[PointIndex]);
}