#include "BezierCurve.h"

void CBezierCurve::SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue)
{
	QPoints = QPointsValue;
	kCurve = QPoints->points.size() - 1;
}

//计算K次 Bernstein多项式 在U处的值 
double CBezierCurve::GetBernsteinValue(int K, int i, double U)
{
	if ( i == -1 || i > K)
		return 0;
	else
		return BezierPolynomial(K, i) * pow(U, i) * pow(1 - U, K - i);	
}

double CBezierCurve::GetBernsteinDerivativeValue(int K, int i, double U, int Order)
{
	if (Order == 1)
	{
		return K*(GetBernsteinValue(K-1, i-1, U) - GetBernsteinValue(K-1,i,U));
	}
	else
		return 0;
}

// 计算 U 位置处的点
pcl::PointXYZRGB CBezierCurve::GetBezierPoint(double u)
{
	pcl::PointXYZRGB TempPoint; 
	TempPoint.x = 0;
	TempPoint.y = 0;
	TempPoint.z = 0;
	for(int i = 0; i <= kCurve; i++)
	{
		double TempValue = GetBernsteinValue(kCurve, i, u);
		TempPoint.x = TempPoint.x + TempValue * QPoints->points[i].x;
		TempPoint.y = TempPoint.y + TempValue * QPoints->points[i].y;
		TempPoint.z = TempPoint.z + TempValue * QPoints->points[i].z;
	}	
	return TempPoint;
}


//获取bezier曲线 并得到曲线长度
double CBezierCurve::GetBezierCurve(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CurvePointsValue,
	double UStepValue)
{
	//UStep = UStepValue;
	//步长调整为每 UStepValue 一个点
	UStep = 1.0/(PointDis(QPoints->points[0], QPoints->points[3])/UStepValue);
	CurvePointsValue->points.clear();
	double Length = 0;
	double U = 0;
	CalcBase<double> CalcBaseFloat;
	PointUValues.clear();

	while ( U <= 1)
	{
		PointUValues.push_back(U);
		CurvePointsValue->points.push_back(GetBezierPoint(U));

		U = U + UStep;
	}
	return GetBezierLengthBySimpson();
}

double CBezierCurve::BezierPolynomial(int n, int i)
{
	return Math::CombinatorialNumber(n, i);	
}

vector<double> CBezierCurve::GetPointsUValues()
{
	return PointUValues;
}

//计算 U 位置处的一阶导数
pcl::PointXYZRGB CBezierCurve::GetBezierDerivativePoint(double u, int Order)
{	
	pcl::PointXYZRGB TempPoint; 
	if (Order == 1)
	{
		for(int i = 0; i <= kCurve; i++)
		{
			double TempValue = GetBernsteinDerivativeValue(kCurve, i, u);
			TempPoint.x = TempPoint.x + TempValue * QPoints->points[i].x;
			TempPoint.y = TempPoint.y + TempValue * QPoints->points[i].y;
			TempPoint.z = TempPoint.z + TempValue * QPoints->points[i].z;
		}	
	}
	else if (Order == 2)
	{
		pcl::PointXYZRGB PointA; //根据求导公式
		PointA = PointBase::PointsMinus(QPoints->points[2], 
			PointBase::PointsMutile(QPoints->points[1], 2));
		PointA = PointBase::PointsAdd(PointA, QPoints->points[0]);
		PointA = PointBase::PointsMutile(PointA, 6*(-u+1));

		pcl::PointXYZRGB PointB; 
		PointB = PointBase::PointsMinus(QPoints->points[3],
			PointBase::PointsMutile(QPoints->points[2], 2));
		PointB = PointBase::PointsAdd(PointB, QPoints->points[1]);
		PointB = PointBase::PointsMutile(PointB, 6*u);

		TempPoint = PointBase::PointsAdd(PointA, PointB);
	}

	return TempPoint;
}

//计算Bezier曲线长度，根据Simpson公式
double CBezierCurve::GetBezierLengthBySimpson()
{
	double TempUStart = 0, TempUEnd = 1;

	double Dis = PointDis(QPoints->points[0],
		QPoints->points[QPoints->points.size() - 1]);

	int KValue = 1;

	if (Dis >= EPSM3)
		KValue = Dis / EPSM3;
	else if(Dis >= EPSM6 && Dis < EPSM3)
		KValue = Dis / EPSM6;
	else return 0;

	pcl::PointXYZRGB TempPoint;
	TempPoint = GetBezierDerivativePoint(TempUStart);			
	double aValue = sqrt(pow(TempPoint.x,2)+pow(TempPoint.y,2)+pow(TempPoint.z,2));
			
	TempPoint = GetBezierDerivativePoint(TempUEnd);			
	double bValue = sqrt(pow(TempPoint.x,2)+pow(TempPoint.y,2)+pow(TempPoint.z,2));

	double XMiddleValue = 0, XkMiddleValue = 0;
			
	double hStep = (TempUEnd-TempUStart)/KValue;

	double U = TempUStart;	
	double UMiddle = U + hStep/2;

	TempPoint = GetBezierDerivativePoint(UMiddle);	
	XMiddleValue = sqrt(pow(TempPoint.x,2)+pow(TempPoint.y,2)+pow(TempPoint.z,2));

	U = U + hStep;//从第1个点开始执行循环，起始点是第0个点
	for(int j = 1; j < KValue; j++)
	{
		UMiddle = U + hStep/2;
		TempPoint = GetBezierDerivativePoint(UMiddle);
		XMiddleValue = XMiddleValue + sqrt(pow(TempPoint.x,2)+pow(TempPoint.y,2)+pow(TempPoint.z,2));
		//XMiddleValue = XMiddleValue + sqrt(1 + pow(TempPoint.y/TempPoint.x,2));
				
		TempPoint = GetBezierDerivativePoint(U);
		XkMiddleValue = XkMiddleValue + sqrt(pow(TempPoint.x,2)+pow(TempPoint.y,2)+pow(TempPoint.z,2));
		//XkMiddleValue = XkMiddleValue + sqrt(1 + pow(TempPoint.y/TempPoint.x,2));
		U = U + hStep;
	}
	return hStep*(aValue+ 4*XMiddleValue+ 2*XkMiddleValue + bValue)/6;
}

//2016.04.17 计算Bezier曲线所围区域面积
double CBezierCurve::GetBezierArea()
{
	vector<string> FormulaS = GetUFormula();
		
	pcl::PointXYZRGB TempPoint = this->GetBezierPoint(0.5);

	CFormulaAnalytic FormulaAnalytic;

	FormulaAnalytic.SetInputs(FormulaS[0]);
	string FormulaStrX = FormulaAnalytic.GetFormula();	

	FormulaAnalytic.SetInputs(FormulaS[1]);
	string FormulaStrY = FormulaAnalytic.GetFormula();		
		
	string DFormulaStrX = FormulaAnalytic.DerivativeStr(FormulaStrX);
	//string DFormulaStrY = FormulaAnalytic.DerivativeStr(FormulaStrY);

	//cout<<"参数对应点: "<<TempPoint<<endl;
	//cout<<"表达式计算的X坐标: "<<FormulaAnalytic.ResolveValue(FormulaStrX, 0.5)<<endl;
	//cout<<"表达式计算的Y坐标: "<<FormulaAnalytic.ResolveValue(FormulaStrY, 0.5)<<endl;
	//	
	//cout<<"切向量:"<<endl;
	//pcl::PointXYZRGB CurrentDeriPoint = this->GetBezierDerivativePoint(0.5, 1);
	//cout<<"参数对应点的切向量: "<<CurrentDeriPoint<<endl;
	//cout<<"表达式计算的切向量X坐标: "<<FormulaAnalytic.ResolveValue(DFormulaStrX, 0.5)<<endl;
	//cout<<"表达式计算的切向量Y坐标: "<<FormulaAnalytic.ResolveValue(DFormulaStrY, 0.5)<<endl;

	string Formula;
	//此段是根据格式公式P = -y, Q = x
	//Formula = "0.5*(((" + FormulaStrX + ")*(" + DFormulaStrY + "))-((" 
	//		+ FormulaStrY + ")*(" + DFormulaStrX + ")))";

	//////此段是根据格式公式P = 0, Q = x
	//Formula = "((" + FormulaStrX + ")*(" + DFormulaStrY + "))";

	////此段是根据格式公式P = y, Q = 0
	Formula = "-((" + FormulaStrY + ")*(" + DFormulaStrX + "))";

	FormulaAnalytic.SetInputs(Formula);		
	Formula = FormulaAnalytic.GetFormula();

	Formula = FormulaAnalytic.PolynomialMerge(Formula);
	return FormulaAnalytic.ResolveIntegralValue(Formula, 0, 1);
}

//获取获取曲线上的点时对应的计算字符串
vector<string> CBezierCurve::GetUFormula()
{
	vector<string> ResultStr;

	string XFormula = "", YFormula = "", ZFormula = "";

	XFormula = StringBase::FloatToStr(QPoints->points[0].x) + "*(-u+1)*(-u+1)*(-u+1)";
	
	if (QPoints->points[1].x > 0)
		XFormula = XFormula + "+" + StringBase::FloatToStr(3*QPoints->points[1].x) +  "*(-u+1)*(-u+1)*(u)";
	else
		XFormula = XFormula + "-" + StringBase::FloatToStr(abs(3*QPoints->points[1].x)) +  "*(-u+1)*(-u+1)*(u)";
	
	if (QPoints->points[2].x > 0)
		XFormula = XFormula + "+" + StringBase::FloatToStr(3*QPoints->points[2].x) +  "*(-u+1)*(u)*(u)";
	else
		XFormula = XFormula + "-" + StringBase::FloatToStr(abs(3*QPoints->points[2].x)) +  "*(-u+1)*(u)*(u)";
	
	if (QPoints->points[3].x > 0)
		XFormula = XFormula + "+" + StringBase::FloatToStr(QPoints->points[3].x) +  "*u*u*u";
	else
		XFormula = XFormula + "-" + StringBase::FloatToStr(abs(QPoints->points[3].x)) +  "*u*u*u";

	YFormula = StringBase::FloatToStr(QPoints->points[0].y) + "*(-u+1)*(-u+1)*(-u+1)";
	
	if (QPoints->points[1].y > 0)
		YFormula = YFormula + "+" + StringBase::FloatToStr(3*QPoints->points[1].y) +  "*(-u+1)*(-u+1)*(u)";
	else
		YFormula = YFormula + "-" + StringBase::FloatToStr(abs(3*QPoints->points[1].y)) +  "*(-u+1)*(-u+1)*(u)";
	
	if (QPoints->points[2].y > 0)
		YFormula = YFormula + "+" + StringBase::FloatToStr(3*QPoints->points[2].y) +  "*(-u+1)*(u)*(u)";
	else
		YFormula = YFormula + "-" + StringBase::FloatToStr(abs(3*QPoints->points[2].y)) +  "*(-u+1)*(u)*(u)";
	
	if (QPoints->points[3].y > 0)
		YFormula = YFormula + "+" + StringBase::FloatToStr(QPoints->points[3].y) +  "*u*u*u";
	else
		YFormula = YFormula + "-" + StringBase::FloatToStr(abs(QPoints->points[3].y)) +  "*u*u*u";

	ZFormula = StringBase::FloatToStr(QPoints->points[0].z) + "*(-u+1)*(-u+1)*(-u+1)";
	
	if (QPoints->points[1].z > 0)
		ZFormula = ZFormula + "+" + StringBase::FloatToStr(3*QPoints->points[1].z) +  "*(-u+1)*(-u+1)*(u)";
	else
		ZFormula = ZFormula + "-" + StringBase::FloatToStr(abs(3*QPoints->points[1].z)) +  "*(-u+1)*(-u+1)*(u)";
	
	if (QPoints->points[2].z > 0)
		ZFormula = ZFormula + "+" + StringBase::FloatToStr(3*QPoints->points[2].z) +  "*(-u+1)*(u)*(u)";
	else
		ZFormula = ZFormula + "-" + StringBase::FloatToStr(abs(3*QPoints->points[2].z)) +  "*(-u+1)*(u)*(u)";
	
	if (QPoints->points[3].z > 0)
		ZFormula = ZFormula + "+" + StringBase::FloatToStr(QPoints->points[3].z) +  "*u*u*u";
	else
		ZFormula = ZFormula + "-" + StringBase::FloatToStr(abs(QPoints->points[3].z)) +  "*u*u*u";

	CFormulaAnalytic FormulaAnalytic;

	FormulaAnalytic.SetInputs(XFormula);	
	XFormula = FormulaAnalytic.GetFormula();

	FormulaAnalytic.SetInputs(YFormula);	
	YFormula = FormulaAnalytic.GetFormula();

	FormulaAnalytic.SetInputs(ZFormula);	
	ZFormula = FormulaAnalytic.GetFormula();

	ResultStr.push_back(XFormula);	// X 
	ResultStr.push_back(YFormula);	// Y
	ResultStr.push_back(ZFormula);	// Z

	return ResultStr;
}

//获取导数对应的计算字符串
vector<string> CBezierCurve::GetDerivativeStr(int Order)
{
	if (PointSStr.size() == 0)
		PointSStr = this->GetUFormula();
	CFormulaAnalytic FormulaAnalytic;
	vector<string>  ResultStr;	
	
	ResultStr.push_back(FormulaAnalytic.DerivativeStr(PointSStr[0], Order));
	ResultStr.push_back(FormulaAnalytic.DerivativeStr(PointSStr[1], Order));
	ResultStr.push_back(FormulaAnalytic.DerivativeStr(PointSStr[2], Order));

	return ResultStr;
}

//获取特定点的曲率 Relative = true时，是相对曲率
double CBezierCurve::GetCurvature(double U, bool Relative)
{
	pcl::PointXYZRGB OneDerivative = GetBezierDerivativePoint(U, 1);
	pcl::PointXYZRGB TwoDerivative = GetBezierDerivativePoint(U, 2);	
	
	pcl::PointXYZRGB OneCrossTwo = PointBase::PointsCrossProduct(OneDerivative, TwoDerivative);
	pcl::PointXYZRGB UnitPoint;
	UnitPoint.x = 0; UnitPoint.y = 0; UnitPoint.z = 1;

	double Denominator = pow(sqrt(pow(OneDerivative.x, 2) + pow(OneDerivative.y, 2) + pow(OneDerivative.z, 2)),3);
	
	//if (Denominator < eps) 
	//	return 0;
	//else
	{
		if (Relative)
			return PointBase::PointsDotProduct(OneCrossTwo, UnitPoint) / Denominator;
		else
			return sqrt(pow(OneCrossTwo.x,2) + pow(OneCrossTwo.y, 2) + pow(OneCrossTwo.z,2)) /Denominator;
	}
}