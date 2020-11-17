//������ֵ  ���������ϵĵ� �������Ŀ��Ƶ���ڵ����� 2015.09.26

#include "SplineInterpolation.h"


//���ó�ʼ����  IsClosed �պ�  IsSuccessiveValue ����
void CSplineInterpolation::SetInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, 
	int dValue, bool IsClosedValue, bool IsSuccessiveValue)
{
	dSpline = dValue;
	IsClosed = IsClosedValue;

	QPoints->points.clear();	
	QPoints->points.insert(QPoints->points.end(), 
		QPointsValue->points.begin(), QPointsValue->points.end());
	
	//��ִ���ҳ����Ȼ� ��ʵ������
	//if (ConvexityPreserving)
	//{
	//	HomogenizationSplineInterPolatingPoints();
	//}

	//��Ҫ����������������ֵ
	if (IsClosed)
	{		
		if (IsSuccessiveValue)
		{
			for(int i = 0; i < dSpline; i++)	
			{
				//β�������ײ��ڵ�
				if (QPointsValue->points.size() > i)	
					QPoints->points.push_back(QPointsValue->points[i]);

				 //�ײ�����β���ڵ�
				if (QPointsValue->points.size() > QPointsValue->points.size() - 1 - i)
					QPoints->points.insert(QPoints->points.begin(), 
						QPointsValue->points[QPointsValue->points.size() - 1 - i]);
			}
		}
		else
		{	// //�ײ�����β���ڵ�
			if (QPointsValue->points.size() > QPointsValue->points.size() - 1)
				QPoints->points.insert(QPoints->points.end(), 
					QPointsValue->points[0]);
		}
	}

	nSpline = QPoints->points.size();	
}

////��ȡ��͹�������ʼ��Χ 2015.12.22 ֻ�����������
//bool CSplineInterpolation::GetNoConvexHullZone(int i, vector<double> KnotVector,		
//		double & NoConvexHullUStart, double & NoConvexHullUEnd)
//{
//	if(dSpline != 3){cout<<"�˷���ֻ�������������"<<endl;	return true;}
//	
//	if (i >= nSpline || i < dSpline) 
//	{	cout<<"�����˽ڵ�������ȡֵ��Χ"<<endl; return true;}
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
//		cout<<"����ʽ��ϼ������X���ʽ�Ľ���"<<XCoefficient.size()
//			<<"��������ָ���Ľ���"<<dSpline<<endl;
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
//		cout<<"����ʽ��ϼ������Y���ʽ�Ľ���"<<XCoefficient.size()
//			<<"��������ָ���Ľ���"<<dSpline<<endl;
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
//		//cout<<"�������ް�������"<<endl;
//	else if(d > 0 && a > 0)
//	{
//		double RootLeft, RootRight;
//		RootLeft = (-b-sqrt(d))/2/a;
//		RootRight = (-b+sqrt(d))/2/a;
//		if (UEnd < RootLeft || UStart > RootRight)
//		{	
//			cout<<"�������ް�������"<<endl;
//			ReturnValue = true;
//		}
//		else if( UStart > RootLeft &&  UEnd < RootRight)
//		{
//			ReturnValue = false;
//			NoConvexHullUStart = UStart;
//			NoConvexHullUEnd = UEnd;
//			cout<<"������ȫ���ǰ�������"<<endl;
//		}
//		else
//		{
//			if( UStart > RootLeft &&  UStart < RootRight)			
//			{
//				cout<<"��"<<i<<"�����䰼������Ϊ "<< UStart<<" �� "<<RootRight<<endl;
//				ReturnValue = false;
//				NoConvexHullUStart = UStart;
//				NoConvexHullUEnd = RootRight;
//			}
//			else if( UEnd > RootLeft &&  UEnd < RootRight)
//			{
//				cout<<"��"<<i<<"�����䰼������Ϊ "<< RootLeft<<" �� "<<UEnd<<endl;
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
//			cout<<"��"<<i<<"�������ް�������"<<endl;
//			ReturnValue = true;
//		}
//		else if (UStart < RootLeft && UEnd > RootLeft)
//		{
//			cout<<"��"<<i<<"�����䰼������Ϊ "<<UStart<<" �� "<<RootLeft<<endl;
//			ReturnValue = false;
//			NoConvexHullUStart = UStart;
//			NoConvexHullUEnd = RootLeft;
//		}
//		else if (UStart < RootLeft && UEnd > RootRight)	//����������
//		{
//			cout<<"��"<<i<<"�����䰼������Ϊ "<<UStart<<" �� "<<RootLeft <<" �� ����"<<RootRight<<" �� "<<UEnd<<endl;
//			ReturnValue = false;
//			NoConvexHullUStart = UStart;
//			NoConvexHullUEnd = RootLeft;
//		}
//		else if (UStart < RootRight && UEnd > RootRight)
//		{
//			cout<<"��"<<i<<"�����䰼������Ϊ "<<RootRight<<" �� "<<UEnd<<endl;
//			ReturnValue = false;
//			NoConvexHullUStart = RootRight;
//			NoConvexHullUEnd = UEnd;
//		}	
//		else if (RootRight < UStart ||  UEnd < RootLeft)
//		{
//			ReturnValue = false;
//			NoConvexHullUStart = UStart;
//			NoConvexHullUEnd = UEnd;
//			cout<<"������ȫ���ǰ�������"<<endl;
//		}
//	}
//	return ReturnValue;
//}



//��ȡ͹�Ապϵ������������ߵĿ��Ƶ���ڵ�����//  *****����(��СU��ֵ�������͹�м��) �в�ͨ*****
//2015.12.26 ����Ϊ������С����ֵ�ķ���
//pcl::PointXYZRGB CSplineInterpolation::GetConvexHullAndClosedControlPoints(
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPointsValue, 
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr & ControlPointsValue,
//	vector<double> & OutKnotValue)
//{
//	QPoints->points.clear();	
//	QPoints->points.insert(QPoints->points.end(), 
//		QPointsValue->points.begin(), QPointsValue->points.end());	
//
//	//������β��� ��β����γɱպ�����
//	for(int i = 0; i < dSpline; i++)	
//	{
//		//β�������ײ��ڵ�
//		if (QPoints->points.size() > i)	
//			QPoints->points.push_back(QPoints->points[i]);
//
//			//�ײ�����β���ڵ�
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
//	//��ȡ��ֵ���ƽ������
//	//double AvgDis = GeometryBase::GetAvgDisOfPoints(QPoints);
//	int InsertNode = 0;
//	
//	//��ȡ��͹�����Uֵȡֵ��Χ
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
//		cout<<"���ڽ��е�"<<++IterationNumber<<"�ε���,��ǰ��͹���乲"
//			<<NoConvexHullStartU.size()<<"��"<<endl<<endl;
//		vector<int> InsertPointIndex;
//		//���������ʸ��������֮��ֱ�ӵĽ���
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
//					//2015.12.26 ����н��㣬˵��������Ĳ�����Χ��Ҫ��С
//					////ShrinkKnotParameters(i);
//					////HaveCross = true;
//					////break;
//
//					//����2015.12.26ע��
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
//			cout<<"����"<<IterationNumber<<"�ε������Ѿ��ɹ�������һ������İ���"<<endl;
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
	//		cout<<i<<"��Ӧ��Index"<<Index<<endl;
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
	//��Ӻ��ٻ�ȡһ��
	//QPoints = TempQPointsValue;
	//GetControlPointsAndKnotValue(ControlPointsValue, OutKnotValue);
	
//	pcl::PointXYZRGB ResultPoint = QPoints->points[0];
//		
//	//ControlPointsValue = TempQPointsValue;	
//	return ResultPoint;
//}

//2015.12.24 ������������������ӷ�͹�����U����ʼ����
void CSplineInterpolation::InsertNoConvexHullU(double StartU, double EndU)
{
	if (NoConvexHullStartU.size() == 0)
	{
		NoConvexHullStartU.push_back(StartU);
		NoConvexHullEndU.push_back(EndU);
		return;
	}
	bool Find = false;
	for(int i = NoConvexHullStartU.size() - 1; i >= 0; i--) //����ʼ����
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


//��λ���ڲ�ֵ������䷶Χ
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

//���Ȼ������Ĳ�ֵ��
void CSplineInterpolation::HomogenizationSplineInterPolatingPoints()
{
	double AvgDis = 0;
	CalcBase<double> CalcBaseFloat;
	vector<double> VecDis;

	//2015.12.22 ����Ϊ������С������Ȼ�������Ч��
	double MinDis = 0;

	if (IsClosed)
	{
		QPoints->points.push_back(QPoints->points[0]);
	}

	for(int i = 0; i < QPoints->points.size() - 1; i++)
	{
		double TempDis = PointDis(QPoints->points[i], QPoints->points[i+1]);
		VecDis.push_back(TempDis);
		
		//ȡ��С����
		if (MinDis == 0)
			MinDis = TempDis;
		else
		{
			if (TempDis < MinDis)
				MinDis = TempDis;
		}
		//ȡ��С����
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

//����ڵ����ҳ�ֵ
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


//����ڵ�Ľڵ����ֵ �����ҳ���
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


//����ڵ�Ľڵ����ֵ ���Ĳ����� ��The Nurbs books��P257 2015.12.24
void CSplineInterpolation::GetUParametesByCentripetalParameter()
{
	ChordLength = GetPointsChordLength();
	UParameters.clear();	
	
	//vector<double> Angle;
	//CalcChordAngle(Angle);	//����Ƕ�

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

//����ڵ�Ľڵ����ֵ ���������� ��������������������Ǿ���B������P45 2015.12.24
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

	//�������߼нǵ����	��������������������Ǿ���B������P48 2015.12.24	
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


//����ڵ����ֵ
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

//����ڵ����� ����ȡƽ��ֵ��
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

//�������Է����鲢��� ����������ص� ControlPoints �㼯��
bool CSplineInterpolation::ResolveEquations()
{
	//����ڵ�����
	GetKnotValuesByAvg();

	//�������������Ĳ�����Ϣ����Ҫ���ڼ��� �ڽڵ�������ȡ������λ�� 	
	RationalSpline.SetSplineInputs(QPoints, dSpline, KnotValues);
	// AX=B
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nSpline, nSpline); //Ĭ����0����
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nSpline, 3);	// Ĭ����0����
	
	//��A��ֵ  ��Ҫ����
	for(int i = 0; i < nSpline; i++)	//��
	{	//��ȡ�������ڵ�λ��
		int KnotVectorIndex = RationalSpline.FindUIndex(UParameters[i]);
		for(int j = 0; j < nSpline; j++)	//��
		{				
			int K = KnotVectorIndex - dSpline + j;
			if (K >= nSpline) continue;
			
			A(i,K) = RationalSpline.CoxdeBoor(K, dSpline, UParameters[i]);
			//cout<<"i: "<<i<<" k:"<<K<<" d:"<<dSpline<<" u:"<<UParameters[i]<<" CoxdeBoor:"<<A(i,K)<<endl;
		}
	}
	A(nSpline - 1, nSpline - 1) = 1;
	//cout<<"����A��"<<endl;
	//cout<<A<<endl;
	//cout<<A(0,0)<<endl;
	//��B��ֵ
	for(int i = 0; i < nSpline; i++)
	{
		B(i,0) = QPoints->points[i].x;
		B(i,1) = QPoints->points[i].y;
		B(i,2) = QPoints->points[i].z;
	}
	//cout<<"����B��"<<endl;
	//cout<<B<<endl;
	//��ⷽ����
	
	Eigen::MatrixXd X = Math::ResloveLinearEquationsByLUDecomposition(A, B);
		
	//cout<<"����X��"<<endl;
	//cout<<X<<endl;

	//�������תΪ���Ƶ�
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

//��ȡ���Ƶ���ڵ�����
bool CSplineInterpolation::ResolveControlPoints()
{
	//����ڵ����ֵ	
	CalcKnotParameters();

	//��ⷽ���� ��ȡ���Ƶ�����
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


//��С�ڵ��Ĳ���ֵ
void CSplineInterpolation::ShrinkKnotParameters(int Index, double Multiple)
{
	if (Index < 0 || Index >= UParameters.size() - 2) return;

	double MinusU = (UParameters[Index+1] - UParameters[Index]) * Multiple;
	
	//���ַ�������ȷ����仯
	for(int i = Index+1; i < UParameters.size(); i++)
	{
		UParameters[i] = UParameters[i] - MinusU;
	}
	//ֻ�����ڵ�������ȷ����仯��������Ȳ���
	//UParameters[Index+1] = UParameters[Index+1] - MinusU;
	//UParameters[Index+2] = UParameters[Index+2] + MinusU;
}