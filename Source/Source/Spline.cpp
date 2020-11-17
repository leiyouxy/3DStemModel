#include "Spline.h"

//������������
void CSpline::SetSplineInputs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ControlPointsPtr,
	int dValue, vector<double> KnotVectorValue, 
	bool IsClosedCurve, bool IsConvexhullControlValue)
{
	ControlPoints->points.clear();
	CurvePoints->points.clear();
	ControlPoints->points.insert(ControlPoints->points.end(),
		ControlPointsPtr->points.begin(),
		ControlPointsPtr->points.end());	
	nSpline = ControlPoints->points.size();	//���Ƶ����	
	//d = dValue + 1;								//dValue�������� 
	dSpline = dValue;								//dValue�������� 
	KnotVector = KnotVectorValue;
	IsClosed = IsClosedCurve;
	KnotValuePoints->points.clear();	
	IsConvexhullControl = IsConvexhullControlValue;	
	
	FirstPointOfCloseCurve.x = FirstPointOfCloseCurve.y = FirstPointOfCloseCurve.z = 0;
}

//�����Ϻ��� ע�� d ������ʹ�õĶ���ʽ�Ĵ����������ǽ�����
double CSpline::CoxdeBoor(int i, int d, double u)
{
	//d�Ǵ���ʱ�ļ��㷽�� *****************************	
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
	//*/		//d�Ǵ���ʱ�ļ��㷽�� *****************************

	////d�ǽ���ʱ�ļ��㷽�� *****************************	
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
	//*/		//d�ǽ���ʱ�ļ��㷽�� *****************************
}

//�����Ϻ��� ע�� d������ʹ�õĶ���ʽ�Ĵ����������ǽ����� ���ع�ʽ
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

//���������Ni,p(u)��k�׵���
double CSpline::DerivativeOfCoxdeBoor(int i, int p, double u, int Order)
{
	if (Order == 1)	//һ�׵��� ���㷽ʽ�����Ǿ�������B��������42ҳ ��ʽ 2.7
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
	else	//�߽׵�	���㷽ʽ�����Ǿ�������B��������44ҳ ��ʽ 2.9
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

//2015.12.24 ������������������ӷ�͹�����U����ʼ����
void CSpline::InsertNoConvexHullU(double StartU, double EndU)
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


//�������������ĵ� //�������������ĵ� ���������ĳ��� �ڵ��������Ϊ0.000001
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
	//�����ڵ�����
	if (KnotVector.size() == 0) 
	{
		CanCreateCurve = false;
		cout<<"�ڵ�����Ϊ�գ��޷�����������"<<endl;
		return;
	}

	if (KnotVector.size() != nSpline + dSpline + 1) 
	{
		cout<<"�ڵ��������������ڿ��Ƶ�ĸ���(n)��ײ����ĸ���(d)�ĺͲ��ȣ����ʵ��"<<endl;
		CanCreateCurve = false;
		return ;
	}

	if (!CanCreateCurve)
	{
		cout<<"���Ƶ������ڵ���������������Ҫ���޷�����������"<<endl;
		return;
	}

	if (IsClosed)
	{
		if (FirstPointOfCloseCurve.z == -100000)
		{
			cout<<"�պ����ߵ��׽ڵ�δ��ֵ���޷�����������"<<endl;
			return ;			
		}
	}

	//if ( DerivativeOrder > dSpline) DerivativeOrder = dSpline;	//�������������� ��������

	//cout<<"���ڹ������������ϵĵ�"<<endl;

	//����һ�׵����Ŀ��Ƶ���ڵ����� ���ݽڵ����ڵ�������Ĺ�ϵ
	//GetOneDerivativeData();

	int KnotStartIndex, KnotEndIndex;
	int ClosedZone = 1;
	
	if (IsClosed) ClosedZone = dSpline;

	////����Ƿ���ѭ���Ŀ�ʼ����
	//if (IsClosed) KnotStartIndex = dSpline * 2 + 1;
	//else KnotStartIndex = dSpline;
	//
	//if (IsClosed) KnotStartIndex = dSpline + dSpline - 1;
	//else KnotStartIndex = dSpline;

	//////����Ƿ��������Ľ�������
	//if (IsClosed) KnotEndIndex = nSpline - dSpline + 1;
	//else KnotEndIndex = nSpline;

	KnotStartIndex = dSpline;
	KnotEndIndex = nSpline;

	PointUValues.clear();
	CurvePoints->points.clear();

	double UValue = KnotVector[KnotStartIndex];
	double Dis = 0, Error = 0.001;	//���������С�Ļ�������ʶ���һ����ʧ�� 0.001
	CalcBase<double> CalcBaseFloat;

	pcl::PointXYZRGB PriorPoint = GetSplinePoint(UValue);
	
	pcl::PointXYZRGB NextPoint;

	bool FindFirstNode = false;
	//������Ǳպ����� ��Ҫ�ӿ�ʼλ�ü���
	if (!IsClosed) 
	{
		CurvePoints->points.push_back(PriorPoint);	//����ǱպϾ�Ҫ���ϵ�һ���ڵ� 2016.12.12
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

		//���սڵ������������ ��ʾ
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
			
		//Ѱ�Ҵ�����ķ�͹����
			//���ҵ��պ����ߵ���β�ڵ�ʱ ��Ч
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
			//��ɫ����
			NextPoint.rgba = ColorBase::BlueColor;

		//PointAndDerivative CurrentPoint;	
		
		if (UValue < KnotVector[KnotStartIndex + ClosedZone])
		{	//��ʼ����
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
			//��������
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
			//����׵���
		//if (DerivativeOrder > 0)
		//{
		//	CurrentPoint.push_back(NextPoint);
		//	for (int Order = 1; Order <= DerivativeOrder; Order++)	//�Դ�����׵���
		//	{
		//		//��ȡ�� Order �� ����
		//		pcl::PointXYZRGB DerivativePoint = GetSplineDerivativePoint(UValue, Order);
		//		CurrentPoint.push_back(DerivativePoint);
		//	}	
		//	CurvePointsAndDerivativeS.push_back(CurrentPoint);
		//}
		//����׵���
		//cout<<"��ǰ u ֵΪ " <<U<<" �˴��������� "<<GetCurvature(U)<<" �˴��������� "<<GetTorsion(U)<<endl; 
						
		PriorPoint = NextPoint;
		UValue = UValue + UStep;		
	}
	//cout<<"�������߹������"<<endl;
	if (IsConvexhullControl)
	{
		for(int i = 0 ; i < NoConvexHullStartU.size(); i++)
		{
			double UTempMiddle = (NoConvexHullStartU[i] + NoConvexHullEndU[i]) / 2.0;
			pcl::PointXYZRGB BasePoint = GetSplinePoint(UTempMiddle);
			//pcl::PointXYZRGB TempPoint = GetSplineDerivativePoint(UTempMiddle, 2);
			//2016.03.26  ����ʾ�����ŵ�����ֵ
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
				//2016.03.26  ����ʾ�����ŵ�����ֵ
				//Viewer->addText3D(StringBase::IntToStr(i) + "-1", Point1, 0.5, 255,0,0,StringBase::ClockValue());
				//Viewer->addText3D(StringBase::IntToStr(i) + "-2", Point2, 0.5, 0, 255,0,StringBase::ClockValue());
			}
		}
	}

	return;
	
	////��ȡ�ڵ������Ĳ���
	////GetKnoteVectorStep();

	////��KnotVector[d] ��ʼ �� KnotVector[n] ���� �˴���nΪ�ڵ������1 d �������Ĵ���
	//double U = KnotVector[dSpline];		
	////ע��ڵ���������ʼλ��
	//while((U - KnotVector[nSpline]) < eps)
	//{					
	//	PointAndDerivative CurrentPoint;

	//	pcl::PointXYZRGB TempPoint = GetSplinePoint(U);
	//	CurvePoints->points.push_back(TempPoint);

	//	//����׵���
	//	if (DerivativeOrder > 0)
	//	{
	//		CurrentPoint.push_back(TempPoint);
	//		for (int Order = 1; Order <= DerivativeOrder; Order++)	//�Դ�����׵���
	//		{
	//			//��ȡ�� Order �� ����
	//			pcl::PointXYZRGB DerivativePoint = GetSplineDerivativePoint(U, Order);
	//			CurrentPoint.push_back(DerivativePoint);
	//		}	
	//		CurvePointsAndDerivativeS.push_back(CurrentPoint);
	//	}
	//	//����׵���
	//	//cout<<"��ǰ u ֵΪ " <<U<<" �˴��������� "<<GetCurvature(U)<<" �˴��������� "<<GetTorsion(U)<<endl; 
	//	U = U + UStep;
	//}
}

//��ȡ������SplinePoint 2020.08.11
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

//����uֵ���� u ���ַ�����ʽ���ʽ 2015.10.17
vector<string> CSpline::GetUFormula(double U)
{
	vector<string> ResultStrS;

	ResultStrS.push_back("");
	ResultStrS.push_back("");
	ResultStrS.push_back("");

	int KnotVectorIndex = FindUIndex(U);
	CalcBase<double> CalcBaseFloat;

	//for(int j = 0; j < d; j++) //d�ǽ���ʱ��ʹ�����
	for(int j = 0; j < dSpline + 1; j++) //�����ϵĵ�ʹ��ǰ dSpline + 1 �����Ƶ�
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

//���ݽڵ�������ֵ��ȡ�ڵ� 2015.09.24
pcl::PointXYZRGB CSpline::GetSplinePoint(double U)
{
	pcl::PointXYZRGB TempPoint;
	TempPoint.x = 0;
	TempPoint.y = 0;
	TempPoint.z = 0;
	TempPoint.rgba = ColorBase::RedColor;
	
	int KnotVectorIndex = FindUIndex(U);

	//for(int j = 0; j < d; j++) //d�ǽ���ʱ��ʹ�����
	for(int j = 0; j < dSpline + 1; j++) //�����ϵĵ�ʹ��ǰ dSpline + 1 �����Ƶ�
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

////���� һ�׵����Ŀ��Ƶ���ڵ����� ����u����һ�׵��� 
pcl::PointXYZRGB CSpline::GetDerivativePointByPoints(double u)
{
	pcl::PointXYZRGB TempPoint;
	TempPoint.x = 0;
	TempPoint.y = 0;
	TempPoint.z = 0;
	TempPoint.rgba = ColorBase::RedColor;

	int KnotVectorIndex = FindUIndex(u) - 1;

	int TempdSpline = dSpline - 1;
	//for(int j = 0; j < d; j++) //d�ǽ���ʱ��ʹ�����
	for (int j = 0; j < TempdSpline + 1; j++) //�����ϵĵ�ʹ��ǰ dSpline + 1 �����Ƶ�
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

//���ݽڵ������ĵ� k �׵���ʸ�� 2015.09.24
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

////���ɾ��ȵ������ڵ�������0Ϊ��ֵ, 1Ϊ���
//vector<double> CSpline::CreatePeriodicKnoteVector(int Number)
//{
//	vector<double> KnotVectorValue;
//	for(int i = 0; i < Number; i++)
//	{
//		KnotVectorValue.push_back(i);	
//	}
//	return KnotVectorValue;
//}

//����������
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

//���ýڵ�����
void CSpline::SetKnoteVector(vector<double> KnotVectorValue)
{
	KnotVector = KnotVectorValue;	
}

//�����ڵ�����ֵ���ڵ�����
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

//���ɿ��ž��������Ľڵ�����
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

//���ɾ��������Ľڵ�����
void CSpline::CreateUniformKnoteVector()
{
	KnotVector.clear();
	for(int i = 0; i < nSpline + dSpline + 1; i++)
	{
		KnotVector.push_back(i*1.0);		
	}	
}

////��ȡ������Step
//void CSpline::GetKnoteVectorStep()
//{
//	//��ȡ�ڵ����������ֵ����Сֵ
//	for(int i = dSpline; i <= nSpline; i++)
//	{
//		if (KnotVectorMinValue > KnotVector[i])
//			KnotVectorMinValue = KnotVector[i];
//		if (KnotVectorMaxValue < KnotVector[i])
//			KnotVectorMaxValue = KnotVector[i];
//	}
//	//UStep = (KnotVectorMaxValue - KnotVectorMinValue) / CurvePointsNumber;	
//}

//���������ߵ� Order ������ Ĭ�� Order = 1
void CSpline::DrawSplineDerivative(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
	int StartIndex, int EndIndex, int Order, string SplineName,
	int r, int g, int b, double LineLength)
{	
	if (CurvePointsAndDerivativeS.size() == 0)
	{
		cout<<"��δ��ȡ�ڵ�ĸ��׵�������"<<endl;
		return ;
	}

	if (CurvePointsAndDerivativeS[0].size() < Order + 1)
	{
		cout<<"��δ��ȡ�ڵ�ĵ� "<<Order<<" �׵ĵ�������"<<endl;
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

//������������ U ���� Order ������ Ĭ�� Order = 1
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

		Viewer->addText3D(CalcBaseInt.ConvertToString(i) + "�׵�", DerivativePoint, 0.5, 1,1,1,
			SplineName+ "_Text" + CalcBaseInt.ConvertToString(i) + "_" + CalcBaseInt.ConvertToString(Order));

		Viewer->addLine(BasePoint, DerivativePoint, r, g, b,
			SplineName + "_" + CalcBaseInt.ConvertToString(i) + "_"
			+ CalcBaseInt.ConvertToString(Order));
	}
}


//����㹹�����ߵ�͹�� 2015.12.04
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
			//cout<<"���ڴ����"<<i<<"���Ƕȷ���"<<endl;
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

			/////////�����ǻ�����
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
	cout<<"���� "<<NoConvexhull<<" ����͹��, ��͹�������"<<100.0*NoConvexhull/360.0<<"%"<<endl;
}

//�жϴ˵��Ƿ���͹�����ϵĵ�
bool CSpline::IsConvexHullPoint(int AngleIndex, pcl::PointXYZRGB TangentLinePoint,
	vector<AnglePartitionStruct> & SectionAngle)
{
	pcl::PointXYZRGB BasePoint = CurvePoints->points[SectionAngle[AngleIndex].PointIndexs[0]];
	
	//���� y=a*x+b
	double a = TangentLinePoint.y / TangentLinePoint.x;
	double b = BasePoint.y-a * BasePoint.x;

	if (AngleIndex == 191) 
		cout<<endl;

	int Flag = 0;
	bool Return = true;

	//ѡ��������һ���ο�����������һ����
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
	if (Type == "BSpline")	//�Ǿ���B����
	{		
		return GetSplineDerivativePoint(PointUValues[PointIndex], 1);
	}
	else	//��Ȼ��������
	{
		
	}
}

//����Simpson�����ʽ�������� �� ���� u λ�ô��ĳ��� KValue �ǷֵĶ���, 
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

		U = U + hStep;//�ӵ�1���㿪ʼִ��ѭ������ʼ���ǵ�0����
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

//����Simpson�����ʽ�������߳���
double CSpline::GetSplineLengthBySimpson()
{
	double Length = 0;
	double UStart, UEnd, U;
	double aValue, bValue;
	//cout << "���ڼ����������߳��ȣ�" << endl;

	//Simpson��ʽ �ֵĶ���		
	if (IsClosed && PointUValues.size() == 0)
	{
		//cout<<"��¼�ڵ������޷����㳤�ȣ�"<<endl;
		return 0;
	}

	//��������ڵ�������Ӧ��������������

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
	for(int i = StartUZoneIndex; i <= EndUZoneIndex; i++) //ÿһ��
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

			U = U + hStep;//�ӵ�1���㿪ʼִ��ѭ������ʼ���ǵ�0����
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

		//����ǰ�ڵ����������丳ֵ
		//��������
		SplineLengthOfKnotVector[i + 1] = Length;

		//�߶�����
		SplineHeightOfKnotVector[i + 1] = EndPoint.z - SplineStartPoint.z;
		
	}

	return Length;
}

	//��ȡ�պ�����������Χ��������
double CSpline::GetSplineArea()
{
	double Area = 0;
	//cout<<"���ڼ�����������Ժ�"<<endl;
	if (PointUValues.size() == 0)
	{
		//cout<<"��¼�ڵ������޷��������"<<endl;
		return 0;
	}
	//��ʼ�Ľڵ���������
	int StartUZoneIndex = FindUIndex(PointUValues[0]);	
	int EndUZoneIndex = FindUIndex(PointUValues[PointUValues.size() - 1]); 

	//��ʼ�Ľڵ�����
	double UStart = PointUValues[0];
	double UEnd = PointUValues[PointUValues.size() - 1];

	double TempUStart, TempUEnd; 
	string LastDFormulaStrX = "", LastDFormulaStrY ="";
	//double LastEnd;

	for(int i = StartUZoneIndex; i <= EndUZoneIndex; i++) //ÿһ��
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

		//cout<<"���ڼ���� "<<i<<" �ε����"<<endl;
		//
		//cout<<"������Ӧ��: "<<TempPoint<<endl;
		//cout<<"���ʽ�����X����: "<<FormulaAnalytic.ResolveValue(FormulaStrX, UMiddle)<<endl;
		//cout<<"���ʽ�����Y����: "<<FormulaAnalytic.ResolveValue(FormulaStrY, UMiddle)<<endl;
		//
		//cout<<"������:"<<endl;
		//pcl::PointXYZRGB CurrentDeriPoint = this->GetSplineDerivativePoint(UMiddle,1);
		//cout<<"������Ӧ���������: "<<CurrentDeriPoint<<endl;
		//cout<<"���ʽ�����������X����: "<<FormulaAnalytic.ResolveValue(DFormulaStrX, UMiddle)<<endl;
		//cout<<"���ʽ�����������Y����: "<<FormulaAnalytic.ResolveValue(DFormulaStrY, UMiddle)<<endl;


		//if (LastDFormulaStrX != "")
		//{
		//	cout<<"��һ�������㴦�ĵ���X: "<<FormulaAnalytic.ResolveValue(LastDFormulaStrX,LastEnd)<<endl;
		//	cout<<"��һ�������㴦�ĵ���Y: "<<FormulaAnalytic.ResolveValue(LastDFormulaStrY,LastEnd)<<endl;	

		//	cout<<"��ǰ��ʼ�㴦�ĵ���X: "<<FormulaAnalytic.ResolveValue(DFormulaStrX,	TempUStart)<<endl;
		//	cout<<"��ǰ��ʼ�㴦�ĵ���Y: "<<FormulaAnalytic.ResolveValue(DFormulaStrY,	TempUStart)<<endl;				
		//}
		//cout<<endl;
		//LastDFormulaStrX = DFormulaStrX;
		//LastDFormulaStrY = DFormulaStrY;
		//LastEnd = TempUEnd; 
		
		string Formula;
		//�˶��Ǹ��ݸ�ʽ��ʽP = -y, Q = x
		Formula = "((" + FormulaStrX + ")*(" + DFormulaStrY + "))-((" 
				+ FormulaStrY + ")*(" + DFormulaStrX + "))";

		////�˶��Ǹ��ݸ�ʽ��ʽP = 0, Q = x
		Formula = "2*((" + FormulaStrX + ")*(" + DFormulaStrY + "))";

		////�˶��Ǹ��ݸ�ʽ��ʽP = y, Q = 0
		Formula = "-2*((" + FormulaStrY + ")*(" + DFormulaStrX + "))";

		FormulaAnalytic.SetInputs(Formula);		
		Formula = FormulaAnalytic.GetFormula();

	/*	FormulaStrX = "(" + FormulaStrX + ")*(" + DFormulaStrY + ")"; 
		FormulaAnalytic.SetInputs(FormulaStrX);
		FormulaStrX = FormulaAnalytic.GetFormula();

		FormulaStrY = "(" + FormulaStrY + ")*(" + DFormulaStrX + ")"; 
		FormulaAnalytic.SetInputs(FormulaStrY);
		FormulaStrY = FormulaAnalytic.GetFormula();
		///��Ҫ�и���������
		string TempFormulaStrY = FormulaAnalytic.OppositeFormula(FormulaStrY);
		if (TempFormulaStrY.substr(0,1) == "+" || TempFormulaStrY.substr(0,1) == "-")
			Formula = FormulaStrX + TempFormulaStrY;
		else
			Formula = FormulaStrX + "+" + TempFormulaStrY;	*/

		Formula = FormulaAnalytic.PolynomialMerge(Formula);
		double TempArea = FormulaAnalytic.ResolveIntegralValue(Formula, TempUStart, TempUEnd);
		//cout<<"��ǰ�������:"<<TempArea<<endl;
		
		Area = Area + TempArea;
	}
	//cout<<"�����������ɣ�"<<endl;
	return Area /2.0;
}

//��ȡ��͹�������ʼ��Χ 2015.12.22 ֻ�����������
bool CSpline::GetNoConvexHullZone(int i, double & NoConvexHullUStart, double & NoConvexHullUEnd)
{
	if(dSpline != 3)
	{
	//	cout<<"�˷���ֻ�������������"<<endl;	
		return true;
	}
	
	if (i >= nSpline || i < dSpline) 
	{	cout<<"�����˽ڵ�������ȡֵ��Χ"<<endl; return true;}
	
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
		cout<<"����ʽ��ϼ������X���ʽ�Ľ���"<<XCoefficient.size()
			<<"��������ָ���Ľ���"<<dSpline<<endl;
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
		cout<<"����ʽ��ϼ������Y���ʽ�Ľ���"<<XCoefficient.size()
			<<"��������ָ���Ľ���"<<dSpline<<endl;
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
		//cout<<"�������ް�������"<<endl;
	else if(d > 0 && a > 0)
	{
		double RootLeft, RootRight;
		RootLeft = (-b-sqrt(d))/2/a;
		RootRight = (-b+sqrt(d))/2/a;
		if (UEnd < RootLeft || UStart > RootRight)
		{	
			cout<<"�������ް�������"<<endl;
			ReturnValue = true;
		}
		else if( UStart > RootLeft &&  UEnd < RootRight)
		{
			ReturnValue = false;
			NoConvexHullUStart = UStart;
			NoConvexHullUEnd = UEnd;
			cout<<"������ȫ���ǰ�������"<<endl;
		}
		else
		{
			if( UStart > RootLeft &&  UStart < RootRight)			
			{
				cout<<"��"<<i<<"�����䰼������Ϊ "<< UStart<<" �� "<<RootRight<<endl;
				ReturnValue = false;
				NoConvexHullUStart = UStart;
				NoConvexHullUEnd = RootRight;
			}
			else if( UEnd > RootLeft &&  UEnd < RootRight)
			{
				cout<<"��"<<i<<"�����䰼������Ϊ "<< RootLeft<<" �� "<<UEnd<<endl;
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
			cout<<"��"<<i<<"�������ް�������"<<endl;
			ReturnValue = true;
		}
		else if (UStart < RootLeft && UEnd > RootLeft)
		{
			cout<<"��"<<i<<"�����䰼������Ϊ "<<UStart<<" �� "<<RootLeft<<endl;
			ReturnValue = false;
			NoConvexHullUStart = UStart;
			NoConvexHullUEnd = RootLeft;
		}
		else if (UStart < RootLeft && UEnd > RootRight)	//����������
		{
			cout<<"��"<<i<<"�����䰼������Ϊ "<<UStart<<" �� "<<RootLeft <<" �� ����"<<RootRight<<" �� "<<UEnd<<endl;
			ReturnValue = false;
			NoConvexHullUStart = UStart;
			NoConvexHullUEnd = RootLeft;
		}
		else if (UStart < RootRight && UEnd > RootRight)
		{
			cout<<"��"<<i<<"�����䰼������Ϊ "<<RootRight<<" �� "<<UEnd<<endl;
			ReturnValue = false;
			NoConvexHullUStart = RootRight;
			NoConvexHullUEnd = UEnd;
		}	
		else if (RootRight < UStart ||  UEnd < RootLeft)
		{
			ReturnValue = false;
			NoConvexHullUStart = UStart;
			NoConvexHullUEnd = UEnd;
			cout<<"������ȫ���ǰ�������"<<endl;
		}
	}
	return ReturnValue;
}

//�����͹�����λ�� 2015.12.22 �˴�ֻ�����������
void CSpline::JudgeNoConvexhullKnoteValue(int i, string XFormula, string YFormula, 
		double UStart, double UEnd)
{
	if(dSpline != 3){cout<<"�˷���ֻ�������������"<<endl;	return;}
	vector<double> XCoefficient, YCoefficient;
	vector<int> XOrder, YOrder;
	CFormulaAnalytic FormulaAnalytic;
	FormulaAnalytic.ExtractCofficientAndVairantOrder(XFormula, XCoefficient, XOrder);
	FormulaAnalytic.ExtractCofficientAndVairantOrder(YFormula, YCoefficient, YOrder);
	
	cout<<"��"<<i<<"������ "<<endl;
	//if (i == 6)
	//	cout<<endl;
	if (XCoefficient.size() > dSpline + 1)
		cout<<"����ʽ��ϼ������X���ʽ�Ľ���"<<XCoefficient.size()
			<<"��������ָ���Ľ���"<<dSpline<<endl;
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
		cout<<"����ʽ��ϼ������Y���ʽ�Ľ���"<<XCoefficient.size()
			<<"��������ָ���Ľ���"<<dSpline<<endl;
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
		cout<<"�������ް�������"<<endl;
	else if(d > 0 && a > 0)
	{
		double RootLeft, RootRight;
		RootLeft = (-b-sqrt(d))/2/a;
		RootRight = (-b+sqrt(d))/2/a;
		if (UEnd < RootLeft || UStart > RootRight)
		{	//cout<<"�������ް�������"<<endl;
		}
		else if( UStart > RootLeft &&  UEnd < RootRight)
			cout<<"������ȫ���ǰ�������"<<endl;
		else
		{
			if( UStart > RootLeft &&  UStart < RootRight)			
				cout<<"�����䰼������Ϊ "<< UStart<<" �� "<<RootRight<<endl;
			else if( UEnd > RootLeft &&  UEnd < RootRight)
				cout<<"�����䰼������Ϊ "<< RootLeft<<" �� "<<UEnd<<endl;
		}
	}
	else if (d > 0 && a < 0)
	{
		double RootLeft, RootRight;
		RootLeft = (-b+sqrt(d))/2/a;
		RootRight = (-b-sqrt(d))/2/a;
		if (UStart > RootLeft && UEnd < RootRight)
		{
			//cout<<"�������ް�������"<<endl;
		}
		else if (UStart < RootLeft && UEnd > RootLeft)
			cout<<"�����䰼������Ϊ "<<UStart<<" �� "<<RootLeft<<endl;
		else if (UStart < RootLeft && UEnd > RootRight)
			cout<<"�����䰼������Ϊ "<<UStart<<" �� "<<RootLeft <<" �� ����"<<RootRight<<" �� "<<UEnd<<endl;
		else if (UStart < RootRight && UEnd > RootRight)
			cout<<"�����䰼������Ϊ "<<RootRight<<" �� "<<UEnd<<endl;
		
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

//��ȡ�ض�������� Relative = trueʱ�����������
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

//��ȡ�ض��������
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

//��ȡ�ض���λ�ô�������ˮƽ��ļн�
double CSpline::GetInclination(double U)
{
	pcl::PointXYZRGB OneDerivative = GetSplineDerivativePoint(U, 1, false);
	//2019.02.14 
	return 90.0 - 180.0 * GeometryBase::AngleOfTwoVector(
		OneDerivative.x, OneDerivative.y, OneDerivative.z, 0, 0, 1) / M_PI;	
}

	//��ȡ�ڵ������ж�Ӧ�����������ϵĵ�
void CSpline::GetKnotValuePoints()
{
	KnotValuePoints->points.clear();

	for(int i = dSpline; i <= nSpline; i++)	//����Щ�ڵ�֮�� ע��������ϵ
	{
		pcl::PointXYZRGB TempPoint;
		if ( i == nSpline)
			TempPoint = GetSplinePoint(0.999);	//	�������һ���ڵ�,�ǳ��ӽ������һ����ֵ��
		else
			TempPoint = GetSplinePoint(KnotVector[i]);
		KnotValuePoints->points.push_back(TempPoint);
	}
}

//����x(y��z)�����ȡ ���������ϵĽڵ�Uֵ
//��ȡ�ƽ���ʵ��
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

//���ݸ���������ֵ���������ֵ�¶�Ӧ�Ľڵ�
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

	//��UMinֵ �� UMin ֵ�� ʹ�ö��ַ���λ ��λ Axis ���� Value ��Ӧ�� Uֵ
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

//��ȡ��������λ�ô���Uֵ �˴���Ҫ�Ľ�����Ҫʹ��Simpson��ʽ����(2016.01.21)
double CSpline::GetUValueBySplineLength(double TempLength)
{
	//2018.09.25 modified by yl 
	VectorBase<double> DoubleCalcBase;
	
	if (SplineLengthOfKnotVector.size() == 0)	//�ȼ����������߳���
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

//��ȡ�����߶�λ�ô���Uֵ
double CSpline::GetUValueBySplineHeight(double TempHeight)
{
	//2018.09.25 modified by yl 
	VectorBase<double> DoubleCalcBase;

	if (SplineLengthOfKnotVector.size() == 0)	//�ȼ����������߳���
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

//��ȡһ�׵���������Ϣ ���ݽڵ��Ĺ�ϵ �� �ڵ����� ��Ĺ�ϵ
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
/////*******************************************����Ϊ����B������ʵ����



//���ɿ��Ƶ�Ȩ�� �ֽ׶�Ȩ�ض�����Ϊ1
void CRationalSpline::CreatePointWeights()
{
	PointWeights.clear();
	for(int i = 0; i < ControlPoints->points.size(); i++)
	{
		//PointWeights.push_back(i*1.0); Ȩ�ض�������Ӱ��
		PointWeights.push_back(1);	
	}
}

////���ÿ��Ƶ�Ȩ��
//void CRationalSpline::SetPointWeights(vector<double> PointWeightsValue)
//{
//	if (PointWeightsValue.size() != ControlPoints->points.size())
//	{
//		cout<<"���Ƶ�Ȩ������Ƶ�ĸ�������ͬ�����������"<<endl;
//		this->CanCreateCurve = false;
//		return;
//	}
//
//	PointWeights.clear();	
//	PointWeights.insert(PointWeightsValue.begin(), PointWeightsValue.end());
//}

////��������B�����ĵ�
//void CRationalSpline::CreateRationalSpline(int DerivativeOrder)
//{
//	if (KnotVector.size() == 0) 
//	{
//		CanCreateCurve = false;
//		cout<<"�ڵ�����Ϊ�գ��޷�����������"<<endl;
//		return; 		
//	}
//
//	if (KnotVector.size() != nSpline + dSpline + 1) 
//	{
//		cout<<"�ڵ��������������ڿ��Ƶ�ĸ���(n)��ײ����ĸ���(d)�ĺͲ��ȣ����ʵ��"<<endl;
//		CanCreateCurve = false;
//		return;
//	}
//
//	if (PointWeights.size() != ControlPoints->points.size())
//	{
//		cout<<"���Ƶ�Ȩ������Ƶ�ĸ�������ͬ�����������"<<endl;
//		CanCreateCurve = false;
//		return;
//	}
//
//	//if ( DerivativeOrder > d) DerivativeOrder = d;	//�������������� ��������
//
//	cout<<"���ڹ��� NURBS ���������ϵĵ�"<<endl;
//	
//	//��ȡ�ڵ������Ĳ���
//	GetKnoteVectorStep();
//
//	//��KnotVector[d] ��ʼ �� KnotVector[n] ���� �˴���nΪ�ڵ������1 d �������Ĵ���
//	double U = KnotVector[dSpline];		
//	//ע��ڵ���������ʼλ��
//	while((U - KnotVector[nSpline]) < eps)
//	{		
//		PointAndDerivative CurrentPoint;
//		
//		pcl::PointXYZRGB TempPoint = GetSplinePoint(U);		
//		
//		//��¼��ǰ�Ľڵ�
//		CurvePoints->points.push_back(TempPoint);
//		
//		//if (abs(U - 0.997499)  < eps)
//		//{
//		//	cout<<FindUIndex(U)<<endl;
//		//}
//		//����׵���  ��������������Ҫ����
//		if (DerivativeOrder > 0)
//		{
//			CurrentPoint.push_back(TempPoint);
//			for (int Order = 1; Order <= DerivativeOrder; Order++)	//��������׵���
//			{
//				pcl::PointXYZRGB DerivativePoint = 
//					GetSplineDerivativePoint(U, Order);
//
//				CurrentPoint.push_back(DerivativePoint);
//			}	
//			CurvePointsAndDerivativeS.push_back(CurrentPoint);
//		}
//		//����׵���
//		//cout<<"��ǰ u ֵΪ " <<U<<" �˴��������� "<<GetCurvature(U)<<" �˴��������� "<<GetTorsion(U)<<endl; 
//
//		U = U + UStep;
//	}
//	cout<<"�����������߹������"<<endl;
//}

//��������B�����Ļ�����Ni,p(u)��k�׵���   i �ǽڵ�����������d����������ʽ�Ĵ���, k���󵼵Ĵ��� 
// ���㷽�������Ǿ�������B������ ��91ҳ ��ʽ 4.8
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
	
	if (k == 1)	//��һ�׵���ʱ�Ĵ���
	{
		double W = DerivativeOfRationalCoxdeBoor_W(dSpline, u, 1);
		DerivativePointWC = GetSplinePoint(u);

		DerivativePointWC.x = DerivativePointWC.x * W;
		DerivativePointWC.y = DerivativePointWC.y * W;
		DerivativePointWC.z = DerivativePointWC.z * W;		
	}
	else	//���׼������ϵ���
	{
		//���������ϵ���� w �� C �Ķ�Ӧ�ĵ���
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
	//��ĸ
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

//���㷽�������Ǿ�������B������ ��91ҳ ��ʽ 4.8 �е� A �� k �׵���
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

//���㷽�������Ǿ�������B������ ��91ҳ ��ʽ 4.8 �е� W �� k �׵���
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


//���� ��ʽ 4.8 �ķ�ĸ
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

//����uֵ���� u ���ַ�����ʽ���ʽ 2015.10.17
vector<string> CRationalSpline::GetUFormula(double U)
{
	vector<string> ResultStrS;

	ResultStrS.push_back("");
	ResultStrS.push_back("");
	ResultStrS.push_back("");

	int KnotVectorIndex = FindUIndex(U);
	CalcBase<double> CalcBaseFloat;
	string DenominatorStr = "";

	//for(int j = 0; j < d; j++) //d�ǽ���ʱ��ʹ�����
	for(int j = 0; j < dSpline + 1; j++) //�����ϵĵ�ʹ��ǰ dSpline + 1 �����Ƶ�
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


//���ݽڵ�������ֵ��ȡ�ڵ� 2015.09.24
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
	//for(int j = 0; j < d; j++) //d�ǽ���ʱ��ʹ�����
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
