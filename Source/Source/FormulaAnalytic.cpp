//////��ʽ���� ���ַ��������Ķ���ʽ��ʽ���򣬺ϲ��������մ�����������
/////yl 2015.10.18
/////****************************************************************

#include "FormulaAnalytic.h"

//��������
void CFormulaAnalytic::SetInputs(string FormulaValue)
{
	Formula="";
	//����ȥ���ظ�������
	for(int i = 10; i > 1; i--)
	{
		FormulaValue = RemoveRepeatedBracket(FormulaValue, i);
	}	
	//ȥ����ʽ�еĿո� //2016.04.16
	Formula = ReplaceStr(FormulaValue, " ", "");	//
}

//���
string CFormulaAnalytic::GetFormula()
{
	ExtractVariableAndConstant(Formula);
	//cout<<"��Ҫ�����Ĺ�ʽΪ��"<<Formula<<endl;
	//FormulaStrToOperationItemS();
	//ExtractVariableAndConstant("+1.5*u-0.3*u+2U");
	Formula = PolynomialProduct(Formula);

	return Formula;
}

//ɾ����ʽ FormulaValue �ж�������� RepeatedNumber ��ʾ�����ظ��Ĵ���
string CFormulaAnalytic::RemoveRepeatedBracket(string FormulaValue, int RepeatedNumber)
{	
	string RightBracket = "";
	string LeftBracket = "";
	for(int i = 0; i < RepeatedNumber; i++)
	{
		RightBracket = RightBracket + ")";
		LeftBracket = LeftBracket + "(";
	}	

	//int Index = FormulaValue.find("))", 0);	
	int Index = FormulaValue.find(RightBracket, 0);	
	//cout<<FormulaValue<<endl;
	//û���ҵ� ���� �ҵ���������β����
	if (Index == -1) return FormulaValue; 
	int LeftEndIndex = Index + RepeatedNumber;	

	string LeftStr = FormulaValue.substr(0, LeftEndIndex); 
	string RightStr = "";
	
	if ( LeftEndIndex + 1 < FormulaValue.size()) 
	{
		 RightStr = FormulaValue.substr(LeftEndIndex, FormulaValue.size() - 1); 
	}	

	int LastIndexOfDoubleBracket = LeftStr.rfind(LeftBracket);
	int LastIndexOfSingleBracket = LeftStr.rfind(LeftBracket.substr(0, LeftBracket.size() - 1));

	if (LastIndexOfDoubleBracket + 1 == LastIndexOfSingleBracket)	//˵����������"(("
	{		
		string TempLeftStr = LeftStr.substr(0, LastIndexOfSingleBracket);
		//string TempLeftStr = LeftStr.substr(LastIndexOfDoubleBracket + RepeatedNumber - 1, 1);
		string TempRightStr = LeftStr.substr(LastIndexOfDoubleBracket + RepeatedNumber, 
			Index - LastIndexOfDoubleBracket - RepeatedNumber + 1);
		LeftStr = TempLeftStr + TempRightStr;					
	}

	if (RightStr == "")
		return LeftStr + RightStr;
	else
		return LeftStr + RemoveRepeatedBracket(RightStr, RepeatedNumber);	
}

//���ַ����� OldStr �滻Ϊ NewStr 2015.10.18
string CFormulaAnalytic::ReplaceStr(string FormulaValue, string OldStr, string NewStr)
{
	int Index = FormulaValue.find(OldStr);
	int LastIndex = Index;
	while(Index >= 0)
	{
		if (OldStr == " ")
		{
			LastIndex = Index;
			string Left = FormulaValue.substr(0, Index);
			string Right = FormulaValue.substr(Index + 1, 
					FormulaValue.size() - Index - 1); 
			FormulaValue = Left + Right;
			Index = FormulaValue.find(OldStr);
			if (Index == -1)
				break;		
		}
		else
		{
			LastIndex = Index;
			string Left = FormulaValue.substr(0, Index);
			string Right = FormulaValue.substr(Index + OldStr.size(), 
					FormulaValue.size() - Index - OldStr.size()); 
			FormulaValue = Left + NewStr + Right;
			Index = FormulaValue.find(OldStr);
			if (Index == LastIndex)
				break;
		}
	}
	return FormulaValue;
}

//����ʽ���Թ�ʽ���ܳ˻��Ķ���ʽ���ж���ʽ�˻� ������
string CFormulaAnalytic::PolynomialProduct(string FormulaValue)
{
	vector<string> VariableVec;
	int i = 0;
	string Resultstr = "";
	int ProductIndex = FormulaValue.find(")*(");
	while (ProductIndex > 0)
	{
		string Left = FormulaValue.substr(0, ProductIndex + 1);
		string Right = FormulaValue.substr(ProductIndex + 2, FormulaValue.size() - ProductIndex - 2);
		//�������Ŷ����������� ������ ))*((����� ��֮һ
		if (Left.substr(Left.size() - 2, 1) != ")" && Right.substr(1, 1) != "(")	
		{
			string ResultLeft, ResultRight;
			int LeftStartIndex = Left.rfind("(");
			string LeftOperationStr = Left.substr(LeftStartIndex, Left.size() - LeftStartIndex);
			ResultLeft = Left.substr(0, LeftStartIndex);

			int RightEndIndex = Right.find(")");
			string RightOperationStr = Right.substr(0, RightEndIndex + 1);
			ResultRight = Right.substr(RightEndIndex + 1,  Right.size() - RightEndIndex);
			
			//�˴���Ҫ����ǰ��������� �����Ƿ������

			//������������ǰ��һ���ַ�
			string LeftChar = "", RightChar = "";
			if (ResultLeft.size() > 0)
				LeftChar = ResultLeft.substr(ResultLeft.size() - 1, 1);
			if (ResultRight.size() > 0)
				RightChar = ResultRight.substr(0, 1);
			
			string TempStr = PolynomialProduct(LeftOperationStr, RightOperationStr);

			//�����������Ƿ���Ҫ������
			if (LeftChar=="*" || RightChar== "*" || LeftChar=="/" || RightChar== "/" )
				FormulaValue = ResultLeft + "(" + TempStr 
						+ ")" + ResultRight;
			else
			{	if (LeftChar=="+") //ȥ��ǰ������+��-��
				{
					if (TempStr.substr(0,1) == "+" || TempStr.substr(0,1) == "-" )	
						FormulaValue =
							ResultLeft.substr(0, ResultLeft.size() - 1) + TempStr + ResultRight;
					else
						FormulaValue = ResultLeft + TempStr + ResultRight;
				}
				else if (LeftChar=="-")
				{
					if (TempStr.substr(0,1) == "+")
						FormulaValue = ResultLeft + TempStr.substr(1, TempStr.size() -1) + ResultRight;
					else if (TempStr.substr(0,1) == "-")
						FormulaValue = ResultLeft.substr(0, ResultLeft.size() - 1) + "+" + TempStr.substr(1, TempStr.size() -1) + ResultRight;
					else
						FormulaValue = ResultLeft + TempStr + ResultRight;
				}
				else
					FormulaValue = ResultLeft + TempStr + ResultRight;
			}

			//cout<<"��"<<i++<<"����ɵĲ��ֽ����Ĺ�ʽ��"<<FormulaValue<<endl;			
			FormulaValue = RemoveRepeatedBracket(FormulaValue,2);
			ProductIndex = FormulaValue.find(")*(");
		}
		else
			ProductIndex = -1;
	}
	//cout<<endl<<"������ʽ:"<<FormulaValue<<endl<<endl;
	return PolynomialNumberProduct(FormulaValue);
}

//����ʽ ����ʽ����ֵ�˻� ����ִ����һ����
string CFormulaAnalytic::PolynomialNumberProduct(string FormulaValue)
{	
	if ((FormulaValue.substr(0,1) =="(")) 
	{
		FormulaValue = "+" + FormulaValue;
	}

	FormulaValue = ReplaceStr(FormulaValue, "+(", "+1.0*(");
	FormulaValue = ReplaceStr(FormulaValue, "-(", "-1.0*(");

	string GlobalTempStr = FormulaValue;
	int ProductIndex = FormulaValue.find("*(");		

	while (ProductIndex > 0)
	{
		string LeftStr = FormulaValue.substr(0, ProductIndex);
		string RightStr = FormulaValue.substr(ProductIndex + 2, FormulaValue.size()- ProductIndex - 2);

		////�˴���Ҫ�����ѧ����e�ı�ʾ
		//int LeftStartPositiveIndex = LeftStr.rfind("+");
		//int LeftStartNegativeIndex = LeftStr.rfind("-");

		////��û�У�˵����һ������������
		string ResultLeft, ResultRight;
		double Number = 0;

		//int LeftStartIndex = 0;
		//if (LeftStartPositiveIndex == -1 && LeftStartNegativeIndex == -1)		
		//	LeftStartIndex = 0;		
		//else if (LeftStartPositiveIndex > LeftStartNegativeIndex)		
		//	LeftStartIndex = LeftStartPositiveIndex;
		//else		
		//	LeftStartIndex = LeftStartNegativeIndex;
		string LeftOperationStr;
		//if (LeftStartPositiveIndex > 0)
		//	LeftOperationStr = LeftStr.substr(LeftStartIndex, 
		//		LeftStr.size() - LeftStartPositiveIndex);
		//else
		//	LeftOperationStr = LeftStr.substr(LeftStartIndex, 
		//	LeftStr.size() - LeftStartPositiveIndex + 1);
	

		//2016.04.15����
		int LeftStartPositiveIndex = FindLastOperator(LeftStr);

		if( LeftStr!= "" && LeftStartPositiveIndex == -1)
			LeftStartPositiveIndex = 0;

		LeftOperationStr = LeftStr.substr(LeftStartPositiveIndex, LeftStr.size() - LeftStartPositiveIndex);
		//2016.04.15����
		//int FindE0 = LeftOperationStr.find("e+");
		//int FindE1 = LeftOperationStr.find("e-");
		//if (FindE0 > 0 || FindE1 > 0)
		//{
		//	if (LeftOperationStr.substr(0,1) == "-" || LeftOperationStr.substr(0,1) == "+")
		//		LeftOperationStr = "0" + LeftOperationStr;
		//	
		//	Number= Arithmetic::expressionCalculate(LeftOperationStr);
		//}
		//else
		//	Number = atof(LeftOperationStr.c_str());

		Number = atof(LeftOperationStr.c_str());
		if (LeftStartPositiveIndex > 0)
			ResultLeft = LeftStr.substr(0, LeftStartPositiveIndex);
		else
			ResultLeft ="";

		int RightEndIndex = RightStr.find(")");
		string RightOperationStr = RightStr.substr(0, RightEndIndex); 
		ResultRight = RightStr.substr(RightEndIndex + 1,  RightStr.size() - RightEndIndex);

		string TempStr = PolynomialNumberProduct(Number, RightOperationStr);
		if (TempStr.substr(0,1) != "+" && TempStr.substr(0,1) != "-")
			FormulaValue = ResultLeft + "+" + TempStr + ResultRight;
		else
			FormulaValue = ResultLeft + TempStr + ResultRight;

		ProductIndex = FormulaValue.find("*(");	
	}
	
	return PolynomialMerge(FormulaValue);
}

//�Ƴ���ʽ�е����ţ�
string CFormulaAnalytic::RemoveBracket(string FormulaValue)
{	
	if (FormulaValue.substr(0,1) != "+" || FormulaValue.substr(0,1) != "-")
		FormulaValue = "+" + FormulaValue;	

	int ProductIndex = FormulaValue.find("+(");	
	while (ProductIndex >= 0)	
	{
		string LeftStr = FormulaValue.substr(0, ProductIndex);
		string RightStr = FormulaValue.substr(ProductIndex + 2, FormulaValue.size()- ProductIndex - 2);
		
		int RightIndex = RightStr.find(")");
		string MiddleStr = RightStr.substr(0, RightIndex);
		string ResultRight = RightStr.substr(RightIndex + 1, RightStr.size() - RightIndex);

		if (MiddleStr.substr(0,1) != "+" || MiddleStr.substr(0,1) != "-")
			MiddleStr = "+" + MiddleStr;	

		FormulaValue = LeftStr + MiddleStr + ResultRight;
		ProductIndex = FormulaValue.find("+(");	
	}

	ProductIndex = FormulaValue.find("-(");	
	while (ProductIndex >= 0)	
	{
		string LeftStr = FormulaValue.substr(0, ProductIndex);
		string RightStr = FormulaValue.substr(ProductIndex + 2, FormulaValue.size()- ProductIndex - 2);
		
		int RightIndex = RightStr.find(")");
		string MiddleStr = RightStr.substr(0, RightIndex);
		string ResultRight = RightStr.substr(RightIndex + 1, RightStr.size() - RightIndex);

		MiddleStr = OppositeFormula(MiddleStr);

		if (MiddleStr.substr(0,1) != "+" || MiddleStr.substr(0,1) != "-")
			MiddleStr = "+" + MiddleStr;	

		FormulaValue = LeftStr + MiddleStr + ResultRight;
		ProductIndex = FormulaValue.find("-(");	
	}
	return FormulaValue;
}

string CFormulaAnalytic::PolynomialNumberProduct(double Value, string FormulaValue)
{
	FormulaValue = PolynomialMerge(FormulaValue);
	vector<string> RightOperand = SeparationOperand(FormulaValue);
	CalcBase<double> CalcBaseFloat;

	string ResultStr = "";

	for(int i = 0; i < RightOperand.size(); i++)
	{
		double RightCoefficient;

		vector<string> RightVariableAndConstant;
		RightCoefficient = ExtractCoefficientAndOthers(RightOperand[i], RightVariableAndConstant);	

		double CalcValue = Value * RightCoefficient;	
		string TempValue = CalcBaseFloat.ConvertToString(abs(CalcValue));

		string SymbolValue;
		if (CalcValue >= 0)
			SymbolValue = "+";
		else
			SymbolValue = "-";

		string TempVariableAndConstant = "";
		for(int m = 0; m < RightVariableAndConstant.size(); m++)
		{
			if (TempVariableAndConstant == "")
				TempVariableAndConstant =  RightVariableAndConstant[m];
			else
				TempVariableAndConstant = TempVariableAndConstant + "*" + RightVariableAndConstant[m];
		}

		if (TempVariableAndConstant == "")
			ResultStr = ResultStr + SymbolValue + TempValue;
		else
			ResultStr = ResultStr + SymbolValue + TempValue + "*" + TempVariableAndConstant;	
	}
	return ResultStr;
}

//�԰������ŵ���������г˻����� 2015.10.18  �� (x-a)*(x-b) ��ʽ���޿ո�
string CFormulaAnalytic::PolynomialProduct(string LeftPart, string RightPart)
{
	//�Ƚ�����ʽ�ϲ�
	LeftPart = PolynomialMerge(LeftPart);
	RightPart = PolynomialMerge(RightPart);

	vector<string> LeftOperand = SeparationOperand(LeftPart);
	vector<string> RightOperand = SeparationOperand(RightPart);
	CalcBase<double> CalcBaseFloat;
	
	string ResultStr = "";

	for(int i = 0; i < LeftOperand.size(); i++)
	{
		double LeftCoefficient;

		vector<string> LeftVariableAndConstant;
		LeftCoefficient = ExtractCoefficientAndOthers(LeftOperand[i], LeftVariableAndConstant);

		//ѭ�����Ҳ�����������
		for(int j = 0; j < RightOperand.size(); j++)
		{
			double RightCoefficient;
			vector<string> RightVariableAndConstant;
			RightCoefficient = ExtractCoefficientAndOthers(RightOperand[j], RightVariableAndConstant);

			double CalcValue = LeftCoefficient * RightCoefficient;						
			string TempValue = CalcBaseFloat.ConvertToString(abs(CalcValue)); 				

			string SymbolValue;
			if (CalcValue >= 0)
				SymbolValue = "+";
			else
				SymbolValue = "-";

			string TempVariableAndConstant = "";
			for(int m = 0; m < LeftVariableAndConstant.size(); m++)
			{				
				if (TempVariableAndConstant == "")
					TempVariableAndConstant = LeftVariableAndConstant[m];
				else
					TempVariableAndConstant = TempVariableAndConstant + "*" + LeftVariableAndConstant[m];				
			}
			for(int m = 0; m < RightVariableAndConstant.size(); m++)
			{
				if (TempVariableAndConstant == "")
					TempVariableAndConstant =  RightVariableAndConstant[m];
				else
					TempVariableAndConstant = TempVariableAndConstant + "*" + RightVariableAndConstant[m];
			}

			if (TempVariableAndConstant == "")
				ResultStr = ResultStr + SymbolValue + TempValue;
			else
				ResultStr = ResultStr + SymbolValue + TempValue + "*" + TempVariableAndConstant;		
		}
	}
	
	//�˴����������ʽ�ϲ�
	return PolynomialMerge(ResultStr);
}

//2015.12.15 ����ʽ�ϲ�������ͬ�����Ķ���ʽ��� �˴�ֻ����һ������ u ����������������� ��Ҫ����
string CFormulaAnalytic::PolynomialMerge(string FormulaValue)
{	
	//string TempStr = RemoveBracket(FormulaValue);
	//if (FormulaValue == "(u)")
	//	cout<<endl;
	string TempStr = FormulaValue;
	vector<double> CoefficientS;
	vector<int> OrderS;
	CalcBase<double> CalcBaseDouble;

	vector<string> OperandStr = SeparationOperand(TempStr);
	for(int i = 0; i < OperandStr.size(); i++)
	{
		vector<string> VariableAndConstant;
		double Coefficient = ExtractCoefficientAndOthers(OperandStr[i], VariableAndConstant);
		CoefficientS.push_back(Coefficient); //ϵ��
		OrderS.push_back(VariableAndConstant.size()); //����
	}

	vector<double> ResultCoefficientS;
	vector<int> ResultOrderS;

	for(int i = 0; i < OperandStr.size(); i++)
	{		
		bool Find = false;
		//��Ҫ�ڽ������Ѱ �Ƿ��Ѿ����ڣ������ڣ������ۼƣ�������������
		for(int j = 0; j < ResultOrderS.size(); j++)
		{
			if (ResultOrderS[j] == OrderS[i])
			{				
				ResultCoefficientS[j] = ResultCoefficientS[j] + CoefficientS[i];
				Find = true;
				break;
			}
		}
		if (!Find)
		{
			ResultOrderS.push_back(OrderS[i]);
			ResultCoefficientS.push_back(CoefficientS[i]);
		}
	}

	//�����ս�����ս״ν�������

	for(int i = 0; i < ResultOrderS.size(); i++)
	{
		for(int j = i + 1; j < ResultOrderS.size(); j++)
		{
			if (ResultOrderS[i] < ResultOrderS[j])	//ǰ��ıȺ����С
			{
				swap(ResultOrderS[i], ResultOrderS[j]);
				swap(ResultCoefficientS[i], ResultCoefficientS[j]);
			}
		}
	}

	//���
	TempStr = "";
	for(int i = 0; i < ResultOrderS.size(); i++)
	{
		string Variant = StringBase::GetRepeatedStr("u", ResultOrderS[i], "*");
		if (TempStr == "")
		{
			if (ResultCoefficientS[i] != 0)
			{
				if (Variant!= "")
					TempStr = CalcBaseDouble.ConvertToString(ResultCoefficientS[i]) 
						+ "*" + Variant;
				else
					TempStr = CalcBaseDouble.ConvertToString(ResultCoefficientS[i]);
			}
		}
		else
		{
			if (ResultCoefficientS[i] > 0) //���ǵ���������
			{	if  (Variant != "")
					TempStr = TempStr + "+" + CalcBaseDouble.ConvertToString(ResultCoefficientS[i])
						+ "*" + Variant;
				else
					TempStr = TempStr + "+" + CalcBaseDouble.ConvertToString(ResultCoefficientS[i]);
			}
			else if (ResultCoefficientS[i] < 0)
			{
				if  (Variant != "")
					TempStr = TempStr + CalcBaseDouble.ConvertToString(ResultCoefficientS[i]) 
						+ "*" + Variant;
				else
					TempStr = TempStr + CalcBaseDouble.ConvertToString(ResultCoefficientS[i]);
			}
		}
	}

	return TempStr;
}


//���� �� ��������������ֻʶ��Ӽ�����������������˵���Ϊ��һ�����������Ұ����������� 2015.10.18
vector<string> CFormulaAnalytic::SeparationOperand(string FormulaValue)
{
	string TempStr = FormulaValue;
	//������ȥ��������
	if (TempStr.substr(0, 1) == "(" && TempStr.substr(TempStr.size() - 1, 1) == ")")
	{
		TempStr = TempStr.substr(1, TempStr.size() - 1);	
		TempStr = TempStr.substr(0, TempStr.size() - 1);
	}
	////�ѳ���һ��������� ����
	//if (FormulaValue.substr(0, 1) == "+")
	//	FormulaValue = FormulaValue.substr(1, FormulaValue.size() - 1);
	
	TempStr = TempStr + "+";

	//2016.04.15 Ϊ�����ݺ���
	//int Index = GetFirstSeparationInFormula(TempStr);

	int Index = FindFirstOperator(TempStr);

	vector<string> Result;

	while(Index != -1)
	{
		string LeftStr = TempStr.substr(0, Index);		
		Result.push_back(LeftStr);
		TempStr = TempStr.substr(Index, TempStr.size() - Index);
		if (TempStr == "+") 
			Index = -1;
		else
		//Index = GetFirstSeparationInFormula(TempStr);
		Index = FindFirstOperator(TempStr);
	}

	////ĩβ�����һ��
	//if (FormulaValue != "")
	//	Result.push_back(FormulaValue);

	return Result;
}

//�� FormulaValue �� ��ȡ��һ����������Ľ���λ��
int CFormulaAnalytic::GetFirstSeparationInFormula(string FormulaValue)
{
	//2016.04.15 ��д  Ϊ�����ݺ���

	//int Index = this->getfirst

	///*

	int Index = -1;
	int PositiveIndex = FormulaValue.find("+");
	int NegativeIndex = FormulaValue.find("-");
	
	if (PositiveIndex != -1 && NegativeIndex != -1)
	{
		if (PositiveIndex == 0 || NegativeIndex == 0)	//��һ�����Ƿ��ţ�Ҫ�ҵڶ���������Ϊ�������Ľ���λ��
		{
			FormulaValue = FormulaValue.substr(1, FormulaValue.size() -1); 
			return GetFirstSeparationInFormula(FormulaValue) + 1; //��Ϊǰ��ȥ��һ���ַ�������Ҫ��1
		}		
		else if (PositiveIndex < NegativeIndex)		
			return PositiveIndex;			
		else
			return NegativeIndex;			
	}
	else if (PositiveIndex != -1)	
		Index = PositiveIndex;		
	else if (NegativeIndex != -1)
		Index = NegativeIndex;
	else
		return -1;

	if (Index == 0)
	{
		string TempFormulaValue = FormulaValue.substr(1, FormulaValue.size() - 1);
		int TempIndex = GetFirstSeparationInFormula(TempFormulaValue);
		if (TempIndex != - 1) return TempIndex + 1;
		else return  -1;
	}
	else
		return Index;
		//*/
}

//��ȡ���еı����볣��
void CFormulaAnalytic::ExtractVariableAndConstant(string FormulaValue)
{

	//2016.04.15  ֻ����һ������

	//AddVariableAndConstant("u");

	int i = 0;
	while ( i < FormulaValue.size())
	{
		int j = i;
		string TempVariableAndConstant = "";
		while(j < FormulaValue.size())
		{
			if (((FormulaValue[j] > 'a' && FormulaValue[j] < 'z') || 
				(FormulaValue[j] > 'A' && FormulaValue[j] < 'Z')) && FormulaValue[j] != 'e')
			{
				TempVariableAndConstant = TempVariableAndConstant + FormulaValue[j];

				if( j == FormulaValue.size() - 1)
				{
					if (TempVariableAndConstant != "")
					{
						AddVariableAndConstant(TempVariableAndConstant);	
						i = j + 1;
						break;
					}
				}
			}
			else 
			{
				if (TempVariableAndConstant != "")
				{
					AddVariableAndConstant(TempVariableAndConstant);	
					TempVariableAndConstant = "";
					i = j;
					break;
				}				
			}

			j++;
			i = j;
		}
	}
	//*/
}

//�������������뵽 VariableAndConstant ���� ��
void CFormulaAnalytic::AddVariableAndConstant(string TempVariableAndConstant)
{
	//cout<<TempVariableAndConstant<<endl;
	bool Find = false;
	for(int i = 0; i < VariableAndConstant.size(); i++)
	{
		if (VariableAndConstant[i] == TempVariableAndConstant)
		{
			Find = true;
			break;
		}
	}

	if (!Find) VariableAndConstant.push_back(TempVariableAndConstant);
}

//�Ƿ������������ ������� ���� true
bool CFormulaAnalytic::IsHaveVariableAndConstant(string ExpressionValue)
{
	bool IsHave = false;
	
	for(int i = 0; i < VariableAndConstant.size(); i++)
	{
		int Index = ExpressionValue.find(VariableAndConstant[i]);
		if (Index >= 0)
		{
			IsHave = true;
			break;
		}
	}

	return IsHave;
}

//�ӵ���������ȡϵ�� ����� ����(����ʹ��) ����ֵϵ��
double CFormulaAnalytic::ExtractCoefficientAndOthers(string ExpressionValue, 
	vector<string> & VariableAndConstantS)
{
	if (ExpressionValue == "+") return 0;

	//�õ����еı����볣��
	string TempStr = ExpressionValue;
	VariableAndConstantS.clear();
	for(int i = 0; i < VariableAndConstant.size(); i++)
	{
		//int Index = ExpressionValue.find(VariableAndConstant[i] + "^"); 
		int Index = TempStr.find(VariableAndConstant[i]); 
		while (Index >= 0) // �ݲ� ���� �� ����ʽ 2015.12.15
		{			
			VariableAndConstantS.push_back(VariableAndConstant[i]);
			TempStr = TempStr.substr(Index + 1, TempStr.size() - Index - 1);
			Index = TempStr.find(VariableAndConstant[i]); 
		}		
	}

	for(int i = 0; i < VariableAndConstantS.size(); i++)
	{
		ExpressionValue = ReplaceStr(ExpressionValue, VariableAndConstantS[i], "");
	}
	ExpressionValue = ReplaceStr(ExpressionValue, "*", "");
	
	if (ExpressionValue == "" && VariableAndConstantS.size() > 0)	//˵���� "u"����ʽ���滻���û����
		ExpressionValue = "+";

	if (ExpressionValue == "+" || ExpressionValue == "-")
		ExpressionValue = ExpressionValue + "1";
	return atof(ExpressionValue.c_str());
}

//2015.12.15 �������ַ������浽��ջ��
void CFormulaAnalytic::FormulaStrToOperationItemS()
{
	OperationItemS.clear();
}


//Ѱ�����һ����������ڵ�λ�� 2016.04.15 
int CFormulaAnalytic::FindLastOperator(string Formula)
{
	if (Formula == "+")
	return -1;
	int PositiveIndex, NegativeIndex, Index, TempIndex;
	PositiveIndex = Formula.rfind("+");
	NegativeIndex = Formula.rfind("-");
	if (PositiveIndex == Formula.size()-1 || NegativeIndex == Formula.size()-1)
		return FindLastOperator(Formula.substr(1, Formula.size() - 1)) + 1;	
	else if (PositiveIndex == -1)
		Index = NegativeIndex;
	else if (NegativeIndex == -1)
		Index = PositiveIndex;
	else if (PositiveIndex < NegativeIndex)
		Index = NegativeIndex;
	else 
		Index = PositiveIndex;	

	if (Index > 0)
	{
		if (Formula.substr(Index-1, 1) == "e")	//˵���п�ѧ������
		{			
			string TempStr = Formula.substr(0, Index-1);
			return FindLastOperator(TempStr);
		}
		else
			return Index;
	}
	else
		return Index;	
}


	//Ѱ�ҵ�һ����������ڵ�λ��
int CFormulaAnalytic::FindFirstOperator(string Formula)
{
	if (Formula == "+")
		return -1;

	int PositiveIndex, NegativeIndex, Index, TempIndex;
	PositiveIndex = Formula.find("+");
	NegativeIndex = Formula.find("-");
	if (PositiveIndex == 0 || NegativeIndex == 0)
		return FindFirstOperator(Formula.substr(1, Formula.size() - 1)) + 1;	
	else if (PositiveIndex == -1)
		Index = NegativeIndex;
	else if (NegativeIndex == -1)
		Index = PositiveIndex;
	else if (PositiveIndex > NegativeIndex)
		Index = NegativeIndex;
	else 
		Index = PositiveIndex;	

	if (Formula.substr(Index-1, 1) == "e")	//˵���п�ѧ������
	{
		string TempStr = Formula.substr(Index+1, Formula.size() - Index-1);
		return FindFirstOperator(TempStr) + Index + 1;
	}
	else
		return Index;
}

//��ȡ Formula ���෴���ʽ������Ӧ���ڼ�������
string CFormulaAnalytic::OppositeFormula(string Formula)
{
	if (Formula.substr(0,1) != "+" || Formula.substr(0,1) != "-")
		Formula = "+" + Formula;
	Formula = ReplaceStr(Formula, "e+", "EA");
	Formula = ReplaceStr(Formula, "e-", "EZ");
	Formula = ReplaceStr(Formula, "+", "B");
	Formula = ReplaceStr(Formula, "-", "+");
	Formula = ReplaceStr(Formula, "B", "-");
	Formula = ReplaceStr(Formula, "EZ", "e-");
	Formula = ReplaceStr(Formula, "EA", "e+");

	return Formula;
}

//���ַ������ʽ�Ļ��ֱ��ʽ
string CFormulaAnalytic::IntegralStr(string Formula)
{
	string ResultStr;
	//cout<<"Formula = "<<Formula<<endl;
	if (Formula == "" || Formula == "+") return "";
	double Value;

	if (Formula.substr(0,1) == "+")
	{
		Formula = Formula.substr(1, Formula.size() -1);
	}

	if (Formula.substr(Formula.size() - 1, 1) != "+")
		Formula = Formula + "+";	

	int Index = FindFirstOperator(Formula);
	//�ָ�ÿһ����ʽ
	string LeftStr = Formula.substr(0, Index);
	string RightStr = Formula.substr(Index, Formula.size() - Index);

	double Coefficient;
	CalcBase<double> CalcBasedouble;

	vector<string> LeftVariableAndConstant;
	Coefficient = ExtractCoefficientAndOthers(LeftStr, LeftVariableAndConstant);
	
	if (Coefficient != 0)
	{		
		Coefficient = Coefficient / (LeftVariableAndConstant.size() + 1);
		LeftStr = CalcBasedouble.ConvertToString(Coefficient) 
				+ "*" + StringBase::GetRepeatedStr("u", LeftVariableAndConstant.size() + 1, "*");
	}	
	
	Coefficient = ExtractCoefficientAndOthers(RightStr, LeftVariableAndConstant);

	if (Coefficient != 0)
	{
		RightStr = IntegralStr(RightStr);	//�ݹ�ʵ��

		if (RightStr.substr(0,1) != "+" && RightStr.substr(0,1) != "-" &&  RightStr != "")
			Formula = LeftStr + "+" +RightStr;
		else if (RightStr != "")
			Formula = LeftStr + RightStr;		
	}
	else
		Formula = LeftStr;

	return Formula;
}

//����ʽ�Ķ�����ֵ�� Formula����δ��������ı��ʽ
double CFormulaAnalytic::ResolveIntegralValue(
	string Formula, double UStartValue, double UEndValue)
{
	string IntegralStrValue = IntegralStr(Formula);

	return ResolveValue(IntegralStrValue, UEndValue) - ResolveValue(IntegralStrValue, UStartValue);
}

//������ʽ�е�ϵ������� 2015.12.22
void CFormulaAnalytic::ExtractCofficientAndVairantOrder(string Formula, vector<double> & Cofficients, 
	vector<int> & Order)
{
	if (Formula == "") return ;
	double Value;
	AddVariableAndConstant("u");
	if (Formula.substr(0,1) == "+")
	{
		Formula = Formula.substr(1, Formula.size() -1);
	}

	if (Formula.substr(Formula.size() - 1, 1) != "+")
		Formula = Formula + "+";	

	int Index = FindFirstOperator(Formula);

	string LeftStr = Formula.substr(0, Index);
	string RightStr = Formula.substr(Index, Formula.size() - Index);

	double Coefficient;
	CalcBase<double> CalcBasedouble;

	vector<string> LeftVariableAndConstant;
	Coefficient = ExtractCoefficientAndOthers(LeftStr, LeftVariableAndConstant);

	Cofficients.push_back(Coefficient);
	Order.push_back(LeftVariableAndConstant.size());

	if (RightStr != "" && RightStr != "+")
	{
		//vector<double> RightCofficients;
		//vector<int> RightOrder;
		ExtractCofficientAndVairantOrder(RightStr, Cofficients, Order);
		//for(int i = )
	}
}

//���ַ������ʽ�� Order �׵���
string CFormulaAnalytic::DerivativeStr(string Formula)
{
	if (Formula == "") return "";
	ExtractVariableAndConstant(Formula);
	double Value;

	if (Formula.substr(0,1) == "+")
	{
		Formula = Formula.substr(1, Formula.size() -1);
	}

	if (Formula.substr(Formula.size() - 1, 1) != "+")
		Formula = Formula + "+";	

	int Index = FindFirstOperator(Formula);

	string LeftStr = Formula.substr(0, Index);
	string RightStr = Formula.substr(Index, Formula.size() - Index);

	double Coefficient;
	CalcBase<double> CalcBasedouble;

	vector<string> LeftVariableAndConstant;
	Coefficient = ExtractCoefficientAndOthers(LeftStr, LeftVariableAndConstant);

	Coefficient = Coefficient*(LeftVariableAndConstant.size());

	if (Coefficient != 0 && LeftVariableAndConstant.size() > 0)
	{		
		if (LeftVariableAndConstant.size() > 1 )
		{
			LeftStr = CalcBasedouble.ConvertToString(Coefficient) 
				+ "*" + StringBase::GetRepeatedStr("u", LeftVariableAndConstant.size() - 1, "*");
		}
		else
		{
			LeftStr = CalcBasedouble.ConvertToString(Coefficient);	
		}
	}	
	if (LeftVariableAndConstant.size() == 0)
		return "";

	Coefficient = ExtractCoefficientAndOthers(RightStr, LeftVariableAndConstant);

	if (LeftVariableAndConstant.size() > 0 && RightStr != "+")
	{
		RightStr = DerivativeStr(RightStr);		//�ݹ�ʵ��

		if (RightStr.substr(0,1) != "+" && RightStr.substr(0,1) != "-" &&  RightStr != "")
			Formula = LeftStr + "+" +RightStr;
		else if (RightStr != "")
			Formula = LeftStr + RightStr;		
	}
	else
		Formula = LeftStr;
	return Formula;
}

string CFormulaAnalytic::DerivativeStr(string Formula, int n)
{
	for(int i = 0; i < n; i++)
	{
		Formula = DerivativeStr(Formula);
	}
	return Formula;
}

//���ݴ���ı��ʽ�����ʽ��ֵ
double CFormulaAnalytic::ResolveValue(string Formula, vector<string> Variant, vector<double> VariantValue)
{
	return 0;	
}

//��Ϊ�˴�ֻ�� u һ�����������Լ򻯴���
double CFormulaAnalytic::ResolveValue(string Formula, double VariantValue)
{
	if (Formula == "")
	{
		cout<<"��Ҫ����ı��ʽΪ�գ�"<<endl;
		return 0;
	}
	CalcBase<double> CalcBaseDouble;
	Formula = StringBase::ReplaceStr(Formula, "u", CalcBaseDouble.ConvertToString(VariantValue));
	if (Formula.substr(0,1) == "-")	//��Ե�һ���������������0��������д���
		Formula = "0" + Formula;
	return Arithmetic::expressionCalculate(Formula);
}
