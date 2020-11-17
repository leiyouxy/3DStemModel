//////公式解析 将字符串包含的多项式公式化简，合并，并按照次数降序排列
/////yl 2015.10.18
/////****************************************************************

#include "FormulaAnalytic.h"

//输入设置
void CFormulaAnalytic::SetInputs(string FormulaValue)
{
	Formula="";
	//首先去掉重复的括号
	for(int i = 10; i > 1; i--)
	{
		FormulaValue = RemoveRepeatedBracket(FormulaValue, i);
	}	
	//去掉公式中的空格 //2016.04.16
	Formula = ReplaceStr(FormulaValue, " ", "");	//
}

//输出
string CFormulaAnalytic::GetFormula()
{
	ExtractVariableAndConstant(Formula);
	//cout<<"需要解析的公式为："<<Formula<<endl;
	//FormulaStrToOperationItemS();
	//ExtractVariableAndConstant("+1.5*u-0.3*u+2U");
	Formula = PolynomialProduct(Formula);

	return Formula;
}

//删除公式 FormulaValue 中多余的括号 RepeatedNumber 表示括号重复的次数
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
	//没有找到 或者 找到的在最后的尾部，
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

	if (LastIndexOfDoubleBracket + 1 == LastIndexOfSingleBracket)	//说明有连续的"(("
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

//将字符串中 OldStr 替换为 NewStr 2015.10.18
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

//化简公式，对公式中能乘积的多项式进行多项式乘积 并化简
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
		//左右括号都不是连续的 即不是 ))*((的情况 或之一
		if (Left.substr(Left.size() - 2, 1) != ")" && Right.substr(1, 1) != "(")	
		{
			string ResultLeft, ResultRight;
			int LeftStartIndex = Left.rfind("(");
			string LeftOperationStr = Left.substr(LeftStartIndex, Left.size() - LeftStartIndex);
			ResultLeft = Left.substr(0, LeftStartIndex);

			int RightEndIndex = Right.find(")");
			string RightOperationStr = Right.substr(0, RightEndIndex + 1);
			ResultRight = Right.substr(RightEndIndex + 1,  Right.size() - RightEndIndex);
			
			//此处需要根据前后括号情况 决定是否加括号

			//左侧运算符括号前的一个字符
			string LeftChar = "", RightChar = "";
			if (ResultLeft.size() > 0)
				LeftChar = ResultLeft.substr(ResultLeft.size() - 1, 1);
			if (ResultRight.size() > 0)
				RightChar = ResultRight.substr(0, 1);
			
			string TempStr = PolynomialProduct(LeftOperationStr, RightOperationStr);

			//决定计算结果是否需要加括号
			if (LeftChar=="*" || RightChar== "*" || LeftChar=="/" || RightChar== "/" )
				FormulaValue = ResultLeft + "(" + TempStr 
						+ ")" + ResultRight;
			else
			{	if (LeftChar=="+") //去掉前面多余的+或-号
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

			//cout<<"第"<<i++<<"次完成的部分解析的公式："<<FormulaValue<<endl;			
			FormulaValue = RemoveRepeatedBracket(FormulaValue,2);
			ProductIndex = FormulaValue.find(")*(");
		}
		else
			ProductIndex = -1;
	}
	//cout<<endl<<"解析公式:"<<FormulaValue<<endl<<endl;
	return PolynomialNumberProduct(FormulaValue);
}

//化简公式 多项式与数值乘积 最后才执行这一步，
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

		////此处需要处理科学计算e的表示
		//int LeftStartPositiveIndex = LeftStr.rfind("+");
		//int LeftStartNegativeIndex = LeftStr.rfind("-");

		////都没有，说明第一个就是运算数
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
	

		//2016.04.15更新
		int LeftStartPositiveIndex = FindLastOperator(LeftStr);

		if( LeftStr!= "" && LeftStartPositiveIndex == -1)
			LeftStartPositiveIndex = 0;

		LeftOperationStr = LeftStr.substr(LeftStartPositiveIndex, LeftStr.size() - LeftStartPositiveIndex);
		//2016.04.15更新
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

//移除公式中的括号，
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

//对包括括号的左右项进行乘积运行 2015.10.18  如 (x-a)*(x-b) 公式中无空格
string CFormulaAnalytic::PolynomialProduct(string LeftPart, string RightPart)
{
	//先将多项式合并
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

		//循环对右侧的运算符运算
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
	
	//此处最好做多项式合并
	return PolynomialMerge(ResultStr);
}

//2015.12.15 多项式合并，将相同次数的多项式相加 此处只考虑一个参数 u 的情况，如果多个参数 需要调整
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
		CoefficientS.push_back(Coefficient); //系数
		OrderS.push_back(VariableAndConstant.size()); //阶数
	}

	vector<double> ResultCoefficientS;
	vector<int> ResultOrderS;

	for(int i = 0; i < OperandStr.size(); i++)
	{		
		bool Find = false;
		//需要在结果中搜寻 是否已经存在，若存在，继续累计，不存在则新增
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

	//对最终结果按照阶次降序排序

	for(int i = 0; i < ResultOrderS.size(); i++)
	{
		for(int j = i + 1; j < ResultOrderS.size(); j++)
		{
			if (ResultOrderS[i] < ResultOrderS[j])	//前面的比后面的小
			{
				swap(ResultOrderS[i], ResultOrderS[j]);
				swap(ResultCoefficientS[i], ResultCoefficientS[j]);
			}
		}
	}

	//输出
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
			if (ResultCoefficientS[i] > 0) //考虑到符号问题
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


//分离 中 包括的运算数，只识别加减运算的运算数，连乘的认为是一个运算数，且包括正负符号 2015.10.18
vector<string> CFormulaAnalytic::SeparationOperand(string FormulaValue)
{
	string TempStr = FormulaValue;
	//首先脱去左右括号
	if (TempStr.substr(0, 1) == "(" && TempStr.substr(TempStr.size() - 1, 1) == ")")
	{
		TempStr = TempStr.substr(1, TempStr.size() - 1);	
		TempStr = TempStr.substr(0, TempStr.size() - 1);
	}
	////脱出第一个运算符的 正号
	//if (FormulaValue.substr(0, 1) == "+")
	//	FormulaValue = FormulaValue.substr(1, FormulaValue.size() - 1);
	
	TempStr = TempStr + "+";

	//2016.04.15 为处理幂函数
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

	////末尾的最后一个
	//if (FormulaValue != "")
	//	Result.push_back(FormulaValue);

	return Result;
}

//在 FormulaValue 中 获取第一个运算的数的结束位置
int CFormulaAnalytic::GetFirstSeparationInFormula(string FormulaValue)
{
	//2016.04.15 改写  为处理幂函数

	//int Index = this->getfirst

	///*

	int Index = -1;
	int PositiveIndex = FormulaValue.find("+");
	int NegativeIndex = FormulaValue.find("-");
	
	if (PositiveIndex != -1 && NegativeIndex != -1)
	{
		if (PositiveIndex == 0 || NegativeIndex == 0)	//第一个就是符号，要找第二个符号作为运算数的结束位置
		{
			FormulaValue = FormulaValue.substr(1, FormulaValue.size() -1); 
			return GetFirstSeparationInFormula(FormulaValue) + 1; //因为前面去掉一个字符，所以要加1
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

//提取其中的变量与常量
void CFormulaAnalytic::ExtractVariableAndConstant(string FormulaValue)
{

	//2016.04.15  只包含一个参数

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

//将变量或常量加入到 VariableAndConstant 容器 中
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

//是否包含变量或常量 如果包含 返回 true
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

//从单变量中提取系数 与变量 或函数(将来使用) 返回值系数
double CFormulaAnalytic::ExtractCoefficientAndOthers(string ExpressionValue, 
	vector<string> & VariableAndConstantS)
{
	if (ExpressionValue == "+") return 0;

	//得到所有的变量与常量
	string TempStr = ExpressionValue;
	VariableAndConstantS.clear();
	for(int i = 0; i < VariableAndConstant.size(); i++)
	{
		//int Index = ExpressionValue.find(VariableAndConstant[i] + "^"); 
		int Index = TempStr.find(VariableAndConstant[i]); 
		while (Index >= 0) // 暂不 考虑 幂 的形式 2015.12.15
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
	
	if (ExpressionValue == "" && VariableAndConstantS.size() > 0)	//说明是 "u"的形式，替换后就没有了
		ExpressionValue = "+";

	if (ExpressionValue == "+" || ExpressionValue == "-")
		ExpressionValue = ExpressionValue + "1";
	return atof(ExpressionValue.c_str());
}

//2015.12.15 将运算字符串保存到堆栈中
void CFormulaAnalytic::FormulaStrToOperationItemS()
{
	OperationItemS.clear();
}


//寻找最后一个运算符所在的位置 2016.04.15 
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
		if (Formula.substr(Index-1, 1) == "e")	//说明有科学计数法
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


	//寻找第一个运算符所在的位置
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

	if (Formula.substr(Index-1, 1) == "e")	//说明有科学计数法
	{
		string TempStr = Formula.substr(Index+1, Formula.size() - Index-1);
		return FindFirstOperator(TempStr) + Index + 1;
	}
	else
		return Index;
}

//获取 Formula 的相反表达式，方便应用于减法运算
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

//求字符串表达式的积分表达式
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
	//分割每一块表达式
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
		RightStr = IntegralStr(RightStr);	//递归实现

		if (RightStr.substr(0,1) != "+" && RightStr.substr(0,1) != "-" &&  RightStr != "")
			Formula = LeftStr + "+" +RightStr;
		else if (RightStr != "")
			Formula = LeftStr + RightStr;		
	}
	else
		Formula = LeftStr;

	return Formula;
}

//求表达式的定积分值， Formula是尚未积分运算的表达式
double CFormulaAnalytic::ResolveIntegralValue(
	string Formula, double UStartValue, double UEndValue)
{
	string IntegralStrValue = IntegralStr(Formula);

	return ResolveValue(IntegralStrValue, UEndValue) - ResolveValue(IntegralStrValue, UStartValue);
}

//分离表达式中的系数与变量 2015.12.22
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

//求字符串表达式的 Order 阶导数
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
		RightStr = DerivativeStr(RightStr);		//递归实现

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

//根据传入的表达式求解表达式的值
double CFormulaAnalytic::ResolveValue(string Formula, vector<string> Variant, vector<double> VariantValue)
{
	return 0;	
}

//因为此处只有 u 一个参数，所以简化处理
double CFormulaAnalytic::ResolveValue(string Formula, double VariantValue)
{
	if (Formula == "")
	{
		cout<<"需要计算的表达式为空！"<<endl;
		return 0;
	}
	CalcBase<double> CalcBaseDouble;
	Formula = StringBase::ReplaceStr(Formula, "u", CalcBaseDouble.ConvertToString(VariantValue));
	if (Formula.substr(0,1) == "-")	//针对第一个运算数必须大于0情况的折中处理
		Formula = "0" + Formula;
	return Arithmetic::expressionCalculate(Formula);
}
