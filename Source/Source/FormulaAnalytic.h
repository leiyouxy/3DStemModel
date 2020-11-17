//////公式解析 将字符串包含的多项式公式化简，合并，并按照次数降序排列
/////2015.10.18 leiyou
/////
/////****************************************************************
#ifndef FormulaAnalytic_H
#define FormulaAnalytic_H

#include <iostream>
#include <string>
#include <vector>
#include "CommClass.h"

using namespace std;

class CFormulaAnalytic
{
private:
	//输出的结果值
	string Formula;

	//变量与常量列表
	vector<string> VariableAndConstant;

	//删除公式 FormulaValue 中多余的括号 RepeatedNumber 表示括号重复的次数
	string RemoveRepeatedBracket(string FormulaValue, int RepeatedNumber = 10);

	//将字符串中 OldStr 替换为 NewStr 2015.10.18
	string ReplaceStr(string FormulaValue, string OldStr, string NewStr);

	//化简公式，对公式中能乘积的多项式进行多项式乘积 并化简
	string PolynomialProduct(string FormulaValue);

	//对包括括号的左右项进行乘积运行 2015.10.18 如 (x-a)*(x-b) 公式中无空格
	string PolynomialProduct(string LeftPart, string RightPart);

	//化简公式 多项式与数值乘积
	string PolynomialNumberProduct(string FormulaValue);

	string PolynomialNumberProduct(double Value, string FormulaValue);
		
	//移除公式中的括号，
	string RemoveBracket(string FormulaValue);

	//分离 中 包括的运算数，只识别加减运算的运算数，连乘的认为是一个运算数，且包括正负符号 2015.10.18
	vector<string> SeparationOperand(string FormulaValue);

	//在 FormulaValue 中 获取第一个运算的数的结束位置
	int GetFirstSeparationInFormula(string FormulaValue);

	//提取其中的变量与常量
	void ExtractVariableAndConstant(string FormulaValue);

	//将变量或常量加入到 VariableAndConstant 容器 中
	void AddVariableAndConstant(string TempVariableAndConstant);

	//是否包含变量或常量 如果包含 返回 true
	bool IsHaveVariableAndConstant(string ExpressionValue);

	//从单变量中提取系数 与变量 或函数(将来使用) 返回值系数
	double ExtractCoefficientAndOthers(string ExpressionValue, 
		vector<string> & VariableAndConstantS);

	//2015.12.15 修改此算法，值得注意的是，一个算法完成后做另外一个算法，不要半途而废，浪费时间与精力
	//以堆栈的形式保存的运算数
	vector<string> OperationItemS;

	//2015.12.15 将运算字符串保存到堆栈中
	void FormulaStrToOperationItemS();

	//寻找第一个运算符所在的位置
	int FindFirstOperator(string Formula);

	//寻找最后一个运算符所在的位置 2016.04.15 
	int FindLastOperator(string Formula);
public:
	//输入设置
	void SetInputs(string FormulaValue);

	//输出
	string GetFormula();

	//2015.12.15 多项式合并，将相同次数的多项式相加 此处只考虑一个参数 u 的情况，如果多个参数 需要调整
	string PolynomialMerge(string FormulaValue);

	//获取 Formula 的相反表达式，方便应用于减法运算 还未经过测试，不知道是否正确
	string OppositeFormula(string Formula);

	//求字符串表达式的 1 阶导数
	string DerivativeStr(string Formula);
	//求字符串表达式的 n 阶导数
	string DerivativeStr(string Formula, int n);
	
	//分离表达式中的系数与变量 2015.12.22
	void ExtractCofficientAndVairantOrder(string Formula, vector<double> & Cofficients, 
		vector<int> & Order);

	//求字符串表达式的积分
	string IntegralStr(string Formula);

	//根据传入的表达式求解表达式的值
	double ResolveValue(string Formula, vector<string> Variant, vector<double> VariantValue);

	//因为此处只有 u 一个参数，所以简化处理
	double ResolveValue(string Formula, double VariantValue);

	//求表达式的定积分值， Formula是尚未积分运算的表达式
	double ResolveIntegralValue(string Formula, double UStartValue, double UEndValue);
};

#endif


	////float UValue = 0.3;

	////pcl::PointXYZRGB Point = RationalSpline.GetSplinePoint(UValue);
	////pcl::PointXYZRGB DerivativePoint  = RationalSpline.GetSplineDerivativePoint(UValue, 1, false);

	////vector<string> FormulaS = RationalSpline.GetUFormula(UValue);

	////string FormulaStrX = FormulaS[0];
	////string FormulaStrY = FormulaS[1];
	////string FormulaStrZ = FormulaS[2];
	////
	////string FormulaStr, DerivativeStr;
	////CFormulaAnalytic FormulaAnalytic;

	////FormulaAnalytic.SetInputs(FormulaStrX);	
	////FormulaStr = FormulaAnalytic.GetFormula();
	////cout<<"X轴的表达式:"<<FormulaStr<<endl;
	////cout<<"X值:"<<FormulaAnalytic.ResolveValue(FormulaStr, UValue)<<endl;
	////DerivativeStr = FormulaAnalytic.DerivativeStr(FormulaStr);
	////cout<<"X轴的一阶导表达式:"<<DerivativeStr<<endl;
	////cout<<endl<<"解析式的一阶导X值:"<<FormulaAnalytic.ResolveValue(DerivativeStr,UValue)<<endl;

	////FormulaAnalytic.SetInputs(FormulaStrY);	
	////FormulaStr = FormulaAnalytic.GetFormula();
	////cout<<"Y轴的表达式:"<<FormulaStr<<endl;
	////cout<<"Y值:"<<FormulaAnalytic.ResolveValue(FormulaStr, UValue)<<endl;
	////DerivativeStr = FormulaAnalytic.DerivativeStr(FormulaStr);
	////cout<<"Y轴的一阶导表达式:"<<DerivativeStr<<endl;
	////cout<<endl<<"解析式的一阶导Y值:"<<FormulaAnalytic.ResolveValue(DerivativeStr,UValue)<<endl;
	////
	//////
	////FormulaAnalytic.SetInputs(FormulaStrZ);	
	////FormulaStr = FormulaAnalytic.GetFormula();
	////cout<<"Z轴的表达式:"<<FormulaStr<<endl;
	////cout<<"Z值:"<<FormulaAnalytic.ResolveValue(FormulaStr, UValue)<<endl;
	////DerivativeStr = FormulaAnalytic.DerivativeStr(FormulaStr);
	////cout<<"Z轴的一阶导表达式:"<<DerivativeStr<<endl;
	////cout<<endl<<"解析式的一阶导Z值:"<<FormulaAnalytic.ResolveValue(DerivativeStr,UValue)<<endl;

	////cout<<"点的坐标:"<<Point<<endl;
	////cout<<"一阶导数的坐标:"<<DerivativePoint<<endl;	