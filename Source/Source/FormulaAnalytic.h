//////��ʽ���� ���ַ��������Ķ���ʽ��ʽ���򣬺ϲ��������մ�����������
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
	//����Ľ��ֵ
	string Formula;

	//�����볣���б�
	vector<string> VariableAndConstant;

	//ɾ����ʽ FormulaValue �ж�������� RepeatedNumber ��ʾ�����ظ��Ĵ���
	string RemoveRepeatedBracket(string FormulaValue, int RepeatedNumber = 10);

	//���ַ����� OldStr �滻Ϊ NewStr 2015.10.18
	string ReplaceStr(string FormulaValue, string OldStr, string NewStr);

	//����ʽ���Թ�ʽ���ܳ˻��Ķ���ʽ���ж���ʽ�˻� ������
	string PolynomialProduct(string FormulaValue);

	//�԰������ŵ���������г˻����� 2015.10.18 �� (x-a)*(x-b) ��ʽ���޿ո�
	string PolynomialProduct(string LeftPart, string RightPart);

	//����ʽ ����ʽ����ֵ�˻�
	string PolynomialNumberProduct(string FormulaValue);

	string PolynomialNumberProduct(double Value, string FormulaValue);
		
	//�Ƴ���ʽ�е����ţ�
	string RemoveBracket(string FormulaValue);

	//���� �� ��������������ֻʶ��Ӽ�����������������˵���Ϊ��һ�����������Ұ����������� 2015.10.18
	vector<string> SeparationOperand(string FormulaValue);

	//�� FormulaValue �� ��ȡ��һ����������Ľ���λ��
	int GetFirstSeparationInFormula(string FormulaValue);

	//��ȡ���еı����볣��
	void ExtractVariableAndConstant(string FormulaValue);

	//�������������뵽 VariableAndConstant ���� ��
	void AddVariableAndConstant(string TempVariableAndConstant);

	//�Ƿ������������ ������� ���� true
	bool IsHaveVariableAndConstant(string ExpressionValue);

	//�ӵ���������ȡϵ�� ����� ����(����ʹ��) ����ֵϵ��
	double ExtractCoefficientAndOthers(string ExpressionValue, 
		vector<string> & VariableAndConstantS);

	//2015.12.15 �޸Ĵ��㷨��ֵ��ע����ǣ�һ���㷨��ɺ�������һ���㷨����Ҫ��;���ϣ��˷�ʱ���뾫��
	//�Զ�ջ����ʽ�����������
	vector<string> OperationItemS;

	//2015.12.15 �������ַ������浽��ջ��
	void FormulaStrToOperationItemS();

	//Ѱ�ҵ�һ����������ڵ�λ��
	int FindFirstOperator(string Formula);

	//Ѱ�����һ����������ڵ�λ�� 2016.04.15 
	int FindLastOperator(string Formula);
public:
	//��������
	void SetInputs(string FormulaValue);

	//���
	string GetFormula();

	//2015.12.15 ����ʽ�ϲ�������ͬ�����Ķ���ʽ��� �˴�ֻ����һ������ u ����������������� ��Ҫ����
	string PolynomialMerge(string FormulaValue);

	//��ȡ Formula ���෴���ʽ������Ӧ���ڼ������� ��δ�������ԣ���֪���Ƿ���ȷ
	string OppositeFormula(string Formula);

	//���ַ������ʽ�� 1 �׵���
	string DerivativeStr(string Formula);
	//���ַ������ʽ�� n �׵���
	string DerivativeStr(string Formula, int n);
	
	//������ʽ�е�ϵ������� 2015.12.22
	void ExtractCofficientAndVairantOrder(string Formula, vector<double> & Cofficients, 
		vector<int> & Order);

	//���ַ������ʽ�Ļ���
	string IntegralStr(string Formula);

	//���ݴ���ı��ʽ�����ʽ��ֵ
	double ResolveValue(string Formula, vector<string> Variant, vector<double> VariantValue);

	//��Ϊ�˴�ֻ�� u һ�����������Լ򻯴���
	double ResolveValue(string Formula, double VariantValue);

	//����ʽ�Ķ�����ֵ�� Formula����δ��������ı��ʽ
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
	////cout<<"X��ı��ʽ:"<<FormulaStr<<endl;
	////cout<<"Xֵ:"<<FormulaAnalytic.ResolveValue(FormulaStr, UValue)<<endl;
	////DerivativeStr = FormulaAnalytic.DerivativeStr(FormulaStr);
	////cout<<"X���һ�׵����ʽ:"<<DerivativeStr<<endl;
	////cout<<endl<<"����ʽ��һ�׵�Xֵ:"<<FormulaAnalytic.ResolveValue(DerivativeStr,UValue)<<endl;

	////FormulaAnalytic.SetInputs(FormulaStrY);	
	////FormulaStr = FormulaAnalytic.GetFormula();
	////cout<<"Y��ı��ʽ:"<<FormulaStr<<endl;
	////cout<<"Yֵ:"<<FormulaAnalytic.ResolveValue(FormulaStr, UValue)<<endl;
	////DerivativeStr = FormulaAnalytic.DerivativeStr(FormulaStr);
	////cout<<"Y���һ�׵����ʽ:"<<DerivativeStr<<endl;
	////cout<<endl<<"����ʽ��һ�׵�Yֵ:"<<FormulaAnalytic.ResolveValue(DerivativeStr,UValue)<<endl;
	////
	//////
	////FormulaAnalytic.SetInputs(FormulaStrZ);	
	////FormulaStr = FormulaAnalytic.GetFormula();
	////cout<<"Z��ı��ʽ:"<<FormulaStr<<endl;
	////cout<<"Zֵ:"<<FormulaAnalytic.ResolveValue(FormulaStr, UValue)<<endl;
	////DerivativeStr = FormulaAnalytic.DerivativeStr(FormulaStr);
	////cout<<"Z���һ�׵����ʽ:"<<DerivativeStr<<endl;
	////cout<<endl<<"����ʽ��һ�׵�Zֵ:"<<FormulaAnalytic.ResolveValue(DerivativeStr,UValue)<<endl;

	////cout<<"�������:"<<Point<<endl;
	////cout<<"һ�׵���������:"<<DerivativePoint<<endl;	