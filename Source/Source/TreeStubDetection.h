#ifndef TreeStub_H
#define TreeStub_H

#include "TreeBase.h"

#include "HorizontalPartition.h"
#include "CommVector.h"
#include "GeneratedFiles/ui_StemStubRemoval.h"

using namespace std;

class CTreeStubDetection : public CTreeBase
{
	Q_OBJECT
private:
	Ui::StemStubRemovalForm StemStubRemovalForm;

	//������ĵ��Ʒ�������
	CHorizontalPartition HorizontalPartition;
	int SearchStartIndex;

	//
	int StemStartIndex;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints;
	const string PlanePointsStr = "PlanePoints";
	void GetParameters();

	int StubIndex;
public:	

	//���ݲ���̽������λ��
	int GetStemStartIndex(int StartSectionIndex, double AllowError);

signals:

public Q_SLOTS:	
	void ShowSlicesPoints(int CheckValue);

	void CheckStub();

	void Redo();

	void StubRemoval();

	//2021.01.30 �����Ƴ���׮��λ�ĵ���
	void StubRemovalUnderGivenHeight(); 

	void StubRemovalUnderGivenHeightBat();

	void ShowHeightPlane(int Value);

	void CircleCuttingCheck();

	void CircleCuttingRedo();

	void CircleCutting();

	void CircleCuttingBat();
public:	
	CTreeStubDetection(QGroupBox * ParentWin);
	~CTreeStubDetection();

	void RefreshData();
	void SetViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue); 
};

#endif


	////̽�ⷥ��λ��
	//CTreeStubDetection TreeStubDetection;
	//TreeStubDetection.SetInputClouds(HorizontalPartition.SectionsVector, 
	//	HorizontalPartition.MassCenterPointsPtr);
	//TreeStubDetection.GetTreeStub(
	//	HorizontalPartition.FindSectionIndex(StubDetectionPosition),
	//	StubAllowError);
	//TreeStubDetection.GetStubAndStemPoint(UnderStubPtr,AboveStubPtr);
	////����λ�����