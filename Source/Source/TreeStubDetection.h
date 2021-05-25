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

	//分区后的点云分区容器
	CHorizontalPartition HorizontalPartition;
	int SearchStartIndex;

	//
	int StemStartIndex;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanePoints;
	const string PlanePointsStr = "PlanePoints";
	void GetParameters();

	int StubIndex;
public:	

	//根据参数探测树根位置
	int GetStemStartIndex(int StartSectionIndex, double AllowError);

signals:

public Q_SLOTS:	
	void ShowSlicesPoints(int CheckValue);

	void CheckStub();

	void Redo();

	void StubRemoval();

	//2021.01.30 批量移除树桩部位的点云
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


	////探测伐根位置
	//CTreeStubDetection TreeStubDetection;
	//TreeStubDetection.SetInputClouds(HorizontalPartition.SectionsVector, 
	//	HorizontalPartition.MassCenterPointsPtr);
	//TreeStubDetection.GetTreeStub(
	//	HorizontalPartition.FindSectionIndex(StubDetectionPosition),
	//	StubAllowError);
	//TreeStubDetection.GetStubAndStemPoint(UnderStubPtr,AboveStubPtr);
	////伐根位置完毕