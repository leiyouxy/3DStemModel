#include "TreeBase.h"
#include "../TreePclQtGui.h"

CTreeBase::CTreeBase()
{
	ZMax = 0, ZMin = 0, YMax = 0, YMin = 0, XMax = 0, XMin = 0;
	
	connect(this, SIGNAL(PointIndex(int)), this, SLOT(GetPointIndex(int)), Qt::UniqueConnection);
	
	p_TreePclQtGui = NULL;
	Viewer = NULL;
	InputCloud = NULL;
	RedoCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	PointSize = 2;
	ClassName = "";
}

CTreeBase::~CTreeBase()
{
	//emitUpdateUI();
}

void CTreeBase::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TreePointsValue)
{
	InputCloud = TreePointsValue;
	RedoCloud->points.clear();
	RedoCloud->points.insert(RedoCloud->points.end(), InputCloud->points.begin(), InputCloud->points.end());
	PointBase::GetPointsMaxAndMin(InputCloud, XMax, XMin, YMax, YMin, ZMax, ZMin);
	cout<<"ZMax-ZMin:"<< ZMax - ZMin << "; YMax-YMin:" << YMax - YMin << "; XMax-XMin:" << XMax - XMin << endl;
}

void CTreeBase::Redo()
{
	InputCloud->points.clear();
	InputCloud->points.insert(InputCloud->points.end(), RedoCloud->points.begin(), RedoCloud->points.end());
	PointBase::GetPointsMaxAndMin(InputCloud, XMax, XMin, YMax, YMin, ZMax, ZMin);
	emitUpdateStatusBar("Redo has been done ",3000);
	emitUpdateUI();
}

void CTreeBase::SaveToRedo()
{
	RedoCloud->points.clear();
	RedoCloud->points.insert(RedoCloud->points.end(), InputCloud->points.begin(), InputCloud->points.end());
}

void CTreeBase::SetViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerValue)
{
	Viewer = ViewerValue;
}

void CTreeBase::RefreshData()
{	
	PointBase::GetPointsMaxAndMin(InputCloud, XMax, XMin, YMax, YMin, ZMax, ZMin);
}

void CTreeBase::emitUpdateUI()
{
	if (p_TreePclQtGui == NULL)
		return;
	
	emit p_TreePclQtGui->UpdateUI();
}

void CTreeBase::emitResetCamera()
{
	if (p_TreePclQtGui == NULL)
		return;

	emit p_TreePclQtGui->UpdateResetCamera();
}

void CTreeBase::emitShowMsg(string Msg)
{
	if (p_TreePclQtGui == NULL)
		return;

	emit p_TreePclQtGui->ShowMsg(Msg);
}


void CTreeBase::emitUpdateStatusBar(QString StrValue, int TimeOut)
{
	if (p_TreePclQtGui == NULL)
		return;

	emit p_TreePclQtGui->UpdateStatus(tr(StrValue.toStdString().c_str()), TimeOut);
	emit p_TreePclQtGui->UpdateUI();
}
void CTreeBase::emitUpdateAppTitle(string FileName)
{
	if (p_TreePclQtGui == NULL)
		return;

	emit p_TreePclQtGui->UpdateAppTitle(FileName);
}

void CTreeBase::GetPointIndex(int IndexValue)
{
	SelectedPointIndex = IndexValue;
}

void CTreeBase::ShowPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, string PointsStr, int PointSize,
	bool ShowIndex, double IndexScale)
{
	if (Points->points.size() == 0) return;
	if (Viewer != NULL)
	{
		Viewer->removePointCloud(PointsStr);
		Viewer->addPointCloud(Points, PointsStr);
		Viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, PointsStr);	

		if (ShowIndex)
		{
			CalcBase<int> CalcBaseInt;
			for(int i = 0; i < Points->points.size(); i++)
			{
				Viewer->addText3D(CalcBaseInt.ConvertToString(i + 1), Points->points[i],
					IndexScale, 255, 0, 0,
					StringBase::ClockValue() + CalcBaseInt.ConvertToString(i + 1));
			}
		}

		emitUpdateUI();
	}
}

void CTreeBase::ShowNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, 
	pcl::PointCloud<pcl::Normal>::Ptr Normals, string PointsStr, int Level, float Scale)
{
	if (Points->points.size() == 0) return;
	if (Normals->points.size() == 0) return;

	if (Viewer != NULL)
	{		
		Viewer->removePointCloud(PointsStr);
		Viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(Points, 
			Normals, Level, Scale, PointsStr);
		
		emitUpdateUI();
	}	
}

void CTreeBase::ShowPrincipalCurvatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	pcl::PointCloud<pcl::Normal>::Ptr Normals,
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr PrincipalCurvatures, string PointsStr, int Level, float Scale)
{
	if (Points->points.size() == 0) return;
	if (Normals->points.size() == 0) return;
	if (PrincipalCurvatures->points.size() == 0) return;

	if (Viewer != NULL)
	{
		Viewer->removePointCloud(PointsStr);
		Viewer->addPointCloudPrincipalCurvatures<pcl::PointXYZRGB, pcl::Normal>
			(Points, Normals, PrincipalCurvatures, Level * 100, Scale * 100, PointsStr);
		emitUpdateUI();
	}
}