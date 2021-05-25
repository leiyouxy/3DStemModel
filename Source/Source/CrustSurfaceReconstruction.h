#pragma once

#include "vtkPLYReader.h"
#include "vtkPLYWriter.h"
#include "vtkSmartPointer.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include "CommPointBase.h"

#include "vtkPowerCrustSurfaceReconstruction.h"
#include "DelaunayGrowthWithHalfEdge.h"

class CCrustSurfaceReconstruction
{

public:
	static void PeformSurfaceReconstruction(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr InputCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutputCloud,
		pcl::PolygonMesh & OutputMesh,
		bool IsTriangle = false)
	{
		vtkSmartPointer<vtkPowerCrustSurfaceReconstruction> SurfaceReconstructor =
			vtkSmartPointer<vtkPowerCrustSurfaceReconstruction>::New();

		vtkPoints * p_Points = vtkPoints::New();

		for (int i = 0; i < InputCloud->points.size(); i++)
		{
			float Point[3];
			Point[0] = InputCloud->points[i].x;
			Point[1] = InputCloud->points[i].y;
			Point[2] = InputCloud->points[i].z;

			p_Points->InsertNextPoint(Point);
		}

		vtkPolyData * p_PolyData = vtkPolyData::New();
		p_PolyData->SetPoints(p_Points);

		SurfaceReconstructor->SetInputData(p_PolyData);
		SurfaceReconstructor->Update();
		p_Points->Delete();

		//2020.01.04 Crust���ɵı���ģ���Ƕ���α���ģ�ͣ����Ҵ��ڱߵķ���һ�µ������
		//��ת��Ϊ������ģ��ʱ������⴦��

		//2020.01.05 ��Ҫ���� TetGen �Ծ����Ƿ���Ҫ������һ�����ж� �� ��άƽ�� ������������ 2020.01.05
		//��άƽ�� ������ �������� ���Ը��ݷ��������ж�������Ȼ�����źܶ����⣬��Ҫ��� 

		vtkSmartPointer<vtkPLYWriter> writer = 	vtkSmartPointer<vtkPLYWriter>::New();
		writer->SetInputData(SurfaceReconstructor->GetOutput());
		writer->SetFileName("I:\\Ply\\CrustPolygonByVtk.ply");
		writer->SetFileTypeToASCII();
		writer->Write();

		vtkCellArray * p_CellArray = SurfaceReconstructor->GetOutput()->GetPolys();
		vtkCellArray * p_OutputCellArray = vtkCellArray::New();
		p_Points = SurfaceReconstructor->GetOutput()->GetPoints();

		OutputCloud->points.clear();
		OutputMesh.polygons.clear();

		for (int i = 0; i < p_Points->GetNumberOfPoints(); i++)
		{
			double Point[3];

			p_Points->GetPoint(i, Point);
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = Point[0];
			TempPoint.y = Point[1];
			TempPoint.z = Point[2];

			OutputCloud->points.push_back(TempPoint);
		}
		PointBase::SetPointColor(OutputCloud, ColorBase::StemColor);
		
		vtkIdList * TempList = vtkIdList::New();
		p_CellArray->InitTraversal();
		int k = 0;
		while (p_CellArray->GetNextCell(TempList))
		{
			pcl::Vertices TempVertices;
			for (auto i = 0; i < TempList->GetNumberOfIds(); ++i)
			{
				TempVertices.vertices.push_back(TempList->GetId(i));
			}
			//Crust ԭʼ�����Surface�����Ű�߽ṹ�� �ߵĴ����Ǻϵ����� 2020.01.04
			//Mesh.polygons.push_back(TempVertices);

			///*
			if (TempVertices.vertices.size() == 3)
			{
				OutputMesh.polygons.push_back(TempVertices);

				p_OutputCellArray->InsertNextCell(TempList);
			}
			else if (TempVertices.vertices.size() > 3)
			{				
				/* //ֱ�Ӹ��Ӷ���εı� ��������Σ�������ȷ����Delaunay������
				for (int i = 1; i < TempVertices.vertices.size() - 1; i++)
				{
					//Indexs.push_back(TempVertices.vertices[i]);
					//TempCloud->points.push_back(OutputCloud->points[i]);

					pcl::Vertices TempVertices0;

					TempVertices0.vertices.push_back(TempVertices.vertices[0]);
					TempVertices0.vertices.push_back(TempVertices.vertices[i]);
					TempVertices0.vertices.push_back(TempVertices.vertices[i+1]);
					Mesh.polygons.push_back(TempVertices0);
				}
				//ֱ�Ӹ��Ӷ���εı� ��������Σ�������ȷ����Delaunay������
				//*/

				///*  ����ά�㼯ת���ά�㼯��Ȼ�����Delaunay������
				vector<int> Indexs;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr Temp2DCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
				for (int i = 0; i < TempVertices.vertices.size(); i++)
				{
					Indexs.push_back(TempVertices.vertices[i]);
					TempCloud->points.push_back(OutputCloud->points[TempVertices.vertices[i]]);
				}
				int ShouldMeshCount = TempVertices.vertices.size() - 2;
				int Result = GeometryBase::Points3DTo2D(TempCloud, Temp2DCloud);	

				if (Result == 0)
				{
					cout<<"��ά�㼯ת��ά�㼯ʧ�ܣ�"<<endl;
					for (int i = 1; i < TempVertices.vertices.size() - 1; i++)
					{
						pcl::Vertices TempVertices0;

						TempVertices0.vertices.push_back(TempVertices.vertices[0]);
						TempVertices0.vertices.push_back(TempVertices.vertices[i]);
						TempVertices0.vertices.push_back(TempVertices.vertices[i + 1]);
						OutputMesh.polygons.push_back(TempVertices0);

						//��VTK ��ӽ������
						vtkIdList * TempResultList = vtkIdList::New();
						TempResultList->InsertNextId(TempVertices.vertices[0]);
						TempResultList->InsertNextId(TempVertices.vertices[i]);
						TempResultList->InsertNextId(TempVertices.vertices[i + 1]);
						p_OutputCellArray->InsertNextCell(TempResultList);

					}
					continue;
				}

				int TempMeshpolygonsCount = OutputMesh.polygons.size();
				//CContourAndConvexHull<pcl::PointXYZRGB> ConvexHull;
				//ConvexHull.SetInputs(Temp2DCloud);

				//vector<int> HullIndexs, NewHullIndexs;
				//ConvexHull.GetPointsConvexHull(HullIndexs, false);

				//��ľ���̫�ߣ�������Щ��ǳ��ӽ�������Ҫ����͹��
				//if (HullIndexs.size() < 3)
				//{
				//	for (int i = 1; i < TempVertices.vertices.size() - 1; i++)
				//	{
				//		pcl::Vertices TempVertices0;

				//		TempVertices0.vertices.push_back(TempVertices.vertices[0]);
				//		TempVertices0.vertices.push_back(TempVertices.vertices[i]);
				//		TempVertices0.vertices.push_back(TempVertices.vertices[i + 1]);
				//		Mesh.polygons.push_back(TempVertices0);
				//	}
				//	continue;
				//}	
				
		
				V_HE_Edge BorderEdges, ReverseBorderEdges;

					for (int i = 0; i < Indexs.size(); i++)
					{
						HE_Edge TempEdge, ReverseEdge;
						//ԭ�㼯����ʱ�����е㼯
						//if (NewHullIndexs[2] > NewHullIndexs[1])
						{
							TempEdge.StartIndex = i;
							TempEdge.EndIndex = (i + 1) % Indexs.size();
						}
						//else //ԭ�㼯��˳ʱ�����е㼯
						//{
						//	TempEdge.EndIndex = i;
						//	TempEdge.StartIndex = (i + 1) % Indexs.size();
						//}
						TempEdge.StartAngle = 360;
						TempEdge.EndAngle = 360;
						TempEdge.NextEdgeIndex = -1;
						TempEdge.OppositeEdgeIndex = -1;
						TempEdge.EdgeIndex = -1;
						BorderEdges.push_back(TempEdge);

						ReverseEdge = TempEdge;
						ReverseEdge.EndIndex = TempEdge.StartIndex;
						ReverseEdge.StartIndex = TempEdge.EndIndex;
						ReverseBorderEdges.push_back(ReverseEdge);
					}

					CDelaunayGrowthWithHalfEdge DelaunayGrowthWithHalfEdge;
					pcl::PolygonMesh ForwardTempMesh;
					pcl::PolygonMesh ReverseTempMesh;

					DelaunayGrowthWithHalfEdge.SetInputCloud(Temp2DCloud);
					DelaunayGrowthWithHalfEdge.SetOuterBorderEdges(BorderEdges);				
					DelaunayGrowthWithHalfEdge.performReconstructionWithPriority();
					DelaunayGrowthWithHalfEdge.OutputMesh(ForwardTempMesh);

					//PointBase::SaveMeshToPLY(Temp2DCloud, TempMesh, "I:\\Ply\\Temp2DCloud.ply");

					//�������ݷ���Ҫ��
					if (ShouldMeshCount == ForwardTempMesh.polygons.size())
					{
						for (int ii = 0; ii < ForwardTempMesh.polygons.size(); ii++)
						{
							pcl::Vertices TempVertices0;

							TempVertices0.vertices.push_back(Indexs[ForwardTempMesh.polygons[ii].vertices[0]]);
							TempVertices0.vertices.push_back(Indexs[ForwardTempMesh.polygons[ii].vertices[1]]);
							TempVertices0.vertices.push_back(Indexs[ForwardTempMesh.polygons[ii].vertices[2]]);
							OutputMesh.polygons.push_back(TempVertices0);

							//��VTK ��ӽ������
							vtkIdList * TempResultList = vtkIdList::New();
							TempResultList->InsertNextId(Indexs[ForwardTempMesh.polygons[ii].vertices[0]]);
							TempResultList->InsertNextId(Indexs[ForwardTempMesh.polygons[ii].vertices[1]]);
							TempResultList->InsertNextId(Indexs[ForwardTempMesh.polygons[ii].vertices[2]]);
							p_OutputCellArray->InsertNextCell(TempResultList);
						}
					}
					else if (ShouldMeshCount != ForwardTempMesh.polygons.size())
					{
						//����û�У���������
						CDelaunayGrowthWithHalfEdge ReverseDelaunayGrowthWithHalfEdge;
						ReverseDelaunayGrowthWithHalfEdge.SetInputCloud(Temp2DCloud);
						ReverseDelaunayGrowthWithHalfEdge.SetOuterBorderEdges(ReverseBorderEdges);
						ReverseDelaunayGrowthWithHalfEdge.performReconstructionWithPriority();
						
						ReverseDelaunayGrowthWithHalfEdge.OutputMesh(ReverseTempMesh);

						if (ShouldMeshCount == ReverseTempMesh.polygons.size())
						{
							for (int ii = 0; ii < ReverseTempMesh.polygons.size(); ii++)
							{
								pcl::Vertices TempVertices0;

								TempVertices0.vertices.push_back(Indexs[ReverseTempMesh.polygons[ii].vertices[2]]);								
								TempVertices0.vertices.push_back(Indexs[ReverseTempMesh.polygons[ii].vertices[1]]);
								TempVertices0.vertices.push_back(Indexs[ReverseTempMesh.polygons[ii].vertices[0]]);
								OutputMesh.polygons.push_back(TempVertices0);

								//��VTK ��ӽ������
								vtkIdList * TempResultList = vtkIdList::New();								
								TempResultList->InsertNextId(Indexs[ReverseTempMesh.polygons[ii].vertices[2]]);
								TempResultList->InsertNextId(Indexs[ReverseTempMesh.polygons[ii].vertices[1]]);
								TempResultList->InsertNextId(Indexs[ReverseTempMesh.polygons[ii].vertices[0]]);								
								p_OutputCellArray->InsertNextCell(TempResultList);

							}
							continue;
						}

						//for (int i = 1; i < TempVertices.vertices.size() - 1; i++)
						//{
						//	pcl::Vertices TempVertices0;

						//	TempVertices0.vertices.push_back(TempVertices.vertices[0]);
						//	TempVertices0.vertices.push_back(TempVertices.vertices[i]);
						//	TempVertices0.vertices.push_back(TempVertices.vertices[i + 1]);
						//	OutputMesh.polygons.push_back(TempVertices0);

						//	//��VTK ��ӽ������
						//	vtkIdList * TempResultList = vtkIdList::New();
						//	TempResultList->InsertNextId(TempVertices.vertices[0]);
						//	TempResultList->InsertNextId(TempVertices.vertices[i]);
						//	TempResultList->InsertNextId(TempVertices.vertices[i + 1]);
						//	p_OutputCellArray->InsertNextCell(TempResultList);
						//}
						//PointBase::SavePCDToFileName(TempCloud, "I:\\ply\\3DPoints.pcd");
						//PointBase::SavePCDToFileName(Temp2DCloud, "I:\\ply\\2DPoints.pcd");	
						cout << "��������û�����ɺ��ʵ���������ֻ����Ĭ�Ϲ���ǰ��" << endl;

						PointBase::SavePCDToFileName(TempCloud, "I:\\ply\\3DPoints.pcd");
						PointBase::SavePCDToFileName(Temp2DCloud, "I:\\ply\\2DPoints.pcd");
						for (int i = 1; i < TempVertices.vertices.size() - 1; i++)
						{
							pcl::Vertices TempVertices0;

							TempVertices0.vertices.push_back(TempVertices.vertices[0]);
							TempVertices0.vertices.push_back(TempVertices.vertices[i]);
							TempVertices0.vertices.push_back(TempVertices.vertices[i + 1]);
							OutputMesh.polygons.push_back(TempVertices0);

							//��VTK ��ӽ������
							vtkIdList * TempResultList = vtkIdList::New();
							TempResultList->InsertNextId(TempVertices.vertices[0]);
							TempResultList->InsertNextId(TempVertices.vertices[i]);
							TempResultList->InsertNextId(TempVertices.vertices[i + 1]);
							p_OutputCellArray->InsertNextCell(TempResultList);
						}
					}

					//if (ShouldMeshCount != ForwardTempMesh.polygons.size())
					//{
					//	PointBase::SavePCDToFileName(TempCloud, "I:\\ply\\3DPoints.pcd");
					//	PointBase::SavePCDToFileName(Temp2DCloud, "I:\\ply\\2DPoints.pcd");
					//	cout << "Ӧ����������ε�������ԭ��������ȣ�" << endl;
					//	for (int i = 1; i < TempVertices.vertices.size() - 1; i++)
					//	{
					//		pcl::Vertices TempVertices0;

					//		TempVertices0.vertices.push_back(TempVertices.vertices[0]);
					//		TempVertices0.vertices.push_back(TempVertices.vertices[i]);
					//		TempVertices0.vertices.push_back(TempVertices.vertices[i + 1]);
					//		OutputMesh.polygons.push_back(TempVertices0);

					//		//��VTK ��ӽ������
					//		vtkIdList * TempResultList = vtkIdList::New();
					//		TempResultList->InsertNextId(TempVertices.vertices[0]);
					//		TempResultList->InsertNextId(TempVertices.vertices[i]);
					//		TempResultList->InsertNextId(TempVertices.vertices[i + 1]);
					//		p_OutputCellArray->InsertNextCell(TempResultList);
					//	}
					//	continue;
					//}

					//for (int ii = 0; ii < TempMesh.polygons.size(); ii++)
					//{
					//	pcl::Vertices TempVertices0;

					//	TempVertices0.vertices.push_back(Indexs[TempMesh.polygons[ii].vertices[0]]);
					//	TempVertices0.vertices.push_back(Indexs[TempMesh.polygons[ii].vertices[1]]);
					//	TempVertices0.vertices.push_back(Indexs[TempMesh.polygons[ii].vertices[2]]);
					//	OutputMesh.polygons.push_back(TempVertices0);

					//	//��VTK ��ӽ������
					//	vtkIdList * TempResultList = vtkIdList::New();
					//	TempResultList->InsertNextId(Indexs[TempMesh.polygons[ii].vertices[0]]);
					//	TempResultList->InsertNextId(Indexs[TempMesh.polygons[ii].vertices[1]]);
					//	TempResultList->InsertNextId(Indexs[TempMesh.polygons[ii].vertices[2]]);
					//	p_OutputCellArray->InsertNextCell(TempResultList);
					//}
				//}
				//*/
	///*				
			}
			else
			{
				cout << "���ڶ���εı���С��3��" << endl;
			}
			//*/			
		}
				
		//UnClosedHalfEdgeInTrianlges(OutputMesh);

		p_PolyData->SetPoints(p_Points);
		p_PolyData->SetPolys(p_OutputCellArray);

		vtkSmartPointer<vtkPLYWriter> MeshPLYwriter = 	vtkSmartPointer<vtkPLYWriter>::New();
		MeshPLYwriter->SetInputData(p_PolyData);
		MeshPLYwriter->SetFileName("I:\\Ply\\CrustPolyDataByVtk.ply");
		MeshPLYwriter->SetFileTypeToASCII();
		MeshPLYwriter->Write();

		PointBase::SaveMeshToPLY(OutputCloud, OutputMesh, "I:\\Ply\\Test.ply");
		OutputCloud->width = OutputCloud->points.size();
		cout << "The number of points :" << OutputCloud->points.size() << endl;
		cout << "The number of polygons :" << OutputMesh.polygons.size() << endl;
	}

	//δ�պϵ����������ڵ�����
	static vector<int> UnClosedHalfEdgeInTrianlges(const pcl::PolygonMesh & TempMesh)
	{
		V_HE_Edge UnCloseEdges;
		vector<int> FaceIndexs;
		
		for (int i = 0; i < TempMesh.polygons.size(); i++)
		{
			int StartIndex1 = TempMesh.polygons[i].vertices[0];
			int EndIndex1 = TempMesh.polygons[i].vertices[1];

			//�Ա��Ƿ���δƥ���б��У�����ھ�ɾ�����������
			FindEdgeIsExists(UnCloseEdges, TempMesh.polygons[i].vertices[1], i, TempMesh.polygons[i].vertices[0]);
			FindEdgeIsExists(UnCloseEdges, TempMesh.polygons[i].vertices[2], i, TempMesh.polygons[i].vertices[1]);
			FindEdgeIsExists(UnCloseEdges, TempMesh.polygons[i].vertices[0], i, TempMesh.polygons[i].vertices[2]);
		}

		VectorBase<int> VectorBaseInt;
		for (int i = 0; i < UnCloseEdges.size(); i++)
		{
			if (UnCloseEdges[i].EdgeIndex == -1)
				continue;

			int TempIndex = VectorBaseInt.FindIndexInVector(FaceIndexs, UnCloseEdges[i].EdgeIndex);
			if (TempIndex == -1)
				FaceIndexs.push_back(UnCloseEdges[i].EdgeIndex);
		}

		for (int i = 0; i < FaceIndexs.size(); i++)
		{
			cout << "δ�պϵıߵ��棺" << FaceIndexs[i] << endl;
		}

		return FaceIndexs;
	}

	static void FindEdgeIsExists(V_HE_Edge & UnCloseEdges, int FaceIndex, int StartIndex, int EndIndex)
	{
		CDelaunayGrowthWithHalfEdge DelaunayGrowthWithHalfEdge;
		int Index1 = DelaunayGrowthWithHalfEdge.FindEdgeinEdges(UnCloseEdges, StartIndex, EndIndex);
		if (Index1 == -1)
		{
			HE_Edge TempEdge;
			TempEdge.StartIndex = StartIndex;
			TempEdge.EndIndex = EndIndex;
			TempEdge.EdgeIndex = FaceIndex;
			int FindIndex = DelaunayGrowthWithHalfEdge.FindEdgeinEdges(UnCloseEdges, -1, -1);
			if (FindIndex == -1)
				UnCloseEdges.push_back(TempEdge);
			else
			{
				UnCloseEdges[FindIndex].StartIndex = StartIndex;
				UnCloseEdges[FindIndex].EndIndex = EndIndex;
				UnCloseEdges[FindIndex].EdgeIndex = FaceIndex;
			}
		}
		else
		{
			//UnCloseEdges.erase(UnCloseEdges.begin() + Index1);
			UnCloseEdges[Index1].StartIndex = -1;
			UnCloseEdges[Index1].EndIndex = -1;
			UnCloseEdges[Index1].EdgeIndex = -1;
		}
	}
};

