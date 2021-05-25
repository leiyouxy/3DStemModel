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

		//2020.01.04 Crust生成的表面模型是多边形表面模型，而且存在边的方向不一致的情况，
		//在转换为三角网模型时仍需额外处理

		//2020.01.05 需要根据 TetGen 以决定是否需要做法向一致性判断 和 三维平面 三角网的生成 2020.01.05
		//三维平面 三角网 的正逆向 可以根据法向方向来判定，但依然存在着很多问题，需要解决 

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
			//Crust 原始多边形Surface存在着半边结构中 边的次序不吻合的现象 2020.01.04
			//Mesh.polygons.push_back(TempVertices);

			///*
			if (TempVertices.vertices.size() == 3)
			{
				OutputMesh.polygons.push_back(TempVertices);

				p_OutputCellArray->InsertNextCell(TempList);
			}
			else if (TempVertices.vertices.size() > 3)
			{				
				/* //直接更加多边形的边 添加三角形，但不能确保是Delaunay三角形
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
				//直接更加多边形的边 添加三角形，但不能确保是Delaunay三角形
				//*/

				///*  将三维点集转向二维点集，然后计算Delaunay三角网
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
					cout<<"三维点集转二维点集失败！"<<endl;
					for (int i = 1; i < TempVertices.vertices.size() - 1; i++)
					{
						pcl::Vertices TempVertices0;

						TempVertices0.vertices.push_back(TempVertices.vertices[0]);
						TempVertices0.vertices.push_back(TempVertices.vertices[i]);
						TempVertices0.vertices.push_back(TempVertices.vertices[i + 1]);
						OutputMesh.polygons.push_back(TempVertices0);

						//向VTK 添加结果数据
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

				//点的精度太高，导致有些点非常接近，不需要计算凸包
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
						//原点集是逆时针排列点集
						//if (NewHullIndexs[2] > NewHullIndexs[1])
						{
							TempEdge.StartIndex = i;
							TempEdge.EndIndex = (i + 1) % Indexs.size();
						}
						//else //原点集是顺时针排列点集
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

					//正向数据符合要求
					if (ShouldMeshCount == ForwardTempMesh.polygons.size())
					{
						for (int ii = 0; ii < ForwardTempMesh.polygons.size(); ii++)
						{
							pcl::Vertices TempVertices0;

							TempVertices0.vertices.push_back(Indexs[ForwardTempMesh.polygons[ii].vertices[0]]);
							TempVertices0.vertices.push_back(Indexs[ForwardTempMesh.polygons[ii].vertices[1]]);
							TempVertices0.vertices.push_back(Indexs[ForwardTempMesh.polygons[ii].vertices[2]]);
							OutputMesh.polygons.push_back(TempVertices0);

							//向VTK 添加结果数据
							vtkIdList * TempResultList = vtkIdList::New();
							TempResultList->InsertNextId(Indexs[ForwardTempMesh.polygons[ii].vertices[0]]);
							TempResultList->InsertNextId(Indexs[ForwardTempMesh.polygons[ii].vertices[1]]);
							TempResultList->InsertNextId(Indexs[ForwardTempMesh.polygons[ii].vertices[2]]);
							p_OutputCellArray->InsertNextCell(TempResultList);
						}
					}
					else if (ShouldMeshCount != ForwardTempMesh.polygons.size())
					{
						//正向没有，逆向找下
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

								//向VTK 添加结果数据
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

						//	//向VTK 添加结果数据
						//	vtkIdList * TempResultList = vtkIdList::New();
						//	TempResultList->InsertNextId(TempVertices.vertices[0]);
						//	TempResultList->InsertNextId(TempVertices.vertices[i]);
						//	TempResultList->InsertNextId(TempVertices.vertices[i + 1]);
						//	p_OutputCellArray->InsertNextCell(TempResultList);
						//}
						//PointBase::SavePCDToFileName(TempCloud, "I:\\ply\\3DPoints.pcd");
						//PointBase::SavePCDToFileName(Temp2DCloud, "I:\\ply\\2DPoints.pcd");	
						cout << "正向逆向都没有生成合适的三角网，只好以默认规则前进" << endl;

						PointBase::SavePCDToFileName(TempCloud, "I:\\ply\\3DPoints.pcd");
						PointBase::SavePCDToFileName(Temp2DCloud, "I:\\ply\\2DPoints.pcd");
						for (int i = 1; i < TempVertices.vertices.size() - 1; i++)
						{
							pcl::Vertices TempVertices0;

							TempVertices0.vertices.push_back(TempVertices.vertices[0]);
							TempVertices0.vertices.push_back(TempVertices.vertices[i]);
							TempVertices0.vertices.push_back(TempVertices.vertices[i + 1]);
							OutputMesh.polygons.push_back(TempVertices0);

							//向VTK 添加结果数据
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
					//	cout << "应该添加三角形的数量与原数量不相等！" << endl;
					//	for (int i = 1; i < TempVertices.vertices.size() - 1; i++)
					//	{
					//		pcl::Vertices TempVertices0;

					//		TempVertices0.vertices.push_back(TempVertices.vertices[0]);
					//		TempVertices0.vertices.push_back(TempVertices.vertices[i]);
					//		TempVertices0.vertices.push_back(TempVertices.vertices[i + 1]);
					//		OutputMesh.polygons.push_back(TempVertices0);

					//		//向VTK 添加结果数据
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

					//	//向VTK 添加结果数据
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
				cout << "存在多边形的边数小于3！" << endl;
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

	//未闭合的三角形所在的索引
	static vector<int> UnClosedHalfEdgeInTrianlges(const pcl::PolygonMesh & TempMesh)
	{
		V_HE_Edge UnCloseEdges;
		vector<int> FaceIndexs;
		
		for (int i = 0; i < TempMesh.polygons.size(); i++)
		{
			int StartIndex1 = TempMesh.polygons[i].vertices[0];
			int EndIndex1 = TempMesh.polygons[i].vertices[1];

			//对边是否在未匹配列表中，如果在就删除，否则添加
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
			cout << "未闭合的边的面：" << FaceIndexs[i] << endl;
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

