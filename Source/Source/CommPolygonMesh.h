#pragma once
#ifndef CommPolygonMesh_H
#define CommPolygonMesh_H

#include "Commdefinitions.h"

//多边形表面的类
static class PolygonMeshBase
{
public:
	static void AddEdge(vector<EdgeCount> & CurrentEdgeS, int PointOne, int PointTwo)
	{
		int Index = FindEdge(CurrentEdgeS, PointOne, PointTwo);

		if (Index == -1)
		{
			EdgeCount Temp;
			Temp.IndexOne = PointOne;
			Temp.IndexTwo = PointTwo;
			Temp.Count = 1;
			CurrentEdgeS.push_back(Temp);
		}
		else
		{
			CurrentEdgeS[Index].Count++;
		}
	}

	static int FindEdge(vector<EdgeCount> & CurrentEdgeS, int PointOne, int PointTwo)
	{
		int Index = -1;
		for (int i = 0; i < CurrentEdgeS.size(); i++)
		{
			EdgeCount Temp = CurrentEdgeS[i];

			if ((Temp.IndexOne == PointOne && Temp.IndexTwo == PointTwo) ||
				(Temp.IndexOne == PointTwo && Temp.IndexTwo == PointOne))
			{
				Index = i;
				break;
			}
		}
		return Index;
	}

	//删除列表中的边
	static void DeleteEdge(
		vector<EdgeCount> & CurrentEdgeS, int PointOne, int PointTwo)
	{
		int Index = FindEdge(CurrentEdgeS, PointOne, PointTwo);
		if (Index == -1)
			CurrentEdgeS.erase(CurrentEdgeS.begin() + Index);
	}

	static void HoleCheck(pcl::PolygonMesh Mesh, vector<EdgeCount> & HoleEdgeS,
		int StartIndex = 0)
	{
		HoleEdgeS.clear();
		vector<EdgeCount> EdgeS;

		for (int i = StartIndex; i < Mesh.polygons.size(); i++)
		{
			int PointA = Mesh.polygons[i].vertices[0];
			int PointB = Mesh.polygons[i].vertices[1];
			int PointC = Mesh.polygons[i].vertices[2];

			AddEdge(EdgeS, PointA, PointB);
			AddEdge(EdgeS, PointB, PointC);
			AddEdge(EdgeS, PointC, PointA);
		}

		for (int j = 0; j < EdgeS.size(); j++)
		{
			if (EdgeS[j].Count == 1)
			{
				AddEdge(HoleEdgeS, EdgeS[j].IndexOne, EdgeS[j].IndexTwo);
			}
		}
	}

	//搜索空洞中的下一个节点
	static int GetNextHoleIndex(vector<EdgeCount> & HoleEdgeS, int EdgeIndex)
	{
		int ReturnIndex = -1;
		int j = 0;
		while (j < HoleEdgeS.size())
		{
			if (HoleEdgeS[j].IndexOne == EdgeIndex)
			{
				ReturnIndex = HoleEdgeS[j].IndexTwo;
				HoleEdgeS[j].Count = HoleEdgeS[j].Count - 1;
				if (HoleEdgeS[j].Count == 0)
					HoleEdgeS.erase(HoleEdgeS.begin() + j);
				break;
			}
			else if (HoleEdgeS[j].IndexTwo == EdgeIndex)
			{
				ReturnIndex = HoleEdgeS[j].IndexOne;
				HoleEdgeS[j].Count = HoleEdgeS[j].Count - 1;
				if (HoleEdgeS[j].Count == 0)
					HoleEdgeS.erase(HoleEdgeS.begin() + j);
				break;
			}
			else
				j++;
		}
		return ReturnIndex;
	}

	static void RemoveRepeatedPolygon(pcl::PolygonMesh & SectionMesh, int StartIndex = 0)
	{
		for (int i = StartIndex; i < SectionMesh.polygons.size(); i++)
		{
			int j = i + 1;

			if (i >= SectionMesh.polygons.size()) break;

			while (i < SectionMesh.polygons.size() && j < SectionMesh.polygons.size())
			{
				if ((SectionMesh.polygons[i].vertices[0] == SectionMesh.polygons[j].vertices[0]
					|| SectionMesh.polygons[i].vertices[0] == SectionMesh.polygons[j].vertices[1]
					|| SectionMesh.polygons[i].vertices[0] == SectionMesh.polygons[j].vertices[2])
					&& (SectionMesh.polygons[i].vertices[1] == SectionMesh.polygons[j].vertices[0]
						|| SectionMesh.polygons[i].vertices[1] == SectionMesh.polygons[j].vertices[1]
						|| SectionMesh.polygons[i].vertices[1] == SectionMesh.polygons[j].vertices[2])
					&& (SectionMesh.polygons[i].vertices[2] == SectionMesh.polygons[j].vertices[0]
						|| SectionMesh.polygons[i].vertices[2] == SectionMesh.polygons[j].vertices[1]
						|| SectionMesh.polygons[i].vertices[2] == SectionMesh.polygons[j].vertices[2]))
				{
					SectionMesh.polygons.erase(SectionMesh.polygons.begin() + j);
				}
				else
					j++;
			}
		}
	}

};

#endif