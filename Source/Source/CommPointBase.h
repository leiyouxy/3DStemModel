#pragma once
#ifndef CommPointBase_H
#define CommPointBase_H

#include <tchar.h>
#include <sstream>
#include <fstream>
#include <strstream>
#include <string>
#include <iomanip>
#include <limits>
#include <math.h>
#include <iostream>
#include <vector>
#include <stack>
#include <map>
#include <io.h>
#include <ctime>
#include <stdio.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
//#include <sensor_msgs/LaserScan.h>

#include <pcl/console/parse.h>
#include <Eigen/src/Eigenvalues/EigenSolver.h>
#include <pcl/filters/project_inliers.h>
#include <boost/function.hpp>
//#include <liblas/liblas.hpp>

#include "Commdefinitions.h"
#include "CommClass.h" 
#include "CommVector.h"

//2019.01.30/
//This part and OpenLASFile, SaveToLAS should be commented
//if the pdal lib is not installed  2019.03.30
#include <pdal/io/LasReader.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>

template<typename T>
inline double PointDis(T x1, T y1, T z1, T x2, T y2, T z2, bool Is2D = false)
{
	if (Is2D)
		return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
	else
		return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

template<typename T>
inline double PointDis(T x1, T y1, T x2, T y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

template<typename PointNT>
inline double PointDis(PointNT PointA, PointNT PointB, bool Is2D = false)
{
	return PointDis(PointA.x, PointA.y, PointA.z, PointB.x, PointB.y, PointB.z, Is2D);
}

//return PointA - PointB
template<typename PointNT>
inline PointNT PointMinus(PointNT PointA, PointNT PointB)
{
	PointNT Result;
	
	Result.x = PointA.x - PointB.x, Result.y = PointA.y - PointB.y, Result.z = PointA.z - PointB.z;

	return Result;
}

//return PointA + PointB
template<typename PointNT>
inline PointNT PointAdd(PointNT PointA, PointNT PointB)
{
	PointNT Result;

	Result.x = PointA.x + PointB.x, Result.y = PointA.y + PointB.y, Result.z = PointA.z + PointB.z;

	return Result;
}

//////如果函数待实现模板类，尚未成行
//template<typename PointNT>
//inline void PointsMove(pcl::PointCloud<PointNT>::Ptr pPoints, double X, double Y, double Z)	//设置点云的坐标平移
//{
//	for (int i = 0; i < pPoints->points.size(); i++)
//	{
//		pPoints->points[i].x = pPoints->points[i].x + X;
//		pPoints->points[i].y = pPoints->points[i].y + Y;
//		pPoints->points[i].z = pPoints->points[i].z + Z;
//	}
//}

inline void PointsMove(pcl::PointCloud<PointXYZRGBIndex>::Ptr pPoints, double X, double Y, double Z)	//设置点云的坐标平移
{
	for (int i = 0; i < pPoints->points.size(); i++)
	{
		pPoints->points[i].x = pPoints->points[i].x + X;
		pPoints->points[i].y = pPoints->points[i].y + Y;
		pPoints->points[i].z = pPoints->points[i].z + Z;
	}
}

inline void PointsMove(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPoints, double X, double Y, double Z)	//设置点云的坐标平移
{
	cout << "点云位置发生移动！X："<<X<<", Y:"<<Y<<", Z:"<<Z << endl;
	for (int i = 0; i < pPoints->points.size(); i++)
	{
		pPoints->points[i].x = pPoints->points[i].x + X;
		pPoints->points[i].y = pPoints->points[i].y + Y;
		pPoints->points[i].z = pPoints->points[i].z + Z;
	}
}

inline void PointsMove(pcl::PointCloud<pcl::PointXYZ>::Ptr pPoints, double X, double Y, double Z)	//设置点云的坐标平移
{
	for (int i = 0; i < pPoints->points.size(); i++)
	{
		pPoints->points[i].x = pPoints->points[i].x + X;
		pPoints->points[i].y = pPoints->points[i].y + Y;
		pPoints->points[i].z = pPoints->points[i].z + Z;
	}
}

inline void PointsMove(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPoints, double X, double Y, double Z)	//设置点云的坐标平移
{
	for (int i = 0; i < pPoints->points.size(); i++)
	{
		pPoints->points[i].x = pPoints->points[i].x + X;
		pPoints->points[i].y = pPoints->points[i].y + Y;
		pPoints->points[i].z = pPoints->points[i].z + Z;
	}
}

inline void SaveStringToFile(string FileName, string SaveContentRow)
{
	ofstream write;
	write.open(FileName, ios::app);
	write << SaveContentRow << endl;
	write.close();
}

inline void ReadFileToStrings(string FileName, vector<string> & ReadStrings)
{
	ifstream Read;
	Read.open(FileName, ios::in);
	string Temps = "";
	while (getline(Read, Temps))	
		ReadStrings.push_back(Temps);	
	Read.close();
}

inline string GetDateTime()
{
	time_t now = time(0);
	tm *ltm = localtime(&now);
	string Value = StringBase::IntToStr(1900 + ltm->tm_year) + "-" + StringBase::IntToStr(1 + ltm->tm_mon);
	Value = Value + "-" + StringBase::IntToStr(ltm->tm_mday);
	Value = Value + " " + StringBase::IntToStr(ltm->tm_hour);
	Value = Value + ":" + StringBase::IntToStr(ltm->tm_min) + ":" + StringBase::IntToStr(ltm->tm_sec);
	return Value;
}

inline void GetFiles(string FilePath, vector<string> & FileNames)
{
	long   hFile = 0;
	struct _finddata_t fileinfo;	
	string p;

	//if (FilePath.substr(FilePath.length() - 1, 1).compare("\\") == 0)
	FilePath.pop_back();

	FilePath = FilePath.append("\\*");

	_finddata_t file;
	intptr_t lf = _findfirst(FilePath.c_str(), &file);
	//输入文件夹路径
	if (lf == -1) 
	{
		cout << FilePath << " not found!!!" << endl;
	}
	else
	{
		while (_findnext(lf, &file) == 0) {
			//输出文件名
			//cout<<file.name<<endl;
			if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
				continue;
			FileNames.push_back(file.name);
		}
	}

	//if ((hFile = _findfirst(p.assign(FilePath).append("\\*").c_str(), &fileinfo)) != -1)
	//{
	//	do
	//	{
	//		// sub directory	
	//		if ((fileinfo.attrib &  _A_SUBDIR))
	//		{
	//			//if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
	//			//	GetFiles(p.assign(FilePath).append("\\").append(fileinfo.name), FileNames);
	//		}
	//		else// File
	//		{
	//			FileNames.push_back(p.assign(FilePath).append("\\").append(fileinfo.name));
	//		}
	//	} while (_findnext(hFile, &fileinfo) == 0);
	//	_findclose(hFile);
	//}
}

class PointBase	//对点云一些基础设置的类
{
public:

	static void OpenPCLFile(std::string FileName,
		pcl::PointCloud<PointXYZRGBIndex>::Ptr cloud_ptr)
		//打开 FileName 文件中的点云
	{
		time_t c_start, c_end;
		c_start = clock();
		cloud_ptr->points.clear();
		//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
		//		TempPtr (new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
			TempPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

		std::cout << "正在读取点云文件，请稍候！\n";
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(FileName, *TempPtr) == -1)//*打开点云文件
		{
			PCL_ERROR("Couldn't read pcd file \n");
		}
		else
			std::cout << "读取点云文件成功！\n";
		PointMoveToOrigin(TempPtr);
		PointXYZRGBToPointXYZRGBIndex(TempPtr, cloud_ptr);
		c_end = clock();
		std::cout << "读取点云数据库完成，用时: " << difftime(c_end, c_start) << " ms" << endl;
	}

	static void SaveToXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string XYZName)
	{
		ofstream SaveFile;
		SaveFile.open(XYZName);
		
		for (int i = 0; i < cloud->points.size(); i++)
		{
			SaveFile << cloud->points[i].x << " " << cloud->points[i].y << " "
				<< cloud->points[i].z << " " << cloud->points[i].r << " "
				<< cloud->points[i].g << " " << cloud->points[i].b << endl;
		}
		SaveFile << flush; 
		SaveFile.close();
	}

	//2020.05.21 随机生成 Count 个数的点
	static void Generate2DPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud, int Count)
	{
		Cloud->points.clear();

		float Step = sqrt(Count);

		for (int i = 0; i < Step; i++)
		{
			//x 的基数
			float xValue = i * Step;
			for (int j = 0; j < Step; j++)
			{
				//y 的基数
				float yValue = j * Step;

				pcl::PointXYZRGB Temp;

				Temp.x = xValue + GetRandom(1, Count) * 1.0 / Step;
				Temp.y = yValue + GetRandom(1, Count) * 1.0 / Step;
				
				Temp.z = 0;

				Cloud->points.push_back(Temp);
			}
		}
	}

	static void SaveToPLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string PLYName)
	{
		pcl::PLYWriter Writer;
		Writer.write<pcl::PointXYZRGB>(PLYName, *cloud);
	}

	static void SaveMeshToPLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
		pcl::PolygonMesh Mesh, string PLYName)
	{
		ofstream SaveFile;
		SaveFile.open(PLYName);

		//File Header
		SaveFile << "ply\n";
		SaveFile << "format ascii 1.0\n";
		SaveFile << "comment VTK generated PLY File\n";
		SaveFile << "obj_info vtkPolyData points and polygons: vtk4.0\n" ;
		SaveFile << "element vertex "<< cloud->points.size() << "\n";
		SaveFile << "property float x\n";
		SaveFile << "property float y\n";
		SaveFile << "property float z\n";
		//SaveFile << "property float r" << endl;
		//SaveFile << "property float g" << endl;
		//SaveFile << "property float b" << endl;
		SaveFile << "element face " << Mesh.polygons.size()  << "\n";
		SaveFile << "property list uchar int vertex_indices\n";
		SaveFile << "end_header\n" ;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			//SaveFile << cloud->points[i].x << " " << cloud->points[i].y << " "
			//	<< cloud->points[i].z << " " << cloud->points[i].r << " "
			//	<< cloud->points[i].g << " " << cloud->points[i].b << endl;
			SaveFile << cloud->points[i].x << " " << cloud->points[i].y << " "
				<< cloud->points[i].z << " \n";
		}
		for (int i = 0; i < Mesh.polygons.size(); i++)
		{
			SaveFile << Mesh.polygons[i].vertices.size();
			for (int j = 0; j < Mesh.polygons[i].vertices.size(); j++)
			{
				SaveFile << " " << Mesh.polygons[i].vertices[j];
			}
			SaveFile << " \n" ;
		}
		//SaveFile << flush;
		SaveFile.close();
	}

	static void SaveToLASBYLibLas(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string LASName)
	{
		/*
		std::ofstream ofs(LASName.c_str(), ios::out | ios::binary);
		if (!ofs.is_open())
		{
			std::cout << "err  to  open  file  las....." << std::endl;
			return;
		}
		liblas::Header header;
		header.SetVersionMajor(1);
		header.SetVersionMinor(2);
		header.SetDataFormatId(liblas::ePointFormat3);
		header.SetScale(0.001, 0.001, 0.001);

		liblas::Writer writer(ofs, header);
		liblas::Point point(&header);

		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			double x = cloud->points[i].x;
			double y = cloud->points[i].y;
			double z = cloud->points[i].z;
			point.SetCoordinates(x, y, z);

			uint32_t red = (uint32_t)cloud->points[i].r;
			uint32_t green = (uint32_t)cloud->points[i].g;
			uint32_t blue = (uint32_t)cloud->points[i].b;
			liblas::Color pointColor(red, green, blue);
			point.SetColor(pointColor);

			writer.WritePoint(point);
			//std::cout << x << "," << y << "," << z << std::endl;
		}
		double minPt[3] = { 9999999, 9999999, 9999999 };
		double maxPt[3] = { 0, 0, 0 };
		header.SetPointRecordsCount(cloud->points.size());
		header.SetPointRecordsByReturnCount(0, cloud->points.size());
		header.SetMax(maxPt[0], maxPt[1], maxPt[2]);
		header.SetMin(minPt[0], minPt[1], minPt[2]);
		writer.SetHeader(header);
		//*/
	}

	static void SaveToLAS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string LASName)
	{			
		//File need be created first.
		ofstream Lasfile;
		Lasfile.open(LASName);
		Lasfile.close();

		pdal::Options options;
		options.add("filename", LASName);

		pdal::PointTable table;
		table.layout()->registerDim(pdal::Dimension::Id::X);
		table.layout()->registerDim(pdal::Dimension::Id::Y);
		table.layout()->registerDim(pdal::Dimension::Id::Z);
		//table.layout()->registerDim(pdal::Dimension::Id::Red);
		//table.layout()->registerDim(pdal::Dimension::Id::Green);
		//table.layout()->registerDim(pdal::Dimension::Id::Blue);

		pdal::PointViewPtr view(new pdal::PointView(table));
		
		for (int i = 0; i < cloud->points.size(); i++)
		{			
			view->setField(pdal::Dimension::Id::X, i, cloud->points[i].x);
			view->setField(pdal::Dimension::Id::Y, i, cloud->points[i].y);
			view->setField(pdal::Dimension::Id::Z, i, cloud->points[i].z);
			
			//view->setField(pdal::Dimension::Id::Red, i, cloud->points[i].r);
			//view->setField(pdal::Dimension::Id::Green, i, cloud->points[i].g);
			//view->setField(pdal::Dimension::Id::Blue, i, cloud->points[i].b);
		}

		pdal::BufferReader reader;
		reader.addView(view);

		pdal::StageFactory factory;		
		pdal::Stage *writer = factory.createStage(LASName);

		writer->setInput(reader);
		writer->setOptions(options);
		writer->prepare(table);
		writer->execute(table);
	}

	static void SaveToVTX(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string VTXName)
	{
		ofstream SaveFile;
		SaveFile.open(VTXName);

		SaveFile << "# Geomagic Studio" << endl;
		SaveFile << "# New Model" << endl;

		for (int i = 0; i < cloud->points.size(); i++)
		{
			double R, G, B;

			R = cloud->points[i].r / 255.0;
			G = cloud->points[i].g / 255.0;
			B = cloud->points[i].b / 255.0;

			SaveFile << cloud->points[i].x << " " << cloud->points[i].y << " "
				<< cloud->points[i].z << " " << R << " "
				<< G << " " << B << endl;
		}
		SaveFile << flush;
		SaveFile.close();
	}	

	template<typename PointT>
	static void PcdToPly(string PcdFile, string PlyName)
	{
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

		pcl::PCDReader reader;
		reader.read<PointT>(PcdFile, *cloud);
		pcl::PLYWriter plywriter;
		plywriter.write<PointT>(PlyName, *cloud);
	}

	template<typename PointT>
	static void PlyToPcd(string PcdFile, string PlyName)
	{
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

		pcl::PLYReader reader;
		reader.read<PointT>(PcdFile, *cloud);

		pcl::PCDWriter pcdwriter;
		pcdwriter.write<PointT>(PlyName, *cloud);
	}

	////2016.01.27 坐标互换
	//static void AxisExchange(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
	//	string Axis0, string Axis1)
	//{
	//	float Value;
	//	for (int i = 0; i < Points->points.size(); i++)
	//	{
	//		if (Axis0 == "x" && Axis1 == "y")
	//		{
	//			Value = Points->points[i].x;
	//			Points->points[i].x = Points->points[i].y;
	//			Points->points[i].y = Value;
	//		}
	//		else if (Axis0 == "x" && Axis1 == "z")
	//		{
	//			Value = Points->points[i].x;
	//			Points->points[i].x = Points->points[i].z;
	//			Points->points[i].z = Value;
	//		}
	//		else if (Axis0 == "y" && Axis1 == "z")
	//		{
	//			Value = Points->points[i].y;
	//			Points->points[i].y = Points->points[i].z;
	//			Points->points[i].z = Value;
	//		}
	//	}
	//}

	static void OpenLASFile(std::string FileName,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
	{
		ifstream in(FileName);

		if (!in)
		{
			cout <<"no such file" <<endl;
			return;
		}

		pdal::Option las_opt("filename", FileName);
		pdal::Options las_opts;
		las_opts.add(las_opt);
		pdal::PointTable table;
		pdal::LasReader las_reader;
		las_reader.setOptions(las_opts);
		las_reader.prepare(table);
		pdal::PointViewSet point_view_set = las_reader.execute(table);
		pdal::PointViewPtr point_view = *point_view_set.begin();
		pdal::Dimension::IdList dims = point_view->dims();
		pdal::LasHeader las_header = las_reader.header();
		bool HasColor = las_header.hasColor();
		if (HasColor)
			cout<<"has Color"<<endl;
		else
			cout << "No Color" << endl;
		//write point

		cloud_ptr->clear();
		for (pdal::PointId idx = 0; idx < point_view->size(); ++idx) 
		{
			pcl::PointXYZRGB thePt;  //int rgba = 255<<24 | ((int)r) << 16 | ((int)g) << 8 | ((int)b);

			thePt.x = point_view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
			thePt.y = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
			thePt.z = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);

			if (HasColor)
			{
				thePt.rgba = 255 << 24 | ((int)point_view->getFieldAs<int>(pdal::Dimension::Id::Red, idx)) << 16 
					| ((int)point_view->getFieldAs<int>(pdal::Dimension::Id::Green, idx)) << 8 
					| ((int)point_view->getFieldAs<int>(pdal::Dimension::Id::Blue, idx));

				//thePt.r = point_view->getFieldAs<int>(pdal::Dimension::Id::Red, idx);
				//thePt.g = point_view->getFieldAs<int>(pdal::Dimension::Id::Green, idx);
				//thePt.b = point_view->getFieldAs<int>(pdal::Dimension::Id::Blue, idx);	
				if (idx < 1000)
				//{
					cout << "thePt.rgba:" << thePt.rgba<< endl;
				//	cout << "thePt.g:" << point_view->getFieldAs<int>(pdal::Dimension::Id::Green, idx) << endl;
				//	cout << "thePt.b:" << point_view->getFieldAs<int>(pdal::Dimension::Id::Blue, idx) << endl;
				//}
			}						
			
			cloud_ptr->push_back(thePt);
		}

		if (!HasColor)
		{
			SetPointColorByHeight(cloud_ptr);
		}
		
		cout << "open Las file, and readying for Zoom 100 size!" << endl;
		PointBase::PointZoom(cloud_ptr, 100);
	}
	//*/
	//2018.10.22 读XYZRGB文件 
	static void OpenXYZRGBFile(std::string FileName,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
	{
		ifstream in(FileName);
		string filename;
		string line;

		if (cloud_ptr == NULL)
		{
			cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		}

		double MinDis = 1.0e+10;
		int MinDisPointIndex = -1;

		if (in) // 有该文件
		{
			while (getline(in, line)) // line中不包括每行的换行符
			{
				//cout << line << endl;

				string Str, LeftStr, RightStr;
				double Value = 0;
				int Index = -1;

				pcl::PointXYZRGB TempPoint;
				TempPoint.r = 128, TempPoint.g = 128, TempPoint.b = 128;
				int I = 0;

				Str = line + " ";
				while ((Index = Str.find_first_of(" ", 1)) != -1)
				{
					LeftStr = Str.substr(0, Index);
					RightStr = Str.substr(Index + 1, Str.length() - Index);
					Str = RightStr;

					//istringstream iss(LeftStr);
					//iss >> Value;
					Value = atof(LeftStr.c_str());
					if (abs(Value) < 1.0e-10)
					{
						Str = "";
						continue;
					}

					if (I == 0)
					{
						TempPoint.x = Value;
					}
					else if (I == 1)
					{
						TempPoint.y = Value;
					}
					else if (I == 2)
					{
						TempPoint.z = Value;
					}
					else if (I == 3)
					{
						TempPoint.r = Value;
					}
					else if (I == 4)
					{
						TempPoint.g = Value;
					}
					else if (I == 5)
					{
						TempPoint.b = Value;
					}

					I++;
				}
				if (I >= 3)
				{
					cloud_ptr->points.push_back(TempPoint);
				}
			}
		}
		else // 没有该文件
		{
			cout << "no such file" << endl;
		}
	}

	static void OpenPLYFile(std::string FileName,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
	{
		ifstream in(FileName);
		if (!in)
		{
			cout << "no such file" << endl;
			return;
		}

		if (cloud_ptr == NULL)
			cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);				
		pcl::PLYReader reader;
		reader.read<pcl::PointXYZRGB>(FileName, *cloud_ptr);	
	}

	//2018.10.22 读 VTX 文件 
	static void OpenVTXFile(std::string FileName,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
	{		
		ifstream in(FileName);	

		string filename;
		string line;

		if (cloud_ptr == NULL)		
			cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);		

		if (in) // 有该文件
		{
			//cout << "正在读取数据文件，请稍候！" << endl;
			while (getline(in, line)) // line中不包括每行的换行符
			{	
				string Str, LeftStr, RightStr;
				double Value = 0;
				int Index = -1;

				pcl::PointXYZRGB TempPoint;
				TempPoint.r = 128, TempPoint.g = 128, TempPoint.b = 128;
				int I = 0;

				Str = line + " ";
				//cout << "Cur Str: " << Str << endl;
				while ((Index = Str.find_first_of(" ", 1)) != -1)
				{
					LeftStr = Str.substr(0, Index);
					RightStr = Str.substr(Index + 1, Str.length() - Index);
					Str = RightStr;

					//istringstream iss(LeftStr);
					//iss >> Value;
					Value = atof(LeftStr.c_str());
					
					if (abs(Value) < 1.0e-10)
					{
						Str = "";
						continue;
					}

					if (I == 0)
					{
						TempPoint.x = Value;
					}
					else if (I == 1)
					{
						TempPoint.y = Value;
					}
					else if (I == 2)
					{
						TempPoint.z = Value;
					}
					else if (I == 3)
					{
						TempPoint.r = ceil(Value * 255);
					}
					else if (I == 4)
					{
						TempPoint.g = ceil(Value * 255);
					}
					else if (I == 5)
					{
						TempPoint.b = ceil(Value * 255);
					}

					I++;
				}

				//if (I >= 3)
				if (I >= 2) //xyz,无颜色数据时，2021，1，20
				{
					cloud_ptr->points.push_back(TempPoint);
				}
			}
		}
		else // 没有该文件
		{
			cout << "no such file" << endl;
		}
	}

	static void OpenPCLFile(std::string FileName,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, bool MoveToOriginal = false)
		//打开 FileName 文件中的点云
	{
		cout<<"正在打开点云文件："<< FileName <<endl;
		string ExtName = FileName.substr(FileName.length() - 3, 3);
		
		if (0 == ExtName.compare("vtx"))
		{
			PointBase::OpenVTXFile(FileName, cloud_ptr);
		}
		else if (0 == ExtName.compare("xyz"))
		{
			PointBase::OpenXYZRGBFile(FileName, cloud_ptr);
		}
		else if (0 == ExtName.compare("las"))
		{
			PointBase::OpenLASFile(FileName, cloud_ptr);
		}
		else if (0 == ExtName.compare("ply"))
		{
			PointBase::OpenPLYFile(FileName, cloud_ptr);
		}
		else if (0 == ExtName.compare("pcd"))
		{
			//time_t c_start, c_end;
			//c_start = clock();
			cloud_ptr->points.clear();
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr TempPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);			
			if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(FileName, *TempPtr) == -1)//*打开点云文件
				PCL_ERROR("Couldn't read Pcd file \n");
						
			PointXYZRGBAToPointXYZRGB(TempPtr, cloud_ptr);									
			//SetPointColorByHeight(cloud_ptr);
			//c_end = clock();

		}
		if (MoveToOriginal)
			PointMoveToOrigin(cloud_ptr);
	}

	static void PointZoom(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPoints, double Zoom)
	{
		cout << "Points Zoomed, Zoom:" << Zoom << endl;
		for (int i = 0; i < pPoints->points.size(); i++)
		{
			pPoints->points[i].x = pPoints->points[i].x * Zoom;
			pPoints->points[i].y = pPoints->points[i].y * Zoom;
			pPoints->points[i].z = pPoints->points[i].z * Zoom;
		}
	}

	static void SetPointColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPoints,
		int R, int G, int B)
	{	//设置点云的颜色
		for (int i = 0; i < pPoints->points.size(); i++)
		{
			pPoints->points[i].r = R;
			pPoints->points[i].g = G;
			pPoints->points[i].b = B;
		}
	}

	//
	static bool PointsIs2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPoints)
	{
		for (int i = 1; i < pPoints->points.size(); i++)
		{
			if (abs(pPoints->points[i].z - pPoints->points[i - 1].z) > EPSM3)
			{
				return false;
			}
		}

		return true;
	}

	static void SaveKnotValues(vector<double> KnotValues, string FileName)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr KnotValuesPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (int i = 0; i < KnotValues.size(); i++)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = KnotValues[i], TempPoint.y = 0, TempPoint.z = 0;
			KnotValuesPoints->points.push_back(TempPoint);
		}
		SavePCDToFileName(KnotValuesPoints, FileName);
	}

	static void SetPointColorByHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPoints)
	{	//设置点云的颜色
		double MaxX, MaxY, MaxZ, MinX, MinY, MinZ;
		PointBase::GetPointsMaxAndMin(pPoints, MaxX, MinX, MaxY, MinY, MaxZ, MinZ);
		
		double Height = abs(MaxZ - MinZ);

		for (int i = 0; i < pPoints->points.size(); i++)
		{
			int R = 255 * abs(pPoints->points[i].z - MinZ) / Height;
			int G = 128;// 255 * abs(pPoints->points[i].z - MinZ) / Height;
			int B = 250;// 255 * abs(pPoints->points[i].z - MinZ) / Height;
			pPoints->points[i].r = R;
			pPoints->points[i].g = G;
			pPoints->points[i].b = B;
		}
	}

	static void SetPointColor(pcl::PointCloud<PointXYZRGBIndex>::Ptr pPoints,
		int R, int G, int B)
	{	//设置点云的颜色
		for (int i = 0; i < pPoints->points.size(); i++)
		{
			pPoints->points[i].r = R;
			pPoints->points[i].g = G;
			pPoints->points[i].b = B;
		}
	}

	//将点云的某一个坐标轴的值设为同一值
	static void SetPointsCoordinateValue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		string Coordinate, double Value)
	{
		for (int i = 0; i < Cloud->points.size(); i++)
		{
			if (Coordinate == "X")
				Cloud->points[i].x = Value;
			else if (Coordinate == "Y")
				Cloud->points[i].y = Value;
			else if (Coordinate == "Z")
				Cloud->points[i].z = Value;
		}
	}

	static void GetPointsCenterXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		float & xCenter, float & yCenter, float & zCenter)
	{	//获取分区的中心点坐标 平均值中心点坐标 
		int PointsCount = Cloud->points.size();

		xCenter = 0;
		yCenter = 0;
		zCenter = 0;

		if (PointsCount > 0)	//如果当前分区的个数大于0，则执行此操作
		{
			for (int j = 0; j < PointsCount; j++)
			{
				xCenter = xCenter + Cloud->points[j].x / PointsCount;
				yCenter = yCenter + Cloud->points[j].y / PointsCount;
				zCenter = zCenter + Cloud->points[j].z / PointsCount;
			}
		}
	}

	static void GetPointsCenterXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		double & xCenter, double & yCenter, double & zCenter)
	{	//获取分区的中心点坐标 平均值中心点坐标 
		int PointsCount = Cloud->points.size();

		xCenter = 0;
		yCenter = 0;
		zCenter = 0;

		if (PointsCount > 0)	//如果当前分区的个数大于0，则执行此操作
		{
			for (int j = 0; j < PointsCount; j++)
			{
				xCenter = xCenter + Cloud->points[j].x / PointsCount;
				yCenter = yCenter + Cloud->points[j].y / PointsCount;
				zCenter = zCenter + Cloud->points[j].z / PointsCount;
			}
		}
	}

	static void GetPointsCenterXY2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		double & xCenter, double & yCenter)
	{	//获取分区的中心点坐标 平均值中心点坐标 二维平面
		int PointsCount = Cloud->points.size();

		xCenter = 0;
		yCenter = 0;

		if (PointsCount > 0)	//如果当前分区的个数大于0，则执行此操作
		{
			for (int j = 0; j < PointsCount; j++)
			{
				xCenter = xCenter + Cloud->points[j].x / PointsCount;
				yCenter = yCenter + Cloud->points[j].y / PointsCount;
			}
		}
	}


	static void GetRevisedCenterPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		double & xCenter, double & yCenter, double & zCenter, int IntervalNumber = 20)
	{	//获取修正的中心点坐标 ，因为 点云存在着疏密的情况，此种情况确保每个象限都有记录。		

		//xCenter = 0;
		//yCenter = 0;
		//zCenter = 0;

		//double XMax, XMin, YMax, YMin, ZMax, ZMin;		
		//	
		//GetPointsMaxAndMin(Cloud, XMax, XMin, YMax, YMin, ZMax,ZMin);

		//double TempZCenter = (ZMax + ZMin) / 2;
		//double TempYCenter = (YMax + YMin) / 2;		
		//	
		//double XIntervalValue = (XMax - XMin) / IntervalNumber;

		////对数据进行分区
		//int i = 0, j = 0;
		//
		//vector <Area> CircleArea;
		//
		//double XValue = XMin, YValue, ZValue;		
		//
		//for (i = 0; i < IntervalNumber; i++)	//CircleArea的记录个数为 IntervalNumber * 2;
		//{		
		//	Area UpArea;
		//	UpArea.XMin = XValue;
		//	UpArea.XMax = UpArea.XMin + XIntervalValue;
		//	UpArea.YMin = YMin;
		//	UpArea.YMax = TempYCenter;
		//	UpArea.PointIndex = -1;
		//	CircleArea.push_back(UpArea);	
		//	//x坐标位于 XMin 至 XMax 之间 且 y坐标位于 YMin 至 TempYCenter 之间

		//	Area DownArea;
		//	DownArea.XMin = UpArea.XMin;
		//	DownArea.XMax = UpArea.XMax;
		//	DownArea.YMin = TempYCenter;
		//	DownArea.YMax = YMax;
		//	DownArea.PointIndex = -1;
		//	CircleArea.push_back(DownArea);
		//	//x坐标位于 XMin 至 XMax 之间 且 y坐标位于 TempYCenter 至 YMax 之间

		//	XValue = XValue + XIntervalValue;
		//	//XValue的值逐渐增大
		//}

		//for (i = 0; i < Cloud->points.size(); i++)
		//{
		//	XValue = Cloud->points[i].x;
		//	YValue = Cloud->points[i].y;
		//	ZValue = Cloud->points[i].z;
		//	for (j = 0; j < CircleArea.size(); j++)
		//	{
		//		if ((CircleArea[j].PointIndex == -1) 
		//				//&& (abs(TempZCenter - ZValue) < 0.1) 
		//				&& (XValue >= CircleArea[j].XMin) && (XValue < CircleArea[j].XMax)
		//				&& (YValue >= CircleArea[j].YMin) && (YValue < CircleArea[j].YMax)
		//			)
		//		{
		//			CircleArea[j].PointIndex = i;	
		//		}
		//	}
		//}
		//int Count = 0;
		//for (i = 0; i < CircleArea.size(); i++)
		//{
		//	if (CircleArea[i].PointIndex != -1)
		//	{
		//		Count++;
		//		xCenter = xCenter + Cloud->points[CircleArea[i].PointIndex].x;
		//		yCenter = yCenter + Cloud->points[CircleArea[i].PointIndex].y;
		//		zCenter = zCenter + Cloud->points[CircleArea[i].PointIndex].z;
		//	}
		//}
		//if (Count > 0)
		//{
		//	xCenter = xCenter / Count;
		//	yCenter = yCenter / Count;
		//	zCenter = zCenter / Count;
		//}
		//else
		//{
		//	xCenter = 0;
		//	yCenter = 0;
		//	zCenter = 0;
		//}
	}

	static void GetPointsMaxAndMin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		double & XMax, double & XMin, double & YMax, double & YMin, double & ZMax, double & ZMin,
		double MaxUpValue = 0)
	{	
		if (Cloud->points.size() == 0) return;

		XMax = Cloud->points[0].x;	
		YMax = Cloud->points[0].y;
		ZMax = Cloud->points[0].z;

		XMin = Cloud->points[0].x;	
		YMin = Cloud->points[0].y;
		ZMin = Cloud->points[0].z;

		for (int i = 1; i < Cloud->size(); i++)
		{
			XMax = std::max(XMax, double(Cloud->points[i].x));
			YMax = std::max(YMax, double(Cloud->points[i].y));
			ZMax = std::max(ZMax, double(Cloud->points[i].z));

			XMin = std::min(XMin, double(Cloud->points[i].x));
			YMin = std::min(YMin, double(Cloud->points[i].y));
			ZMin = std::min(ZMin, double(Cloud->points[i].z));			
		}
		//XMax = XMax + MaxUpValue;
		//YMax = YMax + MaxUpValue;
		//ZMax = ZMax + MaxUpValue;
	}

	static void GetPointsMaxAndMin(pcl::PointCloud<PointXYZRGBIndex>::Ptr Cloud,
		double & XMax, double & XMin, double & YMax, double & YMin, double & ZMax, double & ZMin,
		double MaxUpValue = 0)
	{
		if (Cloud->points.size() == 0) return;

		XMax = Cloud->points[0].x;	
		YMax = Cloud->points[0].y;
		ZMax = Cloud->points[0].z;

		XMin = Cloud->points[0].x;	
		YMin = Cloud->points[0].y;
		ZMin = Cloud->points[0].z;

		for (int i = 1; i < Cloud->size(); i++)
		{
			XMax = std::max(XMax, double(Cloud->points[i].x));
			YMax = std::max(YMax, double(Cloud->points[i].y));
			ZMax = std::max(ZMax, double(Cloud->points[i].z));

			XMin = std::min(XMin, double(Cloud->points[i].x));
			YMin = std::min(YMin, double(Cloud->points[i].y));
			ZMin = std::min(ZMin, double(Cloud->points[i].z));
		}
	}
/*
	static void PointsMove(pcl::PointCloud<pcl::PointXYZ>::Ptr pPoints,
		double X, double Y, double Z)	//设置点云的坐标平移
	{
		for (int i = 0; i < pPoints->points.size(); i++)
		{
			pPoints->points[i].x = pPoints->points[i].x + X;
			pPoints->points[i].y = pPoints->points[i].y + Y;
			pPoints->points[i].z = pPoints->points[i].z + Z;
		}
	}

	static void PointsMove(PointXYZRGBIndex & Point,
		double X, double Y, double Z)	//设置点云的坐标平移
	{
		Point.x = Point.x + X;
		Point.y = Point.y + Y;
		Point.z = Point.z + Z;
	}

	static void PointsMove(pcl::PointXYZRGB & Point,
		double X, double Y, double Z)	//设置点云的坐标平移
	{
		Point.x = Point.x + X;
		Point.y = Point.y + Y;
		Point.z = Point.z + Z;
	}

	static void PointsMove(pcl::PointCloud<PointXYZRGBIndex>::Ptr pPoints,
		double X, double Y, double Z)	//设置点云的坐标平移
	{
		for (int i = 0; i < pPoints->points.size(); i++)
		{
			pPoints->points[i].x = pPoints->points[i].x + X;
			pPoints->points[i].y = pPoints->points[i].y + Y;
			pPoints->points[i].z = pPoints->points[i].z + Z;
		}
	}

	static void PointsMove(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPoints,
		double X, double Y, double Z)	//设置点云的坐标平移
	{
		for (int i = 0; i < pPoints->points.size(); i++)
		{
			pPoints->points[i].x = pPoints->points[i].x + X;
			pPoints->points[i].y = pPoints->points[i].y + Y;
			pPoints->points[i].z = pPoints->points[i].z + Z;
		}
	}

	static void PointsMove(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPoints,
		double X, double Y, double Z)	//设置点云的坐标平移
	{
		for (int i = 0; i < pPoints->points.size(); i++)
		{
			pPoints->points[i].x = pPoints->points[i].x + X;
			pPoints->points[i].y = pPoints->points[i].y + Y;
			pPoints->points[i].z = pPoints->points[i].z + Z;
		}
	}

	*/

	static void SavePCDToFileName(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPoints, string FileName)
		//将 points 保存到FileName中
	{
		string ExtName = FileName.substr(FileName.length() - 3, 3);
		//cout << ExtName << endl;
		if (0 == ExtName.compare("vtx"))
		{
			PointBase::SaveToVTX(pPoints, FileName);
		}
		else if (0 == ExtName.compare("xyz"))
		{
			PointBase::SaveToXYZ(pPoints, FileName);
		}
		else if (0 == ExtName.compare("las"))
		{
			PointBase::SaveToLAS(pPoints, FileName);
			//PointBase::SaveToLASBYLibLas(pPoints, FileName);
		}
		else if (0 == ExtName.compare("ply"))
		{
			PointBase::SaveToPLY(pPoints, FileName);
		}
		else if (0 == ExtName.compare("pcd"))
		{
			if (pPoints->size() < 1) return;
			//cout << "数据保存到" << FileName << "进行中......" << endl;
			pPoints->width = pPoints->size();
			pPoints->height = 1;
			pPoints->is_dense = false;
			//PointXYZRGBToPointXYZRGBA(pPoints, XYZRGBAPoints);
			//保存前去重复化处理，2016.01.13 因太耗时而取消
			//GetDistinctPoints(pPoints);
			pcl::io::savePCDFileASCII(FileName, *pPoints);
			//cout << "数据保存到" << FileName << "完成！" << endl;
		}
	}

	static void SavePCDToFileName(pcl::PointCloud<PointXYZRGBIndex>::Ptr pPoints, string FileName)
		//将 points 保存到FileName中
	{
		if (pPoints->size() < 1) return;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
			PointXYZRGBPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

		PointXYZRGBIndexToPointXYZRGB(pPoints, PointXYZRGBPtr);

		SavePCDToFileName(PointXYZRGBPtr, FileName);
	}

	static void SavePCDToFileName(pcl::PointCloud<pcl::PointXYZ>::Ptr pPoints, string FileName)
		//将 points 保存到FileName中
	{
		if (pPoints->size() < 1) return;
		pPoints->width = pPoints->size();
		pPoints->height = 1;
		pPoints->is_dense = false;
		pcl::io::savePCDFileASCII(FileName, *pPoints);
	}

	static void Bresenham_Circle(int MultipleValue,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CircleCloud,
		double xCenter, double yCenter, double zCenter, double r,
		int Color = ColorBase::WhiteColor)
		//以 x,y,z 为圆心，生成半径为r的圆的点云
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		xCenter = xCenter * MultipleValue;
		yCenter = yCenter * MultipleValue;
		zCenter = zCenter * MultipleValue;
		r = r * MultipleValue;
		Bresenham_Circle(TempCloud, xCenter, yCenter, zCenter, r, ColorBase::WhiteColor);
		for (int i = 0; i < TempCloud->points.size(); i++)
		{
			pcl::PointXYZRGB TempPoint;

			TempPoint.x = TempCloud->points[i].x / MultipleValue;
			TempPoint.y = TempCloud->points[i].y / MultipleValue;
			TempPoint.z = TempCloud->points[i].z / MultipleValue;
			TempPoint.rgba = TempCloud->points[i].rgba;

			CircleCloud->points.push_back(TempPoint);
		}
	}

	static void Bresenham_Circle(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CircleCloud,
		double xCenter, double yCenter, double zCenter, double r, int Color = ColorBase::WhiteColor)
		//以 x,y,z 为圆心，生成半径为r的圆的点云
	{
		double x, y, d;
		pcl::PointXYZRGB point;
		y = r;
		d = 3 - 2 * r;
		x = 0;

		point.rgba = Color;

		while (x <= y)
		{
			point.x = xCenter + x;
			point.y = yCenter + y;
			point.z = zCenter;
			CircleCloud->points.push_back(point);

			point.x = xCenter + x;
			point.y = yCenter - y;
			point.z = zCenter;
			CircleCloud->points.push_back(point);

			point.x = xCenter - x;
			point.y = yCenter + y;
			point.z = zCenter;
			CircleCloud->points.push_back(point);

			point.x = xCenter - x;
			point.y = yCenter - y;
			point.z = zCenter;
			CircleCloud->points.push_back(point);

			point.x = xCenter + y;
			point.y = yCenter + x;
			point.z = zCenter;
			CircleCloud->points.push_back(point);

			point.x = xCenter + y;
			point.y = yCenter - x;
			point.z = zCenter;
			CircleCloud->points.push_back(point);

			point.x = xCenter - y;
			point.y = yCenter + x;
			point.z = zCenter;
			CircleCloud->points.push_back(point);

			point.x = xCenter - y;
			point.y = yCenter - x;
			point.z = zCenter;
			CircleCloud->points.push_back(point);

			if (d < 0)
				d += 4 * x + 6;
			else
			{
				d += 4 * (x - y) + 10;
				y = y - 1;
			}
			x = x + 1;
		}
	}

	static void ShowCoordinateSystem(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointXYZRGB Point, double CoordinateSize = 1, double FontSize = 1, bool ShowAxis = true)
	{
		pcl::PointXYZRGB  XAxis, YAxis, ZAxis;
		XAxis = Point, YAxis = Point, ZAxis = Point;

		XAxis.x += CoordinateSize + 0.1;

		YAxis.y += CoordinateSize + 0.1;

		ZAxis.z += CoordinateSize + 0.1;

		if (ShowAxis)
		{
			Viewer->addText3D("x", XAxis, FontSize, 0, 0, 0, "x" + StringBase::ClockValue());
			Viewer->addText3D("y", YAxis, FontSize, 0, 0, 0, "y" + StringBase::ClockValue());
			Viewer->addText3D("z", ZAxis, FontSize, 0, 0, 0, "z" + StringBase::ClockValue());
		}
		Viewer->addCoordinateSystem(CoordinateSize, Point.x, Point.y, Point.z);
	}


	static void ShowPointXYZRGB(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
		const std::string IDStr, double PointSizeValue, int Viewport = 0)
	{	//显示点云数据
		if (cloud->points.size() < 1) return;
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		//Viewer->removePointCloud(IDStr);
		Viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, IDStr, Viewport);
		Viewer->setPointCloudRenderingProperties
		(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSizeValue, IDStr);
		//Viewer->addCoordinateSystem(1); //缩放比率 默认是1 原始大小	
		//Viewer->initCameraParameters();
	}

	//根据节点距离的平均值，插入新的节点
	static void HomogenizationPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints)
	{
		double AvgDis = 0;
		CalcBase<double> CalcBaseFloat;
		vector<double> VecDis;

		for (int i = 0; i < TempPoints->points.size() - 1; i++)
		{
			double TempDis = PointDis(TempPoints->points[i], TempPoints->points[i + 1]);
			VecDis.push_back(TempDis);
			AvgDis = AvgDis + TempDis / TempPoints->points.size();
		}

		int AddNumber = 0;
		for (int i = 0; i < VecDis.size(); i++)
		{
			int K = VecDis[i] / AvgDis;

			if (K > 2)
			{
				double XStep = (TempPoints->points[i + AddNumber + 1].x - TempPoints->points[i + AddNumber].x) / K;
				double YStep = (TempPoints->points[i + AddNumber + 1].y - TempPoints->points[i + AddNumber].y) / K;
				double ZStep = (TempPoints->points[i + AddNumber + 1].z - TempPoints->points[i + AddNumber].z) / K;

				for (int j = 1; j < K; j++)
				{
					pcl::PointXYZRGB NewPoint;
					NewPoint.x = TempPoints->points[i + AddNumber].x + XStep;
					NewPoint.y = TempPoints->points[i + AddNumber].y + YStep;
					NewPoint.z = TempPoints->points[i + AddNumber].z + ZStep;
					TempPoints->insert(TempPoints->begin() + i + AddNumber + 1, NewPoint);
					AddNumber = AddNumber + 1;
				}
			}
		}
	}

	static void ShowPointXYZRGBText(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
		const std::string TextName, double TextSizeValue, int Viewport = 0,
		bool Is2D = false, std::string ShowTextValue = "")
	{
		string ShowText = "";
		CalcBase<int> CalcBaseInt;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			Viewer->removeShape(TextName + CalcBaseInt.ConvertToString(i));

			if (ShowTextValue == "")
				ShowText = CalcBaseInt.ConvertToString(i);
			else
				ShowText = ShowTextValue + "_" + CalcBaseInt.ConvertToString(i);
			
			if (Is2D)
			{				
				Viewer->addText(ShowText, cloud->points[i].x,
					cloud->points[i].y,
					TextSizeValue, 255, 0, 0,
					TextName + CalcBaseInt.ConvertToString(i), Viewport);
			}
			else
			{				
				Viewer->addText3D(ShowText, cloud->points[i],
					TextSizeValue, 255, 0, 0,
					TextName + CalcBaseInt.ConvertToString(i), Viewport);
			}			
		}
	}

	static void ShowPointXYZRGBText(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointCloud<PointXYZRGBIndex>::Ptr cloud,
		const std::string TextName, double TextSizeValue, int Viewport = 0,
		bool Is2D = false)
	{
		if (cloud->points.size() < 1) return;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
			PointXYZRGBPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

		PointXYZRGBIndexToPointXYZRGB(cloud, PointXYZRGBPtr);
		ShowPointXYZRGBText(Viewer, PointXYZRGBPtr, TextName, TextSizeValue, Viewport, Is2D);
	}

	static void ShowPointXYZ(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
		const std::string IDStr, double PointSizeValue)
	{	//显示点云数据	
		if (cloud->points.size() < 1) return;
		Viewer->addPointCloud<pcl::PointXYZ>(cloud);
		Viewer->setPointCloudRenderingProperties
		(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSizeValue, IDStr);
		//Viewer->addCoordinateSystem(1); //缩放比率 默认是1 原始大小	
		//Viewer->initCameraParameters();
	}

	static void ShowPointXYZRGBIndex(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointCloud<PointXYZRGBIndex>::Ptr cloud,
		const std::string IDStr, double PointSizeValue, int Viewport = 0)
	{	//显示点云数据

		if (cloud->points.size() < 1) return;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
			PointXYZRGBPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

		PointXYZRGBIndexToPointXYZRGB(cloud, PointXYZRGBPtr);

		ShowPointXYZRGB(Viewer, PointXYZRGBPtr, IDStr, PointSizeValue, Viewport);
	}

	//创建法线 在 Radius 半径内创建法线
	static pcl::PointCloud<pcl::Normal>::Ptr CalcPointNormalsWithKDTree(
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr Cloud, double Radius) //
	{
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree
		(new pcl::search::KdTree<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::Normal>::Ptr Cloud_Normals(new pcl::PointCloud<pcl::Normal>);

		if (Cloud->points.size() > 0)
		{
			ne.setInputCloud(Cloud);
			ne.setSearchMethod(tree);

			ne.setRadiusSearch(Radius);
			ne.compute(*Cloud_Normals);
		}

		//计算单个点的法线
		//computePointNormal (const pcl::PointCloud<PointInT>&cloud, 
		//	const std::vector<int>&indices, Eigen::Vector4f &plane_parameters, float&curvature);

		return 	Cloud_Normals;
	}

	static pcl::PointCloud<pcl::Normal>::Ptr CalcPointNormalsWithOCTree(
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr Cloud, double Radius) //
	{
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

		pcl::search::Octree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::Octree<pcl::PointXYZRGB>(0.1f));
		pcl::PointCloud<pcl::Normal>::Ptr Cloud_Normals(new pcl::PointCloud<pcl::Normal>);

		if (Cloud->points.size() > 0)
		{
			ne.setInputCloud(Cloud);
			ne.setSearchMethod(tree);

			ne.setRadiusSearch(Radius);
			ne.compute(*Cloud_Normals);
		}

		//计算单个点的法线
		//computePointNormal (const pcl::PointCloud<PointInT>&cloud, 
		//	const std::vector<int>&indices, Eigen::Vector4f &plane_parameters, float&curvature);

		return 	Cloud_Normals;
	}

	static void ShowNormals(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr Cloud,
		pcl::PointCloud<pcl::Normal>::ConstPtr Normals,
		const std::string PointStr, const std::string NormalStr,
		const double NormalScale, const double PointSizeValue)
	{
		//添加点云
		Viewer->addPointCloud<pcl::PointXYZRGB>(Cloud, PointStr);
		//设置点云属性
		Viewer->setPointCloudRenderingProperties
		(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSizeValue, PointStr);
		//添加点云法向量
		Viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>
			(Cloud, Normals, 100, NormalScale, NormalStr);

		Viewer->addCoordinateSystem(1.0);
		Viewer->initCameraParameters();
	}

	static boost::shared_ptr<pcl::visualization::PCLVisualizer> GetViewer()
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer
		(new pcl::visualization::PCLVisualizer("3D Viewer"));

		//Viewer->getCameraParameters(argc, argv);

		//设置背景颜色 应该修改为白色，便于显示点云  2014.10.24
		//Viewer->setBackgroundColor (255, 255, 255);  
		Viewer->setBackgroundColor(0, 0, 0);

		////以下语句缺少会导致显示不出内容 或调用  initCameraParameters 方法
		//Viewer->camera_.clip[0] = 1877.96;
		//Viewer->camera_.clip[1] = 4733.13;

		//Viewer->camera_.pos[0] = 212.715;
		//Viewer->camera_.pos[1] = -310.856;
		//Viewer->camera_.pos[2] = 7887.16;

		//Viewer->camera_.focal[0] = 3313.03;
		//Viewer->camera_.focal[1] = 272.438;
		//Viewer->camera_.focal[2] = 7651.18;

		//Viewer->camera_.view[0] = 0.0648286;
		//Viewer->camera_.view[1] = 0.0584453;
		//Viewer->camera_.view[2] = 0.996183;

		//Viewer->camera_.fovy = 0.523599;
		//Viewer->camera_.window_size[0] = 300;
		//Viewer->camera_.window_size[1] = 550;
		//Viewer->initCameraParameters();
		Viewer->resetCamera();

		return Viewer;
	}

	static void SetViewerCamera(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		double focalx, double focaly, double focalz)
	{
		//Viewer->camera_.focal[0] = focalx;
		//Viewer->camera_.focal[1] = focaly;
		//Viewer->camera_.focal[2] = focalz;
		Viewer->updateCamera();
	}

	//2014.09.24 设置点云的颜色
	static void SetPointColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud, int ColorValue)
	{
		for (int i = 0; i < Cloud->points.size(); i++)
		{
			Cloud->points[i].rgba = ColorValue;
		}
	}

	static void SetPointColor(pcl::PointCloud<PointXYZRGBIndex>::Ptr Cloud, int ColorValue)
	{
		for (int i = 0; i < Cloud->points.size(); i++)
		{
			Cloud->points[i].rgba = ColorValue;
		}
	}

	//2014.10.20 得到点云的一份Copy 返回点云的指针
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		GetPointCopy(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReturnCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

		for (int i = 0; i < Cloud->points.size(); i++)
		{
			ReturnCloud->points.push_back(Cloud->points[i]);
		}

		return ReturnCloud;
	}

	static pcl::PointXYZRGB	GetGravityPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud)
	{
		pcl::PointXYZRGB GravityPoint;
		GravityPoint.x = 0;
		GravityPoint.y = 0;
		GravityPoint.z = 0;

		for (int i = 0; i < Cloud->points.size(); i++)
		{
			GravityPoint.x = GravityPoint.x + Cloud->points[i].x;
			GravityPoint.y = GravityPoint.y + Cloud->points[i].y;
			GravityPoint.z = GravityPoint.z + Cloud->points[i].z;
		}

		GravityPoint.x = GravityPoint.x / Cloud->points.size();
		GravityPoint.y = GravityPoint.y / Cloud->points.size();
		GravityPoint.z = GravityPoint.z / Cloud->points.size();

		return GravityPoint;
	}

	//2014.10.20 得到点云的一份Copy 返回点云的指针
	static pcl::PointCloud<pcl::PointXYZ>::Ptr
		GetPointCopy(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr ReturnCloud(new pcl::PointCloud<pcl::PointXYZ>());

		for (int i = 0; i < Cloud->points.size(); i++)
		{
			ReturnCloud->points.push_back(Cloud->points[i]);
		}

		return ReturnCloud;
	}



	//2014.10.27 只显示文件中的数据
	static void ShowPCDFileOnly(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_ptr)
	{
		string FileName;

		cout << "请输入需要显示的PCD文件名称" << endl;
		cin >> FileName;
		std::cout << "正在读取点云文件，请稍候！\n";

		if (pcl::io::loadPCDFile<pcl::PointXYZ>
			(FileName, *Cloud_ptr) == -1)//*打开点云文件
		{
			PCL_ERROR("Couldn't read Pcd file \n");
		}
		else

			std::cout << "读取点云文件成功！\n";
	}

	//2014.11.01 将XYZRGB点云转换为XYZ点云
	static void PointXYZRGBToXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZRGBCloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr XYZCloud)
	{
		XYZCloud->points.clear();
		for (int i = 0; i < XYZRGBCloud->points.size(); i++)
		{
			pcl::PointXYZ TempPoint;
			TempPoint.x = XYZRGBCloud->points[i].x;
			TempPoint.y = XYZRGBCloud->points[i].y;
			TempPoint.z = XYZRGBCloud->points[i].z;
			XYZCloud->points.push_back(TempPoint);
		}
	}

	//2014.11.02 对点云下采样
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		VoxelGridDownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudIn,
			double Leafx = 0.1f, double Leafy = 0.1f, double Leafz = 0.1f)
	{
		// 创建滤波器对象
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudOut(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::VoxelGrid<pcl::PointXYZRGB> VoxelGridFilter;

		VoxelGridFilter.setInputCloud(CloudIn);
		VoxelGridFilter.setLeafSize(Leafx, Leafy, Leafz);

		VoxelGridFilter.filter(*CloudOut);
		//每一个立方毫米取一个点	
		return CloudOut;
	}

	//2014.11.02 对点云下采样
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		VoxelGridDownSampleByCenterPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudIn,
			double Leafx = 0.1f, double Leafy = 0.1f, double Leafz = 0.1f)
	{
		// 创建滤波器对象
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudOut(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> ApproximateVoxelGrid;

		ApproximateVoxelGrid.setInputCloud(CloudIn);
		ApproximateVoxelGrid.setLeafSize(Leafx, Leafy, Leafz);

		ApproximateVoxelGrid.filter(*CloudOut);
		//每一个立方毫米取一个点	
		return CloudOut;
	}


	//2014.11.02 对点云下采样
	static pcl::PointCloud<pcl::PointXYZ>::Ptr
		VoxelGridDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudIn,
			double Leafx = 0.1f, double Leafy = 0.1f, double Leafz = 0.1f)
	{
		// 创建滤波器对象
		pcl::PointCloud<pcl::PointXYZ>::Ptr CloudOut(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::VoxelGrid<pcl::PointXYZ> VoxelGridFilter;

		VoxelGridFilter.setInputCloud(CloudIn);
		VoxelGridFilter.setLeafSize(Leafx, Leafy, Leafz);

		VoxelGridFilter.filter(*CloudOut);
		//每一个立方毫米取一个点	
		return CloudOut;
	}

	//2014.11.02 对点云下采样
	static pcl::PointCloud<PointXYZRGBIndex>::Ptr
		VoxelGridDownSample(pcl::PointCloud<PointXYZRGBIndex>::Ptr CloudIn,
			double Leafx = 0.1f, double Leafy = 0.1f, double Leafz = 0.1f)
	{
		// 创建滤波器对象
		pcl::PointCloud<PointXYZRGBIndex>::Ptr CloudOut(new pcl::PointCloud<PointXYZRGBIndex>);

		pcl::VoxelGrid<PointXYZRGBIndex> VoxelGridFilter;

		VoxelGridFilter.setInputCloud(CloudIn);
		VoxelGridFilter.setLeafSize(Leafx, Leafy, Leafz);

		VoxelGridFilter.filter(*CloudOut);
		//每一个立方毫米取一个点	
		return CloudOut;
	}

	//2014.11.03 
	static void AddPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr SourceCloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr DestinationCloud)
	{
		for (int i = 0; i < SourceCloud->points.size(); i++)
		{
			pcl::PointXYZ TempPoint;
			TempPoint.x = SourceCloud->points[i].x;
			TempPoint.y = SourceCloud->points[i].y;
			TempPoint.z = SourceCloud->points[i].z;
			DestinationCloud->points.push_back(TempPoint);
		}
	}

	//2014.11.03 
	static void AddPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr SourceCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr DestinationCloud)
	{
		for (int i = 0; i < SourceCloud->points.size(); i++)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = SourceCloud->points[i].x;
			TempPoint.y = SourceCloud->points[i].y;
			TempPoint.z = SourceCloud->points[i].z;
			TempPoint.rgba = SourceCloud->points[i].rgba;
			DestinationCloud->points.push_back(TempPoint);
		}
	}

	static void PointCopy(pcl::PointCloud<PointXYZRGBIndex>::Ptr SourcePtr,
		pcl::PointCloud<PointXYZRGBIndex>::Ptr DestinationPtr,
		bool IndexReArrange = false)
	{
		DestinationPtr->points.clear();
		DestinationPtr->insert(DestinationPtr->points.end(),
			SourcePtr->points.begin(),
			SourcePtr->points.end());

		// Index 需要重新编排
		if (IndexReArrange)
		{
			for (int i = 0; i < DestinationPtr->points.size(); i++)
			{
				DestinationPtr->points[i].Index = i;
			}
		}
	}

	static void PointCopy(pcl::PointCloud<pcl::PointXYZRGB>::Ptr SourcePtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr DestinationPtr)
	{
		DestinationPtr->points.clear();
		DestinationPtr->insert(DestinationPtr->points.end(),
			SourcePtr->points.begin(),
			SourcePtr->points.end());
	}

	//2016.03.17 修改为移动到原点
	static void PointMoveToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr)
	{
		if (PointPtr->points.size() == 0) return;

		double MinDis = EPSP9;
		int MinDisPointIndex = -1;
		double MinZ = EPSP9;
		int LowestZPointIndex = -1;
		//Move the point cloud to the closest point to the origin point
		//the method is invalid when the origin point is in the point cloud.
		for (int i = 0; i < PointPtr->points.size(); i++)
		{
			double TempDis = PointDis(double(0), double(0), double(0),
				double(PointPtr->points[i].x), double(PointPtr->points[i].y), double(PointPtr->points[i].z));
			
			if (TempDis < MinDis)
			{
				MinDis = TempDis;
				MinDisPointIndex = i;
			}
			if (PointPtr->points[i].z < MinZ)
			{
				LowestZPointIndex = i;
				MinZ = PointPtr->points[i].z;
				//MinZ = std::min(MinZ, double(PointPtr->points[i].z));
			}
		}

		double OriginDis = PointDis(double(0), double(0), double(0),
			double(PointPtr->points[MinDisPointIndex].x), double(PointPtr->points[MinDisPointIndex].y), 
			double(PointPtr->points[MinDisPointIndex].z));

		double LowestDis = PointDis(double(0), double(0), double(0),
			double(PointPtr->points[LowestZPointIndex].x), double(PointPtr->points[LowestZPointIndex].y),
			double(PointPtr->points[LowestZPointIndex].z));

		// if the origin point is in the point cloud, the point cloud move to the lowest point
		if (OriginDis < LowestDis)	
		{
			MinDisPointIndex = LowestZPointIndex;
		}

		//2D 点云不移动
		if (!PointsIs2D(PointPtr))
		{
			cout<<"点云移动到坐标原点附近以使其更好地显示！"<<endl;
			PointsMove(PointPtr, PointPtr->points[MinDisPointIndex].x * -1,
				PointPtr->points[MinDisPointIndex].y * -1, PointPtr->points[MinDisPointIndex].z * -1);
		}
	}

	//2016.03.17 修改为移动到原点
	static void PointMoveToOrigin(pcl::PointCloud<PointXYZRGBIndex>::Ptr PointPtr)
	{
		if (PointPtr->points.size() == 0) return;

		double MinDis = 1.0e+10;
		int MinDisPointIndex = -1;

		for (int i = 0; i < PointPtr->points.size(); i++)
		{
			double TempDis = PointDis(float(0), float(0), float(0), 
				PointPtr->points[i].x, PointPtr->points[i].y, PointPtr->points[i].z);

			if (TempDis < MinDis)
			{
				MinDis = TempDis;
				MinDisPointIndex = i;
			}
		}

		//根据距离原点最近的点,将其移动
		if (MinDisPointIndex > -1)
		{
			cout << "点云移动到坐标原点附近以使其更好地显示！" << endl;
			PointsMove(PointPtr, PointPtr->points[MinDisPointIndex].x * -1,
				PointPtr->points[MinDisPointIndex].y * -1, PointPtr->points[MinDisPointIndex].z * -1);
		}
	}

	//2016.03.17 修改为移动到原点
	static void PointMoveToOrigin(pcl::PointCloud <pcl::PointXYZRGBA>::Ptr PointPtr)
	{
		if (PointPtr->points.size() == 0) return;

		double MinDis = 1.0e+10;
		int MinDisPointIndex = -1;

		for (int i = 0; i < PointPtr->points.size(); i++)
		{
			double TempDis = PointDis(float(0), float(0), float(0), PointPtr->points[i].x, PointPtr->points[i].y, PointPtr->points[i].z);

			if (TempDis < MinDis)
			{
				MinDis = TempDis;
				MinDisPointIndex = i;
			}
		}

		//根据距离原点最近的点,将其移动
		if (MinDisPointIndex > -1)
		{
			cout << "点云移动到坐标原点附近以使其更好地显示！" << endl;
			PointsMove(PointPtr, PointPtr->points[MinDisPointIndex].x * -1,
				PointPtr->points[MinDisPointIndex].y * -1, PointPtr->points[MinDisPointIndex].z * -1);
		}
	}

	static void StatColorInfor(pcl::PointCloud<PointXYZRGBIndex>::Ptr PointPtr)
	{
		vector<int> ColorR;
		vector<int> ColorG;
		vector<int> ColorB;
		for (int i = 0; i < PointPtr->points.size(); i++)
		{
			int R, G, B;
			R = PointPtr->points[i].r;
			G = PointPtr->points[i].g;
			B = PointPtr->points[i].b;

			bool Find = false;
			for (int j = 0; j < ColorR.size(); j++)
			{
				if (ColorR[j] == R && ColorG[j] == G && ColorB[j] == B)
				{
					Find = true;
					break;
				}
			}

			if (!Find)
			{
				ColorR.push_back(R);
				ColorG.push_back(G);
				ColorB.push_back(B);
			}
		}
		cout << "颜色已经找完" << endl;
	}

	static void PointXYZRGBIndexToPointXYZRGB(
		pcl::PointCloud<PointXYZRGBIndex>::Ptr PointXYZRGBIndexPtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr)
		//将带有索引的点云转换为不带索引的点云
	{
		PointXYZRGBPtr->points.clear();

		for (int i = 0; i < PointXYZRGBIndexPtr->points.size(); i++)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = PointXYZRGBIndexPtr->points[i].x;
			TempPoint.y = PointXYZRGBIndexPtr->points[i].y;
			TempPoint.z = PointXYZRGBIndexPtr->points[i].z;
			TempPoint.r = PointXYZRGBIndexPtr->points[i].r;
			TempPoint.g = PointXYZRGBIndexPtr->points[i].g;
			TempPoint.b = PointXYZRGBIndexPtr->points[i].b;
			TempPoint.rgb = PointXYZRGBIndexPtr->points[i].rgb;
			TempPoint.rgba = PointXYZRGBIndexPtr->points[i].rgba;
			PointXYZRGBPtr->points.push_back(TempPoint);
		}
	}


	static void PointXYZRGBAToPointXYZRGB(
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointXYZRGBAPtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr)
		//将不带有索引的点云转换为带索引的点云
	{
		PointXYZRGBPtr->points.clear();

		for (int i = 0; i < PointXYZRGBAPtr->points.size(); i++)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = PointXYZRGBAPtr->points[i].x;
			TempPoint.y = PointXYZRGBAPtr->points[i].y;
			TempPoint.z = PointXYZRGBAPtr->points[i].z;
			TempPoint.r = PointXYZRGBAPtr->points[i].r;
			TempPoint.g = PointXYZRGBAPtr->points[i].g;
			TempPoint.b = PointXYZRGBAPtr->points[i].b;
			TempPoint.rgb = PointXYZRGBAPtr->points[i].rgb;
			TempPoint.rgba = PointXYZRGBAPtr->points[i].rgba;
			PointXYZRGBPtr->points.push_back(TempPoint);
		}
	}

	static void PointXYZRGBToPointXYZRGBA(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBAPtr,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointXYZRGBPtr)
		//将不带有索引的点云转换为带索引的点云
	{
		PointXYZRGBPtr->points.clear();

		for (int i = 0; i < PointXYZRGBAPtr->points.size(); i++)
		{
			pcl::PointXYZRGBA TempPoint;
			TempPoint.x = PointXYZRGBAPtr->points[i].x;
			TempPoint.y = PointXYZRGBAPtr->points[i].y;
			TempPoint.z = PointXYZRGBAPtr->points[i].z;
			TempPoint.r = PointXYZRGBAPtr->points[i].r;
			TempPoint.g = PointXYZRGBAPtr->points[i].g;
			TempPoint.b = PointXYZRGBAPtr->points[i].b;
			TempPoint.rgb = PointXYZRGBAPtr->points[i].rgb;
			TempPoint.rgba = PointXYZRGBAPtr->points[i].rgba;
			PointXYZRGBPtr->points.push_back(TempPoint);
		}
	}

	//static void PointXYZRGBToPointXYZRGBIndex(
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr,
	//	pcl::PointCloud<PointXYZRGBIndex>::Ptr PointXYZRGBIndexPtr)
	////将不带有索引的点云转换为带索引的点云
	//{
	//	PointXYZRGBIndexPtr->points.clear();
	//
	//	for(int i = 0; i < PointXYZRGBPtr->points.size(); i++ )
	//	{		
	//		PointXYZRGBIndex TempPoint;
	//		TempPoint.x = PointXYZRGBPtr->points[i].x;
	//		TempPoint.y = PointXYZRGBPtr->points[i].y;
	//		TempPoint.z = PointXYZRGBPtr->points[i].z;
	//		TempPoint.r = PointXYZRGBPtr->points[i].r;
	//		TempPoint.g = PointXYZRGBPtr->points[i].g;
	//		TempPoint.b = PointXYZRGBPtr->points[i].b;
	//		TempPoint.rgb = PointXYZRGBPtr->points[i].rgb;
	//		TempPoint.rgba = PointXYZRGBPtr->points[i].rgba;
	//		TempPoint.Index = i;		
	//		TempPoint.Category = 1;	//默认分类是1
	//		PointXYZRGBIndexPtr->points.push_back(TempPoint);
	//	}
	//}

	static void DeepenColor(pcl::PointXYZRGB Point)
	{
		//if (abs(Point.x - 20.9299) < 0.1  && abs(Point.y - -35.89) < 0.1 && abs(Point.z - 16.9902) < 0.1)
		//{
		//	cout<<"红"<<endl;
		//}
		//if (abs(Point.x - 30.8301) < 0.1 && abs(Point.y - -9.17993) < 0.1 && abs(Point.z - 28.2402) < 0.1)
		//{
		//	cout<<"黄"<<endl;
		//}
		//if (abs(Point.x - 6.92004) < 0.1 && abs(Point.y - -35.4199) < 0.1 && abs(Point.z - 13.2705) < 0.1)
		//{
		//	cout<<"绿"<<endl;
		//}
		//if (abs(Point.x - 2.33997) < 0.1 && abs(Point.y - -0.699951) < 0.1 && abs(Point.z - 26.2607) < 0.1)
		//{
		//	cout<<"紫"<<endl;
		//}

		int BandWidth = 50;

		//if (Point.r >= 200 && Point.r <= 255 && Point.g >=0 && Point.g <= 50 && Point.b >=0 && Point.b <= 50)
		if (abs(Point.r - 224) <= BandWidth && abs(Point.g - 90) <= BandWidth && abs(Point.b - 85) <= BandWidth)
		{	//红
			Point.r = 255;
			Point.g = 0;
			Point.b = 0;
		}

		//if (Point.r >= 200 && Point.r <= 255 && Point.g >=200 && Point.g <= 255 && Point.b >=0 && Point.b <= 50)
		if (abs(Point.r - 238) <= BandWidth && abs(Point.g - 253) <= BandWidth && abs(Point.b - 176) <= BandWidth)
		{	//黄
			Point.r = 255;
			Point.g = 255;
			Point.b = 0;
		}
		//if (Point.r >= 0 && Point.r <= 50 && Point.g >=200 && Point.g <= 255 && Point.b >=0 && Point.b <= 50)
		if (abs(Point.r - 131) <= BandWidth && abs(Point.g - 186) <= BandWidth && abs(Point.b - 122) <= BandWidth)
		{	//绿
			Point.r = 0;
			Point.g = 255;
			Point.b = 0;
		}
		if (abs(Point.r - 129) <= BandWidth && abs(Point.g - 121) <= BandWidth && abs(Point.b - 226) <= BandWidth)
		{	//紫
			Point.r = 0;
			Point.g = 0;
			Point.b = 255;
		}
	}


	static void PointXYZRGBToPointXYZRGBIndex(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr,
		pcl::PointCloud<PointXYZRGBIndex>::Ptr PointXYZRGBIndexPtr)
		//将不带有索引的点云转换为带索引的点云
	{
		PointXYZRGBIndexPtr->points.clear();

		for (int i = 0; i < PointXYZRGBPtr->points.size(); i++)
		{
			//if (i == 85979)
			//{
			//	cout<<endl;
			//}

			DeepenColor(PointXYZRGBPtr->points[i]);
			PointXYZRGBIndex TempPoint;
			TempPoint.x = PointXYZRGBPtr->points[i].x;
			TempPoint.y = PointXYZRGBPtr->points[i].y;
			TempPoint.z = PointXYZRGBPtr->points[i].z;
			TempPoint.r = PointXYZRGBPtr->points[i].r;
			TempPoint.g = PointXYZRGBPtr->points[i].g;
			TempPoint.b = PointXYZRGBPtr->points[i].b;
			TempPoint.rgb = PointXYZRGBPtr->points[i].rgb;
			TempPoint.rgba = PointXYZRGBPtr->points[i].rgba;
			TempPoint.Index = i;
			TempPoint.Category = 1;	//默认分类是1
			PointXYZRGBIndexPtr->points.push_back(TempPoint);
		}
	}

	//将Mesh转换为三角形显示
	// 为了显示结合部位，角度在-M_PI/4～M_PI/6的点才显示网格结构
	static void MeshToTriangles(
		boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr,
		pcl::PolygonMesh Mesh,
		string LineNameValue = "",
		double CenterPointX = -1,
		double CenterPointY = -1,
		int Viewport = 0)
	{
		if (LineNameValue == "") LineNameValue = "Line";
		CalcBase<int> IntCalcBase;
		for (int i = 0; i < Mesh.polygons.size(); i++)
		{
			int OneIndex, SecondIndex, ThirdIndex;
			OneIndex = Mesh.polygons[i].vertices[0];
			SecondIndex = Mesh.polygons[i].vertices[1];
			ThirdIndex = Mesh.polygons[i].vertices[2];
			bool NeedDraw = false;

			if (CenterPointX != -1 && CenterPointY != -1)	//一四象限
			{
				if (PointPtr->points[OneIndex].x > CenterPointX ||
					PointPtr->points[SecondIndex].x > CenterPointX ||
					PointPtr->points[ThirdIndex].x > CenterPointX)
				{	//角度在-M_PI/4～M_PI/6的点才显示网格结构

					if (PointPtr->points[OneIndex].x != CenterPointX)
					{
						double Angle = atan((PointPtr->points[OneIndex].y - CenterPointY)
							/ (PointPtr->points[OneIndex].x - CenterPointX));
						if (Angle <= M_PI / 6 && Angle >= -1 * M_PI / 4)
						{
							NeedDraw = true;
						}
					}
					else if (PointPtr->points[SecondIndex].x != CenterPointX)
					{
						double Angle = atan((PointPtr->points[SecondIndex].y - CenterPointY)
							/ (PointPtr->points[SecondIndex].x - CenterPointX));
						if (Angle <= M_PI / 6 && Angle >= -1 * M_PI / 4)
						{
							NeedDraw = true;
						}
					}
					else if (PointPtr->points[ThirdIndex].x != CenterPointX)
					{
						double Angle = atan((PointPtr->points[ThirdIndex].y - CenterPointY)
							/ (PointPtr->points[ThirdIndex].x - CenterPointX));
						if (Angle <= M_PI / 6 && Angle >= -1 * M_PI / 4)
						{
							NeedDraw = true;
						}
					}
				}

			}
			else	//否则显示所有
			{
				NeedDraw = true;
			}

			if (NeedDraw)
			{
				Viewer->addLine(PointPtr->points[OneIndex],
					PointPtr->points[SecondIndex],
					0, 0, 0,
					LineNameValue + IntCalcBase.ConvertToString(i * 3), Viewport);
				Viewer->addLine(PointPtr->points[OneIndex],
					PointPtr->points[ThirdIndex],
					0, 0, 0,
					LineNameValue + IntCalcBase.ConvertToString(i * 3 + 1), Viewport);
				Viewer->addLine(PointPtr->points[SecondIndex],
					PointPtr->points[ThirdIndex],
					0, 0, 0,
					LineNameValue + IntCalcBase.ConvertToString(i * 3 + 2), Viewport);
			}
		}
	}

	static void CovertZToY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointPtr)				//将Z值转换到Y
	{
		double TempY = 0;
		for (int i = 0; i < PointPtr->points.size(); i++)
		{
			TempY = PointPtr->points[i].y;
			PointPtr->points[i].y = PointPtr->points[i].z;
			PointPtr->points[i].z = TempY;
		}
	}

	//取Mesh中不重复的三角形 也即删除Mesh中重复的三角形
	static void GetDistinctTriangles(pcl::PolygonMesh & Mesh)
	{
		int i = 0;
		while (i < Mesh.polygons.size())
		{
			int j = i + 1;
			while (j < Mesh.polygons.size())
			{
				if ((Mesh.polygons[i].vertices[0] == Mesh.polygons[j].vertices[0] ||
					Mesh.polygons[i].vertices[0] == Mesh.polygons[j].vertices[1] ||
					Mesh.polygons[i].vertices[0] == Mesh.polygons[j].vertices[2]) &&
					(Mesh.polygons[i].vertices[1] == Mesh.polygons[j].vertices[0] ||
						Mesh.polygons[i].vertices[1] == Mesh.polygons[j].vertices[1] ||
						Mesh.polygons[i].vertices[1] == Mesh.polygons[j].vertices[2]) &&
						(Mesh.polygons[i].vertices[2] == Mesh.polygons[j].vertices[0] ||
							Mesh.polygons[i].vertices[2] == Mesh.polygons[j].vertices[1] ||
							Mesh.polygons[i].vertices[2] == Mesh.polygons[j].vertices[2]))
				{
					Mesh.polygons.erase(Mesh.polygons.begin() + j);
				}
				else
					j++;
			}
			i++;
		}
	}





	//画线
	static void ShowLine(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		double a, double b, pcl::PointXYZRGB Point,
		string LineName = "", double size = 3,
		int Colorr = 255, int Colorg = 0, int Colorb = 0)
	{
		pcl::PointXYZRGB Point1, Point2;

		Point1 = Point; Point2 = Point;
		double k;
		if (b == 0)
		{
			Point1.x = Point.x + size;
			Point2.x = Point.x - size;
		}
		else if (a == 0)
		{
			Point1.y = Point.y + size;
			Point2.y = Point.y - size;
		}
		else
		{
			//a*x+b*y+d=0; k = -a/b;
			double k = -a / b;
			double Dis = sqrt(size * size / (k*k + 1));
			Point1.x = Point.x + Dis;
			//Point1.y = k * Point1.x - d;
			Point1.y = k * (Point1.x - Point.x) + Point.y;	//采用斜率方式计算较好 对点集d移动不敏感

			Point2.x = Point.x - Dis;
			//Point2.y = k * Point2.x - d;
			Point2.y = k * (Point2.x - Point.x) + Point.y;	//采用斜率方式计算较好 对点集d移动不敏感
		}

		Viewer->addLine(Point1, Point2, Colorr, Colorg, Colorb, LineName);
	}

	//画以 RreferencePoint 为参考点，投影长度为2×length的正方形  法向量为 (a,b,c)的切平面，
	static void ShowPlane(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		double a, double b, double c, double d, pcl::PointXYZRGB RreferencePoint,
		string PlaneName = "", string NumberValue = "0", bool DrawLine = true,
		double LineSize = 3, double FontSize = 3, int Colorr = 255, int Colorg = 0, int Colorb = 0)
	{
		pcl::PointXYZRGB CenterPoint = RreferencePoint;
		if (c != 0)
			CenterPoint.z = -(d + a * CenterPoint.x + b * CenterPoint.y) / c;
		else if (a != 0)
			CenterPoint.x = -(d + c * CenterPoint.z + b * CenterPoint.y) / a;
		else if (b != 0)
			CenterPoint.y = -(d + c * CenterPoint.z + a * CenterPoint.x) / b;

		ShowPlane(Viewer, a, b, c, CenterPoint, PlaneName, NumberValue,
			DrawLine, LineSize, FontSize, Colorr, Colorg, Colorb);
	}


	//计算 到 A 与 B 的中心点 距离为Size的两个点，在分别返回到 PointA 与 PointB 中
	static void GetMiddlePointofDis(pcl::PointXYZRGB & PointA, pcl::PointXYZRGB & PointB, double Size)
	{
		pcl::PointXYZRGB Middle;
		Middle.x = (PointA.x + PointB.x) / 2;
		Middle.y = (PointA.y + PointB.y) / 2;
		Middle.z = (PointA.z + PointB.z) / 2;
		double u = PointA.x - PointB.x;
		double v = PointA.y - PointB.y;
		double w = PointA.z - PointB.z;
		double t = sqrt(Size*Size / (u*u + v * v + w * w));

		PointA.x = Middle.x + t * u;
		PointA.y = Middle.y + t * v;
		PointA.z = Middle.z + t * w;

		PointB.x = Middle.x - t * u;
		PointB.y = Middle.y - t * v;
		PointB.z = Middle.z - t * w;
	}

	static void ReviseQuadrangle(pcl::PointXYZRGB & PointA, pcl::PointXYZRGB & PointB,
		pcl::PointXYZRGB & PointC, pcl::PointXYZRGB & PointD, double Size)
	{
		pcl::PointXYZRGB MiddlePoint;

		MiddlePoint.x = (PointA.x + PointB.x + PointC.x + PointD.x) / 4;
		MiddlePoint.y = (PointA.y + PointB.y + PointC.y + PointD.y) / 4;
		MiddlePoint.z = (PointA.z + PointB.z + PointC.z + PointD.z) / 4;

		double u0 = PointA.x - PointB.x;
		double v0 = PointA.y - PointB.y;
		double w0 = PointA.z - PointB.z;
		double t0 = sqrt(Size*Size / (u0*u0 + v0 * v0 + w0 * w0));

		pcl::PointXYZRGB TempPointA, TempPointB;

		TempPointA.x = MiddlePoint.x + t0 * u0;
		TempPointA.y = MiddlePoint.y + t0 * v0;
		TempPointA.z = MiddlePoint.z + t0 * w0;

		TempPointB.x = MiddlePoint.x - t0 * u0;
		TempPointB.y = MiddlePoint.y - t0 * v0;
		TempPointB.z = MiddlePoint.z - t0 * w0;

		u0 = PointA.x - PointC.x;
		v0 = PointA.y - PointC.y;
		w0 = PointA.z - PointC.z;
		t0 = sqrt(Size*Size / (u0*u0 + v0 * v0 + w0 * w0));

		PointA.x = TempPointA.x + t0 * u0;
		PointA.y = TempPointA.y + t0 * v0;
		PointA.z = TempPointA.z + t0 * w0;

		PointD.x = TempPointA.x - t0 * u0;
		PointD.y = TempPointA.y - t0 * v0;
		PointD.z = TempPointA.z - t0 * w0;

		PointB.x = TempPointB.x + t0 * u0;
		PointB.y = TempPointB.y + t0 * v0;
		PointB.z = TempPointB.z + t0 * w0;

		PointC.x = TempPointB.x - t0 * u0;
		PointC.y = TempPointB.y - t0 * v0;
		PointC.z = TempPointB.z - t0 * w0;
	}

	static pcl::PointXYZRGB PointNormalized(pcl::PointXYZRGB & NormalPoint)
	{
		double Model = sqrt(pow(NormalPoint.x, 2) + pow(NormalPoint.y, 2) + pow(NormalPoint.z, 2));
		if (Model != 0)
		{
			NormalPoint.x = NormalPoint.x / Model;
			NormalPoint.y = NormalPoint.y / Model;
			NormalPoint.z = NormalPoint.z / Model;
		}
		else
		{
			NormalPoint.x = 0, NormalPoint.y = 0, NormalPoint.z = 0;
		}
		return NormalPoint;
	}

	static void PointNormalized(pcl::Normal & NormalPoint)
	{
		double Model = sqrt(pow(NormalPoint.normal_x, 2) + pow(NormalPoint.normal_y, 2) + pow(NormalPoint.normal_z, 2));
		if (Model != 0)
		{
			NormalPoint.normal_x = NormalPoint.normal_x / Model;
			NormalPoint.normal_y = NormalPoint.normal_y / Model;
			NormalPoint.normal_z = NormalPoint.normal_z / Model;
		}
	} 

	static void PointNormalized(double & x, double & y, double & z)
	{
		double Model = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
		if (Model != 0)
		{
			x = x / Model;
			y = y / Model;
			z = z / Model;
		}
	}



	//Draw a plane (represeneted by a Square) passed through the CenterPoint，the edge length is equal to 2×LineWidth and the normal of the plane is (a,b,c)
	static void ShowPlane(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer, 
		double a, double b, double c, pcl::PointXYZRGB CenterPoint,
		string PlaneName = "", string NumberValue = "0", bool DrawLine = true,
		double LineWidth = 3, double FontSize = 1, int Colorr = 255, int Colorg = 0, int Colorb = 0)
	{	//平面方程 ax+by+cz+d = 0
		PointNormalized(a, b, c);
		//先对法向量正规化
		
		double d = -1 * (a * CenterPoint.x + b * CenterPoint.y + c * CenterPoint.z);
		pcl::PointXYZRGB Point1;
		pcl::PointXYZRGB Point2;
		pcl::PointXYZRGB Point3;
		pcl::PointXYZRGB Point4;

		if (a != 0 && abs(a) > 0.5)
		{
			//pcl::PointXYZRGB Point1;	 //三象限
			Point1.y = CenterPoint.y - LineWidth;
			Point1.z = CenterPoint.z - LineWidth;
			Point1.x = -1 * (d + b * Point1.y + c * Point1.z) / a;

			//pcl::PointXYZRGB Point2;	//二象限
			Point2.y = CenterPoint.y - LineWidth;
			Point2.z = CenterPoint.z + LineWidth;
			Point2.x = -1 * (d + b * Point2.y + c * Point2.z) / a;

			//pcl::PointXYZRGB Point3;	//一象限
			Point3.y = CenterPoint.y + LineWidth;
			Point3.z = CenterPoint.z + LineWidth;
			Point3.x = -1 * (d + b * Point3.y + c * Point3.z) / a;

			//pcl::PointXYZRGB Point4;	//四象限
			Point4.y = CenterPoint.y + LineWidth;
			Point4.z = CenterPoint.z - LineWidth;
			Point4.x = -1 * (d + b * Point4.y + c * Point4.z) / a;
		}
		else if (c != 0 && abs(c) > 0.5)
		{
			//pcl::PointXYZRGB Point1;	//三象限
			Point1.x = CenterPoint.x - LineWidth;
			Point1.y = CenterPoint.y - LineWidth;
			Point1.z = -1 * (d + a * Point1.x + b * Point1.y) / c;

			//pcl::PointXYZRGB Point2;	//二象限
			Point2.x = CenterPoint.x - LineWidth;
			Point2.y = CenterPoint.y + LineWidth;
			Point2.z = -1 * (d + a * Point2.x + b * Point2.y) / c;

			//pcl::PointXYZRGB Point3;	//一象限
			Point3.x = CenterPoint.x + LineWidth;
			Point3.y = CenterPoint.y + LineWidth;
			Point3.z = -1 * (d + a * Point3.x + b * Point3.y) / c;

			//pcl::PointXYZRGB Point4;	//四象限
			Point4.x = CenterPoint.x + LineWidth;
			Point4.y = CenterPoint.y - LineWidth;
			Point4.z = -1 * (d + a * Point4.x + b * Point4.y) / c;
		}
		else if (b != 0)
		{
			//pcl::PointXYZRGB Point1;	 //三象限
			Point1.x = CenterPoint.x - LineWidth;
			Point1.z = CenterPoint.z - LineWidth;
			Point1.y = -1 * (d + a * Point1.x + c * Point1.z) / b;

			//pcl::PointXYZRGB Point2;	//二象限
			Point2.x = CenterPoint.x - LineWidth;
			Point2.z = CenterPoint.z + LineWidth;
			Point2.y = -1 * (d + a * Point2.x + c * Point2.z) / b;

			//pcl::PointXYZRGB Point3;	//一象限
			Point3.x = CenterPoint.x + LineWidth;
			Point3.z = CenterPoint.z + LineWidth;
			Point3.y = -1 * (d + a * Point3.x + c * Point3.z) / b;

			//pcl::PointXYZRGB Point4;	//四象限
			Point4.x = CenterPoint.x + LineWidth;
			Point4.z = CenterPoint.z - LineWidth;
			Point4.y = -1 * (d + a * Point4.x + c * Point4.z) / b;
		}

		if (DrawLine)
		{
			ReviseQuadrangle(Point1, Point2, Point3, Point4, LineWidth);

			Viewer->addLine(Point1, Point2, Colorr, Colorg, Colorb, PlaneName + "Line1");
			Viewer->addLine(Point2, Point3, Colorr, Colorg, Colorb, PlaneName + "Line2");
			Viewer->addLine(Point3, Point4, Colorr, Colorg, Colorb, PlaneName + "Line3");
			Viewer->addLine(Point4, Point1, Colorr, Colorg, Colorb, PlaneName + "Line4");
		}
		Viewer->addText3D(NumberValue, CenterPoint, FontSize, 
			Colorr, Colorg, Colorb, PlaneName + "Text3D");
	}
	   
	//返回[a,b] 之间的随机整数 a b 均大于0
	static int GetRandom(int a, int b)
	{
		//要取得[a,b)的随机整数，使用(rand() % (b-a))+ a; 
		//要取得[a,b]的随机整数，使用(rand() % (b-a+1))+ a; 
		//要取得(a,b]的随机整数，使用(rand() % (b-a))+ a + 1; 

		srand((unsigned)time(NULL));
		if (b - a + 1 != 0)
			return (rand() % (b - a + 1)) + a;
		else
			return a;
	}

	//在点云集Points中 发现 TempPoint 是否存在
	static bool FindPointIsExists(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
		pcl::PointXYZRGB TempPoint)
	{
		for (int i = 0; i < Points->points.size(); i++)
		{
			if ((Points->points[i].x == TempPoint.x) &&
				(Points->points[i].y == TempPoint.y) &&
				(Points->points[i].z == TempPoint.z))
			{
				return true;
			}
		}
		return false;
	}




	static double DirectionOfThreePoints(double X1, double Y1, double Z1,
		double X2, double Y2, double Z2,
		double X3, double Y3, double Z3)
	{
		//// 向量 A(a1, a2, a3)= Point3-Point1 与 B(b1, b2, b3) Point2-Point1
		//// 则 A×B 符合右手系 其差乘后的z值系数(a1*b2-a2*b1)大于0
		//// 返回值大于 0 说明 第三点在 1、2的右侧，注意方向性 
		double a1 = (X3 - X1);
		double a2 = (Y3 - Y1);

		double b1 = (X2 - X1);
		double b2 = (Y2 - Y1);

		return (a1*b2 - a2 * b1);
	}

	//计算特定点到点集的统计数据
	static void StatPointToPtrDis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
		pcl::PointXYZRGB TempPoint,
		double & MaxDis, double & MinDis, double & AvgDis, double & DisVariance)
	{
		MaxDis = 0;
		MinDis = 99999999999;
		AvgDis = 0;
		DisVariance = 0;
		vector<double> Dis;

		VectorBase<double> VectorBaseFloat;

		for (int i = 0; i < Points->points.size(); i++)
		{
			double TempDis = PointDis(
				Points->points[i].x, Points->points[i].y, Points->points[i].z,
				TempPoint.x, TempPoint.y, TempPoint.z);

			Dis.push_back(TempDis);

			if (TempDis > MaxDis)
				MaxDis = TempDis;
			if (TempDis < MinDis)
				MinDis = TempDis;
		}

		DisVariance = VectorBaseFloat.CalcVariances(Dis, AvgDis);
	}

	//返回两个向量的叉乘
	static pcl::PointXYZRGB PointsCrossProduct(pcl::PointXYZRGB A, pcl::PointXYZRGB B)
	{
		pcl::PointXYZRGB Result;

		Result.x = A.y * B.z - B.y * A.z;
		Result.y = A.z * B.x - A.x * B.z;
		Result.z = A.x * B.y - B.x * A.y;

		return Result;
	}

	static pcl::Normal PointsCrossProductToNormal(pcl::PointXYZRGB A, pcl::PointXYZRGB B)
	{
		pcl::Normal Result;

		Result.normal_x = A.y * B.z - B.y * A.z;
		Result.normal_y = A.z * B.x - A.x * B.z;
		Result.normal_z = A.x * B.y - B.x * A.y;

		return Result;
	}

	static pcl::PointXYZRGB PointsCrossProduct(pcl::Normal A, pcl::Normal B)
	{
		pcl::PointXYZRGB Result;

		Result.x = A.normal_y * B.normal_z - B.normal_y * A.normal_z;
		Result.y = A.normal_z * B.normal_x - A.normal_x * B.normal_z;
		Result.z = A.normal_x * B.normal_y - B.normal_x * A.normal_y;

		return Result;
	}

	static pcl::Normal PointsCrossProductToNormal(pcl::Normal A, pcl::Normal B)
	{
		pcl::Normal Result;

		Result.normal_x = A.normal_y * B.normal_z - B.normal_y * A.normal_z;
		Result.normal_y = A.normal_z * B.normal_x - A.normal_x * B.normal_z;
		Result.normal_z = A.normal_x * B.normal_y - B.normal_x * A.normal_y;

		return Result;
	}

	//返回两个向量的点乘
	static double PointsDotProduct(pcl::PointXYZRGB A, pcl::PointXYZRGB B)
	{
		return A.x * B.x + A.y * B.y + A.z * B.z;
	}

	//在XY平面内通过最小二乘法法拟合圆 计算圆心与半径
	static void FittingCircleByLeastSquares(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud,
		double & CenterX, double & CenterY, double & AvgRadius)
	{
		if (Cloud->points.size() < 3)
		{
			return;
		}

		double X1 = 0;
		double Y1 = 0;
		double X2 = 0;
		double Y2 = 0;
		double X3 = 0;
		double Y3 = 0;
		double X1Y1 = 0;
		double X1Y2 = 0;
		double X2Y1 = 0;

		for (int i = 0; i < Cloud->points.size(); i++)
		{
			X1 = X1 + Cloud->points[i].x;
			Y1 = Y1 + Cloud->points[i].y;
			X2 = X2 + Cloud->points[i].x * Cloud->points[i].x;
			Y2 = Y2 + Cloud->points[i].y * Cloud->points[i].y;
			X3 = X3 + Cloud->points[i].x * Cloud->points[i].x * Cloud->points[i].x;
			Y3 = Y3 + Cloud->points[i].y * Cloud->points[i].y * Cloud->points[i].y;
			X1Y1 = X1Y1 + Cloud->points[i].x * Cloud->points[i].y;
			X1Y2 = X1Y2 + Cloud->points[i].x * Cloud->points[i].y * Cloud->points[i].y;
			X2Y1 = X2Y1 + Cloud->points[i].x * Cloud->points[i].x * Cloud->points[i].y;
		}

		double C, D, E, G, H, N;
		double a, b, c;
		N = Cloud->points.size();
		C = N * X2 - X1 * X1;
		D = N * X1Y1 - X1 * Y1;
		E = N * X3 + N * X1Y2 - (X2 + Y2)*X1;
		G = N * Y2 - Y1 * Y1;
		H = N * X2Y1 + N * Y3 - (X2 + Y2)*Y1;
		a = (H*D - E * G) / (C*G - D * D);
		b = (H*C - E * D) / (D*D - G * C);
		c = -(a*X1 + b * Y1 + X2 + Y2) / N;

		CenterX = a / (-2);
		CenterY = b / (-2);
		AvgRadius = sqrt(a*a + b * b - 4 * c) / 2;	 //这是标准最小二乘法定义的半径	
	}

	static pcl::PointXYZRGB PointsMinus(pcl::PointXYZRGB A, pcl::PointXYZRGB B)
	{
		pcl::PointXYZRGB Result;
		Result.x = A.x - B.x;
		Result.y = A.y - B.y;
		Result.z = A.z - B.z;
		return Result;
	}

	static pcl::PointXYZRGB PointsAdd(pcl::PointXYZRGB A, pcl::PointXYZRGB B)
	{
		pcl::PointXYZRGB Result;
		Result.x = A.x + B.x;
		Result.y = A.y + B.y;
		Result.z = A.z + B.z;
		return Result;
	}

	static pcl::PointXYZRGB PointsMutile(pcl::PointXYZRGB A, double K)
	{
		A.x = A.x * K;
		A.y = A.y * K;
		A.z = A.z * K;
		return A;
	}

	static int PointIsInCloud(pcl::PointXYZRGB Point, pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPtr)
	{
		int Index = -1;
		for (int i = 0; i < CloudPtr->points.size(); i++)
		{
			if ((CloudPtr->points[i].x == Point.x)
				&& (CloudPtr->points[i].y == Point.y)
				&& (CloudPtr->points[i].z == Point.z))
			{
				Index = i;
				break;
			}
		}
		return Index;
	}

	//删除点云集中的重复点
	static void GetDistinctPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPtr)
	{
		//cout << "正在过滤点集中重复的点！点集个数:" << CloudPtr->points.size() << endl;
		int i = 0;
		while (i < CloudPtr->points.size())
		{
			int j = i + 1;
			while (j < CloudPtr->points.size())
			{
				if ((abs(CloudPtr->points[i].x - CloudPtr->points[j].x) < EPSM6)
					&& (abs(CloudPtr->points[i].y - CloudPtr->points[j].y) < EPSM6)
					&& (abs(CloudPtr->points[i].z - CloudPtr->points[j].z) < EPSM6) )
				{
					CloudPtr->points.erase(CloudPtr->points.begin() + j);
					//break; 可能有多个重复点
				}
				else
					j++;
			}
			i++;
		}
	}

	//删除点云集中的重复点
	static void GetDistinctPoints(pcl::PointCloud<PointXYZRGBIndex>::Ptr CloudPtr)
	{
		cout << "正在过滤点集中重复的点！点集个数:" << CloudPtr->points.size() << endl;
		int i = 0;
		while (i < CloudPtr->points.size())
		{
			int j = i + 1;
			while (j < CloudPtr->points.size())
			{
				if ((abs(CloudPtr->points[i].x - CloudPtr->points[j].x) < EPSM6)
					&& (abs(CloudPtr->points[i].y - CloudPtr->points[j].y) < EPSM6)
					&& (abs(CloudPtr->points[i].z - CloudPtr->points[j].z) < EPSM6))
				{
					CloudPtr->points.erase(CloudPtr->points.begin() + j);
					//break; 可能有多个重复点
				}
				else
					j++;
			}
			i++;
		}
	}

	//删除点云集中的重复点
	static void GetDistinctPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr)
	{
		cout << "正在过滤点集中重复的点！点集个数:" << CloudPtr->points.size() << endl;
		int i = 0;
		while (i < CloudPtr->points.size())
		{
			int j = i + 1;
			while (j < CloudPtr->points.size())
			{
				if ((abs(CloudPtr->points[i].x - CloudPtr->points[j].x) < EPSM6)
					&& (abs(CloudPtr->points[i].y - CloudPtr->points[j].y) < EPSM6)
					&& (abs(CloudPtr->points[i].z - CloudPtr->points[j].z) < EPSM6))
				{
					CloudPtr->points.erase(CloudPtr->points.begin() + j);
					//break; 可能有多个重复点
				}
				else
					j++;
			}
			i++;
		}
	}

	static void GetDistinctPointsInXY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPtr)
	{
		int i = 0;
		while (i < CloudPtr->points.size())
		{
			int j = i + 1;
			while (j < CloudPtr->points.size())
			{
				if ((abs(CloudPtr->points[i].x - CloudPtr->points[j].x) < EPSM6)
					&& (abs(CloudPtr->points[i].y - CloudPtr->points[j].y) < EPSM6))
				{
					CloudPtr->points.erase(CloudPtr->points.begin() + j);
					//break; 可能有多个重复点
				}
				else
					j++;
			}
			i++;
		}
	}


	//处理点云凸包集中距离比较近的数据，也即精简凸包集 在相邻数据点间操作
	static void SimplifyConvexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvexHullCloud,
		double DisValue = 0.01)
	{
		CalcBase<double> CalcBaseFloat;
		int i = 0;
		while (i < ConvexHullCloud->points.size() - 1)
		{
			int j = i + 1;
			while (j < ConvexHullCloud->points.size())
			{
				double Dis = PointDis(ConvexHullCloud->points[i].x, ConvexHullCloud->points[i].y, ConvexHullCloud->points[i].z,
					ConvexHullCloud->points[j].x, ConvexHullCloud->points[j].y, ConvexHullCloud->points[j].z);
				if (Dis < DisValue)
				{
					ConvexHullCloud->points.erase(ConvexHullCloud->points.begin() + j);
				}
				else
					j++;
			}
			i++;
		}
	}

	//返回向量的模
	static double PointModel(pcl::PointXYZRGB Point)
	{
		return sqrt(Point.x * Point.x + Point.y * Point.y + Point.z * Point.z);
	}


	static void HomogenizationSplineInterPolatingPoints(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr QPoints, int multipleValue = 2)
	{
		double AvgDis = 0;
		CalcBase<double> CalcBaseFloat;
		vector<double> VecDis;

		for (int i = 0; i < QPoints->points.size() - 1; i++)
		{
			double TempDis = PointDis(QPoints->points[i].x, QPoints->points[i].y, QPoints->points[i].z,
				QPoints->points[i + 1].x, QPoints->points[i + 1].y, QPoints->points[i + 1].z);
			VecDis.push_back(TempDis);
			AvgDis = AvgDis + TempDis / QPoints->points.size();
		}

		int AddNumber = 0;
		for (int i = 0; i < VecDis.size(); i++)
		{
			int K = VecDis[i] / AvgDis;

			if (K > multipleValue)
			{
				double XStep = (QPoints->points[i + AddNumber + 1].x - QPoints->points[i + AddNumber].x) / K;
				double YStep = (QPoints->points[i + AddNumber + 1].y - QPoints->points[i + AddNumber].y) / K;
				double ZStep = (QPoints->points[i + AddNumber + 1].z - QPoints->points[i + AddNumber].z) / K;

				for (int j = 1; j < K; j++)
				{
					pcl::PointXYZRGB NewPoint;
					NewPoint.x = QPoints->points[i + AddNumber].x + XStep;
					NewPoint.y = QPoints->points[i + AddNumber].y + YStep;
					NewPoint.z = QPoints->points[i + AddNumber].z + ZStep;
					QPoints->insert(QPoints->begin() + i + AddNumber + 1, NewPoint);
					AddNumber = AddNumber + 1;
				}
			}
		}
	}

	//判断当前点的切线值是不是使此点为凸  二维空间
	static bool PointIsConvexHull(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, int Index, pcl::PointXYZRGB PointTangent)
	{
		pcl::PointXYZRGB CenterPoints;
		CenterPoints.x = 0;
		CenterPoints.y = 0;
		CenterPoints.z = 0;

		for (int i = 0; i < Points->points.size(); i++)
		{
			CenterPoints.x = CenterPoints.x + Points->points[i].x / Points->points.size();
			CenterPoints.y = CenterPoints.y + Points->points[i].y / Points->points.size();
			CenterPoints.z = CenterPoints.z + Points->points[i].z / Points->points.size();
		}
		double a = PointTangent.y / PointTangent.x;
		double b = Points->points[Index].y - a * Points->points[Index].x;

		double Symbol = CenterPoints.y - a * CenterPoints.x - b;

		for (int i = 0; i < Points->points.size(); i++)
		{
			double TempSymbol = Points->points[i].y - a * Points->points[i].x - b;
			if (Symbol * TempSymbol < 0)
			{
				return false;
			}
		}
		return true;
	}

	template<typename PointNT>
	static PointNT GetPointAlongLine(
		pcl::PointXYZRGB Normal, PointNT APoint, double Dis, bool Alonged = true)
	{
		PointBase::PointNormalized(Normal);

		PointNT ResultPoint0, ResultPoint1;
		double t = Dis / sqrt(pow(Normal.x, 2) + pow(Normal.y, 2) + pow(Normal.z, 2));

		ResultPoint0.x = APoint.x + Normal.x*t;
		ResultPoint0.y = APoint.y + Normal.y*t;
		ResultPoint0.z = APoint.z + Normal.z*t;

		ResultPoint1.x = APoint.x - Normal.x*t;
		ResultPoint1.y = APoint.y - Normal.y*t;
		ResultPoint1.z = APoint.z - Normal.z*t;

		if (Alonged)
		{
			if ((ResultPoint0.z - APoint.z) * Normal.z > 0
				|| (ResultPoint0.x - APoint.x) * Normal.x > 0 || (ResultPoint0.y - APoint.y) * Normal.y > 0)
				return ResultPoint0;
			else
				return ResultPoint1;
		}
		else
		{
			if ((ResultPoint0.z - APoint.z) * Normal.z > 0
				|| (ResultPoint0.x - APoint.x) * Normal.x > 0 || (ResultPoint0.y - APoint.y) * Normal.y > 0)
				return ResultPoint1;
			else
				return ResultPoint0;
		}
	}

	//画以BasePoint为起点，方向向量为 DirectionPoint 至点PointB间的一段长度为 LineLength的直线
	static void ShowDirection(
		boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointXYZRGB BasePoint, pcl::PointXYZRGB DirectionPoint, double LineLength)
	{
		//pcl::PointXYZRGB PointB = PointBase::PointsAdd(BasePoint, DirectionPoint);		

		pcl::PointXYZRGB PointB = GetPointAlongLine(DirectionPoint, BasePoint, LineLength);

		DrawExtentedLineBetweenTwoPoints(Viewer, BasePoint, PointB, LineLength);
	}	
	   
	//画以BasePoint为起点，至点PointB间的一段长度为 LineLength的直线
	static void DrawExtentedLineBetweenTwoPoints(
		boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointXYZRGB BasePoint, pcl::PointXYZRGB PointB, double LineLength)
	{

		CalcBase<int> CalcBaseInt;
		CalcBase<double> CalcBaseFloat;

		pcl::PointXYZRGB TangentLinePoint;

		TangentLinePoint.x = PointB.x - BasePoint.x;
		TangentLinePoint.y = PointB.y - BasePoint.y;
		TangentLinePoint.z = PointB.z - BasePoint.z;

		pcl::PointXYZRGB DerivativePoint;

		double L = sqrt(LineLength*LineLength /
			(TangentLinePoint.x * TangentLinePoint.x
				+ TangentLinePoint.y * TangentLinePoint.y +
				TangentLinePoint.z * TangentLinePoint.z));

		DerivativePoint.x = BasePoint.x + L * TangentLinePoint.x;
		DerivativePoint.y = BasePoint.y + L * TangentLinePoint.y;
		DerivativePoint.z = BasePoint.z + L * TangentLinePoint.z;

		BasePoint.x = BasePoint.x - L * TangentLinePoint.x;
		BasePoint.y = BasePoint.y - L * TangentLinePoint.y;
		BasePoint.z = BasePoint.z - L * TangentLinePoint.z;

		Viewer->addLine(BasePoint, DerivativePoint, 0, 255, 0,
			"Line_" + CalcBaseFloat.ConvertToString(clock())
			+ CalcBaseInt.ConvertToString(31) + "_"
			+ CalcBaseInt.ConvertToString(1));
	}

	//static double CalcPointsDis(pcl::PointXYZRGB PointA, pcl::PointXYZRGB PointB, bool Is2D = false)
	//{
	//	CalcBase<double> CalcBaseFloat;
	//	return PointDis(PointA, PointB, Is2D);
	//}

	//计算点集中相邻点距离的方差 2016.01.05
	static double VarianceOfPointsDis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points)
	{
		double Variance;

		vector<double> PointsDis;
		for (int i = 0; i < Points->points.size() - 1; i++)
		{
			PointsDis.push_back(PointDis(Points->points[i], Points->points[i + 1]));
		}
		VectorBase<double> VectorBasedouble;

		return VectorBasedouble.CalcVariances(PointsDis);
	}

	static bool CheckLineOfPointsConvexPropertity(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
		int LineIndexA, int LineIndexB, pcl::PointXYZRGB CenterPoint, double Error = 0.0000001)
	{
		double a, b, Symbol, TempSymbol;
		if ((Points->points[LineIndexB].x - Points->points[LineIndexA].x) != 0)
		{
			a = (Points->points[LineIndexB].y - Points->points[LineIndexA].y) /
				(Points->points[LineIndexB].x - Points->points[LineIndexA].x);

			b = Points->points[LineIndexB].y - Points->points[LineIndexB].x * a;
			Symbol = CenterPoint.y - a * CenterPoint.x - b;
		}
		else
		{
			a = 0;
			Symbol = CenterPoint.x - Points->points[LineIndexA].x;
		}

		for (int i = 0; i < Points->points.size(); i++)
		{
			if (a != 0)
				TempSymbol = Points->points[i].y - a * Points->points[i].x - b;
			else
				TempSymbol = Points->points[i].x - Points->points[LineIndexA].x;

			if (Symbol * TempSymbol < 0 && abs(TempSymbol) > Error) return false;
		}
		return true;
	}

	static double CheckPolygonOfPointsConvexPropertity(
		boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, bool IsShowed = false, double Error = 0.0000001)
	{
		double Number = 0;
		pcl::PointXYZRGB CenterPoint;
		CenterPoint.x = 0;
		CenterPoint.y = 0;
		CenterPoint.z = 0;
		for (int i = 0; i < Points->points.size(); i++)
		{
			CenterPoint.x = CenterPoint.x + Points->points[i].x / Points->points.size();
			CenterPoint.y = CenterPoint.y + Points->points[i].y / Points->points.size();
			CenterPoint.z = CenterPoint.z + Points->points[i].z / Points->points.size();
		}
		CalcBase<double> CalcBaseFloat;
		for (int i = 0; i < Points->points.size() - 1; i++)
		{
			if (!CheckLineOfPointsConvexPropertity(Points, i, i + 1, CenterPoint))
			{
				Number++;
				if (IsShowed)
				{
					Viewer->addLine(Points->points[i], Points->points[i + 1], 255, 0, 0,
						CalcBaseFloat.ConvertToString(clock()));
				}
			}
		}

		if (!CheckLineOfPointsConvexPropertity(Points, 0, Points->points.size() - 1, CenterPoint))
		{
			Number++;
			if (IsShowed)
			{
				Viewer->addLine(Points->points[0], Points->points[Points->points.size() - 1], 255, 0, 0,
					CalcBaseFloat.ConvertToString(clock()));
			}
		}

		return Number / Points->points.size();
	}

	static PointXYZRGBIndex PointRGBToPointRGBIndex(pcl::PointXYZRGB Point)
	{
		PointXYZRGBIndex Result;
		Result.x = Point.x;
		Result.y = Point.y;
		Result.z = Point.z;
		return Result;
	}

	static pcl::PointXYZRGB PointRGBIndexToPointRGB(PointXYZRGBIndex Point)
	{
		pcl::PointXYZRGB Result;
		Result.x = Point.x;
		Result.y = Point.y;
		Result.z = Point.z;
		return Result;
	}

	//在给定法向量NormalPoint 时，点云集的最高和最低的两个片面所在位置处的最高点与最低点
	static void GetMaxPointdAndMinPointOfPlane(pcl::PointCloud<PointXYZRGBIndex>::Ptr Points,
		pcl::PointXYZRGB NormalPoint, PointXYZRGBIndex & MaxPoint, PointXYZRGBIndex & MinPoint)
	{
		double Maxd, Mind;
		if (Points->points.size() == 0)
			return;
		Maxd = -(NormalPoint.x * Points->points[0].x + NormalPoint.y * Points->points[0].y + NormalPoint.z * Points->points[0].z);
		Mind = Maxd;

		MaxPoint.x = Points->points[0].x;
		MaxPoint.y = Points->points[0].y;
		MaxPoint.z = Points->points[0].z;
		MinPoint = MaxPoint;

		for (int i = 1; i < Points->points.size(); i++)
		{
			double Tempd = -(NormalPoint.x * Points->points[i].x + NormalPoint.y * Points->points[i].y + NormalPoint.z * Points->points[i].z);
			if (Tempd > Maxd)
			{
				Maxd = Tempd;
				MaxPoint.x = Points->points[i].x;
				MaxPoint.y = Points->points[i].y;
				MaxPoint.z = Points->points[i].z;
			}
			if (Tempd < Mind)
			{
				Mind = Tempd;
				MinPoint.x = Points->points[i].x;
				MinPoint.y = Points->points[i].y;
				MinPoint.z = Points->points[i].z;
			}
		}
	}


	static void GetMaxAndMinPoint(pcl::PointCloud<PointXYZRGBIndex>::Ptr Points,
		string Axis, PointXYZRGBIndex & MaxPoint, PointXYZRGBIndex & MinPoint)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr TempPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		PointBase::PointXYZRGBIndexToPointXYZRGB(Points, TempPoints);

		pcl::PointXYZRGB MaxPoint0, MinPoint0;

		GetMaxAndMinPoint(TempPoints, Axis, MaxPoint0, MinPoint0);
		MaxPoint = PointRGBToPointRGBIndex(MaxPoint0);
		MinPoint = PointRGBToPointRGBIndex(MinPoint0);
	}
	//
	static void GetMaxAndMinPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points,
		string Axis, pcl::PointXYZRGB & MaxPoint, pcl::PointXYZRGB & MinPoint)
	{
		double MinValue, MaxValue;

		MaxPoint = Points->points[0];
		MinPoint = MaxPoint;

		if (Axis == "X")
		{
			MaxValue = MaxPoint.x;
		}
		else if (Axis == "Y")
		{
			MaxValue = MaxPoint.y;
		}
		else if (Axis == "Z")
		{
			MaxValue = MaxPoint.z;
		}
		MinValue = MaxValue;

		for (int i = 1; i < Points->points.size(); i++)
		{
			if (Axis == "X")
			{
				if (Points->points[i].x > MaxValue)
				{
					MaxValue = Points->points[i].x;
					MaxPoint = Points->points[i];
				}
				if (Points->points[i].x < MinValue)
				{
					MinValue = Points->points[i].x;
					MinPoint = Points->points[i];
				}
			}
			else if (Axis == "Y")
			{
				if (Points->points[i].y > MaxValue)
				{
					MaxValue = Points->points[i].y;
					MaxPoint = Points->points[i];
				}
				if (Points->points[i].y < MinValue)
				{
					MinValue = Points->points[i].y;
					MinPoint = Points->points[i];
				}
			}
			else if (Axis == "Z")
			{
				if (Points->points[i].z > MaxValue)
				{
					MaxValue = Points->points[i].z;
					MaxPoint = Points->points[i];
				}
				if (Points->points[i].z < MinValue)
				{
					MinValue = Points->points[i].z;
					MinPoint = Points->points[i];
				}
			}
		}
	}

	//2016.01.15 因最近邻获取的邻域点有时不包含自己，做一个是否包含自己的检测
	// 被 VectorBase中 的FindIndexInVector 取代
	static bool FindSelfInNegighbourIndex(vector<int> NeighbourIndex, int SelfIndex)
	{
		bool Find = false;
		for (int i = 0; i < NeighbourIndex.size(); i++)
		{
			if (NeighbourIndex[i] == SelfIndex)
			{
				Find = true;
				break;
			}
		}
		return Find;
	}

	static long double AreaOfThreePointsIn2D(double Point1x, double Point1y,
		double Point2x, double Point2y, double Point3x, double Point3y)
	{
		//返回 三角形面积，如果 > 0 是逆时针排列，
		 //  Dim Area As Double //二维平面
		 //Area = p1(1) * P2(2) + P2(1) * P3(2) + P3(1) * p1(2)
		 //Area = Area - p1(1) * P3(2) - P2(1) * p1(2) - P3(1) * P2(2)  
		////2015.07.22 此处的计算精度与matlab有差异 matlab 结果是极低的负值，而此处还是正值。

		long double Area;
		Area = Point1x * Point2y + Point2x * Point3y + Point3x * Point1y
			- Point1x * Point3y - Point2x * Point1y - Point3x * Point2y;
		return Area / 2;
	}


	///2016.01.15 判断一个点是否在三角形的内部 或者在边上 
//=0 是在边上，=-1是在外部  =1在内部 包括 三维空间中 点A在三角形所在的平面的投影是否会落在三角形中
	static int PointIsInTriange2D(pcl::PointXYZRGB TriangleA,
		pcl::PointXYZRGB TriangleB,
		pcl::PointXYZRGB TriangleC,
		pcl::PointXYZRGB A)
	{
		double Area1 = AreaOfThreePointsIn2D(A.x, A.y, TriangleA.x, TriangleA.y, TriangleB.x, TriangleB.y);
		double Area2 = AreaOfThreePointsIn2D(A.x, A.y, TriangleB.x, TriangleB.y, TriangleC.x, TriangleC.y);
		double Area3 = AreaOfThreePointsIn2D(A.x, A.y, TriangleC.x, TriangleC.y, TriangleA.x, TriangleA.y);
		if (Area1 == 0 || Area2 == 0 || Area3 == 0)
			return 0;
		else if (Area1 * Area2 < 0 || Area1 * Area3 < 0 || Area2 * Area3 < 0)
			return -1;
		else
			return 1;
	}

	static void ShowPolygonMesh(boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Points, pcl::PolygonMesh & Mesh, bool ShowLine = false)
	{
		pcl::PCLPointCloud2 TempPointCloud2;
		Points->width = Points->points.size();
		Points->height = 1;
		pcl::toPCLPointCloud2(*Points, TempPointCloud2);
		Mesh.header = Points->header;
		Mesh.cloud = TempPointCloud2;

		if (!ShowLine)
			Viewer->addPolygonMesh(Mesh);
		else
			Viewer->addPolylineFromPolygonMesh(Mesh);
	
	}

}; //The end for Class PointBase


#endif;