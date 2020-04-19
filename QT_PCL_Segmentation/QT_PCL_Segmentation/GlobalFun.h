#pragma once
#include <vector>
#include <algorithm>

#include <QString>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "PointInfo.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

namespace GlobalFun {
	void pcd_ply(std::string input) {
		pcl::PCLPointCloud2 cloud;
		if (pcl::io::loadPCDFile(input.substr(0, input.length() - 4) + ".pcd", cloud) < 0)
		{
			cout << "Error: cannot load the PCD file!!!" << endl;
			return;
		}
		pcl::PLYWriter writer;
		writer.write(input.substr(0, input.length() - 4) + ".ply", cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
	}

	void obj_pcd(std::string input) {
		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFileOBJ(input.substr(0, input.length() - 4) + ".obj", mesh);
		pcl::PointCloud<pcl::PointXYZ>::Ptr converterCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(mesh.cloud, *converterCloud);
		pcl::io::savePCDFileASCII(input.substr(0, input.length() - 4) + ".pcd", *converterCloud);
	}

	void off_obj(std::string input)
	{
		/*函数说明：读取off文件*/
		float vertex[10000][3];
		int surface[10000][3];
		char k;
		int vertex_num;
		int surface_num;
		int other_num;
		int i, j;
		int s;
		ifstream fin;
		ofstream fout;
		fin.open(input);
		while (fin.fail())
		{
			cout << "Fail to open the file!" << endl;
			exit(1);
		}
		fout.open(input.substr(0, input.length() - 4) + ".obj");
		while (fout.fail())
		{
			cout << "Fail to open the fail!" << endl;
			exit(1);
		}
		do
		{
			cout << fin.get();
		} while (fin.get() != '\n');
		fin >> vertex_num >> surface_num >> other_num;
		cout << vertex_num;
		for (i = 0; i < vertex_num; i++)
		{
			for (j = 0; j < 3; j++)
			{
				fin >> vertex[i][j];
			}
		}
		for (i = 0; i < surface_num; i++)
		{
			fin >> s;
			cout << s << endl;
			for (j = 0; j < 3; j++)
			{
				fin >> surface[i][j];
			}
		}

		for (i = 0; i < vertex_num; i++)
		{
			fout.put('v');
			fout.put(' ');
			for (j = 0; j < 3; j++)
			{
				fout << vertex[i][j];
				fout.put(' ');
			}
			fout.put('\n');

		}
		fout.put('\n'); //注意控制换行

		for (i = 0; i < surface_num; i++)
		{
			fout.put('f');
			fout.put(' ');
			for (j = 0; j < 3; j++)
			{
				fout << (surface[i][j] + 1);
				fout.put(' ');
			}
			fout.put('\n');
		}
		fin.close();
		fout.close();
		//cout << "end" << endl;
	}

	// 随机采样函数
	PointCloud<PointXYZ>::Ptr randomSampling(PointCloud<PointXYZ>::Ptr inCloud, int num)
	{
		PointCloud<PointXYZ>::Ptr sampleCloud(new PointCloud<PointXYZ>);
		int size = inCloud->points.size();
		if (num >= size) return inCloud;
		vector<int> nCard(size, 0);
		for (int i = 0; i < size; ++i) {
			nCard[i] = i;
		}
		random_shuffle(nCard.begin(), nCard.begin() + size);
		for (int i = 0; i < num; ++i) {
			sampleCloud->points.push_back(inCloud->points[nCard[i]]);
		}
		return sampleCloud;
	}

	PointCloud<PointXYZ>::Ptr readOFF(std::string filename)
	{
		PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
		ifstream in(filename);
		char buffer[8];
		char bufferChar;
		int branchLenBuffer;
		float pos[6];

		int num = 0;
		in.get(bufferChar);//NOFF / OFF
		if (bufferChar == 'N') { in.get(buffer, 4); in >> num >> branchLenBuffer; }//NOFF
		else { in.get(buffer, 3); in >> num >> branchLenBuffer >> branchLenBuffer; }//OFF

		for (int i = 0; i < num; i++)
		{
			in >> pos[0] >> pos[1] >> pos[2];
			if (bufferChar == 'N')
				in >> pos[3] >> pos[4] >> pos[5];
			cloud->points.push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
		}
		return cloud;
	}


	PointXYZ median(PointCloud<PointXYZ>::Ptr inCloud)
	{
		pcl::PointXYZ center(0, 0, 0);
		// Generate the data
		for (size_t i = 0; i < inCloud->points.size(); ++i)
		{
			center.x += inCloud->points[i].x;
			center.y += inCloud->points[i].y;
			center.z += inCloud->points[i].z;
		}
		center.x /= inCloud->points.size();
		center.y /= inCloud->points.size();
		center.z /= inCloud->points.size();
		return center;
	}
	bool isDigitString(const QString& src) {
		const string s = src.toStdString();
		for (char c : s) {
			if (!isdigit(c) && c!='.') {
				return false;
			}
		}
		return true;
	}

	// black - blue - green - yellow - red
	vector<pi::RGB> color_map = { {0,0,0},{0,0,255},{0,255,0},{255,255,0},{255,0,0} };

	pi::RGB getHotMapColor(double val) {
		pi::RGB c1, c2;
		double percent = 0.0;
		if (val >= 0.75) {
			c1 = color_map[3];
			c2 = color_map[4];
			percent = (val - 0.75) / 0.25;
		}
		else if(val>=0.50) {
			c1 = color_map[2];
			c2 = color_map[3];
			percent = (val - 0.50) / 0.25;
		}
		else if (val >= 0.25) {
			c1 = color_map[1];
			c2 = color_map[2];
			percent = (val - 0.25) / 0.25;
		}
		else {
			c1 = color_map[0];
			c2 = color_map[1];
			percent = val / 0.25;
		}
		percent = (percent > 1.0) ? 1.0 : percent;
		return {
			int(c1.r*(1.0 - percent) + c2.r*percent),
			int(c1.g*(1.0 - percent) + c2.g*percent),
			int(c1.b*(1.0 - percent) + c2.b*percent)
		};
	}


	/*vector<int> BHG(PointCloud<PointXYZ>::Ptr inCloud) {
		vector<int> res;
		return res;
	}*/
	//vector<PointCloud<PointXYZ>::Ptr> getSegClouds(PointCloud<PointXYZ>::Ptr, vector<int>);
	//double disBetweenSkelAndPt(Skeleton, PointXYZ);

}


