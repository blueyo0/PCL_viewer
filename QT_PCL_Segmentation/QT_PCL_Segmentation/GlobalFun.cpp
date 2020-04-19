#include "GlobalFun.h"

#include <algorithm>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
//#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；



double GlobalFun::computeDirectionalityDegree(vector<PointXYZ> diff) {
	if (diff.size() < 1) return 0;
	Matrix3d mm;
	for (PointXYZ d : diff) {
		Vector3d v(d.x, d.y, d.z);
		mm += v * v.adjoint();
	}
	Vector3cd eigens = mm.eigenvalues();
	Vector3d eigenVec(eigens(0).real(), eigens(1).real(), eigens(2).real());
	return eigenVec.maxCoeff() / eigens.sum().real();
}

int GlobalFun::synInfoWithCloud(vector<SamplePoint> &info, PointCloud<PointXYZ>::Ptr cloud) {
	// 先判断cloud 和 info大小是否匹配
	if (info.size() == 0) info.resize(cloud->points.size());
	else if (cloud->points.size() != info.size()) return -1;
	int index = 0;
	for (PointXYZ pt : cloud->points) {
		info[index].setPos(pt);
		index++;
	}
	return 0;
}

double GlobalFun::weight(float r, double h) {
	return exp(-4 * (r*r) / (h*h));
}


PointXYZ GlobalFun::nextPos(SamplePoint xi, pi::PcPtr xc, pi::PcPtr qc, 
							vector<double> density, double mu, bool need_density) {
	Vector3d up1(0,0,0);
	double down1 = 0.0;
	int i = 0;
	vector<int> o_indics = xi.getOriginalIndics();
	for (double a : xi.alpha) {
		if (need_density) {
			a *= density[o_indics[i]];
		}
		Vector3d q(qc->points[i].x, qc->points[i].y, qc->points[i].z);
		up1 += a * q;
		down1 += a;
		i++;
	}

	Vector3d up2(0, 0, 0);
	double down2 = 0.0;
	int j = 0;
	for (double b : xi.beta) {
		Vector3d xii(
			xi.pos.x - xc->points[j].x,
			xi.pos.y - xc->points[j].y,
			xi.pos.z - xc->points[j].z
		);
		up2 += b * xii;
		down2 += b;
		j++;
	}
	Vector3d res(0, 0, 0);
	if (down1 != 0) {
		res += up1 / down1;
	}
	if (down2 != 0) {
		res += mu * up2 / down2;
		// res += mu * xi.getSigma()*up2 / down2;
	}
	for (int i = 0; i < 3; ++i) {
		if (!isfinite(res(i))) 
			res(i) = 1.2;
		else if (res(i) > 1.2) 
			res(i) = 1.2;
		else if (res(i) < -1.2) 
			res(i) = -1.2;
	}
	return PointXYZ(res(0), res(1), res(2));
}


void GlobalFun::pcd_ply(std::string input) {
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPCDFile(input.substr(0, input.length() - 4) + ".pcd", cloud) < 0)
	{
		cout << "Error: cannot load the PCD file!!!" << endl;
		return;
	}
	pcl::PLYWriter writer;
	writer.write(input.substr(0, input.length() - 4) + ".ply", cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
}

void GlobalFun::obj_pcd(std::string input) {
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFileOBJ(input.substr(0, input.length() - 4) + ".obj", mesh);
	pcl::PointCloud<pcl::PointXYZ>::Ptr converterCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *converterCloud);
	pcl::io::savePCDFileASCII(input.substr(0, input.length() - 4) + ".pcd", *converterCloud);
}

void GlobalFun::off_obj(std::string input)
{
	/*函数说明：读取off文件
	 * */
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
PointCloud<PointXYZ>::Ptr GlobalFun::randomSampling(PointCloud<PointXYZ>::Ptr inCloud, int num) 
{
	PointCloud<PointXYZ>::Ptr sampleCloud(new PointCloud<PointXYZ>);
	int size = inCloud->points.size();
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

PointCloud<PointXYZ>::Ptr GlobalFun::readOFF(std::string filename)
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


PointXYZ GlobalFun::median(PointCloud<PointXYZ>::Ptr inCloud)
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