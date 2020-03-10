#pragma once

#include <QtWidgets/QMainWindow>
#include <QResizeEvent>
#include "ui_QT_PCL_Segmentation.h"

#include <vector>
#include <windows.h>
#include <string>
#include <fstream>
#include <algorithm>

#include "GlobalFun.h"
#include "PointInfo.h"
#include "SamplePoint.h"
#include "ParameterMgr.h"

#include "PointCloudAlgorithm.h"

#include <pcl/common/projection_matrix.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；

using namespace pi;
using namespace std;
using namespace pcl;

class QT_PCL_Segmentation : public QMainWindow
{
	Q_OBJECT

public:
	QT_PCL_Segmentation(QWidget *parent = Q_NULLPTR);

private:
	Ui::QT_PCL_SegmentationClass ui;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	ParameterMgr *paraMgr;
	PointCloudAlgorithm *algorithm;

private:
	PointCloud<PointXYZ>::Ptr originCloud;
	PointCloud<PointXYZ>::Ptr sampleCloud;
	PointCloud<PointXYZ>::Ptr cloudfiltered; 
	PointCloud<Normal>::Ptr normalCloud;

	string cloudPath;
	double skelScale;
	pcl::PointXYZ midPoint;
	int colorCloudIndex;
	float vertex[10000][3];
	int surface[10000][3];


	/*
	std::vector<double> density;
	std::vector<int> tag;

	std::vector<SamplePoint> xInfo;

	pcl::PointCloud<pcl::PointXYZ>::Ptr skelCloud;
	int skelFlag;
	int skelIndex;
	double skelSize;
	int branchNum;
	std::vector<int> branchLen;
	std::string modelSkelName;*/

public:
	//QSize viewSize(0,0);
	//virtual void resizeEvent(QResizeEvent *event);
	//bool BayesTest(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
	//void connectSkel(int i,int j, pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
	//bool colorFlag;
	//double kmeansRadius,c;//distance衰减函数的参数
	//void correctCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
	//double distance(pcl::PointXYZ a, pcl::PointXYZ b, int model = 1);
	//pcl::PointXYZ median(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);

	void initialVtkWidget();
	PointCloud<PointXYZ>::Ptr normalize(PointCloud<PointXYZ>::Ptr inCloud, bool divMode = true);
	void offReader(string filename);
	void computeNormal();
	void saveNoff(string filename);
	void off_obj(string input);
	void downSample(string path);
	void displaySampleCloud(PointCloud<PointXYZ>::Ptr);
	void displaySelectedCloud(const PointCloud<PointXYZ>::Ptr, const pi::RGB, double, string);

	/*void updateDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr, double);
	void updateALLNeighbors(PcPtr qc, PcPtr xc, double radius, vector<SamplePoint> &info);
	void updateALLAlphaAndBeta(vector<SamplePoint> &info, double h);
	void computeALLDirectionalityDegree();
	double computeDirectionalityDegree(int index);
	void updateALLInfo(double h);
	void updateXPos(double h, double mu);*/

	//void l1_median();




private slots:
	void onUpdate();
	void showDemo(); // 弹出messageBox和InfoText输出,用于测试文本框和软件是否正常运行
	void showPCL();  // 随机生成点云并显示，用于测试点云显示功能
	void colorByAxis(); // 按坐标给点云上色，用于测试点云彩色显示功能
	void drawLine(); // 从点云的[0]到[5]画线，用于测试骨架生成核心功能

	void onOpen();   // 打开文件函数，支持pcd，ply
	void onOpenTxt();// 打开txt文件
	void onOpenOff(); // 打开off文件

	void off_ply();  // off文件转ply，支持off，noff
	void color(pcl::PointCloud<pcl::PointXYZ>::Ptr,int,int,int); // 点云上色函数

	//void segmentation(); // pcl库的min-cut 
	//void kmeans();

	void onL1();
	void onMoving();

	void noise();
	void outlier();

	void clearPointCloud();
	void resetPointCloud();

	void onRandomSample();
	void onDownSample();
	void onRandomMissing();

	void onSaveNoff();
	void onSavePLY();


	//void clustering(int num);
	//void normalizeOfSkel();
	//bool skelParam(std::string params, int mode=1);
	//void drawSkel();
	//void reDrawSkel();
	//void BayesSkel();
	//void KNNsmooth();
};
