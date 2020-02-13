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

class QT_PCL_Segmentation : public QMainWindow
{
	Q_OBJECT

public:
	QT_PCL_Segmentation(QWidget *parent = Q_NULLPTR);

private:
	Ui::QT_PCL_SegmentationClass ui;

	pcl::PointCloud<pcl::Normal>::Ptr normalCloud;

	std::string cloudPath;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector<int> tag;
	pcl::PointXYZ midPoint;
	double skelScale;

	pcl::PointCloud<pcl::PointXYZ>::Ptr xCloud;
	std::vector<SamplePoint> xInfo;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfiltered;
	float vertex[10000][3];
	int surface[10000][3];
	void off_obj(std::string input);

	pcl::PointCloud<pcl::PointXYZ>::Ptr skelCloud;
	int skelFlag;
	int skelIndex;
	double skelSize;
	int branchNum;
	std::vector<int> branchLen;
	std::string modelSkelName;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	void initialVtkWidget();
	//QSize viewSize(0,0);
	//virtual void resizeEvent(QResizeEvent *event);
	pcl::PointXYZ median(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
	bool BayesTest(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
	void connectSkel(int i,int j, pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
	bool colorFlag;
	double kmeansRadius,c;//distance衰减函数的参数
	int colorCloudIndex;
	void correctCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
	double distance(pcl::PointXYZ a, pcl::PointXYZ b, int model = 1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr normalize(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, bool divMode = true);//标准化函数
	void offReader(std::string filename);
	void saveNoff(std::string filename);
	void l1_median();
	void updateALLNeighbors(PcPtr qc, PcPtr xc, double radius, vector<SamplePoint> &info);
	void computeALLDirectionalityDegree();
	double computeDirectionalityDegree(int index);

	void computeNormal();
	void downSample(std::string path);
	void displaySampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);




private slots:
	void showDemo(); // 弹出messageBox和InfoText输出,用于测试文本框和软件是否正常运行
	void showPCL();  // 随机生成点云并显示，用于测试点云显示功能
	void colorByAxis(); // 按坐标给点云上色，用于测试点云彩色显示功能
	void drawLine(); // 从点云的[0]到[5]画线，用于测试骨架生成核心功能

	void onOpen();   // 打开文件函数，支持pcd，ply
	void onOpenTxt();// 打开txt文件
	void onOpenOff(); // 打开off文件
	// TO-DO： 将TXT集成到onOpen里

	void off_ply();  // off文件转ply，支持off，noff
	void segmentation(); // pcl库的min-cut 
	void color(pcl::PointCloud<pcl::PointXYZ>::Ptr,int,int,int); // 点云上色函数

	//void clustering(int num);
	void kmeans();
	void noise();
	void outlier();
	void normalizeOfSkel();


	bool skelParam(std::string params, int mode=1);
	void drawSkel();
	void reDrawSkel();
	void BayesSkel();

	void onRandomSample();

	void clearPointCloud();
	void resetPointCloud();
	void KNNsmooth();
	void onSaveNoff();
	void onDownSample();
	void onRandomMissing();
	void onSavePLY();
};
