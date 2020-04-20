#pragma once

#include <QtWidgets/QMainWindow>
#include <QResizeEvent>
#include <QTableWidget>
#include <QTableWidgetItem>
#include "ui_QT_PCL_Segmentation.h"

#include <vector>
#include <windows.h>
#include <string>
#include <fstream>

#include "PointInfo.h"
#include "ParameterMgr.h"
#include "PointCloudAlgorithm.h"
#include "AlgorithmThread.h"

#include <pcl/common/projection_matrix.h>
#include <pcl/ModelCoefficients.h>
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
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>

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
	QDockWidget *paraDock = NULL;
	QTableWidget *paraTable = NULL;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

private:
	ParameterMgr *paraMgr = NULL;
	AlgorithmThread *atPtr = NULL;
	PointCloudAlgorithm *algorithm = NULL;
	bool isAlgorithmRunning = false;
	vector<vector<PointXYZ>>* skeleton = new vector<vector<PointXYZ>>({});

private:
	PointCloud<PointXYZ>::Ptr originCloud;
	PointCloud<PointXYZ>::Ptr sampleCloud;
	vector<PtKind> sampleStatus;
	vector<double> sampleSigma;
	PointCloud<PointXYZ>::Ptr cloudfiltered; 
	PointCloud<Normal>::Ptr normalCloud;

	/*历史遗留*/
	string cloudPath;
	double skelScale;
	pcl::PointXYZ midPoint;

public:
	void initialVtkWidget();
	/*文件相关*/
	PointCloud<PointXYZ>::Ptr normalize(PointCloud<PointXYZ>::Ptr inCloud, bool divMode = true);
	void computeNormal();
	void saveNoff(string filename);
	void downSample(string path);
	/*UI相关*/
	void displayOriginCloud(PointCloud<PointXYZ>::Ptr);
	void displaySelectedCloud(const PointCloud<PointXYZ>::Ptr, const pi::RGB, double, string);
	void initParaMgr();
	void initParaDockWithParaMgr(ParameterMgr*);

private slots:
	/*功能测试函数*/
	void testUIready(); // 弹出messageBox和InfoText输出,用于测试文本框和软件是否正常运行
	void testPCLready();  // 随机生成点云并显示，用于测试点云显示功能
	void testColorAmongAxis(); // 按坐标给点云上色，用于测试点云彩色显示功能
	void testSkelPainting(); // 从点云的[0]到[5]画线，用于测试骨架生成核心功能

	/*文件slot*/
	void onOpen();   // 打开文件函数，支持pcd，ply
	void onOpenTxt();// 打开txt文件
	void onOpenOff(); // 打开off文件
	void off_ply();  // off文件转ply，支持off，noff
	void onSaveNoff();
	void onSavePLY();

	/*算法slot*/
	void onL1();
	void onMoving();

	void noise();
	void outlier();
	void onRandomSample();
	void onDownSample();
	void onRandomMissing();

	/*更新slot*/
	void onUpdate(); // 更新sampleCloud的显示
	void displaySampleWithKind();
	void displaySampleWithSigma();

	void clearPointCloud();
	void resetPointCloud();
	void updateParameterMgr(QTableWidgetItem *item); // 更新paraMgr的内容与UI相符合
	void updateAllDockWidget(); // 更新dock的显示状态
	void updateBarStatus();// 更新视图菜单的check状况
	void updateAlgorithmState();
	void displaySkeleton();

	void connectCommonSlots();
	void displaySampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void displayAlgorithmInfo(const QString name);
	void displayAlgorithmError(const QString name);

};
