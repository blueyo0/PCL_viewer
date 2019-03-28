#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QT_PCL_Segmentation.h"

#include <vector>

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

class QT_PCL_Segmentation : public QMainWindow
{
	Q_OBJECT

public:
	QT_PCL_Segmentation(QWidget *parent = Q_NULLPTR);

private:
	Ui::QT_PCL_SegmentationClass ui;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector<int> tag;

	pcl::PointCloud<pcl::PointXYZ>::Ptr skelCloud;
	int branchNum;
	std::vector<int> branchLen;
	std::string modelSkelName;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	void initialVtkWidget();
	pcl::PointXYZ median(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
	bool colorFlag;
	int colorCloudIndex;
	void correctCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
	double distance(pcl::PointXYZ a, pcl::PointXYZ b, int model = 1);

private slots:
	void showDemo();
	void showPCL();
	void onOpen();
	void segmentation();
	void colorByAxis();
	void color(pcl::PointCloud<pcl::PointXYZ>::Ptr,int,int ,int);
	void drawLine();

	//void clustering(int num);
	void kmeans();

	void readSkel(std::string filename);
	void drawSkel();
	void reDrawSkel();
	void clearPointCloud();
	void KNNsmooth();
};
