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
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���

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
	double kmeansRadius,c;//distance˥�������Ĳ���
	int colorCloudIndex;
	void correctCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
	double distance(pcl::PointXYZ a, pcl::PointXYZ b, int model = 1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr normalize(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, bool divMode = true);//��׼������
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
	void showDemo(); // ����messageBox��InfoText���,���ڲ����ı��������Ƿ���������
	void showPCL();  // ������ɵ��Ʋ���ʾ�����ڲ��Ե�����ʾ����
	void colorByAxis(); // �������������ɫ�����ڲ��Ե��Ʋ�ɫ��ʾ����
	void drawLine(); // �ӵ��Ƶ�[0]��[5]���ߣ����ڲ��ԹǼ����ɺ��Ĺ���

	void onOpen();   // ���ļ�������֧��pcd��ply
	void onOpenTxt();// ��txt�ļ�
	void onOpenOff(); // ��off�ļ�
	// TO-DO�� ��TXT���ɵ�onOpen��

	void off_ply();  // off�ļ�תply��֧��off��noff
	void segmentation(); // pcl���min-cut 
	void color(pcl::PointCloud<pcl::PointXYZ>::Ptr,int,int,int); // ������ɫ����

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
