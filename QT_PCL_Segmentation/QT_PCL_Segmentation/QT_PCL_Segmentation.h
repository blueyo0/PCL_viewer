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

	/*��ʷ����*/
	string cloudPath;
	double skelScale;
	pcl::PointXYZ midPoint;

public:
	void initialVtkWidget();
	/*�ļ����*/
	PointCloud<PointXYZ>::Ptr normalize(PointCloud<PointXYZ>::Ptr inCloud, bool divMode = true);
	void computeNormal();
	void saveNoff(string filename);
	void downSample(string path);
	/*UI���*/
	void displayOriginCloud(PointCloud<PointXYZ>::Ptr);
	void displaySelectedCloud(const PointCloud<PointXYZ>::Ptr, const pi::RGB, double, string);
	void initParaMgr();
	void initParaDockWithParaMgr(ParameterMgr*);

private slots:
	/*���ܲ��Ժ���*/
	void testUIready(); // ����messageBox��InfoText���,���ڲ����ı��������Ƿ���������
	void testPCLready();  // ������ɵ��Ʋ���ʾ�����ڲ��Ե�����ʾ����
	void testColorAmongAxis(); // �������������ɫ�����ڲ��Ե��Ʋ�ɫ��ʾ����
	void testSkelPainting(); // �ӵ��Ƶ�[0]��[5]���ߣ����ڲ��ԹǼ����ɺ��Ĺ���

	/*�ļ�slot*/
	void onOpen();   // ���ļ�������֧��pcd��ply
	void onOpenTxt();// ��txt�ļ�
	void onOpenOff(); // ��off�ļ�
	void off_ply();  // off�ļ�תply��֧��off��noff
	void onSaveNoff();
	void onSavePLY();

	/*�㷨slot*/
	void onL1();
	void onMoving();

	void noise();
	void outlier();
	void onRandomSample();
	void onDownSample();
	void onRandomMissing();

	/*����slot*/
	void onUpdate(); // ����sampleCloud����ʾ
	void displaySampleWithKind();
	void displaySampleWithSigma();

	void clearPointCloud();
	void resetPointCloud();
	void updateParameterMgr(QTableWidgetItem *item); // ����paraMgr��������UI�����
	void updateAllDockWidget(); // ����dock����ʾ״̬
	void updateBarStatus();// ������ͼ�˵���check״��
	void updateAlgorithmState();
	void displaySkeleton();

	void connectCommonSlots();
	void displaySampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void displayAlgorithmInfo(const QString name);
	void displayAlgorithmError(const QString name);

};
