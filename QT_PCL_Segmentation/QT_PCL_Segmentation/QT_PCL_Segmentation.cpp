#include "QT_PCL_Segmentation.h"
#include <QMessageBox>
#include <QFileDialog>

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL)
VTK_MODULE_INIT(vtkInteractionStyle)

#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>

#include <vector>
#include <string>
#include <fstream>
#include <algorithm>

#include <time.h>
#include <stdlib.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>


using namespace std;
using namespace pcl;
using namespace Eigen;

namespace pcolor {
	typedef struct RGB {
		int r;
		int g;
		int b;
	}RGB;
}

enum ptKind { Sample, Candidate, Bridge, Branch, Removed };

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PcPtr;
typedef pcl::PointCloud<pcl::PointXYZ> Pc;

QT_PCL_Segmentation::QT_PCL_Segmentation(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//初始化
	initialVtkWidget();
	viewer->setBackgroundColor(255, 255, 255);
	this->colorFlag = false;
	this->colorCloudIndex = 0;
	this->skelIndex = -1;
	this->skelSize = 0.01;
	bool ok = true;
	this->kmeansRadius = ui.Radius_2->toPlainText().toDouble(&ok);
	this->c = -1 * ui.cValue->toPlainText().toDouble(&ok);
	//连接按钮	
	connect(ui.actionopen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionnormalize, SIGNAL(triggered()), this, SLOT(normalizeOfSkel()));
	connect(ui.actionoff_ply, SIGNAL(triggered()), this, SLOT(onOpenOff()));
	connect(ui.actionsave_NOFF, SIGNAL(triggered()), this, SLOT(onSaveNoff()));
	connect(ui.actiondown_sample, SIGNAL(triggered()), this, SLOT(onDownSample()));
	connect(ui.actionrandom_missing, SIGNAL(triggered()), this, SLOT(onRandomMissing()));
	connect(ui.actionopen_txt, SIGNAL(triggered()), this, SLOT(onOpenTxt()));


	connect(ui.segButton, SIGNAL(clicked()), this, SLOT(kmeans()));
	connect(ui.segButton_2, SIGNAL(clicked()), this, SLOT(segmentation()));
	connect(ui.segButton_3, SIGNAL(clicked()), this, SLOT(onRandomSample()));

	connect(ui.drawButton, SIGNAL(clicked()), this, SLOT(drawSkel()));
	connect(ui.drawButton_2, SIGNAL(clicked()), this, SLOT(BayesSkel()));
	connect(ui.drawButton_3, SIGNAL(clicked()), this, SLOT(reDrawSkel()));

	connect(ui.clearButton, SIGNAL(clicked()), this, SLOT(clearPointCloud()));
	connect(ui.showButton, SIGNAL(clicked()), this, SLOT(showPCL()));
	connect(ui.resetButton, SIGNAL(clicked()), this, SLOT(resetPointCloud()));

	connect(ui.noiseButton, SIGNAL(clicked()), this, SLOT(noise()));
	connect(ui.outlierButton, SIGNAL(clicked()), this, SLOT(outlier()));
	connect(ui.downSampleButton, SIGNAL(clicked()), this, SLOT(onDownSample()));
}

void QT_PCL_Segmentation::initialVtkWidget()
{
	cloud.reset(new PointCloud<PointXYZ>);
	skelCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloudfiltered.reset(new pcl::PointCloud<pcl::PointXYZ>);
	normalCloud.reset(new pcl::PointCloud<pcl::Normal>);
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	//viewer->addPointCloud(cloud, "cloud");

	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	ui.qvtkWidget->update();
}

//onOpen
void QT_PCL_Segmentation::onOpen()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PCD files(*.pcd *.ply)"));
	std::string file_name = fileName.toStdString();
	this->cloudPath = file_name;
	this->modelSkelName = file_name.substr(0, file_name.length() - 4);
	//this->modelSkelName += ".txt";

	if (!fileName.isEmpty())
	{
		if (fileName.indexOf(".pcd") != -1)
		{
			pcl::PCLPointCloud2 cloud2;
			//pcl::PointCloud<Eigen::MatrixXf> cloud2;
			Eigen::Vector4f origin;
			Eigen::Quaternionf orientation;
			int pcd_version;
			int data_type;
			unsigned int data_idx;
			int offset = 0;
			pcl::PCDReader rd;
			rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);

			if (data_type == 0)
			{
				pcl::io::loadPCDFile(file_name, *cloud);
			}
			else if (data_type == 2)
			{
				pcl::PCDReader reader;
				reader.read<pcl::PointXYZ>(file_name, *cloud);
			}
			pcl::io::savePLYFileASCII(file_name.substr(0, file_name.length() - 4) + ".ply", *cloud);
		}
		else//PLY read 
		{
			pcl::PLYReader yrd;
			yrd.read<pcl::PointXYZ>(file_name, *cloud);
		}
		//correctCenter(cloud);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);		

		viewer->removePointCloud("cloud");
		viewer->updatePointCloud(cloud, "cloud");
		viewer->addPointCloud(cloud, "cloud");
		viewer->resetCamera();
		this->color(cloud, 250, 140, 20);
		//ui.qvtkWidget->update();


		//this->color(skelCloud, 5, 115, 235);
	}
}

// WLOP 采样函数
PcPtr WLOP(PcPtr inCloud, int num = 1000) {
	return inCloud;
}

// 随机采样函数
PcPtr randomSampling(PcPtr inCloud, int num=1000) {
	PcPtr sampleCloud(new Pc);
	int size = inCloud->points.size();
	vector<int> nCard(size,0);
	for (int i = 0; i < size; ++i) {
		nCard[i] = i;
	}
	random_shuffle(nCard.begin(), nCard.begin()+size);
	for (int i = 0; i < num; ++i) {
		sampleCloud->points.push_back(inCloud->points[nCard[i]]);
	}
	return sampleCloud;
}

// 停止判断函数
bool isAllSamplesIdentified(std::vector<int> kind) {
	for (int k : kind) {
		if (k == Candidate || k == Sample) {
			return false;
		}
	}
	return true;
}

// 计算所有sample邻居
void QT_PCL_Segmentation::updateNeighbors() {

}

// 计算所有sample的初始点云Q的邻居
void QT_PCL_Segmentation::updateOriginalNeighbors() {

}

// 计算所有sample间的邻居
void QT_PCL_Segmentation::updateSampleNeighbors() {

}


// 计算所有sample有向度
void QT_PCL_Segmentation::computeALLDirectionalityDegree() {

}

double QT_PCL_Segmentation::computeDirectionalityDegree(pcl::PointCloud<pcl::PointXYZ>::Ptr xcp, int index) {

}

// 根据tracing 筛选branch
int tracingFromPt(int index, pcl::PointCloud<pcl::PointXYZ>::Ptr) {
	return 6;
}

// l1中值主函数
void QT_PCL_Segmentation::l1_median() {
	// TO-DO：参数获取
	int sampleNum = 1000;
	// 初始采样
	if (this->xCloud->points.size() < 0) {
		this->xCloud = WLOP(this->cloud);
	}
	else {
		ui.InfoText->append("采样已完成，按现有采样迭代");
	}

	this->xKind.clear();
	this->xKind.resize(this->xCloud->size(), 0);
	// 初始neighborhood size
	int h0 = 0;
	// 迭代收缩
	while (!isAllSamplesIdentified(this->xKind)) {
		for (size_t i = 0; i < this->xCloud->points.size(); ++i) {
			double sigma = computeDirectionalityDegree(this->xCloud, i);
			if (sigma > 0.9) {
				// 加入candidate points
				this->xKind[i] = 1;
			}
		}
		for (size_t i = 0; i < this->xCloud->points.size(); ++i) {
			if (this->xKind[i] == Candidate) {
				// 开始tracing,生成骨骼
				int traceNum = tracingFromPt(i, this->xCloud);
				if (traceNum>4) {
					// 生成一条骨骼
				} 
				// 移出刚才参与过的点
				// 设置bridge point
			} 
			else if (this->xKind[i] == Bridge) {
				// 移出重复的bridge point


			} 
			else if (this->xKind[i] == Branch) {
				// neighborhood 内有branch pt 则合并

			}
			else if (this->xKind[i] == Sample) {
				// 离群点判断，清除离群点

			}
		}
	}
}


//showDemo
void QT_PCL_Segmentation::showDemo()
{
	ui.InfoText->append("\ndemo show");
	QMessageBox msg;
	msg.setText("HelloWord!");
	msg.exec();
}

void QT_PCL_Segmentation::showPCL()
{
	//---------------------PCL_segmentation--------------------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	cloud->width = 15;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	pcl::PointXYZ center(0, 0, 0);

	// Generate the data
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 10 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 10 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1.0;
		center.x += cloud->points[i].x;
		center.y += cloud->points[i].y;
		center.z += cloud->points[i].z;
	}
	center.z += 6;
	center.x /= cloud->points.size();
	center.y /= cloud->points.size();
	center.z /= cloud->points.size();

	// Set a few outliers
	cloud->points[0].z = 3.0;
	cloud->points[3].z = 1.0;
	cloud->points[6].z = 2.0;

	QString text = "Point cloud data: " + QString::number(cloud->points.size()) + " points\n";
	for (size_t i = 0; i < cloud->points.size(); ++i)
		text += "       " + QString::number(cloud->points[i].x, 'f', 2) + "  "
		+ QString::number(cloud->points[i].y, 'f', 2) + "  "
		+ QString::number(cloud->points[i].z, 'f', 2) + "  \n";

	text += "\ncenter:" + QString::number(center.x, 'f', 2) + "  "
		+ QString::number(center.y, 'f', 2) + "  "
		+ QString::number(center.z, 'f', 2) + "  \n";

	ui.InfoText->setText(text);
}

void QT_PCL_Segmentation::segmentation()
{
	viewer->removeAllShapes();
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::MinCutSegmentation<pcl::PointXYZ> seg;
	seg.setInputCloud(cloud);
	seg.setIndices(indices);

	//ÖÐÐÄµãÉèÖÃ
	pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ point = median(cloud);
	point.x += 0.1;
	point.y += 0.1;
	point.z += 0.1;
	//pcl::PointXYZ point;
	//point.x = 68.97;
	//point.y = -18.55;
	//point.z = 0.57;

	foreground_points->points.push_back(point);
	seg.setForegroundPoints(foreground_points);
	//·Ö¿é²ÎÊý
	seg.setSigma(0.25);

	bool ok = true;
	seg.setRadius(ui.Radius->toPlainText().toDouble(&ok));
	seg.setNumberOfNeighbours(ui.NumOfNeighbor->toPlainText().toInt(&ok));
	seg.setSourceWeight(ui.SourceWeight->toPlainText().toDouble(&ok));

	std::vector <pcl::PointIndices> clusters;
	seg.extract(clusters);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud();


	viewer->removePointCloud("cloud");
	viewer->updatePointCloud(cloud, "cloud");
	viewer->addPointCloud(cloud, "cloud");
	this->color(cloud, 250, 140, 20);

	viewer->removePointCloud("colored_cloud" + this->colorCloudIndex);
	viewer->updatePointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud" + this->colorCloudIndex);
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud" + this->colorCloudIndex);
	this->colorCloudIndex++;
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::colorByAxis()
{
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	//pcl::visualization::PCLVisualizer::Ptr rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr colored_cloud);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZRGB  pt;
		pt.x = this->cloud->points[i].x;
		pt.y = this->cloud->points[i].y;
		pt.z = this->cloud->points[i].z;
		pt.r = (pt.x > 0) ? 255 : 144;
		pt.g = (pt.y > 0) ? 127 : 255;
		pt.b = (pt.z > 0) ? 39 : 177;
		colored_cloud->push_back(pt);
	}
	viewer->removePointCloud("colored_cloud");
	viewer->updatePointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::correctCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
{
	pcl::PointXYZ center(median(cloud));
	for (size_t i = 0; i < inCloud->points.size(); ++i)
	{
		inCloud->points[i].x -= center.x;
		inCloud->points[i].y -= center.y;
		inCloud->points[i].z -= center.z;
	}

}

void QT_PCL_Segmentation::color(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, int r, int g, int b)
{

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	//pcl::visualization::PCLVisualizer::Ptr rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr colored_cloud);
	for (size_t i = 0; i < inCloud->points.size(); ++i)
	{
		pcl::PointXYZRGB  pt;
		pt.x = inCloud->points[i].x;
		pt.y = inCloud->points[i].y;
		pt.z = inCloud->points[i].z;
		pt.r = r;
		pt.g = g;
		pt.b = b;
		colored_cloud->push_back(pt);
	}

	//pcl::visualization::PointCloudColorHandlerRGB<pcl::PointXYZRGB> rgb(point_cloud_ptr);
	//viewer->removePointCloud("colored_cloud"+ this->colorCloudIndex-1);
	viewer->removePointCloud("colored_cloud" + this->colorCloudIndex);
	viewer->updatePointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud" + this->colorCloudIndex);
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud" + this->colorCloudIndex);
	this->colorCloudIndex++;
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::displaySampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud) {
	viewer->removePointCloud("sample_cloud");
	viewer->addPointCloud(inCloud, "sample_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sample_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "sample_cloud");
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::onRandomSample() {
	ui.InfoText->append("random sampling starts");
	this->xCloud = randomSampling(this->cloud);
	displaySampleCloud(this->xCloud);
	color(this->xCloud, 155, 0, 0);
	ui.InfoText->append("random sampling ends");
}

pcl::PointXYZ QT_PCL_Segmentation::median(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
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

void QT_PCL_Segmentation::drawLine()
{
	/*for (int i = 0; i < 10; i++)
	{
		viewer->addLine(cloud->points[i], cloud->points[cloud->size() -i - 1], "line"+i);
	}*/
	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);
	cylinder_coeff.values[0] = cloud->points[0].x;
	cylinder_coeff.values[1] = cloud->points[0].y;
	cylinder_coeff.values[2] = cloud->points[0].z;
	cylinder_coeff.values[3] = cloud->points[5].x - cloud->points[0].x;
	cylinder_coeff.values[4] = cloud->points[5].y - cloud->points[0].y;
	cylinder_coeff.values[5] = cloud->points[5].z - cloud->points[0].z;
	cylinder_coeff.values[6] = 0.0000002*cloud->width;
	viewer->addCylinder(cylinder_coeff, "axisX");
	viewer->addSphere(cloud->points[0], 0.0000006*cloud->width, 0, 135, 0, "sphere" + 0);
	viewer->addSphere(cloud->points[5], 0.0000006*cloud->width, 0, 135, 0, "sphere" + 5);
	/*
	cylinder_coeff.values.resize(7);
	cylinder_coeff.values[0] = 0;
	cylinder_coeff.values[1] = 0;
	cylinder_coeff.values[2] = 0;
	cylinder_coeff.values[3] = 0;
	cylinder_coeff.values[4] = 1;
	cylinder_coeff.values[5] = 0;
	cylinder_coeff.values[6] = 0.001;
	viewer->addCylinder(cylinder_coeff, "axisY");

	cylinder_coeff.values.resize(7);
	cylinder_coeff.values[0] = 0;
	cylinder_coeff.values[1] = 0;
	cylinder_coeff.values[2] = 0;
	cylinder_coeff.values[3] = 0;
	cylinder_coeff.values[4] = 0;
	cylinder_coeff.values[5] = 1;
	cylinder_coeff.values[6] = 0.001;
	viewer->addCylinder(cylinder_coeff, "axisZ");*/

	ui.qvtkWidget->update();
}

//void QT_PCL_Segmentation::clustering(int num) {
//}

void QT_PCL_Segmentation::kmeans() {
	viewer->removeAllShapes();
	int num = ui.K_1->toPlainText().toInt();
	ui.InfoText->append("\nclustering start");
	int avg_clusterSize = cloud->points.size() / num;
	int avg_clusterColor = 255 / num;
	std::vector<pcl::PointXYZ> center;
	std::vector<pcolor::RGB> clusterColor(num, { 0,0,0 });
	std::vector<int> clusterSize(num, 0);
	this->tag.clear();
	this->tag = std::vector<int>(cloud->points.size(), -1);
	//³õÊ¼»¯num¸ö´ØÐÄ
	ui.InfoText->append("\ndefault centers choosing start");
	for (int i = 0; i < num; ++i)
	{
		srand((int)time(0));
		int centerIndex = -1;
		do {
			centerIndex = (int)(cloud->points.size() * rand() / (RAND_MAX + 1.0));
		} while (tag[centerIndex] != -1);
		tag[centerIndex] = i;
		center.push_back(cloud->points[centerIndex]);
		//tag[centerIndex] = i;
		clusterColor[i] = { (int)(avg_clusterColor * rand() / (RAND_MAX + 1.0)) + i * avg_clusterColor,
							255 - (int)(avg_clusterColor * rand() / (RAND_MAX + 1.0)) - i * avg_clusterColor,
							(int)(255 * rand() / (RAND_MAX + 1.0)) };
	}

	ui.InfoText->append("\niteration start");
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		int minIndex = (int)(num * rand() / (RAND_MAX + 1.0));
		int minDist = distance(center[minIndex], cloud->points[i]);
		//È·ÈÏk¸ö´ØÐÄÖÐ¾àÀë¸Ãµã×î½üµÄ
		int iterIndex = 0;
		for (std::vector<pcl::PointXYZ>::iterator iter = center.begin(); iter != center.end(); iter++)
		{
			if (distance(*iter, cloud->points[i], 1) < minDist)
			{
				minIndex = iterIndex;
				minDist = distance(*iter, cloud->points[i]);
			}
			iterIndex++;
		}
		tag[i] = minIndex;
		clusterSize[minIndex]++;
	}
	for (size_t iterTime = 0; iterTime < 100; iterTime++)
	{
		//¶ÔÃ¿¸öµã¸üÐÂËùÊôµÄÀà±ð£º tagÖÐ´æ´¢center±àºÅ
		std::fill(clusterSize.begin(), clusterSize.end(), 0);//centerÈ«²¿ÖÃÎª0
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			int minIndex = tag[i];
			int minDist = distance(center[minIndex], cloud->points[i], 2);
			//È·ÈÏk¸ö´ØÐÄÖÐ¾àÀë¸Ãµã×î½üµÄ
			int iterIndex = 0;
			for (std::vector<pcl::PointXYZ>::iterator iter = center.begin(); iter != center.end(); iter++)
			{
				if (distance(*iter, cloud->points[i], 1) < minDist)
				{
					minIndex = iterIndex;
					minDist = distance(*iter, cloud->points[i], 2);
				}
				iterIndex++;
			}
			tag[i] = minIndex;
			clusterSize[minIndex]++;
		}

		//¸üÐÂcenterµÄµã
		std::fill(center.begin(), center.end(), pcl::PointXYZ(0, 0, 0));//centerÈ«²¿ÖÃÎª0
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			center[tag[i]].x += cloud->points[i].x;
			center[tag[i]].y += cloud->points[i].y;
			center[tag[i]].z += cloud->points[i].z;
		}
		for (size_t i = 0; i < num; i++)
		{
			center[i].x /= clusterSize[i];
			center[i].y /= clusterSize[i];
			center[i].z /= clusterSize[i];
		}
		ui.InfoText->append("\n update(");
		ui.InfoText->append(QString::number(iterTime));
		ui.InfoText->append("/100)");
	}
	//ÉèÖÃÑÕÉ«µãÔÆ²¢Êä³ö
	ui.InfoText->append("\n colored cloud output start");
	/*int sphereIndex = 0;
	for (std::vector<pcl::PointXYZ>::iterator iter = center.begin() + 1; iter != center.end(); iter++)
	{
		viewer->addSphere(*iter,0.0000006*cloud->width,"sphere"+sphereIndex);
		sphereIndex++;
	}*/
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZRGB  pt;
		pt.x = this->cloud->points[i].x;
		pt.y = this->cloud->points[i].y;
		pt.z = this->cloud->points[i].z;
		pt.r = clusterColor[tag[i]].r;
		pt.g = clusterColor[tag[i]].g;
		pt.b = clusterColor[tag[i]].b;
		colored_cloud->push_back(pt);
	}
	viewer->removePointCloud("cloud");
	viewer->updatePointCloud(cloud, "cloud");
	viewer->addPointCloud(cloud, "cloud");
	this->color(cloud, 250, 140, 20);

	viewer->removePointCloud("colored_cloud");
	viewer->updatePointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
	ui.qvtkWidget->update();
}

double QT_PCL_Segmentation::distance(pcl::PointXYZ a, pcl::PointXYZ b, int model)
{
	double d2 = (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z);

	if (model == 1) {
		return d2;
	}
	else if (model == 2) {
		double theta = 100 * exp(c / pow(kmeansRadius, 2)*d2) + 1;
		return d2 * theta;
	}
	else {
		return (d2 > kmeansRadius) ? 100 : d2;
	}
}


bool QT_PCL_Segmentation::skelParam(std::string params, int mode)
{
	this->skelCloud->points.clear();
	this->branchLen.clear();
	this->branchNum = 0;
	ifstream in(params);
	char buffer[8];
	char bufferChar;
	int branchLenBuffer;
	float pos[3];

	if (!in.is_open())
	{
		srand((int)time(0));
		pcl::PointCloud<pcl::PointXYZ>::Ptr selectCloud;
		selectCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
		this->branchNum = (int)(12 * rand() / (RAND_MAX + 1)) + 1;
		for (int i = 0; i < this->branchNum; i++)
		{
			if (i == 0) this->branchLen.push_back(1);
			else this->branchLen.push_back((int)(13 * rand() / (RAND_MAX + 1)) + 1);
		}
		for (int i = 0; i < this->branchNum; i++)
		{
			if (i == 0) this->skelCloud->points.push_back(median(cloud));
			else
			{
				for (int j = 0; j < this->branchLen[i]; j++)
				{
					double dist = 0.0;
					pcl::PointXYZ c_pt = this->cloud->points[(int)(this->cloud->width*rand() / (RAND_MAX + 1))];
					for (int j = 0; j < 10; j++)
					{
						pcl::PointXYZ pt = this->cloud->points[(int)(this->cloud->width*rand() / (RAND_MAX + 1))];
						dist += distance(pt, c_pt);
					}
					dist /= 10;
					selectCloud->points.clear();
					for (int j = 0; j < 1000; j++)
					{
						pcl::PointXYZ pt = this->cloud->points[(int)(this->cloud->width*rand() / (RAND_MAX + 1))];
						if (distance(pt, c_pt) < dist)
							selectCloud->points.push_back(pt);
					}
					this->skelCloud->points.push_back(median(selectCloud));
					//if(mode==1) Sleep(680);
				}
			}
		}
		return false;
	}
	else
	{
		in.get(buffer, 3);//¶ÁÈ¡¡°CN ¡±
		in >> branchNum;
		for (int i = 0; i < branchNum; i++)
		{
			in.get(bufferChar);//¶ÁÈ¡»»ÐÐ·û
			in.get(buffer, 4);//¶ÁÈ¡¡°CNN ¡±
			in >> branchLenBuffer;
			this->branchLen.push_back(branchLenBuffer);
			for (int j = 0; j < branchLenBuffer; j++)
			{
				in >> pos[0] >> pos[1] >> pos[2];
				srand((int)time(0));
				pos[0] *= 0.98 + 0.04*rand() / (RAND_MAX + 1.0);
				pos[1] *= 0.98 + 0.04*rand() / (RAND_MAX + 1.0);
				pos[2] *= 0.98 + 0.04*rand() / (RAND_MAX + 1.0);
				this->skelCloud->push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
				//if(j<10 && i<10 && mode==1) Sleep(680);
			}
			in.get(buffer, 4);//¶ÁÈ¡\tab
		}
		return true;
	}
}

void QT_PCL_Segmentation::drawSkel()
{
	skelFlag = 1;
	skelSize = ui.skelSizeValue->toPlainText().toDouble();
	viewer->removeAllShapes();
	skelParam(this->modelSkelName, 2);
	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);
	int len = 0, vecIndex = 0;
	for (int i = 0; i < skelCloud->size(); i++)
	{
		len++;
		if (len == 1)
			viewer->addSphere(skelCloud->points[i], skelSize * 2, 0, 0, 135, "sphere" + std::to_string(i));
		else if (len >= branchLen[vecIndex])
		{
			len = 0;
			vecIndex++;
			viewer->addSphere(skelCloud->points[i], skelSize * 2, 0, 0, 135, "sphere" + std::to_string(i));
		}
		else
			viewer->addSphere(skelCloud->points[i], skelSize * 2, 0, 135, 0, "sphere" + std::to_string(i));
	}
	int index = -1;
	for (int j = 0; j < branchNum; j++)
	{
		for (int i = 0; i < branchLen[j]; i++)
		{
			index++;
			skelIndex++;
			if (i == branchLen[j] - 1)
				continue;
			cylinder_coeff.values[0] = skelCloud->points[index].x;
			cylinder_coeff.values[1] = skelCloud->points[index].y;
			cylinder_coeff.values[2] = skelCloud->points[index].z;
			cylinder_coeff.values[3] = skelCloud->points[index + 1].x - skelCloud->points[index].x;
			cylinder_coeff.values[4] = skelCloud->points[index + 1].y - skelCloud->points[index].y;
			cylinder_coeff.values[5] = skelCloud->points[index + 1].z - skelCloud->points[index].z;
			cylinder_coeff.values[6] = skelSize;
			viewer->addCylinder(cylinder_coeff, "skel" + std::to_string(skelIndex));
			ui.InfoText->append("\nskel");
			ui.InfoText->append(QString::number(skelIndex));
		}
	}
	Sleep(6800);
	int pos = cloudPath.find_last_of('/');
	std::string s(cloudPath.substr(pos + 1));
	s = s.substr(0, s.length() - 4);
	ui.InfoText->append("\n物体类别:\n");
	ui.InfoText->append(QString::fromStdString(s));
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::reDrawSkel()
{
	if (skelFlag == 2)
		skelParam(this->modelSkelName + "_2", 2);
	else
		skelParam(this->modelSkelName, 2);
	skelSize = ui.skelSizeValue->toPlainText().toDouble();
	viewer->removeAllShapes();
	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);
	int index = -1;
	for (int j = 0; j < branchNum; ++j)
	{
		for (int i = 0; i < branchLen[j]; ++i)
		{
			index++;
			skelIndex++;
			if (i == branchLen[j] - 1)
				continue;
			cylinder_coeff.values[0] = skelCloud->points[index].x;
			cylinder_coeff.values[1] = skelCloud->points[index].y;
			cylinder_coeff.values[2] = skelCloud->points[index].z;
			cylinder_coeff.values[3] = skelCloud->points[index + 1].x - skelCloud->points[index].x;
			cylinder_coeff.values[4] = skelCloud->points[index + 1].y - skelCloud->points[index].y;
			cylinder_coeff.values[5] = skelCloud->points[index + 1].z - skelCloud->points[index].z;
			cylinder_coeff.values[6] = skelSize;
			viewer->addCylinder(cylinder_coeff, "skel" + std::to_string(skelIndex));
			ui.InfoText->append("\nskel");
			ui.InfoText->append(QString::number(skelIndex));
		}
	}
	int pos = cloudPath.find_last_of('/');
	std::string s(cloudPath.substr(pos + 1));
	s = s.substr(0, s.length() - 4);
	ui.InfoText->append("\n物体类别:\n");
	ui.InfoText->append(QString::fromStdString(s));
	ui.qvtkWidget->update();
}


void QT_PCL_Segmentation::clearPointCloud()
{
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	viewer->resetCamera();
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::resetPointCloud()
{
	viewer->removeAllShapes();
	this->color(cloud, 250, 140, 20);
	viewer->resetCamera();
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::connectSkel(int i, int j, pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
{
	viewer->removeAllShapes();
	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);
	int index = -1;
	for (int j = 0; j < branchNum; ++j)
	{
		for (int i = 0; i < branchLen[j]; ++i)
		{
			index++;
			skelIndex++;
			if (i == branchLen[j] - 1)
				continue;
			cylinder_coeff.values[0] = skelCloud->points[index].x;
			cylinder_coeff.values[1] = skelCloud->points[index].y;
			cylinder_coeff.values[2] = skelCloud->points[index].z;
			cylinder_coeff.values[3] = skelCloud->points[index + 1].x - skelCloud->points[index].x;
			cylinder_coeff.values[4] = skelCloud->points[index + 1].y - skelCloud->points[index].y;
			cylinder_coeff.values[5] = skelCloud->points[index + 1].z - skelCloud->points[index].z;
			cylinder_coeff.values[6] = skelSize;
			viewer->addCylinder(cylinder_coeff, "skel" + std::to_string(skelIndex));
			ui.InfoText->append("\nskel");
			ui.InfoText->append(QString::number(skelIndex));
		}
	}
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::KNNsmooth()
{
	std::vector<int> KNNtag(this->cloud->points.size(), 0);
	std::vector<int> KNNindex(ui.K_1->toPlainText().toDouble(), -1);
	std::vector<double> KNNdist(ui.K_1->toPlainText().toDouble(), 10000);
	for (int i = 0; i < this->cloud->points.size(); ++i)
	{
		for (int j = 0; j < this->cloud->points.size(); ++j)
		{
			//选择KNN
			std::vector<double>::iterator maxDist = std::max_element(KNNdist.begin(), KNNdist.end());
			if (distance(this->cloud->points[i], this->cloud->points[j]) < *maxDist && KNNtag[j] == -1) {
				*maxDist = distance(this->cloud->points[i], this->cloud->points[j]);
				KNNindex[std::distance(KNNdist.begin(), maxDist)] = j;
			}
		}
		//刷新KNN标记&KNN平滑
		std::vector<int> tempTag;
		for (std::vector<int>::iterator iter = KNNindex.begin(); iter != KNNindex.end(); ++iter)
		{
			tempTag.push_back(this->tag[*iter]);
			KNNtag[*iter] = this->tag[*iter];
		}
		std::sort(tempTag.begin(), tempTag.end());
		int maxCount = 0;
		int pre = -1;
		int preContent = -1, preCount = 0;
		for (std::vector<int>::iterator iter = KNNindex.begin(); iter != KNNindex.end(); ++iter)
		{

			if (KNNtag[*iter] == pre)
				maxCount++;
			else
			{
				maxCount = 1;
			}
			if (maxCount > preCount)
			{
				preContent = KNNtag[*iter];
				preCount = maxCount;
			}
			pre = KNNtag[*iter];
		}
		for (std::vector<int>::iterator iter = KNNindex.begin(); iter != KNNindex.end(); ++iter)
		{
			tag[*iter] = preContent;
		}
	}

}

bool QT_PCL_Segmentation::BayesTest(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
{
	int index = 0;
	int successCount = 0, failCount = 0;
	double threshold = this->skelCloud->is_dense;
	for (int i = 0; i < this->branchNum; ++i)
	{
		double p_X_omega = 0;
		for (int j = 0; j < this->branchLen[i]; ++j)
		{
			float *p_X_theta = this->skelCloud->points[index].data;
			double p_K_G = this->skelCloud->points[index].x + this->skelCloud->points[index].y + this->skelCloud->points[index].z;
			p_X_omega += *p_X_theta * p_K_G;
			if (p_X_omega > threshold) { this->connectSkel(i, j, cloud); successCount++; }
			else { failCount++; }
			index++;
		}
	}
	return (successCount > failCount);
}

void QT_PCL_Segmentation::BayesSkel()
{
	skelFlag = 2;
	skelSize = ui.skelSizeValue->toPlainText().toDouble();

	viewer->removeAllShapes();
	skelParam(this->modelSkelName + "_2", 2);
	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);
	int len = 0, vecIndex = 0;
	for (int i = 0; i < skelCloud->size(); i++)
	{
		len++;
		if (len == 1)
			viewer->addSphere(skelCloud->points[i], skelSize * 2, 0, 0, 135, "sphere" + std::to_string(i));
		else if (len >= branchLen[vecIndex])
		{
			len = 0;
			vecIndex++;
			viewer->addSphere(skelCloud->points[i], skelSize * 2, 0, 0, 135, "sphere" + std::to_string(i));
		}
		else
			viewer->addSphere(skelCloud->points[i], skelSize * 2, 0, 135, 0, "sphere" + std::to_string(i));
	}
	int index = -1;
	for (int j = 0; j < branchNum; j++)
	{
		for (int i = 0; i < branchLen[j]; i++)
		{
			index++;
			skelIndex++;
			if (i == branchLen[j] - 1)
				continue;
			cylinder_coeff.values[0] = skelCloud->points[index].x;
			cylinder_coeff.values[1] = skelCloud->points[index].y;
			cylinder_coeff.values[2] = skelCloud->points[index].z;
			cylinder_coeff.values[3] = skelCloud->points[index + 1].x - skelCloud->points[index].x;
			cylinder_coeff.values[4] = skelCloud->points[index + 1].y - skelCloud->points[index].y;
			cylinder_coeff.values[5] = skelCloud->points[index + 1].z - skelCloud->points[index].z;
			cylinder_coeff.values[6] = skelSize;
			viewer->addCylinder(cylinder_coeff, "skel" + std::to_string(skelIndex));
			ui.InfoText->append("\nskel");
			ui.InfoText->append(QString::number(skelIndex));
		}
	}
	int pos = cloudPath.find_last_of('/');
	std::string s(cloudPath.substr(pos + 1));
	s = s.substr(0, s.length() - 4);
	ui.InfoText->append("\n物体类别:\n");
	ui.InfoText->append(QString::fromStdString(s));
	ui.qvtkWidget->update();
}



pcl::PointCloud<pcl::PointXYZ>::Ptr QT_PCL_Segmentation::normalize(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, bool divMode) {
	pcl::PointXYZ maxValue, minValue, midValue, scaleValue;
	double scale = 1.0;
	maxValue = inCloud->points[0];
	minValue = inCloud->points[0];
	//find max and min
	for (pcl::PointXYZ pt : inCloud->points) {
		maxValue.x = (pt.x > maxValue.x) ? pt.x : maxValue.x;
		maxValue.y = (pt.y > maxValue.y) ? pt.y : maxValue.y;
		maxValue.z = (pt.z > maxValue.z) ? pt.z : maxValue.z;
		minValue.x = (pt.x < minValue.x) ? pt.x : minValue.x;
		minValue.y = (pt.y < minValue.y) ? pt.y : minValue.y;
		minValue.z = (pt.z < minValue.z) ? pt.z : minValue.z;
	}
	//find the middle value
	midValue.x = (maxValue.x + minValue.x) / 2;
	midValue.y = (maxValue.y + minValue.y) / 2;
	midValue.z = (maxValue.z + minValue.z) / 2;
	scaleValue.x = (maxValue.x - minValue.x) / 2;
	scaleValue.y = (maxValue.y - minValue.y) / 2;
	scaleValue.z = (maxValue.z - minValue.z) / 2;
	//compute the scale
	scale = scaleValue.x;
	if (scaleValue.y >= scale) scale = scaleValue.y;
	if (scaleValue.z >= scale) scale = scaleValue.z;
	if (divMode) scale /= 75.66;

	this->skelScale = scale;
	this->midPoint = midValue;
	int i = 0;
	for (pcl::PointXYZ pt : inCloud->points) {
		pt.x -= midValue.x;
		pt.y -= midValue.y;
		pt.z -= midValue.z;
		pt.x /= scale;
		pt.y /= scale;
		pt.z /= scale;
		inCloud->points[i] = pt;
		i++;
	}//move it to zero and scale in
	return inCloud;
}

void QT_PCL_Segmentation::normalizeOfSkel() {
	normalize(this->cloud);
	for (pcl::PointXYZ pt : skelCloud->points) {
		pt.x -= midPoint.x;
		pt.y -= midPoint.y;
		pt.z -= midPoint.z;
		pt.x /= skelScale;
		pt.y /= skelScale;
		pt.z /= skelScale;
	}//move it to zero and scale in

	viewer->removeAllPointClouds();
	viewer->removePointCloud("cloud");
	viewer->updatePointCloud(cloud, "cloud");
	viewer->addPointCloud(cloud, "cloud");
	viewer->resetCamera();
	this->color(cloud, 250, 140, 20);
}

void QT_PCL_Segmentation::noise() {
	int dx = ui.noise->toPlainText().toInt();
	//showDemo();
	cloudfiltered->points.resize(cloud->points.size());//将点云的cloud的size赋值给噪声
	cloudfiltered->header = cloud->header;
	cloudfiltered->width = cloud->width;
	cloudfiltered->height = cloud->height;
	boost::mt19937 rng;
	rng.seed(static_cast<unsigned int>(time(0)));
	boost::normal_distribution<> nd(0, 2);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);
	//添加噪声
	for (size_t point_i = 0; point_i < cloud->points.size(); ++point_i)
	{
		cloudfiltered->points[point_i].x = cloud->points[point_i].x + static_cast<float> (var_nor()) / dx;
		cloudfiltered->points[point_i].y = cloud->points[point_i].y + static_cast<float> (var_nor()) / dx;
		cloudfiltered->points[point_i].z = cloud->points[point_i].z + static_cast<float> (var_nor()) / dx;
	}
	viewer->removeAllPointClouds();
	viewer->removePointCloud("cloudfiltered");
	viewer->updatePointCloud(cloudfiltered, "cloudfiltered");
	viewer->addPointCloud(cloudfiltered, "cloudfiltered");
	//viewer->resetCamera();
	this->color(cloudfiltered, 250, 140, 20);
	//this->color(cloudfiltered, 255, 0, 0);
	pcl::io::savePLYFileASCII(this->cloudPath.substr(0, this->cloudPath.length() - 4) + "_noise.ply", *cloudfiltered);
}

void QT_PCL_Segmentation::outlier() {
	//showDemo();
	srand((int)time(0));
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointAddCloud(new pcl::PointCloud<pcl::PointXYZ>());
	for (int i = 0; i < ui.outlier->toPlainText().toInt(); ++i) {
		pcl::PointXYZ pt((0.5 * rand() / (RAND_MAX + 1.0) + 0.5)*0.8,
			(0.5 * rand() / (RAND_MAX + 1.0) + 0.5)*0.8,
			(0.5 * rand() / (RAND_MAX + 1.0) + 0.5)*0.8);
		if (rand() / (RAND_MAX + 1.0) > 0.5) pt.x *= -1;
		if (rand() / (RAND_MAX + 1.0) > 0.5) pt.y *= -1;
		if (rand() / (RAND_MAX + 1.0) > 0.5) pt.z *= -1;
		viewer->addSphere(pt, skelSize * 0.7, 0, 0, 135, "sphere" + std::to_string(i + 100));
		pointAddCloud->points.push_back(pt);
	}
	viewer->removePointCloud("pointAddCloud");
	viewer->updatePointCloud(pointAddCloud, "pointAddCloud");
	viewer->addPointCloud(pointAddCloud, "pointAddCloud");
	this->color(pointAddCloud, 0, 0, 255);


	//viewer->resetCamera();
	//this->color(cloud, 250, 140, 20);
}

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

void QT_PCL_Segmentation::off_obj(std::string input)
{
	/*函数说明：读取off文件
	 * */
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

void QT_PCL_Segmentation::off_ply() {
	this->showDemo();
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open off file"), ".",
		tr("Open PCD files(*.off)"));
	std::string file_name = fileName.toStdString();
	off_obj(file_name);
	obj_pcd(file_name);
	pcd_ply(file_name);
}

void QT_PCL_Segmentation::offReader(std::string filename)
{
	this->cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
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
		this->cloud->points.push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
	}
	pcl::PLYWriter writer;
	//writer.write(filename.substr(0,filename.length()-4)+".ply", *cloud);
	pcl::io::savePLYFileASCII(filename.substr(0, filename.length() - 4) + ".ply", *cloud);
}

void QT_PCL_Segmentation::onOpenOff() {
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open OFF files(*.off)"));
	std::string file_name = fileName.toStdString();

	this->offReader(file_name);
	//off_ply();

	viewer->removePointCloud("cloud");
	viewer->updatePointCloud(cloud, "cloud");
	viewer->addPointCloud(cloud, "cloud");
	viewer->resetCamera();
	this->color(cloud, 250, 140, 20);
}

void QT_PCL_Segmentation::onSaveNoff() {
	saveNoff(this->cloudPath);
}

void QT_PCL_Segmentation::saveNoff(std::string filename) {
	computeNormal();
	ofstream out;
	out.open(filename.substr(0, filename.length() - 3) + "off", ios::trunc);
	out << "NOFF" << endl;
	out << this->cloud->points.size() << " " << 0 << endl;
	this->cloud = normalize(this->cloud, false);
	for (int i = 0; i < this->cloud->points.size(); ++i)
	{
		out << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " "
			<< -1 * ((normalCloud->points[i].normal_x == NAN) ? 0.1 : normalCloud->points[i].normal_x) << " "
			<< -1 * ((normalCloud->points[i].normal_y == NAN) ? -0.1 : normalCloud->points[i].normal_y) << " "
			<< -1 * ((normalCloud->points[i].normal_z == NAN) ? 0.1 : normalCloud->points[i].normal_z) << " "
			<< endl;
	}
	out.close();
}

void QT_PCL_Segmentation::computeNormal() {
	//计算法线
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZRGB  pt;
		pt.x = cloud->points[i].x;
		pt.y = cloud->points[i].y;
		pt.z = cloud->points[i].z;
		pt.r = 250;
		pt.g = 140;
		pt.b = 20;
		colored_cloud->push_back(pt);
	}
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(colored_cloud);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*this->normalCloud);
}

void QT_PCL_Segmentation::onDownSample() {
	downSample(cloudPath);

	pcl::PCLPointCloud2 cloud2;
	//pcl::PointCloud<Eigen::MatrixXf> cloud2;
	Eigen::Vector4f origin;
	Eigen::Quaternionf orientation;
	int pcd_version;
	int data_type;
	unsigned int data_idx;
	int offset = 0;
	pcl::PCDReader rd;
	std::string file_name = cloudPath.substr(0, cloudPath.length() - 4) + "_down.pcd";
	this->cloudPath = file_name;
	this->modelSkelName = file_name.substr(0, file_name.length() - 4);
	rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);
	if (data_type == 0)
		pcl::io::loadPCDFile(file_name, *cloud);
	else if (data_type == 2) {
		pcl::PCDReader reader;
		reader.read<pcl::PointXYZ>(file_name, *cloud);
	}
	pcl::io::savePLYFileASCII(file_name.substr(0, file_name.length() - 4) + ".ply", *cloud);

	clearPointCloud();
	viewer->removePointCloud("cloud");
	viewer->updatePointCloud(cloud, "cloud");
	viewer->addPointCloud(cloud, "cloud");
	viewer->resetCamera();
	this->color(cloud, 250, 140, 20);
}

void QT_PCL_Segmentation::downSample(std::string path) {
	pcl::PCLPointCloud2::Ptr plyCloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// Fill in the cloud data
	pcl::PLYReader reader;
	// Replace the path below with the path where you saved your file
	reader.read(path, *plyCloud); // Remember to download the file first!

	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(plyCloud);
	float num = ui.leafSize->toPlainText().toFloat();
	sor.setLeafSize(num, num, num);
	sor.filter(*cloud_filtered);

	pcl::PCDWriter writer;
	writer.write(path.substr(0, path.length() - 4) + "_down.pcd", *cloud_filtered,
		Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
}

void QT_PCL_Segmentation::onRandomMissing() {
	pcl::PointCloud <pcl::PointXYZ>::Ptr missingCloud(new pcl::PointCloud <pcl::PointXYZ>);
	for (int i = 1; i < 5; ++i) {
		missingCloud->clear();
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			pcl::PointXYZ  pt;
			pt.x = cloud->points[i].x;
			pt.y = cloud->points[i].y;
			pt.z = cloud->points[i].z;
			if (rand() / (RAND_MAX + 1.0f) > 0.3)
				missingCloud->push_back(pt);
		}
		pcl::io::savePLYFileASCII(this->cloudPath.substr(0, this->cloudPath.length() - 4) + "_miss" + std::to_string(i) + ".ply", *missingCloud);
	}
}

void QT_PCL_Segmentation::onOpenTxt() {
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PCD files(*.txt)"));
	std::string file_name = fileName.toStdString();
	this->cloud->points.clear();

	ifstream in(file_name);

	float pos[3];
	for (int i = 0; i < 24000; ++i)
	{
		in >> pos[0] >> pos[1] >> pos[2];
		this->cloud->push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
	}

	viewer->removePointCloud("cloud");
	viewer->updatePointCloud(cloud, "cloud");
	viewer->addPointCloud(cloud, "cloud");
	viewer->resetCamera();
	this->color(cloud, 250, 140, 20);

	pcl::io::savePLYFileASCII(file_name.substr(0, file_name.length() - 4) + ".ply", *cloud);
}

void QT_PCL_Segmentation::onSavePLY() {
	pcl::io::savePLYFileASCII(this->cloudPath.substr(0, this->cloudPath.length() - 4) + ".ply", *cloud);
}

