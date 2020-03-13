#include "QT_PCL_Segmentation.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QTableWidget>

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL)
VTK_MODULE_INIT(vtkInteractionStyle)

#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>

#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "AlgorithmThread.h"
#include "TestMoving.h"
#include "GlobalDef.h"
#include "GlobalFun.h"


using namespace std;
using namespace pcl;
using namespace Eigen;



QT_PCL_Segmentation::QT_PCL_Segmentation(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//初始化
	initParaMgr();
	initialVtkWidget();
	initParaDockWithParaMgr(this->paraMgr);

	/*读取，保存等功能（文件）*/
	connect(ui.actionopen, SIGNAL(triggered()), this, SLOT(onOpen()));
	//connect(ui.actionnormalize, SIGNAL(triggered()), this, SLOT(normalizeOfSkel()));
	connect(ui.actionoff_ply, SIGNAL(triggered()), this, SLOT(onOpenOff()));
	connect(ui.actionsave_NOFF, SIGNAL(triggered()), this, SLOT(onSaveNoff()));
	connect(ui.actiondown_sample, SIGNAL(triggered()), this, SLOT(onDownSample()));
	connect(ui.actionrandom_missing, SIGNAL(triggered()), this, SLOT(onRandomMissing()));
	connect(ui.actionopen_txt, SIGNAL(triggered()), this, SLOT(onOpenTxt()));

	/*按钮的connect*/
	connect(ui.segButton_3, SIGNAL(clicked()), this, SLOT(onRandomSample()));

	connect(ui.drawButton, SIGNAL(clicked()), this, SLOT(onL1()));
	connect(ui.drawButton_2, SIGNAL(clicked()), this, SLOT(onMoving()));

	connect(ui.clearButton, SIGNAL(clicked()), this, SLOT(clearPointCloud()));
	connect(ui.showButton, SIGNAL(clicked()), this, SLOT(testPCLready()));
	connect(ui.resetButton, SIGNAL(clicked()), this, SLOT(resetPointCloud()));

	connect(ui.noiseButton, SIGNAL(clicked()), this, SLOT(noise()));
	connect(ui.outlierButton, SIGNAL(clicked()), this, SLOT(outlier()));
	connect(ui.downSampleButton, SIGNAL(clicked()), this, SLOT(onDownSample()));

	/*事件信号处理*/
	connect(ui.menuVis, SIGNAL(aboutToShow()), this, SLOT(updateBarStatus()));
	vector<QAction*> actionList = {
		ui.actionsample, ui.actionseg,
		ui.actionother, ui.actioninfo,
		ui.actionL1, ui.actionBayes
	};
	for (int i = 0; i < actionList.size(); ++i) {
		connect(actionList[i], SIGNAL(changed()), this, SLOT(updateAllDockWidget()));
	}
}

void QT_PCL_Segmentation::initialVtkWidget()
{
	originCloud.reset(new PointCloud<PointXYZ>);
	sampleCloud.reset(new PointCloud<PointXYZ>);
	cloudfiltered.reset(new pcl::PointCloud<pcl::PointXYZ>);
	normalCloud.reset(new pcl::PointCloud<pcl::Normal>);

	//skelCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	//viewer->addPointCloud(cloud, "cloud");

	QWidget* titleWidget = new QWidget(this);
	QWidget *visTitlebar = ui.visDock->titleBarWidget();
	ui.visDock->setTitleBarWidget(titleWidget);
	delete visTitlebar;

	ui.visDock->setWidget(ui.qvtkWidget);
	
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	ui.qvtkWidget->update();

	this->tabifyDockWidget(ui.skelDock2, ui.skelDock);

	viewer->setBackgroundColor(255, 255, 255);
}

/******算法部分******/
//L1median
void QT_PCL_Segmentation::onL1() 
{

}
//TestMoving
void QT_PCL_Segmentation::onMoving()
{
	ParameterSet para = this->paraMgr->getSubSet(GlobalDef::TestMoving);

	if (this->algorithm != NULL) delete this->algorithm;
	if (this->sampleCloud->points.size() < 1) onRandomSample();
	this->algorithm = new TestMoving(para, this->sampleCloud);
	if (!connect(this->algorithm, SIGNAL(iterateSignal()), this, SLOT(onUpdate())))
	{
		ui.InfoText->append("algorithm connect wrong!");
	}

	AlgorithmThread *at = new AlgorithmThread(this->algorithm);
	this->moveToThread(at);
	at->start();

}

//一些简单的小功能
void QT_PCL_Segmentation::onRandomSample() {
	int num = ui.sampleNum->toPlainText().toInt();
	ui.InfoText->append("random sampling starts");
	this->sampleCloud = GlobalFun::randomSampling(this->originCloud, num);
	displaySampleCloud(this->sampleCloud);
	ui.InfoText->append("random sampling ends");
}

void QT_PCL_Segmentation::onRandomMissing() {
	pcl::PointCloud <pcl::PointXYZ>::Ptr missingCloud(new pcl::PointCloud <pcl::PointXYZ>);
	for (int i = 1; i < 5; ++i) {
		missingCloud->clear();
		for (size_t i = 0; i < originCloud->points.size(); ++i)
		{
			pcl::PointXYZ  pt;
			pt.x = originCloud->points[i].x;
			pt.y = originCloud->points[i].y;
			pt.z = originCloud->points[i].z;
			if (rand() / (RAND_MAX + 1.0f) > 0.3)
				missingCloud->push_back(pt);
		}
		pcl::io::savePLYFileASCII(this->cloudPath.substr(0, this->cloudPath.length() - 4) + "_miss" + std::to_string(i) + ".ply", *missingCloud);
	}
}

void QT_PCL_Segmentation::noise() {
	int dx = ui.noise->toPlainText().toInt();
	//showDemo();
	cloudfiltered->points.resize(originCloud->points.size());//将点云的cloud的size赋值给噪声
	cloudfiltered->header = originCloud->header;
	cloudfiltered->width = originCloud->width;
	cloudfiltered->height = originCloud->height;
	boost::mt19937 rng;
	rng.seed(static_cast<unsigned int>(time(0)));
	boost::normal_distribution<> nd(0, 2);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);
	//添加噪声
	for (size_t point_i = 0; point_i < originCloud->points.size(); ++point_i)
	{
		cloudfiltered->points[point_i].x = originCloud->points[point_i].x + static_cast<float> (var_nor()) / dx;
		cloudfiltered->points[point_i].y = originCloud->points[point_i].y + static_cast<float> (var_nor()) / dx;
		cloudfiltered->points[point_i].z = originCloud->points[point_i].z + static_cast<float> (var_nor()) / dx;
	}

	this->displaySelectedCloud(cloudfiltered, { 255, 0, 0 }, 1, "noises");
	//pcl::io::savePLYFileASCII(this->cloudPath.substr(0, this->cloudPath.length() - 4) + "_noise.ply", *cloudfiltered);
}

void QT_PCL_Segmentation::outlier() {
	//showDemo();
	srand((int)time(0));
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointAddCloud(new pcl::PointCloud<pcl::PointXYZ>());
	for (int i = 0; i < ui.outlier->toPlainText().toInt(); ++i) {
		pcl::PointXYZ pt((0.5 * rand() / (RAND_MAX + 1.0) + 0.5)*0.7,
			(0.5 * rand() / (RAND_MAX + 1.0) + 0.5)*0.7,
			(0.5 * rand() / (RAND_MAX + 1.0) + 0.5)*0.7);
		if (rand() / (RAND_MAX + 1.0) > 0.5) pt.x *= -1;
		if (rand() / (RAND_MAX + 1.0) > 0.5) pt.y *= -1;
		if (rand() / (RAND_MAX + 1.0) > 0.5) pt.z *= -1;
		pointAddCloud->points.push_back(pt);
	}
	this->cloudfiltered = pointAddCloud;
	this->displaySelectedCloud(cloudfiltered, { 0,0,255 }, 4, "outliers");

	//viewer->resetCamera();
	//this->color(cloud, 250, 140, 20);
}

void QT_PCL_Segmentation::computeNormal() {
	//计算法线
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	for (size_t i = 0; i < originCloud->points.size(); ++i)
	{
		pcl::PointXYZRGB  pt;
		pt.x = originCloud->points[i].x;
		pt.y = originCloud->points[i].y;
		pt.z = originCloud->points[i].z;
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
	//this->modelSkelName = file_name.substr(0, file_name.length() - 4);
	rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);
	if (data_type == 0)
		pcl::io::loadPCDFile(file_name, *originCloud);
	else if (data_type == 2) {
		pcl::PCDReader reader;
		reader.read<pcl::PointXYZ>(file_name, *originCloud);
	}
	pcl::io::savePLYFileASCII(file_name.substr(0, file_name.length() - 4) + ".ply", *originCloud);

	this->displaySelectedCloud(originCloud, { 250, 140, 20 }, 1, "cloud");
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


/******文件部分******/
//打开pcd, ply
void QT_PCL_Segmentation::onOpen()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PCD files(*.pcd *.ply)"));
	std::string file_name = fileName.toStdString();
	this->cloudPath = file_name;
	//this->modelSkelName = file_name.substr(0, file_name.length() - 4);
	//this->modelSkelName += ".txt";

	if (!fileName.isEmpty())
	{
		if (fileName.indexOf(".pcd") != -1)
		{
			pcl::PCLPointCloud2 cloud2;
			//pcl::PointCloud<Eigen::MatrixXf> cloud2;
			Eigen::Vector4f ori;
			Eigen::Quaternionf orientation;
			int pcd_version;
			int data_type;
			unsigned int data_idx;
			int offset = 0;
			pcl::PCDReader rd;
			rd.readHeader(file_name, cloud2, ori, orientation, pcd_version, data_type, data_idx);

			if (data_type == 0)
			{
				pcl::io::loadPCDFile(file_name, *originCloud);
			}
			else if (data_type == 2)
			{
				pcl::PCDReader reader;
				reader.read<pcl::PointXYZ>(file_name, *originCloud);
			}
			pcl::io::savePLYFileASCII(file_name.substr(0, file_name.length() - 4) + ".ply", *originCloud);
		}
		else//PLY read 
		{
			pcl::PLYReader yrd;
			yrd.read<pcl::PointXYZ>(file_name, *originCloud);
		}
		//correctCenter(cloud);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);		

		displayOriginCloud(originCloud);
		viewer->resetCamera();
		//ui.qvtkWidget->update();


		//this->color(skelCloud, 5, 115, 235);
	}
}
//保存ply文件
void QT_PCL_Segmentation::onSavePLY() {
	pcl::io::savePLYFileASCII(this->cloudPath.substr(0, this->cloudPath.length() - 4) + ".ply", *originCloud);
}
//打开off文件
void QT_PCL_Segmentation::onOpenOff() {
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open OFF files(*.off)"));
	std::string file_name = fileName.toStdString();

	GlobalFun::readOFF(file_name);

	displaySelectedCloud(originCloud, { 250, 140, 20 }, 1, "cloud");

}
//保存off格式的数据(ROSA算法使用格式)
void QT_PCL_Segmentation::onSaveNoff() {
	saveNoff(this->cloudPath);
}
//off转ply
void QT_PCL_Segmentation::off_ply() {
	this->testUIready();
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open off file"), ".",
		tr("Open PointCloud files(*.off)"));
	std::string file_name = fileName.toStdString();
	GlobalFun::off_obj(file_name);
	GlobalFun::obj_pcd(file_name);
	GlobalFun::pcd_ply(file_name);
}
//打开txt文件
void QT_PCL_Segmentation::onOpenTxt() {
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PCD files(*.txt)"));
	std::string file_name = fileName.toStdString();
	this->originCloud->points.clear();

	ifstream in(file_name);

	float pos[3];
	for (int i = 0; i < 24000; ++i)
	{
		in >> pos[0] >> pos[1] >> pos[2];
		this->originCloud->push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
	}

	displayOriginCloud(originCloud);
}

void QT_PCL_Segmentation::saveNoff(std::string filename) {
	computeNormal();
	ofstream out;
	out.open(filename.substr(0, filename.length() - 3) + "off", ios::trunc);
	out << "NOFF" << endl;
	out << this->originCloud->points.size() << " " << 0 << endl;
	this->originCloud = normalize(this->originCloud, false);
	for (int i = 0; i < this->originCloud->points.size(); ++i)
	{
		out << originCloud->points[i].x << " " << originCloud->points[i].y << " " << originCloud->points[i].z << " "
			<< -1 * ((normalCloud->points[i].normal_x == NAN) ? 0.1 : normalCloud->points[i].normal_x) << " "
			<< -1 * ((normalCloud->points[i].normal_y == NAN) ? -0.1 : normalCloud->points[i].normal_y) << " "
			<< -1 * ((normalCloud->points[i].normal_z == NAN) ? 0.1 : normalCloud->points[i].normal_z) << " "
			<< endl;
	}
	out.close();
}


/******测试函数******/
void QT_PCL_Segmentation::testUIready()
{
	ui.InfoText->append("\ndemo show");
	QMessageBox msg;
	msg.setText("HelloWord!");
	msg.exec();
}

void QT_PCL_Segmentation::testPCLready()
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

void QT_PCL_Segmentation::testColorAmongAxis()
{
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	//pcl::visualization::PCLVisualizer::Ptr rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr colored_cloud);
	for (size_t i = 0; i < originCloud->points.size(); ++i)
	{
		pcl::PointXYZRGB  pt;
		pt.x = this->originCloud->points[i].x;
		pt.y = this->originCloud->points[i].y;
		pt.z = this->originCloud->points[i].z;
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

void QT_PCL_Segmentation::testSkelPainting()
{
	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);
	cylinder_coeff.values[0] = originCloud->points[0].x;
	cylinder_coeff.values[1] = originCloud->points[0].y;
	cylinder_coeff.values[2] = originCloud->points[0].z;
	cylinder_coeff.values[3] = originCloud->points[5].x - originCloud->points[0].x;
	cylinder_coeff.values[4] = originCloud->points[5].y - originCloud->points[0].y;
	cylinder_coeff.values[5] = originCloud->points[5].z - originCloud->points[0].z;
	cylinder_coeff.values[6] = 0.0000002*originCloud->width;
	viewer->addCylinder(cylinder_coeff, "axisX");
	viewer->addSphere(originCloud->points[0], 0.0000006*originCloud->width, 0, 135, 0, "sphere" + 0);
	viewer->addSphere(originCloud->points[5], 0.0000006*originCloud->width, 0, 135, 0, "sphere" + 5);

	ui.qvtkWidget->update();
}



/******UI相关+参数/显示更新******/
void QT_PCL_Segmentation::displaySelectedCloud(const PointCloud<PointXYZ>::Ptr inCloud,
	const pi::RGB color, double size, string tag) {
	viewer->removePointCloud(tag);
	viewer->addPointCloud(inCloud, tag);
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, size, tag);
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR,
		color.r / 255.0, color.g / 255.0, color.b / 255.0, tag);
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::displayOriginCloud(PointCloud<PointXYZ>::Ptr cloud)
{
	this->displaySelectedCloud(cloud, { 250, 140, 20 }, 1, "cloud");
}

void QT_PCL_Segmentation::displaySampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud) {
	viewer->removePointCloud("sample_cloud");
	viewer->addPointCloud(inCloud, "sample_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sample_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample_cloud");
	viewer->removeShape("sp0");
	viewer->addSphere(inCloud->points[0], 0.0000006*originCloud->width, 0, 135, 0, "sp0");

	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::clearPointCloud()
{
	this->sampleCloud.reset(new PointCloud<PointXYZ>);
	this->cloudfiltered.reset(new PointCloud<PointXYZ>);
	this->normalCloud.reset(new PointCloud<Normal>);
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	this->displaySelectedCloud(originCloud, { 250, 140, 20 }, 1, "cloud");
	viewer->resetCamera();

	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::resetPointCloud()
{
	viewer->resetCamera();
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::onUpdate() {
	this->displaySampleCloud(this->sampleCloud);
	ui.InfoText->append(
		"坐标: (" + 
		QString::number(this->sampleCloud->points[0].x) + ", " + 
		QString::number(this->sampleCloud->points[0].y) + ", " + 
		QString::number(this->sampleCloud->points[0].z) + ")\n"
	);
}

void QT_PCL_Segmentation::updateAllDockWidget()
{
	vector<QDockWidget*> dockList = {
		ui.sampleDock, ui.segDock,
		ui.otherDock, ui.infoDock,
		ui.skelDock, ui.skelDock2
	};
	vector<QAction*> actionList = {
		ui.actionsample, ui.actionseg,
		ui.actionother, ui.actioninfo,
		ui.actionL1, ui.actionBayes
	};
	for (int i = 0; i < actionList.size(); ++i) {
		dockList[i]->setVisible(actionList[i]->isChecked());
	}
}

void QT_PCL_Segmentation::updateBarStatus()
{
	//ui.InfoText->append("updateBarStatus");
	vector<QDockWidget*> dockList = { 
		ui.sampleDock, ui.segDock, 
		ui.otherDock, ui.infoDock,
		ui.skelDock, ui.skelDock2
	};
	vector<QAction*> actionList = {
		ui.actionsample, ui.actionseg,
		ui.actionother, ui.actioninfo,
		ui.actionL1, ui.actionBayes 
	};

	for (int i = 0; i < dockList.size(); ++i) {
		actionList[i]->setChecked(dockList[i]->isVisible());
	}

}

void QT_PCL_Segmentation::initParaMgr()
{
	if (paraMgr != NULL) delete paraMgr;
	paraMgr = new ParameterMgr;
	for (pair<string, vector<pair<string, Value>>> para_subSet : GlobalDef::TOTAL_PARA_LIST) {
		this->paraMgr->addSubSet(para_subSet.first);
		for (pair<string, Value> para_pair : para_subSet.second) {
			this->paraMgr->add(para_pair.first, para_pair.second);
		}
	}
}

void QT_PCL_Segmentation::updateParameterMgr()
{
	// TO-DO
}

void QT_PCL_Segmentation::initParaDockWithParaMgr(ParameterMgr* pm)
{
	ui.InfoText->append("Paramter DockWidget initialized!\n");
	if (paraDock != NULL) delete paraDock;
	paraDock = new QDockWidget(tr("参数面板"), this);
	QTableWidget *tw = new QTableWidget();
	tw->setColumnCount(2);
	QStringList header;
	header << "paramter" << "value";
	tw->setHorizontalHeaderLabels(header);

	map<string, ParameterSet> para_data = pm->getALLSubSet();
	int rowNo = 0;
	for (map<string, ParameterSet>::iterator iter = para_data.begin(); 
			iter != para_data.end(); ++iter) {
		string subset_name = iter->first;
		map<string, Value> para_map = iter->second.getALLParameter();

		tw->insertRow(rowNo);
		tw->setSpan(rowNo, 0, 1, 2);
		tw->setItem(rowNo, 0, new QTableWidgetItem(QString::fromStdString(subset_name)));
		rowNo++;

		for (map<string, Value>::iterator para = para_map.begin();
				para != para_map.end(); ++para) {
			string para_name = para->first;
			Value para_val = para->second;

			tw->insertRow(rowNo);
			tw->setItem(rowNo, 0, new QTableWidgetItem(QString::fromStdString(para_name)));
			tw->item(rowNo, 0)->setFlags(Qt::NoItemFlags);
			switch (para_val.type) {
			case ParaType::intType:
				tw->setItem(rowNo, 1, new QTableWidgetItem(QString::number(para_val.toInt())));
				break;
			case ParaType::floatType:
				tw->setItem(rowNo, 1, new QTableWidgetItem(QString::number(para_val.toFloat())));
				break;
			case ParaType::doubleType:
				tw->setItem(rowNo, 1, new QTableWidgetItem(QString::number(para_val.toDouble())));
				break;
			default:
				tw->setItem(rowNo, 1, new QTableWidgetItem("WRONG"));
			}
			rowNo++;
		}
	}
	paraDock->setWidget(tw);
	paraDock->setFloating(true);
	paraDock->resize(240, 300);
	paraDock->show();
}

