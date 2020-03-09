#include "QT_PCL_Segmentation.h"
#include <QMessageBox>
#include <QFileDialog>

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL)
VTK_MODULE_INIT(vtkInteractionStyle)

#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>

#include <time.h>
#include <stdlib.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "MedialThread.h"


using namespace std;
using namespace pcl;
using namespace Eigen;



QT_PCL_Segmentation::QT_PCL_Segmentation(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//初始化
	paraMgr = new ParameterMgr;
	initialVtkWidget();
	viewer->setBackgroundColor(255, 255, 255);
	//this->colorFlag = false;
	this->colorCloudIndex = 0;
	//this->skelIndex = -1;
	//this->skelSize = 0.01;
	bool ok = true;
	//this->kmeansRadius = ui.Radius_2->toPlainText().toDouble(&ok);
	//this->c = -1 * ui.cValue->toPlainText().toDouble(&ok);
	//连接按钮	
	connect(ui.actionopen, SIGNAL(triggered()), this, SLOT(onOpen()));
	//connect(ui.actionnormalize, SIGNAL(triggered()), this, SLOT(normalizeOfSkel()));
	connect(ui.actionoff_ply, SIGNAL(triggered()), this, SLOT(onOpenOff()));
	connect(ui.actionsave_NOFF, SIGNAL(triggered()), this, SLOT(onSaveNoff()));
	connect(ui.actiondown_sample, SIGNAL(triggered()), this, SLOT(onDownSample()));
	connect(ui.actionrandom_missing, SIGNAL(triggered()), this, SLOT(onRandomMissing()));
	connect(ui.actionopen_txt, SIGNAL(triggered()), this, SLOT(onOpenTxt()));


	//connect(ui.segButton, SIGNAL(clicked()), this, SLOT(kmeans()));
	//connect(ui.segButton_2, SIGNAL(clicked()), this, SLOT(segmentation()));
	connect(ui.segButton_3, SIGNAL(clicked()), this, SLOT(onRandomSample()));

	//connect(ui.drawButton, SIGNAL(clicked()), this, SLOT(onL1()));
	//connect(ui.drawButton_6, SIGNAL(clicked()), this, SLOT(BayesSkel()));
	// connect(ui.drawButton_3, SIGNAL(clicked()), this, SLOT(reDrawSkel()));

	connect(ui.clearButton, SIGNAL(clicked()), this, SLOT(clearPointCloud()));
	connect(ui.showButton, SIGNAL(clicked()), this, SLOT(showPCL()));
	connect(ui.resetButton, SIGNAL(clicked()), this, SLOT(resetPointCloud()));

	connect(ui.noiseButton, SIGNAL(clicked()), this, SLOT(noise()));
	connect(ui.outlierButton, SIGNAL(clicked()), this, SLOT(outlier()));
	connect(ui.downSampleButton, SIGNAL(clicked()), this, SLOT(onDownSample()));

	connect(this, SIGNAL(updateSignal()), this, SLOT(onUpdate()));

}

void QT_PCL_Segmentation::initialVtkWidget()
{
	originCloud.reset(new PointCloud<PointXYZ>);
	sampleCloud.reset(new PointCloud<PointXYZ>);

	//skelCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	//cloudfiltered.reset(new pcl::PointCloud<pcl::PointXYZ>);
	//normalCloud.reset(new pcl::PointCloud<pcl::Normal>);
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

	this->tabifyDockWidget(ui.skelDock, ui.skelDock2);
}

//onOpen
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

		viewer->removePointCloud("cloud");
		viewer->updatePointCloud(originCloud, "cloud");
		viewer->addPointCloud(originCloud, "cloud");
		viewer->resetCamera();
		this->color(originCloud, 250, 140, 20);
		//ui.qvtkWidget->update();


		//this->color(skelCloud, 5, 115, 235);
	}
}


void outputInfo(const vector<SamplePoint> info, QTextEdit* qte) {
	int num = 0;
	for (SamplePoint pi : info) {
		num++;
		qte->append(
			"["+QString::number(num)+"]"+
			"sigma: "+QString::number(pi.getSigma())+
			"alpha_size: "+ QString::number(pi.alpha.size())+
			"beta_size: "+ QString::number(pi.beta.size())+
			"o_neigh_size:"+ QString::number(pi.getOriginalIndics().size())+
			"s_neigh_size:"+ QString::number(pi.getSelfIndics().size())+"\n"
		);
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


void QT_PCL_Segmentation::colorByAxis()
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
	viewer->removeShape("sp0");
	viewer->addSphere(inCloud->points[0], 0.0000006*originCloud->width, 0, 135, 0, "sp0");

	ui.qvtkWidget->update();
}

// 随机采样函数
PcPtr randomSampling(PcPtr inCloud, int num = 1000) {
	PcPtr sampleCloud(new PointCloud<PointXYZ>);
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

void QT_PCL_Segmentation::onRandomSample() {
	ui.InfoText->append("random sampling starts");
	this->sampleCloud = randomSampling(this->originCloud);
	displaySampleCloud(this->sampleCloud);
	color(this->sampleCloud, 155, 0, 0);
	ui.InfoText->append("random sampling ends");
}

pcl::PointXYZ median(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
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
	this->color(originCloud, 250, 140, 20);
	viewer->resetCamera();
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
		viewer->addSphere(pt, 0.5, 0, 0, 135, "sphere" + std::to_string(i + 100));
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
	this->originCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
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
		this->originCloud->points.push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
	}
	pcl::PLYWriter writer;
	//writer.write(filename.substr(0,filename.length()-4)+".ply", *cloud);
	pcl::io::savePLYFileASCII(filename.substr(0, filename.length() - 4) + ".ply", *originCloud);
}

void QT_PCL_Segmentation::onOpenOff() {
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open OFF files(*.off)"));
	std::string file_name = fileName.toStdString();

	this->offReader(file_name);
	//off_ply();

	viewer->removePointCloud("cloud");
	viewer->updatePointCloud(originCloud, "cloud");
	viewer->addPointCloud(originCloud, "cloud");
	viewer->resetCamera();
	this->color(originCloud, 250, 140, 20);
}

void QT_PCL_Segmentation::onSaveNoff() {
	saveNoff(this->cloudPath);
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

	clearPointCloud();
	viewer->removePointCloud("cloud");
	viewer->updatePointCloud(originCloud, "cloud");
	viewer->addPointCloud(originCloud, "cloud");
	viewer->resetCamera();
	this->color(originCloud, 250, 140, 20);
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

void QT_PCL_Segmentation::onUpdate() {
	this->displaySampleCloud(this->sampleCloud);
	/*ui.InfoText->append(
		"坐标: (" + 
		QString::number(this->sampleCloud->points[0].x) + ", " + 
		QString::number(this->sampleCloud->points[0].y) + ", " + 
		QString::number(this->sampleCloud->points[0].z) + ")\n"+
		"alhpa size:" + QString::number(this->xInfo[0].alpha.size()) +
		"beta size:" + QString::number(this->xInfo[0].beta.size())
	);*/
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

	viewer->removePointCloud("cloud");
	viewer->updatePointCloud(originCloud, "cloud");
	viewer->addPointCloud(originCloud, "cloud");
	viewer->resetCamera();
	this->color(originCloud, 250, 140, 20);

	pcl::io::savePLYFileASCII(file_name.substr(0, file_name.length() - 4) + ".ply", *originCloud);
}

void QT_PCL_Segmentation::onSavePLY() {
	pcl::io::savePLYFileASCII(this->cloudPath.substr(0, this->cloudPath.length() - 4) + ".ply", *originCloud);
}

