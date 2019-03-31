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

#include<time.h>
#include<stdlib.h>
#include <math.h>

typedef struct RGB {
	int r;
	int g;
	int b;
}RGB;

QT_PCL_Segmentation::QT_PCL_Segmentation(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//³õÊ¼»¯
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

	connect(ui.segButton, SIGNAL(clicked()), this, SLOT(kmeans()));
	connect(ui.segButton_2, SIGNAL(clicked()), this, SLOT(segmentation()));
	connect(ui.segButton_3, SIGNAL(clicked()), this, SLOT(KNNsmooth()));

	connect(ui.drawButton, SIGNAL(clicked()), this, SLOT(drawSkel()));
	connect(ui.drawButton_2, SIGNAL(clicked()), this, SLOT(BayesSkel()));
	connect(ui.drawButton_3, SIGNAL(clicked()), this, SLOT(reDrawSkel()));

	connect(ui.clearButton, SIGNAL(clicked()), this, SLOT(clearPointCloud()));
	connect(ui.showButton, SIGNAL(clicked()), this, SLOT(showPCL()));
	connect(ui.resetButton, SIGNAL(clicked()), this, SLOT(resetPointCloud()));
}

void QT_PCL_Segmentation::initialVtkWidget()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	skelCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
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

	QString text = "Point cloud data: "+QString::number(cloud->points.size())+" points\n";
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
	point.x +=0.1;
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
		pt.r = (pt.x>0)?255:144;
		pt.g = (pt.y>0)?127:255;
		pt.b = (pt.z>0)?39:177;
		colored_cloud->push_back(pt);
	}
	viewer->removePointCloud("colored_cloud");
	viewer->updatePointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud,"colored_cloud");
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

void QT_PCL_Segmentation::color(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud,int r,int g,int b)
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
	viewer->updatePointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud"+ this->colorCloudIndex);
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud"+ this->colorCloudIndex);
	this->colorCloudIndex++;
	ui.qvtkWidget->update();
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
	cylinder_coeff.values[3] = cloud->points[5].x-cloud->points[0].x;
	cylinder_coeff.values[4] = cloud->points[5].y - cloud->points[0].y;
	cylinder_coeff.values[5] = cloud->points[5].z - cloud->points[0].z;
	cylinder_coeff.values[6] = 0.0000002*cloud->width;
	viewer->addCylinder(cylinder_coeff,"axisX");
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
	int avg_clusterSize = cloud->points.size()/num;
	int avg_clusterColor = 255 / num;
	std::vector<pcl::PointXYZ> center;
	std::vector<RGB> clusterColor(num, {0,0,0});
	std::vector<int> clusterSize(num,0);
	this->tag = std::vector<int>(cloud->points.size(),-1);
	//³õÊ¼»¯num¸ö´ØÐÄ
	ui.InfoText->append("\ndefault centers choosing start");
	for (int i = 0; i < num; ++i)
	{
		srand((int)time(0));
		int centerIndex = -1;
		do{
			centerIndex = (int)(cloud->points.size() * rand() / (RAND_MAX + 1.0));
		} while (tag[centerIndex] != -1);
		tag[centerIndex] = i;
		center.push_back(cloud->points[centerIndex]);
		//tag[centerIndex] = i;
		clusterColor[i] = { (int)(avg_clusterColor * rand() / (RAND_MAX + 1.0)) + i * avg_clusterColor,
							255-(int)(avg_clusterColor * rand() / (RAND_MAX + 1.0)) - i * avg_clusterColor,
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
		std::fill(center.begin(), center.end(), pcl::PointXYZ(0,0,0));//centerÈ«²¿ÖÃÎª0
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

double QT_PCL_Segmentation::distance(pcl::PointXYZ a, pcl::PointXYZ b,int model) 
{
	double d2 = (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z);
	
	if (model == 1) {
		return d2;
	}
	else if (model == 2) {
		double theta = 100*exp(c/pow(kmeansRadius,2)*d2)+1;
		return d2*theta;
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
					if(mode==1) Sleep(680);
				}				
			}			
		}
		return false;
	}
	else 
	{
		in.get(buffer,3);//¶ÁÈ¡¡°CN ¡±
		in >> branchNum;
		for (int i = 0; i < branchNum; i++) 
		{
			in.get(bufferChar);//¶ÁÈ¡»»ÐÐ·û
			in.get(buffer,4);//¶ÁÈ¡¡°CNN ¡±
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
				if(j<10 && i<10 && mode==1) Sleep(680);
			}
			in.get(buffer, 4);//¶ÁÈ¡\tab
		}
		return true;
	}
}

void QT_PCL_Segmentation::drawSkel()
{
	skelFlag = 1;
	if (skelParam(this->modelSkelName)) skelSize = 0.01;
	else skelSize = 0.001;

	viewer->removeAllShapes();
	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);
	int len = 0, vecIndex = 0;
	for (int i = 0; i < skelCloud->size(); i++)
	{
		len++;
		if (len == 1)
			viewer->addSphere(skelCloud->points[i], skelSize*2, 0, 0, 135, "sphere" + std::to_string(i));
		else if (len >= branchLen[vecIndex])
		{
			len = 0;
			vecIndex++;
			viewer->addSphere(skelCloud->points[i], skelSize*2, 0, 0, 135, "sphere" + std::to_string(i));
		}
		else
			viewer->addSphere(skelCloud->points[i], skelSize*2, 0, 135, 0, "sphere" + std::to_string(i));
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
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::reDrawSkel()
{
	if(skelFlag == 2)
		skelParam(this->modelSkelName+"_2",2);
	else
		skelParam(this->modelSkelName,2);
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

void QT_PCL_Segmentation::connectSkel(int i,int j,pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
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
	std::vector<int> KNNtag(this->cloud->points.size(),0);
	std::vector<int> KNNindex(ui.K_1->toPlainText().toDouble(), -1);
	std::vector<double> KNNdist(ui.K_1->toPlainText().toDouble(),10000);
	for (int i = 0; i < this->cloud->points.size(); ++i)
	{
		for (int j = 0; j < this->cloud->points.size(); ++j)
		{
			//选择KNN
			std::vector<double>::iterator maxDist = std::max_element(KNNdist.begin(), KNNdist.end());
			if (distance(this->cloud->points[i], this->cloud->points[j]) < *maxDist && KNNtag[j]==-1) {
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
		int preContent =-1, preCount = 0;
		for (std::vector<int>::iterator iter = KNNindex.begin(); iter != KNNindex.end(); ++iter) 
		{
			
			if (KNNtag[*iter]==pre)
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
			double p_K_G = this->skelCloud->points[index].x+ this->skelCloud->points[index].y+ this->skelCloud->points[index].z;
			p_X_omega += *p_X_theta * p_K_G;
			if (p_X_omega > threshold) { this->connectSkel(i, j, cloud); successCount++; }
			else { failCount++; }
			index++;
		}
	}
	return (successCount>failCount);
}

void QT_PCL_Segmentation::BayesSkel()
{
	skelFlag = 2;
	if (skelParam(this->modelSkelName+"_2")) this->skelSize = 0.01;
	else this->skelSize = 0.001;

	viewer->removeAllShapes();
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

	ui.qvtkWidget->update();
}
