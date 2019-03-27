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

#include<time.h>
#include<stdlib.h>

typedef struct RGB {
	int r;
	int g;
	int b;
}RGB;

QT_PCL_Segmentation::QT_PCL_Segmentation(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//初始化
	initialVtkWidget();
	viewer->setBackgroundColor(255, 255, 255);	
	colorFlag = false;
	colorCloudIndex = 0;
	//连接信号和槽
	connect(ui.showButton, SIGNAL(clicked()), this, SLOT(showPCL()));
	connect(ui.actionopen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.segButton, SIGNAL(clicked()), this, SLOT(kmeans()));
	//connect(ui.segButton, SIGNAL(clicked()), this, SLOT(showDemo()));
	connect(ui.drawButton, SIGNAL(clicked()), this, SLOT(drawLine()));
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

	if (!fileName.isEmpty())
	{
		if (fileName.indexOf(".pcd") != -1)
		{
			std::string file_name = fileName.toStdString();
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
				pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
			}
			else if (data_type == 2)
			{
				pcl::PCDReader reader;
				reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud);
			}
		}
		else//PLY read 
		{
			pcl::PLYReader yrd;
			yrd.read<pcl::PointXYZ>(fileName.toStdString(), *cloud);
		}		
		//correctCenter();
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
		
		

		viewer->removePointCloud("cloud");
		viewer->updatePointCloud(cloud, "cloud");
		viewer->addPointCloud(cloud, "cloud");

		pcl::io::loadPCDFile(std::string("E:/gitRepos/PCL_viewer/QT_PCL_Segmentation/QT_PCL_Segmentation/dinasourSkel.pcd"), *skelCloud);
		viewer->removePointCloud("skelCloud");
		viewer->updatePointCloud(skelCloud, "skelCloud");
		viewer->addPointCloud(skelCloud, "skelCloud");

		viewer->resetCamera();
		ui.qvtkWidget->update();

		this->color(cloud,250,140,20);
		this->color(skelCloud, 5, 115, 235);
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

	ui.label->setText(text);
}

void QT_PCL_Segmentation::segmentation()
{
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::MinCutSegmentation<pcl::PointXYZ> seg;
	seg.setInputCloud(cloud);
	seg.setIndices(indices);

	//中心点设置
	pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ point = median(cloud);
	//pcl::PointXYZ point;
	//point.x = 68.97;
	//point.y = -18.55;
	//point.z = 0.57;
	
	foreground_points->points.push_back(point);
	seg.setForegroundPoints(foreground_points);
	//分块参数
	seg.setSigma(0.25);
	seg.setRadius(3.0433856);
	seg.setNumberOfNeighbours(14);
	seg.setSourceWeight(0.8);

	std::vector <pcl::PointIndices> clusters;
	seg.extract(clusters);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud();

	viewer->updatePointCloud(colored_cloud, "colored_cloud");
	viewer->addPointCloud(colored_cloud, "colored_cloud");
	ui.qvtkWidget->update();
}

void QT_PCL_Segmentation::colorAxis()
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

void QT_PCL_Segmentation::correctCenter()
{
	pcl::PointXYZ center(median(cloud));
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x -= center.x;
		cloud->points[i].y -= center.y;
		cloud->points[i].z -= center.z;
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
	int num = 100;
	ui.InfoText->append("\nclustering start");
	int avg_clusterSize = cloud->points.size()/num;
	int avg_clusterColor = 255 / num;
	std::vector<pcl::PointXYZ> center;
	std::vector<RGB> clusterColor(num, {0,0,0});
	std::vector<int> clusterSize(num,0);
	std::vector<int> tag(cloud->points.size(),-1);
	//初始化num个簇心
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
	for (size_t iterTime = 0; iterTime < 10; iterTime++) 
	{
		//对每个点更新所属的类别： tag中存储center编号
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			int minIndex = (int)(num * rand() / (RAND_MAX + 1.0));
			int minDist = distance(center[minIndex], cloud->points[i]);
			//确认k个簇心中距离该点最近的
			int iterIndex = 0;
			for (std::vector<pcl::PointXYZ>::iterator iter = center.begin(); iter != center.end(); iter++)
			{
				if (distance(*iter, cloud->points[i]) < minDist)
				{
					minIndex = iterIndex;
					minDist = distance(*iter, cloud->points[i]);
				}
				iterIndex++;
			}
			tag[i] = minIndex;
			clusterSize[minIndex]++;
		}

		//更新center的点
		std::fill(center.begin(), center.end(), pcl::PointXYZ(0,0,0));//center全部置为0
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
	//设置颜色点云并输出
	ui.InfoText->append("\n colored cloud output start");
	int sphereIndex = 0;
	for (std::vector<pcl::PointXYZ>::iterator iter = center.begin() + 1; iter != center.end(); iter++)
	{
		viewer->addSphere(*iter,0.0000006*cloud->width,"sphere"+sphereIndex);
		sphereIndex++;
	}
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	//pcl::visualization::PCLVisualizer::Ptr rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr colored_cloud);
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
	viewer->removePointCloud("colored_cloud");
	viewer->updatePointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
	ui.qvtkWidget->update();
}

double QT_PCL_Segmentation::distance(pcl::PointXYZ a, pcl::PointXYZ b) 
{
	return (a.x-b.x)*(a.x - b.x)+ (a.y - b.y)*(a.y - b.y)+ (a.z - b.z)*(a.z - b.z);
}