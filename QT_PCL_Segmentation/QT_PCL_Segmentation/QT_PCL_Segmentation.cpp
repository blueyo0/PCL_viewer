#include "QT_PCL_Segmentation.h"
#include <QMessageBox>
#include <QFileDialog>

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL)
VTK_MODULE_INIT(vtkInteractionStyle)

#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>


QT_PCL_Segmentation::QT_PCL_Segmentation(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//初始化
	initialVtkWidget();
	viewer->setBackgroundColor(255, 255, 255);
	colorFlag = false;
	//连接信号和槽
	connect(ui.showButton, SIGNAL(clicked()), this, SLOT(showPCL()));
	connect(ui.actionopen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.segButton, SIGNAL(clicked()), this, SLOT(colorAxis()));
}

void QT_PCL_Segmentation::initialVtkWidget()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
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
		else//PCD read 
		{
			pcl::PLYReader yrd;
			yrd.read<pcl::PointXYZ>(fileName.toStdString(), *cloud);
		}		
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
		viewer->removePointCloud("cloud");
		viewer->updatePointCloud(cloud, "cloud");
		viewer->addPointCloud(cloud, "cloud");
		viewer->resetCamera();
		ui.qvtkWidget->update();
		this->color(162,162,162);
	}
}

//showDemo
void QT_PCL_Segmentation::showDemo()
{
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

void QT_PCL_Segmentation::color(int r,int g,int b)
{
	
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	//pcl::visualization::PCLVisualizer::Ptr rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr colored_cloud);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZRGB  pt;
		pt.x = this->cloud->points[i].x;
		pt.y = this->cloud->points[i].y;
		pt.z = this->cloud->points[i].z;
		pt.r = r;
		pt.g = g;
		pt.b = b;
		colored_cloud->push_back(pt);
	}

	//pcl::visualization::PointCloudColorHandlerRGB<pcl::PointXYZRGB> rgb(point_cloud_ptr);
	viewer->removePointCloud("colored_cloud");
	viewer->updatePointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
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
