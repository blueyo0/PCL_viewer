#pragma once
#include "PointCloudAlgorithm.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Skeleton.h"

using namespace pcl;

class TestMoving :
	public PointCloudAlgorithm
{
	Q_OBJECT
public:
	TestMoving(ParameterSet, PointCloud<PointXYZ>::Ptr, vector<vector<PointXYZ>>*);
	virtual ~TestMoving();

	virtual void iterate();
	virtual void run();
	virtual void reset();

private:
	vector<vector<PointXYZ>> *skeleton;
	PointCloud<PointXYZ>::Ptr cloud;
};

