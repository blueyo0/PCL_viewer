#pragma once
#include "PointCloudAlgorithm.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

class TestMoving :
	public PointCloudAlgorithm
{
	Q_OBJECT
public:
	TestMoving(ParameterSet, PointCloud<PointXYZ>::Ptr);
	virtual ~TestMoving();
	virtual void setParameterSet(ParameterSet);
	virtual ParameterSet getParameterSet();
	virtual void iterate();
	virtual void run();
	virtual void reset();

private:
	ParameterSet para;
	PointCloud<PointXYZ>::Ptr cloud;
};

