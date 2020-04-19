#pragma once
#include "PointCloudAlgorithm.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Skeleton.h"
#include "PointInfo.h"
#include <vector>

using namespace pcl;

class TestMoving :
	public PointCloudAlgorithm
{
	Q_OBJECT
public:
	TestMoving(ParameterSet, PointCloud<PointXYZ>::Ptr, vector<vector<PointXYZ>>*, vector<pi::PtKind>*);
	virtual ~TestMoving();

	virtual void iterate();
	virtual void run();
	virtual void reset();

private:
	vector<vector<PointXYZ>> *skeleton;
	PointCloud<PointXYZ>::Ptr cloud;
	vector<pi::PtKind> *status;
};

