#pragma once
#include "L1median.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "PointInfo.h"

class Bayes :
	public L1median
{
public:
	Bayes(
		ParameterSet*,
		PointCloud<PointXYZ>::Ptr,
		PointCloud<PointXYZ>::Ptr,
		vector<vector<PointXYZ>>*,
		vector<pi::PtKind>*
	); // para + origin + sample
	~Bayes();
};

