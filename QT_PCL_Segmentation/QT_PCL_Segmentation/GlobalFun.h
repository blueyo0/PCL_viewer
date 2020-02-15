#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/point_types.h>
#include <vector>
#include "SamplePoint.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

namespace GlobalFun {
	double computeDirectionalityDegree(vector<PointXYZ> diff);
	int synInfoWithCloud(vector<SamplePoint> &info, PointCloud<PointXYZ>::Ptr cloud);
	double weight(float r, double h);
	PointXYZ nextPos(SamplePoint xi, pi::PcPtr xc, pi::PcPtr qc, double mu = 0.35);

}


