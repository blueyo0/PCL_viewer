#pragma once
#include "L1median.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "PointInfo.h"

class BayesInfo
{
public:
	BayesInfo(int in_tag, PointXYZ in_pos) {
		this->tag = in_tag;
		this->pos = in_pos;
	}
	BayesInfo(PointXYZ in_pos) {
		this->tag = -1;
		this->pos = in_pos;
	}
public:
	int tag = -1;//分类信息
	PointXYZ pos;
};


class Bayes :
	public L1median
{
	Q_OBJECT
signals:
	void segSignal();

protected:
	vector<BayesInfo> originInfo;

	vector<pi::RGB> clusterColor;
	vector<int> segment_size;

	PointCloud<PointXYZ>::Ptr centers;
	PointCloud<PointXYZRGB>::Ptr segCloud;
public:
	Bayes(
		ParameterSet*,
		PointCloud<PointXYZ>::Ptr,
		PointCloud<PointXYZ>::Ptr,
		vector<vector<PointXYZ>>*,
		vector<pi::PtKind>*
	); // para + origin + sample
	virtual ~Bayes();
	double updateSamplePos();

	void setCenterCloud(PointCloud<PointXYZ>::Ptr);
	void setSegCloud(PointCloud<PointXYZRGB>::Ptr);

	void computeOverSeg();
	void computeSegMerge();

	double getSkelProbOfSegment(int s); 
	double getSkelProbOfTWOSegment(int s1, int s2);
	void mergeSegments(int s1, int s2);

	virtual void run();
	virtual double iterateReturnError();
};

