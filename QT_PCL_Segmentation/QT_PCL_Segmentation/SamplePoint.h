#pragma once
#include <vector>
#include <pcl/point_types.h>
#include "PointInfo.h"

using namespace std;
using namespace pcl;

class SamplePoint
{
protected:
	vector<int> o_indics; // 邻居对应的original cloud (Q)内下标
	vector<float> o_dists; // 对应邻居qi的距离

	vector<int> s_indics; // 邻居对应的self cloud (X)内下标
	vector<float> s_dists; // 对应邻居xi的距离
	double sigma; // 有向度
public:
	PointXYZ pos;
	int o_size;
	int s_size;
	pi::PtKind kind;
	vector<double> alpha; // original neighbor 对应的alpha值
	vector<double> beta; // self neighbor 对应的beta值

	SamplePoint();
	~SamplePoint();

	double computeSigma(pi::PcPtr cloud, vector<SamplePoint> info);

	void setSigma(double);
	void setPos(PointXYZ);
	void setPos(double x, double y, double z);
	void setOriginalIndics(vector<int>);
	void setOriginalDistances(vector<float>);
	void setSelfIndics(vector<int>);
	void setSelfDistances(vector<float>);

	double getSigma();
	vector<int> getOriginalIndics();
	vector<float> getOriginalDistances();
	vector<int> getSelfIndics();
	vector<float> getSelfDistances();
};

