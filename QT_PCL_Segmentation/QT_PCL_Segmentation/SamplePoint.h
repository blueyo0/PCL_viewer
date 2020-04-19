#pragma once
#include <vector>
#include <pcl/point_types.h>
#include "PointInfo.h"

using namespace std;
using namespace pcl;

class SamplePoint
{
protected:
	vector<int> o_indics; // �ھӶ�Ӧ��original cloud (Q)���±�
	vector<float> o_dists; // ��Ӧ�ھ�qi�ľ���

	vector<int> s_indics; // �ھӶ�Ӧ��self cloud (X)���±�
	vector<float> s_dists; // ��Ӧ�ھ�xi�ľ���
	double sigma; // �����
public:
	PointXYZ pos;
	int o_size;
	int s_size;
	pi::PtKind kind;
	vector<double> alpha; // original neighbor ��Ӧ��alphaֵ
	vector<double> beta; // self neighbor ��Ӧ��betaֵ

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

