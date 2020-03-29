#pragma once
#include "PointCloudAlgorithm.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "PointInfo.h"

using namespace pcl;
using namespace std;

class L1SampleInfo 
{
public:
	L1SampleInfo(PointXYZ pt) {
		this->pos = pt;
		this->average_term = pt;
		this->repulsion_term = PointXYZ(0, 0, 0);
	}

public:
	vector<int> o_indics; // �ھӶ�Ӧ��original cloud (Q)���±�
	vector<float> o_dists; // ��Ӧ�ھ�qi�ľ���

	vector<int> s_indics; // �ھӶ�Ӧ��self cloud (X)���±�
	vector<float> s_dists; // ��Ӧ�ھ�xi�ľ���
	
	double sigma = 0.0; // �����
	pi::PtKind kind = pi::Sample;
	
	PointXYZ pos;
	vector<double> alpha; // original neighbor ��Ӧ��alphaֵ
	PointXYZ average_term;
	vector<double> beta; // self neighbor ��Ӧ��betaֵ
	PointXYZ repulsion_term;
};

class L1median :
	public PointCloudAlgorithm
{
private:
	PointCloud<PointXYZ>::Ptr sample;
	PointCloud<PointXYZ> sample_save;
	PointCloud<PointXYZ>::Ptr origin;
	vector<L1SampleInfo> sampleInfo;
	double h = -1.0, h_increment=0;
	int avg_power=2, rep_power=2;
	bool isDensityWeighted = false;

public:
	void initalizeParam();
	void computeNeighbors();
	void computeSigmas();
	void computeAlphasTerms();
	void computeBetasTerms();

	void updateSamplePos();
	void growAllBranches();
	double updateRadius();
	bool synSampleWithInfo();

public:
	L1median(ParameterSet, PointCloud<PointXYZ>::Ptr, 
						   PointCloud<PointXYZ>::Ptr); // para + origin + sample
	virtual ~L1median();

	virtual void run();
	virtual void iterate();
	double iterateReturnError();
	virtual void reset();
};

