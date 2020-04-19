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
	vector<int> o_indics; // 邻居对应的original cloud (Q)内下标
	vector<float> o_dists; // 对应邻居qi的距离

	vector<int> s_indics; // 邻居对应的self cloud (X)内下标
	vector<float> s_dists; // 对应邻居xi的距离
	
	double sigma = 0.0; // 有向度
	pi::PtKind kind = pi::Sample;
	
	PointXYZ pos;
	vector<double> alpha; // original neighbor 对应的alpha值
	PointXYZ average_term;
	vector<double> beta; // self neighbor 对应的beta值
	PointXYZ repulsion_term;
};

class L1median :
	public PointCloudAlgorithm
{
private:
	ParameterSet* paraPtr;
	PointCloud<PointXYZ>::Ptr sample;
	PointCloud<PointXYZ> sample_save;
	vector<L1SampleInfo> sampleInfo;
	vector<pi::PtKind> *status;
	vector<double> *sigmaPtr;
	PointCloud<PointXYZ>::Ptr origin;
	vector<vector<PointXYZ>> *skelPtr;
	vector<double> density;
	double h = -1.0, h_increment = 0, guassin_factor=0;
	double h_boundary = 1;
	double rep_factor = 0;
	double resetFlag = false;
	int avg_power=2, rep_power=2;
	bool isDensityWeighted = false;

public:
	void initalizeParam();
	bool isSampleFixed(int);

	void computeNeighbors();
	void computeOriginNeighbors();
	void computeSelfNeighbors();

	void computeSigmas();
	void computeAlphasTerms();
	void computeBetasTerms();

	//density weight
	void computeDensity();
	void preDownSample();

	double updateSamplePos(); 
	bool updateRemovedSample();
	void growAllBranches();
	double updateRadius();
	bool synSampleWithInfo();

public:
	L1median(
		ParameterSet*, 
		PointCloud<PointXYZ>::Ptr,  
		PointCloud<PointXYZ>::Ptr,
		vector<vector<PointXYZ>>*,
		vector<pi::PtKind>*					
		); // para + origin + sample
	void setSigmaPtr(vector<double>*);

	virtual ~L1median();

	virtual void run();
	virtual void iterate();
	double iterateReturnError();
	virtual void reset();
};

