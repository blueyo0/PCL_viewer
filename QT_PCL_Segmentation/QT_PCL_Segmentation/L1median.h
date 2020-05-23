#pragma once
#include "PointCloudAlgorithm.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "PointInfo.h"

using namespace pcl;
using namespace std;
using namespace Eigen;

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
protected:
	ParameterSet* paraPtr;
	PointCloud<PointXYZ>::Ptr sample;
	PointCloud<PointXYZ> sample_save;
	vector<L1SampleInfo> sampleInfo;
	vector<pi::PtKind> *status;
	vector<double> *sigmaPtr;
	string filename;
	PointCloud<PointXYZ>::Ptr origin;
	vector<vector<PointXYZ>> *skelPtr;
	vector<double> density;
	KdTreeFLANN<PointXYZ> originKdtree;
	KdTreeFLANN<PointXYZ> selfKdtree;
	double h = -1.0, h_increment = 0, guassin_factor=0;
	double h_boundary = 1;
	double rep_factor = 0;
	double resetFlag = false;
	int avg_power=2, rep_power=2;
	bool isDensityWeighted = false;

	//����
	int radius_increase_time = 1;

public:
	void initalizeParam();
	bool isSampleFixed(int);

	inline void computeNeighbors();
	inline void computeOriginNeighbors();
	inline void computeSelfNeighbors();

	void computeSigmas();
	void computeAlphasTerms();
	void computeBetasTerms();

	//density weight
	void computeDensity();
	void preDownSample();

	double updateSamplePos();
	bool updateRemovedSample();

	void growAllBranches();
	vector<PointXYZ> searchBranchByDirection(int start_index, Vector3d direction, vector<int>& branch_index);

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
	void setFileName(string);

	virtual ~L1median();

	virtual void run();
	virtual void iterate();
	virtual double iterateReturnError();
	virtual void reset();
};

