#include "L1median.h"
#include <QThread>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using namespace Eigen;

L1median::L1median(ParameterSet in_paraSet, PointCloud<PointXYZ>::Ptr in_origin, 
				   PointCloud<PointXYZ>::Ptr in_sample, vector<vector<PointXYZ>>* in_skel)
	: origin(in_origin), sample(in_sample), skelPtr(in_skel)
{
	this->para = in_paraSet;
	this->sample_save = PointCloud<PointXYZ>(*this->sample);
}

L1median::~L1median()
{

}

void L1median::run()
{
	initalizeParam();
	this->skelPtr->clear();
	/*预备信息计算*/
	emit infoSignal("loading...");
	computeNeighbors();
	computeSigmas();
	if (isDensityWeighted) computeDensity();
	emit infoSignal("finish loading");
	int N = this->para.getInt("max_iterate_time");
	double err_threshold = this->para.getDouble("moving_error_threshold");

	for (int i = 0; i < N; ++i) {
		double error = iterateReturnError();
		//emit infoSignal("mse:"+QString::number(error));
		if (error < err_threshold || i==N-1) {
			if (updateRadius() >= h_boundary) {
				break;
			}
			i = 0;
		}
	}
	emit endSignal();
}

void L1median::iterate()
{
	iterateReturnError();
}

double L1median::iterateReturnError()
{
	/*位置迭代更新*/
	computeAlphasTerms();
	computeBetasTerms();
	double error = updateSamplePos();
	emit infoSignal("MSE:" + QString::number(error, 'f', 2));

	/*位置同步*/
	if (synSampleWithInfo()) {
		emit iterateSignal();
	} else {
		emit errorSignal(QString("fail to synchronize sample info"));
		return 0.0;
	}
	/*删除重合点*/
	if (updateRemovedSample()) {
		emit iterateSignal();
	}
	else {
		emit errorSignal(QString("fail to remove samples"));
		return 0.0;
	}
	/*位置更新后的预备信息计算*/
	computeNeighbors();
	computeSigmas();

	/*tracing搜寻骨骼*/
	// TO-DO：tracing
	growAllBranches();



	return error;
}


void L1median::reset()
{
	// TO-DO: reset function
}

void L1median::initalizeParam()
{
	// 初始neighborhood size
	pcl::PointXYZ min;//用于存放三个轴的最小值
	pcl::PointXYZ max;//用于存放三个轴的最大值
	pcl::getMinMax3D(*origin, min, max);
	double dbb2 = (max.x - min.x)*(max.x - min.x)
		+ (max.y - min.y)*(max.y - min.y)
		+ (max.z - min.z)*(max.z - min.z);
	double h0 = sqrt(dbb2) / pow(this->origin->points.size(), 1.0 / 3);// h0= dbb/三次根号下点云点的数量
	this->h = h0;
	this->h_boundary = sqrt(dbb2);
	emit infoSignal("initial neighborhood size: "+QString::number(h0, 'f', 2));

	this->h_increment = this->para.getDouble("h_increasing_rate")*h0;
	this->rep_factor = this->para.getDouble("repulsion_factor");

	this->avg_power = this->para.getInt("average_power");
	this->rep_power = this->para.getInt("repulsion_power");

	this->isDensityWeighted = bool(this->para.getInt("use_desity_weight"));
	
	//initialize sample Info
	this->sampleInfo.clear();
	for (PointXYZ pt : this->sample->points) {
		this->sampleInfo.push_back(L1SampleInfo(pt));
	}
}

bool L1median::isSampleFixed(int index)
{
	if (index >= sampleInfo.size()) {
		emit errorSignal("sample info out of range!");
		return true;
	}
	return (
		(sampleInfo[index].kind == pi::Branch) ||
		(sampleInfo[index].kind == pi::Bridge) ||
		(sampleInfo[index].kind == pi::Removed)
	);
}

void L1median::computeNeighbors()
{
	// origin neighbors
	KdTreeFLANN<PointXYZ> kdtree;
	kdtree.setInputCloud(origin);
	int index = 0;
	for (L1SampleInfo xsi : sampleInfo) {
		if (isSampleFixed(index)) {
			sampleInfo[index].o_indics.clear();
			sampleInfo[index].o_dists.clear();
			continue;
		}
		PointXYZ xpt = xsi.pos;
		vector<int> neighIndex;
		vector<float> neighDistance;
		int num = kdtree.radiusSearch(xpt, this->h, neighIndex, neighDistance);

		while (neighDistance.size() > 0 && neighDistance[0] == 0) {
			neighDistance.erase(neighDistance.begin());
			neighIndex.erase(neighIndex.begin());
		}
		for (int i = 0; i < neighDistance.size(); ++i) {
			if (neighDistance[i]<1e-8 || neighDistance[i]>1e8) {
				neighIndex.erase(neighIndex.begin() + i);
				neighDistance.erase(neighDistance.begin() + i);
				i--;
			}
		}

		sampleInfo[index].o_indics = neighIndex;
		sampleInfo[index].o_dists = neighDistance;
		index++;
	}

	// self neighbors
	KdTreeFLANN<PointXYZ> kdtree2;
	kdtree2.setInputCloud(this->sample);
	index = 0;
	for (int j = 0; j < this->sample->points.size(); ++j) {
		if (isSampleFixed(index)) {
			sampleInfo[index].s_indics.clear();
			sampleInfo[index].s_dists.clear();
			index++;
			continue;
		}
		vector<int> neighIndex;
		vector<float> neighDistance;
		int num = kdtree2.radiusSearch(index, this->h, neighIndex, neighDistance);

		while (neighDistance.size() > 0 && neighDistance[0] == 0) {
			neighIndex.erase(neighIndex.begin());
			neighDistance.erase(neighDistance.begin());
		}
		for (int i = 0; i < neighDistance.size(); ++i) {
			if (neighDistance[i]<1e-8 || neighDistance[i]>1e8) {
				neighIndex.erase(neighIndex.begin() + i);
				neighDistance.erase(neighDistance.begin() + i);
				i--;
			}
		}
		sampleInfo[index].s_indics = neighIndex;
		sampleInfo[index].s_dists = neighDistance;
		index++;
	}
}

void L1median::computeSigmas()
{
	double threshold = para.getDouble("candidate_sigma_threshold");
	for (int i = 0; i < sampleInfo.size(); ++i) {
		if (isSampleFixed(i) || sampleInfo[i].s_indics.size() < 1) {
			sampleInfo[i].sigma = 0.0;
			continue;
		}
		Matrix3d mm;
		for (int x_index : sampleInfo[i].s_indics) {
			if (x_index >= sampleInfo.size()) {
				emit errorSignal("out of range in computing sigma");
				continue;
			}
			Vector3d v (sampleInfo[i].pos.x - sample->points[x_index].x,
						sampleInfo[i].pos.y - sample->points[x_index].y,
						sampleInfo[i].pos.z - sample->points[x_index].z);
			mm += v * v.adjoint();
		}
		Vector3cd eigens = mm.eigenvalues();
		Vector3d eigenVec(eigens(0).real(), eigens(1).real(), eigens(2).real());
		sampleInfo[i].sigma = eigenVec.maxCoeff() / eigens.sum().real();
		if (sampleInfo[i].sigma > threshold) {
			sampleInfo[i].kind = pi::Candidate;
		}
	}
	//emit infoSignal("compute sigmas");
}

double L1median::updateRadius()
{
	h += h_increment;
	emit infoSignal("update radius to"+QString::number(h, 'f', 2));
	return h;
}

bool L1median::synSampleWithInfo()
{
	if (this->sample->points.size() != this->sampleInfo.size()) {
		return false;
	}
	vector<L1SampleInfo> old_sampleInfo = sampleInfo;
	int i = 0;
	for (L1SampleInfo si : old_sampleInfo) {
		this->sample->points[i] = si.pos;
		i++;
	}

	return true;
}

bool L1median::updateRemovedSample()
{
	if (this->sample->points.size() != this->sampleInfo.size()) {
		return false;
	}
	vector<L1SampleInfo> old_sampleInfo = sampleInfo;
	int i = 0;
	for (L1SampleInfo si : old_sampleInfo) {
		if (si.kind == pi::Removed) {
			sampleInfo.erase(sampleInfo.begin() + i);
			sample->points.erase(sample->points.begin() + i);
			continue;
		}
		i++;
	}

	return true;
}


/*double distance2(PointXYZ a, PointXYZ b) {
	return ((a.x - b.x)*(a.x - b.x) +
		(a.y - b.y)*(a.y - b.y) +
		(a.z - b.z)*(a.z - b.z));
}*/

void L1median::computeAlphasTerms()
{
	guassin_factor = -4.0 / (h*h);
	for (int i = 0; i < this->sampleInfo.size(); ++i) {
		sampleInfo[i].average_term = PointXYZ(0, 0, 0);
		sampleInfo[i].alpha.clear();
		sampleInfo[i].alpha.resize(sampleInfo[i].o_indics.size());
		for (int j = 0; j < sampleInfo[i].alpha.size(); ++j) {
			double dist = sampleInfo[i].o_dists[j];
			double dist2 = dist * dist;
			PointXYZ qi = this->origin->points[sampleInfo[i].o_indics[j]];
			// alpha = theta(dist)/dist
			sampleInfo[i].alpha[j] = exp(dist2*guassin_factor)/pow(dist, avg_power);
			if (isDensityWeighted) {
				sampleInfo[i].alpha[j] *= density[sampleInfo[i].o_indics[j]];
			}
			// average term = sum(q*alpha)/sum(alpha)
			sampleInfo[i].average_term.x += qi.x * sampleInfo[i].alpha[j];
			sampleInfo[i].average_term.y += qi.y * sampleInfo[i].alpha[j];
			sampleInfo[i].average_term.z += qi.z * sampleInfo[i].alpha[j];
		}
		double sum_alpha = std::accumulate(sampleInfo[i].alpha.begin(), sampleInfo[i].alpha.end(), 0.0);
		if (sum_alpha != 0) {
			sampleInfo[i].average_term.x /= sum_alpha;
			sampleInfo[i].average_term.y /= sum_alpha;
			sampleInfo[i].average_term.z /= sum_alpha;
		}
		else {
			sampleInfo[i].average_term = sampleInfo[i].pos;
		}
	}
	//emit infoSignal("Alpha - end");
}

void L1median::computeBetasTerms()
{
	guassin_factor = -4.0 / (h*h);
	for (int i = 0; i < this->sampleInfo.size(); ++i) {
		sampleInfo[i].repulsion_term = PointXYZ(0, 0, 0);
		sampleInfo[i].beta.clear();
		sampleInfo[i].beta.resize(sampleInfo[i].s_indics.size());
		for (int j = 0; j < sampleInfo[i].beta.size(); ++j) {
			double dist = sampleInfo[i].s_dists[j];
			double dist2 = dist * dist;
			PointXYZ xi = this->sample->points[sampleInfo[i].s_indics[j]];
			PointXYZ diff = PointXYZ(sampleInfo[i].pos.x - xi.x,
									 sampleInfo[i].pos.y - xi.y,
									 sampleInfo[i].pos.z - xi.z);
			// beta = theta(dist)/dist^2
			sampleInfo[i].beta[j] = exp(dist2*guassin_factor) / pow(dist, rep_power);

			// repulsion term = sum(diff*beta)/sum(beta)
			sampleInfo[i].repulsion_term.x += diff.x * sampleInfo[i].beta[j];
			sampleInfo[i].repulsion_term.y += diff.y * sampleInfo[i].beta[j];
			sampleInfo[i].repulsion_term.z += diff.z * sampleInfo[i].beta[j];
		}
		double sum_beta = std::accumulate(sampleInfo[i].beta.begin(), sampleInfo[i].beta.end(), 0.0);
		if (sum_beta != 0) {
			sampleInfo[i].repulsion_term.x /= sum_beta;
			sampleInfo[i].repulsion_term.y /= sum_beta;
			sampleInfo[i].repulsion_term.z /= sum_beta;
		}
		else {
			sampleInfo[i].repulsion_term = PointXYZ(0,0,0);
		}
	}
	//emit infoSignal("compute Betas Terms");
}

double L1median::updateSamplePos()
{
	int size = this->sampleInfo.size();
	if (size < 1) return 0;
	double error = 0.0;
	int count = 0;
	for (int i = 0; i < size; ++i) {
		if (isSampleFixed(i)) {
			continue;
		}
		double rep_weight = sampleInfo[i].sigma*this->rep_factor;		
		PointXYZ newPt = PointXYZ(
			sampleInfo[i].average_term.x + rep_weight * sampleInfo[i].repulsion_term.x,
			sampleInfo[i].average_term.y + rep_weight * sampleInfo[i].repulsion_term.y,
			sampleInfo[i].average_term.z + rep_weight * sampleInfo[i].repulsion_term.z
		);
		error += sqrt(
			(newPt.x - sampleInfo[i].pos.x)*(newPt.x - sampleInfo[i].pos.x) +
			(newPt.y - sampleInfo[i].pos.y)*(newPt.y - sampleInfo[i].pos.y) +
			(newPt.z - sampleInfo[i].pos.z)*(newPt.z - sampleInfo[i].pos.z)
		);
		sampleInfo[i].pos = newPt;
		count++;
	}
	if (count == 0) return 0;
	return error / count;
}


void L1median::growAllBranches()
{
	// TO-DO: grow branch 的方向问题解决
	vector<int> candidate_indics;
	for (int i = 0; i < sampleInfo.size(); ++i) {
		if (sampleInfo[i].kind == pi::Candidate) {
			if (candidate_indics.empty()) candidate_indics.push_back(i);
			else if (sampleInfo[i].sigma > sampleInfo[0].sigma) {
				candidate_indics.insert(candidate_indics.begin(), i);
			}
			else {
				candidate_indics.push_back(i);
			}
		}
	}
	bool addNewBranch = false;
	double combine_threshold = para.getDouble("too_close_dist_threshold");
	for (int index : candidate_indics) {
		vector<PointXYZ> branch = { sampleInfo[index].pos };
		vector<int> branch_id = { index };
		for (int i = 0; i < sampleInfo[index].s_dists.size(); ++i) {
			double dist = sampleInfo[index].s_dists[i];
			if (dist <= combine_threshold) {
				sampleInfo[sampleInfo[index].s_indics[i]].kind = pi::Removed;
				continue;
			}
			branch.push_back(sample->points[sampleInfo[index].s_indics[i]]);
			branch_id.push_back(sampleInfo[index].s_indics[i]);
		}
		if (branch.size() > 2) {
			for (int id : branch_id) {
				sampleInfo[id].kind = pi::Branch;
			}
			this->skelPtr->push_back(branch);
			addNewBranch = true;
		}
	}
	if(addNewBranch) emit skelChangeSignal();
}


void L1median::computeDensity()
{
	density.clear();
	int size = origin->points.size();
	density.resize(size);
	KdTreeFLANN<PointXYZ> kdtree;
	kdtree.setInputCloud(origin);
	int index = 0;
	for (int i = 0; i < size; ++i) {
		vector<int> neighIndex;
		vector<float> neighDistance;
		density[i] = kdtree.radiusSearch(origin->points[i], this->h, 
										 neighIndex, neighDistance);
		if (density[i] == 0) density[i] = 1;
		else density[i] = 1 / density[i];
	}
}
