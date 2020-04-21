#include "L1median.h"
#include "omp.h"
#include <QThread>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <ctime>
#include <queue>
#include <thread>
#include <fstream>

using namespace Eigen;

L1median::L1median(ParameterSet* in_paraSet, PointCloud<PointXYZ>::Ptr in_origin, 
				   PointCloud<PointXYZ>::Ptr in_sample, vector<vector<PointXYZ>>* in_skel,
				   vector<pi::PtKind> *in_status)
	: origin(in_origin), sample(in_sample), skelPtr(in_skel), status(in_status)
{
	this->para = *in_paraSet;
	this->paraPtr = in_paraSet;
	this->sample_save = PointCloud<PointXYZ>(*this->sample);
}

void L1median::setSigmaPtr(vector<double>* in_sigma_ptr)
{
	this->sigmaPtr = in_sigma_ptr;
}

L1median::~L1median()
{

}

void L1median::run()
{
	initalizeParam();
	this->skelPtr->clear();
	if (bool(para.getInt("use_down_sample"))) preDownSample();
	/*预备信息计算*/
	computeNeighbors();
	computeSigmas();
	if (isDensityWeighted) {
		emit infoSignal(">> computing density...");
		computeDensity();
		emit infoSignal(">> finish computing");
	}
	int N = this->para.getInt("max_iterate_time");
	double err_threshold = this->para.getDouble("moving_error_threshold");
	double err_factor = this->para.getDouble("moving_error_factor");

	int total_iterate_count = 0;
	for (int i = 0; i < N; ++i) {
		if (resetFlag) {
			for (int s = 0; s < sample->points.size(); ++s) {
				sample->points[s] = sample_save.points[s];
			}
			paraPtr->add("neighborhood_size", 0.00);
			resetFlag = false;
			emit iterateSignal();
			return;
		}
		total_iterate_count++;
		emit infoSignal("iterate time: "+QString::number(total_iterate_count));
		double error = iterateReturnError();

		/*tracing搜寻骨骼*/
		if (radius_increase_time > 1 && i >= 3) {
			growAllBranches();
		}

		// for couple.ply test
		int si_index = 0;
		int min_si_index = 0;
		double min_si_z = 1.0;
		for (L1SampleInfo si : sampleInfo) {
			if (si.pos.z < min_si_z) {
				min_si_z = si.pos.z;
				min_si_index = si_index;
			}
			si_index++;
		}

		if (bool(para.getInt("use_auto_error")) && i == 5) {
			err_threshold = error * err_factor;
			paraPtr->add("moving_error_threshold", err_threshold);
			continue;
		}
		if (error < err_threshold || i == N-1) {
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
	paraPtr->add("neighborhood_size", h);
	for (int i = 0; i < sampleInfo.size(); ++i) {
		sampleInfo[i].kind = (sampleInfo[i].kind == pi::Candidate) ? pi::Sample : sampleInfo[i].kind;
	}

	computeAlphasTerms();
	computeBetasTerms();
	double error = updateSamplePos();
	emit infoSignal("RMS:" + QString::number(error, 'f', 6)+'\n');

	/*位置同步*/
	if (synSampleWithInfo()) {
		emit iterateSignal();
	} else {
		emit errorSignal(QString("fail to synchronize sample info"));
		return 0.0;
	}
	/*位置更新后的预备信息计算*/
	computeNeighbors();
	computeSigmas();

	/*remove重合点*/
	/*if (updateRemovedSample()) {
		emit iterateSignal();
	}
	else {
		emit errorSignal(QString("fail to remove samples"));
		return 0.0;
	}*/

	return error;
}


void L1median::reset()
{
	resetFlag = true;
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
	this->h_boundary = sqrt(dbb2)/2;
	emit infoSignal("initial h: "+QString::number(h0, 'f', 2));

	this->h_increment = this->para.getDouble("h_increasing_rate")*h0;
	this->rep_factor = this->para.getDouble("repulsion_factor");

	this->avg_power = this->para.getInt("average_power");
	this->rep_power = this->para.getInt("repulsion_power");

	this->isDensityWeighted = bool(this->para.getInt("use_density_weight"));
	
	//initialize sample Info
	this->sampleInfo.clear();
	for (PointXYZ pt : this->sample->points) {
		this->sampleInfo.push_back(L1SampleInfo(pt));
	}
	// 性能相关初始化
	originKdtree.setInputCloud(origin);
	selfKdtree.setInputCloud(sample);
}

bool L1median::isSampleFixed(int index)
{
	if (index >= sampleInfo.size()) {
		emit errorSignal("sample info out of range!");
		return true;
	}
	return ((sampleInfo[index].kind == pi::Branch) 
			||(sampleInfo[index].kind == pi::Bridge) 
			||(sampleInfo[index].kind == pi::Removed));
}

void L1median::computeOriginNeighbors()
{
	clock_t start = clock();
	// origin neighbors
	// 使用omp方式进行多线程计算

	omp_set_num_threads(para.getInt("density_thread_num"));
#pragma omp parallel for 
	for (int index = 0; index < sampleInfo.size(); ++index) {
		vector<int> neighIndex(1000);
		vector<float> neighDistance(1000);
		PointXYZ xpt = sampleInfo[index].pos;
		int num = originKdtree.radiusSearch(xpt, this->h, neighIndex, neighDistance);
		sampleInfo[index].o_indics = neighIndex;
		sampleInfo[index].o_dists = neighDistance;
	}

	//omp_set_num_threads(1);

	clock_t end = clock();
	if (para.getInt("use_timecost_output")) {
		emit infoSignal("o_neigh time cost:" +
			QString::number((double)(end - start) / CLOCKS_PER_SEC, 'f', 2) + 's');
	}
}
void L1median::computeSelfNeighbors()
{
	clock_t start = clock();
	// self neighbors
	double combine_threshold = para.getDouble("too_close_dist_threshold");
	combine_threshold = combine_threshold * combine_threshold;
	for (int j = 0; j < this->sample->points.size(); ++j) {
		if (isSampleFixed(j)) {
			sampleInfo[j].s_indics.clear();
			sampleInfo[j].s_dists.clear();
			continue;
		}
		vector<int> neighIndex(1000);
		vector<float> neighDistance(1000);
		int num = selfKdtree.radiusSearch(j, this->h, neighIndex, neighDistance);

		while (neighDistance.size() > 0 && neighDistance[0] == 0) {
			neighIndex.erase(neighIndex.begin());
			neighDistance.erase(neighDistance.begin());
		}
		if (para.getInt("use_close_neigh_removement")) {
			for (int i = 0; i < neighDistance.size(); ++i) {
				if (neighDistance[i] < combine_threshold || isSampleFixed(neighIndex[i])) {
					// 会导致原因不明的发散问题，暂时弃用此Remove机制
					//sampleInfo[neighIndex[i]].kind = (sampleInfo[neighIndex[i]].kind == pi::Sample)? 
					//								  pi::Removed : sampleInfo[neighIndex[i]].kind;
					neighIndex.erase(neighIndex.begin() + i);
					neighDistance.erase(neighDistance.begin() + i);
					i--;
				}
			}
		}
		sampleInfo[j].s_indics = neighIndex;
		sampleInfo[j].s_dists = neighDistance;
	}
	clock_t end = clock();
	if (para.getInt("use_timecost_output")) {
		emit infoSignal("s_neigh time cost:" +
			QString::number((double)(end - start) / CLOCKS_PER_SEC, 'f', 2) + 's');
	}
}

void L1median::computeNeighbors()
{
	computeOriginNeighbors();
	computeSelfNeighbors();
}

void L1median::computeSigmas()
{
	clock_t start = clock();
	double max_sigma = 0.0;
	for (int i = 0; i < sampleInfo.size(); ++i) {
		int size = sampleInfo[i].s_indics.size();
		if (isSampleFixed(i) || size < 1) {
			sampleInfo[i].sigma = 0.0;
			continue;
		}
		/*if (size == 1) {
			sampleInfo[i].sigma = 0.0;
			continue;
		}*/
		Matrix3f mm = Matrix3f::Zero(3,3);
		for (int x_index : sampleInfo[i].s_indics) {
			if (x_index >= sampleInfo.size()) {
				emit errorSignal("out of range in computing sigma");
				continue;
			}
			Vector3f v (sampleInfo[i].pos.x - sample->points[x_index].x,
						sampleInfo[i].pos.y - sample->points[x_index].y,
						sampleInfo[i].pos.z - sample->points[x_index].z);
			mm += v * v.adjoint();
		}
		Vector3cf eigens = mm.eigenvalues();
		Vector3f eigenVec(eigens(0).real() > 0.0 ? eigens(0).real() : 0.0,
						  eigens(1).real() > 0.0 ? eigens(1).real() : 0.0,
						  eigens(2).real() > 0.0 ? eigens(2).real() : 0.0);

		sampleInfo[i].sigma = eigenVec.maxCoeff() / eigens.sum().real();
		if (sampleInfo[i].sigma > 1.0) {
			sampleInfo[i].sigma = 1.0;
		}
		if (sampleInfo[i].sigma > max_sigma) {
			max_sigma = sampleInfo[i].sigma;
		}
	}
	clock_t end = clock();
	if (para.getInt("use_timecost_output")) {
		emit infoSignal("sigma time cost:" +
			QString::number((double)(end - start) / CLOCKS_PER_SEC, 'f', 2) + 's');
	}
}

double L1median::updateRadius()
{
	h += h_increment;
	h_increment = this->para.getDouble("h_increasing_rate")*h;
	emit infoSignal("<font color=\"#049f11\">update radius to "+QString::number(h, 'f', 2)+"</font>");
	radius_increase_time++;
	return h;
}

bool L1median::synSampleWithInfo()
{
	if (this->sample->points.size() != this->sampleInfo.size()) {
		return false;
	}
	vector<L1SampleInfo> old_sampleInfo = sampleInfo;
	int i = 0;
	if (status->size() < 1) {
		status->resize(sampleInfo.size(), pi::Sample);
	}
	if (sigmaPtr->size() < 1) {
		sigmaPtr->resize(sampleInfo.size(), 0.0);
	}
	for (L1SampleInfo si : old_sampleInfo) {
		this->sample->points[i] = si.pos;
		(*status)[i] = si.kind;
		(*sigmaPtr)[i] = si.sigma;
		i++;
	}

	return true;
}

bool L1median::updateRemovedSample()
{
	double combine_threshold = para.getDouble("too_close_dist_threshold");
	combine_threshold = combine_threshold * combine_threshold;
	if (this->sample->points.size() != this->sampleInfo.size()) {
		return false;
	}
	for (int i = 0; i < sampleInfo.size(); ++i) {
		if (isSampleFixed(i)) continue;
		for (int j = 0; j < sampleInfo[i].s_indics.size(); ++j){
			if (isSampleFixed(sampleInfo[i].s_indics[j])) {
				sampleInfo[i].s_indics.erase(sampleInfo[i].s_indics.begin() + j);
				sampleInfo[i].s_dists.erase(sampleInfo[i].s_dists.begin() + j);
				j--;
				continue;
			}
			if (sampleInfo[i].s_dists[j] < combine_threshold) {
				sampleInfo[sampleInfo[i].s_indics[j]].kind = pi::Removed;
			}
		}
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
	clock_t start = clock();
	bool isPowerUsed = bool(para.getInt("use_multi_power_distance"));
	guassin_factor = -4.0 / (h*h);

	omp_set_num_threads(para.getInt("density_thread_num"));
#pragma omp parallel for
	for (int i = 0; i < this->sampleInfo.size(); ++i) {
		sampleInfo[i].average_term = PointXYZ(0, 0, 0);
		sampleInfo[i].alpha.clear();
		sampleInfo[i].alpha.resize(sampleInfo[i].o_indics.size());
		double sum_alpha = 0;
		for (int j = 0; j < sampleInfo[i].alpha.size(); ++j) {
			double dist2 = sampleInfo[i].o_dists[j];
			double dist = sqrt(dist2);
			if (isPowerUsed) {
				dist = sampleInfo[i].o_dists[j];
				dist2 = dist * dist;
			}			
			PointXYZ qi = this->origin->points[sampleInfo[i].o_indics[j]];
			// alpha = theta(dist)/dist
			if(dist!=0) sampleInfo[i].alpha[j] = exp(dist2*guassin_factor)/pow(dist, avg_power);
			else sampleInfo[i].alpha[j] = 0;
			if (isDensityWeighted) {
				sampleInfo[i].alpha[j] *= density[sampleInfo[i].o_indics[j]];
			}
			sum_alpha += sampleInfo[i].alpha[j];
			// average term = sum(q*alpha)/sum(alpha)
			sampleInfo[i].average_term.x += qi.x * sampleInfo[i].alpha[j];
			sampleInfo[i].average_term.y += qi.y * sampleInfo[i].alpha[j];
			sampleInfo[i].average_term.z += qi.z * sampleInfo[i].alpha[j];
		}
		if (sum_alpha != 0) {
			sampleInfo[i].average_term.x /= sum_alpha;
			sampleInfo[i].average_term.y /= sum_alpha;
			sampleInfo[i].average_term.z /= sum_alpha;
		}
		else {
			sampleInfo[i].average_term = sampleInfo[i].pos;
		}
	}
	clock_t end = clock();

	if (para.getInt("use_timecost_output")) {
		emit infoSignal("alpha time cost:" +
			QString::number((double)(end - start) / CLOCKS_PER_SEC, 'f', 2) + 's');
	}
}

void L1median::computeBetasTerms()
{
	clock_t start = clock();
	bool isPowerUsed = bool(para.getInt("use_multi_power_distance"));
	guassin_factor = -4.0 / (h*h);

	omp_set_num_threads(para.getInt("density_thread_num"));
#pragma omp parallel for
	for (int i = 0; i < this->sampleInfo.size(); ++i) {
		sampleInfo[i].repulsion_term = PointXYZ(0, 0, 0);
		sampleInfo[i].beta.clear();
		sampleInfo[i].beta.resize(sampleInfo[i].s_indics.size());
		for (int j = 0; j < sampleInfo[i].beta.size(); ++j) {
			double dist2 = sampleInfo[i].s_dists[j];
			double dist = sqrt(dist2);
			if (isPowerUsed) {
				dist = sampleInfo[i].s_dists[j];
				dist2 = dist * dist;
			}

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
	clock_t end = clock();

	if (para.getInt("use_timecost_output")) {
		emit infoSignal("beta time cost:" +
			QString::number((double)(end - start) / CLOCKS_PER_SEC, 'f', 2) + 's');
	}

}

double L1median::updateSamplePos()
{
	int size = this->sampleInfo.size();
	double mu_max = rep_factor;
	double mu_min = rep_factor * rep_factor;
	double min_sigma = sampleInfo[0].sigma;
	double max_sigma = sampleInfo[0].sigma;
	for (int i = 0; i < size; ++i) {
		if (isSampleFixed(i)) {
			continue;
		}
		if (sampleInfo[i].sigma < min_sigma) {
			min_sigma = sampleInfo[i].sigma;
		}
		if (sampleInfo[i].sigma > max_sigma) {
			max_sigma = sampleInfo[i].sigma;
		}
	}
	double mu_length = abs(mu_max - mu_min);
	double sigma_length = abs(max_sigma - min_sigma);

	if (size < 1) return 0;
	double error = 0.0;
	int count = 0;
	for (int i = 0; i < size; ++i) {
		if (isSampleFixed(i)) {
			continue;
		}
		//double mu = (mu_length / sigma_length) * (sampleInfo[i].sigma - min_sigma) + mu_min;
		double mu = mu_max;
		double rep_weight = sampleInfo[i].sigma*mu;
		PointXYZ newPt = PointXYZ(
			sampleInfo[i].average_term.x + rep_weight * sampleInfo[i].repulsion_term.x,
			sampleInfo[i].average_term.y + rep_weight * sampleInfo[i].repulsion_term.y,
			sampleInfo[i].average_term.z + rep_weight * sampleInfo[i].repulsion_term.z
		);
		if (sampleInfo[i].sigma != 0) {
			error += sqrt(
				(newPt.x - sampleInfo[i].pos.x)*(newPt.x - sampleInfo[i].pos.x) +
				(newPt.y - sampleInfo[i].pos.y)*(newPt.y - sampleInfo[i].pos.y) +
				(newPt.z - sampleInfo[i].pos.z)*(newPt.z - sampleInfo[i].pos.z)
			);
			count++;
		}
		sampleInfo[i].pos = newPt;
	}
	if (count == 0) return 0;
	return error / count;
}


void L1median::growAllBranches()
{
	// TO-DO: grow branch 的方向问题解决
	int a = 0;
	double threshold = para.getDouble("candidate_sigma_threshold");
	vector<int> candidate_indics;
	clock_t start = clock();
	
	for (int i = 0; i < sampleInfo.size(); ++i) {
		if (!isSampleFixed(i)) {
			sampleInfo[i].kind = (sampleInfo[i].sigma > threshold) ? pi::Candidate : sampleInfo[i].kind;
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
	}

	bool addNewBranch = false;
	double combine_threshold = para.getDouble("too_close_dist_threshold")*10;
	combine_threshold = combine_threshold * combine_threshold;
	for (int index : candidate_indics) {
		if (addNewBranch) continue;
		if (isSampleFixed(index)) continue;
		vector<PointXYZ> branch = { sampleInfo[index].pos };
		vector<int> branch_id = { index };
		for (int i = 0; i < sampleInfo[index].s_dists.size(); ++i) {
			if (isSampleFixed(sampleInfo[index].s_indics[i])) continue;
			if (sampleInfo[index].s_dists[i] <= combine_threshold) {
				sampleInfo[sampleInfo[index].s_indics[i]].kind = pi::Removed;
				continue;
			}
			// TO-DO: 添加方向
			branch.push_back(sample->points[sampleInfo[index].s_indics[i]]);
			branch_id.push_back(sampleInfo[index].s_indics[i]);
		}
		if (branch.size() >= para.getInt("branch_tracing_num")) {
			for (int id : branch_id) {
				sampleInfo[id].kind = pi::Branch;
				for (int n_i = 0; n_i < sampleInfo[id].s_dists.size(); ++n_i) {
					if (sampleInfo[id].s_dists[n_i] <= combine_threshold) {
						sampleInfo[sampleInfo[id].s_indics[n_i]].kind = pi::Removed;
					}
					else {
						break;
					}
				}

			}
			this->skelPtr->push_back(branch);
			addNewBranch = true;
			// TO-DO: 删除多余点
		}
	}
	if(addNewBranch) emit skelChangeSignal();
	clock_t end = clock();

	if (para.getInt("use_timecost_output")) {
		emit infoSignal("branch grow time:" +
			QString::number((double)(end - start) / CLOCKS_PER_SEC, 'f', 2) + 's');
	}
}

void L1median::setFileName(string name)
{
	filename = name.substr(0, name.length() - 3) + "dens";
}


void L1median::computeDensity()
{
	clock_t start = clock();
	density.clear();
	int size = origin->points.size();
	density.resize(size);

	// 先尝试读取file
	ifstream infile(filename, ios::in);
	if (infile.is_open()) {
		int i = 0;
		while (!infile.eof()) {
			if (i >= density.size()) {
				break;
			}
			infile >> density[i];
			i++;
		}
		infile.close();
		if (i != size) {
			emit errorSignal("point num error in density file");
		}
		else {
			return;
		}
	}

	// 读取失败

	omp_set_num_threads(para.getInt("density_thread_num"));
#pragma omp parallel for
	for (int i = 0; i < size; ++i) {
		vector<int> neighIndex(1000);
		vector<float> neighDistance(1000);
		density[i] = originKdtree.radiusSearch(i, this->h, neighIndex, neighDistance);
		density[i] = 1 / density[i];
	}

	clock_t end = clock();
	ofstream outfile(filename, ios::out);
	for (double dens : density) {
		outfile << dens << endl;
	}
	outfile.close();
	omp_set_num_threads(1);
	if(para.getInt("use_timecost_output")){
		emit infoSignal("density time cost:" +
			QString::number((double)(end - start) / CLOCKS_PER_SEC, 'f', 2) + 's');
	}
}

void L1median::preDownSample()
{
	VoxelGrid<PointXYZ> sor;
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<pcl::PointXYZ>);
	sor.setInputCloud(origin);
	float leaf_size = para.getFloat("down_sample_leaf");
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);
	sor.filter(*cloud_filtered);
	this->origin = cloud_filtered;
}