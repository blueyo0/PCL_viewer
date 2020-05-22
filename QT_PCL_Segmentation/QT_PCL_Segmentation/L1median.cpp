#include "L1median.h"
#include "omp.h"
#include <QThread>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>


#include <ctime>
#include <queue>
#include <thread>
#include <fstream>



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
	int max_nn = 1000;
	omp_set_num_threads(para.getInt("density_thread_num"));
#pragma omp parallel for 
	for (int index = 0; index < sampleInfo.size(); ++index) {
		vector<int> neighIndex(max_nn);
		vector<float> neighDistance(max_nn);
		PointXYZ xpt = sampleInfo[index].pos;
		if (isfinite(xpt.x) && isfinite(xpt.y) && isfinite(xpt.z)) {
			int num = originKdtree.radiusSearch(xpt, this->h, neighIndex, neighDistance);
			//int num = originKdtree.nearestKSearch(xpt, 1000, neighIndex, neighDistance);
		}
		while (neighDistance.size() > 0 && neighDistance[0] < 0.00000001) {
			neighIndex.erase(neighIndex.begin());
			neighDistance.erase(neighDistance.begin());
		}
		sampleInfo[index].o_indics = neighIndex;
		sampleInfo[index].o_dists = neighDistance;
	}

	/*for (int index = 0; index < sampleInfo.size(); ++index) {
		for (int j = 0; j < sampleInfo[index].o_dists.size(); ++j) {
			sampleInfo[index].o_dists[j]
		}

		sampleInfo[index].o_indics
		
	}*/
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
		PointXYZ xpt = sampleInfo[j].pos;
		if (isfinite(xpt.x) && isfinite(xpt.y) && isfinite(xpt.z)) {
			int num = selfKdtree.radiusSearch(j, this->h, neighIndex, neighDistance);
			//int num = selfKdtree.nearestKSearch(j, 1000, neighIndex, neighDistance);
		}
		for (int e_id = 0; e_id < neighIndex.size(); ++e_id) {
			if (neighDistance[e_id] < 0.00000001 || neighIndex[e_id] == j) {
				neighIndex.erase(neighIndex.begin() + e_id);
				neighDistance.erase(neighDistance.begin() + e_id);

			}
		}
		/*while (neighDistance.size() > 0 && neighDistance[0] < 0.00000001) {
			neighIndex.erase(neighIndex.begin());
			neighDistance.erase(neighDistance.begin());
		}*/
		



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
		if (!isfinite(newPt.x) || !isfinite(newPt.y) || !isfinite(newPt.z)) {
			newPt = sampleInfo[i].pos;
		}

		if (sampleInfo[i].sigma != 0) {
			double rms = sqrt(
				(newPt.x - sampleInfo[i].pos.x)*(newPt.x - sampleInfo[i].pos.x) +
				(newPt.y - sampleInfo[i].pos.y)*(newPt.y - sampleInfo[i].pos.y) +
				(newPt.z - sampleInfo[i].pos.z)*(newPt.z - sampleInfo[i].pos.z)
			);
			if (isfinite(rms)) {
				error += rms;
				count++;
			}				
		}
		sampleInfo[i].pos = newPt;
	}
	if (count == 0) return 0;
	return error / count;
}


vector<PointXYZ> L1median::searchBranchByDirection(int start_index, Vector3d direction, vector<int>& branch_index)
{
	vector<pair<int, PointXYZ>> branch;
	double dist2_threshold = para.getDouble("too_close_dist_threshold");
	int angle_threshold = para.getInt("branch_max_angle");
	dist2_threshold = dist2_threshold * dist2_threshold;
	int next_index = start_index;
	int curr_index = -1;
	while (curr_index!=next_index) {
		// TO-DO: 这个循环还有部分问题需要验证
		curr_index = next_index;
		L1SampleInfo xi = sampleInfo[curr_index];
		// 验证是否形成环
		bool isCircle = false;
		for (pair<int, PointXYZ> xp : branch) {
			if (xp.first == curr_index) {
				isCircle = true;
				break;
			}
		}
		if (isCircle) {
			break;
		}
		branch.push_back(make_pair(curr_index, xi.pos));
		for (int i = 0; i < xi.s_indics.size(); ++i) {
			double dist2 = xi.s_dists[i];
			if (dist2 < dist2_threshold) sampleInfo[xi.s_indics[i]].kind = pi::Removed;
			if (xi.s_indics[i] == curr_index) continue;
			if (sampleInfo[xi.s_indics[i]].kind != pi::Candidate) continue;
			L1SampleInfo xj = sampleInfo[xi.s_indics[i]];
			Vector3d next_direction(
				xj.pos.x - xi.pos.x,
				xj.pos.y - xi.pos.y,
				xj.pos.z - xi.pos.z
			);
			if (next_direction.dot(direction) < 0) continue;
			double angle = acos(next_direction.dot(direction) / (
								sqrt(next_direction.dot(next_direction)) *
								sqrt(direction.dot(direction)))) * 180/3.1415926;
			if (angle < angle_threshold) {
				// 距离不过近的,方向统一的candidate neighbor
				direction = next_direction;
				next_index = xi.s_indics[i];
				break;
			}
		}
	}
	vector<PointXYZ> pt_branch;
	for (pair<int, PointXYZ> node : branch) {
		pt_branch.push_back(node.second);
		branch_index.push_back(node.first);
	}
	return pt_branch;
}



bool compare(pair<int, double> a, pair<int, double> b)
{
	return a.second > b.second; //降序排列
}

void L1median::growAllBranches()
{
	int a = 0;
	double threshold = para.getDouble("candidate_sigma_threshold");
	int tracing_num = para.getInt("branch_tracing_num");
	vector<pair<int, double>> candidate_vec;
	clock_t start = clock();
	
	//降序获得sigma数组
	for (int i = 0; i < sampleInfo.size(); ++i) {
		if (!isSampleFixed(i) && sampleInfo[i].sigma > threshold) {
			sampleInfo[i].kind = pi::Candidate;
			candidate_vec.push_back(make_pair(i, sampleInfo[i].sigma));
		}
	}
	sort(candidate_vec.begin(), candidate_vec.end(), compare);

	//从sigma最高的点开始search branch
	bool addNewBranch = true;
	for (pair<int, double> candidate : candidate_vec) {
		L1SampleInfo xi = sampleInfo[candidate.first];
		if (xi.kind != pi::Candidate) continue;
		int xj_index = -1;
		// 找到距xi最近的candidate(xj), 沿着xi->xj和xi<-xj两个方向搜索
		for (int s_index : xi.s_indics) {
			if (sampleInfo[s_index].kind == pi::Candidate) {
				xj_index = s_index;
				break;
			}
		}
		if (xj_index != -1) {
			Vector3d direction(
				xi.pos.x - sampleInfo[xj_index].pos.x,
				xi.pos.y - sampleInfo[xj_index].pos.y,
				xi.pos.z - sampleInfo[xj_index].pos.z
			);
			vector<int> bi_1, bi_2, bi_total;
			vector<PointXYZ> posi_branch = searchBranchByDirection(candidate.first, direction, bi_1);
			vector<PointXYZ> nega_branch = searchBranchByDirection(candidate.first, -1*direction, bi_2);
			bi_total.resize(bi_1.size()+ bi_2.size());
			sort(bi_1.begin(), bi_1.end(), less<int>());
			sort(bi_2.begin(), bi_2.end(), less<int>());
			merge(bi_1.begin(), bi_1.end(), bi_2.begin(), bi_2.end(), bi_total.begin());
			vector<PointXYZ> total_branch;
			for (vector<PointXYZ>::reverse_iterator riter = nega_branch.rbegin(); 
				 riter != nega_branch.rend(); ++riter) {
				total_branch.push_back(*riter);
			}
			for (int posi_index = 1; posi_index < posi_branch.size(); ++posi_index) {
				total_branch.push_back(posi_branch[posi_index]);
			}
			// 获得两个方向拼接的骨骼后
			if (total_branch.size() >= tracing_num) {
				//skelPtr->push_back(total_branch); 
				// TO-DO: 获取branch下标并修改类型为branch
				for (int bi_index : bi_total) {
					sampleInfo[bi_index].kind = pi::Branch;
				}
				addNewBranch = true;
			}
		}
	}
	if (addNewBranch) emit skelChangeSignal();


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