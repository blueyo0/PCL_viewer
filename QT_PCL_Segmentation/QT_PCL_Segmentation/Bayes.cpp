#include "Bayes.h"
#include <algorithm>
#include "GlobalFun.h"


Bayes::Bayes(ParameterSet* in_paraSet, PointCloud<PointXYZ>::Ptr in_origin,
	PointCloud<PointXYZ>::Ptr in_sample, vector<vector<PointXYZ>>* in_skel,
	vector<pi::PtKind> *in_status)
	:L1median::L1median(in_paraSet, in_origin, in_sample, in_skel, in_status)
{
	
}

Bayes::~Bayes()
{

}

void Bayes::setCenterCloud(PointCloud<PointXYZ>::Ptr inCloud)
{
	this->centers = inCloud;
}


void Bayes::setSegCloud(PointCloud<PointXYZRGB>::Ptr inCloud)
{
	this->segCloud = inCloud;
}


void Bayes::computeOverSeg()
{
	emit infoSignal("--start over-seg");
	int size = this->para.getInt("over_seg_size");
	//this->segCloud.reset(new PointCloud<PointXYZRGB>);
	this->clusterColor = vector<pi::RGB>(size, { 0,0,0 });
	int avg_clusterColor = 255 / size;
	for (int i = 0; i < size; ++i) {
		clusterColor[i] = { (int)(avg_clusterColor * rand() / (RAND_MAX + 1.0)) + i * avg_clusterColor,
							255 - (int)(avg_clusterColor * rand() / (RAND_MAX + 1.0)) - i * avg_clusterColor,
							(int)(255 * rand() / (RAND_MAX + 1.0)) };
	}
	emit infoSignal("--finish random color generation");

	// 初始centers和tags
	this->centers = GlobalFun::randomSampling(origin, size);
	originInfo.clear();
	for (PointXYZ pt : origin->points) {
		double min_dis = 100;
		int min_tag = -1;
		for (int i = 0; i < size; ++i) {
			double dis = GlobalFun::euclidDistance(pt, centers->points[i]);
			if (dis < min_dis) {
				min_dis = dis;
				min_tag = i;
			}
		}
		if (min_tag < 0 || min_tag >= size) {
			emit errorSignal("error in over-seg");
		}
		originInfo.push_back(BayesInfo(min_tag, pt));
	}
	emit infoSignal("--start iteration");
	//迭代过程
	int time = this->para.getInt("max_seg_iterate_time");
	double seg_threshold = this->para.getDouble("seg_error_threshold");
	PointCloud<PointXYZ>::Ptr next_centers(new PointCloud<PointXYZ>);
	next_centers->points.resize(centers->points.size(), PointXYZ(0,0,0));
	this->segment_size = vector<int>(centers->points.size(), 0);
	for (int t = 0; t < time; ++t) {
		// 更新centers
		for (BayesInfo bi : originInfo) {
			if (bi.tag >= 0 && bi.tag < size) {
				next_centers->points[bi.tag].x += bi.pos.x;
				next_centers->points[bi.tag].y += bi.pos.y;
				next_centers->points[bi.tag].z += bi.pos.z;
				segment_size[bi.tag]++;
			}
		}
		for (int i = 0; i < centers->points.size(); ++i) {
			next_centers->points[i].x /= segment_size[i];
			next_centers->points[i].y /= segment_size[i];
			next_centers->points[i].z /= segment_size[i];
		}
		// 更新tags
		for (int i = 0; i < originInfo.size(); ++i) {
			double min_dis = 100;
			int tag = 0;
			for (PointXYZ center_pt : centers->points) {
				double dis = GlobalFun::euclidDistance(center_pt, originInfo[i].pos);
				if (dis < min_dis) {
					min_dis = dis;
					originInfo[i].tag = tag;
				}
				tag++;
			}
		}


		// 迭代停止条件
		double avg_error = 0.0;
		int i = 0;
		for (PointXYZ pt : centers->points) {
			avg_error += GlobalFun::euclidDistance(pt, next_centers->points[i]);
			++i;
		}
		avg_error /= size;
		if (avg_error <= seg_threshold) {
			emit infoSignal("迭代" + QString::number(t) + "次计算出over-seg");
			break;
		}
	}
	emit infoSignal("迭代" + QString::number(50) + "次计算出over-seg");

	//生成RGB点云	
	this->segCloud->points.clear();
	for (BayesInfo bi : originInfo) {
		pi::RGB color = { 0,0,0 };
		if (bi.tag >= 0 && bi.tag < size) {
			color = clusterColor[bi.tag];
		}
		PointXYZRGB cpt = PointXYZRGB(color.r, color.g, color.b);
		cpt.x = bi.pos.x;
		cpt.y = bi.pos.y;
		cpt.z = bi.pos.z;
		this->segCloud->points.push_back(cpt);
	}
	emit segSignal();
}

void Bayes::run()
{
	computeOverSeg();
	emit segSignal();
	//return;

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
	int merge_threshold = para.getInt("seg_merge_iterate_time");
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
		if (total_iterate_count % merge_threshold == merge_threshold-1) {
			//每T次执行一次
			computeSegMerge();
		}
		total_iterate_count++;
		emit infoSignal("iterate time: " + QString::number(total_iterate_count));
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
		if (error < err_threshold || i == N - 1) {
			if (updateRadius() >= h_boundary) {
				break;
			}
			i = 0;
		}
	}

	emit endSignal();
}

/*segment 合并*/
void Bayes::computeSegMerge() 
{
	// 计算各个块的prob
	vector<double> probs;
	for (int s = 0; s < centers->points.size(); ++s) {
		double sp = getSkelProbOfSegment(s);
		probs.push_back(sp);
	}
	// 开始比较各个块的合并可能性
	for (int s1 = 0; s1 < centers->points.size(); ++s1) {
		for (int s2 = s1+1; s2 < centers->points.size(); ++s2) {
			double merge_prob = getSkelProbOfTWOSegment(s1, s2);
			if (merge_prob > (probs[s1] + probs[s1]) / 2) {
				if (GlobalFun::euclidDistance(centers->points[s1], centers->points[s2]) < h*2) {
					mergeSegments(s1, s2);
				}
			}
		}
	}
}

void Bayes::mergeSegments(int s1, int s2)
{
	// TO-DO：segment合并
}

double Bayes::getSkelProbOfTWOSegment(int s1, int s2)
{
	if (s1 >= centers->points.size() || s1 < 0) return 0.0;
	if (s2 >= centers->points.size() || s2 < 0) return 0.0;
	// 寻找{X^*_m}
	vector<PointXYZ> segSkel;
	if (skelPtr->size() > 0) {
		for (vector<PointXYZ> vp : *skelPtr) {
			for (PointXYZ pt : vp) {
				if (GlobalFun::euclidDistance(pt, centers->points[s1]) < h ||
					GlobalFun::euclidDistance(pt, centers->points[s2]) < h) {
					segSkel.push_back(pt);
				}
			}
		}
	}

	int seg_size = segSkel.size();
	// 计算均值和方差，并记录dist
	vector<double> dist_vec;
	double avg_dist = 0.0;
	for (BayesInfo bi : originInfo) {
		if (bi.tag == s1 || bi.tag == s2) {
			PointXYZ nearestPt = centers->points[s1];
			double dist = GlobalFun::euclidDistance(bi.pos, nearestPt);
			if (seg_size > 0) {
				for (PointXYZ pt : segSkel) {
					double pt_dist = GlobalFun::euclidDistance(bi.pos, pt);
					if (pt_dist < dist) {
						dist = pt_dist;
					}
				}
			}
			dist_vec.push_back(dist);
			avg_dist += dist;
		}
	}
	avg_dist /= dist_vec.size();

	double var = 0.0;
	for (double d : dist_vec) {
		var += (d - avg_dist)*(d - avg_dist);
	}
	var = sqrt(var / dist_vec.size());

	//遍历验证距离分布
	double low_bound = avg_dist - var;
	double high_bound = avg_dist - var;
	int out_num = 0, dist_size = dist_vec.size();
	for (double d : dist_vec) {
		if (d<low_bound || d>high_bound) {
			out_num++;
		}
	}
	double percent = (dist_size - out_num) / dist_size;
	double result = abs(percent - 0.68) / 0.68;
	if (result > 1) result = 1;
	else if (result < 0) result = 0;
	return result;
}




double Bayes::getSkelProbOfSegment(int s)
{
	if (s >= centers->points.size() || s < 0) return 0.0;
	// 寻找{X^*_m}
	vector<PointXYZ> segSkel;
	if (skelPtr->size() > 0) {
		for (vector<PointXYZ> vp : *skelPtr) {
			for (PointXYZ pt : vp) {
				if (GlobalFun::euclidDistance(pt, centers->points[s]) < h) {
					segSkel.push_back(pt);
				}
			}
		}
	}

	int seg_size = segSkel.size();
	// 计算均值和方差，并记录dist
	vector<double> dist_vec;
	double avg_dist = 0.0;
	for (BayesInfo bi : originInfo) {
		if (bi.tag == s) {
			PointXYZ nearestPt = centers->points[s];
			double dist = GlobalFun::euclidDistance(bi.pos, nearestPt);
			if (seg_size > 0) {
				for (PointXYZ pt : segSkel) {
					double pt_dist = GlobalFun::euclidDistance(bi.pos, pt);
					if (pt_dist < dist) {
						dist = pt_dist;
					}
				}
			}
			dist_vec.push_back(dist);
			avg_dist += dist;
		}
	}
	avg_dist /= dist_vec.size();

	double var = 0.0;
	for (double d : dist_vec) {
		var += (d - avg_dist)*(d - avg_dist);
	}
	var = sqrt(var / dist_vec.size());

	//遍历验证距离分布
	double low_bound = avg_dist - var;
	double high_bound = avg_dist - var;
	int out_num = 0, dist_size = dist_vec.size();
	for (double d : dist_vec) {
		if (d<low_bound || d>high_bound) {
			out_num++;
		}
	}
	double percent = (dist_size - out_num) / dist_size;
	double result = abs(percent - 0.68) / 0.68;
	if (result > 1) result = 1;
	else if (result < 0) result = 0;
	return result;
}

double Bayes::iterateReturnError()
{
	/*位置迭代更新*/
	paraPtr->add("neighborhood_size", h);
	for (int i = 0; i < sampleInfo.size(); ++i) {
		sampleInfo[i].kind = (sampleInfo[i].kind == pi::Candidate) ? pi::Sample : sampleInfo[i].kind;
	}

	computeAlphasTerms();
	computeBetasTerms();
	double error = updateSamplePos();
	emit infoSignal("RMS:" + QString::number(error, 'f', 6) + '\n');

	/*位置同步*/
	if (synSampleWithInfo()) {
		emit iterateSignal();
	}
	else {
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

double Bayes::updateSamplePos()
{
	int size = this->sampleInfo.size();
	double prob_factor = 1.0 + para.getDouble("prob_tactor");
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
		PointXYZ diff(
			newPt.x - sampleInfo[i].pos.x,
			newPt.y - sampleInfo[i].pos.y,
			newPt.z - sampleInfo[i].pos.z
		);
		newPt = PointXYZ(
			sampleInfo[i].pos.x + prob_factor * diff.x,
			sampleInfo[i].pos.y + prob_factor * diff.y,
			sampleInfo[i].pos.z + prob_factor * diff.z
		);
		sampleInfo[i].pos = newPt;
	}
	if (count == 0) return 0;
	return error / count;
}

