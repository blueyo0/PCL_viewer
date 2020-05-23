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
	std::vector<pi::RGB> clusterColor(size, { 0,0,0 });
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
	vector<int> segment_size(centers->points.size(), 0);
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
			break;
		}
	}

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
	return;

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