#include "L1median.h"
#include <QThread>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>

L1median::L1median(ParameterSet in_paraSet, PointCloud<PointXYZ>::Ptr in_origin, PointCloud<PointXYZ>::Ptr in_sample)
	: origin(in_origin), sample(in_sample)
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
	/*预备信息计算*/
	computeNeighbors();
	computeSigmas();
	int N = this->para.getInt("max_iterate_time");
	double err_threshold = this->para.getDouble("moving_error_threshold");
	for (int i = 0; i < N; ++i) {
		double error = iterateReturnError();
		if (error < err_threshold) {
			break;
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
	updateSamplePos();

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

	/*tracing搜寻骨骼*/
	// TO-DO：tracing




	return 0.1;
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
	emit infoSignal("initial neighborhood size: "+QString::number(h0));

	this->h_increment = this->para.getDouble("h_increasing_rate")*h0;
	this->avg_power = this->para.getInt("average_power");
	this->rep_power = this->para.getInt("repulsion_power");
	this->isDensityWeighted = bool(this->para.getInt("use_desity_weight"));
	
	//initialize sample Info
	this->sampleInfo.clear();
	for (PointXYZ pt : this->sample->points) {
		this->sampleInfo.push_back(L1SampleInfo(pt));
	}
}


void L1median::computeNeighbors()
{
	// origin neighbors
	KdTreeFLANN<PointXYZ> kdtree;
	kdtree.setInputCloud(origin);
	int index = 0;
	for (L1SampleInfo xsi : sampleInfo) {
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
	for (int i = 0; i < this->sample->points.size(); ++i) {

		vector<int> neighIndex;
		vector<float> neighDistance;
		int num = kdtree2.radiusSearch(i, this->h, neighIndex, neighDistance);

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
	emit infoSignal("compute sigmas");
	QThread::sleep(1);
}

double L1median::updateRadius()
{
	emit infoSignal("update radius");
	// TO-DO update raduis
	return this->h;
}

bool L1median::synSampleWithInfo()
{
	if (this->sample->points.size() != this->sampleInfo.size()) {
		return false;
	}

	int i = 0;
	for (L1SampleInfo si : this->sampleInfo) {
		this->sample->points[i] = si.pos;
		i++;
	}

	return true;
}

void L1median::computeAlphasTerms()
{
	for (int i = 0; i < this->sampleInfo.size(); ++i) {
		sampleInfo[i].alpha.clear();
		// TO-DO: alpha&beta



		sampleInfo[i].average_term = this->sample->points[i];
	}
	emit infoSignal("compute Alphas Terms");
}

void L1median::computeBetasTerms()
{
	for (int i = 0; i < this->sampleInfo.size(); ++i) {
		sampleInfo[i].beta.clear();




		sampleInfo[i].repulsion_term = {0, 0.05, 0};
	}
	emit infoSignal("compute Betas Terms");
}

void L1median::updateSamplePos()
{
	for (int i = 0; i < this->sampleInfo.size(); ++i) {
		sampleInfo[i].pos.x = sampleInfo[i].average_term.x + sampleInfo[i].repulsion_term.x;
		sampleInfo[i].pos.y = sampleInfo[i].average_term.y + sampleInfo[i].repulsion_term.y;
		sampleInfo[i].pos.z = sampleInfo[i].average_term.z + sampleInfo[i].repulsion_term.z;;
	}
}


void L1median::growAllBranches()
{
	emit infoSignal("grow all branches");
}