#include "SamplePoint.h"
#include "GlobalFun.h"


SamplePoint::SamplePoint() : sigma(0), kind(pi::Sample), pos(0,0,0) {
}


SamplePoint::~SamplePoint() {
}


double SamplePoint::computeSigma(pi::PcPtr cloud, vector<SamplePoint> info) {
	// ¸ù¾Ýs_indic¼ÆËã
	vector<PointXYZ> delta;
	for (int i : this->s_indics) {
		if (info[i].kind != pi::Sample) {
			continue;
		}
		delta.push_back(PointXYZ(
			this->pos.x - cloud->points[i].x,
			this->pos.y - cloud->points[i].y,
			this->pos.z - cloud->points[i].z
		));
	}
	this->sigma = GlobalFun::computeDirectionalityDegree(delta);
	
	return this->sigma;
}


// getter & setter
void SamplePoint::setPos(PointXYZ pt) {
	this->pos.x = pt.x;
	this->pos.y = pt.y;
	this->pos.z = pt.z;
}

void SamplePoint::setPos(double x, double y, double z) {
	this->pos.x = x;
	this->pos.y = y;
	this->pos.z = z;
}

void SamplePoint::setSigma(double in) {
	this->sigma = in;
}
void SamplePoint::setOriginalIndics(vector<int> in) {
	this->o_indics = in;
}
void SamplePoint::setOriginalDistances(vector<float> in) {
	this->o_dists = in;
}
void SamplePoint::setSelfIndics(vector<int> in) {
	this->s_indics = in;
}
void SamplePoint::setSelfDistances(vector<float> in) {
	this->s_dists = in;
}

double SamplePoint::getSigma() {
	return this->sigma;
}

vector<int> SamplePoint::getOriginalIndics() {
	return this->o_indics;
}
vector<float> SamplePoint::getOriginalDistances() {
	return this->o_dists;
}
vector<int> SamplePoint::getSelfIndics() {
	return this->s_indics;
}
vector<float> SamplePoint::getSelfDistances() {
	return this->s_dists;
}
