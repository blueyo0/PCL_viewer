#include "TestMoving.h"
#include <QThread>


TestMoving::TestMoving(ParameterSet para_ptr, PointCloud<PointXYZ>::Ptr inCloud, 
					   vector<vector<PointXYZ>>* skel, vector<pi::PtKind>* in_status) {
	this->para = para_ptr;
	this->cloud = inCloud;
	this->skeleton = skel;
	this->status = in_status;
}


TestMoving::~TestMoving()
{
}

void TestMoving::iterate()
{
	double length = this->para.getDouble("moving_length");
	if (length == -1) {
		emit errorSignal("error with moving length parameter");
		return;
	}
	for (int i = 0; i < this->cloud->points.size(); ++i) {
		this->cloud->points[i].x += length;
	}
	for (int i = 0; i < skeleton->size(); ++i) {
		for (int j = 0; j < (*skeleton)[i].size(); ++j) {
			(*skeleton)[i][j].x += length;
			(*skeleton)[i][j].y += length;
		}
	}

	emit iterateSignal();
}

void TestMoving::run()
{
	vector<PointXYZ> branch = { PointXYZ(0,0.5,0), PointXYZ(0.5,0.5,0), PointXYZ(1,0.5,0) };
	skeleton->clear();
	skeleton->push_back(branch);
	status->resize(cloud->points.size(), pi::Sample);

	int n = this->para.getInt("iterate_time");
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < this->cloud->points.size()*((1.0*i)/(n-1)); ++j) {
			if (j >= status->size()) break;
			(*status)[j] = pi::Candidate;
		}
		iterate();
		emit skelChangeSignal();
		QThread::sleep(1);
	}
	emit endSignal();
}

void TestMoving::reset()
{

}



