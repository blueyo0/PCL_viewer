#include "TestMoving.h"
#include <QThread>


TestMoving::TestMoving(ParameterSet para_ptr, PointCloud<PointXYZ>::Ptr inCloud) {
	this->para = para_ptr;
	this->cloud = inCloud;
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
	emit iterateSignal();
}

void TestMoving::run()
{
	int n = this->para.getInt("iterate_time");
	for (int i = 0; i < n; ++i) {
		iterate();
		QThread::sleep(1);
	}
	emit endSignal();
}

void TestMoving::reset()
{

}


