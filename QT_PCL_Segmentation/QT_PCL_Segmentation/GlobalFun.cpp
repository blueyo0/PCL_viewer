#include "GlobalFun.h"

double GlobalFun::computeDirectionalityDegree(vector<PointXYZ> diff) {
	if (diff.size() < 1) return 0;
	Matrix3d mm;
	for (PointXYZ d : diff) {
		Vector3d v(d.x, d.y, d.z);
		mm += v * v.adjoint();
	}
	Vector3cd eigens = mm.eigenvalues();
	Vector3d eigenVec(eigens(0).real(), eigens(1).real(), eigens(2).real());
	return eigenVec.maxCoeff() / eigens.sum().real();
}

int GlobalFun::synInfoWithCloud(vector<SamplePoint> &info, PointCloud<PointXYZ>::Ptr cloud) {
	// 先判断cloud 和 info大小是否匹配
	if (cloud->points.size() != info.size()) return -1;
	int index = 0;
	for (PointXYZ pt : cloud->points) {
		info[index].setPos(pt);
		index++;
	}
	return 0;
}
