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
	if (info.size() == 0) info.resize(cloud->points.size());
	else if (cloud->points.size() != info.size()) return -1;
	int index = 0;
	for (PointXYZ pt : cloud->points) {
		info[index].setPos(pt);
		index++;
	}
	return 0;
}

double GlobalFun::weight(float r, double h) {
	return exp(-4 * (r*r) / (h*h));
}


PointXYZ GlobalFun::nextPos(SamplePoint xi, pi::PcPtr xc, pi::PcPtr qc, double mu) {
	Vector3d up1(0,0,0);
	double down1 = 0.0;
	int i = 0;
	for (double a : xi.alpha) {
		Vector3d q(qc->points[i].x, qc->points[i].y, qc->points[i].z);
		up1 += a * q;
		down1 += a;
		i++;
	}

	Vector3d up2(0, 0, 0);
	double down2 = 0.0;
	int j = 0;
	for (double b : xi.beta) {
		Vector3d xii(
			xi.pos.x - xc->points[j].x,
			xi.pos.y - xc->points[j].y,
			xi.pos.z - xc->points[j].z
		);
		up2 += b * xii;
		down2 += b;
		j++;
	}
	Vector3d res = up1 / down1 + mu * xi.getSigma()*up2 / down2;
	for (int i = 0; i < 3; ++i) {
		if (!isfinite(res(i))) res(i) = 1.2;
		else if (res(i) > 1.2) res(i) = 1.2;
		else if (res(i) < -1.2) res(i) = -1.2;
	}
	return PointXYZ(res(0), res(1), res(2));
}
