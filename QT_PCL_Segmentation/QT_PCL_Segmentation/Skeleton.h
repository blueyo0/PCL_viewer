#pragma once
#include <vector>
#include <pcl/point_types.h>
#include <QObject>

using namespace std;
using namespace pcl;


class Skeleton : public QObject
{
	Q_OBJECT
public:
	vector<vector<PointXYZ>> branches;
public:
	Skeleton() {}
	~Skeleton() {}
	void addBranch(vector<PointXYZ> branch) {
		branches.push_back(branch);
	}
};

