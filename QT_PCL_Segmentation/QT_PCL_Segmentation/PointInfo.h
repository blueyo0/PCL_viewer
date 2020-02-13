#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace std;
using namespace pcl;

namespace PointInfo {
	typedef struct RGB {
		int r;
		int g;
		int b;
	}RGB;

	enum PtKind { Sample, Candidate, Bridge, Branch, Removed };

	typedef PointCloud<PointXYZ>::Ptr PcPtr;
}

namespace pi = PointInfo;