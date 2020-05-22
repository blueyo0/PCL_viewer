#include "Bayes.h"

Bayes::Bayes(ParameterSet* in_paraSet, PointCloud<PointXYZ>::Ptr in_origin,
	PointCloud<PointXYZ>::Ptr in_sample, vector<vector<PointXYZ>>* in_skel,
	vector<pi::PtKind> *in_status)
	:L1median::L1median(in_paraSet, in_origin, in_sample, in_skel, in_status)
{
	
}

Bayes::~Bayes()
{
}
