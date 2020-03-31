#pragma once

#include <vector>
#include <utility>
#include "ParameterMgr.h"

using namespace std;

namespace GlobalDef {
	const string TestMoving = "TestMoving";
	const string L1median = "L1median";
	const string Common = "Common";

	// 参数设定
	vector<pair<string, Value>> PARA_TestMoving = {
		make_pair("iterate_time",	5	),
		make_pair("moving_length",	0.1	)
	};

	vector<pair<string, Value>> PARA_L1median = {
		make_pair("neighborhood_size",	1.0),
		make_pair("h_increasing_rate",	0.5),
		make_pair("average_power",		1),
		make_pair("repulsion_power",	2),
		make_pair("use_desity_weight", int(true)),
		make_pair("max_iterate_time", 5),
		make_pair("moving_error_threshold", 0.005),
		make_pair("candidate_sigma_threshold", 0.75),
		make_pair("too_close_dist_threshold", 0.001),
		make_pair("repulsion_factor", 0.15)
	};

	vector<pair<string, Value>> PARA_Common = {
		make_pair("skeleton_branch_size", 0.01),
		make_pair("skeleton_point_size", 0.02),
		make_pair("sample_point_size", 8)
	};


	vector<pair<string, vector<pair<string, Value>>>> TOTAL_PARA_LIST = {
		make_pair(TestMoving, PARA_TestMoving),
		make_pair(L1median, PARA_L1median),
		make_pair(Common, PARA_Common)
	};
};

