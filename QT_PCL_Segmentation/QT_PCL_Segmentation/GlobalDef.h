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
		make_pair("neighborhood_size",			1.0),
		make_pair("h_increasing_rate",			0.5),
		make_pair("average_power",				1),
		make_pair("repulsion_power",			3),
		make_pair("down_sample_leaf",			0.01f),
		make_pair("max_iterate_time",			55),
		make_pair("moving_error_threshold",		0.0005),
		make_pair("moving_error_factor",		0.30),
		make_pair("candidate_sigma_threshold",	0.80),
		make_pair("too_close_dist_threshold",	0.001),
		make_pair("repulsion_factor",			0.35),
		make_pair("use_density_weight",		int(true)),
		make_pair("use_down_sample",		int(false)),
		make_pair("use_power_distance",		int(false)),
		make_pair("use_error_measurement",	int(false))
	};

	vector<pair<string, Value>> PARA_Common = {
		make_pair("skeleton_branch_size", 0.01),
		make_pair("skeleton_point_size", 0.02),
		make_pair("sample_point_size", 8),
		make_pair("sigma_display_mode", 1),
		make_pair("use_test_hole",	int(false))
	};


	vector<pair<string, vector<pair<string, Value>>>> TOTAL_PARA_LIST = {
		make_pair(TestMoving, PARA_TestMoving),
		make_pair(L1median, PARA_L1median),
		make_pair(Common, PARA_Common)
	};
};

