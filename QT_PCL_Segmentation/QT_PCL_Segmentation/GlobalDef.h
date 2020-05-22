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
		make_pair("max_iterate_time",			20),
		make_pair("moving_error_threshold",		0.0005),
		make_pair("moving_error_factor",		0.30),
		make_pair("candidate_sigma_threshold",	0.85),
		make_pair("too_close_dist_threshold",	0.01),
		make_pair("branch_max_angle",			120),
		make_pair("repulsion_factor",			0.35),
		make_pair("branch_tracing_num",			5),
		make_pair("density_thread_num",			4), 
		// 不使用多线程时couple density计算需要35.55s
		// 经过测试，线程数为8-16时可以稳定在9s内
		make_pair("use_density_weight",			int(true)),
		make_pair("use_down_sample",			int(false)),
		make_pair("use_multi_power_distance",	int(false)),
		make_pair("use_auto_error",				int(false)),
		make_pair("use_timecost_output",		int(true)),
		make_pair("use_close_neigh_removement",	int(false))
	};

	vector<pair<string, Value>> PARA_Common = {
		make_pair("skeleton_branch_size", 0.01),
		make_pair("skeleton_point_size", 0.02),
		make_pair("sample_point_size", 8),
		make_pair("sigma_display_mode", 1), // 1 为使用sigma模式， 其它为使用kind模式
		make_pair("use_test_hole",				int(false)),
		make_pair("use_coordinate",				int(false)),
		make_pair("use_ball_neigh_display",		int(true))
	};


	vector<pair<string, vector<pair<string, Value>>>> TOTAL_PARA_LIST = {
		make_pair(TestMoving, PARA_TestMoving),
		make_pair(L1median, PARA_L1median),
		make_pair(Common, PARA_Common)
	};
};

