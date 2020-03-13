#pragma once

#include <vector>
#include <utility>
#include "ParameterMgr.h"

using namespace std;

namespace GlobalDef {
	const string TestMoving = "TestMoving";
	const string L1median = "L1median";

	// 参数设定
	vector<pair<string, Value>> PARA_TestMoving = {
		make_pair("iterate_time",	5	),
		make_pair("moving_length",	0.1	)
	};

	vector<pair<string, Value>> PARA_L1median = {
	};


	vector<pair<string, vector<pair<string, Value>>>> TOTAL_PARA_LIST = {
		make_pair(TestMoving, PARA_TestMoving),
		make_pair(L1median, PARA_L1median)
	};
};

