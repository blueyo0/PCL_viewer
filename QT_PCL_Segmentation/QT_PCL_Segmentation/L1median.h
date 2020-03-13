#pragma once
#include "PointCloudAlgorithm.h"
class L1median :
	public PointCloudAlgorithm
{
public:
	L1median();
	virtual ~L1median();

	virtual void run();
	virtual void iterate();
	virtual void reset();
};

