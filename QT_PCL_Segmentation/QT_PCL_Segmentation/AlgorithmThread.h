#pragma once

#include <QThread>
#include "PointCloudAlgorithm.h"

class AlgorithmThread :
	public QThread
{
private:
	PointCloudAlgorithm *algorithm;

public:
	AlgorithmThread(PointCloudAlgorithm*);
	~AlgorithmThread();

protected:
	void run();
};

