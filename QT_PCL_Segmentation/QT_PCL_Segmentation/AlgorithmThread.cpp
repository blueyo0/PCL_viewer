#include "AlgorithmThread.h"



AlgorithmThread::AlgorithmThread(PointCloudAlgorithm* a)
	: algorithm(a)
{
}


AlgorithmThread::~AlgorithmThread()
{
}

void AlgorithmThread::run() 
{
	this->algorithm->run();
}

