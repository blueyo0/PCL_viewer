#pragma once

#include "ParameterMgr.h"
#include <QObject>
#include <QString>

class PointCloudAlgorithm : public QObject
{
	Q_OBJECT
signals:
	void iterateSignal();
	void errorSignal(const QString &name);

public:
	PointCloudAlgorithm(ParameterSet) {}
	virtual ~PointCloudAlgorithm() {}

	virtual void setParameterSet(ParameterSet) = 0;
	virtual ParameterSet getParameterSet() = 0;
	virtual void run() = 0;
	virtual void iterate() = 0;
	virtual void reset() = 0;

protected:
	PointCloudAlgorithm() {}
};

