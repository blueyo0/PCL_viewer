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

protected:
	ParameterSet para;

public:
	PointCloudAlgorithm(ParameterSet) {}
	virtual ~PointCloudAlgorithm() {}

	virtual void setParameterSet(ParameterSet para_ptr) {this->para = para_ptr;}
	virtual ParameterSet getParameterSet() {return this->para;}

	virtual void run() = 0;
	virtual void iterate() = 0;
	virtual void reset() = 0;

protected:
	PointCloudAlgorithm() {}
};

