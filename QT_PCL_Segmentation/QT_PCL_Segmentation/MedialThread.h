#include <QThread>
#include "QT_PCL_Segmentation.h"
#pragma once
class MedialThread :
	public QThread
{
private:
	QT_PCL_Segmentation* window;
public:
	MedialThread(QT_PCL_Segmentation* wp);
	~MedialThread();
protected:
	void run();
};

