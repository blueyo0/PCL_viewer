#include "MedialThread.h"
#include <QMessageBox>



MedialThread::MedialThread(QT_PCL_Segmentation* wp) : window(wp)
{
}


MedialThread::~MedialThread()
{
}


void MedialThread::run() {
	//this->window->l1_median();
}