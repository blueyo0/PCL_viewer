#include "QT_PCL_Segmentation.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QT_PCL_Segmentation w;
	w.show();
	return a.exec();
}
