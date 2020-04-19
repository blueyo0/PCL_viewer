#include "QT_PCL_Segmentation.h"
#include <QtWidgets/QApplication>
#include <QDesktopWidget>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QT_PCL_Segmentation w;
	w.show();
	QDesktopWidget* desktop = QApplication::desktop();
	w.move((desktop->width() - w.width()) / 2, (desktop->height() - w.height()) / 2);
	return a.exec();
}
