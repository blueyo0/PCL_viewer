/********************************************************************************
** Form generated from reading UI file 'QT_PCL_Segmentation.ui'
**
** Created by: Qt User Interface Compiler version 5.11.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QT_PCL_SEGMENTATION_H
#define UI_QT_PCL_SEGMENTATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_QT_PCL_SegmentationClass
{
public:
    QAction *actionopen;
    QWidget *centralWidget;
    QLabel *label;
    QPushButton *showButton;
    QVTKWidget *qvtkWidget;
    QPushButton *segButton;
    QMenuBar *menuBar;
    QMenu *menufile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *QT_PCL_SegmentationClass)
    {
        if (QT_PCL_SegmentationClass->objectName().isEmpty())
            QT_PCL_SegmentationClass->setObjectName(QStringLiteral("QT_PCL_SegmentationClass"));
        QT_PCL_SegmentationClass->resize(926, 568);
        actionopen = new QAction(QT_PCL_SegmentationClass);
        actionopen->setObjectName(QStringLiteral("actionopen"));
        centralWidget = new QWidget(QT_PCL_SegmentationClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(660, 10, 251, 331));
        showButton = new QPushButton(centralWidget);
        showButton->setObjectName(QStringLiteral("showButton"));
        showButton->setGeometry(QRect(670, 440, 75, 23));
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(10, 10, 631, 531));
        segButton = new QPushButton(centralWidget);
        segButton->setObjectName(QStringLiteral("segButton"));
        segButton->setGeometry(QRect(780, 440, 101, 23));
        QT_PCL_SegmentationClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(QT_PCL_SegmentationClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 926, 23));
        menufile = new QMenu(menuBar);
        menufile->setObjectName(QStringLiteral("menufile"));
        QT_PCL_SegmentationClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(QT_PCL_SegmentationClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        QT_PCL_SegmentationClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(QT_PCL_SegmentationClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        QT_PCL_SegmentationClass->setStatusBar(statusBar);

        menuBar->addAction(menufile->menuAction());
        menufile->addAction(actionopen);

        retranslateUi(QT_PCL_SegmentationClass);

        QMetaObject::connectSlotsByName(QT_PCL_SegmentationClass);
    } // setupUi

    void retranslateUi(QMainWindow *QT_PCL_SegmentationClass)
    {
        QT_PCL_SegmentationClass->setWindowTitle(QApplication::translate("QT_PCL_SegmentationClass", "QT_PCL_Segmentation", nullptr));
        actionopen->setText(QApplication::translate("QT_PCL_SegmentationClass", "open", nullptr));
        label->setText(QApplication::translate("QT_PCL_SegmentationClass", "TextLabel", nullptr));
        showButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "show", nullptr));
        segButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "segmentation", nullptr));
        menufile->setTitle(QApplication::translate("QT_PCL_SegmentationClass", "file", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QT_PCL_SegmentationClass: public Ui_QT_PCL_SegmentationClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QT_PCL_SEGMENTATION_H
