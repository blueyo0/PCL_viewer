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
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_QT_PCL_SegmentationClass
{
public:
    QAction *actionopen;
    QWidget *centralWidget;
    QPushButton *showButton;
    QVTKWidget *qvtkWidget;
    QPushButton *segButton;
    QPushButton *drawButton;
    QTextEdit *InfoText;
    QLabel *label_2;
    QPushButton *drawButton_2;
    QPushButton *segButton_2;
    QPushButton *clearButton;
    QLabel *label_3;
    QLabel *label_4;
    QTextEdit *Radius;
    QLabel *label_5;
    QTextEdit *NumOfNeighbor;
    QLabel *label_6;
    QLabel *label_7;
    QTextEdit *SourceWeight;
    QLabel *label_8;
    QTextEdit *Radius_2;
    QLabel *label_9;
    QTextEdit *K_1;
    QLabel *label_10;
    QLabel *label_11;
    QPushButton *segButton_3;
    QLabel *label_12;
    QTextEdit *K_2;
    QLabel *label_14;
    QTextEdit *cValue;
    QLabel *label_13;
    QPushButton *drawButton_3;
    QPushButton *resetButton;
    QMenuBar *menuBar;
    QMenu *menufile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *QT_PCL_SegmentationClass)
    {
        if (QT_PCL_SegmentationClass->objectName().isEmpty())
            QT_PCL_SegmentationClass->setObjectName(QStringLiteral("QT_PCL_SegmentationClass"));
        QT_PCL_SegmentationClass->resize(1659, 935);
        actionopen = new QAction(QT_PCL_SegmentationClass);
        actionopen->setObjectName(QStringLiteral("actionopen"));
        centralWidget = new QWidget(QT_PCL_SegmentationClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        showButton = new QPushButton(centralWidget);
        showButton->setObjectName(QStringLiteral("showButton"));
        showButton->setGeometry(QRect(1440, 810, 71, 61));
        QFont font;
        font.setFamily(QString::fromUtf8("004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223"));
        font.setPointSize(20);
        showButton->setFont(font);
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(0, 0, 1341, 911));
        segButton = new QPushButton(centralWidget);
        segButton->setObjectName(QStringLiteral("segButton"));
        segButton->setGeometry(QRect(1360, 170, 91, 61));
        QFont font1;
        font1.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font1.setPointSize(12);
        segButton->setFont(font1);
        drawButton = new QPushButton(centralWidget);
        drawButton->setObjectName(QStringLiteral("drawButton"));
        drawButton->setGeometry(QRect(1360, 60, 91, 61));
        drawButton->setFont(font1);
        InfoText = new QTextEdit(centralWidget);
        InfoText->setObjectName(QStringLiteral("InfoText"));
        InfoText->setGeometry(QRect(1360, 560, 291, 241));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(1360, 530, 111, 31));
        drawButton_2 = new QPushButton(centralWidget);
        drawButton_2->setObjectName(QStringLiteral("drawButton_2"));
        drawButton_2->setGeometry(QRect(1460, 60, 91, 61));
        drawButton_2->setFont(font1);
        segButton_2 = new QPushButton(centralWidget);
        segButton_2->setObjectName(QStringLiteral("segButton_2"));
        segButton_2->setGeometry(QRect(1460, 170, 91, 61));
        segButton_2->setFont(font1);
        clearButton = new QPushButton(centralWidget);
        clearButton->setObjectName(QStringLiteral("clearButton"));
        clearButton->setGeometry(QRect(1360, 810, 71, 61));
        clearButton->setFont(font);
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(1350, 20, 151, 31));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(1350, 130, 111, 31));
        Radius = new QTextEdit(centralWidget);
        Radius->setObjectName(QStringLiteral("Radius"));
        Radius->setGeometry(QRect(1390, 260, 51, 31));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(1370, 260, 16, 31));
        NumOfNeighbor = new QTextEdit(centralWidget);
        NumOfNeighbor->setObjectName(QStringLiteral("NumOfNeighbor"));
        NumOfNeighbor->setGeometry(QRect(1480, 260, 51, 31));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(1450, 260, 31, 31));
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(1540, 260, 31, 31));
        SourceWeight = new QTextEdit(centralWidget);
        SourceWeight->setObjectName(QStringLiteral("SourceWeight"));
        SourceWeight->setGeometry(QRect(1560, 260, 51, 31));
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(1350, 230, 191, 31));
        Radius_2 = new QTextEdit(centralWidget);
        Radius_2->setObjectName(QStringLiteral("Radius_2"));
        Radius_2->setGeometry(QRect(1390, 320, 51, 31));
        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(1350, 290, 301, 31));
        K_1 = new QTextEdit(centralWidget);
        K_1->setObjectName(QStringLiteral("K_1"));
        K_1->setGeometry(QRect(1470, 320, 51, 31));
        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(1370, 320, 16, 31));
        label_11 = new QLabel(centralWidget);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(1450, 320, 16, 31));
        segButton_3 = new QPushButton(centralWidget);
        segButton_3->setObjectName(QStringLiteral("segButton_3"));
        segButton_3->setGeometry(QRect(1560, 170, 91, 61));
        segButton_3->setFont(font1);
        label_12 = new QLabel(centralWidget);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(1350, 350, 191, 31));
        K_2 = new QTextEdit(centralWidget);
        K_2->setObjectName(QStringLiteral("K_2"));
        K_2->setGeometry(QRect(1390, 380, 51, 31));
        label_14 = new QLabel(centralWidget);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(1370, 380, 16, 31));
        cValue = new QTextEdit(centralWidget);
        cValue->setObjectName(QStringLiteral("cValue"));
        cValue->setGeometry(QRect(1550, 320, 51, 31));
        label_13 = new QLabel(centralWidget);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(1530, 320, 16, 31));
        drawButton_3 = new QPushButton(centralWidget);
        drawButton_3->setObjectName(QStringLiteral("drawButton_3"));
        drawButton_3->setGeometry(QRect(1560, 60, 91, 61));
        drawButton_3->setFont(font1);
        resetButton = new QPushButton(centralWidget);
        resetButton->setObjectName(QStringLiteral("resetButton"));
        resetButton->setGeometry(QRect(1520, 810, 71, 61));
        resetButton->setFont(font);
        QT_PCL_SegmentationClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(QT_PCL_SegmentationClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1659, 23));
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
        showButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "test", nullptr));
        segButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "over-seg", nullptr));
        drawButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "L1-medial", nullptr));
        label_2->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">\344\277\241\346\201\257\346\240\217</span></p></body></html>", nullptr));
        drawButton_2->setText(QApplication::translate("QT_PCL_SegmentationClass", "Bayes", nullptr));
        segButton_2->setText(QApplication::translate("QT_PCL_SegmentationClass", "min-cut", nullptr));
        clearButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "clear", nullptr));
        label_3->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">skeletonization</span></p></body></html>", nullptr));
        label_4->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">segmentation</span></p></body></html>", nullptr));
        Radius->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">0.1</p></body></html>", nullptr));
        label_5->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">R</span></p></body></html>", nullptr));
        NumOfNeighbor->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">14</p></body></html>", nullptr));
        label_6->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">NoN</span></p></body></html>", nullptr));
        label_7->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">SW</span></p></body></html>", nullptr));
        SourceWeight->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">0.8</p></body></html>", nullptr));
        label_8->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">min-cut parameters</span></p></body></html>", nullptr));
        Radius_2->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1</p></body></html>", nullptr));
        label_9->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">local Kmeans parameters </span><span style=\" font-size:12pt; font-style:italic;\">R(.95,1.4]</span></p></body></html>", nullptr));
        K_1->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">15</p></body></html>", nullptr));
        label_10->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">R</span></p></body></html>", nullptr));
        label_11->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">K</span></p></body></html>", nullptr));
        segButton_3->setText(QApplication::translate("QT_PCL_SegmentationClass", "KNN-smooth", nullptr));
        label_12->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">KNN smooth parameters</span></p></body></html>", nullptr));
        K_2->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">15</p></body></html>", nullptr));
        label_14->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">K</span></p></body></html>", nullptr));
        cValue->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">8</p></body></html>", nullptr));
        label_13->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">c</span></p></body></html>", nullptr));
        drawButton_3->setText(QApplication::translate("QT_PCL_SegmentationClass", "only skel", nullptr));
        resetButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "reset", nullptr));
        menufile->setTitle(QApplication::translate("QT_PCL_SegmentationClass", "file", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QT_PCL_SegmentationClass: public Ui_QT_PCL_SegmentationClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QT_PCL_SEGMENTATION_H
