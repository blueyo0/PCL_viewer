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
#include <QtWidgets/QDockWidget>
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
    QAction *actionnormalize;
    QAction *actionoff_ply;
    QAction *actionsave_NOFF;
    QAction *actiondown_sample;
    QAction *actionrandom_missing;
    QAction *actionopen_txt;
    QAction *actionvis;
    QAction *actionseg;
    QAction *actioninfo;
    QAction *actionskel;
    QAction *actionother;
    QAction *actionsample;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menufile;
    QMenu *menu;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QDockWidget *sampleDock;
    QWidget *otherWidget_2;
    QPushButton *segButton_3;
    QDockWidget *segDock;
    QWidget *segWidget;
    QLabel *label_13;
    QLabel *label_5;
    QPushButton *segButton;
    QTextEdit *Radius;
    QLabel *label_4;
    QLabel *label_8;
    QLabel *label_10;
    QTextEdit *SourceWeight;
    QLabel *label_6;
    QTextEdit *NumOfNeighbor;
    QLabel *label_9;
    QTextEdit *cValue;
    QPushButton *segButton_2;
    QLabel *label_11;
    QTextEdit *K_1;
    QTextEdit *Radius_2;
    QLabel *label_7;
    QDockWidget *otherDock;
    QWidget *otherWidget;
    QPushButton *downSampleButton;
    QTextEdit *noise;
    QPushButton *outlierButton;
    QPushButton *noiseButton;
    QTextEdit *outlier;
    QTextEdit *leafSize;
    QDockWidget *infoDock;
    QWidget *infoWidget;
    QPushButton *resetButton;
    QPushButton *showButton;
    QTextEdit *InfoText;
    QPushButton *clearButton;
    QDockWidget *skelDock2;
    QWidget *skelWidget_2;
    QLabel *label_19;
    QTextEdit *skelSizeValue;
    QPushButton *drawButton_6;
    QDockWidget *skelDock;
    QWidget *skelWidget;
    QLabel *label_3;
    QPushButton *drawButton;
    QLabel *label_14;
    QTextEdit *K_2;
    QTextEdit *K_3;
    QLabel *label_15;
    QLabel *label_12;
    QDockWidget *visDock;
    QWidget *visWidget;
    QVTKWidget *qvtkWidget;

    void setupUi(QMainWindow *QT_PCL_SegmentationClass)
    {
        if (QT_PCL_SegmentationClass->objectName().isEmpty())
            QT_PCL_SegmentationClass->setObjectName(QStringLiteral("QT_PCL_SegmentationClass"));
        QT_PCL_SegmentationClass->resize(1334, 988);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(QT_PCL_SegmentationClass->sizePolicy().hasHeightForWidth());
        QT_PCL_SegmentationClass->setSizePolicy(sizePolicy);
        QT_PCL_SegmentationClass->setLayoutDirection(Qt::LeftToRight);
        actionopen = new QAction(QT_PCL_SegmentationClass);
        actionopen->setObjectName(QStringLiteral("actionopen"));
        actionnormalize = new QAction(QT_PCL_SegmentationClass);
        actionnormalize->setObjectName(QStringLiteral("actionnormalize"));
        actionoff_ply = new QAction(QT_PCL_SegmentationClass);
        actionoff_ply->setObjectName(QStringLiteral("actionoff_ply"));
        actionsave_NOFF = new QAction(QT_PCL_SegmentationClass);
        actionsave_NOFF->setObjectName(QStringLiteral("actionsave_NOFF"));
        actiondown_sample = new QAction(QT_PCL_SegmentationClass);
        actiondown_sample->setObjectName(QStringLiteral("actiondown_sample"));
        actionrandom_missing = new QAction(QT_PCL_SegmentationClass);
        actionrandom_missing->setObjectName(QStringLiteral("actionrandom_missing"));
        actionopen_txt = new QAction(QT_PCL_SegmentationClass);
        actionopen_txt->setObjectName(QStringLiteral("actionopen_txt"));
        actionvis = new QAction(QT_PCL_SegmentationClass);
        actionvis->setObjectName(QStringLiteral("actionvis"));
        actionvis->setCheckable(true);
        actionvis->setChecked(true);
        actionseg = new QAction(QT_PCL_SegmentationClass);
        actionseg->setObjectName(QStringLiteral("actionseg"));
        actionseg->setCheckable(true);
        actionseg->setChecked(true);
        actioninfo = new QAction(QT_PCL_SegmentationClass);
        actioninfo->setObjectName(QStringLiteral("actioninfo"));
        actioninfo->setCheckable(true);
        actioninfo->setChecked(true);
        actionskel = new QAction(QT_PCL_SegmentationClass);
        actionskel->setObjectName(QStringLiteral("actionskel"));
        actionskel->setCheckable(true);
        actionskel->setChecked(true);
        actionother = new QAction(QT_PCL_SegmentationClass);
        actionother->setObjectName(QStringLiteral("actionother"));
        actionother->setCheckable(true);
        actionother->setChecked(true);
        actionsample = new QAction(QT_PCL_SegmentationClass);
        actionsample->setObjectName(QStringLiteral("actionsample"));
        actionsample->setCheckable(true);
        actionsample->setChecked(true);
        centralWidget = new QWidget(QT_PCL_SegmentationClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        centralWidget->setEnabled(false);
        centralWidget->setStyleSheet(QStringLiteral(""));
        QT_PCL_SegmentationClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(QT_PCL_SegmentationClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1334, 23));
        menufile = new QMenu(menuBar);
        menufile->setObjectName(QStringLiteral("menufile"));
        menu = new QMenu(menuBar);
        menu->setObjectName(QStringLiteral("menu"));
        QT_PCL_SegmentationClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(QT_PCL_SegmentationClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        QT_PCL_SegmentationClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(QT_PCL_SegmentationClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        QT_PCL_SegmentationClass->setStatusBar(statusBar);
        sampleDock = new QDockWidget(QT_PCL_SegmentationClass);
        sampleDock->setObjectName(QStringLiteral("sampleDock"));
        sampleDock->setMinimumSize(QSize(310, 70));
        sampleDock->setAllowedAreas(Qt::RightDockWidgetArea);
        otherWidget_2 = new QWidget();
        otherWidget_2->setObjectName(QStringLiteral("otherWidget_2"));
        segButton_3 = new QPushButton(otherWidget_2);
        segButton_3->setObjectName(QStringLiteral("segButton_3"));
        segButton_3->setGeometry(QRect(10, 10, 91, 31));
        QFont font;
        font.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font.setPointSize(12);
        segButton_3->setFont(font);
        sampleDock->setWidget(otherWidget_2);
        QT_PCL_SegmentationClass->addDockWidget(static_cast<Qt::DockWidgetArea>(2), sampleDock);
        segDock = new QDockWidget(QT_PCL_SegmentationClass);
        segDock->setObjectName(QStringLiteral("segDock"));
        segDock->setMinimumSize(QSize(310, 210));
        segDock->setStyleSheet(QStringLiteral(""));
        segDock->setAllowedAreas(Qt::RightDockWidgetArea);
        segWidget = new QWidget();
        segWidget->setObjectName(QStringLiteral("segWidget"));
        label_13 = new QLabel(segWidget);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(230, 150, 16, 31));
        QFont font1;
        font1.setFamily(QStringLiteral("Times New Roman"));
        font1.setPointSize(10);
        label_13->setFont(font1);
        label_13->setContextMenuPolicy(Qt::DefaultContextMenu);
        label_5 = new QLabel(segWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(20, 90, 16, 31));
        label_5->setFont(font1);
        label_5->setContextMenuPolicy(Qt::DefaultContextMenu);
        segButton = new QPushButton(segWidget);
        segButton->setObjectName(QStringLiteral("segButton"));
        segButton->setGeometry(QRect(10, 30, 91, 31));
        segButton->setFont(font);
        Radius = new QTextEdit(segWidget);
        Radius->setObjectName(QStringLiteral("Radius"));
        Radius->setGeometry(QRect(50, 90, 51, 31));
        QFont font2;
        font2.setFamily(QString::fromUtf8("004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223"));
        font2.setPointSize(9);
        Radius->setFont(font2);
        label_4 = new QLabel(segWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 0, 111, 31));
        label_8 = new QLabel(segWidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(10, 60, 191, 31));
        label_10 = new QLabel(segWidget);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(20, 150, 16, 31));
        label_10->setFont(font1);
        label_10->setContextMenuPolicy(Qt::DefaultContextMenu);
        SourceWeight = new QTextEdit(segWidget);
        SourceWeight->setObjectName(QStringLiteral("SourceWeight"));
        SourceWeight->setGeometry(QRect(250, 90, 51, 31));
        QFont font3;
        font3.setFamily(QString::fromUtf8("004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223"));
        font3.setPointSize(12);
        SourceWeight->setFont(font3);
        label_6 = new QLabel(segWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(120, 90, 31, 31));
        label_6->setFont(font1);
        label_6->setContextMenuPolicy(Qt::DefaultContextMenu);
        NumOfNeighbor = new QTextEdit(segWidget);
        NumOfNeighbor->setObjectName(QStringLiteral("NumOfNeighbor"));
        NumOfNeighbor->setGeometry(QRect(160, 90, 51, 31));
        NumOfNeighbor->setFont(font3);
        label_9 = new QLabel(segWidget);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(10, 120, 301, 31));
        cValue = new QTextEdit(segWidget);
        cValue->setObjectName(QStringLiteral("cValue"));
        cValue->setGeometry(QRect(250, 150, 51, 31));
        cValue->setFont(font3);
        segButton_2 = new QPushButton(segWidget);
        segButton_2->setObjectName(QStringLiteral("segButton_2"));
        segButton_2->setGeometry(QRect(110, 30, 91, 31));
        segButton_2->setFont(font);
        label_11 = new QLabel(segWidget);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(130, 150, 16, 31));
        label_11->setFont(font1);
        label_11->setContextMenuPolicy(Qt::DefaultContextMenu);
        K_1 = new QTextEdit(segWidget);
        K_1->setObjectName(QStringLiteral("K_1"));
        K_1->setGeometry(QRect(160, 150, 51, 31));
        K_1->setFont(font3);
        Radius_2 = new QTextEdit(segWidget);
        Radius_2->setObjectName(QStringLiteral("Radius_2"));
        Radius_2->setGeometry(QRect(50, 150, 51, 31));
        Radius_2->setFont(font3);
        label_7 = new QLabel(segWidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(220, 90, 31, 31));
        label_7->setFont(font1);
        label_7->setContextMenuPolicy(Qt::DefaultContextMenu);
        segDock->setWidget(segWidget);
        QT_PCL_SegmentationClass->addDockWidget(static_cast<Qt::DockWidgetArea>(2), segDock);
        otherDock = new QDockWidget(QT_PCL_SegmentationClass);
        otherDock->setObjectName(QStringLiteral("otherDock"));
        otherDock->setMinimumSize(QSize(310, 110));
        otherDock->setAllowedAreas(Qt::RightDockWidgetArea);
        otherWidget = new QWidget();
        otherWidget->setObjectName(QStringLiteral("otherWidget"));
        downSampleButton = new QPushButton(otherWidget);
        downSampleButton->setObjectName(QStringLiteral("downSampleButton"));
        downSampleButton->setGeometry(QRect(190, 50, 111, 31));
        downSampleButton->setFont(font);
        noise = new QTextEdit(otherWidget);
        noise->setObjectName(QStringLiteral("noise"));
        noise->setGeometry(QRect(20, 10, 51, 31));
        noise->setFont(font3);
        outlierButton = new QPushButton(otherWidget);
        outlierButton->setObjectName(QStringLiteral("outlierButton"));
        outlierButton->setGeometry(QRect(230, 10, 71, 31));
        outlierButton->setFont(font);
        noiseButton = new QPushButton(otherWidget);
        noiseButton->setObjectName(QStringLiteral("noiseButton"));
        noiseButton->setGeometry(QRect(80, 10, 71, 31));
        noiseButton->setFont(font);
        outlier = new QTextEdit(otherWidget);
        outlier->setObjectName(QStringLiteral("outlier"));
        outlier->setGeometry(QRect(170, 10, 51, 31));
        outlier->setFont(font3);
        leafSize = new QTextEdit(otherWidget);
        leafSize->setObjectName(QStringLiteral("leafSize"));
        leafSize->setGeometry(QRect(20, 50, 131, 31));
        leafSize->setFont(font3);
        otherDock->setWidget(otherWidget);
        QT_PCL_SegmentationClass->addDockWidget(static_cast<Qt::DockWidgetArea>(2), otherDock);
        infoDock = new QDockWidget(QT_PCL_SegmentationClass);
        infoDock->setObjectName(QStringLiteral("infoDock"));
        infoDock->setMinimumSize(QSize(310, 290));
        infoDock->setAllowedAreas(Qt::RightDockWidgetArea);
        infoWidget = new QWidget();
        infoWidget->setObjectName(QStringLiteral("infoWidget"));
        resetButton = new QPushButton(infoWidget);
        resetButton->setObjectName(QStringLiteral("resetButton"));
        resetButton->setGeometry(QRect(210, 200, 91, 61));
        QFont font4;
        font4.setFamily(QString::fromUtf8("004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223"));
        font4.setPointSize(28);
        resetButton->setFont(font4);
        showButton = new QPushButton(infoWidget);
        showButton->setObjectName(QStringLiteral("showButton"));
        showButton->setGeometry(QRect(110, 200, 91, 61));
        showButton->setFont(font4);
        InfoText = new QTextEdit(infoWidget);
        InfoText->setObjectName(QStringLiteral("InfoText"));
        InfoText->setGeometry(QRect(10, 0, 291, 191));
        clearButton = new QPushButton(infoWidget);
        clearButton->setObjectName(QStringLiteral("clearButton"));
        clearButton->setGeometry(QRect(10, 200, 91, 61));
        clearButton->setFont(font4);
        infoDock->setWidget(infoWidget);
        QT_PCL_SegmentationClass->addDockWidget(static_cast<Qt::DockWidgetArea>(2), infoDock);
        skelDock2 = new QDockWidget(QT_PCL_SegmentationClass);
        skelDock2->setObjectName(QStringLiteral("skelDock2"));
        skelDock2->setMinimumSize(QSize(310, 70));
        skelDock2->setStyleSheet(QStringLiteral(""));
        skelDock2->setAllowedAreas(Qt::RightDockWidgetArea);
        skelWidget_2 = new QWidget();
        skelWidget_2->setObjectName(QStringLiteral("skelWidget_2"));
        skelWidget_2->setStyleSheet(QStringLiteral(""));
        label_19 = new QLabel(skelWidget_2);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(10, 10, 31, 31));
        skelSizeValue = new QTextEdit(skelWidget_2);
        skelSizeValue->setObjectName(QStringLiteral("skelSizeValue"));
        skelSizeValue->setGeometry(QRect(40, 10, 61, 31));
        skelSizeValue->setFont(font3);
        drawButton_6 = new QPushButton(skelWidget_2);
        drawButton_6->setObjectName(QStringLiteral("drawButton_6"));
        drawButton_6->setGeometry(QRect(200, 10, 91, 31));
        drawButton_6->setFont(font);
        skelDock2->setWidget(skelWidget_2);
        QT_PCL_SegmentationClass->addDockWidget(static_cast<Qt::DockWidgetArea>(2), skelDock2);
        skelDock = new QDockWidget(QT_PCL_SegmentationClass);
        skelDock->setObjectName(QStringLiteral("skelDock"));
        skelDock->setMinimumSize(QSize(310, 160));
        skelDock->setStyleSheet(QStringLiteral(""));
        skelDock->setAllowedAreas(Qt::RightDockWidgetArea);
        skelWidget = new QWidget();
        skelWidget->setObjectName(QStringLiteral("skelWidget"));
        skelWidget->setStyleSheet(QStringLiteral(""));
        label_3 = new QLabel(skelWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 10, 141, 31));
        drawButton = new QPushButton(skelWidget);
        drawButton->setObjectName(QStringLiteral("drawButton"));
        drawButton->setGeometry(QRect(200, 10, 91, 31));
        drawButton->setFont(font);
        label_14 = new QLabel(skelWidget);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(10, 70, 81, 31));
        label_14->setFont(font1);
        label_14->setContextMenuPolicy(Qt::DefaultContextMenu);
        K_2 = new QTextEdit(skelWidget);
        K_2->setObjectName(QStringLiteral("K_2"));
        K_2->setGeometry(QRect(100, 70, 51, 31));
        K_2->setFont(font3);
        K_3 = new QTextEdit(skelWidget);
        K_3->setObjectName(QStringLiteral("K_3"));
        K_3->setGeometry(QRect(240, 70, 51, 31));
        K_3->setFont(font3);
        label_15 = new QLabel(skelWidget);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(160, 70, 71, 31));
        label_15->setFont(font1);
        label_15->setContextMenuPolicy(Qt::DefaultContextMenu);
        label_12 = new QLabel(skelWidget);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(10, 40, 281, 31));
        skelDock->setWidget(skelWidget);
        QT_PCL_SegmentationClass->addDockWidget(static_cast<Qt::DockWidgetArea>(2), skelDock);
        visDock = new QDockWidget(QT_PCL_SegmentationClass);
        visDock->setObjectName(QStringLiteral("visDock"));
        sizePolicy.setHeightForWidth(visDock->sizePolicy().hasHeightForWidth());
        visDock->setSizePolicy(sizePolicy);
        visDock->setMinimumSize(QSize(1000, 600));
        visDock->setBaseSize(QSize(1000, 960));
        visDock->setStyleSheet(QStringLiteral(""));
        visDock->setAllowedAreas(Qt::AllDockWidgetAreas);
        visWidget = new QWidget();
        visWidget->setObjectName(QStringLiteral("visWidget"));
        qvtkWidget = new QVTKWidget(visWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(0, 10, 990, 950));
        qvtkWidget->setFocusPolicy(Qt::NoFocus);
        visDock->setWidget(visWidget);
        QT_PCL_SegmentationClass->addDockWidget(static_cast<Qt::DockWidgetArea>(1), visDock);

        menuBar->addAction(menufile->menuAction());
        menuBar->addAction(menu->menuAction());
        menufile->addAction(actionopen);
        menufile->addAction(actionnormalize);
        menufile->addAction(actionoff_ply);
        menufile->addAction(actionsave_NOFF);
        menufile->addAction(actiondown_sample);
        menufile->addAction(actionrandom_missing);
        menufile->addAction(actionopen_txt);
        menu->addAction(actionvis);
        menu->addAction(actionseg);
        menu->addAction(actioninfo);
        menu->addAction(actionskel);
        menu->addAction(actionother);
        menu->addAction(actionsample);

        retranslateUi(QT_PCL_SegmentationClass);

        QMetaObject::connectSlotsByName(QT_PCL_SegmentationClass);
    } // setupUi

    void retranslateUi(QMainWindow *QT_PCL_SegmentationClass)
    {
        QT_PCL_SegmentationClass->setWindowTitle(QApplication::translate("QT_PCL_SegmentationClass", "PCL_viewer", nullptr));
        actionopen->setText(QApplication::translate("QT_PCL_SegmentationClass", "open", nullptr));
        actionnormalize->setText(QApplication::translate("QT_PCL_SegmentationClass", "normalize", nullptr));
        actionoff_ply->setText(QApplication::translate("QT_PCL_SegmentationClass", "off->ply", nullptr));
        actionsave_NOFF->setText(QApplication::translate("QT_PCL_SegmentationClass", "save NOFF", nullptr));
        actiondown_sample->setText(QApplication::translate("QT_PCL_SegmentationClass", "down sample", nullptr));
        actionrandom_missing->setText(QApplication::translate("QT_PCL_SegmentationClass", "random missing", nullptr));
        actionopen_txt->setText(QApplication::translate("QT_PCL_SegmentationClass", "open txt", nullptr));
        actionvis->setText(QApplication::translate("QT_PCL_SegmentationClass", "\346\230\276\347\244\272", nullptr));
        actionseg->setText(QApplication::translate("QT_PCL_SegmentationClass", "\345\210\206\345\235\227", nullptr));
        actioninfo->setText(QApplication::translate("QT_PCL_SegmentationClass", "\344\277\241\346\201\257", nullptr));
        actionskel->setText(QApplication::translate("QT_PCL_SegmentationClass", "\351\252\250\346\236\266\346\217\220\345\217\226", nullptr));
        actionother->setText(QApplication::translate("QT_PCL_SegmentationClass", "\345\205\266\345\256\203", nullptr));
        actionsample->setText(QApplication::translate("QT_PCL_SegmentationClass", "\351\207\207\346\240\267", nullptr));
        menufile->setTitle(QApplication::translate("QT_PCL_SegmentationClass", "\346\226\207\344\273\266", nullptr));
        menu->setTitle(QApplication::translate("QT_PCL_SegmentationClass", "\350\247\206\345\233\276", nullptr));
        sampleDock->setWindowTitle(QApplication::translate("QT_PCL_SegmentationClass", "\351\207\207\346\240\267", nullptr));
        segButton_3->setText(QApplication::translate("QT_PCL_SegmentationClass", "random", nullptr));
        segDock->setWindowTitle(QApplication::translate("QT_PCL_SegmentationClass", "\345\210\206\345\235\227", nullptr));
        label_13->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt;\">c</span></p></body></html>", nullptr));
        label_5->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt;\">R</span></p></body></html>", nullptr));
        segButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "over-seg", nullptr));
        Radius->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">0.1</span></p></body></html>", nullptr));
        label_4->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">segmentation</span></p></body></html>", nullptr));
        label_8->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">min-cut parameters</span></p></body></html>", nullptr));
        label_10->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt;\">R</span></p></body></html>", nullptr));
        SourceWeight->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">0.8</span></p></body></html>", nullptr));
        label_6->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt;\">NoN</span></p></body></html>", nullptr));
        NumOfNeighbor->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">14</span></p></body></html>", nullptr));
        label_9->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">local Kmeans parameters </span><span style=\" font-size:12pt; font-style:italic;\">R(.95,1.4]</span></p></body></html>", nullptr));
        cValue->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">8</span></p></body></html>", nullptr));
        segButton_2->setText(QApplication::translate("QT_PCL_SegmentationClass", "min-cut", nullptr));
        label_11->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt;\">K</span></p></body></html>", nullptr));
        K_1->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">15</span></p></body></html>", nullptr));
        Radius_2->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">1</span></p></body></html>", nullptr));
        label_7->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt;\">SW</span></p></body></html>", nullptr));
        otherDock->setWindowTitle(QApplication::translate("QT_PCL_SegmentationClass", "\345\205\266\345\256\203", nullptr));
        downSampleButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "downSample", nullptr));
        noise->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">50</span></p></body></html>", nullptr));
        outlierButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "outlier", nullptr));
        noiseButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "noise", nullptr));
        outlier->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">10</span></p></body></html>", nullptr));
        leafSize->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">0.02</span></p></body></html>", nullptr));
        infoDock->setWindowTitle(QApplication::translate("QT_PCL_SegmentationClass", "\344\277\241\346\201\257", nullptr));
        resetButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "reset", nullptr));
        showButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "test", nullptr));
        clearButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "clear", nullptr));
        skelDock2->setWindowTitle(QApplication::translate("QT_PCL_SegmentationClass", "\350\264\235\345\217\266\346\226\257", nullptr));
        label_19->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p>size</p></body></html>", nullptr));
        skelSizeValue->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">0.01</span></p></body></html>", nullptr));
        drawButton_6->setText(QApplication::translate("QT_PCL_SegmentationClass", "Bayes", nullptr));
        skelDock->setWindowTitle(QApplication::translate("QT_PCL_SegmentationClass", "L1\344\270\255\345\200\274", nullptr));
        label_3->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">skeletonization</span></p></body></html>", nullptr));
        drawButton->setText(QApplication::translate("QT_PCL_SegmentationClass", "L1-medial", nullptr));
        label_14->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt;\">h growth rate</span></p></body></html>", nullptr));
        K_2->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">0.5</span></p></body></html>", nullptr));
        K_3->setHtml(QApplication::translate("QT_PCL_SegmentationClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'004\346\226\271\346\255\243\345\217\244\351\232\266\347\256\200\344\275\223'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun'; font-size:9pt;\">0.35</span></p></body></html>", nullptr));
        label_15->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt;\">repulsion u</span></p></body></html>", nullptr));
        label_12->setText(QApplication::translate("QT_PCL_SegmentationClass", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600;\">L1-medial skeleton parameters</span></p></body></html>", nullptr));
        visDock->setWindowTitle(QApplication::translate("QT_PCL_SegmentationClass", "\346\230\276\347\244\272", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QT_PCL_SegmentationClass: public Ui_QT_PCL_SegmentationClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QT_PCL_SEGMENTATION_H
