/****************************************************************************
** Meta object code from reading C++ file 'QT_PCL_Segmentation.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../QT_PCL_Segmentation.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QT_PCL_Segmentation.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QT_PCL_Segmentation_t {
    QByteArrayData data[16];
    char stringdata0[178];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QT_PCL_Segmentation_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QT_PCL_Segmentation_t qt_meta_stringdata_QT_PCL_Segmentation = {
    {
QT_MOC_LITERAL(0, 0, 19), // "QT_PCL_Segmentation"
QT_MOC_LITERAL(1, 20, 8), // "showDemo"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 7), // "showPCL"
QT_MOC_LITERAL(4, 38, 6), // "onOpen"
QT_MOC_LITERAL(5, 45, 12), // "segmentation"
QT_MOC_LITERAL(6, 58, 11), // "colorByAxis"
QT_MOC_LITERAL(7, 70, 5), // "color"
QT_MOC_LITERAL(8, 76, 35), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(9, 112, 8), // "drawLine"
QT_MOC_LITERAL(10, 121, 6), // "kmeans"
QT_MOC_LITERAL(11, 128, 8), // "readSkel"
QT_MOC_LITERAL(12, 137, 11), // "std::string"
QT_MOC_LITERAL(13, 149, 8), // "filename"
QT_MOC_LITERAL(14, 158, 8), // "drawSkel"
QT_MOC_LITERAL(15, 167, 10) // "reDrawSkel"

    },
    "QT_PCL_Segmentation\0showDemo\0\0showPCL\0"
    "onOpen\0segmentation\0colorByAxis\0color\0"
    "pcl::PointCloud<pcl::PointXYZ>::Ptr\0"
    "drawLine\0kmeans\0readSkel\0std::string\0"
    "filename\0drawSkel\0reDrawSkel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QT_PCL_Segmentation[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x08 /* Private */,
       3,    0,   70,    2, 0x08 /* Private */,
       4,    0,   71,    2, 0x08 /* Private */,
       5,    0,   72,    2, 0x08 /* Private */,
       6,    0,   73,    2, 0x08 /* Private */,
       7,    4,   74,    2, 0x08 /* Private */,
       9,    0,   83,    2, 0x08 /* Private */,
      10,    0,   84,    2, 0x08 /* Private */,
      11,    1,   85,    2, 0x08 /* Private */,
      14,    0,   88,    2, 0x08 /* Private */,
      15,    0,   89,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 8, QMetaType::Int, QMetaType::Int, QMetaType::Int,    2,    2,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void QT_PCL_Segmentation::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QT_PCL_Segmentation *_t = static_cast<QT_PCL_Segmentation *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->showDemo(); break;
        case 1: _t->showPCL(); break;
        case 2: _t->onOpen(); break;
        case 3: _t->segmentation(); break;
        case 4: _t->colorByAxis(); break;
        case 5: _t->color((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZ>::Ptr(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 6: _t->drawLine(); break;
        case 7: _t->kmeans(); break;
        case 8: _t->readSkel((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 9: _t->drawSkel(); break;
        case 10: _t->reDrawSkel(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QT_PCL_Segmentation::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_QT_PCL_Segmentation.data,
      qt_meta_data_QT_PCL_Segmentation,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *QT_PCL_Segmentation::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QT_PCL_Segmentation::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QT_PCL_Segmentation.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int QT_PCL_Segmentation::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
