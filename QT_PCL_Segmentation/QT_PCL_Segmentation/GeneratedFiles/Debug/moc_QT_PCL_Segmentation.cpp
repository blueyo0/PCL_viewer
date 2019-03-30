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
    QByteArrayData data[21];
    char stringdata0[234];
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
QT_MOC_LITERAL(11, 128, 9), // "skelParam"
QT_MOC_LITERAL(12, 138, 11), // "std::string"
QT_MOC_LITERAL(13, 150, 6), // "params"
QT_MOC_LITERAL(14, 157, 4), // "mode"
QT_MOC_LITERAL(15, 162, 8), // "drawSkel"
QT_MOC_LITERAL(16, 171, 10), // "reDrawSkel"
QT_MOC_LITERAL(17, 182, 9), // "BayesSkel"
QT_MOC_LITERAL(18, 192, 15), // "clearPointCloud"
QT_MOC_LITERAL(19, 208, 15), // "resetPointCloud"
QT_MOC_LITERAL(20, 224, 9) // "KNNsmooth"

    },
    "QT_PCL_Segmentation\0showDemo\0\0showPCL\0"
    "onOpen\0segmentation\0colorByAxis\0color\0"
    "pcl::PointCloud<pcl::PointXYZ>::Ptr\0"
    "drawLine\0kmeans\0skelParam\0std::string\0"
    "params\0mode\0drawSkel\0reDrawSkel\0"
    "BayesSkel\0clearPointCloud\0resetPointCloud\0"
    "KNNsmooth"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QT_PCL_Segmentation[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x08 /* Private */,
       3,    0,   95,    2, 0x08 /* Private */,
       4,    0,   96,    2, 0x08 /* Private */,
       5,    0,   97,    2, 0x08 /* Private */,
       6,    0,   98,    2, 0x08 /* Private */,
       7,    4,   99,    2, 0x08 /* Private */,
       9,    0,  108,    2, 0x08 /* Private */,
      10,    0,  109,    2, 0x08 /* Private */,
      11,    2,  110,    2, 0x08 /* Private */,
      11,    1,  115,    2, 0x28 /* Private | MethodCloned */,
      15,    0,  118,    2, 0x08 /* Private */,
      16,    0,  119,    2, 0x08 /* Private */,
      17,    0,  120,    2, 0x08 /* Private */,
      18,    0,  121,    2, 0x08 /* Private */,
      19,    0,  122,    2, 0x08 /* Private */,
      20,    0,  123,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 8, QMetaType::Int, QMetaType::Int, QMetaType::Int,    2,    2,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Bool, 0x80000000 | 12, QMetaType::Int,   13,   14,
    QMetaType::Bool, 0x80000000 | 12,   13,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
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
        case 8: { bool _r = _t->skelParam((*reinterpret_cast< std::string(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 9: { bool _r = _t->skelParam((*reinterpret_cast< std::string(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 10: _t->drawSkel(); break;
        case 11: _t->reDrawSkel(); break;
        case 12: _t->BayesSkel(); break;
        case 13: _t->clearPointCloud(); break;
        case 14: _t->resetPointCloud(); break;
        case 15: _t->KNNsmooth(); break;
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
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
