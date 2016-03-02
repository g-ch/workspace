/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../OpenAndDecodeGPSTrackFiles/mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      36,   11,   11,   11, 0x08,
      64,   11,   11,   11, 0x08,
      90,   11,   11,   11, 0x08,
     116,   11,   11,   11, 0x08,
     144,  142,   11,   11, 0x08,
     167,   11,  162,   11, 0x08,
     183,   11,   11,   11, 0x08,
     200,   11,   11,   11, 0x08,
     225,  213,   11,   11, 0x08,
     279,  267,   11,   11, 0x08,
     321,   11,   11,   11, 0x08,
     356,  344,  338,   11, 0x08,
     400,  392,  338,   11, 0x08,
     441,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0on_pushButton_clicked()\0"
    "on_listWidget_itemClicked()\0"
    "on_pushButton_2_clicked()\0"
    "on_pushButton_3_clicked()\0"
    "on_pushButton_4_clicked()\0x\0"
    "delete_point(int)\0bool\0restore_point()\0"
    "draw_gps_fence()\0draw_route()\0lat,lon,x,y\0"
    "gps_to_local(double,double,float*,float*)\0"
    "x,y,lat,lon\0local_to_gps(float,float,double*,double*)\0"
    "turn_point_cal()\0float\0x1,y1,x2,y2\0"
    "point_dist(float,float,float,float)\0"
    "m,n,k,b\0point_line_dist(float,float,float,float)\0"
    "on_pushButton_5_clicked()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_pushButton_clicked(); break;
        case 1: _t->on_listWidget_itemClicked(); break;
        case 2: _t->on_pushButton_2_clicked(); break;
        case 3: _t->on_pushButton_3_clicked(); break;
        case 4: _t->on_pushButton_4_clicked(); break;
        case 5: _t->delete_point((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: { bool _r = _t->restore_point();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 7: _t->draw_gps_fence(); break;
        case 8: _t->draw_route(); break;
        case 9: _t->gps_to_local((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< float*(*)>(_a[3])),(*reinterpret_cast< float*(*)>(_a[4]))); break;
        case 10: _t->local_to_gps((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< double*(*)>(_a[3])),(*reinterpret_cast< double*(*)>(_a[4]))); break;
        case 11: _t->turn_point_cal(); break;
        case 12: { float _r = _t->point_dist((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = _r; }  break;
        case 13: { float _r = _t->point_line_dist((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = _r; }  break;
        case 14: _t->on_pushButton_5_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
