/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../CAN/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[21];
    char stringdata0[348];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 16), // "canRecvedCANData"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 18), // "ZCAN_Receive_Data*"
QT_MOC_LITERAL(4, 48, 11), // "recvCANData"
QT_MOC_LITERAL(5, 60, 10), // "frameCount"
QT_MOC_LITERAL(6, 71, 7), // "channel"
QT_MOC_LITERAL(7, 79, 18), // "canRecvedCANFDData"
QT_MOC_LITERAL(8, 98, 20), // "ZCAN_ReceiveFD_Data*"
QT_MOC_LITERAL(9, 119, 13), // "recvCANFDData"
QT_MOC_LITERAL(10, 133, 23), // "on_cleanListBtn_clicked"
QT_MOC_LITERAL(11, 157, 24), // "on_openDeviceBtn_clicked"
QT_MOC_LITERAL(12, 182, 25), // "on_closeDeviceBtn_clicked"
QT_MOC_LITERAL(13, 208, 21), // "on_initCANBtn_clicked"
QT_MOC_LITERAL(14, 230, 22), // "on_StartCANBtn_clicked"
QT_MOC_LITERAL(15, 253, 10), // "afterReSet"
QT_MOC_LITERAL(16, 264, 22), // "on_reSetCANBtn_clicked"
QT_MOC_LITERAL(17, 287, 18), // "on_sendBtn_clicked"
QT_MOC_LITERAL(18, 306, 13), // "AddDataToList"
QT_MOC_LITERAL(19, 320, 7), // "strList"
QT_MOC_LITERAL(20, 328, 19) // "on_checkBox_clicked"

    },
    "MainWindow\0canRecvedCANData\0\0"
    "ZCAN_Receive_Data*\0recvCANData\0"
    "frameCount\0channel\0canRecvedCANFDData\0"
    "ZCAN_ReceiveFD_Data*\0recvCANFDData\0"
    "on_cleanListBtn_clicked\0"
    "on_openDeviceBtn_clicked\0"
    "on_closeDeviceBtn_clicked\0"
    "on_initCANBtn_clicked\0on_StartCANBtn_clicked\0"
    "afterReSet\0on_reSetCANBtn_clicked\0"
    "on_sendBtn_clicked\0AddDataToList\0"
    "strList\0on_checkBox_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    3,   74,    2, 0x08 /* Private */,
       7,    3,   81,    2, 0x08 /* Private */,
      10,    0,   88,    2, 0x08 /* Private */,
      11,    0,   89,    2, 0x08 /* Private */,
      12,    0,   90,    2, 0x08 /* Private */,
      13,    0,   91,    2, 0x08 /* Private */,
      14,    0,   92,    2, 0x08 /* Private */,
      15,    0,   93,    2, 0x08 /* Private */,
      16,    0,   94,    2, 0x08 /* Private */,
      17,    0,   95,    2, 0x08 /* Private */,
      18,    1,   96,    2, 0x08 /* Private */,
      20,    0,   99,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::UInt, QMetaType::UInt,    4,    5,    6,
    QMetaType::Void, 0x80000000 | 8, QMetaType::UInt, QMetaType::UInt,    9,    5,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Bool,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QStringList,   19,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->canRecvedCANData((*reinterpret_cast< ZCAN_Receive_Data*(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3]))); break;
        case 1: _t->canRecvedCANFDData((*reinterpret_cast< ZCAN_ReceiveFD_Data*(*)>(_a[1])),(*reinterpret_cast< uint(*)>(_a[2])),(*reinterpret_cast< uint(*)>(_a[3]))); break;
        case 2: _t->on_cleanListBtn_clicked(); break;
        case 3: _t->on_openDeviceBtn_clicked(); break;
        case 4: _t->on_closeDeviceBtn_clicked(); break;
        case 5: _t->on_initCANBtn_clicked(); break;
        case 6: _t->on_StartCANBtn_clicked(); break;
        case 7: { bool _r = _t->afterReSet();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 8: _t->on_reSetCANBtn_clicked(); break;
        case 9: _t->on_sendBtn_clicked(); break;
        case 10: _t->AddDataToList((*reinterpret_cast< QStringList(*)>(_a[1]))); break;
        case 11: _t->on_checkBox_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
