/****************************************************************************
** Meta object code from reading C++ file 'teleop_panel.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/aerostack2/as2_user_interfaces/as2_visualization/as2_rviz_plugins/src/teleop_panel.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'teleop_panel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_as2_rviz_plugins__TeleopPanel_t {
    QByteArrayData data[10];
    char stringdata0[154];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_as2_rviz_plugins__TeleopPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_as2_rviz_plugins__TeleopPanel_t qt_meta_stringdata_as2_rviz_plugins__TeleopPanel = {
    {
QT_MOC_LITERAL(0, 0, 29), // "as2_rviz_plugins::TeleopPanel"
QT_MOC_LITERAL(1, 30, 7), // "takeoff"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 4), // "land"
QT_MOC_LITERAL(4, 44, 6), // "disarm"
QT_MOC_LITERAL(5, 51, 70), // "rclcpp_action::ClientGoalHand..."
QT_MOC_LITERAL(6, 122, 6), // "result"
QT_MOC_LITERAL(7, 129, 5), // "hover"
QT_MOC_LITERAL(8, 135, 4), // "kill"
QT_MOC_LITERAL(9, 140, 13) // "updateDroneNs"

    },
    "as2_rviz_plugins::TeleopPanel\0takeoff\0"
    "\0land\0disarm\0"
    "rclcpp_action::ClientGoalHandle<as2_msgs::action::Land>::WrappedResult\0"
    "result\0hover\0kill\0updateDroneNs"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_as2_rviz_plugins__TeleopPanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x0a /* Public */,
       3,    0,   45,    2, 0x0a /* Public */,
       4,    1,   46,    2, 0x0a /* Public */,
       7,    0,   49,    2, 0x0a /* Public */,
       8,    0,   50,    2, 0x0a /* Public */,
       9,    0,   51,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void as2_rviz_plugins::TeleopPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<TeleopPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->takeoff(); break;
        case 1: _t->land(); break;
        case 2: _t->disarm((*reinterpret_cast< const rclcpp_action::ClientGoalHandle<as2_msgs::action::Land>::WrappedResult(*)>(_a[1]))); break;
        case 3: _t->hover(); break;
        case 4: _t->kill(); break;
        case 5: _t->updateDroneNs(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject as2_rviz_plugins::TeleopPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Panel::staticMetaObject>(),
    qt_meta_stringdata_as2_rviz_plugins__TeleopPanel.data,
    qt_meta_data_as2_rviz_plugins__TeleopPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *as2_rviz_plugins::TeleopPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *as2_rviz_plugins::TeleopPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_as2_rviz_plugins__TeleopPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Panel::qt_metacast(_clname);
}

int as2_rviz_plugins::TeleopPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
