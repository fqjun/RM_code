TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle

CONFIG -= qt

SOURCES += main.cpp \
#    buff_detect.cpp \
#    solve_pnp.cpp \
    detect_buff/buff_detect.cpp \
    solve_buff/solve_pnp.cpp

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv4
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_* \
        /lib/libMVSDK.so

HEADERS += \
#    buff_detect.h \
#    solve_pnp.h \
    detect_buff/buff_detect.h \
    solve_buff/solve_pnp.h \
    base.h
