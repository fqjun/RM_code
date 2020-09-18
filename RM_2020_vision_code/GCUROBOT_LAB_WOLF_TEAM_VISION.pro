TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
DEFINES += _GLIBCXX_USE_CXX11_ABI=0


INCLUDEPATH +=  /usr/local/include \
                /usr/local/include/opencv4 \
                /usr/local/include/opencv4/opencv2 \
#                /home/rcxxx/workspace/tensorflow/ \
#                /home/rcxxx/workspace/tensorflow/bazel-genfiles/ \
#                /home/rcxxx/workspace/tensorflow/tensorflow/contrib/makefile/downloads/ \
#                /home/rcxxx/workspace/tensorflow/tensorflow/contrib/makefile/downloads/eigen/ \
#                /home/rcxxx/workspace/tensorflow/tensorflow/contrib/makefile/downloads/absl/ \
#                /home/rcxxx/workspace/tensorflow/tensorflow/contrib/makefile/downloads/protobuf/ \
#                /home/rcxxx/workspace/tensorflow/tensorflow/contrib/makefile/downloads/protobuf/src/ \

LIBS += /usr/local/lib/libopencv_* \
#        /usr/local/lib/libtensorflow_cc.so \
        /home/gcu/workspace/RM_2020_vision_code/lib/libMVSDK.so \
#        ../GCUROBOT_LAB_WOLF_TEAM_VISION/lib/libMVSDK.so \
#        `pkg-config --cflags --libs protobuf` \

        /usr/local/lib/libopencv_imgcodecs.so.4.1

SOURCES += \
        main.cpp \
        armor/rm_armorfitted.cpp \
        camera/rm_videocapture.cpp \
        control/rm_link.cpp \
        filter/rm_kalmanfilter.cpp \
        serial/serialport.cpp \
        solve_PNP/solve_pnp.cpp

HEADERS += \
        configure.h \
        armor/rm_armorfitted.h \
        camera/rm_videocapture.h \
        control/debug_control.h \
        control/rm_link.h \
        filter/rm_kalmanfilter.h \
        serial/serialport.h \
        solve_PNP/solve_pnp.h
