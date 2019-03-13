QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    src/main.cpp \
    src/armor_monocular.cpp \
    src/parameters.cpp \
    src/device.cpp \
    src/rin_porc_lib.cpp \
    src/rune_monocular.cpp \
    src/rin_videocapture.cpp

HEADERS += \
    inc/includes.h \
    inc/armor_monocular.h \
    inc/parameters.h \
    inc/device.h \
    inc/rune_monocular.h \
    inc/rin_videocapture.h \
    inc/rin_proc_lib.h

INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
               /usr/local/include/opencv2 \

LIBS += /usr/local/lib/libopencv_calib3d.so \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_dnn.so \
        /usr/local/lib/libopencv_features2d.so \
        /usr/local/lib/libopencv_flann.so \
        /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_imgcodecs.so \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_ml.so \
        /usr/local/lib/libopencv_objdetect.so \
        /usr/local/lib/libopencv_photo.so \
        /usr/local/lib/libopencv_shape.so \
        /usr/local/lib/libopencv_stitching.so \
        /usr/local/lib/libopencv_superres.so \
        /usr/local/lib/libopencv_videoio.so \
        /usr/local/lib/libopencv_video.so \
        /usr/local/lib/libopencv_videostab.so

DISTFILES += \
    data/cam_params.xml \
    data/proc_params.xml
