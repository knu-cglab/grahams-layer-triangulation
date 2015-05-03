TEMPLATE = app
CONFIG += console
CONFIG += app_bundle
CONFIG -= qt
CONFIG += c++11

HEADERS += \
    Point2D.h \
    Defs.h \
    LayerTriangulation.h \
    Timer.h


SOURCES += \
    main.cpp \
    Point2D.cpp \
    LayerTriangulation.cpp

