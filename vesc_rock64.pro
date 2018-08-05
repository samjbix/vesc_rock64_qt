TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
        VescUart.cpp \
        buffer.cpp \
        crc.cpp \
        server.cpp

HEADERS += \
        VescUart.h \
        buffer.h \
        crc.h \
        datatypes.h \
        local_datatypes.h \
    server.h

DISTFILES += \
    Makefile
