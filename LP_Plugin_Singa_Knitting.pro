QT += gui widgets

TEMPLATE = lib
DEFINES += LP_PLUGIN_SINGA_KNITTING_LIBRARY _USE_MATH_DEFINES

CONFIG += c++17

QMAKE_POST_LINK=$(MAKE) install

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    lp_plugin_singa_knitting.cpp

HEADERS += \
    LP_Plugin_Singa_Knitting_global.h \
    lp_plugin_singa_knitting.h

# Default rules for deployment.
target.path = $$OUT_PWD/../App/plugins/$$TARGET
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../Functional/release/ -lFunctional
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../Functional/debug/ -lFunctional
else:unix:!macx: LIBS += -L$$OUT_PWD/../Functional/ -lFunctional

INCLUDEPATH += $$PWD/../Functional
DEPENDPATH += $$PWD/../Functional

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../OpenMesh/lib/ -lOpenMeshCore
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../OpenMesh/lib/ -lOpenMeshCored
else:unix:!macx: LIBS += -L$$PWD/../../OpenMesh/lib/ -lOpenMeshCore

INCLUDEPATH += $$PWD/../../OpenMesh/include
DEPENDPATH += $$PWD/../../OpenMesh/include

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../Model/release/ -lModel
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../Model/debug/ -lModel
else:unix:!macx: LIBS += -L$$OUT_PWD/../Model/ -lModel

INCLUDEPATH += $$PWD/../Model
DEPENDPATH += $$PWD/../Model
