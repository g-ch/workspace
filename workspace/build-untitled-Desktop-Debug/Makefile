#############################################################################
# Makefile for building: untitled
# Generated by qmake (2.01a) (Qt 4.8.7) on: ?? 2? 29 20:32:30 2016
# Project:  ../OpenAndDecodeGPSTrackFiles/untitled.pro
# Template: app
# Command: /usr/local/Trolltech/Qt-4.8.7/bin/qmake -spec /usr/local/Trolltech/Qt-4.8.7/mkspecs/linux-g++ CONFIG+=debug -o Makefile ../OpenAndDecodeGPSTrackFiles/untitled.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED
CFLAGS        = -pipe -g -Wall -W -D_REENTRANT $(DEFINES)
CXXFLAGS      = -pipe -g -Wall -W -D_REENTRANT $(DEFINES)
INCPATH       = -I/usr/local/Trolltech/Qt-4.8.7/mkspecs/linux-g++ -I../OpenAndDecodeGPSTrackFiles -I/usr/local/Trolltech/Qt-4.8.7/include/QtCore -I/usr/local/Trolltech/Qt-4.8.7/include/QtGui -I/usr/local/Trolltech/Qt-4.8.7/include -I. -I. -I../OpenAndDecodeGPSTrackFiles -I.
LINK          = g++
LFLAGS        = -Wl,-rpath,/usr/local/Trolltech/Qt-4.8.7/lib
LIBS          = $(SUBLIBS)  -L/usr/local/Trolltech/Qt-4.8.7/lib -lQtGui -L/usr/local/Trolltech/Qt-4.8.7/lib -L/usr/X11R6/lib -lQtCore -lpthread 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/local/Trolltech/Qt-4.8.7/bin/qmake
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = ../OpenAndDecodeGPSTrackFiles/main.cpp \
		../OpenAndDecodeGPSTrackFiles/mainwindow.cpp moc_mainwindow.cpp
OBJECTS       = main.o \
		mainwindow.o \
		moc_mainwindow.o
DIST          = /usr/local/Trolltech/Qt-4.8.7/mkspecs/common/unix.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/linux.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/gcc-base.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/gcc-base-unix.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/g++-base.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/g++-unix.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/qconfig.pri \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/modules/qt_webkit_version.pri \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/qt_functions.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/qt_config.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/exclusive_builds.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/default_pre.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/debug.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/default_post.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/shared.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/warn_on.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/qt.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/unix/thread.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/moc.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/resources.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/uic.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/yacc.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/lex.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/include_source_dir.prf \
		../OpenAndDecodeGPSTrackFiles/untitled.pro
QMAKE_TARGET  = untitled
DESTDIR       = 
TARGET        = untitled

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET): ui_mainwindow.h $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)
	{ test -n "$(DESTDIR)" && DESTDIR="$(DESTDIR)" || DESTDIR=.; } && test $$(gdb --version | sed -e 's,[^0-9][^0-9]*\([0-9]\)\.\([0-9]\).*,\1\2,;q') -gt 72 && gdb --nx --batch --quiet -ex 'set confirm off' -ex "save gdb-index $$DESTDIR" -ex quit '$(TARGET)' && test -f $(TARGET).gdb-index && objcopy --add-section '.gdb_index=$(TARGET).gdb-index' --set-section-flags '.gdb_index=readonly' '$(TARGET)' '$(TARGET)' && rm -f $(TARGET).gdb-index || true

Makefile: ../OpenAndDecodeGPSTrackFiles/untitled.pro  /usr/local/Trolltech/Qt-4.8.7/mkspecs/linux-g++/qmake.conf /usr/local/Trolltech/Qt-4.8.7/mkspecs/common/unix.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/linux.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/gcc-base.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/gcc-base-unix.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/g++-base.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/g++-unix.conf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/qconfig.pri \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/modules/qt_webkit_version.pri \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/qt_functions.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/qt_config.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/exclusive_builds.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/default_pre.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/debug.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/default_post.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/shared.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/warn_on.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/qt.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/unix/thread.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/moc.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/resources.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/uic.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/yacc.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/lex.prf \
		/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/include_source_dir.prf \
		/usr/local/Trolltech/Qt-4.8.7/lib/libQtGui.prl \
		/usr/local/Trolltech/Qt-4.8.7/lib/libQtCore.prl
	$(QMAKE) -spec /usr/local/Trolltech/Qt-4.8.7/mkspecs/linux-g++ CONFIG+=debug -o Makefile ../OpenAndDecodeGPSTrackFiles/untitled.pro
/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/unix.conf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/linux.conf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/gcc-base.conf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/gcc-base-unix.conf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/g++-base.conf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/common/g++-unix.conf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/qconfig.pri:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/modules/qt_webkit_version.pri:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/qt_functions.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/qt_config.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/exclusive_builds.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/default_pre.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/debug.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/default_post.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/shared.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/unix/gdb_dwarf_index.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/warn_on.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/qt.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/unix/thread.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/moc.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/resources.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/uic.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/yacc.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/lex.prf:
/usr/local/Trolltech/Qt-4.8.7/mkspecs/features/include_source_dir.prf:
/usr/local/Trolltech/Qt-4.8.7/lib/libQtGui.prl:
/usr/local/Trolltech/Qt-4.8.7/lib/libQtCore.prl:
qmake:  FORCE
	@$(QMAKE) -spec /usr/local/Trolltech/Qt-4.8.7/mkspecs/linux-g++ CONFIG+=debug -o Makefile ../OpenAndDecodeGPSTrackFiles/untitled.pro

dist: 
	@$(CHK_DIR_EXISTS) .tmp/untitled1.0.0 || $(MKDIR) .tmp/untitled1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) .tmp/untitled1.0.0/ && $(COPY_FILE) --parents ../OpenAndDecodeGPSTrackFiles/mainwindow.h .tmp/untitled1.0.0/ && $(COPY_FILE) --parents ../OpenAndDecodeGPSTrackFiles/main.cpp ../OpenAndDecodeGPSTrackFiles/mainwindow.cpp .tmp/untitled1.0.0/ && $(COPY_FILE) --parents ../OpenAndDecodeGPSTrackFiles/mainwindow.ui .tmp/untitled1.0.0/ && (cd `dirname .tmp/untitled1.0.0` && $(TAR) untitled1.0.0.tar untitled1.0.0 && $(COMPRESS) untitled1.0.0.tar) && $(MOVE) `dirname .tmp/untitled1.0.0`/untitled1.0.0.tar.gz . && $(DEL_FILE) -r .tmp/untitled1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


check: first

mocclean: compiler_moc_header_clean compiler_moc_source_clean

mocables: compiler_moc_header_make_all compiler_moc_source_make_all

compiler_moc_header_make_all: moc_mainwindow.cpp
compiler_moc_header_clean:
	-$(DEL_FILE) moc_mainwindow.cpp
moc_mainwindow.cpp: ../OpenAndDecodeGPSTrackFiles/mainwindow.h
	/usr/local/Trolltech/Qt-4.8.7/bin/moc $(DEFINES) $(INCPATH) ../OpenAndDecodeGPSTrackFiles/mainwindow.h -o moc_mainwindow.cpp

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_moc_source_make_all:
compiler_moc_source_clean:
compiler_uic_make_all: ui_mainwindow.h
compiler_uic_clean:
	-$(DEL_FILE) ui_mainwindow.h
ui_mainwindow.h: ../OpenAndDecodeGPSTrackFiles/mainwindow.ui
	/usr/local/Trolltech/Qt-4.8.7/bin/uic ../OpenAndDecodeGPSTrackFiles/mainwindow.ui -o ui_mainwindow.h

compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: compiler_moc_header_clean compiler_uic_clean 

####### Compile

main.o: ../OpenAndDecodeGPSTrackFiles/main.cpp ../OpenAndDecodeGPSTrackFiles/mainwindow.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o ../OpenAndDecodeGPSTrackFiles/main.cpp

mainwindow.o: ../OpenAndDecodeGPSTrackFiles/mainwindow.cpp ../OpenAndDecodeGPSTrackFiles/mainwindow.h \
		ui_mainwindow.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o mainwindow.o ../OpenAndDecodeGPSTrackFiles/mainwindow.cpp

moc_mainwindow.o: moc_mainwindow.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_mainwindow.o moc_mainwindow.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

