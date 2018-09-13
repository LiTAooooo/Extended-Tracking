#
#  Makefile
#  libEden
#
#  This file is part of The Eden Library.
#
#  The Eden Library is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  The Eden Library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with The Eden Library.  If not, see <http:#www.gnu.org/licenses/>.
#
#  As a special exception, the copyright holders of this library give you
#  permission to link this library with independent modules to produce an
#  executable, regardless of the license terms of these independent modules, and to
#  copy and distribute the resulting executable under terms of your choice,
#  provided that you also meet, for each linked independent module, the terms and
#  conditions of the license of that module. An independent module is a module
#  which is neither derived from nor based on this library. If you modify this
#  library, you may extend this exception to your version of the library, but you
#  are not obligated to do so. If you do not wish to do so, delete this exception
#  statement from your version.
#
#  Copyright 2001-2015 Philip Lamb
#
#  Author(s): Philip Lamb
#

UNAME = $(shell uname)

AR_HOME = ../../..

CC=gcc
CXX=g++ -std=c++11
CPPFLAGS = -I$(AR_HOME)/include/linux-x86_64 -I$(AR_HOME)/include
CFLAGS = -O3 -fPIC -march=core2 -DHAVE_NFT=1 -I/usr/include/x86_64-linux-gnu -pthread -I/usr/include/gstreamer-0.10 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/libxml2
CXXFLAGS = -O3 -fPIC -march=core2 -DHAVE_NFT=1 -I/usr/include/x86_64-linux-gnu -pthread -I/usr/include/gstreamer-0.10 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/libxml2
LDFLAGS = -L$(AR_HOME)/lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib64
LIBS = -lglut -lGLU -lGL -lX11 -lm -lpthread -ljpeg -lgstreamer-0.10 -lgobject-2.0 -lgmodule-2.0 -pthread -lgthread-2.0 -pthread -lglib-2.0 -lxml2 -ldc1394 -lraw1394
AR=ar
ARFLAGS=rs
RANLIB = true

TARGET = $(AR_HOME)/lib/libEden.a

HEADERS = \
	$(AR_HOME)/include/Eden/Eden.h \
	$(AR_HOME)/include/Eden/EdenError.h \
	$(AR_HOME)/include/Eden/EdenGLFont.h \
	$(AR_HOME)/include/Eden/EdenMath.h \
	$(AR_HOME)/include/Eden/EdenMessage.h \
	$(AR_HOME)/include/Eden/EdenSound.h \
	$(AR_HOME)/include/Eden/EdenSurfaces.h \
	$(AR_HOME)/include/Eden/EdenTime.h \
	$(AR_HOME)/include/Eden/EdenUtil.h \
	$(AR_HOME)/include/Eden/EdenUtil.h \
	$(AR_HOME)/include/Eden/glm.h \
	$(AR_HOME)/include/Eden/readtex.h \
	ALFramework.h \
	CWaves.h \


OBJS = \
	ALFramework.o \
	CWaves.o \
	EdenError.o \
	EdenGLFont.o \
	EdenMath.o \
	EdenMessage.o \
	EdenSound.o \
	EdenSurfaces.o \
	EdenTime.o \
	EdenUtil.o \
	glm.o \
	readtex.o \


# Implicit rule, to compile Objective-C files with the .m suffix.
#%.o : %.m
#	$(CC) -c $(CPPFLAGS) $(CFLAGS) $< -o $@

default build all: $(TARGET)

$(OBJS) : $(HEADERS)

$(TARGET): $(OBJS)
	$(AR) $(ARFLAGS) $@ $?
	$(RANLIB) $@

clean:
	-rm -f $(OBJS)
	-rm -f $(TARGET)

allclean:
	-rm -f $(OBJS)
	-rm -f $(TARGET)
	-rm -f Makefile

distclean:
	rm -f $(OBJS)
	rm -f Makefile

