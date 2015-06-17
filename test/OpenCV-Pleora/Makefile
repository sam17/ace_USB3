SRC_CPPS = \
           ace_usb3.cpp \
	   main.cpp \

EXEC     = ace_usb3

ifeq ($(SRC_CS) $(SRC_CPPS),)
  $(error No source files specified)
endif

ifeq ($(EXEC),)
  $(error No executable file specified)
endif

CC                  ?= gcc
CXX                 ?= g++

PUREGEV_ROOT        ?= /opt/pleora/ebus_sdk/linux-gnueabihf-arm/
PV_LIBRARY_PATH      =$(PUREGEV_ROOT)/lib

CFLAGS              += -I$(PUREGEV_ROOT)/include -I.
CPPFLAGS            += -g -I$(PUREGEV_ROOT)/include -I/usr/local/include/opencv2 -I. 
ifdef _DEBUG
    CFLAGS    += -g -D_DEBUG
    CPPFLAGS  += -g -D_DEBUG
else
    CFLAGS    += -O3
    CPPFLAGS  += -O3
endif
CFLAGS    += -D_UNIX_ -D_LINUX_
CPPFLAGS  += -D_UNIX_ -D_LINUX_

LDFLAGS             += -L$(PUREGEV_ROOT)/lib         \
                        -lPvBase                     \
                        -lPvDevice                   \
                        -lPvBuffer                   \
                        -lPvGenICam                  \
                        -lPvStream                   \
                        -lPvTransmitter              \
                        -lPvVirtualDevice	         \
			            -lPvAppUtils                 \
                        -lPvPersistence              \
                        -lPvSerial

LDFLAGS      += -Wl,-rpath=$(PUREGEV_ROOT)/lib

# Configure Genicam
GEN_LIB_PATH = $(PUREGEV_ROOT)/lib/genicam/bin/Linux32_ARM
LDFLAGS      += -L$(GEN_LIB_PATH)
LDFLAGS      += -Wl,-rpath=$(GEN_LIB_PATH)

#Configure OpenCV
LIBS			:= -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_objdetect


LD_LIBRARY_PATH       = $(PV_LIBRARY_PATH):$(QT_LIBRARY_PATH):$(GEN_LIB_PATH)
export LD_LIBRARY_PATH

OBJS      += $(SRC_CPPS:%.cpp=%.o)
OBJS      += $(SRC_CS:%.c=%.o)

all: $(EXEC)

clean:
	rm -rf $(OBJS) $(EXEC) $(SRC_MOC) $(SRC_QRC)

moc_%.cxx: %.h
	$(MOC) $< -o $@ 

qrc_%.cxx: %.qrc
	$(RCC) $< -o $@

%.o: %.cxx
	$(CXX) -c $(CPPFLAGS) -o $@ $<

%.o: %.cpp
	$(CXX) -c $(CPPFLAGS) -o $@ $<

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

$(EXEC): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(LIBS)

.PHONY: all clean

