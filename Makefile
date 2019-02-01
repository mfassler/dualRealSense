## Build Options:
NETWORK_DISPLAY = 0
QR_CODES = 0


######
######   What are we building?
######

TARGET = dualMe


# Objects that must be built in order to link

OBJECTS = main.o
OBJECTS += UdpJpegSenderThread.o


######
######   Binaries and flags
######

CPPFLAGS = -std=c++11
#CPPFLAGS += -O3
CPPFLAGS += -g

ifeq ($(NETWORK_DISPLAY), 1)
CPPFLAGS += -DUSE_NETWORK_DISPLAY
endif


LD = g++

#LDFLAGS += -L/usr/local/lib/
LDFLAGS += -pthread
LDLIBS += -lrealsense2
LDLIBS += $(shell pkg-config --libs opencv)

ifeq ($(QR_CODES), 1)
CPPFLAGS += -DUSE_ZBAR
LDLIBS += -lzbar
endif


# Default target:
.PHONY: all
all: $(TARGET)


$(TARGET): $(OBJECTS)
	$(LD) $(LDFLAGS) $(OBJECTS) -o $@ $(LDLIBS)


.PHONY: clean
clean:
	rm -f $(OBJECTS)
	rm -f $(TARGET)


