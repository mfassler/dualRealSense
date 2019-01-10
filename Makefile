######
######   What are we building?
######

TARGET = dualMe


# Objects that must be built in order to link

OBJECTS = main.o
OBJECTS += UdpSender.o

######
######   Binaries and flags
######

CPPFLAGS = -std=c++11
#CPPFLAGS += -O3
CPPFLAGS += -g

LD = g++

LDFLAGS = -pthread
#LDFLAGS += -L/usr/local/lib/
LDLIBS += -lrealsense2
LDLIBS += $(shell pkg-config --libs opencv)


# Default target:
.PHONY: all
all: $(TARGET)


$(TARGET): $(OBJECTS)
	$(LD) $(LDFLAGS) $(OBJECTS) -o $@ $(LDLIBS)


.PHONY: clean
clean:
	rm -f $(OBJECTS)
	rm -f $(TARGET)


