CC=g++
CFLAGS=-c -w
#CFLAGS=
LDFLAGS= 
SOURCES=I2Cdev.cpp MPU6050.cpp PID.cpp controls.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=controls

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS)  -lrt -lpthread /usr/local/lib/librf24.so.1.1.6 -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
