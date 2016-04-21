CC=g++
CFLAGS=-c -w
#CFLAGS=
LDFLAGS= 
SOURCES=I2Cdev.cpp MPU6050.cpp controls.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=controls

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS)  -lrt -lpthread -o $@ 

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
