CC = g++

CFLAGS = -w -Wall -O2
SRCS = Car_Contorl_I2C.cpp -lwiringPi

PROG = test

OPENCV = `pkg-config --cflags --libs opencv`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)
	
	$(./PROG)

