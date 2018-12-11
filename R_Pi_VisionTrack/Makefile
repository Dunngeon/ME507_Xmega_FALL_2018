# THis is a makefile for testing
CC = g++


CFLAGS = -Wall -std=c99 -I /usr/local/include -L /usr/local/lib -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc

all: Vision.o
	$(CC) Vision.cpp -o Vision -I /usr/local/include -L /usr/local/lib -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc -lrt

clean:
	rm -f Vision Vision.o *~
