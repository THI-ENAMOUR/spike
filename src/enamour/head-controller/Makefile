.PHONY = all clean

CC := g++

CFLAGS := -g -Wall

LDFLAGS := -pthread -lroscpp -lrosconsole -lrostime -lroscpp_serialization

SRCS := ./src/head-controller.cpp

BINS := ./bin/head-controller

all: head-controller

head-controller:
		${CC} ${CFLAGS} -o ${BINS} ${SRCS} ${LDFLAGS}

clean:
		rm ${BINS}