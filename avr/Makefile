CXX = avr-gcc
CC = avr-gcc
CPPFLAGS = -std=c++11 -Os -fno-exceptions -mmcu=atmega644p \
           -I.. \
           -DF_CPU=1000000
LDLIBS = 

SRCS = melzi.cpp
OBJS = $(subst .cpp,.o,$(SRCS))

melzi : $(OBJS)
