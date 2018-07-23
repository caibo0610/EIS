CC  ?= gcc
CXX ?= g++

CXXFLAGS+= -g -Wall -Werror -mfpu=neon -std=c++11 -O3 -lOpenCL
#CXXFLAGS+= --std=c++11 -lOpenCL

all:libyuvresize.so

libyuvresize.so: yuvresize.cpp TimeMeasure.cpp ionBuffManager.cpp
	$(CXX) -o $@ $^ $(CXXFLAGS) -L. -shared -fPIC

clean:
	rm -f *.so *.o
