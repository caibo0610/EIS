# Simple Makefile to compile hello.c. 
# Needed because hello.c can't actually be cross-compiled from the command
# line. The Makefile takes advantage of the environment variables set up by the
# environment-setup-<target> script to correctly cross-compile.

CXX ?= g++
CC  ?= gcc

TARGET = libeis.so

INCLUDES = -I./CalcM/inc -I. -I../eis_resize -isystem/home/by/zero/qrlSDK/sdkinstall/sysroots/eagle8074/usr/src/kernel/include
CXXFLAGS+= -Wall -pthread  -std=c++11 -O2 -fopenmp -mfpu=neon -march=armv7-a -DCV_CPU_HAS_SUPPORT_NEON=1

SRCDIRS = ./ ./CalcM/src
SOURCES = $(shell find $(SRCDIRS) -name "*.cpp" | sed 's/^\.\///g')
LIBS = -lpthread -ldl -fopenmp -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc 
LIBS += -lopencv_video -lopencv_objdetect -lopencv_photo -lopencv_videoio -lopencv_calib3d -lopencv_features2d -lOpenCL -lyuvresize
LFLAGS = -L. -L../eis_resize

OBJ_DIR = obj
OBJS += $(addprefix $(OBJ_DIR)/,$(SOURCES:.cpp=.o))

.PHONY : all
all : $(TARGET)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -pv $(dir $@)
	@echo "[cc] $< ..."
	@$(CXX) -c $< -o $@ $(CXXFLAGS) $(LIBS) $(INCLUDES) -fPIC


$(TARGET): $(OBJS)
	@echo "Linking..."
	@$(CXX) $^ -fPIC -shared -o $@  -g $(CXXFLAGS) $(INCLUDES) $(LFLAGS) $(LIBS)
	@echo "done"

clean:
	@rm -rvf $(OBJ_DIR)

