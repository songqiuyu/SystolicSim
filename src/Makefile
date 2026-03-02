# Compiler settings
CXX      = g++
CXXFLAGS = -std=c++14 -Wall -Wno-unused-variable -O2 -g

# SystemC paths - modify if installed elsewhere
# Usually systemc is at /usr/local/systemc-2.3.3 or similar
SYSTEMC_HOME ?= /home/songqiuyu/Workspace/systemc-2.3.3

# Flags and Includes
INCLUDES = -I. -Icore -Isystolic_array -I$(SYSTEMC_HOME)/include
# Try both lib-linux64 and lib depending on the system config
LDFLAGS  = -L$(SYSTEMC_HOME)/lib-linux64 -L$(SYSTEMC_HOME)/lib -lsystemc

# Additional RPATH so we don't need LD_LIBRARY_PATH
LDFLAGS += -Wl,-rpath=$(SYSTEMC_HOME)/lib-linux64 -Wl,-rpath=$(SYSTEMC_HOME)/lib

TARGET = systolic_sim
SRCS   = main.cpp
OBJS   = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET) wave.vcd

run: $(TARGET)
	./$(TARGET)

wave:
	gtkwave wave.vcd

.PHONY: all clean run
