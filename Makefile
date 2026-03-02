# =============================================================
#  Systolic Array SystemC Simulator — Makefile
#  Usage:
#    make                        # build (SIMPLE memory mode)
#    make MEM_BACKEND=NONE       # zero-latency mode
#    make MEM_BACKEND=RAMULATOR  # cycle-accurate DDR (needs Ramulator 2.0)
#    make run                    # build & run with default config
#    make run ARGS="--mem-mode none --config config/sim_config.yaml"
#    make wave                   # open GTKWave (if VCD generated)
#    make clean
# =============================================================

TARGET   = systolic_sc
WAVEFILE = wave.vcd

# ── Source / Object layout ────────────────────────────────────
SRCDIR = .
OBJDIR = build

SRCS = MemoryInterface.cpp \
       Scheduler.cpp       \
       sc_main.cpp

OBJS = $(addprefix $(OBJDIR)/, $(SRCS:.cpp=.o))

# ── SystemC installation path ─────────────────────────────────
# Override on command line: make SCPATH=/path/to/systemc
SCPATH = /home/songqiuyu/Workspace/systemc-2.3.3
SCLIB  = $(SCPATH)/lib-linux64

# ── Memory backend selection (NONE / SIMPLE / RAMULATOR) ──────
MEM_BACKEND ?= SIMPLE

# ── Ramulator 2.0 path (only needed for RAMULATOR mode) ───────
RAMULATOR_DIR = ../ramulator2

# ── Compiler flags ────────────────────────────────────────────
CXX      = g++
CXXFLAGS = -std=c++14 -O2 -g -Wall
CXXFLAGS += -I$(SRCDIR)                   # headers in same dir
CXXFLAGS += -I$(SCPATH)/include           # SystemC headers
CXXFLAGS += -DME_BACKEND_$(MEM_BACKEND)=1 # backend compile-time tag

# ── Linker flags ──────────────────────────────────────────────
LDFLAGS  = -L$(SCLIB) -Wl,-rpath,$(SCLIB)
LIBS     = -lsystemc -lm

# ── RAMULATOR mode: add Ramulator includes and libs ───────────
ifeq ($(MEM_BACKEND),RAMULATOR)
    CXXFLAGS += -DUSE_RAMULATOR=1
    CXXFLAGS += -I$(RAMULATOR_DIR)/src
    CXXFLAGS += -I$(RAMULATOR_DIR)/ext/spdlog/include
    CXXFLAGS += -I$(RAMULATOR_DIR)/ext/yaml-cpp/include
    CXXFLAGS += -I$(RAMULATOR_DIR)/ext/argparse/include
    LDFLAGS  += -L$(RAMULATOR_DIR)/build -Wl,-rpath,$(RAMULATOR_DIR)/build
    LIBS     += -lramulator -lyaml-cpp
endif

# ── Default target ────────────────────────────────────────────
.PHONY: all run wave clean info

all: $(OBJDIR) $(TARGET)
	@echo ""
	@echo "Build complete: $(TARGET)  [MEM_BACKEND=$(MEM_BACKEND)]"

# ── Link ──────────────────────────────────────────────────────
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(LIBS) -o $@

# ── Compile rules ─────────────────────────────────────────────
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# ── Create build directory ────────────────────────────────────
$(OBJDIR):
	mkdir -p $(OBJDIR)

# ── Run ───────────────────────────────────────────────────────
run: all
	./$(TARGET) --config config/sim_config.yaml $(ARGS)

# ── GTKWave ───────────────────────────────────────────────────
wave:
	gtkwave $(WAVEFILE)

# ── Clean ─────────────────────────────────────────────────────
clean:
	$(RM) -r $(OBJDIR) $(TARGET) $(WAVEFILE)

# ── Print config info ─────────────────────────────────────────
info:
	@echo "TARGET      = $(TARGET)"
	@echo "SCPATH      = $(SCPATH)"
	@echo "MEM_BACKEND = $(MEM_BACKEND)"
	@echo "SRCS        = $(SRCS)"
	@echo "CXXFLAGS    = $(CXXFLAGS)"
	@echo "LDFLAGS     = $(LDFLAGS)"
	@echo "LIBS        = $(LIBS)"
