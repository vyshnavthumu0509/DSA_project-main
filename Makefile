
CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2 -g

# --- Phase 1 Definitions ---
PHASE1_DIR = Phase-1
PHASE1_INC = -I./Phase-1
PHASE1_SRC = $(PHASE1_DIR)/main.cpp \
             $(PHASE1_DIR)/Graph.cpp \
             $(PHASE1_DIR)/Algorithm.cpp
PHASE1_OBJ = $(PHASE1_SRC:.cpp=.o)
PHASE1_TARGET = phase1

# --- Phase 2 Definitions ---
PHASE2_DIR = Phase-2
PHASE2_INC = -I./Phase-2
PHASE2_SRC = $(PHASE2_DIR)/main.cpp \
             $(PHASE2_DIR)/Graph.cpp \
             $(PHASE2_DIR)/Algorithm.cpp
PHASE2_OBJ = $(PHASE2_SRC:.cpp=.o)
PHASE2_TARGET = phase2

# --- Phase 3 Definitions ---
PHASE3_DIR = Phase-3
PHASE3_INC = -I./Phase-3
PHASE3_SRC = $(PHASE3_DIR)/main.cpp \
             $(PHASE3_DIR)/Graph.cpp \
             $(PHASE3_DIR)/Algorithm.cpp
PHASE3_OBJ = $(PHASE3_SRC:.cpp=.o)
PHASE3_TARGET = phase3

# --- Targets ---
.PHONY: all clean phase1 phase2 phase3

# Default target builds all three phases
all: phase1 phase2 phase3

# --- Phase 1 Build Rules ---
phase1: $(PHASE1_OBJ)
	$(CXX) $(CXXFLAGS) $(PHASE1_INC) -o $(PHASE1_TARGET) $(PHASE1_OBJ)
	@echo "Build successful: $(PHASE1_TARGET)"

$(PHASE1_DIR)/%.o: $(PHASE1_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(PHASE1_INC) -c $< -o $@

# --- Phase 2 Build Rules ---
phase2: $(PHASE2_OBJ)
	$(CXX) $(CXXFLAGS) $(PHASE2_INC) -o $(PHASE2_TARGET) $(PHASE2_OBJ)
	@echo "Build successful: $(PHASE2_TARGET)"

$(PHASE2_DIR)/%.o: $(PHASE2_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(PHASE2_INC) -c $< -o $@

# --- Phase 3 Build Rules ---
# UPDATED: Now links Phase-2/Algorithm.o alongside Phase-3 objects
phase3: $(PHASE3_OBJ) $(PHASE2_DIR)/Algorithm.o
	$(CXX) $(CXXFLAGS) $(PHASE3_INC) -o $(PHASE3_TARGET) $(PHASE3_OBJ) $(PHASE2_DIR)/Algorithm.o
	@echo "Build successful: $(PHASE3_TARGET)"

$(PHASE3_DIR)/%.o: $(PHASE3_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(PHASE3_INC) -c $< -o $@

# --- Utilities ---
clean:
	rm -f $(PHASE1_DIR)/*.o $(PHASE2_DIR)/*.o $(PHASE3_DIR)/*.o
	rm -f $(PHASE1_TARGET) $(PHASE2_TARGET) $(PHASE3_TARGET)
	rm -f output.json
	rm -f $(SAMPLE_TARGET)
	rm -f output.json
	@echo "Clean complete!"