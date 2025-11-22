CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2 -g

# --- Phase 1 Definitions ---
PHASE1_DIR = Phase1
PHASE1_INC = -I./Phase1
PHASE1_SRC = $(PHASE1_DIR)/main.cpp \
             $(PHASE1_DIR)/Graph.cpp \
             $(PHASE1_DIR)/Algorithm.cpp
PHASE1_OBJ = $(PHASE1_SRC:.cpp=.o)
PHASE1_TARGET = phase1

# --- Phase 2 Definitions ---
PHASE2_DIR = Phase2
PHASE2_INC = -I./Phase2
PHASE2_SRC = $(PHASE2_DIR)/main.cpp \
             $(PHASE2_DIR)/Graph.cpp \
             $(PHASE2_DIR)/Algorithm.cpp
PHASE2_OBJ = $(PHASE2_SRC:.cpp=.o)
PHASE2_TARGET = phase2

# --- Targets ---
.PHONY: all clean phase1 phase2

# Default target builds both
all: phase1 phase2

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

# --- Utilities ---
clean:
	rm -f $(PHASE1_DIR)/*.o $(PHASE2_DIR)/*.o
	rm -f $(PHASE1_TARGET) $(PHASE2_TARGET)
	rm -f output.json
	@echo "Clean complete!"