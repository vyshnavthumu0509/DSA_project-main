CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2 -g
INCLUDES = -I./Phase1

# --- Directory Definitions ---
PHASE1_DIR = Phase1

# Sources: main.cpp, Graph.cpp, and Algorithm.cpp are all inside Phase1/
PHASE1_SRC = $(PHASE1_DIR)/main.cpp \
             $(PHASE1_DIR)/Graph.cpp \
             $(PHASE1_DIR)/Algorithm.cpp

# Generate object file names (.o) from source file names (.cpp)
PHASE1_OBJ = $(PHASE1_SRC:.cpp=.o)

# The final executable name
PHASE1_TARGET = phase1

# --- Targets ---
.PHONY: all clean

# Default target
all: phase1

# Target to build the phase1 executable
phase1: $(PHASE1_OBJ)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(PHASE1_TARGET) $(PHASE1_OBJ)
	@echo "Build successful! Executable '$(PHASE1_TARGET)' created."
	@echo "Usage: ./$(PHASE1_TARGET) <graph.json> <queries.json> <output.json>"

# --- Compilation Rules ---

# Generic rule to compile .cpp files in Phase1 directory to .o files
$(PHASE1_DIR)/%.o: $(PHASE1_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# --- Utilities ---

clean:
	rm -f $(PHASE1_DIR)/*.o
	rm -f $(PHASE1_TARGET)
	rm -f output.json
	@echo "Clean complete!"