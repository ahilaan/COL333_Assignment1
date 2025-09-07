# Compiler & flags
CXX      := g++
CXXFLAGS := -std=c++17 -Wall -O2

# Dirs
SRCDIR   := starter_code

# ----- Solver (main) -----
# Build the solver binary at repo root as ./main (per instructions)
TARGET    := main
SOURCES   := $(SRCDIR)/main.cpp $(SRCDIR)/planner.cpp
HEADERS   := $(SRCDIR)/planner.h
OBJECTS   := $(SOURCES:.cpp=.o)

# ----- Format checker (at repo root) -----
# Needs io_handler to resolve readInputData
CHECKER_SRC := format_checker.cpp io_handler.cpp
CHECKER_OBJ := $(CHECKER_SRC:.cpp=.o)
CHECKER_BIN := format_checker

.PHONY: all clean run check checker

# Build both by default
all: $(TARGET) $(CHECKER_BIN)

# Link solver
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile solver objects
$(SRCDIR)/%.o: $(SRCDIR)/%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Build checker (links with io_handler.o)
$(CHECKER_BIN): $(CHECKER_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile checker objects
%.o: %.cpp io_handler.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Run solver:
#   make
#   ./main <input_filename> <output_filename>
run: $(TARGET)
	@if [ -z "$(in)" ] || [ -z "$(out)" ]; then \
	  echo "Usage: ./main <input_filename> <output_filename>"; \
	  echo "Or:    make run in=starter_code/input1.txt out=starter_code/output1.txt"; exit 1; \
	fi
	@./$(TARGET) $(in) $(out)

# Build-only target for the checker (per instructions)
checker: $(CHECKER_BIN)
	@echo "Built $(CHECKER_BIN). Run it as: ./format_checker <input_filename> <output_filename>"

# Build & run checker in one go (kept for convenience)
#   make check in=starter_code/input1.txt out=starter_code/output1.txt
check: $(CHECKER_BIN)
	@if [ -z "$(in)" ] || [ -z "$(out)" ]; then \
	  echo "Usage: ./format_checker <input_filename> <output_filename>"; \
	  echo "Or:    make check in=starter_code/input1.txt out=starter_code/output1.txt"; exit 1; \
	fi
	@./$(CHECKER_BIN) $(in) $(out)

clean:
	@rm -f $(OBJECTS) $(TARGET) $(CHECKER_OBJ) $(CHECKER_BIN)
