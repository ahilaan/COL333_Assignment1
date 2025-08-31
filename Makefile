# Compiler & flags
CXX      := g++
CXXFLAGS := -std=c++17 -Wall -O2

# Dirs
SRCDIR   := starter_code

# ----- Solver (main) -----
TARGET    := $(SRCDIR)/main
SOURCES   := $(SRCDIR)/main.cpp $(SRCDIR)/planner.cpp
HEADERS   := $(SRCDIR)/planner.h
OBJECTS   := $(SOURCES:.cpp=.o)

# ----- Format checker (at repo root) -----
CHECKER_SRC := format_checker.cpp
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

# Build checker
$(CHECKER_BIN): $(CHECKER_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile checker object
$(CHECKER_OBJ): $(CHECKER_SRC)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Run solver:
#   make run in=starter_code/input1.txt out=starter_code/output1.txt
run: $(TARGET)
	@if [ -z "$(in)" ] || [ -z "$(out)" ]; then \
	  echo "Usage: make run in=starter_code/input1.txt out=starter_code/output1.txt"; exit 1; \
	fi
	@$(TARGET) $(in) $(out)

# Run checker on produced output:
#   make check in=starter_code/input1.txt out=starter_code/output1.txt
check: $(TARGET) $(CHECKER_BIN)
	@if [ -z "$(in)" ] || [ -z "$(out)" ]; then \
	  echo "Usage: make check in=starter_code/input1.txt out=starter_code/output1.txt"; exit 1; \
	fi
	@./$(CHECKER_BIN) $(in) $(out)

clean:
	@rm -f $(OBJECTS) $(TARGET) $(CHECKER_OBJ) $(CHECKER_BIN)
