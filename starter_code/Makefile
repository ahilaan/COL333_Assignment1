# Compiler and flags
CXX     := g++
CXXFLAGS:= -std=c++17 -O2 -Wall

# Target executable
TARGET  := main

# Sources and objects
SRCS    := main.cpp planner.cpp
OBJS    := $(SRCS:.cpp=.o)

# Default rule
all: $(TARGET)

# Linking
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

# Compile .cpp to .o
%.o: %.cpp planner.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f $(OBJS) $(TARGET)
