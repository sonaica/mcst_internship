CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic -g
SANITIZER_FLAGS = -fsanitize=address,undefined -fno-omit-frame-pointer
TARGET = main
SRC = $(wildcard src/*.cpp)
INC = -Iinclude

TEST_NUM ?= 1

all: build test

build: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SANITIZER_FLAGS) $(INC) $^ -o $@

check:
	cppcheck --enable=all --suppress=missingIncludeSystem $(INC) $(SRC)

test: $(TARGET) run_test.sh
	chmod +x run_test.sh  
	./run_test.sh $(TEST_NUM)  

clean:
	rm -f $(TARGET) output*.txt

.PHONY: all build test clean check