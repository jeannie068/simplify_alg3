CXX      := g++
CXXFLAGS := -std=c++17 -O3 -Wall -Wextra -MMD
LIBS     := -lm
EXEC     := ../bin/hw4
SRC_DIRS := .\
            parser\
            data_struct\
            solver
SRCS     := $(wildcard $(SRC_DIRS:=/*.cpp))
OBJS     := $(SRCS:.cpp=.o)
DEPS     := $(OBJS:.o=.d)

all: $(EXEC)

$(EXEC): $(OBJS)
	@mkdir -p ../bin
	$(CXX) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(EXEC) $(OBJS) $(DEPS)

ifeq (test, $(firstword $(MAKECMDGOALS)))
  TESTCASE := $(word 2, $(MAKECMDGOALS))
  RATIO    := $(word 3, $(MAKECMDGOALS))
  $(eval $(TESTCASE):;@:)
  $(eval $(RATIO):;@:)
endif

test: $(EXEC)
	@echo Testing with $(TESTCASE) and ratio $(RATIO)
	./$(EXEC) ../testcase/$(TESTCASE).txt ../output/$(TESTCASE).out $(RATIO)

.PHONY: all clean test
-include $(DEPS)