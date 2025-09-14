CXX	 = g++
CXXFLAGS = -std=c++17 -g -Ofast -march=native -W -Wall -MMD -MP
INCLUDES = -I/usr/include/z3 -I.
LIBS     = -lz3
PROG	 = sematrope

all: $(PROG)

$(PROG): | .deps
	$(CXX) $(CXXFLAGS) -MF .deps/$(PROG).d $(INCLUDES) $(PROG).cc $(LIBS) -o $@

.deps:
	mkdir -p .deps

clean:
	rm -rf *.o .deps $(PROG)

-include .deps/*.d

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c -o $@ $<
