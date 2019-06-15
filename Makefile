CXX	 = g++
CXXFLAGS = -std=c++17 -g -Ofast -march=native -W -Wall
INCLUDES = -I/usr/include/z3 -I.
LIBS     = -lz3
PROG	 = sematrope

all: $(PROG)

$(PROG): $(PROG).cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ $(LIBS) -o $@

clean:
	rm -f *.o $(PROG)

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c -o $@ $<
