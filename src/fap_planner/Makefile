CXX := g++
objects := planner.o environment.o graph.o dubins.o

all: clean main

main: main.o $(objects)
	$(CXX) -o main main.o $(objects) -o safeparking

planner.o: src/planner.cpp include/planner.h
	$(CXX) -c src/planner.cpp

environment.o: src/environment.cpp include/environment.h
	$(CXX) -c src/environment.cpp

graph.o: src/graph.cpp include/graph.h
	$(CXX) -c src/graph.cpp

dubins.o: src/dubins.c include/dubins.h
	$(CXX) -c src/dubins.c

main.o: main.cpp 
	$(CXX) -c main.cpp

clean:
	rm -f safeparking *.o


# CXX := g++
# CXXFLAGS := -I /usr/include/python3.8 -I /usr/include/x86_64-linux-gnu/python3.8 -w -lpython3.8
# LLFLAGS := /usr/bin/python3.8-config
# objects := planner.o environment.o graph.o dubins.o

# all: main

# main: main.o $(objects)
# 	$(CXX) -o main main.o $(objects) -o safeparking

# planner.o: src/planner.cpp include/planner.h
# 	$(CXX) -c src/planner.cpp

# environment.o: src/environment.cpp include/environment.h
# 	$(CXX) -c src/environment.cpp

# graph.o: src/graph.cpp include/graph.h
# 	$(CXX) -c src/graph.cpp

# main.o: main.cpp 
# 	# $(CXX) -c main.cpp `pkg-config python3-embed --libs --cflags`
# 	$(CXX) -c main.cpp

# dubins.o: src/dubins.c include/dubins.h
# 	$(CXX) -c src/dubins.c

# clean:
# 	rm -f safeparking *.o