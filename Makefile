CXX := g++
objects := planner.o map.o graph.o

all: main

main: main.o $(objects)
	$(CXX) -o main main.o $(objects) -o safeparking

planner.o: src/planner.cpp include/planner.h
	$(CXX) -c src/planner.cpp

map.o: src/map.cpp include/map.h
	$(CXX) -c src/map.cpp

graph.o: src/graph.cpp include/graph.h
	$(CXX) -c src/graph.cpp

main.o: main.cpp 
	$(CXX) -c main.cpp

clean:
	rm -f safeparking *.o