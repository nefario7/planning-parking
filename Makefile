CXX := g++
objects := planner.o map.o graph.o

all: main

main: main.o $(objects)
	$(CXX) -o main main.o $(objects) -o safeparking

planner.o: src/planner.cpp include/planner.h
	$(CXX) -c src/planner.cpp

environment.o: src/map.cpp include/environment.h
	$(CXX) -c src/environment.cpp

graph.o: src/graph.cpp include/graph.h
	$(CXX) -c src/graph.cpp

main.o: main.cpp 
	$(CXX) -c main.cpp

clean:
	rm -f safeparking *.o