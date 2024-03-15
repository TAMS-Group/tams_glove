all: unwrap

unwrap: src/main.cpp makefile $(wildcard src/*.hpp)
	g++ -I/usr/include/eigen3 -fopenmp --std=c++17 -O3 -DEIGEN_NO_DEBUG src/main.cpp -o unwrap \
	-g \
	-lpthread \
	-lboost_program_options \
	`pkg-config eigen3 opencv4 --cflags --libs`
