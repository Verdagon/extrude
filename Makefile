current_dir = $(shell pwd)

CC = clang++
CFLAGS = -std=c++11 -stdlib=libc++ -g -I$(current_dir)/glm/glm -I$(current_dir)/freetype/include/freetype2
LDFLAGS = -stdlib=libc++
LIBS =  -lfreetype -L$(current_dir)/freetype/lib

SRCS = src/utilities.cpp $(filter-out src/utilities.cpp, $(wildcard src/*.cpp))
OBJS = $(patsubst src/%, build/%, $(patsubst %.cpp, %.o, $(SRCS)))

all:	build extrudecpp

build:
	mkdir build

extrudecpp:	$(OBJS)
	$(CC) $^ -o extrudecpp $(LIBS) $(LDFLAGS)

build/%.o:	src/%.cpp src/*.h
	$(CC) $< -c -o $@ $(CFLAGS)

run:	extrudecpp
	./extrudecpp

debug:	extrudecpp
	gdb ./extrudecpp

clean:
	rm -f extrudecpp
	rm -f build/*.o
