#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL
PWD=$(shell pwd)

# Compiler and Linker flags
CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing
CPPFLAGS+=-I external/include -I /usr/include
LDFLAGS+=-L external/lib -L /usr/lib

# list of all the cpp files in src
SRCS := $(wildcard src/*.cpp)
# list of all header files in src
INCS := $(wildcard src/*.hpp)
# list of all .o files in myobjs


OBJS := $(SRCS:src/%.cpp=myobjs/%.o)

STATIC_LIB=TRUE	

.PHONY: all dir installed

all: dir installed exelib


dir:
	@echo "Setting up compilation..."
	@mkdir -p myobjs
	@mkdir -p mylibs
	@mkdir -p mybins


installed:
#assuming Box2D to either be installed in both directories or neither
ifeq ($(wildcard ./external/src/Box2D/.*),) 
	tar  zxvf ./external/src/Box2D.tgz -C ./external/src; \
	mkdir -p ./external/src/Box2D/build251; \
	cd ./external/src/Box2D/build251; \
	cmake ../; \
	make; \
	make install; 
endif

executable: $(OBJS)
	@g++ -o mybins/id408_exe_07 $(LDFLAGS) $(OBJS) $(LIBS)

TEMP := $(filter-out ./myobjs/main.o,$(OBJS))

libid408.a: $(TEMP)
	ar rcs ./mylibs/libid408.a $(TEMP)

exelib: dir installed $(OBJS) libid408.a 
		g++ -o ./mybins/id408_exelib_07 myobjs/main.o -L ./external/lib -L /usr/lib -L ./mylibs -lid408 $(LIBS); \
	

$(OBJS): myobjs/%.o : src/%.cpp
	@g++ -fPIC $(CPPFLAGS) -c $< -o $@ -MD

report:
	@rm -rf doc/beamer/*.aux doc/beamer/*.bbl doc/beamer/*.toc doc/beamer/*.blg doc/beamer/*.log doc/beamer/*.nav doc/beamer/*.out doc/beamer/*.snm
	@cp ./doc/beamer/* .
	@pdflatex main.tex
	@bibtex main.aux
	@pdflatex main.tex
	@pdflatex main.tex
	@rm -rf *.jpg *.png content.tex
	@mv main* ./doc/beamer

clean:
	@echo -n "Cleaning up..."
	@rm -rf my*
	@rm -rf external/lib/*
	@rm -rf external/include/*
	@echo "Done"

distclean: clean
	@rm -rf external/src/Box2D
