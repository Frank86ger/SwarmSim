objects := Vector2D.o Boid.o SwarmAndOpenGL.o
CXXFLAGS := -std=c++11

exe: CC := $(CXX)
exe: LDLIBS := -lglut -lGLEW -lGL
exe: $(objects)
	$(LINK.o) $^ -o $@ $(LDLIBS)

SwarmAndOpenGL.o: SwarmAndOpenGL.cpp
Boid.o: Boid.h
Vector2D.o: Vector2D.h

clean:
	$(RM) $(objects)
