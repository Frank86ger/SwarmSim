objects := Vector2D.o Boid.o SwarmAndOpenGL.o
CXXFLAGS := -std=c++11

program1: CC := $(CXX)
program1: LDLIBS := -lglut -lGLEW -lGL
program1: $(objects)
	$(LINK.o) $^ -o $@ $(LDLIBS)

SwarmAndOpenGL.o: SwarmAndOpenGL.cpp
Boid.o: Boid.h
Vector2D.o: Vector2D.h



# .PHONY: clean
clean:
	$(RM) $(objects)





# output: Vector2D.o Boid.o Swarm.o
# 	g++ -std=c++11 -Wall Vector2D.o Boid.o Swarm.o -o output

# Swarm.o: Swarm.cpp
# 	g++ -c Swarm.cpp

# Boid.o: Boid.cpp Boid.h
# 	g++ -c Boid.cpp

# Vector2D.o: Vector2D.cpp Vector2D.h
# 	g++ -c Vector2D.cpp

# clean:
# 	@echo "Cleanup"
# 	rm *.o




	