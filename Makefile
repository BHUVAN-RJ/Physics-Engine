CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Isrc/math -Isrc/physics -Isrc/rendering -I/opt/homebrew/include
LDFLAGS = -L/opt/homebrew/lib
SFML_FLAGS = -lsfml-graphics -lsfml-window -lsfml-system

SOURCES = main.cpp src/math/Vector2.cpp src/physics/ForceGenerator.cpp src/physics/Collision.cpp src/physics/CollisionResolver.cpp src/physics/Constraint.cpp src/physics/RigidBody.cpp src/physics/RigidBodyResolver.cpp src/physics/Particle.cpp  src/rendering/Renderer.cpp src/rendering/PhysicsWorld.cpp 
OBJECTS = $(SOURCES:.cpp=.o)
TARGET = physics_sim

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJECTS) $(SFML_FLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run