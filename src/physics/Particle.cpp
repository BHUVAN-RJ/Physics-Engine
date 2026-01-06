#include "Particle.h"
#include <limits>

Particle::Particle() 
    : position(0, 0), velocity(0, 0), acceleration(0, 0),
      mass(1.0f), radius(5.0f), forceAccumulator(0, 0) {}

Particle::Particle(const Vector2& pos, float mass, float radius)
    : position(pos), velocity(0, 0), acceleration(0, 0),
      mass(mass), radius(radius), forceAccumulator(0, 0) {}

void Particle::addForce(const Vector2& force) {
    forceAccumulator += force;
}

void Particle::clearForces() {
    forceAccumulator.x = 0;
    forceAccumulator.y = 0;
}

void Particle::integrate(float dt) {
    if (hasInfiniteMass()) return;  
    
    acceleration = forceAccumulator / mass;
    
    velocity += acceleration * dt;
    
    position += velocity * dt;
    
    clearForces();
}

void Particle::setVelocity(const Vector2& vel) {
    velocity = vel;
}

void Particle::setPosition(const Vector2& pos) {
    position = pos;
}

bool Particle::hasInfiniteMass() const {
    return mass <= 0.0f;  
}