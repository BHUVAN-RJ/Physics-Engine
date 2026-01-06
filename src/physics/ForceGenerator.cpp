#include "ForceGenerator.h"
#include <cmath>

GravityForce::GravityForce(const Vector2& g) : gravity(g) {}

void GravityForce::applyForce(Particle& particle, float dt) {
    if (!particle.hasInfiniteMass()) {
        particle.addForce(gravity * particle.mass);
    }
}

DragForce::DragForce(float k1, float k2) : k1(k1), k2(k2) {}

void DragForce::applyForce(Particle& particle, float dt) {
    if (particle.hasInfiniteMass()) return;
    
    Vector2 velocity = particle.velocity;
    float speed = velocity.magnitude();
    
    if (speed > 0) {
        float dragMagnitude = k1 * speed + k2 * speed * speed;
        Vector2 dragForce = velocity.normalize() * (-dragMagnitude);
        particle.addForce(dragForce);
    }
}

WindForce::WindForce(const Vector2& velocity, float strength)
    : windVelocity(velocity), strength(strength) {}

void WindForce::applyForce(Particle& particle, float dt) {
    if (!particle.hasInfiniteMass()) {
        particle.addForce(windVelocity * strength);
    }
}

void WindForce::setWind(const Vector2& velocity, float strength) {
    this->windVelocity = velocity;
    this->strength = strength;
}

AttractorForce::AttractorForce(const Vector2& pos, float strength, float minDist)
    : position(pos), strength(strength), minDistance(minDist) {}

void AttractorForce::applyForce(Particle& particle, float dt) {
    if (particle.hasInfiniteMass()) return;
    
    Vector2 direction = position - particle.position;
    float distance = direction.magnitude();
    
    if (distance < minDistance) {
        distance = minDistance;
    }
    
    float forceMagnitude = strength / (distance * distance);
    
    Vector2 force = direction.normalize() * forceMagnitude;
    particle.addForce(force);
}

FrictionForce::FrictionForce(float coeff) : coefficient(coeff) {}

void FrictionForce::applyForce(Particle& particle, float dt) {
    if (particle.hasInfiniteMass()) return;
    
    Vector2 velocity = particle.velocity;
    float speed = velocity.magnitude();
    
    if (speed > 0) {
        Vector2 frictionForce = velocity.normalize() * (-coefficient * speed);
        particle.addForce(frictionForce);
    }
}