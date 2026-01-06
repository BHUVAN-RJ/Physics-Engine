#pragma once
#include "Vector2.h"

class Particle {
public:
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
    
    float mass;
    float radius;  
    
    Vector2 forceAccumulator;  
    
    Particle();
    Particle(const Vector2& pos, float mass, float radius = 5.0f);
    
    void addForce(const Vector2& force);
    void clearForces();
    void integrate(float dt);  
    
    void setVelocity(const Vector2& vel);
    void setPosition(const Vector2& pos);
    
    bool hasInfiniteMass() const;
};