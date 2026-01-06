#pragma once
#include "Particle.h"
#include "Vector2.h"
#include <vector>

class ForceGenerator {
public:
    virtual ~ForceGenerator() = default;
    virtual void applyForce(Particle& particle, float dt) = 0;
};

class GravityForce : public ForceGenerator {
private:
    Vector2 gravity;
    
public:
    GravityForce(const Vector2& g);
    void applyForce(Particle& particle, float dt) override;
    void setGravity(const Vector2& g) { gravity = g; }
};

class DragForce : public ForceGenerator {
private:
    float k1;  
    float k2; 
    
public:
    DragForce(float k1, float k2);
    void applyForce(Particle& particle, float dt) override;
};

class WindForce : public ForceGenerator {
private:
    Vector2 windVelocity;
    float strength;
    
public:
    WindForce(const Vector2& velocity, float strength);
    void applyForce(Particle& particle, float dt) override;
    void setWind(const Vector2& velocity, float strength);
};

class AttractorForce : public ForceGenerator {
private:
    Vector2 position;
    float strength;  
    float minDistance;  
    
public:
    AttractorForce(const Vector2& pos, float strength, float minDist = 10.0f);
    void applyForce(Particle& particle, float dt) override;
    void setPosition(const Vector2& pos) { position = pos; }
    void setStrength(float s) { strength = s; }
};

class FrictionForce : public ForceGenerator {
private:
    float coefficient;
    
public:
    FrictionForce(float coeff);
    void applyForce(Particle& particle, float dt) override;
};