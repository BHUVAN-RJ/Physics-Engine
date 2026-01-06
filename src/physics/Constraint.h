#pragma once
#include "Particle.h"
#include "Vector2.h"

class Constraint {
public:
    virtual ~Constraint() = default;
    virtual void solve() = 0;  
    virtual void render(class Renderer& renderer) = 0;  
};

class SpringConstraint : public Constraint {
private:
    Particle* particleA;
    Particle* particleB;
    float restLength;    
    float stiffness;     
    float damping;       
    
public:
    SpringConstraint(Particle* a, Particle* b, float restLength, 
                     float stiffness, float damping = 0.1f);
    
    void solve() override;
    void render(Renderer& renderer) override;
    
    void setStiffness(float k) { stiffness = k; }
    void setDamping(float d) { damping = d; }
};

class DistanceConstraint : public Constraint {
private:
    Particle* particleA;
    Particle* particleB;
    float distance;      
    float stiffness;     
    
public:
    DistanceConstraint(Particle* a, Particle* b, float distance, 
                       float stiffness = 1.0f);
    
    void solve() override;
    void render(Renderer& renderer) override;
    
    void setStiffness(float s) { stiffness = s; }
};

class PinConstraint : public Constraint {
private:
    Particle* particle;
    Vector2 position;
    float stiffness;
    
public:
    PinConstraint(Particle* p, const Vector2& pos, float stiffness = 1.0f);
    
    void solve() override;
    void render(Renderer& renderer) override;
    
    void setPosition(const Vector2& pos) { position = pos; }
};

class AngleConstraint : public Constraint {
private:
    Particle* particleA;
    Particle* particleB;
    Particle* particleC;
    float targetAngle;  
    float stiffness;
    
public:
    AngleConstraint(Particle* a, Particle* b, Particle* c, 
                    float angle, float stiffness = 0.5f);
    
    void solve() override;
    void render(Renderer& renderer) override;
};