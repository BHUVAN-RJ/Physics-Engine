#include "Constraint.h"
#include "Renderer.h"
#include <cmath>

SpringConstraint::SpringConstraint(Particle* a, Particle* b, float restLength,
                                   float stiffness, float damping)
    : particleA(a), particleB(b), restLength(restLength), 
      stiffness(stiffness), damping(damping) {}

void SpringConstraint::solve() {
    Vector2 delta = particleB->position - particleA->position;
    float currentLength = delta.magnitude();
    
    if (currentLength == 0) return;  
    float displacement = currentLength - restLength;
    float forceMagnitude = -stiffness * displacement;
    
    Vector2 relativeVelocity = particleB->velocity - particleA->velocity;
    float dampingForceMag = -damping * (relativeVelocity.dot(delta) / currentLength);
    
    float totalForce = forceMagnitude + dampingForceMag;
    Vector2 force = delta.normalize() * totalForce;
    
    if (!particleA->hasInfiniteMass()) {
        particleA->addForce(force * -1.0f);
    }
    if (!particleB->hasInfiniteMass()) {
        particleB->addForce(force);
    }
}

void SpringConstraint::render(Renderer& renderer) {
    renderer.drawLine(particleA->position, particleB->position, 
                     sf::Color(100, 200, 255));
}

DistanceConstraint::DistanceConstraint(Particle* a, Particle* b, 
                                       float distance, float stiffness)
    : particleA(a), particleB(b), distance(distance), stiffness(stiffness) {}

void DistanceConstraint::solve() {
    Vector2 delta = particleB->position - particleA->position;
    float currentDistance = delta.magnitude();
    
    if (currentDistance == 0) return;
    
    float difference = (currentDistance - distance) / currentDistance;
    Vector2 correction = delta * (difference * 0.5f * stiffness);
    
    if (!particleA->hasInfiniteMass()) {
        particleA->position += correction;
    }
    if (!particleB->hasInfiniteMass()) {
        particleB->position -= correction;
    }
}

void DistanceConstraint::render(Renderer& renderer) {
    renderer.drawLine(particleA->position, particleB->position, 
                     sf::Color(255, 200, 100));
}

PinConstraint::PinConstraint(Particle* p, const Vector2& pos, float stiffness)
    : particle(p), position(pos), stiffness(stiffness) {}

void PinConstraint::solve() {
    if (particle->hasInfiniteMass()) return;
    
    Vector2 delta = position - particle->position;
    particle->position += delta * stiffness;
}

void PinConstraint::render(Renderer& renderer) {
    renderer.drawCircle(position, 5, sf::Color::Red);
    renderer.drawLine(position, particle->position, sf::Color(255, 100, 100));
}

AngleConstraint::AngleConstraint(Particle* a, Particle* b, Particle* c,
                                 float angle, float stiffness)
    : particleA(a), particleB(b), particleC(c), 
      targetAngle(angle), stiffness(stiffness) {}

void AngleConstraint::solve() {
    Vector2 ba = particleA->position - particleB->position;
    Vector2 bc = particleC->position - particleB->position;
    
    float lenBA = ba.magnitude();
    float lenBC = bc.magnitude();
    
    if (lenBA == 0 || lenBC == 0) return;
    
    float dotProduct = ba.dot(bc) / (lenBA * lenBC);
    dotProduct = std::max(-1.0f, std::min(1.0f, dotProduct)); 
    float currentAngle = std::acos(dotProduct);
    
    float angleDiff = targetAngle - currentAngle;
    
    float force = angleDiff * stiffness;
    
}

void AngleConstraint::render(Renderer& renderer) {
    renderer.drawLine(particleA->position, particleB->position, 
                     sf::Color(150, 150, 255));
    renderer.drawLine(particleB->position, particleC->position, 
                     sf::Color(150, 150, 255));
}