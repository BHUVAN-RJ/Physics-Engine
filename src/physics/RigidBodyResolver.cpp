#include "RigidBodyResolver.h"
#include <algorithm>
#include <cmath>

void RigidBodyResolver::resolveVelocity(RigidContact& contact) {
    Vector2 r1 = contact.contactPoint - contact.bodyA->position;
    Vector2 r2 = contact.contactPoint - contact.bodyB->position;
    
    Vector2 v1 = contact.bodyA->velocity + Vector2(-r1.y, r1.x) * contact.bodyA->angularVelocity;
    Vector2 v2 = contact.bodyB->velocity + Vector2(-r2.y, r2.x) * contact.bodyB->angularVelocity;
    
    Vector2 relativeVelocity = v2 - v1;
    float separatingVelocity = relativeVelocity.dot(contact.normal);
    
    if (separatingVelocity > 0) return;
    
    float restitution = std::min(contact.bodyA->restitution, contact.bodyB->restitution);
    
    const float restingThreshold = 1.0f;
    if (std::abs(separatingVelocity) < restingThreshold) {
        restitution = 0.0f;
    }
    
    float numerator = -(1.0f + restitution) * separatingVelocity;
    
    float linearTerm = contact.bodyA->inverseMass + contact.bodyB->inverseMass;
    
    float angularTerm1 = 0.0f;
    if (!contact.bodyA->hasInfiniteInertia()) {
        float r1CrossN = r1.cross(contact.normal);
        angularTerm1 = (r1CrossN * r1CrossN) * contact.bodyA->inverseInertia;
    }
    
    float angularTerm2 = 0.0f;
    if (!contact.bodyB->hasInfiniteInertia()) {
        float r2CrossN = r2.cross(contact.normal);
        angularTerm2 = (r2CrossN * r2CrossN) * contact.bodyB->inverseInertia;
    }
    
    float denominator = linearTerm + angularTerm1 + angularTerm2;
    
    if (denominator < 0.0001f) return;  
    
    float impulse = numerator / denominator;
    
    const float maxImpulse = 1000.0f;
    impulse = std::max(-maxImpulse, std::min(impulse, maxImpulse));
    
    Vector2 impulseVector = contact.normal * impulse;
    
    if (!contact.bodyA->hasInfiniteMass()) {
        contact.bodyA->velocity -= impulseVector * contact.bodyA->inverseMass;
    }
    
    if (!contact.bodyB->hasInfiniteMass()) {
        contact.bodyB->velocity += impulseVector * contact.bodyB->inverseMass;
    }
    
    if (!contact.bodyA->hasInfiniteInertia()) {
        float torqueA = r1.cross(impulseVector);
        float angularImpulseA = torqueA * contact.bodyA->inverseInertia;
        
        const float maxAngularImpulse = 10.0f;
        angularImpulseA = std::max(-maxAngularImpulse, std::min(angularImpulseA, maxAngularImpulse));
        
        contact.bodyA->angularVelocity -= angularImpulseA;
    }
    
    if (!contact.bodyB->hasInfiniteInertia()) {
        float torqueB = r2.cross(impulseVector);
        float angularImpulseB = torqueB * contact.bodyB->inverseInertia;
        
        const float maxAngularImpulse = 10.0f;
        angularImpulseB = std::max(-maxAngularImpulse, std::min(angularImpulseB, maxAngularImpulse));
        
        contact.bodyB->angularVelocity += angularImpulseB;
    }
    
    Vector2 tangent(-contact.normal.y, contact.normal.x);
    
    v1 = contact.bodyA->velocity + Vector2(-r1.y, r1.x) * contact.bodyA->angularVelocity;
    v2 = contact.bodyB->velocity + Vector2(-r2.y, r2.x) * contact.bodyB->angularVelocity;
    relativeVelocity = v2 - v1;
    
    float tangentVelocity = relativeVelocity.dot(tangent);
    
    if (std::abs(tangentVelocity) < 0.001f) return;  
    
    float frictionNumerator = -tangentVelocity;
    float frictionImpulse = frictionNumerator / denominator;
    
    float friction = std::sqrt(contact.bodyA->friction * contact.bodyB->friction);
    float maxFriction = friction * std::abs(impulse);
    
    frictionImpulse = std::max(-maxFriction, std::min(frictionImpulse, maxFriction));
    
    Vector2 frictionVector = tangent * frictionImpulse;
    
    if (!contact.bodyA->hasInfiniteMass()) {
        contact.bodyA->velocity -= frictionVector * contact.bodyA->inverseMass;
    }
    
    if (!contact.bodyB->hasInfiniteMass()) {
        contact.bodyB->velocity += frictionVector * contact.bodyB->inverseMass;
    }
    
    if (!contact.bodyA->hasInfiniteInertia()) {
        float frictionTorqueA = r1.cross(frictionVector);
        contact.bodyA->angularVelocity -= frictionTorqueA * contact.bodyA->inverseInertia * 0.5f;
    }
    
    if (!contact.bodyB->hasInfiniteInertia()) {
        float frictionTorqueB = r2.cross(frictionVector);
        contact.bodyB->angularVelocity += frictionTorqueB * contact.bodyB->inverseInertia * 0.5f;
    }
}

void RigidBodyResolver::resolveInterpenetration(RigidContact& contact) {
    const float slop = 0.01f;
    const float percent = 0.8f;  
    
    float correctionMagnitude = std::max(contact.penetration - slop, 0.0f) * percent;
    
    if (correctionMagnitude <= 0.0f) return;
    
    float totalInverseMass = contact.bodyA->inverseMass + contact.bodyB->inverseMass;
    
    if (totalInverseMass <= 0.0f) return;
    
    Vector2 correction = contact.normal * (correctionMagnitude / totalInverseMass);
    
    if (!contact.bodyA->hasInfiniteMass()) {
        contact.bodyA->position -= correction * contact.bodyA->inverseMass;
    }
    
    if (!contact.bodyB->hasInfiniteMass()) {
        contact.bodyB->position += correction * contact.bodyB->inverseMass;
    }
}

void RigidBodyResolver::resolveContact(RigidContact& contact) {
    resolveVelocity(contact);
    resolveInterpenetration(contact);
}

void RigidBodyResolver::resolveContacts(std::vector<RigidContact>& contacts) {
    for (auto& contact : contacts) {
        resolveContact(contact);
    }
}