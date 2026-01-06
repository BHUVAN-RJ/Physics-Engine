#include "CollisionResolver.h"
#include <algorithm>
#include <cmath>

CollisionResolver::CollisionResolver(float restitution)
    : restitution(restitution) {}

float CollisionResolver::calculateSeparatingVelocity(const Contact& contact) {
    Vector2 relativeVelocity = contact.particleB->velocity - contact.particleA->velocity;
    
    return relativeVelocity.dot(contact.normal);
}

void CollisionResolver::resolveVelocity(Contact& contact) {
    float separatingVelocity = calculateSeparatingVelocity(contact);
    
    if (separatingVelocity > 0) {
        return; 
    }
    
    float newSeparatingVelocity = -separatingVelocity * restitution;
    
    float deltaVelocity = newSeparatingVelocity - separatingVelocity;
    
    float totalInverseMass = 0.0f;
    if (!contact.particleA->hasInfiniteMass()) {
        totalInverseMass += 1.0f / contact.particleA->mass;
    }
    if (!contact.particleB->hasInfiniteMass()) {
        totalInverseMass += 1.0f / contact.particleB->mass;
    }
    
    if (totalInverseMass <= 0) return;
    
    float impulse = deltaVelocity / totalInverseMass;
    
    Vector2 impulsePerMass = contact.normal * impulse;
    
    if (!contact.particleA->hasInfiniteMass()) {
        contact.particleA->velocity += impulsePerMass * (-1.0f / contact.particleA->mass);
    }
    if (!contact.particleB->hasInfiniteMass()) {
        contact.particleB->velocity += impulsePerMass * (1.0f / contact.particleB->mass);
    }
}

void CollisionResolver::resolveInterpenetration(Contact& contact) {
    if (contact.penetration <= 0) return;
    
    float totalInverseMass = 0.0f;
    if (!contact.particleA->hasInfiniteMass()) {
        totalInverseMass += 1.0f / contact.particleA->mass;
    }
    if (!contact.particleB->hasInfiniteMass()) {
        totalInverseMass += 1.0f / contact.particleB->mass;
    }
    
    if (totalInverseMass <= 0) return;
    
    Vector2 movePerInverseMass = contact.normal * (contact.penetration / totalInverseMass);
    
    if (!contact.particleA->hasInfiniteMass()) {
        float inverseMassA = 1.0f / contact.particleA->mass;
        contact.particleA->position -= movePerInverseMass * inverseMassA;
    }
    if (!contact.particleB->hasInfiniteMass()) {
        float inverseMassB = 1.0f / contact.particleB->mass;
        contact.particleB->position += movePerInverseMass * inverseMassB;
    }
}

void CollisionResolver::resolveContact(Contact& contact) {
    resolveVelocity(contact);
    
    resolveInterpenetration(contact);
}

void CollisionResolver::resolveContacts(std::vector<Contact>& contacts) {
    for (auto& contact : contacts) {
        resolveContact(contact);
    }
}