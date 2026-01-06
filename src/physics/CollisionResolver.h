#pragma once
#include "Particle.h"
#include "Collision.h"
#include <vector>

class CollisionResolver {
private:
    float restitution; 
    
public:
    CollisionResolver(float restitution = 0.7f);
    
    void setRestitution(float e) { restitution = e; }
    float getRestitution() const { return restitution; }
    
    void resolveContact(Contact& contact);
    
    void resolveContacts(std::vector<Contact>& contacts);
    
    void resolveVelocity(Contact& contact);
    
    void resolveInterpenetration(Contact& contact);
    
private:
    float calculateSeparatingVelocity(const Contact& contact);
};