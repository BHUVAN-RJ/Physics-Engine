#pragma once
#include "RigidBody.h"
#include "Collision.h"
#include <vector>

class RigidBodyResolver {
public:
    void resolveContact(RigidContact& contact);
    void resolveContacts(std::vector<RigidContact>& contacts);
    
private:
    void resolveVelocity(RigidContact& contact);
    void resolveInterpenetration(RigidContact& contact);
};