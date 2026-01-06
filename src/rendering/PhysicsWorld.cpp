#include "PhysicsWorld.h"

PhysicsWorld::PhysicsWorld(int screenWidth, int screenHeight)
    : screenWidth(screenWidth), screenHeight(screenHeight), 
      useCollisions(false), constraintIterations(3) {
    spatialGrid = std::make_unique<SpatialGrid>(screenWidth, screenHeight, 50);
    collisionResolver = std::make_unique<CollisionResolver>(0.7f);
}

void PhysicsWorld::addParticle(const Particle& particle) {
    particles.push_back(particle);
}

void PhysicsWorld::addForceGenerator(std::shared_ptr<ForceGenerator> generator) {
    forceGenerators.push_back(generator);
}

void PhysicsWorld::addConstraint(std::shared_ptr<Constraint> constraint) {
    constraints.push_back(constraint);
}

void PhysicsWorld::applyForces(float dt) {
    for (auto& generator : forceGenerators) {
        for (auto& particle : particles) {
            generator->applyForce(particle, dt);
        }
    }
}

void PhysicsWorld::solveConstraints() {
    for (int i = 0; i < constraintIterations; i++) {
        for (auto& constraint : constraints) {
            constraint->solve();
        }
    }
}

void PhysicsWorld::detectAndResolveCollisions() {
    if (!useCollisions || particles.size() < 2) return;
    
    spatialGrid->clear();
    for (auto& particle : particles) {
        spatialGrid->insert(particle);
    }
    
    std::vector<Contact> contacts;
    
    for (size_t i = 0; i < particles.size(); i++) {
        auto nearby = spatialGrid->query(particles[i]);
        
        for (auto* other : nearby) {
            if (other <= &particles[i]) continue;
            
            Contact* contact = CollisionDetector::generateContact(particles[i], *other);
            if (contact) {
                contacts.push_back(*contact);
                delete contact;
            }
        }
    }
    
    collisionResolver->resolveContacts(contacts);
}

void PhysicsWorld::applyBoundaryConstraints() {
    for (auto& particle : particles) {
        if (particle.position.y + particle.radius > screenHeight) {
            particle.position.y = screenHeight - particle.radius;
            particle.velocity.y *= -0.6f;
        }
        
        if (particle.position.y - particle.radius < 0) {
            particle.position.y = particle.radius;
            particle.velocity.y *= -0.6f;
        }
        
        if (particle.position.x + particle.radius > screenWidth) {
            particle.position.x = screenWidth - particle.radius;
            particle.velocity.x *= -0.6f;
        }
        
        if (particle.position.x - particle.radius < 0) {
            particle.position.x = particle.radius;
            particle.velocity.x *= -0.6f;
        }
    }
}

void PhysicsWorld::update(float dt) {
    applyForces(dt);
    
    for (auto& particle : particles) {
        particle.integrate(dt);
    }
    
    solveConstraints();
    
    detectAndResolveCollisions();
    
    applyBoundaryConstraints();
}