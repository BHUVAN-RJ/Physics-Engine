#pragma once
#include <vector>
#include <memory>
#include "Particle.h"
#include "Vector2.h"
#include "ForceGenerator.h"
#include "Collision.h"
#include "CollisionResolver.h"
#include "Constraint.h"

class PhysicsWorld {
private:
    std::vector<Particle> particles;
    std::vector<std::shared_ptr<ForceGenerator>> forceGenerators;
    std::vector<std::shared_ptr<Constraint>> constraints;
    std::unique_ptr<SpatialGrid> spatialGrid;
    std::unique_ptr<CollisionResolver> collisionResolver;
    
    int screenWidth;
    int screenHeight;
    bool useCollisions;
    int constraintIterations;
    
public:
    PhysicsWorld(int screenWidth, int screenHeight);
    
    void addParticle(const Particle& particle);
    void addForceGenerator(std::shared_ptr<ForceGenerator> generator);
    void addConstraint(std::shared_ptr<Constraint> constraint);
    void update(float dt);
    
    void setCollisionsEnabled(bool enabled) { useCollisions = enabled; }
    void setRestitution(float e) { collisionResolver->setRestitution(e); }
    void setConstraintIterations(int iterations) { constraintIterations = iterations; }
    
    void applyForces(float dt);
    void solveConstraints();
    void applyBoundaryConstraints();
    void detectAndResolveCollisions();
    
    const std::vector<Particle>& getParticles() const { return particles; }
    std::vector<Particle>& getParticles() { return particles; }
    std::vector<std::shared_ptr<Constraint>>& getConstraints() { return constraints; }
    
    void clear() { 
        particles.clear(); 
        constraints.clear();
    }
};