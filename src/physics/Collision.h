#pragma once
#include "Particle.h"
#include "Vector2.h"
#include <vector>
#include "RigidBody.h"

struct Contact {
    Particle* particleA;
    Particle* particleB;
    Vector2 normal;        
    float penetration;    
    
    Contact(Particle* a, Particle* b, const Vector2& n, float pen)
        : particleA(a), particleB(b), normal(n), penetration(pen) {}
};

struct AABB {
    Vector2 min;
    Vector2 max;
    
    AABB() : min(0, 0), max(0, 0) {}
    AABB(const Vector2& min, const Vector2& max) : min(min), max(max) {}
    
    bool intersects(const AABB& other) const;
    bool contains(const Vector2& point) const;
    
    static AABB fromParticle(const Particle& particle);
};
struct RigidContact {
        RigidBody* bodyA;
        RigidBody* bodyB;
        Vector2 contactPoint;
        Vector2 normal;
        float penetration;
        
        RigidContact(RigidBody* a, RigidBody* b, const Vector2& point,
                    const Vector2& n, float pen)
            : bodyA(a), bodyB(b), contactPoint(point), normal(n), penetration(pen) {}
    };
class CollisionDetector {
public:
    static bool checkCollision(const Particle& a, const Particle& b);
    static Contact* generateContact(Particle& a, Particle& b);
    static bool checkAABBCollision(const AABB& a, const AABB& b);
    static std::vector<Contact> detectCollisions(std::vector<Particle>& particles);
    static std::vector<Contact> detectCollisionsBroadPhase(std::vector<Particle>& particles);
    
    static bool checkCollision(const RigidBody& a, const RigidBody& b);
    static RigidContact* generateRigidContact(RigidBody& a, RigidBody& b);
    
    static bool checkCircleBoxCollision(const RigidBody& circle, const RigidBody& box);
    static RigidContact* generateCircleBoxContact(RigidBody& circle, RigidBody& box);
    
    static bool checkBoxBoxCollision(const RigidBody& a, const RigidBody& b);
    static RigidContact* generateBoxBoxContact(RigidBody& a, RigidBody& b); 
};

class SpatialGrid {
private:
    int cellSize;
    int gridWidth;
    int gridHeight;
    std::vector<std::vector<std::vector<Particle*>>> grid;
    
public:
    SpatialGrid(int screenWidth, int screenHeight, int cellSize);
    
    void clear();
    void insert(Particle& particle);
    std::vector<Particle*> query(const Particle& particle);
    
private:
    int getGridX(float x) const;
    int getGridY(float y) const;
};