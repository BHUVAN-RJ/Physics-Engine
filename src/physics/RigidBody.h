#pragma once
#include "Vector2.h"
#include <vector>

enum class ShapeType {
    CIRCLE,
    BOX
};

class RigidBody {
public:
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
    Vector2 forceAccumulator;
    
    float orientation;        
    float angularVelocity;    
    float angularAcceleration;
    float torqueAccumulator;  
    
    float mass;
    float inverseMass;        
    float inertia;            
    float inverseInertia;     
    
    float restitution;        
    float friction;           
    
    ShapeType shapeType;
    float radius;             
    float width, height;      
    
    RigidBody();
    static RigidBody createCircle(const Vector2& pos, float radius, float mass);
    static RigidBody createBox(const Vector2& pos, float width, float height, float mass);
    
    void addForce(const Vector2& force);
    void addForceAtPoint(const Vector2& force, const Vector2& point);
    void addTorque(float torque);
    void clearForces();
    void integrate(float dt);
    
    bool hasInfiniteMass() const { return mass <= 0.0f; }
    bool hasInfiniteInertia() const { return inertia <= 0.0f; }
    
    std::vector<Vector2> getVertices() const;
    
    std::vector<Vector2> getAxes() const;
    
private:
    void calculateInertia();
};