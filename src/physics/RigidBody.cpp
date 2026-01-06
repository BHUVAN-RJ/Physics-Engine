#include "RigidBody.h"
#include <cmath>
#include <limits>

RigidBody::RigidBody()
    : position(0, 0), velocity(0, 0), acceleration(0, 0), forceAccumulator(0, 0),
      orientation(0), angularVelocity(0), angularAcceleration(0), torqueAccumulator(0),
      mass(1.0f), inverseMass(1.0f), inertia(1.0f), inverseInertia(1.0f),
      restitution(0.5f), friction(0.3f),
      shapeType(ShapeType::CIRCLE), radius(10.0f), width(20.0f), height(20.0f) {}

RigidBody RigidBody::createCircle(const Vector2& pos, float radius, float mass) {
    RigidBody body;
    body.position = pos;
    body.radius = radius;
    body.mass = mass;
    body.shapeType = ShapeType::CIRCLE;
    
    if (mass > 0) {
        body.inverseMass = 1.0f / mass;
        body.inertia = 0.5f * mass * radius * radius;
        body.inverseInertia = 1.0f / body.inertia;
    } else {
        body.inverseMass = 0.0f;
        body.inertia = 0.0f;
        body.inverseInertia = 0.0f;
    }
    
    return body;
}

RigidBody RigidBody::createBox(const Vector2& pos, float width, float height, float mass) {
    RigidBody body;
    body.position = pos;
    body.width = width;
    body.height = height;
    body.mass = mass;
    body.shapeType = ShapeType::BOX;
    
    if (mass > 0) {
        body.inverseMass = 1.0f / mass;
        body.inertia = mass * (width * width + height * height) / 12.0f;
        body.inverseInertia = 1.0f / body.inertia;
    } else {
        body.inverseMass = 0.0f;
        body.inertia = 0.0f;
        body.inverseInertia = 0.0f;
    }
    
    return body;
}

void RigidBody::addForce(const Vector2& force) {
    forceAccumulator += force;
}

void RigidBody::addForceAtPoint(const Vector2& force, const Vector2& point) {
    forceAccumulator += force;
    
    Vector2 r = point - position;
    float torque = r.cross(force);
    addTorque(torque);
}

void RigidBody::addTorque(float torque) {
    torqueAccumulator += torque;
}

void RigidBody::clearForces() {
    forceAccumulator.x = 0;
    forceAccumulator.y = 0;
    torqueAccumulator = 0;
}

void RigidBody::integrate(float dt) {
    if (hasInfiniteMass()) return;
    
    acceleration = forceAccumulator * inverseMass;
    velocity += acceleration * dt;
    
    velocity *= 0.995f;
    
    if (velocity.magnitudeSquared() < 0.01f) {
        velocity.x = 0;
        velocity.y = 0;
    }
    
    position += velocity * dt;
    
    if (!hasInfiniteInertia()) {
        angularAcceleration = torqueAccumulator * inverseInertia;
        angularVelocity += angularAcceleration * dt;
        
        angularVelocity *= 0.95f;
        
        if (std::abs(angularVelocity) < 0.05f) {
            angularVelocity = 0.0f;
        }
        
        float maxAngularVelocity = 15.0f;
        if (std::abs(angularVelocity) > maxAngularVelocity) {
            angularVelocity = angularVelocity > 0 ? maxAngularVelocity : -maxAngularVelocity;
        }
        
        orientation += angularVelocity * dt;
    }
    
    clearForces();
}

std::vector<Vector2> RigidBody::getVertices() const {
    if (shapeType != ShapeType::BOX) return {};
    
    std::vector<Vector2> vertices(4);
    
    float hw = width * 0.5f;
    float hh = height * 0.5f;
    
    vertices[0] = Vector2(-hw, -hh);
    vertices[1] = Vector2(hw, -hh);
    vertices[2] = Vector2(hw, hh);
    vertices[3] = Vector2(-hw, hh);
    
    float cos_a = std::cos(orientation);
    float sin_a = std::sin(orientation);
    
    for (auto& v : vertices) {
        float x = v.x * cos_a - v.y * sin_a;
        float y = v.x * sin_a + v.y * cos_a;
        v.x = x + position.x;
        v.y = y + position.y;
    }
    
    return vertices;
}

std::vector<Vector2> RigidBody::getAxes() const {
    if (shapeType != ShapeType::BOX) return {};
    
    std::vector<Vector2> axes(2);
    
    float cos_a = std::cos(orientation);
    float sin_a = std::sin(orientation);
    
    axes[0] = Vector2(sin_a, -cos_a);
    axes[1] = Vector2(cos_a, sin_a);
    
    return axes;
}