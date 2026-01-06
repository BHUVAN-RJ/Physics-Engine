#include "Collision.h"
#include <algorithm>
#include <cmath>

bool AABB::intersects(const AABB& other) const {
    return !(max.x < other.min.x || min.x > other.max.x ||
             max.y < other.min.y || min.y > other.max.y);
}

bool AABB::contains(const Vector2& point) const {
    return point.x >= min.x && point.x <= max.x &&
           point.y >= min.y && point.y <= max.y;
}

AABB AABB::fromParticle(const Particle& particle) {
    Vector2 min(particle.position.x - particle.radius,
                particle.position.y - particle.radius);
    Vector2 max(particle.position.x + particle.radius,
                particle.position.y + particle.radius);
    return AABB(min, max);
}

bool CollisionDetector::checkCollision(const Particle& a, const Particle& b) {
    float distance = Vector2::distance(a.position, b.position);
    float radiusSum = a.radius + b.radius;
    return distance < radiusSum;
}

Contact* CollisionDetector::generateContact(Particle& a, Particle& b) {
    Vector2 diff = b.position - a.position;
    float distance = diff.magnitude();
    float radiusSum = a.radius + b.radius;
    
    if (distance >= radiusSum) {
        return nullptr;  
    }
    
    Vector2 normal;
    if (distance > 0) {
        normal = diff / distance;
    } else {
        normal = Vector2(1, 0);
    }
    
    float penetration = radiusSum - distance;
    
    return new Contact(&a, &b, normal, penetration);
}

bool CollisionDetector::checkAABBCollision(const AABB& a, const AABB& b) {
    return a.intersects(b);
}

std::vector<Contact> CollisionDetector::detectCollisions(std::vector<Particle>& particles) {
    std::vector<Contact> contacts;
    
    for (size_t i = 0; i < particles.size(); i++) {
        for (size_t j = i + 1; j < particles.size(); j++) {
            Contact* contact = generateContact(particles[i], particles[j]);
            if (contact) {
                contacts.push_back(*contact);
                delete contact;
            }
        }
    }
    
    return contacts;
}

std::vector<Contact> CollisionDetector::detectCollisionsBroadPhase(std::vector<Particle>& particles) {
    std::vector<Contact> contacts;
    
    std::vector<AABB> aabbs;
    for (const auto& particle : particles) {
        aabbs.push_back(AABB::fromParticle(particle));
    }
    
    for (size_t i = 0; i < particles.size(); i++) {
        for (size_t j = i + 1; j < particles.size(); j++) {
            if (checkAABBCollision(aabbs[i], aabbs[j])) {
                Contact* contact = generateContact(particles[i], particles[j]);
                if (contact) {
                    contacts.push_back(*contact);
                    delete contact;
                }
            }
        }
    }
    
    return contacts;
}

SpatialGrid::SpatialGrid(int screenWidth, int screenHeight, int cellSize)
    : cellSize(cellSize) {
    gridWidth = (screenWidth / cellSize) + 1;
    gridHeight = (screenHeight / cellSize) + 1;
    grid.resize(gridWidth, std::vector<std::vector<Particle*>>(gridHeight));
}

void SpatialGrid::clear() {
    for (auto& col : grid) {
        for (auto& cell : col) {
            cell.clear();
        }
    }
}

int SpatialGrid::getGridX(float x) const {
    int gx = static_cast<int>(x / cellSize);
    return std::max(0, std::min(gx, gridWidth - 1));
}

int SpatialGrid::getGridY(float y) const {
    int gy = static_cast<int>(y / cellSize);
    return std::max(0, std::min(gy, gridHeight - 1));
}

void SpatialGrid::insert(Particle& particle) {
    int gx = getGridX(particle.position.x);
    int gy = getGridY(particle.position.y);
    grid[gx][gy].push_back(&particle);
}

std::vector<Particle*> SpatialGrid::query(const Particle& particle) {
    std::vector<Particle*> nearby;
    
    int gx = getGridX(particle.position.x);
    int gy = getGridY(particle.position.y);
    
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            int nx = gx + dx;
            int ny = gy + dy;
            
            if (nx >= 0 && nx < gridWidth && ny >= 0 && ny < gridHeight) {
                for (auto* p : grid[nx][ny]) {
                    nearby.push_back(p);
                }
            }
        }
    }
    
    return nearby;
}


bool CollisionDetector::checkCollision(const RigidBody& a, const RigidBody& b) {
    if (a.shapeType == ShapeType::CIRCLE && b.shapeType == ShapeType::CIRCLE) {
        float distance = Vector2::distance(a.position, b.position);
        return distance < (a.radius + b.radius);
    } else if (a.shapeType == ShapeType::CIRCLE && b.shapeType == ShapeType::BOX) {
        return checkCircleBoxCollision(a, b);
    } else if (a.shapeType == ShapeType::BOX && b.shapeType == ShapeType::CIRCLE) {
        return checkCircleBoxCollision(b, a);
    } else {
        return checkBoxBoxCollision(a, b);
    }
}

RigidContact* CollisionDetector::generateRigidContact(RigidBody& a, RigidBody& b) {
    if (a.shapeType == ShapeType::CIRCLE && b.shapeType == ShapeType::CIRCLE) {
        Vector2 delta = b.position - a.position;
        float distance = delta.magnitude();
        float radiusSum = a.radius + b.radius;
        
        if (distance >= radiusSum) return nullptr;
        
        Vector2 normal;
        if (distance > 0.001f) {
            normal = delta / distance;
        } else {
            normal = Vector2(0, -1);
        }
        
        float penetration = radiusSum - distance;
        Vector2 contactPoint = a.position + normal * a.radius;
        
        return new RigidContact(&a, &b, contactPoint, normal, penetration);
    }
    
    if ((a.shapeType == ShapeType::CIRCLE && b.shapeType == ShapeType::BOX) ||
        (a.shapeType == ShapeType::BOX && b.shapeType == ShapeType::CIRCLE)) {
        return nullptr; 
    }
    
    if (a.shapeType == ShapeType::BOX && b.shapeType == ShapeType::BOX) {
        return generateBoxBoxContact(a, b);
    }
    
    return nullptr;
}

bool CollisionDetector::checkCircleBoxCollision(const RigidBody& circle, const RigidBody& box) {
    auto vertices = box.getVertices();
    if (vertices.empty()) return false;
    
    for (size_t i = 0; i < 4; i++) {
        Vector2 p1 = vertices[i];
        Vector2 p2 = vertices[(i + 1) % 4];
        
        Vector2 edge = p2 - p1;
        float edgeLength = edge.magnitude();
        
        if (edgeLength < 0.001f) continue;
        
        Vector2 toCircle = circle.position - p1;
        
        float t = toCircle.dot(edge) / (edgeLength * edgeLength);
        t = std::max(0.0f, std::min(1.0f, t));  
        
        Vector2 closest = p1 + edge * t;
        
        float distance = Vector2::distance(circle.position, closest);
        
        if (distance < circle.radius) {
            return true;
        }
    }
    
    return false;
}

RigidContact* CollisionDetector::generateCircleBoxContact(RigidBody& circle, RigidBody& box) {
    auto vertices = box.getVertices();
    if (vertices.empty()) return nullptr;
    
    float minDistance = std::numeric_limits<float>::max();
    Vector2 closestPoint;
    Vector2 bestNormal;
    bool found = false;
    
    for (size_t i = 0; i < 4; i++) {
        Vector2 p1 = vertices[i];
        Vector2 p2 = vertices[(i + 1) % 4];
        
        Vector2 edge = p2 - p1;
        float edgeLength = edge.magnitude();
        
        if (edgeLength < 0.001f) continue;
        
        Vector2 toCircle = circle.position - p1;
        
        float t = toCircle.dot(edge) / (edgeLength * edgeLength);
        t = std::max(0.0f, std::min(1.0f, t));
        
        Vector2 pointOnEdge = p1 + edge * t;
        
        float distance = Vector2::distance(circle.position, pointOnEdge);
        
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = pointOnEdge;
            
            Vector2 diff = circle.position - pointOnEdge;
            if (diff.magnitude() > 0.001f) {
                bestNormal = diff.normalize();
            } else {
                Vector2 edgeNorm = edge.normalize();
                bestNormal = Vector2(-edgeNorm.y, edgeNorm.x);
            }
            
            found = true;
        }
    }
    
    if (!found) return nullptr;
    
    if (minDistance >= circle.radius) return nullptr;
    
    float penetration = circle.radius - minDistance;
    Vector2 contactPoint = closestPoint;
    
    return new RigidContact(&circle, &box, contactPoint, bestNormal, penetration);
}

bool CollisionDetector::checkBoxBoxCollision(const RigidBody& a, const RigidBody& b) {
    float radiusA = std::max(a.width, a.height) * 0.7f;
    float radiusB = std::max(b.width, b.height) * 0.7f;
    
    float distance = Vector2::distance(a.position, b.position);
    return distance < (radiusA + radiusB);
}

RigidContact* CollisionDetector::generateBoxBoxContact(RigidBody& a, RigidBody& b) {
    Vector2 delta = b.position - a.position;
    float distance = delta.magnitude();
    
    if (distance < 0.001f) return nullptr;
    
    Vector2 normal = delta / distance;
    
    float radiusA = std::max(a.width, a.height) * 0.5f;
    float radiusB = std::max(b.width, b.height) * 0.5f;
    float penetration = (radiusA + radiusB) - distance;
    
    if (penetration <= 0) return nullptr;
    
    Vector2 contactPoint = a.position + normal * radiusA;
    
    return new RigidContact(&a, &b, contactPoint, normal, penetration);
}