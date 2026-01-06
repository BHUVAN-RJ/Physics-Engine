#include <iostream>
#include <random>
#include <memory>
#include <cmath>
#include "Vector2.h"
#include "RigidBody.h"
#include "Renderer.h"
#include "Collision.h"
#include "RigidBodyResolver.h"

enum class DemoMode {
    SANDBOX,           
    CIRCLE_FOUNTAIN,   
    BOX_TOWER,        
    POOL_TABLE,       
    NEWTON_CRADLE,    
    EXPLOSION         
};

void spawnCircleFountain(std::vector<RigidBody>& bodies, const Vector2& pos, int count) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> angleDist(0, 2 * M_PI);
    std::uniform_real_distribution<float> speedDist(100, 300);
    std::uniform_real_distribution<float> radiusDist(8, 15);
    
    for (int i = 0; i < count; i++) {
        float angle = angleDist(gen);
        float speed = speedDist(gen);
        float radius = radiusDist(gen);
        
        auto circle = RigidBody::createCircle(pos, radius, 1.0f);
        circle.velocity = Vector2(std::cos(angle), std::sin(angle)) * speed;
        circle.restitution = 0.6f;
        circle.friction = 0.3f;
        bodies.push_back(circle);
    }
}

void spawnBoxTower(std::vector<RigidBody>& bodies, const Vector2& basePos, int height, float boxSize) {
    for (int i = 0; i < height; i++) {
        auto box = RigidBody::createBox(
            Vector2(basePos.x, basePos.y - i * (boxSize + 2)), 
            boxSize, boxSize, 2.0f
        );
        box.restitution = 0.3f;
        box.friction = 0.6f;
        bodies.push_back(box);
    }
}

void spawnPoolTable(std::vector<RigidBody>& bodies, const Vector2& center) {
    float radius = 12.0f;
    float spacing = radius * 2.2f;
    
    for (int row = 0; row < 5; row++) {
        for (int col = 0; col <= row; col++) {
            float x = center.x + (col - row / 2.0f) * spacing;
            float y = center.y - row * spacing * 0.866f;  
            
            auto ball = RigidBody::createCircle(Vector2(x, y), radius, 1.0f);
            ball.restitution = 0.9f; 
            ball.friction = 0.1f;      
            bodies.push_back(ball);
        }
    }
    
    auto cueBall = RigidBody::createCircle(Vector2(center.x, center.y + 150), radius, 1.0f);
    cueBall.restitution = 0.9f;
    cueBall.friction = 0.1f;
    cueBall.velocity = Vector2(0, -400);  
    bodies.push_back(cueBall);
}

void spawnNewtonCradle(std::vector<RigidBody>& bodies, const Vector2& center, int count) {
    float radius = 15.0f;
    float spacing = radius * 2.1f;
    
    for (int i = 0; i < count; i++) {
        float x = center.x + (i - count / 2.0f) * spacing;
        auto ball = RigidBody::createCircle(Vector2(x, center.y), radius, 1.0f);
        ball.restitution = 0.99f; 
        ball.friction = 0.0f;
        bodies.push_back(ball);
    }
    
    if (!bodies.empty()) {
        bodies[bodies.size() - count].velocity = Vector2(200, 0);
    }
}

void spawnExplosion(std::vector<RigidBody>& bodies, const Vector2& center, int count) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> angleDist(0, 2 * M_PI);
    std::uniform_real_distribution<float> speedDist(200, 500);
    std::uniform_real_distribution<float> sizeDist(10, 20);
    
    for (int i = 0; i < count; i++) {
        float angle = angleDist(gen);
        float speed = speedDist(gen);
        float size = sizeDist(gen);
        
        if (rand() % 2 == 0) {
            auto circle = RigidBody::createCircle(center, size * 0.5f, 1.5f);
            circle.velocity = Vector2(std::cos(angle), std::sin(angle)) * speed;
            circle.angularVelocity = (rand() % 100 - 50) * 0.1f;
            circle.restitution = 0.7f;
            circle.friction = 0.4f;
            bodies.push_back(circle);
        } else {
            auto box = RigidBody::createBox(center, size, size, 2.0f);
            box.velocity = Vector2(std::cos(angle), std::sin(angle)) * speed;
            box.angularVelocity = (rand() % 100 - 50) * 0.1f;
            box.restitution = 0.7f;
            box.friction = 0.4f;
            bodies.push_back(box);
        }
    }
}

int main() {
    const int SCREEN_WIDTH = 800;
    const int SCREEN_HEIGHT = 600;
    
    Renderer renderer(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics Engine - Fun Demos!");
    
    std::vector<RigidBody> bodies;
    RigidBodyResolver resolver;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> xDist(100, 700);
    std::uniform_real_distribution<float> sizeDist(15, 30);
    
    sf::Clock clock;
    float spawnTimer = 0.0f;
    
    Vector2 gravity(0, 400.0f);
    DemoMode currentMode = DemoMode::SANDBOX;
    bool autoSpawn = false;
    
    std::cout << "\n=== PHYSICS ENGINE - FUN DEMOS ===" << std::endl;
    std::cout << "\nControls:" << std::endl;
    std::cout << "  1 - Sandbox Mode (manual spawn)" << std::endl;
    std::cout << "  2 - Circle Fountain" << std::endl;
    std::cout << "  3 - Box Tower Builder" << std::endl;
    std::cout << "  4 - Pool Table" << std::endl;
    std::cout << "  5 - Newton's Cradle" << std::endl;
    std::cout << "  6 - Explosion" << std::endl;
    std::cout << "\n  SPACE - Spawn Box (sandbox)" << std::endl;
    std::cout << "  C - Spawn Circle (sandbox)" << std::endl;
    std::cout << "  Mouse Click - Create explosion at cursor" << std::endl;
    std::cout << "  R - Clear all" << std::endl;
    std::cout << "  G - Toggle gravity" << std::endl;
    std::cout << "  ESC - Exit\n" << std::endl;
    
    bool gravityEnabled = true;
    
    auto& window = renderer.getWindow();
    
    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        dt = std::min(dt, 0.02f);
        
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Escape) {
                    window.close();
                }
                else if (event.key.code == sf::Keyboard::Num1) {
                    currentMode = DemoMode::SANDBOX;
                    autoSpawn = false;
                    std::cout << "Mode: SANDBOX" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::Num2) {
                    currentMode = DemoMode::CIRCLE_FOUNTAIN;
                    bodies.clear();
                    autoSpawn = true;
                    std::cout << "Mode: CIRCLE FOUNTAIN" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::Num3) {
                    currentMode = DemoMode::BOX_TOWER;
                    bodies.clear();
                    spawnBoxTower(bodies, Vector2(400, 550), 12, 30);
                    autoSpawn = false;
                    std::cout << "Mode: BOX TOWER" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::Num4) {
                    currentMode = DemoMode::POOL_TABLE;
                    bodies.clear();
                    spawnPoolTable(bodies, Vector2(400, 250));
                    autoSpawn = false;
                    std::cout << "Mode: POOL TABLE" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::Num5) {
                    currentMode = DemoMode::NEWTON_CRADLE;
                    bodies.clear();
                    spawnNewtonCradle(bodies, Vector2(400, 300), 5);
                    autoSpawn = false;
                    gravityEnabled = false;
                    std::cout << "Mode: NEWTON'S CRADLE (gravity off)" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::Num6) {
                    currentMode = DemoMode::EXPLOSION;
                    bodies.clear();
                    spawnExplosion(bodies, Vector2(400, 300), 30);
                    autoSpawn = false;
                    std::cout << "Mode: EXPLOSION" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::Space) {
                    if (currentMode == DemoMode::SANDBOX) {
                        float x = xDist(gen);
                        float size = sizeDist(gen);
                        auto box = RigidBody::createBox(Vector2(x, 100), size, size, 1.5f);
                        box.restitution = 0.4f;
                        box.friction = 0.5f;
                        box.angularVelocity = (rand() % 100 - 50) * 0.02f;
                        bodies.push_back(box);
                    }
                }
                else if (event.key.code == sf::Keyboard::C) {
                    if (currentMode == DemoMode::SANDBOX) {
                        float x = xDist(gen);
                        float radius = sizeDist(gen) * 0.5f;
                        auto circle = RigidBody::createCircle(Vector2(x, 100), radius, 1.0f);
                        circle.restitution = 0.4f;
                        circle.friction = 0.5f;
                        bodies.push_back(circle);
                    }
                }
                else if (event.key.code == sf::Keyboard::R) {
                    bodies.clear();
                    std::cout << "Cleared all bodies" << std::endl;
                }
                else if (event.key.code == sf::Keyboard::G) {
                    gravityEnabled = !gravityEnabled;
                    std::cout << "Gravity: " << (gravityEnabled ? "ON" : "OFF") << std::endl;
                }
            }
            
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    Vector2 mousePos = renderer.getMousePosition();
                    spawnExplosion(bodies, mousePos, 20);
                    std::cout << "Explosion at mouse!" << std::endl;
                }
            }
        }
        
        if (autoSpawn && currentMode == DemoMode::CIRCLE_FOUNTAIN && bodies.size() < 50) {
            spawnTimer += dt;
            if (spawnTimer > 0.2f) {
                spawnCircleFountain(bodies, Vector2(400, 50), 3);
                spawnTimer = 0.0f;
            }
        }
        
        if (gravityEnabled) {
            for (auto& body : bodies) {
                if (!body.hasInfiniteMass()) {
                    body.addForce(gravity * body.mass);
                }
            }
        }
        
        for (auto& body : bodies) {
            body.integrate(dt);
        }
        
        for (int iteration = 0; iteration < 2; iteration++) {
            std::vector<RigidContact> contacts;
            
            for (size_t i = 0; i < bodies.size(); i++) {
                for (size_t j = i + 1; j < bodies.size(); j++) {
                    auto contact = CollisionDetector::generateRigidContact(bodies[i], bodies[j]);
                    if (contact) {
                        contacts.push_back(*contact);
                        delete contact;
                    }
                }
            }
            
            resolver.resolveContacts(contacts);
        }
        
        for (auto& body : bodies) {
            float restitution = 0.5f;
            
            if (body.shapeType == ShapeType::CIRCLE) {
                if (body.position.y + body.radius > SCREEN_HEIGHT) {
                    body.position.y = SCREEN_HEIGHT - body.radius;
                    body.velocity.y *= -restitution;
                    body.velocity.x *= 0.95f;
                    body.angularVelocity *= 0.95f;
                }
                
                if (body.position.y - body.radius < 0) {
                    body.position.y = body.radius;
                    body.velocity.y *= -restitution;
                }
                
                if (body.position.x - body.radius < 0) {
                    body.position.x = body.radius;
                    body.velocity.x *= -restitution;
                }
                if (body.position.x + body.radius > SCREEN_WIDTH) {
                    body.position.x = SCREEN_WIDTH - body.radius;
                    body.velocity.x *= -restitution;
                }
            } else {
                float maxRadius = std::max(body.width, body.height) * 0.7f;
                
                if (body.position.y + maxRadius > SCREEN_HEIGHT) {
                    body.position.y = SCREEN_HEIGHT - maxRadius;
                    body.velocity.y *= -restitution;
                    body.velocity.x *= 0.95f;
                    body.angularVelocity *= 0.95f;
                }
                
                if (body.position.y - maxRadius < 0) {
                    body.position.y = maxRadius;
                    body.velocity.y *= -restitution;
                }
                
                if (body.position.x - maxRadius < 0) {
                    body.position.x = maxRadius;
                    body.velocity.x *= -restitution;
                }
                if (body.position.x + maxRadius > SCREEN_WIDTH) {
                    body.position.x = SCREEN_WIDTH - maxRadius;
                    body.velocity.x *= -restitution;
                }
            }
        }
        
        bodies.erase(
            std::remove_if(bodies.begin(), bodies.end(),
                [SCREEN_HEIGHT](const RigidBody& b) {
                    return b.position.y > SCREEN_HEIGHT + 200;
                }),
            bodies.end()
        );
        
        renderer.clear();
        
        for (const auto& body : bodies) {
            sf::Color color;
            if (body.shapeType == ShapeType::CIRCLE) {
                color = sf::Color(100, 150, 255);
            } else {
                color = sf::Color::White;
            }
            renderer.drawRigidBody(body, color);
        }
        
        renderer.display();
    }
    
    return 0;
}