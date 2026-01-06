#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include "Vector2.h"
#include "Particle.h"
#include "RigidBody.h"

class Renderer {
private:
    sf::RenderWindow window;
    int width;
    int height;
    
public:
    Renderer(int width, int height, const std::string& title);
    
    bool isOpen() const;
    void clear();
    void display();
    void handleEvents();
    
    void drawParticle(const Particle& particle, sf::Color color = sf::Color::White);
    void drawCircle(const Vector2& position, float radius, sf::Color color = sf::Color::White);
    void drawLine(const Vector2& start, const Vector2& end, sf::Color color = sf::Color::White);
    void drawText(const std::string& text, const Vector2& position, int size = 20);
    
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    sf::RenderWindow& getWindow() { return window; }



    Vector2 getMousePosition() const;
    bool isMouseButtonPressed(int button) const;
    void drawCollisionIndicator(const Vector2& position, float radius);
    void drawRigidBody(const RigidBody& body, sf::Color color = sf::Color::White);


};