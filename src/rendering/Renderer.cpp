#include "Renderer.h"
#include "RigidBody.h"

void Renderer::drawRigidBody(const RigidBody& body, sf::Color color) {
    if (body.shapeType == ShapeType::CIRCLE) {
        sf::CircleShape circle(body.radius);
        circle.setPosition(body.position.x - body.radius, 
                          body.position.y - body.radius);
        circle.setFillColor(color);
        window.draw(circle);
        
        Vector2 dir(std::cos(body.orientation), std::sin(body.orientation));
        Vector2 end = body.position + dir * body.radius;
        drawLine(body.position, end, sf::Color::Red);
        
    } else if (body.shapeType == ShapeType::BOX) {
        auto vertices = body.getVertices();
        
        sf::ConvexShape shape;
        shape.setPointCount(4);
        for (int i = 0; i < 4; i++) {
            shape.setPoint(i, sf::Vector2f(vertices[i].x, vertices[i].y));
        }
        shape.setFillColor(color);
        window.draw(shape);
    }
}
Renderer::Renderer(int width, int height, const std::string& title)
    : width(width), height(height) {
    window.create(sf::VideoMode(width, height), title);
    window.setFramerateLimit(60);  
}

bool Renderer::isOpen() const {
    return window.isOpen();
}

void Renderer::clear() {
    window.clear(sf::Color::Black);
}

void Renderer::display() {
    window.display();
}

void Renderer::handleEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
        }
        if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
            window.close();
        }
    }
}

void Renderer::drawCollisionIndicator(const Vector2& position, float radius) {
    sf::CircleShape circle(radius);
    circle.setPosition(position.x - radius, position.y - radius);
    circle.setFillColor(sf::Color::Transparent);
    circle.setOutlineColor(sf::Color::Red);
    circle.setOutlineThickness(2.0f);
    window.draw(circle);
}

void Renderer::drawParticle(const Particle& particle, sf::Color color) {
    sf::CircleShape circle(particle.radius);
    circle.setPosition(particle.position.x - particle.radius, 
                       particle.position.y - particle.radius);
    circle.setFillColor(color);
    window.draw(circle);
}

void Renderer::drawCircle(const Vector2& position, float radius, sf::Color color) {
    sf::CircleShape circle(radius);
    circle.setPosition(position.x - radius, position.y - radius);
    circle.setFillColor(color);
    window.draw(circle);
}

void Renderer::drawLine(const Vector2& start, const Vector2& end, sf::Color color) {
    sf::Vertex line[] = {
        sf::Vertex(sf::Vector2f(start.x, start.y), color),
        sf::Vertex(sf::Vector2f(end.x, end.y), color)
    };
    window.draw(line, 2, sf::Lines);
}

Vector2 Renderer::getMousePosition() const {
    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
    return Vector2(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));
}

bool Renderer::isMouseButtonPressed(int button) const {
    if (button == 0) return sf::Mouse::isButtonPressed(sf::Mouse::Left);
    if (button == 1) return sf::Mouse::isButtonPressed(sf::Mouse::Right);
    return false;
}

