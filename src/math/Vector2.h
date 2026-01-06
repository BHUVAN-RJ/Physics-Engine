#pragma once
#include <cmath>

class Vector2 {
public:
    float x, y;
    
    Vector2();
    Vector2(float x, float y);
    
    Vector2 operator+(const Vector2& v) const;
    Vector2 operator-(const Vector2& v) const;
    Vector2 operator*(float scalar) const;
    Vector2 operator/(float scalar) const;
    
    Vector2& operator+=(const Vector2& v);
    Vector2& operator-=(const Vector2& v);
    Vector2& operator*=(float scalar);
    
    float magnitude() const;
    float magnitudeSquared() const;  
    Vector2 normalize() const;
    void normalizeSelf();
    
    float dot(const Vector2& v) const;
    float cross(const Vector2& v) const;  
    
    static float distance(const Vector2& a, const Vector2& b);
    static float distanceSquared(const Vector2& a, const Vector2& b);
    static Vector2 rotate(const Vector2& v, float angle);
};