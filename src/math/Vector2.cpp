#include <Vector2.h>


Vector2::Vector2() : x(0.0f), y(0.0f) {}

Vector2::Vector2(float x, float y): x(x), y(y){}

Vector2 Vector2::operator+(const Vector2& v)const{
    return Vector2(x + v.x, y + v.y);
}

Vector2 Vector2::operator-(const Vector2& v)const{
    return Vector2(x - v.x, y - v.y);
}

Vector2 Vector2::operator*(float scaler)const{
    return Vector2(x * scaler, y * scaler);
}

Vector2 Vector2::operator/(float scaler)const{
    return Vector2(x / scaler, y / scaler);
}

Vector2& Vector2::operator+=(const Vector2& v){
    x += v.x;
    y += v.y;
    return *this;
}

Vector2& Vector2::operator-=(const Vector2& v){
    x -= v.x;
    y -= v.y;
    return *this;
}

Vector2& Vector2::operator*=(float scaler){
    x *= scaler;
    y *= scaler;
    return *this;
}

float Vector2::magnitude()const{
    return std::sqrt(x * x + y * y);

}

float Vector2::magnitudeSquared() const{
    return x * x + y * y;    
}

Vector2 Vector2::normalize() const {
    float mag = magnitude();
    if (mag > 0){
        return Vector2(x / mag, y / mag);
    }
    return Vector2(0, 0);
}

void Vector2::normalizeSelf() {
    float mag = magnitude();
    if (mag > 0) {
        x /= mag;
        y /= mag;
    }
}

float Vector2::dot(const Vector2& v) const {
    return x * v.x + y * v.y;
}

float Vector2::cross(const Vector2& v) const {
    return x * v.y - y * v.x;
}


float Vector2::distance(const Vector2& a, const Vector2& b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

float Vector2::distanceSquared(const Vector2& a, const Vector2& b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return dx * dx + dy * dy;
}

Vector2 Vector2::rotate(const Vector2& v, float angle) {
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    return Vector2(
        v.x * cos_a - v.y * sin_a,
        v.x * sin_a + v.y * cos_a
    );
}


