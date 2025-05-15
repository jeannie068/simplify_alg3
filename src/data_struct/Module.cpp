#include "Module.hpp"
#include <iostream>
#include <algorithm>



// Constructors
Module::Module(const std::string& name, int width, int height)
    : name(name), width(width), height(height), x(0), y(0), isRotated(false) {
}

Module::Module(const Module& other)
    : name(other.name), width(other.width), height(other.height),
      x(other.x), y(other.y), isRotated(other.isRotated) {
}

bool Module::overlaps(const Module& other) const {
    // Check if two modules overlap
    if (x + getWidth() <= other.x || other.x + other.getWidth() <= x) {
        return false; // No horizontal overlap
    }
    if (y + getHeight() <= other.y || other.y + other.getHeight() <= y) {
        return false; // No vertical overlap
    }
    return true; // There is overlap
}

// Getters
const std::string& Module::getName() const {
    return name;
}

int Module::getWidth() const {
    return isRotated ? height : width;
}

int Module::getHeight() const {
    return isRotated ? width : height;
}

int Module::getOriginalWidth() const {
    return width;
}

int Module::getOriginalHeight() const {
    return height;
}

int Module::getX() const {
    return x;
}

int Module::getY() const {
    return y;
}

bool Module::getRotated() const {
    return isRotated;
}

// Setters
void Module::setPosition(int x, int y) {
    this->x = x;
    this->y = y;
}

void Module::rotate() {
    isRotated = !isRotated;
}

void Module::setRotation(bool rotate) {
    isRotated = rotate;
}

// Utility functions
int Module::getArea() const {
    return width * height; // Area doesn't change with rotation
}

int Module::getRight() const {
    return x + getWidth();
}

int Module::getTop() const {
    return y + getHeight();
}

void Module::print() const {
    std::cout << "Module: " << name << std::endl;
    std::cout << "  Position: (" << x << ", " << y << ")" << std::endl;
    std::cout << "  Dimensions: " << getWidth() << " x " << getHeight() << std::endl;
    std::cout << "  Rotated: " << (isRotated ? "Yes" : "No") << std::endl;
}