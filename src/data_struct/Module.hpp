#pragma once
#include <string>
#include <vector>
#include <memory>

class Module {
private:
    std::string name;     // Name of the module/block
    int width;            // Original width of the module
    int height;           // Original height of the module
    int x;                // x-coordinate of the lower-left corner
    int y;                // y-coordinate of the lower-left corner
    bool isRotated;       // Rotation status (true for rotated, false for unrotated)
    
public:
    // Constructors
    Module(const std::string& name, int width, int height);
    Module(const Module& other); // Copy constructor

    /**
     * Checks if this module overlaps with another module
     * 
     * @param other The other module to check for overlap
     * @return True if the modules overlap, false otherwise
     */
    bool overlaps(const Module& other) const;
    
    // Getters
    const std::string& getName() const;
    int getWidth() const;        // Returns effective width (accounting for rotation)
    int getHeight() const;       // Returns effective height (accounting for rotation)
    int getOriginalWidth() const; // Returns original width regardless of rotation
    int getOriginalHeight() const; // Returns original height regardless of rotation
    int getX() const;
    int getY() const;
    bool getRotated() const;
    
    // Setters
    void setPosition(int x, int y);
    void rotate();  // Toggle rotation status
    void setRotation(bool rotate); // Set specific rotation status
    
    // Utility functions
    int getArea() const;
    
    // Boundary points
    int getRight() const; // x + width
    int getTop() const;   // y + height
    
    // Visualization
    void print() const;
};