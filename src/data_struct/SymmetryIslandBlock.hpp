/**
 * SymmetryIslandBlock.hpp
 * 
 * This class represents a symmetry island as a single rectangular block
 * for use in the global B*-tree. It wraps an ASFBStarTree and presents it
 * as a unified block while maintaining internal symmetry.
 */

#pragma once

#include <memory>
#include <string>
#include "Module.hpp"
#include "ASFBStarTree.hpp"

class SymmetryIslandBlock {
private:
    std::string name;  // Name of the symmetry group
    std::shared_ptr<ASFBStarTree> asfTree;  // The ASF-B*-tree managing internal symmetry
    int width;   // Width of the bounding rectangle
    int height;  // Height of the bounding rectangle
    int x;       // X-coordinate of the lower-left corner in global placement
    int y;       // Y-coordinate of the lower-left corner in global placement
    
    // Cached original positions of modules before global placement
    std::map<std::string, std::pair<int, int>> originalPositions;
    
public:
    /**
     * Constructor
     * 
     * @param name Name of the symmetry group
     * @param asfTree ASF-B*-tree that manages internal symmetry
     */
    SymmetryIslandBlock(const std::string& name, std::shared_ptr<ASFBStarTree> asfTree)
        : name(name), asfTree(asfTree), width(0), height(0), x(0), y(0) {
        updateBoundingBox();
    }
    
    /**
     * Updates the bounding box dimensions based on the current internal layout
     */
    void updateBoundingBox() {
        // Pack the ASF-B*-tree to get the internal layout
        asfTree->pack();
        
        // Find the bounding rectangle
        int minX = std::numeric_limits<int>::max();
        int minY = std::numeric_limits<int>::max();
        int maxX = std::numeric_limits<int>::min();
        int maxY = std::numeric_limits<int>::min();
        
        for (const auto& pair : asfTree->getModules()) {
            const auto& module = pair.second;
            minX = std::min(minX, module->getX());
            minY = std::min(minY, module->getY());
            maxX = std::max(maxX, module->getX() + module->getWidth());
            maxY = std::max(maxY, module->getY() + module->getHeight());
        }
        
        // Update dimensions
        width = maxX - minX;
        height = maxY - minY;
        
        // Save original positions relative to island origin
        saveOriginalPositions(minX, minY);
    }
    
    /**
     * Saves the original positions of all modules relative to the island origin
     */
    void saveOriginalPositions(int originX, int originY) {
        originalPositions.clear();
        
        for (const auto& pair : asfTree->getModules()) {
            const auto& module = pair.second;
            // Store positions relative to origin
            originalPositions[pair.first] = {
                module->getX() - originX,
                module->getY() - originY
            };
        }
    }
    
    /**
     * Updates the positions of all internal modules based on global position
     */
    void updateModulePositions() {
        for (const auto& pair : asfTree->getModules()) {
            const auto& moduleName = pair.first;
            auto& module = pair.second;
            
            // Get relative position
            const auto& relPos = originalPositions[moduleName];
            
            // Set absolute position
            module->setPosition(x + relPos.first, y + relPos.second);
        }
        
        // Update symmetry axis position
        auto symmetryGroup = asfTree->getSymmetryGroup();
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            double relativeAxis = asfTree->getSymmetryAxisPosition() - originalPositions.begin()->second.first;
            symmetryGroup->setAxisPosition(x + relativeAxis);
        } else {
            double relativeAxis = asfTree->getSymmetryAxisPosition() - originalPositions.begin()->second.second;
            symmetryGroup->setAxisPosition(y + relativeAxis);
        }
    }
    
    // Getters
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    int getX() const { return x; }
    int getY() const { return y; }
    std::string getName() const { return name; }
    std::shared_ptr<ASFBStarTree> getASFBStarTree() const { return asfTree; }
    
    // Setters
    void setPosition(int x, int y) {
        this->x = x;
        this->y = y;
        updateModulePositions();
    }
    
    /**
     * Rotates the entire island by 90 degrees
     */
    void rotate() {
        // Swap width and height
        std::swap(width, height);
        
        // Swap all internal module positions and dimensions
        for (auto& pair : originalPositions) {
            // Swap x and y in the relative positions
            std::swap(pair.second.first, pair.second.second);
        }
        
        // Rotate each module
        for (auto& pair : asfTree->getModules()) {
            pair.second->rotate();
        }
        
        // Toggle symmetry type
        auto symmetryGroup = asfTree->getSymmetryGroup();
        symmetryGroup->setType(
            symmetryGroup->getType() == SymmetryType::VERTICAL ? 
            SymmetryType::HORIZONTAL : SymmetryType::VERTICAL
        );
        
        // Update internal layout
        asfTree->pack();
        updateBoundingBox();
    }
    
    /**
     * Gets the area of the island
     */
    int getArea() const {
        return width * height;
    }
    
    /**
     * Checks if this island overlaps with another island
     */
    bool overlaps(const SymmetryIslandBlock& other) const {
        return !(x + width <= other.x || other.x + other.width <= x ||
                y + height <= other.y || other.y + other.height <= y);
    }
    
    /**
     * Checks if this island overlaps with a module
     */
    bool overlaps(const Module& module) const {
        return !(x + width <= module.getX() || module.getX() + module.getWidth() <= x ||
                y + height <= module.getY() || module.getY() + module.getHeight() <= y);
    }
};