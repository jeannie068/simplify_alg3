// SymmetryConstraint.cpp
#include "SymmetryConstraint.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
using namespace std;

// Constructor
SymmetryGroup::SymmetryGroup(const string& name, SymmetryType type)
    : name(name), type(type), axisPosition(-1) {}

// Module management functions
void SymmetryGroup::addSymmetryPair(const string& module1, const string& module2) {
    symmetryPairs.push_back(make_pair(module1, module2));
    
    // Update lookup structures for O(1) access
    pairMap[module1] = module2;
    pairMap[module2] = module1;
    allModules.insert(module1);
    allModules.insert(module2);
}

void SymmetryGroup::addSelfSymmetric(const string& module) {
    selfSymmetric.push_back(module);
    
    // Update lookup structures for O(1) access
    selfSymSet.insert(module);
    allModules.insert(module);
}

// Query functions with O(1) complexity
bool SymmetryGroup::isInGroup(const string& moduleName) const {
    return allModules.find(moduleName) != allModules.end();
}

bool SymmetryGroup::isSelfSymmetric(const string& moduleName) const {
    return selfSymSet.find(moduleName) != selfSymSet.end();
}

bool SymmetryGroup::isSymmetryPair(const string& module1, const string& module2) const {
    auto it = pairMap.find(module1);
    return (it != pairMap.end() && it->second == module2);
}

string SymmetryGroup::getSymmetricPair(const string& moduleName) const {
    auto it = pairMap.find(moduleName);
    return (it != pairMap.end()) ? it->second : "";
}

// Determine if modules form a connected symmetry island
bool SymmetryGroup::isSymmetryIsland(const unordered_map<string, pair<int, int>>& positions,
                                    const unordered_map<string, pair<int, int>>& dimensions) const {
    if (allModules.empty()) return true;
    
    // Check if all modules are present
    for (const auto& module : allModules) {
        if (positions.find(module) == positions.end() || 
            dimensions.find(module) == dimensions.end()) {
            return false;
        }
    }
    
    // Check connectivity using BFS
    unordered_set<string> visited;
    queue<string> queue;
    
    // Start with the first module
    auto it = allModules.begin();
    queue.push(*it);
    visited.insert(*it);
    
    // Helper lambda to check if two modules are adjacent
    auto isAdjacent = [&](const string& m1, const string& m2) -> bool {
        const auto& pos1 = positions.at(m1);
        const auto& pos2 = positions.at(m2);
        const auto& dim1 = dimensions.at(m1);
        const auto& dim2 = dimensions.at(m2);
        
        int x1_left = pos1.first;
        int x1_right = pos1.first + dim1.first;
        int y1_bottom = pos1.second;
        int y1_top = pos1.second + dim1.second;
        
        int x2_left = pos2.first;
        int x2_right = pos2.first + dim2.first;
        int y2_bottom = pos2.second;
        int y2_top = pos2.second + dim2.second;
        
        // Check if modules touch horizontally
        bool horizontal_touch = (x1_right == x2_left || x2_right == x1_left) &&
                               !(y1_top <= y2_bottom || y2_top <= y1_bottom);
        
        // Check if modules touch vertically
        bool vertical_touch = (y1_top == y2_bottom || y2_top == y1_bottom) &&
                             !(x1_right <= x2_left || x2_right <= x1_left);
        
        return horizontal_touch || vertical_touch;
    };
    
    // BFS to check connectivity
    while (!queue.empty()) {
        string current = queue.front();
        queue.pop();
        
        for (const auto& module : allModules) {
            if (visited.find(module) == visited.end() && isAdjacent(current, module)) {
                visited.insert(module);
                queue.push(module);
            }
        }
    }
    
    // If all modules are visited, they form a connected group
    return visited.size() == allModules.size();
}

// Getters
string SymmetryGroup::getName() const {
    return name;
}

const vector<pair<string, string>>& SymmetryGroup::getSymmetryPairs() const {
    return symmetryPairs;
}

const vector<string>& SymmetryGroup::getSelfSymmetric() const {
    return selfSymmetric;
}

SymmetryType SymmetryGroup::getType() const {
    return type;
}

int SymmetryGroup::getNumModules() const {
    return allModules.size();
}

int SymmetryGroup::getNumPairs() const {
    return symmetryPairs.size();
}

int SymmetryGroup::getNumSelfSymmetric() const {
    return selfSymmetric.size();
}

double SymmetryGroup::getAxisPosition() const {
    return axisPosition;
}

const unordered_set<string>& SymmetryGroup::getAllModules() const {
    return allModules;
}

// Setters
void SymmetryGroup::setAxisPosition(double position) {
    axisPosition = position;
}

void SymmetryGroup::setType(SymmetryType newType) {
    type = newType;
}

// Actions
void SymmetryGroup::changeSymmetryType() {
    type = (type == SymmetryType::VERTICAL) ? SymmetryType::HORIZONTAL : SymmetryType::VERTICAL;
}

// Validation and utility functions
bool SymmetryGroup::validateSymmetricPlacement(
    const unordered_map<string, pair<int, int>>& positions,
    const unordered_map<string, pair<int, int>>& dimensions) const {
    
    // Calculate symmetry axis if not set
    double axis = (axisPosition < 0) ? calculateAxisPosition(positions) : axisPosition;
    if (axis < 0) return false; // Unable to determine axis
    
    // Check symmetry pairs
    for (const auto& pair : symmetryPairs) {
        auto it1 = positions.find(pair.first);
        auto it2 = positions.find(pair.second);
        auto dim1 = dimensions.find(pair.first);
        auto dim2 = dimensions.find(pair.second);
        
        if (it1 == positions.end() || it2 == positions.end() ||
            dim1 == dimensions.end() || dim2 == dimensions.end()) {
            return false; // Module not found
        }
        
        const auto& pos1 = it1->second;
        const auto& pos2 = it2->second;
        const auto& size1 = dim1->second;
        const auto& size2 = dim2->second;
        
        // Center coordinates
        double center1_x = pos1.first + size1.first / 2.0;
        double center1_y = pos1.second + size1.second / 2.0;
        double center2_x = pos2.first + size2.first / 2.0;
        double center2_y = pos2.second + size2.second / 2.0;
        
        if (type == SymmetryType::VERTICAL) {
            // For vertical symmetry: x1 + x2 = 2*axis, y1 = y2
            if (abs(center1_x + center2_x - 2*axis) > 1e-6 || 
                abs(center1_y - center2_y) > 1e-6) {
                return false;
            }
        } else { // HORIZONTAL
            // For horizontal symmetry: y1 + y2 = 2*axis, x1 = x2
            if (abs(center1_y + center2_y - 2*axis) > 1e-6 || 
                abs(center1_x - center2_x) > 1e-6) {
                return false;
            }
        }
    }
    
    // Check self-symmetric modules
    for (const auto& module : selfSymmetric) {
        auto it = positions.find(module);
        auto dim = dimensions.find(module);
        
        if (it == positions.end() || dim == dimensions.end()) {
            return false; // Module not found
        }
        
        const auto& pos = it->second;
        const auto& size = dim->second;
        
        // Center coordinates
        double center_x = pos.first + size.first / 2.0;
        double center_y = pos.second + size.second / 2.0;
        
        if (type == SymmetryType::VERTICAL) {
            // For vertical symmetry, center x should be on the axis
            if (abs(center_x - axis) > 1e-6) {
                return false;
            }
        } else { // HORIZONTAL
            // For horizontal symmetry, center y should be on the axis
            if (abs(center_y - axis) > 1e-6) {
                return false;
            }
        }
    }
    
    return true;
}

double SymmetryGroup::calculateAxisPosition(const unordered_map<string, pair<int, int>>& positions) const {
    if (symmetryPairs.empty() && selfSymmetric.empty()) {
        return -1.0; // No modules to calculate axis
    }
    
    // Calculate axis based on the existing symmetry pairs and self-symmetric modules
    double sum = 0.0;
    int count = 0;
    
    // Use symmetry pairs to determine axis
    for (const auto& pair : symmetryPairs) {
        auto it1 = positions.find(pair.first);
        auto it2 = positions.find(pair.second);
        
        if (it1 != positions.end() && it2 != positions.end()) {
            if (type == SymmetryType::VERTICAL) {
                // For vertical symmetry, axis is at the middle x-coordinate
                sum += (it1->second.first + it2->second.first) / 2.0;
            } else {
                // For horizontal symmetry, axis is at the middle y-coordinate
                sum += (it1->second.second + it2->second.second) / 2.0;
            }
            count++;
        }
    }
    
    return (count > 0) ? (sum / count) : -1.0;
}