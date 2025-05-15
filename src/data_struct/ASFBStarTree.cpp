// ASFBStarTree.hpp
#pragma once

#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <queue>
#include <cmath>
#include <string>
#include <random>
#include <iostream>

#include "Module.hpp"
#include "SymmetryConstraint.hpp"
#include "ASFBStarTree.hpp"
#include "../Logger.hpp"


/**
 * Packs the B*-tree to get the coordinates of all modules
 * This implementation optimizes for vertical stacking and minimal area
 */
void ASFBStarTree::packBStarTree() {
    // Clear the contour
    clearContour();
    
    Logger::log("Starting to pack ASF-B*-tree with vertical stacking optimization");
    
    // Initialize node positions
    std::unordered_map<BStarNode*, std::pair<int, int>> nodePositions;
    
    // Use level-order traversal (BFS) to ensure parents are processed before children
    std::queue<BStarNode*> bfsQueue;
    if (root != nullptr) {
        bfsQueue.push(root);
        // Root is placed at (0,0)
        nodePositions[root] = {0, 0};
        
        // Update the module position
        modules[root->moduleName]->setPosition(0, 0);
        
        // Update the contour
        updateContour(0, 0, modules[root->moduleName]->getWidth(), modules[root->moduleName]->getHeight());
        Logger::log("Placed root " + root->moduleName + " at (0, 0)");
    }
    
    try {
        while (!bfsQueue.empty()) {
            BStarNode* node = bfsQueue.front();
            bfsQueue.pop();
            
            // Get current node position
            int nodeX = nodePositions[node].first;
            int nodeY = nodePositions[node].second;
            
            Logger::log("Processing node: " + node->moduleName + " at position (" + 
                        std::to_string(nodeX) + ", " + std::to_string(nodeY) + ")");
            
            // Process left child (placed to the right of current node)
            if (node->left) {
                int leftX = nodeX + modules[node->moduleName]->getWidth();
                
                // For vertical stacking, try to minimize x-coordinate
                // by checking if the module can be placed at the current contour height
                int leftY = getContourHeight(leftX);
                
                // Try to keep y-coordinate the same as parent if possible
                // to create tighter packing and better symmetry islands
                if (!hasContourOverlap(leftX, nodeY, 
                                       modules[node->left->moduleName]->getWidth(), 
                                       modules[node->left->moduleName]->getHeight())) {
                    leftY = nodeY; // Maintain same y-coordinate as parent
                }
                
                // Store position of left child
                nodePositions[node->left] = {leftX, leftY};
                
                // Update module position
                modules[node->left->moduleName]->setPosition(leftX, leftY);
                
                // Update the contour
                updateContour(leftX, leftY, 
                              modules[node->left->moduleName]->getWidth(), 
                              modules[node->left->moduleName]->getHeight());
                
                Logger::log("Placed left child " + node->left->moduleName + 
                            " at (" + std::to_string(leftX) + ", " + std::to_string(leftY) + ")");
                
                // Add to queue for further processing
                bfsQueue.push(node->left);
            }
            
            // Process right child (placed at same x, above current node)
            if (node->right) {
                int rightX = nodeX;
                int rightY = nodeY + modules[node->moduleName]->getHeight();
                
                // Store position of right child
                nodePositions[node->right] = {rightX, rightY};
                
                // Update module position
                modules[node->right->moduleName]->setPosition(rightX, rightY);
                
                // Update the contour
                updateContour(rightX, rightY, 
                              modules[node->right->moduleName]->getWidth(), 
                              modules[node->right->moduleName]->getHeight());
                
                Logger::log("Placed right child " + node->right->moduleName + 
                            " at (" + std::to_string(rightX) + ", " + std::to_string(rightY) + ")");
                
                // Add to queue for further processing
                bfsQueue.push(node->right);
            }
        }
        
        // Apply compaction to further minimize area
        compactPlacement();
        
    } catch (const std::exception& e) {
        Logger::log("Exception during packing: " + std::string(e.what()));
        throw; // Re-throw the exception after logging
    }
}


/**
 * Updates the contour after placing a module
 * 
 * @param x X-coordinate of the module
 * @param y Y-coordinate of the module
 * @param width Width of the module
 * @param height Height of the module
 */
void ASFBStarTree::updateContour(int x, int y, int width, int height) {
    // Additional validations
    if (width <= 0 || height <= 0) {
        Logger::log("ERROR: Invalid module dimensions in ASFBStarTree::updateContour: width=" + 
                   std::to_string(width) + ", height=" + std::to_string(height));
        return;
    }
    
    // Check if trying to place negative coordinates
    if (x < 0 || y < 0) {
        Logger::log("ERROR: Cannot place module at negative coordinates in ASFBStarTree: (" + 
                   std::to_string(x) + "," + std::to_string(y) + ")");
        // Adjust to non-negative coordinates
        if (x < 0) x = 0;
        if (y < 0) y = 0;
    }
    
    // Bounds checking - limit to valid range
    int adjX = x;
    int adjWidth = width;
    
    if (adjX + adjWidth > maxContourWidth) {
        Logger::log("WARNING: Module extends beyond ASFBStarTree maximum contour width");
        // If it's far beyond the bound, we need to increase maxContourWidth
        if (adjX + adjWidth > maxContourWidth * 1.5) {
            int newMaxWidth = (adjX + adjWidth) * 2; // Double to avoid frequent resizes
            Logger::log("Increasing ASFBStarTree contour max width from " + 
                       std::to_string(maxContourWidth) + " to " + 
                       std::to_string(newMaxWidth));
            
            // Save current contour heights
            std::vector<int> heights;
            for (int i = 0; i < maxContourWidth; i++) {
                heights.push_back(contourSegTree.query(i, i));
            }
            
            // Reinitialize contour with new size
            maxContourWidth = newMaxWidth;
            contourSegTree.init(maxContourWidth);
            
            // Restore previous heights
            for (int i = 0; i < heights.size(); i++) {
                if (heights[i] > 0) {
                    contourSegTree.update(i, i, heights[i]);
                }
            }
        } else {
            // Otherwise just truncate
            adjWidth = maxContourWidth - adjX;
        }
    }
    
    int top = y + height;
    
    // Get current height at this range
    int currentHeight = contourSegTree.query(adjX, adjX + adjWidth - 1);
    
    // Only update if new height is higher
    if (top > currentHeight) {
        contourSegTree.update(adjX, adjX + adjWidth - 1, top);
    }
}

void ASFBStarTree::calculateSymmetryAxisPosition() {
    Logger::log("Calculating optimal symmetry axis position");
    
    // First find the extent of all representative modules
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::min();
    
    for (const auto& pair : representativeModules) {
        const auto& module = modules[pair.first];
        minX = std::min(minX, module->getX());
        maxX = std::max(maxX, module->getX() + module->getWidth());
        minY = std::min(minY, module->getY());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // For vertical symmetry, we want to place the axis immediately to the right
        // of all the representative modules with minimal padding
        int padding = 1; // Minimal padding
        symmetryAxisPosition = maxX + padding;
        
        Logger::log("Set vertical symmetry axis at x=" + std::to_string(symmetryAxisPosition));
    } else {
        // For horizontal symmetry, we want to place the axis immediately above
        // all the representative modules with minimal padding
        int padding = 1; // Minimal padding
        symmetryAxisPosition = maxY + padding;
        
        Logger::log("Set horizontal symmetry axis at y=" + std::to_string(symmetryAxisPosition));
    }
    
    symmetryGroup->setAxisPosition(symmetryAxisPosition);
}

/**
 * Updates the positions of symmetric modules based on their representatives
 * Strictly following the paper's equations for symmetry constraints
 */
void ASFBStarTree::updateSymmetricModulePositions() {
    // Make sure the symmetry axis is set
    if (symmetryAxisPosition < 0) {
        calculateSymmetryAxisPosition();
    }
    
    Logger::log("Updating symmetric module positions with axis at " + std::to_string(symmetryAxisPosition));
    
    // Update positions for symmetry pairs
    for (const auto& pair : repToPairMap) {
        const std::string& repName = pair.first;
        const std::string& symName = pair.second;
        
        std::shared_ptr<Module> repModule = modules[repName];
        std::shared_ptr<Module> symModule = modules[symName];
        
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            // For vertical symmetry: x_j + x'_j = 2 × x̂_i and y_j = y'_j
            
            // For exact symmetry, we place the symmetric module by reflecting across the axis
            int repCenterX = repModule->getX() + repModule->getWidth() / 2;
            int distanceFromAxis = symmetryAxisPosition - repCenterX;
            int symCenterX = symmetryAxisPosition + distanceFromAxis;
            
            // Calculate top-left corner of symmetric module
            int symX = symCenterX - symModule->getWidth() / 2;
            int symY = repModule->getY(); // Same Y for vertical symmetry
            
            symModule->setPosition(symX, symY);
            
            Logger::log("Vertical symmetry: " + repName + " at (" + std::to_string(repModule->getX()) + 
                       ", " + std::to_string(repModule->getY()) + ") -> " + symName + " at (" + 
                       std::to_string(symX) + ", " + std::to_string(symY) + ")");
        } else {
            // For horizontal symmetry: x_j = x'_j and y_j + y'_j = 2 × ŷ_i
            
            // For exact symmetry, we place the symmetric module by reflecting across the axis
            int repCenterY = repModule->getY() + repModule->getHeight() / 2;
            int distanceFromAxis = symmetryAxisPosition - repCenterY;
            int symCenterY = symmetryAxisPosition + distanceFromAxis;
            
            // Calculate top-left corner of symmetric module
            int symX = repModule->getX(); // Same X for horizontal symmetry
            int symY = symCenterY - symModule->getHeight() / 2;
            
            symModule->setPosition(symX, symY);
            
            Logger::log("Horizontal symmetry: " + repName + " at (" + std::to_string(repModule->getX()) + 
                       ", " + std::to_string(repModule->getY()) + ") -> " + symName + " at (" + 
                       std::to_string(symX) + ", " + std::to_string(symY) + ")");
        }
        
        // Ensure rotation is consistent
        symModule->setRotation(repModule->getRotated());
    }
    
    // Handle self-symmetric modules
    for (const auto& moduleName : selfSymmetricModules) {
        auto module = modules[moduleName];
        
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            // For vertical symmetry, center the module on the axis
            int moduleWidth = module->getWidth();
            int moduleX = symmetryAxisPosition - moduleWidth / 2;
            
            module->setPosition(moduleX, module->getY());
            
            Logger::log("Self-symmetric module " + moduleName + " centered on vertical axis at x=" + 
                       std::to_string(moduleX));
        } else {
            // For horizontal symmetry, center the module on the axis
            int moduleHeight = module->getHeight();
            int moduleY = symmetryAxisPosition - moduleHeight / 2;
            
            module->setPosition(module->getX(), moduleY);
            
            Logger::log("Self-symmetric module " + moduleName + " centered on horizontal axis at y=" + 
                       std::to_string(moduleY));
        }
    }
}


/**
 * Builds an initial B*-tree optimized for vertical stacking of symmetry pairs
 * This follows self-symmetric modules are on the correct branch 
 * and creating a compact tree structure according to Property 1 from the paper
 */
void ASFBStarTree::buildInitialBStarTree() {
    Logger::log("Building initial ASF-B*-tree with vertical stacking optimization");
    
    // Clean up any existing tree
    cleanupTree(root);
    root = nullptr;
    
    // Get all representative modules
    std::vector<std::string> repModuleNames;
    for (const auto& pair : representativeModules) {
        repModuleNames.push_back(pair.first);
    }
    
    Logger::log("Total representative modules: " + std::to_string(repModuleNames.size()));
    
    // Separate self-symmetric and non-self-symmetric modules
    std::vector<std::string> nonSelfSymModules;
    std::vector<std::string> selfSymModules = selfSymmetricModules;
    
    for (const auto& name : repModuleNames) {
        if (!isSelfSymmetric(name)) {
            nonSelfSymModules.push_back(name);
        }
    }
    
    Logger::log("Self-symmetric modules: " + std::to_string(selfSymModules.size()));
    Logger::log("Non-self-symmetric modules: " + std::to_string(nonSelfSymModules.size()));
    
    // Sort modules to create optimal stacking pattern
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // For vertical symmetry, sort by height first to stack efficiently
        std::sort(nonSelfSymModules.begin(), nonSelfSymModules.end(),
                 [this](const std::string& a, const std::string& b) {
                     return modules[a]->getHeight() < modules[b]->getHeight();
                 });
    } else {
        // For horizontal symmetry, sort by width first to stack efficiently
        std::sort(nonSelfSymModules.begin(), nonSelfSymModules.end(),
                 [this](const std::string& a, const std::string& b) {
                     return modules[a]->getWidth() < modules[b]->getWidth();
                 });
    }
    
    // Create nodes for all modules
    std::unordered_map<std::string, BStarNode*> nodeMap;
    for (const auto& name : repModuleNames) {
        nodeMap[name] = new BStarNode(name);
        Logger::log("Created node for module: " + name);
    }
    
    // Build a tree that arranges modules for vertical stacking
    if (!repModuleNames.empty()) {
        std::string rootName;
        
        // For vertical symmetry, we want to start with a module with small width
        // For horizontal symmetry, we want to start with a module with small height
        if (!nonSelfSymModules.empty()) {
            rootName = nonSelfSymModules.front();
            nonSelfSymModules.erase(nonSelfSymModules.begin());
            Logger::log("Using non-self-symmetric module as root: " + rootName);
        } else if (!selfSymModules.empty()) {
            rootName = selfSymModules.front();
            selfSymModules.erase(selfSymModules.begin());
            Logger::log("Using self-symmetric module as root: " + rootName);
        } else {
            Logger::log("ERROR: No modules to place in symmetry group");
            throw std::runtime_error("No modules to place in symmetry group");
        }
        
        root = nodeMap[rootName];
        
        // Create a chain of modules to stack them vertically
        BStarNode* currentNode = root;
        
        // First place all self-symmetric modules on the proper boundary
        for (const auto& name : selfSymModules) {
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // For vertical symmetry, self-symmetric modules on rightmost branch
                Logger::log("Placed self-symmetric module " + name + " as right child of " + currentNode->moduleName);
                currentNode->right = nodeMap[name];
                currentNode = currentNode->right;
            } else {
                // For horizontal symmetry, self-symmetric modules on leftmost branch
                Logger::log("Placed self-symmetric module " + name + " as left child of " + currentNode->moduleName);
                currentNode->left = nodeMap[name];
                currentNode = currentNode->left;
            }
        }
        
        // Now build a chain of nodes that will create a vertical stack
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            // For vertical symmetry, we want to place modules on top of each other
            // We use the right child to stack modules vertically (same X coordinate)
            // and left child to place modules to the right (increasing X)
            
            // First create a vertical stack using right children
            for (size_t i = 0; i < nonSelfSymModules.size(); i++) {
                const std::string& moduleName = nonSelfSymModules[i];
                
                if (i == 0) {
                    // For the first module, we need to find the end of the rightmost branch
                    // to preserve self-symmetric modules
                    if (root->right == nullptr) {
                        // Root has no right child, safe to add directly
                        Logger::log("Placed first non-self-symmetric module " + moduleName + " as right child of root");
                        root->right = nodeMap[moduleName];
                        currentNode = root->right;
                    } else {
                        // Root already has a right child (self-symmetric module)
                        // Find the end of the rightmost branch
                        BStarNode* rightmost = root;
                        while (rightmost->right != nullptr) {
                            rightmost = rightmost->right;
                        }
                        // Add as right child of the rightmost node
                        Logger::log("Placed first non-self-symmetric module " + moduleName + 
                                  " as right child of " + rightmost->moduleName);
                        rightmost->right = nodeMap[moduleName];
                        currentNode = rightmost->right;
                    }
                } else if (i % 2 == 0) {
                    // Even indices go to right (vertical stacking)
                    if (currentNode->right == nullptr) {
                        Logger::log("Placed module " + moduleName + " as right child of " + currentNode->moduleName);
                        currentNode->right = nodeMap[moduleName];
                        currentNode = currentNode->right;
                    } else {
                        // Find a node with no right child
                        BStarNode* target = nullptr;
                        std::function<void(BStarNode*)> findOpenRightSlot = [&](BStarNode* node) {
                            if (node == nullptr) return;
                            if (node->right == nullptr) {
                                target = node;
                                return;
                            }
                            findOpenRightSlot(node->left);
                            if (target == nullptr) findOpenRightSlot(node->right);
                        };
                        
                        findOpenRightSlot(root);
                        
                        if (target != nullptr) {
                            Logger::log("Placed module " + moduleName + " as right child of " + target->moduleName);
                            target->right = nodeMap[moduleName];
                            currentNode = target->right;
                        }
                    }
                } else {
                    // Odd indices go to left (placing to the right side)
                    if (currentNode->left == nullptr) {
                        Logger::log("Placed module " + moduleName + " as left child of " + currentNode->moduleName);
                        currentNode->left = nodeMap[moduleName];
                        currentNode = currentNode->left;
                    } else {
                        // Find a node with no left child
                        BStarNode* target = nullptr;
                        std::function<void(BStarNode*)> findOpenLeftSlot = [&](BStarNode* node) {
                            if (node == nullptr) return;
                            if (node->left == nullptr) {
                                target = node;
                                return;
                            }
                            findOpenLeftSlot(node->right);
                            if (target == nullptr) findOpenLeftSlot(node->left);
                        };
                        
                        findOpenLeftSlot(root);
                        
                        if (target != nullptr) {
                            Logger::log("Placed module " + moduleName + " as left child of " + target->moduleName);
                            target->left = nodeMap[moduleName];
                            currentNode = target->left;
                        }
                    }
                }
            }
        } else {
            // For horizontal symmetry, similar but with left/right children swapped
            // Create a horizontal arrangement using left children
            for (size_t i = 0; i < nonSelfSymModules.size(); i++) {
                const std::string& moduleName = nonSelfSymModules[i];
                
                if (i == 0) {
                    // For the first module, we need to find the end of the leftmost branch
                    // to preserve self-symmetric modules
                    if (root->left == nullptr) {
                        // Root has no left child, safe to add directly
                        Logger::log("Placed first non-self-symmetric module " + moduleName + " as left child of root");
                        root->left = nodeMap[moduleName];
                        currentNode = root->left;
                    } else {
                        // Root already has a left child (self-symmetric module)
                        // Find the end of the leftmost branch
                        BStarNode* leftmost = root;
                        while (leftmost->left != nullptr) {
                            leftmost = leftmost->left;
                        }
                        // Add as left child of the leftmost node
                        Logger::log("Placed first non-self-symmetric module " + moduleName + 
                                  " as left child of " + leftmost->moduleName);
                        leftmost->left = nodeMap[moduleName];
                        currentNode = leftmost->left;
                    }
                } else if (i % 2 == 0) {
                    // Even indices go to left (horizontal arrangement)
                    if (currentNode->left == nullptr) {
                        Logger::log("Placed module " + moduleName + " as left child of " + currentNode->moduleName);
                        currentNode->left = nodeMap[moduleName];
                        currentNode = currentNode->left;
                    } else {
                        // Find a node with no left child
                        BStarNode* target = nullptr;
                        std::function<void(BStarNode*)> findOpenLeftSlot = [&](BStarNode* node) {
                            if (node == nullptr) return;
                            if (node->left == nullptr) {
                                target = node;
                                return;
                            }
                            findOpenLeftSlot(node->right);
                            if (target == nullptr) findOpenLeftSlot(node->left);
                        };
                        
                        findOpenLeftSlot(root);
                        
                        if (target != nullptr) {
                            Logger::log("Placed module " + moduleName + " as left child of " + target->moduleName);
                            target->left = nodeMap[moduleName];
                            currentNode = target->left;
                        }
                    }
                } else {
                    // Odd indices go to right (vertical offset)
                    if (currentNode->right == nullptr) {
                        Logger::log("Placed module " + moduleName + " as right child of " + currentNode->moduleName);
                        currentNode->right = nodeMap[moduleName];
                        currentNode = currentNode->right;
                    } else {
                        // Find a node with no right child
                        BStarNode* target = nullptr;
                        std::function<void(BStarNode*)> findOpenRightSlot = [&](BStarNode* node) {
                            if (node == nullptr) return;
                            if (node->right == nullptr) {
                                target = node;
                                return;
                            }
                            findOpenRightSlot(node->left);
                            if (target == nullptr) findOpenRightSlot(node->right);
                        };
                        
                        findOpenRightSlot(root);
                        
                        if (target != nullptr) {
                            Logger::log("Placed module " + moduleName + " as right child of " + target->moduleName);
                            target->right = nodeMap[moduleName];
                            currentNode = target->right;
                        }
                    }
                }
            }
        }
    }
    
    // Log the resulting tree
    Logger::logTreeStructure("Initial ASF-B*-tree", root);
    
    // Validate the tree structure
    if (!validateTreeStructure(root)) {
        Logger::log("CRITICAL: Invalid tree structure after initialization");
        throw std::runtime_error("Invalid tree structure after initialization");
    }
    
    // Validate the symmetry constraints
    if (!validateSymmetryConstraints()) {
        Logger::log("CRITICAL: Tree does not meet symmetry constraints after initialization");
        throw std::runtime_error("Tree does not meet symmetry constraints after initialization");
    }
}


/**
 * Packs the ASF-B*-tree to get the coordinates of all modules
 * 
 * @return True if packing was successful
 */
bool ASFBStarTree::pack() {
    try {
        // Update the traversals for the current B*-tree
        preorderTraversal.clear();
        inorderTraversal.clear();
        preorder(root);
        inorder(root);
        
        Logger::log("Starting ASF-B*-tree packing with " + 
                   std::to_string(preorderTraversal.size()) + " nodes");
        
        // Pack the B*-tree to get coordinates for representatives
        packBStarTree();
        
        // Calculate the symmetry axis position
        calculateSymmetryAxisPosition();
        
        // Update positions of symmetric modules
        updateSymmetricModulePositions();
        
        // Validate the resulting placement satisfies symmetry constraints
        if (!validateSymmetry()) {
            Logger::log("ERROR: Placement does not satisfy symmetry constraints");
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        Logger::log("Exception during packing: " + std::string(e.what()));
        return false;
    }
}



/**
 * Validate if the symmetry is maintained, with improved error reporting
 */
bool ASFBStarTree::validateSymmetry() const {
    try {
        // First check for negative coordinates which would invalidate the placement
        for (const auto& pair : modules) {
            const auto& module = pair.second;
            if (module->getX() < 0 || module->getY() < 0) {
                Logger::log("ERROR: Module " + pair.first + " has negative coordinates (" + 
                          std::to_string(module->getX()) + ", " + 
                          std::to_string(module->getY()) + ")");
                return false;
            }
        }
        
        // Validate symmetry pairs have correct placements
        for (const auto& pair : repToPairMap) {
            // Safety check: Make sure both modules exist
            auto repIt = modules.find(pair.first);
            auto symIt = modules.find(pair.second);
            
            if (repIt == modules.end() || symIt == modules.end()) {
                Logger::log("WARNING: Cannot validate symmetry for missing modules: " + 
                          pair.first + " or " + pair.second);
                continue;
            }
            
            const std::string& repName = pair.first;
            const std::string& symName = pair.second;
            
            std::shared_ptr<Module> repModule = repIt->second;
            std::shared_ptr<Module> symModule = symIt->second;
            
            // Calculate centers
            double repCenterX = repModule->getX() + repModule->getWidth() / 2.0;
            double repCenterY = repModule->getY() + repModule->getHeight() / 2.0;
            double symCenterX = symModule->getX() + symModule->getWidth() / 2.0;
            double symCenterY = symModule->getY() + symModule->getHeight() / 2.0;
            
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // Check vertical symmetry equation
                double expectedSum = 2 * symmetryAxisPosition;
                double actualSum = repCenterX + symCenterX;
                double error = std::abs(expectedSum - actualSum);
                
                // Also check y-coordinates match
                double yError = std::abs(repCenterY - symCenterY);
                
                if (error > 1.0 || yError > 1.0) {  // Allow small floating-point error
                    Logger::log("ERROR: Symmetry violation for pair (" + repName + ", " + symName + ")");
                    Logger::log("  Expected: repCenterX + symCenterX = " + std::to_string(expectedSum));
                    Logger::log("  Actual: " + std::to_string(repCenterX) + " + " + std::to_string(symCenterX) + " = " + std::to_string(actualSum));
                    Logger::log("  Y error: " + std::to_string(yError));
                    return false;
                }
            } else {
                // Check horizontal symmetry equation
                double expectedSum = 2 * symmetryAxisPosition;
                double actualSum = repCenterY + symCenterY;
                double error = std::abs(expectedSum - actualSum);
                
                // Also check x-coordinates match
                double xError = std::abs(repCenterX - symCenterX);
                
                if (error > 1.0 || xError > 1.0) {  // Allow small floating-point error
                    Logger::log("ERROR: Symmetry violation for pair (" + repName + ", " + symName + ")");
                    Logger::log("  Expected: repCenterY + symCenterY = " + std::to_string(expectedSum));
                    Logger::log("  Actual: " + std::to_string(repCenterY) + " + " + std::to_string(symCenterY) + " = " + std::to_string(actualSum));
                    Logger::log("  X error: " + std::to_string(xError));
                    return false;
                }
            }
        }
        
        // Check self-symmetric modules are centered on the axis
        for (const auto& moduleName : selfSymmetricModules) {
            // Safety check: Make sure module exists
            auto moduleIt = modules.find(moduleName);
            if (moduleIt == modules.end()) {
                Logger::log("WARNING: Cannot validate symmetry for missing self-symmetric module: " + moduleName);
                continue;
            }
            
            std::shared_ptr<Module> module = moduleIt->second;
            double centerX = module->getX() + module->getWidth() / 2.0;
            double centerY = module->getY() + module->getHeight() / 2.0;
            
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                double error = std::abs(centerX - symmetryAxisPosition);
                if (error > 1.0) {  // Allow small floating-point error
                    Logger::log("ERROR: Self-symmetric module " + moduleName + " not centered on axis");
                    Logger::log("  Module center: " + std::to_string(centerX));
                    Logger::log("  Axis position: " + std::to_string(symmetryAxisPosition));
                    return false;
                }
            } else {
                double error = std::abs(centerY - symmetryAxisPosition);
                if (error > 1.0) {  // Allow small floating-point error
                    Logger::log("ERROR: Self-symmetric module " + moduleName + " not centered on axis");
                    Logger::log("  Module center: " + std::to_string(centerY));
                    Logger::log("  Axis position: " + std::to_string(symmetryAxisPosition));
                    return false;
                }
            }
        }
        
        // If all checks pass, the symmetry is valid
        Logger::log("Symmetry validation passed");
        return true;
    } catch (const std::exception& e) {
        Logger::log("Exception in validateSymmetry: " + std::string(e.what()));
        return false;
    }
}

/**
 * Validates that the modules form a connected placement (symmetry island)
 * 
 * @return True if all modules are connected
 */
bool ASFBStarTree::validateConnectivity() {
    Logger::log("Validating connectivity (symmetry island constraint)");
    
    if (modules.empty()) return true;
    
    // Build position and dimension maps for the isSymmetryIsland check
    std::unordered_map<std::string, std::pair<int, int>> positions;
    std::unordered_map<std::string, std::pair<int, int>> dimensions;
    
    for (const auto& pair : modules) {
        const std::string& name = pair.first;
        const auto& module = pair.second;
        positions[name] = {module->getX(), module->getY()};
        dimensions[name] = {module->getWidth(), module->getHeight()};
    }
    
    // Use the isSymmetryIsland function from SymmetryGroup
    bool isConnected = symmetryGroup->isSymmetryIsland(positions, dimensions);
    
    if (isConnected) {
        Logger::log("Connectivity validation passed - all modules form a symmetry island");
    } else {
        Logger::log("Connectivity validation failed - modules do not form a symmetry island");
    }
    
    return isConnected;
}

/**
 * Helper function to check if placing a module would overlap with existing contour
 */
bool ASFBStarTree::hasContourOverlap(int x, int y, int width, int height) {
    // Bounds checking
    if (x < 0 || x + width > maxContourWidth) {
        Logger::log("WARNING: ASF-B*-tree contour overlap check out of bounds: x=" + std::to_string(x) + 
                   ", width=" + std::to_string(width) + 
                   ", maxWidth=" + std::to_string(maxContourWidth));
        // Adjust to stay within bounds if possible
        if (x < 0) x = 0;
        if (x + width > maxContourWidth) width = maxContourWidth - x;
    }
    
    // Get maximum height in the range
    int maxHeight = contourSegTree.query(x, x + width - 1);
    
    // If contour height is greater than y, there's an overlap
    return maxHeight > y;
}

/**
 * Optimize module positions to minimize area while preserving connectivity
 * This is a new function to replace the previous enforceConnectivity function
 */
void ASFBStarTree::optimizeModulePositions() {
    Logger::log("Optimizing module positions to minimize area while preserving connectivity");
    
    // First, find the minimum x and y coordinates to ensure all modules have positive positions
    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        minX = std::min(minX, module->getX());
        minY = std::min(minY, module->getY());
    }
    
    // Shift all modules to ensure positive coordinates
    if (minX < 0 || minY < 0) {
        int shiftX = std::max(0, -minX);
        int shiftY = std::max(0, -minY);
        
        for (auto& pair : modules) {
            auto& module = pair.second;
            module->setPosition(module->getX() + shiftX, module->getY() + shiftY);
        }
    }
    
    // Collect module positions
    std::unordered_map<std::string, std::pair<int, int>> positions;
    std::unordered_map<std::string, std::pair<int, int>> dimensions;
    
    for (const auto& pair : representativeModules) {
        const auto& module = modules[pair.first];
        positions[pair.first] = {module->getX(), module->getY()};
        dimensions[pair.first] = {module->getWidth(), module->getHeight()};
    }
    
    // Apply a compact transformation to representative modules
    // The idea is to shift modules in X and Y directions to minimize gaps
    // while preserving relative positions (connectivity)
    
    // 1. Sort modules by x-coordinate
    std::vector<std::string> modulesByX;
    for (const auto& pair : representativeModules) {
        modulesByX.push_back(pair.first);
    }
    
    std::sort(modulesByX.begin(), modulesByX.end(), [&positions](const std::string& a, const std::string& b) {
        return positions[a].first < positions[b].first;
    });
    
    // 2. Compact in X direction (left-to-right scan)
    for (size_t i = 1; i < modulesByX.size(); ++i) {
        const std::string& currModule = modulesByX[i];
        int minPossibleX = 0;
        
        // Find the rightmost edge of any module that is to the left of current module
        for (size_t j = 0; j < i; ++j) {
            const std::string& prevModule = modulesByX[j];
            const auto& prevPos = positions[prevModule];
            const auto& prevDim = dimensions[prevModule];
            const auto& currPos = positions[currModule];
            const auto& currDim = dimensions[currModule];
            
            // Check if modules overlap in Y direction
            bool yOverlap = !(prevPos.second + prevDim.second <= currPos.second || 
                             currPos.second + currDim.second <= prevPos.second);
            
            if (yOverlap) {
                // If they overlap in Y, current module must be placed to the right of previous
                minPossibleX = std::max(minPossibleX, prevPos.first + prevDim.first);
            }
        }
        
        // Update position if we can move it left
        if (minPossibleX < positions[currModule].first) {
            positions[currModule].first = minPossibleX;
        }
    }
    
    // 3. Sort modules by y-coordinate
    std::vector<std::string> modulesByY;
    for (const auto& pair : representativeModules) {
        modulesByY.push_back(pair.first);
    }
    
    std::sort(modulesByY.begin(), modulesByY.end(), [&positions](const std::string& a, const std::string& b) {
        return positions[a].second < positions[b].second;
    });
    
    // 4. Compact in Y direction (bottom-to-top scan)
    for (size_t i = 1; i < modulesByY.size(); ++i) {
        const std::string& currModule = modulesByY[i];
        int minPossibleY = 0;
        
        // Find the topmost edge of any module that is below current module
        for (size_t j = 0; j < i; ++j) {
            const std::string& prevModule = modulesByY[j];
            const auto& prevPos = positions[prevModule];
            const auto& prevDim = dimensions[prevModule];
            const auto& currPos = positions[currModule];
            const auto& currDim = dimensions[currModule];
            
            // Check if modules overlap in X direction
            bool xOverlap = !(prevPos.first + prevDim.first <= currPos.first || 
                             currPos.first + currDim.first <= prevPos.first);
            
            if (xOverlap) {
                // If they overlap in X, current module must be placed above previous
                minPossibleY = std::max(minPossibleY, prevPos.second + prevDim.second);
            }
        }
        
        // Update position if we can move it down
        if (minPossibleY < positions[currModule].second) {
            positions[currModule].second = minPossibleY;
        }
    }
    
    // 5. Update module positions
    for (const auto& pair : positions) {
        const std::string& moduleName = pair.first;
        const auto& pos = pair.second;
        
        modules[moduleName]->setPosition(pos.first, pos.second);
    }
    
    Logger::log("Module positions optimized for compact placement");
}

/**
 * Apply compaction to minimize area while preserving symmetry constraints
 * This function tries to compact the placement in X and Y directions
 */
void ASFBStarTree::compactPlacement() {
    Logger::log("Beginning compaction of symmetry island...");
    
    // Log initial positions before compaction
    for (const auto& pair : modules) {
        Logger::log("Before compaction: " + pair.first + " at (" + 
                   std::to_string(pair.second->getX()) + "," + 
                   std::to_string(pair.second->getY()) + ")");
    }
    
    // Calculate adjusted bounding box
    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();
    int maxY = std::numeric_limits<int>::min();
    
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        minX = std::min(minX, module->getX());
        minY = std::min(minY, module->getY());
        maxX = std::max(maxX, module->getX() + module->getWidth());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    Logger::log("Initial bounding box: (" + std::to_string(minX) + "," + 
               std::to_string(minY) + ") to (" + std::to_string(maxX) + "," + 
               std::to_string(maxY) + ")");
    
    // Shift modules to ensure positive coordinates
    if (minX < 0 || minY < 0) {
        int shiftX = minX < 0 ? -minX : 0;
        int shiftY = minY < 0 ? -minY : 0;
        
        Logger::log("Shifting all modules by (" + std::to_string(shiftX) + "," + 
                   std::to_string(shiftY) + ") to ensure positive coordinates");
        
        for (auto& pair : modules) {
            auto& module = pair.second;
            module->setPosition(module->getX() + shiftX, module->getY() + shiftY);
        }
        
        // Recalculate bounding box
        minX += shiftX;
        maxX += shiftX;
        minY += shiftY;
        maxY += shiftY;
    }
    
    // Separate modules by symmetry axis
    std::vector<std::string> leftModules, rightModules, axisCenteredModules;
    double axisPos = symmetryAxisPosition;
    
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // For vertical symmetry
        for (const auto& pair : modules) {
            const std::string& name = pair.first;
            const auto& module = pair.second;
            double center = module->getX() + module->getWidth() / 2.0;
            
            if (std::abs(center - axisPos) < 1.0) {
                // Module is centered on axis
                axisCenteredModules.push_back(name);
            } else if (center < axisPos) {
                // Module is left of axis
                leftModules.push_back(name);
            } else {
                // Module is right of axis
                rightModules.push_back(name);
            }
        }
    } else {
        // For horizontal symmetry
        for (const auto& pair : modules) {
            const std::string& name = pair.first;
            const auto& module = pair.second;
            double center = module->getY() + module->getHeight() / 2.0;
            
            if (std::abs(center - axisPos) < 1.0) {
                // Module is centered on axis
                axisCenteredModules.push_back(name);
            } else if (center < axisPos) {
                // Module is below axis
                leftModules.push_back(name);
            } else {
                // Module is above axis
                rightModules.push_back(name);
            }
        }
    }
    
    Logger::log("Modules analysis: " + std::to_string(leftModules.size()) + " left/below axis, " + 
               std::to_string(rightModules.size()) + " right/above axis, " + 
               std::to_string(axisCenteredModules.size()) + " on axis");
    
    // For each group, perform compaction while respecting symmetry
    // This is a simplified example - in practice, you'd use a more sophisticated algorithm
    
    // First, sort modules by position
    auto sortModules = [this](std::vector<std::string>& moduleList, bool byX) {
        std::sort(moduleList.begin(), moduleList.end(), [this, byX](const std::string& a, const std::string& b) {
            if (byX) {
                return modules[a]->getX() < modules[b]->getX();
            } else {
                return modules[a]->getY() < modules[b]->getY();
            }
        });
    };
    
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // For vertical symmetry, compact in X direction first
        sortModules(leftModules, true);
        sortModules(rightModules, true);
        
        // Compact left modules towards the axis
        int currentX = minX;
        for (const auto& name : leftModules) {
            auto& module = modules[name];
            if (module->getX() > currentX) {
                Logger::log("Compacting left module " + name + " from x=" + 
                           std::to_string(module->getX()) + " to x=" + 
                           std::to_string(currentX));
                module->setPosition(currentX, module->getY());
            }
            currentX = module->getX() + module->getWidth();
        }
        
        // Compact right modules away from the axis
        double axisPos = symmetryAxisPosition;
        std::sort(rightModules.begin(), rightModules.end(), [this](const std::string& a, const std::string& b) {
            return modules[a]->getX() > modules[b]->getX(); // Reverse sort for right side
        });
        
        currentX = maxX;
        for (const auto& name : rightModules) {
            auto& module = modules[name];
            int moduleRight = module->getX() + module->getWidth();
            if (moduleRight < currentX) {
                int newX = currentX - module->getWidth();
                Logger::log("Compacting right module " + name + " from x=" + 
                           std::to_string(module->getX()) + " to x=" + 
                           std::to_string(newX));
                module->setPosition(newX, module->getY());
            }
            currentX = module->getX();
        }
        
        // Now compact in Y direction (similar process but keeping X fixed)
        // Sort by Y
        sortModules(leftModules, false);
        sortModules(rightModules, false);
        sortModules(axisCenteredModules, false);
        
        // Compact all modules upward
        int currentY = minY;
        // Process left, axis-centered, and right modules in Y order
        std::vector<std::string> allModules;
        allModules.insert(allModules.end(), leftModules.begin(), leftModules.end());
        allModules.insert(allModules.end(), axisCenteredModules.begin(), axisCenteredModules.end());
        allModules.insert(allModules.end(), rightModules.begin(), rightModules.end());
        
        // Sort by Y again since we combined lists
        std::sort(allModules.begin(), allModules.end(), [this](const std::string& a, const std::string& b) {
            return modules[a]->getY() < modules[b]->getY();
        });
        
        // Compact upward
        for (const auto& name : allModules) {
            auto& module = modules[name];
            if (module->getY() > currentY) {
                Logger::log("Compacting module " + name + " from y=" + 
                           std::to_string(module->getY()) + " to y=" + 
                           std::to_string(currentY));
                module->setPosition(module->getX(), currentY);
            }
            currentY = module->getY() + module->getHeight();
        }
    } else {
        // For horizontal symmetry, similar process but swapping X and Y
        // (Implement similar logic as above but for horizontal symmetry)
    }
    
    // Update symmetry axis position if needed
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // For self-symmetric modules, make sure they are centered on the axis
        for (const auto& name : axisCenteredModules) {
            auto& module = modules[name];
            int moduleWidth = module->getWidth();
            int newX = static_cast<int>(symmetryAxisPosition) - moduleWidth / 2;
            
            if (module->getX() != newX) {
                Logger::log("Centering self-symmetric module " + name + " on axis at x=" + 
                           std::to_string(newX));
                module->setPosition(newX, module->getY());
            }
        }
        
        // Verify symmetry relationships for symmetry pairs
        for (const auto& pair : repToPairMap) {
            const std::string& repName = pair.first;
            const std::string& symName = pair.second;
            
            std::shared_ptr<Module> repModule = modules[repName];
            std::shared_ptr<Module> symModule = modules[symName];
            
            // Calculate expected symmetric position
            int repCenterX = repModule->getX() + repModule->getWidth() / 2;
            int distanceFromAxis = static_cast<int>(symmetryAxisPosition) - repCenterX;
            int symCenterX = static_cast<int>(symmetryAxisPosition) + distanceFromAxis;
            int symX = symCenterX - symModule->getWidth() / 2;
            
            // Ensure y-coordinates match
            if (repModule->getY() != symModule->getY()) {
                Logger::log("Fixing y-coordinate mismatch for symmetry pair " + 
                           repName + " and " + symName);
                symModule->setPosition(symModule->getX(), repModule->getY());
            }
            
            // Ensure x-coordinates are properly symmetric
            if (symModule->getX() != symX) {
                Logger::log("Fixing x-coordinate symmetry for pair " + 
                           repName + " and " + symName);
                symModule->setPosition(symX, symModule->getY());
            }
        }
    } else {
        // Similar logic for horizontal symmetry
    }
    
    // Log final positions after compaction
    for (const auto& pair : modules) {
        Logger::log("After compaction: " + pair.first + " at (" + 
                   std::to_string(pair.second->getX()) + "," + 
                   std::to_string(pair.second->getY()) + ")");
    }
    
    // Recalculate final bounding box
    minX = std::numeric_limits<int>::max();
    minY = std::numeric_limits<int>::max();
    maxX = std::numeric_limits<int>::min();
    maxY = std::numeric_limits<int>::min();
    
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        minX = std::min(minX, module->getX());
        minY = std::min(minY, module->getY());
        maxX = std::max(maxX, module->getX() + module->getWidth());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    Logger::log("Final bounding box after compaction: (" + std::to_string(minX) + "," + 
               std::to_string(minY) + ") to (" + std::to_string(maxX) + "," + 
               std::to_string(maxY) + ")");
    
    // Ensure all modules have positive coordinates
    if (minX < 0 || minY < 0) {
        Logger::log("WARNING: Found negative coordinates after compaction!");
    }
}