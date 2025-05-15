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
    int right = x + width;
    int top = y + height;
    
    // Create a new contour point if contour is empty
    if (contourHead == nullptr) {
        contourHead = new ContourPoint(x, top);
        contourHead->next = new ContourPoint(right, 0);
        return;
    }
    
    // Find the position to insert or update
    ContourPoint* curr = contourHead;
    ContourPoint* prev = nullptr;
    
    // Skip points to the left of the module
    while (curr != nullptr && curr->x < x) {
        prev = curr;
        curr = curr->next;
    }
    
    // Handle case where we need to insert a point at x
    if (curr == nullptr || curr->x > x) {
        // Create a new point at the left edge of the module
        ContourPoint* newPoint = new ContourPoint(x, top);
        
        // Link the new point
        if (prev) {
            prev->next = newPoint;
        } else {
            contourHead = newPoint;
        }
        newPoint->next = curr;
        
        // Update for next operations
        prev = newPoint;
    } else if (curr->x == x) {
        // Update existing point at x
        curr->height = std::max(curr->height, top);
        prev = curr;
        curr = curr->next;
    }
    
    // Process or remove intermediate points up to right edge
    while (curr != nullptr && curr->x < right) {
        if (curr->height <= top) {
            // This point is below or at our new contour height, so remove it
            ContourPoint* temp = curr;
            curr = curr->next;
            delete temp;
            
            if (prev) {
                prev->next = curr;
            } else {
                contourHead = curr;
            }
        } else {
            // This point is above our new contour, keep it
            prev = curr;
            curr = curr->next;
        }
    }
    
    // Handle the right edge point
    if (curr == nullptr || curr->x > right) {
        // Add a new point at the right edge
        ContourPoint* newPoint = new ContourPoint(right, prev ? prev->height : 0);
        newPoint->next = curr;
        
        if (prev) {
            prev->next = newPoint;
        } else {
            contourHead = newPoint;
        }
    } else if (curr->x == right) {
        // Update existing point at the right edge
        curr->height = std::max(curr->height, prev ? prev->height : 0);
    }
}


/**
 * Calculates the symmetry axis position based on the current placement
 * Optimized to minimize overall area while maintaining symmetry constraints
 */
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
    int right = x + width;
    for (int currX = x; currX < right; currX++) {
        if (getContourHeight(currX) > y) {
            return true;
        }
    }
    return false;
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
    Logger::log("Applying compaction to minimize area");
    
    // Create a copy of the current positions for working
    std::unordered_map<std::string, std::pair<int, int>> positions;
    std::unordered_map<std::string, std::pair<int, int>> dimensions;
    
    for (const auto& pair : representativeModules) {
        const auto& module = modules[pair.first];
        positions[pair.first] = {module->getX(), module->getY()};
        dimensions[pair.first] = {module->getWidth(), module->getHeight()};
    }
    
    // Find minimum X and Y to ensure all modules have positive coordinates
    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    
    for (const auto& pair : positions) {
        minX = std::min(minX, pair.second.first);
        minY = std::min(minY, pair.second.second);
    }
    
    // Shift all modules to ensure positive coordinates
    if (minX > 0 || minY > 0) {
        for (auto& pair : positions) {
            pair.second.first -= minX;
            pair.second.second -= minY;
        }
    }
    
    // For vertical symmetry, focus on minimizing width
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // Sort modules by x-coordinate
        std::vector<std::string> modulesByX;
        for (const auto& pair : positions) {
            modulesByX.push_back(pair.first);
        }
        
        std::sort(modulesByX.begin(), modulesByX.end(), [&positions](const std::string& a, const std::string& b) {
            return positions[a].first < positions[b].first;
        });
        
        // Compact in X direction by shifting modules to the left when possible
        for (size_t i = 1; i < modulesByX.size(); i++) {
            const std::string& currModule = modulesByX[i];
            int currX = positions[currModule].first;
            int currY = positions[currModule].second;
            int currWidth = dimensions[currModule].first;
            int currHeight = dimensions[currModule].second;
            
            // Try to shift this module left as much as possible without overlapping
            int maxLeftShift = currX;
            
            for (size_t j = 0; j < i; j++) {
                const std::string& prevModule = modulesByX[j];
                int prevX = positions[prevModule].first;
                int prevY = positions[prevModule].second;
                int prevWidth = dimensions[prevModule].first;
                int prevHeight = dimensions[prevModule].second;
                
                // Check if there's a potential overlap in Y direction
                bool yOverlap = !(prevY + prevHeight <= currY || currY + currHeight <= prevY);
                
                if (yOverlap) {
                    // If they could overlap in Y, calculate the minimum X distance needed
                    maxLeftShift = std::min(maxLeftShift, currX - (prevX + prevWidth));
                }
            }
            
            // Apply the shift
            if (maxLeftShift > 0) {
                positions[currModule].first -= maxLeftShift;
            }
        }
        
        // Sort modules by y-coordinate to compact in Y direction
        std::vector<std::string> modulesByY;
        for (const auto& pair : positions) {
            modulesByY.push_back(pair.first);
        }
        
        std::sort(modulesByY.begin(), modulesByY.end(), [&positions](const std::string& a, const std::string& b) {
            return positions[a].second < positions[b].second;
        });
        
        // Compact in Y direction by shifting modules down when possible
        for (size_t i = 1; i < modulesByY.size(); i++) {
            const std::string& currModule = modulesByY[i];
            int currX = positions[currModule].first;
            int currY = positions[currModule].second;
            int currWidth = dimensions[currModule].first;
            int currHeight = dimensions[currModule].second;
            
            // Try to shift this module down as much as possible without overlapping
            int maxDownShift = currY;
            
            for (size_t j = 0; j < i; j++) {
                const std::string& prevModule = modulesByY[j];
                int prevX = positions[prevModule].first;
                int prevY = positions[prevModule].second;
                int prevWidth = dimensions[prevModule].first;
                int prevHeight = dimensions[prevModule].second;
                
                // Check if there's a potential overlap in X direction
                bool xOverlap = !(prevX + prevWidth <= currX || currX + currWidth <= prevX);
                
                if (xOverlap) {
                    // If they could overlap in X, calculate the minimum Y distance needed
                    maxDownShift = std::min(maxDownShift, currY - (prevY + prevHeight));
                }
            }
            
            // Apply the shift
            if (maxDownShift > 0) {
                positions[currModule].second -= maxDownShift;
            }
        }
    } else {
        // For horizontal symmetry, focus on minimizing height
        // Sort modules by y-coordinate
        std::vector<std::string> modulesByY;
        for (const auto& pair : positions) {
            modulesByY.push_back(pair.first);
        }
        
        std::sort(modulesByY.begin(), modulesByY.end(), [&positions](const std::string& a, const std::string& b) {
            return positions[a].second < positions[b].second;
        });
        
        // Compact in Y direction by shifting modules down when possible
        for (size_t i = 1; i < modulesByY.size(); i++) {
            const std::string& currModule = modulesByY[i];
            int currX = positions[currModule].first;
            int currY = positions[currModule].second;
            int currWidth = dimensions[currModule].first;
            int currHeight = dimensions[currModule].second;
            
            // Try to shift this module down as much as possible without overlapping
            int maxDownShift = currY;
            
            for (size_t j = 0; j < i; j++) {
                const std::string& prevModule = modulesByY[j];
                int prevX = positions[prevModule].first;
                int prevY = positions[prevModule].second;
                int prevWidth = dimensions[prevModule].first;
                int prevHeight = dimensions[prevModule].second;
                
                // Check if there's a potential overlap in X direction
                bool xOverlap = !(prevX + prevWidth <= currX || currX + currWidth <= prevX);
                
                if (xOverlap) {
                    // If they could overlap in X, calculate the minimum Y distance needed
                    maxDownShift = std::min(maxDownShift, currY - (prevY + prevHeight));
                }
            }
            
            // Apply the shift
            if (maxDownShift > 0) {
                positions[currModule].second -= maxDownShift;
            }
        }
        
        // Sort modules by x-coordinate to compact in X direction
        std::vector<std::string> modulesByX;
        for (const auto& pair : positions) {
            modulesByX.push_back(pair.first);
        }
        
        std::sort(modulesByX.begin(), modulesByX.end(), [&positions](const std::string& a, const std::string& b) {
            return positions[a].first < positions[b].first;
        });
        
        // Compact in X direction by shifting modules left when possible
        for (size_t i = 1; i < modulesByX.size(); i++) {
            const std::string& currModule = modulesByX[i];
            int currX = positions[currModule].first;
            int currY = positions[currModule].second;
            int currWidth = dimensions[currModule].first;
            int currHeight = dimensions[currModule].second;
            
            // Try to shift this module left as much as possible without overlapping
            int maxLeftShift = currX;
            
            for (size_t j = 0; j < i; j++) {
                const std::string& prevModule = modulesByX[j];
                int prevX = positions[prevModule].first;
                int prevY = positions[prevModule].second;
                int prevWidth = dimensions[prevModule].first;
                int prevHeight = dimensions[prevModule].second;
                
                // Check if there's a potential overlap in Y direction
                bool yOverlap = !(prevY + prevHeight <= currY || currY + currHeight <= prevY);
                
                if (yOverlap) {
                    // If they could overlap in Y, calculate the minimum X distance needed
                    maxLeftShift = std::min(maxLeftShift, currX - (prevX + prevWidth));
                }
            }
            
            // Apply the shift
            if (maxLeftShift > 0) {
                positions[currModule].first -= maxLeftShift;
            }
        }
    }
    
    // Update module positions
    for (const auto& pair : positions) {
        const std::string& moduleName = pair.first;
        const auto& pos = pair.second;
        
        modules[moduleName]->setPosition(pos.first, pos.second);
    }
    
    Logger::log("Compaction complete for tight symmetry island packing");
}