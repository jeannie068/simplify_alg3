#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <fstream>
#include <iomanip>
#include "solver.hpp"

#include "../Logger.hpp"
/**
 * Initialize global placement debug logger
 */
void PlacementSolver::initGlobalDebugger() {
    // Open the log file - do this only once in constructor
    globalLogFile.open("globalPlacement_debug.log");
    
    if (globalLogFile.is_open()) {
        globalDebugEnabled = true;
        
        // Write header to file
        globalLogFile << "Global Placement Debug Log" << std::endl;
        globalLogFile << "==========================" << std::endl;
        
        // Get current time
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        
        // Write timestamp using safe method
        globalLogFile << "Time: " << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << std::endl;
        globalLogFile << std::endl;
    } else {
        globalDebugEnabled = false;
    }
}

/**
 * Log global placement information safely
 */
void PlacementSolver::logGlobalPlacement(const std::string& message) {
    if (globalDebugEnabled && globalLogFile.is_open()) {
        globalLogFile << message << std::endl;
        globalLogFile.flush(); // Ensure content is written immediately
    }
}
/**
 * Logs the current contour to the debug file safely
 */
void PlacementSolver::logContour() {
    if (!globalDebugEnabled || !globalLogFile.is_open()) {
        return; // Early return if debugging is disabled
    }
    
    std::stringstream ss;
    ss << "Current contour: ";
    
    // Safely iterate through contour points
    ContourPoint* current = contourHead;
    int safetyCounter = 0; // Prevent infinite loops
    const int maxPoints = 1000; // Reasonable upper limit
    
    while (current != nullptr && safetyCounter < maxPoints) {
        ss << "(" << current->x << "," << current->height << ") ";
        current = current->next;
        safetyCounter++;
    }
    
    if (safetyCounter >= maxPoints) {
        ss << " [WARNING: Contour list may be corrupt - too many points]";
    }
    
    logGlobalPlacement(ss.str());
}

/**
 * Prints the B*-tree structure for debugging with safety checks
 */
void PlacementSolver::printBStarTree(BStarNode* node, std::string prefix, bool isLast) {
    if (!globalDebugEnabled || !globalLogFile.is_open()) {
        return; // Early return if debugging is disabled
    }
    
    if (node == nullptr) {
        logGlobalPlacement(prefix + (isLast ? "└── " : "├── ") + "<nullptr>");
        return;
    }
    
    try {
        std::string nodeInfo = node->name + " (isIsland: " + (node->isSymmetryIsland ? "true" : "false") + ")";
        logGlobalPlacement(prefix + (isLast ? "└── " : "├── ") + nodeInfo);
        
        // Prepare prefix for children
        prefix += isLast ? "    " : "│   ";
        
        // Process children with proper nullptr checks
        printBStarTree(node->left, prefix, node->right == nullptr);
        printBStarTree(node->right, prefix, true);
    } catch (const std::exception& e) {
        logGlobalPlacement(prefix + "[ERROR: Exception while printing node: " + std::string(e.what()) + "]");
    }
}


// Constructor
PlacementSolver::PlacementSolver()
    : bstarRoot(nullptr), contourHead(nullptr),
      solutionArea(0), solutionWirelength(0),
      bestSolutionArea(std::numeric_limits<int>::max()), bestSolutionWirelength(0),
      initialTemperature(1000.0), finalTemperature(0.1),
      coolingRate(0.95), iterationsPerTemperature(100), noImprovementLimit(1000),
      rotateProb(0.3), moveProb(0.3), swapProb(0.3),
      changeRepProb(0.05), convertSymProb(0.05),
      areaWeight(1.0), wirelengthWeight(0.0),
      timeLimit(300),
      globalDebugEnabled(false) {  // Initialize debug as disabled initially
    
    // Initialize random number generator
    std::random_device rd;
    rng = std::mt19937(rd());
    
    // Initialize global placement debugger
    initGlobalDebugger();
}

// Destructor
PlacementSolver::~PlacementSolver() {
    cleanupBStarTree(bstarRoot);
    clearContour();
    
    // Close global log file if open
    if (globalLogFile.is_open()) {
        globalLogFile.close();
    }
}

// Cleanup B*-tree
void PlacementSolver::cleanupBStarTree(BStarNode* node) {
    if (node == nullptr) return;
    
    cleanupBStarTree(node->left);
    cleanupBStarTree(node->right);
    delete node;
}

// Clear contour data structure
void PlacementSolver::clearContour() {
    while (contourHead != nullptr) {
        ContourPoint* temp = contourHead;
        contourHead = contourHead->next;
        delete temp;
    }
    contourHead = nullptr;
}

// Update contour after placing a module
void PlacementSolver::updateContour(int x, int y, int width, int height) {
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
    
    // Update or insert points for the module
    if (curr == nullptr || curr->x > right) {
        // Module is beyond the current contour
        ContourPoint* newPoint = new ContourPoint(x, top);
        newPoint->next = new ContourPoint(right, prev ? prev->height : 0);
        newPoint->next->next = curr;
        
        if (prev) {
            prev->next = newPoint;
        } else {
            contourHead = newPoint;
        }
    } else {
        // Module intersects with existing contour
        if (curr->x > x) {
            // Insert a new point at the left edge of the module
            ContourPoint* newPoint = new ContourPoint(x, top);
            newPoint->next = curr;
            
            if (prev) {
                prev->next = newPoint;
            } else {
                contourHead = newPoint;
            }
            
            prev = newPoint;
        } else if (curr->x == x) {
            // Update the existing point
            curr->height = std::max(curr->height, top);
            prev = curr;
            curr = curr->next;
        }
        
        // Update or merge intermediate points
        while (curr != nullptr && curr->x <= right) {
            if (curr->x == right) {
                curr->height = std::max(curr->height, top);
                break;
            } else {
                ContourPoint* temp = curr;
                curr = curr->next;
                delete temp;
            }
        }
        
        if (prev) {
            prev->next = curr;
        } else {
            contourHead = curr;
        }
        
        // If we reached the end without finding a point at 'right'
        if (curr == nullptr || curr->x > right) {
            ContourPoint* newPoint = new ContourPoint(right, prev ? prev->height : 0);
            newPoint->next = curr;
            
            if (prev) {
                prev->next = newPoint;
            } else {
                contourHead = newPoint;
            }
        }
    }
}

// Get height of contour at x-coordinate
int PlacementSolver::getContourHeight(int x) {
    if (contourHead == nullptr) return 0;
    
    ContourPoint* curr = contourHead;
    while (curr->next != nullptr && curr->next->x <= x) {
        curr = curr->next;
    }
    
    return curr->height;
}

/**
 * Enhanced buildInitialBStarTree with intelligent node arrangement
 * Replace the existing buildInitialBStarTree in PlacementSolver
 */
// Fix for buildInitialBStarTree function to prevent creating cycles
// Enhanced buildInitialBStarTree Implementation for Balanced Tree
/**
 * Builds a more balanced B*-tree for global placement
 * The key improvement is to generate a tree that uses both left and right children
 * to create a more compact placement
 */
void PlacementSolver::buildInitialBStarTree() {
    // Clean up any existing tree
    cleanupBStarTree(bstarRoot);
    bstarRoot = nullptr;
    
    logGlobalPlacement("======== BUILDING INITIAL GLOBAL B*-TREE ========");
    
    // Get all module and island names
    std::vector<std::string> entities;
    std::unordered_map<std::string, std::pair<int, int>> entityDimensions;
    std::unordered_map<std::string, bool> isIslandMap;
    
    // Add symmetry islands
    for (size_t i = 0; i < symmetryIslands.size(); i++) {
        // Skip null islands
        if (!symmetryIslands[i]) {
            logGlobalPlacement("WARNING: nullptr found for island " + std::to_string(i));
            continue;
        }
        
        std::string name = "island_" + std::to_string(i);
        entities.push_back(name);
        entityDimensions[name] = {
            symmetryIslands[i]->getWidth(),
            symmetryIslands[i]->getHeight()
        };
        isIslandMap[name] = true;
        
        logGlobalPlacement("Adding symmetry island: " + name + 
                          " width=" + std::to_string(symmetryIslands[i]->getWidth()) + 
                          " height=" + std::to_string(symmetryIslands[i]->getHeight()));
    }
    
    // Add regular modules
    for (const auto& pair : regularModules) {
        // Skip null modules
        if (!pair.second) {
            logGlobalPlacement("WARNING: nullptr found for module " + pair.first);
            continue;
        }
        
        entities.push_back(pair.first);
        entityDimensions[pair.first] = {
            pair.second->getWidth(),
            pair.second->getHeight()
        };
        isIslandMap[pair.first] = false;
        
        logGlobalPlacement("Adding regular module: " + pair.first + 
                          " width=" + std::to_string(pair.second->getWidth()) + 
                          " height=" + std::to_string(pair.second->getHeight()));
    }
    
    // Sort entities by area (descending) to place larger modules first
    std::sort(entities.begin(), entities.end(), [&](const std::string& a, const std::string& b) {
        int areaA = entityDimensions[a].first * entityDimensions[a].second;
        int areaB = entityDimensions[b].first * entityDimensions[b].second;
        return areaA > areaB;  // Descending order
    });
    
    logGlobalPlacement("Entities sorted by area (descending):");
    for (const auto& entity : entities) {
        logGlobalPlacement(" - " + entity + " [" + 
                          std::to_string(entityDimensions[entity].first) + "x" + 
                          std::to_string(entityDimensions[entity].second) + "]");
    }
    
    // Create nodes for all entities
    std::unordered_map<std::string, BStarNode*> nodeMap;
    for (const auto& name : entities) {
        bool isIsland = isIslandMap[name];
        nodeMap[name] = new BStarNode(name, isIsland);
        logGlobalPlacement("Created node for: " + name + " (isIsland: " + 
                          (isIsland ? "true" : "false") + ")");
    }
    
    // Keep track of nodes already placed as children to prevent multiple parents
    std::unordered_set<std::string> placedAsChild;
    
    // Build a tree using BFS approach for a balanced tree
    if (!entities.empty()) {
        // Start with the first entity as the root
        bstarRoot = nodeMap[entities[0]];
        placedAsChild.insert(entities[0]);
        logGlobalPlacement("Set root to: " + entities[0]);
        
        // Queue for BFS traversal
        std::queue<BStarNode*> nodeQueue;
        nodeQueue.push(bstarRoot);
        
        // Now assign the rest of the entities
        size_t entityIndex = 1;
        
        while (!nodeQueue.empty() && entityIndex < entities.size()) {
            BStarNode* currentParent = nodeQueue.front();
            nodeQueue.pop();
            
            // Try to add left child (placed to the right of parent)
            if (entityIndex < entities.size()) {
                const std::string& leftChildName = entities[entityIndex++];
                BStarNode* leftChild = nodeMap[leftChildName];
                
                currentParent->left = leftChild;
                placedAsChild.insert(leftChildName);
                nodeQueue.push(leftChild);
                
                logGlobalPlacement("Placed " + leftChildName + " as left child of " + currentParent->name);
            }
            
            // Try to add right child (placed on top of parent)
            if (entityIndex < entities.size()) {
                const std::string& rightChildName = entities[entityIndex++];
                BStarNode* rightChild = nodeMap[rightChildName];
                
                currentParent->right = rightChild;
                placedAsChild.insert(rightChildName);
                nodeQueue.push(rightChild);
                
                logGlobalPlacement("Placed " + rightChildName + " as right child of " + currentParent->name);
            }
        }
        
        // Check if all entities were placed
        if (placedAsChild.size() != entities.size()) {
            logGlobalPlacement("WARNING: Not all entities were placed in the tree!");
            
            // This should never happen with BFS, but let's add a safety check
            for (const auto& entity : entities) {
                if (placedAsChild.find(entity) == placedAsChild.end()) {
                    logGlobalPlacement("Entity not placed: " + entity);
                }
            }
        }
    }
    
    // Log the final tree structure
    logGlobalPlacement("Final B*-tree structure:");
    printBStarTree(bstarRoot, "", true);
    
    // Do a final validation to make sure the tree is well-formed
    if (!validateBStarTree()) {
        logGlobalPlacement("WARNING: Initial B*-tree validation failed. The tree may have structural issues.");
    } else {
        // Log how many nodes are in the tree
        size_t totalNodes = 0;
        std::function<void(BStarNode*)> countNodes = [&](BStarNode* n) {
            if (n == nullptr) return;
            totalNodes++;
            countNodes(n->left);
            countNodes(n->right);
        };
        countNodes(bstarRoot);
        
        logGlobalPlacement("Tree building complete: " + std::to_string(totalNodes) + 
                          " nodes out of " + std::to_string(entities.size()) + " entities");
    }
}


// Modified preorder traversal method - stores names instead of pointers
void PlacementSolver::safePreorder(BStarNode* node) {
    if (node == nullptr) return;
    
    // Store name instead of pointer
    preorderNodeNames.push_back(node->name);
    safePreorder(node->left);
    safePreorder(node->right);
}

// Modified inorder traversal method - stores names instead of pointers
void PlacementSolver::safeInorder(BStarNode* node) {
    if (node == nullptr) return;
    
    safeInorder(node->left);
    // Store name instead of pointer
    inorderNodeNames.push_back(node->name);
    safeInorder(node->right);
}

// Safe preorder traversal for whole BStarTree
void PlacementSolver::preorder(BStarNode* node) {
    if (node == nullptr) return;
    
    preorderTraversal.push_back(node);
    preorder(node->left);
    preorder(node->right);
}

// Safe inorder traversal for whole BStarTree
void PlacementSolver::inorder(BStarNode* node) {
    if (node == nullptr) return;
    
    inorder(node->left);
    inorderTraversal.push_back(node);
    inorder(node->right);
}

// Helper method to find a node by name
PlacementSolver::BStarNode* PlacementSolver::findNodeByName(const std::string& name) {
    // Traverse the tree to find the node with the given name
    std::function<BStarNode*(BStarNode*)> find = [&](BStarNode* current) -> BStarNode* {
        if (current == nullptr) return nullptr;
        if (current->name == name) return current;
        
        BStarNode* leftResult = find(current->left);
        if (leftResult) return leftResult;
        
        return find(current->right);
    };
    
    return find(bstarRoot);
}

// Fixed packBStarTree function to properly handle BFS traversal
/**
 * Pack the B*-tree to get the coordinates of all modules and islands
 * Ensures proper traversal of the tree structure to place all modules
 */
void PlacementSolver::packBStarTree() {
    // Clear the contour
    clearContour();
    
    if (globalDebugEnabled) {
        logGlobalPlacement("======== PACKING GLOBAL B*-TREE ========");
        logContour();
    }
    
    // Validate tree structure before packing
    if (!validateBStarTree()) {
        logGlobalPlacement("ERROR: Invalid tree structure detected before packing. Attempting to rebuild tree.");
        buildInitialBStarTree();
        if (!validateBStarTree()) {
            logGlobalPlacement("CRITICAL ERROR: Still unable to build a valid tree. Aborting packing.");
            return;
        }
    }
    
    // Initialize node positions
    std::unordered_map<BStarNode*, std::pair<int, int>> nodePositions;
    
    // Use level-order traversal (BFS) to ensure parents are processed before children
    std::queue<BStarNode*> bfsQueue;
    // Track visited nodes to prevent processing the same node twice
    std::unordered_set<BStarNode*> visited;
    
    if (bstarRoot != nullptr) {
        bfsQueue.push(bstarRoot);
        visited.insert(bstarRoot);
        
        // Root is placed at (0,0)
        nodePositions[bstarRoot] = {0, 0};
        
        // Update the module/island position
        if (bstarRoot->isSymmetryIsland) {
            // Extract island index from name (format: "island_X")
            size_t islandIndex = std::stoi(bstarRoot->name.substr(7));
            if (islandIndex < symmetryIslands.size()) {
                symmetryIslands[islandIndex]->setPosition(0, 0);
                updateContour(0, 0, 
                             symmetryIslands[islandIndex]->getWidth(), 
                             symmetryIslands[islandIndex]->getHeight());
                
                logGlobalPlacement("Placed root (island_" + std::to_string(islandIndex) + ") at (0,0)");
            } else {
                logGlobalPlacement("ERROR: Invalid island index for root: " + bstarRoot->name);
            }
        } else {
            // Regular module
            if (regularModules.find(bstarRoot->name) != regularModules.end()) {
                regularModules[bstarRoot->name]->setPosition(0, 0);
                updateContour(0, 0, 
                             regularModules[bstarRoot->name]->getWidth(), 
                             regularModules[bstarRoot->name]->getHeight());
                
                logGlobalPlacement("Placed root (" + bstarRoot->name + ") at (0,0)");
            } else {
                logGlobalPlacement("ERROR: Root module not found: " + bstarRoot->name);
            }
        }
    }
    
    // Track the maximum x and y coordinates
    int maxX = 0;
    int maxY = 0;
    
    // Process the queue
    while (!bfsQueue.empty()) {
        BStarNode* node = bfsQueue.front();
        bfsQueue.pop();
        
        // Skip null nodes
        if (!node) {
            logGlobalPlacement("WARNING: Encountered null node in BFS queue");
            continue;
        }
        
        logGlobalPlacement("Processing node: " + node->name);
        
        // Get current node position
        int nodeX = nodePositions[node].first;
        int nodeY = nodePositions[node].second;
        
        // Get current node dimensions
        int width = 0;
        int height = 0;
        
        if (node->isSymmetryIsland) {
            // Extract island index
            size_t islandIndex = std::stoi(node->name.substr(7));
            if (islandIndex < symmetryIslands.size() && symmetryIslands[islandIndex]) {
                width = symmetryIslands[islandIndex]->getWidth();
                height = symmetryIslands[islandIndex]->getHeight();
            } else {
                logGlobalPlacement("ERROR: Invalid symmetry island: " + node->name);
                continue;
            }
        } else {
            // Regular module
            if (regularModules.find(node->name) != regularModules.end() && 
                regularModules[node->name]) {
                width = regularModules[node->name]->getWidth();
                height = regularModules[node->name]->getHeight();
            } else {
                logGlobalPlacement("ERROR: Regular module not found: " + node->name);
                continue;
            }
        }
        
        // Update max coordinates
        maxX = std::max(maxX, nodeX + width);
        maxY = std::max(maxY, nodeY + height);
        
        // Process left child (placed to the right of current node)
        if (node->left && visited.find(node->left) == visited.end()) {
            int leftX = nodeX + width;
            int leftY = nodeY;
            
            // Log before placing
            logGlobalPlacement("Placing left child " + node->left->name + 
                              " at (" + std::to_string(leftX) + "," + 
                              std::to_string(leftY) + ")");
            
            // Store position of left child
            nodePositions[node->left] = {leftX, leftY};
            visited.insert(node->left);
            
            // Update module/island position
            if (node->left->isSymmetryIsland) {
                // Symmetry island
                size_t islandIndex = std::stoi(node->left->name.substr(7));
                if (islandIndex < symmetryIslands.size() && symmetryIslands[islandIndex]) {
                    symmetryIslands[islandIndex]->setPosition(leftX, leftY);
                    updateContour(leftX, leftY, 
                                 symmetryIslands[islandIndex]->getWidth(), 
                                 symmetryIslands[islandIndex]->getHeight());
                } else {
                    logGlobalPlacement("ERROR: Invalid symmetry island for left child: " + node->left->name);
                }
            } else {
                // Regular module
                if (regularModules.find(node->left->name) != regularModules.end() && 
                    regularModules[node->left->name]) {
                    regularModules[node->left->name]->setPosition(leftX, leftY);
                    updateContour(leftX, leftY, 
                                 regularModules[node->left->name]->getWidth(), 
                                 regularModules[node->left->name]->getHeight());
                } else {
                    logGlobalPlacement("ERROR: Regular module not found for left child: " + node->left->name);
                }
            }
            
            // Add to queue for further processing
            bfsQueue.push(node->left);
        } else if (node->left) {
            logGlobalPlacement("WARNING: Left child " + node->left->name + 
                              " has already been visited (cycle detected)");
        }
        
        // Process right child (placed at same x, above current node)
        if (node->right && visited.find(node->right) == visited.end()) {
            int rightX = nodeX;
            int rightY = nodeY + height;
            
            // Log before placing
            logGlobalPlacement("Placing right child " + node->right->name + 
                              " at (" + std::to_string(rightX) + "," + 
                              std::to_string(rightY) + ")");
            
            // Store position of right child
            nodePositions[node->right] = {rightX, rightY};
            visited.insert(node->right);
            
            // Update module/island position
            if (node->right->isSymmetryIsland) {
                // Symmetry island
                size_t islandIndex = std::stoi(node->right->name.substr(7));
                if (islandIndex < symmetryIslands.size() && symmetryIslands[islandIndex]) {
                    symmetryIslands[islandIndex]->setPosition(rightX, rightY);
                    updateContour(rightX, rightY, 
                                 symmetryIslands[islandIndex]->getWidth(), 
                                 symmetryIslands[islandIndex]->getHeight());
                } else {
                    logGlobalPlacement("ERROR: Invalid symmetry island for right child: " + node->right->name);
                }
            } else {
                // Regular module
                if (regularModules.find(node->right->name) != regularModules.end() && 
                    regularModules[node->right->name]) {
                    regularModules[node->right->name]->setPosition(rightX, rightY);
                    updateContour(rightX, rightY, 
                                 regularModules[node->right->name]->getWidth(), 
                                 regularModules[node->right->name]->getHeight());
                } else {
                    logGlobalPlacement("ERROR: Regular module not found for right child: " + node->right->name);
                }
            }
            
            // Add to queue for further processing
            bfsQueue.push(node->right);
        } else if (node->right) {
            logGlobalPlacement("WARNING: Right child " + node->right->name + 
                              " has already been visited (cycle detected)");
        }
        
        // Log the current contour
        if (globalDebugEnabled) {
            logContour();
        }
    }
    
    // Count how many nodes were visited vs how many should be in the tree
    size_t totalNodes = 0;
    std::function<void(BStarNode*)> countNodes = [&](BStarNode* n) {
        if (n == nullptr) return;
        totalNodes++;
        countNodes(n->left);
        countNodes(n->right);
    };
    countNodes(bstarRoot);
    
    logGlobalPlacement("Visited " + std::to_string(visited.size()) + " nodes out of " + 
                      std::to_string(totalNodes) + " total nodes in tree");
    
    // Log the resulting bounding box
    logGlobalPlacement("Final bounding box: (" + std::to_string(maxX) + "," + 
                      std::to_string(maxY) + ") with area " + std::to_string(maxX * maxY));
    
    // Update traversal lists with names instead of pointers
    preorderNodeNames.clear();
    inorderNodeNames.clear();
    safePreorder(bstarRoot);
    safeInorder(bstarRoot);
    
    // Do a final validation to ensure the tree remains valid after packing
    if (!validateBStarTree()) {
        logGlobalPlacement("WARNING: Tree validation failed after packing.");
    }
}

// Calculate bounding box area
int PlacementSolver::calculateArea() {
    int minX = 0;
    int minY = 0;
    int maxX = 0;
    int maxY = 0;
    
    // Check symmetry islands
    for (const auto& island : symmetryIslands) {
        maxX = std::max(maxX, island->getX() + island->getWidth());
        maxY = std::max(maxY, island->getY() + island->getHeight());
    }
    
    // Check regular modules
    for (const auto& pair : regularModules) {
        const auto& module = pair.second;
        maxX = std::max(maxX, module->getX() + module->getWidth());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    return maxX * maxY;
}

// Calculate half-perimeter wirelength (placeholder)
double PlacementSolver::calculateWirelength() {
    // This is a placeholder - in a real implementation, you would calculate
    // the wirelength based on module connectivity information
    
    // For this simplified version, we'll use a proxy: sum of distances from modules to the origin
    double wirelength = 0.0;
    
    // Include symmetry islands
    for (const auto& island : symmetryIslands) {
        wirelength += island->getX() + island->getY();
    }
    
    // Include regular modules
    for (const auto& pair : regularModules) {
        const auto& module = pair.second;
        wirelength += module->getX() + module->getY();
    }
    
    return wirelength;
}

// Calculate cost of current solution
double PlacementSolver::calculateCost() {
    int area = calculateArea();
    double wirelength = calculateWirelength();
    
    return areaWeight * area + wirelengthWeight * wirelength;
}

// Perform random perturbation
bool PlacementSolver::perturb() {
    double rand = static_cast<double>(std::rand()) / RAND_MAX;
    double cumulativeProb = 0.0;
    
    // Backup the tree structure before any perturbation
    backupBStarTree();
    
    bool success = false;
    
    // Select perturbation type based on probabilities
    if ((cumulativeProb += rotateProb) > rand) {
        // Rotate a module
        BStarNode* node = findRandomNode();
        if (node) {
            success = rotateModule(node->isSymmetryIsland, node->name);
        }
    } else if ((cumulativeProb += moveProb) > rand) {
        // Move a node
        BStarNode* node = findRandomNode();
        if (node) {
            success = moveNode(node);
        }
    } else if ((cumulativeProb += swapProb) > rand) {
        // Swap two nodes
        success = swapNodes();
    } else if ((cumulativeProb += changeRepProb) > rand) {
        // Change representative
        success = changeRepresentative();
    } else if ((cumulativeProb += convertSymProb) > rand) {
        // Convert symmetry type
        success = convertSymmetryType();
    }
    
    // Validate the tree after perturbation
    if (success && !validateBStarTree()) {
        // If validation fails, restore from backup
        restoreBStarTree();
        success = false;
    }
    
    return success;
}

// Rotate a module
bool PlacementSolver::rotateModule(bool isSymmetryIsland, const std::string& name) {
    if (isSymmetryIsland) {
        // Extract island index
        size_t islandIndex = std::stoi(name.substr(7));
        if (islandIndex < symmetryIslands.size()) {
            // Rotate the symmetry island
            symmetryIslands[islandIndex]->rotate();
            return true;
        }
    } else {
        // Rotate a regular module
        if (regularModules.find(name) != regularModules.end()) {
            regularModules[name]->rotate();
            return true;
        }
    }
    
    return false;
}

// Move a node in the B*-tree
bool PlacementSolver::moveNode(BStarNode* node) {
    if (node == nullptr || node == bstarRoot) {
        return false;
    }
    
    // Before perturbation, backup the tree structure
    backupBStarTree();
    
    // Find the parent of the node to move
    BStarNode* parent = nullptr;
    BStarNode* current = bstarRoot;
    std::queue<BStarNode*> queue;
    queue.push(current);
    
    while (!queue.empty()) {
        current = queue.front();
        queue.pop();
        
        if (current->left == node || current->right == node) {
            parent = current;
            break;
        }
        
        if (current->left) queue.push(current->left);
        if (current->right) queue.push(current->right);
    }
    
    if (parent == nullptr) {
        return false;
    }
    
    // Detach node from its parent
    if (parent->left == node) {
        parent->left = nullptr;
    } else {
        parent->right = nullptr;
    }
    
    // Find potential new parents (exclude descendants of node to prevent cycles)
    std::vector<BStarNode*> potentialParents;
    std::unordered_set<BStarNode*> descendants;
    
    // First collect all descendants of the node
    std::function<void(BStarNode*)> collectDescendants = [&](BStarNode* n) {
        if (n == nullptr) return;
        descendants.insert(n);
        collectDescendants(n->left);
        collectDescendants(n->right);
    };
    collectDescendants(node);
    
    // Now collect all nodes that are not descendants
    std::function<void(BStarNode*)> collectNonDescendants = [&](BStarNode* n) {
        if (n == nullptr) return;
        if (descendants.find(n) == descendants.end()) {
            potentialParents.push_back(n);
        }
        collectNonDescendants(n->left);
        collectNonDescendants(n->right);
    };
    collectNonDescendants(bstarRoot);
    
    if (potentialParents.empty()) {
        // Restore original connection and return failure
        if (parent->left == nullptr) {
            parent->left = node;
        } else {
            parent->right = node;
        }
        return false;
    }
    
    // Try to find a parent with an available child slot
    bool placed = false;
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(potentialParents.begin(), potentialParents.end(), g);
    
    for (BStarNode* newParent : potentialParents) {
        // Try left child first if it's empty
        if (newParent->left == nullptr) {
            newParent->left = node;
            placed = true;
            break;
        }
        // Try right child if it's empty
        else if (newParent->right == nullptr) {
            newParent->right = node;
            placed = true;
            break;
        }
    }
    
    // If we couldn't place the node (all potential parents have both children),
    // find a leaf node to attach to
    if (!placed) {
        // Find leaf nodes (nodes with at least one nullptr child)
        std::vector<BStarNode*> leafNodes;
        for (BStarNode* potential : potentialParents) {
            if (potential->left == nullptr || potential->right == nullptr) {
                leafNodes.push_back(potential);
            }
        }
        
        if (!leafNodes.empty()) {
            // Select a random leaf node
            BStarNode* leafNode = leafNodes[std::rand() % leafNodes.size()];
            
            // Add to whichever child pointer is nullptr
            if (leafNode->left == nullptr) {
                leafNode->left = node;
                placed = true;
            } else if (leafNode->right == nullptr) {
                leafNode->right = node;
                placed = true;
            }
        }
    }
    
    // If still not placed, restore original position and return failure
    if (!placed) {
        if (parent->left == nullptr) {
            parent->left = node;
        } else {
            parent->right = node;
        }
        return false;
    }
    
    // Validate the resulting tree structure
    if (!validateBStarTree()) {
        // If invalid, restore from backup and return failure
        restoreBStarTree();
        return false;
    }
    
    return true;
}

// Swap two nodes in the B*-tree
bool PlacementSolver::swapNodes() {
    // Need at least 2 nodes
    if (preorderTraversal.size() < 2) {
        return false;
    }
    
    // Select two random nodes
    int idx1 = std::rand() % preorderTraversal.size();
    int idx2;
    do {
        idx2 = std::rand() % preorderTraversal.size();
    } while (idx1 == idx2);
    
    BStarNode* node1 = preorderTraversal[idx1];
    BStarNode* node2 = preorderTraversal[idx2];
    
    // Swap node contents (name and isSymmetryIsland flag)
    std::swap(node1->name, node2->name);
    std::swap(node1->isSymmetryIsland, node2->isSymmetryIsland);
    
    return true;
}

// Change representative for a symmetry pair
bool PlacementSolver::changeRepresentative() {
    // Select a random symmetry island
    if (symmetryIslands.empty()) {
        return false;
    }
    
    size_t islandIndex = std::rand() % symmetryIslands.size();
    auto island = symmetryIslands[islandIndex];
    
    // Get the ASF-B*-tree from the island
    auto asfBStarTree = island->getASFBStarTree();
    
    // Perturb the ASF-B*-tree by changing a representative
    return asfBStarTree->perturb(3); // Type 3 is "change representative"
}

// Convert symmetry type for a symmetry group
bool PlacementSolver::convertSymmetryType() {
    // Select a random symmetry island
    if (symmetryIslands.empty()) {
        return false;
    }
    
    size_t islandIndex = std::rand() % symmetryIslands.size();
    auto island = symmetryIslands[islandIndex];
    
    // Get the ASF-B*-tree from the island
    auto asfBStarTree = island->getASFBStarTree();
    
    // Perturb the ASF-B*-tree by converting symmetry type
    return asfBStarTree->perturb(4); // Type 4 is "convert symmetry type"
}

// Added tree validation for PlacementSolver
/**
 * Improved validation function to better identify issues
 */
bool PlacementSolver::validateBStarTree() {
    if (bstarRoot == nullptr) return true;
    
    logGlobalPlacement("Validating tree structure...");
    
    // Set to keep track of visited nodes
    std::unordered_set<BStarNode*> visited;
    
    // Set to track node parents (to check for multiple parents issue)
    std::unordered_map<BStarNode*, BStarNode*> parentMap;
    
    // Function to check for cycles in the tree
    std::function<bool(BStarNode*, BStarNode*, std::unordered_set<BStarNode*>&)> hasNoCycles =
        [&](BStarNode* current, BStarNode* parent, std::unordered_set<BStarNode*>& path) -> bool {
            if (current == nullptr) return true;
            
            // If we've seen this node in the current path, we have a cycle
            if (path.find(current) != path.end()) {
                logGlobalPlacement("CYCLE DETECTED at node: " + current->name);
                return false;
            }
            
            // Check if this node already has a different parent (multiple parents issue)
            if (parentMap.find(current) != parentMap.end()) {
                if (parentMap[current] != parent && parent != nullptr) {
                    logGlobalPlacement("MULTIPLE PARENTS DETECTED for node: " + current->name + 
                                      " (Parents: " + parentMap[current]->name + " and " + parent->name + ")");
                    return false;
                }
            } else if (parent != nullptr) {
                parentMap[current] = parent;
            }
            
            // Add this node to the current path
            path.insert(current);
            visited.insert(current);
            
            // Check children
            bool leftValid = hasNoCycles(current->left, current, path);
            bool rightValid = hasNoCycles(current->right, current, path);
            
            // Remove this node from the current path (backtracking)
            path.erase(current);
            
            return leftValid && rightValid;
        };
    
    // Start DFS from the root to check for cycles
    std::unordered_set<BStarNode*> path;
    bool noCycles = hasNoCycles(bstarRoot, nullptr, path);
    
    if (!noCycles) {
        logGlobalPlacement("Tree validation FAILED: Cycles detected");
        return false;
    }
    
    // Make sure each node in the tree is reachable from the root
    // First count all nodes in the tree
    size_t totalNodes = 0;
    std::function<void(BStarNode*)> countNodes = [&](BStarNode* n) {
        if (n == nullptr) return;
        totalNodes++;
        countNodes(n->left);
        countNodes(n->right);
    };
    countNodes(bstarRoot);
    
    logGlobalPlacement("Total nodes in tree: " + std::to_string(totalNodes));
    logGlobalPlacement("Visited nodes during validation: " + std::to_string(visited.size()));
    
    // Make sure we visited all nodes in the cycle check
    if (visited.size() != totalNodes) {
        logGlobalPlacement("Tree validation FAILED: Not all nodes are reachable from root");
        return false;
    }
    
    // Verify that all nodes in the tree reference valid modules/islands
    std::function<bool(BStarNode*)> verifyEntitiesExist = [&](BStarNode* n) -> bool {
        if (n == nullptr) return true;
        
        if (n->isSymmetryIsland) {
            // Check if island exists
            try {
                size_t islandIndex = std::stoi(n->name.substr(7));
                if (islandIndex >= symmetryIslands.size() || !symmetryIslands[islandIndex]) {
                    logGlobalPlacement("Tree validation FAILED: Node " + n->name + " references invalid symmetry island");
                    return false;
                }
            } catch (const std::exception& e) {
                logGlobalPlacement("Tree validation FAILED: Invalid island name format: " + n->name);
                return false;
            }
        } else {
            // Check if module exists
            if (regularModules.find(n->name) == regularModules.end() || !regularModules[n->name]) {
                logGlobalPlacement("Tree validation FAILED: Node " + n->name + " references invalid regular module");
                return false;
            }
        }
        
        return verifyEntitiesExist(n->left) && verifyEntitiesExist(n->right);
    };
    
    bool entitiesValid = verifyEntitiesExist(bstarRoot);
    if (!entitiesValid) {
        return false;
    }
    
    logGlobalPlacement("Tree validation PASSED: Valid tree structure with " + std::to_string(totalNodes) + " nodes");
    return true;
}

// Methods for backing up and restoring B*-tree structure in PlacementSolver
void PlacementSolver::backupBStarTree() {
    // Clear and update the traversal name lists first
    preorderNodeNames.clear();
    inorderNodeNames.clear();
    safePreorder(bstarRoot);
    safeInorder(bstarRoot);
    
    // Store the current tree structure using node names
    bstarTreeBackup.preorderNodes.clear();
    bstarTreeBackup.inorderNodes.clear();
    
    for (const auto& name : preorderNodeNames) {
        BStarNode* node = findNodeByName(name);
        if (node) {
            bstarTreeBackup.preorderNodes.push_back({name, node->isSymmetryIsland});
        }
    }
    
    for (const auto& name : inorderNodeNames) {
        BStarNode* node = findNodeByName(name);
        if (node) {
            bstarTreeBackup.inorderNodes.push_back({name, node->isSymmetryIsland});
        }
    }
}

// Restore B*-tree from backup
void PlacementSolver::restoreBStarTree() {
    if (bstarTreeBackup.preorderNodes.empty() || bstarTreeBackup.inorderNodes.empty()) {
        return; // No backup available
    }
    
    // Clean up existing tree
    cleanupBStarTree(bstarRoot);
    bstarRoot = nullptr;
    
    // Create nodes for all entities
    std::unordered_map<std::string, BStarNode*> nodeMap;
    for (const auto& info : bstarTreeBackup.preorderNodes) {
        if (nodeMap.find(info.name) == nodeMap.end()) {
            nodeMap[info.name] = new BStarNode(info.name, info.isIsland);
        }
    }
    
    // Create mapping from name to inorder index
    std::unordered_map<std::string, size_t> inorderMap;
    for (size_t i = 0; i < bstarTreeBackup.inorderNodes.size(); i++) {
        inorderMap[bstarTreeBackup.inorderNodes[i].name] = i;
    }
    
    // Recursive function to rebuild the tree
    std::function<BStarNode*(size_t&, size_t, size_t)> rebuildTree = 
        [&](size_t& preIdx, size_t inStart, size_t inEnd) -> BStarNode* {
            if (inStart > inEnd || preIdx >= bstarTreeBackup.preorderNodes.size()) {
                return nullptr;
            }
            
            // Get the root of current subtree from preorder traversal
            const auto& nodeInfo = bstarTreeBackup.preorderNodes[preIdx++];
            BStarNode* node = nodeMap[nodeInfo.name];
            
            // If this is the only element in this subtree
            if (inStart == inEnd) {
                return node;
            }
            
            // Find the index of current node in inorder traversal
            size_t inIndex = inorderMap[nodeInfo.name];
            
            // Recursively build left and right subtrees
            if (inIndex > inStart) {
                node->left = rebuildTree(preIdx, inStart, inIndex - 1);
            }
            if (inIndex < inEnd) {
                node->right = rebuildTree(preIdx, inIndex + 1, inEnd);
            }
            
            return node;
        };
    
    // Rebuild the tree
    size_t preIdx = 0;
    bstarRoot = rebuildTree(preIdx, 0, bstarTreeBackup.inorderNodes.size() - 1);
}

/**
 * Enhanced check for module overlaps with detailed diagnostics
 */
bool PlacementSolver::hasOverlaps() {
    logGlobalPlacement("======== CHECKING FOR MODULE OVERLAPS ========");
    
    // Define a helper function to log module bounds safely
    auto logModuleBounds = [this](const std::string& name, int x, int y, int width, int height) {
        std::stringstream ss;
        ss << name << ": (" << x << "," << y << ") to (" 
           << (x + width) << "," << (y + height) << ") [" 
           << width << "x" << height << "]";
        logGlobalPlacement(ss.str());
    };
    
    // Check overlaps between regular modules
    logGlobalPlacement("--- Regular Module vs Regular Module Checks ---");
    for (const auto& pair1 : regularModules) {
        const auto& name1 = pair1.first;
        const auto& module1 = pair1.second;
        
        // Skip null modules
        if (!module1) {
            logGlobalPlacement("WARNING: nullptr found for module " + name1);
            continue;
        }
        
        int m1Left = module1->getX();
        int m1Right = m1Left + module1->getWidth();
        int m1Bottom = module1->getY();
        int m1Top = m1Bottom + module1->getHeight();
        
        logModuleBounds(name1, m1Left, m1Bottom, module1->getWidth(), module1->getHeight());
        
        // Check against other regular modules
        for (const auto& pair2 : regularModules) {
            const auto& name2 = pair2.first;
            if (name1 == name2) continue; // Skip self
            
            const auto& module2 = pair2.second;
            
            // Skip null modules
            if (!module2) {
                logGlobalPlacement("WARNING: nullptr found for module " + name2);
                continue;
            }
            
            int m2Left = module2->getX();
            int m2Right = m2Left + module2->getWidth();
            int m2Bottom = module2->getY();
            int m2Top = m2Bottom + module2->getHeight();
            
            bool xOverlap = m1Right > m2Left && m2Right > m1Left;
            bool yOverlap = m1Top > m2Bottom && m2Top > m1Bottom;
            bool overlaps = xOverlap && yOverlap;
            
            if (overlaps) {
                std::stringstream ss;
                ss << "OVERLAP DETECTED: " << name1 << " and " << name2;
                logGlobalPlacement(ss.str());
                logModuleBounds(name2, m2Left, m2Bottom, module2->getWidth(), module2->getHeight());
                return true;
            }
        }
    }
    
    // Check overlaps between regular modules and symmetry islands
    logGlobalPlacement("--- Regular Module vs Symmetry Island Checks ---");
    for (const auto& pair : regularModules) {
        const auto& name = pair.first;
        const auto& module = pair.second;
        
        // Skip null modules
        if (!module) {
            logGlobalPlacement("WARNING: nullptr found for module " + name);
            continue;
        }
        
        int mLeft = module->getX();
        int mRight = mLeft + module->getWidth();
        int mBottom = module->getY();
        int mTop = mBottom + module->getHeight();
        
        logModuleBounds(name, mLeft, mBottom, module->getWidth(), module->getHeight());
        
        // Check against all symmetry islands
        for (size_t i = 0; i < symmetryIslands.size(); i++) {
            const auto& island = symmetryIslands[i];
            
            // Skip null islands
            if (!island) {
                logGlobalPlacement("WARNING: nullptr found for island " + std::to_string(i));
                continue;
            }
            
            std::string islandName = "island_" + std::to_string(i);
            
            int islandLeft = island->getX();
            int islandRight = islandLeft + island->getWidth();
            int islandBottom = island->getY();
            int islandTop = islandBottom + island->getHeight();
            
            logModuleBounds(islandName, islandLeft, islandBottom, island->getWidth(), island->getHeight());
            
            bool xOverlap = mRight > islandLeft && islandRight > mLeft;
            bool yOverlap = mTop > islandBottom && islandTop > mBottom;
            bool overlaps = xOverlap && yOverlap;
            
            if (overlaps) {
                std::stringstream ss;
                ss << "OVERLAP DETECTED: " << name << " and " << islandName;
                logGlobalPlacement(ss.str());
                return true;
            }
        }
    }
    
    // Check overlaps between symmetry islands
    logGlobalPlacement("--- Symmetry Island vs Symmetry Island Checks ---");
    for (size_t i = 0; i < symmetryIslands.size(); i++) {
        for (size_t j = i + 1; j < symmetryIslands.size(); j++) {
            const auto& island1 = symmetryIslands[i];
            const auto& island2 = symmetryIslands[j];
            
            // Skip null islands
            if (!island1 || !island2) {
                logGlobalPlacement("WARNING: nullptr found for islands " + 
                                  std::to_string(i) + " or " + std::to_string(j));
                continue;
            }
            
            std::string islandName1 = "island_" + std::to_string(i);
            std::string islandName2 = "island_" + std::to_string(j);
            
            int i1Left = island1->getX();
            int i1Right = i1Left + island1->getWidth();
            int i1Bottom = island1->getY();
            int i1Top = i1Bottom + island1->getHeight();
            
            int i2Left = island2->getX();
            int i2Right = i2Left + island2->getWidth();
            int i2Bottom = island2->getY();
            int i2Top = i2Bottom + island2->getHeight();
            
            logModuleBounds(islandName1, i1Left, i1Bottom, island1->getWidth(), island1->getHeight());
            logModuleBounds(islandName2, i2Left, i2Bottom, island2->getWidth(), island2->getHeight());
            
            bool xOverlap = i1Right > i2Left && i2Right > i1Left;
            bool yOverlap = i1Top > i2Bottom && i2Top > i1Bottom;
            bool overlaps = xOverlap && yOverlap;
            
            if (overlaps) {
                std::stringstream ss;
                ss << "OVERLAP DETECTED: " << islandName1 << " and " << islandName2;
                logGlobalPlacement(ss.str());
                return true;
            }
        }
    }
    
    logGlobalPlacement("No overlaps detected");
    return false;
}


// Deep copy of module data
std::map<std::string, std::shared_ptr<Module>> PlacementSolver::copyModules(
    const std::map<std::string, std::shared_ptr<Module>>& source) {
    
    std::map<std::string, std::shared_ptr<Module>> result;
    
    for (const auto& pair : source) {
        result[pair.first] = std::make_shared<Module>(*pair.second);
    }
    
    return result;
}

// Find a random node in the B*-tree
PlacementSolver::BStarNode* PlacementSolver::findRandomNode() {
    if (preorderTraversal.empty()) {
        return nullptr;
    }
    
    return preorderTraversal[std::rand() % preorderTraversal.size()];
}

// Copy current solution to best solution
void PlacementSolver::updateBestSolution() {
    bestSolutionArea = solutionArea;
    bestSolutionWirelength = solutionWirelength;
    
    // Update all modules in the solution
    bestSolutionModules.clear();
    
    // Add regular modules
    for (const auto& pair : regularModules) {
        bestSolutionModules[pair.first] = std::make_shared<Module>(*pair.second);
    }
    
    // Add modules from symmetry islands
    for (const auto& island : symmetryIslands) {
        for (const auto& pair : island->getASFBStarTree()->getModules()) {
            bestSolutionModules[pair.first] = std::make_shared<Module>(*pair.second);
        }
    }
}

// Copy best solution to current solution
void PlacementSolver::restoreBestSolution() {
    // Restore module positions from best solution
    for (const auto& pair : bestSolutionModules) {
        const std::string& name = pair.first;
        const auto& module = pair.second;
        
        // Check if it's a regular module
        if (regularModules.find(name) != regularModules.end()) {
            regularModules[name]->setPosition(module->getX(), module->getY());
            regularModules[name]->setRotation(module->getRotated());
        } else {
            // It's in a symmetry island - find and update it
            for (const auto& island : symmetryIslands) {
                const auto& islandModules = island->getASFBStarTree()->getModules();
                if (islandModules.find(name) != islandModules.end()) {
                    islandModules.at(name)->setPosition(module->getX(), module->getY());
                    islandModules.at(name)->setRotation(module->getRotated());
                    break;
                }
            }
        }
    }
    
    // Update solution metrics
    solutionArea = bestSolutionArea;
    solutionWirelength = bestSolutionWirelength;
}

// Load the problem data
bool PlacementSolver::loadProblem(
    const std::map<std::string, std::shared_ptr<Module>>& modules,
    const std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups) {
    
    // Clear current data
    this->modules = modules;
    this->symmetryGroups = symmetryGroups;
    
    regularModules.clear();
    symmetryIslands.clear();
    
    // Create symmetry islands
    for (size_t i = 0; i < symmetryGroups.size(); i++) {
        auto symmetryGroup = symmetryGroups[i];
        
        // Collect modules in this symmetry group
        std::map<std::string, std::shared_ptr<Module>> groupModules;
        
        // Add symmetry pairs
        for (const auto& pair : symmetryGroup->getSymmetryPairs()) {
            if (modules.find(pair.first) != modules.end()) {
                groupModules[pair.first] = modules.at(pair.first);
            }
            if (modules.find(pair.second) != modules.end()) {
                groupModules[pair.second] = modules.at(pair.second);
            }
        }
        
        // Add self-symmetric modules
        for (const auto& name : symmetryGroup->getSelfSymmetric()) {
            if (modules.find(name) != modules.end()) {
                groupModules[name] = modules.at(name);
            }
        }
        
        // Create ASF-B*-tree for this symmetry group
        auto asfBStarTree = std::make_shared<ASFBStarTree>(symmetryGroup, groupModules);
        
        // Pack the ASF-B*-tree to get initial layout
        asfBStarTree->pack();
        
        // Create symmetry island
        auto island = std::make_shared<SymmetryIslandBlock>("sg_" + std::to_string(i), asfBStarTree);
        
        // Update bounding box
        island->updateBoundingBox();
        
        symmetryIslands.push_back(island);
    }
    
    // Collect regular modules (not in any symmetry group)
    std::unordered_set<std::string> symmetryModules;
    
    for (const auto& group : symmetryGroups) {
        for (const auto& pair : group->getSymmetryPairs()) {
            symmetryModules.insert(pair.first);
            symmetryModules.insert(pair.second);
        }
        for (const auto& name : group->getSelfSymmetric()) {
            symmetryModules.insert(name);
        }
    }
    
    for (const auto& pair : modules) {
        if (symmetryModules.find(pair.first) == symmetryModules.end()) {
            regularModules[pair.first] = pair.second;
        }
    }
    
    // Build initial B*-tree for global placement
    buildInitialBStarTree();
    
    return true;
}

// Set simulated annealing parameters
void PlacementSolver::setAnnealingParameters(
    double initialTemp, double finalTemp, double cooling,
    int iterations, int noImprovementLimit) {
    
    initialTemperature = initialTemp;
    finalTemperature = finalTemp;
    coolingRate = cooling;
    iterationsPerTemperature = iterations;
    this->noImprovementLimit = noImprovementLimit;
}

// Set perturbation probabilities
void PlacementSolver::setPerturbationProbabilities(
    double rotate, double move, double swap,
    double changeRep, double convertSym) {
    
    rotateProb = rotate;
    moveProb = move;
    swapProb = swap;
    changeRepProb = changeRep;
    convertSymProb = convertSym;
}

// Set cost weights
void PlacementSolver::setCostWeights(double areaWeight, double wirelengthWeight) {
    this->areaWeight = areaWeight;
    this->wirelengthWeight = wirelengthWeight;
}

// Set random seed
void PlacementSolver::setRandomSeed(unsigned int seed) {
    rng.seed(seed);
    std::srand(seed);
}

// Set time limit
void PlacementSolver::setTimeLimit(int seconds) {
    timeLimit = seconds;
}

// Solve the placement problem
bool PlacementSolver::solve() {
    // Record start time
    startTime = std::chrono::steady_clock::now();
    
    // Before packing, ensure traversal containers are empty
    preorderTraversal.clear();
    inorderTraversal.clear();
    preorderNodeNames.clear();
    inorderNodeNames.clear();
    
    // Initial packing
    packBStarTree();
    
    // Calculate initial solution metrics
    solutionArea = calculateArea();
    solutionWirelength = calculateWirelength();
    
    // Initialize best solution
    bestSolutionArea = solutionArea;
    bestSolutionWirelength = solutionWirelength;
    updateBestSolution();
    
    // Also backup the initial tree structure
    backupBStarTree();
    bestBStarTreeBackup = bstarTreeBackup;
    
    std::cout << "Initial solution - Area: " << solutionArea << ", Cost: " << calculateCost() << std::endl;
    
    // Simulated annealing
    double temperature = initialTemperature;
    int iterations = 0;
    int noImprovementCount = 0;
    
    while (temperature > finalTemperature && noImprovementCount < noImprovementLimit) {
        bool improved = false;
        
        for (int i = 0; i < iterationsPerTemperature; i++) {
            // Check time limit
            auto currentTime = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                currentTime - startTime).count();
            
            if (elapsed >= timeLimit) {
                std::cout << "Time limit reached." << std::endl;
                break;
            }
            
            // Clear traversal containers before each iteration
            preorderTraversal.clear();
            inorderTraversal.clear();
            preorderNodeNames.clear();
            inorderNodeNames.clear();
            
            // Save current state and metrics
            int oldArea = solutionArea;
            double oldWirelength = solutionWirelength;
            double oldCost = calculateCost();
            
            // Backup current tree structure
            backupBStarTree();
            
            // Create backup of module positions
            std::map<std::string, std::pair<int, int>> oldPositions;
            std::map<std::string, bool> oldRotations;
            
            for (const auto& pair : regularModules) {
                if (pair.second) {
                    oldPositions[pair.first] = {pair.second->getX(), pair.second->getY()};
                    oldRotations[pair.first] = pair.second->getRotated();
                }
            }
            
            // Create backup of islands
            std::vector<std::pair<int, int>> oldIslandPositions;
            for (const auto& island : symmetryIslands) {
                if (island) {
                    oldIslandPositions.push_back({island->getX(), island->getY()});
                }
            }
            
            // Perturb the solution
            bool perturbSuccess = perturb();
            
            if (perturbSuccess) {
                // Before re-packing, clear traversal containers
                preorderTraversal.clear();
                inorderTraversal.clear();
                preorderNodeNames.clear();
                inorderNodeNames.clear();
                
                // Re-pack
                packBStarTree();
                
                // Calculate new metrics
                int newArea = calculateArea();
                double newWirelength = calculateWirelength();
                double newCost = areaWeight * newArea + wirelengthWeight * newWirelength;
                
                // Check if we should accept this solution
                double costDelta = newCost - oldCost;
                bool accept = false;
                
                if (costDelta <= 0) {
                    // Accept improvement
                    accept = true;
                    
                    if (newArea < bestSolutionArea) {
                        // Found a new best solution
                        bestSolutionArea = newArea;
                        bestSolutionWirelength = newWirelength;
                        updateBestSolution();
                        bestBStarTreeBackup = bstarTreeBackup; // Save tree structure too
                        improved = true;
                        noImprovementCount = 0;
                        
                        std::cout << "New best solution - Area: " << newArea 
                                  << ", Cost: " << newCost 
                                  << ", Temp: " << temperature << std::endl;
                    }
                } else {
                    // Accept with probability based on temperature
                    double acceptProb = exp(-costDelta / temperature);
                    if (static_cast<double>(std::rand()) / RAND_MAX < acceptProb) {
                        accept = true;
                    }
                }
                
                if (accept) {
                    // Update solution metrics
                    solutionArea = newArea;
                    solutionWirelength = newWirelength;
                } else {
                    // Restore the tree structure - clear traversal containers first
                    preorderTraversal.clear();
                    inorderTraversal.clear();
                    preorderNodeNames.clear();
                    inorderNodeNames.clear();
                    
                    // Restore the tree
                    restoreBStarTree();
                    
                    // Restore old positions
                    for (const auto& pair : oldPositions) {
                        if (regularModules.find(pair.first) != regularModules.end() && 
                            regularModules[pair.first]) {
                            regularModules[pair.first]->setPosition(pair.second.first, pair.second.second);
                            regularModules[pair.first]->setRotation(oldRotations[pair.first]);
                        }
                    }
                    
                    // Restore islands
                    for (size_t i = 0; i < symmetryIslands.size() && i < oldIslandPositions.size(); i++) {
                        if (symmetryIslands[i]) {
                            symmetryIslands[i]->setPosition(oldIslandPositions[i].first, oldIslandPositions[i].second);
                        }
                    }
                    
                    // Restore metrics
                    solutionArea = oldArea;
                    solutionWirelength = oldWirelength;
                }
            }
            
            iterations++;
        }
        
        // Update temperature
        temperature *= coolingRate;
        
        if (!improved) {
            noImprovementCount++;
        }
    }
    
    std::cout << "Annealing complete - " << iterations << " iterations" << std::endl;
    
    // Restore best solution including tree structure
    bstarTreeBackup = bestBStarTreeBackup;
    restoreBStarTree();
    restoreBestSolution();
    
    std::cout << "Final solution - Area: " << solutionArea << std::endl;
    
    return true;
}

// Get solution area
int PlacementSolver::getSolutionArea() const {
    return solutionArea;
}

// Get solution modules
const std::map<std::string, std::shared_ptr<Module>>& PlacementSolver::getSolutionModules() const {
    return bestSolutionModules;
}