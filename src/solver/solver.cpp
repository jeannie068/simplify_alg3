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
    
    // Define sampling parameters
    const int maxSamples = 50; // Maximum number of points to show
    int samplingInterval = std::max(1, maxContourWidth / maxSamples);
    
    // Track previous height to detect changes
    int prevHeight = contourSegTree.query(0, 0);
    ss << "(0," << prevHeight << ") ";
    
    // Sample the contour at regular intervals
    for (int x = samplingInterval; x < maxContourWidth; x += samplingInterval) {
        int height = contourSegTree.query(x, x);
        
        // Always include points where height changes
        if (height != prevHeight) {
            ss << "(" << x << "," << height << ") ";
            prevHeight = height;
        }
    }
    
    // Always include rightmost point
    if (maxContourWidth > 0) {
        int rightHeight = contourSegTree.query(maxContourWidth-1, maxContourWidth-1);
        ss << "(" << (maxContourWidth-1) << "," << rightHeight << ")";
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
    : bstarRoot(nullptr),
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

// Clear contour data structure
void PlacementSolver::clearContour() {
    // Calculate maximum possible width for all modules
    maxContourWidth = 0;
    
    // Account for regular modules
    for (const auto& pair : regularModules) {
        if (pair.second) {
            maxContourWidth += pair.second->getWidth();
        }
    }
    
    // Account for symmetry islands
    for (const auto& island : symmetryIslands) {
        if (island) {
            maxContourWidth += island->getWidth();
        }
    }
    
    // Add some buffer to be safe
    maxContourWidth *= 2;
    
    // Initialize segment tree with appropriate size
    contourSegTree.init(maxContourWidth);
    
    logGlobalPlacement("Initialized contour segment tree with width " + std::to_string(maxContourWidth));
}

// Update contour after placing a module
void PlacementSolver::updateContour(int x, int y, int width, int height) {
    // Additional validations
    if (width <= 0 || height <= 0) {
        logGlobalPlacement("ERROR: Invalid module dimensions in updateContour: width=" + 
                          std::to_string(width) + ", height=" + std::to_string(height));
        return;
    }
    
    // Check if trying to place negative coordinates - this indicates a serious problem
    if (x < 0 || y < 0) {
        logGlobalPlacement("ERROR: Cannot place module at negative coordinates: (" + 
                          std::to_string(x) + "," + std::to_string(y) + ")");
        if (x < 0) x = 0;
        if (y < 0) y = 0;
    }
    
    // Bounds checking - limit to valid range
    int adjX = x;
    int adjWidth = width;
    
    if (adjX + adjWidth > maxContourWidth) {
        logGlobalPlacement("WARNING: Module extends beyond maximum contour width");
        // If it's far beyond the bound, we need to increase maxContourWidth
        if (adjX + adjWidth > maxContourWidth * 1.5) {
            int newMaxWidth = (adjX + adjWidth) * 2; // Double to avoid frequent resizes
            logGlobalPlacement("Increasing contour max width from " + 
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
    
    // Debug logging for significant height increase
    int heightIncrease = top - currentHeight;
    if (heightIncrease > height * 2) {
        logGlobalPlacement("Large height increase at x=" + std::to_string(adjX) + 
                          ": " + std::to_string(currentHeight) + " -> " + 
                          std::to_string(top) + " (+" + std::to_string(heightIncrease) + ")");
    }
    
    // Only update if new height is higher
    if (top > currentHeight) {
        contourSegTree.update(adjX, adjX + adjWidth - 1, top);
    }
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
        // Instead of logContour() we'll log the max height
        int maxHeight = contourSegTree.query(0, maxContourWidth - 1);
        logGlobalPlacement("Initial contour max height: " + std::to_string(maxHeight));
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
            int leftY = getContourHeight(leftX);
            
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
            // Instead of old logContour()
            int maxHeight = contourSegTree.query(0, maxContourWidth - 1);
            logGlobalPlacement("Current contour max height: " + std::to_string(maxHeight));
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

    compactGlobalPlacement();
    
    // Do a final validation to ensure the tree remains valid after packing
    if (!validateBStarTree()) {
        logGlobalPlacement("WARNING: Tree validation failed after packing.");
    }
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

// Add a post-processing step to fix any overlaps that might still occur
bool PlacementSolver::fixOverlaps() {
    logGlobalPlacement("Beginning comprehensive overlap fixing process");
    
    bool hadOverlaps = hasOverlaps();
    if (!hadOverlaps) {
        logGlobalPlacement("No overlaps detected, no fixing needed");
        return true;
    }
    
    int iterations = 0;
    const int MAX_ITERATIONS = 10;
    
    while (hasOverlaps() && iterations < MAX_ITERATIONS) {
        iterations++;
        logGlobalPlacement("Overlap fixing iteration " + std::to_string(iterations));
        
        std::vector<ModuleInfo> modules;
        
        // Add regular modules
        for (const auto& pair : regularModules) {
            if (!pair.second) continue;
            
            modules.emplace_back(
                pair.first,
                false,
                pair.second->getX(),
                pair.second->getY(),
                pair.second->getWidth(),
                pair.second->getHeight()
            );
        }
        
        // Add symmetry islands
        for (size_t i = 0; i < symmetryIslands.size(); i++) {
            if (!symmetryIslands[i]) continue;
            
            modules.emplace_back(
                "island_" + std::to_string(i),
                true,
                symmetryIslands[i]->getX(),
                symmetryIslands[i]->getY(),
                symmetryIslands[i]->getWidth(),
                symmetryIslands[i]->getHeight()
            );
        }
        
        logGlobalPlacement("Collected info for " + std::to_string(modules.size()) + " modules");
        
        // Stage 2: Sort modules by different criteria to try multiple approaches
        bool fixed = false;
        
        // Approach 1: Sort by area (largest first) and resolve
        std::sort(modules.begin(), modules.end(), [](const ModuleInfo& a, const ModuleInfo& b) {
            return a.area > b.area; // Largest first
        });
        
        logGlobalPlacement("Approach 1: Resolving overlaps by module area (largest first)");
        if (resolveOverlapsInSortedOrder(modules)) {
            fixed = true;
            logGlobalPlacement("Approach 1 successfully resolved all overlaps");
            break;
        }
        
        // Approach 2: Sort by y-coordinate (bottom to top) and resolve
        std::sort(modules.begin(), modules.end(), [](const ModuleInfo& a, const ModuleInfo& b) {
            return a.y < b.y;
        });
        
        logGlobalPlacement("Approach 2: Resolving overlaps by y-coordinate (bottom to top)");
        if (resolveOverlapsInSortedOrder(modules)) {
            fixed = true;
            logGlobalPlacement("Approach 2 successfully resolved all overlaps");
            break;
        }
        
        // Approach 3: Sort by x-coordinate (left to right) and resolve
        std::sort(modules.begin(), modules.end(), [](const ModuleInfo& a, const ModuleInfo& b) {
            return a.x < b.x;
        });
        
        logGlobalPlacement("Approach 3: Resolving overlaps by x-coordinate (left to right)");
        if (resolveOverlapsInSortedOrder(modules)) {
            fixed = true;
            logGlobalPlacement("Approach 3 successfully resolved all overlaps");
            break;
        }
        
        // If all approaches failed, try more drastic measures
        if (!fixed) {
            logGlobalPlacement("All standard approaches failed, trying grid-based placement");
            if (gridBasedPlacement()) {
                fixed = true;
                logGlobalPlacement("Grid-based placement successfully resolved all overlaps");
                break;
            }
        }
    }
    
    bool success = !hasOverlaps();
    if (success) {
        logGlobalPlacement("Successfully fixed all overlaps in " + std::to_string(iterations) + " iterations");
    } else {
        logGlobalPlacement("Failed to fix all overlaps after " + std::to_string(iterations) + " iterations");
    }
    
    return success;
}

// Helper method to resolve overlaps in the current order of modules
bool PlacementSolver::resolveOverlapsInSortedOrder(const std::vector<ModuleInfo>& sortedModules) {
    // Create a temporary contour to track used space
    int maxWidth = 0;
    for (const auto& module : sortedModules) {
        maxWidth = std::max(maxWidth, module.x + module.width);
    }
    
    // Add buffer to avoid edge cases
    maxWidth = maxWidth * 2;
    
    SegmentTree<int> placementContour;
    placementContour.init(maxWidth);
    
    // Place each module in order, avoiding overlaps
    for (const auto& module : sortedModules) {
        // Try to place at original position first
        int x = module.x;
        int y = module.y;
        
        // Check if original position overlaps with placed modules
        int contourHeight = placementContour.query(x, x + module.width - 1);
        if (contourHeight > y) {
            // Overlap detected, find new position
            y = contourHeight;
        }
        
        // Update module position
        if (module.isIsland) {
            // It's a symmetry island
            size_t islandIndex = std::stoi(module.name.substr(7));
            if (islandIndex < symmetryIslands.size() && symmetryIslands[islandIndex]) {
                logGlobalPlacement("Moving island_" + std::to_string(islandIndex) + 
                                  " from (" + std::to_string(module.x) + "," + 
                                  std::to_string(module.y) + ") to (" + 
                                  std::to_string(x) + "," + std::to_string(y) + ")");
                symmetryIslands[islandIndex]->setPosition(x, y);
            }
        } else {
            // It's a regular module
            if (regularModules.find(module.name) != regularModules.end() && 
                regularModules[module.name]) {
                logGlobalPlacement("Moving module " + module.name + 
                                  " from (" + std::to_string(module.x) + "," + 
                                  std::to_string(module.y) + ") to (" + 
                                  std::to_string(x) + "," + std::to_string(y) + ")");
                regularModules[module.name]->setPosition(x, y);
            }
        }
        
        // Update contour with the new placement
        placementContour.update(x, x + module.width - 1, y + module.height);
    }
    
    // Check if this fixed the overlaps
    return !hasOverlaps();
}

// Grid-based placement approach as a last resort
bool PlacementSolver::gridBasedPlacement() {
    logGlobalPlacement("Attempting grid-based placement");
    
    // Collect all module dimensions
    std::vector<std::pair<std::string, std::pair<int, int>>> moduleInfo; // (name, (width, height))
    
    // Add regular modules
    for (const auto& pair : regularModules) {
        if (!pair.second) continue;
        moduleInfo.push_back({
            pair.first,
            {pair.second->getWidth(), pair.second->getHeight()}
        });
    }
    
    // Add symmetry islands
    for (size_t i = 0; i < symmetryIslands.size(); i++) {
        if (!symmetryIslands[i]) continue;
        moduleInfo.push_back({
            "island_" + std::to_string(i),
            {symmetryIslands[i]->getWidth(), symmetryIslands[i]->getHeight()}
        });
    }
    
    // Sort modules by area (largest first)
    std::sort(moduleInfo.begin(), moduleInfo.end(), [](const auto& a, const auto& b) {
        int areaA = a.second.first * a.second.second;
        int areaB = b.second.first * b.second.second;
        return areaA > areaB;
    });
    
    // Initialize grid
    int gridWidth = 0;
    int totalArea = 0;
    
    // Calculate total area to get a sense of required grid size
    for (const auto& module : moduleInfo) {
        totalArea += module.second.first * module.second.second;
        gridWidth = std::max(gridWidth, module.second.first);
    }
    
    // Estimate grid height assuming square aspect ratio
    int estimatedSide = static_cast<int>(std::sqrt(totalArea * 1.5)); // Add 50% buffer
    gridWidth = std::max(gridWidth, estimatedSide);
    
    // Create a grid to track used space (1 = occupied, 0 = free)
    std::vector<std::vector<bool>> grid(gridWidth * 2, std::vector<bool>(gridWidth * 2, false));
    
    // Place each module in the grid
    for (const auto& module : moduleInfo) {
        const std::string& name = module.first;
        int width = module.second.first;
        int height = module.second.second;
        
        bool placed = false;
        
        // Try to find a free spot in the grid
        for (int y = 0; y < grid[0].size() - height && !placed; y++) {
            for (int x = 0; x < grid.size() - width && !placed; x++) {
                // Check if this spot is free
                bool spotIsFree = true;
                for (int dy = 0; dy < height && spotIsFree; dy++) {
                    for (int dx = 0; dx < width && spotIsFree; dx++) {
                        if (grid[x + dx][y + dy]) {
                            spotIsFree = false;
                        }
                    }
                }
                
                // If spot is free, place the module here
                if (spotIsFree) {
                    // Mark grid cells as occupied
                    for (int dy = 0; dy < height; dy++) {
                        for (int dx = 0; dx < width; dx++) {
                            grid[x + dx][y + dy] = true;
                        }
                    }
                    
                    // Update module position
                    if (name.substr(0, 7) == "island_") {
                        // It's a symmetry island
                        size_t islandIndex = std::stoi(name.substr(7));
                        if (islandIndex < symmetryIslands.size() && symmetryIslands[islandIndex]) {
                            logGlobalPlacement("Grid placing island_" + std::to_string(islandIndex) + 
                                             " at (" + std::to_string(x) + "," + std::to_string(y) + ")");
                            symmetryIslands[islandIndex]->setPosition(x, y);
                        }
                    } else {
                        // It's a regular module
                        if (regularModules.find(name) != regularModules.end() && 
                            regularModules[name]) {
                            logGlobalPlacement("Grid placing module " + name + 
                                             " at (" + std::to_string(x) + "," + std::to_string(y) + ")");
                            regularModules[name]->setPosition(x, y);
                        }
                    }
                    
                    placed = true;
                }
            }
        }
        
        if (!placed) {
            logGlobalPlacement("ERROR: Could not place module " + name + " in grid");
            return false;
        }
    }
    
    // Check if this fixed the overlaps
    return !hasOverlaps();
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
    solutionWirelength = 0;
    
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
                double newWirelength = 0;
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

void PlacementSolver::compactGlobalPlacement() {
    logGlobalPlacement("Performing global compaction to remove unnecessary gaps");
    
    std::vector<ModuleInfo> modules;
    
    // Add regular modules
    for (const auto& pair : regularModules) {
        if (!pair.second) continue;
        
        ModuleInfo info{
            pair.first,
            false,
            pair.second->getX(),
            pair.second->getY(),
            pair.second->getWidth(),
            pair.second->getHeight()
        };
        modules.push_back(info);
    }
    
    // Add symmetry islands
    for (size_t i = 0; i < symmetryIslands.size(); i++) {
        if (!symmetryIslands[i]) continue;
        
        ModuleInfo info{
            "island_" + std::to_string(i),
            true,
            symmetryIslands[i]->getX(),
            symmetryIslands[i]->getY(),
            symmetryIslands[i]->getWidth(),
            symmetryIslands[i]->getHeight()
        };
        modules.push_back(info);
    }
    
    // Sort modules by y-coordinate for vertical compaction
    std::sort(modules.begin(), modules.end(), [](const ModuleInfo& a, const ModuleInfo& b) {
        return a.y < b.y;
    });
    
    // Compact modules vertically
    int currentY = 0;
    for (size_t i = 0; i < modules.size(); i++) {
        ModuleInfo& module = modules[i];
        
        // Check if any module is below (y < module.y) but has top > currentY
        // that would block our compaction
        bool blocked = false;
        for (size_t j = 0; j < i; j++) {
            const ModuleInfo& prevModule = modules[j];
            
            // Check if modules overlap horizontally
            bool xOverlap = module.x < prevModule.right() && prevModule.x < module.right();
            
            // Check if prevModule extends above currentY
            bool yExtends = prevModule.top() > currentY;
            
            if (xOverlap && yExtends) {
                // This module blocks our compaction, adjust currentY
                currentY = prevModule.top();
                blocked = true;
            }
        }
        
        // If module can be moved down, update its position
        if (module.y > currentY) {
            int yDiff = module.y - currentY;
            logGlobalPlacement("Compacting " + module.name + " down by " + 
                              std::to_string(yDiff) + " pixels");
            
            module.y = currentY;
            
            // Update the actual module/island position
            if (module.isIsland) {
                size_t islandIndex = std::stoi(module.name.substr(7));
                if (islandIndex < symmetryIslands.size() && symmetryIslands[islandIndex]) {
                    symmetryIslands[islandIndex]->setPosition(module.x, module.y);
                }
            } else {
                if (regularModules.find(module.name) != regularModules.end() && 
                    regularModules[module.name]) {
                    regularModules[module.name]->setPosition(module.x, module.y);
                }
            }
        }
        
        // Update currentY for next module
        currentY = module.y + module.height;
    }
    
    // Sort modules by x-coordinate for horizontal compaction
    std::sort(modules.begin(), modules.end(), [](const ModuleInfo& a, const ModuleInfo& b) {
        return a.x < b.x;
    });
    
    // Compact modules horizontally (similar logic as vertical compaction)
    int currentX = 0;
    for (size_t i = 0; i < modules.size(); i++) {
        ModuleInfo& module = modules[i];
        
        // Check if any module is to the left (x < module.x) but has right > currentX
        // that would block our compaction
        bool blocked = false;
        for (size_t j = 0; j < i; j++) {
            const ModuleInfo& prevModule = modules[j];
            
            // Check if modules overlap vertically
            bool yOverlap = module.y < prevModule.top() && prevModule.y < module.top();
            
            // Check if prevModule extends beyond currentX
            bool xExtends = prevModule.right() > currentX;
            
            if (yOverlap && xExtends) {
                // This module blocks our compaction, adjust currentX
                currentX = prevModule.right();
                blocked = true;
            }
        }
        
        // If module can be moved left, update its position
        if (module.x > currentX) {
            int xDiff = module.x - currentX;
            logGlobalPlacement("Compacting " + module.name + " left by " + 
                              std::to_string(xDiff) + " pixels");
            
            module.x = currentX;
            
            // Update the actual module/island position
            if (module.isIsland) {
                size_t islandIndex = std::stoi(module.name.substr(7));
                if (islandIndex < symmetryIslands.size() && symmetryIslands[islandIndex]) {
                    symmetryIslands[islandIndex]->setPosition(module.x, module.y);
                }
            } else {
                if (regularModules.find(module.name) != regularModules.end() && 
                    regularModules[module.name]) {
                    regularModules[module.name]->setPosition(module.x, module.y);
                }
            }
        }
        
        // Update currentX for next module
        currentX = module.x + module.width;
    }
}