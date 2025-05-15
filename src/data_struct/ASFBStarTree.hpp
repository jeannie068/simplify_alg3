// ASFBStarTree.hpp - Revised implementation according to the paper
#pragma once

#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <string>
#include <random>
#include <iostream>
#include <queue>

#include "Module.hpp"
#include "SymmetryConstraint.hpp"
#include "../Logger.hpp"
#include "BStarTree.hpp"

/**
 * @brief Automatically Symmetry-Feasible B*-tree (ASF-B*-tree)
 * 
 * This class implements a B*-tree variant that automatically enforces
 * symmetry constraints for a symmetry group, strictly following the
 * principles in "Analog Placement Based on Symmetry-Island Formulation" by Lin et al.
 */
class ASFBStarTree {
public:
    // Symmetry group represented by this ASF-B*-tree
    std::shared_ptr<SymmetryGroup> symmetryGroup;
    
    // Module map: module name -> module pointer
    std::map<std::string, std::shared_ptr<Module>> modules;
    
    // Map to store representative modules for each symmetry pair and self-symmetric module
    std::map<std::string, std::shared_ptr<Module>> representativeModules;
    
    // Map to store the relationship between representative and non-representative modules
    std::unordered_map<std::string, std::string> pairMap; // non-rep -> rep
    std::unordered_map<std::string, std::string> repToPairMap; // rep -> non-rep
    
    // Self-symmetric modules (center on symmetry axis)
    std::vector<std::string> selfSymmetricModules;
    
    // B*-tree representation (for representatives only)
    struct BStarNode {
        std::string moduleName;
        BStarNode* left;  // Left child: left-adjacent module
        BStarNode* right; // Right child: top-adjacent module
        
        BStarNode(const std::string& name) : moduleName(name), left(nullptr), right(nullptr) {}
    };
    
    // Root of the B*-tree
    BStarNode* root;

    // Backup storage for tree structure
    struct TreeBackup {
        std::vector<std::string> preorderTraversal;
        std::vector<std::string> inorderTraversal;
    };
    TreeBackup treeBackup;
    
    // Current symmetry axis position
    double symmetryAxisPosition;
    
    // Preorder and inorder traversals of the B*-tree
    std::vector<BStarNode*> preorderTraversal;
    std::vector<BStarNode*> inorderTraversal;
    
    // Contour data structure for packing
    SegmentTree<int> contourSegTree;
    int maxContourWidth; // Maximum possible width for contour
    
    /**
     * Constructor
     * 
     * @param symmetryGroup The symmetry group to represent
     * @param modules Map of all modules
     */
    ASFBStarTree(std::shared_ptr<SymmetryGroup> symmetryGroup, const std::map<std::string, std::shared_ptr<Module>>& modules)
        : symmetryGroup(symmetryGroup), modules(modules), root(nullptr), symmetryAxisPosition(-1) {
        
        // Initialize module relationships
        initializeRepresentatives();
        
        // Build an initial B*-tree
        buildInitialBStarTree();
    }
    
    /**
     * Destructor
     */
    ~ASFBStarTree() {
        cleanupTree(root);
        clearContour();
    }
    
    /**
     * Initializes representative modules for the symmetry group
     * following Definition 2 and Definition 3 from the paper
     */
    void initializeRepresentatives() {
        // Clear existing data
        representativeModules.clear();
        pairMap.clear();
        repToPairMap.clear();
        selfSymmetricModules.clear();
        
        // Process symmetry pairs
        for (const auto& pair : symmetryGroup->getSymmetryPairs()) {
            const std::string& module1 = pair.first;
            const std::string& module2 = pair.second;
            
            // According to Definition 2: The representative b_j^r of a symmetry pair (b_j, b'_j) is b'_j
            const std::string& rep = module2;
            const std::string& sym = module1;
            
            representativeModules[rep] = modules[rep];
            pairMap[sym] = rep;
            repToPairMap[rep] = sym;
            
            Logger::log("Added symmetry pair: " + sym + " -> " + rep + " (rep)");
        }
        
        // Process self-symmetric modules
        for (const auto& moduleName : symmetryGroup->getSelfSymmetric()) {
            selfSymmetricModules.push_back(moduleName);
            representativeModules[moduleName] = modules[moduleName];
            
            Logger::log("Added self-symmetric module: " + moduleName);
        }
    }
    
    /**
     * Builds a random initial B*-tree for the symmetry group
     * This ensures self-symmetric modules are on the correct branch
     * according to Property 1 from the paper
     */
    void buildInitialBStarTree();
    
    /**
     * Validates that self-symmetric modules are on the correct branch
     * according to Property 1 from the paper
     */
    bool validateSymmetryConstraints() {
        if (root == nullptr) return true;
        
        // For each self-symmetric module, check if it's on the correct branch
        for (const auto& moduleName : selfSymmetricModules) {
            // Skip the root as it's always valid
            if (root->moduleName == moduleName) continue;
            
            bool foundOnCorrectBranch = false;
            
            // Check if the module is on the correct branch
            BStarNode* current = root;
            while (current != nullptr) {
                if (current->moduleName == moduleName) {
                    foundOnCorrectBranch = true;
                    break;
                }
                
                if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                    current = current->right; // Follow rightmost path
                } else {
                    current = current->left;  // Follow leftmost path
                }
            }
            
            if (!foundOnCorrectBranch) {
                Logger::log("Self-symmetric module " + moduleName + " is not on the correct branch");
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * Validates the tree structure
     */
    bool validateTreeStructure(BStarNode* node) {
        if (node == nullptr) return true;
        
        Logger::log("Validating tree structure starting at " + (node == root ? "root" : node->moduleName));
        
        // Set to keep track of visited nodes
        std::unordered_set<BStarNode*> visited;
        
        // Function to check for cycles in the tree
        std::function<bool(BStarNode*, std::unordered_set<BStarNode*>&, std::string)> hasNoCycles =
            [&](BStarNode* current, std::unordered_set<BStarNode*>& path, std::string pathStr) -> bool {
                if (current == nullptr) return true;
                
                // Log the current path
                std::string currentPathStr = pathStr + " -> " + current->moduleName;
                
                // If we've seen this node in the current path, we have a cycle
                if (path.find(current) != path.end()) {
                    Logger::log("CYCLE DETECTED: " + currentPathStr);
                    return false;
                }
                
                // Add this node to the current path
                path.insert(current);
                visited.insert(current);
                
                // Check children
                bool leftValid = hasNoCycles(current->left, path, currentPathStr);
                bool rightValid = hasNoCycles(current->right, path, currentPathStr);
                
                // Remove this node from the current path (backtracking)
                path.erase(current);
                
                return leftValid && rightValid;
            };
        
        // Start DFS from the root to check for cycles
        std::unordered_set<BStarNode*> path;
        bool noCycles = hasNoCycles(node, path, "START");
        
        if (!noCycles) {
            Logger::log("Tree has cycles!");
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
        countNodes(node);
        
        Logger::log("Total nodes in tree: " + std::to_string(totalNodes));
        Logger::log("Visited nodes during cycle check: " + std::to_string(visited.size()));
        
        // Make sure we visited all nodes in the cycle check
        if (visited.size() != totalNodes) {
            Logger::log("Some nodes are unreachable from the root!");
            return false;
        }
        
        // Also verify that all nodes in the tree are actually in our modules map
        std::function<bool(BStarNode*)> verifyModuleExists = [&](BStarNode* n) -> bool {
            if (n == nullptr) return true;
            
            if (modules.find(n->moduleName) == modules.end()) {
                Logger::log("Node " + n->moduleName + " doesn't exist in modules map!");
                return false;
            }
            
            return verifyModuleExists(n->left) && verifyModuleExists(n->right);
        };
        
        bool allModulesExist = verifyModuleExists(node);
        if (!allModulesExist) {
            Logger::log("Some nodes reference non-existent modules!");
            return false;
        }
        
        Logger::log("Tree structure is valid");
        return true;
    }
    
    /**
     * Clears the contour data structure
     */
    void clearContour() {
        // Calculate maximum possible width for all modules
        maxContourWidth = 0;
        
        // Account for all modules in this symmetry group
        for (const auto& pair : modules) {
            if (pair.second) {
                maxContourWidth += pair.second->getWidth();
            }
        }
        
        // Add some buffer to be safe
        maxContourWidth *= 2;
        
        // Initialize segment tree with appropriate size
        contourSegTree.init(maxContourWidth);
        
        Logger::log("Initialized ASF-B*-tree contour segment tree with width " + std::to_string(maxContourWidth));
    }
    
    /**
     * Updates the contour after placing a module
     */
    void updateContour(int x, int y, int width, int height);
    
    /**
     * Gets the height of the contour at a given x-coordinate
     */
    int getContourHeight(int x) {
        // Bounds checking
        if (x < 0 || x >= maxContourWidth) {
            Logger::log("WARNING: ASF-B*-tree contour query out of bounds: " + std::to_string(x));
            return 0;
        }
        
        // Query segment tree for height at x
        return contourSegTree.query(x, x);
    }
    
    /**
     * Checks if a module is self-symmetric
     */
    bool isSelfSymmetric(const std::string& moduleName) const {
        return std::find(selfSymmetricModules.begin(), selfSymmetricModules.end(), moduleName) != selfSymmetricModules.end();
    }

    /**
     * Checks if a module is a representative (not a symmetric module)
     */
    bool isRepresentative(const std::string& moduleName) const {
        return representativeModules.find(moduleName) != representativeModules.end();
    }
    
    /**
     * Performs a preorder traversal of the B*-tree
     */
    void preorder(BStarNode* node) {
        if (node == nullptr) return;
        
        preorderTraversal.push_back(node);
        preorder(node->left);
        preorder(node->right);
    }
    
    /**
     * Performs an inorder traversal of the B*-tree
     */
    void inorder(BStarNode* node) {
        if (node == nullptr) return;
        
        inorder(node->left);
        inorderTraversal.push_back(node);
        inorder(node->right);
    }
    
    /**
     * Cleans up the B*-tree
     */
    void cleanupTree(BStarNode* node) {
        if (node == nullptr) return;
        
        cleanupTree(node->left);
        cleanupTree(node->right);
        delete node;
    }
    
    /**
     * Packs the B*-tree to get the coordinates of all modules
     * This implementation uses level-order traversal (BFS)
     */
    void packBStarTree();
    
    /**
     * Updates the positions of symmetric modules based on their representatives
     */
    void updateSymmetricModulePositions();
    
    /**
     * Calculates the position of the symmetry axis based on the current placement
     * Using the method described in the paper
     */
    void calculateSymmetryAxisPosition();
    
    /**
     * Packs the ASF-B*-tree to get the coordinates of all modules
     * 
     * @return True if packing was successful
     */
    bool pack();

    bool validateSymmetry() const;

    bool validateConnectivity();

    bool hasContourOverlap(int x, int y, int width, int height);

    void optimizeModulePositions();

    void compactPlacement();

    // Backup/restore methods for the tree structure
    void backupTreeStructure() {
        // Store the current tree structure to enable complete restoration
        treeBackup.preorderTraversal.clear();
        treeBackup.inorderTraversal.clear();
        
        // Populate the backup traversals
        std::function<void(BStarNode*)> preorderBackup = [&](BStarNode* node) {
            if (node == nullptr) return;
            treeBackup.preorderTraversal.push_back(node->moduleName);
            preorderBackup(node->left);
            preorderBackup(node->right);
        };
        
        std::function<void(BStarNode*)> inorderBackup = [&](BStarNode* node) {
            if (node == nullptr) return;
            inorderBackup(node->left);
            treeBackup.inorderTraversal.push_back(node->moduleName);
            inorderBackup(node->right);
        };
        
        preorderBackup(root);
        inorderBackup(root);
    }
    
    // Restore from the backed up tree structure
    void restoreTreeStructure() {
        if (treeBackup.preorderTraversal.empty() || treeBackup.inorderTraversal.empty()) {
            // No backup available
            return;
        }
        
        // Clean up existing tree
        cleanupTree(root);
        root = nullptr;
        
        // Create nodes for all modules
        std::unordered_map<std::string, BStarNode*> nodeMap;
        for (const std::string& name : treeBackup.preorderTraversal) {
            if (nodeMap.find(name) == nodeMap.end()) {
                nodeMap[name] = new BStarNode(name);
            }
        }
        
        // Create mapping from module name to inorder index
        std::unordered_map<std::string, size_t> inorderMap;
        for (size_t i = 0; i < treeBackup.inorderTraversal.size(); i++) {
            inorderMap[treeBackup.inorderTraversal[i]] = i;
        }
        
        // Recursive function to rebuild the tree
        std::function<BStarNode*(size_t&, size_t, size_t)> rebuildTree = 
            [&](size_t& preIdx, size_t inStart, size_t inEnd) -> BStarNode* {
                if (inStart > inEnd || preIdx >= treeBackup.preorderTraversal.size()) {
                    return nullptr;
                }
                
                // Get the root of current subtree from preorder traversal
                std::string moduleName = treeBackup.preorderTraversal[preIdx++];
                BStarNode* node = nodeMap[moduleName];
                
                // If this is the only element in this subtree
                if (inStart == inEnd) {
                    return node;
                }
                
                // Find the index of current node in inorder traversal
                size_t inIndex = inorderMap[moduleName];
                
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
        root = rebuildTree(preIdx, 0, treeBackup.inorderTraversal.size() - 1);
        
        // Validate the restored structure
        if (!validateTreeStructure(root)) {
            throw std::runtime_error("Invalid tree structure after restoration");
        }
    }
    
    /**
     * Gets the symmetry axis position
     */
    double getSymmetryAxisPosition() const {
        return symmetryAxisPosition;
    }
    
    /**
     * Sets the symmetry axis position
     */
    void setSymmetryAxisPosition(double position) {
        symmetryAxisPosition = position;
        symmetryGroup->setAxisPosition(position);
    }
    
    /**
     * Gets the symmetry group
     */
    std::shared_ptr<SymmetryGroup> getSymmetryGroup() const {
        return symmetryGroup;
    }
    
    /**
     * Gets all modules in the symmetry group
     */
    const std::map<std::string, std::shared_ptr<Module>>& getModules() const {
        return modules;
    }
    
    /**
     * Gets the bounding rectangle of the symmetry island
     */
    std::pair<int, int> getBoundingBox() const {
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
        
        return {maxX - minX, maxY - minY};
    }
    
    /**
     * Gets the area of the symmetry island
     */
    int getArea() const {
        auto [width, height] = getBoundingBox();
        return width * height;
    }
    
    /**
     * Rotates all modules in the symmetry group
     * 
     * @param rotate True to rotate, false to revert to original orientation
     */
    void rotateAll(bool rotate) {
        Logger::log("Rotating all modules in symmetry group");
        
        // Rotate all modules
        for (auto& pair : modules) {
            pair.second->setRotation(rotate);
        }
        
        // Toggle the symmetry type
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            symmetryGroup->setType(SymmetryType::HORIZONTAL);
        } else {
            symmetryGroup->setType(SymmetryType::VERTICAL);
        }
        
        // Rebuild the B*-tree to adjust to new dimensions
        buildInitialBStarTree();
        
        // Re-pack to update positions
        pack();
    }
    
    /**
     * Rotates a specific module and its symmetric pair
     */
    void rotateModule(const std::string& moduleName) {
        // Check if the module is in this symmetry group
        if (modules.find(moduleName) == modules.end()) {
            return;
        }
        
        Logger::log("Rotating module " + moduleName);
        
        // Get the module to rotate
        std::shared_ptr<Module> module = modules[moduleName];
        
        // Rotate the module
        module->rotate();
        
        // If it's part of a symmetry pair, rotate the other module too
        if (repToPairMap.find(moduleName) != repToPairMap.end()) {
            modules[repToPairMap[moduleName]]->rotate();
            Logger::log("Also rotating symmetric pair: " + repToPairMap[moduleName]);
        } else if (pairMap.find(moduleName) != pairMap.end()) {
            modules[pairMap[moduleName]]->rotate();
            Logger::log("Also rotating symmetric pair: " + pairMap[moduleName]);
        }
        
        // Rebuild the B*-tree to adjust to new dimensions
        buildInitialBStarTree();
        
        // Re-pack to update positions
        pack();
    }
    
    /**
     * Changes the representative module for a symmetry pair
     * Following the paper's definition and requirements
     */
    bool changeRepresentative(const std::string& moduleName) {
        // Check if the module is in this symmetry group
        if (modules.find(moduleName) == modules.end()) {
            return false;
        }
        
        Logger::log("Attempting to change representative for " + moduleName);
        
        // Check if it's part of a symmetry pair
        if (repToPairMap.find(moduleName) != repToPairMap.end()) {
            // Current rep -> non-rep
            std::string rep = moduleName;
            std::string nonRep = repToPairMap[rep];
            
            Logger::log("Changing representative from " + rep + " to " + nonRep);
            
            // Backup the tree structure
            backupTreeStructure();
            
            // Swap the representative
            representativeModules.erase(rep);
            representativeModules[nonRep] = modules[nonRep];
            
            repToPairMap.erase(rep);
            repToPairMap[nonRep] = rep;
            
            pairMap.erase(nonRep);
            pairMap[rep] = nonRep;
            
            // Rebuild the B*-tree with the new representative
            cleanupTree(root);
            root = nullptr;
            buildInitialBStarTree();
            
            // Check if the new tree maintains symmetry constraints
            if (!validateSymmetryConstraints()) {
                Logger::log("Failed to maintain symmetry constraints, reverting change");
                
                // Revert the change
                representativeModules.erase(nonRep);
                representativeModules[rep] = modules[rep];
                
                repToPairMap.erase(nonRep);
                repToPairMap[rep] = nonRep;
                
                pairMap.erase(rep);
                pairMap[nonRep] = rep;
                
                // Restore the original tree structure
                restoreTreeStructure();
                
                return false;
            }
            
            // Re-pack to update positions
            pack();
            
            return true;
        } else if (pairMap.find(moduleName) != pairMap.end()) {
            // Current non-rep -> rep
            std::string nonRep = moduleName;
            std::string rep = pairMap[nonRep];
            
            Logger::log("Changing representative from " + rep + " to " + nonRep);
            
            // Backup the tree structure
            backupTreeStructure();
            
            // Swap the representative
            representativeModules.erase(rep);
            representativeModules[nonRep] = modules[nonRep];
            
            repToPairMap.erase(rep);
            repToPairMap[nonRep] = rep;
            
            pairMap.erase(nonRep);
            pairMap[rep] = nonRep;
            
            // Rebuild the B*-tree with the new representative
            cleanupTree(root);
            root = nullptr;
            buildInitialBStarTree();
            
            // Check if the new tree maintains symmetry constraints
            if (!validateSymmetryConstraints()) {
                Logger::log("Failed to maintain symmetry constraints, reverting change");
                
                // Revert the change
                representativeModules.erase(nonRep);
                representativeModules[rep] = modules[rep];
                
                repToPairMap.erase(nonRep);
                repToPairMap[rep] = nonRep;
                
                pairMap.erase(rep);
                pairMap[nonRep] = rep;
                
                // Restore the original tree structure
                restoreTreeStructure();
                
                return false;
            }
            
            // Re-pack to update positions
            pack();
            
            return true;
        }
        
        // Not part of a symmetry pair
        Logger::log("Module " + moduleName + " is not part of a symmetry pair");
        return false;
    }
    
    /**
     * Converts the symmetry type between vertical and horizontal
     * according to the paper's approach
     */
    bool convertSymmetryType() {
        std::string fromTo;
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            fromTo = "vertical to horizontal";
        } else {
            fromTo = "horizontal to vertical";
        }
        Logger::log("Converting symmetry type from " + fromTo);
        
        // Backup the tree structure
        backupTreeStructure();
        
        // Toggle the symmetry type
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            symmetryGroup->setType(SymmetryType::HORIZONTAL);
        } else {
            symmetryGroup->setType(SymmetryType::VERTICAL);
        }
        
        // First rotate all modules
        for (auto& pair : modules) {
            pair.second->rotate();
        }
        
        // Then rebuild the B*-tree to adjust to the new symmetry type
        cleanupTree(root);
        root = nullptr;
        buildInitialBStarTree();
        
        // Validate the new tree structure
        if (!validateSymmetryConstraints()) {
            Logger::log("Failed to maintain symmetry constraints after type conversion, reverting");
            
            // Revert the symmetry type
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                symmetryGroup->setType(SymmetryType::HORIZONTAL);
            } else {
                symmetryGroup->setType(SymmetryType::VERTICAL);
            }
            
            // Revert the rotation
            for (auto& pair : modules) {
                pair.second->rotate();
            }
            
            // Restore the original tree structure
            restoreTreeStructure();
            
            return false;
        }
        
        // Re-pack to update positions
        pack();
        
        return true;
    }
    
    /**
     * Performs a random perturbation on the B*-tree
     * 
     * @param perturbationType 0: rotate, 1: move, 2: swap, 3: change representative, 4: convert symmetry type
     * @return True if perturbation was successful
     */
    bool perturb(int perturbationType) {
        Logger::log("ASF-B*-tree perturbation type " + std::to_string(perturbationType));
        
        // Backup tree structure before perturbation
        backupTreeStructure();
        
        bool success = false;
        
        try {
            switch (perturbationType) {
                case 0: // Rotate
                    {
                        // Rotate a random module (avoid self-symmetric if possible)
                        std::vector<std::string> moduleNames;
                        for (const auto& pair : modules) {
                            if (!isSelfSymmetric(pair.first)) {
                                moduleNames.push_back(pair.first);
                            }
                        }
                        
                        // If no non-self-symmetric modules, include all modules
                        if (moduleNames.empty()) {
                            for (const auto& pair : modules) {
                                moduleNames.push_back(pair.first);
                            }
                        }
                        
                        if (!moduleNames.empty()) {
                            std::string randomModule = moduleNames[std::rand() % moduleNames.size()];
                            Logger::log("Rotating module " + randomModule);
                            rotateModule(randomModule);
                            success = true;
                        }
                    }
                    break;
                    
                case 1: // Move (rebuild tree with different topology)
                    {
                        Logger::log("Rebuilding tree with different topology");
                        
                        // Ensure we don't lose the original representatives
                        std::map<std::string, std::shared_ptr<Module>> originalReps = representativeModules;
                        
                        // Just rebuild the tree randomly
                        buildInitialBStarTree();
                        
                        // Re-pack to update positions
                        success = pack();
                        
                        // If failed, restore original representatives
                        if (!success) {
                            representativeModules = originalReps;
                            // Restore the original tree structure
                            restoreTreeStructure();
                        }
                    }
                    break;
                    
                case 2: // Swap two nodes in the tree
                    {
                        // Get all non-self-symmetric representative modules
                        std::vector<std::string> repModules;
                        for (const auto& pair : representativeModules) {
                            if (!isSelfSymmetric(pair.first)) {
                                repModules.push_back(pair.first);
                            }
                        }
                        
                        // Need at least 2 non-self-symmetric modules to swap
                        if (repModules.size() >= 2) {
                            // Select two random modules
                            int idx1 = std::rand() % repModules.size();
                            int idx2;
                            do {
                                idx2 = std::rand() % repModules.size();
                            } while (idx1 == idx2);
                            
                            Logger::log("Swapping " + repModules[idx1] + " and " + repModules[idx2]);
                            
                            // Locate the nodes in the B*-tree
                            BStarNode* node1 = nullptr;
                            BStarNode* node2 = nullptr;
                            
                            std::function<void(BStarNode*)> findNodes = [&](BStarNode* node) {
                                if (node == nullptr) return;
                                
                                if (node->moduleName == repModules[idx1]) {
                                    node1 = node;
                                } else if (node->moduleName == repModules[idx2]) {
                                    node2 = node;
                                }
                                
                                findNodes(node->left);
                                findNodes(node->right);
                            };
                            
                            findNodes(root);
                            
                            if (node1 != nullptr && node2 != nullptr) {
                                Logger::log("Found both nodes, swapping module names");
                                
                                // Check if either node is on the critical boundary path
                                bool node1OnCriticalPath = false;
                                bool node2OnCriticalPath = false;
                                
                                // Check for node1
                                BStarNode* current = root;
                                while (current != nullptr) {
                                    if (current == node1) {
                                        node1OnCriticalPath = true;
                                        break;
                                    }
                                    
                                    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                                        current = current->right; // Follow rightmost path
                                    } else {
                                        current = current->left;  // Follow leftmost path
                                    }
                                }
                                
                                // Check for node2
                                current = root;
                                while (current != nullptr) {
                                    if (current == node2) {
                                        node2OnCriticalPath = true;
                                        break;
                                    }
                                    
                                    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                                        current = current->right; // Follow rightmost path
                                    } else {
                                        current = current->left;  // Follow leftmost path
                                    }
                                }
                                
                                // Reject swap if only one node is on critical path 
                                // and it's a self-symmetric module
                                if ((node1OnCriticalPath && !node2OnCriticalPath && 
                                     isSelfSymmetric(node1->moduleName)) ||
                                    (node2OnCriticalPath && !node1OnCriticalPath && 
                                     isSelfSymmetric(node2->moduleName))) {
                                    Logger::log("Cannot swap - would violate Property 1 for self-symmetric modules");
                                    return false;
                                }
                                
                                // Swap the module names
                                std::swap(node1->moduleName, node2->moduleName);
                                
                                // Re-pack to update positions
                                Logger::log("Re-packing after swap");
                                success = pack();
                                
                                // If packing failed, restore the original node names
                                if (!success) {
                                    std::swap(node1->moduleName, node2->moduleName);
                                    // Restore the original tree structure
                                    restoreTreeStructure();
                                }
                            } else {
                                Logger::log("Failed to find one or both nodes in the tree");
                            }
                        }
                    }
                    break;
                    
                case 3: // Change representative
                    {
                        // Get all symmetry pairs
                        std::vector<std::string> symPairs;
                        for (const auto& pair : repToPairMap) {
                            symPairs.push_back(pair.first);
                        }
                        
                        if (!symPairs.empty()) {
                            // Select a random symmetry pair
                            std::string randomRep = symPairs[std::rand() % symPairs.size()];
                            Logger::log("Changing representative for " + randomRep);
                            success = changeRepresentative(randomRep);
                        }
                    }
                    break;
                    
                case 4: // Convert symmetry type
                    {
                        Logger::log("Converting symmetry type");
                        success = convertSymmetryType();
                    }
                    break;
            }
        } catch (const std::exception& e) {
            Logger::log("Exception during perturbation: " + std::string(e.what()));
            
            // Attempt to restore the tree structure
            Logger::log("Attempting to restore tree structure after exception");
            restoreTreeStructure();
            return false;
        }
        
        // Validate tree structure after perturbation
        if (success) {
            Logger::log("Validating tree structure after perturbation");
            if (!validateTreeStructure(root)) {
                Logger::log("Invalid tree structure after perturbation, restoring");
                restoreTreeStructure();
                success = false;
            } else {
                Logger::log("Tree structure valid after perturbation");
            }
        }
        
        return success;
    }
};