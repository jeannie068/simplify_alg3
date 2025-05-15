#pragma once

#include <map>
#include <vector>
#include <memory>
#include <string>
#include <random>
#include <chrono>
#include <functional>
#include <iostream>

#include "../data_struct/Module.hpp"
#include "../data_struct/SymmetryConstraint.hpp"
#include "../data_struct/ASFBStarTree.hpp"
#include "../data_struct/SymmetryIslandBlock.hpp"
#include "../data_struct/BStarTree.hpp"

/**
 * @brief Placement solver using simulated annealing
 * 
 * This class implements a simulated annealing algorithm for analog placement
 * with symmetry constraints. It handles both global placement optimization
 * and local symmetry island optimization.
 */
class PlacementSolver {
public:
    // Input data
    std::map<std::string, std::shared_ptr<Module>> modules;
    std::vector<std::shared_ptr<SymmetryGroup>> symmetryGroups;
    
    // Symmetry islands (one for each symmetry group)
    std::vector<std::shared_ptr<SymmetryIslandBlock>> symmetryIslands;
    
    // Regular modules (not in symmetry groups)
    std::map<std::string, std::shared_ptr<Module>> regularModules;
    
    // B*-tree for global placement
    struct BStarNode {
        std::string name;
        bool isSymmetryIsland;
        BStarNode* left;
        BStarNode* right;
        
        BStarNode(const std::string& name, bool isSymmetryIsland)
            : name(name), isSymmetryIsland(isSymmetryIsland), left(nullptr), right(nullptr) {}
    };
    
    BStarNode* bstarRoot;

    // Backup storage for B*-tree structure
    struct TreeNodeInfo {
        std::string name;
        bool isIsland;
    };

    // Stage 1: Collect all module information
    struct ModuleInfo {
        std::string name;
        bool isIsland;
        int x, y, width, height;
        int area;
        
        // Constructor for easy creation
        ModuleInfo(const std::string& n, bool island, int posX, int posY, int w, int h)
            : name(n), isIsland(island), x(posX), y(posY), width(w), height(h) {
            area = width * height;
        }
        
        // Helper methods for boundary calculations
        int right() const { return x + width; }
        int top() const { return y + height; }
    };
    
    struct BStarTreeBackup {
        std::vector<TreeNodeInfo> preorderNodes;
        std::vector<TreeNodeInfo> inorderNodes;
    };
    
    BStarTreeBackup bstarTreeBackup;
    BStarTreeBackup bestBStarTreeBackup;

    // Logger members
    std::ofstream globalLogFile;
    bool globalDebugEnabled;
    void initGlobalDebugger();
    void logGlobalPlacement(const std::string& message);
    void logContour();
    void printBStarTree(BStarNode *node, std::string prefix, bool isLast);
    
    // Current solution
    std::map<std::string, std::shared_ptr<Module>> solutionModules;
    int solutionArea;
    double solutionWirelength;
    
    // Best solution found so far
    std::map<std::string, std::shared_ptr<Module>> bestSolutionModules;
    int bestSolutionArea;
    double bestSolutionWirelength;
    
    // Simulated annealing parameters
    double initialTemperature;
    double finalTemperature;
    double coolingRate;
    int iterationsPerTemperature;
    int noImprovementLimit;
    
    // Perturbation probabilities
    double rotateProb;
    double moveProb;
    double swapProb;
    double changeRepProb;
    double convertSymProb;
    
    // Cost function weights
    double areaWeight;
    double wirelengthWeight;
    
    // Random number generation
    std::mt19937 rng;
    
    // Time limit in seconds
    int timeLimit;
    std::chrono::steady_clock::time_point startTime;
    
    // Contour data structure for packing
    SegmentTree<int> contourSegTree;
    int maxContourWidth; // Maximum possible width for contour
    
    // Preorder and inorder traversals for B*-tree
    std::vector<BStarNode*> preorderTraversal;
    std::vector<BStarNode*> inorderTraversal;
    // Use vectors of names instead of pointers for safer traversals
    std::vector<std::string> preorderNodeNames;
    std::vector<std::string> inorderNodeNames;
    
    // Safe traversal methods
    void safePreorder(BStarNode* node) {
        if (node == nullptr) return;
        
        // Store name instead of pointer
        preorderNodeNames.push_back(node->name);
        safePreorder(node->left);
        safePreorder(node->right);
    }

    void safeInorder(BStarNode* node){
        if (node == nullptr) return;
        
        safeInorder(node->left);
        // Store name instead of pointer
        inorderNodeNames.push_back(node->name);
        safeInorder(node->right);
    }

    BStarNode* findNodeByName(const std::string& name) {
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
    
    /**
     * Cleans up the B*-tree
     * 
     * @param node Root node to start cleanup
     */
    void cleanupBStarTree(BStarNode* node) {
        if (node == nullptr) return;
        
        cleanupBStarTree(node->left);
        cleanupBStarTree(node->right);
        delete node;
    }
    
    /**
     * Clears the contour data structure
     */
    void clearContour();
    
    /**
     * Updates the contour after placing a module
     * 
     * @param x X-coordinate of the module
     * @param y Y-coordinate of the module
     * @param width Width of the module
     * @param height Height of the module
     */
    void updateContour(int x, int y, int width, int height);
    
    /**
     * Gets the height of the contour at a given x-coordinate
     * 
     * @param x X-coordinate
     * @return The height of the contour at x
     */
    int getContourHeight(int x) {
        // Bounds checking
        if (x < 0 || x >= maxContourWidth) {
            logGlobalPlacement("WARNING: Contour query out of bounds: " + std::to_string(x));
            return 0;
        }
        
        // Query segment tree for height at x
        return contourSegTree.query(x, x);
    }
    
    /**
     * Builds an initial B*-tree for global placement
     */
    void buildInitialBStarTree();
    
    /**
     * Performs a preorder traversal of the B*-tree
     * 
     * @param node The current node
     */
    void preorder(BStarNode* node) {
        if (node == nullptr) return;
        
        preorderTraversal.push_back(node);
        preorder(node->left);
        preorder(node->right);
    }
    
    /**
     * Performs an inorder traversal of the B*-tree
     * 
     * @param node The current node
     */
    void inorder(BStarNode* node) {
        if (node == nullptr) return;
        
        inorder(node->left);
        inorderTraversal.push_back(node);
        inorder(node->right);
    }
    
    
    /**
     * Packs the B*-tree to get the coordinates of all modules and islands
     */
    void packBStarTree();
    
    /**
     * Calculates the bounding box area of the current placement
     * 
     * @return Area of the bounding box
     */
    int calculateArea()  {
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
    
    /**
     * Calculates the cost of the current solution
     * 
     * @return Cost value
     */
    double calculateCost() {
        int area = calculateArea();
        double wirelength = 0;
        
        return areaWeight * area + wirelengthWeight * wirelength;
    }
    
    /**
     * Performs a random perturbation on the solution
     * 
     * @return True if perturbation was successful
     */
    bool perturb();

    /**
     * Moves a node in the B*-tree
     * 
     * @param node The node to move
     * @return True if perturbation was successful
     */
    bool moveNode(BStarNode* node);    

    /**
     * Rotates a module
     * 
     * @param isSymmetryIsland True if perturbing a symmetry island, false for regular module
     * @param name Name of the module or symmetry island
     * @return True if perturbation was successful
     */
    bool rotateModule(bool isSymmetryIsland, const std::string& name){
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
    
    /**
     * Swaps two nodes in the B*-tree
     * 
     * @return True if perturbation was successful
     */
    bool swapNodes() {
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
    
    /**
     * Changes representative for a symmetry pair
     * 
     * @return True if perturbation was successful
     */
    bool changeRepresentative() {
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
    
    /**
     * Converts symmetry type for a symmetry group
     * 
     * @return True if perturbation was successful
     */
    bool convertSymmetryType() {
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

    bool validateBStarTree();

    /**
     * Checks if any modules overlap in the current placement
     * 
     * @return True if there are overlaps
     */
    bool hasOverlaps();

    bool fixOverlaps();

    bool resolveOverlapsInSortedOrder(const std::vector<ModuleInfo> &sortedModules);

    bool gridBasedPlacement();

    /**
     * Performs deep copy of module data
     *
     * @param source Source map of modules
     * @return New map with copied modules
     */
    std::map<std::string, std::shared_ptr<Module>> copyModules(
        const std::map<std::string, std::shared_ptr<Module>> &source) {
    
        std::map<std::string, std::shared_ptr<Module>> result;
        for (const auto& pair : source) {
            result[pair.first] = std::make_shared<Module>(*pair.second);
        }
        
        return result;
    }

    void backupBStarTree();

    void restoreBStarTree();
    /**
     * Finds a random node in the B*-tree
     * 
     * @return Pointer to a random node
     */
    BStarNode* findRandomNode() {
        if (preorderTraversal.empty()) {
            return nullptr;
        }
        
        return preorderTraversal[std::rand() % preorderTraversal.size()];
    }
    
    /**
     * Copies the current solution to the best solution
     */
    void updateBestSolution();
    
    /**
     * Copies the best solution to the current solution
     */
    void restoreBestSolution();
    
public:

    /**
     * Constructor
     */
    PlacementSolver();
    
    /**
     * Destructor
     */
    ~PlacementSolver();
    
    /**
     * Loads the problem data
     * 
     * @param modules Map of all modules
     * @param symmetryGroups Vector of symmetry groups
     * @return True if loading was successful
     */
    bool loadProblem(const std::map<std::string, std::shared_ptr<Module>>& modules, 
                     const std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups);
    
    /**
     * Sets simulated annealing parameters
     * 
     * @param initialTemp Initial temperature
     * @param finalTemp Final temperature
     * @param cooling Cooling rate (0-1)
     * @param iterations Iterations per temperature
     * @param noImprovementLimit Number of consecutive non-improving iterations before early termination
     */
    void setAnnealingParameters(double initialTemp, double finalTemp, double cooling, 
                                int iterations, int noImprovementLimit) {
    
        initialTemperature = initialTemp;
        finalTemperature = finalTemp;
        coolingRate = cooling;
        iterationsPerTemperature = iterations;
        this->noImprovementLimit = noImprovementLimit;
    }

    
    /**
     * Sets perturbation probabilities
     * 
     * @param rotate Probability of rotation
     * @param move Probability of move
     * @param swap Probability of swap
     * @param changeRep Probability of changing representative
     * @param convertSym Probability of converting symmetry type
     */
    void setPerturbationProbabilities(double rotate, double move, double swap, 
                                     double changeRep, double convertSym) {
    
        rotateProb = rotate;
        moveProb = move;
        swapProb = swap;
        changeRepProb = changeRep;
        convertSymProb = convertSym;
    }
    
    /**
     * Sets cost function weights
     * 
     * @param areaWeight Weight for area cost
     * @param wirelengthWeight Weight for wirelength cost
     */
    void setCostWeights(double areaWeight, double wirelengthWeight) {
        this->areaWeight = areaWeight;
        this->wirelengthWeight = wirelengthWeight;
    }
    
    /**
     * Sets the random seed
     * 
     * @param seed Random seed
     */
    void setRandomSeed(unsigned int seed) {
        rng.seed(seed);
        std::srand(seed);
    }
    
    /**
     * Sets the time limit in seconds
     * 
     * @param seconds Time limit in seconds
     */
    void setTimeLimit(int seconds) {
        timeLimit = seconds;
    }
    
    /**
     * Solves the placement problem
     * 
     * @return True if solution was found
     */
    bool solve();

    void compactGlobalPlacement();

    /**
     * Gets the solution area
     * 
     * @return Area of the solution
     */
    int getSolutionArea() const {
        return solutionArea;
    }
    
    /**
     * Gets the solution modules
     * 
     * @return Map of modules with their final positions
     */
    const std::map<std::string, std::shared_ptr<Module>>& getSolutionModules() const {
        return bestSolutionModules;
    }
};