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
    struct ContourPoint {
        int x;
        int height;
        ContourPoint* next;
        
        ContourPoint(int x, int height) : x(x), height(height), next(nullptr) {}
    };
    
    ContourPoint* contourHead;
    
    // Preorder and inorder traversals for B*-tree
    std::vector<BStarNode*> preorderTraversal;
    std::vector<BStarNode*> inorderTraversal;
    // Use vectors of names instead of pointers for safer traversals
    std::vector<std::string> preorderNodeNames;
    std::vector<std::string> inorderNodeNames;
    
    // Safe traversal methods
    void safePreorder(BStarNode* node);
    void safeInorder(BStarNode* node);
    BStarNode* findNodeByName(const std::string& name);
    
    /**
     * Cleans up the B*-tree
     * 
     * @param node Root node to start cleanup
     */
    void cleanupBStarTree(BStarNode* node);
    
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
    int getContourHeight(int x);
    
    /**
     * Builds an initial B*-tree for global placement
     */
    void buildInitialBStarTree();
    
    /**
     * Performs a preorder traversal of the B*-tree
     * 
     * @param node The current node
     */
    void preorder(BStarNode* node);
    
    /**
     * Performs an inorder traversal of the B*-tree
     * 
     * @param node The current node
     */
    void inorder(BStarNode* node);
    
    
    /**
     * Packs the B*-tree to get the coordinates of all modules and islands
     */
    void packBStarTree();
    
    /**
     * Calculates the bounding box area of the current placement
     * 
     * @return Area of the bounding box
     */
    int calculateArea();
    
    /**
     * Calculates the half-perimeter wirelength (placeholder)
     * 
     * @return Half-perimeter wirelength
     */
    double calculateWirelength();
    
    /**
     * Calculates the cost of the current solution
     * 
     * @return Cost value
     */
    double calculateCost();
    
    /**
     * Performs a random perturbation on the solution
     * 
     * @return True if perturbation was successful
     */
    bool perturb();
    
    /**
     * Rotates a module
     * 
     * @param isSymmetryIsland True if perturbing a symmetry island, false for regular module
     * @param name Name of the module or symmetry island
     * @return True if perturbation was successful
     */
    bool rotateModule(bool isSymmetryIsland, const std::string& name);
    
    /**
     * Moves a node in the B*-tree
     * 
     * @param node The node to move
     * @return True if perturbation was successful
     */
    bool moveNode(BStarNode* node);
    
    /**
     * Swaps two nodes in the B*-tree
     * 
     * @return True if perturbation was successful
     */
    bool swapNodes();
    
    /**
     * Changes representative for a symmetry pair
     * 
     * @return True if perturbation was successful
     */
    bool changeRepresentative();
    
    /**
     * Converts symmetry type for a symmetry group
     * 
     * @return True if perturbation was successful
     */
    bool convertSymmetryType();

    bool validateBStarTree();

    /**
     * Checks if any modules overlap in the current placement
     * 
     * @return True if there are overlaps
     */
    bool hasOverlaps();
    
    /**
     * Performs deep copy of module data
     * 
     * @param source Source map of modules
     * @return New map with copied modules
     */
    std::map<std::string, std::shared_ptr<Module>> copyModules(const std::map<std::string, std::shared_ptr<Module>> &source);

    void backupBStarTree();

    void restoreBStarTree();
    /**
     * Finds a random node in the B*-tree
     * 
     * @return Pointer to a random node
     */
    BStarNode* findRandomNode();
    
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
                                int iterations, int noImprovementLimit);
    
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
                                     double changeRep, double convertSym);
    
    /**
     * Sets cost function weights
     * 
     * @param areaWeight Weight for area cost
     * @param wirelengthWeight Weight for wirelength cost
     */
    void setCostWeights(double areaWeight, double wirelengthWeight);
    
    /**
     * Sets the random seed
     * 
     * @param seed Random seed
     */
    void setRandomSeed(unsigned int seed);
    
    /**
     * Sets the time limit in seconds
     * 
     * @param seconds Time limit in seconds
     */
    void setTimeLimit(int seconds);
    
    /**
     * Solves the placement problem
     * 
     * @return True if solution was found
     */
    bool solve();
    
    /**
     * Gets the solution area
     * 
     * @return Area of the solution
     */
    int getSolutionArea() const;
    
    /**
     * Gets the solution modules
     * 
     * @return Map of modules with their final positions
     */
    const std::map<std::string, std::shared_ptr<Module>>& getSolutionModules() const;
};