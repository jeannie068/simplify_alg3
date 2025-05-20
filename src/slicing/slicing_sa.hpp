#pragma once

#include "slicing_struct.hpp"
#include <vector>
#include <utility>
#include <unordered_map>
#include <iostream>
#include <fstream>

class SimulatedAnnealing {
public:
    SimulatedAnnealing(FloorplanData* data);
    ~SimulatedAnnealing();
    
    // Run the simulated annealing algorithm
    void run();

    int calculateArea(const std::vector<int> &expression);

    // Get the best solution found
    FloorplanSolution* getBestSolution() const;
    
private:
    FloorplanData* data;
    FloorplanSolution* bestSolution;
    std::vector<std::shared_ptr<SlicingTreeNode>> blockNodes;
    std::vector<std::shared_ptr<SlicingTreeNode>> cutNodes;

    // Logger members
    mutable std::ofstream slicingLogFile;
    mutable bool slicingDebugEnabled;
    void initSlicingDebugger();
    void logSlicingPlacement(const std::string &message) const;

    // Generate the initial Polish expression
    std::vector<int> generateInitialExpression() const;
    
    // Generate alternative Polish expressions with different block ordering
    std::vector<int> generateAlternativeExpression(int strategy) const;
    
    // Recursive for balanced expression generation
    std::vector<int> buildBalancedTree(const std::vector<int>& blocks, int start, int end, bool vertical) const;
    
    // Helper functions
    bool isCut(int id) const;
    bool isSkewed(const std::vector<int>& expression, size_t index) const;
    bool satisfiesBallotProperty(const std::vector<int>& expression, size_t index) const;
    
    // Validate a Polish expression
    bool validatePolishExpression(const std::vector<int>& expression) const;
    
    // Generate a neighboring solution
    std::vector<int> perturbExpression(const std::vector<int>& expression, int moveType) const;
    
    // Slicing tree construction and evaluation
    std::shared_ptr<SlicingTreeNode> buildSlicingTree(const std::vector<int>& expression);
    void setBlockPositions(SlicingTreeNode* node, int x, int y, int recordIndex);
    
    // Calculate the cost of a solution
    int calculateCost(const std::vector<int>& expression, bool includeArea);
    
    // Direct repair of invalid floorplans
    bool repairFloorplan();

    bool hasOverlaps(const FloorplanSolution *solution) const;

    // The simulated annealing algorithm for both area and wirelength optimization
    std::pair<std::vector<int>, int> runSimulatedAnnealing(
        std::vector<int> expression, 
        bool includeArea,
        double initialTemperature,
        double minTemperature,
        double coolingRate,
        int movesPerTemperature,
        double maxRejectRatio,
        double maxRuntime = 180.0
    );
    
    // Method for wirelength optimization
    std::vector<int> runAreaOptimization(
        const std::vector<int>& initialExpression,
        double initialTemperature,
        double coolingRate,
        double maxRuntime
    );
    
};
