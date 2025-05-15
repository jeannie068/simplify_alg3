// Logger.cpp
#include "Logger.hpp"
#include "data_struct/ASFBStarTree.hpp"
#include "solver/solver.hpp"

// Specialization for ASFBStarTree::BStarNode
template<>
std::string Logger::nodeToString(ASFBStarTree::BStarNode* node) {
    return "Module: " + node->moduleName;
}

// Specialization for PlacementSolver::BStarNode
template<>
std::string Logger::nodeToString(PlacementSolver::BStarNode* node) {
    return (node->isSymmetryIsland ? "Island: " : "Module: ") + node->name;
}

// Initialize static members
std::ofstream Logger::logFile;
bool Logger::initialized = false;
int Logger::indent = 0;