#include "slicing_sa.hpp"
#include "../Logger.hpp"
#include <algorithm>
#include <random>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <limits>
#include <stack>
#include <sstream>
#include <chrono>
#include <unordered_map>
using namespace std;

/**
 * Initialize global placement debug logSlicingPlacementger
 */
void SimulatedAnnealing::initSlicingDebugger() {
    // Close any existing log file first to prevent resource leaks
    if (slicingLogFile.is_open()) {
        slicingLogFile.close();
    }
    
    // Open the log file with a proper path - change extension to .log
    slicingLogFile.open("slicingPlacement_debug.log");
    
    if (slicingLogFile.is_open()) {
        slicingDebugEnabled = true;
        
        // Write header to file with safer string handling
        slicingLogFile << "Slicing Placement Debug Log" << std::endl;
        slicingLogFile << "==========================" << std::endl;
        
        // Get current time using safer methods
        auto now = std::chrono::system_clock::now();
        auto timeT = std::chrono::system_clock::to_time_t(now);
        
        // Write timestamp using a safer method
        std::stringstream ss;
        ss << "Time: " << std::put_time(std::localtime(&timeT), "%Y-%m-%d %H:%M:%S");
        slicingLogFile << ss.str() << std::endl;
        slicingLogFile << std::endl;
        
        // Flush immediately to ensure content is written
        slicingLogFile.flush();
    } else {
        slicingDebugEnabled = false;
        std::cerr << "Warning: Could not open slicing debug log file" << std::endl;
    }
}

/**
 * logSlicingPlacement slicing placement information safely
 */
void SimulatedAnnealing::logSlicingPlacement(const std::string& message) const {
    if (slicingDebugEnabled && slicingLogFile.is_open()) {
        try {
            // Use direct output instead of string concatenation
            slicingLogFile << message << std::endl;
            slicingLogFile.flush(); // Ensure content is written immediately
        } catch (const std::exception& e) {
            std::cerr << "Error writing to log file: " << e.what() << std::endl;
            slicingDebugEnabled = false; // Disable logging to prevent further errors
        } catch (...) {
            std::cerr << "Unknown error writing to log file" << std::endl;
            slicingDebugEnabled = false;
        }
    }
}

// Define a struct to track valid solutions for multi-start approach
struct ValidSolution {
    vector<int> expression;
    int area;
    
    ValidSolution(const vector<int>& expr, int area) : expression(expr), area(area) {}
    
    // Compare solutions by area (for sorting)
    bool operator<(const ValidSolution& other) const {
        return area < other.area;
    }
};

SimulatedAnnealing::SimulatedAnnealing(FloorplanData* data)
    : data(data), bestSolution(new FloorplanSolution(data)),
      slicingDebugEnabled(false) {  // Initialize to false first
    
    // Initialize block nodes and cut nodes with shared_ptr
    for (int i = 0; i < data->getNumBlocks(); ++i) {
        Block* block = data->getBlock(i);
        blockNodes.push_back(std::make_shared<SlicingTreeNode>(SlicingTreeNode::BLOCK, block));
    }
    
    // (n-1) cut nodes for n blocks
    for (int i = 0; i < data->getNumBlocks() - 1; ++i) {
        cutNodes.push_back(std::make_shared<SlicingTreeNode>());
    }
    
    // Seed random number generator
    srand(static_cast<unsigned int>(time(nullptr)));
    
    // Initialize logger after everything else is set up
    initSlicingDebugger();
    
    // Now log initialization info
    if (slicingDebugEnabled) {
        std::stringstream ss;
        ss << "Simulated Annealing initialized with " << data->getNumBlocks() << " blocks";
        logSlicingPlacement(ss.str());
    }
}

SimulatedAnnealing::~SimulatedAnnealing() {
    // Close log file
    if (slicingLogFile.is_open()) {
        slicingLogFile.close();
    }
    
    // Delete bestSolution which is owned exclusively by this class
    delete bestSolution;

}

void SimulatedAnnealing::run() {
    auto startTime = std::chrono::high_resolution_clock::now();
    double globalTimeLimit = 250.0; // 4 minutes total time limit
    
    logSlicingPlacement("Starting simulated annealing algorithm for analog placement...");
    
    // Generate initial expression
    vector<int> expression = generateInitialExpression();
    
    // Calculate initial area
    int area = calculateArea(expression);
    logSlicingPlacement("Initial solution area: " + to_string(area));
    
    // Store multiple valid solutions for multi-start approach
    vector<ValidSolution> validSolutions;
    const int maxValidSolutions = 5; // Max number of valid solutions to store
    
    // PHASE 1: Generate initial valid placements
    int attempt = 1;
    const int maxAttempts = 10;
    
    while (validSolutions.size() < maxValidSolutions && attempt <= maxAttempts) {
        // Check time remaining
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = currentTime - startTime;
        double timeRemaining = globalTimeLimit - elapsed.count();
        
        // Ensure we leave enough time for area optimization
        if (timeRemaining < 60.0) {
            logSlicingPlacement("Time limit approaching. Moving to area optimization.");
            break;
        }
        
        logSlicingPlacement("Initial valid placement attempt #" + to_string(attempt));
        
        // First phase: Find a valid placement (no overlap)
        // Run SA with focus on validity, not area optimization
        auto result = runSimulatedAnnealing(expression, false, 500.0, 
                                          0.1, 0.95, 10, 0.95, min(timeRemaining * 0.3, 60.0));
        
        expression = result.first;
        int newArea = calculateArea(expression);
        
        logSlicingPlacement("Found placement with area: " + to_string(newArea));
        
        // Add to our valid solutions
        validSolutions.emplace_back(expression, newArea);
        
        // Sort solutions by area (best first)
        std::sort(validSolutions.begin(), validSolutions.end());
        
        // Try a new initial expression for diversity
        expression = generateAlternativeExpression(attempt % 4);
        attempt++;
    }
    
    // PHASE 2: Area optimization using multi-start approach
    if (!validSolutions.empty()) {
        // Get remaining time
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = currentTime - startTime;
        double remainingTime = globalTimeLimit - elapsed.count();
        
        // Calculate time for each area optimization attempt
        double timePerAttempt = remainingTime / min(validSolutions.size(), (size_t)3);
        
        if (remainingTime > 30.0) {
            logSlicingPlacement("Optimizing area with multi-start approach (" + 
                               to_string(remainingTime) + " seconds remaining)");
            
            int bestArea = numeric_limits<int>::max();
            vector<int> bestAreaExpression;
            
            // Try the top 3 solutions (or all if we have fewer)
            for (size_t i = 0; i < min(validSolutions.size(), (size_t)3); i++) {
                logSlicingPlacement("Area optimization attempt #" + to_string(i+1) + 
                                  " starting from solution with area=" + 
                                  to_string(validSolutions[i].area));
                
                // Enhanced area optimization with more aggressive parameters
                vector<int> result = runAreaOptimization(validSolutions[i].expression, 
                                                       2000.0, // Higher temperature 
                                                       0.97,   // Slower cooling
                                                       timePerAttempt);
                
                // Check if this is better than our current best
                int newArea = calculateArea(result);
                logSlicingPlacement("Area optimization result: " + to_string(newArea));
                
                if (newArea < bestArea) {
                    bestArea = newArea;
                    bestAreaExpression = result;
                }
            }
            
            // Use the best area solution
            expression = bestAreaExpression;
        } else {
            logSlicingPlacement("Not enough time for area optimization, using best valid solution.");
            expression = validSolutions[0].expression;
        }
    }
    
    // Set the best solution
    bestSolution->setPolishExpression(expression);
    bestSolution->setCost(0);
    bestSolution->applyFloorplanToBlocks();
    
    // Final validation and reporting
    int finalArea = calculateArea(expression);
    logSlicingPlacement("Final solution area: " + to_string(finalArea));
    
    // Report total runtime
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> totalElapsed = endTime - startTime;
    logSlicingPlacement("Total algorithm runtime: " + to_string(totalElapsed.count()) + " seconds");
}



// Calculate weighted area of a solution
int SimulatedAnnealing::calculateArea(const std::vector<int>& expression) {
    logSlicingPlacement("In SimulatedAnnealing::calculateArea");
    auto root = buildSlicingTree(expression);
    
    if (!root || root->shapeRecords.empty()) {
        logSlicingPlacement("Error: Failed to build valid tree for area calculation");
        return std::numeric_limits<int>::max();
    }
    
    // Select an appropriate shape record - for analog placement with no fixed outline,
    // we should choose the record with minimum area
    int minArea = std::numeric_limits<int>::max();
    int bestRecordIndex = 0;
    
    for (size_t i = 0; i < root->shapeRecords.size(); ++i) {
        const ShapeRecord& record = root->shapeRecords[i];
        int area = record.width * record.height;
        
        if (area < minArea) {
            minArea = area;
            bestRecordIndex = i;
        }
    }
    
    // Set block positions using the best shape
    setBlockPositions(root.get(), 0, 0, bestRecordIndex);
    
    // Calculate total area by finding the bounding box of all blocks
    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    int maxX = 0;
    int maxY = 0;
    
    for (int i = 0; i < data->getNumBlocks(); i++) {
        Block* block = data->getBlock(i);
        int blockWidth = block->isRotated() ? block->getHeight() : block->getWidth();
        int blockHeight = block->isRotated() ? block->getWidth() : block->getHeight();
        logSlicingPlacement("block index" + to_string(i) + " block x: " + to_string(block->getX()) + " block y: " + to_string(block->getY()) + 
            " block width: " + to_string(blockWidth) + " block height: " + to_string(blockHeight));
        
        minX = std::min(minX, block->getX());
        minY = std::min(minY, block->getY());
        maxX = std::max(maxX, block->getX() + blockWidth);
        maxY = std::max(maxY, block->getY() + blockHeight);
    }
    
    // Calculate bounding box area
    int totalArea = (maxX - minX) * (maxY - minY);
    
    // No need to manually delete root anymore, shared_ptr handles it automatically
    
    return totalArea;
}


vector<int> SimulatedAnnealing::runAreaOptimization(
    const vector<int>& initialExpression, 
    double initialTemperature,
    double coolingRate,
    double maxRuntime) 
{
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Make a copy of the initial expression
    vector<int> expression = initialExpression;
    vector<int> bestExpression = expression;
    
    // Get initial area
    int area = calculateArea(expression);
    int bestArea = area;
    
    logSlicingPlacement("Starting area optimization with initial area=" + to_string(area));
    
    double temperature = initialTemperature;
    const double minTemperature = 0.5; // Higher min temperature for area phase
    
    // Use higher number of moves for area optimization
    const int movesPerTemperature = 15;
    int maxTryingCount = movesPerTemperature * data->getNumBlocks();
    
    // For early termination if not making progress
    int noImprovementCount = 0;
    const int maxNoImprovementCount = 10;
    int lastBestArea = bestArea;
    
    // Main optimization loop
    while (temperature >= minTemperature) {
        int tryingCount = 0;
        int uphillCount = 0;
        int acceptedCount = 0;
        
        // Check runtime
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = currentTime - startTime;
        if (elapsed.count() > maxRuntime) {
            logSlicingPlacement("Runtime limit reached for area optimization.");
            break;
        }
        
        // Inner loop
        do {
            // Choose move type - for area, M1 (operand swap) is most effective
            // Use 70% chance of M1 moves
            int moveType = (rand() % 100 < 70) ? 0 : (rand() % 3);
            vector<int> newExpression = perturbExpression(expression, moveType);
            
            // Skip if the move failed
            if (newExpression == expression) {
                ++tryingCount;
                continue;
            }
            
            ++tryingCount;
            
            // Calculate new area
            int newArea = calculateArea(newExpression);
            int deltaArea = newArea - area;
            
            // Accept or reject the move
            if (deltaArea < 0 || 
                static_cast<double>(rand()) / RAND_MAX < exp(-deltaArea / temperature)) {
                
                if (deltaArea > 0) {
                    ++uphillCount;
                }
                
                ++acceptedCount;
                expression = newExpression;
                area = newArea;
                
                // Update best solution if improved
                if (area < bestArea) {
                    bestArea = area;
                    bestExpression = expression;
                    
                    // Log significant improvements
                    if (area < lastBestArea * 0.95) {
                        logSlicingPlacement("Improved area to: " + to_string(bestArea));
                        lastBestArea = bestArea;
                    }
                }
            }
            
            // Check time limit periodically
            if (tryingCount % 100 == 0) {
                currentTime = std::chrono::high_resolution_clock::now();
                elapsed = currentTime - startTime;
                if (elapsed.count() > maxRuntime) {
                    break;
                }
            }
        } while (uphillCount <= maxTryingCount && tryingCount <= 2 * maxTryingCount);
        
        // Check for improvement at this temperature
        if (bestArea < lastBestArea) {
            noImprovementCount = 0;
            lastBestArea = bestArea;
        } else {
            ++noImprovementCount;
        }
        
        // Apply cooling
        temperature *= coolingRate;
        
        // If not making progress, try a reheating strategy
        if (noImprovementCount >= maxNoImprovementCount) {
            // Reheat with diminishing returns
            temperature = initialTemperature * (0.7 - 0.1 * noImprovementCount / maxNoImprovementCount);
            
            // Start from the best solution again
            expression = bestExpression;
            area = bestArea;
            noImprovementCount = 0;
        }
    }
    
    return bestExpression;
}


FloorplanSolution* SimulatedAnnealing::getBestSolution() const {
    return bestSolution;
}


vector<int> SimulatedAnnealing::generateInitialExpression() const {
    // Improved initial expression generation for analog placement
    vector<int> expression;
    
    // Sort blocks by area (descending)
    vector<int> sortedBlocks;
    for (int i = 0; i < data->getNumBlocks(); ++i) {
        sortedBlocks.push_back(i);
    }
    
    // Sort by area (largest blocks first)
    sort(sortedBlocks.begin(), sortedBlocks.end(), [this](int a, int b) {
        Block* blockA = data->getBlock(a);
        Block* blockB = data->getBlock(b);
        return (blockA->getWidth() * blockA->getHeight()) > 
               (blockB->getWidth() * blockB->getHeight());
    });
    
    // For analog placement, use a balanced binary tree structure
    // which tends to produce more compact placements
    expression = buildBalancedTree(sortedBlocks, 0, sortedBlocks.size() - 1, true);
    
    // Verify that the expression satisfies balloting property
    if (!validatePolishExpression(expression)) {
        logSlicingPlacement("ERROR: Initial expression violates balloting property!");
        
        // Fall back to a simple chain of vertical cuts
        expression.clear();
        for (int i = 0; i < data->getNumBlocks() - 1; ++i) {
            expression.push_back(sortedBlocks[i]);
            expression.push_back(SlicingTreeNode::VERTICAL_CUT);
        }
        expression.push_back(sortedBlocks.back());
    }
    
    return expression;
}



vector<int> SimulatedAnnealing::generateAlternativeExpression(int strategy) const {
    vector<int> expression;
    vector<int> blockIndices;
    
    for (int i = 0; i < data->getNumBlocks(); ++i) {
        blockIndices.push_back(i);
    }
    
    // Different sorting strategies
    if (strategy == 0) {
        // Sort by width
        sort(blockIndices.begin(), blockIndices.end(), [this](int a, int b) {
            Block* blockA = data->getBlock(a);
            Block* blockB = data->getBlock(b);
            return blockA->getWidth() > blockB->getWidth();
        });
        logSlicingPlacement("Generated alternative expression sorted by width (descending)");
    } 
    else if (strategy == 1) {
        // Sort by height
        sort(blockIndices.begin(), blockIndices.end(), [this](int a, int b) {
            Block* blockA = data->getBlock(a);
            Block* blockB = data->getBlock(b);
            return blockA->getHeight() > blockB->getHeight();
        });
        logSlicingPlacement("Generated alternative expression sorted by height (descending)");
    }
    else if (strategy == 2) {
        // Sort by perimeter
        sort(blockIndices.begin(), blockIndices.end(), [this](int a, int b) {
            Block* blockA = data->getBlock(a);
            Block* blockB = data->getBlock(b);
            int perimeterA = 2 * (blockA->getWidth() + blockA->getHeight());
            int perimeterB = 2 * (blockB->getWidth() + blockB->getHeight());
            return perimeterA > perimeterB;
        });
        logSlicingPlacement("Generated alternative expression sorted by perimeter (descending)");
    }
    else if (strategy == 3) {
        // Sort by aspect ratio
        sort(blockIndices.begin(), blockIndices.end(), [this](int a, int b) {
            Block* blockA = data->getBlock(a);
            Block* blockB = data->getBlock(b);
            double aspectRatioA = static_cast<double>(blockA->getWidth()) / blockA->getHeight();
            if (aspectRatioA < 1.0) aspectRatioA = 1.0 / aspectRatioA;
            double aspectRatioB = static_cast<double>(blockB->getWidth()) / blockB->getHeight();
            if (aspectRatioB < 1.0) aspectRatioB = 1.0 / aspectRatioB;
            return aspectRatioA < aspectRatioB; // Less extreme aspect ratios first
        });
        logSlicingPlacement("Generated alternative expression sorted by aspect ratio (closest to 1 first)");
    }
    
    // Two different construction patterns
    if (strategy % 2 == 0) {
        // Pattern 1: Simple alternating cuts
        for (int i = 0; i < blockIndices.size() - 1; ++i) {
            expression.push_back(blockIndices[i]);
            expression.push_back((i % 2 == 0) ? 
                                SlicingTreeNode::VERTICAL_CUT : 
                                SlicingTreeNode::HORIZONTAL_CUT);
        }
        expression.push_back(blockIndices.back());
    } else {
        // Pattern 2: Hierarchy of cuts (creates more balanced tree)
        vector<int> result = buildBalancedTree(blockIndices, 0, blockIndices.size() - 1, true);
        expression = result;
    }
    
    // Validate the expression
    if (!validatePolishExpression(expression)) {
        logSlicingPlacement("ERROR: Alternative expression violates balloting property!");
        // Fall back to simple expression
        expression.clear();
        for (int i = 0; i < blockIndices.size() - 1; ++i) {
            expression.push_back(blockIndices[i]);
            expression.push_back(SlicingTreeNode::VERTICAL_CUT);
        }
        expression.push_back(blockIndices.back());
    }
    
    return expression;
}

vector<int> SimulatedAnnealing::buildBalancedTree(const vector<int>& blocks, int start, int end, bool vertical) const {
    vector<int> result;
    
    if (start == end) {
        // Just one block
        result.push_back(blocks[start]);
        return result;
    }
    
    if (start + 1 == end) {
        // Two blocks
        result.push_back(blocks[start]);
        result.push_back(blocks[end]);
        result.push_back(vertical ? SlicingTreeNode::VERTICAL_CUT : SlicingTreeNode::HORIZONTAL_CUT);
        return result;
    }
    
    // Recursively build left and right subtrees
    int mid = start + (end - start) / 2;
    vector<int> left = buildBalancedTree(blocks, start, mid, !vertical);
    vector<int> right = buildBalancedTree(blocks, mid + 1, end, !vertical);
    
    // Combine the results
    result.insert(result.end(), left.begin(), left.end());
    result.insert(result.end(), right.begin(), right.end());
    result.push_back(vertical ? SlicingTreeNode::VERTICAL_CUT : SlicingTreeNode::HORIZONTAL_CUT);
    
    return result;
}

bool SimulatedAnnealing::isCut(int id) const {
    return id < 0; // HORIZONTAL_CUT or VERTICAL_CUT
}

bool SimulatedAnnealing::isSkewed(const vector<int>& expression, size_t index) const {
    if (isCut(expression[index])) {
        if (index + 2 < expression.size() && expression[index] == expression[index + 2]) {
            return false; // Not skewed
        }
    } else if (index + 1 < expression.size() && isCut(expression[index + 1])) {
        if (index > 0 && expression[index - 1] == expression[index + 1]) {
            return false; // Not skewed
        }
    }
    return true; // Skewed
}

bool SimulatedAnnealing::satisfiesBallotProperty(const vector<int>& expression, size_t index) const {
    if (index + 1 < expression.size() && isCut(expression[index + 1])) {
        size_t operatorCount = 0;
        for (size_t i = 0; i <= index; ++i) {
            if (isCut(expression[i])) {
                ++operatorCount;
            }
        }
        
        // For balloting property: #operands > #operators
        // means: (index+1) - operatorCount > operatorCount
        if ((index + 1) - operatorCount <= operatorCount) {
            return false; // Ballot property violated
        }
    }
    return true; // Ballot property satisfied
}

bool SimulatedAnnealing::validatePolishExpression(const vector<int>& expression) const {
    if (expression.empty()) {
        return false;
    }
    
    int operandCount = 0;
    int operatorCount = 0;
    
    // Check balloting property for each prefix
    for (size_t i = 0; i < expression.size(); ++i) {
        if (!isCut(expression[i])) {
            // Is operand (block)
            operandCount++;
        } else {
            // Is operator (cut)
            operatorCount++;
        }
        
        // Balloting property: #operands > #operators at every prefix
        if (operandCount <= operatorCount) {
            return false;
        }
    }
    
    // Final check: n operands and n-1 operators for a valid Polish expression
    return (operandCount == operatorCount + 1);
}

vector<int> SimulatedAnnealing::perturbExpression(const vector<int>& expression, int moveType) const {
    // Create a copy for modification
    vector<int> newExpression = expression;
    
    // Try each move type in sequence until finding a valid one
    for (int attempts = 0; attempts < 10; attempts++) {
        // Make a copy to try this move
        vector<int> candidateExpression = newExpression;
        
        // Try the current move type
        int currentMoveType = (moveType + attempts) % 3; // M1, M2, M3 moves
        
        if (currentMoveType == 0) { // M1: Operand swap
            // Find all operand indices
            vector<size_t> operandIndices;
            for (size_t i = 0; i < candidateExpression.size(); ++i) {
                if (!isCut(candidateExpression[i])) {
                    operandIndices.push_back(i);
                }
            }
            
            if (operandIndices.size() < 2) continue;
            
            // Enhanced selection to prioritize more impactful swaps
            size_t idx1 = operandIndices[std::rand() % operandIndices.size()];
            size_t idx2;
            
            // Try to find operands that are further apart for more diversity
            if (operandIndices.size() > 5) {
                // Pick an operand with large positional difference
                int maxDistance = 0;
                for (int i = 0; i < 5; i++) {
                    size_t candidate = operandIndices[std::rand() % operandIndices.size()];
                    int distance = abs((int)candidate - (int)idx1);
                    if (distance > maxDistance && candidate != idx1) {
                        maxDistance = distance;
                        idx2 = candidate;
                    }
                }
                if (maxDistance == 0) {
                    // If no good candidate found, just pick a different one
                    do {
                        idx2 = operandIndices[std::rand() % operandIndices.size()];
                    } while (idx1 == idx2);
                }
            } else {
                do {
                    idx2 = operandIndices[std::rand() % operandIndices.size()];
                } while (idx1 == idx2);
            }
            
            // Swap the operands
            std::swap(candidateExpression[idx1], candidateExpression[idx2]);
        }
        else if (currentMoveType == 1) { // M2: Chain invert
            // Find all chains (consecutive operators of different types)
            vector<size_t> chainStarts;
            
            for (size_t i = 0; i < candidateExpression.size() - 1; ++i) {
                if (isCut(candidateExpression[i]) && 
                    i + 1 < candidateExpression.size() && 
                    isCut(candidateExpression[i+1]) && 
                    candidateExpression[i] != candidateExpression[i+1]) {
                    
                    chainStarts.push_back(i);
                    // Skip to end of chain
                    while (i + 1 < candidateExpression.size() && 
                           isCut(candidateExpression[i+1]) && 
                           (i+1 == candidateExpression.size()-1 || 
                            candidateExpression[i+1] != candidateExpression[i+2])) {
                        i++;
                    }
                }
            }
            
            // If no chains found, invert a single cut
            if (chainStarts.empty()) {
                vector<size_t> cutIndices;
                for (size_t i = 0; i < candidateExpression.size(); ++i) {
                    if (isCut(candidateExpression[i])) {
                        cutIndices.push_back(i);
                    }
                }
                
                if (cutIndices.empty()) continue;
                
                // Pick a random cut to invert
                size_t cutIdx = cutIndices[std::rand() % cutIndices.size()];
                
                // Invert this cut (H -> V or V -> H)
                candidateExpression[cutIdx] = (candidateExpression[cutIdx] == SlicingTreeNode::HORIZONTAL_CUT) ? 
                                             SlicingTreeNode::VERTICAL_CUT : SlicingTreeNode::HORIZONTAL_CUT;
            } else {
                // Pick a random chain to invert
                size_t chainIdx = chainStarts[std::rand() % chainStarts.size()];
                
                // Complement the chain (H->V, V->H)
                size_t i = chainIdx;
                while (i < candidateExpression.size() && isCut(candidateExpression[i])) {
                    candidateExpression[i] = (candidateExpression[i] == SlicingTreeNode::HORIZONTAL_CUT) ? 
                                            SlicingTreeNode::VERTICAL_CUT : SlicingTreeNode::HORIZONTAL_CUT;
                    i++;
                }
            }
        }
        else if (currentMoveType == 2) { // M3: Operator/Operand swap
            // Find adjacent operator-operand pairs that could be swapped
            vector<size_t> swapCandidates;
            for (size_t i = 0; i + 1 < candidateExpression.size(); ++i) {
                // Need an operand and an operator adjacent to each other
                if ((isCut(candidateExpression[i]) && !isCut(candidateExpression[i+1])) ||
                    (!isCut(candidateExpression[i]) && isCut(candidateExpression[i+1]))) {
                    
                    // Try making the swap and check if it maintains validity
                    vector<int> testExpr = candidateExpression;
                    std::swap(testExpr[i], testExpr[i+1]);
                    
                    if (validatePolishExpression(testExpr)) {
                        swapCandidates.push_back(i);
                    }
                }
            }
            
            if (swapCandidates.empty()) continue;
            
            // Pick a random valid swap
            size_t swapIdx = swapCandidates[std::rand() % swapCandidates.size()];
            std::swap(candidateExpression[swapIdx], candidateExpression[swapIdx+1]);
        }
        
        // Validate the resulting expression
        if (validatePolishExpression(candidateExpression)) {
            return candidateExpression;
        }
    }
    
    // If all attempts failed, return the original expression
    return expression;
}


std::shared_ptr<SlicingTreeNode> SimulatedAnnealing::buildSlicingTree(const std::vector<int>& expression) {
    size_t cutIndex = 0;
    std::stack<std::shared_ptr<SlicingTreeNode>> nodeStack;
    
    try {
        // Validate expression
        if (expression.empty()) {
            logSlicingPlacement("Error: Empty expression");
            return nullptr;
        }
        
        // Basic validation of Polish expression
        int operands = 0;
        int operators = 0;
        for (int id : expression) {
            if (!isCut(id)) {
                operands++;
            } else {
                operators++;
            }
            
            // Check balloting property
            if (operands <= operators) {
                logSlicingPlacement("Error: Expression violates balloting property");
                return nullptr;
            }
        }
        
        // Final check: n operands and n-1 operators
        if (operands != operators + 1) {
            logSlicingPlacement("Error: Expression has invalid operand/operator count");
            logSlicingPlacement("Operands: " + std::to_string(operands) + ", Operators: " + std::to_string(operators));
            return nullptr;
        }
        
        // Process expression
        for (int id : expression) {
            if (!isCut(id)) {
                // Ensure valid block index
                if (id < 0 || id >= static_cast<int>(blockNodes.size())) {
                    logSlicingPlacement("Error: Invalid block index: " + std::to_string(id));
                    return nullptr;
                }
                
                // Store a copy of the shared_ptr in the stack
                nodeStack.push(blockNodes[id]);
            } else {
                // Need at least 2 nodes for a cut
                if (nodeStack.size() < 2) {
                    logSlicingPlacement("Error: Not enough nodes for cut operation");
                    return nullptr;
                }
                
                // Ensure valid cut index
                if (cutIndex >= cutNodes.size()) {
                    logSlicingPlacement("Error: Cut index out of bounds: " + std::to_string(cutIndex));
                    return nullptr;
                }
                
                // Get the cut node and keep a copy of the shared_ptr
                std::shared_ptr<SlicingTreeNode> cutNode = cutNodes[cutIndex++];
                
                // Safe handling of the stack
                auto rightChild = nodeStack.top();
                nodeStack.pop();
                auto leftChild = nodeStack.top();
                nodeStack.pop();
                
                // Set the cut node properties
                cutNode->type = id;
                cutNode->rightChild = rightChild.get(); // Still using raw pointer for child links
                cutNode->leftChild = leftChild.get();   // Still using raw pointer for child links
                
                // Keep the shared_ptrs in memory by storing them in the cut node
                // Adding this custom field to track ownership
                if (!cutNode->userData) {
                    cutNode->userData = new std::vector<std::shared_ptr<SlicingTreeNode>>();
                }
                
                // Store the shared pointers to maintain ownership
                auto childPtrs = static_cast<std::vector<std::shared_ptr<SlicingTreeNode>>*>(cutNode->userData);
                childPtrs->push_back(leftChild);
                childPtrs->push_back(rightChild);
                
                // Update shape records and push to stack
                cutNode->updateShapeRecords();
                nodeStack.push(cutNode);
            }
        }
        
        // Should have exactly one node left
        if (nodeStack.size() != 1) {
            logSlicingPlacement("Error: Invalid expression, final stack size: " + std::to_string(nodeStack.size()));
            return nullptr;
        }
        
        return nodeStack.top();
    }
    catch (const std::bad_alloc& e) {
        logSlicingPlacement("Memory allocation failed in buildSlicingTree: " + std::string(e.what()));
        // Clear stack to help with cleanup
        while (!nodeStack.empty()) {
            nodeStack.pop();
        }
        return nullptr;
    }
    catch (const std::exception& e) {
        logSlicingPlacement("Exception in buildSlicingTree: " + std::string(e.what()));
        // Clear stack to help with cleanup
        while (!nodeStack.empty()) {
            nodeStack.pop();
        }
        return nullptr;
    }
    catch (...) {
        logSlicingPlacement("Unknown exception in buildSlicingTree");
        // Clear stack to help with cleanup
        while (!nodeStack.empty()) {
            nodeStack.pop();
        }
        return nullptr;
    }
}

void SimulatedAnnealing::setBlockPositions(SlicingTreeNode* node, int x, int y, int recordIndex) {
    if (!node) return;
    
    const ShapeRecord& record = node->shapeRecords[recordIndex];
    
    if (node->type == SlicingTreeNode::BLOCK) {
        // Update block position and rotation
        node->block->updatePosition(x, y, record.width, record.height);
    } else {
        // Process children
        setBlockPositions(node->leftChild, x, y, record.leftChoice);
        
        if (node->type == SlicingTreeNode::HORIZONTAL_CUT) {
            // For horizontal cut, the right child is below the left child
            y += node->leftChild->shapeRecords[record.leftChoice].height;
        } else {  // VERTICAL_CUT
            // For vertical cut, the right child is to the right of the left child
            x += node->leftChild->shapeRecords[record.leftChoice].width;
        }
        
        setBlockPositions(node->rightChild, x, y, record.rightChoice);
    }
}

int SimulatedAnnealing::calculateCost(const std::vector<int>& expression, bool includeArea) {
    try {
        auto root = buildSlicingTree(expression);
        
        if (!root || root->shapeRecords.empty()) {
            logSlicingPlacement("Error: Failed to build valid slicing tree or empty shape records");
            return std::numeric_limits<int>::max();
        }
        
        // Find minimum area shape record
        int minArea = std::numeric_limits<int>::max();
        int bestRecordIndex = 0;
        
        for (size_t i = 0; i < root->shapeRecords.size(); ++i) {
            ShapeRecord& record = root->shapeRecords[i];
            int area = record.width * record.height;
            
            if (area < minArea) {
                minArea = area;
                bestRecordIndex = i;
            }
        }
        
        // For area optimization - use actual calculated area
        if (includeArea) {
            // Set block positions using best record
            setBlockPositions(root.get(), 0, 0, bestRecordIndex);
            
            // Get bounding box of all blocks
            int minX = std::numeric_limits<int>::max();
            int minY = std::numeric_limits<int>::max();
            int maxX = 0;
            int maxY = 0;
            
            for (int i = 0; i < data->getNumBlocks(); i++) {
                Block* block = data->getBlock(i);
                int blockWidth = block->isRotated() ? block->getHeight() : block->getWidth();
                int blockHeight = block->isRotated() ? block->getWidth() : block->getHeight();
                
                minX = std::min(minX, block->getX());
                minY = std::min(minY, block->getY());
                maxX = std::max(maxX, block->getX() + blockWidth);
                maxY = std::max(maxY, block->getY() + blockHeight);
            }
            
            // Calculate total area
            minArea = (maxX - minX) * (maxY - minY);
        }
        
        // No need to manually delete root - shared_ptr handles cleanup
        
        return minArea;
    } catch (const std::exception& e) {
        logSlicingPlacement("Exception in calculateCost: " + std::string(e.what()));
        return std::numeric_limits<int>::max();
    } catch (...) {
        logSlicingPlacement("Unknown exception in calculateCost");
        return std::numeric_limits<int>::max();
    }
}

pair<vector<int>, int> SimulatedAnnealing::runSimulatedAnnealing(
    vector<int> expression, 
    bool includeArea,
    double initialTemperature,
    double minTemperature,
    double coolingRate,
    int movesPerTemperature,
    double maxRejectRatio,
    double maxRuntime) 
{
    auto startTime = std::chrono::high_resolution_clock::now();
    
    int cost = calculateCost(expression, includeArea);
    
    vector<int> bestExpression = expression;
    int bestCost = cost;
    
    double temperature = initialTemperature;
    int maxTryingCount = movesPerTemperature * data->getNumBlocks();
    
    // SA parameters
    const double logProbabilityThreshold = 0.01;
    int stagnationCount = 0;
    const int maxStagnationCount = 5;
    
    // Main SA loop
    while (temperature >= minTemperature && stagnationCount < maxStagnationCount) {
        int acceptedCount = 0;
        int tryingCount = 0;
        int uphillCount = 0;
        int rejectCount = 0;
        bool improved = false;
        
        // Check runtime
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = currentTime - startTime;
        if (elapsed.count() > maxRuntime) {
            logSlicingPlacement("Runtime limit reached. Terminating optimization.");
            break;
        }
        
        // Inner loop - try perturbations at this temperature
        do {
            // Choose move type based on optimization goal
            int moveType;
            if (includeArea) {
                // For area optimization, prefer M1 (operand swap) and M2 (chain invert)
                moveType = (rand() % 100 < 70) ? rand() % 2 : 2;
            } else {
                // For solution validity, use balanced approach
                moveType = rand() % 3;
            }
            
            vector<int> newExpression = perturbExpression(expression, moveType);
            
            // If perturbation failed, try again
            if (newExpression == expression) {
                ++rejectCount;
                ++tryingCount;
                continue;
            }
            
            ++tryingCount;
            
            int newCost = calculateCost(newExpression, includeArea);
            int deltaCost = newCost - cost;
            
            // Accept or reject the move
            if (deltaCost <= 0 || static_cast<double>(rand()) / RAND_MAX < exp(-deltaCost / temperature)) {
                if (deltaCost > 0) {
                    ++uphillCount;
                }
                
                ++acceptedCount;
                expression = newExpression;
                cost = newCost;
                
                if (cost < bestCost) {
                    bestExpression = expression;
                    bestCost = cost;
                    improved = true;
                }
            } else {
                ++rejectCount;
            }
        } while (uphillCount <= maxTryingCount && tryingCount <= 2 * maxTryingCount);
        
        // Update stagnation count
        if (improved) {
            stagnationCount = 0;
        } else {
            stagnationCount++;
            
            // Reheat the temperature if stagnating
            if (stagnationCount > 0 && stagnationCount < maxStagnationCount) {
                temperature = initialTemperature * (0.8 - 0.1 * stagnationCount / maxStagnationCount);
                continue;
            }
        }
        
        // Apply cooling
        temperature *= coolingRate;
        
        // Termination conditions
        double acceptanceRatio = (tryingCount > 0) ? static_cast<double>(acceptedCount) / tryingCount : 0;
        if (acceptanceRatio < logProbabilityThreshold || acceptedCount == 0) {
            break;
        }
        
        if (static_cast<double>(rejectCount) / tryingCount > maxRejectRatio) {
            break;
        }
    }
    
    return {bestExpression, bestCost};
}


bool SimulatedAnnealing::repairFloorplan() {
    logSlicingPlacement("Attempting to repair floorplan (remove overlaps)...");
    
    bool overlapsExist = true;
    int iterations = 0;
    const int maxIterations = 200;
    
    // Loop until no overlaps or max iterations reached
    while (overlapsExist && iterations < maxIterations) {
        overlapsExist = false;
        
        // Check all pairs of blocks for overlaps
        for (int i = 0; i < data->getNumBlocks(); ++i) {
            Block* block1 = data->getBlock(i);
            int block1Width = block1->isRotated() ? block1->getHeight() : block1->getWidth();
            int block1Height = block1->isRotated() ? block1->getWidth() : block1->getHeight();
            
            for (int j = i + 1; j < data->getNumBlocks(); ++j) {
                Block* block2 = data->getBlock(j);
                int block2Width = block2->isRotated() ? block2->getHeight() : block2->getWidth();
                int block2Height = block2->isRotated() ? block2->getWidth() : block2->getHeight();
                
                // Check for overlap
                if (!(block1->getX() + block1Width <= block2->getX() || 
                      block2->getX() + block2Width <= block1->getX() ||
                      block1->getY() + block1Height <= block2->getY() ||
                      block2->getY() + block2Height <= block1->getY())) {
                    
                    overlapsExist = true;
                    
                    // Calculate overlap amount in both directions
                    int overlapX = std::min(block1->getX() + block1Width, block2->getX() + block2Width) - 
                                  std::max(block1->getX(), block2->getX());
                    
                    int overlapY = std::min(block1->getY() + block1Height, block2->getY() + block2Height) - 
                                  std::max(block1->getY(), block2->getY());
                    
                    // Introduce some randomness every 20 iterations to avoid getting stuck
                    if (iterations % 20 == 0) {
                        int randomOffset = rand() % 10 + 1;
                        if (rand() % 2 == 0) {
                            // Horizontal shift
                            if (block1->getX() < block2->getX()) {
                                block1->setX(block1->getX() - randomOffset);
                                block2->setX(block2->getX() + randomOffset);
                            } else {
                                block1->setX(block1->getX() + randomOffset);
                                block2->setX(block2->getX() - randomOffset);
                            }
                        } else {
                            // Vertical shift
                            if (block1->getY() < block2->getY()) {
                                block1->setY(block1->getY() - randomOffset);
                                block2->setY(block2->getY() + randomOffset);
                            } else {
                                block1->setY(block1->getY() + randomOffset);
                                block2->setY(block2->getY() - randomOffset);
                            }
                        }
                    } 
                    // Normal deterministic resolution - move along minimum overlap direction
                    else if (overlapX <= overlapY) {
                        // Shift horizontally
                        int shift = (overlapX / 2) + 1;  // +1 to ensure separation
                        
                        if (block1->getX() < block2->getX()) {
                            block1->setX(block1->getX() - shift);
                            block2->setX(block2->getX() + shift);
                        } else {
                            block1->setX(block1->getX() + shift);
                            block2->setX(block2->getX() - shift);
                        }
                    } else {
                        // Shift vertically
                        int shift = (overlapY / 2) + 1;  // +1 to ensure separation
                        
                        if (block1->getY() < block2->getY()) {
                            block1->setY(block1->getY() - shift);
                            block2->setY(block2->getY() + shift);
                        } else {
                            block1->setY(block1->getY() + shift);
                            block2->setY(block2->getY() - shift);
                        }
                    }
                    
                    break; // Move to the next block after resolving this overlap
                }
            }
            
            if (overlapsExist) break; // Restart checking after modifying positions
        }
        
        iterations++;
    }
    
    // Final verification - check if any overlaps remain
    bool valid = true;
    for (int i = 0; i < data->getNumBlocks() && valid; ++i) {
        Block* block1 = data->getBlock(i);
        int block1Width = block1->isRotated() ? block1->getHeight() : block1->getWidth();
        int block1Height = block1->isRotated() ? block1->getWidth() : block1->getHeight();
        
        for (int j = i + 1; j < data->getNumBlocks() && valid; ++j) {
            Block* block2 = data->getBlock(j);
            int block2Width = block2->isRotated() ? block2->getHeight() : block2->getWidth();
            int block2Height = block2->isRotated() ? block2->getWidth() : block2->getHeight();
            
            if (!(block1->getX() + block1Width <= block2->getX() || 
                  block2->getX() + block2Width <= block1->getX() ||
                  block1->getY() + block1Height <= block2->getY() ||
                  block2->getY() + block2Height <= block1->getY())) {
                valid = false;
                break;
            }
        }
    }
    
    if (valid) {
        logSlicingPlacement("Successfully repaired floorplan - all overlaps resolved in " + 
            to_string(iterations) + " iterations");
        return true;
    } else {
        logSlicingPlacement("Could not completely resolve all overlaps within " + 
            to_string(maxIterations) + " iterations");
        return false;
    }
}

bool SimulatedAnnealing::hasOverlaps(const FloorplanSolution* solution) const {
    // Check for block overlaps
    for (int i = 0; i < data->getNumBlocks(); ++i) {
        Block* block1 = data->getBlock(i);
        int block1Width = block1->isRotated() ? block1->getHeight() : block1->getWidth();
        int block1Height = block1->isRotated() ? block1->getWidth() : block1->getHeight();
        
        for (int j = i + 1; j < data->getNumBlocks(); ++j) {
            Block* block2 = data->getBlock(j);
            int block2Width = block2->isRotated() ? block2->getHeight() : block2->getWidth();
            int block2Height = block2->isRotated() ? block2->getWidth() : block2->getHeight();
            
            // Check for overlap
            if (!(block1->getX() + block1Width <= block2->getX() || 
                  block2->getX() + block2Width <= block1->getX() ||
                  block1->getY() + block1Height <= block2->getY() ||
                  block2->getY() + block2Height <= block1->getY())) {
                
                // Log the overlap with stringstream instead of string concatenation
                std::stringstream ss;
                ss << "Overlap detected between blocks " << i << " and " << j;
                ss << " (" << block1->getName() << " at (" << block1->getX() << "," << block1->getY() 
                   << ") and " << block2->getName() << " at (" << block2->getX() << "," << block2->getY() << "))";
                
                // Here we need to make the method non-const or use a global logger
                // For now, we'll just print to stderr as this is a const method
                std::cerr << ss.str() << std::endl;
                
                return true;
            }
        }
    }
    
    return false;
}