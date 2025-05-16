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
    
    // Initialize block nodes and cut nodes
    for (int i = 0; i < data->getNumBlocks(); ++i) {
        Block* block = data->getBlock(i);
        blockNodes.push_back(new SlicingTreeNode(SlicingTreeNode::BLOCK, block));
    }
    
    // (n-1) cut nodes for n blocks
    for (int i = 0; i < data->getNumBlocks() - 1; ++i) {
        cutNodes.push_back(new SlicingTreeNode());
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
    // Close log file if open
    if (slicingLogFile.is_open()) {
        try {
            slicingLogFile << "Closing log file" << std::endl;
            slicingLogFile.close();
        } catch (...) {
            // Silent catch to prevent destructor from throwing
        }
    }
    
    // Clean up other resources
    delete bestSolution;
    
    // Clean up nodes
    for (SlicingTreeNode* node : blockNodes) {
        delete node;
    }
    
    for (SlicingTreeNode* node : cutNodes) {
        delete node;
    }
}

void SimulatedAnnealing::run() {
    try {
        auto startTime = std::chrono::high_resolution_clock::now();
        double globalTimeLimit = 180.0; // 3 minutes total time limit
        
        logSlicingPlacement("Starting simulated annealing algorithm for analog placement...");
        
        // Generate initial expression
        logSlicingPlacement("Generating initial expression...");
        vector<int> expression = generateInitialExpression();
        
        if (expression.empty()) {
            logSlicingPlacement("ERROR: Initial expression generation failed. Using failsafe approach.");
            // Create a simple chain of vertical cuts for fallback
            for (int i = 0; i < data->getNumBlocks() - 1; ++i) {
                expression.push_back(i);
                expression.push_back(SlicingTreeNode::VERTICAL_CUT);
            }
            expression.push_back(data->getNumBlocks() - 1);
        }
        
        // Calculate initial area
        logSlicingPlacement("Calculating initial area...");
        int area = calculateArea(expression);
        
        stringstream ss;
        ss << "Initial solution area: " << area;
        logSlicingPlacement(ss.str());
        
        // Store multiple valid solutions for multi-start area optimization
        vector<ValidSolution> validSolutions;
        const int maxValidSolutions = 5; // Max number of valid solutions to store
        
        // PHASE 1: Generate initial valid placements
        int attempt = 1;
        
        while (validSolutions.size() < maxValidSolutions) {
            // Check time remaining
            auto currentTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = currentTime - startTime;
            double timeRemaining = globalTimeLimit - elapsed.count();
            
            // Ensure we leave enough time for area optimization
            if (timeRemaining < 180.0 || validSolutions.size() >= maxValidSolutions) {
                logSlicingPlacement("Time limit approaching or enough valid solutions found. Moving to area optimization.");
                break;
            }
            
            ss.str("");
            ss << "Initial valid placement attempt #" << attempt << " (remaining time: " << timeRemaining << "s)";
            logSlicingPlacement(ss.str());
            
            // First phase: Find a valid placement (no overlap)
            // Lower temperature for this phase to accept fewer uphill moves
            auto result = runSimulatedAnnealing(expression, false, 500.0, 
                                                0.1, 0.95, 10, 0.95, min(timeRemaining * 0.3, 60.0));
                                                
            expression = result.first;
            int newArea = calculateArea(expression);
            
            ss.str("");
            ss << "Found placement with area: " << newArea;
            logSlicingPlacement(ss.str());
            
            // Add to our valid solutions
            if (validSolutions.size() < maxValidSolutions) {
                validSolutions.emplace_back(expression, newArea);
                
                // Sort solutions by area (best first)
                std::sort(validSolutions.begin(), validSolutions.end());
            } else {
                // Replace the worst solution if this one is better
                auto worstSolution = std::max_element(validSolutions.begin(), validSolutions.end());
                if (newArea < worstSolution->area) {
                    *worstSolution = ValidSolution(expression, newArea);
                    // Re-sort solutions
                    std::sort(validSolutions.begin(), validSolutions.end());
                    
                    ss.str("");
                    ss << "Replaced a solution with better area: " << newArea;
                    logSlicingPlacement(ss.str());
                }
            }
            
            // Try a new initial expression for diversity
            expression = generateAlternativeExpression(attempt % 4);
            attempt++;
        }
        
        // PHASE 2: Area optimization using multi-start approach
        if (!validSolutions.empty()) {
            // Sort valid solutions by area (should already be sorted)
            std::sort(validSolutions.begin(), validSolutions.end());
            
            ss.str("");
            ss << "Found " << validSolutions.size() << " initial valid solutions with areas:";
            for (const auto& sol : validSolutions) {
                ss << " " << sol.area;
            }
            logSlicingPlacement(ss.str());
            
            // Get remaining time
            auto currentTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = currentTime - startTime;
            double remainingTime = globalTimeLimit - elapsed.count();
            
            // Calculate time for each area optimization attempt
            double timePerAttempt = remainingTime / min(validSolutions.size(), (size_t)3);
            
            if (remainingTime > 30.0) {
                logSlicingPlacement("Optimizing area with multi-start approach (" + to_string(remainingTime) + " seconds remaining)");
                
                int bestArea = numeric_limits<int>::max();
                vector<int> bestAreaExpression;
                
                // Try the top 3 solutions (or all if we have fewer)
                for (size_t i = 0; i < min(validSolutions.size(), (size_t)3); i++) {
                    ss.str("");
                    ss << "Area optimization attempt #" << (i+1) << " starting from solution with area=" 
                       << validSolutions[i].area;
                    logSlicingPlacement(ss.str());
                    
                    // Enhanced area optimization with more aggressive parameters
                    double initTemp = 2000.0; // Higher temperature for more exploration
                    double cooling = 0.97;    // Slower cooling to allow more exploration
                    
                    // Run area optimization
                    auto result = runAreaOptimization(validSolutions[i].expression, 
                                                     initTemp, cooling, timePerAttempt);
                    
                    // Check if this is better than our current best
                    int newArea = calculateArea(result);
                    ss.str("");
                    ss << "Area optimization result: " << newArea 
                       << " (started from " << validSolutions[i].area << ")";
                    logSlicingPlacement(ss.str());
                    
                    if (newArea < bestArea) {
                        bestArea = newArea;
                        bestAreaExpression = result;
                    }
                }
                
                // Use the best area solution
                expression = bestAreaExpression;
                ss.str("");
                ss << "Best area after optimization: " << bestArea;
                logSlicingPlacement(ss.str());
                
            } else {
                logSlicingPlacement("Not enough time for area optimization, using best valid solution.");
                expression = validSolutions[0].expression;
            }
        } else {
            logSlicingPlacement("Could not find valid solution. Attempting repair...");
            
            // Try to repair the solution
            bestSolution->setPolishExpression(expression);
            bestSolution->setCost(0);  // No cost for analog placement with no fixed outline
            bestSolution->applyFloorplanToBlocks();
            
            if (repairFloorplan()) {
                logSlicingPlacement("Successfully repaired floorplan to be valid!");
            } else {
                logSlicingPlacement("Could not repair floorplan to be fully valid.");
            }
        }
        
        // Set the best solution
        bestSolution->setPolishExpression(expression);
        bestSolution->setCost(0);  // No cost for analog placement with no fixed outline
        bestSolution->applyFloorplanToBlocks();
        
        // Final validation and reporting
        bool isNonOverlapping = !hasOverlaps(bestSolution);
        logSlicingPlacement("Final solution valid (no overlaps): " + string(isNonOverlapping ? "Yes" : "No"));
        
        int finalArea = calculateArea(expression);
        ss.str("");
        ss << "Final solution area: " << finalArea;
        logSlicingPlacement(ss.str());
        
        // Report total runtime
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> totalElapsed = endTime - startTime;
        logSlicingPlacement("Total algorithm runtime: " + to_string(totalElapsed.count()) + " seconds");
        
    } catch (const std::exception& e) {
        logSlicingPlacement("Exception in run method: " + string(e.what()));
        throw; // Re-throw to let the caller handle it
    } catch (...) {
        logSlicingPlacement("Unknown exception in run method");
        throw; // Re-throw to let the caller handle it
    }
}

// Calculate weighted area of a solution
int SimulatedAnnealing::calculateArea(const vector<int>& expression) {
    SlicingTreeNode* root = buildSlicingTree(expression);
    
    if (root == nullptr || root->shapeRecords.empty()) {
        logSlicingPlacement("Error: Failed to build valid tree for area calculation");
        return numeric_limits<int>::max();
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
    setBlockPositions(root, 0, 0, bestRecordIndex);
    
    // Calculate total area by finding the bounding box of all blocks
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
    
    // Calculate bounding box area
    int totalArea = (maxX - minX) * (maxY - minY);
    
    // Clean up
    delete root;
    
    return totalArea;
}

// Method for area optimization with more sophisticated approach
vector<int> SimulatedAnnealing::runAreaOptimization(
    const vector<int>& initialExpression, 
    double initialTemperature,
    double coolingRate,
    double maxRuntime
) {
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
        int rejectCount = 0;
        
        // Check runtime
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = currentTime - startTime;
        if (elapsed.count() > maxRuntime) {
            logSlicingPlacement("Runtime limit reached for area optimization.");
            break;
        }
        
        // Inner loop
        do {
            if (elapsed.count() > maxRuntime) {
                break;
            }
            
            // Choose move type - for area, M1 (operand swap) is most effective
            // Use 70% chance of M1 moves
            int moveType = (rand() % 100 < 70) ? 0 : (rand() % 3);
            vector<int> newExpression = perturbExpression(expression, moveType);
            
            // Skip if the move failed
            if (newExpression == expression) {
                ++rejectCount;
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
                    if (area < lastBestArea * 0.98) {
                        logSlicingPlacement("Improved area to: " + to_string(bestArea));
                        lastBestArea = bestArea;
                    }
                }
            } else {
                ++rejectCount;
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
            
            logSlicingPlacement("Reheating to temperature " + to_string(temperature) + 
                " and restarting from best solution (area=" + to_string(bestArea) + ")");
        }
        
        // Terminate if acceptance ratio is too low
        if (acceptedCount == 0 || static_cast<double>(acceptedCount) / tryingCount < 0.01) {
            logSlicingPlacement("Terminating area optimization due to low acceptance ratio");
            break;
        }
    }
    
    // Final logging
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    logSlicingPlacement("Area optimization completed in " + to_string(elapsed.count()) + 
        "s. Best area: " + to_string(bestArea));
    
    return bestExpression;
}

FloorplanSolution* SimulatedAnnealing::getBestSolution() const {
    return bestSolution;
}

vector<int> SimulatedAnnealing::generateInitialExpression() const {
    // Improved initial expression generation specifically for analog placement
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
    
    // For analog placement without fixed outline, use a balanced binary tree structure
    // which tends to produce more compact placements
    expression = buildBalancedTree(sortedBlocks, 0, sortedBlocks.size() - 1, true);
    
    // Log the expression for debugging
    stringstream ss;
    ss << "Initial expression: ";
    for (int val : expression) {
        if (val == SlicingTreeNode::HORIZONTAL_CUT) {
            ss << "H ";
        } else if (val == SlicingTreeNode::VERTICAL_CUT) {
            ss << "V ";
        } else {
            ss << val << " ";
        }
    }
    logSlicingPlacement(ss.str());
    
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
        
        logSlicingPlacement("Generated fallback expression");
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
    // First, validate the input expression
    if (expression.empty()) {
        // logSlicingPlacement("Error: Empty expression in perturbExpression");
        return expression;
    }
    
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
                // Pick the operand with largest positional difference from multiple candidates
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
                // For small expressions, simply pick a different operand
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
            
            // If no chains found, revert to simpler approach - invert a single cut
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
            // Find all adjacent operator-operand pairs that could be swapped
            vector<size_t> swapCandidates;
            for (size_t i = 0; i + 1 < candidateExpression.size(); ++i) {
                // We need an operand and an operator adjacent to each other
                if ((isCut(candidateExpression[i]) && !isCut(candidateExpression[i+1])) ||
                    (!isCut(candidateExpression[i]) && isCut(candidateExpression[i+1]))) {
                    
                    // Try making the swap and check if it maintains a valid Polish expression
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

SlicingTreeNode* SimulatedAnnealing::buildSlicingTree(const vector<int>& expression) {
    size_t cutIndex = 0;
    stack<SlicingTreeNode*> nodeStack;
    
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
            logSlicingPlacement("Operands: " + to_string(operands) + ", Operators: " + to_string(operators));
            return nullptr;
        }
        
        // Process expression
        for (int id : expression) {
            if (!isCut(id)) {
                // Ensure valid block index
                if (id < 0 || id >= data->getNumBlocks()) {
                    logSlicingPlacement("Error: Invalid block index: " + to_string(id));
                    return nullptr;
                }
                nodeStack.push(blockNodes[id]);
            } else {
                // Need at least 2 nodes for a cut
                if (nodeStack.size() < 2) {
                    logSlicingPlacement("Error: Not enough nodes for cut operation");
                    return nullptr;
                }
                
                // Ensure valid cut index
                if (cutIndex >= cutNodes.size()) {
                    logSlicingPlacement("Error: Cut index out of bounds: " + to_string(cutIndex));
                    return nullptr;
                }
                
                SlicingTreeNode* cutNode = cutNodes[cutIndex++];
                cutNode->type = id;
                cutNode->rightChild = nodeStack.top();
                nodeStack.pop();
                cutNode->leftChild = nodeStack.top();
                nodeStack.pop();
                cutNode->updateShapeRecords();
                nodeStack.push(cutNode);
            }
        }
        
        // Should have exactly one node left
        if (nodeStack.size() != 1) {
            logSlicingPlacement("Error: Invalid expression, final stack size: " + to_string(nodeStack.size()));
            return nullptr;
        }
        
        return nodeStack.top();
    } catch (const std::exception& e) {
        logSlicingPlacement("Exception in buildSlicingTree: " + string(e.what()));
        return nullptr;
    } catch (...) {
        logSlicingPlacement("Unknown exception in buildSlicingTree");
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

int SimulatedAnnealing::calculateCost(const vector<int>& expression, bool includeArea) {
    try {
        SlicingTreeNode* root = buildSlicingTree(expression);
        
        if (root == nullptr || root->shapeRecords.empty()) {
            logSlicingPlacement("Error: Failed to build valid slicing tree or empty shape records");
            return numeric_limits<int>::max();
        }
        
        // Find minimum area shape record
        int minArea = numeric_limits<int>::max();
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
            setBlockPositions(root, 0, 0, bestRecordIndex);
            
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
        
        // Clean up
        delete root;
        
        return minArea;
    } catch (const std::exception& e) {
        logSlicingPlacement("Exception in calculateCost: " + string(e.what()));
        return numeric_limits<int>::max();
    } catch (...) {
        logSlicingPlacement("Unknown exception in calculateCost");
        return numeric_limits<int>::max();
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
    double maxRuntime
) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Validate initial expression
    if (!validatePolishExpression(expression)) {
        logSlicingPlacement("Error: Invalid initial expression in simulated annealing");
        return {expression, numeric_limits<int>::max()};
    }
    
    int cost = calculateCost(expression, includeArea);
    
    vector<int> bestExpression = expression;
    int bestCost = cost;
    
    double temperature = initialTemperature;
    int maxTryingCount = movesPerTemperature * data->getNumBlocks();
    
    // Simulated annealing parameters
    const double logProbabilityThreshold = 0.01; // Controls early termination
    int stagnationCount = 0;
    const int maxStagnationCount = 5; // Terminate after X iterations without improvement
    
    // Log initial state
    logSlicingPlacement("Starting SA with T=" + to_string(temperature) + 
        ", minT=" + to_string(minTemperature) + 
        ", cooling=" + to_string(coolingRate));
    
    // Main simulated annealing loop
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
            if (elapsed.count() > maxRuntime) {
                logSlicingPlacement("Runtime limit reached during move evaluation. Terminating.");
                break;
            }
            
            // Choose move type
            int moveType;
            if (includeArea) {
                // For area optimization, prefer M1 (operand swap) and M2 (chain invert)
                moveType = (rand() % 100 < 70) ? rand() % 2 : 2;
            } else {
                // For solution validity, use balanced approach
                moveType = rand() % 3;
            }
            
            vector<int> newExpression = perturbExpression(expression, moveType);
            
            // If perturbation failed to produce a different valid expression, try again
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
                    
                    string qualityNote = "";
                    if (cost < bestCost * 0.95) {
                        qualityNote = " (significant improvement)";
                    }
                    
                    logSlicingPlacement("New best cost: " + to_string(bestCost) + qualityNote);
                }
            } else {
                ++rejectCount;
            }
            
            // Check time limit periodically
            if (tryingCount % 100 == 0) {
                currentTime = std::chrono::high_resolution_clock::now();
                elapsed = currentTime - startTime;
                if (elapsed.count() > maxRuntime) {
                    logSlicingPlacement("Runtime limit reached during move evaluation. Terminating.");
                    break;
                }
            }
        } while (uphillCount <= maxTryingCount && tryingCount <= 2 * maxTryingCount);
        
        // Update stagnation count
        if (improved) {
            stagnationCount = 0;
        } else {
            stagnationCount++;
            
            // Reheat the temperature if stagnating but not yet at max count
            if (stagnationCount > 0 && stagnationCount < maxStagnationCount) {
                // Reheat with diminishing returns
                temperature = initialTemperature * (0.8 - 0.1 * stagnationCount / maxStagnationCount);
                logSlicingPlacement("Reheating to temperature " + to_string(temperature) + 
                    " after stagnation (" + to_string(stagnationCount) + "/" + to_string(maxStagnationCount) + ")");
                
                continue; // Skip the normal cooling for this reheat iteration
            }
        }
        
        // Apply cooling
        temperature *= coolingRate;
        
        // Termination condition - if acceptance ratio is too low
        double acceptanceRatio = (tryingCount > 0) ? static_cast<double>(acceptedCount) / tryingCount : 0;
        if (acceptanceRatio < logProbabilityThreshold || acceptedCount == 0) {
            logSlicingPlacement("Terminating SA due to low acceptance ratio: " + to_string(acceptanceRatio));
            break;
        }
        
        // Additional termination condition from original code
        if (static_cast<double>(rejectCount) / tryingCount > maxRejectRatio) {
            logSlicingPlacement("Terminating SA due to high rejection ratio: " + 
                to_string(static_cast<double>(rejectCount) / tryingCount));
            break;
        }
    }
    
    // Log runtime statistics
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    
    if (elapsed.count() >= maxRuntime) {
        logSlicingPlacement("Maximum runtime (" + to_string(maxRuntime) + "s) reached for this annealing run.");
    } else {
        logSlicingPlacement("Simulated annealing completed in " + to_string(elapsed.count()) + 
            "s (stagnation count: " + to_string(stagnationCount) + ")");
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