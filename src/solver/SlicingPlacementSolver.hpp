
#include <map>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include "../data_struct/Module.hpp"
#include "../data_struct/SymmetryIslandBlock.hpp"
#include "../slicing/slicing_struct.hpp"
#include "../slicing/slicing_sa.hpp"

class SlicingPlacementSolver {
private:
    // Original data
    std::map<std::string, std::shared_ptr<Module>>& regularModules;
    std::vector<std::shared_ptr<SymmetryIslandBlock>>& symmetryIslands;
    
    // Slicing structures
    std::unique_ptr<FloorplanData> floorplanData;
    std::unique_ptr<SimulatedAnnealing> optimizer;
    
    // Mapping between block indices and original modules/islands
    std::unordered_map<int, std::string> blockToRegularModule;
    std::unordered_map<int, int> blockToSymmetryIsland;
    
public:
    SlicingPlacementSolver(
        std::map<std::string, std::shared_ptr<Module>>& regularModules,
        std::vector<std::shared_ptr<SymmetryIslandBlock>>& symmetryIslands,
        int maxWidth, int maxHeight)
        : regularModules(regularModules), symmetryIslands(symmetryIslands) {
    
        floorplanData = std::make_unique<FloorplanData>();
        floorplanData->setFloorplanDimensions(maxWidth, maxHeight);
        
        initializeFloorplanData();
        
        optimizer = std::make_unique<SimulatedAnnealing>(floorplanData.get());
    }

    
    bool solve() {
        // Run the slicing optimizer
        optimizer->run();
        
        // Get the best solution
        const FloorplanSolution* solution = optimizer->getBestSolution();
        
        // Apply positions back to original structures
        applyOptimizedPositions(solution);
        
        return solution->isValid();
    }

    int calculateFinalArea() {
        // Get the current solution
        const FloorplanSolution* solution = optimizer->getBestSolution();
        
        // Calculate bounding box
        int minX = std::numeric_limits<int>::max();
        int minY = std::numeric_limits<int>::max();
        int maxX = 0;
        int maxY = 0;
        
        for (int i = 0; i < floorplanData->getNumBlocks(); i++) {
            Block* block = floorplanData->getBlock(i);
            int blockWidth = block->isRotated() ? block->getHeight() : block->getWidth();
            int blockHeight = block->isRotated() ? block->getWidth() : block->getHeight();
            
            minX = std::min(minX, block->getX());
            minY = std::min(minY, block->getY());
            maxX = std::max(maxX, block->getX() + blockWidth);
            maxY = std::max(maxY, block->getY() + blockHeight);
        }
        
        return (maxX - minX) * (maxY - minY);
    }
    
private:
    void initializeFloorplanData() {
        int blockIndex = 0;
        
        // Add symmetry islands as blocks
        for (size_t i = 0; i < symmetryIslands.size(); i++) {
            auto island = symmetryIslands[i];
            if (!island) continue;
            
            std::string name = "island_" + std::to_string(i);
            Block* block = new Block(name, island->getWidth(), island->getHeight());
            floorplanData->addBlock(block);
            
            blockToSymmetryIsland[blockIndex++] = i;
        }
        
        // Add regular modules as blocks
        for (const auto& pair : regularModules) {
            const auto& moduleName = pair.first;
            const auto& module = pair.second;
            if (!module) continue;
            
            Block* block = new Block(moduleName, module->getWidth(), module->getHeight());
            floorplanData->addBlock(block);
            
            blockToRegularModule[blockIndex++] = moduleName;
        }
    }

    void applyOptimizedPositions(const FloorplanSolution* solution) {
        // Process each block in the solution
        for (int i = 0; i < floorplanData->getNumBlocks(); i++) {
            Block* block = floorplanData->getBlock(i);
            bool rotated = block->isRotated();
            int x = block->getX();
            int y = block->getY();
            
            // Check if this is a symmetry island
            auto islandIt = blockToSymmetryIsland.find(i);
            if (islandIt != blockToSymmetryIsland.end()) {
                int islandIndex = islandIt->second;
                auto island = symmetryIslands[islandIndex];
                
                // If island was rotated in the solution, rotate it
                if (rotated) {
                    island->rotate();
                }
                
                // Set island position
                island->setPosition(x, y);
            } 
            // Or a regular module
            else {
                auto moduleIt = blockToRegularModule.find(i);
                if (moduleIt != blockToRegularModule.end()) {
                    auto& moduleName = moduleIt->second;
                    auto module = regularModules[moduleName];
                    
                    // Set rotation and position
                    module->setRotation(rotated);
                    module->setPosition(x, y);
                }
            }
        }
    }
};