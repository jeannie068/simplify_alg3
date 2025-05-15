#include "../parser/Parser.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cctype>

/**
 * Parses the input file and creates Module and SymmetryGroup objects
 */
bool Parser::parseInputFile(const std::string& filename, 
                           std::map<std::string, std::shared_ptr<Module>>& modules,
                           std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups) {
    // Clear the output containers
    modules.clear();
    symmetryGroups.clear();
    
    // Open the input file
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        std::cerr << "Error: Could not open input file " << filename << std::endl;
        return false;
    }
    
    // Parse the file line by line
    std::string line;
    int numHardBlocks = 0;
    int numSymGroups = 0;
    int currentSymGroupIndex = -1;
    
    while (std::getline(inFile, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '/' || line[0] == '#') {
            continue;
        }
        
        // Create a string stream for the line
        std::istringstream iss(line);
        
        // Parse the line based on the keyword
        std::string keyword;
        iss >> keyword;
        
        if (keyword == "NumHardBlocks") {
            // Parse the number of hard blocks
            iss >> numHardBlocks;
            std::cout << "Number of hard blocks: " << numHardBlocks << std::endl;
        } 
        else if (keyword == "HardBlock") {
            // Parse a hard block definition
            std::string name;
            int width, height;
            iss >> name >> width >> height;
            
            // Create a module object
            auto module = std::make_shared<Module>(name, width, height);
            modules[name] = module;
            
            std::cout << "Hard block: " << name << " " << width << " " << height << std::endl;
        } 
        else if (keyword == "NumSymGroups") {
            // Parse the number of symmetry groups
            iss >> numSymGroups;
            std::cout << "Number of symmetry groups: " << numSymGroups << std::endl;
        } 
        else if (keyword == "SymGroup") {
            // Parse a symmetry group definition
            std::string name;
            int numBlocks;
            iss >> name >> numBlocks;
            
            // Create a symmetry group object
            auto symGroup = std::make_shared<SymmetryGroup>(name, SymmetryType::VERTICAL);
            symmetryGroups.push_back(symGroup);
            currentSymGroupIndex = symmetryGroups.size() - 1;
            
            std::cout << "Symmetry group: " << name << " " << numBlocks << std::endl;
        } 
        else if (keyword == "SymPair") {
            // Parse a symmetry pair definition
            std::string name1, name2;
            iss >> name1 >> name2;
            
            // Add the symmetry pair to the current symmetry group
            if (currentSymGroupIndex >= 0 && currentSymGroupIndex < static_cast<int>(symmetryGroups.size())) {
                symmetryGroups[currentSymGroupIndex]->addSymmetryPair(name1, name2);
                std::cout << "Symmetry pair: " << name1 << " " << name2 << std::endl;
            } else {
                std::cerr << "Error: SymPair defined outside of a SymGroup" << std::endl;
                inFile.close();
                return false;
            }
        } 
        else if (keyword == "SymSelf") {
            // Parse a self-symmetric module definition
            std::string name;
            iss >> name;
            
            // Add the self-symmetric module to the current symmetry group
            if (currentSymGroupIndex >= 0 && currentSymGroupIndex < static_cast<int>(symmetryGroups.size())) {
                symmetryGroups[currentSymGroupIndex]->addSelfSymmetric(name);
                std::cout << "Self-symmetric module: " << name << std::endl;
            } else {
                std::cerr << "Error: SymSelf defined outside of a SymGroup" << std::endl;
                inFile.close();
                return false;
            }
        } 
        else {
            // Unknown keyword
            std::cerr << "Warning: Unknown keyword " << keyword << std::endl;
        }
    }
    
    // Close the input file
    inFile.close();
    
    // Check if the number of hard blocks matches
    if (static_cast<int>(modules.size()) != numHardBlocks) {
        std::cerr << "Error: Number of hard blocks does not match" << std::endl;
        return false;
    }
    
    // Check if the number of symmetry groups matches
    if (static_cast<int>(symmetryGroups.size()) != numSymGroups) {
        std::cerr << "Error: Number of symmetry groups does not match" << std::endl;
        return false;
    }
    
    // Verify that all modules in symmetry groups exist
    for (const auto& group : symmetryGroups) {
        for (const auto& pair : group->getSymmetryPairs()) {
            if (modules.find(pair.first) == modules.end()) {
                std::cerr << "Error: Module " << pair.first << " in symmetry pair does not exist" << std::endl;
                return false;
            }
            if (modules.find(pair.second) == modules.end()) {
                std::cerr << "Error: Module " << pair.second << " in symmetry pair does not exist" << std::endl;
                return false;
            }
        }
        
        for (const auto& name : group->getSelfSymmetric()) {
            if (modules.find(name) == modules.end()) {
                std::cerr << "Error: Self-symmetric module " << name << " does not exist" << std::endl;
                return false;
            }
        }
    }
    
    // Print some statistics
    std::cout << "Successfully parsed " << modules.size() << " modules and " 
              << symmetryGroups.size() << " symmetry groups" << std::endl;
    
    return true;
}

/**
 * Writes the placement result to the output file
 */
bool Parser::writeOutputFile(const std::string& filename,
                            const std::map<std::string, std::shared_ptr<Module>>& modules,
                            int totalArea) {
    // Open the output file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open output file " << filename << std::endl;
        return false;
    }
    
    // Write the total area
    outFile << "Area " << totalArea << std::endl;
    
    // Write the number of hard blocks
    outFile << "NumHardBlocks " << modules.size() << std::endl;
    
    // Write the module positions and rotation status
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        outFile << module->getName() << " " 
                << module->getX() << " " 
                << module->getY() << " " 
                << (module->getRotated() ? "1" : "0") 
                << std::endl;
    }
    
    // Close the output file
    outFile.close();
    
    std::cout << "Successfully wrote output to " << filename << std::endl;
    
    return true;
}