#pragma once
#include <string>
#include <memory>
#include <vector>
#include <map>
#include "../data_struct/Module.hpp"
#include "../data_struct/SymmetryConstraint.hpp"

class Parser {
public:
    /**
     * Parses the input file and creates Module and SymmetryGroup objects
     * 
     * @param filename Path to the input file
     * @param modules Output map of module names to Module objects
     * @param symmetryGroups Output vector of SymmetryGroup objects
     * @return True if parsing was successful, false otherwise
     */
    static bool parseInputFile(const std::string& filename, 
                              std::map<std::string, std::shared_ptr<Module>>& modules,
                              std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups);
    
    /**
     * Writes the placement result to the output file
     * 
     * @param filename Path to the output file
     * @param modules Map of module names to Module objects with their final positions
     * @param totalArea Total area of the placement
     * @return True if writing was successful, false otherwise
     */
    static bool writeOutputFile(const std::string& filename,
                               const std::map<std::string, std::shared_ptr<Module>>& modules,
                               int totalArea);
};