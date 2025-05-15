// SymmetryConstraint.hpp
#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <memory>
using namespace std;

enum class SymmetryType {
    VERTICAL,
    HORIZONTAL
};

class SymmetryGroup {
private:
    string name;                                // Name of the symmetry group
    vector<pair<string, string>> symmetryPairs;  // Pairs of modules
    vector<string> selfSymmetric;          // Self-symmetric modules
    SymmetryType type;                               // Symmetry type (vertical/horizontal)
    
    // Hash table for lookup structures
    unordered_map<string, string> pairMap;  // For quick symmetry pair lookup
    unordered_set<string> selfSymSet;            // For quick self-symmetric lookup
    unordered_set<string> allModules;            // All modules in this group
    
    // Symmetry axis position - VERTICAL: x; HORIZONTAL: y
    double axisPosition;

public:
    // Constructors
    SymmetryGroup(const string& name, SymmetryType type = SymmetryType::VERTICAL);
    
    // Module management
    void addSymmetryPair(const string& module1, const string& module2);
    void addSelfSymmetric(const string& module);
    
    // Query functions
    bool isInGroup(const string& moduleName) const;
    bool isSelfSymmetric(const string& moduleName) const;
    bool isSymmetryPair(const string& module1, const string& module2) const;
    string getSymmetricPair(const string& moduleName) const;
    
    // Symmetry island formation validation
    bool isSymmetryIsland(const unordered_map<string, pair<int, int>>& positions, 
                         const unordered_map<string, pair<int, int>>& dimensions) const;
    
    // Getters
    string getName() const;
    const vector<pair<string, string>>& getSymmetryPairs() const;
    const vector<string>& getSelfSymmetric() const;
    SymmetryType getType() const;
    int getNumModules() const;
    int getNumPairs() const;
    int getNumSelfSymmetric() const;
    double getAxisPosition() const;
    const unordered_set<string>& getAllModules() const;
    
    // Setters
    void setAxisPosition(double position);
    void setType(SymmetryType newType);
    
    // Actions
    void changeSymmetryType();
    
    // Validation and utility functions
    bool validateSymmetricPlacement(const unordered_map<string, pair<int, int>>& positions,
                                  const unordered_map<string, pair<int, int>>& dimensions) const;
    double calculateAxisPosition(const unordered_map<string, pair<int, int>>& positions) const;
};