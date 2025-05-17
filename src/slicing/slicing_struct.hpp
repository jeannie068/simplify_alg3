#pragma once

#include <string>
#include <vector>
#include <memory>

// Forward declarations
class Block;
class SlicingTreeNode;

class Block {
public:
    Block(const std::string& name, int width, int height);
    ~Block();
    
    const std::string& getName() const;
    int getWidth() const;
    int getHeight() const;
    int getX() const;
    int getY() const;
    bool isRotated() const;
    
    void setX(int x);
    void setY(int y);
    void setRotated(bool rotated);
    void updatePosition(int x, int y, int width, int height);
    
    int getCenterX() const;
    int getCenterY() const;
    
private:
    std::string name;
    int width;
    int height;
    int x;
    int y;
    bool rotated;
};

class FloorplanData {
public:
    FloorplanData();
    ~FloorplanData();
    
    // Add components
    void addBlock(Block* block);
    
    // Set floorplan dimensions and deadspace ratio
    void setFloorplanDimensions(int width, int height);
    
    int getNumBlocks() const;
    Block* getBlock(int index) const;
    Block* getBlockByName(const std::string& name) const;
    int getFloorplanWidth() const;
    int getFloorplanHeight() const;
    
private:
    std::vector<Block*> blocks;
    int floorplanWidth;
    int floorplanHeight;
};

// Shape record for slicing tree nodes
struct ShapeRecord {
    int width;
    int height;
    int leftChoice;  // Index of child's shape record
    int rightChoice; // Index of child's shape record
    
    ShapeRecord();
    ShapeRecord(int w, int h, int lc, int rc);
};

// Node in the slicing tree
class SlicingTreeNode {
public:
    enum Type {
        HORIZONTAL_CUT = -2,
        VERTICAL_CUT = -1,
        BLOCK = 0
    };
    
    SlicingTreeNode();
    SlicingTreeNode(int type, Block* block = nullptr);
    ~SlicingTreeNode();
    
    // Update shape records
    void updateShapeRecords();
    
    // Member variables
    int type;
    Block* block;
    SlicingTreeNode* leftChild;
    SlicingTreeNode* rightChild;
    std::vector<ShapeRecord> shapeRecords;
    void* userData;
};

// Solution representation for the floorplan
class FloorplanSolution {
public:
    FloorplanSolution(FloorplanData* data);
    ~FloorplanSolution();
    
    // Polish expression operations
    void setPolishExpression(const std::vector<int>& expr);
    const std::vector<int>& getPolishExpression() const;
    
    // Solution cost and validation
    int getCost() const;
    void setCost(int cost);
    bool isValid() const;
    
    // Apply floorplan to blocks
    void applyFloorplanToBlocks();
    
private:
    FloorplanData* data;
    std::vector<int> polishExpression;
    int cost;
    
    // Build slicing tree from polish expression
    SlicingTreeNode* buildSlicingTree();
    
    // Set positions of blocks based on slicing tree
    void setBlockPositions(SlicingTreeNode* node, int x, int y, int recordIndex);
};
