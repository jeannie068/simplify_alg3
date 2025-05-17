#include "slicing_struct.hpp"
#include <algorithm>
#include <limits>
#include <iostream>
#include <stack>

// Block implementation
Block::Block(const std::string& name, int width, int height)
    : name(name), width(width), height(height), x(0), y(0), rotated(false) {
}

Block::~Block() {
}

const std::string& Block::getName() const {
    return name;
}

int Block::getWidth() const {
    return width;
}

int Block::getHeight() const {
    return height;
}

int Block::getX() const {
    return x;
}

int Block::getY() const {
    return y;
}

bool Block::isRotated() const {
    return rotated;
}

void Block::setX(int x) {
    this->x = x;
}

void Block::setY(int y) {
    this->y = y;
}

void Block::setRotated(bool rotated) {
    this->rotated = rotated;
}

void Block::updatePosition(int x, int y, int width, int height) {
    this->x = x;
    this->y = y;
    this->rotated = !(this->width == width && this->height == height);
}

int Block::getCenterX() const {
    return x + (rotated ? height : width) / 2;
}

int Block::getCenterY() const {
    return y + (rotated ? width : height) / 2;
}

// FloorplanData implementation
FloorplanData::FloorplanData()
    : floorplanWidth(0), floorplanHeight(0) {
}

FloorplanData::~FloorplanData() {
    for (Block* block : blocks) {
        delete block;
    }
}

void FloorplanData::addBlock(Block* block) {
    blocks.push_back(block);
}

void FloorplanData::setFloorplanDimensions(int width, int height) {
    floorplanWidth = width;
    floorplanHeight = height;
}

int FloorplanData::getNumBlocks() const {
    return blocks.size();
}

Block* FloorplanData::getBlock(int index) const {
    return blocks[index];
}

Block* FloorplanData::getBlockByName(const std::string& name) const {
    for (Block* block : blocks) {
        if (block->getName() == name) {
            return block;
        }
    }
    return nullptr;
}

int FloorplanData::getFloorplanWidth() const {
    return floorplanWidth;
}

int FloorplanData::getFloorplanHeight() const {
    return floorplanHeight;
}

// ShapeRecord implementation
ShapeRecord::ShapeRecord()
    : width(0), height(0), leftChoice(0), rightChoice(0) {
}

ShapeRecord::ShapeRecord(int w, int h, int lc, int rc)
    : width(w), height(h), leftChoice(lc), rightChoice(rc) {
}

// SlicingTreeNode implementation
SlicingTreeNode::SlicingTreeNode() 
    : type(BLOCK), block(nullptr), leftChild(nullptr), rightChild(nullptr), userData(nullptr) {

}

SlicingTreeNode::SlicingTreeNode(int type, Block* block)
    : type(type), block(block), leftChild(nullptr), rightChild(nullptr) {
    
    if (block) {
        // Create shape records for the block (unrotated and rotated)
        shapeRecords.emplace_back(block->getWidth(), block->getHeight(), 0, 0);
        shapeRecords.emplace_back(block->getHeight(), block->getWidth(), 1, 1);
    }
}

SlicingTreeNode::~SlicingTreeNode() {
        // Cleanup userData if it exists
        if (userData) {
            delete static_cast<std::vector<std::shared_ptr<SlicingTreeNode>>*>(userData);
            userData = nullptr;
        }
    }

void SlicingTreeNode::updateShapeRecords() {
    shapeRecords.clear();
    
    if (type == HORIZONTAL_CUT) {
        // Sort by width (ascending order)
        auto compareWidth = [](const ShapeRecord& a, const ShapeRecord& b) -> bool {
            return a.width <= b.width;
        };
        
        std::sort(leftChild->shapeRecords.begin(), leftChild->shapeRecords.end(), compareWidth);
        std::sort(rightChild->shapeRecords.begin(), rightChild->shapeRecords.end(), compareWidth);
        
        int l = leftChild->shapeRecords.size() - 1;
        int r = rightChild->shapeRecords.size() - 1;
        
        while (l >= 0 && r >= 0) {
            const ShapeRecord& leftRecord = leftChild->shapeRecords[l];
            const ShapeRecord& rightRecord = rightChild->shapeRecords[r];
            
            shapeRecords.emplace_back(
                std::max(leftRecord.width, rightRecord.width),
                leftRecord.height + rightRecord.height,
                l, r
            );
            
            if (leftRecord.width >= rightRecord.width) {
                --l;
            }
            if (leftRecord.width <= rightRecord.width) {
                --r;
            }
        }
    } else if (type == VERTICAL_CUT) {
        // Sort by height (descending order)
        auto compareHeight = [](const ShapeRecord& a, const ShapeRecord& b) -> bool {
            return a.height >= b.height;
        };
        
        std::sort(leftChild->shapeRecords.begin(), leftChild->shapeRecords.end(), compareHeight);
        std::sort(rightChild->shapeRecords.begin(), rightChild->shapeRecords.end(), compareHeight);
        
        size_t l = 0, r = 0;
        
        while (l < leftChild->shapeRecords.size() && r < rightChild->shapeRecords.size()) {
            const ShapeRecord& leftRecord = leftChild->shapeRecords[l];
            const ShapeRecord& rightRecord = rightChild->shapeRecords[r];
            
            shapeRecords.emplace_back(
                leftRecord.width + rightRecord.width,
                std::max(leftRecord.height, rightRecord.height),
                l, r
            );
            
            if (leftRecord.height >= rightRecord.height) {
                ++l;
            }
            if (leftRecord.height <= rightRecord.height) {
                ++r;
            }
        }
    }
}

// FloorplanSolution implementation
FloorplanSolution::FloorplanSolution(FloorplanData* data)
    : data(data), cost(0) {
}

FloorplanSolution::~FloorplanSolution() {
}

void FloorplanSolution::setPolishExpression(const std::vector<int>& expr) {
    polishExpression = expr;
}

const std::vector<int>& FloorplanSolution::getPolishExpression() const {
    return polishExpression;
}

int FloorplanSolution::getCost() const {
    return cost;
}

void FloorplanSolution::setCost(int cost) {
    this->cost = cost;
}

bool FloorplanSolution::isValid() const {
    // Check if all blocks are within floorplan boundaries
    for (int i = 0; i < data->getNumBlocks(); ++i) {
        Block* block = data->getBlock(i);
        int blockWidth = block->isRotated() ? block->getHeight() : block->getWidth();
        int blockHeight = block->isRotated() ? block->getWidth() : block->getHeight();
        
        if (block->getX() < 0 || block->getY() < 0 || 
            block->getX() + blockWidth > data->getFloorplanWidth() ||
            block->getY() + blockHeight > data->getFloorplanHeight()) {
            return false;
        }
    }
    
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
                return false;
            }
        }
    }
    
    return true;
}

void FloorplanSolution::applyFloorplanToBlocks() {
    SlicingTreeNode* root = buildSlicingTree();
    
    // Find the best shape record that fits within the floorplan boundaries
    int floorplanWidth = data->getFloorplanWidth();
    int floorplanHeight = data->getFloorplanHeight();
    
    int selectedRecord = -1;
    for (size_t i = 0; i < root->shapeRecords.size(); ++i) {
        const ShapeRecord& record = root->shapeRecords[i];
        if (record.width <= floorplanWidth && record.height <= floorplanHeight) {
            selectedRecord = i;
            break;
        }
    }
    
    // If no valid shape found, use the first one
    if (selectedRecord == -1 && !root->shapeRecords.empty()) {
        selectedRecord = 0;
    }
    
    // Set block positions
    if (selectedRecord != -1) {
        setBlockPositions(root, 0, 0, selectedRecord);
    }
    
    // Clean up the slicing tree
    delete root;
}

SlicingTreeNode* FloorplanSolution::buildSlicingTree() {
    std::stack<SlicingTreeNode*> nodeStack;
    
    for (int id : polishExpression) {
        if (id >= 0) {  // Block
            Block* block = data->getBlock(id);
            SlicingTreeNode* node = new SlicingTreeNode(SlicingTreeNode::BLOCK, block);
            nodeStack.push(node);
        } else {  // Cut
            SlicingTreeNode* node = new SlicingTreeNode(id, nullptr);
            
            // Pop the right and left children
            node->rightChild = nodeStack.top();
            nodeStack.pop();
            node->leftChild = nodeStack.top();
            nodeStack.pop();
            
            // Update shape records
            node->updateShapeRecords();
            
            // Push the new node
            nodeStack.push(node);
        }
    }
    
    // The root of the slicing tree
    return nodeStack.top();
}

void FloorplanSolution::setBlockPositions(SlicingTreeNode* node, int x, int y, int recordIndex) {
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