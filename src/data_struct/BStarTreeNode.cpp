// BStarTreeNode.cpp

#include "BStarTreeNode.hpp"
#include <queue>

/**
 * Constructor
 */
BStarTreeNode::BStarTreeNode(const std::string& moduleName)
    : moduleName(moduleName), 
      leftChild(nullptr), 
      rightChild(nullptr) {
    // Parent is initialized as empty weak_ptr by default
}

BStarTreeNode::~BStarTreeNode() {
}

std::string BStarTreeNode::getModuleName() const {
    return moduleName;
}

std::shared_ptr<BStarTreeNode> BStarTreeNode::getLeftChild() const {
    return leftChild;
}

std::shared_ptr<BStarTreeNode> BStarTreeNode::getRightChild() const {
    return rightChild;
}

std::shared_ptr<BStarTreeNode> BStarTreeNode::getParent() const {
    // Convert weak_ptr to shared_ptr for returning
    return parent.lock();
}

void BStarTreeNode::setLeftChild(std::shared_ptr<BStarTreeNode> node) {
    leftChild = node;
}

void BStarTreeNode::setRightChild(std::shared_ptr<BStarTreeNode> node) {
    rightChild = node;
}

void BStarTreeNode::setParent(std::shared_ptr<BStarTreeNode> node) {
    parent = node;  // Automatically converts to weak_ptr
}

/**
 * Creates a deep copy of this node and its children
 */
std::shared_ptr<BStarTreeNode> BStarTreeNode::clone() const {
    // Create a new node with the same module name
    auto clonedNode = std::make_shared<BStarTreeNode>(moduleName);
    
    // Recursively clone children
    if (leftChild) {
        auto clonedLeftChild = leftChild->clone();
        clonedNode->setLeftChild(clonedLeftChild);
        clonedLeftChild->setParent(clonedNode);
    }
    
    if (rightChild) {
        auto clonedRightChild = rightChild->clone();
        clonedNode->setRightChild(clonedRightChild);
        clonedRightChild->setParent(clonedNode);
    }
    
    return clonedNode;
}

/**
 * Checks if this node is a leaf node (no children)
 */
bool BStarTreeNode::isLeaf() const {
    return !leftChild && !rightChild;
}

/**
 * Checks if this node is the left child of its parent
 */
bool BStarTreeNode::isLeftChild() const {
    auto parentNode = parent.lock();
    if (!parentNode) return false;  // No parent
    
    return parentNode->getLeftChild().get() == this;
}

/**
 * Checks if this node is the right child of its parent
 */
bool BStarTreeNode::isRightChild() const {
    auto parentNode = parent.lock();
    if (!parentNode) return false;  // No parent
    
    return parentNode->getRightChild().get() == this;
}

/**
 * Counts the total number of nodes in the subtree rooted at this node
 */
int BStarTreeNode::countSubtreeNodes() const {
    int count = 1;  // Count this node
    
    // Use breadth-first search to count all nodes
    std::queue<std::shared_ptr<BStarTreeNode>> nodeQueue;
    
    // Add children to queue if they exist
    if (leftChild) nodeQueue.push(leftChild);
    if (rightChild) nodeQueue.push(rightChild);
    
    // Process all nodes in the queue
    while (!nodeQueue.empty()) {
        auto current = nodeQueue.front();
        nodeQueue.pop();
        
        count++;  // Count this node
        
        // Add children to queue if they exist
        if (current->getLeftChild()) nodeQueue.push(current->getLeftChild());
        if (current->getRightChild()) nodeQueue.push(current->getRightChild());
    }
    
    return count;
}