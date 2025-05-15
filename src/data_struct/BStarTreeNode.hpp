/**
 * BStarTreeNode.hpp
 * 
 * This file defines the BStarTreeNode class, which represents a node in the B*-tree
 * representation for analog placement. B*-trees provide an efficient way to represent
 * non-slicing floorplans, with O(n) packing time complexity.
 */

#pragma once

#include <memory>
#include <string>

class BStarTreeNode {
private:
    // Module name corresponding to this node
    std::string moduleName;
    
    // Tree structure links
    std::shared_ptr<BStarTreeNode> leftChild;  // Module to the right
    std::shared_ptr<BStarTreeNode> rightChild; // Module above
    std::weak_ptr<BStarTreeNode> parent;       // Parent node (weak to avoid circular references)
    
public:
    /**
     * Constructor
     * 
     * @param moduleName Name of the module corresponding to this node
     */
    BStarTreeNode(const std::string& moduleName);
    ~BStarTreeNode();
    
    std::string getModuleName() const;
    
    /**
     * Gets the left child (module to the right)
     * 
     * @return Left child node
     */
    std::shared_ptr<BStarTreeNode> getLeftChild() const;
    
    /**
     * Gets the right child (module above)
     * 
     * @return Right child node
     */
    std::shared_ptr<BStarTreeNode> getRightChild() const;
    
    /**
     * Gets the parent node
     * 
     * @return Parent node
     */
    std::shared_ptr<BStarTreeNode> getParent() const;
    
    /**
     * Sets the left child (module to the right)
     * 
     * @param node New left child node
     */
    void setLeftChild(std::shared_ptr<BStarTreeNode> node);
    
    /**
     * Sets the right child (module above)
     * 
     * @param node New right child node
     */
    void setRightChild(std::shared_ptr<BStarTreeNode> node);
    
    /**
     * Sets the parent node
     * 
     * @param node New parent node
     */
    void setParent(std::shared_ptr<BStarTreeNode> node);
    
    /**
     * Creates a deep copy of this node and its children
     * 
     * @return A new node that is a deep copy of this one
     */
    std::shared_ptr<BStarTreeNode> clone() const;
    
    /**
     * Checks if this node is a leaf node (no children)
     * 
     * @return True if the node is a leaf node, false otherwise
     */
    bool isLeaf() const;
    
    /**
     * Checks if this node is the left child of its parent
     * 
     * @return True if the node is the left child, false otherwise
     */
    bool isLeftChild() const;
    
    /**
     * Checks if this node is the right child of its parent
     * 
     * @return True if the node is the right child, false otherwise
     */
    bool isRightChild() const;
    
    /**
     * Counts the total number of nodes in the subtree rooted at this node
     * 
     * @return Number of nodes in the subtree
     */
    int countSubtreeNodes() const;
};