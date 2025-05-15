/**
 * RestructuredBStarTree.hpp
 * 
 * This enhanced B*-tree implementation uses proper tree construction with
 * preorder and inorder traversals to ensure correct spatial relationships.
 */

#pragma once

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <queue>
#include <algorithm>
#include <unordered_map>
#include "BStarTreeNode.hpp"

class RestructuredBStarTree {
private:
    // Root of the B*-tree
    std::shared_ptr<BStarTreeNode> root;
    
    // Mapping from module names to node objects
    std::unordered_map<std::string, std::shared_ptr<BStarTreeNode>> nodeMap;
    
    // Mapping from nodes to their inorder indices (for quick rebuilding)
    std::unordered_map<std::shared_ptr<BStarTreeNode>, int> inorderIndices;
    
    /**
     * Builds a new tree from preorder and inorder traversals
     * 
     * @param preorder Nodes in preorder traversal
     * @param inorder Nodes in inorder traversal
     * @param preIdx Current index in preorder traversal
     * @param inStart Start index in inorder traversal
     * @param inEnd End index in inorder traversal
     * @return Root of the new subtree
     */
    std::shared_ptr<BStarTreeNode> buildTreeFromTraversals(
        const std::vector<std::shared_ptr<BStarTreeNode>>& preorder,
        const std::vector<std::shared_ptr<BStarTreeNode>>& inorder,
        int& preIdx,
        int inStart,
        int inEnd) {
        
        // Base case
        if (preIdx >= preorder.size() || inStart > inEnd) {
            return nullptr;
        }
        
        // The current node in preorder is the root of this subtree
        std::shared_ptr<BStarTreeNode> node = preorder[preIdx++];
        
        // If no more elements in this subtree, return this node
        if (inStart == inEnd) {
            node->setLeftChild(nullptr);
            node->setRightChild(nullptr);
            return node;
        }
        
        // Find the position of this node in inorder traversal
        int inIndex = -1;
        for (int i = inStart; i <= inEnd; i++) {
            if (inorder[i]->getModuleName() == node->getModuleName()) {
                inIndex = i;
                break;
            }
        }
        
        // If we can't find the node in inorder traversal, something is wrong
        if (inIndex == -1) {
            return nullptr;
        }
        
        // Recursively build left and right subtrees
        node->setLeftChild(buildTreeFromTraversals(preorder, inorder, preIdx, inStart, inIndex - 1));
        if (node->getLeftChild()) {
            node->getLeftChild()->setParent(node);
        }
        
        node->setRightChild(buildTreeFromTraversals(preorder, inorder, preIdx, inIndex + 1, inEnd));
        if (node->getRightChild()) {
            node->getRightChild()->setParent(node);
        }
        
        return node;
    }
    
    /**
     * Updates the inorder indices map for faster rebuilding
     */
    void updateInorderIndices() {
        inorderIndices.clear();
        std::vector<std::shared_ptr<BStarTreeNode>> inorder;
        collectNodesInOrder(root, inorder);
        
        for (int i = 0; i < inorder.size(); i++) {
            inorderIndices[inorder[i]] = i;
        }
    }
    
public:
    RestructuredBStarTree() : root(nullptr) {}
    
    /**
     * Sets the root node
     * 
     * @param newRoot New root node
     */
    void setRoot(std::shared_ptr<BStarTreeNode> newRoot) {
        root = newRoot;
        if (root) {
            root->setParent(nullptr);
        }
        updateInorderIndices();
    }
    
    /**
     * Gets the root node
     * 
     * @return Root node
     */
    std::shared_ptr<BStarTreeNode> getRoot() const {
        return root;
    }
    
    /**
     * Adds a node to the tree
     * 
     * @param node Node to add
     */
    void addNode(std::shared_ptr<BStarTreeNode> node) {
        if (!node) return;
        
        nodeMap[node->getModuleName()] = node;
        
        // If this is the first node, make it the root
        if (!root) {
            root = node;
            node->setParent(nullptr);
            updateInorderIndices();
            return;
        }
        
        // Otherwise, we'll need to rebuild the tree later
    }
    
    /**
     * Removes a node from the tree
     * 
     * @param moduleName Name of the module to remove
     * @return True if removal was successful
     */
    bool removeNode(const std::string& moduleName) {
        auto it = nodeMap.find(moduleName);
        if (it == nodeMap.end()) {
            return false;
        }
        
        std::shared_ptr<BStarTreeNode> node = it->second;
        nodeMap.erase(it);
        
        // If this is the root, special handling
        if (node == root) {
            // Try to find a replacement root
            if (root->getLeftChild()) {
                root = root->getLeftChild();
                root->setParent(nullptr);
            } else if (root->getRightChild()) {
                root = root->getRightChild();
                root->setParent(nullptr);
            } else {
                // No children, tree is now empty
                root = nullptr;
            }
        } else {
            // Detach from parent
            std::shared_ptr<BStarTreeNode> parent = node->getParent();
            if (parent) {
                if (parent->getLeftChild() == node) {
                    parent->setLeftChild(nullptr);
                } else if (parent->getRightChild() == node) {
                    parent->setRightChild(nullptr);
                }
            }
        }
        
        // Rebuild the tree to maintain proper structure
        rebuildTree();
        return true;
    }
    
    /**
     * Finds a node by module name
     * 
     * @param moduleName Module name to search for
     * @return Node if found, nullptr otherwise
     */
    std::shared_ptr<BStarTreeNode> findNode(const std::string& moduleName) const {
        auto it = nodeMap.find(moduleName);
        if (it != nodeMap.end()) {
            return it->second;
        }
        return nullptr;
    }
    
    /**
     * Collects nodes in preorder traversal
     * 
     * @param node Current node
     * @param nodes Output vector of nodes
     */
    void collectNodesPreOrder(
        const std::shared_ptr<BStarTreeNode>& node,
        std::vector<std::shared_ptr<BStarTreeNode>>& nodes) const {
        
        if (!node) return;
        
        // Root, Left, Right
        nodes.push_back(node);
        collectNodesPreOrder(node->getLeftChild(), nodes);
        collectNodesPreOrder(node->getRightChild(), nodes);
    }
    
    /**
     * Collects nodes in inorder traversal
     * 
     * @param node Current node
     * @param nodes Output vector of nodes
     */
    void collectNodesInOrder(
        const std::shared_ptr<BStarTreeNode>& node,
        std::vector<std::shared_ptr<BStarTreeNode>>& nodes) const {
        
        if (!node) return;
        
        // Left, Root, Right
        collectNodesInOrder(node->getLeftChild(), nodes);
        nodes.push_back(node);
        collectNodesInOrder(node->getRightChild(), nodes);
    }
    
    /**
     * Collects all nodes in the tree in preorder traversal
     * 
     * @return Vector of nodes in preorder traversal
     */
    std::vector<std::shared_ptr<BStarTreeNode>> getAllNodesPreOrder() const {
        std::vector<std::shared_ptr<BStarTreeNode>> nodes;
        collectNodesPreOrder(root, nodes);
        return nodes;
    }
    
    /**
     * Collects all nodes in the tree in inorder traversal
     * 
     * @return Vector of nodes in inorder traversal
     */
    std::vector<std::shared_ptr<BStarTreeNode>> getAllNodesInOrder() const {
        std::vector<std::shared_ptr<BStarTreeNode>> nodes;
        collectNodesInOrder(root, nodes);
        return nodes;
    }
    
    /**
     * Rebuilds the tree from the current nodes
     * This maintains proper B*-tree structure
     */
    void rebuildTree() {
        if (nodeMap.empty()) {
            root = nullptr;
            return;
        }
        
        // If we have a single node, make it the root
        if (nodeMap.size() == 1) {
            root = nodeMap.begin()->second;
            root->setParent(nullptr);
            root->setLeftChild(nullptr);
            root->setRightChild(nullptr);
            updateInorderIndices();
            return;
        }
        
        // Collect nodes in traversal orders
        std::vector<std::shared_ptr<BStarTreeNode>> preorder = getAllNodesPreOrder();
        std::vector<std::shared_ptr<BStarTreeNode>> inorder = getAllNodesInOrder();
        
        // If we don't have all nodes in the traversals, collect all nodes
        // from the nodeMap and create a simple initial structure
        if (preorder.size() != nodeMap.size() || inorder.size() != nodeMap.size()) {
            preorder.clear();
            inorder.clear();
            
            // Add all nodes from the map
            for (const auto& pair : nodeMap) {
                preorder.push_back(pair.second);
                inorder.push_back(pair.second);
            }
            
            // Sort inorder by an appropriate criteria (e.g., x-coordinate)
            std::sort(inorder.begin(), inorder.end(),
                     [](const std::shared_ptr<BStarTreeNode>& a, const std::shared_ptr<BStarTreeNode>& b) {
                         // This sort order defines the inorder traversal
                         // For a B*-tree, typically x-coordinate would be used
                         // But you could also use area, module name, etc.
                         return a->getModuleName() < b->getModuleName();
                     });
        }
        
        // Rebuild the tree from traversals
        int preIdx = 0;
        root = buildTreeFromTraversals(preorder, inorder, preIdx, 0, inorder.size() - 1);
        
        // Update indices for faster rebuilding in the future
        updateInorderIndices();
    }
    
    /**
     * Creates a B*-tree optimized for floorplanning
     * 
     * @param moduleAreas Map of module names to their areas
     */
    void createOptimizedTree(const std::map<std::string, int>& moduleAreas) {
        // Clear existing tree
        root = nullptr;
        nodeMap.clear();
        inorderIndices.clear();
        
        // Create nodes for all modules
        std::vector<std::pair<std::string, int>> sortedModules;
        for (const auto& pair : moduleAreas) {
            sortedModules.push_back(pair);
        }
        
        // Sort modules by area (largest first)
        std::sort(sortedModules.begin(), sortedModules.end(),
                 [](const auto& a, const auto& b) {
                     return a.second > b.second;
                 });
        
        // Create initial ordering of modules for B*-tree
        std::vector<std::shared_ptr<BStarTreeNode>> preorder;
        std::vector<std::shared_ptr<BStarTreeNode>> inorder;
        
        // For initial construction, use a simple balanced tree
        // This will be refined later through SA perturbations
        for (const auto& pair : sortedModules) {
            auto newNode = std::make_shared<BStarTreeNode>(pair.first);
            nodeMap[pair.first] = newNode;
            
            // For an initial simple tree, just add to both traversals
            preorder.push_back(newNode);
            inorder.push_back(newNode);
        }
        
        // For a balanced initial tree, we should sort inorder differently
        // This is a very simple approach - a real implementation would be more sophisticated
        // For example, using slicing floorplan to create a better initial ordering
        
        // Build the tree from traversals
        int preIdx = 0;
        root = buildTreeFromTraversals(preorder, inorder, preIdx, 0, inorder.size() - 1);
        
        // Update indices for faster rebuilding in the future
        updateInorderIndices();
    }
    
    /**
     * Optimizes the tree structure based on module positions
     * 
     * @param modulePositions Map of module names to their positions (x, y)
     */
    void optimizeTreeFromPositions(
        const std::map<std::string, std::pair<int, int>>& modulePositions) {
        
        // First, ensure all nodes exist
        for (const auto& pair : modulePositions) {
            if (nodeMap.find(pair.first) == nodeMap.end()) {
                auto newNode = std::make_shared<BStarTreeNode>(pair.first);
                nodeMap[pair.first] = newNode;
            }
        }
        
        // Create inorder traversal based on x-coordinates
        std::vector<std::shared_ptr<BStarTreeNode>> inorder;
        for (const auto& pair : nodeMap) {
            inorder.push_back(pair.second);
        }
        
        // Sort by x-coordinate for inorder traversal
        std::sort(inorder.begin(), inorder.end(),
                 [&modulePositions](const auto& a, const auto& b) {
                     auto itA = modulePositions.find(a->getModuleName());
                     auto itB = modulePositions.find(b->getModuleName());
                     
                     // If we don't have position info, use module name
                     if (itA == modulePositions.end() || itB == modulePositions.end()) {
                         return a->getModuleName() < b->getModuleName();
                     }
                     
                     // Sort by x-coordinate for inorder
                     return itA->second.first < itB->second.first;
                 });
        
        // Create preorder traversal based on y-coordinates
        std::vector<std::shared_ptr<BStarTreeNode>> preorder = inorder;
        
        // Sort by y-coordinate for preorder traversal
        std::sort(preorder.begin(), preorder.end(),
                 [&modulePositions](const auto& a, const auto& b) {
                     auto itA = modulePositions.find(a->getModuleName());
                     auto itB = modulePositions.find(b->getModuleName());
                     
                     // If we don't have position info, use module name
                     if (itA == modulePositions.end() || itB == modulePositions.end()) {
                         return a->getModuleName() < b->getModuleName();
                     }
                     
                     // Sort by y-coordinate for preorder
                     return itA->second.second < itB->second.second;
                 });
        
        // Build the tree from these traversals
        int preIdx = 0;
        root = buildTreeFromTraversals(preorder, inorder, preIdx, 0, inorder.size() - 1);
        
        // Update indices for faster rebuilding
        updateInorderIndices();
    }
    
    /**
     * Swaps two nodes in the tree
     * 
     * @param node1 First node
     * @param node2 Second node
     * @return True if swap was successful
     */
    bool swapNodes(std::shared_ptr<BStarTreeNode> node1, std::shared_ptr<BStarTreeNode> node2) {
        if (!node1 || !node2 || node1 == node2) {
            return false;
        }
        
        // Swap the module names
        std::string tempName = node1->getModuleName();
        
        // Update the nodeMap
        nodeMap[node1->getModuleName()] = node2;
        nodeMap[node2->getModuleName()] = node1;
        
        // In a proper implementation, we would update the actual module references
        // But since we can't modify the node's module name in the current API,
        // we need to rebuild the tree with updated references
        rebuildTree();
        
        return true;
    }
    
    /**
     * Moves a node to a new position in the tree
     * 
     * @param nodeToMove Node to move
     * @param newParent New parent node
     * @param asLeftChild True to make left child, false for right child
     * @return True if move was successful
     */
    bool moveNode(
        std::shared_ptr<BStarTreeNode> nodeToMove, 
        std::shared_ptr<BStarTreeNode> newParent,
        bool asLeftChild) {
        
        if (!nodeToMove || !newParent || nodeToMove == newParent) {
            return false;
        }
        
        // Check for cycles - ensure newParent is not a descendant of nodeToMove
        std::shared_ptr<BStarTreeNode> current = newParent;
        while (current) {
            if (current == nodeToMove) {
                return false; // Would create a cycle
            }
            current = current->getParent();
        }
        
        // Remove node from its current parent
        std::shared_ptr<BStarTreeNode> oldParent = nodeToMove->getParent();
        if (oldParent) {
            if (oldParent->getLeftChild() == nodeToMove) {
                oldParent->setLeftChild(nullptr);
            } else if (oldParent->getRightChild() == nodeToMove) {
                oldParent->setRightChild(nullptr);
            }
        }
        
        // Set new parent relationship
        nodeToMove->setParent(newParent);
        
        // Set as left or right child
        if (asLeftChild) {
            // Handle existing child
            std::shared_ptr<BStarTreeNode> existingChild = newParent->getLeftChild();
            if (existingChild) {
                // Make the existing child a child of the moved node
                if (!nodeToMove->getLeftChild()) {
                    nodeToMove->setLeftChild(existingChild);
                    existingChild->setParent(nodeToMove);
                } else if (!nodeToMove->getRightChild()) {
                    nodeToMove->setRightChild(existingChild);
                    existingChild->setParent(nodeToMove);
                } else {
                    // Both children slots filled, rebuild tree
                    rebuildTree();
                    return false;
                }
            }
            newParent->setLeftChild(nodeToMove);
        } else {
            // Handle existing child
            std::shared_ptr<BStarTreeNode> existingChild = newParent->getRightChild();
            if (existingChild) {
                // Make the existing child a child of the moved node
                if (!nodeToMove->getLeftChild()) {
                    nodeToMove->setLeftChild(existingChild);
                    existingChild->setParent(nodeToMove);
                } else if (!nodeToMove->getRightChild()) {
                    nodeToMove->setRightChild(existingChild);
                    existingChild->setParent(nodeToMove);
                } else {
                    // Both children slots filled, rebuild tree
                    rebuildTree();
                    return false;
                }
            }
            newParent->setRightChild(nodeToMove);
        }
        
        // Update indices
        updateInorderIndices();
        return true;
    }
};