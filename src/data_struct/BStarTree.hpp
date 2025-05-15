#pragma once
#include <vector>
#include <limits>
#include <memory>
#include <unordered_map>
#include <cassert>

/**
 * @brief A very simple segment tree for RMQ from TAs
 */
template <typename T>
class SegmentTree
{
    struct Node
    {
        T data, tag;
        bool hasTag;

        Node() : data(0), tag(0), hasTag(false) {}
    };

    size_t n;
    std::vector<Node> seg;

    T getVal(size_t id) const
    {
        return (seg[id].hasTag) ? seg[id].tag : seg[id].data;
    }

    void pull(size_t id)
    {
        seg[id].data = std::max(getVal(id * 2), getVal(id * 2 + 1));
    }

    void push(size_t id)
    {
        if (seg[id].hasTag)
        {
            seg[id].data = getVal(id);
            seg[id].hasTag = false;
            seg[id * 2].tag = seg[id].tag;
            seg[id * 2].hasTag = true;
            seg[id * 2 + 1].tag = seg[id].tag;
            seg[id * 2 + 1].hasTag = true;
        }
    }

    T query(size_t ql, size_t qr, size_t l, size_t r, size_t id = 1)
    {
        if (ql > r || qr < l)
            return std::numeric_limits<T>::min();
        if (ql <= l && qr >= r)
            return getVal(id);

        push(id);
        int mid = (l + r) / 2;
        return std::max(query(ql, qr, l, mid, id * 2), query(ql, qr, mid + 1, r, id * 2 + 1));
    }

    void update(T val, size_t ql, size_t qr, size_t l, size_t r, size_t id = 1)
    {
        if (ql > r || qr < l)
            return;
        if (ql <= l && qr >= r)
        {
            seg[id].tag = val;
            seg[id].hasTag = true;
            return;
        }

        push(id);
        size_t mid = (l + r) / 2;
        update(val, ql, qr, l, mid, id * 2);
        update(val, ql, qr, mid + 1, r, id * 2 + 1);
        pull(id);
    }

public:
    SegmentTree() : n(0) {}

    void init(size_t n_)
    {
        n = n_;
        seg.assign(n_ * 4, {});
    }

    T query(size_t ql, size_t qr)
    {
        return query(ql, qr, 0ULL, n - 1);
    }

    void update(size_t ql, size_t qr, T val)
    {
        update(val, ql, qr, 0ULL, n - 1);
    }
};

/**
 * @brief The node structure of the B*-tree
 */
template <typename T>
struct Node
{
    using ptr = std::unique_ptr<Node>;

    T x, y;
    T width, height;
    Node *lchild, *rchild;

    Node() : x(0), y(0), width(0), height(0), lchild(nullptr), rchild(nullptr) {}

    void setPosition(T x_, T y_)
    {
        x = x_;
        y = y_;
    }

    void setShape(T width_, T height_)
    {
        width = width_;
        height = height_;
    }
};

/**
 * @brief A B*-tree to calculate the coordinates of nodes and the area of placement
 */
template <typename T>
class BStarTree
{
    std::unordered_map<Node<T> *, int64_t> toInorderIdx;
    SegmentTree<T> contourH;

    Node<T> *buildTree(const std::vector<Node<T> *> &preorder, const std::vector<Node<T> *> &inorder, size_t &i, int64_t l, int64_t r)
    {
        if (l > r || i >= preorder.size())
            return nullptr;

        Node<T> *node = preorder[i++];
        assert(toInorderIdx.count(node) > 0 && "Node not found in inorder map.");
        int64_t idx = toInorderIdx[node];
        node->lchild = buildTree(preorder, inorder, i, l, idx - 1);
        node->rchild = buildTree(preorder, inorder, i, idx + 1, r);
        return node;
    }

    T getTotalWidth(Node<T> *node) const
    {
        if (!node)
            return 0;

        return node->width + getTotalWidth(node->lchild) + getTotalWidth(node->rchild);
    }

    void setPosition(Node<T> *node, T startX)
    {
        if (!node)
            return;

        T endX = startX + node->width;
        T y = contourH.query(startX, endX - 1);
        contourH.update(startX, endX - 1, y + node->height);
        node->setPosition(startX, y);
        setPosition(node->lchild, endX);
        setPosition(node->rchild, startX);
    }

public:
    Node<T> *root;

    BStarTree() : root(nullptr) {}

    void buildTree(const std::vector<Node<T> *> &preorder, const std::vector<Node<T> *> &inorder)
    {
        assert(preorder.size() == inorder.size() && "The size of preorder and inorder must be the same.");
        int64_t n = inorder.size();

        toInorderIdx.clear();
        for (int64_t i = 0; i < n; ++i)
            toInorderIdx[inorder[i]] = i;

        size_t i = 0;
        root = buildTree(preorder, inorder, i, 0LL, n - 1);
    }

    void setPosition()
    {
        contourH.init(getTotalWidth(root));
        setPosition(root, 0);
    }

    T getArea() const
    {
        auto [width, height] = getWidthHeight(root);
        return width * height;
    }

    std::pair<T, T> getWidthHeight(Node<T> *node) const
    {
        if (!node)
            return {0, 0};

        auto [lMaxWidth, lMaxHeight] = getWidthHeight(node->lchild);
        auto [rMaxWidth, rMaxHeight] = getWidthHeight(node->rchild);
        T maxWidth = std::max(std::max(lMaxWidth, rMaxWidth), node->x + node->width);
        T maxHeight = std::max(std::max(lMaxHeight, rMaxHeight), node->y + node->height);
        return {maxWidth, maxHeight};
    }
};
