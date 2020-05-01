#ifndef NODE_LIST_HPP
#define NODE_LIST_HPP

#include <vector>
#include <planning/node.hpp>

class Node_list
{
    public:
    Node_list(void);
    bool is_member(int x, int y);
    void put(Node node);
    Node* get(int x, int y);

    private:
    std::vector<Node> nodes_;

};

#endif //NODE_LIST_HPP
