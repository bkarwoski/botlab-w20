#include <planning/node_list.hpp>

Node_list::Node_list(void)
: nodes_()
{
    
}

bool Node_list::is_member(int x, int y){
    for(const auto& n: nodes_){
        if(n.x() == x && n.y() == y) return true;
    }
    return false;
}

void Node_list::put(Node node){
    nodes_.push_back(node);
}

Node* Node_list::get(int x, int y){
    for(auto& n: nodes_){
        if(n.x() == x && n.y() == y) return &n;
    }
}
